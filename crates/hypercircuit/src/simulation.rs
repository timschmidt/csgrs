//! Lowering retained primitive device models into executable linear MNA stamps.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter};

use hyperreal::{Real, RealSign};

#[cfg(feature = "behavior-async")]
use crate::AsyncBehaviorRuntime;
use crate::{
    BehaviorError, BehaviorRuntime, BranchId, Circuit, CircuitCertificationReport, CircuitEvent,
    CircuitEventAgenda, CircuitEventAgendaError, CircuitEventCounters, CircuitEventKind,
    CircuitEventLifecycleRecord, CircuitEventReplayReport, CircuitEventTarget, CircuitInstance,
    CircuitSimulationFingerprint, ComponentId, DeviceModel, DeviceModelKind, DiodeNewtonPolicy,
    DiodeNewtonSolveError, DiodeNewtonSolveReport, DiodeNewtonStatus, ExactBreakpointError,
    ExactBreakpointProvider, ExactBreakpointSchedule, LinearMnaSystem, LinearSolveReport,
    LinearStamp, LogicValue, MnaUnknown, NetId, SourceWaveform, SwitchState, TransientPolicy,
    earliest_exact_breakpoint_after, solve_shockley_diode_newton,
};

/// Device instance that could not be lowered into the requested linear model.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DeviceLoweringIssue {
    /// Circuit/model structural validation must be fixed first.
    InvalidCircuit,
    /// A primitive requires more connected pins than the instance supplies.
    MissingPins {
        component: ComponentId,
        required: usize,
    },
    /// A required exact parameter was absent.
    MissingParameter {
        component: ComponentId,
        parameter: String,
    },
    /// Resistance was nonpositive or could not be certified positive.
    InvalidResistance(ComponentId),
    /// The model family is not linearized by this path.
    UnsupportedModel(ComponentId),
    /// An independent-source waveform could not be evaluated exactly.
    SourceEvaluation(ComponentId),
}

/// Replayable lowering result before matrix assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct LinearDeviceLoweringReport {
    /// Exact stamps generated in circuit instance order.
    pub stamps: Vec<LinearStamp>,
    /// Instances that did not produce a complete linear representation.
    pub issues: Vec<DeviceLoweringIssue>,
    /// Explicit modeling decisions such as DC-open capacitors.
    pub notes: Vec<String>,
    /// Evaluated time-dependent independent-source values by component.
    pub source_values: BTreeMap<ComponentId, Real>,
}

impl LinearDeviceLoweringReport {
    /// True when every instance was either lowered or explicitly modeled by a complete DC rule.
    pub fn is_complete(&self) -> bool {
        self.issues.is_empty()
    }
}

/// Failure to construct an executable linear MNA system from device models.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DeviceLoweringError {
    /// One or more instances could not be represented.
    Incomplete(Vec<DeviceLoweringIssue>),
    /// Generated stamps could not be assembled.
    Assembly(crate::CircuitError),
}

impl Display for DeviceLoweringError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Incomplete(issues) => write!(
                formatter,
                "{} device instance(s) could not be lowered",
                issues.len()
            ),
            Self::Assembly(error) => write!(formatter, "linear MNA assembly failed: {error}"),
        }
    }
}

impl std::error::Error for DeviceLoweringError {}

/// Failure to evaluate a retained exact source waveform.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SourceWaveformEvaluationError {
    /// A piecewise-linear waveform contained no points.
    Empty,
    /// Piecewise-linear point times were not exactly and strictly increasing.
    NonIncreasingTimes,
    /// The queried time could not be exactly ordered against a transition.
    IndeterminateTime,
    /// Exact interpolation arithmetic could not be completed.
    Arithmetic,
    /// A pulse train's exact timing contract is invalid.
    InvalidPulseTiming,
    /// A sine source's frequency, delay, or damping is invalid.
    InvalidSineTiming,
    /// An exponential source's delays or time constants are invalid.
    InvalidExponentialTiming,
}

impl Display for SourceWaveformEvaluationError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Empty => formatter.write_str("piecewise-linear source waveform is empty"),
            Self::NonIncreasingTimes => formatter
                .write_str("piecewise-linear source times must be exactly and strictly increasing"),
            Self::IndeterminateTime => {
                formatter.write_str("source waveform time ordering is indeterminate")
            }
            Self::Arithmetic => formatter.write_str("exact source waveform interpolation failed"),
            Self::InvalidPulseTiming => formatter.write_str(
                "pulse timing must be nonnegative, fit within a strictly positive period, and be exactly ordered",
            ),
            Self::InvalidSineTiming => formatter.write_str(
                "sine frequency, delay, and damping must be nonnegative and exactly classified",
            ),
            Self::InvalidExponentialTiming => formatter.write_str(
                "exponential delays must be nonnegative and ordered, with positive exact time constants",
            ),
        }
    }
}

impl std::error::Error for SourceWaveformEvaluationError {}

impl SourceWaveform {
    /// Evaluates this retained waveform at one exact simulation time.
    pub fn value_at(&self, time: &Real) -> Result<Real, SourceWaveformEvaluationError> {
        match self {
            Self::Constant(value) => Ok(value.clone()),
            Self::Step {
                initial,
                final_value,
                at,
            } => match time.partial_cmp(at) {
                Some(Ordering::Less) => Ok(initial.clone()),
                Some(Ordering::Equal | Ordering::Greater) => Ok(final_value.clone()),
                None => Err(SourceWaveformEvaluationError::IndeterminateTime),
            },
            Self::PiecewiseLinear { points } => {
                let Some(first) = points.first() else {
                    return Err(SourceWaveformEvaluationError::Empty);
                };
                if points
                    .windows(2)
                    .any(|pair| pair[0].time.partial_cmp(&pair[1].time) != Some(Ordering::Less))
                {
                    return Err(SourceWaveformEvaluationError::NonIncreasingTimes);
                }
                match time.partial_cmp(&first.time) {
                    Some(Ordering::Less | Ordering::Equal) => return Ok(first.value.clone()),
                    Some(Ordering::Greater) => {}
                    None => return Err(SourceWaveformEvaluationError::IndeterminateTime),
                }
                for pair in points.windows(2) {
                    let end = &pair[1];
                    match time.partial_cmp(&end.time) {
                        Some(Ordering::Less) => {
                            let elapsed = time.clone() - pair[0].time.clone();
                            let duration = end.time.clone() - pair[0].time.clone();
                            let fraction = (elapsed / duration)
                                .map_err(|_| SourceWaveformEvaluationError::Arithmetic)?;
                            return Ok(pair[0].value.clone()
                                + (end.value.clone() - pair[0].value.clone()) * fraction);
                        }
                        Some(Ordering::Equal) => return Ok(end.value.clone()),
                        Some(Ordering::Greater) => {}
                        None => return Err(SourceWaveformEvaluationError::IndeterminateTime),
                    }
                }
                Ok(points
                    .last()
                    .expect("nonempty waveform checked above")
                    .value
                    .clone())
            }
            Self::Pulse {
                low_value,
                high_value,
                delay,
                rise_time,
                high_time,
                fall_time,
                period,
            } => {
                validate_pulse_timing(delay, rise_time, high_time, fall_time, period)?;
                let Some((_, phase)) = pulse_cycle_position(time, delay, period)? else {
                    return Ok(low_value.clone());
                };
                let rise_end = rise_time.clone();
                if phase.partial_cmp(&rise_end) == Some(Ordering::Less) {
                    let fraction = (phase / rise_time.clone())
                        .map_err(|_| SourceWaveformEvaluationError::Arithmetic)?;
                    return Ok(
                        low_value.clone() + (high_value.clone() - low_value.clone()) * fraction
                    );
                }
                let high_end = rise_end + high_time.clone();
                if phase.partial_cmp(&high_end) == Some(Ordering::Less) {
                    return Ok(high_value.clone());
                }
                let fall_end = high_end.clone() + fall_time.clone();
                if phase.partial_cmp(&fall_end) == Some(Ordering::Less) {
                    let elapsed = phase - high_end;
                    let fraction = (elapsed / fall_time.clone())
                        .map_err(|_| SourceWaveformEvaluationError::Arithmetic)?;
                    return Ok(
                        high_value.clone() + (low_value.clone() - high_value.clone()) * fraction
                    );
                }
                match phase.partial_cmp(&fall_end) {
                    Some(Ordering::Equal | Ordering::Greater) => Ok(low_value.clone()),
                    None => Err(SourceWaveformEvaluationError::IndeterminateTime),
                    Some(Ordering::Less) => unreachable!("fall interval returned above"),
                }
            }
            Self::Sine {
                offset,
                amplitude,
                frequency,
                delay,
                damping,
                phase_degrees,
            } => {
                validate_sine_timing(frequency, delay, damping)?;
                let elapsed = match time.partial_cmp(delay) {
                    Some(Ordering::Less) => Real::zero(),
                    Some(Ordering::Equal | Ordering::Greater) => time.clone() - delay.clone(),
                    None => return Err(SourceWaveformEvaluationError::IndeterminateTime),
                };
                let phase = (phase_degrees.clone() / Real::from(180))
                    .map_err(|_| SourceWaveformEvaluationError::Arithmetic)?;
                let angle = Real::from(2) * frequency.clone() * elapsed.clone() + phase;
                let envelope = (-damping.clone() * elapsed)
                    .exp()
                    .map_err(|_| SourceWaveformEvaluationError::Arithmetic)?;
                Ok(offset.clone() + amplitude.clone() * envelope * angle.sin_pi())
            }
            Self::Exponential {
                initial,
                pulsed,
                rise_delay,
                rise_time_constant,
                fall_delay,
                fall_time_constant,
            } => {
                validate_exponential_timing(
                    rise_delay,
                    rise_time_constant,
                    fall_delay,
                    fall_time_constant,
                )?;
                match time.partial_cmp(rise_delay) {
                    Some(Ordering::Less) => return Ok(initial.clone()),
                    Some(Ordering::Equal | Ordering::Greater) => {}
                    None => return Err(SourceWaveformEvaluationError::IndeterminateTime),
                }
                let excursion = pulsed.clone() - initial.clone();
                let rise_elapsed = time.clone() - rise_delay.clone();
                let mut value = initial.clone()
                    + excursion.clone()
                        * exponential_transition(&rise_elapsed, rise_time_constant)?;
                match time.partial_cmp(fall_delay) {
                    Some(Ordering::Less) => {}
                    Some(Ordering::Equal | Ordering::Greater) => {
                        let fall_elapsed = time.clone() - fall_delay.clone();
                        value -=
                            excursion * exponential_transition(&fall_elapsed, fall_time_constant)?;
                    }
                    None => return Err(SourceWaveformEvaluationError::IndeterminateTime),
                }
                Ok(value)
            }
        }
    }

    /// Returns the first retained waveform breakpoint strictly after `time`.
    ///
    /// Breakpoints include steps, PWL knots, and every pulse phase boundary.
    /// Transient runners use this to avoid stepping across authored source
    /// events even when the caller proposes a larger timestep.
    pub fn next_breakpoint_after(
        &self,
        time: &Real,
    ) -> Result<Option<Real>, SourceWaveformEvaluationError> {
        match self {
            Self::Constant(_) => Ok(None),
            Self::Step { at, .. } => match time.partial_cmp(at) {
                Some(Ordering::Less) => Ok(Some(at.clone())),
                Some(Ordering::Equal | Ordering::Greater) => Ok(None),
                None => Err(SourceWaveformEvaluationError::IndeterminateTime),
            },
            Self::PiecewiseLinear { points } => {
                if points.is_empty() {
                    return Err(SourceWaveformEvaluationError::Empty);
                }
                if points
                    .windows(2)
                    .any(|pair| pair[0].time.partial_cmp(&pair[1].time) != Some(Ordering::Less))
                {
                    return Err(SourceWaveformEvaluationError::NonIncreasingTimes);
                }
                for point in points {
                    match time.partial_cmp(&point.time) {
                        Some(Ordering::Less) => return Ok(Some(point.time.clone())),
                        Some(Ordering::Equal | Ordering::Greater) => {}
                        None => return Err(SourceWaveformEvaluationError::IndeterminateTime),
                    }
                }
                Ok(None)
            }
            Self::Pulse {
                delay,
                rise_time,
                high_time,
                fall_time,
                period,
                ..
            } => {
                validate_pulse_timing(delay, rise_time, high_time, fall_time, period)?;
                let Some((cycle_start, _)) = pulse_cycle_position(time, delay, period)? else {
                    return Ok(Some(delay.clone()));
                };
                let offsets = [
                    Real::zero(),
                    rise_time.clone(),
                    rise_time.clone() + high_time.clone(),
                    rise_time.clone() + high_time.clone() + fall_time.clone(),
                ];
                for offset in offsets {
                    let candidate = cycle_start.clone() + offset;
                    match time.partial_cmp(&candidate) {
                        Some(Ordering::Less) => return Ok(Some(candidate)),
                        Some(Ordering::Equal | Ordering::Greater) => {}
                        None => return Err(SourceWaveformEvaluationError::IndeterminateTime),
                    }
                }
                Ok(Some(cycle_start + period.clone()))
            }
            Self::Sine {
                frequency,
                delay,
                damping,
                ..
            } => {
                validate_sine_timing(frequency, delay, damping)?;
                match time.partial_cmp(delay) {
                    Some(Ordering::Less) => Ok(Some(delay.clone())),
                    Some(Ordering::Equal | Ordering::Greater) => Ok(None),
                    None => Err(SourceWaveformEvaluationError::IndeterminateTime),
                }
            }
            Self::Exponential {
                rise_delay,
                rise_time_constant,
                fall_delay,
                fall_time_constant,
                ..
            } => {
                validate_exponential_timing(
                    rise_delay,
                    rise_time_constant,
                    fall_delay,
                    fall_time_constant,
                )?;
                for breakpoint in [rise_delay, fall_delay] {
                    match time.partial_cmp(breakpoint) {
                        Some(Ordering::Less) => return Ok(Some(breakpoint.clone())),
                        Some(Ordering::Equal | Ordering::Greater) => {}
                        None => return Err(SourceWaveformEvaluationError::IndeterminateTime),
                    }
                }
                Ok(None)
            }
        }
    }
}

fn validate_pulse_timing(
    delay: &Real,
    rise_time: &Real,
    high_time: &Real,
    fall_time: &Real,
    period: &Real,
) -> Result<(), SourceWaveformEvaluationError> {
    let nonnegative = |value: &Real| {
        matches!(
            value.structural_facts().sign,
            Some(RealSign::Zero | RealSign::Positive)
        )
    };
    let active_time = rise_time.clone() + high_time.clone() + fall_time.clone();
    if !nonnegative(delay)
        || !nonnegative(rise_time)
        || !nonnegative(high_time)
        || !nonnegative(fall_time)
        || period.structural_facts().sign != Some(RealSign::Positive)
        || !matches!(
            active_time.partial_cmp(period),
            Some(Ordering::Less | Ordering::Equal)
        )
    {
        return Err(SourceWaveformEvaluationError::InvalidPulseTiming);
    }
    Ok(())
}

fn pulse_cycle_position(
    time: &Real,
    delay: &Real,
    period: &Real,
) -> Result<Option<(Real, Real)>, SourceWaveformEvaluationError> {
    match time.partial_cmp(delay) {
        Some(Ordering::Less) => return Ok(None),
        Some(Ordering::Equal | Ordering::Greater) => {}
        None => return Err(SourceWaveformEvaluationError::IndeterminateTime),
    }
    let elapsed = time.clone() - delay.clone();
    let cycles = (elapsed.clone() / period.clone())
        .map_err(|_| SourceWaveformEvaluationError::Arithmetic)?
        .floor_certified()
        .map_err(|_| SourceWaveformEvaluationError::Arithmetic)?;
    let cycle_start = delay.clone() + period.clone() * Real::integer(cycles);
    let phase = time.clone() - cycle_start.clone();
    Ok(Some((cycle_start, phase)))
}

fn validate_sine_timing(
    frequency: &Real,
    delay: &Real,
    damping: &Real,
) -> Result<(), SourceWaveformEvaluationError> {
    let nonnegative = |value: &Real| {
        matches!(
            value.structural_facts().sign,
            Some(RealSign::Zero | RealSign::Positive)
        )
    };
    if !nonnegative(frequency) || !nonnegative(delay) || !nonnegative(damping) {
        return Err(SourceWaveformEvaluationError::InvalidSineTiming);
    }
    Ok(())
}

fn validate_exponential_timing(
    rise_delay: &Real,
    rise_time_constant: &Real,
    fall_delay: &Real,
    fall_time_constant: &Real,
) -> Result<(), SourceWaveformEvaluationError> {
    if !matches!(
        rise_delay.structural_facts().sign,
        Some(RealSign::Zero | RealSign::Positive)
    ) || rise_time_constant.structural_facts().sign != Some(RealSign::Positive)
        || !matches!(
            rise_delay.partial_cmp(fall_delay),
            Some(Ordering::Less | Ordering::Equal)
        )
        || fall_time_constant.structural_facts().sign != Some(RealSign::Positive)
    {
        return Err(SourceWaveformEvaluationError::InvalidExponentialTiming);
    }
    Ok(())
}

fn exponential_transition(
    elapsed: &Real,
    time_constant: &Real,
) -> Result<Real, SourceWaveformEvaluationError> {
    let exponent = (-elapsed.clone() / time_constant.clone())
        .map_err(|_| SourceWaveformEvaluationError::Arithmetic)?;
    let decay = exponent
        .exp()
        .map_err(|_| SourceWaveformEvaluationError::Arithmetic)?;
    Ok(Real::one() - decay)
}

/// Exact capacitor/inductor state retained between transient steps.
#[derive(Clone, Debug, PartialEq)]
pub struct ReactiveState {
    /// Voltage from the model's first pin to its second pin.
    pub voltage: Real,
    /// Current flowing from the model's first pin to its second pin.
    pub current: Real,
}

impl Default for ReactiveState {
    fn default() -> Self {
        Self {
            voltage: Real::zero(),
            current: Real::zero(),
        }
    }
}

/// Component-addressed reactive history supplied to a transient step.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TransientHistory {
    /// Capacitor and inductor state by stable component identity.
    pub reactive: BTreeMap<ComponentId, ReactiveState>,
}

/// One solved and exactly replayed transient companion step.
#[derive(Clone, Debug, PartialEq)]
pub struct TransientStepReport {
    /// Exact simulation time at this solved endpoint.
    pub time: Real,
    /// Exact authored timestep.
    pub timestep: Real,
    /// Complete generated stamp set for this step.
    pub stamps: Vec<LinearStamp>,
    /// Stable ordering corresponding to the solution candidate.
    pub unknowns: Vec<MnaUnknown>,
    /// Exact linear solution and residual replay.
    pub solution: LinearSolveReport,
    /// State to pass to the next step.
    pub next_history: TransientHistory,
    /// Evaluated time-dependent independent-source values by component.
    pub source_values: BTreeMap<ComponentId, Real>,
}

/// One converged mixed linear/reactive/diode transient endpoint.
#[derive(Clone, Debug, PartialEq)]
pub struct DiodeTransientStepReport {
    /// Exact simulation time at this solved endpoint.
    pub time: Real,
    /// Exact authored timestep.
    pub timestep: Real,
    /// Linear and reactive companion stamps supplied to Newton.
    pub linear_stamps: Vec<LinearStamp>,
    /// Bounded Newton proposal and true-law replay evidence.
    pub nonlinear: DiodeNewtonSolveReport,
    /// State to pass to the next reactive companion step.
    pub next_history: TransientHistory,
    /// Evaluated time-dependent independent-source values by component.
    pub source_values: BTreeMap<ComponentId, Real>,
}

/// Failure to form or converge one mixed diode transient endpoint.
#[derive(Clone, Debug, PartialEq)]
pub enum DiodeTransientStepError {
    /// Circuit graph validation failed.
    InvalidCircuit,
    /// A strictly positive timestep is required.
    InvalidTimestep,
    /// The circuit selected a non-executable transient policy.
    UnsupportedPolicy(TransientPolicy),
    /// Primitive linear model lowering was incomplete.
    Incomplete(Vec<DeviceLoweringIssue>),
    /// Reactive companion construction failed.
    Reactive(TransientStepError),
    /// Diode extraction, proposal construction, or exact replay failed.
    Nonlinear(DiodeNewtonSolveError),
    /// Newton reached its explicit iteration bound without convergence.
    IterationLimit(Box<DiodeNewtonSolveReport>),
}

impl Display for DiodeTransientStepError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidCircuit => formatter.write_str("invalid mixed diode transient circuit"),
            Self::InvalidTimestep => {
                formatter.write_str("mixed diode transient timestep must be positive")
            }
            Self::UnsupportedPolicy(policy) => {
                write!(formatter, "unsupported mixed transient policy {policy:?}")
            }
            Self::Incomplete(issues) => write!(
                formatter,
                "{} linear device(s) could not be lowered beside diodes",
                issues.len()
            ),
            Self::Reactive(error) => write!(formatter, "reactive companion failed: {error}"),
            Self::Nonlinear(error) => write!(formatter, "diode Newton failed: {error}"),
            Self::IterationLimit(report) => write!(
                formatter,
                "diode Newton reached {} iterations without convergence",
                report.iterations.len()
            ),
        }
    }
}

impl std::error::Error for DiodeTransientStepError {}

/// Failure to form or certify a transient companion step.
#[derive(Clone, Debug, PartialEq)]
pub enum TransientStepError {
    /// Circuit graph validation failed.
    InvalidCircuit,
    /// A strictly positive timestep is required.
    InvalidTimestep,
    /// The circuit selected a non-executable transient policy.
    UnsupportedPolicy(TransientPolicy),
    /// Primitive model lowering was incomplete.
    Incomplete(Vec<DeviceLoweringIssue>),
    /// A reactive value was absent or not certified positive.
    InvalidReactiveParameter {
        component: ComponentId,
        parameter: String,
    },
    /// Exact companion arithmetic could not be completed.
    Arithmetic,
    /// Generated MNA assembly or solve failed.
    Solve(crate::CircuitError),
}

impl Display for TransientStepError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidCircuit => formatter.write_str("cannot step an invalid circuit"),
            Self::InvalidTimestep => formatter.write_str("transient timestep must be positive"),
            Self::UnsupportedPolicy(policy) => {
                write!(
                    formatter,
                    "unsupported executable transient policy: {policy:?}"
                )
            }
            Self::Incomplete(issues) => write!(
                formatter,
                "{} non-reactive device(s) could not be lowered",
                issues.len()
            ),
            Self::InvalidReactiveParameter {
                component,
                parameter,
            } => write!(
                formatter,
                "component {} has invalid reactive parameter {parameter}",
                component.as_str()
            ),
            Self::Arithmetic => formatter.write_str("exact transient companion arithmetic failed"),
            Self::Solve(error) => write!(formatter, "transient MNA solve failed: {error}"),
        }
    }
}

impl std::error::Error for TransientStepError {}

/// Run-level timestep strategy over the exact companion-step kernel.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum TransientAdaptation {
    /// Advance with the authored timestep, truncating only the final step.
    Fixed,
    /// Compare one full step against two half steps and retain the refined state.
    StepDoubling {
        /// Strictly positive absolute tolerance for every compared state value.
        absolute_tolerance: Real,
        /// Nonnegative tolerance proportional to the larger compared magnitude.
        relative_tolerance: Real,
        /// Factor in `(0, 1)` applied after a rejected attempt.
        shrink_factor: Real,
        /// Factor greater than one applied when the exact error ratio is at most one quarter.
        growth_factor: Real,
    },
}

/// Bounded exact transient time-series policy.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct TransientRunPolicy {
    /// Exact first simulation time.
    pub start_time: Real,
    /// Exact terminal simulation time.
    pub stop_time: Real,
    /// First proposed timestep.
    pub initial_timestep: Real,
    /// Smallest adaptive timestep, except an exact final or source-event step.
    pub minimum_timestep: Real,
    /// Largest proposed timestep.
    pub maximum_timestep: Real,
    /// Maximum accepted output samples.
    pub maximum_accepted_steps: usize,
    /// Maximum rejected adaptive attempts.
    pub maximum_rejected_steps: usize,
    /// Fixed or exact step-doubling control.
    pub adaptation: TransientAdaptation,
}

impl Default for TransientRunPolicy {
    fn default() -> Self {
        let hundred = Real::from(100);
        let million = Real::from(1_000_000);
        Self {
            start_time: Real::zero(),
            stop_time: Real::one(),
            initial_timestep: (Real::one() / hundred.clone())
                .expect("default timestep divisor is nonzero"),
            minimum_timestep: (Real::one() / million.clone())
                .expect("default minimum timestep divisor is nonzero"),
            maximum_timestep: (Real::one() / Real::from(10))
                .expect("default maximum timestep divisor is nonzero"),
            maximum_accepted_steps: 100_000,
            maximum_rejected_steps: 10_000,
            adaptation: TransientAdaptation::StepDoubling {
                absolute_tolerance: (Real::one() / million)
                    .expect("default absolute-tolerance divisor is nonzero"),
                relative_tolerance: (Real::one() / hundred)
                    .expect("default relative-tolerance divisor is nonzero"),
                shrink_factor: (Real::one() / Real::from(2))
                    .expect("default shrink-factor divisor is nonzero"),
                growth_factor: Real::from(2),
            },
        }
    }
}

impl TransientRunPolicy {
    /// Validates exact time bounds, step bounds, and adaptation controls.
    pub fn validate(&self) -> Result<(), TransientRunError> {
        validate_run_policy(self)
    }
}

/// Outcome of one attempted run-level timestep.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TransientStepDecisionKind {
    /// The proposal became an output sample.
    Accepted,
    /// Step-doubling error exceeded the authored tolerance.
    Rejected,
}

/// Audited decision for one fixed or adaptive timestep attempt.
#[derive(Clone, Debug, PartialEq)]
pub struct TransientStepDecision {
    /// Zero-based attempt index.
    pub attempt: usize,
    /// Exact time before the attempt.
    pub start_time: Real,
    /// Exact total attempted timestep.
    pub timestep: Real,
    /// Acceptance decision.
    pub kind: TransientStepDecisionKind,
    /// Maximum exact `absolute_error / authored_tolerance`; absent for fixed stepping.
    pub maximum_error_ratio: Option<Real>,
}

/// One accepted transient endpoint with stable unknown and reactive identities.
#[derive(Clone, Debug, PartialEq)]
pub struct TransientSample {
    /// Exact endpoint time.
    pub time: Real,
    /// Exact elapsed time since the preceding accepted endpoint.
    pub timestep: Real,
    /// MNA unknown values after exact residual replay.
    pub unknowns: BTreeMap<MnaUnknown, Real>,
    /// Component-addressed capacitor/inductor state.
    pub reactive: BTreeMap<ComponentId, ReactiveState>,
    /// Evaluated independent-source values at this endpoint.
    pub source_values: BTreeMap<ComponentId, Real>,
}

/// Terminal condition for a bounded transient run.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TransientRunStatus {
    /// The requested stop time was reached exactly.
    Complete,
    /// The accepted-step bound stopped the run.
    AcceptedStepLimit,
    /// The rejected-attempt bound stopped adaptive refinement.
    RejectedStepLimit,
    /// Another rejected refinement would cross the minimum timestep.
    MinimumTimestep,
}

/// Complete or bounded partial transient time series and decision audit.
#[derive(Clone, Debug, PartialEq)]
pub struct TransientRunReport {
    /// Run terminal condition.
    pub status: TransientRunStatus,
    /// Initial reactive state supplied by the caller.
    pub initial_history: TransientHistory,
    /// Accepted endpoint samples in increasing exact time order.
    pub samples: Vec<TransientSample>,
    /// Every accepted or rejected attempt.
    pub decisions: Vec<TransientStepDecision>,
    /// State at the final accepted endpoint.
    pub final_history: TransientHistory,
}

impl TransientRunReport {
    /// Extracts a complete `(time, value)` series for one MNA unknown.
    pub fn unknown_waveform(&self, unknown: &MnaUnknown) -> Option<Vec<(Real, Real)>> {
        self.samples
            .iter()
            .map(|sample| {
                sample
                    .unknowns
                    .get(unknown)
                    .cloned()
                    .map(|value| (sample.time.clone(), value))
            })
            .collect()
    }

    /// Extracts capacitor/inductor terminal voltage at accepted endpoints.
    pub fn reactive_voltage_waveform(&self, component: &ComponentId) -> Option<Vec<(Real, Real)>> {
        self.reactive_waveform(component, |state| state.voltage.clone())
    }

    /// Extracts capacitor/inductor terminal current at accepted endpoints.
    pub fn reactive_current_waveform(&self, component: &ComponentId) -> Option<Vec<(Real, Real)>> {
        self.reactive_waveform(component, |state| state.current.clone())
    }

    /// Extracts an evaluated independent-source waveform at accepted endpoints.
    pub fn source_waveform(&self, component: &ComponentId) -> Option<Vec<(Real, Real)>> {
        self.samples
            .iter()
            .map(|sample| {
                sample
                    .source_values
                    .get(component)
                    .cloned()
                    .map(|value| (sample.time.clone(), value))
            })
            .collect()
    }

    fn reactive_waveform(
        &self,
        component: &ComponentId,
        value: impl Fn(&ReactiveState) -> Real,
    ) -> Option<Vec<(Real, Real)>> {
        self.samples
            .iter()
            .map(|sample| {
                sample
                    .reactive
                    .get(component)
                    .map(|state| (sample.time.clone(), value(state)))
            })
            .collect()
    }
}

/// Execution state of an incremental transient session.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TransientSessionStatus {
    /// More analog time or pending events remain.
    Running,
    /// Exact requested stop time was reached.
    Complete,
    /// A bounded transient policy stopped further progress.
    Bounded(TransientRunStatus),
}

/// Effect of delivering one typed circuit event.
#[derive(Clone, Debug, PartialEq)]
pub enum CircuitEventApplication {
    /// An independent-source waveform was replaced by an exact constant value.
    SourceOverride { component: ComponentId, value: Real },
    /// A switch state was retained for a behavioral or future topology adapter.
    SwitchState {
        component: ComponentId,
        state: SwitchState,
    },
    /// A digital value was retained at its stable target.
    LogicState {
        target: CircuitEventTarget,
        value: LogicValue,
    },
    /// Event was delivered and retained but has no built-in electrical effect.
    Observed,
    /// Payload and target combination could not be applied.
    Unsupported { detail: String },
}

/// Result of one incremental session action.
#[derive(Clone, Debug, PartialEq)]
pub struct TransientSessionStep {
    /// Accepted analog endpoint, absent for an event-only delta action.
    pub sample: Option<TransientSample>,
    /// Events delivered at the resulting exact session time.
    pub events: Vec<CircuitEvent>,
    /// Typed application result corresponding to each delivered event.
    pub applications: Vec<CircuitEventApplication>,
    /// Session status after the action.
    pub status: TransientSessionStatus,
}

/// Failure to create or advance an incremental session.
#[derive(Clone, Debug, PartialEq)]
pub enum TransientSessionError {
    /// Session policy or agenda clock is invalid.
    InvalidPolicy,
    /// Requested stepping limit precedes the current session time or exceeds its stop time.
    InvalidTargetTime,
    /// Exact transient execution failed.
    Run(TransientRunError),
    /// Pending event time cannot be compared with session time.
    IndeterminateEventTime,
    /// Exact agenda clock advancement failed.
    Agenda(CircuitEventAgendaError),
    /// A runtime-only behavioral callback failed.
    Behavior(BehaviorError),
    /// Same-time callback activity exceeded the explicit delta-cycle bound.
    DeltaCycleLimit,
}

impl Display for TransientSessionError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidPolicy => formatter.write_str("invalid transient session policy"),
            Self::InvalidTargetTime => formatter.write_str("invalid transient session target time"),
            Self::Run(error) => Display::fmt(error, formatter),
            Self::IndeterminateEventTime => {
                formatter.write_str("session event time is indeterminate")
            }
            Self::Agenda(error) => Display::fmt(error, formatter),
            Self::Behavior(error) => Display::fmt(error, formatter),
            Self::DeltaCycleLimit => formatter.write_str("session same-time delta-cycle limit"),
        }
    }
}

impl std::error::Error for TransientSessionError {}

impl From<BehaviorError> for TransientSessionError {
    fn from(value: BehaviorError) -> Self {
        Self::Behavior(value)
    }
}

impl From<CircuitEventAgendaError> for TransientSessionError {
    fn from(value: CircuitEventAgendaError) -> Self {
        Self::Agenda(value)
    }
}

/// Complete deterministic evidence accumulated by a transient session.
#[derive(Clone, Debug, PartialEq)]
pub struct TransientSessionAuditReport {
    /// Ordinary analog transient evidence.
    pub transient: TransientRunReport,
    /// Full event lifecycle, including cancellation and rescheduling.
    pub event_lifecycle: Vec<CircuitEventLifecycleRecord>,
    /// Aggregate agenda counters.
    pub event_counters: CircuitEventCounters,
    /// Independent replay of event lifecycle evidence.
    pub event_replay: CircuitEventReplayReport,
    /// Additional retained exact breakpoint schedules used by the session.
    pub breakpoint_schedules: Vec<ExactBreakpointSchedule>,
    /// Events delivered in deterministic execution order.
    pub delivered_events: Vec<CircuitEvent>,
    /// Built-in application result for each delivered event.
    pub applications: Vec<CircuitEventApplication>,
    /// Last retained logic value at each typed target.
    pub logic_states: BTreeMap<CircuitEventTarget, LogicValue>,
    /// Last retained switch state by component.
    pub switch_states: BTreeMap<ComponentId, SwitchState>,
    /// Fingerprint covering analog decisions and retained event evidence.
    pub fingerprint: CircuitSimulationFingerprint,
}

impl TransientSessionAuditReport {
    /// Projects deterministic mixed-signal run evidence into the crate's
    /// circuit-level certification carrier.
    pub fn certification(&self) -> CircuitCertificationReport {
        let unsupported = self
            .applications
            .iter()
            .filter(|application| {
                matches!(application, CircuitEventApplication::Unsupported { .. })
            })
            .count();
        let complete = self.transient.status == TransientRunStatus::Complete
            && unsupported == 0
            && self.event_replay.is_valid();
        CircuitCertificationReport {
            status: if complete {
                "certified".into()
            } else {
                "blocked".into()
            },
            evidence: vec![
                format!("mixed-signal-fingerprint:{}", self.fingerprint.value),
                format!("accepted-analog-steps:{}", self.transient.samples.len()),
                format!("delivered-circuit-events:{}", self.event_counters.delivered),
                format!("event-replay-issues:{}", self.event_replay.issues.len()),
                format!("unsupported-event-applications:{unsupported}"),
            ],
        }
    }
}

/// Stateful façade over the pure exact transient kernels.
#[derive(Clone, Debug, PartialEq)]
pub struct TransientSession {
    circuit: Circuit,
    policy: TransientRunPolicy,
    initial_history: TransientHistory,
    history: TransientHistory,
    time: Real,
    next_timestep: Real,
    agenda: CircuitEventAgenda,
    breakpoint_schedules: Vec<ExactBreakpointSchedule>,
    samples: Vec<TransientSample>,
    decisions: Vec<TransientStepDecision>,
    delivered_events: Vec<CircuitEvent>,
    applications: Vec<CircuitEventApplication>,
    logic_states: BTreeMap<CircuitEventTarget, LogicValue>,
    switch_states: BTreeMap<ComponentId, SwitchState>,
    status: TransientSessionStatus,
}

impl TransientSession {
    /// Creates a session without changing the retained source circuit.
    pub fn new(
        circuit: &Circuit,
        policy: TransientRunPolicy,
        initial_history: TransientHistory,
        agenda: CircuitEventAgenda,
    ) -> Result<Self, TransientSessionError> {
        validate_run_policy(&policy).map_err(|_| TransientSessionError::InvalidPolicy)?;
        if agenda.time().partial_cmp(&policy.start_time) != Some(Ordering::Equal) {
            return Err(TransientSessionError::InvalidPolicy);
        }
        Ok(Self {
            circuit: circuit.clone(),
            time: policy.start_time.clone(),
            next_timestep: policy.initial_timestep.clone(),
            policy,
            initial_history: initial_history.clone(),
            history: initial_history,
            agenda,
            breakpoint_schedules: Vec::new(),
            samples: Vec::new(),
            decisions: Vec::new(),
            delivered_events: Vec::new(),
            applications: Vec::new(),
            logic_states: BTreeMap::new(),
            switch_states: BTreeMap::new(),
            status: TransientSessionStatus::Running,
        })
    }

    /// Returns the exact current session time.
    pub fn time(&self) -> &Real {
        &self.time
    }

    /// Returns the mutable event agenda for scoped authoring adapters.
    pub fn agenda_mut(&mut self) -> &mut CircuitEventAgenda {
        &mut self.agenda
    }

    /// Adds a retained exact breakpoint schedule for an external trace,
    /// topology device, protection device, or coupled-domain adapter.
    pub fn add_breakpoint_schedule(&mut self, schedule: ExactBreakpointSchedule) {
        self.breakpoint_schedules.push(schedule);
    }

    /// Returns the current session status.
    pub fn status(&self) -> TransientSessionStatus {
        self.status
    }

    /// Advances by one event-only delta action or one accepted analog endpoint.
    pub fn step(&mut self) -> Result<TransientSessionStep, TransientSessionError> {
        self.step_to_limit(self.policy.stop_time.clone())
    }

    /// Advances one action and dispatches every resulting event to registered
    /// component-scoped callbacks. Causally emitted same-time events are
    /// drained in phase/sequence order within the same bounded delta cycle.
    pub fn step_with_runtime(
        &mut self,
        runtime: &mut BehaviorRuntime,
        maximum_delta_events: usize,
    ) -> Result<TransientSessionStep, TransientSessionError> {
        if maximum_delta_events == 0 {
            return Err(TransientSessionError::DeltaCycleLimit);
        }
        let mut result = self.step_to_limit_with_delivery(self.policy.stop_time.clone(), 1)?;
        let mut dispatch_index = 0;
        loop {
            while dispatch_index < result.events.len() {
                if dispatch_index >= maximum_delta_events {
                    return Err(TransientSessionError::DeltaCycleLimit);
                }
                runtime.dispatch_if_registered(&result.events[dispatch_index], &mut self.agenda)?;
                dispatch_index += 1;
            }
            let (events, applications) = self.deliver_current_events_limit(1)?;
            if events.is_empty() {
                break;
            }
            result.events.extend(events);
            result.applications.extend(applications);
        }
        result.status = self.status;
        Ok(result)
    }

    /// Advances one action through the optional cooperative asynchronous
    /// behavior adapter, draining causally emitted same-time events.
    #[cfg(feature = "behavior-async")]
    pub fn step_with_async_runtime(
        &mut self,
        runtime: &mut AsyncBehaviorRuntime,
        maximum_delta_events: usize,
    ) -> Result<TransientSessionStep, TransientSessionError> {
        if maximum_delta_events == 0 {
            return Err(TransientSessionError::DeltaCycleLimit);
        }
        let mut result = self.step_to_limit_with_delivery(self.policy.stop_time.clone(), 1)?;
        let mut dispatch_index = 0;
        loop {
            while dispatch_index < result.events.len() {
                if dispatch_index >= maximum_delta_events {
                    return Err(TransientSessionError::DeltaCycleLimit);
                }
                runtime.dispatch(&result.events[dispatch_index], &mut self.agenda)?;
                dispatch_index += 1;
            }
            let (events, applications) = self.deliver_current_events_limit(1)?;
            if events.is_empty() {
                break;
            }
            result.events.extend(events);
            result.applications.extend(applications);
        }
        result.status = self.status;
        Ok(result)
    }

    /// Performs at most `count` incremental actions.
    pub fn steps(
        &mut self,
        count: usize,
    ) -> Result<Vec<TransientSessionStep>, TransientSessionError> {
        let mut output = Vec::new();
        for _ in 0..count {
            if self.status != TransientSessionStatus::Running {
                break;
            }
            output.push(self.step()?);
        }
        Ok(output)
    }

    /// Advances through all events and analog endpoints up to an exact time.
    pub fn step_until(
        &mut self,
        target: Real,
    ) -> Result<Vec<TransientSessionStep>, TransientSessionError> {
        if !matches!(
            self.time.partial_cmp(&target),
            Some(Ordering::Less | Ordering::Equal)
        ) || !matches!(
            target.partial_cmp(&self.policy.stop_time),
            Some(Ordering::Less | Ordering::Equal)
        ) {
            return Err(TransientSessionError::InvalidTargetTime);
        }
        let mut output = Vec::new();
        while self.status == TransientSessionStatus::Running {
            match self.time.partial_cmp(&target) {
                Some(Ordering::Less) => output.push(self.step_to_limit(target.clone())?),
                Some(Ordering::Equal) => {
                    let Some(next) = self.agenda.peek() else {
                        break;
                    };
                    match next.time.partial_cmp(&self.time) {
                        Some(Ordering::Equal) => output.push(self.step_to_limit(target.clone())?),
                        Some(Ordering::Greater) => break,
                        Some(Ordering::Less) | None => {
                            return Err(TransientSessionError::IndeterminateEventTime);
                        }
                    }
                }
                Some(Ordering::Greater) | None => {
                    return Err(TransientSessionError::InvalidTargetTime);
                }
            }
        }
        Ok(output)
    }

    /// Advances by an exact nonnegative duration.
    pub fn step_for(
        &mut self,
        duration: Real,
    ) -> Result<Vec<TransientSessionStep>, TransientSessionError> {
        let target = self.time.clone() + duration;
        self.step_until(target)
    }

    /// Advances only until the pending event agenda becomes empty.
    pub fn run_until_quiescent(
        &mut self,
    ) -> Result<Vec<TransientSessionStep>, TransientSessionError> {
        let mut output = Vec::new();
        while !self.agenda.pending().is_empty() && self.status == TransientSessionStatus::Running {
            output.push(self.step()?);
        }
        Ok(output)
    }

    /// Advances with callbacks until the pending event agenda becomes empty.
    pub fn run_until_quiescent_with_runtime(
        &mut self,
        runtime: &mut BehaviorRuntime,
        maximum_delta_events_per_step: usize,
    ) -> Result<Vec<TransientSessionStep>, TransientSessionError> {
        let mut output = Vec::new();
        while !self.agenda.pending().is_empty() && self.status == TransientSessionStatus::Running {
            output.push(self.step_with_runtime(runtime, maximum_delta_events_per_step)?);
        }
        Ok(output)
    }

    /// Advances with the optional cooperative async adapter until quiescent.
    #[cfg(feature = "behavior-async")]
    pub fn run_until_quiescent_with_async_runtime(
        &mut self,
        runtime: &mut AsyncBehaviorRuntime,
        maximum_delta_events_per_step: usize,
    ) -> Result<Vec<TransientSessionStep>, TransientSessionError> {
        let mut output = Vec::new();
        while !self.agenda.pending().is_empty() && self.status == TransientSessionStatus::Running {
            output.push(self.step_with_async_runtime(runtime, maximum_delta_events_per_step)?);
        }
        Ok(output)
    }

    /// Produces the accumulated ordinary transient report.
    pub fn report(&self) -> TransientRunReport {
        TransientRunReport {
            status: match self.status {
                TransientSessionStatus::Complete => TransientRunStatus::Complete,
                TransientSessionStatus::Bounded(status) => status,
                TransientSessionStatus::Running => TransientRunStatus::AcceptedStepLimit,
            },
            initial_history: self.initial_history.clone(),
            samples: self.samples.clone(),
            decisions: self.decisions.clone(),
            final_history: self.history.clone(),
        }
    }

    /// Produces a complete replay and audit report for analog and event execution.
    pub fn audit_report(&self) -> TransientSessionAuditReport {
        let transient = self.report();
        let agenda_fingerprint = self.agenda.fingerprint();
        let event_replay = self.agenda.verify_lifecycle();
        let canonical = format!(
            "transient={transient:?};agenda={};breakpoints={:?};events={:?};applications={:?};logic={:?};switches={:?}",
            agenda_fingerprint.value,
            self.breakpoint_schedules,
            self.delivered_events,
            self.applications,
            self.logic_states,
            self.switch_states,
        );
        TransientSessionAuditReport {
            transient,
            event_lifecycle: self.agenda.lifecycle().to_vec(),
            event_counters: self.agenda.counters().clone(),
            event_replay,
            breakpoint_schedules: self.breakpoint_schedules.clone(),
            delivered_events: self.delivered_events.clone(),
            applications: self.applications.clone(),
            logic_states: self.logic_states.clone(),
            switch_states: self.switch_states.clone(),
            fingerprint: CircuitSimulationFingerprint {
                algorithm: "hypercircuit-session-fnv1a64-pair-v1".into(),
                value: crate::event_simulation::stable_fingerprint(canonical.as_bytes()),
            },
        }
    }

    /// Returns every event delivered by the session.
    pub fn delivered_events(&self) -> &[CircuitEvent] {
        &self.delivered_events
    }

    /// Returns every event application result.
    pub fn applications(&self) -> &[CircuitEventApplication] {
        &self.applications
    }

    fn step_to_limit(
        &mut self,
        limit: Real,
    ) -> Result<TransientSessionStep, TransientSessionError> {
        self.step_to_limit_with_delivery(limit, usize::MAX)
    }

    fn step_to_limit_with_delivery(
        &mut self,
        limit: Real,
        event_limit: usize,
    ) -> Result<TransientSessionStep, TransientSessionError> {
        let (events, applications) = self.deliver_current_events_limit(event_limit)?;
        if !events.is_empty() {
            return Ok(TransientSessionStep {
                sample: None,
                events,
                applications,
                status: self.status,
            });
        }
        if self.time.partial_cmp(&limit) == Some(Ordering::Equal) {
            if self.time.partial_cmp(&self.policy.stop_time) == Some(Ordering::Equal) {
                self.status = TransientSessionStatus::Complete;
            }
            return Ok(TransientSessionStep {
                sample: None,
                events: Vec::new(),
                applications: Vec::new(),
                status: self.status,
            });
        }
        if self.samples.len() >= self.policy.maximum_accepted_steps {
            self.status = TransientSessionStatus::Bounded(TransientRunStatus::AcceptedStepLimit);
            return Ok(TransientSessionStep {
                sample: None,
                events: Vec::new(),
                applications: Vec::new(),
                status: self.status,
            });
        }
        let rejected = self
            .decisions
            .iter()
            .filter(|decision| decision.kind == TransientStepDecisionKind::Rejected)
            .count();
        if rejected >= self.policy.maximum_rejected_steps {
            self.status = TransientSessionStatus::Bounded(TransientRunStatus::RejectedStepLimit);
            return Ok(TransientSessionStep {
                sample: None,
                events: Vec::new(),
                applications: Vec::new(),
                status: self.status,
            });
        }

        let mut step_policy = self.policy.clone();
        step_policy.start_time = self.time.clone();
        step_policy.stop_time = limit;
        step_policy.initial_timestep = self.next_timestep.clone();
        step_policy.maximum_accepted_steps = 1;
        step_policy.maximum_rejected_steps = self.policy.maximum_rejected_steps - rejected;
        let mut providers =
            Vec::<&dyn ExactBreakpointProvider>::with_capacity(self.breakpoint_schedules.len() + 1);
        providers.push(&self.agenda);
        providers.extend(
            self.breakpoint_schedules
                .iter()
                .map(|schedule| schedule as &dyn ExactBreakpointProvider),
        );
        let report = self
            .circuit
            .transient_run_with_breakpoints(&step_policy, self.history.clone(), &providers)
            .map_err(TransientSessionError::Run)?;
        let decision_offset = self.decisions.len();
        self.decisions
            .extend(
                report
                    .decisions
                    .into_iter()
                    .enumerate()
                    .map(|(index, mut decision)| {
                        decision.attempt = decision_offset + index;
                        decision
                    }),
            );
        let Some(sample) = report.samples.into_iter().next() else {
            self.status = TransientSessionStatus::Bounded(report.status);
            return Ok(TransientSessionStep {
                sample: None,
                events: Vec::new(),
                applications: Vec::new(),
                status: self.status,
            });
        };
        self.agenda.advance_time(sample.time.clone())?;
        self.time = sample.time.clone();
        self.next_timestep =
            next_session_timestep(&self.policy, &sample.timestep, self.decisions.last())?;
        self.history = report.final_history;
        self.samples.push(sample.clone());
        if self.time.partial_cmp(&self.policy.stop_time) == Some(Ordering::Equal) {
            self.status = TransientSessionStatus::Complete;
        }
        let (events, applications) = self.deliver_current_events_limit(event_limit)?;
        Ok(TransientSessionStep {
            sample: Some(sample),
            events,
            applications,
            status: self.status,
        })
    }

    fn deliver_current_events_limit(
        &mut self,
        maximum: usize,
    ) -> Result<(Vec<CircuitEvent>, Vec<CircuitEventApplication>), TransientSessionError> {
        let mut events = Vec::new();
        let mut applications = Vec::new();
        while events.len() < maximum {
            let Some(next) = self.agenda.peek() else {
                break;
            };
            match next.time.partial_cmp(&self.time) {
                Some(Ordering::Equal) => {}
                Some(Ordering::Greater) => break,
                Some(Ordering::Less) | None => {
                    return Err(TransientSessionError::IndeterminateEventTime);
                }
            }
            let Some(event) = self.agenda.deliver_next() else {
                break;
            };
            let application = self.apply_event(&event);
            self.delivered_events.push(event.clone());
            self.applications.push(application.clone());
            events.push(event);
            applications.push(application);
        }
        Ok((events, applications))
    }

    fn apply_event(&mut self, event: &CircuitEvent) -> CircuitEventApplication {
        match (&event.target, &event.kind) {
            (
                CircuitEventTarget::Component(component),
                CircuitEventKind::SourceOverride { value },
            ) => {
                let Some(stimulus) = self
                    .circuit
                    .source_stimuli
                    .iter_mut()
                    .find(|stimulus| stimulus.component == *component)
                else {
                    return CircuitEventApplication::Unsupported {
                        detail: format!(
                            "component {} has no retained source stimulus",
                            component.as_str()
                        ),
                    };
                };
                stimulus.waveform = SourceWaveform::Constant(value.clone());
                CircuitEventApplication::SourceOverride {
                    component: component.clone(),
                    value: value.clone(),
                }
            }
            (
                CircuitEventTarget::Component(component),
                CircuitEventKind::SwitchTransition { state },
            ) => {
                self.switch_states.insert(component.clone(), *state);
                CircuitEventApplication::SwitchState {
                    component: component.clone(),
                    state: *state,
                }
            }
            (target, CircuitEventKind::DigitalTransition { value }) => {
                self.logic_states.insert(target.clone(), *value);
                CircuitEventApplication::LogicState {
                    target: target.clone(),
                    value: *value,
                }
            }
            (
                CircuitEventTarget::Component(component),
                CircuitEventKind::TraceSample { value, .. },
            ) => {
                let Some(stimulus) = self
                    .circuit
                    .source_stimuli
                    .iter_mut()
                    .find(|stimulus| stimulus.component == *component)
                else {
                    return CircuitEventApplication::Unsupported {
                        detail: format!(
                            "component {} has no retained source stimulus",
                            component.as_str()
                        ),
                    };
                };
                stimulus.waveform = SourceWaveform::Constant(value.clone());
                CircuitEventApplication::SourceOverride {
                    component: component.clone(),
                    value: value.clone(),
                }
            }
            _ => CircuitEventApplication::Observed,
        }
    }
}

/// Compact convergence evidence for one accepted mixed diode endpoint.
#[derive(Clone, Debug, PartialEq)]
pub struct DiodeTransientStepEvidence {
    /// Exact accepted endpoint time.
    pub time: Real,
    /// Newton attempts required at this endpoint.
    pub iterations: usize,
    /// Largest true-law KCL residual.
    pub maximum_kcl_residual: Real,
    /// Largest ideal-source constraint residual.
    pub maximum_branch_residual: Real,
    /// True-law residuals satisfied the exact authored tolerances.
    pub replay_accepted: bool,
    /// Whether the true nonlinear equations happened to replay at exact zero.
    pub exact_zero: bool,
}

/// Fixed-step mixed linear/reactive/diode transient time series.
#[derive(Clone, Debug, PartialEq)]
pub struct DiodeTransientRunReport {
    /// Terminal condition.
    pub status: TransientRunStatus,
    /// Initial reactive state supplied by the caller.
    pub initial_history: TransientHistory,
    /// Accepted stable-identity endpoint samples.
    pub samples: Vec<TransientSample>,
    /// Newton and exact-replay evidence for each sample.
    pub nonlinear_steps: Vec<DiodeTransientStepEvidence>,
    /// State at the final accepted endpoint.
    pub final_history: TransientHistory,
}

impl DiodeTransientRunReport {
    /// Extracts a complete `(time, value)` series for one MNA unknown.
    pub fn unknown_waveform(&self, unknown: &MnaUnknown) -> Option<Vec<(Real, Real)>> {
        self.samples
            .iter()
            .map(|sample| {
                sample
                    .unknowns
                    .get(unknown)
                    .cloned()
                    .map(|value| (sample.time.clone(), value))
            })
            .collect()
    }
}

/// Run-level failure before a mixed diode report can be accepted.
#[derive(Clone, Debug, PartialEq)]
pub enum DiodeTransientRunError {
    /// Time bounds or fixed-step limits are invalid.
    InvalidPolicy,
    /// Step doubling is not yet supported for nonlinear endpoints.
    UnsupportedAdaptation,
    /// One endpoint failed or did not converge.
    Step(Box<DiodeTransientStepError>),
    /// Exact time ordering could not be decided.
    IndeterminateTime,
    /// A retained source could not expose its next exact breakpoint.
    SourceBreakpoint {
        /// Stable source component identity.
        component: ComponentId,
        /// Exact waveform failure.
        error: SourceWaveformEvaluationError,
    },
    /// An external exact-breakpoint provider failed its ordering contract.
    Breakpoint(ExactBreakpointError),
}

impl Display for DiodeTransientRunError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidPolicy => formatter.write_str("invalid mixed diode transient run policy"),
            Self::UnsupportedAdaptation => {
                formatter.write_str("mixed diode transient run currently requires fixed steps")
            }
            Self::Step(error) => write!(formatter, "mixed diode transient step failed: {error}"),
            Self::IndeterminateTime => {
                formatter.write_str("mixed diode transient time ordering is indeterminate")
            }
            Self::SourceBreakpoint { component, error } => write!(
                formatter,
                "source {} breakpoint evaluation failed: {error}",
                component.as_str()
            ),
            Self::Breakpoint(error) => Display::fmt(error, formatter),
        }
    }
}

impl std::error::Error for DiodeTransientRunError {}

/// Structural or executable failure before a bounded report can be returned.
#[derive(Clone, Debug, PartialEq)]
pub enum TransientRunError {
    /// Time bounds, step bounds, adaptation factors, or tolerances were invalid.
    InvalidPolicy,
    /// The exact companion-step kernel failed.
    Step(TransientStepError),
    /// Step-doubling candidates did not expose the same stable state identities.
    IncompatibleStepState,
    /// Exact error ordering or ratio arithmetic was indeterminate.
    IndeterminateErrorEstimate,
    /// A retained source could not expose its next exact breakpoint.
    SourceBreakpoint {
        /// Stable source component identity.
        component: ComponentId,
        /// Exact waveform failure.
        error: SourceWaveformEvaluationError,
    },
    /// An external exact-breakpoint provider failed its ordering contract.
    Breakpoint(ExactBreakpointError),
}

impl Display for TransientRunError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidPolicy => formatter.write_str("invalid transient run policy"),
            Self::Step(error) => write!(formatter, "transient step failed: {error}"),
            Self::IncompatibleStepState => {
                formatter.write_str("step-doubling states have incompatible identities")
            }
            Self::IndeterminateErrorEstimate => {
                formatter.write_str("exact step-doubling error estimate was indeterminate")
            }
            Self::SourceBreakpoint { component, error } => write!(
                formatter,
                "source {} breakpoint evaluation failed: {error}",
                component.as_str()
            ),
            Self::Breakpoint(error) => Display::fmt(error, formatter),
        }
    }
}

impl std::error::Error for TransientRunError {}

#[derive(Clone)]
struct ReactiveCompanion {
    component: ComponentId,
    pos: Option<NetId>,
    neg: Option<NetId>,
    conductance: Real,
    history_current: Real,
}

impl Circuit {
    /// Lowers supported primitive models at DC time zero.
    pub fn lower_linear_devices(&self) -> LinearDeviceLoweringReport {
        self.lower_linear_devices_at(&Real::zero())
    }

    /// Lowers supported primitive models at an exact simulation time.
    ///
    /// A retained [`SourceWaveform`] overrides the scalar `current` or
    /// `voltage` parameter of its target independent source. Evaluated values
    /// remain in the report as replay evidence.
    pub fn lower_linear_devices_at(&self, time: &Real) -> LinearDeviceLoweringReport {
        if !self.validate().is_valid() {
            return LinearDeviceLoweringReport {
                stamps: Vec::new(),
                issues: vec![DeviceLoweringIssue::InvalidCircuit],
                notes: Vec::new(),
                source_values: BTreeMap::new(),
            };
        }
        let mut report = LinearDeviceLoweringReport {
            stamps: Vec::new(),
            issues: Vec::new(),
            notes: Vec::new(),
            source_values: BTreeMap::new(),
        };
        for instance in &self.instances {
            let model = self
                .device_models
                .iter()
                .find(|model| model.id == instance.model)
                .expect("validated instance model must exist");
            lower_instance_at(self, instance, model, time, &mut report);
        }
        report
    }

    /// Builds a linear MNA system directly from supported retained device models.
    pub fn linear_mna_from_devices(&self) -> Result<LinearMnaSystem, DeviceLoweringError> {
        let lowered = self.lower_linear_devices();
        if !lowered.is_complete() {
            return Err(DeviceLoweringError::Incomplete(lowered.issues));
        }
        LinearMnaSystem::from_stamps(
            self.nets
                .iter()
                .filter(|net| !net.is_ground)
                .map(|net| net.id.clone())
                .collect(),
            &lowered.stamps,
        )
        .map_err(DeviceLoweringError::Assembly)
    }

    /// Executes one exact linear capacitor/inductor companion step.
    ///
    /// `Trapezoidal` and first-order `GearBdf` policies are executable. Missing
    /// component history means a zero-voltage, zero-current initial condition.
    /// Every accepted solution is replayed through the generated exact MNA
    /// equations before the next history is returned.
    pub fn transient_step(
        &self,
        timestep: Real,
        history: &TransientHistory,
    ) -> Result<TransientStepReport, TransientStepError> {
        self.transient_step_at(timestep.clone(), timestep, history)
    }

    /// Executes one exact transient step ending at `time`.
    ///
    /// The explicit endpoint is required for time-varying source evaluation.
    /// [`Circuit::transient_step`] remains the time-zero convenience path.
    pub fn transient_step_at(
        &self,
        time: Real,
        timestep: Real,
        history: &TransientHistory,
    ) -> Result<TransientStepReport, TransientStepError> {
        if !self.validate().is_valid() {
            return Err(TransientStepError::InvalidCircuit);
        }
        if timestep.structural_facts().sign != Some(RealSign::Positive) {
            return Err(TransientStepError::InvalidTimestep);
        }
        if !matches!(
            self.transient_policy,
            TransientPolicy::Trapezoidal | TransientPolicy::GearBdf { order: 1 }
        ) {
            return Err(TransientStepError::UnsupportedPolicy(
                self.transient_policy.clone(),
            ));
        }

        let mut lowering = LinearDeviceLoweringReport {
            stamps: Vec::new(),
            issues: Vec::new(),
            notes: Vec::new(),
            source_values: BTreeMap::new(),
        };
        let mut reactive = Vec::new();
        for instance in &self.instances {
            let model = self
                .device_models
                .iter()
                .find(|model| model.id == instance.model)
                .expect("validated instance model must exist");
            if matches!(
                model.kind,
                DeviceModelKind::Capacitor | DeviceModelKind::Inductor
            ) {
                reactive.push(self.reactive_companion(instance, model, &timestep, history)?);
            } else {
                lower_instance_at(self, instance, model, &time, &mut lowering);
            }
        }
        if !lowering.is_complete() {
            return Err(TransientStepError::Incomplete(lowering.issues));
        }
        lowering
            .stamps
            .extend(reactive.iter().map(|companion| LinearStamp::Companion {
                component: companion.component.clone(),
                pos: companion.pos.clone(),
                neg: companion.neg.clone(),
                conductance: companion.conductance.clone(),
                history_current: companion.history_current.clone(),
            }));
        let system = LinearMnaSystem::from_stamps(
            self.nets
                .iter()
                .filter(|net| !net.is_ground)
                .map(|net| net.id.clone())
                .collect(),
            &lowering.stamps,
        )
        .map_err(TransientStepError::Solve)?;
        let solution = system.solve_exact().map_err(TransientStepError::Solve)?;
        let mut next_history = TransientHistory::default();
        for companion in reactive {
            let voltage = terminal_voltage(&system, &solution, &companion.pos)
                - terminal_voltage(&system, &solution, &companion.neg);
            let current = companion.conductance * voltage.clone() + companion.history_current;
            next_history
                .reactive
                .insert(companion.component, ReactiveState { voltage, current });
        }
        Ok(TransientStepReport {
            time,
            timestep,
            stamps: lowering.stamps,
            unknowns: system.unknowns,
            solution,
            next_history,
            source_values: lowering.source_values,
        })
    }

    /// Executes one converged mixed linear/reactive/diode transient endpoint.
    ///
    /// Linear and companion stamps are exact. Diode Newton linearizations are
    /// explicitly lossy proposals, while every accepted endpoint replays the
    /// true retained Shockley law against exact authored tolerances.
    pub fn diode_transient_step_at(
        &self,
        time: Real,
        timestep: Real,
        history: &TransientHistory,
        newton: &DiodeNewtonPolicy,
        initial_voltages: &BTreeMap<NetId, Real>,
    ) -> Result<DiodeTransientStepReport, DiodeTransientStepError> {
        if !self.validate().is_valid() {
            return Err(DiodeTransientStepError::InvalidCircuit);
        }
        if timestep.structural_facts().sign != Some(RealSign::Positive) {
            return Err(DiodeTransientStepError::InvalidTimestep);
        }
        if !matches!(
            self.transient_policy,
            TransientPolicy::Trapezoidal | TransientPolicy::GearBdf { order: 1 }
        ) {
            return Err(DiodeTransientStepError::UnsupportedPolicy(
                self.transient_policy.clone(),
            ));
        }

        let diodes = self
            .shockley_diodes()
            .map_err(DiodeTransientStepError::Nonlinear)?;
        let mut lowering = LinearDeviceLoweringReport {
            stamps: Vec::new(),
            issues: Vec::new(),
            notes: Vec::new(),
            source_values: BTreeMap::new(),
        };
        let mut reactive = Vec::new();
        for instance in &self.instances {
            let model = self
                .device_models
                .iter()
                .find(|model| model.id == instance.model)
                .expect("validated instance model must exist");
            match model.kind {
                DeviceModelKind::Capacitor | DeviceModelKind::Inductor => {
                    reactive.push(
                        self.reactive_companion(instance, model, &timestep, history)
                            .map_err(DiodeTransientStepError::Reactive)?,
                    );
                }
                DeviceModelKind::Diode => {}
                _ => lower_instance_at(self, instance, model, &time, &mut lowering),
            }
        }
        if !lowering.is_complete() {
            return Err(DiodeTransientStepError::Incomplete(lowering.issues));
        }
        lowering
            .stamps
            .extend(reactive.iter().map(|companion| LinearStamp::Companion {
                component: companion.component.clone(),
                pos: companion.pos.clone(),
                neg: companion.neg.clone(),
                conductance: companion.conductance.clone(),
                history_current: companion.history_current.clone(),
            }));
        let nonlinear = solve_shockley_diode_newton(
            self.nets
                .iter()
                .filter(|net| !net.is_ground)
                .map(|net| net.id.clone())
                .collect(),
            &lowering.stamps,
            &diodes,
            newton,
            initial_voltages,
        )
        .map_err(DiodeTransientStepError::Nonlinear)?;
        if nonlinear.status != DiodeNewtonStatus::Converged {
            return Err(DiodeTransientStepError::IterationLimit(Box::new(nonlinear)));
        }
        let values = nonlinear
            .unknowns
            .iter()
            .cloned()
            .zip(nonlinear.candidate.iter().cloned())
            .collect::<BTreeMap<_, _>>();
        let mut next_history = TransientHistory::default();
        for companion in reactive {
            let voltage = nonlinear_terminal_voltage(&values, &companion.pos)
                - nonlinear_terminal_voltage(&values, &companion.neg);
            let current = companion.conductance * voltage.clone() + companion.history_current;
            next_history
                .reactive
                .insert(companion.component, ReactiveState { voltage, current });
        }
        Ok(DiodeTransientStepReport {
            time,
            timestep,
            linear_stamps: lowering.stamps,
            nonlinear,
            next_history,
            source_values: lowering.source_values,
        })
    }

    /// Runs a bounded fixed-step mixed linear/reactive/diode time series.
    pub fn diode_transient_run(
        &self,
        policy: &TransientRunPolicy,
        newton: &DiodeNewtonPolicy,
        initial_history: TransientHistory,
        initial_voltages: BTreeMap<NetId, Real>,
    ) -> Result<DiodeTransientRunReport, DiodeTransientRunError> {
        self.diode_transient_run_with_breakpoints(
            policy,
            newton,
            initial_history,
            initial_voltages,
            &[],
        )
    }

    /// Runs a bounded mixed diode time series while honoring generalized exact
    /// breakpoint providers for events, switches, protection, and trace input.
    pub fn diode_transient_run_with_breakpoints(
        &self,
        policy: &TransientRunPolicy,
        newton: &DiodeNewtonPolicy,
        initial_history: TransientHistory,
        initial_voltages: BTreeMap<NetId, Real>,
        breakpoint_providers: &[&dyn ExactBreakpointProvider],
    ) -> Result<DiodeTransientRunReport, DiodeTransientRunError> {
        validate_run_policy(policy).map_err(|_| DiodeTransientRunError::InvalidPolicy)?;
        if !matches!(policy.adaptation, TransientAdaptation::Fixed) {
            return Err(DiodeTransientRunError::UnsupportedAdaptation);
        }
        let mut time = policy.start_time.clone();
        let mut history = initial_history.clone();
        let mut guesses = initial_voltages;
        let mut samples = Vec::new();
        let mut nonlinear_steps = Vec::new();
        let status = loop {
            match time.partial_cmp(&policy.stop_time) {
                Some(Ordering::Equal) => break TransientRunStatus::Complete,
                Some(Ordering::Less) => {}
                None => return Err(DiodeTransientRunError::IndeterminateTime),
                Some(Ordering::Greater) => return Err(DiodeTransientRunError::InvalidPolicy),
            }
            if samples.len() >= policy.maximum_accepted_steps {
                break TransientRunStatus::AcceptedStepLimit;
            }
            let remaining = policy.stop_time.clone() - time.clone();
            let proposed = exact_min(&policy.initial_timestep, &remaining)
                .ok_or(DiodeTransientRunError::IndeterminateTime)?;
            let timestep = self
                .event_limited_timestep(&time, &proposed, breakpoint_providers)
                .map_err(|error| match error {
                    TransientRunError::SourceBreakpoint { component, error } => {
                        DiodeTransientRunError::SourceBreakpoint { component, error }
                    }
                    TransientRunError::Breakpoint(error) => {
                        DiodeTransientRunError::Breakpoint(error)
                    }
                    _ => DiodeTransientRunError::IndeterminateTime,
                })?;
            let endpoint = time.clone() + timestep.clone();
            let step = self
                .diode_transient_step_at(
                    endpoint.clone(),
                    timestep.clone(),
                    &history,
                    newton,
                    &guesses,
                )
                .map_err(|error| DiodeTransientRunError::Step(Box::new(error)))?;
            let unknowns = step
                .nonlinear
                .unknowns
                .iter()
                .cloned()
                .zip(step.nonlinear.candidate.iter().cloned())
                .collect::<BTreeMap<_, _>>();
            guesses = unknowns
                .iter()
                .filter_map(|(unknown, value)| match unknown {
                    MnaUnknown::NetVoltage(net) => Some((net.clone(), value.clone())),
                    MnaUnknown::BranchCurrent(_) => None,
                })
                .collect();
            nonlinear_steps.push(DiodeTransientStepEvidence {
                time: endpoint.clone(),
                iterations: step.nonlinear.iterations.len(),
                maximum_kcl_residual: step.nonlinear.replay.maximum_kcl_residual.clone(),
                maximum_branch_residual: step.nonlinear.replay.maximum_branch_residual.clone(),
                replay_accepted: step.nonlinear.replay.accepted,
                exact_zero: step.nonlinear.replay.exact_zero,
            });
            history = step.next_history.clone();
            samples.push(TransientSample {
                time: endpoint.clone(),
                timestep,
                unknowns,
                reactive: history.reactive.clone(),
                source_values: step.source_values,
            });
            time = endpoint;
        };
        Ok(DiodeTransientRunReport {
            status,
            initial_history,
            samples,
            nonlinear_steps,
            final_history: history,
        })
    }

    /// Runs a complete or explicitly bounded exact transient time series.
    ///
    /// Adaptive runs use one full step only as an error proposal. An accepted
    /// endpoint always comes from the second of two exact half steps, each of
    /// which has passed the ordinary MNA residual replay.
    pub fn transient_run(
        &self,
        policy: &TransientRunPolicy,
        initial_history: TransientHistory,
    ) -> Result<TransientRunReport, TransientRunError> {
        self.transient_run_with_breakpoints(policy, initial_history, &[])
    }

    /// Runs a bounded exact transient time series while honoring additional
    /// exact breakpoint providers such as a scheduled circuit-event agenda.
    pub fn transient_run_with_breakpoints(
        &self,
        policy: &TransientRunPolicy,
        initial_history: TransientHistory,
        breakpoint_providers: &[&dyn ExactBreakpointProvider],
    ) -> Result<TransientRunReport, TransientRunError> {
        validate_run_policy(policy)?;
        let mut time = policy.start_time.clone();
        let mut timestep = policy.initial_timestep.clone();
        let mut history = initial_history.clone();
        let mut samples = Vec::new();
        let mut decisions = Vec::new();
        let mut rejected_steps = 0_usize;
        let status = loop {
            match time.partial_cmp(&policy.stop_time) {
                Some(Ordering::Equal) => break TransientRunStatus::Complete,
                Some(Ordering::Less) => {}
                _ => return Err(TransientRunError::InvalidPolicy),
            }
            if samples.len() >= policy.maximum_accepted_steps {
                break TransientRunStatus::AcceptedStepLimit;
            }
            let remaining = policy.stop_time.clone() - time.clone();
            let proposed = exact_min(&timestep, &remaining)
                .ok_or(TransientRunError::IndeterminateErrorEstimate)?;
            let attempted = self.event_limited_timestep(&time, &proposed, breakpoint_providers)?;
            let attempt = decisions.len();
            match &policy.adaptation {
                TransientAdaptation::Fixed => {
                    let endpoint = time.clone() + attempted.clone();
                    let step = self
                        .transient_step_at(endpoint, attempted.clone(), &history)
                        .map_err(TransientRunError::Step)?;
                    decisions.push(TransientStepDecision {
                        attempt,
                        start_time: time.clone(),
                        timestep: attempted.clone(),
                        kind: TransientStepDecisionKind::Accepted,
                        maximum_error_ratio: None,
                    });
                    time += attempted.clone();
                    history = step.next_history.clone();
                    samples.push(transient_sample(time.clone(), attempted, &step)?);
                }
                TransientAdaptation::StepDoubling {
                    absolute_tolerance,
                    relative_tolerance,
                    shrink_factor,
                    growth_factor,
                } => {
                    let endpoint = time.clone() + attempted.clone();
                    let coarse = self
                        .transient_step_at(endpoint.clone(), attempted.clone(), &history)
                        .map_err(TransientRunError::Step)?;
                    let half = (attempted.clone() / Real::from(2))
                        .map_err(|_| TransientRunError::InvalidPolicy)?;
                    let midpoint = time.clone() + half.clone();
                    let first_half = self
                        .transient_step_at(midpoint, half.clone(), &history)
                        .map_err(TransientRunError::Step)?;
                    let second_half = self
                        .transient_step_at(endpoint, half, &first_half.next_history)
                        .map_err(TransientRunError::Step)?;
                    let error_ratio = step_error_ratio(
                        &coarse,
                        &second_half,
                        absolute_tolerance,
                        relative_tolerance,
                    )?;
                    let accepted = match error_ratio.partial_cmp(&Real::one()) {
                        Some(Ordering::Less | Ordering::Equal) => true,
                        Some(Ordering::Greater) => false,
                        None => return Err(TransientRunError::IndeterminateErrorEstimate),
                    };
                    decisions.push(TransientStepDecision {
                        attempt,
                        start_time: time.clone(),
                        timestep: attempted.clone(),
                        kind: if accepted {
                            TransientStepDecisionKind::Accepted
                        } else {
                            TransientStepDecisionKind::Rejected
                        },
                        maximum_error_ratio: Some(error_ratio.clone()),
                    });
                    if accepted {
                        time += attempted.clone();
                        history = second_half.next_history.clone();
                        samples.push(transient_sample(
                            time.clone(),
                            attempted.clone(),
                            &second_half,
                        )?);
                        let quarter =
                            (Real::one() / Real::from(4)).expect("quarter denominator is nonzero");
                        match error_ratio.partial_cmp(&quarter) {
                            Some(Ordering::Less | Ordering::Equal) => {
                                let grown = attempted * growth_factor.clone();
                                timestep = exact_min(&grown, &policy.maximum_timestep)
                                    .ok_or(TransientRunError::IndeterminateErrorEstimate)?;
                            }
                            Some(Ordering::Greater) => timestep = attempted,
                            None => {
                                return Err(TransientRunError::IndeterminateErrorEstimate);
                            }
                        }
                    } else {
                        rejected_steps += 1;
                        if rejected_steps >= policy.maximum_rejected_steps {
                            break TransientRunStatus::RejectedStepLimit;
                        }
                        let shrunk = attempted * shrink_factor.clone();
                        match shrunk.partial_cmp(&policy.minimum_timestep) {
                            Some(Ordering::Less) => {
                                break TransientRunStatus::MinimumTimestep;
                            }
                            Some(Ordering::Equal | Ordering::Greater) => timestep = shrunk,
                            None => {
                                return Err(TransientRunError::IndeterminateErrorEstimate);
                            }
                        }
                    }
                }
            }
        };
        Ok(TransientRunReport {
            status,
            initial_history,
            samples,
            decisions,
            final_history: history,
        })
    }

    fn reactive_companion(
        &self,
        instance: &CircuitInstance,
        model: &DeviceModel,
        timestep: &Real,
        history: &TransientHistory,
    ) -> Result<ReactiveCompanion, TransientStepError> {
        let pins = model
            .pins
            .iter()
            .filter_map(|pin| {
                instance
                    .pins
                    .iter()
                    .find(|binding| binding.pin == pin.pin)
                    .map(|binding| net_terminal(self, &binding.net))
            })
            .collect::<Vec<_>>();
        if pins.len() < 2 {
            return Err(TransientStepError::Incomplete(vec![
                DeviceLoweringIssue::MissingPins {
                    component: instance.component.clone(),
                    required: 2,
                },
            ]));
        }
        let state = history
            .reactive
            .get(&instance.component)
            .cloned()
            .unwrap_or_default();
        let parameter_name = match model.kind {
            DeviceModelKind::Capacitor => "capacitance",
            DeviceModelKind::Inductor => "inductance",
            _ => unreachable!(),
        };
        let Some(value) = parameter(instance, model, parameter_name) else {
            return Err(TransientStepError::InvalidReactiveParameter {
                component: instance.component.clone(),
                parameter: parameter_name.into(),
            });
        };
        if value.structural_facts().sign != Some(RealSign::Positive) {
            return Err(TransientStepError::InvalidReactiveParameter {
                component: instance.component.clone(),
                parameter: parameter_name.into(),
            });
        }
        let (conductance, history_current) = match (&model.kind, &self.transient_policy) {
            (DeviceModelKind::Capacitor, TransientPolicy::GearBdf { order: 1 }) => {
                let conductance = (value.clone() / timestep.clone())
                    .map_err(|_| TransientStepError::Arithmetic)?;
                let history_current = -(conductance.clone() * state.voltage);
                (conductance, history_current)
            }
            (DeviceModelKind::Capacitor, TransientPolicy::Trapezoidal) => {
                let conductance = (Real::from(2) * value.clone() / timestep.clone())
                    .map_err(|_| TransientStepError::Arithmetic)?;
                let history_current = -(conductance.clone() * state.voltage) - state.current;
                (conductance, history_current)
            }
            (DeviceModelKind::Inductor, TransientPolicy::GearBdf { order: 1 }) => {
                let conductance = (timestep.clone() / value.clone())
                    .map_err(|_| TransientStepError::Arithmetic)?;
                (conductance, state.current)
            }
            (DeviceModelKind::Inductor, TransientPolicy::Trapezoidal) => {
                let conductance = (timestep.clone() / (Real::from(2) * value.clone()))
                    .map_err(|_| TransientStepError::Arithmetic)?;
                let history_current = state.current + conductance.clone() * state.voltage;
                (conductance, history_current)
            }
            _ => unreachable!(),
        };
        Ok(ReactiveCompanion {
            component: instance.component.clone(),
            pos: pins[0].clone(),
            neg: pins[1].clone(),
            conductance,
            history_current,
        })
    }

    fn source_event_limited_timestep(
        &self,
        time: &Real,
        proposed: &Real,
    ) -> Result<Real, (ComponentId, SourceWaveformEvaluationError)> {
        let mut endpoint = time.clone() + proposed.clone();
        for stimulus in &self.source_stimuli {
            let breakpoint = stimulus
                .waveform
                .next_breakpoint_after(time)
                .map_err(|error| (stimulus.component.clone(), error))?;
            let Some(breakpoint) = breakpoint else {
                continue;
            };
            match breakpoint.partial_cmp(&endpoint) {
                Some(Ordering::Less) => endpoint = breakpoint,
                Some(Ordering::Equal | Ordering::Greater) => {}
                None => {
                    return Err((
                        stimulus.component.clone(),
                        SourceWaveformEvaluationError::IndeterminateTime,
                    ));
                }
            }
        }
        Ok(endpoint - time.clone())
    }

    fn event_limited_timestep(
        &self,
        time: &Real,
        proposed: &Real,
        providers: &[&dyn ExactBreakpointProvider],
    ) -> Result<Real, TransientRunError> {
        let source_limited =
            self.source_event_limited_timestep(time, proposed)
                .map_err(|(component, error)| TransientRunError::SourceBreakpoint {
                    component,
                    error,
                })?;
        let mut endpoint = time.clone() + source_limited;
        if let Some(breakpoint) = earliest_exact_breakpoint_after(time, providers)
            .map_err(TransientRunError::Breakpoint)?
        {
            match breakpoint.partial_cmp(&endpoint) {
                Some(Ordering::Less) => endpoint = breakpoint,
                Some(Ordering::Equal | Ordering::Greater) => {}
                None => {
                    return Err(TransientRunError::Breakpoint(ExactBreakpointError {
                        provider: "combined-breakpoint-set".into(),
                        detail: "breakpoint cannot be ordered against proposed endpoint".into(),
                    }));
                }
            }
        }
        Ok(endpoint - time.clone())
    }
}

fn terminal_voltage(
    system: &LinearMnaSystem,
    solution: &LinearSolveReport,
    terminal: &Option<NetId>,
) -> Real {
    let Some(net) = terminal else {
        return Real::zero();
    };
    system
        .unknowns
        .iter()
        .position(
            |unknown| matches!(unknown, MnaUnknown::NetVoltage(candidate) if candidate == net),
        )
        .map(|index| solution.candidate[index].clone())
        .expect("assembled non-ground terminal must have a voltage unknown")
}

fn nonlinear_terminal_voltage(
    values: &BTreeMap<MnaUnknown, Real>,
    terminal: &Option<NetId>,
) -> Real {
    terminal
        .as_ref()
        .and_then(|net| values.get(&MnaUnknown::NetVoltage(net.clone())))
        .cloned()
        .unwrap_or_else(Real::zero)
}

fn validate_run_policy(policy: &TransientRunPolicy) -> Result<(), TransientRunError> {
    let positive = |value: &Real| value.structural_facts().sign == Some(RealSign::Positive);
    let nonnegative = |value: &Real| {
        matches!(
            value.structural_facts().sign,
            Some(RealSign::Positive | RealSign::Zero)
        )
    };
    if policy.start_time.partial_cmp(&policy.stop_time) != Some(Ordering::Less)
        || !positive(&policy.initial_timestep)
        || !positive(&policy.minimum_timestep)
        || !positive(&policy.maximum_timestep)
        || !matches!(
            policy
                .minimum_timestep
                .partial_cmp(&policy.initial_timestep),
            Some(Ordering::Less | Ordering::Equal)
        )
        || !matches!(
            policy
                .initial_timestep
                .partial_cmp(&policy.maximum_timestep),
            Some(Ordering::Less | Ordering::Equal)
        )
        || policy.maximum_accepted_steps == 0
        || policy.maximum_rejected_steps == 0
    {
        return Err(TransientRunError::InvalidPolicy);
    }
    match &policy.adaptation {
        TransientAdaptation::StepDoubling {
            absolute_tolerance,
            relative_tolerance,
            shrink_factor,
            growth_factor,
        } if !positive(absolute_tolerance)
            || !nonnegative(relative_tolerance)
            || !positive(shrink_factor)
            || shrink_factor.partial_cmp(&Real::one()) != Some(Ordering::Less)
            || growth_factor.partial_cmp(&Real::one()) != Some(Ordering::Greater) =>
        {
            return Err(TransientRunError::InvalidPolicy);
        }
        TransientAdaptation::Fixed | TransientAdaptation::StepDoubling { .. } => {}
    }
    Ok(())
}

fn transient_sample(
    time: Real,
    timestep: Real,
    step: &TransientStepReport,
) -> Result<TransientSample, TransientRunError> {
    if step.unknowns.len() != step.solution.candidate.len() {
        return Err(TransientRunError::IncompatibleStepState);
    }
    Ok(TransientSample {
        time,
        timestep,
        unknowns: step
            .unknowns
            .iter()
            .cloned()
            .zip(step.solution.candidate.iter().cloned())
            .collect(),
        reactive: step.next_history.reactive.clone(),
        source_values: step.source_values.clone(),
    })
}

fn step_error_ratio(
    coarse: &TransientStepReport,
    refined: &TransientStepReport,
    absolute_tolerance: &Real,
    relative_tolerance: &Real,
) -> Result<Real, TransientRunError> {
    if coarse.unknowns != refined.unknowns
        || coarse.solution.candidate.len() != refined.solution.candidate.len()
        || coarse.source_values != refined.source_values
    {
        return Err(TransientRunError::IncompatibleStepState);
    }
    let coarse_components = coarse
        .next_history
        .reactive
        .keys()
        .cloned()
        .collect::<BTreeSet<_>>();
    let refined_components = refined
        .next_history
        .reactive
        .keys()
        .cloned()
        .collect::<BTreeSet<_>>();
    if coarse_components != refined_components {
        return Err(TransientRunError::IncompatibleStepState);
    }
    let mut maximum = Real::zero();
    for (coarse, refined) in coarse
        .solution
        .candidate
        .iter()
        .zip(&refined.solution.candidate)
    {
        update_error_ratio(
            &mut maximum,
            coarse,
            refined,
            absolute_tolerance,
            relative_tolerance,
        )?;
    }
    for component in coarse_components {
        let coarse = &coarse.next_history.reactive[&component];
        let refined = &refined.next_history.reactive[&component];
        for (coarse, refined) in [
            (&coarse.voltage, &refined.voltage),
            (&coarse.current, &refined.current),
        ] {
            update_error_ratio(
                &mut maximum,
                coarse,
                refined,
                absolute_tolerance,
                relative_tolerance,
            )?;
        }
    }
    Ok(maximum)
}

fn update_error_ratio(
    maximum: &mut Real,
    coarse: &Real,
    refined: &Real,
    absolute_tolerance: &Real,
    relative_tolerance: &Real,
) -> Result<(), TransientRunError> {
    let error = real_abs(&(refined.clone() - coarse.clone()));
    let scale = exact_max(&real_abs(coarse), &real_abs(refined))
        .ok_or(TransientRunError::IndeterminateErrorEstimate)?;
    let tolerance = absolute_tolerance.clone() + relative_tolerance.clone() * scale;
    let ratio = (error / tolerance).map_err(|_| TransientRunError::IndeterminateErrorEstimate)?;
    match ratio.partial_cmp(maximum) {
        Some(Ordering::Greater) => *maximum = ratio,
        Some(Ordering::Less | Ordering::Equal) => {}
        None => return Err(TransientRunError::IndeterminateErrorEstimate),
    }
    Ok(())
}

fn exact_min(first: &Real, second: &Real) -> Option<Real> {
    match first.partial_cmp(second)? {
        Ordering::Less | Ordering::Equal => Some(first.clone()),
        Ordering::Greater => Some(second.clone()),
    }
}

fn next_session_timestep(
    policy: &TransientRunPolicy,
    accepted: &Real,
    decision: Option<&TransientStepDecision>,
) -> Result<Real, TransientSessionError> {
    let TransientAdaptation::StepDoubling { growth_factor, .. } = &policy.adaptation else {
        return Ok(accepted.clone());
    };
    let Some(ratio) = decision.and_then(|decision| decision.maximum_error_ratio.as_ref()) else {
        return Ok(accepted.clone());
    };
    let quarter = (Real::one() / Real::from(4))
        .map_err(|_| TransientSessionError::Run(TransientRunError::InvalidPolicy))?;
    match ratio.partial_cmp(&quarter) {
        Some(Ordering::Less | Ordering::Equal) => {
            let grown = accepted.clone() * growth_factor.clone();
            exact_min(&grown, &policy.maximum_timestep).ok_or(TransientSessionError::Run(
                TransientRunError::IndeterminateErrorEstimate,
            ))
        }
        Some(Ordering::Greater) => Ok(accepted.clone()),
        None => Err(TransientSessionError::Run(
            TransientRunError::IndeterminateErrorEstimate,
        )),
    }
}

fn exact_max(first: &Real, second: &Real) -> Option<Real> {
    match first.partial_cmp(second)? {
        Ordering::Less => Some(second.clone()),
        Ordering::Equal | Ordering::Greater => Some(first.clone()),
    }
}

fn real_abs(value: &Real) -> Real {
    if value.structural_facts().sign == Some(RealSign::Negative) {
        -value.clone()
    } else {
        value.clone()
    }
}

fn lower_instance_at(
    circuit: &Circuit,
    instance: &CircuitInstance,
    model: &DeviceModel,
    time: &Real,
    report: &mut LinearDeviceLoweringReport,
) {
    let source_value = if matches!(
        model.kind,
        DeviceModelKind::CurrentSource | DeviceModelKind::VoltageSource
    ) {
        let Some(stimulus) = circuit
            .source_stimuli
            .iter()
            .find(|stimulus| stimulus.component == instance.component)
        else {
            lower_instance(circuit, instance, model, None, report);
            return;
        };
        match stimulus.waveform.value_at(time) {
            Ok(value) => {
                report
                    .source_values
                    .insert(instance.component.clone(), value.clone());
                Some(value)
            }
            Err(_) => {
                report.issues.push(DeviceLoweringIssue::SourceEvaluation(
                    instance.component.clone(),
                ));
                return;
            }
        }
    } else {
        None
    };
    lower_instance(circuit, instance, model, source_value.as_ref(), report);
}

fn lower_instance(
    circuit: &Circuit,
    instance: &CircuitInstance,
    model: &DeviceModel,
    source_value: Option<&Real>,
    report: &mut LinearDeviceLoweringReport,
) {
    let pins = model
        .pins
        .iter()
        .filter_map(|pin| {
            instance
                .pins
                .iter()
                .find(|binding| binding.pin == pin.pin)
                .map(|binding| net_terminal(circuit, &binding.net))
        })
        .collect::<Vec<_>>();
    let required = match &model.kind {
        DeviceModelKind::ControlledSource => 4,
        DeviceModelKind::Resistor
        | DeviceModelKind::Capacitor
        | DeviceModelKind::Inductor
        | DeviceModelKind::CurrentSource
        | DeviceModelKind::VoltageSource => 2,
        DeviceModelKind::Diode
        | DeviceModelKind::Mosfet { .. }
        | DeviceModelKind::NonlinearPlaceholder
        | DeviceModelKind::Custom(_) => {
            report.issues.push(DeviceLoweringIssue::UnsupportedModel(
                instance.component.clone(),
            ));
            return;
        }
    };
    if pins.len() < required {
        report.issues.push(DeviceLoweringIssue::MissingPins {
            component: instance.component.clone(),
            required,
        });
        return;
    }
    match &model.kind {
        DeviceModelKind::Resistor => {
            let conductance = if let Some(value) = parameter(instance, model, "conductance") {
                value.clone()
            } else if let Some(resistance) = parameter(instance, model, "resistance") {
                if resistance.structural_facts().sign != Some(RealSign::Positive) {
                    report.issues.push(DeviceLoweringIssue::InvalidResistance(
                        instance.component.clone(),
                    ));
                    return;
                }
                let Ok(conductance) = Real::one() / resistance.clone() else {
                    report.issues.push(DeviceLoweringIssue::InvalidResistance(
                        instance.component.clone(),
                    ));
                    return;
                };
                conductance
            } else {
                missing_parameter(report, instance, "resistance or conductance");
                return;
            };
            report.stamps.push(LinearStamp::Conductance {
                component: instance.component.clone(),
                part: instance.part.clone(),
                pos: pins[0].clone(),
                neg: pins[1].clone(),
                conductance,
            });
        }
        DeviceModelKind::CurrentSource => {
            let Some(current) = source_value.or_else(|| parameter(instance, model, "current"))
            else {
                missing_parameter(report, instance, "current");
                return;
            };
            report.stamps.push(LinearStamp::CurrentSource {
                component: instance.component.clone(),
                pos: pins[0].clone(),
                neg: pins[1].clone(),
                current: current.clone(),
            });
        }
        DeviceModelKind::VoltageSource => {
            let Some(voltage) = source_value.or_else(|| parameter(instance, model, "voltage"))
            else {
                missing_parameter(report, instance, "voltage");
                return;
            };
            report.stamps.push(LinearStamp::VoltageSource {
                component: instance.component.clone(),
                branch: branch(instance, "voltage"),
                pos: pins[0].clone(),
                neg: pins[1].clone(),
                voltage: voltage.clone(),
            });
        }
        DeviceModelKind::ControlledSource => {
            let Some(transconductance) = parameter(instance, model, "transconductance") else {
                missing_parameter(report, instance, "transconductance");
                return;
            };
            report.stamps.push(LinearStamp::Vccs {
                component: instance.component.clone(),
                pos: pins[0].clone(),
                neg: pins[1].clone(),
                ctrl_pos: pins[2].clone(),
                ctrl_neg: pins[3].clone(),
                transconductance: transconductance.clone(),
            });
        }
        DeviceModelKind::Capacitor => report.notes.push(format!(
            "{}: capacitor is open-circuit in static/DC lowering",
            instance.component.as_str()
        )),
        DeviceModelKind::Inductor => report.stamps.push(LinearStamp::VoltageSource {
            component: instance.component.clone(),
            branch: branch(instance, "inductor-dc-short"),
            pos: pins[0].clone(),
            neg: pins[1].clone(),
            voltage: Real::zero(),
        }),
        DeviceModelKind::Diode
        | DeviceModelKind::Mosfet { .. }
        | DeviceModelKind::NonlinearPlaceholder
        | DeviceModelKind::Custom(_) => unreachable!(),
    }
}

fn parameter<'a>(
    instance: &'a CircuitInstance,
    model: &'a DeviceModel,
    name: &str,
) -> Option<&'a Real> {
    instance
        .parameters
        .iter()
        .find(|parameter| parameter.name == name)
        .or_else(|| {
            model
                .parameters
                .iter()
                .find(|parameter| parameter.name == name)
        })
        .map(|parameter| &parameter.value)
}

fn net_terminal(circuit: &Circuit, net: &NetId) -> Option<NetId> {
    circuit
        .nets
        .iter()
        .find(|candidate| candidate.id == *net)
        .filter(|candidate| !candidate.is_ground)
        .map(|candidate| candidate.id.clone())
}

fn missing_parameter(
    report: &mut LinearDeviceLoweringReport,
    instance: &CircuitInstance,
    name: &str,
) {
    report.issues.push(DeviceLoweringIssue::MissingParameter {
        component: instance.component.clone(),
        parameter: name.to_owned(),
    });
}

fn branch(instance: &CircuitInstance, suffix: &str) -> BranchId {
    BranchId::new(format!("{}:{suffix}", instance.id.as_str()))
        .expect("branch generated from nonempty instance id is nonempty")
}
