//! Nonlinear and event-device report surfaces.
//!
//! Nonlinear circuit devices need numeric Newton proposals in practical
//! simulators, but their model domains, monotonicity/slope facts, parameters,
//! and event decisions should be explicit before a solver is trusted. Diode
//! Newton linearization is therefore an explicit lossy proposal boundary:
//! every proposed linearization is converted to exact coefficients, solved to
//! an exact candidate, and replayed
//! through the retained Shockley law with exact authored tolerances. Retained
//! three-terminal MOSFETs use exact square-law derivatives and true-region
//! replay, so neither device family is accepted from a linear proposal alone.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter};

use hyperreal::{Real, RealSign};

use crate::{
    BranchId, Circuit, CircuitParameter, ComponentId, DeviceLoweringIssue, DeviceModelKind,
    LinearMnaSystem, LinearSolveReport, LinearStamp, MnaUnknown, NetId,
};

/// Nonlinear device family.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum NonlinearDeviceKind {
    /// Diode model.
    Diode,
    /// Executable Shichman-Hodges MOSFET.
    Mosfet,
    /// Piecewise-linear source or device.
    PiecewiseLinear,
    /// Ideal or controlled switch.
    Switch,
    /// Protection device such as TVS, fuse, or clamp placeholder.
    Protection,
}

/// Event policy for switches and protection devices.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum EventPolicy {
    /// Event state is fixed by caller policy.
    Fixed,
    /// Event candidate must be replayed through exact residuals.
    ExactReplayRequired,
    /// Event comes from a lossy adapter proposal.
    LossyAdapterProposal,
}

/// Switch state exposed to the solver policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SwitchState {
    /// Switch is open.
    Open,
    /// Switch is closed.
    Closed,
    /// Switch state was proposed by an adapter and still needs replay.
    Proposed,
}

/// Exact piecewise-linear segment.
#[derive(Clone, Debug, PartialEq)]
pub struct PiecewiseLinearSegment {
    /// Lower domain bound.
    pub lower: Real,
    /// Upper domain bound.
    pub upper: Real,
    /// Exact slope.
    pub slope: Real,
    /// Exact intercept.
    pub intercept: Real,
}

/// Nonlinear device report before Newton or event lowering.
#[derive(Clone, Debug, PartialEq)]
pub struct NonlinearDeviceReport {
    /// Component id.
    pub component: ComponentId,
    /// Device family.
    pub kind: NonlinearDeviceKind,
    /// Exact parameter provenance.
    pub parameters: Vec<CircuitParameter>,
    /// Human-readable domain assumptions.
    pub domains: Vec<String>,
    /// Human-readable monotonicity or slope facts.
    pub slope_facts: Vec<String>,
    /// Optional event policy.
    pub event_policy: Option<EventPolicy>,
    /// Optional switch state.
    pub switch_state: Option<SwitchState>,
    /// Piecewise-linear segments when available.
    pub segments: Vec<PiecewiseLinearSegment>,
}

impl NonlinearDeviceReport {
    /// Creates a diode model-domain report with exact parameter provenance.
    pub fn diode(component: ComponentId, parameters: Vec<CircuitParameter>) -> Self {
        Self {
            component,
            kind: NonlinearDeviceKind::Diode,
            parameters,
            domains: vec![
                "diode exponential model; Newton proposal requires residual replay".into(),
            ],
            slope_facts: vec![
                "forward branch monotone under positive saturation-current assumptions".into(),
            ],
            event_policy: None,
            switch_state: None,
            segments: Vec::new(),
        }
    }

    /// Creates an executable square-law MOSFET domain report.
    pub fn mosfet(component: ComponentId, parameters: Vec<CircuitParameter>) -> Self {
        Self {
            component,
            kind: NonlinearDeviceKind::Mosfet,
            parameters,
            domains: vec![
                "three-terminal body-tied-source square-law cutoff/triode/saturation model".into(),
            ],
            slope_facts: vec![
                "exact gm/gds derivatives are replayed against the selected operating region"
                    .into(),
            ],
            event_policy: None,
            switch_state: None,
            segments: Vec::new(),
        }
    }

    /// Creates a piecewise-linear report from exact segments.
    pub fn piecewise_linear(component: ComponentId, segments: Vec<PiecewiseLinearSegment>) -> Self {
        let slope_facts = segments
            .iter()
            .enumerate()
            .map(|(index, _)| format!("segment {index} has exact slope and intercept"))
            .collect();
        Self {
            component,
            kind: NonlinearDeviceKind::PiecewiseLinear,
            parameters: Vec::new(),
            domains: vec!["piecewise-linear domains are exact interval endpoints".into()],
            slope_facts,
            event_policy: None,
            switch_state: None,
            segments,
        }
    }

    /// Creates a switch/event report with visible policy.
    pub fn switch(component: ComponentId, event_policy: EventPolicy, state: SwitchState) -> Self {
        Self {
            component,
            kind: NonlinearDeviceKind::Switch,
            parameters: Vec::new(),
            domains: vec!["switch event state is policy-visible".into()],
            slope_facts: vec!["open/closed topology change is not hidden in tolerance".into()],
            event_policy: Some(event_policy),
            switch_state: Some(state),
            segments: Vec::new(),
        }
    }
}

/// Executable exact two-terminal piecewise-linear current law.
#[derive(Clone, Debug, PartialEq)]
pub struct PiecewiseLinearDevice {
    /// Stable component identity.
    pub component: ComponentId,
    /// Positive voltage/current terminal; `None` means ground.
    pub pos: Option<NetId>,
    /// Negative voltage/current terminal; `None` means ground.
    pub neg: Option<NetId>,
    /// Exact current-law regions, where `i = slope * v + intercept`.
    pub segments: Vec<PiecewiseLinearSegment>,
}

/// Certified active-region solution of a piecewise-linear circuit.
#[derive(Clone, Debug, PartialEq)]
pub struct PiecewiseLinearSolveReport {
    /// Exact linear solution for the selected active regions.
    pub solution: LinearSolveReport,
    /// Selected segment index for each device in input order.
    pub active_segments: Vec<usize>,
    /// Region combinations attempted before the accepted solution.
    pub candidates_tried: usize,
    /// Additional exact region combinations that represent the same boundary state.
    pub alternative_regions: usize,
}

/// Failure to enumerate and certify piecewise-linear active regions.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PiecewiseLinearSolveError {
    /// A device has no segment or has an unordered/indeterminate interval.
    InvalidSegments(ComponentId),
    /// The requested enumeration bound is zero or below the region product.
    CombinationLimit { required: usize, maximum: usize },
    /// No solvable region produced voltages inside every selected exact interval.
    NoConsistentRegion,
}

impl Display for PiecewiseLinearSolveError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidSegments(component) => write!(
                formatter,
                "piecewise-linear component {} has invalid segments",
                component.as_str()
            ),
            Self::CombinationLimit { required, maximum } => write!(
                formatter,
                "piecewise-linear solve needs {required} regions, above limit {maximum}"
            ),
            Self::NoConsistentRegion => {
                formatter.write_str("no consistent piecewise-linear region was found")
            }
        }
    }
}

impl std::error::Error for PiecewiseLinearSolveError {}

/// Retained executable two-terminal Shockley diode law.
#[derive(Clone, Debug, PartialEq)]
pub struct ShockleyDiode {
    /// Stable component identity.
    pub component: ComponentId,
    /// Anode terminal; `None` means ground.
    pub anode: Option<NetId>,
    /// Cathode terminal; `None` means ground.
    pub cathode: Option<NetId>,
    /// Strictly positive reverse saturation current.
    pub saturation_current: Real,
    /// Strictly positive effective thermal voltage, including emission factor.
    pub thermal_voltage: Real,
}

impl ShockleyDiode {
    /// Imports the incremental Shockley conductance and affine intercept at one
    /// exact retained voltage.
    ///
    /// The exponential proposal crosses the explicit primitive-float boundary,
    /// while the returned dyadic coefficients are exact `Real` values suitable
    /// for exact DC Newton or complex small-signal MNA assembly.
    pub fn linearize_at(
        &self,
        voltage: &Real,
    ) -> Result<DiodeLinearizationEvidence, DiodeNewtonSolveError> {
        if self.saturation_current.structural_facts().sign != Some(RealSign::Positive)
            || self.thermal_voltage.structural_facts().sign != Some(RealSign::Positive)
        {
            return Err(DiodeNewtonSolveError::InvalidDiode(self.component.clone()));
        }
        let (conductance, current_intercept) = lossy_diode_linearization(self, voltage)?;
        Ok(DiodeLinearizationEvidence {
            component: self.component.clone(),
            voltage: voltage.clone(),
            conductance,
            current_intercept,
        })
    }
}

/// Bounded Newton proposal and exact replay policy.
#[derive(Clone, Debug, PartialEq)]
pub struct DiodeNewtonPolicy {
    /// Hard iteration bound.
    pub maximum_iterations: usize,
    /// Strictly positive exact node-voltage/branch-constraint tolerance.
    pub voltage_tolerance: Real,
    /// Strictly positive exact KCL residual tolerance.
    pub current_tolerance: Real,
    /// Exact update factor in `(0, 1]`.
    pub damping: Real,
}

impl Default for DiodeNewtonPolicy {
    fn default() -> Self {
        let billion = Real::from(1_000_000_000);
        Self {
            maximum_iterations: 64,
            voltage_tolerance: (Real::one() / billion.clone())
                .expect("default voltage tolerance divisor is nonzero"),
            current_tolerance: (Real::one() / billion)
                .expect("default current tolerance divisor is nonzero"),
            damping: Real::one(),
        }
    }
}

/// True-law nonlinear residual replay for one exact candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct DiodeResidualReplayReport {
    /// KCL residual by non-ground net.
    pub kcl_residuals: BTreeMap<NetId, Real>,
    /// Ideal-source constraint residual by branch identity.
    pub branch_residuals: BTreeMap<BranchId, Real>,
    /// Largest absolute KCL residual.
    pub maximum_kcl_residual: Real,
    /// Largest absolute voltage-constraint residual.
    pub maximum_branch_residual: Real,
    /// Every residual satisfies its authored exact tolerance.
    pub accepted: bool,
    /// Every residual was certified exactly zero.
    pub exact_zero: bool,
}

/// Exact imported coefficients from one lossy diode linearization.
#[derive(Clone, Debug, PartialEq)]
pub struct DiodeLinearizationEvidence {
    /// Stable retained diode identity.
    pub component: ComponentId,
    /// Exact voltage whose lossy view seeded the proposal.
    pub voltage: Real,
    /// Exact imported dyadic small-signal conductance.
    pub conductance: Real,
    /// Exact imported dyadic current-source intercept.
    pub current_intercept: Real,
}

/// One lossy-linearization/exact-replay Newton attempt.
#[derive(Clone, Debug, PartialEq)]
pub struct DiodeNewtonIteration {
    /// Zero-based attempt index.
    pub iteration: usize,
    /// Exact voltage used to linearize each diode.
    pub linearization_voltages: BTreeMap<ComponentId, Real>,
    /// Reproducible exact coefficients imported from each lossy linearization.
    pub linearizations: Vec<DiodeLinearizationEvidence>,
    /// Damped exact candidate in stable MNA ordering.
    pub candidate: Vec<Real>,
    /// Largest exact node-voltage update.
    pub maximum_voltage_update: Real,
    /// True-law residual replay at `candidate`.
    pub replay: DiodeResidualReplayReport,
    /// Exact replay of the undamped linearized MNA solve.
    pub linear_proposal_replay_accepted: bool,
}

/// Terminal status of a bounded diode Newton solve.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DiodeNewtonStatus {
    /// Update and true-law residual criteria were exactly satisfied.
    Converged,
    /// The hard iteration bound ended before both criteria were satisfied.
    IterationLimit,
}

/// Bounded nonlinear solve result with every proposal and replay retained.
#[derive(Clone, Debug, PartialEq)]
pub struct DiodeNewtonSolveReport {
    /// Terminal status.
    pub status: DiodeNewtonStatus,
    /// Stable MNA ordering for every recorded candidate.
    pub unknowns: Vec<MnaUnknown>,
    /// Final exact candidate.
    pub candidate: Vec<Real>,
    /// Every bounded Newton attempt.
    pub iterations: Vec<DiodeNewtonIteration>,
    /// True-law replay at the final candidate.
    pub replay: DiodeResidualReplayReport,
    /// This path used explicit primitive-float linearization proposals.
    pub used_lossy_linearization: bool,
}

impl DiodeNewtonSolveReport {
    /// Returns one solved non-ground node voltage.
    pub fn net_voltage(&self, net: &NetId) -> Option<&Real> {
        self.unknowns
            .iter()
            .position(|unknown| matches!(unknown, MnaUnknown::NetVoltage(id) if id == net))
            .map(|index| &self.candidate[index])
    }
}

/// Structural, proposal, or exact-replay failure during diode Newton solving.
#[derive(Clone, Debug, PartialEq)]
pub enum DiodeNewtonSolveError {
    /// Retained circuit structure is invalid.
    InvalidCircuit,
    /// No retained diode was available to solve.
    NoDiodes,
    /// A diode lacks terminals or strictly positive model parameters.
    InvalidDiode(ComponentId),
    /// Linear devices could not be lowered beside the diode set.
    LinearLowering(Vec<DeviceLoweringIssue>),
    /// Iteration count, tolerances, or damping are invalid.
    InvalidPolicy,
    /// An initial voltage names a net outside the solved non-ground set.
    UnknownInitialNet(NetId),
    /// A lossy proposal overflowed or could not become an exact dyadic value.
    ProposalArithmetic,
    /// Exact Shockley-law construction failed.
    ExactLawArithmetic,
    /// Exact ordering against a convergence tolerance remained undecidable.
    IndeterminateComparison,
    /// Exact MNA assembly, solve, or residual replay failed.
    Mna(crate::CircuitError),
}

impl Display for DiodeNewtonSolveError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidCircuit => formatter.write_str("invalid circuit for diode Newton solve"),
            Self::NoDiodes => formatter.write_str("circuit contains no executable diode"),
            Self::InvalidDiode(component) => write!(
                formatter,
                "diode {} has invalid terminals or parameters",
                component.as_str()
            ),
            Self::LinearLowering(issues) => write!(
                formatter,
                "{} non-diode device(s) could not be lowered",
                issues.len()
            ),
            Self::InvalidPolicy => formatter.write_str("invalid diode Newton policy"),
            Self::UnknownInitialNet(net) => {
                write!(formatter, "unknown initial-voltage net {}", net.as_str())
            }
            Self::ProposalArithmetic => {
                formatter.write_str("diode Newton proposal arithmetic failed")
            }
            Self::ExactLawArithmetic => {
                formatter.write_str("exact Shockley-law construction failed")
            }
            Self::IndeterminateComparison => {
                formatter.write_str("nonlinear convergence comparison is indeterminate")
            }
            Self::Mna(error) => write!(formatter, "nonlinear MNA failed: {error}"),
        }
    }
}

impl std::error::Error for DiodeNewtonSolveError {}

/// Solves retained Shockley diodes using lossy Newton proposals and exact replay.
///
/// Primitive floats decide only the next linearization proposal. Each proposal
/// is imported as exact `Real` coefficients and the linearized MNA problem is
/// solved exactly; accepted convergence is decided by evaluating the true
/// exponential law and exact MNA residuals at the resulting candidate.
pub fn solve_shockley_diode_newton(
    nets: Vec<NetId>,
    linear_stamps: &[LinearStamp],
    diodes: &[ShockleyDiode],
    policy: &DiodeNewtonPolicy,
    initial_voltages: &BTreeMap<NetId, Real>,
) -> Result<DiodeNewtonSolveReport, DiodeNewtonSolveError> {
    validate_diode_policy(policy)?;
    if diodes.is_empty() {
        return Err(DiodeNewtonSolveError::NoDiodes);
    }
    let net_set = nets.iter().cloned().collect::<BTreeSet<_>>();
    if let Some(net) = initial_voltages.keys().find(|net| !net_set.contains(*net)) {
        return Err(DiodeNewtonSolveError::UnknownInitialNet(net.clone()));
    }
    for diode in diodes {
        if diode.saturation_current.structural_facts().sign != Some(RealSign::Positive)
            || diode.thermal_voltage.structural_facts().sign != Some(RealSign::Positive)
            || diode
                .anode
                .as_ref()
                .is_some_and(|net| !net_set.contains(net))
            || diode
                .cathode
                .as_ref()
                .is_some_and(|net| !net_set.contains(net))
        {
            return Err(DiodeNewtonSolveError::InvalidDiode(diode.component.clone()));
        }
    }

    let mut previous = nets
        .iter()
        .map(|net| {
            (
                MnaUnknown::NetVoltage(net.clone()),
                initial_voltages
                    .get(net)
                    .cloned()
                    .unwrap_or_else(Real::zero),
            )
        })
        .collect::<BTreeMap<_, _>>();
    let mut iterations = Vec::new();
    let mut final_unknowns = Vec::new();
    let mut final_candidate = Vec::new();
    for iteration in 0..policy.maximum_iterations {
        let mut stamps = linear_stamps.to_vec();
        let mut linearization_voltages = BTreeMap::new();
        let mut linearizations = Vec::new();
        for diode in diodes {
            let voltage = diode_voltage_from_unknowns(diode, &previous);
            linearization_voltages.insert(diode.component.clone(), voltage.clone());
            let linearization = diode.linearize_at(&voltage)?;
            stamps.push(LinearStamp::Conductance {
                component: diode.component.clone(),
                part: None,
                pos: diode.anode.clone(),
                neg: diode.cathode.clone(),
                conductance: linearization.conductance.clone(),
            });
            stamps.push(LinearStamp::CurrentSource {
                component: diode.component.clone(),
                pos: diode.anode.clone(),
                neg: diode.cathode.clone(),
                current: linearization.current_intercept.clone(),
            });
            linearizations.push(linearization);
        }
        let system = LinearMnaSystem::from_stamps(nets.clone(), &stamps)
            .map_err(DiodeNewtonSolveError::Mna)?;
        let proposal = system.solve_exact().map_err(DiodeNewtonSolveError::Mna)?;
        let candidate = system
            .unknowns
            .iter()
            .zip(&proposal.candidate)
            .map(|(unknown, proposed)| {
                let old = previous.get(unknown).cloned().unwrap_or_else(Real::zero);
                old.clone() + policy.damping.clone() * (proposed.clone() - old)
            })
            .collect::<Vec<_>>();
        let mut maximum_voltage_update = Real::zero();
        for (unknown, candidate) in system.unknowns.iter().zip(&candidate) {
            if matches!(unknown, MnaUnknown::NetVoltage(_)) {
                let update = exact_abs(
                    &(candidate.clone()
                        - previous.get(unknown).cloned().unwrap_or_else(Real::zero)),
                )?;
                maximum_voltage_update = exact_max_result(maximum_voltage_update, update)?;
            }
        }
        let replay = replay_shockley_candidate(
            nets.clone(),
            linear_stamps,
            diodes,
            &system.unknowns,
            &candidate,
            policy,
        )?;
        let update_accepted = exact_le(&maximum_voltage_update, &policy.voltage_tolerance)?;
        iterations.push(DiodeNewtonIteration {
            iteration,
            linearization_voltages,
            linearizations,
            candidate: candidate.clone(),
            maximum_voltage_update,
            replay: replay.clone(),
            linear_proposal_replay_accepted: proposal.replay.accepted,
        });
        previous = system
            .unknowns
            .iter()
            .cloned()
            .zip(candidate.iter().cloned())
            .collect();
        final_unknowns.clone_from(&system.unknowns);
        final_candidate.clone_from(&candidate);
        if update_accepted && replay.accepted {
            return Ok(DiodeNewtonSolveReport {
                status: DiodeNewtonStatus::Converged,
                unknowns: system.unknowns,
                candidate,
                iterations,
                replay,
                used_lossy_linearization: true,
            });
        }
    }
    let final_iteration = iterations
        .last()
        .expect("validated positive iteration bound produces an attempt");
    Ok(DiodeNewtonSolveReport {
        status: DiodeNewtonStatus::IterationLimit,
        unknowns: final_unknowns,
        candidate: final_candidate,
        iterations: iterations.clone(),
        replay: final_iteration.replay.clone(),
        used_lossy_linearization: true,
    })
}

impl Circuit {
    /// Extracts executable diode laws from retained model/instance parameters.
    pub fn shockley_diodes(&self) -> Result<Vec<ShockleyDiode>, DiodeNewtonSolveError> {
        if !self.validate().is_valid() {
            return Err(DiodeNewtonSolveError::InvalidCircuit);
        }
        self.instances
            .iter()
            .filter_map(|instance| {
                let model = self
                    .device_models
                    .iter()
                    .find(|model| model.id == instance.model)
                    .expect("validated instance model must exist");
                (model.kind == DeviceModelKind::Diode).then_some((instance, model))
            })
            .map(|(instance, model)| {
                let terminals = model
                    .pins
                    .iter()
                    .filter_map(|pin| {
                        instance
                            .pins
                            .iter()
                            .find(|binding| binding.pin == pin.pin)
                            .map(|binding| {
                                self.nets
                                    .iter()
                                    .find(|net| net.id == binding.net)
                                    .filter(|net| !net.is_ground)
                                    .map(|net| net.id.clone())
                            })
                    })
                    .collect::<Vec<_>>();
                let saturation_current = diode_parameter(instance, model, "saturation_current");
                let thermal_voltage = diode_parameter(instance, model, "thermal_voltage");
                let (Some(saturation_current), Some(thermal_voltage)) =
                    (saturation_current, thermal_voltage)
                else {
                    return Err(DiodeNewtonSolveError::InvalidDiode(
                        instance.component.clone(),
                    ));
                };
                if terminals.len() < 2 {
                    return Err(DiodeNewtonSolveError::InvalidDiode(
                        instance.component.clone(),
                    ));
                }
                Ok(ShockleyDiode {
                    component: instance.component.clone(),
                    anode: terminals[0].clone(),
                    cathode: terminals[1].clone(),
                    saturation_current,
                    thermal_voltage,
                })
            })
            .collect()
    }

    /// Executes a retained mixed linear/diode DC solve.
    pub fn solve_diode_dc(
        &self,
        policy: &DiodeNewtonPolicy,
        initial_voltages: &BTreeMap<NetId, Real>,
    ) -> Result<DiodeNewtonSolveReport, DiodeNewtonSolveError> {
        let diodes = self.shockley_diodes()?;
        if diodes.is_empty() {
            return Err(DiodeNewtonSolveError::NoDiodes);
        }
        let diode_components = diodes
            .iter()
            .map(|diode| diode.component.clone())
            .collect::<BTreeSet<_>>();
        let mut lowering = self.lower_linear_devices();
        lowering.issues.retain(|issue| {
            !matches!(
                issue,
                DeviceLoweringIssue::UnsupportedModel(component)
                    if diode_components.contains(component)
            )
        });
        if !lowering.is_complete() {
            return Err(DiodeNewtonSolveError::LinearLowering(lowering.issues));
        }
        solve_shockley_diode_newton(
            self.nets
                .iter()
                .filter(|net| !net.is_ground)
                .map(|net| net.id.clone())
                .collect(),
            &lowering.stamps,
            &diodes,
            policy,
            initial_voltages,
        )
    }
}

fn validate_diode_policy(policy: &DiodeNewtonPolicy) -> Result<(), DiodeNewtonSolveError> {
    if policy.maximum_iterations == 0
        || policy.voltage_tolerance.structural_facts().sign != Some(RealSign::Positive)
        || policy.current_tolerance.structural_facts().sign != Some(RealSign::Positive)
        || policy.damping.structural_facts().sign != Some(RealSign::Positive)
        || !matches!(
            policy.damping.partial_cmp(&Real::one()),
            Some(Ordering::Less | Ordering::Equal)
        )
    {
        return Err(DiodeNewtonSolveError::InvalidPolicy);
    }
    Ok(())
}

fn lossy_diode_linearization(
    diode: &ShockleyDiode,
    voltage: &Real,
) -> Result<(Real, Real), DiodeNewtonSolveError> {
    let voltage = voltage
        .to_f64_lossy()
        .ok_or(DiodeNewtonSolveError::ProposalArithmetic)?;
    let saturation = diode
        .saturation_current
        .to_f64_lossy()
        .ok_or(DiodeNewtonSolveError::ProposalArithmetic)?;
    let thermal = diode
        .thermal_voltage
        .to_f64_lossy()
        .ok_or(DiodeNewtonSolveError::ProposalArithmetic)?;
    let exponential = (voltage / thermal).exp();
    let conductance = saturation / thermal * exponential;
    let current = saturation * (exponential - 1.0);
    let intercept = current - conductance * voltage;
    if !conductance.is_finite() || !intercept.is_finite() {
        return Err(DiodeNewtonSolveError::ProposalArithmetic);
    }
    Ok((
        Real::try_from(conductance).map_err(|_| DiodeNewtonSolveError::ProposalArithmetic)?,
        Real::try_from(intercept).map_err(|_| DiodeNewtonSolveError::ProposalArithmetic)?,
    ))
}

fn replay_shockley_candidate(
    nets: Vec<NetId>,
    linear_stamps: &[LinearStamp],
    diodes: &[ShockleyDiode],
    unknowns: &[MnaUnknown],
    candidate: &[Real],
    policy: &DiodeNewtonPolicy,
) -> Result<DiodeResidualReplayReport, DiodeNewtonSolveError> {
    let values = unknowns
        .iter()
        .cloned()
        .zip(candidate.iter().cloned())
        .collect::<BTreeMap<_, _>>();
    let mut stamps = linear_stamps.to_vec();
    for diode in diodes {
        stamps.push(LinearStamp::CurrentSource {
            component: diode.component.clone(),
            pos: diode.anode.clone(),
            neg: diode.cathode.clone(),
            current: exact_diode_current(diode, &diode_voltage_from_unknowns(diode, &values))?,
        });
    }
    let system = LinearMnaSystem::from_stamps(nets, &stamps).map_err(DiodeNewtonSolveError::Mna)?;
    if system.unknowns != unknowns {
        return Err(DiodeNewtonSolveError::Mna(
            crate::CircuitError::CandidateLengthMismatch,
        ));
    }
    let raw = system
        .replay_candidate(candidate)
        .map_err(DiodeNewtonSolveError::Mna)?;
    let mut kcl_residuals = BTreeMap::new();
    let mut branch_residuals = BTreeMap::new();
    let mut maximum_kcl_residual = Real::zero();
    let mut maximum_branch_residual = Real::zero();
    for (unknown, residual) in unknowns.iter().zip(&raw.residuals) {
        let absolute = exact_abs(residual)?;
        match unknown {
            MnaUnknown::NetVoltage(net) => {
                maximum_kcl_residual = exact_max_result(maximum_kcl_residual, absolute.clone())?;
                kcl_residuals.insert(net.clone(), residual.clone());
            }
            MnaUnknown::BranchCurrent(branch) => {
                maximum_branch_residual =
                    exact_max_result(maximum_branch_residual, absolute.clone())?;
                branch_residuals.insert(branch.clone(), residual.clone());
            }
        }
    }
    let accepted = exact_le(&maximum_kcl_residual, &policy.current_tolerance)?
        && exact_le(&maximum_branch_residual, &policy.voltage_tolerance)?;
    Ok(DiodeResidualReplayReport {
        kcl_residuals,
        branch_residuals,
        maximum_kcl_residual,
        maximum_branch_residual,
        accepted,
        exact_zero: raw.accepted,
    })
}

fn exact_diode_current(
    diode: &ShockleyDiode,
    voltage: &Real,
) -> Result<Real, DiodeNewtonSolveError> {
    let normalized = (voltage.clone() / diode.thermal_voltage.clone())
        .map_err(|_| DiodeNewtonSolveError::ExactLawArithmetic)?;
    let exponential = normalized
        .exp()
        .map_err(|_| DiodeNewtonSolveError::ExactLawArithmetic)?;
    Ok(diode.saturation_current.clone() * (exponential - Real::one()))
}

fn diode_voltage_from_unknowns(diode: &ShockleyDiode, values: &BTreeMap<MnaUnknown, Real>) -> Real {
    terminal_value(&diode.anode, values) - terminal_value(&diode.cathode, values)
}

fn terminal_value(terminal: &Option<NetId>, values: &BTreeMap<MnaUnknown, Real>) -> Real {
    terminal
        .as_ref()
        .and_then(|net| values.get(&MnaUnknown::NetVoltage(net.clone())))
        .cloned()
        .unwrap_or_else(Real::zero)
}

fn diode_parameter(
    instance: &crate::CircuitInstance,
    model: &crate::DeviceModel,
    name: &str,
) -> Option<Real> {
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
        .map(|parameter| parameter.value.clone())
}

fn exact_max_result(current: Real, candidate: Real) -> Result<Real, DiodeNewtonSolveError> {
    match current.partial_cmp(&candidate) {
        Some(Ordering::Less) => Ok(candidate),
        Some(Ordering::Equal | Ordering::Greater) => Ok(current),
        None => Err(DiodeNewtonSolveError::IndeterminateComparison),
    }
}

fn exact_le(left: &Real, right: &Real) -> Result<bool, DiodeNewtonSolveError> {
    match left.partial_cmp(right) {
        Some(Ordering::Less | Ordering::Equal) => Ok(true),
        Some(Ordering::Greater) => Ok(false),
        None => Err(DiodeNewtonSolveError::IndeterminateComparison),
    }
}

fn exact_abs(value: &Real) -> Result<Real, DiodeNewtonSolveError> {
    match value.partial_cmp(&Real::zero()) {
        Some(Ordering::Less) => Ok(-value.clone()),
        Some(Ordering::Equal | Ordering::Greater) => Ok(value.clone()),
        None => Err(DiodeNewtonSolveError::IndeterminateComparison),
    }
}

/// Enumerates exact active regions and replays every accepted linear solution.
///
/// This is a deterministic small-circuit path rather than a scalable Newton
/// implementation. Singular or inconsistent candidate regions are skipped;
/// a result is returned only when every device voltage is certified inside its
/// selected exact interval.
pub fn solve_piecewise_linear(
    nets: Vec<NetId>,
    linear_stamps: &[LinearStamp],
    devices: &[PiecewiseLinearDevice],
    maximum_combinations: usize,
) -> Result<PiecewiseLinearSolveReport, PiecewiseLinearSolveError> {
    let mut combinations = 1_usize;
    for device in devices {
        if device.segments.is_empty()
            || device.segments.iter().any(|segment| {
                !matches!(
                    segment.lower.partial_cmp(&segment.upper),
                    Some(Ordering::Less | Ordering::Equal)
                )
            })
        {
            return Err(PiecewiseLinearSolveError::InvalidSegments(
                device.component.clone(),
            ));
        }
        combinations = combinations.saturating_mul(device.segments.len());
    }
    if maximum_combinations == 0 || combinations > maximum_combinations {
        return Err(PiecewiseLinearSolveError::CombinationLimit {
            required: combinations,
            maximum: maximum_combinations,
        });
    }

    let mut accepted = None;
    let mut alternative_regions = 0;
    let mut candidates_tried = 0;
    for ordinal in 0..combinations {
        candidates_tried += 1;
        let active_segments = decode_region(ordinal, devices);
        let mut stamps = linear_stamps.to_vec();
        for (device, active) in devices.iter().zip(&active_segments) {
            let segment = &device.segments[*active];
            stamps.push(LinearStamp::Conductance {
                component: device.component.clone(),
                part: None,
                pos: device.pos.clone(),
                neg: device.neg.clone(),
                conductance: segment.slope.clone(),
            });
            stamps.push(LinearStamp::CurrentSource {
                component: device.component.clone(),
                pos: device.pos.clone(),
                neg: device.neg.clone(),
                current: segment.intercept.clone(),
            });
        }
        let Ok(system) = LinearMnaSystem::from_stamps(nets.clone(), &stamps) else {
            continue;
        };
        let Ok(solution) = system.solve_exact() else {
            continue;
        };
        let consistent = devices
            .iter()
            .zip(&active_segments)
            .all(|(device, active)| {
                let voltage = terminal_voltage(&system, &solution, &device.pos)
                    - terminal_voltage(&system, &solution, &device.neg);
                interval_contains(&device.segments[*active], &voltage)
            });
        if !consistent {
            continue;
        }
        if accepted.is_none() {
            accepted = Some((solution, active_segments, candidates_tried));
        } else {
            alternative_regions += 1;
        }
    }
    let Some((solution, active_segments, candidates_tried)) = accepted else {
        return Err(PiecewiseLinearSolveError::NoConsistentRegion);
    };
    Ok(PiecewiseLinearSolveReport {
        solution,
        active_segments,
        candidates_tried,
        alternative_regions,
    })
}

fn decode_region(mut ordinal: usize, devices: &[PiecewiseLinearDevice]) -> Vec<usize> {
    let mut active = vec![0; devices.len()];
    for (slot, device) in devices.iter().enumerate().rev() {
        active[slot] = ordinal % device.segments.len();
        ordinal /= device.segments.len();
    }
    active
}

fn interval_contains(segment: &PiecewiseLinearSegment, value: &Real) -> bool {
    matches!(
        segment.lower.partial_cmp(value),
        Some(Ordering::Less | Ordering::Equal)
    ) && matches!(
        value.partial_cmp(&segment.upper),
        Some(Ordering::Less | Ordering::Equal)
    )
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
        .expect("assembled piecewise-linear terminal must have a voltage unknown")
}
