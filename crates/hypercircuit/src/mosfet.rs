//! Executable retained three-terminal MOSFET laws.
//!
//! The model is the level-one Shichman-Hodges square law with the body tied to
//! source. Newton uses exact active-region derivatives. Every candidate is
//! replayed through the exact cutoff/triode/saturation law and ordinary MNA
//! residuals before convergence can be accepted.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter};

use hyperreal::{Real, RealSign};

use crate::{
    BranchId, Circuit, CircuitInstance, ComponentId, DeviceLoweringIssue, DeviceModel,
    DeviceModelKind, LinearMnaSystem, LinearStamp, MnaUnknown, MosfetPolarity, NetId, PinRef,
};

/// Exact square-law operating region selected at one candidate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MosfetRegion {
    /// Normalized gate voltage is at or below threshold.
    Cutoff,
    /// Channel is formed and normalized drain voltage is below overdrive.
    Triode,
    /// Channel is pinched off at the drain.
    Saturation,
}

/// Retained executable three-terminal, body-tied-source MOSFET law.
#[derive(Clone, Debug, PartialEq)]
pub struct SquareLawMosfet {
    /// Stable component identity.
    pub component: ComponentId,
    /// N-channel or P-channel orientation.
    pub polarity: MosfetPolarity,
    /// Drain terminal; `None` means ground.
    pub drain: Option<NetId>,
    /// Gate terminal; `None` means ground.
    pub gate: Option<NetId>,
    /// Source/body terminal; `None` means ground.
    pub source: Option<NetId>,
    /// Strictly positive threshold-voltage magnitude.
    pub threshold_voltage: Real,
    /// Strictly positive square-law beta coefficient.
    pub transconductance_parameter: Real,
    /// Nonnegative channel-length-modulation coefficient.
    pub channel_length_modulation: Real,
}

/// Exact true-law values and derivatives at one MOSFET bias point.
#[derive(Clone, Debug, PartialEq)]
pub struct MosfetOperatingPoint {
    /// Stable component identity.
    pub component: ComponentId,
    /// Exactly selected operating region.
    pub region: MosfetRegion,
    /// Physical `Vg - Vs`.
    pub gate_source_voltage: Real,
    /// Physical `Vd - Vs`.
    pub drain_source_voltage: Real,
    /// Polarity-normalized gate overdrive `s*(Vg-Vs) - Vth`.
    pub normalized_gate_overdrive: Real,
    /// Polarity-normalized drain voltage `s*(Vd-Vs)`.
    pub normalized_drain_source_voltage: Real,
    /// Physical drain-to-source current.
    pub drain_current: Real,
    /// Exact derivative with respect to physical `Vg - Vs`.
    pub transconductance: Real,
    /// Exact derivative with respect to physical `Vd - Vs`.
    pub output_conductance: Real,
}

/// Exact coefficients used for one affine MOSFET Newton stamp.
#[derive(Clone, Debug, PartialEq)]
pub struct MosfetLinearizationEvidence {
    /// True-law operating point that produced the derivatives.
    pub operating_point: MosfetOperatingPoint,
    /// Exact affine current-source intercept.
    pub current_intercept: Real,
}

/// Bounded exact MOSFET Newton policy.
#[derive(Clone, Debug, PartialEq)]
pub struct MosfetNewtonPolicy {
    /// Hard iteration bound.
    pub maximum_iterations: usize,
    /// Strictly positive exact node-voltage/branch-constraint tolerance.
    pub voltage_tolerance: Real,
    /// Strictly positive exact KCL residual tolerance.
    pub current_tolerance: Real,
    /// Exact update factor in `(0, 1]`.
    pub damping: Real,
}

impl Default for MosfetNewtonPolicy {
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

/// True-law MOSFET residual replay for one exact candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct MosfetResidualReplayReport {
    /// Exact operating point of each retained MOSFET.
    pub operating_points: Vec<MosfetOperatingPoint>,
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

/// One exact-linearization/exact-replay MOSFET Newton attempt.
#[derive(Clone, Debug, PartialEq)]
pub struct MosfetNewtonIteration {
    /// Zero-based attempt index.
    pub iteration: usize,
    /// Exact affine coefficients for every MOSFET.
    pub linearizations: Vec<MosfetLinearizationEvidence>,
    /// Damped exact candidate in stable MNA ordering.
    pub candidate: Vec<Real>,
    /// Largest exact node-voltage update.
    pub maximum_voltage_update: Real,
    /// True-law residual replay at `candidate`.
    pub replay: MosfetResidualReplayReport,
    /// Exact replay of the undamped affine MNA solve.
    pub linear_proposal_replay_accepted: bool,
}

/// Terminal status of a bounded MOSFET Newton solve.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MosfetNewtonStatus {
    /// Update and true-law residual criteria were exactly satisfied.
    Converged,
    /// The hard iteration bound ended before both criteria were satisfied.
    IterationLimit,
}

/// Bounded MOSFET solve result with every affine proposal and replay retained.
#[derive(Clone, Debug, PartialEq)]
pub struct MosfetNewtonSolveReport {
    /// Terminal status.
    pub status: MosfetNewtonStatus,
    /// Stable MNA ordering for every recorded candidate.
    pub unknowns: Vec<MnaUnknown>,
    /// Final exact candidate.
    pub candidate: Vec<Real>,
    /// Every bounded Newton attempt.
    pub iterations: Vec<MosfetNewtonIteration>,
    /// True-law replay at the final candidate.
    pub replay: MosfetResidualReplayReport,
    /// MOSFET derivatives and true-law replay remained exact.
    pub used_lossy_linearization: bool,
}

impl MosfetNewtonSolveReport {
    /// Returns one solved non-ground node voltage.
    pub fn net_voltage(&self, net: &NetId) -> Option<&Real> {
        self.unknowns
            .iter()
            .position(|unknown| matches!(unknown, MnaUnknown::NetVoltage(id) if id == net))
            .map(|index| &self.candidate[index])
    }
}

/// Structural, region, or exact-replay failure during MOSFET Newton solving.
#[derive(Clone, Debug, PartialEq)]
pub enum MosfetNewtonSolveError {
    /// Retained circuit structure is invalid.
    InvalidCircuit,
    /// No retained MOSFET was available to solve.
    NoMosfets,
    /// A MOSFET lacks terminals or valid model parameters.
    InvalidMosfet(ComponentId),
    /// The candidate requires reverse-channel behavior outside this model.
    ReverseConduction(ComponentId),
    /// Linear devices could not be lowered beside the MOSFET set.
    LinearLowering(Vec<DeviceLoweringIssue>),
    /// Iteration count, tolerances, or damping are invalid.
    InvalidPolicy,
    /// An initial voltage names a net outside the solved non-ground set.
    UnknownInitialNet(NetId),
    /// Exact ordering against a region or convergence boundary was undecidable.
    IndeterminateComparison,
    /// Exact affine-law arithmetic failed.
    Arithmetic,
    /// Exact MNA assembly, solve, or residual replay failed.
    Mna(crate::CircuitError),
}

impl Display for MosfetNewtonSolveError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidCircuit => formatter.write_str("invalid circuit for MOSFET Newton solve"),
            Self::NoMosfets => formatter.write_str("circuit contains no executable MOSFET"),
            Self::InvalidMosfet(component) => write!(
                formatter,
                "MOSFET {} has invalid terminals or parameters",
                component.as_str()
            ),
            Self::ReverseConduction(component) => write!(
                formatter,
                "MOSFET {} requires unsupported reverse-channel conduction",
                component.as_str()
            ),
            Self::LinearLowering(issues) => write!(
                formatter,
                "{} non-MOSFET device(s) could not be lowered",
                issues.len()
            ),
            Self::InvalidPolicy => formatter.write_str("invalid MOSFET Newton policy"),
            Self::UnknownInitialNet(net) => {
                write!(formatter, "unknown initial-voltage net {}", net.as_str())
            }
            Self::IndeterminateComparison => {
                formatter.write_str("MOSFET region or convergence comparison is indeterminate")
            }
            Self::Arithmetic => formatter.write_str("exact MOSFET-law arithmetic failed"),
            Self::Mna(error) => write!(formatter, "MOSFET MNA failed: {error}"),
        }
    }
}

impl std::error::Error for MosfetNewtonSolveError {}

impl SquareLawMosfet {
    /// Evaluates the exact retained law at physical terminal voltages.
    pub fn operating_point(
        &self,
        gate_voltage: &Real,
        drain_voltage: &Real,
        source_voltage: &Real,
    ) -> Result<MosfetOperatingPoint, MosfetNewtonSolveError> {
        validate_mosfet(self, None)?;
        let gate_source_voltage = gate_voltage.clone() - source_voltage.clone();
        let drain_source_voltage = drain_voltage.clone() - source_voltage.clone();
        let sign = match self.polarity {
            MosfetPolarity::NChannel => Real::one(),
            MosfetPolarity::PChannel => -Real::one(),
        };
        let normalized_gate = sign.clone() * gate_source_voltage.clone();
        let normalized_drain = sign.clone() * drain_source_voltage.clone();
        let overdrive = normalized_gate - self.threshold_voltage.clone();
        if exact_le(&overdrive, &Real::zero())? {
            return Ok(MosfetOperatingPoint {
                component: self.component.clone(),
                region: MosfetRegion::Cutoff,
                gate_source_voltage,
                drain_source_voltage,
                normalized_gate_overdrive: overdrive,
                normalized_drain_source_voltage: normalized_drain,
                drain_current: Real::zero(),
                transconductance: Real::zero(),
                output_conductance: Real::zero(),
            });
        }
        if exact_lt(&normalized_drain, &Real::zero())? {
            return Err(MosfetNewtonSolveError::ReverseConduction(
                self.component.clone(),
            ));
        }

        let beta = self.transconductance_parameter.clone();
        let lambda = self.channel_length_modulation.clone();
        let modulation = Real::one() + lambda.clone() * normalized_drain.clone();
        let (region, normalized_current, transconductance, output_conductance) =
            if exact_lt(&normalized_drain, &overdrive)? {
                let half = (Real::one() / Real::from(2))
                    .map_err(|_| MosfetNewtonSolveError::Arithmetic)?;
                let base = beta.clone()
                    * (overdrive.clone() * normalized_drain.clone()
                        - half * normalized_drain.clone() * normalized_drain.clone());
                let current = base.clone() * modulation.clone();
                let gm = beta.clone() * normalized_drain.clone() * modulation.clone();
                let gds = beta * (overdrive.clone() - normalized_drain.clone()) * modulation
                    + base * lambda;
                (MosfetRegion::Triode, current, gm, gds)
            } else {
                let half = (Real::one() / Real::from(2))
                    .map_err(|_| MosfetNewtonSolveError::Arithmetic)?;
                let base = half * beta * overdrive.clone() * overdrive.clone();
                let current = base.clone() * modulation.clone();
                let gm = self.transconductance_parameter.clone() * overdrive.clone() * modulation;
                let gds = base * lambda;
                (MosfetRegion::Saturation, current, gm, gds)
            };
        Ok(MosfetOperatingPoint {
            component: self.component.clone(),
            region,
            gate_source_voltage,
            drain_source_voltage,
            normalized_gate_overdrive: overdrive,
            normalized_drain_source_voltage: normalized_drain,
            drain_current: sign * normalized_current,
            transconductance,
            output_conductance,
        })
    }
}

/// Solves retained square-law MOSFETs using exact Newton derivatives and replay.
pub fn solve_square_law_mosfet_newton(
    nets: Vec<NetId>,
    linear_stamps: &[LinearStamp],
    mosfets: &[SquareLawMosfet],
    policy: &MosfetNewtonPolicy,
    initial_voltages: &BTreeMap<NetId, Real>,
) -> Result<MosfetNewtonSolveReport, MosfetNewtonSolveError> {
    validate_policy(policy)?;
    if mosfets.is_empty() {
        return Err(MosfetNewtonSolveError::NoMosfets);
    }
    let net_set = nets.iter().cloned().collect::<BTreeSet<_>>();
    if let Some(net) = initial_voltages.keys().find(|net| !net_set.contains(*net)) {
        return Err(MosfetNewtonSolveError::UnknownInitialNet(net.clone()));
    }
    for mosfet in mosfets {
        validate_mosfet(mosfet, Some(&net_set))?;
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
        let mut linearizations = Vec::new();
        for mosfet in mosfets {
            let operating_point = operating_point_from_unknowns(mosfet, &previous)?;
            let intercept = operating_point.drain_current.clone()
                - operating_point.output_conductance.clone()
                    * operating_point.drain_source_voltage.clone()
                - operating_point.transconductance.clone()
                    * operating_point.gate_source_voltage.clone();
            stamps.push(LinearStamp::Conductance {
                component: mosfet.component.clone(),
                part: None,
                pos: mosfet.drain.clone(),
                neg: mosfet.source.clone(),
                conductance: operating_point.output_conductance.clone(),
            });
            stamps.push(LinearStamp::Vccs {
                component: mosfet.component.clone(),
                pos: mosfet.drain.clone(),
                neg: mosfet.source.clone(),
                ctrl_pos: mosfet.gate.clone(),
                ctrl_neg: mosfet.source.clone(),
                transconductance: operating_point.transconductance.clone(),
            });
            stamps.push(LinearStamp::CurrentSource {
                component: mosfet.component.clone(),
                pos: mosfet.drain.clone(),
                neg: mosfet.source.clone(),
                current: intercept.clone(),
            });
            linearizations.push(MosfetLinearizationEvidence {
                operating_point,
                current_intercept: intercept,
            });
        }
        let system = LinearMnaSystem::from_stamps(nets.clone(), &stamps)
            .map_err(MosfetNewtonSolveError::Mna)?;
        let proposal = system.solve_exact().map_err(MosfetNewtonSolveError::Mna)?;
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
                maximum_voltage_update = exact_max(maximum_voltage_update, update)?;
            }
        }
        let replay = replay_candidate(
            nets.clone(),
            linear_stamps,
            mosfets,
            &system.unknowns,
            &candidate,
            policy,
        )?;
        let update_accepted = exact_le(&maximum_voltage_update, &policy.voltage_tolerance)?;
        iterations.push(MosfetNewtonIteration {
            iteration,
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
            return Ok(MosfetNewtonSolveReport {
                status: MosfetNewtonStatus::Converged,
                unknowns: system.unknowns,
                candidate,
                iterations,
                replay,
                used_lossy_linearization: false,
            });
        }
    }
    let final_iteration = iterations
        .last()
        .expect("validated positive iteration bound produces an attempt");
    Ok(MosfetNewtonSolveReport {
        status: MosfetNewtonStatus::IterationLimit,
        unknowns: final_unknowns,
        candidate: final_candidate,
        iterations: iterations.clone(),
        replay: final_iteration.replay.clone(),
        used_lossy_linearization: false,
    })
}

impl Circuit {
    /// Extracts executable MOSFET laws from retained model roles and parameters.
    pub fn square_law_mosfets(&self) -> Result<Vec<SquareLawMosfet>, MosfetNewtonSolveError> {
        if !self.validate().is_valid() {
            return Err(MosfetNewtonSolveError::InvalidCircuit);
        }
        self.instances
            .iter()
            .filter_map(|instance| {
                let model = self
                    .device_models
                    .iter()
                    .find(|model| model.id == instance.model)
                    .expect("validated instance model must exist");
                match &model.kind {
                    DeviceModelKind::Mosfet {
                        polarity,
                        drain,
                        gate,
                        source,
                    } => Some((instance, model, *polarity, drain, gate, source)),
                    _ => None,
                }
            })
            .map(|(instance, model, polarity, drain, gate, source)| {
                let drain = bound_terminal(self, instance, drain)?;
                let gate = bound_terminal(self, instance, gate)?;
                let source = bound_terminal(self, instance, source)?;
                let threshold_voltage = mosfet_parameter(instance, model, "threshold_voltage");
                let transconductance_parameter =
                    mosfet_parameter(instance, model, "transconductance_parameter");
                let channel_length_modulation =
                    mosfet_parameter(instance, model, "channel_length_modulation")
                        .unwrap_or_else(Real::zero);
                let (Some(threshold_voltage), Some(transconductance_parameter)) =
                    (threshold_voltage, transconductance_parameter)
                else {
                    return Err(MosfetNewtonSolveError::InvalidMosfet(
                        instance.component.clone(),
                    ));
                };
                let mosfet = SquareLawMosfet {
                    component: instance.component.clone(),
                    polarity,
                    drain,
                    gate,
                    source,
                    threshold_voltage,
                    transconductance_parameter,
                    channel_length_modulation,
                };
                validate_mosfet(&mosfet, None)?;
                Ok(mosfet)
            })
            .collect()
    }

    /// Executes a retained mixed linear/MOSFET DC solve.
    pub fn solve_mosfet_dc(
        &self,
        policy: &MosfetNewtonPolicy,
        initial_voltages: &BTreeMap<NetId, Real>,
    ) -> Result<MosfetNewtonSolveReport, MosfetNewtonSolveError> {
        let mosfets = self.square_law_mosfets()?;
        if mosfets.is_empty() {
            return Err(MosfetNewtonSolveError::NoMosfets);
        }
        let components = mosfets
            .iter()
            .map(|mosfet| mosfet.component.clone())
            .collect::<BTreeSet<_>>();
        let mut lowering = self.lower_linear_devices();
        lowering.issues.retain(|issue| {
            !matches!(
                issue,
                DeviceLoweringIssue::UnsupportedModel(component)
                    if components.contains(component)
            )
        });
        if !lowering.is_complete() {
            return Err(MosfetNewtonSolveError::LinearLowering(lowering.issues));
        }
        solve_square_law_mosfet_newton(
            self.nets
                .iter()
                .filter(|net| !net.is_ground)
                .map(|net| net.id.clone())
                .collect(),
            &lowering.stamps,
            &mosfets,
            policy,
            initial_voltages,
        )
    }
}

fn replay_candidate(
    nets: Vec<NetId>,
    linear_stamps: &[LinearStamp],
    mosfets: &[SquareLawMosfet],
    unknowns: &[MnaUnknown],
    candidate: &[Real],
    policy: &MosfetNewtonPolicy,
) -> Result<MosfetResidualReplayReport, MosfetNewtonSolveError> {
    let values = unknowns
        .iter()
        .cloned()
        .zip(candidate.iter().cloned())
        .collect::<BTreeMap<_, _>>();
    let mut stamps = linear_stamps.to_vec();
    let mut operating_points = Vec::with_capacity(mosfets.len());
    for mosfet in mosfets {
        let operating_point = operating_point_from_unknowns(mosfet, &values)?;
        stamps.push(LinearStamp::CurrentSource {
            component: mosfet.component.clone(),
            pos: mosfet.drain.clone(),
            neg: mosfet.source.clone(),
            current: operating_point.drain_current.clone(),
        });
        operating_points.push(operating_point);
    }
    let system =
        LinearMnaSystem::from_stamps(nets, &stamps).map_err(MosfetNewtonSolveError::Mna)?;
    if system.unknowns != unknowns {
        return Err(MosfetNewtonSolveError::Mna(
            crate::CircuitError::CandidateLengthMismatch,
        ));
    }
    let raw = system
        .replay_candidate(candidate)
        .map_err(MosfetNewtonSolveError::Mna)?;
    let mut kcl_residuals = BTreeMap::new();
    let mut branch_residuals = BTreeMap::new();
    let mut maximum_kcl_residual = Real::zero();
    let mut maximum_branch_residual = Real::zero();
    for (unknown, residual) in unknowns.iter().zip(&raw.residuals) {
        let absolute = exact_abs(residual)?;
        match unknown {
            MnaUnknown::NetVoltage(net) => {
                maximum_kcl_residual = exact_max(maximum_kcl_residual, absolute)?;
                kcl_residuals.insert(net.clone(), residual.clone());
            }
            MnaUnknown::BranchCurrent(branch) => {
                maximum_branch_residual = exact_max(maximum_branch_residual, absolute)?;
                branch_residuals.insert(branch.clone(), residual.clone());
            }
        }
    }
    let accepted = exact_le(&maximum_kcl_residual, &policy.current_tolerance)?
        && exact_le(&maximum_branch_residual, &policy.voltage_tolerance)?;
    Ok(MosfetResidualReplayReport {
        operating_points,
        kcl_residuals,
        branch_residuals,
        maximum_kcl_residual,
        maximum_branch_residual,
        accepted,
        exact_zero: raw.accepted,
    })
}

fn operating_point_from_unknowns(
    mosfet: &SquareLawMosfet,
    values: &BTreeMap<MnaUnknown, Real>,
) -> Result<MosfetOperatingPoint, MosfetNewtonSolveError> {
    mosfet.operating_point(
        &terminal_value(&mosfet.gate, values),
        &terminal_value(&mosfet.drain, values),
        &terminal_value(&mosfet.source, values),
    )
}

fn bound_terminal(
    circuit: &Circuit,
    instance: &CircuitInstance,
    pin: &PinRef,
) -> Result<Option<NetId>, MosfetNewtonSolveError> {
    let binding = instance
        .pins
        .iter()
        .find(|binding| binding.pin == *pin)
        .ok_or_else(|| MosfetNewtonSolveError::InvalidMosfet(instance.component.clone()))?;
    let net = circuit
        .nets
        .iter()
        .find(|net| net.id == binding.net)
        .ok_or_else(|| MosfetNewtonSolveError::InvalidMosfet(instance.component.clone()))?;
    Ok((!net.is_ground).then(|| net.id.clone()))
}

fn mosfet_parameter(instance: &CircuitInstance, model: &DeviceModel, name: &str) -> Option<Real> {
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

fn validate_mosfet(
    mosfet: &SquareLawMosfet,
    nets: Option<&BTreeSet<NetId>>,
) -> Result<(), MosfetNewtonSolveError> {
    if mosfet.threshold_voltage.structural_facts().sign != Some(RealSign::Positive)
        || mosfet.transconductance_parameter.structural_facts().sign != Some(RealSign::Positive)
        || !matches!(
            mosfet.channel_length_modulation.structural_facts().sign,
            Some(RealSign::Zero | RealSign::Positive)
        )
        || nets.is_some_and(|nets| {
            [&mosfet.drain, &mosfet.gate, &mosfet.source]
                .into_iter()
                .flatten()
                .any(|net| !nets.contains(net))
        })
    {
        return Err(MosfetNewtonSolveError::InvalidMosfet(
            mosfet.component.clone(),
        ));
    }
    Ok(())
}

fn validate_policy(policy: &MosfetNewtonPolicy) -> Result<(), MosfetNewtonSolveError> {
    if policy.maximum_iterations == 0
        || policy.voltage_tolerance.structural_facts().sign != Some(RealSign::Positive)
        || policy.current_tolerance.structural_facts().sign != Some(RealSign::Positive)
        || policy.damping.structural_facts().sign != Some(RealSign::Positive)
        || !matches!(
            policy.damping.partial_cmp(&Real::one()),
            Some(Ordering::Less | Ordering::Equal)
        )
    {
        return Err(MosfetNewtonSolveError::InvalidPolicy);
    }
    Ok(())
}

fn terminal_value(terminal: &Option<NetId>, values: &BTreeMap<MnaUnknown, Real>) -> Real {
    terminal
        .as_ref()
        .and_then(|net| values.get(&MnaUnknown::NetVoltage(net.clone())))
        .cloned()
        .unwrap_or_else(Real::zero)
}

fn exact_max(current: Real, candidate: Real) -> Result<Real, MosfetNewtonSolveError> {
    match current.partial_cmp(&candidate) {
        Some(Ordering::Less) => Ok(candidate),
        Some(Ordering::Equal | Ordering::Greater) => Ok(current),
        None => Err(MosfetNewtonSolveError::IndeterminateComparison),
    }
}

fn exact_lt(left: &Real, right: &Real) -> Result<bool, MosfetNewtonSolveError> {
    match left.partial_cmp(right) {
        Some(Ordering::Less) => Ok(true),
        Some(Ordering::Equal | Ordering::Greater) => Ok(false),
        None => Err(MosfetNewtonSolveError::IndeterminateComparison),
    }
}

fn exact_le(left: &Real, right: &Real) -> Result<bool, MosfetNewtonSolveError> {
    match left.partial_cmp(right) {
        Some(Ordering::Less | Ordering::Equal) => Ok(true),
        Some(Ordering::Greater) => Ok(false),
        None => Err(MosfetNewtonSolveError::IndeterminateComparison),
    }
}

fn exact_abs(value: &Real) -> Result<Real, MosfetNewtonSolveError> {
    match value.partial_cmp(&Real::zero()) {
        Some(Ordering::Less) => Ok(-value.clone()),
        Some(Ordering::Equal | Ordering::Greater) => Ok(value.clone()),
        None => Err(MosfetNewtonSolveError::IndeterminateComparison),
    }
}
