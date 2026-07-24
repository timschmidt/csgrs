//! Exact linear frequency-domain analysis over retained device models.

use std::collections::BTreeMap;
use std::fmt::{Display, Formatter};

use hyperreal::{Real, RealSign, ZeroKnowledge};

use crate::{
    BranchId, Circuit, CircuitError, CircuitInstance, ComponentId, DeviceModel, DeviceModelKind,
    DiodeNewtonSolveReport, DiodeNewtonStatus, MnaUnknown, MosfetNewtonSolveReport,
    MosfetNewtonStatus, MosfetOperatingPoint, NetId, PartRef, ShockleyDiode, SquareLawMosfet,
};

/// Exact rectangular complex value used for AC voltages, currents, and admittances.
#[derive(Clone, Debug, PartialEq)]
pub struct Phasor {
    /// In-phase component.
    pub real: Real,
    /// Quadrature component.
    pub imaginary: Real,
}

impl Phasor {
    /// Creates a rectangular phasor.
    pub fn new(real: Real, imaginary: Real) -> Self {
        Self { real, imaginary }
    }

    /// Creates a purely real phasor.
    pub fn real(value: Real) -> Self {
        Self::new(value, Real::zero())
    }

    /// Returns exact zero.
    pub fn zero() -> Self {
        Self::real(Real::zero())
    }

    /// Returns the exact squared magnitude without introducing a square root.
    pub fn magnitude_squared(&self) -> Real {
        self.real.clone() * self.real.clone() + self.imaginary.clone() * self.imaginary.clone()
    }

    fn zero_status(&self) -> ZeroKnowledge {
        let real = self.real.zero_status();
        let imaginary = self.imaginary.zero_status();
        if real == ZeroKnowledge::NonZero || imaginary == ZeroKnowledge::NonZero {
            ZeroKnowledge::NonZero
        } else if real == ZeroKnowledge::Zero && imaginary == ZeroKnowledge::Zero {
            ZeroKnowledge::Zero
        } else {
            ZeroKnowledge::Unknown
        }
    }

    fn add(&self, other: &Self) -> Self {
        Self::new(
            self.real.clone() + other.real.clone(),
            self.imaginary.clone() + other.imaginary.clone(),
        )
    }

    fn subtract(&self, other: &Self) -> Self {
        Self::new(
            self.real.clone() - other.real.clone(),
            self.imaginary.clone() - other.imaginary.clone(),
        )
    }

    fn multiply(&self, other: &Self) -> Self {
        Self::new(
            self.real.clone() * other.real.clone()
                - self.imaginary.clone() * other.imaginary.clone(),
            self.real.clone() * other.imaginary.clone()
                + self.imaginary.clone() * other.real.clone(),
        )
    }

    fn try_divide(&self, other: &Self) -> Result<Self, CircuitError> {
        let denominator = other.real.clone() * other.real.clone()
            + other.imaginary.clone() * other.imaginary.clone();
        let real_numerator = self.real.clone() * other.real.clone()
            + self.imaginary.clone() * other.imaginary.clone();
        let imaginary_numerator = self.imaginary.clone() * other.real.clone()
            - self.real.clone() * other.imaginary.clone();
        Ok(Self::new(
            (real_numerator / denominator.clone())
                .map_err(|_| CircuitError::LinearSolveArithmetic)?,
            (imaginary_numerator / denominator).map_err(|_| CircuitError::LinearSolveArithmetic)?,
        ))
    }
}

/// Component-addressed small-signal source value.
#[derive(Clone, Debug, PartialEq)]
pub struct AcExcitation {
    /// Independent voltage- or current-source component.
    pub component: ComponentId,
    /// Exact rectangular AC amplitude.
    pub value: Phasor,
}

impl AcExcitation {
    /// Creates an AC excitation.
    pub fn new(component: ComponentId, value: Phasor) -> Self {
        Self { component, value }
    }
}

/// Provenance for a certified nonlinear DC operating point.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AcOperatingPointProvenance {
    /// Bounded Shockley Newton solve with true-law tolerance replay.
    DiodeNewton {
        /// Number of retained Newton attempts.
        iterations: usize,
        /// Diode proposal coefficients crossed the explicit lossy boundary.
        used_lossy_linearization: bool,
        /// True when the final raw MNA residual was certified exactly zero.
        exact_zero_replay: bool,
    },
    /// Bounded square-law MOSFET Newton solve with exact derivatives and replay.
    MosfetNewton {
        /// Number of retained Newton attempts.
        iterations: usize,
        /// True when the final raw MNA residual was certified exactly zero.
        exact_zero_replay: bool,
    },
}

/// Nonlinear model family certified at a DC operating point.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AcOperatingPointDeviceKind {
    /// Two-terminal Shockley exponential law.
    ShockleyDiode,
    /// Three-terminal body-tied-source square-law MOSFET.
    SquareLawMosfet,
}

/// Certified non-ground DC node voltages used for nonlinear AC linearization.
#[derive(Clone, Debug, PartialEq)]
pub struct AcOperatingPoint {
    /// Stable non-ground node voltages from the accepted DC candidate.
    voltages: BTreeMap<NetId, Real>,
    /// Exact nonlinear component families covered by the accepted replay.
    certified_devices: BTreeMap<ComponentId, AcOperatingPointDeviceKind>,
    /// Solver and replay provenance.
    provenance: AcOperatingPointProvenance,
}

/// Failure to promote a nonlinear DC solve into an AC operating point.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AcOperatingPointError {
    /// Newton reached its hard iteration bound.
    NotConverged(&'static str),
    /// The final true-law residual replay was not accepted.
    ReplayRejected(&'static str),
    /// Unknown and candidate vectors did not agree.
    CandidateLengthMismatch,
    /// The solve unexpectedly repeated a non-ground net identity.
    DuplicateNet(NetId),
    /// The replay unexpectedly repeated a nonlinear component identity.
    DuplicateComponent(ComponentId),
}

impl Display for AcOperatingPointError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotConverged(family) => {
                write!(formatter, "{family} DC operating point did not converge")
            }
            Self::ReplayRejected(family) => {
                write!(
                    formatter,
                    "{family} DC operating point failed true-law replay"
                )
            }
            Self::CandidateLengthMismatch => {
                formatter.write_str("DC operating-point unknown and candidate lengths differ")
            }
            Self::DuplicateNet(net) => {
                write!(formatter, "DC operating point repeats net {}", net.as_str())
            }
            Self::DuplicateComponent(component) => write!(
                formatter,
                "DC operating point repeats nonlinear component {}",
                component.as_str()
            ),
        }
    }
}

impl std::error::Error for AcOperatingPointError {}

impl AcOperatingPoint {
    /// Promotes an accepted converged Shockley Newton result.
    pub fn from_diode_newton(
        report: &DiodeNewtonSolveReport,
    ) -> Result<Self, AcOperatingPointError> {
        if report.status != DiodeNewtonStatus::Converged {
            return Err(AcOperatingPointError::NotConverged("diode"));
        }
        if !report.replay.accepted {
            return Err(AcOperatingPointError::ReplayRejected("diode"));
        }
        let mut certified_devices = BTreeMap::new();
        for linearization in &report
            .iterations
            .last()
            .expect("a converged diode report retains at least one iteration")
            .linearizations
        {
            if certified_devices
                .insert(
                    linearization.component.clone(),
                    AcOperatingPointDeviceKind::ShockleyDiode,
                )
                .is_some()
            {
                return Err(AcOperatingPointError::DuplicateComponent(
                    linearization.component.clone(),
                ));
            }
        }
        Self::from_candidate(
            &report.unknowns,
            &report.candidate,
            certified_devices,
            AcOperatingPointProvenance::DiodeNewton {
                iterations: report.iterations.len(),
                used_lossy_linearization: report.used_lossy_linearization,
                exact_zero_replay: report.replay.exact_zero,
            },
        )
    }

    /// Promotes an accepted converged square-law MOSFET Newton result.
    pub fn from_mosfet_newton(
        report: &MosfetNewtonSolveReport,
    ) -> Result<Self, AcOperatingPointError> {
        if report.status != MosfetNewtonStatus::Converged {
            return Err(AcOperatingPointError::NotConverged("MOSFET"));
        }
        if !report.replay.accepted {
            return Err(AcOperatingPointError::ReplayRejected("MOSFET"));
        }
        let mut certified_devices = BTreeMap::new();
        for device in &report.replay.operating_points {
            if certified_devices
                .insert(
                    device.component.clone(),
                    AcOperatingPointDeviceKind::SquareLawMosfet,
                )
                .is_some()
            {
                return Err(AcOperatingPointError::DuplicateComponent(
                    device.component.clone(),
                ));
            }
        }
        Self::from_candidate(
            &report.unknowns,
            &report.candidate,
            certified_devices,
            AcOperatingPointProvenance::MosfetNewton {
                iterations: report.iterations.len(),
                exact_zero_replay: report.replay.exact_zero,
            },
        )
    }

    fn from_candidate(
        unknowns: &[MnaUnknown],
        candidate: &[Real],
        certified_devices: BTreeMap<ComponentId, AcOperatingPointDeviceKind>,
        provenance: AcOperatingPointProvenance,
    ) -> Result<Self, AcOperatingPointError> {
        if unknowns.len() != candidate.len() {
            return Err(AcOperatingPointError::CandidateLengthMismatch);
        }
        let mut voltages = BTreeMap::new();
        for (unknown, value) in unknowns.iter().zip(candidate) {
            if let MnaUnknown::NetVoltage(net) = unknown
                && voltages.insert(net.clone(), value.clone()).is_some()
            {
                return Err(AcOperatingPointError::DuplicateNet(net.clone()));
            }
        }
        Ok(Self {
            voltages,
            certified_devices,
            provenance,
        })
    }

    /// Returns every certified non-ground node voltage.
    pub fn voltages(&self) -> &BTreeMap<NetId, Real> {
        &self.voltages
    }

    /// Returns one certified non-ground node voltage.
    pub fn voltage(&self, net: &NetId) -> Option<&Real> {
        self.voltages.get(net)
    }

    /// Returns the nonlinear component families covered by the accepted replay.
    pub fn certified_devices(&self) -> &BTreeMap<ComponentId, AcOperatingPointDeviceKind> {
        &self.certified_devices
    }

    /// Returns the nonlinear DC solver and replay provenance.
    pub fn provenance(&self) -> &AcOperatingPointProvenance {
        &self.provenance
    }

    fn terminal_voltage(&self, terminal: &Option<NetId>) -> Real {
        terminal
            .as_ref()
            .and_then(|net| self.voltages.get(net))
            .cloned()
            .unwrap_or_else(Real::zero)
    }
}

/// Device instance that could not be represented by linear AC analysis.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AcDeviceLoweringIssue {
    /// A primitive requires more connected pins than the instance supplies.
    MissingPins {
        /// Component identity.
        component: ComponentId,
        /// Required pin count.
        required: usize,
    },
    /// A required exact parameter was absent.
    MissingParameter {
        /// Component identity.
        component: ComponentId,
        /// Parameter name.
        parameter: String,
    },
    /// A passive value was not certified strictly positive.
    InvalidParameter {
        /// Component identity.
        component: ComponentId,
        /// Parameter name.
        parameter: String,
    },
    /// The device family requires a nonlinear small-signal operating-point model.
    UnsupportedModel(ComponentId),
    /// The supplied DC replay did not certify this retained nonlinear model.
    UncertifiedOperatingPoint {
        /// Stable retained component.
        component: ComponentId,
        /// Model family required for incremental lowering.
        required: AcOperatingPointDeviceKind,
    },
    /// A retained nonlinear device could not be linearized at the supplied point.
    NonlinearLinearization {
        /// Stable failing component.
        component: ComponentId,
        /// Model extraction or derivative detail.
        detail: String,
    },
}

/// Failure to construct or solve a linear AC analysis.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AcAnalysisError {
    /// Circuit structure was invalid.
    InvalidCircuit,
    /// Angular frequency was not certified strictly positive.
    InvalidAngularFrequency,
    /// More than one excitation addressed the same component.
    DuplicateExcitation(ComponentId),
    /// An excitation addressed no circuit instance.
    UnknownExcitation(ComponentId),
    /// An excitation addressed a device other than an independent source.
    InvalidExcitationTarget(ComponentId),
    /// One or more device instances could not be lowered.
    Incomplete(Vec<AcDeviceLoweringIssue>),
    /// A nonlinear DC solve could not become certified AC input.
    OperatingPoint(AcOperatingPointError),
    /// A circuit net was absent from the supplied operating point.
    MissingOperatingPointVoltage(NetId),
    /// The supplied operating point contained a net outside this circuit.
    UnknownOperatingPointVoltage(NetId),
    /// Exact matrix assembly, elimination, or replay failed.
    Circuit(CircuitError),
}

impl Display for AcAnalysisError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidCircuit => formatter.write_str("circuit structure is invalid"),
            Self::InvalidAngularFrequency => {
                formatter.write_str("AC angular frequency must be exactly positive")
            }
            Self::DuplicateExcitation(component) => {
                write!(formatter, "duplicate AC excitation for {component:?}")
            }
            Self::UnknownExcitation(component) => {
                write!(formatter, "AC excitation references unknown {component:?}")
            }
            Self::InvalidExcitationTarget(component) => write!(
                formatter,
                "AC excitation target {component:?} is not an independent source"
            ),
            Self::Incomplete(issues) => write!(
                formatter,
                "{} device instance(s) could not be lowered for AC analysis",
                issues.len()
            ),
            Self::OperatingPoint(error) => write!(formatter, "invalid AC operating point: {error}"),
            Self::MissingOperatingPointVoltage(net) => write!(
                formatter,
                "AC operating point is missing net {}",
                net.as_str()
            ),
            Self::UnknownOperatingPointVoltage(net) => write!(
                formatter,
                "AC operating point contains unknown net {}",
                net.as_str()
            ),
            Self::Circuit(error) => write!(formatter, "AC MNA failed: {error}"),
        }
    }
}

impl std::error::Error for AcAnalysisError {}

impl From<AcOperatingPointError> for AcAnalysisError {
    fn from(error: AcOperatingPointError) -> Self {
        Self::OperatingPoint(error)
    }
}

/// Frequency-domain circuit stamp retained for review.
#[derive(Clone, Debug, PartialEq)]
pub enum AcStamp {
    /// Complex admittance between two terminals.
    Admittance {
        /// Component identity.
        component: ComponentId,
        /// Optional parts-library identity.
        part: Option<PartRef>,
        /// Positive terminal, or ground.
        pos: Option<NetId>,
        /// Negative terminal, or ground.
        neg: Option<NetId>,
        /// Exact complex admittance.
        admittance: Phasor,
    },
    /// Independent AC current source from `pos` to `neg`.
    CurrentSource {
        /// Component identity.
        component: ComponentId,
        /// Positive terminal, or ground.
        pos: Option<NetId>,
        /// Negative terminal, or ground.
        neg: Option<NetId>,
        /// Exact complex source current.
        current: Phasor,
    },
    /// Independent AC voltage source with one introduced branch current.
    VoltageSource {
        /// Component identity.
        component: ComponentId,
        /// Introduced branch identity.
        branch: BranchId,
        /// Positive terminal, or ground.
        pos: Option<NetId>,
        /// Negative terminal, or ground.
        neg: Option<NetId>,
        /// Exact complex source voltage.
        voltage: Phasor,
    },
    /// Voltage-controlled current source.
    Vccs {
        /// Component identity.
        component: ComponentId,
        /// Positive output terminal, or ground.
        pos: Option<NetId>,
        /// Negative output terminal, or ground.
        neg: Option<NetId>,
        /// Positive control terminal, or ground.
        ctrl_pos: Option<NetId>,
        /// Negative control terminal, or ground.
        ctrl_neg: Option<NetId>,
        /// Exact real transconductance represented as a phasor.
        transconductance: Phasor,
    },
}

/// Audited incremental nonlinear model used by small-signal AC.
#[derive(Clone, Debug, PartialEq)]
pub enum AcNonlinearLinearization {
    /// Shockley diode slope imported at the accepted DC junction voltage.
    Diode {
        /// Stable retained component.
        component: ComponentId,
        /// Exact DC anode-to-cathode voltage.
        operating_voltage: Real,
        /// Exact imported dyadic small-signal conductance.
        conductance: Real,
        /// Exact imported dyadic affine intercept retained for audit.
        current_intercept: Real,
        /// The exponential slope crossed the explicit primitive-float boundary.
        used_lossy_linearization: bool,
    },
    /// Exact square-law MOSFET `gm`/`gds` at the accepted DC point.
    Mosfet {
        /// Complete exact operating-region and derivative evidence.
        operating_point: MosfetOperatingPoint,
    },
}

/// Inspectable AC device lowering before matrix assembly.
#[derive(Clone, Debug, PartialEq)]
pub struct AcDeviceLoweringReport {
    /// Exact stamps in circuit instance order.
    pub stamps: Vec<AcStamp>,
    /// Nonlinear operating-point derivatives used by the stamps.
    pub nonlinear_linearizations: Vec<AcNonlinearLinearization>,
    /// Unsupported or incomplete devices.
    pub issues: Vec<AcDeviceLoweringIssue>,
}

impl AcDeviceLoweringReport {
    /// Returns true when every retained device has a complete AC representation.
    pub fn is_complete(&self) -> bool {
        self.issues.is_empty()
    }
}

/// Dense exact complex MNA system at one angular frequency.
#[derive(Clone, Debug, PartialEq)]
pub struct AcMnaSystem {
    /// Exact positive angular frequency in radians per second.
    pub angular_frequency: Real,
    /// Unknown ordering shared with DC/transient MNA.
    pub unknowns: Vec<MnaUnknown>,
    /// Dense complex matrix.
    pub matrix: Vec<Vec<Phasor>>,
    /// Dense complex right-hand side.
    pub rhs: Vec<Phasor>,
    /// Source-addressable stamps used to assemble the system.
    pub stamps: Vec<AcStamp>,
}

/// Exact complex residual replay.
#[derive(Clone, Debug, PartialEq)]
pub struct AcResidualReplayReport {
    /// Complex residual vector `A*x - b`.
    pub residuals: Vec<Phasor>,
    /// True only when every real and imaginary residual is certified zero.
    pub accepted: bool,
}

/// One exact AC solution with mandatory residual replay.
#[derive(Clone, Debug, PartialEq)]
pub struct AcSolveReport {
    /// Candidate phasors in system unknown order.
    pub candidate: Vec<Phasor>,
    /// Replay against the original complex equations.
    pub replay: AcResidualReplayReport,
}

/// One point in an exact authored angular-frequency sweep.
#[derive(Clone, Debug, PartialEq)]
pub struct AcSweepPoint {
    /// Angular frequency in radians per second.
    pub angular_frequency: Real,
    /// Unknown ordering.
    pub unknowns: Vec<MnaUnknown>,
    /// Exact solution and replay.
    pub solution: AcSolveReport,
}

/// Ordered exact AC sweep.
#[derive(Clone, Debug, PartialEq)]
pub struct AcSweepReport {
    /// Results in caller-authored frequency order.
    pub points: Vec<AcSweepPoint>,
}

/// Ordered nonlinear small-signal AC sweep with DC provenance and derivatives.
#[derive(Clone, Debug, PartialEq)]
pub struct AcSmallSignalSweepReport {
    /// Certified DC node voltages and solver/replay provenance.
    pub operating_point: AcOperatingPoint,
    /// Incremental diode/MOSF models shared by every frequency point.
    pub nonlinear_linearizations: Vec<AcNonlinearLinearization>,
    /// Exact complex solutions in caller-authored frequency order.
    pub points: Vec<AcSweepPoint>,
}

impl AcSweepPoint {
    /// Returns one solved value by its stable MNA unknown identity.
    pub fn value(&self, unknown: &MnaUnknown) -> Option<&Phasor> {
        self.unknowns
            .iter()
            .position(|candidate| candidate == unknown)
            .and_then(|index| self.solution.candidate.get(index))
    }

    /// Returns one non-ground net voltage by stable net identity.
    pub fn net_voltage(&self, net: &NetId) -> Option<&Phasor> {
        self.value(&MnaUnknown::NetVoltage(net.clone()))
    }
}

impl AcMnaSystem {
    fn from_stamps(
        angular_frequency: Real,
        nets: Vec<NetId>,
        stamps: Vec<AcStamp>,
    ) -> Result<Self, CircuitError> {
        let mut unknowns = nets
            .into_iter()
            .map(MnaUnknown::NetVoltage)
            .collect::<Vec<_>>();
        for stamp in &stamps {
            if let AcStamp::VoltageSource { branch, .. } = stamp {
                unknowns.push(MnaUnknown::BranchCurrent(branch.clone()));
            }
        }
        let mut net_indices = BTreeMap::new();
        let mut branch_indices = BTreeMap::new();
        for (index, unknown) in unknowns.iter().enumerate() {
            match unknown {
                MnaUnknown::NetVoltage(net) => {
                    if net_indices.insert(net, index).is_some() {
                        return Err(CircuitError::DuplicateUnknown);
                    }
                }
                MnaUnknown::BranchCurrent(branch) => {
                    if branch_indices.insert(branch, index).is_some() {
                        return Err(CircuitError::DuplicateUnknown);
                    }
                }
            }
        }
        let dimension = unknowns.len();
        let mut matrix = vec![vec![Phasor::zero(); dimension]; dimension];
        let mut rhs = vec![Phasor::zero(); dimension];
        for stamp in &stamps {
            match stamp {
                AcStamp::Admittance {
                    pos,
                    neg,
                    admittance,
                    ..
                } => {
                    let pair = net_pair(&net_indices, pos, neg)?;
                    stamp_admittance(&mut matrix, pair.0, pair.1, admittance);
                }
                AcStamp::CurrentSource {
                    pos, neg, current, ..
                } => {
                    let pair = net_pair(&net_indices, pos, neg)?;
                    stamp_current(&mut rhs, pair.0, pair.1, current);
                }
                AcStamp::VoltageSource {
                    branch,
                    pos,
                    neg,
                    voltage,
                    ..
                } => {
                    let pair = net_pair(&net_indices, pos, neg)?;
                    let branch = branch_indices
                        .get(branch)
                        .copied()
                        .ok_or(CircuitError::MissingNet)?;
                    stamp_voltage(&mut matrix, &mut rhs, branch, pair.0, pair.1, voltage);
                }
                AcStamp::Vccs {
                    pos,
                    neg,
                    ctrl_pos,
                    ctrl_neg,
                    transconductance,
                    ..
                } => {
                    let output = net_pair(&net_indices, pos, neg)?;
                    let control = net_pair(&net_indices, ctrl_pos, ctrl_neg)?;
                    stamp_vccs(
                        &mut matrix,
                        output.0,
                        output.1,
                        control.0,
                        control.1,
                        transconductance,
                    );
                }
            }
        }
        Ok(Self {
            angular_frequency,
            unknowns,
            matrix,
            rhs,
            stamps,
        })
    }

    /// Replays one candidate through the original exact complex equations.
    pub fn replay_candidate(
        &self,
        candidate: &[Phasor],
    ) -> Result<AcResidualReplayReport, CircuitError> {
        if candidate.len() != self.unknowns.len() {
            return Err(CircuitError::CandidateLengthMismatch);
        }
        let mut residuals = Vec::with_capacity(self.unknowns.len());
        let mut accepted = true;
        for (row, rhs) in self.matrix.iter().zip(&self.rhs) {
            let sum = row
                .iter()
                .zip(candidate)
                .fold(Phasor::zero(), |sum, (coefficient, value)| {
                    sum.add(&coefficient.multiply(value))
                });
            let residual = sum.subtract(rhs);
            match residual.zero_status() {
                ZeroKnowledge::Zero => {}
                ZeroKnowledge::NonZero => accepted = false,
                ZeroKnowledge::Unknown => return Err(CircuitError::UnknownResidual),
            }
            residuals.push(residual);
        }
        Ok(AcResidualReplayReport {
            residuals,
            accepted,
        })
    }

    /// Solves and exactly replays this dense complex MNA system.
    pub fn solve_exact(&self) -> Result<AcSolveReport, CircuitError> {
        let dimension = self.unknowns.len();
        if self.matrix.len() != dimension
            || self.matrix.iter().any(|row| row.len() != dimension)
            || self.rhs.len() != dimension
        {
            return Err(CircuitError::CandidateLengthMismatch);
        }
        let mut matrix = self.matrix.clone();
        let mut rhs = self.rhs.clone();
        for column in 0..dimension {
            let mut pivot = None;
            let mut unknown_pivot = false;
            for (row, candidate) in matrix.iter().enumerate().skip(column) {
                match candidate[column].zero_status() {
                    ZeroKnowledge::NonZero => {
                        pivot = Some(row);
                        break;
                    }
                    ZeroKnowledge::Unknown => unknown_pivot = true,
                    ZeroKnowledge::Zero => {}
                }
            }
            let Some(pivot) = pivot else {
                return Err(if unknown_pivot {
                    CircuitError::IndeterminateLinearPivot
                } else {
                    CircuitError::SingularLinearSystem
                });
            };
            matrix.swap(column, pivot);
            rhs.swap(column, pivot);
            let divisor = matrix[column][column].clone();
            for entry in &mut matrix[column][column..] {
                *entry = entry.try_divide(&divisor)?;
            }
            rhs[column] = rhs[column].try_divide(&divisor)?;
            let pivot_row = matrix[column].clone();
            let pivot_rhs = rhs[column].clone();
            for row in 0..dimension {
                if row == column {
                    continue;
                }
                let factor = matrix[row][column].clone();
                if factor.zero_status() == ZeroKnowledge::Zero {
                    continue;
                }
                for (entry, pivot_entry) in
                    matrix[row][column..].iter_mut().zip(&pivot_row[column..])
                {
                    *entry = entry.subtract(&factor.multiply(pivot_entry));
                }
                rhs[row] = rhs[row].subtract(&factor.multiply(&pivot_rhs));
            }
        }
        let replay = self.replay_candidate(&rhs)?;
        if !replay.accepted {
            return Err(CircuitError::UnknownResidual);
        }
        Ok(AcSolveReport {
            candidate: rhs,
            replay,
        })
    }
}

impl Circuit {
    /// Lowers retained linear devices at one exact positive angular frequency.
    pub fn lower_ac_devices(
        &self,
        angular_frequency: Real,
        excitations: &[AcExcitation],
    ) -> Result<AcDeviceLoweringReport, AcAnalysisError> {
        self.lower_ac_devices_internal(angular_frequency, excitations, None)
    }

    /// Lowers linear and retained nonlinear devices around one certified DC
    /// operating point.
    pub fn lower_small_signal_ac_devices(
        &self,
        angular_frequency: Real,
        excitations: &[AcExcitation],
        operating_point: &AcOperatingPoint,
    ) -> Result<AcDeviceLoweringReport, AcAnalysisError> {
        self.lower_ac_devices_internal(angular_frequency, excitations, Some(operating_point))
    }

    fn lower_ac_devices_internal(
        &self,
        angular_frequency: Real,
        excitations: &[AcExcitation],
        operating_point: Option<&AcOperatingPoint>,
    ) -> Result<AcDeviceLoweringReport, AcAnalysisError> {
        if !self.validate().is_valid() {
            return Err(AcAnalysisError::InvalidCircuit);
        }
        if angular_frequency.structural_facts().sign != Some(RealSign::Positive) {
            return Err(AcAnalysisError::InvalidAngularFrequency);
        }
        let excitation_map = validate_excitations(self, excitations)?;
        if let Some(operating_point) = operating_point {
            validate_operating_point(self, operating_point)?;
        }
        let mut report = AcDeviceLoweringReport {
            stamps: Vec::new(),
            nonlinear_linearizations: Vec::new(),
            issues: Vec::new(),
        };
        for instance in &self.instances {
            let model = self
                .device_models
                .iter()
                .find(|model| model.id == instance.model)
                .expect("validated instance model must exist");
            lower_ac_instance(
                self,
                instance,
                model,
                &angular_frequency,
                excitation_map
                    .get(&instance.component)
                    .cloned()
                    .unwrap_or_else(Phasor::zero),
                operating_point,
                &mut report,
            );
        }
        Ok(report)
    }

    /// Builds one exact complex MNA system from retained device models.
    pub fn ac_mna_at(
        &self,
        angular_frequency: Real,
        excitations: &[AcExcitation],
    ) -> Result<AcMnaSystem, AcAnalysisError> {
        let lowering = self.lower_ac_devices(angular_frequency.clone(), excitations)?;
        if !lowering.is_complete() {
            return Err(AcAnalysisError::Incomplete(lowering.issues));
        }
        AcMnaSystem::from_stamps(
            angular_frequency,
            self.nets
                .iter()
                .filter(|net| !net.is_ground)
                .map(|net| net.id.clone())
                .collect(),
            lowering.stamps,
        )
        .map_err(AcAnalysisError::Circuit)
    }

    /// Builds one exact complex MNA system around a certified nonlinear DC
    /// operating point.
    pub fn small_signal_ac_mna_at(
        &self,
        angular_frequency: Real,
        excitations: &[AcExcitation],
        operating_point: &AcOperatingPoint,
    ) -> Result<AcMnaSystem, AcAnalysisError> {
        let lowering = self.lower_small_signal_ac_devices(
            angular_frequency.clone(),
            excitations,
            operating_point,
        )?;
        if !lowering.is_complete() {
            return Err(AcAnalysisError::Incomplete(lowering.issues));
        }
        AcMnaSystem::from_stamps(
            angular_frequency,
            self.nets
                .iter()
                .filter(|net| !net.is_ground)
                .map(|net| net.id.clone())
                .collect(),
            lowering.stamps,
        )
        .map_err(AcAnalysisError::Circuit)
    }

    /// Solves an ordered exact angular-frequency sweep.
    pub fn ac_sweep(
        &self,
        angular_frequencies: impl IntoIterator<Item = Real>,
        excitations: &[AcExcitation],
    ) -> Result<AcSweepReport, AcAnalysisError> {
        let mut points = Vec::new();
        for angular_frequency in angular_frequencies {
            let system = self.ac_mna_at(angular_frequency.clone(), excitations)?;
            let solution = system.solve_exact().map_err(AcAnalysisError::Circuit)?;
            points.push(AcSweepPoint {
                angular_frequency,
                unknowns: system.unknowns,
                solution,
            });
        }
        Ok(AcSweepReport { points })
    }

    /// Solves an ordered exact AC sweep around one certified nonlinear DC
    /// operating point and retains every incremental device derivative.
    pub fn small_signal_ac_sweep(
        &self,
        angular_frequencies: impl IntoIterator<Item = Real>,
        excitations: &[AcExcitation],
        operating_point: &AcOperatingPoint,
    ) -> Result<AcSmallSignalSweepReport, AcAnalysisError> {
        let mut points = Vec::new();
        let mut nonlinear_linearizations = None::<Vec<AcNonlinearLinearization>>;
        for angular_frequency in angular_frequencies {
            let lowering = self.lower_small_signal_ac_devices(
                angular_frequency.clone(),
                excitations,
                operating_point,
            )?;
            if !lowering.is_complete() {
                return Err(AcAnalysisError::Incomplete(lowering.issues));
            }
            match &nonlinear_linearizations {
                Some(existing) => debug_assert_eq!(existing, &lowering.nonlinear_linearizations),
                None => {
                    nonlinear_linearizations = Some(lowering.nonlinear_linearizations.clone());
                }
            }
            let system = AcMnaSystem::from_stamps(
                angular_frequency.clone(),
                self.nets
                    .iter()
                    .filter(|net| !net.is_ground)
                    .map(|net| net.id.clone())
                    .collect(),
                lowering.stamps,
            )
            .map_err(AcAnalysisError::Circuit)?;
            let solution = system.solve_exact().map_err(AcAnalysisError::Circuit)?;
            points.push(AcSweepPoint {
                angular_frequency,
                unknowns: system.unknowns,
                solution,
            });
        }
        Ok(AcSmallSignalSweepReport {
            operating_point: operating_point.clone(),
            nonlinear_linearizations: nonlinear_linearizations.unwrap_or_default(),
            points,
        })
    }
}

fn validate_excitations(
    circuit: &Circuit,
    excitations: &[AcExcitation],
) -> Result<BTreeMap<ComponentId, Phasor>, AcAnalysisError> {
    let mut values = BTreeMap::new();
    for excitation in excitations {
        if values
            .insert(excitation.component.clone(), excitation.value.clone())
            .is_some()
        {
            return Err(AcAnalysisError::DuplicateExcitation(
                excitation.component.clone(),
            ));
        }
        let Some(instance) = circuit
            .instances
            .iter()
            .find(|instance| instance.component == excitation.component)
        else {
            return Err(AcAnalysisError::UnknownExcitation(
                excitation.component.clone(),
            ));
        };
        let model = circuit
            .device_models
            .iter()
            .find(|model| model.id == instance.model)
            .expect("validated model exists");
        if !matches!(
            model.kind,
            DeviceModelKind::CurrentSource | DeviceModelKind::VoltageSource
        ) {
            return Err(AcAnalysisError::InvalidExcitationTarget(
                excitation.component.clone(),
            ));
        }
    }
    Ok(values)
}

fn validate_operating_point(
    circuit: &Circuit,
    operating_point: &AcOperatingPoint,
) -> Result<(), AcAnalysisError> {
    for net in circuit.nets.iter().filter(|net| !net.is_ground) {
        if !operating_point.voltages.contains_key(&net.id) {
            return Err(AcAnalysisError::MissingOperatingPointVoltage(
                net.id.clone(),
            ));
        }
    }
    for net in operating_point.voltages.keys() {
        if !circuit
            .nets
            .iter()
            .any(|candidate| !candidate.is_ground && candidate.id == *net)
        {
            return Err(AcAnalysisError::UnknownOperatingPointVoltage(net.clone()));
        }
    }
    Ok(())
}

fn lower_ac_instance(
    circuit: &Circuit,
    instance: &CircuitInstance,
    model: &DeviceModel,
    angular_frequency: &Real,
    excitation: Phasor,
    operating_point: Option<&AcOperatingPoint>,
    report: &mut AcDeviceLoweringReport,
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
    let required = match model.kind {
        DeviceModelKind::ControlledSource => 4,
        DeviceModelKind::Resistor
        | DeviceModelKind::Capacitor
        | DeviceModelKind::Inductor
        | DeviceModelKind::CurrentSource
        | DeviceModelKind::VoltageSource => 2,
        DeviceModelKind::Diode if operating_point.is_some() => 2,
        DeviceModelKind::Mosfet { .. } if operating_point.is_some() => 3,
        DeviceModelKind::Diode | DeviceModelKind::Mosfet { .. } => {
            report.issues.push(AcDeviceLoweringIssue::UnsupportedModel(
                instance.component.clone(),
            ));
            return;
        }
        DeviceModelKind::NonlinearPlaceholder | DeviceModelKind::Custom(_) => {
            report.issues.push(AcDeviceLoweringIssue::UnsupportedModel(
                instance.component.clone(),
            ));
            return;
        }
    };
    if pins.len() < required {
        report.issues.push(AcDeviceLoweringIssue::MissingPins {
            component: instance.component.clone(),
            required,
        });
        return;
    }
    match model.kind {
        DeviceModelKind::Resistor => {
            let conductance = if let Some(value) = parameter(instance, model, "conductance") {
                if !is_positive(value) {
                    invalid_parameter(report, instance, "conductance");
                    return;
                }
                value.clone()
            } else if let Some(value) = parameter(instance, model, "resistance") {
                if !is_positive(value) {
                    invalid_parameter(report, instance, "resistance");
                    return;
                }
                let Ok(conductance) = Real::one() / value.clone() else {
                    invalid_parameter(report, instance, "resistance");
                    return;
                };
                conductance
            } else {
                missing_parameter(report, instance, "resistance or conductance");
                return;
            };
            report.stamps.push(AcStamp::Admittance {
                component: instance.component.clone(),
                part: instance.part.clone(),
                pos: pins[0].clone(),
                neg: pins[1].clone(),
                admittance: Phasor::real(conductance),
            });
        }
        DeviceModelKind::Capacitor => {
            let Some(capacitance) = parameter(instance, model, "capacitance") else {
                missing_parameter(report, instance, "capacitance");
                return;
            };
            if !is_positive(capacitance) {
                invalid_parameter(report, instance, "capacitance");
                return;
            }
            report.stamps.push(AcStamp::Admittance {
                component: instance.component.clone(),
                part: instance.part.clone(),
                pos: pins[0].clone(),
                neg: pins[1].clone(),
                admittance: Phasor::new(
                    Real::zero(),
                    angular_frequency.clone() * capacitance.clone(),
                ),
            });
        }
        DeviceModelKind::Inductor => {
            let Some(inductance) = parameter(instance, model, "inductance") else {
                missing_parameter(report, instance, "inductance");
                return;
            };
            if !is_positive(inductance) {
                invalid_parameter(report, instance, "inductance");
                return;
            }
            let denominator = angular_frequency.clone() * inductance.clone();
            let Ok(susceptance) = -Real::one() / denominator else {
                invalid_parameter(report, instance, "inductance");
                return;
            };
            report.stamps.push(AcStamp::Admittance {
                component: instance.component.clone(),
                part: instance.part.clone(),
                pos: pins[0].clone(),
                neg: pins[1].clone(),
                admittance: Phasor::new(Real::zero(), susceptance),
            });
        }
        DeviceModelKind::CurrentSource => report.stamps.push(AcStamp::CurrentSource {
            component: instance.component.clone(),
            pos: pins[0].clone(),
            neg: pins[1].clone(),
            current: excitation,
        }),
        DeviceModelKind::VoltageSource => report.stamps.push(AcStamp::VoltageSource {
            component: instance.component.clone(),
            branch: branch(instance),
            pos: pins[0].clone(),
            neg: pins[1].clone(),
            voltage: excitation,
        }),
        DeviceModelKind::ControlledSource => {
            let Some(transconductance) = parameter(instance, model, "transconductance") else {
                missing_parameter(report, instance, "transconductance");
                return;
            };
            report.stamps.push(AcStamp::Vccs {
                component: instance.component.clone(),
                pos: pins[0].clone(),
                neg: pins[1].clone(),
                ctrl_pos: pins[2].clone(),
                ctrl_neg: pins[3].clone(),
                transconductance: Phasor::real(transconductance.clone()),
            });
        }
        DeviceModelKind::Diode => {
            let operating_point =
                operating_point.expect("nonlinear AC arm requires an operating point");
            if operating_point.certified_devices.get(&instance.component)
                != Some(&AcOperatingPointDeviceKind::ShockleyDiode)
            {
                report
                    .issues
                    .push(AcDeviceLoweringIssue::UncertifiedOperatingPoint {
                        component: instance.component.clone(),
                        required: AcOperatingPointDeviceKind::ShockleyDiode,
                    });
                return;
            }
            let Some(saturation_current) = parameter(instance, model, "saturation_current") else {
                missing_parameter(report, instance, "saturation_current");
                return;
            };
            let Some(thermal_voltage) = parameter(instance, model, "thermal_voltage") else {
                missing_parameter(report, instance, "thermal_voltage");
                return;
            };
            if !is_positive(saturation_current) {
                invalid_parameter(report, instance, "saturation_current");
                return;
            }
            if !is_positive(thermal_voltage) {
                invalid_parameter(report, instance, "thermal_voltage");
                return;
            }
            let diode = ShockleyDiode {
                component: instance.component.clone(),
                anode: pins[0].clone(),
                cathode: pins[1].clone(),
                saturation_current: saturation_current.clone(),
                thermal_voltage: thermal_voltage.clone(),
            };
            let voltage = operating_point.terminal_voltage(&diode.anode)
                - operating_point.terminal_voltage(&diode.cathode);
            let linearization = match diode.linearize_at(&voltage) {
                Ok(linearization) => linearization,
                Err(error) => {
                    nonlinear_issue(report, instance, error.to_string());
                    return;
                }
            };
            report.stamps.push(AcStamp::Admittance {
                component: instance.component.clone(),
                part: instance.part.clone(),
                pos: diode.anode,
                neg: diode.cathode,
                admittance: Phasor::real(linearization.conductance.clone()),
            });
            report
                .nonlinear_linearizations
                .push(AcNonlinearLinearization::Diode {
                    component: instance.component.clone(),
                    operating_voltage: linearization.voltage,
                    conductance: linearization.conductance,
                    current_intercept: linearization.current_intercept,
                    used_lossy_linearization: true,
                });
        }
        DeviceModelKind::Mosfet {
            polarity,
            ref drain,
            ref gate,
            ref source,
        } => {
            let operating_point =
                operating_point.expect("nonlinear AC arm requires an operating point");
            if operating_point.certified_devices.get(&instance.component)
                != Some(&AcOperatingPointDeviceKind::SquareLawMosfet)
            {
                report
                    .issues
                    .push(AcDeviceLoweringIssue::UncertifiedOperatingPoint {
                        component: instance.component.clone(),
                        required: AcOperatingPointDeviceKind::SquareLawMosfet,
                    });
                return;
            }
            let Some(drain) = bound_ac_terminal(circuit, instance, drain) else {
                nonlinear_issue(report, instance, "missing drain binding");
                return;
            };
            let Some(gate) = bound_ac_terminal(circuit, instance, gate) else {
                nonlinear_issue(report, instance, "missing gate binding");
                return;
            };
            let Some(source) = bound_ac_terminal(circuit, instance, source) else {
                nonlinear_issue(report, instance, "missing source binding");
                return;
            };
            let Some(threshold_voltage) = parameter(instance, model, "threshold_voltage") else {
                missing_parameter(report, instance, "threshold_voltage");
                return;
            };
            let Some(transconductance_parameter) =
                parameter(instance, model, "transconductance_parameter")
            else {
                missing_parameter(report, instance, "transconductance_parameter");
                return;
            };
            let channel_length_modulation = parameter(instance, model, "channel_length_modulation")
                .cloned()
                .unwrap_or_else(Real::zero);
            let mosfet = SquareLawMosfet {
                component: instance.component.clone(),
                polarity,
                drain: drain.clone(),
                gate: gate.clone(),
                source: source.clone(),
                threshold_voltage: threshold_voltage.clone(),
                transconductance_parameter: transconductance_parameter.clone(),
                channel_length_modulation,
            };
            let nonlinear = match mosfet.operating_point(
                &operating_point.terminal_voltage(&gate),
                &operating_point.terminal_voltage(&drain),
                &operating_point.terminal_voltage(&source),
            ) {
                Ok(nonlinear) => nonlinear,
                Err(error) => {
                    nonlinear_issue(report, instance, error.to_string());
                    return;
                }
            };
            report.stamps.push(AcStamp::Admittance {
                component: instance.component.clone(),
                part: instance.part.clone(),
                pos: drain.clone(),
                neg: source.clone(),
                admittance: Phasor::real(nonlinear.output_conductance.clone()),
            });
            report.stamps.push(AcStamp::Vccs {
                component: instance.component.clone(),
                pos: drain,
                neg: source.clone(),
                ctrl_pos: gate,
                ctrl_neg: source,
                transconductance: Phasor::real(nonlinear.transconductance.clone()),
            });
            report
                .nonlinear_linearizations
                .push(AcNonlinearLinearization::Mosfet {
                    operating_point: nonlinear,
                });
        }
        DeviceModelKind::NonlinearPlaceholder | DeviceModelKind::Custom(_) => unreachable!(),
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

fn is_positive(value: &Real) -> bool {
    value.structural_facts().sign == Some(RealSign::Positive)
}

fn missing_parameter(
    report: &mut AcDeviceLoweringReport,
    instance: &CircuitInstance,
    parameter: &str,
) {
    report.issues.push(AcDeviceLoweringIssue::MissingParameter {
        component: instance.component.clone(),
        parameter: parameter.into(),
    });
}

fn invalid_parameter(
    report: &mut AcDeviceLoweringReport,
    instance: &CircuitInstance,
    parameter: &str,
) {
    report.issues.push(AcDeviceLoweringIssue::InvalidParameter {
        component: instance.component.clone(),
        parameter: parameter.into(),
    });
}

fn nonlinear_issue(
    report: &mut AcDeviceLoweringReport,
    instance: &CircuitInstance,
    detail: impl Into<String>,
) {
    report
        .issues
        .push(AcDeviceLoweringIssue::NonlinearLinearization {
            component: instance.component.clone(),
            detail: detail.into(),
        });
}

fn bound_ac_terminal(
    circuit: &Circuit,
    instance: &CircuitInstance,
    pin: &crate::PinRef,
) -> Option<Option<NetId>> {
    instance
        .pins
        .iter()
        .find(|binding| binding.pin == *pin)
        .map(|binding| net_terminal(circuit, &binding.net))
}

fn net_terminal(circuit: &Circuit, net: &NetId) -> Option<NetId> {
    circuit
        .nets
        .iter()
        .find(|candidate| candidate.id == *net)
        .filter(|candidate| !candidate.is_ground)
        .map(|candidate| candidate.id.clone())
}

fn branch(instance: &CircuitInstance) -> BranchId {
    BranchId::new(format!("{}:voltage-ac", instance.id.as_str()))
        .expect("branch generated from nonempty instance id")
}

fn net_index(
    index: &BTreeMap<&NetId, usize>,
    net: &Option<NetId>,
) -> Result<Option<usize>, CircuitError> {
    net.as_ref()
        .map(|net| index.get(net).copied().ok_or(CircuitError::MissingNet))
        .transpose()
}

fn net_pair(
    index: &BTreeMap<&NetId, usize>,
    pos: &Option<NetId>,
    neg: &Option<NetId>,
) -> Result<(Option<usize>, Option<usize>), CircuitError> {
    Ok((net_index(index, pos)?, net_index(index, neg)?))
}

fn stamp_admittance(
    matrix: &mut [Vec<Phasor>],
    pos: Option<usize>,
    neg: Option<usize>,
    admittance: &Phasor,
) {
    if let Some(pos) = pos {
        matrix[pos][pos] = matrix[pos][pos].add(admittance);
    }
    if let Some(neg) = neg {
        matrix[neg][neg] = matrix[neg][neg].add(admittance);
    }
    if let (Some(pos), Some(neg)) = (pos, neg) {
        matrix[pos][neg] = matrix[pos][neg].subtract(admittance);
        matrix[neg][pos] = matrix[neg][pos].subtract(admittance);
    }
}

fn stamp_current(rhs: &mut [Phasor], pos: Option<usize>, neg: Option<usize>, current: &Phasor) {
    if let Some(pos) = pos {
        rhs[pos] = rhs[pos].subtract(current);
    }
    if let Some(neg) = neg {
        rhs[neg] = rhs[neg].add(current);
    }
}

fn stamp_voltage(
    matrix: &mut [Vec<Phasor>],
    rhs: &mut [Phasor],
    branch: usize,
    pos: Option<usize>,
    neg: Option<usize>,
    voltage: &Phasor,
) {
    let one = Phasor::real(Real::one());
    if let Some(pos) = pos {
        matrix[pos][branch] = matrix[pos][branch].add(&one);
        matrix[branch][pos] = matrix[branch][pos].add(&one);
    }
    if let Some(neg) = neg {
        matrix[neg][branch] = matrix[neg][branch].subtract(&one);
        matrix[branch][neg] = matrix[branch][neg].subtract(&one);
    }
    rhs[branch] = rhs[branch].add(voltage);
}

fn stamp_vccs(
    matrix: &mut [Vec<Phasor>],
    pos: Option<usize>,
    neg: Option<usize>,
    ctrl_pos: Option<usize>,
    ctrl_neg: Option<usize>,
    transconductance: &Phasor,
) {
    if let (Some(row), Some(column)) = (pos, ctrl_pos) {
        matrix[row][column] = matrix[row][column].add(transconductance);
    }
    if let (Some(row), Some(column)) = (pos, ctrl_neg) {
        matrix[row][column] = matrix[row][column].subtract(transconductance);
    }
    if let (Some(row), Some(column)) = (neg, ctrl_pos) {
        matrix[row][column] = matrix[row][column].subtract(transconductance);
    }
    if let (Some(row), Some(column)) = (neg, ctrl_neg) {
        matrix[row][column] = matrix[row][column].add(transconductance);
    }
}
