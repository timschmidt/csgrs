//! Linear Modified Nodal Analysis carriers and residual replay.

use std::collections::BTreeMap;

use hyperreal::Real;
use hypersolve::{DenseResidualReplayError, replay_dense_linear_residuals};

use crate::{BranchId, CircuitError, CircuitResult, ComponentId, NetId, PartRef};

/// Unknown in a linear MNA system.
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum MnaUnknown {
    /// Voltage of a non-ground net.
    NetVoltage(NetId),
    /// Current through an introduced branch, such as a voltage source.
    BranchCurrent(BranchId),
}

/// Linear circuit stamp.
#[derive(Clone, Debug, PartialEq)]
pub enum LinearStamp {
    /// Conductance between two nets. `None` means ground.
    Conductance {
        /// Component id.
        component: ComponentId,
        /// Optional part reference.
        part: Option<PartRef>,
        /// Positive terminal.
        pos: Option<NetId>,
        /// Negative terminal.
        neg: Option<NetId>,
        /// Exact conductance.
        conductance: Real,
    },
    /// Current source from `pos` to `neg`.
    CurrentSource {
        /// Component id.
        component: ComponentId,
        /// Positive terminal.
        pos: Option<NetId>,
        /// Negative terminal.
        neg: Option<NetId>,
        /// Exact source current.
        current: Real,
    },
    /// Ideal voltage source introducing one branch-current unknown.
    VoltageSource {
        /// Component id.
        component: ComponentId,
        /// Branch id.
        branch: BranchId,
        /// Positive terminal.
        pos: Option<NetId>,
        /// Negative terminal.
        neg: Option<NetId>,
        /// Exact source voltage.
        voltage: Real,
    },
    /// Voltage-controlled current source from `pos` to `neg`.
    Vccs {
        /// Component id.
        component: ComponentId,
        /// Positive output terminal.
        pos: Option<NetId>,
        /// Negative output terminal.
        neg: Option<NetId>,
        /// Positive control terminal.
        ctrl_pos: Option<NetId>,
        /// Negative control terminal.
        ctrl_neg: Option<NetId>,
        /// Exact transconductance gain.
        transconductance: Real,
    },
    /// Linear companion conductance plus history current for transient R/C/L setup.
    Companion {
        /// Component id.
        component: ComponentId,
        /// Positive terminal.
        pos: Option<NetId>,
        /// Negative terminal.
        neg: Option<NetId>,
        /// Exact companion conductance.
        conductance: Real,
        /// Exact history/source current from `pos` to `neg`.
        history_current: Real,
    },
}

/// Dense exact linear MNA system.
#[derive(Clone, Debug, PartialEq)]
pub struct LinearMnaSystem {
    /// Unknown ordering.
    pub unknowns: Vec<MnaUnknown>,
    /// Dense matrix `A`.
    pub matrix: Vec<Vec<Real>>,
    /// Right-hand side `b`.
    pub rhs: Vec<Real>,
}

/// Exact residual replay report for a candidate vector.
#[derive(Clone, Debug, PartialEq)]
pub struct ResidualReplayReport {
    /// Residual vector `A*x - b`.
    pub residuals: Vec<Real>,
    /// True when every residual was certified zero.
    pub accepted: bool,
}

impl LinearMnaSystem {
    /// Builds a dense exact MNA system from known non-ground nets and stamps.
    pub fn from_stamps(nets: Vec<NetId>, stamps: &[LinearStamp]) -> CircuitResult<Self> {
        let mut unknowns: Vec<MnaUnknown> =
            nets.iter().cloned().map(MnaUnknown::NetVoltage).collect();
        for stamp in stamps {
            if let LinearStamp::VoltageSource { branch, .. } = stamp {
                unknowns.push(MnaUnknown::BranchCurrent(branch.clone()));
            }
        }

        let mut index = BTreeMap::new();
        for (slot, unknown) in unknowns.iter().enumerate() {
            index.insert(unknown.clone(), slot);
        }

        let n = unknowns.len();
        let mut matrix = vec![vec![Real::zero(); n]; n];
        let mut rhs = vec![Real::zero(); n];

        for stamp in stamps {
            match stamp {
                LinearStamp::Conductance {
                    pos,
                    neg,
                    conductance,
                    ..
                } => stamp_conductance(&mut matrix, &index, pos, neg, conductance)?,
                LinearStamp::CurrentSource {
                    pos, neg, current, ..
                } => stamp_current(&mut rhs, &index, pos, neg, current)?,
                LinearStamp::VoltageSource {
                    branch,
                    pos,
                    neg,
                    voltage,
                    ..
                } => stamp_voltage(&mut matrix, &mut rhs, &index, branch, pos, neg, voltage)?,
                LinearStamp::Vccs {
                    pos,
                    neg,
                    ctrl_pos,
                    ctrl_neg,
                    transconductance,
                    ..
                } => stamp_vccs(
                    &mut matrix,
                    &index,
                    pos,
                    neg,
                    ctrl_pos,
                    ctrl_neg,
                    transconductance,
                )?,
                LinearStamp::Companion {
                    pos,
                    neg,
                    conductance,
                    history_current,
                    ..
                } => {
                    stamp_conductance(&mut matrix, &index, pos, neg, conductance)?;
                    stamp_current(&mut rhs, &index, pos, neg, history_current)?;
                }
            }
        }

        Ok(Self {
            unknowns,
            matrix,
            rhs,
        })
    }

    /// Replays a candidate solution through exact residuals.
    pub fn replay_candidate(&self, candidate: &[Real]) -> CircuitResult<ResidualReplayReport> {
        if candidate.len() != self.unknowns.len() {
            return Err(CircuitError::CandidateLengthMismatch);
        }

        let report = replay_dense_linear_residuals(&self.matrix, &self.rhs, candidate, -64)
            .map_err(|error| match error {
                DenseResidualReplayError::DimensionMismatch => {
                    CircuitError::CandidateLengthMismatch
                }
                DenseResidualReplayError::UnknownResidual => CircuitError::UnknownResidual,
            })?;

        Ok(ResidualReplayReport {
            residuals: report.residuals,
            accepted: report.accepted,
        })
    }
}

fn net_index(
    index: &BTreeMap<MnaUnknown, usize>,
    net: &Option<NetId>,
) -> CircuitResult<Option<usize>> {
    net.as_ref()
        .map(|net| {
            index
                .get(&MnaUnknown::NetVoltage(net.clone()))
                .copied()
                .ok_or(CircuitError::MissingNet)
        })
        .transpose()
}

fn stamp_conductance(
    matrix: &mut [Vec<Real>],
    index: &BTreeMap<MnaUnknown, usize>,
    pos: &Option<NetId>,
    neg: &Option<NetId>,
    conductance: &Real,
) -> CircuitResult<()> {
    let p = net_index(index, pos)?;
    let n = net_index(index, neg)?;
    if let Some(p) = p {
        matrix[p][p] = matrix[p][p].clone() + conductance;
    }
    if let Some(n) = n {
        matrix[n][n] = matrix[n][n].clone() + conductance;
    }
    if let (Some(p), Some(n)) = (p, n) {
        matrix[p][n] = matrix[p][n].clone() - conductance;
        matrix[n][p] = matrix[n][p].clone() - conductance;
    }
    Ok(())
}

fn stamp_current(
    rhs: &mut [Real],
    index: &BTreeMap<MnaUnknown, usize>,
    pos: &Option<NetId>,
    neg: &Option<NetId>,
    current: &Real,
) -> CircuitResult<()> {
    if let Some(p) = net_index(index, pos)? {
        rhs[p] = rhs[p].clone() - current;
    }
    if let Some(n) = net_index(index, neg)? {
        rhs[n] = rhs[n].clone() + current;
    }
    Ok(())
}

fn stamp_voltage(
    matrix: &mut [Vec<Real>],
    rhs: &mut [Real],
    index: &BTreeMap<MnaUnknown, usize>,
    branch: &BranchId,
    pos: &Option<NetId>,
    neg: &Option<NetId>,
    voltage: &Real,
) -> CircuitResult<()> {
    let branch_index = index
        .get(&MnaUnknown::BranchCurrent(branch.clone()))
        .copied()
        .ok_or(CircuitError::MissingNet)?;
    if let Some(p) = net_index(index, pos)? {
        matrix[p][branch_index] = matrix[p][branch_index].clone() + Real::one();
        matrix[branch_index][p] = matrix[branch_index][p].clone() + Real::one();
    }
    if let Some(n) = net_index(index, neg)? {
        matrix[n][branch_index] = matrix[n][branch_index].clone() - Real::one();
        matrix[branch_index][n] = matrix[branch_index][n].clone() - Real::one();
    }
    rhs[branch_index] = rhs[branch_index].clone() + voltage;
    Ok(())
}

fn stamp_vccs(
    matrix: &mut [Vec<Real>],
    index: &BTreeMap<MnaUnknown, usize>,
    pos: &Option<NetId>,
    neg: &Option<NetId>,
    ctrl_pos: &Option<NetId>,
    ctrl_neg: &Option<NetId>,
    transconductance: &Real,
) -> CircuitResult<()> {
    let p = net_index(index, pos)?;
    let n = net_index(index, neg)?;
    let cp = net_index(index, ctrl_pos)?;
    let cn = net_index(index, ctrl_neg)?;

    if let (Some(row), Some(col)) = (p, cp) {
        matrix[row][col] = matrix[row][col].clone() + transconductance;
    }
    if let (Some(row), Some(col)) = (p, cn) {
        matrix[row][col] = matrix[row][col].clone() - transconductance;
    }
    if let (Some(row), Some(col)) = (n, cp) {
        matrix[row][col] = matrix[row][col].clone() - transconductance;
    }
    if let (Some(row), Some(col)) = (n, cn) {
        matrix[row][col] = matrix[row][col].clone() + transconductance;
    }
    Ok(())
}
