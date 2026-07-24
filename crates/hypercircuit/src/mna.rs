//! Linear Modified Nodal Analysis carriers and residual replay.

use std::collections::BTreeMap;

use hyperreal::{Real, ZeroKnowledge};
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
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
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

/// Exact linear solve candidate together with mandatory residual certification.
#[derive(Clone, Debug, PartialEq)]
pub struct LinearSolveReport {
    /// Candidate values in [`LinearMnaSystem::unknowns`] order.
    pub candidate: Vec<Real>,
    /// Exact replay of `A*x - b` for the returned candidate.
    pub replay: ResidualReplayReport,
}

impl LinearMnaSystem {
    /// Builds a dense exact MNA system from known non-ground nets and stamps.
    pub fn from_stamps(nets: Vec<NetId>, stamps: &[LinearStamp]) -> CircuitResult<Self> {
        let branch_count = stamps
            .iter()
            .filter(|stamp| matches!(stamp, LinearStamp::VoltageSource { .. }))
            .count();
        let mut unknowns = Vec::with_capacity(nets.len() + branch_count);
        unknowns.extend(nets.into_iter().map(MnaUnknown::NetVoltage));
        for stamp in stamps {
            if let LinearStamp::VoltageSource { branch, .. } = stamp {
                unknowns.push(MnaUnknown::BranchCurrent(branch.clone()));
            }
        }

        let mut net_indices = BTreeMap::new();
        let mut branch_indices = BTreeMap::new();
        for (slot, unknown) in unknowns.iter().enumerate() {
            match unknown {
                MnaUnknown::NetVoltage(net) => {
                    if net_indices.insert(net, slot).is_some() {
                        return Err(CircuitError::DuplicateUnknown);
                    }
                }
                MnaUnknown::BranchCurrent(branch) => {
                    if branch_indices.insert(branch, slot).is_some() {
                        return Err(CircuitError::DuplicateUnknown);
                    }
                }
            }
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
                } => {
                    let (pos, neg) = net_pair(&net_indices, pos, neg)?;
                    stamp_conductance(&mut matrix, pos, neg, conductance);
                }
                LinearStamp::CurrentSource {
                    pos, neg, current, ..
                } => {
                    let (pos, neg) = net_pair(&net_indices, pos, neg)?;
                    stamp_current(&mut rhs, pos, neg, current);
                }
                LinearStamp::VoltageSource {
                    branch,
                    pos,
                    neg,
                    voltage,
                    ..
                } => {
                    let (pos, neg) = net_pair(&net_indices, pos, neg)?;
                    let branch = branch_indices
                        .get(branch)
                        .copied()
                        .ok_or(CircuitError::MissingNet)?;
                    stamp_voltage(&mut matrix, &mut rhs, branch, pos, neg, voltage);
                }
                LinearStamp::Vccs {
                    pos,
                    neg,
                    ctrl_pos,
                    ctrl_neg,
                    transconductance,
                    ..
                } => {
                    let (pos, neg) = net_pair(&net_indices, pos, neg)?;
                    let (ctrl_pos, ctrl_neg) = net_pair(&net_indices, ctrl_pos, ctrl_neg)?;
                    stamp_vccs(&mut matrix, pos, neg, ctrl_pos, ctrl_neg, transconductance);
                }
                LinearStamp::Companion {
                    pos,
                    neg,
                    conductance,
                    history_current,
                    ..
                } => {
                    let (pos, neg) = net_pair(&net_indices, pos, neg)?;
                    stamp_conductance(&mut matrix, pos, neg, conductance);
                    stamp_current(&mut rhs, pos, neg, history_current);
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

    /// Solves a square linear MNA system by exact-aware Gauss-Jordan elimination.
    ///
    /// A pivot must be certified nonzero; an unknown zero predicate is reported
    /// rather than resolved by a floating tolerance. The result is replayed
    /// through the original equations before it is returned.
    pub fn solve_exact(&self) -> CircuitResult<LinearSolveReport> {
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
                *entry = (entry.clone() / divisor.clone())
                    .map_err(|_| CircuitError::LinearSolveArithmetic)?;
            }
            rhs[column] =
                (rhs[column].clone() / divisor).map_err(|_| CircuitError::LinearSolveArithmetic)?;
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
                    *entry = entry.clone() - factor.clone() * pivot_entry.clone();
                }
                rhs[row] = rhs[row].clone() - factor * pivot_rhs.clone();
            }
        }
        let replay = self.replay_candidate(&rhs)?;
        if !replay.accepted {
            return Err(CircuitError::UnknownResidual);
        }
        Ok(LinearSolveReport {
            candidate: rhs,
            replay,
        })
    }
}

fn net_index(index: &BTreeMap<&NetId, usize>, net: &Option<NetId>) -> CircuitResult<Option<usize>> {
    net.as_ref()
        .map(|net| index.get(net).copied().ok_or(CircuitError::MissingNet))
        .transpose()
}

fn net_pair(
    index: &BTreeMap<&NetId, usize>,
    pos: &Option<NetId>,
    neg: &Option<NetId>,
) -> CircuitResult<(Option<usize>, Option<usize>)> {
    Ok((net_index(index, pos)?, net_index(index, neg)?))
}

fn stamp_conductance(
    matrix: &mut [Vec<Real>],
    pos: Option<usize>,
    neg: Option<usize>,
    conductance: &Real,
) {
    if let Some(p) = pos {
        matrix[p][p] = matrix[p][p].clone() + conductance;
    }
    if let Some(n) = neg {
        matrix[n][n] = matrix[n][n].clone() + conductance;
    }
    if let (Some(p), Some(n)) = (pos, neg) {
        matrix[p][n] = matrix[p][n].clone() - conductance;
        matrix[n][p] = matrix[n][p].clone() - conductance;
    }
}

fn stamp_current(rhs: &mut [Real], pos: Option<usize>, neg: Option<usize>, current: &Real) {
    if let Some(p) = pos {
        rhs[p] = rhs[p].clone() - current;
    }
    if let Some(n) = neg {
        rhs[n] = rhs[n].clone() + current;
    }
}

fn stamp_voltage(
    matrix: &mut [Vec<Real>],
    rhs: &mut [Real],
    branch: usize,
    pos: Option<usize>,
    neg: Option<usize>,
    voltage: &Real,
) {
    if let Some(p) = pos {
        matrix[p][branch] = matrix[p][branch].clone() + Real::one();
        matrix[branch][p] = matrix[branch][p].clone() + Real::one();
    }
    if let Some(n) = neg {
        matrix[n][branch] = matrix[n][branch].clone() - Real::one();
        matrix[branch][n] = matrix[branch][n].clone() - Real::one();
    }
    rhs[branch] = rhs[branch].clone() + voltage;
}

fn stamp_vccs(
    matrix: &mut [Vec<Real>],
    pos: Option<usize>,
    neg: Option<usize>,
    ctrl_pos: Option<usize>,
    ctrl_neg: Option<usize>,
    transconductance: &Real,
) {
    if let (Some(row), Some(col)) = (pos, ctrl_pos) {
        matrix[row][col] = matrix[row][col].clone() + transconductance;
    }
    if let (Some(row), Some(col)) = (pos, ctrl_neg) {
        matrix[row][col] = matrix[row][col].clone() - transconductance;
    }
    if let (Some(row), Some(col)) = (neg, ctrl_pos) {
        matrix[row][col] = matrix[row][col].clone() - transconductance;
    }
    if let (Some(row), Some(col)) = (neg, ctrl_neg) {
        matrix[row][col] = matrix[row][col].clone() + transconductance;
    }
}
