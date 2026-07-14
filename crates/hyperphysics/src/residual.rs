//! `hypersolve` residual replay surfaces for coupled physics reports.
//!
//! This module is the small bridge required before `hyperphysics` trusts
//! numerical engines for coupled mechanics, thermal, EM, circuit, or fluid
//! systems. `hypersolve` owns symbolic residuals and candidate certification;
//! `hyperphysics` records the replay result as exact, certified, unknown, or
//! lossy diagnostics. A numerical candidate is only useful once exact residuals
//! can be evaluated or their uncertainty is explicit.
//!
//! The residual formulation is the same separation used by SPICE/MNA and DAE
//! solvers: assemble equations first, then let a numeric backend propose a
//! state, then replay `F(x)` for certification. In Hyper this replay is routed
//! through `hypersolve::evaluate_residuals`, keeping dense-solver estimates as
//! diagnostics rather than authoritative values.

use hyperreal::{Real, RealSign};
use hypersolve::{Problem, context_from_problem, evaluate_residuals};

use crate::{DiagnosticStatus, PhysicsError, PhysicsResult};

/// One exact residual replay row from a `hypersolve` problem.
#[derive(Clone, Debug, PartialEq)]
pub struct HypersolveResidualRow {
    /// Residual row name.
    pub name: String,
    /// Exact residual value after constraint-kind normalization.
    pub value: Real,
    /// Exact structural sign when known.
    pub sign: Option<RealSign>,
    /// Optional lossy dense-solver estimate retained as a diagnostic only.
    pub dense_solver_estimate: Option<f64>,
    /// Optional lossy weighted estimate retained as a diagnostic only.
    pub weighted_dense_solver_estimate: Option<f64>,
}

/// Exact residual replay report for a `hypersolve` problem.
#[derive(Clone, Debug, PartialEq)]
pub struct HypersolveResidualReplayReport {
    /// Number of variables in the replayed problem.
    pub variable_count: usize,
    /// Active residual rows.
    pub residuals: Vec<HypersolveResidualRow>,
    /// Summary status.
    pub status: DiagnosticStatus,
}

impl HypersolveResidualReplayReport {
    /// Evaluates exact residuals for the current variable values in a `hypersolve` problem.
    pub fn replay(problem: &Problem) -> PhysicsResult<Self> {
        let context = context_from_problem(problem);
        let residuals = evaluate_residuals(problem, &context)
            .map_err(|_| PhysicsError::UnknownDiagnostic)?
            .into_iter()
            .map(|row| HypersolveResidualRow {
                name: row.name,
                value: row.value,
                sign: row.sign,
                dense_solver_estimate: row.dense_solver_estimate,
                weighted_dense_solver_estimate: row.weighted_dense_solver_estimate,
            })
            .collect::<Vec<_>>();
        let status = if residuals.iter().all(|row| row.sign.is_some()) {
            DiagnosticStatus::Exact
        } else {
            DiagnosticStatus::BoundedUnknown
        };
        Ok(Self {
            variable_count: problem.variables.len(),
            residuals,
            status,
        })
    }

    /// Returns true when every replayed residual is exactly zero.
    pub fn all_residuals_zero(&self) -> bool {
        self.residuals
            .iter()
            .all(|row| row.sign == Some(RealSign::Zero))
    }
}
