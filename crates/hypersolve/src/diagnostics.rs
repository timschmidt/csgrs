use crate::eval::ResidualEvaluation;
use crate::linalg::LinearSolveReport;

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ConvergenceReason {
    Converged,
    StepTooSmall,
    MaxIterations,
    LinearSolveFailed,
    EvaluationFailed,
}

#[derive(Clone, Debug)]
pub struct SolveReport {
    pub reason: ConvergenceReason,
    pub iterations: usize,
    pub residuals: Vec<ResidualEvaluation>,
    /// Linear-solver adapter reports collected during iteration.
    ///
    /// These reports are deliberately surfaced at the solver boundary so lossy
    /// dense f64 iteration remains visible to callers. They are diagnostics for
    /// selected numerical adapters, not exact certificates for residual
    /// feasibility or symbolic rank. This matches Yap's separation between
    /// exact object facts and approximate numerical stages; see Yap, "Towards
    /// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    pub linear_reports: Vec<LinearSolveReport>,
}
