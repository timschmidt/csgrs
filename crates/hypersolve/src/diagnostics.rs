use crate::eval::ResidualEvaluation;

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
}
