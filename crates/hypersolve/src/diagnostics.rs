use crate::eval::ResidualEvaluation;
use crate::linalg::LinearSolveReport;

/// Numerical proposal engine requested for candidate generation.
///
/// These engines are not proof systems. They generate candidate coordinates
/// that must be replayed through exact residual and predicate certificates
/// before acceptance. The list follows the practical geometric-constraint
/// solver families exposed by SolveSpace/PlaneGCS and MINPACK: damped
/// least-squares/Levenberg-Marquardt, Powell hybrid, dogleg, BFGS, and SQP.
/// Yap's exact-geometric-computation boundary still applies: numerical
/// engines propose, exact/certified replay decides.
///
/// See Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997), for the exact/approximate boundary; Levenberg, "A Method for
/// the Solution of Certain Non-Linear Problems in Least Squares" (1944), and
/// Marquardt, "An Algorithm for Least-Squares Estimation of Nonlinear
/// Parameters" (1963), for the damped least-squares proposal family.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProposalEngineKind {
    /// Current dense damped least-squares implementation.
    DampedLeastSquares,
    /// MINPACK-style Powell hybrid proposal engine.
    PowellHybrid,
    /// Levenberg-Marquardt proposal route.
    ///
    /// This is implemented by the same dense damped normal-equation step as
    /// [`Self::DampedLeastSquares`], surfaced under the Levenberg-Marquardt
    /// name so callers can audit which lossy proposal policy they requested.
    LevenbergMarquardt,
    /// Trust-region dogleg proposal engine.
    Dogleg,
    /// Quasi-Newton BFGS proposal engine.
    Bfgs,
    /// Sequential quadratic programming proposal engine.
    Sqp,
}

impl ProposalEngineKind {
    /// Returns whether this engine is implemented in this crate today.
    pub const fn is_implemented(self) -> bool {
        matches!(self, Self::DampedLeastSquares | Self::LevenbergMarquardt)
    }
}

/// Precision boundary crossed by a proposal engine.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProposalEnginePrecision {
    /// The proposal engine crosses a primitive-float boundary.
    LossyF64,
    /// The requested proposal engine is declared but not implemented.
    Unsupported,
}

/// Report describing the proposal engine selected for a solve attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProposalEngineReport {
    /// Engine requested by the caller.
    pub requested: ProposalEngineKind,
    /// Engine actually used, if any.
    pub used: Option<ProposalEngineKind>,
    /// Precision boundary for the selected route.
    pub precision: ProposalEnginePrecision,
    /// Whether this proposal route is implemented.
    pub supported: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ConvergenceReason {
    Converged,
    StepTooSmall,
    MaxIterations,
    LinearSolveFailed,
    EvaluationFailed,
    UnsupportedProposalEngine,
}

#[derive(Clone, Debug)]
pub struct SolveReport {
    pub reason: ConvergenceReason,
    pub iterations: usize,
    /// Proposal engine selected for candidate generation.
    ///
    /// This makes the construction/proof boundary explicit. The current dense
    /// step supports the default damped least-squares route and the named
    /// Levenberg-Marquardt route; other common geometric-constraint engines
    /// are represented as unsupported proposal choices rather than silently
    /// falling back. Accepted coordinates still require exact candidate
    /// certification; see Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub proposal_engine: ProposalEngineReport,
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
