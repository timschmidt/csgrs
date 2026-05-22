use crate::eval::ResidualEvaluation;
use crate::linalg::LinearSolveReport;

/// Numerical proposal engine requested for candidate generation.
///
/// These engines are not proof systems. They generate candidate coordinates
/// that must be replayed through exact residual and predicate certificates
/// before acceptance. The list follows the practical geometric-constraint
/// solver families exposed by SolveSpace/PlaneGCS and MINPACK: damped
/// least-squares/Levenberg-Marquardt, SolveSpace-style modified Newton least
/// squares, Powell hybrid, dogleg, BFGS, and SQP.
/// Yap's exact-geometric-computation boundary still applies: numerical
/// engines propose, exact/certified replay decides.
///
/// See Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997), for the exact/approximate boundary; Levenberg, "A Method for
/// the Solution of Certain Non-Linear Problems in Least Squares" (1944), and
/// Marquardt, "An Algorithm for Least-Squares Estimation of Nonlinear
/// Parameters" (1963), for the damped least-squares proposal family; and
/// Powell, "A Hybrid Method for Nonlinear Equations" (1970), for dogleg-style
/// trust-region proposals; Broyden/Fletcher/Goldfarb/Shanno (1970) for BFGS;
/// and Nocedal and Wright, *Numerical Optimization*, 2nd ed. (2006), for SQP.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProposalEngineKind {
    /// Current dense damped least-squares implementation.
    DampedLeastSquares,
    /// MINPACK-style Powell hybrid proposal engine.
    ///
    /// The first implementation uses the same dense dogleg trust-region step
    /// as [`Self::Dogleg`], surfaced under the Powell-hybrid name so callers
    /// can audit the requested lossy proposal policy.
    PowellHybrid,
    /// Levenberg-Marquardt proposal route.
    ///
    /// This is implemented by the same dense damped normal-equation step as
    /// [`Self::DampedLeastSquares`], surfaced under the Levenberg-Marquardt
    /// name so callers can audit which lossy proposal policy they requested.
    LevenbergMarquardt,
    /// SolveSpace-style modified Newton least-squares proposal route.
    ///
    /// The first route uses hypersolve's existing symbolic Jacobian,
    /// soluble-alone preprocessing surfaces, fixed-parameter masking, and
    /// damped normal-equation least-squares step under a named report. It is a
    /// compatibility proposal engine, not a convergence or proof certificate.
    ModifiedNewtonLeastSquares,
    /// Trust-region dogleg proposal engine.
    Dogleg,
    /// Quasi-Newton BFGS proposal engine.
    ///
    /// This route retains a dense inverse-Hessian approximation across
    /// iterations. It remains a lossy proposal engine, not an exact Hessian
    /// certificate.
    Bfgs,
    /// Sequential quadratic programming proposal engine.
    ///
    /// The first implementation uses the equality least-squares QP relaxation
    /// already exposed by the dense damped normal-equation adapter.
    Sqp,
}

impl ProposalEngineKind {
    /// Returns whether this engine is implemented in this crate today.
    pub const fn is_implemented(self) -> bool {
        matches!(
            self,
            Self::DampedLeastSquares
                | Self::PowellHybrid
                | Self::LevenbergMarquardt
                | Self::ModifiedNewtonLeastSquares
                | Self::Dogleg
                | Self::Bfgs
                | Self::Sqp
        )
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

/// Exact preprocessing surfaced before a numerical proposal step.
///
/// SolveSpace-style solvers perform substitution and "soluble alone" passes
/// before Newton iteration. In Hyper these passes are exact proposal
/// diagnostics: they may explain or seed a candidate route, but Yap's
/// construction/proof boundary still requires ordinary exact residual replay
/// before any candidate is accepted. See Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProposalPreprocessingReport {
    /// Whether the requested proposal route asked for this preprocessing.
    pub requested: bool,
    /// Number of exact equality-substitution candidates found.
    pub equality_substitutions: usize,
    /// Number of equality-substitution classes applied as initial proposal
    /// seeds.
    pub substitution_seed_classes: usize,
    /// Number of equality-substitution classes rejected before seeding.
    pub rejected_substitution_seed_classes: usize,
    /// Number of exact equality-substitution member assignments written into
    /// the initial proposal state.
    pub substitution_seed_assignments: usize,
    /// Number of active one-variable affine equality rows solved exactly.
    pub affine_soluble_alone_rows: usize,
    /// Number of active univariate quadratic equality rows inspected exactly.
    pub quadratic_soluble_alone_rows: usize,
    /// Number of exact affine soluble-alone assignments applied as initial
    /// proposal seeds.
    pub affine_seed_assignments: usize,
    /// Number of exact affine soluble-alone assignments rejected before
    /// seeding.
    pub rejected_affine_seed_assignments: usize,
    /// Number of unique exact quadratic-root assignments applied as initial
    /// proposal seeds.
    pub quadratic_seed_assignments: usize,
    /// Number of quadratic-root candidates rejected before seeding.
    pub rejected_quadratic_seed_assignments: usize,
    /// Number of dragged-parameter proposal weights accepted for the named
    /// modified-Newton route.
    pub dragged_parameter_weights: usize,
    /// Number of dragged-parameter proposal weights rejected before numerical
    /// iteration.
    pub invalid_dragged_parameter_weights: usize,
    /// Whether exact preprocessing completed without an unsupported branch.
    pub completed: bool,
}

impl ProposalPreprocessingReport {
    /// Return an empty report for proposal engines that do not request this
    /// SolveSpace-style preprocessing route.
    pub const fn not_requested() -> Self {
        Self {
            requested: false,
            equality_substitutions: 0,
            substitution_seed_classes: 0,
            rejected_substitution_seed_classes: 0,
            substitution_seed_assignments: 0,
            affine_soluble_alone_rows: 0,
            quadratic_soluble_alone_rows: 0,
            affine_seed_assignments: 0,
            rejected_affine_seed_assignments: 0,
            quadratic_seed_assignments: 0,
            rejected_quadratic_seed_assignments: 0,
            dragged_parameter_weights: 0,
            invalid_dragged_parameter_weights: 0,
            completed: true,
        }
    }
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
    /// step supports the default damped least-squares route, named
    /// Levenberg-Marquardt and modified-Newton least-squares routes, dense
    /// Powell-hybrid/dogleg routes, BFGS, and an equality-SQP relaxation.
    /// Accepted coordinates still require exact candidate certification; see
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997).
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
    /// Exact proposal preprocessing found before lossy iteration.
    ///
    /// This is currently populated for
    /// [`ProposalEngineKind::ModifiedNewtonLeastSquares`] to expose
    /// SolveSpace-like equality substitution and soluble-alone discovery
    /// without letting those proposal facts bypass exact replay.
    pub preprocessing: ProposalPreprocessingReport,
}
