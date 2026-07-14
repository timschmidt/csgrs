//! Direct exact reductions for simple solver rows.
//!
//! SolveSpace performs substitution and "soluble alone" passes before Newton
//! iteration. This module adds the exact-stack version for prepared affine
//! equality rows with one active variable and square affine equality systems.
//! The result is a candidate assignment produced by exact `hyperreal::Real`
//! arithmetic; callers still replay and certify the full problem before
//! trusting it. This keeps the optimization in the exact expression/object layer
//! instead of making a lossy nonlinear backend responsible for obvious exact
//! algebra.

use hyperreal::{Real, RealSign};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

use crate::certification::{
    CandidateCertificationConfig, CandidateCertificationReport, certify_candidate_with_config,
};
use crate::eval::EvaluationContext;
use crate::model::ConstraintKind;
use crate::prepared::PreparedProblem;
use crate::symbolic::SymbolId;

/// Exact direct solution for one affine equality row.
#[derive(Clone, Debug, PartialEq)]
pub struct DirectAffineSolution {
    /// Source constraint index that produced the solution.
    pub constraint_index: usize,
    /// Solver symbol solved by this row.
    pub symbol: SymbolId,
    /// Exact candidate value.
    pub value: Real,
}

/// Status for exact direct solving of a square affine equality system.
///
/// This is the report-bearing version of SolveSpace's pre-Newton affine
/// reduction for the Hyper stack. It solves only active equality systems whose
/// rows are already prepared affine blocks, using exact Gaussian elimination
/// over [`Real`]. The output remains a candidate assignment: the exact
/// construction/proof boundary still requires ordinary residual replay before
/// accepting it. SolveSpace's documented
/// symbolic/direct solving pipeline.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DirectAffineSystemStatus {
    /// The square affine system was solved exactly.
    Solved,
    /// The active equality row count did not match the variable count.
    ShapeMismatch {
        /// Number of solver variables.
        variables: usize,
        /// Number of active equality rows considered.
        equality_rows: usize,
    },
    /// An active row was not an equality row.
    NonEqualityRow {
        /// Source constraint index.
        constraint_index: usize,
    },
    /// An active equality row did not have a prepared affine block.
    NonAffineRow {
        /// Source constraint index.
        constraint_index: usize,
    },
    /// No certified nonzero pivot could be selected.
    SingularOrUnsupportedPivot {
        /// Pivot column where elimination failed.
        pivot: usize,
    },
    /// The exact pivot division failed.
    UnsupportedDivision {
        /// Pivot column whose division failed.
        pivot: usize,
    },
}

/// Exact value assigned to one variable by direct affine system solving.
#[derive(Clone, Debug, PartialEq)]
pub struct DirectAffineSystemAssignment {
    /// Solver symbol.
    pub symbol: SymbolId,
    /// Exact solved value.
    pub value: Real,
}

/// Report for direct exact solving of a square affine equality system.
#[derive(Clone, Debug, PartialEq)]
pub struct DirectAffineSystemReport {
    /// Final solve status.
    pub status: DirectAffineSystemStatus,
    /// Number of variables in the source problem.
    pub variable_count: usize,
    /// Number of active equality rows considered.
    pub equality_rows: usize,
    /// Source constraint indices used as affine rows.
    pub constraint_indices: Vec<usize>,
    /// Exact assignments when the system solved.
    pub assignments: Vec<DirectAffineSystemAssignment>,
}

impl DirectAffineSystemReport {
    /// Returns true when this report carries a complete exact assignment.
    pub fn solved(&self) -> bool {
        self.status == DirectAffineSystemStatus::Solved
    }
}

/// Exact root candidates for one prepared univariate quadratic equality row.
#[derive(Clone, Debug, PartialEq)]
pub struct DirectQuadraticSolution {
    /// Source constraint index that produced these roots.
    pub constraint_index: usize,
    /// Solver symbol solved by this row.
    pub symbol: SymbolId,
    /// Exact root candidates. Empty means the discriminant is certified negative.
    pub roots: Vec<Real>,
}

/// Replay status for one direct univariate quadratic root candidate.
///
/// Direct quadratic solving is still proposal machinery. The exact
/// computation model requires the candidate object to be replayed through the
/// full residual set before it becomes a decision; see the exact-geometric-computation model.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DirectQuadraticCandidateStatus {
    /// The source quadratic row has no real roots.
    NoRealRoots,
    /// The root candidate satisfied every active row under exact replay.
    ReplayCertified,
    /// Exact replay ran, but at least one active row was not certified
    /// satisfied.
    ReplayRejected,
}

/// Certification report for one exact direct quadratic root candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct DirectQuadraticCandidateReport {
    /// Source constraint index that produced the root candidate.
    pub constraint_index: usize,
    /// Solver symbol solved by the source row.
    pub symbol: SymbolId,
    /// Root ordinal within the source row, if a root exists.
    pub root_index: Option<usize>,
    /// Exact root value, if a root exists.
    pub root: Option<Real>,
    /// Full exact replay report for the candidate, if a root exists.
    pub certification: Option<CandidateCertificationReport>,
    /// Replay status for this candidate.
    pub status: DirectQuadraticCandidateStatus,
}

/// Exact equality substitution `left = right + offset`.
///
/// This is the first union-find-friendly payload for SolveSpace-style
/// substitution passes. It deliberately records the source row and exact offset
/// instead of mutating the problem immediately; callers can choose whether to
/// use it for simplification, diagnostics, or candidate initialization before
/// replaying the full problem.
#[derive(Clone, Debug, PartialEq)]
pub struct EqualitySubstitution {
    /// Source constraint index.
    pub constraint_index: usize,
    /// Symbol being substituted.
    pub left: SymbolId,
    /// Reference symbol.
    pub right: SymbolId,
    /// Exact offset such that `left = right + offset`.
    pub offset: Real,
}

/// Cycle found in equality-substitution candidates.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EqualitySubstitutionCycle {
    /// Symbols participating in the cycle in discovery order.
    pub symbols: Vec<SymbolId>,
}

/// Exact consistency diagnostics for equality substitution candidates.
#[derive(Clone, Debug, PartialEq)]
pub struct EqualitySubstitutionValidation {
    /// Problems found while validating the directed substitution graph.
    pub problems: Vec<EqualitySubstitutionProblem>,
}

impl EqualitySubstitutionValidation {
    /// Return whether the substitution graph can be used as an acyclic rewrite.
    pub fn is_acyclic_rewrite_graph(&self) -> bool {
        self.problems.is_empty()
    }

    /// Return whether any reported problem proves algebraic inconsistency.
    pub fn has_inconsistency(&self) -> bool {
        self.problems
            .iter()
            .any(EqualitySubstitutionProblem::is_inconsistent)
    }
}

/// One member of an exact equality-substitution class.
#[derive(Clone, Debug, PartialEq)]
pub struct EqualitySubstitutionClassMember {
    /// Symbol in this connected equality component.
    pub symbol: SymbolId,
    /// Exact offset from the representative: `symbol = representative + offset`.
    pub offset_from_representative: Real,
}

/// Exact connected component induced by equality substitutions.
///
/// This is the union-find payload without committing to an in-place rewrite of
/// the model. Components preserve exact offsets to a stable representative, so
/// later affine elimination can rewrite symbols without re-solving the same
/// small graph. The construction follows the same pre-Newton symbolic
/// reduction boundary as SolveSpace, while keeping the exact replay boundary
/// intact.
#[derive(Clone, Debug, PartialEq)]
pub struct EqualitySubstitutionClass {
    /// Stable class representative, chosen as the smallest symbol id.
    pub representative: SymbolId,
    /// Members sorted by symbol id.
    pub members: Vec<EqualitySubstitutionClassMember>,
}

/// Status for applying one equality-substitution class to a candidate context.
///
/// Applying a class is a construction step, not a proof step: it propagates
/// exact representative offsets into an [`EvaluationContext`] so the ordinary
/// residual replay can check the candidate. This is the same boundary the exactness boundary
/// draws between exact object construction and certified geometric decisions;
/// see the exact-geometric-computation model.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum EqualitySubstitutionClassApplicationStatus {
    /// At least one bound member anchored the class and every bound member was
    /// exactly consistent with that anchor.
    Applied {
        /// Bound symbol used to reconstruct the representative value.
        anchor_symbol: SymbolId,
    },
    /// No member in the class was bound in the candidate context.
    MissingBoundMember,
    /// Two already-bound members disagree exactly after carrying their
    /// representative offsets.
    InconsistentBoundMember {
        /// Bound member that contradicted the first anchor.
        symbol: SymbolId,
        /// Exact value implied by the class anchor.
        expected: Real,
        /// Exact value already present in the context.
        actual: Real,
    },
}

/// Per-class result from applying equality-substitution classes.
#[derive(Clone, Debug, PartialEq)]
pub struct EqualitySubstitutionClassApplicationRow {
    /// Stable class representative.
    pub representative: SymbolId,
    /// Application status for this class.
    pub status: EqualitySubstitutionClassApplicationStatus,
    /// Symbols written into the context when the class was applied.
    pub applied_symbols: Vec<SymbolId>,
}

/// Report for applying exact equality-substitution classes to a candidate.
///
/// The report is intentionally explicit about skipped and inconsistent
/// classes because substitution classes are proposal machinery. A caller that
/// accepts a rewritten candidate without the later exact residual report would
/// violate this crate's exact-computation discipline.
#[derive(Clone, Debug, PartialEq)]
pub struct EqualitySubstitutionClassApplicationReport {
    /// Per-class application rows.
    pub rows: Vec<EqualitySubstitutionClassApplicationRow>,
    /// Number of symbol bindings written into the context.
    pub applied_bindings: usize,
    /// Number of classes skipped because no member was bound.
    pub skipped_classes: usize,
    /// Number of classes rejected because existing bindings conflict exactly.
    pub inconsistent_classes: usize,
}

impl EqualitySubstitutionClassApplicationReport {
    /// Returns true when no class reported an exact binding conflict.
    pub fn all_consistent(&self) -> bool {
        self.inconsistent_classes == 0
    }
}

/// One affine row rewritten through equality-substitution classes.
///
/// The row remains a proposal artifact: it is useful for future affine
/// elimination and reduced dense blocks, but it does not replace exact
/// residual replay. Coefficients are keyed by solver symbol after substituting
/// every class member as `symbol = representative + offset`. This follows
/// SolveSpace's pre-Newton substitution pattern while preserving the exact
/// construction-versus-certification boundary.
#[derive(Clone, Debug, PartialEq)]
pub struct EliminatedAffineRow {
    /// Source constraint index.
    pub constraint_index: usize,
    /// Exact constant after carrying substitution offsets.
    pub constant: Real,
    /// Exact coefficients keyed by reduced symbol.
    pub coefficients: Vec<(SymbolId, Real)>,
}

/// Exact report for applying equality-substitution classes to affine rows.
///
/// This is deliberately non-mutating. It records which active affine rows can
/// be represented in class representatives and exact offset-carrying
/// constants, leaving problem mutation and candidate certification to later
/// stages. Silent primitive-float elimination would violate the exact replay
/// discipline described by the exact-geometric-computation model.
#[derive(Clone, Debug, PartialEq)]
pub struct EqualitySubstitutionEliminationReport {
    /// Number of active affine rows considered.
    pub affine_rows_considered: usize,
    /// Rewritten rows that still contain at least one reduced variable.
    pub reduced_variable_rows: usize,
    /// Rows reduced to an exact zero constant.
    pub reduced_zero_rows: usize,
    /// Rows reduced to an exact nonzero constant contradiction.
    pub reduced_contradiction_rows: usize,
    /// Rows reduced to a constant whose exact sign is not structurally known.
    pub reduced_unknown_constant_rows: usize,
    /// Rewritten active affine rows.
    pub rows: Vec<EliminatedAffineRow>,
}

/// One exact issue found in a substitution candidate set.
#[derive(Clone, Debug, PartialEq)]
pub enum EqualitySubstitutionProblem {
    /// A row rewrites a symbol to itself.
    ///
    /// This is a harmless tautology only when `offset == 0`; otherwise it
    /// proves `0 == offset` and the candidate set is inconsistent.
    SelfSubstitution {
        /// Source constraint index.
        constraint_index: usize,
        /// Rewritten symbol.
        symbol: SymbolId,
        /// Exact offset in `symbol = symbol + offset`.
        offset: Real,
        /// Whether the self-substitution is exactly consistent.
        consistent: bool,
    },
    /// Two rows try to rewrite the same left-hand symbol.
    DuplicateLeft {
        /// Rewritten symbol.
        left: SymbolId,
        /// First source constraint index.
        first_constraint_index: usize,
        /// Second source constraint index.
        second_constraint_index: usize,
        /// Whether both rows have exactly the same right-hand symbol and offset.
        same_rewrite: bool,
    },
    /// A directed cycle appears in the substitution graph.
    ///
    /// For a cycle `x0 = x1 + a0`, `x1 = x2 + a1`, ..., `xn = x0 + an`,
    /// consistency requires `sum(ai) == 0`. Even a consistent cycle is not an
    /// acyclic rewrite graph, so callers should choose a representative with a
    /// union-find pass before applying substitutions.
    DirectedCycle {
        /// Symbols participating in the cycle in discovery order.
        symbols: Vec<SymbolId>,
        /// Exact sum of offsets around the cycle.
        net_offset: Real,
        /// Whether the cycle is exactly consistent.
        consistent: bool,
    },
}

impl EqualitySubstitutionProblem {
    /// Return whether this problem is an exact algebraic contradiction.
    pub fn is_inconsistent(&self) -> bool {
        match self {
            Self::SelfSubstitution { consistent, .. } | Self::DirectedCycle { consistent, .. } => {
                !consistent
            }
            Self::DuplicateLeft { same_rewrite, .. } => !same_rewrite,
        }
    }
}

/// Failure modes for direct affine reduction.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DirectSolveError {
    /// The row coefficient was structurally unknown, so selecting the variable
    /// would require an unproved branch.
    UnknownCoefficientSign {
        /// Source constraint index.
        constraint_index: usize,
        /// Variable column.
        variable_column: usize,
    },
    /// The exact division needed to isolate a variable failed.
    UnsupportedDivision {
        /// Source constraint index.
        constraint_index: usize,
    },
    /// The exact square root needed for a quadratic root was unsupported.
    UnsupportedSquareRoot {
        /// Source constraint index.
        constraint_index: usize,
    },
}

/// Solve a square active affine equality system exactly.
///
/// This helper is intentionally stricter than the lossy dense solver: every
/// active constraint must be an equality with a prepared affine residual, and
/// the active equality row count must equal the variable count. It constructs
/// `A*x = -c` directly from retained affine coefficients and solves with
/// certified nonzero pivots. The returned assignments are proposal values that
/// should be replayed through [`crate::certify_candidate`] before trust.
pub fn solve_direct_affine_system(prepared: &PreparedProblem<'_>) -> DirectAffineSystemReport {
    let variable_count = prepared.problem().variables.len();
    let mut matrix = Vec::new();
    let mut rhs = Vec::new();
    let mut constraint_indices = Vec::new();

    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active {
            continue;
        }
        if constraint.kind != ConstraintKind::Equality {
            return direct_affine_system_report(
                DirectAffineSystemStatus::NonEqualityRow { constraint_index },
                variable_count,
                matrix.len(),
                constraint_indices,
                Vec::new(),
            );
        }
        let Some(affine) = &prepared.affine_residuals()[constraint_index] else {
            return direct_affine_system_report(
                DirectAffineSystemStatus::NonAffineRow { constraint_index },
                variable_count,
                matrix.len(),
                constraint_indices,
                Vec::new(),
            );
        };
        matrix.push(affine.coefficients().to_vec());
        rhs.push(-affine.constant().clone());
        constraint_indices.push(constraint_index);
    }

    if matrix.len() != variable_count {
        return direct_affine_system_report(
            DirectAffineSystemStatus::ShapeMismatch {
                variables: variable_count,
                equality_rows: matrix.len(),
            },
            variable_count,
            matrix.len(),
            constraint_indices,
            Vec::new(),
        );
    }

    let solution = match solve_exact_affine_matrix(matrix, rhs) {
        Ok(solution) => solution,
        Err(status) => {
            return direct_affine_system_report(
                status,
                variable_count,
                variable_count,
                constraint_indices,
                Vec::new(),
            );
        }
    };
    let assignments = prepared
        .problem()
        .variables
        .iter()
        .zip(solution)
        .map(|(variable, value)| DirectAffineSystemAssignment {
            symbol: variable.symbol,
            value,
        })
        .collect();
    direct_affine_system_report(
        DirectAffineSystemStatus::Solved,
        variable_count,
        variable_count,
        constraint_indices,
        assignments,
    )
}

/// Solve every active one-variable affine equality row exactly.
///
/// This is intentionally conservative. Rows with multiple structurally
/// nonzero coefficients, non-equality kinds, unknown coefficient signs, or
/// unsupported exact division are not silently approximated.
pub fn solve_direct_affine_equalities(
    prepared: &PreparedProblem<'_>,
) -> Result<Vec<DirectAffineSolution>, DirectSolveError> {
    let mut solutions = Vec::new();
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        let Some(affine) = &prepared.affine_residuals()[constraint_index] else {
            continue;
        };
        let mut nonzero = None;
        for (column, coefficient) in affine.coefficients().iter().enumerate() {
            match coefficient.structural_facts().sign {
                Some(RealSign::Zero) => {}
                Some(RealSign::Negative | RealSign::Positive) => {
                    if nonzero.replace((column, coefficient)).is_some() {
                        nonzero = None;
                        break;
                    }
                }
                None => {
                    return Err(DirectSolveError::UnknownCoefficientSign {
                        constraint_index,
                        variable_column: column,
                    });
                }
            }
        }
        let Some((column, coefficient)) = nonzero else {
            continue;
        };
        let value = (-affine.constant().clone() / coefficient.clone())
            .map_err(|_| DirectSolveError::UnsupportedDivision { constraint_index })?;
        solutions.push(DirectAffineSolution {
            constraint_index,
            symbol: prepared.problem().variables[column].symbol,
            value,
        });
    }
    Ok(solutions)
}

fn solve_exact_affine_matrix(
    mut matrix: Vec<Vec<Real>>,
    mut rhs: Vec<Real>,
) -> Result<Vec<Real>, DirectAffineSystemStatus> {
    let n = rhs.len();
    for pivot in 0..n {
        let pivot_row = (pivot..n).find(|&row| {
            matches!(
                matrix[row][pivot].structural_facts().sign,
                Some(RealSign::Negative | RealSign::Positive)
            )
        });
        let Some(pivot_row) = pivot_row else {
            return Err(DirectAffineSystemStatus::SingularOrUnsupportedPivot { pivot });
        };
        if pivot_row != pivot {
            matrix.swap(pivot_row, pivot);
            rhs.swap(pivot_row, pivot);
        }

        let pivot_value = matrix[pivot][pivot].clone();
        for value in matrix[pivot].iter_mut().skip(pivot) {
            *value = (value.clone() / pivot_value.clone())
                .map_err(|_| DirectAffineSystemStatus::UnsupportedDivision { pivot })?;
        }
        rhs[pivot] = (rhs[pivot].clone() / pivot_value)
            .map_err(|_| DirectAffineSystemStatus::UnsupportedDivision { pivot })?;
        let pivot_tail = matrix[pivot][pivot..].to_vec();
        let pivot_rhs = rhs[pivot].clone();

        for row in 0..n {
            if row == pivot {
                continue;
            }
            let factor = matrix[row][pivot].clone();
            if factor.structural_facts().sign == Some(RealSign::Zero) {
                continue;
            }
            for (value, pivot_value) in matrix[row].iter_mut().skip(pivot).zip(&pivot_tail) {
                *value = value.clone() - factor.clone() * pivot_value.clone();
            }
            rhs[row] = rhs[row].clone() - factor * pivot_rhs.clone();
        }
    }
    Ok(rhs)
}

fn direct_affine_system_report(
    status: DirectAffineSystemStatus,
    variable_count: usize,
    equality_rows: usize,
    constraint_indices: Vec<usize>,
    assignments: Vec<DirectAffineSystemAssignment>,
) -> DirectAffineSystemReport {
    DirectAffineSystemReport {
        status,
        variable_count,
        equality_rows,
        constraint_indices,
        assignments,
    }
}

/// Solve prepared univariate quadratic equality rows exactly when possible.
///
/// This is a proposal-stage analogue of SolveSpace's soluble-alone pass for a
/// bounded nonlinear row. Roots from `a*x^2 + b*x + c = 0` are returned only
/// after exact discriminant construction, exact square-root construction, and
/// exact division. Callers must still replay the full problem with ordinary
/// certification before trusting a root candidate.
pub fn solve_direct_univariate_quadratic_equalities(
    prepared: &PreparedProblem<'_>,
) -> Result<Vec<DirectQuadraticSolution>, DirectSolveError> {
    let mut solutions = Vec::new();
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        let Some(quadratic) = &prepared.univariate_quadratic_residuals()[constraint_index] else {
            continue;
        };
        match quadratic.quadratic().structural_facts().sign {
            Some(RealSign::Zero) => continue,
            Some(RealSign::Negative | RealSign::Positive) => {}
            None => {
                return Err(DirectSolveError::UnknownCoefficientSign {
                    constraint_index,
                    variable_column: prepared
                        .problem()
                        .variables
                        .iter()
                        .position(|variable| variable.symbol == quadratic.symbol())
                        .unwrap_or(0),
                });
            }
        }

        let four = Real::from(4);
        let two = Real::from(2);
        let discriminant = quadratic.linear().clone() * quadratic.linear().clone()
            - four * quadratic.quadratic().clone() * quadratic.constant().clone();
        let roots = match discriminant.structural_facts().sign {
            Some(RealSign::Negative) => Vec::new(),
            Some(RealSign::Zero) => {
                let denominator = two * quadratic.quadratic().clone();
                vec![
                    (-quadratic.linear().clone() / denominator)
                        .map_err(|_| DirectSolveError::UnsupportedDivision { constraint_index })?,
                ]
            }
            Some(RealSign::Positive) | None => {
                let sqrt = discriminant
                    .sqrt()
                    .map_err(|_| DirectSolveError::UnsupportedSquareRoot { constraint_index })?;
                let denominator = two * quadratic.quadratic().clone();
                let first = (((-quadratic.linear().clone()) + sqrt.clone()) / denominator.clone())
                    .map_err(|_| DirectSolveError::UnsupportedDivision { constraint_index })?;
                let second = (((-quadratic.linear().clone()) - sqrt) / denominator)
                    .map_err(|_| DirectSolveError::UnsupportedDivision { constraint_index })?;
                vec![first, second]
            }
        };
        solutions.push(DirectQuadraticSolution {
            constraint_index,
            symbol: quadratic.symbol(),
            roots,
        });
    }
    Ok(solutions)
}

/// Solve direct univariate quadratic rows and certify each root candidate.
///
/// This is a convenience bridge between the exact soluble-alone pass and the
/// ordinary candidate certification boundary. Each root from
/// [`solve_direct_univariate_quadratic_equalities`] is bound into a cloned
/// [`EvaluationContext`] and replayed against the full prepared problem. The
/// helper follows SolveSpace's direct-solve-before-Newton pattern while making
/// the exact construction/proof split explicit: exact roots are still only
/// proposals until residual replay certifies them.
pub fn certify_direct_univariate_quadratic_roots(
    prepared: &PreparedProblem<'_>,
    base_context: &EvaluationContext,
) -> Result<Vec<DirectQuadraticCandidateReport>, DirectSolveError> {
    certify_direct_univariate_quadratic_roots_with_config(
        prepared,
        base_context,
        CandidateCertificationConfig::default(),
    )
}

/// Solve direct univariate quadratic rows and certify each root with a policy.
///
/// The bounded sign-refinement policy is passed through to
/// [`crate::certify_candidate_with_config`]. No primitive-float acceptance
/// threshold is introduced here; unresolved replay remains an explicit
/// rejected/uncertain certification report.
pub fn certify_direct_univariate_quadratic_roots_with_config(
    prepared: &PreparedProblem<'_>,
    base_context: &EvaluationContext,
    config: CandidateCertificationConfig,
) -> Result<Vec<DirectQuadraticCandidateReport>, DirectSolveError> {
    let solutions = solve_direct_univariate_quadratic_equalities(prepared)?;
    let mut reports = Vec::new();
    for solution in solutions {
        if solution.roots.is_empty() {
            reports.push(DirectQuadraticCandidateReport {
                constraint_index: solution.constraint_index,
                symbol: solution.symbol,
                root_index: None,
                root: None,
                certification: None,
                status: DirectQuadraticCandidateStatus::NoRealRoots,
            });
            continue;
        }

        for (root_index, root) in solution.roots.into_iter().enumerate() {
            let mut candidate = base_context.clone();
            candidate.bind(solution.symbol, root.clone());
            let certification = certify_candidate_with_config(prepared, &candidate, config);
            let status = if certification.all_satisfied() {
                DirectQuadraticCandidateStatus::ReplayCertified
            } else {
                DirectQuadraticCandidateStatus::ReplayRejected
            };
            reports.push(DirectQuadraticCandidateReport {
                constraint_index: solution.constraint_index,
                symbol: solution.symbol,
                root_index: Some(root_index),
                root: Some(root),
                certification: Some(certification),
                status,
            });
        }
    }
    Ok(reports)
}

/// Find exact two-variable equality substitutions from prepared affine rows.
///
/// Rows of the form `x - y + c = 0` become `x = y - c`. The implementation is
/// intentionally narrow: it accepts only structurally signed unit coefficients
/// and exact equality rows, so this pass cannot silently turn approximate
/// algebra into solver topology.
pub fn find_equality_substitutions(
    prepared: &PreparedProblem<'_>,
) -> Result<Vec<EqualitySubstitution>, DirectSolveError> {
    let mut substitutions = Vec::new();
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        let Some(affine) = &prepared.affine_residuals()[constraint_index] else {
            continue;
        };
        let mut positive = None;
        let mut negative = None;
        let mut unsupported = false;
        for (column, coefficient) in affine.coefficients().iter().enumerate() {
            match coefficient.structural_facts().sign {
                Some(RealSign::Zero) => {}
                Some(RealSign::Positive) if *coefficient == Real::one() => {
                    unsupported |= positive.replace(column).is_some();
                }
                Some(RealSign::Negative) if *coefficient == -Real::one() => {
                    unsupported |= negative.replace(column).is_some();
                }
                Some(RealSign::Positive | RealSign::Negative) => {
                    unsupported = true;
                }
                None => {
                    return Err(DirectSolveError::UnknownCoefficientSign {
                        constraint_index,
                        variable_column: column,
                    });
                }
            }
        }
        if unsupported {
            continue;
        }
        let (Some(left_column), Some(right_column)) = (positive, negative) else {
            continue;
        };
        substitutions.push(EqualitySubstitution {
            constraint_index,
            left: prepared.problem().variables[left_column].symbol,
            right: prepared.problem().variables[right_column].symbol,
            offset: -affine.constant().clone(),
        });
    }
    Ok(substitutions)
}

/// Apply exact equality substitutions to a candidate context.
///
/// This is a candidate-update helper, not a proof. It is useful after
/// `find_equality_substitutions` has identified `x = y + c` rows and the
/// caller wants a candidate that satisfies those rows before exact replay. The
/// full residual certification step remains mandatory, preserving the exact
/// distinction between construction/proposal and certified decision.
pub fn apply_equality_substitutions(
    context: &mut EvaluationContext,
    substitutions: &[EqualitySubstitution],
) -> Result<usize, DirectSolveError> {
    let mut applied = 0;
    for substitution in substitutions {
        let Some(right_value) = context.bindings().get(&substitution.right).cloned() else {
            continue;
        };
        context.bind(substitution.left, right_value + substitution.offset.clone());
        applied += 1;
    }
    Ok(applied)
}

/// Validate exact equality substitutions before using them as rewrites.
///
/// SolveSpace-style substitution relies on orienting simple equations before
/// nonlinear iteration. The exact-computation discipline makes the proof
/// obligation explicit: symbolic reduction may propose cheaper topology, but
/// contradictions and cycles must be represented exactly instead of hidden in
/// floating-point iteration. This function checks self-rewrites, duplicate
/// left-hand sides, and directed cycles with exact offset accumulation.
pub fn validate_equality_substitutions(
    substitutions: &[EqualitySubstitution],
) -> EqualitySubstitutionValidation {
    let mut problems = Vec::new();
    let mut edges: BTreeMap<SymbolId, (SymbolId, Real, usize)> = BTreeMap::new();

    for substitution in substitutions {
        if substitution.left == substitution.right {
            problems.push(EqualitySubstitutionProblem::SelfSubstitution {
                constraint_index: substitution.constraint_index,
                symbol: substitution.left,
                offset: substitution.offset.clone(),
                consistent: substitution.offset == Real::zero(),
            });
        }

        if let Some((right, offset, constraint_index)) = edges.get(&substitution.left) {
            problems.push(EqualitySubstitutionProblem::DuplicateLeft {
                left: substitution.left,
                first_constraint_index: *constraint_index,
                second_constraint_index: substitution.constraint_index,
                same_rewrite: *right == substitution.right && *offset == substitution.offset,
            });
        } else {
            edges.insert(
                substitution.left,
                (
                    substitution.right,
                    substitution.offset.clone(),
                    substitution.constraint_index,
                ),
            );
        }
    }

    let mut globally_seen = BTreeSet::new();
    for &start in edges.keys() {
        if globally_seen.contains(&start) {
            continue;
        }
        let mut path = Vec::new();
        let mut offsets = Vec::new();
        let mut local_index = BTreeMap::new();
        let mut current = start;
        while let Some((next, offset, _)) = edges.get(&current) {
            if let Some(&cycle_start) = local_index.get(&current) {
                let symbols = path[cycle_start..].to_vec();
                let net_offset = offsets[cycle_start..]
                    .iter()
                    .cloned()
                    .fold(Real::zero(), |sum, offset| sum + offset);
                problems.push(EqualitySubstitutionProblem::DirectedCycle {
                    symbols,
                    consistent: net_offset == Real::zero(),
                    net_offset,
                });
                break;
            }
            if globally_seen.contains(&current) {
                break;
            }
            local_index.insert(current, path.len());
            path.push(current);
            offsets.push(offset.clone());
            current = *next;
        }
        globally_seen.extend(path);
    }

    EqualitySubstitutionValidation { problems }
}

/// Build exact equality-substitution classes when the graph is consistent.
///
/// Edges have the form `left = right + offset`. The returned class members use
/// `symbol = representative + offset_from_representative`, which is the shape
/// needed by future union-find and affine-row elimination passes. Consistent
/// cycles are allowed because they still define one equality class; inconsistent
/// cycles and conflicting duplicate rewrites return the validation report.
pub fn build_equality_substitution_classes(
    substitutions: &[EqualitySubstitution],
) -> Result<Vec<EqualitySubstitutionClass>, EqualitySubstitutionValidation> {
    let validation = validate_equality_substitutions(substitutions);
    if validation.has_inconsistency() {
        return Err(validation);
    }

    let mut adjacency: BTreeMap<SymbolId, Vec<(SymbolId, Real)>> = BTreeMap::new();
    for substitution in substitutions {
        adjacency
            .entry(substitution.left)
            .or_default()
            .push((substitution.right, -substitution.offset.clone()));
        adjacency
            .entry(substitution.right)
            .or_default()
            .push((substitution.left, substitution.offset.clone()));
    }

    let mut visited = BTreeSet::new();
    let mut classes = Vec::new();
    for &start in adjacency.keys() {
        if visited.contains(&start) {
            continue;
        }

        let mut offsets_from_start = BTreeMap::new();
        let mut queue = VecDeque::new();
        offsets_from_start.insert(start, Real::zero());
        queue.push_back(start);
        visited.insert(start);

        while let Some(symbol) = queue.pop_front() {
            let Some(neighbors) = adjacency.get(&symbol) else {
                continue;
            };
            let symbol_offset = offsets_from_start
                .get(&symbol)
                .expect("queued symbols must have offsets")
                .clone();
            for (neighbor, delta) in neighbors {
                if visited.insert(*neighbor) {
                    offsets_from_start.insert(*neighbor, symbol_offset.clone() + delta.clone());
                    queue.push_back(*neighbor);
                }
            }
        }

        let representative = offsets_from_start
            .keys()
            .copied()
            .min()
            .expect("component must contain at least the start symbol");
        let representative_offset = offsets_from_start
            .get(&representative)
            .expect("representative must have an offset")
            .clone();
        let members = offsets_from_start
            .into_iter()
            .map(
                |(symbol, offset_from_start)| EqualitySubstitutionClassMember {
                    symbol,
                    offset_from_representative: offset_from_start - representative_offset.clone(),
                },
            )
            .collect();
        classes.push(EqualitySubstitutionClass {
            representative,
            members,
        });
    }

    Ok(classes)
}

/// Apply exact equality-substitution classes to a candidate context.
///
/// For each class, any already-bound member can anchor the representative
/// value because members store `symbol = representative + offset`. The helper
/// then writes every member's exact value into the context, unless another
/// already-bound member contradicts the anchor. This mirrors SolveSpace's
/// symbolic equality-class propagation while keeping the exact construction versus
/// certification boundary intact: callers still replay residuals with
/// [`crate::certify_candidate`] before trusting the candidate.
pub fn apply_equality_substitution_classes(
    context: &mut EvaluationContext,
    classes: &[EqualitySubstitutionClass],
) -> EqualitySubstitutionClassApplicationReport {
    let mut rows = Vec::new();
    let mut applied_bindings = 0;
    let mut skipped_classes = 0;
    let mut inconsistent_classes = 0;

    for class in classes {
        let mut anchor: Option<(SymbolId, Real)> = None;
        let mut inconsistency = None;

        for member in &class.members {
            let Some(actual) = context.bindings().get(&member.symbol).cloned() else {
                continue;
            };
            let representative_value = actual.clone() - member.offset_from_representative.clone();
            if let Some((_, anchored_representative_value)) = &anchor {
                let expected = anchored_representative_value.clone()
                    + member.offset_from_representative.clone();
                if expected != actual {
                    inconsistency = Some((member.symbol, expected, actual));
                    break;
                }
            } else {
                anchor = Some((member.symbol, representative_value));
            }
        }

        if let Some((symbol, expected, actual)) = inconsistency {
            inconsistent_classes += 1;
            rows.push(EqualitySubstitutionClassApplicationRow {
                representative: class.representative,
                status: EqualitySubstitutionClassApplicationStatus::InconsistentBoundMember {
                    symbol,
                    expected,
                    actual,
                },
                applied_symbols: Vec::new(),
            });
            continue;
        }

        let Some((anchor_symbol, representative_value)) = anchor else {
            skipped_classes += 1;
            rows.push(EqualitySubstitutionClassApplicationRow {
                representative: class.representative,
                status: EqualitySubstitutionClassApplicationStatus::MissingBoundMember,
                applied_symbols: Vec::new(),
            });
            continue;
        };

        let mut applied_symbols = Vec::new();
        for member in &class.members {
            context.bind(
                member.symbol,
                representative_value.clone() + member.offset_from_representative.clone(),
            );
            applied_symbols.push(member.symbol);
        }
        applied_bindings += applied_symbols.len();
        rows.push(EqualitySubstitutionClassApplicationRow {
            representative: class.representative,
            status: EqualitySubstitutionClassApplicationStatus::Applied { anchor_symbol },
            applied_symbols,
        });
    }

    EqualitySubstitutionClassApplicationReport {
        rows,
        applied_bindings,
        skipped_classes,
        inconsistent_classes,
    }
}

/// Rewrite prepared affine rows through equality-substitution classes.
///
/// For every active prepared affine residual `c + sum(a_i*x_i)`, each symbol
/// that belongs to a substitution class is replaced by
/// `representative + offset`. The resulting exact row is compacted by symbol
/// and exact zero coefficients are dropped. This is the first affine
/// elimination payload; callers must still certify candidates with
/// `certify_candidate` before accepting a solution.
pub fn eliminate_affine_rows_with_substitution_classes(
    prepared: &PreparedProblem<'_>,
    classes: &[EqualitySubstitutionClass],
) -> EqualitySubstitutionEliminationReport {
    let mut substitutions = BTreeMap::new();
    for class in classes {
        for member in &class.members {
            substitutions.insert(
                member.symbol,
                (
                    class.representative,
                    member.offset_from_representative.clone(),
                ),
            );
        }
    }

    let mut rows = Vec::new();
    let mut affine_rows_considered = 0;
    let mut reduced_variable_rows = 0;
    let mut reduced_zero_rows = 0;
    let mut reduced_contradiction_rows = 0;
    let mut reduced_unknown_constant_rows = 0;
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active {
            continue;
        }
        let Some(affine) = &prepared.affine_residuals()[constraint_index] else {
            continue;
        };
        affine_rows_considered += 1;
        let mut constant = affine.constant().clone();
        let mut coefficients = BTreeMap::<SymbolId, Real>::new();
        for (column, coefficient) in affine.coefficients().iter().enumerate() {
            if coefficient.structural_facts().sign == Some(RealSign::Zero) {
                continue;
            }
            let symbol = prepared.problem().variables[column].symbol;
            let (target, offset) = substitutions
                .get(&symbol)
                .cloned()
                .unwrap_or((symbol, Real::zero()));
            constant += coefficient.clone() * offset;
            let entry = coefficients.entry(target).or_insert_with(Real::zero);
            *entry = entry.clone() + coefficient.clone();
        }
        let coefficients = coefficients
            .into_iter()
            .filter(|(_, coefficient)| coefficient.structural_facts().sign != Some(RealSign::Zero))
            .collect::<Vec<_>>();
        classify_eliminated_affine_row(
            &constant,
            &coefficients,
            &mut reduced_variable_rows,
            &mut reduced_zero_rows,
            &mut reduced_contradiction_rows,
            &mut reduced_unknown_constant_rows,
        );
        rows.push(EliminatedAffineRow {
            constraint_index,
            constant,
            coefficients,
        });
    }

    EqualitySubstitutionEliminationReport {
        affine_rows_considered,
        reduced_variable_rows,
        reduced_zero_rows,
        reduced_contradiction_rows,
        reduced_unknown_constant_rows,
        rows,
    }
}

fn classify_eliminated_affine_row(
    constant: &Real,
    coefficients: &[(SymbolId, Real)],
    reduced_variable_rows: &mut usize,
    reduced_zero_rows: &mut usize,
    reduced_contradiction_rows: &mut usize,
    reduced_unknown_constant_rows: &mut usize,
) {
    if !coefficients.is_empty() {
        *reduced_variable_rows += 1;
        return;
    }
    match constant.structural_facts().sign {
        Some(RealSign::Zero) => *reduced_zero_rows += 1,
        Some(RealSign::Positive | RealSign::Negative) => *reduced_contradiction_rows += 1,
        None => *reduced_unknown_constant_rows += 1,
    }
}

/// Find cycles in exact equality-substitution candidates.
///
/// Substitution cycles are not inherently inconsistent, but blindly applying
/// them as a directed rewrite system can loop. SolveSpace-style substitution
/// passes therefore need this diagnostic before choosing a representative or
/// union-find strategy. This function is structural only; it does not decide
/// feasibility.
pub fn find_equality_substitution_cycles(
    substitutions: &[EqualitySubstitution],
) -> Vec<EqualitySubstitutionCycle> {
    let edges = substitutions
        .iter()
        .map(|substitution| (substitution.left, substitution.right))
        .collect::<BTreeMap<_, _>>();
    let mut cycles = Vec::new();
    let mut globally_seen = BTreeSet::new();

    for &start in edges.keys() {
        if globally_seen.contains(&start) {
            continue;
        }
        let mut path = Vec::new();
        let mut local_index = BTreeMap::new();
        let mut current = start;
        while let Some(&next) = edges.get(&current) {
            if let Some(&cycle_start) = local_index.get(&current) {
                cycles.push(EqualitySubstitutionCycle {
                    symbols: path[cycle_start..].to_vec(),
                });
                break;
            }
            if globally_seen.contains(&current) {
                break;
            }
            local_index.insert(current, path.len());
            path.push(current);
            current = next;
        }
        globally_seen.extend(path);
    }
    cycles
}
