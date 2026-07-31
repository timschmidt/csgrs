//! Fraction-free exact dense linear algebra.
//!
//! This module is a small exact counterpart to the primitive-float linear
//! adapter in [`crate::linalg`]. It uses the Bareiss fraction-free elimination
//! recurrence to keep intermediate values exact while delaying normalization:
//! `a'ij = (pivot * aij - aik * akj) / previous_pivot`. This remains in the
//! exact object layer: numerical solvers may propose, but exact arithmetic and
//! certified sign decisions own the proof boundary.

use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{Certainty as PredicateCertainty, PredicateOutcome, PredicatePolicy, Sign};
use hyperreal::{CertifiedRealSign, Rational, Real, RealSign};

use crate::residual_replay::{
    DenseResidualReplayReport, SparseResidualReplayError, SparseResidualReplayReport,
    SparseResidualTerm, replay_assembled_sparse_rows, replay_dense_linear_residuals,
    replay_sparse_linear_residuals, weaker_certainty,
};
use crate::sparse_pattern::{
    SparsePatternEntryStatus, SparsePatternError, SymbolicSparseFactorizationReport,
    analyze_sparse_bareiss_elimination_pattern,
};

/// Failure mode for exact Bareiss-style dense linear algebra.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BareissError {
    /// The matrix was not square, or the right-hand side shape did not match.
    DimensionMismatch,
    /// A pivot sign could not be certified within the requested bound.
    UndecidedPivot {
        /// Pivot column where sign certification failed.
        pivot: usize,
    },
    /// The matrix determinant is certified zero.
    Singular {
        /// Pivot column where no nonzero pivot remained.
        pivot: usize,
    },
    /// Exact division in the fraction-free recurrence was unsupported.
    UnsupportedDivision {
        /// Pivot column whose exact division failed.
        pivot: usize,
    },
    /// The determinant was nonzero, but a Cramer numerator was not divisible in
    /// the current exact `Real` package.
    UnsupportedSolutionDivision {
        /// Solution column whose numerator division failed.
        column: usize,
    },
    /// Exact residual replay of the solved candidate could not decide a row.
    UnknownResidual,
}

/// Failure mode for exact sparse-input Bareiss solving.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SparseBareissError {
    /// The declared sparse matrix was not square or did not match the right
    /// hand side.
    DimensionMismatch,
    /// A sparse term addressed a row or column outside the declared shape.
    TermOutOfBounds {
        /// Offending term row.
        row: usize,
        /// Offending term column.
        column: usize,
    },
    /// The materialized exact dense Bareiss solve failed.
    DenseSolve(BareissError),
    /// Symbolic sparse pattern analysis failed.
    Pattern(SparsePatternError),
    /// The sparse exact solve requires a fully certified symbolic pattern.
    UncertifiedPattern,
    /// Exact division in sparse fraction-free elimination was unsupported.
    UnsupportedDivision {
        /// Pivot column whose exact division failed.
        pivot: usize,
    },
    /// The sparse factorization was certified singular.
    Singular {
        /// Pivot column where no nonzero pivot remained.
        pivot: usize,
    },
    /// A sparse pivot sign could not be certified.
    UndecidedPivot {
        /// Pivot column where sign certification failed.
        pivot: usize,
    },
    /// Exact sparse back substitution could not divide by a certified pivot.
    UnsupportedSolutionDivision {
        /// Solution row whose exact division failed.
        row: usize,
    },
    /// Exact sparse residual replay of the solved candidate failed.
    SparseReplay(SparseResidualReplayError),
}

/// One certified pivot selected during Bareiss elimination.
#[derive(Clone, Debug, PartialEq)]
pub struct BareissPivot {
    /// Pivot column.
    pub pivot: usize,
    /// Source row swapped into the pivot position.
    pub row: usize,
    /// Exact pivot value before the Bareiss update.
    pub value: Real,
}

/// Exact determinant construction selected after shape validation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BareissDeterminantMethod {
    /// Fraction-free Bareiss elimination completed with certified pivots.
    FractionFree,
    /// A pivot-free Faddeev-LeVerrier construction handled an unresolved pivot
    /// or unsupported fraction-free intermediate.
    PivotFreeFaddeevLeverrier,
}

/// Exact determinant report produced by fraction-free Bareiss elimination.
#[derive(Clone, Debug, PartialEq)]
pub struct BareissDeterminantReport {
    /// Exact determinant of the input matrix.
    pub determinant: Real,
    /// Number of row swaps applied during elimination.
    pub swaps: usize,
    /// Certified pivots used before the final determinant value.
    pub pivots: Vec<BareissPivot>,
    /// Exact construction used to produce the determinant expression.
    pub method: BareissDeterminantMethod,
}

/// Exact dense linear solve report built from Bareiss determinants.
#[derive(Clone, Debug, PartialEq)]
pub struct BareissSolveReport {
    /// Exact solution vector.
    pub solution: Vec<Real>,
    /// Determinant report for the coefficient matrix.
    pub determinant: BareissDeterminantReport,
    /// Numerators from Cramer's rule, one per solution coordinate.
    pub numerators: Vec<Real>,
    /// Exact replay of `A*x-b` for the returned solution.
    pub residual_replay: DenseResidualReplayReport,
    /// Weakest predicate evidence consumed by determinant and residual decisions.
    pub certainty: PredicateCertainty,
}

/// Exact dense linear solve report for several right-hand sides sharing one matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct BareissMultiRhsSolveReport {
    /// Exact solution vector for each right-hand side, in input order.
    pub solutions: Vec<Vec<Real>>,
    /// Shared determinant report for the coefficient matrix.
    pub determinant: BareissDeterminantReport,
    /// Cramer numerators for each solution vector, in input order.
    pub numerators: Vec<Vec<Real>>,
    /// Independent exact `A*x-b` replay for each solution vector.
    pub residual_replays: Vec<DenseResidualReplayReport>,
    /// Weakest predicate evidence consumed by all determinant and residual decisions.
    pub certainty: PredicateCertainty,
}

/// Exact sparse-input linear solve report.
///
/// This is the first production solve surface for sparse caller input. It does
/// not claim to be a pattern-preserving sparse LU: the sparse terms are
/// validated and accumulated exactly into a dense matrix, solved through
/// Bareiss fraction-free determinants, then replayed against the original
/// sparse terms. The report keeps that materialization visible so callers can
/// audit the proof boundary instead of mistaking this for a hidden numeric
/// sparse adapter.
#[derive(Clone, Debug, PartialEq)]
pub struct SparseBareissSolveReport {
    /// Declared row count.
    pub row_count: usize,
    /// Declared column count.
    pub column_count: usize,
    /// Exact materialized dense matrix after accumulating duplicate sparse
    /// terms.
    pub dense_matrix: Vec<Vec<Real>>,
    /// Exact dense Bareiss solve report.
    pub dense_solve: BareissSolveReport,
    /// Exact replay against the original sparse representation.
    pub sparse_residual_replay: SparseResidualReplayReport,
}

/// Exact sparse Bareiss solve report that preserves row sparsity.
///
/// Unlike [`SparseBareissSolveReport`], this report is not the dense
/// materialization path. It performs Bareiss fraction-free elimination directly
/// on sparse row maps, records introduced fill, solves the resulting sparse
/// triangular system exactly, and replays the solution against the original
/// sparse terms. The symbolic pattern audit is retained so callers can inspect
/// the structural schedule that bounded the numeric updates. This follows the
/// sparse elimination pattern model while keeping the numerical
/// recurrence in Bareiss's exact fraction-free form and the exact proof
/// boundary.
#[derive(Clone, Debug, PartialEq)]
pub struct SparsePatternPreservingBareissSolveReport {
    /// Declared row count.
    pub row_count: usize,
    /// Declared column count.
    pub column_count: usize,
    /// Symbolic sparse factorization audit used before numeric elimination.
    pub symbolic_pattern: SymbolicSparseFactorizationReport,
    /// Certified numeric pivots chosen by sparse Bareiss elimination.
    pub pivots: Vec<BareissPivot>,
    /// Unique numeric fill entries introduced during elimination.
    pub fill_in_positions: Vec<(usize, usize)>,
    /// Final sparse upper-triangular rows after fraction-free elimination.
    pub upper_rows: Vec<Vec<SparseResidualTerm>>,
    /// Exact solution vector.
    pub solution: Vec<Real>,
    /// Exact replay against the exactly accumulated original sparse terms.
    pub sparse_residual_replay: SparseResidualReplayReport,
}

/// Exact sparse Bareiss solve report using a deterministic minimum-degree
/// symmetric row/column permutation.
///
/// The permutation changes only the elimination schedule. The returned
/// solution is restored to source-column order and replayed against the
/// original sparse system, so ordering remains outside the proof boundary.
#[derive(Clone, Debug, PartialEq)]
pub struct SparseMinimumDegreeBareissSolveReport {
    /// Certified symbolic pattern of the source matrix.
    pub source_symbolic_pattern: SymbolicSparseFactorizationReport,
    /// Permuted index to source index mapping.
    pub permuted_to_source: Vec<usize>,
    /// Source index to permuted index mapping.
    pub source_to_permuted: Vec<usize>,
    /// Exact sparse solve report for the permuted system.
    pub permuted_solve: SparsePatternPreservingBareissSolveReport,
    /// Exact solution restored to source-column order.
    pub solution: Vec<Real>,
    /// Exact residual replay against the source-order terms and right-hand
    /// side.
    pub source_residual_replay: SparseResidualReplayReport,
}

fn bareiss_quotient(numerator: Real, previous_pivot: &Real) -> Option<Real> {
    if let (Some(numerator), Some(previous_pivot)) = (
        numerator.exact_rational_ref(),
        previous_pivot.exact_rational_ref(),
    ) && let Some(quotient) = numerator.checked_exact_integer_quotient(previous_pivot)
    {
        return Some(Real::from(quotient));
    }
    match numerator.clone() / previous_pivot.clone() {
        Ok(quotient) => Some(quotient),
        Err(_) => previous_pivot
            .inverse_ref_assuming_nonzero()
            .ok()
            .map(|inverse| numerator * inverse),
    }
}

fn bareiss_update(
    pivot: &Real,
    value: &Real,
    eliminand: &Real,
    pivot_value: &Real,
    previous_pivot: &Real,
) -> Option<Real> {
    if let (Some(pivot), Some(value), Some(eliminand), Some(pivot_value), Some(previous_pivot)) = (
        pivot.exact_rational_ref(),
        value.exact_rational_ref(),
        eliminand.exact_rational_ref(),
        pivot_value.exact_rational_ref(),
        previous_pivot.exact_rational_ref(),
    ) && let Some(quotient) = pivot.checked_exact_integer_cross_difference_quotient(
        value,
        eliminand,
        pivot_value,
        previous_pivot,
    ) {
        return Some(Real::from(quotient));
    }
    let numerator = pivot.clone() * value.clone() - eliminand.clone() * pivot_value.clone();
    bareiss_quotient(numerator, previous_pivot)
}

fn pivot_free_determinant(matrix: &[Vec<Real>]) -> BareissDeterminantReport {
    let n = matrix.len();
    if n == 0 {
        return BareissDeterminantReport {
            determinant: Real::one(),
            swaps: 0,
            pivots: Vec::new(),
            method: BareissDeterminantMethod::PivotFreeFaddeevLeverrier,
        };
    }

    // Faddeev-LeVerrier constructs the characteristic coefficients without a
    // data-dependent division. Its only scale is the exact rational -1/k, so
    // an unresolved matrix pivot cannot block determinant construction.
    let mut coefficient_matrix = vec![vec![Real::zero(); n]; n];
    for (index, row) in coefficient_matrix.iter_mut().enumerate() {
        row[index] = Real::one();
    }

    for step in 1..=n {
        let mut product = vec![vec![Real::zero(); n]; n];
        for (row_index, row) in product.iter_mut().enumerate() {
            for (column, value) in row.iter_mut().enumerate() {
                *value = (0..n).fold(Real::zero(), |sum, inner| {
                    sum + &matrix[row_index][inner] * &coefficient_matrix[inner][column]
                });
            }
        }
        let trace = (0..n).fold(Real::zero(), |sum, index| sum + &product[index][index]);
        let denominator = u64::try_from(step).expect("an allocated matrix dimension fits u64");
        let scale = Real::from(
            Rational::fraction(-1, denominator).expect("Faddeev-LeVerrier step is nonzero"),
        );
        let coefficient = trace * scale;
        if step == n {
            let determinant = if n.is_multiple_of(2) {
                coefficient
            } else {
                -coefficient
            };
            return BareissDeterminantReport {
                determinant,
                swaps: 0,
                pivots: Vec::new(),
                method: BareissDeterminantMethod::PivotFreeFaddeevLeverrier,
            };
        }
        for (index, row) in product.iter_mut().enumerate() {
            row[index] += coefficient.clone();
        }
        coefficient_matrix = product;
    }

    unreachable!("a nonempty determinant construction executes at least one step")
}

/// Computes an exact determinant with Bareiss fraction-free elimination.
///
/// Pivot choices are certified through [`Real::certified_sign_until`]. A
/// certified zero determinant is returned as a successful report with
/// `determinant == 0`. If a pivot cannot be certified within the fast bound,
/// or a fraction-free intermediate is unavailable, a pivot-free exact
/// Faddeev-LeVerrier construction completes the determinant without making a
/// topology decision.
pub fn determinant_bareiss(
    matrix: &[Vec<Real>],
    min_precision: i32,
) -> Result<BareissDeterminantReport, BareissError> {
    if matrix.iter().any(|row| row.len() != matrix.len()) {
        return Err(BareissError::DimensionMismatch);
    }

    let n = matrix.len();
    if n == 0 {
        return Ok(BareissDeterminantReport {
            determinant: Real::one(),
            swaps: 0,
            pivots: Vec::new(),
            method: BareissDeterminantMethod::FractionFree,
        });
    }
    if n == 1 {
        return Ok(BareissDeterminantReport {
            determinant: matrix[0][0].clone(),
            swaps: 0,
            pivots: Vec::new(),
            method: BareissDeterminantMethod::FractionFree,
        });
    }

    let mut work = matrix.to_vec();
    let mut swaps = 0;
    let mut pivots = Vec::with_capacity(n - 1);
    let mut previous_pivot = Real::one();

    for pivot in 0..(n - 1) {
        let pivot_row = match select_pivot_row(&work, pivot, min_precision) {
            Ok(pivot_row) => pivot_row,
            Err(BareissError::UndecidedPivot { .. }) => {
                return Ok(pivot_free_determinant(matrix));
            }
            Err(error) => return Err(error),
        };
        let Some(pivot_row) = pivot_row else {
            return Ok(BareissDeterminantReport {
                determinant: Real::zero(),
                swaps,
                pivots,
                method: BareissDeterminantMethod::FractionFree,
            });
        };

        if pivot_row != pivot {
            work.swap(pivot_row, pivot);
            swaps += 1;
        }

        let pivot_value = work[pivot][pivot].clone();
        let pivot_work_row = work[pivot].clone();
        pivots.push(BareissPivot {
            pivot,
            row: pivot_row,
            value: pivot_value.clone(),
        });

        for row in work.iter_mut().take(n).skip(pivot + 1) {
            for column in (pivot + 1)..n {
                let Some(updated) = bareiss_update(
                    &pivot_value,
                    &row[column],
                    &row[pivot],
                    &pivot_work_row[column],
                    &previous_pivot,
                ) else {
                    return Ok(pivot_free_determinant(matrix));
                };
                row[column] = updated;
            }
        }

        for row in work.iter_mut().take(n).skip(pivot + 1) {
            row[pivot] = Real::zero();
        }
        previous_pivot = pivot_value;
    }

    let mut determinant = work[n - 1][n - 1].clone();
    if swaps % 2 == 1 {
        determinant = -determinant;
    }

    Ok(BareissDeterminantReport {
        determinant,
        swaps,
        pivots,
        method: BareissDeterminantMethod::FractionFree,
    })
}

/// Solves a square dense linear system by fraction-free Bareiss elimination.
///
/// The coefficient matrix and right-hand side are eliminated together, so one
/// augmented fraction-free pass constructs the upper-triangular system. The
/// previous determinant/Cramer construction remains a fallback when the
/// current exact scalar package cannot represent an intermediate augmented
/// division. The returned candidate is immediately replayed through
/// [`crate::replay_dense_linear_residuals`] so the report carries both the
/// construction evidence and the exact `A*x-b` proof.
pub fn solve_dense_linear_system_bareiss(
    matrix: &[Vec<Real>],
    rhs: &[Real],
    min_precision: i32,
    policy: PredicatePolicy,
) -> Result<BareissSolveReport, BareissError> {
    if matrix.len() != rhs.len() || matrix.iter().any(|row| row.len() != matrix.len()) {
        return Err(BareissError::DimensionMismatch);
    }

    let n = matrix.len();
    let mut work = matrix.to_vec();
    let mut rhs_work = rhs.to_vec();
    let mut swaps = 0;
    let mut pivots = Vec::with_capacity(n.saturating_sub(1));
    let mut previous_pivot = Real::one();
    let mut solve_certainty = PredicateCertainty::Exact;

    for pivot in 0..n.saturating_sub(1) {
        let pivot_row = match select_pivot_row_for_solve(&work, pivot, min_precision, policy) {
            Ok((pivot_row, certainty)) => {
                solve_certainty = weaker_certainty(solve_certainty, certainty);
                pivot_row
            }
            Err(BareissError::UndecidedPivot { .. }) => {
                return solve_dense_linear_system_bareiss_cramer(
                    matrix,
                    rhs,
                    min_precision,
                    policy,
                );
            }
            Err(error) => return Err(error),
        };
        let Some(pivot_row) = pivot_row else {
            return Err(BareissError::Singular {
                pivot: n.saturating_sub(1),
            });
        };
        if pivot_row != pivot {
            work.swap(pivot_row, pivot);
            rhs_work.swap(pivot_row, pivot);
            swaps += 1;
        }

        let pivot_value = work[pivot][pivot].clone();
        let pivot_work_row = work[pivot].clone();
        let pivot_rhs = rhs_work[pivot].clone();
        pivots.push(BareissPivot {
            pivot,
            row: pivot_row,
            value: pivot_value.clone(),
        });

        for row in (pivot + 1)..n {
            let eliminand = work[row][pivot].clone();
            for column in (pivot + 1)..n {
                let Some(updated) = bareiss_update(
                    &pivot_value,
                    &work[row][column],
                    &eliminand,
                    &pivot_work_row[column],
                    &previous_pivot,
                ) else {
                    return solve_dense_linear_system_bareiss_cramer(
                        matrix,
                        rhs,
                        min_precision,
                        policy,
                    );
                };
                work[row][column] = updated;
            }
            rhs_work[row] = match bareiss_update(
                &pivot_value,
                &rhs_work[row],
                &eliminand,
                &pivot_rhs,
                &previous_pivot,
            ) {
                Some(value) => value,
                None => {
                    return solve_dense_linear_system_bareiss_cramer(
                        matrix,
                        rhs,
                        min_precision,
                        policy,
                    );
                }
            };
            work[row][pivot] = Real::zero();
        }
        previous_pivot = pivot_value;
    }

    let mut determinant_value = if n == 0 {
        Real::one()
    } else {
        work[n - 1][n - 1].clone()
    };
    if swaps % 2 == 1 {
        determinant_value = -determinant_value;
    }
    let determinant_certainty = nonzero_determinant_certainty(
        &determinant_value,
        min_precision,
        policy,
        n.saturating_sub(1),
    )?;
    let determinant = BareissDeterminantReport {
        determinant: determinant_value,
        swaps,
        pivots,
        method: BareissDeterminantMethod::FractionFree,
    };

    let mut solution = vec![Real::zero(); n];
    for row in (0..n).rev() {
        let trailing_sum = ((row + 1)..n).fold(Real::zero(), |sum, column| {
            sum + work[row][column].clone() * solution[column].clone()
        });
        solution[row] =
            quotient_after_nonzero(rhs_work[row].clone() - trailing_sum, &work[row][row])
                .map_err(|_| BareissError::UnsupportedSolutionDivision { column: row })?;
    }
    let numerators = solution
        .iter()
        .map(|value| value.clone() * determinant.determinant.clone())
        .collect::<Vec<_>>();

    let residual_replay =
        replay_dense_linear_residuals(matrix, rhs, &solution, min_precision, policy)
            .map_err(map_dense_replay_error)?;
    let certainty = weaker_certainty(
        solve_certainty,
        weaker_certainty(determinant_certainty, residual_replay.certainty),
    );

    Ok(BareissSolveReport {
        solution,
        determinant,
        numerators,
        residual_replay,
        certainty,
    })
}

/// Solves several square dense linear systems with a shared coefficient matrix.
///
/// The matrix is fraction-free eliminated once while every right-hand side is
/// carried through the same certified row operations. Each solution retains
/// its Cramer numerators and an independent exact residual replay. If an
/// augmented intermediate cannot be represented by the current exact scalar
/// package, the implementation falls back to the determinant construction for
/// each right-hand side without weakening the returned evidence.
pub fn solve_dense_linear_system_bareiss_multi_rhs(
    matrix: &[Vec<Real>],
    right_hand_sides: &[Vec<Real>],
    min_precision: i32,
    policy: PredicatePolicy,
) -> Result<BareissMultiRhsSolveReport, BareissError> {
    if matrix.iter().any(|row| row.len() != matrix.len())
        || right_hand_sides.iter().any(|rhs| rhs.len() != matrix.len())
    {
        return Err(BareissError::DimensionMismatch);
    }

    let n = matrix.len();
    let mut work = matrix.to_vec();
    let mut rhs_work = right_hand_sides.to_vec();
    let mut swaps = 0;
    let mut pivots = Vec::with_capacity(n.saturating_sub(1));
    let mut previous_pivot = Real::one();
    let mut solve_certainty = PredicateCertainty::Exact;

    for pivot in 0..n.saturating_sub(1) {
        let pivot_row = match select_pivot_row_for_solve(&work, pivot, min_precision, policy) {
            Ok((pivot_row, certainty)) => {
                solve_certainty = weaker_certainty(solve_certainty, certainty);
                pivot_row
            }
            Err(BareissError::UndecidedPivot { .. }) => {
                return solve_dense_linear_system_bareiss_multi_rhs_cramer(
                    matrix,
                    right_hand_sides,
                    min_precision,
                    policy,
                );
            }
            Err(error) => return Err(error),
        };
        let Some(pivot_row) = pivot_row else {
            return Err(BareissError::Singular {
                pivot: n.saturating_sub(1),
            });
        };
        if pivot_row != pivot {
            work.swap(pivot_row, pivot);
            for rhs in &mut rhs_work {
                rhs.swap(pivot_row, pivot);
            }
            swaps += 1;
        }

        let pivot_value = work[pivot][pivot].clone();
        let pivot_work_row = work[pivot].clone();
        let pivot_rhs = rhs_work
            .iter()
            .map(|rhs| rhs[pivot].clone())
            .collect::<Vec<_>>();
        pivots.push(BareissPivot {
            pivot,
            row: pivot_row,
            value: pivot_value.clone(),
        });

        for row in (pivot + 1)..n {
            let eliminand = work[row][pivot].clone();
            for column in (pivot + 1)..n {
                let Some(updated) = bareiss_update(
                    &pivot_value,
                    &work[row][column],
                    &eliminand,
                    &pivot_work_row[column],
                    &previous_pivot,
                ) else {
                    return solve_dense_linear_system_bareiss_multi_rhs_cramer(
                        matrix,
                        right_hand_sides,
                        min_precision,
                        policy,
                    );
                };
                work[row][column] = updated;
            }
            for (rhs_index, rhs) in rhs_work.iter_mut().enumerate() {
                rhs[row] = match bareiss_update(
                    &pivot_value,
                    &rhs[row],
                    &eliminand,
                    &pivot_rhs[rhs_index],
                    &previous_pivot,
                ) {
                    Some(value) => value,
                    None => {
                        return solve_dense_linear_system_bareiss_multi_rhs_cramer(
                            matrix,
                            right_hand_sides,
                            min_precision,
                            policy,
                        );
                    }
                };
            }
            work[row][pivot] = Real::zero();
        }
        previous_pivot = pivot_value;
    }

    let mut determinant_value = if n == 0 {
        Real::one()
    } else {
        work[n - 1][n - 1].clone()
    };
    if swaps % 2 == 1 {
        determinant_value = -determinant_value;
    }
    let determinant_certainty = nonzero_determinant_certainty(
        &determinant_value,
        min_precision,
        policy,
        n.saturating_sub(1),
    )?;
    let determinant = BareissDeterminantReport {
        determinant: determinant_value,
        swaps,
        pivots,
        method: BareissDeterminantMethod::FractionFree,
    };

    let mut solutions = Vec::with_capacity(rhs_work.len());
    for rhs in &rhs_work {
        let mut solution = vec![Real::zero(); n];
        for row in (0..n).rev() {
            let trailing_sum = ((row + 1)..n).fold(Real::zero(), |sum, column| {
                sum + work[row][column].clone() * solution[column].clone()
            });
            solution[row] =
                quotient_after_nonzero(rhs[row].clone() - trailing_sum, &work[row][row])
                    .map_err(|_| BareissError::UnsupportedSolutionDivision { column: row })?;
        }
        solutions.push(solution);
    }
    let numerators = solutions
        .iter()
        .map(|solution| {
            solution
                .iter()
                .map(|value| value.clone() * determinant.determinant.clone())
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let residual_replays = right_hand_sides
        .iter()
        .zip(&solutions)
        .map(|(rhs, solution)| {
            replay_dense_linear_residuals(matrix, rhs, solution, min_precision, policy)
                .map_err(map_dense_replay_error)
        })
        .collect::<Result<Vec<_>, _>>()?;
    let certainty = residual_replays.iter().fold(
        weaker_certainty(solve_certainty, determinant_certainty),
        |certainty, replay| weaker_certainty(certainty, replay.certainty),
    );

    Ok(BareissMultiRhsSolveReport {
        solutions,
        determinant,
        numerators,
        residual_replays,
        certainty,
    })
}

fn solve_dense_linear_system_bareiss_multi_rhs_cramer(
    matrix: &[Vec<Real>],
    right_hand_sides: &[Vec<Real>],
    min_precision: i32,
    policy: PredicatePolicy,
) -> Result<BareissMultiRhsSolveReport, BareissError> {
    let determinant = determinant_bareiss(matrix, min_precision)?;
    let determinant_certainty = nonzero_determinant_certainty(
        &determinant.determinant,
        min_precision,
        policy,
        matrix.len().saturating_sub(1),
    )?;
    let Some(inverse) = (!right_hand_sides.is_empty())
        .then(|| determinant.determinant.inverse_ref_assuming_nonzero())
        .transpose()
        .map_err(|_| BareissError::UnsupportedSolutionDivision { column: 0 })?
    else {
        return Ok(BareissMultiRhsSolveReport {
            solutions: Vec::new(),
            determinant,
            numerators: Vec::new(),
            residual_replays: Vec::new(),
            certainty: determinant_certainty,
        });
    };

    let mut solutions = Vec::with_capacity(right_hand_sides.len());
    let mut numerators = Vec::with_capacity(right_hand_sides.len());
    let mut residual_replays = Vec::with_capacity(right_hand_sides.len());
    let mut certainty = determinant_certainty;
    for rhs in right_hand_sides {
        let (solution, rhs_numerators) =
            solve_cramer_coordinates(matrix, rhs, &inverse, min_precision)?;
        let replay = replay_dense_linear_residuals(matrix, rhs, &solution, min_precision, policy)
            .map_err(map_dense_replay_error)?;
        certainty = weaker_certainty(certainty, replay.certainty);
        solutions.push(solution);
        numerators.push(rhs_numerators);
        residual_replays.push(replay);
    }

    Ok(BareissMultiRhsSolveReport {
        determinant,
        solutions,
        numerators,
        residual_replays,
        certainty,
    })
}

fn solve_dense_linear_system_bareiss_cramer(
    matrix: &[Vec<Real>],
    rhs: &[Real],
    min_precision: i32,
    policy: PredicatePolicy,
) -> Result<BareissSolveReport, BareissError> {
    let determinant = determinant_bareiss(matrix, min_precision)?;
    let determinant_certainty = nonzero_determinant_certainty(
        &determinant.determinant,
        min_precision,
        policy,
        matrix.len().saturating_sub(1),
    )?;
    let inverse = determinant
        .determinant
        .inverse_ref_assuming_nonzero()
        .map_err(|_| BareissError::UnsupportedSolutionDivision { column: 0 })?;
    let (solution, numerators) = solve_cramer_coordinates(matrix, rhs, &inverse, min_precision)?;

    let residual_replay =
        replay_dense_linear_residuals(matrix, rhs, &solution, min_precision, policy)
            .map_err(map_dense_replay_error)?;
    let certainty = weaker_certainty(determinant_certainty, residual_replay.certainty);

    Ok(BareissSolveReport {
        solution,
        determinant,
        numerators,
        residual_replay,
        certainty,
    })
}

fn solve_cramer_coordinates(
    matrix: &[Vec<Real>],
    rhs: &[Real],
    determinant_inverse: &Real,
    min_precision: i32,
) -> Result<(Vec<Real>, Vec<Real>), BareissError> {
    let mut replaced = matrix.to_vec();
    let mut numerators = Vec::with_capacity(matrix.len());
    let mut solution = Vec::with_capacity(matrix.len());
    for column in 0..matrix.len() {
        for (row, value) in rhs.iter().enumerate() {
            replaced[row][column] = value.clone();
        }
        let numerator = determinant_bareiss(&replaced, min_precision)?.determinant;
        solution.push(numerator.clone() * determinant_inverse.clone());
        numerators.push(numerator);
        for (row, coefficients) in matrix.iter().enumerate() {
            replaced[row][column] = coefficients[column].clone();
        }
    }
    Ok((solution, numerators))
}

/// Solves a square sparse linear system with exact Bareiss materialization.
///
/// Sparse terms encode `A[row, column] += coefficient`, with repeated entries
/// accumulated exactly. After shape validation, this function materializes the
/// square matrix and delegates construction to
/// [`solve_dense_linear_system_bareiss`]. The returned solution is then replayed
/// through [`crate::replay_sparse_linear_residuals`] against the original sparse
/// terms. This is a the exactness boundary-aligned proof surface for sparse caller input; a true
/// pattern-preserving exact sparse factorization remains a separate backend.
pub fn solve_sparse_linear_system_bareiss(
    row_count: usize,
    column_count: usize,
    terms: &[SparseResidualTerm],
    rhs: &[Real],
    min_precision: i32,
) -> Result<SparseBareissSolveReport, SparseBareissError> {
    if row_count != column_count || rhs.len() != row_count {
        return Err(SparseBareissError::DimensionMismatch);
    }

    let mut dense_matrix = vec![vec![Real::zero(); column_count]; row_count];
    for term in terms {
        if term.row >= row_count || term.column >= column_count {
            return Err(SparseBareissError::TermOutOfBounds {
                row: term.row,
                column: term.column,
            });
        }
        dense_matrix[term.row][term.column] =
            dense_matrix[term.row][term.column].clone() + term.coefficient.clone();
    }

    let dense_solve = solve_dense_linear_system_bareiss(
        &dense_matrix,
        rhs,
        min_precision,
        PredicatePolicy::STRICT,
    )
    .map_err(SparseBareissError::DenseSolve)?;
    let sparse_residual_replay = replay_sparse_linear_residuals(
        row_count,
        column_count,
        terms,
        rhs,
        &dense_solve.solution,
        min_precision,
    )
    .map_err(SparseBareissError::SparseReplay)?;

    Ok(SparseBareissSolveReport {
        row_count,
        column_count,
        dense_matrix,
        dense_solve,
        sparse_residual_replay,
    })
}

/// Solves a square sparse system with sparse fraction-free Bareiss updates.
///
/// Terms encode `A[row, column] += coefficient`. Duplicate terms are
/// accumulated exactly; certified zeros are removed from the active sparse
/// rows. Before numeric elimination, the same input is audited by
/// [`crate::analyze_sparse_bareiss_elimination_pattern`]. Unknown structural
/// signs or structural singularity are refused because accepting a sparse
/// pattern after an undecided sign would violate the exact decision
/// boundary. The numeric phase then applies Bareiss's recurrence only over row
/// map unions that can become nonzero, preserving fill as explicit report
/// evidence rather than hiding it inside a dense matrix.
pub fn solve_sparse_linear_system_bareiss_pattern_preserving(
    row_count: usize,
    column_count: usize,
    terms: &[SparseResidualTerm],
    rhs: &[Real],
    min_precision: i32,
) -> Result<SparsePatternPreservingBareissSolveReport, SparseBareissError> {
    if row_count != column_count || rhs.len() != row_count {
        return Err(SparseBareissError::DimensionMismatch);
    }
    let symbolic_pattern =
        analyze_sparse_bareiss_elimination_pattern(row_count, column_count, terms, min_precision)
            .map_err(SparseBareissError::Pattern)?;
    if !symbolic_pattern.fully_certified_pattern() {
        return Err(SparseBareissError::UncertifiedPattern);
    }

    // The symbolic phase has already accumulated duplicate coordinates and
    // certified every active entry. Reuse that exact work for the numeric
    // phase instead of rebuilding and re-certifying the same sparse rows.
    let mut rows = vec![BTreeMap::<usize, Real>::new(); row_count];
    for entry in &symbolic_pattern.entries {
        match entry.status {
            SparsePatternEntryStatus::CertifiedNonzero => {
                rows[entry.row].insert(entry.column, entry.value.clone());
            }
            SparsePatternEntryStatus::CertifiedZero => {}
            SparsePatternEntryStatus::UnknownSign => {
                unreachable!("a fully certified symbolic pattern cannot contain an unknown entry")
            }
        }
    }
    let proof_rows = rows
        .iter()
        .map(|row| {
            row.iter()
                .map(|(column, coefficient)| (*column, coefficient.clone()))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let mut rhs_work = rhs.to_vec();
    let mut previous_pivot = Real::one();
    let mut pivots = Vec::with_capacity(row_count.saturating_sub(1));
    let mut fill_in_positions = BTreeSet::<(usize, usize)>::new();

    for pivot in 0..row_count {
        let Some(pivot_row) = select_sparse_pivot_row(&rows, pivot, min_precision)? else {
            return Err(SparseBareissError::Singular { pivot });
        };
        if pivot_row != pivot {
            rows.swap(pivot_row, pivot);
            rhs_work.swap(pivot_row, pivot);
        }
        let pivot_value = rows[pivot]
            .get(&pivot)
            .cloned()
            .ok_or(SparseBareissError::Singular { pivot })?;
        pivots.push(BareissPivot {
            pivot,
            row: pivot_row,
            value: pivot_value.clone(),
        });
        if pivot == row_count - 1 {
            break;
        }

        let pivot_row_map = rows[pivot].clone();
        let pivot_rhs = rhs_work[pivot].clone();
        for row_index in (pivot + 1)..row_count {
            let Some(eliminand) = rows[row_index].remove(&pivot) else {
                continue;
            };
            if is_certified_zero(&eliminand, min_precision)? {
                continue;
            }
            let update_columns = rows[row_index]
                .keys()
                .copied()
                .chain(pivot_row_map.keys().copied())
                .filter(|column| *column > pivot)
                .collect::<BTreeSet<_>>();
            for column in update_columns {
                let row_value = rows[row_index]
                    .get(&column)
                    .cloned()
                    .unwrap_or_else(Real::zero);
                let pivot_column_value = pivot_row_map
                    .get(&column)
                    .cloned()
                    .unwrap_or_else(Real::zero);
                let updated = bareiss_update(
                    &pivot_value,
                    &row_value,
                    &eliminand,
                    &pivot_column_value,
                    &previous_pivot,
                )
                .ok_or(SparseBareissError::UnsupportedDivision { pivot })?;
                if is_certified_zero(&updated, min_precision)? {
                    rows[row_index].remove(&column);
                } else {
                    if !rows[row_index].contains_key(&column) {
                        fill_in_positions.insert((row_index, column));
                    }
                    rows[row_index].insert(column, updated);
                }
            }
            rhs_work[row_index] = bareiss_update(
                &pivot_value,
                &rhs_work[row_index],
                &eliminand,
                &pivot_rhs,
                &previous_pivot,
            )
            .ok_or(SparseBareissError::UnsupportedDivision { pivot })?;
        }
        previous_pivot = pivot_value;
    }

    let solution = sparse_back_substitution(&rows, &rhs_work, min_precision)?;
    let sparse_residual_replay = replay_assembled_sparse_rows(
        row_count,
        column_count,
        &proof_rows,
        rhs,
        &solution,
        min_precision,
    )
    .map_err(SparseBareissError::SparseReplay)?;
    let upper_rows = rows
        .iter()
        .enumerate()
        .flat_map(|(row, entries)| {
            entries
                .iter()
                .map(move |(column, coefficient)| SparseResidualTerm {
                    row,
                    column: *column,
                    coefficient: coefficient.clone(),
                })
        })
        .fold(vec![Vec::new(); row_count], |mut rows, term| {
            rows[term.row].push(term);
            rows
        });

    Ok(SparsePatternPreservingBareissSolveReport {
        row_count,
        column_count,
        symbolic_pattern,
        pivots,
        fill_in_positions: fill_in_positions.into_iter().collect(),
        upper_rows,
        solution,
        sparse_residual_replay,
    })
}

/// Solves a square sparse system after a deterministic symmetric
/// minimum-degree permutation.
///
/// The ordering graph is the symmetrized certified-nonzero pattern. At each
/// step the active vertex with the smallest degree is eliminated, ties are
/// resolved by source index, and its active neighbors are connected to model
/// fill. Rows, columns, and the right-hand side are permuted together. This is
/// an opt-in fill-reducing schedule for sparse systems; the ordinary
/// pattern-preserving entry point retains authored ordering.
pub fn solve_sparse_linear_system_bareiss_minimum_degree(
    row_count: usize,
    column_count: usize,
    terms: &[SparseResidualTerm],
    rhs: &[Real],
    min_precision: i32,
) -> Result<SparseMinimumDegreeBareissSolveReport, SparseBareissError> {
    if row_count != column_count || rhs.len() != row_count {
        return Err(SparseBareissError::DimensionMismatch);
    }
    let source_symbolic_pattern =
        analyze_sparse_bareiss_elimination_pattern(row_count, column_count, terms, min_precision)
            .map_err(SparseBareissError::Pattern)?;
    if !source_symbolic_pattern.fully_certified_pattern() {
        return Err(SparseBareissError::UncertifiedPattern);
    }

    let permuted_to_source = minimum_degree_symmetric_permutation(
        row_count,
        source_symbolic_pattern
            .entries
            .iter()
            .filter(|entry| entry.status == SparsePatternEntryStatus::CertifiedNonzero)
            .map(|entry| (entry.row, entry.column)),
    );
    let mut source_to_permuted = vec![0; row_count];
    for (permuted, source) in permuted_to_source.iter().copied().enumerate() {
        source_to_permuted[source] = permuted;
    }

    let permuted_terms = terms
        .iter()
        .map(|term| SparseResidualTerm {
            row: source_to_permuted[term.row],
            column: source_to_permuted[term.column],
            coefficient: term.coefficient.clone(),
        })
        .collect::<Vec<_>>();
    let permuted_rhs = permuted_to_source
        .iter()
        .map(|source| rhs[*source].clone())
        .collect::<Vec<_>>();
    let permuted_solve = solve_sparse_linear_system_bareiss_pattern_preserving(
        row_count,
        column_count,
        &permuted_terms,
        &permuted_rhs,
        min_precision,
    )?;

    let mut solution = vec![Real::zero(); column_count];
    for (permuted, source) in permuted_to_source.iter().copied().enumerate() {
        solution[source] = permuted_solve.solution[permuted].clone();
    }
    let source_residual_replay = replay_sparse_linear_residuals(
        row_count,
        column_count,
        terms,
        rhs,
        &solution,
        min_precision,
    )
    .map_err(SparseBareissError::SparseReplay)?;

    Ok(SparseMinimumDegreeBareissSolveReport {
        source_symbolic_pattern,
        permuted_to_source,
        source_to_permuted,
        permuted_solve,
        solution,
        source_residual_replay,
    })
}

fn minimum_degree_symmetric_permutation(
    order: usize,
    positions: impl IntoIterator<Item = (usize, usize)>,
) -> Vec<usize> {
    let mut adjacency = vec![BTreeSet::new(); order];
    for (row, column) in positions {
        if row != column {
            adjacency[row].insert(column);
            adjacency[column].insert(row);
        }
    }

    let mut active = vec![true; order];
    let mut permutation = Vec::with_capacity(order);
    for _ in 0..order {
        let selected = (0..order)
            .filter(|index| active[*index])
            .min_by_key(|index| {
                (
                    adjacency[*index]
                        .iter()
                        .filter(|neighbor| active[**neighbor])
                        .count(),
                    *index,
                )
            })
            .expect("one active vertex remains per ordering step");
        let neighbors = adjacency[selected]
            .iter()
            .copied()
            .filter(|neighbor| active[*neighbor])
            .collect::<Vec<_>>();
        for (offset, left) in neighbors.iter().copied().enumerate() {
            adjacency[left].remove(&selected);
            for right in neighbors.iter().copied().skip(offset + 1) {
                adjacency[left].insert(right);
                adjacency[right].insert(left);
            }
        }
        active[selected] = false;
        permutation.push(selected);
    }
    permutation
}

fn select_pivot_row_for_solve(
    matrix: &[Vec<Real>],
    pivot: usize,
    min_precision: i32,
    policy: PredicatePolicy,
) -> Result<(Option<usize>, PredicateCertainty), BareissError> {
    match select_pivot_row(matrix, pivot, min_precision) {
        Ok(row) => return Ok((row, PredicateCertainty::Exact)),
        Err(BareissError::UndecidedPivot { .. }) => {}
        Err(error) => return Err(error),
    }

    let mut zero_certainty = PredicateCertainty::Exact;
    let mut saw_unknown = false;
    for (row, matrix_row) in matrix.iter().enumerate().skip(pivot) {
        match certified_sign_with_policy(&matrix_row[pivot], min_precision, policy) {
            Ok((RealSign::Negative | RealSign::Positive, certainty)) => {
                return Ok((Some(row), certainty));
            }
            Ok((RealSign::Zero, certainty)) => {
                zero_certainty = weaker_certainty(zero_certainty, certainty);
            }
            Err(BareissError::UndecidedPivot { .. }) => saw_unknown = true,
            Err(error) => return Err(error),
        }
    }

    if saw_unknown {
        Err(BareissError::UndecidedPivot { pivot })
    } else {
        Ok((None, zero_certainty))
    }
}

fn select_pivot_row(
    matrix: &[Vec<Real>],
    pivot: usize,
    min_precision: i32,
) -> Result<Option<usize>, BareissError> {
    let mut saw_unknown = false;
    for (row, matrix_row) in matrix.iter().enumerate().skip(pivot) {
        match certified_sign(&matrix_row[pivot], min_precision) {
            Ok(RealSign::Negative | RealSign::Positive) => return Ok(Some(row)),
            Ok(RealSign::Zero) => {}
            Err(BareissError::UndecidedPivot { .. }) => saw_unknown = true,
            Err(error) => return Err(error),
        }
    }

    if saw_unknown {
        Err(BareissError::UndecidedPivot { pivot })
    } else {
        Ok(None)
    }
}

fn select_sparse_pivot_row(
    rows: &[BTreeMap<usize, Real>],
    pivot: usize,
    min_precision: i32,
) -> Result<Option<usize>, SparseBareissError> {
    let mut saw_unknown = false;
    for (row, entries) in rows.iter().enumerate().skip(pivot) {
        let Some(value) = entries.get(&pivot) else {
            continue;
        };
        match value.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Negative | RealSign::Positive,
                ..
            } => return Ok(Some(row)),
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => {}
            CertifiedRealSign::Unknown { .. } => saw_unknown = true,
        }
    }
    if saw_unknown {
        Err(SparseBareissError::UndecidedPivot { pivot })
    } else {
        Ok(None)
    }
}

fn sparse_back_substitution(
    rows: &[BTreeMap<usize, Real>],
    rhs: &[Real],
    min_precision: i32,
) -> Result<Vec<Real>, SparseBareissError> {
    let mut solution = vec![Real::zero(); rows.len()];
    for row in (0..rows.len()).rev() {
        let pivot = rows[row]
            .get(&row)
            .ok_or(SparseBareissError::Singular { pivot: row })?;
        match pivot.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Negative | RealSign::Positive,
                ..
            } => {}
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => return Err(SparseBareissError::Singular { pivot: row }),
            CertifiedRealSign::Unknown { .. } => {
                return Err(SparseBareissError::UndecidedPivot { pivot: row });
            }
        }
        let trailing_sum = rows[row]
            .iter()
            .filter(|(column, _)| **column > row)
            .fold(Real::zero(), |sum, (column, coefficient)| {
                sum + coefficient.clone() * solution[*column].clone()
            });
        solution[row] = ((rhs[row].clone() - trailing_sum) / pivot.clone())
            .map_err(|_| SparseBareissError::UnsupportedSolutionDivision { row })?;
    }
    Ok(solution)
}

fn is_certified_zero(value: &Real, min_precision: i32) -> Result<bool, SparseBareissError> {
    match value.certified_sign_until(min_precision) {
        CertifiedRealSign::Known {
            sign: RealSign::Zero,
            ..
        } => Ok(true),
        CertifiedRealSign::Known { .. } => Ok(false),
        CertifiedRealSign::Unknown { .. } => Err(SparseBareissError::UndecidedPivot { pivot: 0 }),
    }
}

fn certified_sign(value: &Real, min_precision: i32) -> Result<RealSign, BareissError> {
    match value.certified_sign_until(min_precision) {
        CertifiedRealSign::Known { sign, .. } => Ok(sign),
        CertifiedRealSign::Unknown { .. } => Err(BareissError::UndecidedPivot { pivot: 0 }),
    }
}

fn certified_sign_with_policy(
    value: &Real,
    min_precision: i32,
    policy: PredicatePolicy,
) -> Result<(RealSign, PredicateCertainty), BareissError> {
    if let CertifiedRealSign::Known { sign, .. } = value.certified_sign_until(min_precision) {
        return Ok((sign, PredicateCertainty::Exact));
    }
    match hyperlimit::classify_real_sign(value, policy) {
        PredicateOutcome::Decided {
            value, certainty, ..
        } => Ok((
            match value {
                Sign::Negative => RealSign::Negative,
                Sign::Zero => RealSign::Zero,
                Sign::Positive => RealSign::Positive,
            },
            certainty,
        )),
        PredicateOutcome::Unknown { .. } => Err(BareissError::UndecidedPivot { pivot: 0 }),
    }
}

fn nonzero_determinant_certainty(
    determinant: &Real,
    min_precision: i32,
    policy: PredicatePolicy,
    pivot: usize,
) -> Result<PredicateCertainty, BareissError> {
    match certified_sign_with_policy(determinant, min_precision, policy) {
        Ok((RealSign::Zero, _)) => Err(BareissError::Singular { pivot }),
        Ok((RealSign::Negative | RealSign::Positive, certainty)) => Ok(certainty),
        Err(BareissError::UndecidedPivot { .. }) => Err(BareissError::UndecidedPivot { pivot }),
        Err(error) => Err(error),
    }
}

fn quotient_after_nonzero(numerator: Real, denominator: &Real) -> Result<Real, hyperreal::Problem> {
    denominator
        .inverse_ref_assuming_nonzero()
        .map(|inverse| numerator * inverse)
}

fn map_dense_replay_error(error: crate::residual_replay::DenseResidualReplayError) -> BareissError {
    match error {
        crate::residual_replay::DenseResidualReplayError::DimensionMismatch => {
            BareissError::DimensionMismatch
        }
        crate::residual_replay::DenseResidualReplayError::UnknownResidual => {
            BareissError::UnknownResidual
        }
    }
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::*;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    fn fraction(numerator: i64, denominator: u64) -> Real {
        Real::from(hyperreal::Rational::fraction(numerator, denominator).unwrap())
    }

    fn arrowhead_system(order: usize) -> (Vec<SparseResidualTerm>, Vec<Real>) {
        let mut terms = Vec::with_capacity(order.saturating_mul(3));
        terms.push(SparseResidualTerm {
            row: 0,
            column: 0,
            coefficient: real(order as i64 + 1),
        });
        for index in 1..order {
            terms.extend([
                SparseResidualTerm {
                    row: 0,
                    column: index,
                    coefficient: real(1),
                },
                SparseResidualTerm {
                    row: index,
                    column: 0,
                    coefficient: real(1),
                },
                SparseResidualTerm {
                    row: index,
                    column: index,
                    coefficient: real(2),
                },
            ]);
        }
        let mut rhs = vec![real(3); order];
        if order > 0 {
            rhs[0] = real((2 * order) as i64);
        }
        (terms, rhs)
    }

    #[test]
    fn bareiss_determinant_reports_swaps_and_exact_value() {
        let report =
            determinant_bareiss(&[vec![real(0), real(2)], vec![real(3), real(4)]], -64).unwrap();

        assert_eq!(report.determinant, real(-6));
        assert_eq!(report.swaps, 1);
        assert_eq!(report.pivots.len(), 1);
        assert_eq!(report.pivots[0].row, 1);
    }

    #[test]
    fn bareiss_determinant_retains_general_rational_fallback() {
        let report = determinant_bareiss(
            &[
                vec![fraction(1, 2), fraction(1, 3)],
                vec![fraction(2, 5), fraction(3, 7)],
            ],
            -64,
        )
        .unwrap();

        assert_eq!(report.determinant, fraction(17, 210));
    }

    #[test]
    fn pivot_free_determinant_matches_closed_forms_through_order_three() {
        assert_eq!(pivot_free_determinant(&[]).determinant, real(1));
        assert_eq!(
            pivot_free_determinant(&[vec![real(7)]]).determinant,
            real(7)
        );
        assert_eq!(
            pivot_free_determinant(&[vec![real(2), real(3)], vec![real(5), real(7)]]).determinant,
            real(-1)
        );
        assert_eq!(
            pivot_free_determinant(&[
                vec![real(1), real(2), real(3)],
                vec![real(0), real(4), real(5)],
                vec![real(1), real(0), real(6)],
            ])
            .determinant,
            real(22)
        );
    }

    #[test]
    fn bareiss_determinant_uses_pivot_free_exact_fallback_before_policy_decision() {
        let [_lower, upper] = Real::pi()
            .certified_dyadic_interval(-256)
            .expect("pi exposes certified dyadic intervals");
        let delayed_positive = Real::from(upper) - Real::pi();
        assert!(matches!(
            delayed_positive.certified_sign_until(-128),
            CertifiedRealSign::Unknown { .. }
        ));
        let matrix = [
            vec![delayed_positive.clone(), Real::zero()],
            vec![Real::zero(), Real::one()],
        ];

        let determinant = determinant_bareiss(&matrix, -128).unwrap();
        assert_eq!(
            determinant.method,
            BareissDeterminantMethod::PivotFreeFaddeevLeverrier
        );
        let solve = solve_dense_linear_system_bareiss(
            &matrix,
            &[Real::zero(), Real::zero()],
            -128,
            PredicatePolicy::STRICT,
        )
        .unwrap();
        assert!(solve.residual_replay.accepted);
        assert_eq!(solve.certainty, PredicateCertainty::Exact);
        assert_eq!(
            solve.determinant.method,
            BareissDeterminantMethod::FractionFree
        );
        assert_eq!(solve.solution, vec![Real::zero(), Real::zero()]);
        assert!(matches!(
            hyperlimit::classify_real_sign(&determinant.determinant, PredicatePolicy::STRICT),
            PredicateOutcome::Decided {
                value: Sign::Positive,
                certainty: PredicateCertainty::Exact,
                ..
            }
        ));
    }

    #[test]
    fn bareiss_solve_obeys_terminal_residual_policy() {
        let matrix = [vec![real(1), real(1)], vec![real(0), real(1)]];
        let rhs = [Real::pi() + Real::e(), Real::pi()];
        assert_eq!(
            solve_dense_linear_system_bareiss(&matrix, &rhs, -128, PredicatePolicy::STRICT,)
                .unwrap_err(),
            BareissError::UnknownResidual
        );

        let approximate = solve_dense_linear_system_bareiss(
            &matrix,
            &rhs,
            -128,
            PredicatePolicy::APPROXIMATE_512,
        )
        .unwrap();
        assert!(approximate.residual_replay.accepted);
        assert_eq!(approximate.certainty, PredicateCertainty::Approximate);
    }

    #[test]
    fn bareiss_solve_replays_exact_solution() {
        let report = solve_dense_linear_system_bareiss(
            &[vec![real(2), real(1)], vec![real(1), real(-1)]],
            &[real(5), real(1)],
            -64,
            PredicatePolicy::STRICT,
        )
        .unwrap();

        assert_eq!(report.solution, vec![real(2), real(1)]);
        assert_eq!(report.determinant.determinant, real(-3));
        assert_eq!(report.numerators, vec![real(-6), real(-3)]);
        assert!(report.residual_replay.accepted);
        assert_eq!(
            report,
            solve_dense_linear_system_bareiss_cramer(
                &[vec![real(2), real(1)], vec![real(1), real(-1)]],
                &[real(5), real(1)],
                -64,
                PredicatePolicy::STRICT,
            )
            .unwrap()
        );
    }

    #[test]
    fn bareiss_solve_swaps_augmented_rhs_with_the_pivot_row() {
        let report = solve_dense_linear_system_bareiss(
            &[vec![real(0), real(2)], vec![real(3), real(4)]],
            &[real(4), real(11)],
            -64,
            PredicatePolicy::STRICT,
        )
        .unwrap();

        assert_eq!(report.solution, vec![real(1), real(2)]);
        assert_eq!(report.determinant.determinant, real(-6));
        assert_eq!(report.numerators, vec![real(-6), real(-12)]);
        assert_eq!(report.determinant.swaps, 1);
        assert!(report.residual_replay.accepted);
    }

    #[test]
    fn bareiss_multi_rhs_solve_shares_elimination_and_replays_each_solution() {
        let matrix = [vec![real(2), real(1)], vec![real(1), real(-1)]];
        let right_hand_sides = [vec![real(5), real(1)], vec![real(0), real(3)]];
        let report = solve_dense_linear_system_bareiss_multi_rhs(
            &matrix,
            &right_hand_sides,
            -64,
            PredicatePolicy::STRICT,
        )
        .unwrap();

        assert_eq!(
            report.solutions,
            vec![vec![real(2), real(1)], vec![real(1), real(-2)]]
        );
        assert_eq!(report.determinant.determinant, real(-3));
        assert_eq!(
            report.numerators,
            vec![vec![real(-6), real(-3)], vec![real(-3), real(6)]]
        );
        assert!(report.residual_replays.iter().all(|replay| replay.accepted));
        for (index, rhs) in right_hand_sides.iter().enumerate() {
            let single =
                solve_dense_linear_system_bareiss(&matrix, rhs, -64, PredicatePolicy::STRICT)
                    .unwrap();
            assert_eq!(report.solutions[index], single.solution);
            assert_eq!(report.numerators[index], single.numerators);
            assert_eq!(report.residual_replays[index], single.residual_replay);
        }
    }

    #[test]
    fn bareiss_multi_rhs_solve_swaps_every_augmented_column() {
        let report = solve_dense_linear_system_bareiss_multi_rhs(
            &[vec![real(0), real(2)], vec![real(3), real(4)]],
            &[vec![real(4), real(11)], vec![real(6), real(15)]],
            -64,
            PredicatePolicy::STRICT,
        )
        .unwrap();

        assert_eq!(
            report.solutions,
            vec![vec![real(1), real(2)], vec![real(1), real(3)]]
        );
        assert_eq!(report.determinant.swaps, 1);
        assert!(report.residual_replays.iter().all(|replay| replay.accepted));
    }

    #[test]
    fn bareiss_rejects_bad_shapes_and_singular_solves() {
        assert_eq!(
            determinant_bareiss(&[vec![real(1), real(2)]], -64).unwrap_err(),
            BareissError::DimensionMismatch
        );
        assert_eq!(
            solve_dense_linear_system_bareiss(
                &[vec![real(1)]],
                &[real(1), real(2)],
                -64,
                PredicatePolicy::STRICT,
            )
            .unwrap_err(),
            BareissError::DimensionMismatch
        );
        assert_eq!(
            solve_dense_linear_system_bareiss_multi_rhs(
                &[vec![real(1)]],
                &[vec![real(1), real(2)]],
                -64,
                PredicatePolicy::STRICT,
            )
            .unwrap_err(),
            BareissError::DimensionMismatch
        );
        assert_eq!(
            solve_dense_linear_system_bareiss(
                &[vec![real(1), real(2)], vec![real(2), real(4)]],
                &[real(1), real(2)],
                -64,
                PredicatePolicy::STRICT,
            )
            .unwrap_err(),
            BareissError::Singular { pivot: 1 }
        );
    }

    #[test]
    fn sparse_bareiss_solve_accumulates_terms_and_replays_sparse_input() {
        let report = solve_sparse_linear_system_bareiss(
            2,
            2,
            &[
                SparseResidualTerm {
                    row: 0,
                    column: 0,
                    coefficient: real(1),
                },
                SparseResidualTerm {
                    row: 0,
                    column: 0,
                    coefficient: real(1),
                },
                SparseResidualTerm {
                    row: 0,
                    column: 1,
                    coefficient: real(1),
                },
                SparseResidualTerm {
                    row: 1,
                    column: 0,
                    coefficient: real(1),
                },
                SparseResidualTerm {
                    row: 1,
                    column: 1,
                    coefficient: real(-1),
                },
            ],
            &[real(5), real(1)],
            -64,
        )
        .unwrap();

        assert_eq!(report.dense_solve.solution, vec![real(2), real(1)]);
        assert_eq!(
            report.dense_matrix,
            vec![vec![real(2), real(1)], vec![real(1), real(-1)]]
        );
        assert!(report.sparse_residual_replay.accepted);
    }

    #[test]
    fn sparse_pattern_preserving_bareiss_tracks_fill_and_replays_solution() {
        let terms = vec![
            SparseResidualTerm {
                row: 0,
                column: 0,
                coefficient: real(1),
            },
            SparseResidualTerm {
                row: 0,
                column: 0,
                coefficient: real(1),
            },
            SparseResidualTerm {
                row: 0,
                column: 2,
                coefficient: real(1),
            },
            SparseResidualTerm {
                row: 1,
                column: 0,
                coefficient: real(1),
            },
            SparseResidualTerm {
                row: 1,
                column: 1,
                coefficient: real(1),
            },
            SparseResidualTerm {
                row: 2,
                column: 0,
                coefficient: real(1),
            },
            SparseResidualTerm {
                row: 2,
                column: 0,
                coefficient: real(-1),
            },
            SparseResidualTerm {
                row: 2,
                column: 2,
                coefficient: real(3),
            },
        ];
        let report = solve_sparse_linear_system_bareiss_pattern_preserving(
            3,
            3,
            &terms,
            &[real(4), real(3), real(6)],
            -64,
        )
        .unwrap();

        assert_eq!(report.solution, vec![real(1), real(2), real(2)]);
        assert!(report.fill_in_positions.contains(&(1, 2)));
        assert!(report.symbolic_pattern.fill_in_positions.contains(&(1, 2)));
        assert!(report.sparse_residual_replay.accepted);
        assert_eq!(
            report.sparse_residual_replay,
            replay_sparse_linear_residuals(
                3,
                3,
                &terms,
                &[real(4), real(3), real(6)],
                &report.solution,
                -64,
            )
            .unwrap()
        );
        assert_eq!(report.upper_rows.len(), 3);
    }

    #[test]
    fn sparse_pattern_preserving_bareiss_rejects_singular_and_bad_terms() {
        assert_eq!(
            solve_sparse_linear_system_bareiss_pattern_preserving(
                1,
                1,
                &[SparseResidualTerm {
                    row: 0,
                    column: 1,
                    coefficient: real(1),
                }],
                &[real(0)],
                -64,
            )
            .unwrap_err(),
            SparseBareissError::Pattern(SparsePatternError::TermOutOfBounds { row: 0, column: 1 })
        );
        assert_eq!(
            solve_sparse_linear_system_bareiss_pattern_preserving(
                2,
                2,
                &[SparseResidualTerm {
                    row: 0,
                    column: 0,
                    coefficient: real(1),
                }],
                &[real(1), real(0)],
                -64,
            )
            .unwrap_err(),
            SparseBareissError::UncertifiedPattern
        );
    }

    #[test]
    fn minimum_degree_sparse_bareiss_reduces_arrowhead_fill_and_replays_source_order() {
        let order = 8;
        let (terms, rhs) = arrowhead_system(order);
        let authored =
            solve_sparse_linear_system_bareiss_pattern_preserving(order, order, &terms, &rhs, -64)
                .unwrap();
        let reordered =
            solve_sparse_linear_system_bareiss_minimum_degree(order, order, &terms, &rhs, -64)
                .unwrap();

        assert!(
            reordered
                .permuted_to_source
                .iter()
                .position(|source| *source == 0)
                .is_some_and(|position| position >= order - 2)
        );
        assert_eq!(reordered.solution, vec![real(1); order]);
        assert!(reordered.source_residual_replay.accepted);
        assert_eq!(
            reordered.source_residual_replay,
            replay_sparse_linear_residuals(order, order, &terms, &rhs, &reordered.solution, -64)
                .unwrap()
        );
        assert!(
            reordered.permuted_solve.fill_in_positions.len() < authored.fill_in_positions.len()
        );
    }

    #[test]
    fn sparse_bareiss_rejects_bad_shapes_bounds_and_singular_systems() {
        assert_eq!(
            solve_sparse_linear_system_bareiss(1, 2, &[], &[real(0)], -64).unwrap_err(),
            SparseBareissError::DimensionMismatch
        );
        assert_eq!(
            solve_sparse_linear_system_bareiss(
                1,
                1,
                &[SparseResidualTerm {
                    row: 0,
                    column: 1,
                    coefficient: real(1),
                }],
                &[real(0)],
                -64,
            )
            .unwrap_err(),
            SparseBareissError::TermOutOfBounds { row: 0, column: 1 }
        );
        assert_eq!(
            solve_sparse_linear_system_bareiss(
                2,
                2,
                &[
                    SparseResidualTerm {
                        row: 0,
                        column: 0,
                        coefficient: real(1),
                    },
                    SparseResidualTerm {
                        row: 1,
                        column: 0,
                        coefficient: real(2),
                    },
                ],
                &[real(1), real(2)],
                -64,
            )
            .unwrap_err(),
            SparseBareissError::DenseSolve(BareissError::Singular { pivot: 1 })
        );
    }

    proptest! {
        #[test]
        fn generated_triangular_determinants_match_diagonal_product(
            a in 1_i16..=16,
            b in 1_i16..=16,
            c in -16_i16..=16,
        ) {
            let a = i64::from(a);
            let b = i64::from(b);
            let c = i64::from(c);
            let report = determinant_bareiss(
                &[vec![real(a), real(c)], vec![Real::zero(), real(b)]],
                -64,
            ).unwrap();

            prop_assert_eq!(report.determinant, real(a * b));
        }

        #[test]
        fn generated_diagonal_systems_solve_and_replay_exactly(
            a in 1_i16..=16,
            b in 1_i16..=16,
            x in -32_i16..=32,
            y in -32_i16..=32,
        ) {
            let a = i64::from(a);
            let b = i64::from(b);
            let x = i64::from(x);
            let y = i64::from(y);
            let report = solve_dense_linear_system_bareiss(
                &[vec![real(a), Real::zero()], vec![Real::zero(), real(b)]],
                &[real(a * x), real(b * y)],
                -64,
                PredicatePolicy::STRICT,
            ).unwrap();

            prop_assert_eq!(report.solution, vec![real(x), real(y)]);
            prop_assert!(report.residual_replay.accepted);
        }

        #[test]
        fn generated_nonsingular_two_by_two_systems_solve_and_replay_exactly(
            a in -8_i16..=8,
            b in -8_i16..=8,
            c in -8_i16..=8,
            d in -8_i16..=8,
            x in -16_i16..=16,
            y in -16_i16..=16,
        ) {
            let a = i64::from(a);
            let b = i64::from(b);
            let c = i64::from(c);
            let d = i64::from(d);
            let x = i64::from(x);
            let y = i64::from(y);
            let determinant = a * d - b * c;
            prop_assume!(determinant != 0);

            let report = solve_dense_linear_system_bareiss(
                &[vec![real(a), real(b)], vec![real(c), real(d)]],
                &[real(a * x + b * y), real(c * x + d * y)],
                -64,
                PredicatePolicy::STRICT,
            ).unwrap();

            prop_assert_eq!(report.solution, vec![real(x), real(y)]);
            prop_assert_eq!(report.determinant.determinant, real(determinant));
            prop_assert_eq!(
                report.numerators,
                vec![real(determinant * x), real(determinant * y)]
            );
            prop_assert!(report.residual_replay.accepted);
        }

        #[test]
        fn generated_sparse_diagonal_systems_solve_and_replay_exactly(
            a in 1_i16..=16,
            b in 1_i16..=16,
            x in -32_i16..=32,
            y in -32_i16..=32,
        ) {
            let a = i64::from(a);
            let b = i64::from(b);
            let x = i64::from(x);
            let y = i64::from(y);
            let report = solve_sparse_linear_system_bareiss(
                2,
                2,
                &[
                    SparseResidualTerm { row: 0, column: 0, coefficient: real(a) },
                    SparseResidualTerm { row: 1, column: 1, coefficient: real(b) },
                ],
                &[real(a * x), real(b * y)],
                -64,
            ).unwrap();

            prop_assert_eq!(report.dense_solve.solution, vec![real(x), real(y)]);
            prop_assert!(report.sparse_residual_replay.accepted);
        }

        #[test]
        fn generated_pattern_preserving_diagonal_systems_solve_and_replay_exactly(
            a in 1_i16..=16,
            b in 1_i16..=16,
            x in -32_i16..=32,
            y in -32_i16..=32,
        ) {
            let a = i64::from(a);
            let b = i64::from(b);
            let x = i64::from(x);
            let y = i64::from(y);
            let report = solve_sparse_linear_system_bareiss_pattern_preserving(
                2,
                2,
                &[
                    SparseResidualTerm { row: 0, column: 0, coefficient: real(a) },
                    SparseResidualTerm { row: 1, column: 1, coefficient: real(b) },
                ],
                &[real(a * x), real(b * y)],
                -64,
            ).unwrap();

            prop_assert_eq!(report.solution, vec![real(x), real(y)]);
            prop_assert!(report.fill_in_positions.is_empty());
            prop_assert!(report.sparse_residual_replay.accepted);
        }
    }
}
