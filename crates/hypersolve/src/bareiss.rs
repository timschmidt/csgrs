//! Fraction-free exact dense linear algebra.
//!
//! This module is a small exact counterpart to the primitive-float linear
//! adapter in [`crate::linalg`]. It uses the Bareiss fraction-free elimination
//! recurrence to keep intermediate values exact while delaying normalization:
//! `a'ij = (pivot * aij - aik * akj) / previous_pivot`. This remains in the
//! exact object layer: numerical solvers may propose, but exact arithmetic and
//! certified sign decisions own the proof boundary.

use std::collections::{BTreeMap, BTreeSet};

use hyperreal::{CertifiedRealSign, Real, RealSign};

use crate::residual_replay::{
    DenseResidualReplayReport, SparseResidualReplayError, SparseResidualReplayReport,
    SparseResidualTerm, replay_dense_linear_residuals, replay_sparse_linear_residuals,
};
use crate::sparse_pattern::{
    SparsePatternError, SymbolicSparseFactorizationReport,
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

/// Exact determinant report produced by fraction-free Bareiss elimination.
#[derive(Clone, Debug, PartialEq)]
pub struct BareissDeterminantReport {
    /// Exact determinant of the input matrix.
    pub determinant: Real,
    /// Number of row swaps applied during elimination.
    pub swaps: usize,
    /// Certified pivots used before the final determinant value.
    pub pivots: Vec<BareissPivot>,
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
    /// Exact replay against the original sparse representation.
    pub sparse_residual_replay: SparseResidualReplayReport,
}

/// Computes an exact determinant with Bareiss fraction-free elimination.
///
/// Pivot choices are certified through [`Real::certified_sign_until`]. A
/// certified zero determinant is returned as a successful report with
/// `determinant == 0`; undecidable pivot signs and unsupported exact divisions
/// are explicit errors because they would otherwise blur the exact decision
/// boundary.
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
        });
    }
    if n == 1 {
        return Ok(BareissDeterminantReport {
            determinant: matrix[0][0].clone(),
            swaps: 0,
            pivots: Vec::new(),
        });
    }

    let mut work = matrix.to_vec();
    let mut swaps = 0;
    let mut pivots = Vec::with_capacity(n - 1);
    let mut previous_pivot = Real::one();

    for pivot in 0..(n - 1) {
        let pivot_row = select_pivot_row(&work, pivot, min_precision)?;
        let Some(pivot_row) = pivot_row else {
            return Ok(BareissDeterminantReport {
                determinant: Real::zero(),
                swaps,
                pivots,
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
                let numerator = pivot_value.clone() * row[column].clone()
                    - row[pivot].clone() * pivot_work_row[column].clone();
                row[column] = (numerator / previous_pivot.clone())
                    .map_err(|_| BareissError::UnsupportedDivision { pivot })?;
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
    })
}

/// Solves a square dense linear system exactly with Bareiss determinants.
///
/// The implementation uses Bareiss determinants inside Cramer's rule. That is
/// intentionally conservative: it gives callers a compact exact solve surface
/// while keeping production sparse or iterative solving outside this module.
/// The returned candidate is immediately replayed through
/// [`crate::replay_dense_linear_residuals`] so the report carries both the
/// construction evidence and the exact `A*x-b` proof.
pub fn solve_dense_linear_system_bareiss(
    matrix: &[Vec<Real>],
    rhs: &[Real],
    min_precision: i32,
) -> Result<BareissSolveReport, BareissError> {
    if matrix.len() != rhs.len() || matrix.iter().any(|row| row.len() != matrix.len()) {
        return Err(BareissError::DimensionMismatch);
    }

    let determinant = determinant_bareiss(matrix, min_precision)?;
    match certified_sign(&determinant.determinant, min_precision)? {
        RealSign::Zero => {
            return Err(BareissError::Singular {
                pivot: matrix.len().saturating_sub(1),
            });
        }
        RealSign::Negative | RealSign::Positive => {}
    }

    let mut numerators = Vec::with_capacity(matrix.len());
    let mut solution = Vec::with_capacity(matrix.len());
    for column in 0..matrix.len() {
        let replaced = replace_column(matrix, rhs, column);
        let numerator_report = determinant_bareiss(&replaced, min_precision)?;
        let numerator = numerator_report.determinant;
        let value = (numerator.clone() / determinant.determinant.clone())
            .map_err(|_| BareissError::UnsupportedSolutionDivision { column })?;
        numerators.push(numerator);
        solution.push(value);
    }

    let residual_replay = replay_dense_linear_residuals(matrix, rhs, &solution, min_precision)
        .map_err(|error| match error {
            crate::residual_replay::DenseResidualReplayError::DimensionMismatch => {
                BareissError::DimensionMismatch
            }
            crate::residual_replay::DenseResidualReplayError::UnknownResidual => {
                BareissError::UnknownResidual
            }
        })?;

    Ok(BareissSolveReport {
        solution,
        determinant,
        numerators,
        residual_replay,
    })
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

    let dense_solve = solve_dense_linear_system_bareiss(&dense_matrix, rhs, min_precision)
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

    let mut rows = accumulate_sparse_rows(row_count, column_count, terms, min_precision)?;
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
                let numerator =
                    pivot_value.clone() * row_value - eliminand.clone() * pivot_column_value;
                let updated = (numerator / previous_pivot.clone())
                    .map_err(|_| SparseBareissError::UnsupportedDivision { pivot })?;
                if is_certified_zero(&updated, min_precision)? {
                    rows[row_index].remove(&column);
                } else {
                    if !rows[row_index].contains_key(&column) {
                        fill_in_positions.insert((row_index, column));
                    }
                    rows[row_index].insert(column, updated);
                }
            }
            let rhs_numerator =
                pivot_value.clone() * rhs_work[row_index].clone() - eliminand * pivot_rhs.clone();
            rhs_work[row_index] = (rhs_numerator / previous_pivot.clone())
                .map_err(|_| SparseBareissError::UnsupportedDivision { pivot })?;
        }
        previous_pivot = pivot_value;
    }

    let solution = sparse_back_substitution(&rows, &rhs_work, min_precision)?;
    let sparse_residual_replay = replay_sparse_linear_residuals(
        row_count,
        column_count,
        terms,
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

fn accumulate_sparse_rows(
    row_count: usize,
    column_count: usize,
    terms: &[SparseResidualTerm],
    min_precision: i32,
) -> Result<Vec<BTreeMap<usize, Real>>, SparseBareissError> {
    let mut rows = vec![BTreeMap::<usize, Real>::new(); row_count];
    for term in terms {
        if term.row >= row_count || term.column >= column_count {
            return Err(SparseBareissError::TermOutOfBounds {
                row: term.row,
                column: term.column,
            });
        }
        rows[term.row]
            .entry(term.column)
            .and_modify(|value| *value = value.clone() + term.coefficient.clone())
            .or_insert_with(|| term.coefficient.clone());
    }
    for row in &mut rows {
        let mut zero_columns = Vec::new();
        for (column, value) in row.iter() {
            if is_certified_zero(value, min_precision)? {
                zero_columns.push(*column);
            }
        }
        for column in zero_columns {
            row.remove(&column);
        }
    }
    Ok(rows)
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

fn replace_column(matrix: &[Vec<Real>], rhs: &[Real], column: usize) -> Vec<Vec<Real>> {
    let mut replaced = matrix.to_vec();
    for (row, value) in rhs.iter().enumerate() {
        replaced[row][column] = value.clone();
    }
    replaced
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::*;

    fn real(value: i64) -> Real {
        Real::from(value)
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
    fn bareiss_solve_replays_exact_solution() {
        let report = solve_dense_linear_system_bareiss(
            &[vec![real(2), real(1)], vec![real(1), real(-1)]],
            &[real(5), real(1)],
            -64,
        )
        .unwrap();

        assert_eq!(report.solution, vec![real(2), real(1)]);
        assert_eq!(report.determinant.determinant, real(-3));
        assert_eq!(report.numerators, vec![real(-6), real(-3)]);
        assert!(report.residual_replay.accepted);
    }

    #[test]
    fn bareiss_rejects_bad_shapes_and_singular_solves() {
        assert_eq!(
            determinant_bareiss(&[vec![real(1), real(2)]], -64).unwrap_err(),
            BareissError::DimensionMismatch
        );
        assert_eq!(
            solve_dense_linear_system_bareiss(&[vec![real(1)]], &[real(1), real(2)], -64)
                .unwrap_err(),
            BareissError::DimensionMismatch
        );
        assert_eq!(
            solve_dense_linear_system_bareiss(
                &[vec![real(1), real(2)], vec![real(2), real(4)]],
                &[real(1), real(2)],
                -64,
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
        let report = solve_sparse_linear_system_bareiss_pattern_preserving(
            3,
            3,
            &[
                SparseResidualTerm {
                    row: 0,
                    column: 0,
                    coefficient: real(2),
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
                    column: 2,
                    coefficient: real(3),
                },
            ],
            &[real(4), real(3), real(6)],
            -64,
        )
        .unwrap();

        assert_eq!(report.solution, vec![real(1), real(2), real(2)]);
        assert!(report.fill_in_positions.contains(&(1, 2)));
        assert!(report.symbolic_pattern.fill_in_positions.contains(&(1, 2)));
        assert!(report.sparse_residual_replay.accepted);
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
            ).unwrap();

            prop_assert_eq!(report.solution, vec![real(x), real(y)]);
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
