//! Fraction-free exact dense linear algebra.
//!
//! This module is a small exact counterpart to the primitive-float linear
//! adapter in [`crate::linalg`]. It uses the Bareiss fraction-free elimination
//! recurrence to keep intermediate values exact while delaying normalization:
//! `a'ij = (pivot * aij - aik * akj) / previous_pivot`. See E. H. Bareiss,
//! "Sylvester's Identity and Multistep Integer-Preserving Gaussian
//! Elimination," *Mathematics of Computation* 22.103 (1968). In Yap's terms,
//! this remains in the exact object layer: numerical solvers may propose, but
//! exact arithmetic and certified sign decisions own the proof boundary. See
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7.1-2 (1997).

use hyperreal::{CertifiedRealSign, Real, RealSign};

use crate::residual_replay::{DenseResidualReplayReport, replay_dense_linear_residuals};

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

/// Computes an exact determinant with Bareiss fraction-free elimination.
///
/// Pivot choices are certified through [`Real::certified_sign_until`]. A
/// certified zero determinant is returned as a successful report with
/// `determinant == 0`; undecidable pivot signs and unsupported exact divisions
/// are explicit errors because they would otherwise blur Yap's exact decision
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
        pivots.push(BareissPivot {
            pivot,
            row: pivot_row,
            value: pivot_value.clone(),
        });

        for row in (pivot + 1)..n {
            for column in (pivot + 1)..n {
                let numerator = pivot_value.clone() * work[row][column].clone()
                    - work[row][pivot].clone() * work[pivot][column].clone();
                work[row][column] = (numerator / previous_pivot.clone())
                    .map_err(|_| BareissError::UnsupportedDivision { pivot })?;
            }
        }

        for row in (pivot + 1)..n {
            work[row][pivot] = Real::zero();
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

fn select_pivot_row(
    matrix: &[Vec<Real>],
    pivot: usize,
    min_precision: i32,
) -> Result<Option<usize>, BareissError> {
    let mut saw_unknown = false;
    for row in pivot..matrix.len() {
        match certified_sign(&matrix[row][pivot], min_precision) {
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
    }
}
