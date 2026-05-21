//! Exact affine rank and degree-of-freedom diagnostics.
//!
//! This module is deliberately diagnostic, not a nonlinear solver. It consumes
//! retained affine residual rows and computes exact coefficient and augmented
//! matrix ranks through Bareiss determinants. That keeps rank/DOF claims out of
//! the lossy dense adapter path, following Yap's exact/approximate boundary;
//! see C. K. Yap, "Towards Exact Geometric Computation" (1997). The
//! determinant kernel follows Bareiss, "Sylvester's Identity and Multistep
//! Integer-Preserving Gaussian Elimination" (1968).

use hyperreal::{CertifiedRealSign, Real, RealSign};

use crate::bareiss::{BareissError, determinant_bareiss};
use crate::model::ConstraintKind;
use crate::prepared::PreparedProblem;

/// Status for exact affine rank diagnostics.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ExactAffineRankStatus {
    /// Coefficient and augmented ranks were certified.
    Certified,
    /// The augmented rank is larger than the coefficient rank.
    Inconsistent,
    /// At least one active equality-like row was not affine.
    UnsupportedNonAffineRows,
    /// A determinant sign or exact Bareiss step could not be decided.
    Undecided,
}

/// Exact affine rank/DOF report for active equality-like rows.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactAffineRankReport {
    /// Final diagnostic status.
    pub status: ExactAffineRankStatus,
    /// Number of source variables.
    pub variable_count: usize,
    /// Number of active equality-like rows consumed.
    pub equality_row_count: usize,
    /// Active non-equality rows skipped by this equality-rank diagnostic.
    pub skipped_non_equality_rows: usize,
    /// Active equality-like rows that were not prepared affine rows.
    pub unsupported_rows: Vec<usize>,
    /// Certified rank of the coefficient matrix, when available.
    pub coefficient_rank: Option<usize>,
    /// Certified rank of the augmented matrix `[A | c]`, when available.
    pub augmented_rank: Option<usize>,
    /// Degrees of freedom `variable_count - coefficient_rank`, when available.
    pub degrees_of_freedom: Option<usize>,
    /// Bareiss error that blocked a rank decision.
    pub error: Option<BareissError>,
}

/// Compute exact affine coefficient/augmented ranks for a prepared problem.
///
/// Active equality and soft rows are treated as equality-like equations.
/// Active inequalities are skipped because their rank contribution depends on
/// active-set certification. Non-affine equality rows are reported explicitly
/// rather than lowered through a lossy Jacobian rank hint.
pub fn analyze_exact_affine_rank(
    prepared: &PreparedProblem<'_>,
    min_precision: i32,
) -> ExactAffineRankReport {
    let mut coefficients = Vec::new();
    let mut augmented = Vec::new();
    let mut unsupported_rows = Vec::new();
    let mut skipped_non_equality_rows = 0_usize;

    for (row_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active {
            continue;
        }
        if !matches!(
            constraint.kind,
            ConstraintKind::Equality | ConstraintKind::Soft
        ) {
            skipped_non_equality_rows += 1;
            continue;
        }
        let Some(affine) = &prepared.affine_residuals()[row_index] else {
            unsupported_rows.push(row_index);
            continue;
        };
        let row = affine.coefficients().to_vec();
        let mut augmented_row = row.clone();
        augmented_row.push(affine.constant().clone());
        coefficients.push(row);
        augmented.push(augmented_row);
    }

    if !unsupported_rows.is_empty() {
        return ExactAffineRankReport {
            status: ExactAffineRankStatus::UnsupportedNonAffineRows,
            variable_count: prepared.problem().variables.len(),
            equality_row_count: coefficients.len(),
            skipped_non_equality_rows,
            unsupported_rows,
            coefficient_rank: None,
            augmented_rank: None,
            degrees_of_freedom: None,
            error: None,
        };
    }

    let coefficient_rank = match exact_matrix_rank(&coefficients, min_precision) {
        Ok(rank) => rank,
        Err(error) => {
            return undecided_report(
                prepared,
                coefficients.len(),
                skipped_non_equality_rows,
                error,
            );
        }
    };
    let augmented_rank = match exact_matrix_rank(&augmented, min_precision) {
        Ok(rank) => rank,
        Err(error) => {
            return undecided_report(
                prepared,
                coefficients.len(),
                skipped_non_equality_rows,
                error,
            );
        }
    };
    let degrees_of_freedom = prepared
        .problem()
        .variables
        .len()
        .saturating_sub(coefficient_rank);
    ExactAffineRankReport {
        status: if augmented_rank > coefficient_rank {
            ExactAffineRankStatus::Inconsistent
        } else {
            ExactAffineRankStatus::Certified
        },
        variable_count: prepared.problem().variables.len(),
        equality_row_count: coefficients.len(),
        skipped_non_equality_rows,
        unsupported_rows,
        coefficient_rank: Some(coefficient_rank),
        augmented_rank: Some(augmented_rank),
        degrees_of_freedom: Some(degrees_of_freedom),
        error: None,
    }
}

fn undecided_report(
    prepared: &PreparedProblem<'_>,
    equality_row_count: usize,
    skipped_non_equality_rows: usize,
    error: BareissError,
) -> ExactAffineRankReport {
    ExactAffineRankReport {
        status: ExactAffineRankStatus::Undecided,
        variable_count: prepared.problem().variables.len(),
        equality_row_count,
        skipped_non_equality_rows,
        unsupported_rows: Vec::new(),
        coefficient_rank: None,
        augmented_rank: None,
        degrees_of_freedom: None,
        error: Some(error),
    }
}

fn exact_matrix_rank(matrix: &[Vec<Real>], min_precision: i32) -> Result<usize, BareissError> {
    let row_count = matrix.len();
    let column_count = matrix.first().map(Vec::len).unwrap_or(0);
    let max_rank = row_count.min(column_count);
    if max_rank == 0 {
        return Ok(0);
    }

    for size in (1..=max_rank).rev() {
        let row_combinations = combinations(row_count, size);
        let column_combinations = combinations(column_count, size);
        for rows in &row_combinations {
            for columns in &column_combinations {
                let minor = minor_matrix(matrix, rows, columns);
                let determinant = determinant_bareiss(&minor, min_precision)?;
                match determinant.determinant.certified_sign_until(min_precision) {
                    CertifiedRealSign::Known {
                        sign: RealSign::Negative | RealSign::Positive,
                        ..
                    } => return Ok(size),
                    CertifiedRealSign::Known {
                        sign: RealSign::Zero,
                        ..
                    } => {}
                    CertifiedRealSign::Unknown { .. } => {
                        return Err(BareissError::UndecidedPivot { pivot: size - 1 });
                    }
                }
            }
        }
    }
    Ok(0)
}

fn minor_matrix(matrix: &[Vec<Real>], rows: &[usize], columns: &[usize]) -> Vec<Vec<Real>> {
    rows.iter()
        .map(|row| {
            columns
                .iter()
                .map(|column| matrix[*row][*column].clone())
                .collect()
        })
        .collect()
}

fn combinations(width: usize, size: usize) -> Vec<Vec<usize>> {
    if size == 0 {
        return vec![Vec::new()];
    }
    if size > width {
        return Vec::new();
    }
    let mut result = Vec::new();
    let mut current = Vec::with_capacity(size);
    push_combinations(0, width, size, &mut current, &mut result);
    result
}

fn push_combinations(
    start: usize,
    width: usize,
    size: usize,
    current: &mut Vec<usize>,
    result: &mut Vec<Vec<usize>>,
) {
    if current.len() == size {
        result.push(current.clone());
        return;
    }
    let remaining = size - current.len();
    for value in start..=width - remaining {
        current.push(value);
        push_combinations(value + 1, width, size, current, result);
        current.pop();
    }
}
