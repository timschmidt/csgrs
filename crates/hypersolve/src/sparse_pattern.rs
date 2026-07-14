//! Exact sparse linear-system pattern audits.
//!
//! This module does not solve a sparse system. It preserves the sparse caller
//! shape and reports the symbolic elimination pattern that a future sparse
//! fraction-free backend would have to honor. The symbolic fill rule follows
//! sparse Gaussian-elimination graph ideas from fill-reducing sparse elimination, while keeping Bareiss-style exact solving and the exact
//! proof boundary separate: structural scheduling is evidence, not a numeric
//! certificate. See fraction-free elimination, and the exact-geometric-computation model.

use std::collections::{BTreeMap, BTreeSet};

use hyperreal::{CertifiedRealSign, Real, RealSign};

use crate::residual_replay::SparseResidualTerm;

/// Failure mode for exact sparse pattern analysis.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SparsePatternError {
    /// The declared matrix was not square.
    DimensionMismatch,
    /// A sparse term addressed a row or column outside the declared shape.
    TermOutOfBounds {
        /// Offending term row.
        row: usize,
        /// Offending term column.
        column: usize,
    },
}

/// Exact sign classification for an accumulated sparse entry.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SparsePatternEntryStatus {
    /// The accumulated entry is certified zero and omitted from the active
    /// structural pattern.
    CertifiedZero,
    /// The accumulated entry is certified nonzero and retained in the active
    /// structural pattern.
    CertifiedNonzero,
    /// The accumulated entry sign did not certify, so the pattern audit keeps
    /// it conservatively active.
    UnknownSign,
}

/// Exact accumulated sparse entry classified for pattern scheduling.
#[derive(Clone, Debug, PartialEq)]
pub struct SparsePatternEntry {
    /// Entry row.
    pub row: usize,
    /// Entry column.
    pub column: usize,
    /// Exact accumulated value for this coordinate.
    pub value: Real,
    /// Certified structural status for the entry.
    pub status: SparsePatternEntryStatus,
}

/// One symbolic elimination step in sparse Bareiss/Gaussian scheduling.
#[derive(Clone, Debug, PartialEq)]
pub struct SymbolicSparseEliminationStep {
    /// Pivot column.
    pub pivot: usize,
    /// Row selected as the pivot row.
    pub selected_row: Option<usize>,
    /// Whether a structural row swap was needed.
    pub row_swap: Option<(usize, usize)>,
    /// Status of the original accumulated pivot entry when known.
    pub pivot_status: Option<SparsePatternEntryStatus>,
    /// Rows whose pivot-column entry caused a symbolic elimination update.
    pub eliminated_rows: Vec<usize>,
    /// New symbolic fill entries introduced by this step.
    pub fill_in: Vec<(usize, usize)>,
    /// Active structural nonzero count after this step.
    pub active_nonzeros_after_step: usize,
}

/// Symbolic sparse factorization audit report.
#[derive(Clone, Debug, PartialEq)]
pub struct SymbolicSparseFactorizationReport {
    /// Declared row count.
    pub row_count: usize,
    /// Declared column count.
    pub column_count: usize,
    /// Exact accumulated entries, including certified zeros.
    pub entries: Vec<SparsePatternEntry>,
    /// Certified-nonzero structural positions before elimination.
    pub certified_nonzero_positions: Vec<(usize, usize)>,
    /// Unknown-sign positions retained conservatively in the structural
    /// pattern.
    pub unknown_positions: Vec<(usize, usize)>,
    /// Symbolic elimination steps.
    pub steps: Vec<SymbolicSparseEliminationStep>,
    /// Unique symbolic fill entries introduced by all steps.
    pub fill_in_positions: Vec<(usize, usize)>,
    /// Final active structural pattern after symbolic elimination.
    pub final_pattern: Vec<(usize, usize)>,
    /// True when a pivot column had no active structural pivot candidate.
    pub structurally_singular: bool,
}

impl SymbolicSparseFactorizationReport {
    /// Returns true when every accumulated active entry had a certified sign
    /// and the symbolic schedule found a pivot in every column.
    pub fn fully_certified_pattern(&self) -> bool {
        self.unknown_positions.is_empty() && !self.structurally_singular
    }
}

/// Analyze an exact sparse matrix's symbolic Bareiss elimination pattern.
///
/// Terms encode `A[row, column] += coefficient`, with duplicate terms
/// accumulated exactly before structural classification. Certified zero
/// entries are removed from the active pattern; unknown signs are retained
/// conservatively because dropping them would be an approximate decision. The
/// fill simulation uses the standard sparse-elimination pattern rule: when a
/// row is eliminated by a pivot row, the row's trailing structure is unioned
/// with the pivot row's trailing structure. This uses sparse triangular
/// factorization scheduling while leaving numeric fraction-free
/// updates to [`crate::solve_sparse_linear_system_bareiss`].
pub fn analyze_sparse_bareiss_elimination_pattern(
    row_count: usize,
    column_count: usize,
    terms: &[SparseResidualTerm],
    min_precision: i32,
) -> Result<SymbolicSparseFactorizationReport, SparsePatternError> {
    if row_count != column_count {
        return Err(SparsePatternError::DimensionMismatch);
    }

    let mut accumulated = BTreeMap::<(usize, usize), Real>::new();
    for term in terms {
        if term.row >= row_count || term.column >= column_count {
            return Err(SparsePatternError::TermOutOfBounds {
                row: term.row,
                column: term.column,
            });
        }
        accumulated
            .entry((term.row, term.column))
            .and_modify(|value| *value = value.clone() + term.coefficient.clone())
            .or_insert_with(|| term.coefficient.clone());
    }

    let mut entries = Vec::with_capacity(accumulated.len());
    let mut certified_nonzero_positions = Vec::new();
    let mut unknown_positions = Vec::new();
    let mut active = BTreeSet::<(usize, usize)>::new();
    let mut entry_status = BTreeMap::<(usize, usize), SparsePatternEntryStatus>::new();
    for ((row, column), value) in accumulated {
        let status = classify_entry(&value, min_precision);
        if status == SparsePatternEntryStatus::CertifiedNonzero {
            certified_nonzero_positions.push((row, column));
            active.insert((row, column));
        } else if status == SparsePatternEntryStatus::UnknownSign {
            unknown_positions.push((row, column));
            active.insert((row, column));
        }
        entry_status.insert((row, column), status.clone());
        entries.push(SparsePatternEntry {
            row,
            column,
            value,
            status,
        });
    }

    let mut steps = Vec::new();
    let mut fill_in_positions = BTreeSet::<(usize, usize)>::new();
    let mut structurally_singular = false;

    for pivot in 0..row_count {
        let selected_row = (pivot..row_count).find(|row| active.contains(&(*row, pivot)));
        let Some(selected_row) = selected_row else {
            structurally_singular = true;
            steps.push(SymbolicSparseEliminationStep {
                pivot,
                selected_row: None,
                row_swap: None,
                pivot_status: None,
                eliminated_rows: Vec::new(),
                fill_in: Vec::new(),
                active_nonzeros_after_step: active.len(),
            });
            continue;
        };

        let row_swap = if selected_row == pivot {
            None
        } else {
            swap_active_rows(&mut active, pivot, selected_row, row_count);
            Some((pivot, selected_row))
        };
        let pivot_status = entry_status
            .get(&(selected_row, pivot))
            .or_else(|| entry_status.get(&(pivot, pivot)))
            .cloned();

        let pivot_trailing = active
            .iter()
            .filter_map(|(row, column)| (*row == pivot && *column > pivot).then_some(*column))
            .collect::<BTreeSet<_>>();
        let eliminated_rows = ((pivot + 1)..row_count)
            .filter(|row| active.contains(&(*row, pivot)))
            .collect::<Vec<_>>();
        let mut step_fill = BTreeSet::<(usize, usize)>::new();
        for row in &eliminated_rows {
            active.remove(&(*row, pivot));
            for column in &pivot_trailing {
                if !active.contains(&(*row, *column)) {
                    active.insert((*row, *column));
                    step_fill.insert((*row, *column));
                    fill_in_positions.insert((*row, *column));
                }
            }
        }

        steps.push(SymbolicSparseEliminationStep {
            pivot,
            selected_row: Some(selected_row),
            row_swap,
            pivot_status,
            eliminated_rows,
            fill_in: step_fill.into_iter().collect(),
            active_nonzeros_after_step: active.len(),
        });
    }

    Ok(SymbolicSparseFactorizationReport {
        row_count,
        column_count,
        entries,
        certified_nonzero_positions,
        unknown_positions,
        steps,
        fill_in_positions: fill_in_positions.into_iter().collect(),
        final_pattern: active.into_iter().collect(),
        structurally_singular,
    })
}

fn classify_entry(value: &Real, min_precision: i32) -> SparsePatternEntryStatus {
    match value.certified_sign_until(min_precision) {
        CertifiedRealSign::Known {
            sign: RealSign::Zero,
            ..
        } => SparsePatternEntryStatus::CertifiedZero,
        CertifiedRealSign::Known { .. } => SparsePatternEntryStatus::CertifiedNonzero,
        CertifiedRealSign::Unknown { .. } => SparsePatternEntryStatus::UnknownSign,
    }
}

fn swap_active_rows(
    active: &mut BTreeSet<(usize, usize)>,
    left: usize,
    right: usize,
    column_count: usize,
) {
    for column in 0..column_count {
        let left_present = active.remove(&(left, column));
        let right_present = active.remove(&(right, column));
        if left_present {
            active.insert((right, column));
        }
        if right_present {
            active.insert((left, column));
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

    #[test]
    fn sparse_pattern_audit_reports_fill_and_zero_cancellation() {
        let report = analyze_sparse_bareiss_elimination_pattern(
            3,
            3,
            &[
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
                    coefficient: real(1),
                },
            ],
            -64,
        )
        .unwrap();

        assert!(report.fully_certified_pattern());
        assert!(report.fill_in_positions.contains(&(1, 2)));
        assert!(report.entries.iter().any(|entry| entry.row == 2
            && entry.column == 0
            && entry.status == SparsePatternEntryStatus::CertifiedZero));
    }

    #[test]
    fn sparse_pattern_audit_rejects_bad_shapes_and_bounds() {
        assert_eq!(
            analyze_sparse_bareiss_elimination_pattern(2, 3, &[], -64).unwrap_err(),
            SparsePatternError::DimensionMismatch
        );
        assert_eq!(
            analyze_sparse_bareiss_elimination_pattern(
                1,
                1,
                &[SparseResidualTerm {
                    row: 1,
                    column: 0,
                    coefficient: real(1),
                }],
                -64,
            )
            .unwrap_err(),
            SparsePatternError::TermOutOfBounds { row: 1, column: 0 }
        );
    }

    proptest! {
        #[test]
        fn generated_diagonal_patterns_have_no_fill(values in proptest::collection::vec(1_i16..=16, 1..8)) {
            let terms = values
                .iter()
                .enumerate()
                .map(|(index, value)| SparseResidualTerm {
                    row: index,
                    column: index,
                    coefficient: real(i64::from(*value)),
                })
                .collect::<Vec<_>>();

            let report = analyze_sparse_bareiss_elimination_pattern(
                values.len(),
                values.len(),
                &terms,
                -64,
            ).unwrap();

            prop_assert!(report.fully_certified_pattern());
            prop_assert!(report.fill_in_positions.is_empty());
            prop_assert_eq!(report.certified_nonzero_positions.len(), values.len());
        }
    }
}
