//! Fixed-size numerical projection without discarding source-domain evidence.
use super::{
    DyadicBudget, DyadicInterval, ExpressionBudget, ExpressionError, RegularJacobianEnclosure,
};
use hyperlattice::IntervalMatrix;

/// Row-major partial enclosures retaining their original regularity evidence.
///
/// Rows follow source output order; columns follow the selected differentiation
/// variables, which need not match input binding order. The borrowed evidence
/// retains source, domain and precision. Matrix arithmetic remains Hyperlattice's
/// responsibility; a numerical product alone is not a new regularity certificate.
#[derive(Debug)]
pub struct JacobianMatrix<'a, const R: usize, const C: usize> {
    evidence: &'a RegularJacobianEnclosure,
    intervals: IntervalMatrix<DyadicInterval, R, C>,
}

impl<'a, const R: usize, const C: usize> JacobianMatrix<'a, R, C> {
    /// Original immutable source, domain and selected-variable association.
    pub fn evidence(&self) -> &'a RegularJacobianEnclosure {
        self.evidence
    }

    /// Numerical matrix for subsequent lattice operations, without copying.
    pub fn intervals(&self) -> &IntervalMatrix<DyadicInterval, R, C> {
        &self.intervals
    }

    /// Explicitly discard this projection's evidence reference and move its matrix.
    /// The original borrowed evidence remains usable.
    pub fn into_intervals(self) -> IntervalMatrix<DyadicInterval, R, C> {
        self.intervals
    }
}

impl RegularJacobianEnclosure {
    /// Project selected partials into a fixed-size matrix with retained evidence.
    ///
    /// Row count, then column count, are checked before handling an empty domain.
    /// An empty domain
    /// yields an empty matrix even when a matrix dimension is zero; otherwise a
    /// zero dimension denotes the nonempty unit product. Scalar copies preserve
    /// the stored exact bounds and grid, including the original enclosure width.
    ///
    /// Graph charges cover domain scanning, entry initialization, row-major copies
    /// and final assembly. All native coefficient copies share `arithmetic`.
    /// Cancellation is checked on entry and before publication; failures publish
    /// no partial matrix and do not mutate evidence. Inline temporary/output
    /// storage grows with R*C; these budgets are not a total stack/heap bound.
    pub fn matrix<const R: usize, const C: usize>(
        &self,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<JacobianMatrix<'_, R, C>, ExpressionError> {
        graph.charge(0)?;
        let program = self.program();
        let actual = (
            program.source().definition().outputs.len(),
            program.variables().len(),
        );
        if actual.0 != R {
            return Err(ExpressionError::JacobianRows {
                expected: R,
                actual: actual.0,
            });
        }
        if actual.1 != C {
            return Err(ExpressionError::JacobianColumns {
                expected: C,
                actual: actual.1,
            });
        }
        let entries = R.checked_mul(C).ok_or(ExpressionError::SizeExhausted)?;
        graph.check_entries(&[R, C, entries, self.arguments().len()])?;
        graph.charge(self.arguments().len())?;
        let intervals = if self.arguments().iter().any(DyadicInterval::is_empty) {
            IntervalMatrix::empty().map_err(ExpressionError::Lattice)?
        } else {
            graph.charge(R + entries)?;
            let mut rows: [[Option<DyadicInterval>; C]; R] =
                core::array::from_fn(|_| core::array::from_fn(|_| None));
            for (row, slots) in rows.iter_mut().enumerate() {
                for (column, slot) in slots.iter_mut().enumerate() {
                    graph.charge(1)?;
                    let index = program
                        .partial_output(row, column)
                        .expect("validated shape");
                    *slot = Some(self.values()[index].checked_pow(1, arithmetic).map_err(
                        |source| ExpressionError::Enclosure {
                            node: program.program().definition().outputs[index],
                            source,
                        },
                    )?);
                }
            }
            graph.charge(R + entries)?;
            IntervalMatrix::new(
                rows.map(|row| row.map(|entry| entry.expect("all entries initialized"))),
            )
            .map_err(ExpressionError::Lattice)?
        };
        graph.charge(0)?;
        Ok(JacobianMatrix {
            evidence: self,
            intervals,
        })
    }
}
