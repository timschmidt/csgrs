//! Exact unpivoted QR construction, checked by source-bound chart admission.
use super::*;

/// Replay input for an exact orthonormal frame with positive QR diagonal.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct QrCoordinateRequest {
    /// Ordered source variables reinterpreted as frame coordinates.
    pub variables: Vec<SymbolId>,
    /// Finite square basis M in row-major order. No inverse is required.
    pub basis: Vec<Vec<Real>>,
    /// Exact original-coordinate origin of the new frame.
    pub origin: Vec<Real>,
}

/// Invalid, dependent, undecided or exhausted frame construction.
#[derive(Clone, Debug)]
pub enum QrCoordinateError {
    /// An exact zero residual norm proves that this column is dependent.
    DependentColumn {
        /// Zero-based column in the unpivoted order.
        column: usize,
        /// Retained strict zero decision.
        outcome: PredicateOutcome<Sign>,
    },
    /// A residual squared norm was not proved strictly positive.
    NormNotCertified {
        /// Zero-based column being normalized.
        column: usize,
        /// Exact squared norm consumed by the predicate.
        squared_norm: Real,
        /// Original outcome; unknown never means dependence.
        outcome: PredicateOutcome<Sign>,
    },
    /// A reconstruction residual is provably nonzero.
    ReconstructionMismatch {
        /// Matrix row.
        row: usize,
        /// Matrix column.
        column: usize,
        /// Original nonzero decision for Q*R-M.
        outcome: PredicateOutcome<Sign>,
    },
    /// A reconstruction residual could not be proved exactly zero.
    ReconstructionUndecided {
        /// Matrix row.
        row: usize,
        /// Matrix column.
        column: usize,
        /// Original undecided or inadmissible outcome.
        outcome: PredicateOutcome<Sign>,
    },
    /// Original shape, binding, strict inverse, graph or exact scalar diagnostic.
    Coordinate(CoordinateError),
}
impl fmt::Display for QrCoordinateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "exact QR coordinates: {self:?}")
    }
}
impl std::error::Error for QrCoordinateError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Coordinate(error) => Some(error),
            _ => None,
        }
    }
}
impl From<CoordinateError> for QrCoordinateError {
    fn from(error: CoordinateError) -> Self {
        Self::Coordinate(error)
    }
}
impl From<ExpressionError> for QrCoordinateError {
    fn from(error: ExpressionError) -> Self {
        Self::Coordinate(error.into())
    }
}
impl From<Problem> for QrCoordinateError {
    fn from(error: Problem) -> Self {
        Self::Coordinate(CoordinateError::Scalar(error))
    }
}

#[derive(Debug)]
struct QrData {
    request: QrCoordinateRequest,
    coordinates: CoordinateProgram,
    upper: Vec<Vec<Real>>,
    squared_norms: Vec<Real>,
    norm_evidence: Vec<PredicateOutcome<Sign>>,
    reconstruction_evidence: Vec<PredicateOutcome<Sign>>,
}
/// Exact M=Q*R with upper-triangular R and positive diagonal, bound to a source.
///
/// Unpivoted modified Gram-Schmidt constructs the frame. Strict norm decisions
/// precede normalization; the existing coordinate verifier independently proves
/// Q-transpose*Q=I, and every Q*R-M residual is checked under STRICT. Reflections
/// are allowed: orthogonal does not imply a positive determinant.
///
/// Replay the source and request; neither this type nor its proof flags can be
/// deserialized. Finite graph work/entry limits cover structural construction and
/// predicate-call setup, not exact scalar arithmetic/refinement, predicate
/// internals, scalar payload lifecycle, total heap or within-scalar cancellation.
/// No universal rank decision or practical bound is promised for arbitrary Real
/// inputs. A positive-dimensional dependent basis does not produce a partial frame.
#[derive(Clone, Debug)]
pub struct QrCoordinateProgram(Arc<QrData>);
impl QrCoordinateProgram {
    /// Original basis, origin and variable order.
    pub fn request(&self) -> &QrCoordinateRequest {
        &self.0.request
    }
    /// Source-bound Q chart with strictly certified inverse Q-transpose.
    pub fn coordinates(&self) -> &CoordinateProgram {
        &self.0.coordinates
    }
    /// Exact row-major upper-triangular R; lower entries are structural zero.
    pub fn upper(&self) -> &[Vec<Real>] {
        &self.0.upper
    }
    /// Exact residual squared norms before each column normalization.
    pub fn squared_norms(&self) -> &[Real] {
        &self.0.squared_norms
    }
    /// Strict positive decisions in column order.
    pub fn norm_evidence(&self) -> &[PredicateOutcome<Sign>] {
        &self.0.norm_evidence
    }
    /// Strict zero decisions for Q*R-M in row-major order.
    pub fn reconstruction_evidence(&self) -> &[PredicateOutcome<Sign>] {
        &self.0.reconstruction_evidence
    }
}
impl ExpressionProgram {
    /// Construct and certify exact QR coordinates without a caller-authored inverse.
    ///
    /// Shape checks precede the finite unpivoted column recurrence. Binding and
    /// inverse admission reuse `change_coordinates`; reconstruction checks and the
    /// final cancellation check precede publication. No term is rounded or dropped.
    /// This builds a chart, not an ODE proof or an adaptive frame-selection rule.
    pub fn qr_coordinates(
        &self,
        request: QrCoordinateRequest,
        budget: &mut ExpressionBudget,
    ) -> Result<QrCoordinateProgram, QrCoordinateError> {
        budget.charge(0)?;
        let n = request.variables.len();
        shape("outputs", None, self.definition().outputs.len(), n)?;
        shape("origin", None, request.origin.len(), n)?;
        shape("basis", None, request.basis.len(), n)?;
        for (row, values) in request.basis.iter().enumerate() {
            budget.charge(1)?;
            shape("basis", Some(row), values.len(), n)?;
        }
        let square = n.checked_mul(n).ok_or(ExpressionError::SizeExhausted)?;
        budget.check_entries(&[square, square, square, square, n, n, n, n])?;
        budget.charge(square)?;
        let mut upper = vec![vec![Real::zero(); n]; n];
        let mut columns: Vec<Vec<Real>> = Vec::with_capacity(n);
        let mut squared_norms = Vec::with_capacity(n);
        let mut norm_evidence = Vec::with_capacity(n);
        for column in 0..n {
            budget.charge(n)?;
            let mut residual: Vec<_> = request.basis.iter().map(|r| r[column].clone()).collect();
            for (previous, q) in columns.iter().enumerate() {
                budget.charge(n)?;
                let coefficient = Real::sum_products(q, &residual)?;
                for (value, direction) in residual.iter_mut().zip(q) {
                    budget.charge(1)?;
                    *value = &*value - &coefficient * direction;
                }
                upper[previous][column] = coefficient;
            }
            budget.charge(n)?;
            let squared = Real::sum_products(&residual, &residual)?;
            budget.charge(1)?;
            let outcome = classify_real_sign(&squared, PredicatePolicy::STRICT);
            match outcome {
                PredicateOutcome::Decided {
                    value: Sign::Positive,
                    certainty: Certainty::Exact | Certainty::Filtered,
                    ..
                } => {}
                PredicateOutcome::Decided {
                    value: Sign::Zero,
                    certainty: Certainty::Exact | Certainty::Filtered,
                    ..
                } => return Err(QrCoordinateError::DependentColumn { column, outcome }),
                _ => {
                    return Err(QrCoordinateError::NormNotCertified {
                        column,
                        squared_norm: squared,
                        outcome,
                    });
                }
            }
            budget.charge(2)?;
            squared_norms.push(squared.clone());
            norm_evidence.push(outcome);
            let length = squared.sqrt()?;
            let inverse = length.clone().inverse()?;
            for value in &mut residual {
                budget.charge(1)?;
                *value = &*value * &inverse;
            }
            upper[column][column] = length;
            columns.push(residual);
        }
        budget.charge(square)?;
        let matrix: Vec<Vec<Real>> = (0..n)
            .map(|row| columns.iter().map(|column| column[row].clone()).collect())
            .collect();
        budget.charge(n)?;
        budget.charge(n)?;
        let coordinates = self.change_coordinates(
            CoordinateChange {
                variables: request.variables.clone(),
                to_source: matrix,
                from_source: columns,
                origin: request.origin.clone(),
            },
            budget,
        )?;
        budget.check_entries(&[square, square, n, n])?;
        let mut reconstruction_evidence = Vec::with_capacity(square);
        for (row, values) in coordinates.change().to_source.iter().enumerate() {
            budget.charge(1)?;
            for column in 0..n {
                budget.charge(n)?;
                let axis: Vec<_> = upper.iter().map(|r| r[column].clone()).collect();
                budget.charge(n)?;
                let residual = Real::sum_products(values, &axis)? - &request.basis[row][column];
                budget.charge(1)?;
                let outcome = classify_real_sign(&residual, PredicatePolicy::STRICT);
                match outcome {
                    PredicateOutcome::Decided {
                        value: Sign::Zero,
                        certainty: Certainty::Exact | Certainty::Filtered,
                        ..
                    } => reconstruction_evidence.push(outcome),
                    PredicateOutcome::Decided {
                        value: Sign::Positive | Sign::Negative,
                        certainty: Certainty::Exact | Certainty::Filtered,
                        ..
                    } => {
                        return Err(QrCoordinateError::ReconstructionMismatch {
                            row,
                            column,
                            outcome,
                        });
                    }
                    _ => {
                        return Err(QrCoordinateError::ReconstructionUndecided {
                            row,
                            column,
                            outcome,
                        });
                    }
                }
            }
        }
        budget.charge(1)?;
        let result = QrCoordinateProgram(Arc::new(QrData {
            request,
            coordinates,
            upper,
            squared_norms,
            norm_evidence,
            reconstruction_evidence,
        }));
        budget.charge(0)?;
        Ok(result)
    }
}
