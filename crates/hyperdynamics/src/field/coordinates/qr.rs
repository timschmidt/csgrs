//! Source-bound QR frames consumed by existing field and transport operations.
use super::*;
use hypersolve::expression_program::{QrCoordinateError, QrCoordinateProgram, QrCoordinateRequest};

/// Exact QR construction or original field-construction failure.
#[derive(Clone, Debug)]
pub enum QrFieldError {
    /// Original shape, rank, orthogonality, reconstruction or resource failure.
    Qr(QrCoordinateError),
    /// Original field interpretation, graph or derivative failure.
    Field(FieldError),
}
impl fmt::Display for QrFieldError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "QR field: {self:?}")
    }
}
impl std::error::Error for QrFieldError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Qr(error) => Some(error),
            Self::Field(error) => Some(error),
        }
    }
}
impl From<ExpressionError> for QrFieldError {
    fn from(error: ExpressionError) -> Self {
        Self::Field(error.into())
    }
}
/// Exact QR factorization and its original/transformed vector field association.
///
/// The chart and its endpoint transport feed existing Picard, Taylor and finite-
/// horizon APIs. A frame alone proves neither existence nor reduced enclosure
/// width. The caller supplies the basis; automatic frame selection remains separate.
/// Replay the original field, basis and origin, never serialized proof flags.
#[derive(Clone, Debug)]
pub struct QrCoordinateField<const N: usize> {
    factorization: QrCoordinateProgram,
    chart: CoordinateField<N>,
}
impl<const N: usize> QrCoordinateField<N> {
    /// Exact input basis, R, norm decisions and reconstruction/orthogonality proofs.
    pub fn factorization(&self) -> &QrCoordinateProgram {
        &self.factorization
    }
    /// Retained source-bound chart consumed by native steps and direct transport.
    pub fn chart(&self) -> &CoordinateField<N> {
        &self.chart
    }
}
impl<const N: usize> ExpressionField<N> {
    /// Construct an orthonormal state chart from an exact square basis and origin.
    ///
    /// No inverse is supplied by the caller. Hypersolve certifies Q-transpose as
    /// the inverse, retains QR reconstruction and supplies the transformed source.
    /// One shared budget covers all stages; the original exact scalar/predicate,
    /// payload, total-heap and within-scalar cancellation exclusions remain.
    pub fn in_qr_coordinates(
        &self,
        basis: Vec<Vec<hyperreal::Real>>,
        origin: Vec<hyperreal::Real>,
        budget: &mut EnclosureBudget,
    ) -> Result<QrCoordinateField<N>, QrFieldError> {
        budget.graph.charge(N)?;
        let factorization = self
            .data
            .identity
            .0
            .source
            .qr_coordinates(
                QrCoordinateRequest {
                    variables: self.data.identity.state().to_vec(),
                    basis,
                    origin,
                },
                &mut budget.graph,
            )
            .map_err(QrFieldError::Qr)?;
        let transformation = factorization.coordinates().clone();
        budget.graph.charge(N)?;
        let field = Self::from_validated_source(
            transformation.program().clone(),
            self.data.identity.time(),
            self.data.identity.state().to_vec(),
            self.data.inputs.len(),
            budget,
        )
        .map_err(QrFieldError::Field)?;
        budget.graph.charge(1)?;
        let result = QrCoordinateField {
            factorization,
            chart: CoordinateField {
                data: Arc::new(CoordinateData {
                    original: self.clone(),
                    transformation,
                    field,
                }),
            },
        };
        budget.graph.charge(0)?;
        Ok(result)
    }
}
