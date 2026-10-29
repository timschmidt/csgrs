//! Shared definitions of selected fields and values, without live proof caches.

mod encode;
mod replay;

use super::{SelectedPolynomialError, SelectedPolynomialValue};
use crate::radical_expression::TwoSquareRootExpression;
use crate::{DenseTensorPolynomial, SymbolId};
use hyperreal::Real;

/// A selected native root's original data, with no trusted validation flags.
/// Positive-width native intervals own `(lower, upper]`; an exact point witness
/// retains the native root admission contract even in a wider interval.
#[derive(Clone, Debug)]
pub struct SelectedPolynomialRootDefinition {
    pub constraint_index: usize,
    pub symbol: SymbolId,
    pub interval_index: usize,
    pub polynomial: Vec<Real>,
    pub lower: Real,
    pub upper: Real,
    pub exact_root: Option<Real>,
}

/// One definition record. Every index references an earlier record of the
/// required type. Native base expressions retain both positive square roots;
/// polynomial values retain rational functions in their selected generator.
#[derive(Clone, Debug)]
pub enum SelectedPolynomialDefinitionNode {
    NativeBase {
        sources: Vec<SelectedPolynomialRootDefinition>,
        first_speed_squared: DenseTensorPolynomial,
        second_speed_squared: DenseTensorPolynomial,
    },
    NativeExtension {
        parent: usize,
        radicand: usize,
    },
    NativeBaseValue {
        field: usize,
        expression: TwoSquareRootExpression<DenseTensorPolynomial>,
    },
    NativeExtensionValue {
        field: usize,
        retained: usize,
        radical: usize,
    },
    NativeField {
        native: usize,
    },
    PolynomialField {
        parent: usize,
        coefficients: Vec<usize>,
        lower: Real,
        upper: Real,
    },
    /// Replays a common-field construction, including its original selected
    /// generator correspondences. Both references name earlier field records.
    /// This is an operation recipe, not a trusted equality assertion.
    JoinedField {
        first: usize,
        second: usize,
    },
    NativeValue {
        field: usize,
        numerator: usize,
        denominator: usize,
    },
    PolynomialValue {
        field: usize,
        numerator: Vec<usize>,
        denominator: Vec<usize>,
    },
}

/// A finite, shared definition graph, suitable for a format owner to encode.
///
/// This is a scalar definition, not a curve's source/incidence provenance.
/// `Real` payloads must use validated exact scalar interchange at a byte
/// boundary. No wire format or geometric replay is implied by this type.
#[derive(Clone, Debug)]
pub struct SelectedPolynomialDefinition {
    pub nodes: Vec<SelectedPolynomialDefinitionNode>,
    /// Output value indices, preserving order and repeated outputs.
    pub roots: Vec<usize>,
}

/// The declared graph-record limit was reached. No partial graph is returned.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SelectedPolynomialNodeLimit {
    pub limit: usize,
}

/// Replay failure before any output is published.
#[derive(Debug)]
pub enum SelectedPolynomialReplayError<E> {
    /// A forward, absent, or wrongly typed reference. For roots, `node` equals
    /// the number of graph records.
    InvalidReference {
        node: usize,
        reference: usize,
    },
    InvalidNativeRoot {
        node: usize,
        source: usize,
    },
    /// A native base square is negative, or an extension square is not positive.
    InvalidRadicand {
        node: usize,
    },
    /// A value has the wrong native variant, tensor rank, or coefficient field.
    InvalidCoordinates {
        node: usize,
    },
    NodeLimit(SelectedPolynomialNodeLimit),
    /// Original selection, arithmetic, denominator, or context error.
    Field(SelectedPolynomialError<E>),
}

impl<E> From<SelectedPolynomialError<E>> for SelectedPolynomialReplayError<E> {
    fn from(error: SelectedPolynomialError<E>) -> Self {
        Self::Field(error)
    }
}

impl<E> From<super::SelectedPolynomialArithmeticError> for SelectedPolynomialReplayError<E> {
    fn from(error: super::SelectedPolynomialArithmeticError) -> Self {
        Self::Field(SelectedPolynomialError::Arithmetic(error))
    }
}

impl SelectedPolynomialDefinition {
    /// Exports shared immutable storage with iterative dependency traversal.
    /// Pointer keys live only during this call and never define value equality.
    ///
    /// `node_limit` counts records, not coefficient storage, arithmetic work,
    /// temporary memory or cancellation. Underlying arithmetic has its own
    /// resource limitations; this limit is not a total resource bound.
    pub fn from_values(
        values: &[SelectedPolynomialValue],
        node_limit: usize,
    ) -> Result<Self, SelectedPolynomialNodeLimit> {
        encode::encode(values, node_limit)
    }
}
