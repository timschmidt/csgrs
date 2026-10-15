//! Retained exact parameters shared by spatial geometry queries.

use std::cmp::Ordering;

use hypercurve::BezierParameter2;
use hyperlimit::{PredicateOutcome, compare_reals};

use crate::{GeometryError, GeometryResult, Real};

/// An exact authored value or a certified selected algebraic parameter.
///
/// Spatial inverse queries retain the selected root even when it has no
/// stored [`Real`] representation. Root arithmetic, comparison and chart
/// transport reuse Hypercurve and Hypersolve's existing authority. This type
/// does not introduce another coefficient scalar.
///
/// Mathematical comparison is fallible; structural inequality is not proof
/// that two parameters differ. Use [`Self::compare`] for exact ordering.
/// Current spatial evaluation, edge domains and persistence still require
/// represented values; retaining a query result does not yet admit it into
/// those operations.
#[derive(Clone, Debug)]
pub struct Parameter {
    value: BezierParameter2,
}

impl Parameter {
    /// Retains an authored exact scalar without conversion.
    pub fn from_real(value: Real) -> Self {
        Self {
            value: BezierParameter2::Exact(value),
        }
    }

    /// Borrows a stored scalar, without reconstructing or approximating a root.
    ///
    /// `None` means this view is unavailable, not that the parameter is invalid
    /// or inexact.
    pub fn try_real(&self) -> Option<&Real> {
        self.value.scalar()
    }

    /// Certifies mathematical order under HyperBREP's strict predicate policy.
    ///
    /// An unresolved comparison is an error, including inside an ambient
    /// provisional Hypercurve scope. Enclose a batch in
    /// [`crate::OperationContext::evaluate`] to share the existing instrumented
    /// solver-work budget across its comparisons and geometry queries.
    pub fn compare(&self, other: &Self) -> GeometryResult<Ordering> {
        crate::context::strict(|| {
            if let (Some(first), Some(second)) = (self.try_real(), other.try_real()) {
                return match compare_reals(first, second, crate::STRICT_PREDICATES) {
                    PredicateOutcome::Decided { value, .. } => Ok(value),
                    PredicateOutcome::Unknown { needed, stage } => {
                        Err(GeometryError::PredicateUnresolved { needed, stage })
                    }
                };
            }
            Ok(self.value.cmp_by_refinement(&other.value)?)
        })
    }

    pub(crate) fn from_bezier(value: BezierParameter2) -> Self {
        Self { value }
    }

    pub(crate) fn affine_image(&self, scale: &Real, offset: &Real) -> GeometryResult<Self> {
        Ok(Self::from_bezier(self.value.affine_image(scale, offset)?))
    }
}

impl From<Real> for Parameter {
    fn from(value: Real) -> Self {
        Self::from_real(value)
    }
}
