//! Retained exact parameters shared by spatial geometry queries.

use std::cmp::Ordering;

use hypercurve::{BezierParameter2, CurveParameter2};
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
/// [`crate::Curve3::point_at`] retains selected images for lines and rational
/// curves. Topology trim ranges and exact edge subdivision also preserve these
/// parameters. Native coefficient subdivision, angular conic evaluation and
/// some independent solid validators still require represented values; their
/// typed blockers remain explicit. Native archives retain selected root graphs.
#[derive(Clone, Debug)]
pub struct Parameter {
    value: CurveParameter2,
}

impl Parameter {
    /// Retains an authored exact scalar without conversion.
    pub fn from_real(value: Real) -> Self {
        Self {
            value: value.into(),
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
            Ok(self.value.compare(&other.value)?)
        })
    }

    pub(crate) fn from_bezier(value: BezierParameter2) -> Self {
        Self {
            value: value.into(),
        }
    }

    pub(crate) fn from_curve(value: &hypercurve::CurveParameter2) -> GeometryResult<Self> {
        Ok(Self {
            value: value.clone(),
        })
    }

    /// Returns the retained Hypercurve parameter for consuming materialized
    /// pcurves and constructing joint parameter images. This preserves the
    /// original exact source; it does not reconstruct scalar coordinates or
    /// certify membership in a different curve's domain.
    pub fn curve_parameter(&self) -> hypercurve::CurveParameter2 {
        self.value.clone()
    }

    /// Returns `scale * self + offset` in the same selected source field.
    ///
    /// This transports exact parameters between curve charts, including
    /// reversal, without reconstructing or rounding independent coordinates.
    /// The result is not a domain-membership or incidence certificate.
    pub fn affine_image(&self, scale: &Real, offset: &Real) -> GeometryResult<Self> {
        crate::context::strict(|| {
            Ok(Self {
                value: self.value.affine_image(scale, offset)?,
            })
        })
    }
}

impl From<Real> for Parameter {
    fn from(value: Real) -> Self {
        Self::from_real(value)
    }
}

/// A strictly increasing closed trim range in a curve's retained parameter chart.
///
/// Unlike an authored [`crate::ParameterDomain`], a trim may end at a selected
/// algebraic contact. Its endpoints retain their evidence without requiring
/// independent scalar coordinates. Reversing traversal does not change this
/// canonical ordering.
#[derive(Clone, Debug)]
pub struct ParameterRange {
    start: Parameter,
    end: Parameter,
}

impl ParameterRange {
    /// Certifies a nonempty increasing range.
    pub fn new(start: Parameter, end: Parameter) -> GeometryResult<Self> {
        match start.compare(&end)? {
            Ordering::Less => Ok(Self { start, end }),
            _ => Err(GeometryError::InvalidParameterDomain),
        }
    }

    /// Returns the exact unit range.
    pub fn unit() -> Self {
        Self {
            start: Real::zero().into(),
            end: Real::one().into(),
        }
    }

    /// Borrows the inclusive lower endpoint.
    pub const fn start(&self) -> &Parameter {
        &self.start
    }

    /// Borrows the inclusive upper endpoint.
    pub const fn end(&self) -> &Parameter {
        &self.end
    }

    /// Certifies closed-range membership without materializing either endpoint.
    pub fn contains(&self, parameter: &Parameter) -> GeometryResult<bool> {
        Ok(parameter.compare(&self.start)? != Ordering::Less
            && parameter.compare(&self.end)? != Ordering::Greater)
    }

    pub(crate) fn contains_real(&self, parameter: &Real) -> GeometryResult<bool> {
        self.contains(&Parameter::from_real(parameter.clone()))
    }

    pub(crate) fn affine_image(&self, scale: &Real, offset: &Real) -> GeometryResult<Self> {
        let mut start = self.start.affine_image(scale, offset)?;
        let mut end = self.end.affine_image(scale, offset)?;
        if start.compare(&end)? == Ordering::Greater {
            std::mem::swap(&mut start, &mut end);
        }
        Self::new(start, end)
    }

    /// Borrows both endpoints when stored scalar views are available.
    pub fn try_real_endpoints(&self) -> Option<(&Real, &Real)> {
        Some((self.start.try_real()?, self.end.try_real()?))
    }

    /// Borrows the lower endpoint when a stored authored scalar is available.
    pub fn represented_start(&self) -> GeometryResult<&Real> {
        self.start
            .try_real()
            .ok_or(GeometryError::UnrepresentableParameter)
    }

    /// Borrows the upper endpoint when a stored authored scalar is available.
    pub fn represented_end(&self) -> GeometryResult<&Real> {
        self.end
            .try_real()
            .ok_or(GeometryError::UnrepresentableParameter)
    }

    /// Materializes the authored interval view without solving or approximating roots.
    /// An unavailable view does not invalidate this retained range.
    pub fn represented_domain(&self) -> GeometryResult<crate::ParameterDomain> {
        crate::ParameterDomain::new(
            self.represented_start()?.clone(),
            self.represented_end()?.clone(),
        )
    }
}

impl From<crate::ParameterDomain> for ParameterRange {
    fn from(domain: crate::ParameterDomain) -> Self {
        Self::from(&domain)
    }
}

impl From<&crate::ParameterDomain> for ParameterRange {
    fn from(domain: &crate::ParameterDomain) -> Self {
        Self {
            start: domain.start().clone().into(),
            end: domain.end().clone().into(),
        }
    }
}
