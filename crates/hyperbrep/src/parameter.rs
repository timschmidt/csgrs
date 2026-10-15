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
/// [`crate::Curve3::point_at`] retains selected images for lines and rational
/// curves. Topology trim ranges and exact edge subdivision also preserve these
/// parameters. Native coefficient subdivision, angular conic evaluation and
/// persistence still require represented values; their typed blockers remain
/// explicit.
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

    pub(crate) fn from_curve(value: &hypercurve::CurveParameter2) -> GeometryResult<Self> {
        value
            .as_bezier_parameter()
            .cloned()
            .map(Self::from_bezier)
            .ok_or(GeometryError::UnsupportedPcurveContour)
    }

    pub(crate) fn curve_parameter(&self) -> hypercurve::CurveParameter2 {
        self.value.clone().into()
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

    /// Borrows both endpoints when stored scalar views are available.
    pub fn try_real_endpoints(&self) -> Option<(&Real, &Real)> {
        Some((self.start.try_real()?, self.end.try_real()?))
    }

    pub(crate) fn represented_start(&self) -> GeometryResult<&Real> {
        self.start
            .try_real()
            .ok_or(GeometryError::UnrepresentableParameter)
    }

    pub(crate) fn represented_end(&self) -> GeometryResult<&Real> {
        self.end
            .try_real()
            .ok_or(GeometryError::UnrepresentableParameter)
    }

    pub(crate) fn represented_domain(&self) -> GeometryResult<crate::ParameterDomain> {
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
