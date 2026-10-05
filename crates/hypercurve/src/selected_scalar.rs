//! One dispatch over Hypercurve's selected scalar parameter authorities.
//!
//! A selected scalar is an ordinary represented or algebraic Bezier
//! parameter, a root retained in one selected fiber, or a value or root over
//! a recursive quadratic tower. Each keeps its native authority and
//! refinement; this type owns the per-authority dispatch for scalar
//! operations so that parameter charts do not repeat it.

use core::cmp::Ordering;

use hyperreal::{Real, RealSign};
use hypersolve::represented_root::scalar_in_open_interval;

use crate::bezier_offset::{
    BezierAlgebraicSelectedFiberParameter2, BezierRecursiveProjectiveParameter2,
};
use crate::classify::compare_reals;
use crate::{BezierParameter2, Classification, CurveContext, CurveError, CurveResult};

#[derive(Clone, Debug)]
pub(crate) enum SelectedScalar2 {
    Bezier(BezierParameter2),
    SelectedFiber(BezierAlgebraicSelectedFiberParameter2),
    RecursiveProjective(BezierRecursiveProjectiveParameter2),
}

impl PartialEq for SelectedScalar2 {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Bezier(first), Self::Bezier(second)) => first == second,
            (Self::SelectedFiber(first), Self::SelectedFiber(second)) => first == second,
            (Self::RecursiveProjective(first), Self::RecursiveProjective(second)) => {
                first == second
            }
            _ => false,
        }
    }
}

impl SelectedScalar2 {
    /// Returns the stored scalar of a directly represented parameter.
    pub(crate) fn scalar(&self) -> Option<&Real> {
        match self {
            Self::Bezier(parameter) => parameter.scalar(),
            Self::SelectedFiber(_) | Self::RecursiveProjective(_) => None,
        }
    }

    /// Replays a scalar polynomial in this value's existing exact field.
    pub(crate) fn polynomial_sign(
        &self,
        coefficients: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        match self {
            Self::Bezier(parameter) => crate::bezier_parameter::signed_coefficients_at_parameter(
                coefficients,
                parameter,
                policy,
            ),
            Self::SelectedFiber(parameter) => parameter.predicate_sign(
                &hypersolve::BivariatePolynomial::new(vec![coefficients.to_vec()]),
                policy,
            ),
            Self::RecursiveProjective(parameter) => parameter.polynomial_sign(coefficients, policy),
        }
    }

    /// Orders two selected scalars, each under its native authority.
    pub(crate) fn cmp_by_refinement(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>> {
        match (self, other) {
            (Self::Bezier(first), Self::Bezier(second)) => {
                first.cmp_by_refinement_with_policy(second, policy)
            }
            (Self::SelectedFiber(first), Self::SelectedFiber(second)) => {
                first.cmp_by_refinement(second, policy)
            }
            (Self::RecursiveProjective(first), Self::RecursiveProjective(second)) => {
                first.cmp_by_refinement(second, policy)
            }
            (Self::SelectedFiber(first), Self::Bezier(second)) => {
                first.cmp_bezier_parameter(second, policy)
            }
            (Self::Bezier(first), Self::SelectedFiber(second)) => Ok(second
                .cmp_bezier_parameter(first, policy)?
                .map(Ordering::reverse)),
            (Self::RecursiveProjective(first), Self::Bezier(second)) => {
                first.cmp_bezier_parameter(second, policy)
            }
            (Self::Bezier(first), Self::RecursiveProjective(second)) => Ok(second
                .cmp_bezier_parameter(first, policy)?
                .map(Ordering::reverse)),
            (Self::SelectedFiber(first), Self::RecursiveProjective(second)) => Ok(second
                .cmp_selected_fiber_parameter(first, policy)?
                .map(Ordering::reverse)),
            (Self::RecursiveProjective(first), Self::SelectedFiber(second)) => {
                first.cmp_selected_fiber_parameter(second, policy)
            }
        }
    }

    pub(crate) fn unit_complement(&self) -> Self {
        match self {
            Self::Bezier(parameter) => Self::Bezier(parameter.unit_complement()),
            Self::SelectedFiber(parameter) => Self::SelectedFiber(parameter.unit_complement()),
            Self::RecursiveProjective(parameter) => {
                Self::RecursiveProjective(parameter.unit_complement())
            }
        }
    }

    /// Returns the exact finite isolating bounds of this scalar. These are
    /// outward certificates, not representative values.
    pub(crate) fn isolating_bounds(&self) -> (&Real, &Real) {
        match self {
            Self::Bezier(BezierParameter2::Exact(parameter)) => (parameter, parameter),
            Self::Bezier(BezierParameter2::Algebraic(parameter)) => {
                (parameter.interval().start(), parameter.interval().end())
            }
            Self::SelectedFiber(parameter) => parameter.isolating_bounds(),
            Self::RecursiveProjective(parameter) => parameter.isolating_bounds(),
        }
    }

    /// Refines the isolating bounds while preserving the native authority.
    pub(crate) fn refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match self {
            Self::Bezier(parameter) => Ok(Classification::Decided(Self::Bezier(
                parameter
                    .clone()
                    .refined_isolating_interval(refinement_steps, policy),
            ))),
            Self::SelectedFiber(parameter) => Ok(parameter
                .refined(refinement_steps, policy)?
                .map(Self::SelectedFiber)),
            Self::RecursiveProjective(parameter) => Ok(parameter
                .refined(refinement_steps, policy)?
                .map(Self::RecursiveProjective)),
        }
    }

    /// Applies a finite affine chart without projecting a retained scalar
    /// into a degree-multiplied global polynomial.
    pub(crate) fn affine_image_unbounded(
        &self,
        scale: &Real,
        offset: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match self {
            Self::Bezier(parameter) => Ok(parameter
                .affine_image_unbounded(scale, offset, policy)?
                .map(Self::Bezier)),
            Self::SelectedFiber(parameter) => Ok(parameter
                .affine_image_unbounded(scale, offset, policy)?
                .map(Self::SelectedFiber)),
            Self::RecursiveProjective(parameter) => Ok(parameter
                .affine_image_unbounded(scale, offset, policy)?
                .map(Self::RecursiveProjective)),
        }
    }

    /// Applies one finite projective chart while preserving the native
    /// authority, including ordinary algebraic singleton parameters.
    pub(crate) fn projective_image_unbounded(
        &self,
        numerator: &[Real; 2],
        denominator: &[Real; 2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match self {
            Self::Bezier(parameter) => Ok(parameter
                .projective_image_unbounded(numerator, denominator, policy)?
                .map(Self::Bezier)),
            Self::SelectedFiber(parameter) => Ok(parameter
                .projective_image_unbounded(numerator, denominator, policy)?
                .map(Self::SelectedFiber)),
            Self::RecursiveProjective(parameter) => Ok(parameter
                .projective_image_unbounded(numerator, denominator, policy)?
                .map(Self::RecursiveProjective)),
        }
    }

    /// Promotes this scalar only for a consumer that requires an ordinary
    /// Bezier parameter.
    pub(crate) fn promoted_bezier_parameter_complete(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParameter2>> {
        match self {
            Self::Bezier(parameter) => Ok(Classification::Decided(parameter.clone())),
            Self::SelectedFiber(parameter) => parameter.promoted_bezier_parameter_complete(policy),
            Self::RecursiveProjective(parameter) => {
                parameter.promoted_bezier_parameter_complete(policy)
            }
        }
    }

    /// Constructs an exact scalar strictly between two scalars already known
    /// to be ordered `self < other`.
    pub(crate) fn strict_scalar_between_ordered(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
        match (self, other) {
            (Self::Bezier(first), Self::Bezier(second)) => {
                first.strict_scalar_between_ordered(second, policy)
            }
            (Self::SelectedFiber(first), Self::SelectedFiber(second)) => {
                first.strict_scalar_between_ordered(second, policy)
            }
            _ => {
                // A scalar gap needs separated enclosures, not a common
                // coefficient field or a global polynomial for either cut.
                // Keep each endpoint under its native refinement authority.
                let mut refinement_steps = 0_usize;
                loop {
                    let first = match self.refined(refinement_steps, policy)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let second = match other.refined(refinement_steps, policy)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let (_, first_upper) = first.isolating_bounds();
                    let (second_lower, _) = second.isolating_bounds();
                    if compare_reals(first_upper, second_lower, &CurveContext::STRICT)
                        == Some(Ordering::Less)
                    {
                        return Ok(Classification::Decided(scalar_in_open_interval(
                            first_upper,
                            second_lower,
                        )));
                    }
                    refinement_steps = refinement_steps
                        .checked_mul(2)
                        .and_then(|steps| steps.checked_add(1))
                        .ok_or_else(|| {
                            CurveError::Topology("finite scalar separation overflow".into())
                        })?;
                }
            }
        }
    }
}
