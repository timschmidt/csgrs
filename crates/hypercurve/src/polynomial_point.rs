//! Correlated plane points in one retained algebraic fiber.

use std::cmp::Ordering;

use hypersolve::{BivariatePolynomial, IsolatedRootInterval};

use crate::bezier_offset::BezierAlgebraicSelectedFiberParameter2;
use crate::{
    Aabb2, Axis2, BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CurveContext, CurveError, CurveOperation2,
    CurveParameter2, CurvePoint2, CurvePointData2, CurveResult, ExactCurveError, ExactCurveResult,
    Point2, Real, UncertaintyReason,
};

/// Both coordinates keep the same selected base/fiber relation.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PolynomialPoint2 {
    base: BezierParameter2,
    fiber: BezierAlgebraicSelectedFiberParameter2,
}

impl PolynomialPoint2 {
    pub(crate) fn from_fiber(
        equation: BivariatePolynomial,
        base: BezierParameter2,
        roots: &[IsolatedRootInterval],
    ) -> ExactCurveResult<Vec<CurvePoint2>> {
        if roots.is_empty() {
            return Ok(Vec::new());
        }
        let policy = &CurveContext::STRICT;
        let selected = match &base {
            BezierParameter2::Algebraic(parameter) => parameter.clone(),
            BezierParameter2::Exact(value) => {
                // A linear field keeps rational slices on the same native
                // selected-fiber path. Its interval is certified, not a point
                // approximation and not a global projection of the second root.
                let polynomial = BezierParameterPolynomial::try_new_power_basis(vec![
                    -value.clone(),
                    Real::one(),
                ])?;
                let interval =
                    BezierParameterInterval::try_new(value - Real::one(), value + Real::one())?;
                BezierAlgebraicParameter2::try_isolate(polynomial, interval)?
            }
        };
        let parameters = BezierAlgebraicSelectedFiberParameter2::from_checked_roots(
            equation, selected, roots, policy,
        )
        .map_err(|cause| {
            ExactCurveError::invalid_unattributed(CurveOperation2::Construction, cause)
        })?;
        let parameters = ExactCurveError::decided(CurveOperation2::Construction, parameters)?;
        Ok(parameters
            .into_iter()
            .map(|fiber| {
                CurvePoint2(CurvePointData2::Polynomial(std::sync::Arc::new(Self {
                    base: base.clone(),
                    fiber,
                })))
            })
            .collect())
    }

    pub(crate) fn definition(&self) -> crate::CurvePointDefinition2 {
        crate::CurvePointDefinition2::PolynomialFiber {
            equation: self.fiber.defining_fiber().0.clone(),
            parameter: self.base.clone(),
            root: self.fiber.root().clone(),
        }
    }

    pub(crate) fn coordinate_parameter(&self, axis: Axis2) -> CurveParameter2 {
        match axis {
            Axis2::X => self.base.clone().into(),
            Axis2::Y => CurveParameter2::from_selected_fiber(self.fiber.clone()),
        }
    }

    pub(crate) fn affine_order(
        &self,
        a: &Real,
        b: &Real,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>> {
        self.fiber
            .predicate_sign(
                &BivariatePolynomial::new(vec![vec![-value.clone(), b.clone()], vec![a.clone()]]),
                policy,
            )
            .map(|sign| {
                sign.map(|sign| match sign {
                    hyperreal::RealSign::Negative => Ordering::Less,
                    hyperreal::RealSign::Zero => Ordering::Equal,
                    hyperreal::RealSign::Positive => Ordering::Greater,
                })
            })
    }

    pub(crate) fn circle_residual_sign(
        &self,
        center: &Point2,
        radius_squared: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<hyperreal::RealSign>> {
        let two = Real::from(2);
        self.fiber.predicate_sign(
            &BivariatePolynomial::new(vec![
                vec![
                    center.x() * center.x() + center.y() * center.y() - radius_squared,
                    -(&two * center.y()),
                    Real::one(),
                ],
                vec![-(&two * center.x())],
                vec![Real::one()],
            ]),
            policy,
        )
    }

    pub(crate) fn affine_parameter(
        &self,
        [a, b, c]: &[Real; 3],
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveParameter2>> {
        if a.definitely_zero() {
            if b.definitely_zero() {
                return Ok(Classification::Decided(c.clone().into()));
            }
            return self
                .coordinate_parameter(Axis2::Y)
                .affine_image_unbounded(b, c, policy);
        }
        if b.definitely_zero() {
            return self
                .coordinate_parameter(Axis2::X)
                .affine_image_unbounded(a, c, policy);
        }
        if let BezierParameter2::Exact(base) = &self.base {
            return self.coordinate_parameter(Axis2::Y).affine_image_unbounded(
                b,
                &(a * base + c),
                policy,
            );
        }
        Ok(self
            .fiber
            .rational_image_in_retained_field(
                &BivariatePolynomial::new(vec![vec![c.clone(), b.clone()], vec![a.clone()]]),
                &BivariatePolynomial::new(vec![vec![Real::one()]]),
                policy,
            )?
            .map(CurveParameter2::from_selected_fiber))
    }

    pub(crate) fn bounds(&self, steps: usize, policy: &CurveContext) -> Classification<Aabb2> {
        let base = self.base.clone().refined_isolating_interval(steps, policy);
        let (lower, upper) = match &base {
            BezierParameter2::Exact(value) => (value, value),
            BezierParameter2::Algebraic(value) => {
                (value.interval().start(), value.interval().end())
            }
        };
        let fiber = match self.fiber.refined(steps, policy) {
            Ok(Classification::Decided(fiber)) => fiber,
            Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
            Err(CurveError::InvalidBezierAlgebraicParameter) => {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Predicate),
        };
        let (bottom, top) = fiber.isolating_bounds();
        Classification::Decided(Aabb2::new_unchecked(
            Point2::new(lower.clone(), bottom.clone()),
            Point2::new(upper.clone(), top.clone()),
        ))
    }
}
