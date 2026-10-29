//! Correlated plane points retain their selected fibers or joint query sources.

use std::cmp::Ordering;

use hypersolve::{BivariatePolynomial, IsolatedRootInterval};

use crate::bezier_offset::BezierAlgebraicSelectedFiberParameter2;
use crate::{
    Aabb2, Axis2, BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, CurveContext, CurveError, CurveOperation2,
    CurveParameter2, CurvePoint2, CurvePointData2, CurveResult, ExactCurveError, ExactCurveResult,
    Point2, Real, UncertaintyReason,
};

/// Coordinates retain their original selected relations and shared field authority.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PolynomialPoint2 {
    authority: PolynomialPointAuthority2,
    maps: Vec<std::sync::Arc<crate::RationalMap2>>,
}

#[derive(Clone, Debug, PartialEq)]
enum PolynomialPointAuthority2 {
    Fiber {
        base: BezierParameter2,
        fiber: BezierAlgebraicSelectedFiberParameter2,
    },
    Queries(std::sync::Arc<JointQueries2>),
}

mod joint;
mod selected_fiber;
use joint::JointQueries2;

impl PolynomialPoint2 {
    pub(crate) fn has_query_authority(&self) -> bool {
        matches!(self.authority, PolynomialPointAuthority2::Queries(_))
    }

    pub(crate) fn has_rational_maps(&self) -> bool {
        !self.maps.is_empty()
    }

    /// Prefix views may have different wrapper allocations while retaining the
    /// same immutable map application or selected fiber. An owning point clone
    /// keeps either key alive for the duration of one native export.
    #[cfg(feature = "svg")]
    pub(crate) fn replay_storage_key(&self) -> (u8, *const ()) {
        if let Some(map) = self.maps.last() {
            (4, std::sync::Arc::as_ptr(map).cast())
        } else {
            match &self.authority {
                PolynomialPointAuthority2::Fiber { fiber, .. } => {
                    (7, std::ptr::from_ref(fiber.root()).cast())
                }
                PolynomialPointAuthority2::Queries(queries) => {
                    (11, std::sync::Arc::as_ptr(queries).cast())
                }
            }
        }
    }

    pub(crate) fn polynomial_sign(
        &self,
        polynomial: &BivariatePolynomial,
        policy: &CurveContext,
    ) -> CurveResult<Classification<hyperreal::RealSign>> {
        let PolynomialPointAuthority2::Fiber { fiber, .. } = &self.authority else {
            return crate::selected_field::finish(|| {
                let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
                crate::selected_field::decided(
                    self.field_query(polynomial, &one, policy)?.sign(policy)?,
                )
            });
        };
        if self.maps.is_empty() {
            return fiber.predicate_sign(polynomial, policy);
        }
        let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let Some((numerator, denominator)) = self.pullback(polynomial, &one) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let sign = match fiber.predicate_sign(&denominator, policy)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if sign == hyperreal::RealSign::Zero {
            return Err(CurveError::ZeroRationalPointDenominator);
        }
        Ok(fiber.predicate_sign(&numerator, policy)?.map(|value| {
            if sign == hyperreal::RealSign::Negative {
                match value {
                    hyperreal::RealSign::Positive => hyperreal::RealSign::Negative,
                    hyperreal::RealSign::Negative => hyperreal::RealSign::Positive,
                    value => value,
                }
            } else {
                value
            }
        }))
    }

    pub(crate) fn rational_parameter(
        &self,
        numerator: &BivariatePolynomial,
        denominator: &BivariatePolynomial,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveParameter2>> {
        let PolynomialPointAuthority2::Fiber { fiber, .. } = &self.authority else {
            return self.joint_parameter(numerator, denominator, policy);
        };
        if self.maps.is_empty() {
            return Ok(fiber
                .rational_image_in_retained_field(numerator, denominator, policy)?
                .map(CurveParameter2::from_selected_fiber));
        }
        let Some((numerator, denominator)) = self.pullback(numerator, denominator) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(fiber
            .rational_image_in_retained_field(&numerator, &denominator, policy)?
            .map(CurveParameter2::from_selected_fiber))
    }

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
                    authority: PolynomialPointAuthority2::Fiber {
                        base: base.clone(),
                        fiber,
                    },
                    maps: Vec::new(),
                })))
            })
            .collect())
    }

    pub(crate) fn definition(&self) -> crate::CurvePointDefinition2 {
        if !self.maps.is_empty() {
            let mut source = self.clone();
            let map = source.maps.pop().expect("nonempty map chain");
            return crate::CurvePointDefinition2::RationalImage {
                source: CurvePoint2(CurvePointData2::Polynomial(std::sync::Arc::new(source))),
                map: (*map).clone(),
            };
        }
        match &self.authority {
            PolynomialPointAuthority2::Fiber { base, fiber } => {
                crate::CurvePointDefinition2::PolynomialFiber {
                    equation: fiber.defining_fiber().0.clone(),
                    parameter: base.clone().into(),
                    root: fiber.root().clone(),
                }
            }
            PolynomialPointAuthority2::Queries(queries) => queries.definition(),
        }
    }

    pub(crate) fn mapped(&self, map: &crate::RationalMap2) -> CurvePoint2 {
        let mut point = self.clone();
        point.maps.push(std::sync::Arc::new(map.clone()));
        CurvePoint2(CurvePointData2::Polynomial(std::sync::Arc::new(point)))
    }

    fn pullback(
        &self,
        numerator: &BivariatePolynomial,
        denominator: &BivariatePolynomial,
    ) -> Option<(BivariatePolynomial, BivariatePolynomial)> {
        let mut pair = (numerator.clone(), denominator.clone());
        for map in self.maps.iter().rev() {
            pair = map.pullback(&pair.0, &pair.1)?;
        }
        Some(pair)
    }

    pub(crate) fn source_query(
        &self,
        numerator: &BivariatePolynomial,
        denominator: &BivariatePolynomial,
    ) -> Option<crate::bezier_offset::PointFormSource2> {
        let (numerator, denominator) = self.pullback(numerator, denominator)?;
        let mut source = self.clone();
        source.maps.clear();
        Some(crate::bezier_offset::PointFormSource2::new(
            CurvePoint2(CurvePointData2::Polynomial(std::sync::Arc::new(source))),
            numerator,
            denominator,
        ))
    }

    pub(crate) fn coordinate_parameter(
        &self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveParameter2>> {
        if self.maps.is_empty() && matches!(self.authority, PolynomialPointAuthority2::Fiber { .. })
        {
            return Ok(Classification::Decided(self.source_coordinate(axis)));
        }
        let polynomial = match axis {
            Axis2::X => BivariatePolynomial::new(vec![vec![Real::zero()], vec![Real::one()]]),
            Axis2::Y => BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()]]),
        };
        self.rational_parameter(
            &polynomial,
            &BivariatePolynomial::new(vec![vec![Real::one()]]),
            policy,
        )
    }

    fn source_coordinate(&self, axis: Axis2) -> CurveParameter2 {
        let PolynomialPointAuthority2::Fiber { base, fiber } = &self.authority else {
            unreachable!("native fiber coordinate")
        };
        match axis {
            Axis2::X => base.clone().into(),
            Axis2::Y => CurveParameter2::from_selected_fiber(fiber.clone()),
        }
    }

    pub(crate) fn affine_order(
        &self,
        a: &Real,
        b: &Real,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>> {
        self.polynomial_sign(
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
        self.polynomial_sign(
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
        if !self.maps.is_empty() || matches!(self.authority, PolynomialPointAuthority2::Queries(_))
        {
            return self.rational_parameter(
                &BivariatePolynomial::new(vec![vec![c.clone(), b.clone()], vec![a.clone()]]),
                &BivariatePolynomial::new(vec![vec![Real::one()]]),
                policy,
            );
        }
        if a.definitely_zero() {
            if b.definitely_zero() {
                return Ok(Classification::Decided(c.clone().into()));
            }
            return self
                .source_coordinate(Axis2::Y)
                .affine_image_unbounded(b, c, policy);
        }
        if b.definitely_zero() {
            return self
                .source_coordinate(Axis2::X)
                .affine_image_unbounded(a, c, policy);
        }
        let PolynomialPointAuthority2::Fiber { base, fiber } = &self.authority else {
            unreachable!("native fiber affine form")
        };
        if let BezierParameter2::Exact(base) = base {
            return self.source_coordinate(Axis2::Y).affine_image_unbounded(
                b,
                &(a * base + c),
                policy,
            );
        }
        Ok(fiber
            .rational_image_in_retained_field(
                &BivariatePolynomial::new(vec![vec![c.clone(), b.clone()], vec![a.clone()]]),
                &BivariatePolynomial::new(vec![vec![Real::one()]]),
                policy,
            )?
            .map(CurveParameter2::from_selected_fiber))
    }

    pub(crate) fn bounds(&self, steps: usize, policy: &CurveContext) -> Classification<Aabb2> {
        let PolynomialPointAuthority2::Fiber { base, fiber } = &self.authority else {
            return self.joint_bounds(steps, policy);
        };
        let base = base.clone().refined_isolating_interval(steps, policy);
        let (lower, upper) = match &base {
            BezierParameter2::Exact(value) => (value, value),
            BezierParameter2::Algebraic(value) => {
                (value.interval().start(), value.interval().end())
            }
        };
        let fiber = match fiber.refined(steps, policy) {
            Ok(Classification::Decided(fiber)) => fiber,
            Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
            Err(CurveError::InvalidBezierAlgebraicParameter) => {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Predicate),
        };
        let (bottom, top) = fiber.isolating_bounds();
        let mut bounds = Aabb2::new_unchecked(
            Point2::new(lower.clone(), bottom.clone()),
            Point2::new(upper.clone(), top.clone()),
        );
        for map in &self.maps {
            let Ok(image) = map.bounds(&bounds) else {
                return Classification::Uncertain(UncertaintyReason::Predicate);
            };
            bounds = image;
        }
        Classification::Decided(bounds)
    }
}
