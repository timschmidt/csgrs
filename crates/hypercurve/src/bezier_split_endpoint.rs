//! Exact algebraic endpoint evidence for Bezier split fragments.
//!
//! A split fragment with an algebraic boundary is not yet a native Bezier
//! subcurve: de Casteljau subdivision needs exact arithmetic in the parameter
//! itself.  The endpoint point and tangent, however, are valid constructed
//! exact objects once the boundary parameter is represented as a root.  This
//! module keeps those endpoint images as first-class evidence so later
//! arrangement code can consume certified predicates without sampling the
//! isolating interval.  That follows the exact-geometric-computation
//! separation between exact object construction and certified branching; see
//! exact-computation discipline.  The point/tangent formulas are the standard
//! polynomial and homogeneous rational Bezier identities from the Bernstein and de Casteljau curve model.

use std::{sync::Arc, sync::OnceLock};

use crate::{
    BezierAlgebraicParameter2, BezierSubcurve2, Classification, CubicBezier2, CurveContext,
    CurveResult, QuadraticBezier2, RationalBezierAlgebraicPointImage2,
    RationalBezierAlgebraicTangentImage2, RationalQuadraticBezier2,
};

/// Exact point and tangent images for one algebraic split endpoint.
#[derive(Clone, Debug)]
pub struct BezierAlgebraicEndpointImage2 {
    data: Arc<BezierAlgebraicEndpointImageData>,
}

#[derive(Clone, Debug)]
enum BezierAlgebraicEndpointImageData {
    Materialized {
        parameter: BezierAlgebraicParameter2,
        point: RationalBezierAlgebraicPointImage2,
        tangent: RationalBezierAlgebraicTangentImage2,
        second_derivative: Option<RationalBezierAlgebraicTangentImage2>,
        third_derivative: Option<RationalBezierAlgebraicTangentImage2>,
    },
    LazyFirstOrder {
        parameter: BezierAlgebraicParameter2,
        curve: Box<BezierSubcurve2>,
        policy: CurveContext,
        point: OnceLock<RationalBezierAlgebraicPointImage2>,
        tangent: OnceLock<RationalBezierAlgebraicTangentImage2>,
    },
}

impl PartialEq for BezierAlgebraicEndpointImage2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.parameter() == other.parameter()
                && self.point() == other.point()
                && self.tangent() == other.tangent()
                && self.second_derivative() == other.second_derivative()
                && self.third_derivative() == other.third_derivative())
    }
}

impl BezierAlgebraicEndpointImage2 {
    /// Constructs endpoint evidence for any retained source Bezier family.
    pub fn from_source_curve(
        source_curve: &BezierSubcurve2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match source_curve {
            BezierSubcurve2::Quadratic(curve) => Self::quadratic(curve, parameter, policy),
            BezierSubcurve2::Cubic(curve) => Self::cubic(curve, parameter, policy),
            BezierSubcurve2::RationalQuadratic(curve) => {
                Self::rational_quadratic(curve, parameter, policy)
            }
            BezierSubcurve2::Rational(curve) => Self::rational(curve, parameter, policy),
        }
    }

    /// Retains replayable point/tangent evidence against one owned source.
    ///
    /// Region transforms need first-order endpoint evidence for connectivity
    /// and tangent ordering, but eagerly materializing higher derivatives can
    /// turn a valid high-degree rational image into an unnecessary algebraic
    /// tower.  This compact form evaluates the exact transformed source lazily
    /// and is therefore the authoritative transport path.
    pub(crate) fn from_source_curve_first_order(
        source_curve: &BezierSubcurve2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> Self {
        Self {
            data: Arc::new(BezierAlgebraicEndpointImageData::LazyFirstOrder {
                parameter: parameter.clone(),
                curve: Box::new(source_curve.clone()),
                policy: *policy,
                point: OnceLock::new(),
                tangent: OnceLock::new(),
            }),
        }
    }

    /// Constructs endpoint evidence for a polynomial quadratic Bezier.
    pub fn quadratic(
        curve: &QuadraticBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let point = match curve.point_at_algebraic_parameter(parameter, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let tangent = match curve.tangent_at_algebraic_parameter(parameter, policy)? {
            Classification::Decided(image) => image,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let second_derivative =
            match curve.second_derivative_at_algebraic_parameter(parameter, policy)? {
                Classification::Decided(image) => image,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        Ok(Classification::Decided(Self {
            data: Arc::new(BezierAlgebraicEndpointImageData::Materialized {
                parameter: parameter.clone(),
                point,
                tangent,
                second_derivative: Some(second_derivative),
                third_derivative: None,
            }),
        }))
    }

    /// Constructs endpoint evidence for a polynomial cubic Bezier.
    pub fn cubic(
        curve: &CubicBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let point = match curve.point_at_algebraic_parameter(parameter, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let tangent = match curve.tangent_at_algebraic_parameter(parameter, policy)? {
            Classification::Decided(image) => image,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let second_derivative =
            match curve.second_derivative_at_algebraic_parameter(parameter, policy)? {
                Classification::Decided(image) => image,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        let third_derivative =
            match curve.third_derivative_at_algebraic_parameter(parameter, policy)? {
                Classification::Decided(image) => image,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        Ok(Classification::Decided(Self {
            data: Arc::new(BezierAlgebraicEndpointImageData::Materialized {
                parameter: parameter.clone(),
                point,
                tangent,
                second_derivative: Some(second_derivative),
                third_derivative: Some(third_derivative),
            }),
        }))
    }

    /// Constructs endpoint evidence for a rational quadratic Bezier/conic.
    pub fn rational_quadratic(
        curve: &RationalQuadraticBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let point = match curve.point_at_algebraic_parameter(parameter, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let mut derivatives =
            match curve.derivatives_at_algebraic_parameter(parameter, 3, policy)? {
                Classification::Decided(images) => images.into_iter(),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        let tangent = derivatives
            .next()
            .expect("three requested rational derivative images");
        let second_derivative = derivatives.next();
        let third_derivative = derivatives.next();
        Ok(Classification::Decided(Self {
            data: Arc::new(BezierAlgebraicEndpointImageData::Materialized {
                parameter: parameter.clone(),
                point,
                tangent,
                second_derivative,
                third_derivative,
            }),
        }))
    }

    /// Constructs endpoint evidence for an arbitrary-degree rational Bezier.
    pub fn rational(
        curve: &crate::RationalBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let point = match curve.point_at_algebraic_parameter(parameter, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let mut derivatives =
            match curve.derivatives_at_algebraic_parameter(parameter, 3, policy)? {
                Classification::Decided(images) => images.into_iter(),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        let tangent = derivatives
            .next()
            .expect("three requested rational derivative images");
        let second_derivative = derivatives.next();
        let third_derivative = derivatives.next();
        Ok(Classification::Decided(Self {
            data: Arc::new(BezierAlgebraicEndpointImageData::Materialized {
                parameter: parameter.clone(),
                point,
                tangent,
                second_derivative,
                third_derivative,
            }),
        }))
    }

    /// Returns the algebraic Bezier parameter at this endpoint.
    pub fn parameter(&self) -> &BezierAlgebraicParameter2 {
        match self.data.as_ref() {
            BezierAlgebraicEndpointImageData::Materialized { parameter, .. }
            | BezierAlgebraicEndpointImageData::LazyFirstOrder { parameter, .. } => parameter,
        }
    }

    /// Returns retained exact second-derivative evidence, including a source
    /// expression whose coordinate roots have not been materialized.
    pub fn second_derivative(&self) -> Option<&RationalBezierAlgebraicTangentImage2> {
        match self.data.as_ref() {
            BezierAlgebraicEndpointImageData::Materialized {
                second_derivative, ..
            } => second_derivative.as_ref(),
            BezierAlgebraicEndpointImageData::LazyFirstOrder { .. } => None,
        }
    }

    /// Returns retained exact third-derivative evidence, including a source
    /// expression whose coordinate roots have not been materialized.
    pub fn third_derivative(&self) -> Option<&RationalBezierAlgebraicTangentImage2> {
        match self.data.as_ref() {
            BezierAlgebraicEndpointImageData::Materialized {
                third_derivative, ..
            } => third_derivative.as_ref(),
            BezierAlgebraicEndpointImageData::LazyFirstOrder { .. } => None,
        }
    }

    /// Returns true when both point and tangent retain exact replayable evidence.
    pub fn is_exact(&self) -> bool {
        self.point()
            .is_ok_and(|point| matches!(point, Classification::Decided(_)))
            && self.tangent().is_ok_and(|tangent| tangent.is_decided())
    }

    pub(crate) fn matches_required_source_evidence(&self, expected: &Self) -> bool {
        self.parameter() == expected.parameter()
            && self.point() == expected.point()
            && self.tangent() == expected.tangent()
            && self
                .second_derivative()
                .is_none_or(|derivative| Some(derivative) == expected.second_derivative())
            && self
                .third_derivative()
                .is_none_or(|derivative| Some(derivative) == expected.third_derivative())
    }

    pub(crate) fn is_lazy_first_order(&self) -> bool {
        matches!(
            self.data.as_ref(),
            BezierAlgebraicEndpointImageData::LazyFirstOrder { .. }
        )
    }

    pub(crate) fn is_exact_or_lazy_first_order(&self) -> bool {
        self.is_lazy_first_order() || self.is_exact()
    }

    /// Returns a certified affine endpoint image, or the construction blocker.
    /// Lazy rational sources must prove a finite point before exposing one.
    pub fn point(&self) -> CurveResult<Classification<&RationalBezierAlgebraicPointImage2>> {
        match self.data.as_ref() {
            BezierAlgebraicEndpointImageData::Materialized { point, .. } => {
                Ok(Classification::Decided(point))
            }
            BezierAlgebraicEndpointImageData::LazyFirstOrder {
                parameter,
                curve,
                policy,
                point,
                ..
            } => {
                if let Some(image) = point.get() {
                    return Ok(Classification::Decided(image));
                }
                let image = match curve.as_ref() {
                    BezierSubcurve2::Quadratic(curve) => {
                        curve.point_at_algebraic_parameter(parameter, policy)
                    }
                    BezierSubcurve2::Cubic(curve) => {
                        curve.point_at_algebraic_parameter(parameter, policy)
                    }
                    BezierSubcurve2::RationalQuadratic(curve) => {
                        curve.point_at_algebraic_parameter(parameter, policy)
                    }
                    BezierSubcurve2::Rational(curve) => {
                        curve.point_at_algebraic_parameter(parameter, policy)
                    }
                }?;
                Ok(image.map(|image| {
                    // Selected-root refinement can make a later request
                    // decidable. Publish only a successful finite image.
                    let _ = point.set(image);
                    point
                        .get()
                        .expect("a successful endpoint image was published")
                }))
            }
        }
    }

    /// Returns a certified finite tangent image, or the construction blocker.
    /// Successful lazy images are shared; a later request may retry unresolved evidence.
    pub fn tangent(&self) -> CurveResult<Classification<&RationalBezierAlgebraicTangentImage2>> {
        match self.data.as_ref() {
            BezierAlgebraicEndpointImageData::Materialized { tangent, .. } => {
                Ok(Classification::Decided(tangent))
            }
            BezierAlgebraicEndpointImageData::LazyFirstOrder {
                parameter,
                curve,
                policy,
                tangent,
                ..
            } => {
                if let Some(image) = tangent.get() {
                    return Ok(Classification::Decided(image));
                }
                let image = match curve.as_ref() {
                    BezierSubcurve2::Quadratic(curve) => {
                        curve.tangent_at_algebraic_parameter(parameter, policy)
                    }
                    BezierSubcurve2::Cubic(curve) => {
                        curve.tangent_at_algebraic_parameter(parameter, policy)
                    }
                    BezierSubcurve2::RationalQuadratic(curve) => curve
                        .derivatives_at_algebraic_parameter(parameter, 1, policy)
                        .map(|images| {
                            images.map(|mut images| {
                                images
                                    .pop()
                                    .expect("one requested rational derivative image")
                            })
                        }),
                    BezierSubcurve2::Rational(curve) => curve
                        .derivatives_at_algebraic_parameter(parameter, 1, policy)
                        .map(|images| {
                            images.map(|mut images| {
                                images
                                    .pop()
                                    .expect("one requested rational derivative image")
                            })
                        }),
                }?;
                Ok(image.map(|image| {
                    let _ = tangent.set(image);
                    tangent
                        .get()
                        .expect("a successful endpoint tangent was published")
                }))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BezierAlgebraicImageStatus, BezierParameterInterval, BezierParameterPolynomial, Point2,
        RationalBezier2, Real, UncertaintyReason,
    };

    #[test]
    fn retained_derivatives_replay_nonrational_source_signs() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let polynomial = crate::tests::decided(
                BezierParameterPolynomial::try_new_power_basis(
                    vec![-Real::pi(), Real::zero(), Real::zero(), Real::from(4)],
                    &policy,
                )
                .unwrap(),
            );
            let interval = crate::tests::decided(
                BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy).unwrap(),
            );
            let parameter = crate::tests::decided(
                BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap(),
            );
            let curve = CubicBezier2::new(
                Point2::from_values(0, 0),
                Point2::from_values(0, 1),
                Point2::from_values(0, 2),
                Point2::new(Real::pi(), Real::from(3)),
            );
            let endpoint = crate::tests::decided(
                BezierAlgebraicEndpointImage2::cubic(&curve, &parameter, &policy).unwrap(),
            );
            for (order, image) in [
                crate::tests::decided(endpoint.tangent().unwrap()),
                endpoint.second_derivative().unwrap(),
                endpoint.third_derivative().unwrap(),
            ]
            .into_iter()
            .enumerate()
            {
                // C(t) = (pi*t^3, 3t) at the positive root of 4t^3 - pi.
                if order == 0 {
                    assert_eq!(
                        image.status(),
                        BezierAlgebraicImageStatus::RetainedRationalExpression
                    );
                }
                assert_eq!(
                    image.coordinate_sign(true, &policy).unwrap(),
                    Classification::Decided(crate::RealSign::Positive)
                );
                assert_eq!(
                    image.coordinate_sign(false, &policy).unwrap(),
                    Classification::Decided(if order == 0 {
                        crate::RealSign::Positive
                    } else {
                        crate::RealSign::Zero
                    })
                );
            }
            assert!(endpoint.is_exact());
        }
    }

    #[test]
    fn rational_endpoints_retain_nonrational_higher_derivatives() {
        let q = |n: i32, d: i32| (Real::from(n) / Real::from(d)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let polynomial = crate::tests::decided(
                BezierParameterPolynomial::try_new_power_basis(
                    vec![-Real::pi(), Real::zero(), Real::zero(), Real::from(4)],
                    &policy,
                )
                .unwrap(),
            );
            let interval = crate::tests::decided(
                BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy).unwrap(),
            );
            let parameter = crate::tests::decided(
                BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap(),
            );
            // C=(1/(1+t),t/(1+t)). At the positive root of 4t^3-pi,
            // the kth x derivative is (-1)^k*k!/(1+t)^(k+1), and y=1-x.
            let conic = RationalQuadraticBezier2::try_new(
                Point2::from_values(1, 0),
                Point2::new(q(2, 3), q(1, 3)),
                Point2::new(q(1, 2), q(1, 2)),
                Real::one(),
                q(3, 2),
                Real::from(2),
            )
            .unwrap();
            let general = RationalBezier2::from(conic.clone());
            for endpoint in [
                BezierAlgebraicEndpointImage2::rational_quadratic(&conic, &parameter, &policy),
                BezierAlgebraicEndpointImage2::rational(&general, &parameter, &policy),
            ] {
                let endpoint = crate::tests::decided(endpoint.unwrap());
                for (index, derivative) in [
                    crate::tests::decided(endpoint.tangent().unwrap()),
                    endpoint
                        .second_derivative()
                        .expect("exact second derivative"),
                    endpoint.third_derivative().expect("exact third derivative"),
                ]
                .into_iter()
                .enumerate()
                {
                    assert_eq!(
                        derivative.status(),
                        BezierAlgebraicImageStatus::RetainedRationalExpression
                    );
                    assert!(derivative.retained_parameter() == Some(&parameter));
                    for use_x in [true, false] {
                        let positive = (index % 2 == 1) == use_x;
                        assert_eq!(
                            derivative.coordinate_sign(use_x, &policy).unwrap(),
                            Classification::Decided(if positive {
                                crate::RealSign::Positive
                            } else {
                                crate::RealSign::Negative
                            })
                        );
                    }
                }
                assert!(endpoint.is_exact());
            }
        }
    }

    #[test]
    fn lazy_endpoint_images_preserve_affine_domain_blockers() {
        let conic = RationalQuadraticBezier2::try_new(
            Point2::from_values(0, 0),
            Point2::from_values(1, 1),
            Point2::from_values(2, 0),
            Real::one(),
            -Real::one(),
            Real::one(),
        )
        .unwrap();
        let sources = [
            BezierSubcurve2::RationalQuadratic(conic.clone()),
            BezierSubcurve2::Rational(RationalBezier2::from(conic)),
        ];
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let polynomial = crate::tests::decided(
                BezierParameterPolynomial::try_new_power_basis(
                    vec![Real::from(-1), Real::from(2)],
                    &policy,
                )
                .unwrap(),
            );
            let interval = crate::tests::decided(
                BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy).unwrap(),
            );
            let parameter = crate::tests::decided(
                BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap(),
            );
            for source in &sources {
                let endpoint = BezierAlgebraicEndpointImage2::from_source_curve_first_order(
                    source, &parameter, &policy,
                );
                for _ in 0..2 {
                    assert!(matches!(
                        endpoint.point(),
                        Ok(Classification::Uncertain(UncertaintyReason::Boundary))
                    ));
                    assert!(matches!(
                        endpoint.tangent(),
                        Ok(Classification::Uncertain(UncertaintyReason::Boundary))
                    ));
                    assert!(!endpoint.is_exact());
                }
            }
        }
    }
}
