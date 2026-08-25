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
    BezierAlgebraicImageStatus, BezierAlgebraicParameter2, BezierAlgebraicPointImage2,
    BezierAlgebraicTangentImage2, BezierSubcurve2, CubicBezier2, CurveContext, CurveResult,
    QuadraticBezier2, RationalBezierAlgebraicPointImage2, RationalBezierAlgebraicTangentImage2,
    RationalQuadraticBezier2,
};

/// Exact point image retained at an algebraic split endpoint.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub enum BezierEndpointPointImage2 {
    /// Polynomial quadratic/cubic Bezier coordinate images.
    Polynomial(BezierAlgebraicPointImage2),
    /// Rational Bezier affine coordinate images of any degree.
    Rational(RationalBezierAlgebraicPointImage2),
}

impl BezierEndpointPointImage2 {
    /// Returns the construction status for the retained point image.
    pub fn status(&self) -> BezierAlgebraicImageStatus {
        match self {
            Self::Polynomial(image) => image.status(),
            Self::Rational(image) => image.status(),
        }
    }

    /// Returns true when both coordinates retain exact replayable evidence.
    pub fn is_exact(&self) -> bool {
        matches!(
            self.status(),
            BezierAlgebraicImageStatus::Transformed
                | BezierAlgebraicImageStatus::RetainedRationalExpression
        )
    }
}

/// Exact tangent image retained at an algebraic split endpoint.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub enum BezierEndpointTangentImage2 {
    /// Polynomial quadratic/cubic Bezier derivative coordinate images.
    Polynomial(BezierAlgebraicTangentImage2),
    /// Rational Bezier affine derivative coordinate images of any degree.
    Rational(RationalBezierAlgebraicTangentImage2),
}

impl BezierEndpointTangentImage2 {
    /// Returns the construction status for the retained tangent image.
    pub fn status(&self) -> BezierAlgebraicImageStatus {
        match self {
            Self::Polynomial(image) => image.status(),
            Self::Rational(image) => image.status(),
        }
    }

    /// Returns true when both tangent coordinates retain exact replayable evidence.
    pub fn is_exact(&self) -> bool {
        matches!(
            self.status(),
            BezierAlgebraicImageStatus::Transformed
                | BezierAlgebraicImageStatus::RetainedRationalExpression
        )
    }
}

/// Exact point and tangent images for one algebraic split endpoint.
#[derive(Clone, Debug)]
pub struct BezierAlgebraicEndpointImage2 {
    data: Arc<BezierAlgebraicEndpointImageData>,
}

#[derive(Clone, Debug)]
enum BezierAlgebraicEndpointImageData {
    Materialized {
        parameter: BezierAlgebraicParameter2,
        point: BezierEndpointPointImage2,
        tangent: BezierEndpointTangentImage2,
        second_derivative: Option<Box<BezierEndpointTangentImage2>>,
        third_derivative: Option<Box<BezierEndpointTangentImage2>>,
    },
    LazyFirstOrder {
        parameter: BezierAlgebraicParameter2,
        curve: Box<BezierSubcurve2>,
        policy: CurveContext,
        point: OnceLock<CurveResult<BezierEndpointPointImage2>>,
        tangent: OnceLock<CurveResult<BezierEndpointTangentImage2>>,
    },
}

impl PartialEq for BezierAlgebraicEndpointImage2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.parameter() == other.parameter()
                && self.try_point() == other.try_point()
                && self.try_tangent() == other.try_tangent()
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
    ) -> CurveResult<Self> {
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
    ) -> CurveResult<Self> {
        match source_curve {
            BezierSubcurve2::Quadratic(curve) => {
                Self::quadratic_first_order(curve, parameter, policy)
            }
            BezierSubcurve2::Cubic(curve) => Self::cubic_first_order(curve, parameter, policy),
            BezierSubcurve2::RationalQuadratic(curve) => {
                Self::rational_quadratic_first_order(curve, parameter, policy)
            }
            BezierSubcurve2::Rational(curve) => {
                Self::rational_first_order(curve, parameter, policy)
            }
        }
    }

    /// Constructs endpoint evidence for a polynomial quadratic Bezier.
    pub fn quadratic(
        curve: &QuadraticBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        Ok(Self {
            data: Arc::new(BezierAlgebraicEndpointImageData::Materialized {
                parameter: parameter.clone(),
                point: BezierEndpointPointImage2::Polynomial(
                    curve.point_at_algebraic_parameter(parameter, policy)?,
                ),
                tangent: BezierEndpointTangentImage2::Polynomial(
                    curve.tangent_at_algebraic_parameter(parameter, policy)?,
                ),
                second_derivative: Some(Box::new(BezierEndpointTangentImage2::Polynomial(
                    curve.second_derivative_at_algebraic_parameter(parameter, policy)?,
                ))),
                third_derivative: None,
            }),
        })
    }

    /// Constructs endpoint evidence for a polynomial cubic Bezier.
    pub fn cubic(
        curve: &CubicBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        Ok(Self {
            data: Arc::new(BezierAlgebraicEndpointImageData::Materialized {
                parameter: parameter.clone(),
                point: BezierEndpointPointImage2::Polynomial(
                    curve.point_at_algebraic_parameter(parameter, policy)?,
                ),
                tangent: BezierEndpointTangentImage2::Polynomial(
                    curve.tangent_at_algebraic_parameter(parameter, policy)?,
                ),
                second_derivative: Some(Box::new(BezierEndpointTangentImage2::Polynomial(
                    curve.second_derivative_at_algebraic_parameter(parameter, policy)?,
                ))),
                third_derivative: Some(Box::new(BezierEndpointTangentImage2::Polynomial(
                    curve.third_derivative_at_algebraic_parameter(parameter, policy)?,
                ))),
            }),
        })
    }

    /// Constructs endpoint evidence for a rational quadratic Bezier/conic.
    pub fn rational_quadratic(
        curve: &RationalQuadraticBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        let mut derivatives = curve
            .derivatives_at_algebraic_parameter(parameter, 3, policy)?
            .into_iter();
        let tangent = derivatives
            .next()
            .expect("three requested rational derivative images");
        let second_derivative = derivatives.next().and_then(transformed_rational_derivative);
        let third_derivative = derivatives.next().and_then(transformed_rational_derivative);
        Ok(Self {
            data: Arc::new(BezierAlgebraicEndpointImageData::Materialized {
                parameter: parameter.clone(),
                point: BezierEndpointPointImage2::Rational(
                    curve.point_at_algebraic_parameter(parameter, policy)?,
                ),
                tangent: BezierEndpointTangentImage2::Rational(tangent),
                second_derivative,
                third_derivative,
            }),
        })
    }

    pub(crate) fn rational_quadratic_first_order(
        curve: &RationalQuadraticBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        Ok(Self {
            data: Arc::new(BezierAlgebraicEndpointImageData::LazyFirstOrder {
                parameter: parameter.clone(),
                curve: Box::new(BezierSubcurve2::RationalQuadratic(curve.clone())),
                policy: *policy,
                point: OnceLock::new(),
                tangent: OnceLock::new(),
            }),
        })
    }

    /// Constructs endpoint evidence for an arbitrary-degree rational Bezier.
    pub fn rational(
        curve: &crate::RationalBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        let mut derivatives = curve
            .derivatives_at_algebraic_parameter(parameter, 3, policy)?
            .into_iter();
        let tangent = derivatives
            .next()
            .expect("three requested rational derivative images");
        let second_derivative = derivatives.next().and_then(transformed_rational_derivative);
        let third_derivative = derivatives.next().and_then(transformed_rational_derivative);
        Ok(Self {
            data: Arc::new(BezierAlgebraicEndpointImageData::Materialized {
                parameter: parameter.clone(),
                point: BezierEndpointPointImage2::Rational(
                    curve.point_at_algebraic_parameter(parameter, policy)?,
                ),
                tangent: BezierEndpointTangentImage2::Rational(tangent),
                second_derivative,
                third_derivative,
            }),
        })
    }

    pub(crate) fn rational_first_order(
        curve: &crate::RationalBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        Ok(Self {
            data: Arc::new(BezierAlgebraicEndpointImageData::LazyFirstOrder {
                parameter: parameter.clone(),
                curve: Box::new(BezierSubcurve2::Rational(curve.clone())),
                policy: *policy,
                point: OnceLock::new(),
                tangent: OnceLock::new(),
            }),
        })
    }

    pub(crate) fn quadratic_first_order(
        curve: &QuadraticBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        Ok(Self {
            data: Arc::new(BezierAlgebraicEndpointImageData::LazyFirstOrder {
                parameter: parameter.clone(),
                curve: Box::new(BezierSubcurve2::Quadratic(curve.clone())),
                policy: *policy,
                point: OnceLock::new(),
                tangent: OnceLock::new(),
            }),
        })
    }

    pub(crate) fn cubic_first_order(
        curve: &CubicBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        Ok(Self {
            data: Arc::new(BezierAlgebraicEndpointImageData::LazyFirstOrder {
                parameter: parameter.clone(),
                curve: Box::new(BezierSubcurve2::Cubic(curve.clone())),
                policy: *policy,
                point: OnceLock::new(),
                tangent: OnceLock::new(),
            }),
        })
    }

    /// Returns the algebraic Bezier parameter at this endpoint.
    pub fn parameter(&self) -> &BezierAlgebraicParameter2 {
        match self.data.as_ref() {
            BezierAlgebraicEndpointImageData::Materialized { parameter, .. }
            | BezierAlgebraicEndpointImageData::LazyFirstOrder { parameter, .. } => parameter,
        }
    }

    /// Returns the exact point image at the endpoint.
    pub fn point(&self) -> &BezierEndpointPointImage2 {
        self.try_point()
            .expect("certified private split endpoint point image must remain constructible")
    }

    /// Returns the exact tangent image at the endpoint.
    pub fn tangent(&self) -> &BezierEndpointTangentImage2 {
        self.try_tangent()
            .expect("certified private split endpoint tangent image must remain constructible")
    }

    /// Returns exact second-derivative endpoint evidence when the source curve
    /// family can currently construct it.
    pub fn second_derivative(&self) -> Option<&BezierEndpointTangentImage2> {
        match self.data.as_ref() {
            BezierAlgebraicEndpointImageData::Materialized {
                second_derivative, ..
            } => second_derivative.as_deref(),
            BezierAlgebraicEndpointImageData::LazyFirstOrder { .. } => None,
        }
    }

    /// Returns exact third-derivative endpoint evidence when retained.
    pub fn third_derivative(&self) -> Option<&BezierEndpointTangentImage2> {
        match self.data.as_ref() {
            BezierAlgebraicEndpointImageData::Materialized {
                third_derivative, ..
            } => third_derivative.as_deref(),
            BezierAlgebraicEndpointImageData::LazyFirstOrder { .. } => None,
        }
    }

    /// Returns true when both point and tangent retain exact replayable evidence.
    pub fn is_exact(&self) -> bool {
        self.try_point().is_ok_and(|point| point.is_exact())
            && self.try_tangent().is_ok_and(|tangent| tangent.is_exact())
    }

    pub(crate) fn matches_required_source_evidence(&self, expected: &Self) -> bool {
        self.parameter() == expected.parameter()
            && self.try_point() == expected.try_point()
            && self.try_tangent() == expected.try_tangent()
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

    pub(crate) fn try_point(&self) -> CurveResult<&BezierEndpointPointImage2> {
        match self.data.as_ref() {
            BezierAlgebraicEndpointImageData::Materialized { point, .. } => Ok(point),
            BezierAlgebraicEndpointImageData::LazyFirstOrder {
                parameter,
                curve,
                policy,
                point,
                ..
            } => point
                .get_or_init(|| match curve.as_ref() {
                    BezierSubcurve2::Quadratic(curve) => curve
                        .point_at_algebraic_parameter(parameter, policy)
                        .map(BezierEndpointPointImage2::Polynomial),
                    BezierSubcurve2::Cubic(curve) => curve
                        .point_at_algebraic_parameter(parameter, policy)
                        .map(BezierEndpointPointImage2::Polynomial),
                    BezierSubcurve2::RationalQuadratic(curve) => curve
                        .point_at_algebraic_parameter(parameter, policy)
                        .map(BezierEndpointPointImage2::Rational),
                    BezierSubcurve2::Rational(curve) => curve
                        .point_at_algebraic_parameter(parameter, policy)
                        .map(BezierEndpointPointImage2::Rational),
                })
                .as_ref()
                .map_err(Clone::clone),
        }
    }

    pub(crate) fn try_tangent(&self) -> CurveResult<&BezierEndpointTangentImage2> {
        match self.data.as_ref() {
            BezierAlgebraicEndpointImageData::Materialized { tangent, .. } => Ok(tangent),
            BezierAlgebraicEndpointImageData::LazyFirstOrder {
                parameter,
                curve,
                policy,
                tangent,
                ..
            } => tangent
                .get_or_init(|| {
                    let tangent = match curve.as_ref() {
                        BezierSubcurve2::Quadratic(curve) => {
                            return curve
                                .tangent_at_algebraic_parameter(parameter, policy)
                                .map(BezierEndpointTangentImage2::Polynomial);
                        }
                        BezierSubcurve2::Cubic(curve) => {
                            return curve
                                .tangent_at_algebraic_parameter(parameter, policy)
                                .map(BezierEndpointTangentImage2::Polynomial);
                        }
                        BezierSubcurve2::RationalQuadratic(curve) => {
                            curve.derivatives_at_algebraic_parameter(parameter, 1, policy)
                        }
                        BezierSubcurve2::Rational(curve) => {
                            curve.derivatives_at_algebraic_parameter(parameter, 1, policy)
                        }
                    }?
                    .pop()
                    .expect("one requested rational derivative image");
                    Ok(BezierEndpointTangentImage2::Rational(tangent))
                })
                .as_ref()
                .map_err(Clone::clone),
        }
    }
}

fn transformed_rational_derivative(
    derivative: RationalBezierAlgebraicTangentImage2,
) -> Option<Box<BezierEndpointTangentImage2>> {
    (derivative.status() == BezierAlgebraicImageStatus::Transformed)
        .then(|| Box::new(BezierEndpointTangentImage2::Rational(derivative)))
}
