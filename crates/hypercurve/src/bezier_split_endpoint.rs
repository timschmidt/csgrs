//! Exact algebraic endpoint evidence for Bezier split fragments.
//!
//! A split fragment with an algebraic boundary is not yet a native Bezier
//! subcurve: de Casteljau subdivision needs exact arithmetic in the parameter
//! itself.  The endpoint point and tangent, however, are valid constructed
//! exact objects once the boundary parameter is represented as a root.  This
//! module keeps those endpoint images as first-class evidence so later
//! arrangement code can consume certified predicates without sampling the
//! isolating interval.  That follows Yap's exact-geometric-computation
//! separation between exact object construction and certified branching; see
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7(1-2), 3-23 (1997).  The point/tangent formulas are the standard
//! polynomial and homogeneous rational Bezier identities from Farin, *Curves
//! and Surfaces for Computer-Aided Geometric Design* (5th ed., 2002).

use crate::{
    BezierAlgebraicImageStatus, BezierAlgebraicParameter2, BezierAlgebraicPointImage2,
    BezierAlgebraicTangentImage2, BezierSubcurve2, CubicBezier2, CurvePolicy, CurveResult,
    QuadraticBezier2, RationalBezierAlgebraicPointImage2, RationalBezierAlgebraicTangentImage2,
    RationalQuadraticBezier2,
};

/// Exact point image retained at an algebraic split endpoint.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub enum BezierEndpointPointImage2 {
    /// Polynomial quadratic/cubic Bezier coordinate images.
    Polynomial(BezierAlgebraicPointImage2),
    /// Rational quadratic/conic affine coordinate images.
    RationalQuadratic(RationalBezierAlgebraicPointImage2),
}

impl BezierEndpointPointImage2 {
    /// Returns the construction status for the retained point image.
    pub const fn status(&self) -> BezierAlgebraicImageStatus {
        match self {
            Self::Polynomial(image) => image.status(),
            Self::RationalQuadratic(image) => image.status(),
        }
    }

    /// Returns true when both coordinates were constructed as exact images.
    pub const fn is_transformed(&self) -> bool {
        matches!(self.status(), BezierAlgebraicImageStatus::Transformed)
    }
}

/// Exact tangent image retained at an algebraic split endpoint.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub enum BezierEndpointTangentImage2 {
    /// Polynomial quadratic/cubic Bezier derivative coordinate images.
    Polynomial(BezierAlgebraicTangentImage2),
    /// Rational quadratic/conic affine derivative coordinate images.
    RationalQuadratic(RationalBezierAlgebraicTangentImage2),
}

impl BezierEndpointTangentImage2 {
    /// Returns the construction status for the retained tangent image.
    pub const fn status(&self) -> BezierAlgebraicImageStatus {
        match self {
            Self::Polynomial(image) => image.status(),
            Self::RationalQuadratic(image) => image.status(),
        }
    }

    /// Returns true when both tangent coordinates were constructed exactly.
    pub const fn is_transformed(&self) -> bool {
        matches!(self.status(), BezierAlgebraicImageStatus::Transformed)
    }
}

/// Exact point and tangent images for one algebraic split endpoint.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAlgebraicEndpointImage2 {
    parameter: BezierAlgebraicParameter2,
    point: BezierEndpointPointImage2,
    tangent: BezierEndpointTangentImage2,
    second_derivative: Option<Box<BezierEndpointTangentImage2>>,
    third_derivative: Option<Box<BezierEndpointTangentImage2>>,
}

impl BezierAlgebraicEndpointImage2 {
    /// Constructs endpoint evidence for any retained source Bezier family.
    pub fn from_source_curve(
        source_curve: &BezierSubcurve2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<Self> {
        match source_curve {
            BezierSubcurve2::Quadratic(curve) => Self::quadratic(curve, parameter, policy),
            BezierSubcurve2::Cubic(curve) => Self::cubic(curve, parameter, policy),
            BezierSubcurve2::RationalQuadratic(curve) => {
                Self::rational_quadratic(curve, parameter, policy)
            }
        }
    }

    /// Constructs endpoint evidence for a polynomial quadratic Bezier.
    pub fn quadratic(
        curve: &QuadraticBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<Self> {
        Ok(Self {
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
        })
    }

    /// Constructs endpoint evidence for a polynomial cubic Bezier.
    pub fn cubic(
        curve: &CubicBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<Self> {
        Ok(Self {
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
        })
    }

    /// Constructs endpoint evidence for a rational quadratic Bezier/conic.
    pub fn rational_quadratic(
        curve: &RationalQuadraticBezier2,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<Self> {
        let second_derivative =
            curve.second_derivative_at_algebraic_parameter(parameter, policy)?;
        Ok(Self {
            parameter: parameter.clone(),
            point: BezierEndpointPointImage2::RationalQuadratic(
                curve.point_at_algebraic_parameter(parameter, policy)?,
            ),
            tangent: BezierEndpointTangentImage2::RationalQuadratic(
                curve.tangent_at_algebraic_parameter(parameter, policy)?,
            ),
            second_derivative: (second_derivative.status()
                == BezierAlgebraicImageStatus::Transformed)
                .then_some(Box::new(BezierEndpointTangentImage2::RationalQuadratic(
                    second_derivative,
                ))),
            third_derivative: None,
        })
    }

    /// Returns the algebraic Bezier parameter at this endpoint.
    pub const fn parameter(&self) -> &BezierAlgebraicParameter2 {
        &self.parameter
    }

    /// Returns the exact point image at the endpoint.
    pub const fn point(&self) -> &BezierEndpointPointImage2 {
        &self.point
    }

    /// Returns the exact tangent image at the endpoint.
    pub const fn tangent(&self) -> &BezierEndpointTangentImage2 {
        &self.tangent
    }

    /// Returns exact second-derivative endpoint evidence when the source curve
    /// family can currently construct it.
    pub fn second_derivative(&self) -> Option<&BezierEndpointTangentImage2> {
        self.second_derivative.as_deref()
    }

    /// Returns exact third-derivative endpoint evidence when retained.
    pub fn third_derivative(&self) -> Option<&BezierEndpointTangentImage2> {
        self.third_derivative.as_deref()
    }

    /// Returns true when both point and tangent images were constructed.
    pub const fn is_transformed(&self) -> bool {
        self.point.is_transformed() && self.tangent.is_transformed()
    }
}
