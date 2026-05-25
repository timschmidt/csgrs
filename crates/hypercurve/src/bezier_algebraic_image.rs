//! Algebraic Bezier point and tangent images.
//!
//! This module is the first materialization bridge between
//! [`BezierAlgebraicParameter2`](crate::BezierAlgebraicParameter2) and concrete
//! curve geometry.  It does not approximate an isolated split parameter.
//! Instead it converts the parameter into a
//! [`hypersolve::AlgebraicRootRepresentation`] and evaluates Bezier coordinate
//! polynomials with `hypersolve`'s resultant-backed polynomial-image package.
//! That follows Yap, "Towards Exact Geometric Computation" (1997): constructed
//! coordinates remain exact objects with replayable evidence, while callers
//! branch only on certified predicates.  The coordinate polynomials are the
//! standard Bernstein-to-power identities for Bezier curves; see Farin,
//! *Curves and Surfaces for Computer-Aided Geometric Design* (5th ed., 2002).

use hyperreal::Real;
use hypersolve::{
    AlgebraicRootKind, AlgebraicRootPolynomialImageReport, AlgebraicRootPolynomialImageStatus,
    AlgebraicRootRationalImageReport, AlgebraicRootRationalImageStatus,
    AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    IsolatedRootInterval, SymbolId, transform_algebraic_root_polynomial_image,
    transform_algebraic_root_rational_image, validate_algebraic_root_representation,
};

use crate::classify::compare_reals;
use crate::{
    BezierAlgebraicParameter2, CubicBezier2, CurvePolicy, CurveResult, QuadraticBezier2,
    RationalQuadraticBezier2,
};
use std::cmp::Ordering;

/// Status for a Bezier algebraic point or tangent image.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierAlgebraicImageStatus {
    /// Both coordinate images were represented exactly.
    Transformed,
    /// The Bezier parameter could not be converted into valid represented-root
    /// evidence.
    InvalidParameterEvidence,
    /// The x coordinate image failed the bounded exact polynomial-image
    /// package.
    XImageFailed,
    /// The y coordinate image failed the bounded exact polynomial-image
    /// package.
    YImageFailed,
}

/// One exact coordinate image of a Bezier expression at an algebraic parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAlgebraicCoordinateImage {
    coefficients: Vec<Real>,
    report: AlgebraicRootPolynomialImageReport,
}

/// One exact rational-function coordinate image at an algebraic parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAlgebraicRationalCoordinateImage {
    numerator_coefficients: Vec<Real>,
    denominator_coefficients: Vec<Real>,
    report: AlgebraicRootRationalImageReport,
}

impl BezierAlgebraicRationalCoordinateImage {
    /// Returns numerator coefficients in ascending powers of the source
    /// Bezier parameter.
    pub fn numerator_coefficients(&self) -> &[Real] {
        &self.numerator_coefficients
    }

    /// Returns denominator coefficients in ascending powers of the source
    /// Bezier parameter.
    pub fn denominator_coefficients(&self) -> &[Real] {
        &self.denominator_coefficients
    }

    /// Returns the exact rational-image report produced by `hypersolve`.
    pub const fn report(&self) -> &AlgebraicRootRationalImageReport {
        &self.report
    }

    /// Returns the represented coordinate when the image was constructed.
    pub fn representation(&self) -> Option<&AlgebraicRootRepresentation> {
        self.report.representation.as_ref()
    }
}

impl BezierAlgebraicCoordinateImage {
    /// Returns the coordinate polynomial in ascending powers of the source
    /// Bezier parameter.
    pub fn coefficients(&self) -> &[Real] {
        &self.coefficients
    }

    /// Returns the exact polynomial-image report produced by `hypersolve`.
    pub const fn report(&self) -> &AlgebraicRootPolynomialImageReport {
        &self.report
    }

    /// Returns the represented coordinate when the image was constructed.
    pub fn representation(&self) -> Option<&AlgebraicRootRepresentation> {
        self.report.representation.as_ref()
    }
}

/// Exact algebraic image of a Bezier point.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAlgebraicPointImage2 {
    status: BezierAlgebraicImageStatus,
    parameter: AlgebraicRootRepresentation,
    x: Option<BezierAlgebraicCoordinateImage>,
    y: Option<BezierAlgebraicCoordinateImage>,
    message: Option<String>,
}

impl BezierAlgebraicPointImage2 {
    /// Returns the final construction status.
    pub const fn status(&self) -> BezierAlgebraicImageStatus {
        self.status
    }

    /// Returns the represented Bezier parameter used as the source root.
    pub const fn parameter(&self) -> &AlgebraicRootRepresentation {
        &self.parameter
    }

    /// Returns the x coordinate image when construction reached it.
    pub const fn x(&self) -> Option<&BezierAlgebraicCoordinateImage> {
        self.x.as_ref()
    }

    /// Returns the y coordinate image when construction reached it.
    pub const fn y(&self) -> Option<&BezierAlgebraicCoordinateImage> {
        self.y.as_ref()
    }

    /// Returns a compact diagnostic message for failed construction.
    pub fn message(&self) -> Option<&str> {
        self.message.as_deref()
    }
}

/// Exact algebraic image of a Bezier derivative vector.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAlgebraicTangentImage2 {
    status: BezierAlgebraicImageStatus,
    parameter: AlgebraicRootRepresentation,
    dx: Option<BezierAlgebraicCoordinateImage>,
    dy: Option<BezierAlgebraicCoordinateImage>,
    message: Option<String>,
}

/// Exact algebraic image of a rational quadratic Bezier affine point.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBezierAlgebraicPointImage2 {
    status: BezierAlgebraicImageStatus,
    parameter: AlgebraicRootRepresentation,
    x: Option<BezierAlgebraicRationalCoordinateImage>,
    y: Option<BezierAlgebraicRationalCoordinateImage>,
    message: Option<String>,
}

impl RationalBezierAlgebraicPointImage2 {
    /// Returns the final construction status.
    pub const fn status(&self) -> BezierAlgebraicImageStatus {
        self.status
    }

    /// Returns the represented Bezier parameter used as the source root.
    pub const fn parameter(&self) -> &AlgebraicRootRepresentation {
        &self.parameter
    }

    /// Returns the x coordinate rational image when construction reached it.
    pub const fn x(&self) -> Option<&BezierAlgebraicRationalCoordinateImage> {
        self.x.as_ref()
    }

    /// Returns the y coordinate rational image when construction reached it.
    pub const fn y(&self) -> Option<&BezierAlgebraicRationalCoordinateImage> {
        self.y.as_ref()
    }

    /// Returns a compact diagnostic message for failed construction.
    pub fn message(&self) -> Option<&str> {
        self.message.as_deref()
    }
}

/// Exact algebraic image of a rational quadratic Bezier derivative vector.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBezierAlgebraicTangentImage2 {
    status: BezierAlgebraicImageStatus,
    parameter: AlgebraicRootRepresentation,
    dx: Option<BezierAlgebraicRationalCoordinateImage>,
    dy: Option<BezierAlgebraicRationalCoordinateImage>,
    message: Option<String>,
}

impl RationalBezierAlgebraicTangentImage2 {
    /// Returns the final construction status.
    pub const fn status(&self) -> BezierAlgebraicImageStatus {
        self.status
    }

    /// Returns the represented Bezier parameter used as the source root.
    pub const fn parameter(&self) -> &AlgebraicRootRepresentation {
        &self.parameter
    }

    /// Returns the derivative x rational image when construction reached it.
    pub const fn dx(&self) -> Option<&BezierAlgebraicRationalCoordinateImage> {
        self.dx.as_ref()
    }

    /// Returns the derivative y rational image when construction reached it.
    pub const fn dy(&self) -> Option<&BezierAlgebraicRationalCoordinateImage> {
        self.dy.as_ref()
    }

    /// Returns a compact diagnostic message for failed construction.
    pub fn message(&self) -> Option<&str> {
        self.message.as_deref()
    }
}

impl BezierAlgebraicTangentImage2 {
    /// Returns the final construction status.
    pub const fn status(&self) -> BezierAlgebraicImageStatus {
        self.status
    }

    /// Returns the represented Bezier parameter used as the source root.
    pub const fn parameter(&self) -> &AlgebraicRootRepresentation {
        &self.parameter
    }

    /// Returns the derivative x component image when construction reached it.
    pub const fn dx(&self) -> Option<&BezierAlgebraicCoordinateImage> {
        self.dx.as_ref()
    }

    /// Returns the derivative y component image when construction reached it.
    pub const fn dy(&self) -> Option<&BezierAlgebraicCoordinateImage> {
        self.dy.as_ref()
    }

    /// Returns a compact diagnostic message for failed construction.
    pub fn message(&self) -> Option<&str> {
        self.message.as_deref()
    }
}

impl QuadraticBezier2 {
    /// Evaluates this quadratic at an isolated algebraic parameter.
    ///
    /// The returned x/y coordinates are `hypersolve` represented roots for the
    /// exact coordinate polynomials
    /// `P0 + 2(P1-P0)t + (P0-2P1+P2)t^2`.  This is intentionally report
    /// bearing: unsupported polynomial-image evidence remains visible instead
    /// of becoming a rounded point.
    pub fn point_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<BezierAlgebraicPointImage2> {
        point_image(parameter, quadratic_point_coefficients(self), policy)
    }

    /// Evaluates this quadratic's first derivative at an isolated algebraic
    /// parameter.
    ///
    /// The derivative coordinate polynomial is
    /// `2(P1-P0) + 2(P0-2P1+P2)t`, again retained as represented-root evidence.
    pub fn tangent_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<BezierAlgebraicTangentImage2> {
        tangent_image(parameter, quadratic_tangent_coefficients(self), policy)
    }

    /// Evaluates this quadratic Bezier's second derivative at an isolated
    /// algebraic parameter.
    ///
    /// The second derivative of a polynomial quadratic Bezier is constant, but
    /// it is still returned as a represented coordinate image so arrangement
    /// predicates can combine it with represented endpoint tangents without
    /// crossing Yap's construction/decision boundary.
    pub fn second_derivative_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<BezierAlgebraicTangentImage2> {
        tangent_image(
            parameter,
            second_derivative_polynomials(quadratic_tangent_coefficients(self)),
            policy,
        )
    }
}

impl CubicBezier2 {
    /// Evaluates this cubic at an isolated algebraic parameter.
    ///
    /// Coordinates use the exact power-basis form
    /// `P0 + 3(P1-P0)t + 3(P0-2P1+P2)t^2`
    /// `+ (-P0+3P1-3P2+P3)t^3`, represented through `hypersolve` polynomial
    /// images rather than sampled into finite coordinates.
    pub fn point_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<BezierAlgebraicPointImage2> {
        point_image(parameter, cubic_point_coefficients(self), policy)
    }

    /// Evaluates this cubic's first derivative at an isolated algebraic
    /// parameter as exact represented coordinate images.
    pub fn tangent_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<BezierAlgebraicTangentImage2> {
        tangent_image(parameter, cubic_tangent_coefficients(self), policy)
    }

    /// Evaluates this cubic Bezier's second derivative at an isolated
    /// algebraic parameter.
    ///
    /// The coordinate polynomials are derived by differentiating the cubic
    /// tangent polynomial. Keeping the image represented lets local branch
    /// order compare signed curvature exactly instead of sampling the
    /// isolating interval; see Yap (1997) and Farin (2002).
    pub fn second_derivative_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<BezierAlgebraicTangentImage2> {
        tangent_image(
            parameter,
            second_derivative_polynomials(cubic_tangent_coefficients(self)),
            policy,
        )
    }

    /// Evaluates this cubic Bezier's third derivative at an isolated algebraic
    /// parameter.
    ///
    /// Cubic third derivatives are constant. The represented image is retained
    /// for the same reason as the second derivative: arrangement code can
    /// consume exact evidence and explicitly defer unresolved signs.
    pub fn third_derivative_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<BezierAlgebraicTangentImage2> {
        tangent_image(
            parameter,
            second_derivative_polynomials(second_derivative_polynomials(
                cubic_tangent_coefficients(self),
            )),
            policy,
        )
    }
}

impl RationalQuadraticBezier2 {
    /// Evaluates this rational quadratic's affine point at an isolated
    /// algebraic parameter.
    ///
    /// Each coordinate is represented as `N(t)/D(t)` using the homogeneous
    /// Bernstein numerator and weight denominator.  Denominator-domain
    /// certification is delegated to `hypersolve`'s rational-image package, so
    /// projective boundary uncertainty stays report-bearing instead of being
    /// sampled into affine space.  This is the rational Bezier analogue of the
    /// polynomial image construction above; see Yap (1997) for the exact-object
    /// boundary and Farin (2002) for the homogeneous conic equations.
    pub fn point_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<RationalBezierAlgebraicPointImage2> {
        rational_point_image(parameter, rational_point_coefficients(self), policy)
    }

    /// Evaluates this rational quadratic's affine derivative vector at an
    /// isolated algebraic parameter.
    ///
    /// The derivative coordinate is `(N'D - ND') / D^2`.  The squared
    /// denominator preserves tangent direction while giving the exact rational
    /// image package a domain predicate that rejects denominator-zero
    /// projective boundaries explicitly.
    pub fn tangent_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<RationalBezierAlgebraicTangentImage2> {
        rational_tangent_image(parameter, rational_tangent_coefficients(self), policy)
    }

    /// Evaluates this rational quadratic's affine second derivative vector.
    ///
    /// For one coordinate `R(t) = N(t)/D(t)`, the retained numerator is
    /// `(A'(t)D(t) - 2A(t)D'(t))` over `D(t)^3`, where
    /// `A(t) = N'(t)D(t) - N(t)D'(t)`.  This is the differentiated quotient
    /// identity for homogeneous rational Beziers described by Farin, *Curves
    /// and Surfaces for CAGD* (5th ed., 2002).  The result remains a
    /// represented rational image of the algebraic parameter, preserving
    /// Yap's construction/decision boundary instead of sampling the conic.
    pub fn second_derivative_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<RationalBezierAlgebraicTangentImage2> {
        rational_tangent_image(
            parameter,
            rational_second_derivative_coefficients(self),
            policy,
        )
    }
}

fn point_image(
    parameter: &BezierAlgebraicParameter2,
    coefficients: CoordinatePolynomials,
    policy: &CurvePolicy,
) -> CurveResult<BezierAlgebraicPointImage2> {
    let parameter_root = parameter_representation(parameter, policy);
    if !parameter_root.is_valid() {
        return Ok(BezierAlgebraicPointImage2 {
            status: BezierAlgebraicImageStatus::InvalidParameterEvidence,
            parameter: parameter_root,
            x: None,
            y: None,
            message: Some("Bezier algebraic parameter evidence did not validate".to_owned()),
        });
    }
    let Some(x) = coordinate_image(&parameter_root, coefficients.x, policy) else {
        return Ok(BezierAlgebraicPointImage2 {
            status: BezierAlgebraicImageStatus::XImageFailed,
            parameter: parameter_root,
            x: None,
            y: None,
            message: Some("x coordinate polynomial image failed".to_owned()),
        });
    };
    let Some(y) = coordinate_image(&parameter_root, coefficients.y, policy) else {
        return Ok(BezierAlgebraicPointImage2 {
            status: BezierAlgebraicImageStatus::YImageFailed,
            parameter: parameter_root,
            x: Some(x),
            y: None,
            message: Some("y coordinate polynomial image failed".to_owned()),
        });
    };
    Ok(BezierAlgebraicPointImage2 {
        status: BezierAlgebraicImageStatus::Transformed,
        parameter: parameter_root,
        x: Some(x),
        y: Some(y),
        message: None,
    })
}

fn rational_point_image(
    parameter: &BezierAlgebraicParameter2,
    coefficients: RationalCoordinatePolynomials,
    policy: &CurvePolicy,
) -> CurveResult<RationalBezierAlgebraicPointImage2> {
    let parameter_root = parameter_representation(parameter, policy);
    if !parameter_root.is_valid() {
        return Ok(RationalBezierAlgebraicPointImage2 {
            status: BezierAlgebraicImageStatus::InvalidParameterEvidence,
            parameter: parameter_root,
            x: None,
            y: None,
            message: Some("Bezier algebraic parameter evidence did not validate".to_owned()),
        });
    }
    let Some(x) = rational_coordinate_image(
        &parameter_root,
        coefficients.x_numerator,
        coefficients.denominator.clone(),
        policy,
    ) else {
        return Ok(RationalBezierAlgebraicPointImage2 {
            status: BezierAlgebraicImageStatus::XImageFailed,
            parameter: parameter_root,
            x: None,
            y: None,
            message: Some("x rational coordinate image failed".to_owned()),
        });
    };
    let Some(y) = rational_coordinate_image(
        &parameter_root,
        coefficients.y_numerator,
        coefficients.denominator,
        policy,
    ) else {
        return Ok(RationalBezierAlgebraicPointImage2 {
            status: BezierAlgebraicImageStatus::YImageFailed,
            parameter: parameter_root,
            x: Some(x),
            y: None,
            message: Some("y rational coordinate image failed".to_owned()),
        });
    };
    Ok(RationalBezierAlgebraicPointImage2 {
        status: BezierAlgebraicImageStatus::Transformed,
        parameter: parameter_root,
        x: Some(x),
        y: Some(y),
        message: None,
    })
}

fn tangent_image(
    parameter: &BezierAlgebraicParameter2,
    coefficients: CoordinatePolynomials,
    policy: &CurvePolicy,
) -> CurveResult<BezierAlgebraicTangentImage2> {
    let parameter_root = parameter_representation(parameter, policy);
    if !parameter_root.is_valid() {
        return Ok(BezierAlgebraicTangentImage2 {
            status: BezierAlgebraicImageStatus::InvalidParameterEvidence,
            parameter: parameter_root,
            dx: None,
            dy: None,
            message: Some("Bezier algebraic parameter evidence did not validate".to_owned()),
        });
    }
    let Some(dx) = coordinate_image(&parameter_root, coefficients.x, policy) else {
        return Ok(BezierAlgebraicTangentImage2 {
            status: BezierAlgebraicImageStatus::XImageFailed,
            parameter: parameter_root,
            dx: None,
            dy: None,
            message: Some("dx coordinate polynomial image failed".to_owned()),
        });
    };
    let Some(dy) = coordinate_image(&parameter_root, coefficients.y, policy) else {
        return Ok(BezierAlgebraicTangentImage2 {
            status: BezierAlgebraicImageStatus::YImageFailed,
            parameter: parameter_root,
            dx: Some(dx),
            dy: None,
            message: Some("dy coordinate polynomial image failed".to_owned()),
        });
    };
    Ok(BezierAlgebraicTangentImage2 {
        status: BezierAlgebraicImageStatus::Transformed,
        parameter: parameter_root,
        dx: Some(dx),
        dy: Some(dy),
        message: None,
    })
}

fn rational_tangent_image(
    parameter: &BezierAlgebraicParameter2,
    coefficients: RationalTangentPolynomials,
    policy: &CurvePolicy,
) -> CurveResult<RationalBezierAlgebraicTangentImage2> {
    let parameter_root = parameter_representation(parameter, policy);
    if !parameter_root.is_valid() {
        return Ok(RationalBezierAlgebraicTangentImage2 {
            status: BezierAlgebraicImageStatus::InvalidParameterEvidence,
            parameter: parameter_root,
            dx: None,
            dy: None,
            message: Some("Bezier algebraic parameter evidence did not validate".to_owned()),
        });
    }
    let Some(dx) = rational_coordinate_image(
        &parameter_root,
        coefficients.dx_numerator,
        coefficients.denominator.clone(),
        policy,
    ) else {
        return Ok(RationalBezierAlgebraicTangentImage2 {
            status: BezierAlgebraicImageStatus::XImageFailed,
            parameter: parameter_root,
            dx: None,
            dy: None,
            message: Some("dx rational coordinate image failed".to_owned()),
        });
    };
    let Some(dy) = rational_coordinate_image(
        &parameter_root,
        coefficients.dy_numerator,
        coefficients.denominator,
        policy,
    ) else {
        return Ok(RationalBezierAlgebraicTangentImage2 {
            status: BezierAlgebraicImageStatus::YImageFailed,
            parameter: parameter_root,
            dx: Some(dx),
            dy: None,
            message: Some("dy rational coordinate image failed".to_owned()),
        });
    };
    Ok(RationalBezierAlgebraicTangentImage2 {
        status: BezierAlgebraicImageStatus::Transformed,
        parameter: parameter_root,
        dx: Some(dx),
        dy: Some(dy),
        message: None,
    })
}

fn coordinate_image(
    parameter: &AlgebraicRootRepresentation,
    coefficients: Vec<Real>,
    policy: &CurvePolicy,
) -> Option<BezierAlgebraicCoordinateImage> {
    if let Some(parameter_value) = parameter.exact_rational_witness() {
        let value = evaluate_power_polynomial(&coefficients, parameter_value);
        let representation = exact_real_representation(&value);
        return Some(BezierAlgebraicCoordinateImage {
            report: AlgebraicRootPolynomialImageReport {
                status: AlgebraicRootPolynomialImageStatus::Transformed,
                image_coefficients: coefficients.clone(),
                representation: Some(representation),
                message: None,
            },
            coefficients,
        });
    }
    if coefficients.len() == 1 {
        let representation = exact_real_representation(&coefficients[0]);
        return Some(BezierAlgebraicCoordinateImage {
            report: AlgebraicRootPolynomialImageReport {
                status: AlgebraicRootPolynomialImageStatus::Transformed,
                image_coefficients: coefficients.clone(),
                representation: Some(representation),
                message: None,
            },
            coefficients,
        });
    }
    let report = transform_algebraic_root_polynomial_image(
        parameter,
        &coefficients,
        policy.predicate_policy,
    );
    (report.status == AlgebraicRootPolynomialImageStatus::Transformed).then_some(
        BezierAlgebraicCoordinateImage {
            coefficients,
            report,
        },
    )
}

fn rational_coordinate_image(
    parameter: &AlgebraicRootRepresentation,
    numerator_coefficients: Vec<Real>,
    denominator_coefficients: Vec<Real>,
    policy: &CurvePolicy,
) -> Option<BezierAlgebraicRationalCoordinateImage> {
    let report = transform_algebraic_root_rational_image(
        parameter,
        &numerator_coefficients,
        &denominator_coefficients,
        policy.predicate_policy,
    );
    (report.status == AlgebraicRootRationalImageStatus::Transformed).then_some(
        BezierAlgebraicRationalCoordinateImage {
            numerator_coefficients,
            denominator_coefficients,
            report,
        },
    )
}

fn exact_real_representation(value: &Real) -> AlgebraicRootRepresentation {
    AlgebraicRootRepresentation {
        constraint_index: 0,
        symbol: SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: vec![Real::zero() - value, Real::one()],
        interval: IsolatedRootInterval {
            lower: value.clone(),
            upper: value.clone(),
            exact_root: Some(value.clone()),
            distinct_root_count: 1,
        },
        kind: AlgebraicRootKind::ExactRationalWitness,
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    }
}

fn evaluate_power_polynomial(coefficients: &[Real], parameter: &Real) -> Real {
    coefficients
        .iter()
        .rev()
        .fold(Real::zero(), |accumulator, coefficient| {
            (accumulator * parameter) + coefficient
        })
}

fn parameter_representation(
    parameter: &BezierAlgebraicParameter2,
    policy: &CurvePolicy,
) -> AlgebraicRootRepresentation {
    let interval = parameter.interval();
    let exact_root = linear_parameter_witness(parameter, policy);
    let mut representation = AlgebraicRootRepresentation {
        constraint_index: 0,
        symbol: SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: parameter.polynomial().coefficients().to_vec(),
        interval: IsolatedRootInterval {
            lower: interval.start().clone(),
            upper: interval.end().clone(),
            exact_root: exact_root.clone(),
            distinct_root_count: parameter.root_count(),
        },
        kind: if exact_root.is_some() {
            AlgebraicRootKind::ExactRationalWitness
        } else {
            AlgebraicRootKind::IsolatingInterval
        },
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    representation.validation =
        validate_algebraic_root_representation(&representation, policy.predicate_policy);
    representation
}

fn linear_parameter_witness(
    parameter: &BezierAlgebraicParameter2,
    policy: &CurvePolicy,
) -> Option<Real> {
    let coefficients = parameter.polynomial().coefficients();
    if coefficients.len() != 2 {
        return None;
    }
    let root = (Real::zero() - coefficients[0].clone()) / coefficients[1].clone();
    let root = root.ok()?;
    let interval = parameter.interval();
    let starts_after_root = compare_reals(interval.start(), &root, policy)? != Ordering::Greater;
    let ends_before_root = compare_reals(&root, interval.end(), policy)? != Ordering::Greater;
    (starts_after_root && ends_before_root).then_some(root)
}

fn quadratic_point_coefficients(curve: &QuadraticBezier2) -> CoordinatePolynomials {
    let two = Real::from(2_i8);
    CoordinatePolynomials {
        x: quadratic_power_coefficients(
            curve.start().x(),
            curve.control().x(),
            curve.end().x(),
            &two,
        ),
        y: quadratic_power_coefficients(
            curve.start().y(),
            curve.control().y(),
            curve.end().y(),
            &two,
        ),
    }
}

fn quadratic_tangent_coefficients(curve: &QuadraticBezier2) -> CoordinatePolynomials {
    let two = Real::from(2_i8);
    CoordinatePolynomials {
        x: quadratic_derivative_coefficients(
            curve.start().x(),
            curve.control().x(),
            curve.end().x(),
            &two,
        ),
        y: quadratic_derivative_coefficients(
            curve.start().y(),
            curve.control().y(),
            curve.end().y(),
            &two,
        ),
    }
}

fn cubic_point_coefficients(curve: &CubicBezier2) -> CoordinatePolynomials {
    let three = Real::from(3_i8);
    CoordinatePolynomials {
        x: cubic_power_coefficients(
            curve.start().x(),
            curve.control1().x(),
            curve.control2().x(),
            curve.end().x(),
            &three,
        ),
        y: cubic_power_coefficients(
            curve.start().y(),
            curve.control1().y(),
            curve.control2().y(),
            curve.end().y(),
            &three,
        ),
    }
}

fn cubic_tangent_coefficients(curve: &CubicBezier2) -> CoordinatePolynomials {
    derivative_polynomials(cubic_point_coefficients(curve))
}

fn rational_point_coefficients(curve: &RationalQuadraticBezier2) -> RationalCoordinatePolynomials {
    let weighted_x = [
        curve.start().x() * curve.start_weight(),
        curve.control().x() * curve.control_weight(),
        curve.end().x() * curve.end_weight(),
    ];
    let weighted_y = [
        curve.start().y() * curve.start_weight(),
        curve.control().y() * curve.control_weight(),
        curve.end().y() * curve.end_weight(),
    ];
    let weights = [
        curve.start_weight().clone(),
        curve.control_weight().clone(),
        curve.end_weight().clone(),
    ];
    RationalCoordinatePolynomials {
        x_numerator: rational_quadratic_power_coefficients(&weighted_x),
        y_numerator: rational_quadratic_power_coefficients(&weighted_y),
        denominator: rational_quadratic_power_coefficients(&weights),
    }
}

fn rational_tangent_coefficients(curve: &RationalQuadraticBezier2) -> RationalTangentPolynomials {
    let point = rational_point_coefficients(curve);
    let denominator_derivative = derivative_coefficients(&point.denominator);
    let denominator_squared = multiply_polynomials(&point.denominator, &point.denominator);
    let dx_numerator = rational_derivative_numerator(
        &point.x_numerator,
        &point.denominator,
        &denominator_derivative,
    );
    let dy_numerator = rational_derivative_numerator(
        &point.y_numerator,
        &point.denominator,
        &denominator_derivative,
    );
    RationalTangentPolynomials {
        dx_numerator,
        dy_numerator,
        denominator: denominator_squared,
    }
}

fn rational_second_derivative_coefficients(
    curve: &RationalQuadraticBezier2,
) -> RationalTangentPolynomials {
    let point = rational_point_coefficients(curve);
    let denominator_derivative = derivative_coefficients(&point.denominator);
    let denominator_squared = multiply_polynomials(&point.denominator, &point.denominator);
    let denominator_cubed = multiply_polynomials(&denominator_squared, &point.denominator);
    let dx_first_numerator = rational_derivative_numerator(
        &point.x_numerator,
        &point.denominator,
        &denominator_derivative,
    );
    let dy_first_numerator = rational_derivative_numerator(
        &point.y_numerator,
        &point.denominator,
        &denominator_derivative,
    );
    let dx_numerator = rational_second_derivative_numerator(
        &dx_first_numerator,
        &point.denominator,
        &denominator_derivative,
    );
    let dy_numerator = rational_second_derivative_numerator(
        &dy_first_numerator,
        &point.denominator,
        &denominator_derivative,
    );
    RationalTangentPolynomials {
        dx_numerator,
        dy_numerator,
        denominator: denominator_cubed,
    }
}

fn quadratic_power_coefficients(p0: &Real, p1: &Real, p2: &Real, two: &Real) -> Vec<Real> {
    vec![p0.clone(), two * &(p1 - p0), p0 - &(two * p1) + p2]
}

fn quadratic_derivative_coefficients(p0: &Real, p1: &Real, p2: &Real, two: &Real) -> Vec<Real> {
    vec![two * &(p1 - p0), two * &(p0 - &(two * p1) + p2)]
}

fn rational_quadratic_power_coefficients(bernstein: &[Real; 3]) -> Vec<Real> {
    let two = Real::from(2_i8);
    quadratic_power_coefficients(&bernstein[0], &bernstein[1], &bernstein[2], &two)
}

fn cubic_power_coefficients(p0: &Real, p1: &Real, p2: &Real, p3: &Real, three: &Real) -> Vec<Real> {
    vec![
        p0.clone(),
        three * &(p1 - p0),
        three * &(p0 - &(Real::from(2_i8) * p1) + p2),
        Real::zero() - p0 + &(three * p1) - &(three * p2) + p3,
    ]
}

fn derivative_polynomials(polynomials: CoordinatePolynomials) -> CoordinatePolynomials {
    CoordinatePolynomials {
        x: derivative_coefficients(&polynomials.x),
        y: derivative_coefficients(&polynomials.y),
    }
}

fn second_derivative_polynomials(polynomials: CoordinatePolynomials) -> CoordinatePolynomials {
    derivative_polynomials(polynomials)
}

fn derivative_coefficients(coefficients: &[Real]) -> Vec<Real> {
    coefficients
        .iter()
        .enumerate()
        .skip(1)
        .map(|(degree, coefficient)| coefficient * &Real::from(degree as i64))
        .collect()
}

fn rational_derivative_numerator(
    numerator: &[Real],
    denominator: &[Real],
    denominator_derivative: &[Real],
) -> Vec<Real> {
    subtract_polynomials(
        &multiply_polynomials(&derivative_coefficients(numerator), denominator),
        &multiply_polynomials(numerator, denominator_derivative),
    )
}

fn rational_second_derivative_numerator(
    first_derivative_numerator: &[Real],
    denominator: &[Real],
    denominator_derivative: &[Real],
) -> Vec<Real> {
    subtract_polynomials(
        &multiply_polynomials(
            &derivative_coefficients(first_derivative_numerator),
            denominator,
        ),
        &scale_polynomial(
            &multiply_polynomials(first_derivative_numerator, denominator_derivative),
            Real::from(2_i8),
        ),
    )
}

fn multiply_polynomials(left: &[Real], right: &[Real]) -> Vec<Real> {
    let mut result = vec![Real::zero(); left.len() + right.len() - 1];
    for (left_degree, left_coefficient) in left.iter().enumerate() {
        for (right_degree, right_coefficient) in right.iter().enumerate() {
            result[left_degree + right_degree] =
                result[left_degree + right_degree].clone() + left_coefficient * right_coefficient;
        }
    }
    result
}

fn subtract_polynomials(left: &[Real], right: &[Real]) -> Vec<Real> {
    let mut result = vec![Real::zero(); left.len().max(right.len())];
    for (index, coefficient) in left.iter().enumerate() {
        result[index] = result[index].clone() + coefficient;
    }
    for (index, coefficient) in right.iter().enumerate() {
        result[index] = result[index].clone() - coefficient;
    }
    result
}

fn scale_polynomial(coefficients: &[Real], scale: Real) -> Vec<Real> {
    coefficients
        .iter()
        .map(|coefficient| coefficient * &scale)
        .collect()
}

#[derive(Clone, Debug)]
struct CoordinatePolynomials {
    x: Vec<Real>,
    y: Vec<Real>,
}

#[derive(Clone, Debug)]
struct RationalCoordinatePolynomials {
    x_numerator: Vec<Real>,
    y_numerator: Vec<Real>,
    denominator: Vec<Real>,
}

#[derive(Clone, Debug)]
struct RationalTangentPolynomials {
    dx_numerator: Vec<Real>,
    dy_numerator: Vec<Real>,
    denominator: Vec<Real>,
}
