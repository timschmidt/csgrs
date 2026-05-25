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
    AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    IsolatedRootInterval, SymbolId, transform_algebraic_root_polynomial_image,
    validate_algebraic_root_representation,
};

use crate::classify::compare_reals;
use crate::{BezierAlgebraicParameter2, CubicBezier2, CurvePolicy, CurveResult, QuadraticBezier2};
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

fn coordinate_image(
    parameter: &AlgebraicRootRepresentation,
    coefficients: Vec<Real>,
    policy: &CurvePolicy,
) -> Option<BezierAlgebraicCoordinateImage> {
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

fn quadratic_power_coefficients(p0: &Real, p1: &Real, p2: &Real, two: &Real) -> Vec<Real> {
    vec![p0.clone(), two * &(p1 - p0), p0 - &(two * p1) + p2]
}

fn quadratic_derivative_coefficients(p0: &Real, p1: &Real, p2: &Real, two: &Real) -> Vec<Real> {
    vec![two * &(p1 - p0), two * &(p0 - &(two * p1) + p2)]
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

fn derivative_coefficients(coefficients: &[Real]) -> Vec<Real> {
    coefficients
        .iter()
        .enumerate()
        .skip(1)
        .map(|(degree, coefficient)| coefficient * &Real::from(degree as i64))
        .collect()
}

#[derive(Clone, Debug)]
struct CoordinatePolynomials {
    x: Vec<Real>,
    y: Vec<Real>,
}
