//! Algebraic Bezier point and tangent images.
//!
//! This module is the first materialization bridge between
//! [`BezierAlgebraicParameter2`](crate::BezierAlgebraicParameter2) and concrete
//! curve geometry.  It does not approximate an isolated split parameter.
//! Instead it converts the parameter into a
//! [`hypersolve::AlgebraicRootRepresentation`] and evaluates Bezier coordinate
//! polynomials with `hypersolve`'s resultant-backed polynomial-image package.
//! That follows exact-computation discipline: constructed
//! coordinates remain exact objects with replayable evidence, while callers
//! branch only on certified predicates.  The coordinate polynomials are the
//! standard Bernstein-to-power identities for Bezier curves; see the Bernstein and de Casteljau curve model.

use hyperreal::Real;
use hypersolve::{
    AlgebraicRootKind, AlgebraicRootPolynomialImageReport, AlgebraicRootPolynomialImageStatus,
    AlgebraicRootRationalImageReport, AlgebraicRootRepresentation, AlgebraicRootValidationReport,
    AlgebraicRootValidationStatus, IsolatedRootInterval, SymbolId,
};
use hypersolve::{
    AlgebraicRootRationalImageStatus, AlgebraicRootRefinementComparisonConfig,
    compare_algebraic_root_representations_by_difference,
    transform_algebraic_root_polynomial_image, transform_algebraic_root_rational_images,
    validate_algebraic_root_representation,
};

use crate::classify::compare_reals;
use crate::{
    Aabb2, BezierAlgebraicParameter2, Classification, CubicBezier2, CurveContext, CurveResult,
    QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2,
};
use std::cmp::Ordering;
use std::sync::Arc;
use std::sync::OnceLock;

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
    /// The exact rational-coordinate expressions and their certified
    /// Real-coefficient source root, or an equivalent exact curve/parameter
    /// source, were retained without forcing coordinate representations into
    /// the rational-coefficient algebraic-number package.
    RetainedRationalExpression,
}

/// One exact coordinate image of a Bezier expression at an algebraic parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAlgebraicCoordinateImage {
    coefficients: Vec<Real>,
    evidence: AlgebraicRootPolynomialImageReport,
}

/// One exact rational-function coordinate image at an algebraic parameter.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAlgebraicRationalCoordinateImage {
    numerator_coefficients: Vec<Real>,
    denominator_coefficients: Vec<Real>,
    evidence: AlgebraicRootRationalImageReport,
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

    /// Returns the represented coordinate when the image was constructed.
    pub fn representation(&self) -> Option<&AlgebraicRootRepresentation> {
        self.evidence.representation.as_ref()
    }

    /// Compares this exact algebraic coordinate with a represented real value.
    ///
    /// The comparison reuses the retained rational-image representation and
    /// performs certified root refinement; it never converts either operand to
    /// a primitive floating-point value.
    pub fn compare_to_real(
        &self,
        value: &Real,
        policy: &CurveContext,
    ) -> crate::Classification<Ordering> {
        let Some(representation) = self.representation() else {
            return crate::Classification::Uncertain(crate::UncertaintyReason::Unsupported);
        };
        compare_root_representation_to_real(representation, value, policy)
    }
}

impl BezierAlgebraicCoordinateImage {
    /// Compares this exact algebraic coordinate with a represented real value.
    ///
    /// The retained polynomial-image representation is refined only as far as
    /// needed to certify order, without materializing a primitive float.
    pub fn compare_to_real(
        &self,
        value: &Real,
        policy: &CurveContext,
    ) -> crate::Classification<Ordering> {
        let Some(representation) = self.representation() else {
            return crate::Classification::Uncertain(crate::UncertaintyReason::Unsupported);
        };
        compare_root_representation_to_real(representation, value, policy)
    }
}

fn compare_root_representation_to_real(
    representation: &AlgebraicRootRepresentation,
    value: &Real,
    policy: &CurveContext,
) -> crate::Classification<Ordering> {
    if let Some(exact) = representation.exact_rational_witness() {
        return compare_reals(exact, value, policy)
            .map(crate::Classification::Decided)
            .unwrap_or(crate::Classification::Uncertain(
                crate::UncertaintyReason::Ordering,
            ));
    }
    compare_algebraic_representation_to_real(representation, value, policy)
}

fn compare_algebraic_representation_to_real(
    representation: &AlgebraicRootRepresentation,
    value: &Real,
    policy: &CurveContext,
) -> crate::Classification<Ordering> {
    let exact = exact_real_algebraic_representation(value);
    let evidence = compare_algebraic_root_representations_by_difference(
        representation,
        &exact,
        AlgebraicRootRefinementComparisonConfig {
            policy: policy.predicate_policy(),
            ..AlgebraicRootRefinementComparisonConfig::default()
        },
    );
    evidence
        .comparison
        .ordering
        .map(crate::Classification::Decided)
        .unwrap_or(crate::Classification::Uncertain(
            crate::UncertaintyReason::Ordering,
        ))
}

impl BezierAlgebraicCoordinateImage {
    /// Returns the coordinate polynomial in ascending powers of the source
    /// Bezier parameter.
    pub fn coefficients(&self) -> &[Real] {
        &self.coefficients
    }

    /// Returns the represented coordinate when the image was constructed.
    pub fn representation(&self) -> Option<&AlgebraicRootRepresentation> {
        self.evidence.representation.as_ref()
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
#[derive(Clone, Debug)]
pub struct RationalBezierAlgebraicPointImage2 {
    data: Arc<RationalBezierAlgebraicPointImageData>,
}

#[derive(Debug, PartialEq)]
struct RationalBezierAlgebraicPointImageData {
    status: BezierAlgebraicImageStatus,
    parameter: AlgebraicRootRepresentation,
    x: Option<BezierAlgebraicRationalCoordinateImage>,
    y: Option<BezierAlgebraicRationalCoordinateImage>,
    retained_expression: Option<RetainedRationalPointExpression>,
    parametric_source: Option<RetainedRationalPointParametricSource>,
    message: Option<String>,
}

#[derive(Debug, PartialEq)]
struct RetainedRationalPointExpression {
    parameter: BezierAlgebraicParameter2,
    x_numerator: Vec<Real>,
    y_numerator: Vec<Real>,
    denominator: Vec<Real>,
}

#[derive(Debug)]
struct RetainedRationalPointParametricSource {
    curve: RationalBezier2,
    parameter: BezierAlgebraicParameter2,
    resolved: OnceLock<Option<RationalBezierAlgebraicPointImage2>>,
}

impl PartialEq for RetainedRationalPointParametricSource {
    fn eq(&self, other: &Self) -> bool {
        self.curve == other.curve && self.parameter == other.parameter
    }
}

impl PartialEq for RationalBezierAlgebraicPointImage2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data) || self.data == other.data
    }
}

impl RationalBezierAlgebraicPointImage2 {
    fn new(
        status: BezierAlgebraicImageStatus,
        parameter: AlgebraicRootRepresentation,
        x: Option<BezierAlgebraicRationalCoordinateImage>,
        y: Option<BezierAlgebraicRationalCoordinateImage>,
        retained_expression: Option<RetainedRationalPointExpression>,
        message: Option<String>,
    ) -> Self {
        Self {
            data: Arc::new(RationalBezierAlgebraicPointImageData {
                status,
                parameter,
                x,
                y,
                retained_expression,
                parametric_source: None,
                message,
            }),
        }
    }

    pub(crate) fn from_parametric_source(
        curve: RationalBezier2,
        parameter: BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> Self {
        Self {
            data: Arc::new(RationalBezierAlgebraicPointImageData {
                status: BezierAlgebraicImageStatus::RetainedRationalExpression,
                parameter: parameter_representation(&parameter, policy),
                x: None,
                y: None,
                retained_expression: None,
                parametric_source: Some(RetainedRationalPointParametricSource {
                    curve,
                    parameter,
                    resolved: OnceLock::new(),
                }),
                message: None,
            }),
        }
    }

    #[inline(never)]
    pub(crate) fn resolved(&self, policy: &CurveContext) -> Option<&Self> {
        let Some(source) = &self.data.parametric_source else {
            return Some(self);
        };
        source
            .resolved
            .get_or_init(|| {
                source
                    .curve
                    .point_at_algebraic_parameter(&source.parameter, policy)
                    .ok()
            })
            .as_ref()
    }

    pub(crate) fn parametric_source_bounds(
        &self,
        policy: &CurveContext,
    ) -> Option<Classification<Aabb2>> {
        self.parametric_source_bounds_refined(0, policy)
    }

    pub(crate) fn parametric_source_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Option<Classification<Aabb2>> {
        if let Some(source) = self.data.parametric_source.as_ref() {
            let parameter = crate::BezierParameter2::Algebraic(source.parameter.clone())
                .refined_isolating_interval(refinement_steps, policy);
            let (start, end) = match &parameter {
                crate::BezierParameter2::Exact(parameter) => (parameter, parameter),
                crate::BezierParameter2::Algebraic(parameter) => {
                    (parameter.interval().start(), parameter.interval().end())
                }
            };
            match source.curve.subcurve_between_exact(start, end, policy) {
                Ok(Classification::Decided(curve)) => curve.certified_bounds_classified(policy),
                Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
                Err(_) => Classification::Uncertain(crate::UncertaintyReason::Unsupported),
            }
            .into()
        } else {
            None
        }
    }

    pub(crate) fn same_injective_parametric_source_point(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Option<Classification<bool>> {
        let (Some(first), Some(second)) = (
            self.data.parametric_source.as_ref(),
            other.data.parametric_source.as_ref(),
        ) else {
            return None;
        };
        if first.curve != second.curve {
            return None;
        }
        if first.parameter == second.parameter {
            return Some(Classification::Decided(true));
        }
        let first_interval = first.parameter.interval();
        let second_interval = second.parameter.interval();
        // Validated algebraic isolators reject roots at either endpoint.
        // Therefore intervals that only touch at one endpoint still contain
        // distinct roots.
        let intervals_disjoint = matches!(
            compare_reals(first_interval.end(), second_interval.start(), policy),
            Some(Ordering::Less | Ordering::Equal)
        ) || matches!(
            compare_reals(second_interval.end(), first_interval.start(), policy),
            Some(Ordering::Less | Ordering::Equal)
        );
        if intervals_disjoint && first.curve.has_certified_injective_axis(policy) {
            Some(Classification::Decided(false))
        } else {
            None
        }
    }

    /// Returns the final construction status.
    pub fn status(&self) -> BezierAlgebraicImageStatus {
        self.data.status
    }

    /// Returns the represented Bezier parameter used as the source root.
    pub fn parameter(&self) -> &AlgebraicRootRepresentation {
        &self.data.parameter
    }

    /// Returns the x coordinate rational image when construction reached it.
    pub fn x(&self) -> Option<&BezierAlgebraicRationalCoordinateImage> {
        self.data.x.as_ref()
    }

    /// Returns the y coordinate rational image when construction reached it.
    pub fn y(&self) -> Option<&BezierAlgebraicRationalCoordinateImage> {
        self.data.y.as_ref()
    }

    /// Returns the exact isolated source parameter retained for a
    /// Real-coefficient rational expression.
    pub fn retained_parameter(&self) -> Option<&BezierAlgebraicParameter2> {
        self.data
            .retained_expression
            .as_ref()
            .map(|expression| &expression.parameter)
            .or_else(|| {
                self.data
                    .parametric_source
                    .as_ref()
                    .map(|source| &source.parameter)
            })
    }

    /// Returns the exact x numerator, y numerator, and shared denominator for
    /// a retained Real-coefficient rational expression.
    pub fn retained_coordinate_polynomials(&self) -> Option<(&[Real], &[Real], &[Real])> {
        self.data.retained_expression.as_ref().map(|expression| {
            (
                expression.x_numerator.as_slice(),
                expression.y_numerator.as_slice(),
                expression.denominator.as_slice(),
            )
        })
    }

    /// Returns a compact diagnostic message for failed construction.
    pub fn message(&self) -> Option<&str> {
        self.data.message.as_deref()
    }
}

/// Exact algebraic image of a rational Bezier derivative vector.
#[derive(Clone, Debug)]
pub struct RationalBezierAlgebraicTangentImage2 {
    data: Arc<RationalBezierAlgebraicTangentImageData>,
}

#[derive(Debug, PartialEq)]
struct RationalBezierAlgebraicTangentImageData {
    status: BezierAlgebraicImageStatus,
    parameter: AlgebraicRootRepresentation,
    dx: Option<BezierAlgebraicRationalCoordinateImage>,
    dy: Option<BezierAlgebraicRationalCoordinateImage>,
    message: Option<String>,
}

impl PartialEq for RationalBezierAlgebraicTangentImage2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data) || self.data == other.data
    }
}

impl RationalBezierAlgebraicTangentImage2 {
    fn new(
        status: BezierAlgebraicImageStatus,
        parameter: AlgebraicRootRepresentation,
        dx: Option<BezierAlgebraicRationalCoordinateImage>,
        dy: Option<BezierAlgebraicRationalCoordinateImage>,
        message: Option<String>,
    ) -> Self {
        Self {
            data: Arc::new(RationalBezierAlgebraicTangentImageData {
                status,
                parameter,
                dx,
                dy,
                message,
            }),
        }
    }

    /// Returns the final construction status.
    pub fn status(&self) -> BezierAlgebraicImageStatus {
        self.data.status
    }

    /// Returns the represented Bezier parameter used as the source root.
    pub fn parameter(&self) -> &AlgebraicRootRepresentation {
        &self.data.parameter
    }

    /// Returns the derivative x rational image when construction reached it.
    pub fn dx(&self) -> Option<&BezierAlgebraicRationalCoordinateImage> {
        self.data.dx.as_ref()
    }

    /// Returns the derivative y rational image when construction reached it.
    pub fn dy(&self) -> Option<&BezierAlgebraicRationalCoordinateImage> {
        self.data.dy.as_ref()
    }

    /// Returns a compact diagnostic message for failed construction.
    pub fn message(&self) -> Option<&str> {
        self.data.message.as_deref()
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
    /// `P0 + 2(P1-P0)t + (P0-2P1+P2)t^2`.  This is intentionally evidence
    /// bearing: unsupported polynomial-image evidence remains visible instead
    /// of becoming a rounded point.
    pub fn point_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> CurveResult<BezierAlgebraicTangentImage2> {
        tangent_image(parameter, quadratic_tangent_coefficients(self), policy)
    }

    /// Evaluates this quadratic Bezier's second derivative at an isolated
    /// algebraic parameter.
    ///
    /// The second derivative of a polynomial quadratic Bezier is constant, but
    /// it is still returned as a represented coordinate image so arrangement
    /// predicates can combine it with represented endpoint tangents without
    /// crossing the exactness model's construction/decision boundary.
    pub fn second_derivative_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> CurveResult<BezierAlgebraicPointImage2> {
        point_image(parameter, cubic_point_coefficients(self), policy)
    }

    /// Evaluates this cubic's first derivative at an isolated algebraic
    /// parameter as exact represented coordinate images.
    pub fn tangent_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<BezierAlgebraicTangentImage2> {
        tangent_image(parameter, cubic_tangent_coefficients(self), policy)
    }

    /// Evaluates this cubic Bezier's second derivative at an isolated
    /// algebraic parameter.
    ///
    /// The coordinate polynomials are derived by differentiating the cubic
    /// tangent polynomial. Keeping the image represented lets local branch
    /// order compare signed curvature exactly instead of sampling the
    /// isolating interval; see the exactness model and the Bernstein curve model.
    pub fn second_derivative_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
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
        policy: &CurveContext,
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
    /// projective boundary uncertainty stays evidence-bearing instead of being
    /// sampled into affine space.  This is the rational Bezier analogue of the
    /// polynomial image construction above; see the exactness model for the exact-object
    /// boundary and the Bernstein curve model for the homogeneous conic equations.
    pub fn point_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<RationalBezierAlgebraicPointImage2> {
        if let Some(image) = parameter.cached_rational_quadratic_point_image(self) {
            return Ok(image);
        }
        let image = rational_point_image(parameter, rational_point_coefficients(self), policy)?;
        if image.status() == BezierAlgebraicImageStatus::Transformed {
            parameter.retain_rational_quadratic_point_image(self, image.clone());
        }
        Ok(image)
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
        policy: &CurveContext,
    ) -> CurveResult<RationalBezierAlgebraicTangentImage2> {
        if let Some(images) = parameter.cached_rational_quadratic_derivative_images(self, 1) {
            return Ok(images
                .into_iter()
                .next()
                .expect("one retained derivative image was requested"));
        }
        let image = rational_tangent_image(parameter, rational_tangent_coefficients(self), policy)?;
        if image.status() == BezierAlgebraicImageStatus::Transformed {
            parameter.retain_rational_quadratic_derivative_images(self, vec![image.clone()]);
        }
        Ok(image)
    }

    /// Evaluates this rational quadratic's affine second derivative vector.
    ///
    /// For one coordinate `R(t) = N(t)/D(t)`, the retained numerator is
    /// `(A'(t)D(t) - 2A(t)D'(t))` over `D(t)^3`, where
    /// `A(t) = N'(t)D(t) - N(t)D'(t)`.  This is the differentiated quotient
    /// identity for homogeneous rational Beziers described by the Bernstein and de Casteljau curve model.  The result remains a
    /// represented rational image of the algebraic parameter, preserving
    /// the exactness model's construction/decision boundary instead of sampling the conic.
    pub fn second_derivative_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<RationalBezierAlgebraicTangentImage2> {
        rational_tangent_image(
            parameter,
            rational_second_derivative_coefficients(self),
            policy,
        )
    }

    /// Evaluates exact affine derivative images through `max_order` in one
    /// quotient-recurrence pass.
    ///
    /// The returned vector stores orders `1..=max_order`; order `k` is retained
    /// as a rational image with denominator `D^(k+1)`.
    pub fn derivatives_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        max_order: usize,
        policy: &CurveContext,
    ) -> CurveResult<Vec<RationalBezierAlgebraicTangentImage2>> {
        if let Some(images) = parameter.cached_rational_quadratic_derivative_images(self, max_order)
        {
            return Ok(images);
        }
        let point = rational_point_coefficients(self);
        let images = rational_derivative_images_from_power_basis(
            parameter,
            point.x_numerator,
            point.y_numerator,
            point.denominator,
            policy,
            max_order,
        )?;
        if images
            .iter()
            .all(|image| image.status() == BezierAlgebraicImageStatus::Transformed)
        {
            parameter.retain_rational_quadratic_derivative_images(self, images.clone());
        }
        Ok(images)
    }
}

fn point_image(
    parameter: &BezierAlgebraicParameter2,
    coefficients: CoordinatePolynomials,
    policy: &CurveContext,
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
    policy: &CurveContext,
) -> CurveResult<RationalBezierAlgebraicPointImage2> {
    let parameter_root = parameter_representation(parameter, policy);
    if !parameter_root.is_valid() {
        let RationalCoordinatePolynomials {
            x_numerator,
            y_numerator,
            denominator,
        } = coefficients;
        return Ok(RationalBezierAlgebraicPointImage2::new(
            BezierAlgebraicImageStatus::RetainedRationalExpression,
            parameter_root,
            None,
            None,
            Some(RetainedRationalPointExpression {
                parameter: parameter.clone(),
                x_numerator,
                y_numerator,
                denominator,
            }),
            Some("retained an exact Real-coefficient rational point expression".to_owned()),
        ));
    }
    let (x, y) = rational_coordinate_image_pair(
        &parameter_root,
        coefficients.x_numerator,
        coefficients.y_numerator,
        coefficients.denominator,
        policy,
    );
    let Some(x) = x else {
        return Ok(RationalBezierAlgebraicPointImage2::new(
            BezierAlgebraicImageStatus::XImageFailed,
            parameter_root,
            None,
            None,
            None,
            Some("x rational coordinate image failed".to_owned()),
        ));
    };
    let Some(y) = y else {
        return Ok(RationalBezierAlgebraicPointImage2::new(
            BezierAlgebraicImageStatus::YImageFailed,
            parameter_root,
            Some(x),
            None,
            None,
            Some("y rational coordinate image failed".to_owned()),
        ));
    };
    Ok(RationalBezierAlgebraicPointImage2::new(
        BezierAlgebraicImageStatus::Transformed,
        parameter_root,
        Some(x),
        Some(y),
        None,
        None,
    ))
}

pub(crate) fn rational_point_image_from_power_basis(
    parameter: &BezierAlgebraicParameter2,
    x_numerator: Vec<Real>,
    y_numerator: Vec<Real>,
    denominator: Vec<Real>,
    policy: &CurveContext,
) -> CurveResult<RationalBezierAlgebraicPointImage2> {
    let parameter_root = parameter_representation(parameter, policy);
    if !parameter_root.is_valid() {
        return Ok(RationalBezierAlgebraicPointImage2::new(
            BezierAlgebraicImageStatus::RetainedRationalExpression,
            parameter_root,
            None,
            None,
            Some(RetainedRationalPointExpression {
                parameter: parameter.clone(),
                x_numerator,
                y_numerator,
                denominator,
            }),
            Some("retained an exact Real-coefficient rational point expression".to_owned()),
        ));
    }
    let x_numerator = reduce_algebraic_image_polynomial(parameter, x_numerator, policy)?;
    let y_numerator = reduce_algebraic_image_polynomial(parameter, y_numerator, policy)?;
    let denominator = reduce_algebraic_image_polynomial(parameter, denominator, policy)?;
    rational_point_image(
        parameter,
        RationalCoordinatePolynomials {
            x_numerator,
            y_numerator,
            denominator,
        },
        policy,
    )
}

pub(crate) fn rational_derivative_images_from_power_basis(
    parameter: &BezierAlgebraicParameter2,
    mut x_numerator: Vec<Real>,
    mut y_numerator: Vec<Real>,
    denominator: Vec<Real>,
    policy: &CurveContext,
    max_order: usize,
) -> CurveResult<Vec<RationalBezierAlgebraicTangentImage2>> {
    let denominator_derivative = derivative_coefficients(&denominator);
    let mut denominator_power = denominator.clone();
    let mut images = Vec::with_capacity(max_order);
    for order in 1..=max_order {
        let coefficient = Real::from(order as u64);
        x_numerator = subtract_polynomials(
            &multiply_polynomials(&derivative_coefficients(&x_numerator), &denominator),
            &scale_polynomial(
                &multiply_polynomials(&x_numerator, &denominator_derivative),
                coefficient.clone(),
            ),
        );
        y_numerator = subtract_polynomials(
            &multiply_polynomials(&derivative_coefficients(&y_numerator), &denominator),
            &scale_polynomial(
                &multiply_polynomials(&y_numerator, &denominator_derivative),
                coefficient,
            ),
        );
        denominator_power = multiply_polynomials(&denominator_power, &denominator);
        let dx_numerator =
            reduce_algebraic_image_polynomial(parameter, x_numerator.clone(), policy)?;
        let dy_numerator =
            reduce_algebraic_image_polynomial(parameter, y_numerator.clone(), policy)?;
        let derivative_denominator =
            reduce_algebraic_image_polynomial(parameter, denominator_power.clone(), policy)?;
        images.push(rational_tangent_image(
            parameter,
            RationalTangentPolynomials {
                dx_numerator,
                dy_numerator,
                denominator: derivative_denominator,
            },
            policy,
        )?);
    }
    Ok(images)
}

fn reduce_algebraic_image_polynomial(
    parameter: &BezierAlgebraicParameter2,
    coefficients: Vec<Real>,
    policy: &CurveContext,
) -> CurveResult<Vec<Real>> {
    match parameter
        .polynomial()
        .reduce_power_basis(coefficients.clone(), policy)?
    {
        crate::Classification::Decided(remainder) => Ok(remainder),
        crate::Classification::Uncertain(_) => Ok(coefficients),
    }
}

fn tangent_image(
    parameter: &BezierAlgebraicParameter2,
    coefficients: CoordinatePolynomials,
    policy: &CurveContext,
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
    policy: &CurveContext,
) -> CurveResult<RationalBezierAlgebraicTangentImage2> {
    let parameter_root = parameter_representation(parameter, policy);
    if !parameter_root.is_valid() {
        return Ok(RationalBezierAlgebraicTangentImage2::new(
            BezierAlgebraicImageStatus::InvalidParameterEvidence,
            parameter_root,
            None,
            None,
            Some("Bezier algebraic parameter evidence did not validate".to_owned()),
        ));
    }
    let (dx, dy) = rational_coordinate_image_pair(
        &parameter_root,
        coefficients.dx_numerator,
        coefficients.dy_numerator,
        coefficients.denominator,
        policy,
    );
    let Some(dx) = dx else {
        return Ok(RationalBezierAlgebraicTangentImage2::new(
            BezierAlgebraicImageStatus::XImageFailed,
            parameter_root,
            None,
            None,
            Some("dx rational coordinate image failed".to_owned()),
        ));
    };
    let Some(dy) = dy else {
        return Ok(RationalBezierAlgebraicTangentImage2::new(
            BezierAlgebraicImageStatus::YImageFailed,
            parameter_root,
            Some(dx),
            None,
            Some("dy rational coordinate image failed".to_owned()),
        ));
    };
    Ok(RationalBezierAlgebraicTangentImage2::new(
        BezierAlgebraicImageStatus::Transformed,
        parameter_root,
        Some(dx),
        Some(dy),
        None,
    ))
}

fn coordinate_image(
    parameter: &AlgebraicRootRepresentation,
    coefficients: Vec<Real>,
    policy: &CurveContext,
) -> Option<BezierAlgebraicCoordinateImage> {
    if let Some(parameter_value) = parameter.exact_rational_witness() {
        let value = evaluate_power_polynomial(&coefficients, parameter_value);
        let representation = exact_real_algebraic_representation(&value);
        return Some(BezierAlgebraicCoordinateImage {
            evidence: AlgebraicRootPolynomialImageReport {
                status: AlgebraicRootPolynomialImageStatus::Transformed,
                image_coefficients: coefficients.clone(),
                representation: Some(representation),
                message: None,
            },
            coefficients,
        });
    }
    if coefficients.len() == 1 {
        let representation = exact_real_algebraic_representation(&coefficients[0]);
        return Some(BezierAlgebraicCoordinateImage {
            evidence: AlgebraicRootPolynomialImageReport {
                status: AlgebraicRootPolynomialImageStatus::Transformed,
                image_coefficients: coefficients.clone(),
                representation: Some(representation),
                message: None,
            },
            coefficients,
        });
    }
    coordinate_image_from_replay(parameter, coefficients, policy)
}

fn rational_coordinate_image_pair(
    parameter: &AlgebraicRootRepresentation,
    first_numerator_coefficients: Vec<Real>,
    second_numerator_coefficients: Vec<Real>,
    denominator_coefficients: Vec<Real>,
    policy: &CurveContext,
) -> (
    Option<BezierAlgebraicRationalCoordinateImage>,
    Option<BezierAlgebraicRationalCoordinateImage>,
) {
    let [first_evidence, second_evidence] = transform_algebraic_root_rational_images(
        parameter,
        [
            first_numerator_coefficients.as_slice(),
            second_numerator_coefficients.as_slice(),
        ],
        &denominator_coefficients,
        policy.predicate_policy(),
    );
    if first_evidence.status != AlgebraicRootRationalImageStatus::Transformed {
        return (None, None);
    }
    let first = BezierAlgebraicRationalCoordinateImage {
        numerator_coefficients: first_numerator_coefficients,
        denominator_coefficients: denominator_coefficients.clone(),
        evidence: first_evidence,
    };
    let second = (second_evidence.status == AlgebraicRootRationalImageStatus::Transformed)
        .then_some(BezierAlgebraicRationalCoordinateImage {
            numerator_coefficients: second_numerator_coefficients,
            denominator_coefficients,
            evidence: second_evidence,
        });
    (Some(first), second)
}

fn coordinate_image_from_replay(
    parameter: &AlgebraicRootRepresentation,
    coefficients: Vec<Real>,
    policy: &CurveContext,
) -> Option<BezierAlgebraicCoordinateImage> {
    let evidence = transform_algebraic_root_polynomial_image(
        parameter,
        &coefficients,
        policy.predicate_policy(),
    );
    (evidence.status == AlgebraicRootPolynomialImageStatus::Transformed).then_some(
        BezierAlgebraicCoordinateImage {
            coefficients,
            evidence,
        },
    )
}

pub(crate) fn exact_real_algebraic_representation(value: &Real) -> AlgebraicRootRepresentation {
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

pub(crate) fn parameter_representation(
    parameter: &BezierAlgebraicParameter2,
    policy: &CurveContext,
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
    validate_parameter_representation(&mut representation, policy);
    representation
}

fn validate_parameter_representation(
    representation: &mut AlgebraicRootRepresentation,
    policy: &CurveContext,
) {
    representation.validation =
        validate_algebraic_root_representation(representation, policy.predicate_policy());
}

fn linear_parameter_witness(
    parameter: &BezierAlgebraicParameter2,
    policy: &CurveContext,
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
