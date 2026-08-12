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

use hyperreal::{Real, RealSign};
use hypersolve::{
    AlgebraicRootArithmeticOp, AlgebraicRootArithmeticReport, AlgebraicRootArithmeticStatus,
    AlgebraicRootKind, AlgebraicRootPolynomialImageReport, AlgebraicRootPolynomialImageStatus,
    AlgebraicRootRationalImageReport, AlgebraicRootRepresentation, AlgebraicRootValidationReport,
    AlgebraicRootValidationStatus, IsolatedRootInterval, SymbolId,
    arithmetic_algebraic_root_representations,
};
use hypersolve::{
    AlgebraicRootComparisonStatus, AlgebraicRootRefinementComparisonConfig,
    compare_algebraic_root_representations_by_difference,
};
use hypersolve::{
    AlgebraicRootRationalImageStatus, transform_algebraic_root_polynomial_image,
    transform_algebraic_root_rational_images, validate_algebraic_root_representation,
};

use crate::bezier_parameter::strict_coefficients_sign_on_parameter_interval;
use crate::bezier_parameter::{evaluate_coefficients, signed_coefficients_at_parameter};
use crate::classify::{compare_reals, real_sign};
use crate::{
    Aabb2, BezierAlgebraicParameter2, BezierParameter2, Classification, CubicBezier2, CurveContext,
    CurveError, CurveResult, Point2, QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2,
    UncertaintyReason,
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

#[cfg(test)]
mod policy_tests {
    use hyperreal::{Rational, Real};
    use hypersolve::{
        AlgebraicRootArithmeticOp, AlgebraicRootArithmeticStatus, AlgebraicRootKind,
        AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
        IsolatedRootInterval, SymbolId,
    };
    use num::{BigInt, BigUint};

    use super::arithmetic_algebraic_representations_with_policy;
    use crate::{CurveCertainty, CurveContext, policy::resolve_certified_operation};

    #[test]
    fn arithmetic_adapter_retries_policy_dependent_validation() {
        let epsilon = Real::new(
            Rational::from_bigint_fraction(BigInt::from(1_u8), BigUint::from(1_u8) << 1200)
                .expect("positive dyadic epsilon"),
        );
        let half = (Real::one() / Real::from(2_i8)).expect("nonzero denominator");
        let lower = (&half - &epsilon).sqrt().expect("positive lower endpoint");
        let upper = (&half + &epsilon).sqrt().expect("positive upper endpoint");
        let root = AlgebraicRootRepresentation {
            constraint_index: 0,
            symbol: SymbolId(0),
            interval_index: 0,
            polynomial_coefficients: vec![-half, Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower,
                upper,
                exact_root: None,
                distinct_root_count: 1,
            },
            kind: AlgebraicRootKind::IsolatingInterval,
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };

        let strict = arithmetic_algebraic_representations_with_policy(
            &root,
            None,
            AlgebraicRootArithmeticOp::Negate,
            &CurveContext::STRICT,
        );
        assert!(!matches!(
            strict.status,
            AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
                | AlgebraicRootArithmeticStatus::ComputedRepresentation
        ));

        let outcome = resolve_certified_operation(&CurveContext::APPROXIMATE_512, |policy| {
            Ok::<_, ()>(arithmetic_algebraic_representations_with_policy(
                &root,
                None,
                AlgebraicRootArithmeticOp::Negate,
                policy,
            ))
        })
        .expect("infallible operation");
        assert_eq!(
            outcome.value.status,
            AlgebraicRootArithmeticStatus::ComputedRepresentation
        );
        assert_eq!(outcome.certainty, CurveCertainty::Approximate512Consumed);
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
    // A canonical Real may itself be an exact radical/expression rather than
    // a rational witness.  Replay it against the selected root's defining
    // polynomial before constructing a second synthetic root.  A zero is
    // sufficient only inside the validated isolating interval, which rejects
    // every foreign conjugate without adjoining the two representations.
    let residual = evaluate_coefficients(&representation.polynomial_coefficients, value);
    if real_sign(&residual, policy) == Some(RealSign::Zero)
        && matches!(
            compare_reals(value, &representation.interval.lower, policy),
            Some(Ordering::Equal | Ordering::Greater)
        )
        && matches!(
            compare_reals(value, &representation.interval.upper, policy),
            Some(Ordering::Equal | Ordering::Less)
        )
    {
        return crate::Classification::Decided(Ordering::Equal);
    }
    let exact = exact_real_algebraic_representation(value);
    compare_algebraic_representations_with_policy(representation, &exact, policy)
        .map(crate::Classification::Decided)
        .unwrap_or(crate::Classification::Uncertain(
            crate::UncertaintyReason::Ordering,
        ))
}

/// Compares represented roots through Hypersolve without hiding a policy
/// terminal from Hypercurve's aggregate certainty.
///
/// A strict comparison is always attempted first. If only
/// `APPROXIMATE_512` decides, the caller's operation frame is marked before
/// the ordering is returned.
pub(crate) fn compare_algebraic_representations_with_policy(
    first: &AlgebraicRootRepresentation,
    second: &AlgebraicRootRepresentation,
    policy: &CurveContext,
) -> Option<Ordering> {
    let compare = |predicate_policy| {
        let evidence = compare_algebraic_root_representations_by_difference(
            first,
            second,
            AlgebraicRootRefinementComparisonConfig {
                policy: predicate_policy,
                ..AlgebraicRootRefinementComparisonConfig::default()
            },
        );
        matches!(
            evidence.comparison.status,
            AlgebraicRootComparisonStatus::Compared
                | AlgebraicRootComparisonStatus::SameRepresentation
        )
        .then_some(evidence.comparison.ordering)
        .flatten()
    };
    if let Some(ordering) = compare(hypersolve::PredicatePolicy::STRICT) {
        return Some(ordering);
    }
    if !policy.permits_approximate_512() {
        return None;
    }
    let ordering = compare(hypersolve::PredicatePolicy::APPROXIMATE_512);
    if ordering.is_some() {
        policy.observe_approximate_512();
    }
    ordering
}

/// Constructs represented-root arithmetic through a strict-first policy
/// adapter and records the terminal when only `APPROXIMATE_512` succeeds.
pub(crate) fn arithmetic_algebraic_representations_with_policy(
    left: &AlgebraicRootRepresentation,
    right: Option<&AlgebraicRootRepresentation>,
    operation: AlgebraicRootArithmeticOp,
    policy: &CurveContext,
) -> AlgebraicRootArithmeticReport {
    let strict = arithmetic_algebraic_root_representations(
        left,
        right,
        operation,
        hypersolve::PredicatePolicy::STRICT,
    );
    if matches!(
        strict.status,
        AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
            | AlgebraicRootArithmeticStatus::ComputedRepresentation
            | AlgebraicRootArithmeticStatus::NonRationalInput
    ) || !policy.permits_approximate_512()
    {
        return strict;
    }
    let approximate = arithmetic_algebraic_root_representations(
        left,
        right,
        operation,
        hypersolve::PredicatePolicy::APPROXIMATE_512,
    );
    if matches!(
        approximate.status,
        AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
            | AlgebraicRootArithmeticStatus::ComputedRepresentation
    ) {
        policy.observe_approximate_512();
    }
    approximate
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

pub(crate) struct RationalBezierAlgebraicPointPredicate2<'a> {
    image: &'a RationalBezierAlgebraicPointImage2,
    root: &'a AlgebraicRootRepresentation,
    x_numerator: &'a [Real],
    y_numerator: &'a [Real],
    denominator: &'a [Real],
    parameter: BezierParameter2,
    denominator_sign: RealSign,
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
    /// Returns whether both values share the same immutable exact image.
    ///
    /// This is stronger than geometric equality evidence and therefore a
    /// constant-time positive certificate.  It is particularly useful when a
    /// topology carrier deliberately reuses an authored endpoint image.
    pub(crate) fn shares_storage(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
    }

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

    #[allow(dead_code)] // Consumed when the algebraic cusp carrier enters the arrangement graph.
    pub(crate) fn from_retained_expression(
        parameter: BezierAlgebraicParameter2,
        parameter_root: AlgebraicRootRepresentation,
        x_numerator: Vec<Real>,
        y_numerator: Vec<Real>,
        denominator: Vec<Real>,
        message: &'static str,
    ) -> Self {
        Self::new(
            BezierAlgebraicImageStatus::RetainedRationalExpression,
            parameter_root,
            None,
            None,
            Some(RetainedRationalPointExpression {
                parameter,
                x_numerator,
                y_numerator,
                denominator,
            }),
            Some(message.to_owned()),
        )
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
    /// this selected-parameter rational expression.
    ///
    /// Successfully transformed coordinate images already retain these power
    /// basis coefficients individually. Reusing them here avoids duplicating
    /// three polynomial vectors merely to preserve the cheaper same-field
    /// equality path.
    pub fn retained_coordinate_polynomials(&self) -> Option<(&[Real], &[Real], &[Real])> {
        if let Some(expression) = self.data.retained_expression.as_ref() {
            return Some((
                expression.x_numerator.as_slice(),
                expression.y_numerator.as_slice(),
                expression.denominator.as_slice(),
            ));
        }
        if let Some(source) = self.data.parametric_source.as_ref() {
            let power_basis = source.curve.homogeneous_power_basis().ok()?;
            return Some((
                power_basis.x_numerator.as_slice(),
                power_basis.y_numerator.as_slice(),
                power_basis.weight.as_slice(),
            ));
        }
        let (Some(x), Some(y)) = (self.data.x.as_ref(), self.data.y.as_ref()) else {
            return None;
        };
        (x.denominator_coefficients() == y.denominator_coefficients()).then_some((
            x.numerator_coefficients(),
            y.numerator_coefficients(),
            x.denominator_coefficients(),
        ))
    }

    /// Returns an exact affine-line equation retained by this rational point
    /// expression, when a coefficientwise proof is available.
    ///
    /// The homogeneous power-basis coefficients are vectors `(X, Y, W)`.
    /// If they all lie in one projective plane `aX + bY + cW = 0`, every
    /// selected affine image lies on the corresponding line.  This is a
    /// deliberately sufficient structural certificate: every residual and
    /// every nondegeneracy decision is made under [`CurveContext::STRICT`], so
    /// an approximate terminal can never create geometry.
    pub(crate) fn strict_retained_affine_line_coefficients(&self) -> Option<[Real; 3]> {
        let (x, y, weight) = self.retained_coordinate_polynomials()?;
        let coefficient_count = x.len().max(y.len()).max(weight.len());
        if coefficient_count < 2 {
            return None;
        }
        let coefficient =
            |source: &[Real], index: usize| source.get(index).cloned().unwrap_or_else(Real::zero);
        let homogeneous = |index| {
            [
                coefficient(x, index),
                coefficient(y, index),
                coefficient(weight, index),
            ]
        };
        let strict = &CurveContext::STRICT;

        for first_index in 0..coefficient_count {
            let first = homogeneous(first_index);
            for second_index in (first_index + 1)..coefficient_count {
                let second = homogeneous(second_index);
                let candidate = [
                    Real::diff_of_products(&first[1], &second[2], &first[2], &second[1]),
                    Real::diff_of_products(&first[2], &second[0], &first[0], &second[2]),
                    Real::diff_of_products(&first[0], &second[1], &first[1], &second[0]),
                ];
                let first_affine_sign = real_sign(&candidate[0], strict);
                let second_affine_sign = real_sign(&candidate[1], strict);
                if matches!(
                    (first_affine_sign, second_affine_sign),
                    (Some(RealSign::Zero), Some(RealSign::Zero))
                ) {
                    continue;
                }
                if first_affine_sign.is_none() || second_affine_sign.is_none() {
                    continue;
                }
                let all_coefficients_incident = (0..coefficient_count).all(|index| {
                    let point = homogeneous(index);
                    real_sign(
                        &Real::dot3_refs(
                            [&candidate[0], &candidate[1], &candidate[2]],
                            [&point[0], &point[1], &point[2]],
                        ),
                        strict,
                    ) == Some(RealSign::Zero)
                });
                if all_coefficients_incident {
                    return Some(candidate);
                }
            }
        }
        None
    }

    /// Compares two rational point expressions in one selected parameter
    /// field without constructing independent coordinate roots.
    ///
    /// Equality of `N1/D1` and `N2/D2` is the selected-root sign of
    /// `N1*D2-N2*D1`. The method applies only after the retained parameters are
    /// certified equal; unrelated fields continue to the general represented-
    /// root or multi-field predicate graph.
    pub(crate) fn same_retained_rational_point(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<bool>>> {
        let first_parameter = self.retained_parameter();
        let second_parameter = other.retained_parameter();
        let parameter = match (first_parameter, second_parameter) {
            (Some(first), Some(second)) => {
                if first != second {
                    match BezierParameter2::Algebraic(first.clone())
                        .same_value(&BezierParameter2::Algebraic(second.clone()), policy)?
                    {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) | Classification::Uncertain(_) => {
                            return Ok(None);
                        }
                    }
                }
                first
            }
            (Some(first), None) => {
                let representation = parameter_representation(first, policy);
                if crate::bezier_arrangement::represented_roots_equal(
                    &representation,
                    &other.data.parameter,
                    policy,
                ) != Some(true)
                {
                    return Ok(None);
                }
                first
            }
            (None, Some(second)) => {
                let representation = parameter_representation(second, policy);
                if crate::bezier_arrangement::represented_roots_equal(
                    &self.data.parameter,
                    &representation,
                    policy,
                ) != Some(true)
                {
                    return Ok(None);
                }
                second
            }
            (None, None) => return Ok(None),
        };
        let (
            Some((first_x, first_y, first_denominator)),
            Some((second_x, second_y, second_denominator)),
        ) = (
            self.retained_coordinate_polynomials(),
            other.retained_coordinate_polynomials(),
        )
        else {
            return Ok(None);
        };
        let parameter = BezierParameter2::Algebraic(parameter.clone());
        for (first, second) in [(first_x, second_x), (first_y, second_y)] {
            let first_length = first
                .len()
                .checked_add(second_denominator.len())
                .and_then(|length| length.checked_sub(1))
                .unwrap_or(0);
            let second_length = second
                .len()
                .checked_add(first_denominator.len())
                .and_then(|length| length.checked_sub(1))
                .unwrap_or(0);
            let mut cross_difference = vec![Real::zero(); first_length.max(second_length)];
            for (power, coefficient) in first.iter().enumerate() {
                for (denominator_power, denominator) in second_denominator.iter().enumerate() {
                    let index = power + denominator_power;
                    cross_difference[index] = &cross_difference[index] + coefficient * denominator;
                }
            }
            for (power, coefficient) in second.iter().enumerate() {
                for (denominator_power, denominator) in first_denominator.iter().enumerate() {
                    let index = power + denominator_power;
                    cross_difference[index] = &cross_difference[index] - coefficient * denominator;
                }
            }
            match signed_coefficients_at_parameter(cross_difference, &parameter, policy)? {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                    return Ok(Some(Classification::Decided(false)));
                }
                Classification::Uncertain(_) => return Ok(None),
            }
        }
        Ok(Some(Classification::Decided(true)))
    }

    /// Compares one affine coordinate with a represented Real without forcing
    /// a retained rational expression into an independent algebraic-number
    /// representation.
    ///
    /// Retained cusp and split images share one exact source root. Signing
    /// `N(root) - value*D(root)` and `D(root)` in that local field is both
    /// cheaper and more general than constructing a resultant image for each
    /// coordinate. A certified zero denominator is rejected as invalid affine
    /// evidence; predicate uncertainty remains explicit.
    pub(crate) fn coordinate_order_to_real(
        &self,
        use_x: bool,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>> {
        if let Some(coordinate) = if use_x { self.x() } else { self.y() } {
            return Ok(coordinate.compare_to_real(value, policy));
        }

        if let (Some(parameter), Some((x_numerator, y_numerator, denominator))) = (
            self.retained_parameter(),
            self.retained_coordinate_polynomials(),
        ) {
            let parameter = BezierParameter2::Algebraic(parameter.clone());
            let denominator_sign =
                match signed_coefficients_at_parameter(denominator.to_vec(), &parameter, policy)? {
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::InvalidBezierAlgebraicParameter);
                    }
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let numerator = if use_x { x_numerator } else { y_numerator };
            let difference_length = numerator.len().max(denominator.len());
            let difference = (0..difference_length)
                .map(|index| {
                    numerator.get(index).cloned().unwrap_or_else(Real::zero)
                        - value * denominator.get(index).cloned().unwrap_or_else(Real::zero)
                })
                .collect();
            return Ok(
                match signed_coefficients_at_parameter(difference, &parameter, policy)? {
                    Classification::Decided(RealSign::Zero) => {
                        Classification::Decided(Ordering::Equal)
                    }
                    Classification::Decided(numerator_sign) => {
                        Classification::Decided(if numerator_sign == denominator_sign {
                            Ordering::Greater
                        } else {
                            Ordering::Less
                        })
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                },
            );
        }

        if let Some(resolved) = self.resolved(policy)
            && !Arc::ptr_eq(&self.data, &resolved.data)
        {
            return resolved.coordinate_order_to_real(use_x, value, policy);
        }
        Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
    }

    pub(crate) fn exact_rational_point(&self, policy: &CurveContext) -> Option<Point2> {
        if let (Some(parameter), Some((x_numerator, y_numerator, denominator))) = (
            self.retained_parameter(),
            self.retained_coordinate_polynomials(),
        ) && let Ok(Classification::Decided(Some(parameter))) =
            parameter.represented_rational_root(policy)
        {
            let denominator = evaluate_coefficients(denominator, &parameter);
            if let (Ok(x), Ok(y)) = (
                evaluate_coefficients(x_numerator, &parameter) / &denominator,
                evaluate_coefficients(y_numerator, &parameter) / denominator,
            ) {
                return Some(Point2::new(x, y));
            }
        }

        let point = self.resolved(policy)?;
        Some(Point2::new(
            point
                .x()?
                .representation()?
                .exact_rational_witness()?
                .clone(),
            point
                .y()?
                .representation()?
                .exact_rational_witness()?
                .clone(),
        ))
    }

    /// Returns one represented coordinate even when the other coordinate
    /// remains selected algebraic evidence.
    ///
    /// Axis-support recovery must not require the complete point to collapse
    /// to a represented pair: a point such as `(alpha, 0)` carries an exact
    /// reusable horizontal-line certificate in its second coordinate.
    pub(crate) fn exact_rational_coordinate(
        &self,
        use_x: bool,
        policy: &CurveContext,
    ) -> Option<Real> {
        if let (Some(parameter), Some((x_numerator, y_numerator, denominator))) = (
            self.retained_parameter(),
            self.retained_coordinate_polynomials(),
        ) && let Ok(Classification::Decided(Some(parameter))) =
            parameter.represented_rational_root(policy)
        {
            let denominator = evaluate_coefficients(denominator, &parameter);
            let numerator =
                evaluate_coefficients(if use_x { x_numerator } else { y_numerator }, &parameter);
            if let Ok(coordinate) = numerator / denominator {
                return Some(coordinate);
            }
        }

        let point = self.resolved(policy)?;
        let coordinate = if use_x { point.x()? } else { point.y()? };
        coordinate
            .representation()?
            .exact_rational_witness()
            .cloned()
    }

    pub(crate) fn predicate_evaluator<'a>(
        &'a self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierAlgebraicPointPredicate2<'a>>> {
        let Some(image) = self.resolved(policy) else {
            return Err(CurveError::Topology(
                "algebraic point image did not retain a replayable source".into(),
            ));
        };
        let (x_numerator, y_numerator, denominator) =
            if let Some(expression) = image.data.retained_expression.as_ref() {
                (
                    expression.x_numerator.as_slice(),
                    expression.y_numerator.as_slice(),
                    expression.denominator.as_slice(),
                )
            } else {
                let (Some(x), Some(y)) = (image.x(), image.y()) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                if x.denominator_coefficients() != y.denominator_coefficients() {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                (
                    x.numerator_coefficients(),
                    y.numerator_coefficients(),
                    x.denominator_coefficients(),
                )
            };
        let parameter = if let Some(parameter) = image.retained_parameter() {
            BezierParameter2::Algebraic(parameter.clone())
        } else {
            match BezierParameter2::from_algebraic_root_representation_unbounded(
                image.parameter(),
                policy,
            )? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        };
        let denominator_sign =
            match signed_coefficients_at_parameter(denominator.to_vec(), &parameter, policy)? {
                Classification::Decided(RealSign::Zero) => {
                    return Err(CurveError::InvalidBezierAlgebraicParameter);
                }
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        Ok(Classification::Decided(
            RationalBezierAlgebraicPointPredicate2 {
                image,
                root: image.parameter(),
                x_numerator,
                y_numerator,
                denominator,
                parameter: parameter.refined_isolating_interval(1, &CurveContext::STRICT),
                denominator_sign,
            },
        ))
    }

    /// Returns a compact diagnostic message for failed construction.
    pub fn message(&self) -> Option<&str> {
        self.data.message.as_deref()
    }
}

impl RationalBezierAlgebraicPointPredicate2<'_> {
    fn geometric_sign(
        &self,
        coefficients: Vec<Real>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let fast = strict_coefficients_sign_on_parameter_interval(
            &coefficients,
            &self.parameter,
            &CurveContext::STRICT,
        )?;
        let sign = match fast {
            Some(sign) => Classification::Decided(sign),
            None => signed_coefficients_at_parameter(coefficients, &self.parameter, policy)?,
        };
        Ok(sign.map(|sign| match sign {
            RealSign::Zero => RealSign::Zero,
            sign if sign == self.denominator_sign => RealSign::Positive,
            RealSign::Positive | RealSign::Negative => RealSign::Negative,
        }))
    }

    pub(crate) fn coordinate_order_to_real(
        &self,
        use_x: bool,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>> {
        let zero = Real::zero();
        let one = Real::one();
        self.linear_order_to_real(
            if use_x { &one } else { &zero },
            if use_x { &zero } else { &one },
            value,
            policy,
        )
    }

    pub(crate) fn linear_order_to_real(
        &self,
        x_factor: &Real,
        y_factor: &Real,
        value: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>> {
        let length = self
            .x_numerator
            .len()
            .max(self.y_numerator.len())
            .max(self.denominator.len());
        let difference = (0..length)
            .map(|index| {
                x_factor
                    * self
                        .x_numerator
                        .get(index)
                        .cloned()
                        .unwrap_or_else(Real::zero)
                    + y_factor
                        * self
                            .y_numerator
                            .get(index)
                            .cloned()
                            .unwrap_or_else(Real::zero)
                    - value
                        * self
                            .denominator
                            .get(index)
                            .cloned()
                            .unwrap_or_else(Real::zero)
            })
            .collect();
        Ok(self
            .geometric_sign(difference, policy)?
            .map(|sign| match sign {
                RealSign::Negative => Ordering::Less,
                RealSign::Zero => Ordering::Equal,
                RealSign::Positive => Ordering::Greater,
            }))
    }

    pub(crate) const fn retained_root(&self) -> &AlgebraicRootRepresentation {
        self.root
    }

    pub(crate) const fn point_image(&self) -> &RationalBezierAlgebraicPointImage2 {
        self.image
    }

    pub(crate) const fn retained_parameter(&self) -> &BezierParameter2 {
        &self.parameter
    }

    pub(crate) const fn denominator_sign(&self) -> RealSign {
        self.denominator_sign
    }

    pub(crate) const fn coordinate_polynomials(&self) -> (&[Real], &[Real], &[Real]) {
        (self.x_numerator, self.y_numerator, self.denominator)
    }

    pub(crate) fn homogeneous_linear_difference_sign(
        &self,
        x_numerator: &Real,
        y_numerator: &Real,
        weight: &Real,
        x_factor: &Real,
        y_factor: &Real,
        curve_weight_sign: RealSign,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let curve_linear = x_factor * x_numerator + y_factor * y_numerator;
        let coefficient_count = self
            .x_numerator
            .len()
            .max(self.y_numerator.len())
            .max(self.denominator.len());
        let difference = (0..coefficient_count)
            .map(|index| {
                let query_linear = x_factor
                    * self
                        .x_numerator
                        .get(index)
                        .cloned()
                        .unwrap_or_else(Real::zero)
                    + y_factor
                        * self
                            .y_numerator
                            .get(index)
                            .cloned()
                            .unwrap_or_else(Real::zero);
                &curve_linear
                    * self
                        .denominator
                        .get(index)
                        .cloned()
                        .unwrap_or_else(Real::zero)
                    - weight * query_linear
            })
            .collect();
        Ok(self.geometric_sign(difference, policy)?.map(|sign| {
            if curve_weight_sign == RealSign::Negative {
                match sign {
                    RealSign::Negative => RealSign::Positive,
                    RealSign::Zero => RealSign::Zero,
                    RealSign::Positive => RealSign::Negative,
                }
            } else {
                sign
            }
        }))
    }

    pub(crate) fn oriented_line_side(
        &self,
        start: &Point2,
        end: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        let direction_x = end.x() - start.x();
        let direction_y = end.y() - start.y();
        let coefficient_count = self
            .x_numerator
            .len()
            .max(self.y_numerator.len())
            .max(self.denominator.len());
        let determinant = (0..coefficient_count)
            .map(|index| {
                let denominator = self
                    .denominator
                    .get(index)
                    .cloned()
                    .unwrap_or_else(Real::zero);
                let x = self
                    .x_numerator
                    .get(index)
                    .cloned()
                    .unwrap_or_else(Real::zero)
                    - start.x() * &denominator;
                let y = self
                    .y_numerator
                    .get(index)
                    .cloned()
                    .unwrap_or_else(Real::zero)
                    - start.y() * denominator;
                &direction_x * y - &direction_y * x
            })
            .collect();
        Ok(self
            .geometric_sign(determinant, policy)?
            .map(crate::classify::LineSide::from_real_sign))
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
    retained_expression: Option<RetainedRationalTangentExpression>,
    message: Option<String>,
}

#[derive(Debug, PartialEq)]
struct RetainedRationalTangentExpression {
    parameter: BezierAlgebraicParameter2,
    dx_numerator: Vec<Real>,
    dy_numerator: Vec<Real>,
    denominator: Vec<Real>,
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
        retained_expression: Option<RetainedRationalTangentExpression>,
        message: Option<String>,
    ) -> Self {
        Self {
            data: Arc::new(RationalBezierAlgebraicTangentImageData {
                status,
                parameter,
                dx,
                dy,
                retained_expression,
                message,
            }),
        }
    }

    pub(crate) fn from_retained_expression(
        parameter: BezierAlgebraicParameter2,
        parameter_root: AlgebraicRootRepresentation,
        dx_numerator: Vec<Real>,
        dy_numerator: Vec<Real>,
        denominator: Vec<Real>,
        message: &'static str,
    ) -> Self {
        Self::new(
            BezierAlgebraicImageStatus::RetainedRationalExpression,
            parameter_root,
            None,
            None,
            Some(RetainedRationalTangentExpression {
                parameter,
                dx_numerator,
                dy_numerator,
                denominator,
            }),
            Some(message.to_owned()),
        )
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

    /// Returns the exact isolated source parameter retained for a derivative
    /// rational expression that did not fit the bounded coordinate-image path.
    pub fn retained_parameter(&self) -> Option<&BezierAlgebraicParameter2> {
        self.data
            .retained_expression
            .as_ref()
            .map(|expression| &expression.parameter)
    }

    /// Returns the exact derivative numerators and their shared denominator
    /// when the bounded coordinate-image path retained the source expression.
    pub fn retained_coordinate_polynomials(&self) -> Option<(&[Real], &[Real], &[Real])> {
        self.data.retained_expression.as_ref().map(|expression| {
            (
                expression.dx_numerator.as_slice(),
                expression.dy_numerator.as_slice(),
                expression.denominator.as_slice(),
            )
        })
    }

    pub(crate) fn coordinate_sign(
        &self,
        use_x: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if let Some(expression) = self.data.retained_expression.as_ref() {
            let parameter = BezierParameter2::Algebraic(expression.parameter.clone());
            let denominator = match signed_coefficients_at_parameter(
                expression.denominator.clone(),
                &parameter,
                policy,
            )? {
                Classification::Decided(RealSign::Zero) => {
                    return Err(CurveError::InvalidBezierAlgebraicParameter);
                }
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let numerator = if use_x {
                expression.dx_numerator.clone()
            } else {
                expression.dy_numerator.clone()
            };
            return Ok(
                signed_coefficients_at_parameter(numerator, &parameter, policy)?.map(
                    |sign| match (sign, denominator) {
                        (RealSign::Zero, _) => RealSign::Zero,
                        (first, second) if first == second => RealSign::Positive,
                        (RealSign::Positive | RealSign::Negative, _) => RealSign::Negative,
                    },
                ),
            );
        }

        let coordinate = if use_x { self.dx() } else { self.dy() };
        Ok(coordinate.map_or(
            Classification::Uncertain(UncertaintyReason::Unsupported),
            |coordinate| {
                coordinate
                    .compare_to_real(&Real::zero(), policy)
                    .map(|order| match order {
                        Ordering::Less => RealSign::Negative,
                        Ordering::Equal => RealSign::Zero,
                        Ordering::Greater => RealSign::Positive,
                    })
            },
        ))
    }

    /// Returns the derivative as two represented [`Real`] values when both
    /// exact rational witnesses are already present in the retained image.
    ///
    /// This is deliberately not an approximation or a request to construct a
    /// larger algebraic-number tower. Retained Real-coefficient expressions
    /// are evaluated directly when their shared source root is rational;
    /// otherwise only rational witnesses already proved by Hypersolve are
    /// accepted.
    pub(crate) fn exact_rational_vector(&self, policy: &CurveContext) -> Option<(Real, Real)> {
        if let Some(expression) = self.data.retained_expression.as_ref()
            && let Ok(Classification::Decided(Some(parameter))) =
                expression.parameter.represented_rational_root(policy)
        {
            let denominator = evaluate_coefficients(&expression.denominator, &parameter);
            if let (Ok(dx), Ok(dy)) = (
                evaluate_coefficients(&expression.dx_numerator, &parameter) / &denominator,
                evaluate_coefficients(&expression.dy_numerator, &parameter) / denominator,
            ) {
                return Some((dx, dy));
            }
        }

        Some((
            self.dx()?
                .representation()?
                .exact_rational_witness()?
                .clone(),
            self.dy()?
                .representation()?
                .exact_rational_witness()?
                .clone(),
        ))
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
    rational_point_image_with_parameter_representation(
        parameter,
        parameter_root,
        coefficients,
        policy,
    )
}

fn rational_point_image_with_parameter_representation(
    parameter: &BezierAlgebraicParameter2,
    parameter_root: AlgebraicRootRepresentation,
    coefficients: RationalCoordinatePolynomials,
    policy: &CurveContext,
) -> CurveResult<RationalBezierAlgebraicPointImage2> {
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
    let mut parameter_root = parameter_representation(parameter, policy);
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
    if let Classification::Decided(Some(exact_root)) =
        parameter.represented_rational_root(policy)?
    {
        parameter_root.interval = IsolatedRootInterval {
            lower: exact_root.clone(),
            upper: exact_root.clone(),
            exact_root: Some(exact_root),
            distinct_root_count: 1,
        };
        parameter_root.kind = AlgebraicRootKind::ExactRationalWitness;
        validate_parameter_representation(&mut parameter_root, policy);
    }
    rational_point_image_with_parameter_representation(
        parameter,
        parameter_root,
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
        None,
    ))
}

#[allow(dead_code)] // Consumed when the algebraic cusp carrier enters the arrangement graph.
pub(crate) fn rational_tangent_image_from_power_basis(
    parameter: &BezierAlgebraicParameter2,
    dx_numerator: Vec<Real>,
    dy_numerator: Vec<Real>,
    denominator: Vec<Real>,
    policy: &CurveContext,
) -> CurveResult<RationalBezierAlgebraicTangentImage2> {
    let dx_numerator = reduce_algebraic_image_polynomial(parameter, dx_numerator, policy)?;
    let dy_numerator = reduce_algebraic_image_polynomial(parameter, dy_numerator, policy)?;
    let denominator = reduce_algebraic_image_polynomial(parameter, denominator, policy)?;
    let image = rational_tangent_image(
        parameter,
        RationalTangentPolynomials {
            dx_numerator: dx_numerator.clone(),
            dy_numerator: dy_numerator.clone(),
            denominator: denominator.clone(),
        },
        policy,
    )?;
    Ok(match image.status() {
        BezierAlgebraicImageStatus::Transformed
        | BezierAlgebraicImageStatus::RetainedRationalExpression => image,
        BezierAlgebraicImageStatus::InvalidParameterEvidence
        | BezierAlgebraicImageStatus::XImageFailed
        | BezierAlgebraicImageStatus::YImageFailed => {
            RationalBezierAlgebraicTangentImage2::from_retained_expression(
                parameter.clone(),
                image.parameter().clone(),
                dx_numerator,
                dy_numerator,
                denominator,
                "retained an exact Real-coefficient rational tangent expression",
            )
        }
    })
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
