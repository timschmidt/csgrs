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
    AlgebraicRootRationalImageReport, AlgebraicRootRepresentation, AlgebraicRootValidationReport,
    AlgebraicRootValidationStatus, IsolatedRootInterval, SymbolId,
    arithmetic_algebraic_root_representations,
};
use hypersolve::{
    AlgebraicRootComparisonStatus, AlgebraicRootRefinementComparisonConfig,
    compare_algebraic_root_representations_by_difference,
};
use hypersolve::{
    AlgebraicRootRationalImageStatus, transform_algebraic_root_rational_image,
    transform_algebraic_root_rational_images, validate_algebraic_root_representation,
};

use crate::bezier_parameter::{quadratic_bernstein_to_power, signed_coefficients_at_parameter};
use crate::classify::{compare_reals, real_sign};
use crate::{
    Aabb2, BezierAlgebraicParameter2, BezierParameter2, Classification, CubicBezier2, CurveContext,
    CurveError, CurveResult, Point2, QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2,
    UncertaintyReason,
};
use std::cmp::Ordering;
use std::sync::Arc;
use std::sync::OnceLock;

/// Exact representation used by a Bezier algebraic point or tangent image.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierAlgebraicImageStatus {
    /// Both coordinate images were represented exactly.
    Transformed,
    /// The exact rational-coordinate expressions and their certified
    /// Real-coefficient source root, or an equivalent exact curve/parameter
    /// source, were retained without forcing coordinate representations into
    /// the rational-coefficient algebraic-number package.
    RetainedRationalExpression,
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

#[cfg(test)]
mod policy_tests {
    use hyperreal::{Rational, Real};
    use hypersolve::{
        AlgebraicRootArithmeticOp, AlgebraicRootArithmeticStatus, AlgebraicRootRepresentation,
        AlgebraicRootValidationReport, AlgebraicRootValidationStatus, IsolatedRootInterval,
        SymbolId,
    };
    use num::{BigInt, BigUint};

    use super::arithmetic_algebraic_representations_with_policy;
    use crate::{
        Classification, CurveCertainty, CurveContext, policy::resolve_certified_operation,
    };

    #[test]
    fn arithmetic_adapter_replays_close_nonrational_bounds_strictly() {
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
            polynomial_coefficients: vec![-&half, Real::zero(), Real::one()],
            interval: IsolatedRootInterval {
                lower,
                upper,
                exact_root: None,
                distinct_root_count: 1,
            },
            validation: AlgebraicRootValidationReport {
                status: AlgebraicRootValidationStatus::Valid,
                message: None,
            },
        };

        // Replay the nonrational bounds and point witness strictly even when
        // their differences are smaller than the approximate budget resolves.
        // A stored Valid label cannot certify reversed bounds or a false root.
        let mut reversed = root.clone();
        reversed.interval.lower = root.interval.upper.clone();
        reversed.interval.upper = root.interval.lower.clone();
        let mut false_witness = root.clone();
        false_witness.interval.exact_root = Some(root.interval.upper.clone());
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (input, valid) in [(&root, true), (&reversed, false), (&false_witness, false)] {
                let outcome = resolve_certified_operation(&policy, |attempt| {
                    Ok::<_, ()>(arithmetic_algebraic_representations_with_policy(
                        input,
                        None,
                        AlgebraicRootArithmeticOp::Negate,
                        attempt,
                    ))
                })
                .expect("infallible operation");
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                if valid {
                    assert_eq!(
                        outcome.value.status,
                        AlgebraicRootArithmeticStatus::ComputedRepresentation
                    );
                    let result = outcome
                        .value
                        .result_representation
                        .expect("negation retains the selected negative root");
                    assert!(result.polynomial_coefficients == root.polynomial_coefficients);
                    assert!(result.interval.lower == -&root.interval.upper);
                    assert!(result.interval.upper == -&root.interval.lower);
                    assert_eq!(result.interval.distinct_root_count, 1);
                    assert_eq!(
                        result.validation.status,
                        AlgebraicRootValidationStatus::Valid
                    );
                } else {
                    assert_eq!(
                        outcome.value.status,
                        AlgebraicRootArithmeticStatus::InvalidEvidence
                    );
                    assert!(outcome.value.result_representation.is_none());
                    assert!(outcome.value.exact_result.is_none());
                }
            }
        }
    }

    #[test]
    fn coordinate_images_preserve_native_polynomial_point_values() {
        let half = Real::new(Rational::fraction(1, 2).unwrap());
        let quarter = Real::new(Rational::fraction(1, 4).unwrap());
        let alpha = half.clone().sqrt().unwrap();
        let nested = -&half + (&alpha + &quarter).sqrt().unwrap();
        let root_two = Real::from(2_i8).sqrt().unwrap();
        let mut high_degree = vec![Real::zero(); 65];
        high_degree[0] = -Real::from(1_u64 << 32);
        high_degree[64] = Real::one();
        for (coefficients, parameter, expected) in [
            (vec![Real::zero()], Real::pi(), Real::zero()),
            (vec![Real::pi()], root_two.clone(), Real::pi()),
            (vec![Real::pi(), Real::e()], Real::zero(), Real::pi()),
            (
                vec![Real::pi(), -Real::pi(), Real::one()],
                Real::one(),
                Real::one(),
            ),
            (
                vec![Real::pi(), Real::from(-2_i8) * Real::pi(), Real::one()],
                half,
                quarter,
            ),
            (vec![-alpha, Real::one(), Real::one()], nested, Real::zero()),
            (high_degree, root_two, Real::zero()),
        ] {
            for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
                let image = hypersolve::transform_algebraic_root_rational_image(
                    &AlgebraicRootRepresentation::from_exact_value(&parameter),
                    &coefficients,
                    &[Real::one()],
                    policy.predicate_policy(),
                );
                assert_eq!(
                    image.status,
                    hypersolve::AlgebraicRootRationalImageStatus::Transformed
                );
                assert_eq!(image.numerator_coefficients, coefficients);
                assert_eq!(
                    super::compare_root_representation_to_real(
                        image
                            .representation
                            .as_ref()
                            .expect("an exact image retains its root"),
                        &expected,
                        &policy,
                    ),
                    Classification::Decided(std::cmp::Ordering::Equal)
                );
            }
        }
    }

    #[test]
    fn exact_real_representation_preserves_native_point_payloads() {
        let irrational = AlgebraicRootRepresentation::from_exact_value(&Real::pi());
        let exact = irrational
            .exact_point_witness()
            .expect("an exact Real representation retains its point");
        assert_eq!(exact, &Real::pi());
        assert!(exact.exact_rational_ref().is_none());

        let rational = AlgebraicRootRepresentation::from_exact_value(&Real::from(3_i8));
        assert!(
            rational
                .exact_point_witness()
                .and_then(Real::exact_rational_ref)
                .is_some()
        );
    }

    #[test]
    fn retained_constant_coordinates_do_not_require_scalar_parameter_images() {
        use super::{RationalBezierAlgebraicPointImage2, parameter_representation};
        use crate::{BezierParameter2, BezierParameterPolynomial, Point2, RationalBezier2};

        let strict = CurveContext::STRICT;
        let Classification::Decided(polynomial) = BezierParameterPolynomial::try_new_power_basis(
            vec![Real::from(-1), Real::zero(), Real::from(2)],
            &strict,
        )
        .unwrap() else {
            panic!("exact quadratic");
        };
        let Classification::Decided(roots) =
            polynomial.isolate_unit_interval_roots(&strict).unwrap()
        else {
            panic!("certified positive quadratic root");
        };
        let [BezierParameter2::Algebraic(parameter)] = roots.as_slice() else {
            panic!("the irrational parameter stays selected");
        };
        assert_eq!(
            parameter.represented_exact_point(&strict).unwrap(),
            Classification::Decided(None),
        );
        let retained = |x, y, weight| {
            RationalBezierAlgebraicPointImage2::from_retained_expression(
                parameter.clone(),
                parameter_representation(parameter, &strict),
                x,
                y,
                weight,
                "selected parameter with a constant coordinate",
            )
        };
        for policy in [strict, CurveContext::APPROXIMATE_512] {
            for constant in [Real::zero(), Real::pi(), Real::from(2).sqrt().unwrap()] {
                for weight in [
                    vec![Real::one()],
                    vec![Real::from(-3)],
                    vec![Real::zero(), Real::one()],
                    vec![Real::from(2), Real::one()],
                ] {
                    let numerator: Vec<_> = weight.iter().map(|c| c * &constant).collect();
                    let mut varying = vec![Real::zero()];
                    varying.extend(weight.iter().cloned());
                    for use_x in [false, true] {
                        let (x, y) = if use_x {
                            (numerator.clone(), varying.clone())
                        } else {
                            (varying.clone(), numerator.clone())
                        };
                        let image = retained(x, y, weight.clone());
                        let outcome = resolve_certified_operation(&policy, |attempt| {
                            Ok::<_, ()>(image.exact_coordinate(use_x, attempt))
                        })
                        .unwrap();
                        assert_eq!(outcome.value, Some(constant.clone()));
                        assert_eq!(outcome.certainty, CurveCertainty::Certified);
                        assert!(image.x().is_none() && image.y().is_none());
                        assert!(image.exact_point(&policy).is_none());
                    }
                }
            }

            // Both constant coordinates can use the same conversion without
            // constructing either a scalar parameter or Cartesian root images.
            let point = Point2::new(Real::pi(), Real::from(2).sqrt().unwrap());
            let image = retained(
                vec![point.x().clone()],
                vec![point.y().clone()],
                vec![Real::one()],
            );
            assert_eq!(image.exact_point(&policy), Some(point.clone()));

            // A lazy curve/parameter source must not resolve its other axis.
            let curve = RationalBezier2::try_new(
                vec![
                    Point2::new(Real::zero(), point.y().clone()),
                    Point2::new(Real::one(), point.y().clone()),
                ],
                vec![Real::one(); 2],
            )
            .unwrap();
            let image = RationalBezierAlgebraicPointImage2::from_parametric_source(
                curve,
                parameter.clone(),
                &policy,
            );
            assert_eq!(
                image.exact_coordinate(false, &policy),
                Some(point.y().clone())
            );
            assert!(image.parametric_source().unwrap().resolved.get().is_none());

            // Proportional homogeneous coordinates do not remove a pole.
            let pole = polynomial.coefficients().to_vec();
            let image = retained(
                vec![Real::zero(), Real::one()],
                pole.iter().map(|c| c * point.y()).collect(),
                pole,
            );
            assert!(image.exact_coordinate(false, &policy).is_none());
            assert!(image.exact_point(&policy).is_none());

            // Neither an unresolved denominator nor an almost-constant
            // numerator can acquire a scalar value through approximation.
            let epsilon = Real::one() - Real::from(2).powi_i64(-600).unwrap().cos();
            assert_eq!(super::real_sign(&epsilon, &strict), None);
            for (numerator, weight) in [
                (vec![epsilon.clone()], vec![epsilon.clone()]),
                (vec![point.y().clone(), epsilon], vec![Real::one()]),
            ] {
                let image = retained(vec![Real::zero(), Real::one()], numerator, weight);
                let outcome = resolve_certified_operation(&policy, |attempt| {
                    Ok::<_, ()>(image.exact_coordinate(false, attempt))
                })
                .unwrap();
                assert!(outcome.value.is_none());
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
            }
        }
    }
}

/// Recognizes a parameter-independent rational coordinate without scalar-root
/// elimination. The caller must separately certify that the denominator does
/// not vanish at its selected parameter; proportionality does not remove poles.
fn strict_constant_rational_coordinate(numerator: &[Real], denominator: &[Real]) -> Option<Real> {
    let pivot = denominator.iter().position(|coefficient| {
        matches!(
            real_sign(coefficient, &CurveContext::STRICT),
            Some(RealSign::Negative | RealSign::Positive)
        )
    })?;
    let zero = Real::zero();
    let numerator_pivot = numerator.get(pivot).unwrap_or(&zero);
    let denominator_pivot = &denominator[pivot];
    let proportional = (0..numerator.len().max(denominator.len())).all(|index| {
        real_sign(
            &Real::diff_of_products(
                numerator.get(index).unwrap_or(&zero),
                denominator_pivot,
                numerator_pivot,
                denominator.get(index).unwrap_or(&zero),
            ),
            &CurveContext::STRICT,
        ) == Some(RealSign::Zero)
    });
    if proportional {
        (numerator_pivot / denominator_pivot).ok()
    } else {
        None
    }
}

fn compare_root_representation_to_real(
    representation: &AlgebraicRootRepresentation,
    value: &Real,
    policy: &CurveContext,
) -> crate::Classification<Ordering> {
    if let Some(exact) = representation.exact_point_witness() {
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
    let residual = Real::eval_poly(&representation.polynomial_coefficients, value);
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
    let exact = AlgebraicRootRepresentation::from_exact_value(value);
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
    if algebraic_arithmetic_succeeded(&strict.status)
        || strict.status == AlgebraicRootArithmeticStatus::NonRationalInput
        || !policy.permits_approximate_512()
    {
        return strict;
    }
    let approximate = arithmetic_algebraic_root_representations(
        left,
        right,
        operation,
        hypersolve::PredicatePolicy::APPROXIMATE_512,
    );
    if algebraic_arithmetic_succeeded(&approximate.status) {
        policy.observe_approximate_512();
    }
    approximate
}

pub(crate) fn algebraic_arithmetic_succeeded(status: &AlgebraicRootArithmeticStatus) -> bool {
    matches!(
        status,
        AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
            | AlgebraicRootArithmeticStatus::ComputedExactRealWitness
            | AlgebraicRootArithmeticStatus::ComputedRepresentation
    )
}

/// Exact affine point of a polynomial or rational Bezier at one selected algebraic parameter.
#[derive(Clone, Debug)]
pub struct RationalBezierAlgebraicPointImage2 {
    data: Arc<RationalBezierAlgebraicPointImageData>,
}

#[derive(Debug, PartialEq)]
struct RationalBezierAlgebraicPointImageData {
    parameter: AlgebraicRootRepresentation,
    definition: RationalPointDefinition,
}

// The enclosing Arc already allocates one immutable image. Keep its active
// definition inline instead of adding a separate allocation for coordinates.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, PartialEq)]
enum RationalPointDefinition {
    Coordinates {
        x: BezierAlgebraicRationalCoordinateImage,
        y: BezierAlgebraicRationalCoordinateImage,
    },
    Expression {
        expression: RetainedRationalPointExpression,
        message: &'static str,
    },
    Parametric(RetainedRationalPointParametricSource),
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
    resolved: OnceLock<RationalBezierAlgebraicPointImage2>,
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

    fn from_coordinates(
        parameter: AlgebraicRootRepresentation,
        x: BezierAlgebraicRationalCoordinateImage,
        y: BezierAlgebraicRationalCoordinateImage,
    ) -> Self {
        Self {
            data: Arc::new(RationalBezierAlgebraicPointImageData {
                parameter,
                definition: RationalPointDefinition::Coordinates { x, y },
            }),
        }
    }

    /// The owning geometry supplies the non-pole certificate for this exact
    /// expression when independent coordinate projection cannot finish.
    pub(crate) fn from_retained_expression(
        parameter: BezierAlgebraicParameter2,
        parameter_root: AlgebraicRootRepresentation,
        x_numerator: Vec<Real>,
        y_numerator: Vec<Real>,
        denominator: Vec<Real>,
        message: &'static str,
    ) -> Self {
        Self {
            data: Arc::new(RationalBezierAlgebraicPointImageData {
                parameter: parameter_root,
                definition: RationalPointDefinition::Expression {
                    expression: RetainedRationalPointExpression {
                        parameter,
                        x_numerator,
                        y_numerator,
                        denominator,
                    },
                    message,
                },
            }),
        }
    }

    /// The caller owns the source's finite-domain proof at this parameter.
    /// Retain that source without forcing its Cartesian coordinate images.
    pub(crate) fn from_parametric_source(
        curve: RationalBezier2,
        parameter: BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> Self {
        Self {
            data: Arc::new(RationalBezierAlgebraicPointImageData {
                parameter: parameter_representation(&parameter, policy),
                definition: RationalPointDefinition::Parametric(
                    RetainedRationalPointParametricSource {
                        curve,
                        parameter,
                        resolved: OnceLock::new(),
                    },
                ),
            }),
        }
    }

    fn retained_expression(&self) -> Option<&RetainedRationalPointExpression> {
        match &self.data.definition {
            RationalPointDefinition::Expression { expression, .. } => Some(expression),
            _ => None,
        }
    }

    fn parametric_source(&self) -> Option<&RetainedRationalPointParametricSource> {
        match &self.data.definition {
            RationalPointDefinition::Parametric(source) => Some(source),
            _ => None,
        }
    }

    #[inline(never)]
    pub(crate) fn resolved(&self, policy: &CurveContext) -> Option<&Self> {
        let Some(source) = self.parametric_source() else {
            return Some(self);
        };
        if let Some(image) = source.resolved.get() {
            return Some(image);
        }
        let Ok(Classification::Decided(image)) = source
            .curve
            .point_at_algebraic_parameter(&source.parameter, policy)
        else {
            return None;
        };
        // A failed attempt is not a point definition or a permanent cache
        // entry. Only exact successful images can satisfy later requests.
        let _ = source.resolved.set(image);
        source.resolved.get()
    }

    /// Materializes both exact coordinate representations from whichever
    /// point authority was retained. Authored rational expressions stay lazy
    /// until a tensor kernel actually needs Cartesian coordinates.
    pub(crate) fn represented_coordinates(
        &self,
        policy: &CurveContext,
    ) -> Option<[AlgebraicRootRepresentation; 2]> {
        let resolved = self.resolved(policy)?;
        if let (Some(x), Some(y)) = (resolved.x(), resolved.y()) {
            return Some([x.representation()?.clone(), y.representation()?.clone()]);
        }
        let expression = resolved.retained_expression()?;
        let Classification::Decided(image) = rational_point_image_from_power_basis(
            &expression.parameter,
            expression.x_numerator.clone(),
            expression.y_numerator.clone(),
            expression.denominator.clone(),
            policy,
        )
        .ok()?
        else {
            return None;
        };
        Some([
            image.x()?.representation()?.clone(),
            image.y()?.representation()?.clone(),
        ])
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
        if let Some(source) = self.parametric_source() {
            let parameter = crate::BezierParameter2::Algebraic(source.parameter.clone())
                .refined_isolating_interval(refinement_steps, policy);
            let (start, end) = match &parameter {
                crate::BezierParameter2::Exact(parameter) => (parameter, parameter),
                crate::BezierParameter2::Algebraic(parameter) => {
                    (parameter.interval().start(), parameter.interval().end())
                }
            };
            match source.curve.subcurve_between_exact(start, end, policy) {
                Ok(Classification::Decided(curve)) => curve.certified_bounds_classified(),
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
        let (Some(first), Some(second)) = (self.parametric_source(), other.parametric_source())
        else {
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

    /// Reports whether coordinates are projected or retained by their exact source.
    pub fn status(&self) -> BezierAlgebraicImageStatus {
        match &self.data.definition {
            RationalPointDefinition::Coordinates { .. } => BezierAlgebraicImageStatus::Transformed,
            RationalPointDefinition::Expression { .. } | RationalPointDefinition::Parametric(_) => {
                BezierAlgebraicImageStatus::RetainedRationalExpression
            }
        }
    }

    /// Returns the represented Bezier parameter used as the source root.
    pub fn parameter(&self) -> &AlgebraicRootRepresentation {
        &self.data.parameter
    }

    /// Returns the x coordinate rational image when construction reached it.
    pub fn x(&self) -> Option<&BezierAlgebraicRationalCoordinateImage> {
        match &self.data.definition {
            RationalPointDefinition::Coordinates { x, .. } => Some(x),
            _ => None,
        }
    }

    /// Returns the y coordinate rational image when construction reached it.
    pub fn y(&self) -> Option<&BezierAlgebraicRationalCoordinateImage> {
        match &self.data.definition {
            RationalPointDefinition::Coordinates { y, .. } => Some(y),
            _ => None,
        }
    }

    /// Returns the exact isolated source parameter retained for a
    /// Real-coefficient rational expression.
    pub fn retained_parameter(&self) -> Option<&BezierAlgebraicParameter2> {
        match &self.data.definition {
            RationalPointDefinition::Coordinates { .. } => None,
            RationalPointDefinition::Expression { expression, .. } => Some(&expression.parameter),
            RationalPointDefinition::Parametric(source) => Some(&source.parameter),
        }
    }

    /// Returns the exact x numerator, y numerator, and shared denominator for
    /// this selected-parameter rational expression.
    ///
    /// Successfully transformed coordinate images already retain these power
    /// basis coefficients individually. Reusing them here avoids duplicating
    /// three polynomial vectors merely to preserve the cheaper same-field
    /// equality path.
    pub fn retained_coordinate_polynomials(&self) -> Option<(&[Real], &[Real], &[Real])> {
        match &self.data.definition {
            RationalPointDefinition::Expression { expression, .. } => Some((
                expression.x_numerator.as_slice(),
                expression.y_numerator.as_slice(),
                expression.denominator.as_slice(),
            )),
            RationalPointDefinition::Parametric(source) => {
                let power_basis = source.curve.homogeneous_power_basis().ok()?;
                Some((
                    power_basis.x_numerator.as_slice(),
                    power_basis.y_numerator.as_slice(),
                    power_basis.weight.as_slice(),
                ))
            }
            RationalPointDefinition::Coordinates { x, y } => {
                (x.denominator_coefficients() == y.denominator_coefficients()).then_some((
                    x.numerator_coefficients(),
                    y.numerator_coefficients(),
                    x.denominator_coefficients(),
                ))
            }
        }
    }

    /// Materializes one exact linear projection while the Cartesian
    /// coordinates still share their selected parameter field.
    ///
    /// Combining `aX + bY` coefficientwise before rational-image elimination
    /// preserves cancellations that independent x/y root representations and
    /// interval boxes necessarily forget. Construction is always STRICT;
    /// callers remain responsible for applying any approximate equality
    /// terminal only while comparing the resulting exact representations.
    pub(crate) fn strict_linear_projection_representation(
        &self,
        coefficient_x: &Real,
        coefficient_y: &Real,
    ) -> Option<AlgebraicRootRepresentation> {
        let Classification::Decided(predicate) =
            self.predicate_evaluator(&CurveContext::STRICT).ok()?
        else {
            return None;
        };
        let x_numerator = predicate.x_numerator;
        let y_numerator = predicate.y_numerator;
        let denominator = predicate.denominator;
        let coefficient_count = x_numerator.len().max(y_numerator.len());
        let zero = Real::zero();
        let mut numerator = Vec::with_capacity(coefficient_count.max(1));
        for index in 0..coefficient_count {
            let x = x_numerator.get(index).unwrap_or(&zero);
            let y = y_numerator.get(index).unwrap_or(&zero);
            numerator.push(Real::dot2_refs([coefficient_x, coefficient_y], [x, y]));
        }
        if numerator.is_empty() {
            numerator.push(Real::zero());
        }
        while numerator.len() > 1
            && real_sign(
                numerator.last().expect("a nonempty projection numerator"),
                &CurveContext::STRICT,
            ) == Some(RealSign::Zero)
        {
            numerator.pop();
        }

        if let Some(value) = strict_constant_rational_coordinate(&numerator, denominator) {
            return Some(AlgebraicRootRepresentation::from_exact_value(&value));
        }
        let evidence = transform_algebraic_root_rational_image(
            predicate.root,
            &numerator,
            denominator,
            hypersolve::PredicatePolicy::STRICT,
        );
        (evidence.status == AlgebraicRootRationalImageStatus::Transformed)
            .then_some(evidence.representation)
            .flatten()
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
        let Some(parameter) = shared_image_parameter(
            self.retained_parameter(),
            &self.data.parameter,
            other.retained_parameter(),
            &other.data.parameter,
            policy,
        )?
        else {
            return Ok(None);
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
            match signed_coefficients_at_parameter(&cross_difference, &parameter, policy)? {
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
                match signed_coefficients_at_parameter(denominator, &parameter, policy)? {
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
            let difference: Vec<_> = (0..difference_length)
                .map(|index| {
                    numerator.get(index).cloned().unwrap_or_else(Real::zero)
                        - value * denominator.get(index).cloned().unwrap_or_else(Real::zero)
                })
                .collect();
            return Ok(
                match signed_coefficients_at_parameter(&difference, &parameter, policy)? {
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

    /// Returns a point when both coordinates can be materialized as exact
    /// [`Real`] values. Callers that require rational payloads must inspect
    /// [`Real::exact_rational_ref`].
    pub(crate) fn exact_point(&self, policy: &CurveContext) -> Option<Point2> {
        if let (Some(parameter), Some((x_numerator, y_numerator, denominator))) = (
            self.retained_parameter(),
            self.retained_coordinate_polynomials(),
        ) && let Ok(Classification::Decided(Some(parameter))) =
            parameter.represented_exact_point(policy)
        {
            let denominator = Real::eval_poly(denominator, &parameter);
            if let (Ok(x), Ok(y)) = (
                Real::eval_poly(x_numerator, &parameter) / &denominator,
                Real::eval_poly(y_numerator, &parameter) / denominator,
            ) {
                return Some(Point2::new(x, y));
            }
        }

        Some(Point2::new(
            self.exact_coordinate(true, policy)?,
            self.exact_coordinate(false, policy)?,
        ))
    }

    /// Returns one represented coordinate even when the other coordinate
    /// remains selected algebraic evidence.
    ///
    /// Axis-support recovery must not require the complete point to collapse
    /// to a represented pair: a point such as `(alpha, 0)` carries an exact
    /// reusable horizontal-line certificate in its second coordinate. The
    /// returned [`Real`] can be non-rational.
    pub(crate) fn exact_coordinate(&self, use_x: bool, policy: &CurveContext) -> Option<Real> {
        if let (Some(parameter), Some((x_numerator, y_numerator, denominator))) = (
            self.retained_parameter(),
            self.retained_coordinate_polynomials(),
        ) {
            let numerator = if use_x { x_numerator } else { y_numerator };
            if let Some(coordinate) = strict_constant_rational_coordinate(numerator, denominator)
                && matches!(
                    signed_coefficients_at_parameter(
                        denominator,
                        &BezierParameter2::Algebraic(parameter.clone()),
                        &policy.strict_counterpart(),
                    ),
                    Ok(Classification::Decided(
                        RealSign::Positive | RealSign::Negative
                    ))
                )
            {
                return Some(coordinate);
            }
            if let Ok(Classification::Decided(Some(parameter))) =
                parameter.represented_exact_point(policy)
            {
                let denominator = Real::eval_poly(denominator, &parameter);
                let numerator = Real::eval_poly(numerator, &parameter);
                if let Ok(coordinate) = numerator / denominator {
                    return Some(coordinate);
                }
            }
        }

        let point = self.resolved(policy)?;
        let coordinate = if use_x { point.x()? } else { point.y()? };
        coordinate.representation()?.exact_point_witness().cloned()
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
            if let Some(expression) = image.retained_expression() {
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
            match signed_coefficients_at_parameter(denominator, &parameter, policy)? {
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

    /// Describes a retained expression when coordinate projection is deferred.
    pub fn message(&self) -> Option<&str> {
        match &self.data.definition {
            RationalPointDefinition::Expression { message, .. } => Some(message),
            _ => None,
        }
    }
}

impl RationalBezierAlgebraicPointPredicate2<'_> {
    fn geometric_sign(
        &self,
        coefficients: Vec<Real>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let sign = signed_coefficients_at_parameter(&coefficients, &self.parameter, policy)?;
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

fn shared_image_parameter<'a>(
    first_parameter: Option<&'a BezierAlgebraicParameter2>,
    first_root: &AlgebraicRootRepresentation,
    second_parameter: Option<&'a BezierAlgebraicParameter2>,
    second_root: &AlgebraicRootRepresentation,
    policy: &CurveContext,
) -> CurveResult<Option<&'a BezierAlgebraicParameter2>> {
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
                second_root,
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
                first_root,
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
    Ok(Some(parameter))
}

/// Exact finite image of a polynomial or rational Bezier derivative vector.
///
/// A value always contains either two exact coordinate images or a retained
/// source expression whose denominator is certified nonzero. Construction
/// blockers remain in [`Classification`] instead of becoming tangent values.
#[derive(Clone, Debug)]
pub struct RationalBezierAlgebraicTangentImage2 {
    data: Arc<RationalBezierAlgebraicTangentImageData>,
}

#[derive(Debug, PartialEq)]
struct RationalBezierAlgebraicTangentImageData {
    parameter: AlgebraicRootRepresentation,
    definition: RationalTangentDefinition,
}

#[derive(Debug, PartialEq)]
enum RationalTangentDefinition {
    Coordinates {
        dx: BezierAlgebraicRationalCoordinateImage,
        dy: BezierAlgebraicRationalCoordinateImage,
    },
    Expression(RetainedRationalTangentExpression),
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
    fn new(parameter: AlgebraicRootRepresentation, definition: RationalTangentDefinition) -> Self {
        Self {
            data: Arc::new(RationalBezierAlgebraicTangentImageData {
                parameter,
                definition,
            }),
        }
    }

    /// Returns the exact representation retained by this tangent.
    pub fn status(&self) -> BezierAlgebraicImageStatus {
        match &self.data.definition {
            RationalTangentDefinition::Coordinates { .. } => {
                BezierAlgebraicImageStatus::Transformed
            }
            RationalTangentDefinition::Expression(_) => {
                BezierAlgebraicImageStatus::RetainedRationalExpression
            }
        }
    }

    /// Returns the represented Bezier parameter used as the source root.
    pub fn parameter(&self) -> &AlgebraicRootRepresentation {
        &self.data.parameter
    }

    /// Returns the derivative x coordinate when its root image is materialized.
    pub fn dx(&self) -> Option<&BezierAlgebraicRationalCoordinateImage> {
        match &self.data.definition {
            RationalTangentDefinition::Coordinates { dx, .. } => Some(dx),
            RationalTangentDefinition::Expression(_) => None,
        }
    }

    /// Returns the derivative y coordinate when its root image is materialized.
    pub fn dy(&self) -> Option<&BezierAlgebraicRationalCoordinateImage> {
        match &self.data.definition {
            RationalTangentDefinition::Coordinates { dy, .. } => Some(dy),
            RationalTangentDefinition::Expression(_) => None,
        }
    }

    fn retained_expression(&self) -> Option<&RetainedRationalTangentExpression> {
        match &self.data.definition {
            RationalTangentDefinition::Coordinates { .. } => None,
            RationalTangentDefinition::Expression(expression) => Some(expression),
        }
    }

    /// Returns the exact isolated source parameter retained when coordinate
    /// projection is unavailable.
    pub fn retained_parameter(&self) -> Option<&BezierAlgebraicParameter2> {
        self.retained_expression()
            .map(|expression| &expression.parameter)
    }

    /// Returns exact derivative numerators and their certified nonzero denominator
    /// when the tangent retains its source expression.
    pub fn retained_coordinate_polynomials(&self) -> Option<(&[Real], &[Real], &[Real])> {
        self.retained_expression()
            .map(|_| self.coordinate_polynomials())
    }

    fn coordinate_polynomials(&self) -> (&[Real], &[Real], &[Real]) {
        match &self.data.definition {
            // The coordinate-pair constructor retains the same source
            // denominator for both coordinates.
            RationalTangentDefinition::Coordinates { dx, dy } => (
                dx.numerator_coefficients(),
                dy.numerator_coefficients(),
                dx.denominator_coefficients(),
            ),
            RationalTangentDefinition::Expression(expression) => (
                &expression.dx_numerator,
                &expression.dy_numerator,
                &expression.denominator,
            ),
        }
    }

    /// Signs a cross or dot product in one certified selected parameter, preserving
    /// correlations that separate coordinate-root images would discard.
    /// Returns None when there is no proved common selected parameter; that
    /// case still uses the represented-coordinate arithmetic path.
    pub(crate) fn shared_parameter_bilinear_sign(
        &self,
        other: &Self,
        dot: bool,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RealSign>>> {
        let strict = policy.strict_counterpart();
        let Some(parameter) = shared_image_parameter(
            self.retained_parameter(),
            &self.data.parameter,
            other.retained_parameter(),
            &other.data.parameter,
            &strict,
        )?
        else {
            return Ok(None);
        };
        let (first_x, first_y, first_denominator) = self.coordinate_polynomials();
        let (second_x, second_y, second_denominator) = other.coordinate_polynomials();
        let selected = BezierParameter2::Algebraic(parameter.clone());
        let mut reverse_sign = false;
        for denominator in [first_denominator, second_denominator] {
            match signed_coefficients_at_parameter(denominator, &selected, &strict)? {
                Classification::Decided(RealSign::Positive) => {}
                Classification::Decided(RealSign::Negative) => reverse_sign = !reverse_sign,
                Classification::Decided(RealSign::Zero) => {
                    return Err(CurveError::InvalidBezierAlgebraicParameter);
                }
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            }
        }
        let determinant = reduce_algebraic_image_polynomial(
            parameter,
            if dot {
                let mut product = multiply_polynomials(first_x, second_x);
                let other = multiply_polynomials(first_y, second_y);
                product.resize(product.len().max(other.len()), Real::zero());
                for (coefficient, term) in product.iter_mut().zip(other) {
                    *coefficient = &*coefficient + term;
                }
                product
            } else {
                subtract_polynomials(
                    &multiply_polynomials(first_x, second_y),
                    &multiply_polynomials(first_y, second_x),
                )
            },
            &strict,
        )?;
        Ok(Some(
            signed_coefficients_at_parameter(&determinant, &selected, &strict)?.map(|sign| match (
                sign,
                reverse_sign,
            ) {
                (RealSign::Positive, true) => RealSign::Negative,
                (RealSign::Negative, true) => RealSign::Positive,
                _ => sign,
            }),
        ))
    }

    /// Signs a linear combination against arbitrary exact constant coordinates.
    /// The selected root and the denominator domain remain authoritative.
    pub(crate) fn constant_linear_combination_sign(
        &self,
        x: &Real,
        y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RealSign>>> {
        let Some(expression) = self.retained_expression() else {
            return Ok(None);
        };
        let strict = policy.strict_counterpart();
        let selected = BezierParameter2::Algebraic(expression.parameter.clone());
        let denominator =
            match signed_coefficients_at_parameter(&expression.denominator, &selected, &strict)? {
                Classification::Decided(RealSign::Zero) => {
                    return Err(CurveError::InvalidBezierAlgebraicParameter);
                }
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
        let zero = Real::zero();
        let numerator = (0..expression
            .dx_numerator
            .len()
            .max(expression.dy_numerator.len()))
            .map(|i| {
                x * expression.dx_numerator.get(i).unwrap_or(&zero)
                    + y * expression.dy_numerator.get(i).unwrap_or(&zero)
            })
            .collect();
        let numerator =
            reduce_algebraic_image_polynomial(&expression.parameter, numerator, &strict)?;
        Ok(Some(
            signed_coefficients_at_parameter(&numerator, &selected, &strict)?.map(|sign| {
                match (sign, denominator) {
                    (RealSign::Zero, _) => RealSign::Zero,
                    (first, second) if first == second => RealSign::Positive,
                    _ => RealSign::Negative,
                }
            }),
        ))
    }

    /// Negates an unprojected derivative while retaining its selected source.
    pub(crate) fn negated_retained_expression(&self) -> Option<Self> {
        let expression = self.retained_expression()?;
        Some(Self::new(
            self.data.parameter.clone(),
            RationalTangentDefinition::Expression(RetainedRationalTangentExpression {
                parameter: expression.parameter.clone(),
                dx_numerator: expression.dx_numerator.iter().map(|value| -value).collect(),
                dy_numerator: expression.dy_numerator.iter().map(|value| -value).collect(),
                denominator: expression.denominator.clone(),
            }),
        ))
    }

    pub(crate) fn coordinate_sign(
        &self,
        use_x: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        if let Some(expression) = self.retained_expression() {
            let parameter = BezierParameter2::Algebraic(expression.parameter.clone());
            let denominator = match signed_coefficients_at_parameter(
                &expression.denominator,
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
                &expression.dx_numerator
            } else {
                &expression.dy_numerator
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
    /// exact point witnesses are already present in the retained image.
    ///
    /// This is deliberately not an approximation or a request to construct a
    /// larger algebraic-number tower. Retained Real-coefficient expressions
    /// are evaluated directly when their shared source root has materialized
    /// as an exact point; otherwise only exact point witnesses already proved
    /// by Hypersolve are accepted.
    pub(crate) fn exact_vector(&self, policy: &CurveContext) -> Option<(Real, Real)> {
        if let Some(expression) = self.retained_expression()
            && let Ok(Classification::Decided(Some(parameter))) =
                expression.parameter.represented_exact_point(policy)
        {
            let denominator = Real::eval_poly(&expression.denominator, &parameter);
            if let (Ok(dx), Ok(dy)) = (
                Real::eval_poly(&expression.dx_numerator, &parameter) / &denominator,
                Real::eval_poly(&expression.dy_numerator, &parameter) / denominator,
            ) {
                return Some((dx, dy));
            }
        }

        Some((
            self.dx()?.representation()?.exact_point_witness()?.clone(),
            self.dy()?.representation()?.exact_point_witness()?.clone(),
        ))
    }
}

impl QuadraticBezier2 {
    /// Evaluates this quadratic at an isolated algebraic parameter.
    ///
    /// The returned x/y coordinates are `hypersolve` represented roots for the
    /// exact coordinate polynomials
    /// `P0 + 2(P1-P0)t + (P0-2P1+P2)t^2`. Polynomial and rational
    /// curves share the same exact point carrier; a unit denominator preserves
    /// arbitrary exact coefficients when coordinate projection is unavailable.
    pub fn point_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierAlgebraicPointImage2>> {
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
    ) -> CurveResult<Classification<RationalBezierAlgebraicTangentImage2>> {
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
    ) -> CurveResult<Classification<RationalBezierAlgebraicTangentImage2>> {
        tangent_image(
            parameter,
            derivative_polynomials(quadratic_tangent_coefficients(self)),
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
    ) -> CurveResult<Classification<RationalBezierAlgebraicPointImage2>> {
        point_image(parameter, cubic_point_coefficients(self), policy)
    }

    /// Evaluates this cubic's first derivative at an isolated algebraic
    /// parameter as exact represented coordinate images.
    pub fn tangent_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierAlgebraicTangentImage2>> {
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
    ) -> CurveResult<Classification<RationalBezierAlgebraicTangentImage2>> {
        tangent_image(
            parameter,
            derivative_polynomials(cubic_tangent_coefficients(self)),
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
    ) -> CurveResult<Classification<RationalBezierAlgebraicTangentImage2>> {
        tangent_image(
            parameter,
            derivative_polynomials(derivative_polynomials(cubic_tangent_coefficients(self))),
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
    /// polynomial image construction above. Only a decided finite image can
    /// become an exact curve point; a pole returns a boundary blocker and an
    /// unresolved denominator retains its predicate reason. See the exactness model for the exact-object
    /// boundary and the Bernstein curve model for the homogeneous conic equations.
    pub fn point_at_algebraic_parameter(
        &self,
        parameter: &BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierAlgebraicPointImage2>> {
        if let Some(image) = parameter.cached_rational_quadratic_point_image(self) {
            return Ok(Classification::Decided(image));
        }
        let image = rational_point_image(parameter, rational_point_coefficients(self), policy)?;
        if let Classification::Decided(image) = &image
            && image.status() == crate::BezierAlgebraicImageStatus::Transformed
        {
            // Retained expressions own this parameter, so storing them here
            // would create a strong ownership cycle.
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
    ) -> CurveResult<Classification<RationalBezierAlgebraicTangentImage2>> {
        if let Some(images) = parameter.cached_rational_quadratic_derivative_images(self, 1) {
            return Ok(Classification::Decided(
                images
                    .into_iter()
                    .next()
                    .expect("one retained derivative image was requested"),
            ));
        }
        let image = rational_tangent_image(parameter, rational_tangent_coefficients(self), policy)?;
        if let Classification::Decided(image) = &image
            && image.status() == BezierAlgebraicImageStatus::Transformed
        {
            // Retained expressions own this parameter and must not form a cache cycle.
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
    ) -> CurveResult<Classification<RationalBezierAlgebraicTangentImage2>> {
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
    ) -> CurveResult<Classification<Vec<RationalBezierAlgebraicTangentImage2>>> {
        if let Some(images) = parameter.cached_rational_quadratic_derivative_images(self, max_order)
        {
            return Ok(Classification::Decided(images));
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
        if let Classification::Decided(images) = &images
            && images
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
) -> CurveResult<Classification<RationalBezierAlgebraicPointImage2>> {
    rational_point_image(
        parameter,
        RationalCoordinatePolynomials {
            x_numerator: coefficients.x,
            y_numerator: coefficients.y,
            denominator: vec![Real::one()],
        },
        policy,
    )
}

fn rational_point_image(
    parameter: &BezierAlgebraicParameter2,
    coefficients: RationalCoordinatePolynomials,
    policy: &CurveContext,
) -> CurveResult<Classification<RationalBezierAlgebraicPointImage2>> {
    let strict = policy.strict_counterpart();
    let parameter_root = parameter_representation(parameter, &strict);
    rational_point_image_with_parameter_representation(
        parameter,
        parameter_root,
        coefficients,
        &strict,
    )
}

fn rational_point_image_with_parameter_representation(
    parameter: &BezierAlgebraicParameter2,
    parameter_root: AlgebraicRootRepresentation,
    coefficients: RationalCoordinatePolynomials,
    policy: &CurveContext,
) -> CurveResult<Classification<RationalBezierAlgebraicPointImage2>> {
    match rational_coordinate_image_pair(
        parameter,
        &parameter_root,
        coefficients.x_numerator,
        coefficients.y_numerator,
        coefficients.denominator,
        policy,
    )? {
        RationalCoordinateImagePair::Transformed {
            first: x,
            second: y,
        } => Ok(Classification::Decided(
            RationalBezierAlgebraicPointImage2::from_coordinates(parameter_root, x, y),
        )),
        RationalCoordinateImagePair::Retained {
            first_numerator: x_numerator,
            second_numerator: y_numerator,
            denominator,
        } => Ok(Classification::Decided(
            RationalBezierAlgebraicPointImage2::from_retained_expression(
                parameter.clone(),
                parameter_root,
                x_numerator,
                y_numerator,
                denominator,
                "retained an exact non-pole Real-coefficient rational point expression",
            ),
        )),
        RationalCoordinateImagePair::Failed(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(crate) fn rational_point_image_from_power_basis(
    parameter: &BezierAlgebraicParameter2,
    x_numerator: Vec<Real>,
    y_numerator: Vec<Real>,
    denominator: Vec<Real>,
    policy: &CurveContext,
) -> CurveResult<Classification<RationalBezierAlgebraicPointImage2>> {
    let strict = policy.strict_counterpart();
    let mut parameter_root = parameter_representation(parameter, &strict);
    let x_numerator = reduce_algebraic_image_polynomial(parameter, x_numerator, &strict)?;
    let y_numerator = reduce_algebraic_image_polynomial(parameter, y_numerator, &strict)?;
    let denominator = reduce_algebraic_image_polynomial(parameter, denominator, &strict)?;
    if let Classification::Decided(Some(exact_root)) = parameter.represented_exact_point(&strict)? {
        parameter_root.interval = IsolatedRootInterval {
            lower: exact_root.clone(),
            upper: exact_root.clone(),
            exact_root: Some(exact_root),
            distinct_root_count: 1,
        };
        validate_parameter_representation(&mut parameter_root, &strict);
    }
    rational_point_image_with_parameter_representation(
        parameter,
        parameter_root,
        RationalCoordinatePolynomials {
            x_numerator,
            y_numerator,
            denominator,
        },
        &strict,
    )
}

pub(crate) fn rational_derivative_images_from_power_basis(
    parameter: &BezierAlgebraicParameter2,
    mut x_numerator: Vec<Real>,
    mut y_numerator: Vec<Real>,
    mut denominator: Vec<Real>,
    policy: &CurveContext,
    max_order: usize,
) -> CurveResult<Classification<Vec<RationalBezierAlgebraicTangentImage2>>> {
    if max_order == 0 {
        return Ok(Classification::Decided(Vec::new()));
    }
    let strict = policy.strict_counterpart();
    let denominator_degree = denominator
        .iter()
        .rposition(|coefficient| !coefficient.definitely_zero())
        .unwrap_or(0);
    let denominator_image =
        reduce_algebraic_image_polynomial(parameter, denominator.clone(), &strict)?;
    let mut previous_denominator_power = vec![Real::one()];
    let mut denominator_power = denominator_image.clone();
    let mut denominator_derivatives = Vec::new();
    let mut numerators = vec![(
        reduce_algebraic_image_polynomial(parameter, x_numerator.clone(), &strict)?,
        reduce_algebraic_image_polynomial(parameter, y_numerator.clone(), &strict)?,
    )];
    let mut images = Vec::with_capacity(max_order);
    for order in 1..=max_order {
        // Differentiate only the original source polynomials. Congruence at
        // the selected root does not preserve their derivatives.
        x_numerator = derivative_coefficients(&x_numerator);
        y_numerator = derivative_coefficients(&y_numerator);
        if order <= denominator_degree {
            denominator = derivative_coefficients(&denominator);
            let derivative =
                reduce_algebraic_image_polynomial(parameter, denominator.clone(), &strict)?;
            denominator_derivatives.push(reduce_algebraic_image_polynomial(
                parameter,
                multiply_polynomials(&derivative, &previous_denominator_power),
                &strict,
            )?);
        }
        let mut dx_numerator = multiply_polynomials(
            &reduce_algebraic_image_polynomial(parameter, x_numerator.clone(), &strict)?,
            &denominator_power,
        );
        let mut dy_numerator = multiply_polynomials(
            &reduce_algebraic_image_polynomial(parameter, y_numerator.clone(), &strict)?,
            &denominator_power,
        );
        // Write N_k and D_j for the original source derivatives. Leibniz
        // applied to N = D*C gives C^(k) = A_k / D^(k+1), where
        // A_k = N_k*D^k - sum_j binomial(k,j)*D_j*D^(j-1)*A_(k-j).
        // These products only use values at the selected root,
        // so every retained A_k and D derivative/power may stay reduced.
        // No inverse modulo the whole source polynomial is required.
        for (index, derivative) in denominator_derivatives.iter().enumerate() {
            if derivative.iter().all(Real::definitely_zero) {
                continue;
            }
            let denominator_order = index + 1;
            let Some(coefficient) =
                crate::rational_bezier_general::exact_binomial(order, denominator_order)
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let (previous_x, previous_y) = &numerators[order - denominator_order];
            dx_numerator = subtract_polynomials(
                &dx_numerator,
                &scale_polynomial(
                    &multiply_polynomials(derivative, previous_x),
                    coefficient.clone(),
                ),
            );
            dy_numerator = subtract_polynomials(
                &dy_numerator,
                &scale_polynomial(&multiply_polynomials(derivative, previous_y), coefficient),
            );
        }
        let dx_numerator = reduce_algebraic_image_polynomial(parameter, dx_numerator, &strict)?;
        let dy_numerator = reduce_algebraic_image_polynomial(parameter, dy_numerator, &strict)?;
        numerators.push((dx_numerator.clone(), dy_numerator.clone()));
        previous_denominator_power = denominator_power;
        denominator_power = reduce_algebraic_image_polynomial(
            parameter,
            multiply_polynomials(&previous_denominator_power, &denominator_image),
            &strict,
        )?;
        match rational_tangent_image(
            parameter,
            RationalTangentPolynomials {
                dx_numerator,
                dy_numerator,
                denominator: denominator_power.clone(),
            },
            &strict,
        )? {
            Classification::Decided(image) => images.push(image),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    Ok(Classification::Decided(images))
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
) -> CurveResult<Classification<RationalBezierAlgebraicTangentImage2>> {
    rational_tangent_image_from_power_basis(
        parameter,
        coefficients.x,
        coefficients.y,
        vec![Real::one()],
        policy,
    )
}

fn rational_tangent_image(
    parameter: &BezierAlgebraicParameter2,
    coefficients: RationalTangentPolynomials,
    policy: &CurveContext,
) -> CurveResult<Classification<RationalBezierAlgebraicTangentImage2>> {
    let strict = policy.strict_counterpart();
    let parameter_root = parameter_representation(parameter, &strict);
    let definition = match rational_coordinate_image_pair(
        parameter,
        &parameter_root,
        coefficients.dx_numerator,
        coefficients.dy_numerator,
        coefficients.denominator,
        &strict,
    )? {
        RationalCoordinateImagePair::Transformed {
            first: dx,
            second: dy,
        } => RationalTangentDefinition::Coordinates { dx, dy },
        RationalCoordinateImagePair::Retained {
            first_numerator: dx_numerator,
            second_numerator: dy_numerator,
            denominator,
        } => RationalTangentDefinition::Expression(RetainedRationalTangentExpression {
            parameter: parameter.clone(),
            dx_numerator,
            dy_numerator,
            denominator,
        }),
        RationalCoordinateImagePair::Failed(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(
        RationalBezierAlgebraicTangentImage2::new(parameter_root, definition),
    ))
}

pub(crate) fn rational_tangent_image_from_power_basis(
    parameter: &BezierAlgebraicParameter2,
    dx_numerator: Vec<Real>,
    dy_numerator: Vec<Real>,
    denominator: Vec<Real>,
    policy: &CurveContext,
) -> CurveResult<Classification<RationalBezierAlgebraicTangentImage2>> {
    let strict = policy.strict_counterpart();
    let dx_numerator = reduce_algebraic_image_polynomial(parameter, dx_numerator, &strict)?;
    let dy_numerator = reduce_algebraic_image_polynomial(parameter, dy_numerator, &strict)?;
    let denominator = reduce_algebraic_image_polynomial(parameter, denominator, &strict)?;
    rational_tangent_image(
        parameter,
        RationalTangentPolynomials {
            dx_numerator,
            dy_numerator,
            denominator,
        },
        &strict,
    )
}

enum RationalCoordinateImagePair {
    Transformed {
        first: BezierAlgebraicRationalCoordinateImage,
        second: BezierAlgebraicRationalCoordinateImage,
    },
    Retained {
        first_numerator: Vec<Real>,
        second_numerator: Vec<Real>,
        denominator: Vec<Real>,
    },
    Failed(UncertaintyReason),
}

fn rational_coordinate_image_pair(
    parameter: &BezierAlgebraicParameter2,
    parameter_root: &AlgebraicRootRepresentation,
    first_numerator_coefficients: Vec<Real>,
    second_numerator_coefficients: Vec<Real>,
    denominator_coefficients: Vec<Real>,
    policy: &CurveContext,
) -> CurveResult<RationalCoordinateImagePair> {
    let strict = policy.strict_counterpart();
    if parameter_root.is_valid() {
        let [first_evidence, second_evidence] = transform_algebraic_root_rational_images(
            parameter_root,
            [
                first_numerator_coefficients.as_slice(),
                second_numerator_coefficients.as_slice(),
            ],
            &denominator_coefficients,
            strict.predicate_policy(),
        );
        if first_evidence.status == AlgebraicRootRationalImageStatus::Transformed
            && second_evidence.status == AlgebraicRootRationalImageStatus::Transformed
        {
            return Ok(RationalCoordinateImagePair::Transformed {
                first: BezierAlgebraicRationalCoordinateImage {
                    numerator_coefficients: first_numerator_coefficients,
                    denominator_coefficients: denominator_coefficients.clone(),
                    evidence: first_evidence,
                },
                second: BezierAlgebraicRationalCoordinateImage {
                    numerator_coefficients: second_numerator_coefficients,
                    denominator_coefficients,
                    evidence: second_evidence,
                },
            });
        }
    }

    let selected_parameter = BezierParameter2::Algebraic(parameter.clone());
    match signed_coefficients_at_parameter(&denominator_coefficients, &selected_parameter, &strict)?
    {
        Classification::Decided(RealSign::Positive | RealSign::Negative) => {
            Ok(RationalCoordinateImagePair::Retained {
                first_numerator: first_numerator_coefficients,
                second_numerator: second_numerator_coefficients,
                denominator: denominator_coefficients,
            })
        }
        Classification::Decided(RealSign::Zero) => Ok(RationalCoordinateImagePair::Failed(
            UncertaintyReason::Boundary,
        )),
        Classification::Uncertain(reason) => Ok(RationalCoordinateImagePair::Failed(reason)),
    }
}

pub(crate) fn parameter_representation(
    parameter: &BezierAlgebraicParameter2,
    policy: &CurveContext,
) -> AlgebraicRootRepresentation {
    let mut representation = certified_parameter_representation(parameter, policy);
    validate_parameter_representation(&mut representation, policy);
    representation
}

/// Bridges an internally certified singleton parameter into Hypersolve root
/// evidence without replaying its complete Sturm proof. The Bezier parameter
/// constructor has already certified this exact polynomial/interval pair, and
/// refined clones share that same certificate and retained Sturm sequence.
pub(crate) fn certified_parameter_representation(
    parameter: &BezierAlgebraicParameter2,
    policy: &CurveContext,
) -> AlgebraicRootRepresentation {
    let interval = parameter.interval();
    let exact_root = parameter.scalar().cloned().or_else(|| {
        if parameter.polynomial().degree() != 1 {
            return None;
        }
        match parameter.represented_exact_point(&policy.strict_counterpart()) {
            Ok(Classification::Decided(root)) => root,
            Ok(Classification::Uncertain(_)) | Err(_) => None,
        }
    });
    AlgebraicRootRepresentation {
        constraint_index: 0,
        symbol: SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: parameter.polynomial().coefficients().to_vec(),
        interval: IsolatedRootInterval {
            lower: interval.start().clone(),
            upper: interval.end().clone(),
            exact_root,
            distinct_root_count: parameter.root_count(),
        },
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    }
}

fn validate_parameter_representation(
    representation: &mut AlgebraicRootRepresentation,
    policy: &CurveContext,
) {
    representation.validation =
        validate_algebraic_root_representation(representation, policy.predicate_policy());
}

fn quadratic_point_coefficients(curve: &QuadraticBezier2) -> CoordinatePolynomials {
    CoordinatePolynomials {
        x: quadratic_bernstein_to_power([curve.start().x(), curve.control().x(), curve.end().x()])
            .into(),
        y: quadratic_bernstein_to_power([curve.start().y(), curve.control().y(), curve.end().y()])
            .into(),
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
        x_numerator: quadratic_bernstein_to_power(weighted_x.each_ref()).into(),
        y_numerator: quadratic_bernstein_to_power(weighted_y.each_ref()).into(),
        denominator: quadratic_bernstein_to_power(weights.each_ref()).into(),
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
    if left.is_empty() || right.is_empty() {
        return Vec::new();
    }
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
