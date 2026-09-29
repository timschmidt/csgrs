//! Certified tangent-order predicates for algebraic Bezier endpoint images.
//!
//! Native and retained arrangement traversal use the same cross/dot half-plane
//! ordering. Represented coordinate roots use Hypersolve arithmetic and certified
//! interval signs. Exact derivative images can instead retain a selected source;
//! their signs are evaluated in that source without coordinate projection.
//! Proved affine parameter relations transport foreign source expressions into
//! the same field; distinct selected roots are never treated as equal.
//! Source identity and denominator domains remain available for replay. Scalar
//! normalization reuses reduced Hypersolve field values in scoped arithmetic
//! batches, preserving correlation without retaining growing expression trees.
//! An unavailable coordinate-root view does not make an exact vector invalid.
//! Unresolved decisions remain classified; no parameter interval is sampled as
//! a coordinate.

use std::{cmp::Ordering, sync::Arc};

use crate::classify::compare_reals;
use crate::{Classification, CurveContext, RationalBezierAlgebraicTangentImage2};
use hyperreal::{Real, RealSign};
use hypersolve::{
    AlgebraicField, AlgebraicFieldError, AlgebraicFieldValue, AlgebraicRootArithmeticOp,
    AlgebraicRootArithmeticReport, AlgebraicRootArithmeticStatus, AlgebraicRootRepresentation,
};

/// An exact tangent vector with represented coordinate roots or a shared
/// selected-source derivative image. Coordinate projection is optional.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierAlgebraicTangentVector2 {
    definition: TangentVectorDefinition,
}

#[derive(Clone, Debug, PartialEq)]
enum TangentVectorDefinition {
    Coordinates(Arc<(AlgebraicRootRepresentation, AlgebraicRootRepresentation)>),
    Image(RationalBezierAlgebraicTangentImage2),
}

impl BezierAlgebraicTangentVector2 {
    /// Constructs a vector from exact represented coordinate roots.
    pub fn new(dx: AlgebraicRootRepresentation, dy: AlgebraicRootRepresentation) -> Self {
        Self {
            definition: TangentVectorDefinition::Coordinates(Arc::new((dx, dy))),
        }
    }

    /// Retains an exact derivative image, including its selected source when
    /// independent coordinate roots are unavailable. This does not project it.
    pub fn from_image(image: &RationalBezierAlgebraicTangentImage2) -> Self {
        Self {
            definition: TangentVectorDefinition::Image(image.clone()),
        }
    }

    /// Borrows both coordinate roots when already represented.
    /// An absent Cartesian view does not make the vector inexact.
    pub fn represented_coordinates(
        &self,
    ) -> Option<(&AlgebraicRootRepresentation, &AlgebraicRootRepresentation)> {
        match &self.definition {
            TangentVectorDefinition::Coordinates(coordinates) => {
                Some((&coordinates.0, &coordinates.1))
            }
            TangentVectorDefinition::Image(image) => {
                Some((image.dx()?.representation()?, image.dy()?.representation()?))
            }
        }
    }

    fn image(&self) -> Option<&RationalBezierAlgebraicTangentImage2> {
        match &self.definition {
            TangentVectorDefinition::Image(image) => Some(image),
            TangentVectorDefinition::Coordinates(_) => None,
        }
    }

    pub(crate) fn negated(&self, policy: &CurveContext) -> Option<Self> {
        if let Some(image) = self
            .image()
            .and_then(RationalBezierAlgebraicTangentImage2::negated_retained_expression)
        {
            return Some(Self::from_image(&image));
        }
        let (x, y) = self.represented_coordinates()?;
        Some(Self::new(
            negate_algebraic_root(x, policy)?,
            negate_algebraic_root(y, policy)?,
        ))
    }
}

/// Certified turn ordering for two candidate tangents around a base tangent.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BezierTangentTurnOrdering2 {
    /// The first candidate is encountered before the second in counter-clockwise order.
    FirstBeforeSecond,
    /// The second candidate is encountered before the first in counter-clockwise order.
    SecondBeforeFirst,
}

/// Status for algebraic tangent-order comparison.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) enum BezierAlgebraicTangentOrderStatus {
    /// The two candidate turns were ordered.
    Ordered,
    /// The candidates have the same exact direction.
    SameDirection,
    /// One of the input tangent vectors was certified zero.
    ZeroTangent,
    /// Exact algebraic arithmetic failed to construct a needed scalar.
    ArithmeticFailed,
    /// A needed scalar sign could not be certified.
    SignUndecided,
}

/// Status for comparing two same-direction algebraic tangent branches with
/// second-order local evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) enum BezierAlgebraicSameTangentOrderStatus {
    /// The two same-tangent candidates were ordered by signed curvature.
    Ordered,
    /// The retained evidence still represents the same local branch direction.
    SameDirection,
    /// One of the input first-derivative vectors was certified zero.
    ZeroTangent,
    /// Exact algebraic arithmetic failed to construct a needed scalar.
    ArithmeticFailed,
    /// A needed scalar sign could not be certified.
    SignUndecided,
}

/// Sign construction evidence for a cross, dot, or norm-squared scalar.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierAlgebraicScalarSignEvidence {
    arithmetic: Vec<AlgebraicRootArithmeticReport>,
    value: Option<TangentScalar>,
    /// Certified sign relative to zero.
    pub sign: Option<Ordering>,
    /// Compact diagnostic for construction or sign failure.
    pub message: Option<String>,
}

// Each scalar has one exact definition. Bilinear inputs are retained lazily
// for sign-only predicates; normalization consumes them in a selected field.
#[derive(Clone, Debug, PartialEq)]
enum TangentScalar {
    Represented(AlgebraicRootRepresentation),
    Selected(AlgebraicFieldValue),
    Bilinear(Box<RetainedTangentBilinear>),
}

impl BezierAlgebraicScalarSignEvidence {
    /// Borrows the scalar root when independently represented.
    pub fn represented_scalar(&self) -> Option<&AlgebraicRootRepresentation> {
        match self.value.as_ref()? {
            TangentScalar::Represented(value) => Some(value),
            _ => None,
        }
    }

    /// Borrows a normalized selected-field value, with finite denominator and
    /// selected-root evidence available for exact arithmetic and sign replay.
    pub fn selected_scalar(&self) -> Option<&AlgebraicFieldValue> {
        match self.value.as_ref()? {
            TangentScalar::Selected(value) => Some(value),
            _ => None,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
struct RetainedTangentBilinear {
    first: BezierAlgebraicTangentVector2,
    second: BezierAlgebraicTangentVector2,
    dot: bool,
}

/// Evidence for a certified algebraic tangent-order predicate.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierAlgebraicTangentOrderEvidence {
    /// Final predicate status.
    pub status: BezierAlgebraicTangentOrderStatus,
    /// Certified ordering when `status == Ordered`.
    pub ordering: Option<BezierTangentTurnOrdering2>,
    /// Base/first cross-product sign evidence.
    pub base_first_cross: Option<BezierAlgebraicScalarSignEvidence>,
    /// Base/second cross-product sign evidence.
    pub base_second_cross: Option<BezierAlgebraicScalarSignEvidence>,
    /// First/second cross-product sign evidence.
    pub first_second_cross: Option<BezierAlgebraicScalarSignEvidence>,
    /// Compact diagnostic for unresolved predicates.
    pub message: Option<String>,
}

/// Evidence for a certified algebraic same-tangent higher-order predicate.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierAlgebraicSameTangentOrderEvidence {
    /// Final predicate status.
    pub status: BezierAlgebraicSameTangentOrderStatus,
    /// Certified ordering when `status == Ordered`.
    pub ordering: Option<BezierTangentTurnOrdering2>,
    /// First candidate's signed normal derivative witness at the compared order.
    pub first_side_witness: Option<BezierAlgebraicScalarSignEvidence>,
    /// Second candidate's signed normal derivative witness at the compared order.
    pub second_side_witness: Option<BezierAlgebraicScalarSignEvidence>,
    /// Difference of normalized derivative witnesses after clearing positive
    /// speed denominators, squared when comparing curvature magnitudes.
    pub normalized_difference: Option<BezierAlgebraicScalarSignEvidence>,
    /// Compact diagnostic for unresolved predicates.
    pub message: Option<String>,
}

#[cfg(test)]
/// Compares two candidate tangent turns from a base tangent.
///
/// The result matches the native branch-order predicate: first classify each
/// candidate into the positive or negative half-turn from `base` using cross
/// and dot signs, then order candidates in the same half by the sign of
/// `first x second`. Represented roots use exact `hypersolve` arithmetic or
/// certified interval bounds. Retained images can use a proved common selected
/// parameter or an exact constant operand, with their denominator signs checked
/// separately. No isolating interval is sampled as a coordinate.
pub(crate) fn compare_algebraic_tangent_turn_from_base(
    base: &BezierAlgebraicTangentVector2,
    first: &BezierAlgebraicTangentVector2,
    second: &BezierAlgebraicTangentVector2,
    policy: &CurveContext,
) -> Classification<BezierAlgebraicTangentOrderEvidence> {
    compare_algebraic_tangent_turn_from_base_impl(base, first, second, policy, true, false)
}

pub(crate) fn compare_algebraic_tangent_turn_from_base_sign_only(
    base: &BezierAlgebraicTangentVector2,
    first: &BezierAlgebraicTangentVector2,
    second: &BezierAlgebraicTangentVector2,
    policy: &CurveContext,
) -> Classification<BezierAlgebraicTangentOrderEvidence> {
    compare_algebraic_tangent_turn_from_base_impl(base, first, second, policy, false, false)
}

pub(crate) fn compare_algebraic_tangent_filled_left_face_sign_only(
    base: &BezierAlgebraicTangentVector2,
    first: &BezierAlgebraicTangentVector2,
    second: &BezierAlgebraicTangentVector2,
    policy: &CurveContext,
) -> Classification<BezierAlgebraicTangentOrderEvidence> {
    compare_algebraic_tangent_turn_from_base_impl(base, first, second, policy, false, true)
}

pub(crate) fn algebraic_endpoint_tangent_cross_sign(
    first: &RationalBezierAlgebraicTangentImage2,
    second: &RationalBezierAlgebraicTangentImage2,
    policy: &CurveContext,
) -> Classification<RealSign> {
    let first = BezierAlgebraicTangentVector2::from_image(first);
    let second = BezierAlgebraicTangentVector2::from_image(second);
    let cross = cross_sign(&first, &second, policy, false);
    match sign_status(&cross) {
        ScalarSignStatus::Positive => Classification::Decided(RealSign::Positive),
        ScalarSignStatus::Negative => Classification::Decided(RealSign::Negative),
        ScalarSignStatus::Zero => Classification::Decided(RealSign::Zero),
        ScalarSignStatus::Undecided => {
            Classification::Uncertain(crate::UncertaintyReason::RealSign)
        }
        ScalarSignStatus::ArithmeticFailed => {
            Classification::Uncertain(crate::UncertaintyReason::Unsupported)
        }
    }
}

fn compare_algebraic_tangent_turn_from_base_impl(
    base: &BezierAlgebraicTangentVector2,
    first: &BezierAlgebraicTangentVector2,
    second: &BezierAlgebraicTangentVector2,
    policy: &CurveContext,
    retain_scalar: bool,
    reverse_within_half: bool,
) -> Classification<BezierAlgebraicTangentOrderEvidence> {
    for tangent in [base, first, second] {
        match tangent_nonzero(tangent, policy) {
            AlgebraicTangentNonzero::Nonzero => {}
            AlgebraicTangentNonzero::Zero(evidence) => {
                return Classification::Decided(order_evidence(
                    BezierAlgebraicTangentOrderStatus::ZeroTangent,
                    None,
                    None,
                    None,
                    None,
                    Some(format!("zero tangent certified by {:?}", evidence.sign)),
                ));
            }
            AlgebraicTangentNonzero::Undecided(evidence) => {
                return Classification::Decided(order_evidence(
                    BezierAlgebraicTangentOrderStatus::SignUndecided,
                    None,
                    None,
                    None,
                    None,
                    evidence.message,
                ));
            }
            AlgebraicTangentNonzero::ArithmeticFailed(evidence) => {
                return Classification::Decided(order_evidence(
                    BezierAlgebraicTangentOrderStatus::ArithmeticFailed,
                    None,
                    None,
                    None,
                    None,
                    evidence.message,
                ));
            }
        }
    }

    let (first_half, base_first_cross) = match turn_half(base, first, policy, retain_scalar) {
        AlgebraicHalfTurn::Half(half, cross) => (half, cross),
        AlgebraicHalfTurn::ZeroTangent(cross, dot) => {
            return Classification::Decided(order_evidence(
                BezierAlgebraicTangentOrderStatus::ZeroTangent,
                None,
                Some(cross),
                None,
                Some(dot),
                Some("first tangent has zero direction relative to base".to_owned()),
            ));
        }
        AlgebraicHalfTurn::Undecided(cross, dot) => {
            return Classification::Decided(order_evidence(
                BezierAlgebraicTangentOrderStatus::SignUndecided,
                None,
                Some(cross),
                None,
                dot,
                Some("could not certify first candidate half-turn".to_owned()),
            ));
        }
        AlgebraicHalfTurn::ArithmeticFailed(cross, dot) => {
            return Classification::Decided(order_evidence(
                BezierAlgebraicTangentOrderStatus::ArithmeticFailed,
                None,
                Some(cross),
                None,
                dot,
                Some("could not construct first candidate half-turn scalar".to_owned()),
            ));
        }
    };
    let (second_half, base_second_cross) = match turn_half(base, second, policy, retain_scalar) {
        AlgebraicHalfTurn::Half(half, cross) => (half, cross),
        AlgebraicHalfTurn::ZeroTangent(cross, dot) => {
            return Classification::Decided(order_evidence(
                BezierAlgebraicTangentOrderStatus::ZeroTangent,
                None,
                Some(base_first_cross),
                Some(cross),
                Some(dot),
                Some("second tangent has zero direction relative to base".to_owned()),
            ));
        }
        AlgebraicHalfTurn::Undecided(cross, dot) => {
            return Classification::Decided(order_evidence(
                BezierAlgebraicTangentOrderStatus::SignUndecided,
                None,
                Some(base_first_cross),
                Some(cross),
                dot,
                Some("could not certify second candidate half-turn".to_owned()),
            ));
        }
        AlgebraicHalfTurn::ArithmeticFailed(cross, dot) => {
            return Classification::Decided(order_evidence(
                BezierAlgebraicTangentOrderStatus::ArithmeticFailed,
                None,
                Some(base_first_cross),
                Some(cross),
                dot,
                Some("could not construct second candidate half-turn scalar".to_owned()),
            ));
        }
    };

    if first_half != second_half {
        return Classification::Decided(order_evidence(
            BezierAlgebraicTangentOrderStatus::Ordered,
            Some(if first_half < second_half {
                BezierTangentTurnOrdering2::FirstBeforeSecond
            } else {
                BezierTangentTurnOrdering2::SecondBeforeFirst
            }),
            Some(base_first_cross),
            Some(base_second_cross),
            None,
            None,
        ));
    }

    let first_second_cross = cross_sign(first, second, policy, retain_scalar);
    match sign_status(&first_second_cross) {
        ScalarSignStatus::Positive => Classification::Decided(order_evidence(
            BezierAlgebraicTangentOrderStatus::Ordered,
            Some(if reverse_within_half {
                BezierTangentTurnOrdering2::SecondBeforeFirst
            } else {
                BezierTangentTurnOrdering2::FirstBeforeSecond
            }),
            Some(base_first_cross),
            Some(base_second_cross),
            Some(first_second_cross),
            None,
        )),
        ScalarSignStatus::Negative => Classification::Decided(order_evidence(
            BezierAlgebraicTangentOrderStatus::Ordered,
            Some(if reverse_within_half {
                BezierTangentTurnOrdering2::FirstBeforeSecond
            } else {
                BezierTangentTurnOrdering2::SecondBeforeFirst
            }),
            Some(base_first_cross),
            Some(base_second_cross),
            Some(first_second_cross),
            None,
        )),
        ScalarSignStatus::Zero => Classification::Decided(order_evidence(
            BezierAlgebraicTangentOrderStatus::SameDirection,
            None,
            Some(base_first_cross),
            Some(base_second_cross),
            Some(first_second_cross),
            Some("candidate tangent directions are collinear with the same half-turn".to_owned()),
        )),
        ScalarSignStatus::Undecided => Classification::Decided(order_evidence(
            BezierAlgebraicTangentOrderStatus::SignUndecided,
            None,
            Some(base_first_cross),
            Some(base_second_cross),
            Some(first_second_cross),
            Some("could not certify candidate tangent order sign".to_owned()),
        )),
        ScalarSignStatus::ArithmeticFailed => Classification::Decided(order_evidence(
            BezierAlgebraicTangentOrderStatus::ArithmeticFailed,
            None,
            Some(base_first_cross),
            Some(base_second_cross),
            Some(first_second_cross),
            Some("could not construct candidate tangent order scalar".to_owned()),
        )),
    }
}

/// Compares same-direction algebraic tangent branches by second-order evidence.
///
/// This is the exact signed-curvature tie breaker used by retained Bezier traversal. Given two candidates already
/// known to have the same first-order direction, it compares the signs of
/// `cross(B'(t), B''(t))`; branches departing on opposite sides are ordered by
/// that sign. When both depart on the same side it compares
/// `cross^2 / |B'|^6` by clearing positive speed denominators. Side witnesses
/// may be signed directly in their selected source. Same-side normalization
/// retains reduced field values when independent scalar roots are unavailable.
/// A common selected field must be certified. The derivatives are with respect to the
/// source parameter, and normalization removes positive parameter-speed factors.
pub(crate) fn compare_algebraic_same_tangent_second_order(
    first_tangent: &BezierAlgebraicTangentVector2,
    first_second_derivative: &BezierAlgebraicTangentVector2,
    second_tangent: &BezierAlgebraicTangentVector2,
    second_second_derivative: &BezierAlgebraicTangentVector2,
    policy: &CurveContext,
) -> Classification<BezierAlgebraicSameTangentOrderEvidence> {
    for tangent in [first_tangent, second_tangent] {
        match tangent_nonzero(tangent, policy) {
            AlgebraicTangentNonzero::Nonzero => {}
            AlgebraicTangentNonzero::Zero(evidence) => {
                return Classification::Decided(same_tangent_evidence(
                    BezierAlgebraicSameTangentOrderStatus::ZeroTangent,
                    None,
                    None,
                    None,
                    None,
                    Some(format!("zero tangent certified by {:?}", evidence.sign)),
                ));
            }
            AlgebraicTangentNonzero::Undecided(evidence) => {
                return Classification::Decided(same_tangent_evidence(
                    BezierAlgebraicSameTangentOrderStatus::SignUndecided,
                    None,
                    None,
                    None,
                    None,
                    evidence.message,
                ));
            }
            AlgebraicTangentNonzero::ArithmeticFailed(evidence) => {
                return Classification::Decided(same_tangent_evidence(
                    BezierAlgebraicSameTangentOrderStatus::ArithmeticFailed,
                    None,
                    None,
                    None,
                    None,
                    evidence.message,
                ));
            }
        }
    }

    let first_cross = cross_sign(first_tangent, first_second_derivative, policy, true);
    let second_cross = cross_sign(second_tangent, second_second_derivative, policy, true);
    match (sign_status(&first_cross), sign_status(&second_cross)) {
        (ScalarSignStatus::ArithmeticFailed, _) | (_, ScalarSignStatus::ArithmeticFailed) => {
            Classification::Decided(same_tangent_evidence(
                BezierAlgebraicSameTangentOrderStatus::ArithmeticFailed,
                None,
                Some(first_cross),
                Some(second_cross),
                None,
                Some("could not construct algebraic curvature cross scalar".to_owned()),
            ))
        }
        (ScalarSignStatus::Undecided, _) | (_, ScalarSignStatus::Undecided) => {
            Classification::Decided(same_tangent_evidence(
                BezierAlgebraicSameTangentOrderStatus::SignUndecided,
                None,
                Some(first_cross),
                Some(second_cross),
                None,
                Some("could not certify algebraic curvature cross sign".to_owned()),
            ))
        }
        (ScalarSignStatus::Zero, ScalarSignStatus::Zero) => {
            Classification::Decided(same_tangent_evidence(
                BezierAlgebraicSameTangentOrderStatus::SameDirection,
                None,
                Some(first_cross),
                Some(second_cross),
                None,
                Some("both algebraic second-order side witnesses vanished".to_owned()),
            ))
        }
        (ScalarSignStatus::Zero, _) | (_, ScalarSignStatus::Zero) => {
            Classification::Decided(same_tangent_evidence(
                BezierAlgebraicSameTangentOrderStatus::SameDirection,
                None,
                Some(first_cross),
                Some(second_cross),
                None,
                Some("one algebraic second-order side witness vanished".to_owned()),
            ))
        }
        (ScalarSignStatus::Positive, ScalarSignStatus::Negative) => {
            Classification::Decided(same_tangent_evidence(
                BezierAlgebraicSameTangentOrderStatus::Ordered,
                Some(BezierTangentTurnOrdering2::FirstBeforeSecond),
                Some(first_cross),
                Some(second_cross),
                None,
                None,
            ))
        }
        (ScalarSignStatus::Negative, ScalarSignStatus::Positive) => {
            Classification::Decided(same_tangent_evidence(
                BezierAlgebraicSameTangentOrderStatus::Ordered,
                Some(BezierTangentTurnOrdering2::SecondBeforeFirst),
                Some(first_cross),
                Some(second_cross),
                None,
                None,
            ))
        }
        (ScalarSignStatus::Positive, ScalarSignStatus::Positive)
        | (ScalarSignStatus::Negative, ScalarSignStatus::Negative) => {
            compare_algebraic_same_side_curvature_magnitude(
                first_tangent,
                first_cross,
                second_tangent,
                second_cross,
                policy,
            )
        }
    }
}

/// Compares same-direction algebraic tangent branches by third-order evidence.
///
/// This is used only after first-order tangents agree and both second-order
/// side witnesses have vanished.  For a cubic Bezier branch the next Taylor
/// witness is `cross(B'(t), B'''(t))`; opposite signs identify the side of
/// departure, and same-side magnitudes are compared as `cross^2 / |B'|^8` by
/// clearing positive speed denominators. Source-side witnesses may be signed
/// in their selected parameter; same-side magnitudes retain reduced selected-field
/// values or represented scalar roots. Only certified signs determine the order.
pub(crate) fn compare_algebraic_same_tangent_third_order(
    first_tangent: &BezierAlgebraicTangentVector2,
    first_third_derivative: &BezierAlgebraicTangentVector2,
    second_tangent: &BezierAlgebraicTangentVector2,
    second_third_derivative: &BezierAlgebraicTangentVector2,
    policy: &CurveContext,
) -> Classification<BezierAlgebraicSameTangentOrderEvidence> {
    for tangent in [first_tangent, second_tangent] {
        match tangent_nonzero(tangent, policy) {
            AlgebraicTangentNonzero::Nonzero => {}
            AlgebraicTangentNonzero::Zero(evidence) => {
                return Classification::Decided(same_tangent_evidence(
                    BezierAlgebraicSameTangentOrderStatus::ZeroTangent,
                    None,
                    None,
                    None,
                    None,
                    Some(format!("zero tangent certified by {:?}", evidence.sign)),
                ));
            }
            AlgebraicTangentNonzero::Undecided(evidence) => {
                return Classification::Decided(same_tangent_evidence(
                    BezierAlgebraicSameTangentOrderStatus::SignUndecided,
                    None,
                    None,
                    None,
                    None,
                    evidence.message,
                ));
            }
            AlgebraicTangentNonzero::ArithmeticFailed(evidence) => {
                return Classification::Decided(same_tangent_evidence(
                    BezierAlgebraicSameTangentOrderStatus::ArithmeticFailed,
                    None,
                    None,
                    None,
                    None,
                    evidence.message,
                ));
            }
        }
    }

    let first_cross = cross_sign(first_tangent, first_third_derivative, policy, true);
    let second_cross = cross_sign(second_tangent, second_third_derivative, policy, true);
    match (sign_status(&first_cross), sign_status(&second_cross)) {
        (ScalarSignStatus::ArithmeticFailed, _) | (_, ScalarSignStatus::ArithmeticFailed) => {
            Classification::Decided(same_tangent_evidence(
                BezierAlgebraicSameTangentOrderStatus::ArithmeticFailed,
                None,
                Some(first_cross),
                Some(second_cross),
                None,
                Some("could not construct algebraic third-order cross scalar".to_owned()),
            ))
        }
        (ScalarSignStatus::Undecided, _) | (_, ScalarSignStatus::Undecided) => {
            Classification::Decided(same_tangent_evidence(
                BezierAlgebraicSameTangentOrderStatus::SignUndecided,
                None,
                Some(first_cross),
                Some(second_cross),
                None,
                Some("could not certify algebraic third-order cross sign".to_owned()),
            ))
        }
        (ScalarSignStatus::Zero, _) | (_, ScalarSignStatus::Zero) => {
            Classification::Decided(same_tangent_evidence(
                BezierAlgebraicSameTangentOrderStatus::SameDirection,
                None,
                Some(first_cross),
                Some(second_cross),
                None,
                Some("an algebraic third-order side witness vanished".to_owned()),
            ))
        }
        (ScalarSignStatus::Positive, ScalarSignStatus::Negative) => {
            Classification::Decided(same_tangent_evidence(
                BezierAlgebraicSameTangentOrderStatus::Ordered,
                Some(BezierTangentTurnOrdering2::FirstBeforeSecond),
                Some(first_cross),
                Some(second_cross),
                None,
                None,
            ))
        }
        (ScalarSignStatus::Negative, ScalarSignStatus::Positive) => {
            Classification::Decided(same_tangent_evidence(
                BezierAlgebraicSameTangentOrderStatus::Ordered,
                Some(BezierTangentTurnOrdering2::SecondBeforeFirst),
                Some(first_cross),
                Some(second_cross),
                None,
                None,
            ))
        }
        (ScalarSignStatus::Positive, ScalarSignStatus::Positive)
        | (ScalarSignStatus::Negative, ScalarSignStatus::Negative) => {
            compare_algebraic_same_side_magnitude(
                first_tangent,
                first_cross,
                second_tangent,
                second_cross,
                4,
                "third-order",
                policy,
            )
        }
    }
}

/// Compares regular branches whose common nonzero signed curvature has
/// already been certified. Each jet contains the first three derivatives.
pub(crate) fn compare_algebraic_equal_curvature_third_order(
    first: [&BezierAlgebraicTangentVector2; 3],
    second: [&BezierAlgebraicTangentVector2; 3],
    policy: &CurveContext,
) -> Classification<BezierAlgebraicSameTangentOrderEvidence> {
    let first_speed = norm_squared_sign(first[0], policy);
    let second_speed = norm_squared_sign(second[0], policy);
    for speed in [&first_speed, &second_speed] {
        let status = match sign_status(speed) {
            ScalarSignStatus::Positive => continue,
            ScalarSignStatus::Zero => BezierAlgebraicSameTangentOrderStatus::ZeroTangent,
            ScalarSignStatus::ArithmeticFailed => {
                BezierAlgebraicSameTangentOrderStatus::ArithmeticFailed
            }
            ScalarSignStatus::Negative | ScalarSignStatus::Undecided => {
                BezierAlgebraicSameTangentOrderStatus::SignUndecided
            }
        };
        return Classification::Decided(same_tangent_evidence(
            status,
            None,
            None,
            None,
            None,
            speed.message.clone(),
        ));
    }
    let first_witness = graph_third_derivative_witness(first, &first_speed, policy);
    let second_witness = graph_third_derivative_witness(second, &second_speed, policy);
    let difference = normalized_graph_third_difference(
        &first_witness,
        &second_witness,
        &first_speed,
        &second_speed,
        policy,
    );
    let (status, ordering) = match sign_status(&difference) {
        ScalarSignStatus::Negative => (
            BezierAlgebraicSameTangentOrderStatus::Ordered,
            Some(BezierTangentTurnOrdering2::FirstBeforeSecond),
        ),
        ScalarSignStatus::Positive => (
            BezierAlgebraicSameTangentOrderStatus::Ordered,
            Some(BezierTangentTurnOrdering2::SecondBeforeFirst),
        ),
        ScalarSignStatus::Zero => (BezierAlgebraicSameTangentOrderStatus::SameDirection, None),
        ScalarSignStatus::Undecided => (BezierAlgebraicSameTangentOrderStatus::SignUndecided, None),
        ScalarSignStatus::ArithmeticFailed => (
            BezierAlgebraicSameTangentOrderStatus::ArithmeticFailed,
            None,
        ),
    };
    let message = difference.message.clone();
    Classification::Decided(same_tangent_evidence(
        status,
        ordering,
        Some(first_witness),
        Some(second_witness),
        Some(difference),
        message,
    ))
}

fn graph_third_derivative_witness(
    [tangent, acceleration, jerk]: [&BezierAlgebraicTangentVector2; 3],
    speed: &BezierAlgebraicScalarSignEvidence,
    policy: &CurveContext,
) -> BezierAlgebraicScalarSignEvidence {
    // If S=v.v, the graph derivative along the common unit tangent is
    // [S*cross(v,j)-3*cross(v,a)*(v.a)]/S^3. The second term removes
    // tangential parameter acceleration; its sign is not a side-of-curve
    // decision when the already-equal curvature is nonzero.
    let jerk_cross = cross_sign(tangent, jerk, policy, true);
    let acceleration_cross = cross_sign(tangent, acceleration, policy, true);
    let acceleration_dot = dot_sign(tangent, acceleration, policy, true);
    if let Some(evidence) = selected_scalar_calculation(
        &[speed, &jerk_cross, &acceleration_cross, &acceleration_dot],
        |field, values| {
            let normal = field.multiply(&values[0], &values[1])?;
            let along = field.multiply(&values[2], &values[3])?;
            let correction = field.multiply(&field.constant(Real::from(3)), &along)?;
            field.subtract(&normal, &correction)
        },
    ) {
        return evidence;
    }
    let normal = binary_from_evidence_values(
        speed.represented_scalar(),
        None,
        jerk_cross.represented_scalar(),
        None,
        AlgebraicRootArithmeticOp::Multiply,
        policy,
    );
    let along = binary_from_evidence_values(
        acceleration_cross.represented_scalar(),
        None,
        acceleration_dot.represented_scalar(),
        None,
        AlgebraicRootArithmeticOp::Multiply,
        policy,
    );
    let correction = binary_from_evidence_values(
        along.result_representation.as_ref(),
        along.exact_result.as_ref(),
        None,
        Some(&Real::from(3)),
        AlgebraicRootArithmeticOp::Multiply,
        policy,
    );
    let difference = subtract(
        normal.result_representation.as_ref(),
        normal.exact_result.as_ref(),
        correction.result_representation.as_ref(),
        correction.exact_result.as_ref(),
        policy,
    );
    let mut arithmetic = jerk_cross.arithmetic;
    arithmetic.extend(acceleration_cross.arithmetic);
    arithmetic.extend(acceleration_dot.arithmetic);
    arithmetic.extend([normal, along, correction, difference]);
    scalar_sign_evidence(arithmetic, policy)
}

fn normalized_graph_third_difference(
    first: &BezierAlgebraicScalarSignEvidence,
    second: &BezierAlgebraicScalarSignEvidence,
    first_speed: &BezierAlgebraicScalarSignEvidence,
    second_speed: &BezierAlgebraicScalarSignEvidence,
    policy: &CurveContext,
) -> BezierAlgebraicScalarSignEvidence {
    if let Some(evidence) = selected_scalar_calculation(
        &[first, second, first_speed, second_speed],
        |field, values| {
            let first_scaled = field.multiply(&values[0], &field.pow(&values[3], 3)?)?;
            let second_scaled = field.multiply(&values[1], &field.pow(&values[2], 3)?)?;
            field.subtract(&first_scaled, &second_scaled)
        },
    ) {
        return evidence;
    }
    let (Some(first), Some(second), Some(first_speed), Some(second_speed)) = (
        first.represented_scalar(),
        second.represented_scalar(),
        first_speed.represented_scalar(),
        second_speed.represented_scalar(),
    ) else {
        return scalar_sign_evidence(
            vec![missing_operand_evidence(
                AlgebraicRootArithmeticOp::Multiply,
                "normalized graph derivative requires represented witnesses and speeds",
            )],
            policy,
        );
    };
    let first_power = power_representation(first_speed, 3, policy);
    let second_power = power_representation(second_speed, 3, policy);
    let first_scaled = binary_from_evidence_values(
        Some(first),
        None,
        second_power.representation.as_ref(),
        second_power.exact.as_ref(),
        AlgebraicRootArithmeticOp::Multiply,
        policy,
    );
    let second_scaled = binary_from_evidence_values(
        Some(second),
        None,
        first_power.representation.as_ref(),
        first_power.exact.as_ref(),
        AlgebraicRootArithmeticOp::Multiply,
        policy,
    );
    let difference = subtract(
        first_scaled.result_representation.as_ref(),
        first_scaled.exact_result.as_ref(),
        second_scaled.result_representation.as_ref(),
        second_scaled.exact_result.as_ref(),
        policy,
    );
    let mut arithmetic = first_power.arithmetic;
    arithmetic.extend(second_power.arithmetic);
    arithmetic.extend([first_scaled, second_scaled, difference]);
    scalar_sign_evidence(arithmetic, policy)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ScalarSignStatus {
    Positive,
    Negative,
    Zero,
    Undecided,
    ArithmeticFailed,
}

enum AlgebraicTangentNonzero {
    Nonzero,
    Zero(BezierAlgebraicScalarSignEvidence),
    Undecided(BezierAlgebraicScalarSignEvidence),
    ArithmeticFailed(BezierAlgebraicScalarSignEvidence),
}

enum AlgebraicHalfTurn {
    Half(u8, BezierAlgebraicScalarSignEvidence),
    ZeroTangent(
        BezierAlgebraicScalarSignEvidence,
        BezierAlgebraicScalarSignEvidence,
    ),
    Undecided(
        BezierAlgebraicScalarSignEvidence,
        Option<BezierAlgebraicScalarSignEvidence>,
    ),
    ArithmeticFailed(
        BezierAlgebraicScalarSignEvidence,
        Option<BezierAlgebraicScalarSignEvidence>,
    ),
}

fn tangent_nonzero(
    tangent: &BezierAlgebraicTangentVector2,
    policy: &CurveContext,
) -> AlgebraicTangentNonzero {
    if tangent.represented_coordinates().is_some_and(|(x, y)| {
        [x, y].into_iter().any(|coordinate| {
            representation_sign(coordinate, policy).is_some_and(|sign| sign != Ordering::Equal)
        })
    }) {
        return AlgebraicTangentNonzero::Nonzero;
    }
    let norm = norm_squared_sign(tangent, policy);
    match sign_status(&norm) {
        ScalarSignStatus::Positive => AlgebraicTangentNonzero::Nonzero,
        ScalarSignStatus::Zero => AlgebraicTangentNonzero::Zero(norm),
        ScalarSignStatus::Negative | ScalarSignStatus::Undecided => {
            AlgebraicTangentNonzero::Undecided(norm)
        }
        ScalarSignStatus::ArithmeticFailed => AlgebraicTangentNonzero::ArithmeticFailed(norm),
    }
}

fn turn_half(
    base: &BezierAlgebraicTangentVector2,
    candidate: &BezierAlgebraicTangentVector2,
    policy: &CurveContext,
    retain_scalar: bool,
) -> AlgebraicHalfTurn {
    let cross = cross_sign(base, candidate, policy, retain_scalar);
    match sign_status(&cross) {
        ScalarSignStatus::Positive => AlgebraicHalfTurn::Half(0, cross),
        ScalarSignStatus::Negative => AlgebraicHalfTurn::Half(1, cross),
        ScalarSignStatus::Zero => {
            let dot = dot_sign(base, candidate, policy, retain_scalar);
            match sign_status(&dot) {
                ScalarSignStatus::Positive => AlgebraicHalfTurn::Half(0, cross),
                ScalarSignStatus::Negative => AlgebraicHalfTurn::Half(1, cross),
                ScalarSignStatus::Zero => AlgebraicHalfTurn::ZeroTangent(cross, dot),
                ScalarSignStatus::Undecided => AlgebraicHalfTurn::Undecided(cross, Some(dot)),
                ScalarSignStatus::ArithmeticFailed => {
                    AlgebraicHalfTurn::ArithmeticFailed(cross, Some(dot))
                }
            }
        }
        ScalarSignStatus::Undecided => AlgebraicHalfTurn::Undecided(cross, None),
        ScalarSignStatus::ArithmeticFailed => AlgebraicHalfTurn::ArithmeticFailed(cross, None),
    }
}

fn cross_sign(
    left: &BezierAlgebraicTangentVector2,
    right: &BezierAlgebraicTangentVector2,
    policy: &CurveContext,
    retain_scalar: bool,
) -> BezierAlgebraicScalarSignEvidence {
    let (Some((left_x, left_y)), Some((right_x, right_y))) = (
        left.represented_coordinates(),
        right.represented_coordinates(),
    ) else {
        return source_bilinear_evidence(left, right, false, policy);
    };
    if !retain_scalar
        && let Some(sign) = interval_bilinear_sign(left_x, right_y, left_y, right_x, false, policy)
    {
        return interval_scalar_sign_evidence(
            sign,
            "cross-product sign certified from rational root enclosures",
        );
    }
    let left_x_right_y = multiply(left_x, right_y, policy);
    let left_y_right_x = multiply(left_y, right_x, policy);
    let scalar = subtract(
        left_x_right_y.result_representation.as_ref(),
        left_x_right_y.exact_result.as_ref(),
        left_y_right_x.result_representation.as_ref(),
        left_y_right_x.exact_result.as_ref(),
        policy,
    );
    let mut evidence = scalar_sign_evidence(vec![left_x_right_y, left_y_right_x, scalar], policy);
    if evidence.sign.is_none() {
        evidence.sign = interval_bilinear_sign(left_x, right_y, left_y, right_x, false, policy);
        if evidence.sign.is_some() {
            evidence.message =
                Some("cross-product sign certified from rational root enclosures".to_owned());
        }
    }
    evidence
}

fn dot_sign(
    left: &BezierAlgebraicTangentVector2,
    right: &BezierAlgebraicTangentVector2,
    policy: &CurveContext,
    retain_scalar: bool,
) -> BezierAlgebraicScalarSignEvidence {
    let (Some((left_x, left_y)), Some((right_x, right_y))) = (
        left.represented_coordinates(),
        right.represented_coordinates(),
    ) else {
        return source_bilinear_evidence(left, right, true, policy);
    };
    if !retain_scalar
        && let Some(sign) = interval_bilinear_sign(left_x, right_x, left_y, right_y, true, policy)
    {
        return interval_scalar_sign_evidence(
            sign,
            "dot-product sign certified from rational root enclosures",
        );
    }
    let left_x_right_x = multiply(left_x, right_x, policy);
    let left_y_right_y = multiply(left_y, right_y, policy);
    let scalar = add(
        left_x_right_x.result_representation.as_ref(),
        left_x_right_x.exact_result.as_ref(),
        left_y_right_y.result_representation.as_ref(),
        left_y_right_y.exact_result.as_ref(),
        policy,
    );
    let mut evidence = scalar_sign_evidence(vec![left_x_right_x, left_y_right_y, scalar], policy);
    if evidence.sign.is_none() {
        evidence.sign = interval_bilinear_sign(left_x, right_x, left_y, right_y, true, policy);
        if evidence.sign.is_some() {
            evidence.message =
                Some("dot-product sign certified from rational root enclosures".to_owned());
        }
    }
    evidence
}

fn norm_squared_sign(
    vector: &BezierAlgebraicTangentVector2,
    policy: &CurveContext,
) -> BezierAlgebraicScalarSignEvidence {
    let Some((x, y)) = vector.represented_coordinates() else {
        return source_bilinear_evidence(vector, vector, true, policy);
    };
    let dx_squared = multiply(x, x, policy);
    let dy_squared = multiply(y, y, policy);
    let scalar = add(
        dx_squared.result_representation.as_ref(),
        dx_squared.exact_result.as_ref(),
        dy_squared.result_representation.as_ref(),
        dy_squared.exact_result.as_ref(),
        policy,
    );
    let mut evidence = scalar_sign_evidence(vec![dx_squared, dy_squared, scalar], policy);
    if evidence.sign.is_none() {
        evidence.sign = interval_bilinear_sign(x, x, y, y, true, policy);
        if evidence.sign.is_some() {
            evidence.message =
                Some("squared-norm sign certified from rational root enclosures".to_owned());
        }
    }
    evidence
}

fn source_bilinear_sign(
    first: &BezierAlgebraicTangentVector2,
    second: &BezierAlgebraicTangentVector2,
    dot: bool,
    policy: &CurveContext,
) -> crate::CurveResult<Option<Classification<RealSign>>> {
    if let (Some(first), Some(second)) = (first.image(), second.image())
        && let Some(sign) = first.shared_parameter_bilinear_sign(second, dot, policy)?
    {
        return Ok(Some(sign));
    }
    let exact = |vector: &BezierAlgebraicTangentVector2| {
        vector.represented_coordinates().and_then(|(x, y)| {
            Some((
                strict_exact_witness(x)?.clone(),
                strict_exact_witness(y)?.clone(),
            ))
        })
    };
    if let (Some(image), Some((x, y))) = (first.image(), exact(second)) {
        return if dot {
            image.constant_linear_combination_sign(&x, &y, policy)
        } else {
            image.constant_linear_combination_sign(&y, &-x, policy)
        };
    }
    if let (Some((x, y)), Some(image)) = (exact(first), second.image()) {
        return if dot {
            image.constant_linear_combination_sign(&x, &y, policy)
        } else {
            image.constant_linear_combination_sign(&-y, &x, policy)
        };
    }
    Ok(None)
}

fn source_bilinear_evidence(
    first: &BezierAlgebraicTangentVector2,
    second: &BezierAlgebraicTangentVector2,
    dot: bool,
    policy: &CurveContext,
) -> BezierAlgebraicScalarSignEvidence {
    let (sign, message) = match source_bilinear_sign(first, second, dot, policy) {
        Ok(Some(Classification::Decided(sign))) => (
            Some(match sign {
                RealSign::Negative => Ordering::Less,
                RealSign::Zero => Ordering::Equal,
                RealSign::Positive => Ordering::Greater,
            }),
            None,
        ),
        Ok(Some(Classification::Uncertain(reason))) => (
            None,
            Some(format!("selected-source sign is unresolved: {reason:?}")),
        ),
        Ok(None) => (
            None,
            Some("no certified common selected parameter or exact constant operand".to_owned()),
        ),
        Err(error) => (
            None,
            Some(format!("selected-source sign construction failed: {error}")),
        ),
    };
    let evidence = BezierAlgebraicScalarSignEvidence {
        arithmetic: Vec::new(),
        value: Some(TangentScalar::Bilinear(Box::new(RetainedTangentBilinear {
            first: first.clone(),
            second: second.clone(),
            dot,
        }))),
        sign,
        message,
    };
    if evidence.sign.is_none()
        && let Some(transported) =
            selected_scalar_calculation(&[&evidence], |_, values| Ok(values[0].clone()))
        && transported.value.is_some()
    {
        return transported;
    }
    evidence
}

fn selected_scalar_calculation(
    inputs: &[&BezierAlgebraicScalarSignEvidence],
    calculate: impl FnOnce(
        &mut AlgebraicField,
        &[AlgebraicFieldValue],
    ) -> Result<AlgebraicFieldValue, AlgebraicFieldError>,
) -> Option<BezierAlgebraicScalarSignEvidence> {
    let context = inputs
        .iter()
        .find_map(|input| {
            input
                .selected_scalar()
                .filter(|value| value.exact_value().is_none())
                .map(|value| Ok(AlgebraicField::from_value(value)))
        })
        .or_else(|| {
            inputs.iter().find_map(|input| {
                let TangentScalar::Bilinear(witness) = input.value.as_ref()? else {
                    return None;
                };
                [&witness.first, &witness.second]
                    .into_iter()
                    .find(|vector| vector.represented_coordinates().is_none())
                    .and_then(BezierAlgebraicTangentVector2::image)
                    .map(|image| AlgebraicField::new(image.parameter()))
            })
        })
        .or_else(|| {
            inputs.iter().find_map(|input| {
                input
                    .selected_scalar()
                    .map(|value| Ok(AlgebraicField::from_value(value)))
            })
        })?;
    let result = (|| {
        let mut field = context?;
        let values = inputs
            .iter()
            .map(|input| scalar_in_field(input, &mut field))
            .collect::<Result<Vec<_>, _>>()?;
        let value = calculate(&mut field, &values)?;
        let sign = field.sign(&value);
        let value = field.finish(value)?;
        Ok::<_, AlgebraicFieldError>((value, sign))
    })();
    Some(match result {
        Ok((value, sign)) => BezierAlgebraicScalarSignEvidence {
            arithmetic: Vec::new(),
            value: Some(TangentScalar::Selected(value)),
            sign: sign.as_ref().ok().copied(),
            message: sign
                .err()
                .map(|error| format!("selected-field sign is unresolved: {error:?}")),
        },
        Err(error) => BezierAlgebraicScalarSignEvidence {
            arithmetic: Vec::new(),
            value: None,
            sign: None,
            message: Some(format!(
                "selected-field scalar construction failed: {error:?}"
            )),
        },
    })
}

fn scalar_in_field(
    input: &BezierAlgebraicScalarSignEvidence,
    field: &mut AlgebraicField,
) -> Result<AlgebraicFieldValue, AlgebraicFieldError> {
    match input.value.as_ref().ok_or(AlgebraicFieldError::Undecided)? {
        TangentScalar::Selected(value) => Ok(value.clone()),
        TangentScalar::Represented(root) => represented_scalar_in_field(root, field),
        TangentScalar::Bilinear(witness) => {
            let [first_x, first_y] = vector_in_field(&witness.first, field)?;
            let [second_x, second_y] = vector_in_field(&witness.second, field)?;
            if witness.dot {
                field.add(
                    &field.multiply(&first_x, &second_x)?,
                    &field.multiply(&first_y, &second_y)?,
                )
            } else {
                field.subtract(
                    &field.multiply(&first_x, &second_y)?,
                    &field.multiply(&first_y, &second_x)?,
                )
            }
        }
    }
}

fn strict_exact_witness(root: &AlgebraicRootRepresentation) -> Option<&Real> {
    let value = root.exact_point_witness()?;
    (root.is_valid()
        && hypersolve::validate_algebraic_root_representation(
            root,
            hyperlimit::PredicatePolicy::STRICT,
        )
        .status
            == hypersolve::AlgebraicRootValidationStatus::Valid)
        .then_some(value)
}

fn represented_scalar_in_field(
    root: &AlgebraicRootRepresentation,
    field: &mut AlgebraicField,
) -> Result<AlgebraicFieldValue, AlgebraicFieldError> {
    if let Some(value) = strict_exact_witness(root) {
        return Ok(field.constant(value.clone()));
    }
    // This root can be the selected parameter itself. An unrelated algebraic
    // coordinate requires an extension field, so the identity guard must hold.
    field.rational_function(root, vec![Real::zero(), Real::one()], vec![Real::one()])
}

fn vector_in_field(
    vector: &BezierAlgebraicTangentVector2,
    field: &mut AlgebraicField,
) -> Result<[AlgebraicFieldValue; 2], AlgebraicFieldError> {
    if let Some((x, y)) = vector.represented_coordinates()
        && let (Some(x), Some(y)) = (strict_exact_witness(x), strict_exact_witness(y))
    {
        return Ok([field.constant(x.clone()), field.constant(y.clone())]);
    }
    if let Some(image) = vector.image() {
        let (x, y, denominator) = image.coordinate_polynomials();
        return Ok([
            field.rational_function(image.parameter(), x.to_vec(), denominator.to_vec())?,
            field.rational_function(image.parameter(), y.to_vec(), denominator.to_vec())?,
        ]);
    }
    let (x, y) = vector
        .represented_coordinates()
        .ok_or(AlgebraicFieldError::Undecided)?;
    Ok([
        represented_scalar_in_field(x, field)?,
        represented_scalar_in_field(y, field)?,
    ])
}

fn interval_scalar_sign_evidence(
    sign: Ordering,
    message: &'static str,
) -> BezierAlgebraicScalarSignEvidence {
    BezierAlgebraicScalarSignEvidence {
        arithmetic: Vec::new(),
        value: None,
        sign: Some(sign),
        message: Some(message.to_owned()),
    }
}

fn representation_sign(
    representation: &AlgebraicRootRepresentation,
    policy: &CurveContext,
) -> Option<Ordering> {
    if !representation.is_valid() {
        return None;
    }
    interval_sign(
        &representation.interval.lower,
        &representation.interval.upper,
        policy,
    )
}

fn interval_bilinear_sign(
    first_left: &AlgebraicRootRepresentation,
    first_right: &AlgebraicRootRepresentation,
    second_left: &AlgebraicRootRepresentation,
    second_right: &AlgebraicRootRepresentation,
    add_products: bool,
    policy: &CurveContext,
) -> Option<Ordering> {
    if [first_left, first_right, second_left, second_right]
        .iter()
        .any(|representation| !representation.is_valid())
    {
        return None;
    }
    let first = interval_product(first_left, first_right, policy)?;
    let second = interval_product(second_left, second_right, policy)?;
    let interval = if add_products {
        (&first.0 + &second.0, &first.1 + &second.1)
    } else {
        (&first.0 - &second.1, &first.1 - &second.0)
    };
    interval_sign(&interval.0, &interval.1, policy)
}

fn interval_product(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: &CurveContext,
) -> Option<(Real, Real)> {
    let products = [
        &left.interval.lower * &right.interval.lower,
        &left.interval.lower * &right.interval.upper,
        &left.interval.upper * &right.interval.lower,
        &left.interval.upper * &right.interval.upper,
    ];
    let mut lower = products[0].clone();
    let mut upper = products[0].clone();
    for product in &products[1..] {
        if compare_reals(product, &lower, policy)? == Ordering::Less {
            lower = product.clone();
        }
        if compare_reals(product, &upper, policy)? == Ordering::Greater {
            upper = product.clone();
        }
    }
    Some((lower, upper))
}

fn interval_sign(lower: &Real, upper: &Real, policy: &CurveContext) -> Option<Ordering> {
    let lower_sign = compare_reals(lower, &Real::zero(), policy)?;
    let upper_sign = compare_reals(upper, &Real::zero(), policy)?;
    if lower_sign == Ordering::Greater {
        Some(Ordering::Greater)
    } else if upper_sign == Ordering::Less {
        Some(Ordering::Less)
    } else if lower_sign == Ordering::Equal && upper_sign == Ordering::Equal {
        Some(Ordering::Equal)
    } else {
        None
    }
}

fn compare_algebraic_same_side_curvature_magnitude(
    first_tangent: &BezierAlgebraicTangentVector2,
    first_cross: BezierAlgebraicScalarSignEvidence,
    second_tangent: &BezierAlgebraicTangentVector2,
    second_cross: BezierAlgebraicScalarSignEvidence,
    policy: &CurveContext,
) -> Classification<BezierAlgebraicSameTangentOrderEvidence> {
    compare_algebraic_same_side_magnitude(
        first_tangent,
        first_cross,
        second_tangent,
        second_cross,
        3,
        "curvature",
        policy,
    )
}

fn compare_algebraic_same_side_magnitude(
    first_tangent: &BezierAlgebraicTangentVector2,
    first_cross: BezierAlgebraicScalarSignEvidence,
    second_tangent: &BezierAlgebraicTangentVector2,
    second_cross: BezierAlgebraicScalarSignEvidence,
    speed_power: usize,
    witness_name: &str,
    policy: &CurveContext,
) -> Classification<BezierAlgebraicSameTangentOrderEvidence> {
    let first_speed = norm_squared_sign(first_tangent, policy);
    let second_speed = norm_squared_sign(second_tangent, policy);
    if !matches!(sign_status(&first_speed), ScalarSignStatus::Positive)
        || !matches!(sign_status(&second_speed), ScalarSignStatus::Positive)
    {
        return Classification::Decided(same_tangent_evidence(
            BezierAlgebraicSameTangentOrderStatus::SignUndecided,
            None,
            Some(first_cross),
            Some(second_cross),
            None,
            Some("could not certify positive tangent speeds".to_owned()),
        ));
    }

    // Reflection reverses the order of two branches on the negative side.
    // The squared magnitudes alone cannot retain that orientation.
    let negative_side = sign_status(&first_cross) == ScalarSignStatus::Negative;
    let magnitude = same_side_magnitude_difference(
        &first_cross,
        &second_cross,
        &first_speed,
        &second_speed,
        speed_power,
        policy,
    );
    match sign_status(&magnitude) {
        ScalarSignStatus::Negative => Classification::Decided(same_tangent_evidence(
            BezierAlgebraicSameTangentOrderStatus::Ordered,
            Some(if negative_side {
                BezierTangentTurnOrdering2::SecondBeforeFirst
            } else {
                BezierTangentTurnOrdering2::FirstBeforeSecond
            }),
            Some(first_cross),
            Some(second_cross),
            Some(magnitude),
            None,
        )),
        ScalarSignStatus::Positive => Classification::Decided(same_tangent_evidence(
            BezierAlgebraicSameTangentOrderStatus::Ordered,
            Some(if negative_side {
                BezierTangentTurnOrdering2::FirstBeforeSecond
            } else {
                BezierTangentTurnOrdering2::SecondBeforeFirst
            }),
            Some(first_cross),
            Some(second_cross),
            Some(magnitude),
            None,
        )),
        ScalarSignStatus::Zero => Classification::Decided(same_tangent_evidence(
            BezierAlgebraicSameTangentOrderStatus::SameDirection,
            None,
            Some(first_cross),
            Some(second_cross),
            Some(magnitude),
            Some(format!(
                "same-side algebraic {witness_name} magnitudes are equal"
            )),
        )),
        ScalarSignStatus::Undecided => Classification::Decided(same_tangent_evidence(
            BezierAlgebraicSameTangentOrderStatus::SignUndecided,
            None,
            Some(first_cross),
            Some(second_cross),
            Some(magnitude),
            Some(format!(
                "could not certify same-side algebraic {witness_name} magnitude"
            )),
        )),
        ScalarSignStatus::ArithmeticFailed => Classification::Decided(same_tangent_evidence(
            BezierAlgebraicSameTangentOrderStatus::ArithmeticFailed,
            None,
            Some(first_cross),
            Some(second_cross),
            Some(magnitude),
            Some(format!(
                "could not construct same-side algebraic {witness_name} magnitude"
            )),
        )),
    }
}

fn same_side_magnitude_difference(
    first_cross: &BezierAlgebraicScalarSignEvidence,
    second_cross: &BezierAlgebraicScalarSignEvidence,
    first_speed: &BezierAlgebraicScalarSignEvidence,
    second_speed: &BezierAlgebraicScalarSignEvidence,
    speed_power: usize,
    policy: &CurveContext,
) -> BezierAlgebraicScalarSignEvidence {
    if let Some(evidence) = selected_scalar_calculation(
        &[first_cross, second_cross, first_speed, second_speed],
        |field, values| {
            let first_scaled = field.multiply(
                &field.pow(&values[0], 2)?,
                &field.pow(&values[3], speed_power)?,
            )?;
            let second_scaled = field.multiply(
                &field.pow(&values[1], 2)?,
                &field.pow(&values[2], speed_power)?,
            )?;
            field.subtract(&first_scaled, &second_scaled)
        },
    ) {
        return evidence;
    }
    let Some(first_cross_scalar) = first_cross.represented_scalar() else {
        return scalar_sign_evidence(
            vec![missing_operand_evidence(
                AlgebraicRootArithmeticOp::Multiply,
                "first curvature cross scalar was absent",
            )],
            policy,
        );
    };
    let Some(second_cross_scalar) = second_cross.represented_scalar() else {
        return scalar_sign_evidence(
            vec![missing_operand_evidence(
                AlgebraicRootArithmeticOp::Multiply,
                "second curvature cross scalar was absent",
            )],
            policy,
        );
    };
    let Some(first_speed_scalar) = first_speed.represented_scalar() else {
        return scalar_sign_evidence(
            vec![missing_operand_evidence(
                AlgebraicRootArithmeticOp::Multiply,
                "first speed scalar was absent",
            )],
            policy,
        );
    };
    let Some(second_speed_scalar) = second_speed.represented_scalar() else {
        return scalar_sign_evidence(
            vec![missing_operand_evidence(
                AlgebraicRootArithmeticOp::Multiply,
                "second speed scalar was absent",
            )],
            policy,
        );
    };

    let first_cross_squared = multiply(first_cross_scalar, first_cross_scalar, policy);
    let second_cross_squared = multiply(second_cross_scalar, second_cross_scalar, policy);
    let first_speed_power = power_representation(first_speed_scalar, speed_power, policy);
    let second_speed_power = power_representation(second_speed_scalar, speed_power, policy);
    let first_scaled = multiply_evidence_results(&first_cross_squared, &second_speed_power, policy);
    let second_scaled =
        multiply_evidence_results(&second_cross_squared, &first_speed_power, policy);
    let difference = subtract(
        first_scaled.result_representation.as_ref(),
        first_scaled.exact_result.as_ref(),
        second_scaled.result_representation.as_ref(),
        second_scaled.exact_result.as_ref(),
        policy,
    );

    let mut arithmetic = Vec::new();
    arithmetic.push(first_cross_squared);
    arithmetic.push(second_cross_squared);
    arithmetic.extend(first_speed_power.arithmetic);
    arithmetic.extend(second_speed_power.arithmetic);
    arithmetic.push(first_scaled);
    arithmetic.push(second_scaled);
    arithmetic.push(difference);
    scalar_sign_evidence(arithmetic, policy)
}

struct AlgebraicPowerEvidence {
    arithmetic: Vec<AlgebraicRootArithmeticReport>,
    representation: Option<AlgebraicRootRepresentation>,
    exact: Option<Real>,
}

fn power_representation(
    value: &AlgebraicRootRepresentation,
    power: usize,
    policy: &CurveContext,
) -> AlgebraicPowerEvidence {
    assert!(power >= 1, "algebraic power must be positive");
    let mut arithmetic = Vec::new();
    let mut representation = Some(value.clone());
    let mut exact = None;
    for _ in 1..power {
        let product = binary_from_evidence_values(
            representation.as_ref(),
            exact.as_ref(),
            Some(value),
            None,
            AlgebraicRootArithmeticOp::Multiply,
            policy,
        );
        representation = product.result_representation.clone();
        exact = product.exact_result.clone();
        arithmetic.push(product);
    }
    AlgebraicPowerEvidence {
        arithmetic,
        representation,
        exact,
    }
}

fn multiply_evidence_results(
    left: &AlgebraicRootArithmeticReport,
    right: &AlgebraicPowerEvidence,
    policy: &CurveContext,
) -> AlgebraicRootArithmeticReport {
    binary_from_evidence_values(
        left.result_representation.as_ref(),
        left.exact_result.as_ref(),
        right.representation.as_ref(),
        right.exact.as_ref(),
        AlgebraicRootArithmeticOp::Multiply,
        policy,
    )
}

pub(crate) fn negate_algebraic_root(
    value: &AlgebraicRootRepresentation,
    policy: &CurveContext,
) -> Option<AlgebraicRootRepresentation> {
    let evidence = crate::bezier_algebraic_image::arithmetic_algebraic_representations_with_policy(
        value,
        None,
        AlgebraicRootArithmeticOp::Negate,
        policy,
    );
    if !crate::bezier_algebraic_image::algebraic_arithmetic_succeeded(&evidence.status) {
        return None;
    }
    if let Some(result) = evidence.result_representation {
        return Some(result);
    }
    evidence
        .exact_result
        .map(|value| AlgebraicRootRepresentation::from_exact_value(&value))
}

fn multiply(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: &CurveContext,
) -> AlgebraicRootArithmeticReport {
    crate::bezier_algebraic_image::arithmetic_algebraic_representations_with_policy(
        left,
        Some(right),
        AlgebraicRootArithmeticOp::Multiply,
        policy,
    )
}

fn add(
    left_representation: Option<&AlgebraicRootRepresentation>,
    left_exact: Option<&Real>,
    right_representation: Option<&AlgebraicRootRepresentation>,
    right_exact: Option<&Real>,
    policy: &CurveContext,
) -> AlgebraicRootArithmeticReport {
    binary_from_evidence_values(
        left_representation,
        left_exact,
        right_representation,
        right_exact,
        AlgebraicRootArithmeticOp::Add,
        policy,
    )
}

fn subtract(
    left_representation: Option<&AlgebraicRootRepresentation>,
    left_exact: Option<&Real>,
    right_representation: Option<&AlgebraicRootRepresentation>,
    right_exact: Option<&Real>,
    policy: &CurveContext,
) -> AlgebraicRootArithmeticReport {
    binary_from_evidence_values(
        left_representation,
        left_exact,
        right_representation,
        right_exact,
        AlgebraicRootArithmeticOp::Subtract,
        policy,
    )
}

fn binary_from_evidence_values(
    left_representation: Option<&AlgebraicRootRepresentation>,
    left_exact: Option<&Real>,
    right_representation: Option<&AlgebraicRootRepresentation>,
    right_exact: Option<&Real>,
    op: AlgebraicRootArithmeticOp,
    policy: &CurveContext,
) -> AlgebraicRootArithmeticReport {
    let left = match representation_or_exact(left_representation, left_exact) {
        Some(value) => value,
        None => return missing_operand_evidence(op, "left arithmetic operand was absent"),
    };
    let right = match representation_or_exact(right_representation, right_exact) {
        Some(value) => value,
        None => return missing_operand_evidence(op, "right arithmetic operand was absent"),
    };
    crate::bezier_algebraic_image::arithmetic_algebraic_representations_with_policy(
        &left,
        Some(&right),
        op,
        policy,
    )
}

fn representation_or_exact(
    representation: Option<&AlgebraicRootRepresentation>,
    exact: Option<&Real>,
) -> Option<AlgebraicRootRepresentation> {
    representation
        .cloned()
        .or_else(|| exact.map(AlgebraicRootRepresentation::from_exact_value))
}

fn missing_operand_evidence(
    operation: AlgebraicRootArithmeticOp,
    message: impl Into<String>,
) -> AlgebraicRootArithmeticReport {
    AlgebraicRootArithmeticReport {
        operation,
        status: AlgebraicRootArithmeticStatus::InvalidEvidence,
        exact_result: None,
        result_representation: None,
        message: Some(message.into()),
    }
}

fn scalar_sign_evidence(
    arithmetic: Vec<AlgebraicRootArithmeticReport>,
    policy: &CurveContext,
) -> BezierAlgebraicScalarSignEvidence {
    let Some(last) = arithmetic.last() else {
        return BezierAlgebraicScalarSignEvidence {
            arithmetic,
            value: None,
            sign: None,
            message: Some("scalar construction produced no arithmetic evidence".to_owned()),
        };
    };
    if !crate::bezier_algebraic_image::algebraic_arithmetic_succeeded(&last.status) {
        return BezierAlgebraicScalarSignEvidence {
            message: last.message.clone(),
            arithmetic,
            value: None,
            sign: None,
        };
    }
    let scalar = match representation_or_exact(
        last.result_representation.as_ref(),
        last.exact_result.as_ref(),
    ) {
        Some(scalar) => scalar,
        None => {
            return BezierAlgebraicScalarSignEvidence {
                arithmetic,
                value: None,
                sign: None,
                message: Some("scalar arithmetic omitted represented result".to_owned()),
            };
        }
    };
    let sign = represented_sign(&scalar, policy);
    let message = sign.is_none().then(|| {
        "represented scalar isolating interval did not certify sign relative to zero".to_owned()
    });
    BezierAlgebraicScalarSignEvidence {
        arithmetic,
        value: Some(TangentScalar::Represented(scalar)),
        sign,
        message,
    }
}

fn represented_sign(
    value: &AlgebraicRootRepresentation,
    policy: &CurveContext,
) -> Option<Ordering> {
    if let Some(witness) = value.exact_point_witness() {
        return compare_reals(witness, &Real::zero(), policy);
    }
    let lower = compare_reals(&value.interval.lower, &Real::zero(), policy)?;
    let upper = compare_reals(&value.interval.upper, &Real::zero(), policy)?;
    if matches!(lower, Ordering::Greater) {
        Some(Ordering::Greater)
    } else if matches!(upper, Ordering::Less) {
        Some(Ordering::Less)
    } else {
        refined_represented_sign(value, policy)
    }
}

fn refined_represented_sign(
    value: &AlgebraicRootRepresentation,
    policy: &CurveContext,
) -> Option<Ordering> {
    let zero = AlgebraicRootRepresentation::from_exact_value(&Real::zero());
    crate::bezier_algebraic_image::compare_algebraic_representations_with_policy(
        value, &zero, policy,
    )
}

fn sign_status(evidence: &BezierAlgebraicScalarSignEvidence) -> ScalarSignStatus {
    match evidence.sign {
        Some(Ordering::Greater) => ScalarSignStatus::Positive,
        Some(Ordering::Less) => ScalarSignStatus::Negative,
        Some(Ordering::Equal) => ScalarSignStatus::Zero,
        None if evidence.value.is_none() => ScalarSignStatus::ArithmeticFailed,
        None => ScalarSignStatus::Undecided,
    }
}

fn order_evidence(
    status: BezierAlgebraicTangentOrderStatus,
    ordering: Option<BezierTangentTurnOrdering2>,
    base_first_cross: Option<BezierAlgebraicScalarSignEvidence>,
    base_second_cross: Option<BezierAlgebraicScalarSignEvidence>,
    first_second_cross: Option<BezierAlgebraicScalarSignEvidence>,
    message: Option<String>,
) -> BezierAlgebraicTangentOrderEvidence {
    BezierAlgebraicTangentOrderEvidence {
        status,
        ordering,
        base_first_cross,
        base_second_cross,
        first_second_cross,
        message,
    }
}

fn same_tangent_evidence(
    status: BezierAlgebraicSameTangentOrderStatus,
    ordering: Option<BezierTangentTurnOrdering2>,
    first_side_witness: Option<BezierAlgebraicScalarSignEvidence>,
    second_side_witness: Option<BezierAlgebraicScalarSignEvidence>,
    normalized_difference: Option<BezierAlgebraicScalarSignEvidence>,
    message: Option<String>,
) -> BezierAlgebraicSameTangentOrderEvidence {
    BezierAlgebraicSameTangentOrderEvidence {
        status,
        ordering,
        first_side_witness,
        second_side_witness,
        normalized_difference,
        message,
    }
}

#[cfg(test)]
mod exact_real_status_tests {
    use super::*;

    fn selected_parameter(
        coefficients: Vec<Real>,
        lower: Real,
        upper: Real,
        policy: &CurveContext,
    ) -> crate::BezierAlgebraicParameter2 {
        let polynomial = crate::tests::decided(
            crate::BezierParameterPolynomial::try_new_power_basis(coefficients, policy).unwrap(),
        );
        let interval = crate::tests::decided(
            crate::BezierParameterInterval::try_new(lower, upper, policy).unwrap(),
        );
        crate::tests::decided(
            crate::BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap(),
        )
    }

    fn cubic_power_curve(rotate: bool, scale: i32) -> crate::CubicBezier2 {
        let q = |n: i32, d: i32| (Real::from(n) / Real::from(d)).unwrap();
        let point = |x: Real, y: Real| {
            if rotate {
                crate::Point2::new(-Real::from(scale) * y, Real::from(scale) * x)
            } else {
                crate::Point2::new(Real::from(scale) * x, Real::from(scale) * y)
            }
        };
        // (t,t^3), optionally rotated counter-clockwise and scaled.
        crate::CubicBezier2::new(
            point(Real::zero(), Real::zero()),
            point(q(1, 3), Real::zero()),
            point(q(2, 3), Real::zero()),
            point(Real::one(), Real::one()),
        )
    }

    #[test]
    fn selected_scalar_normalization_removes_speed_and_tangential_acceleration() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let parameter = selected_parameter(
                vec![-Real::pi(), Real::zero(), Real::zero(), Real::from(4)],
                Real::zero(),
                Real::one(),
                &policy,
            );
            let other_parameter = selected_parameter(
                vec![-Real::pi(), Real::zero(), Real::zero(), Real::from(32)],
                Real::zero(),
                (Real::one() / Real::from(2)).unwrap(),
                &policy,
            );
            let vector = |parameter: &crate::BezierAlgebraicParameter2, x, y| {
                BezierAlgebraicTangentVector2::from_image(&crate::tests::decided(
                    crate::bezier_algebraic_image::rational_tangent_image_from_power_basis(
                        parameter,
                        x,
                        y,
                        vec![Real::one()],
                        &policy,
                    )
                    .unwrap(),
                ))
            };
            let jet = |retimed: bool, s: i32, a: i32, c: i32, d: i32| {
                let parameter = if retimed {
                    &other_parameter
                } else {
                    &parameter
                };
                // y=c*x^2+d*x^3, with x'=s*alpha^2 and x''=a*pi.
                // Thus a_y=2*c*x'^2 and j_y=6*c*x'*x''+6*d*x'^3.
                // The graph derivatives are exactly 2*c and 6*d.
                let tangent = vector(
                    parameter,
                    vec![Real::zero(), Real::zero(), Real::from(s)],
                    vec![Real::zero()],
                );
                assert!(tangent.represented_coordinates().is_none());
                let acceleration = vector(
                    parameter,
                    vec![Real::from(a) * Real::pi()],
                    vec![
                        Real::zero(),
                        Real::zero(),
                        Real::zero(),
                        Real::zero(),
                        Real::from(2 * c * s * s),
                    ],
                );
                let jerk = vector(
                    parameter,
                    vec![Real::one()],
                    vec![
                        Real::zero(),
                        Real::zero(),
                        Real::from(6 * c * s * a) * Real::pi(),
                        Real::zero(),
                        Real::zero(),
                        Real::zero(),
                        Real::from(6 * d * s * s * s),
                    ],
                );
                [tangent, acceleration, jerk]
            };
            for retimed in [false, true] {
                for side in [-1, 1] {
                    for (first_speed, second_speed) in [(1, 2), (3, 1)] {
                        let first = jet(false, first_speed, -2, side, side);
                        let second = jet(retimed, second_speed, 3, 2 * side, 2 * side);
                        let comparison =
                            crate::tests::decided(compare_algebraic_same_tangent_second_order(
                                &first[0], &first[1], &second[0], &second[1], &policy,
                            ));
                        assert_eq!(
                            comparison.status,
                            BezierAlgebraicSameTangentOrderStatus::Ordered
                        );
                        // Negative rays are ordered by the angular cut at zero.
                        let expected = if side > 0 {
                            BezierTangentTurnOrdering2::FirstBeforeSecond
                        } else {
                            BezierTangentTurnOrdering2::SecondBeforeFirst
                        };
                        assert_eq!(comparison.ordering, Some(expected));
                        let difference = comparison.normalized_difference.unwrap();
                        let value = difference.selected_scalar().unwrap();
                        let mut replay = AlgebraicField::from_value(value);
                        assert_eq!(replay.sign(value).ok(), difference.sign);
                        assert!(
                            value.coefficients().0.len()
                                < value.selected_root().polynomial_coefficients.len()
                        );

                        let second = jet(retimed, second_speed, 3, side, 2 * side);
                        let curvature =
                            crate::tests::decided(compare_algebraic_same_tangent_second_order(
                                &first[0], &first[1], &second[0], &second[1], &policy,
                            ));
                        assert_eq!(
                            curvature.status,
                            BezierAlgebraicSameTangentOrderStatus::SameDirection
                        );
                        let third =
                            crate::tests::decided(compare_algebraic_equal_curvature_third_order(
                                first.each_ref(),
                                second.each_ref(),
                                &policy,
                            ));
                        assert_eq!(third.status, BezierAlgebraicSameTangentOrderStatus::Ordered);
                        assert_eq!(third.ordering, Some(expected));
                        let value = third
                            .normalized_difference
                            .as_ref()
                            .unwrap()
                            .selected_scalar()
                            .unwrap();
                        assert!(
                            value.coefficients().0.len()
                                < value.selected_root().polynomial_coefficients.len()
                        );

                        let first = jet(false, first_speed, -2, 0, side);
                        let second = jet(retimed, second_speed, 3, 0, 2 * side);
                        let third =
                            crate::tests::decided(compare_algebraic_same_tangent_third_order(
                                &first[0], &first[2], &second[0], &second[2], &policy,
                            ));
                        assert_eq!(third.status, BezierAlgebraicSameTangentOrderStatus::Ordered);
                        assert_eq!(third.ordering, Some(expected));
                        assert!(
                            third
                                .normalized_difference
                                .as_ref()
                                .unwrap()
                                .selected_scalar()
                                .is_some()
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn selected_constant_scalars_do_not_impose_a_foreign_parameter() {
        let policy = CurveContext::STRICT;
        let field = AlgebraicField::new(&AlgebraicRootRepresentation::from_exact_value(
            &Real::from(2),
        ))
        .unwrap();
        let constant = field.constant(Real::pi());
        let constant = BezierAlgebraicScalarSignEvidence {
            arithmetic: Vec::new(),
            value: Some(TangentScalar::Selected(field.finish(constant).unwrap())),
            sign: Some(Ordering::Greater),
            message: None,
        };
        let parameter = selected_parameter(
            vec![-Real::pi(), Real::zero(), Real::zero(), Real::from(4)],
            Real::zero(),
            Real::one(),
            &policy,
        );
        let vector = BezierAlgebraicTangentVector2::from_image(&crate::tests::decided(
            cubic_power_curve(false, 1)
                .tangent_at_algebraic_parameter(&parameter, &policy)
                .unwrap(),
        ));
        let speed = norm_squared_sign(&vector, &policy);
        let product = selected_scalar_calculation(&[&constant, &speed], |field, values| {
            field.multiply(&values[0], &values[1])
        })
        .unwrap();
        assert_eq!(product.sign, Some(Ordering::Greater));
        assert!(product.selected_scalar().is_some());
    }

    #[test]
    fn source_scalar_constants_reject_stale_exact_point_evidence() {
        let policy = CurveContext::STRICT;
        let parameter = selected_parameter(
            vec![-Real::pi(), Real::zero(), Real::zero(), Real::from(4)],
            Real::zero(),
            Real::one(),
            &policy,
        );
        let source = BezierAlgebraicTangentVector2::from_image(&crate::tests::decided(
            cubic_power_curve(false, 1)
                .tangent_at_algebraic_parameter(&parameter, &policy)
                .unwrap(),
        ));
        let mut invalid = AlgebraicRootRepresentation::from_exact_value(&Real::pi());
        invalid.polynomial_coefficients[0] = -Real::pi() - Real::one();
        let vector = BezierAlgebraicTangentVector2::new(
            invalid,
            AlgebraicRootRepresentation::from_exact_value(&Real::zero()),
        );
        let cross = source_bilinear_evidence(&source, &vector, false, &policy);
        assert!(cross.sign.is_none());
    }

    #[test]
    fn source_tangent_angles_retain_replay_and_reversal() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let parameter = selected_parameter(
                vec![-Real::pi(), Real::zero(), Real::zero(), Real::from(4)],
                Real::zero(),
                Real::one(),
                &policy,
            );
            let vector = |rotate, scale| {
                BezierAlgebraicTangentVector2::from_image(&crate::tests::decided(
                    cubic_power_curve(rotate, scale)
                        .tangent_at_algebraic_parameter(&parameter, &policy)
                        .unwrap(),
                ))
            };
            let first = vector(false, 1);
            let rotated = vector(true, 1);
            let scaled = vector(false, 2);
            let negated = first.negated(&policy).expect("exact retained reversal");
            for v in [&first, &rotated, &scaled, &negated] {
                assert!(v.represented_coordinates().is_none());
            }
            let exact = |x: Real, y: Real| {
                BezierAlgebraicTangentVector2::new(
                    AlgebraicRootRepresentation::from_exact_value(&x),
                    AlgebraicRootRepresentation::from_exact_value(&y),
                )
            };
            let axis = exact(Real::one(), Real::zero());
            let pi_axis = exact(Real::pi(), Real::zero());
            let diagonal = exact(Real::one(), Real::one());
            let up = exact(Real::zero(), Real::one());
            // u=(1,3*alpha^2), Ru=(-3*alpha^2,1), alpha in (0,1).
            // Rotation gives cross(u,Ru)>0. Scaling preserves its ray;
            // negation changes the cross-zero dot sign and angular half.
            for (base, a, b, expected) in [
                (
                    &axis,
                    &first,
                    &rotated,
                    BezierTangentTurnOrdering2::FirstBeforeSecond,
                ),
                (
                    &pi_axis,
                    &first,
                    &rotated,
                    BezierTangentTurnOrdering2::FirstBeforeSecond,
                ),
                (
                    &first,
                    &scaled,
                    &rotated,
                    BezierTangentTurnOrdering2::FirstBeforeSecond,
                ),
                (
                    &first,
                    &negated,
                    &rotated,
                    BezierTangentTurnOrdering2::SecondBeforeFirst,
                ),
                (
                    &axis,
                    &negated,
                    &rotated,
                    BezierTangentTurnOrdering2::SecondBeforeFirst,
                ),
                (
                    &up,
                    &first,
                    &diagonal,
                    BezierTangentTurnOrdering2::SecondBeforeFirst,
                ),
                (
                    &axis,
                    &rotated,
                    &first,
                    BezierTangentTurnOrdering2::SecondBeforeFirst,
                ),
            ] {
                let evidence = crate::tests::decided(compare_algebraic_tangent_turn_from_base(
                    base, a, b, &policy,
                ));
                assert_eq!(evidence.status, BezierAlgebraicTangentOrderStatus::Ordered);
                assert_eq!(evidence.ordering, Some(expected));
                if let Some(witness) =
                    evidence
                        .first_second_cross
                        .as_ref()
                        .and_then(|e| match e.value.as_ref()? {
                            TangentScalar::Bilinear(witness) => Some(witness),
                            _ => None,
                        })
                {
                    let replay =
                        source_bilinear_sign(&witness.first, &witness.second, witness.dot, &policy)
                            .unwrap();
                    let sign = match replay {
                        Some(Classification::Decided(RealSign::Positive)) => Ordering::Greater,
                        Some(Classification::Decided(RealSign::Negative)) => Ordering::Less,
                        Some(Classification::Decided(RealSign::Zero)) => Ordering::Equal,
                        _ => panic!("source sign replay lost its selected root"),
                    };
                    assert_eq!(
                        evidence.first_second_cross.as_ref().unwrap().sign,
                        Some(sign)
                    );
                }
            }
            let restored = negated.negated(&policy).unwrap();
            let cross = cross_sign(&first, &restored, &policy, true);
            let dot = dot_sign(&first, &restored, &policy, true);
            assert_eq!(cross.sign, Some(Ordering::Equal));
            assert_eq!(dot.sign, Some(Ordering::Greater));
            assert!(matches!(cross.value, Some(TangentScalar::Bilinear(_))));
            assert!(matches!(dot.value, Some(TangentScalar::Bilinear(_))));
        }
    }

    #[test]
    fn retained_tangent_determinants_use_selected_source_signs() {
        let q = |n: i32, d: i32| (Real::from(n) / Real::from(d)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let coefficients = vec![-Real::pi(), Real::zero(), Real::zero(), Real::from(4)];
            let parameter =
                selected_parameter(coefficients.clone(), Real::zero(), Real::one(), &policy);
            let tighter = selected_parameter(coefficients, q(9, 10), q(19, 20), &policy);
            let tangent = |curve: &crate::CubicBezier2,
                           parameter: &crate::BezierAlgebraicParameter2| {
                crate::tests::decided(
                    curve
                        .tangent_at_algebraic_parameter(parameter, &policy)
                        .unwrap(),
                )
            };
            let first = tangent(&cubic_power_curve(false, 1), &parameter);
            let rotated = tangent(&cubic_power_curve(true, 1), &parameter);
            let scaled = tangent(&cubic_power_curve(false, 2), &parameter);
            let refined = tangent(&cubic_power_curve(true, 1), &tighter);
            for image in [&first, &rotated, &scaled, &refined] {
                assert_eq!(
                    image.status(),
                    crate::BezierAlgebraicImageStatus::RetainedRationalExpression
                );
            }
            // u=(1,3a^2), Ru=(-3a^2,1): cross(u,Ru)=1+9a^4>0.
            for (left, right, expected) in [
                (&first, &rotated, RealSign::Positive),
                (&rotated, &first, RealSign::Negative),
                (&first, &scaled, RealSign::Zero),
                (&first, &refined, RealSign::Positive),
            ] {
                assert_eq!(
                    algebraic_endpoint_tangent_cross_sign(left, right, &policy),
                    Classification::Decided(expected)
                );
            }
            // Mix a retained tangent with materialized coordinate roots.
            let point = |x: Real| crate::Point2::new(x, Real::zero());
            for stationary in [false, true] {
                let endpoint = if stationary {
                    Real::zero()
                } else {
                    Real::one()
                };
                let curve = crate::QuadraticBezier2::new(
                    point(Real::zero()),
                    point((&endpoint / Real::from(2)).unwrap()),
                    point(endpoint),
                );
                let axis = crate::tests::decided(
                    curve
                        .tangent_at_algebraic_parameter(&parameter, &policy)
                        .unwrap(),
                );
                assert_eq!(
                    axis.status(),
                    crate::BezierAlgebraicImageStatus::Transformed
                );
                assert_eq!(
                    algebraic_endpoint_tangent_cross_sign(&first, &axis, &policy),
                    Classification::Decided(if stationary {
                        RealSign::Zero
                    } else {
                        RealSign::Negative
                    })
                );
                assert_eq!(
                    algebraic_endpoint_tangent_cross_sign(&axis, &first, &policy),
                    Classification::Decided(if stationary {
                        RealSign::Zero
                    } else {
                        RealSign::Positive
                    })
                );
            }
            // For C=(1/(1+t),t/(1+t)), C''=(2,-2)/(1+t)^3.
            // Negating every homogeneous weight preserves C but changes the
            // sign of the retained odd denominator power. Both must give
            // cross(u,C'')=-2*(1+3a^2)/(1+a)^3<0.
            for scale in [Real::one(), -Real::one()] {
                let curve = crate::RationalQuadraticBezier2::try_new(
                    crate::Point2::from_values(1, 0),
                    crate::Point2::new(q(2, 3), q(1, 3)),
                    crate::Point2::new(q(1, 2), q(1, 2)),
                    scale.clone(),
                    &scale * q(3, 2),
                    &scale * Real::from(2),
                )
                .unwrap();
                let second = crate::tests::decided(
                    curve
                        .derivatives_at_algebraic_parameter(&parameter, 2, &policy)
                        .unwrap(),
                )
                .pop()
                .unwrap();
                assert_eq!(
                    second.status(),
                    crate::BezierAlgebraicImageStatus::RetainedRationalExpression
                );
                assert_eq!(
                    algebraic_endpoint_tangent_cross_sign(&first, &second, &policy),
                    Classification::Decided(RealSign::Negative)
                );
                assert_eq!(
                    algebraic_endpoint_tangent_cross_sign(&second, &first, &policy),
                    Classification::Decided(RealSign::Positive)
                );
            }
        }
    }

    #[test]
    fn tangent_determinants_do_not_merge_distinct_selected_roots() {
        let q = |n: i32, d: i32| (Real::from(n) / Real::from(d)).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // P=16(t-1/2)^4-pi/16 has one root in each half of [0,1].
            let coefficients = vec![
                Real::one() - (Real::pi() / Real::from(16)).unwrap(),
                Real::from(-8),
                Real::from(24),
                Real::from(-32),
                Real::from(16),
            ];
            let first_parameter =
                selected_parameter(coefficients.clone(), Real::zero(), q(1, 2), &policy);
            let second_parameter = selected_parameter(coefficients, q(1, 2), Real::one(), &policy);
            let curve = cubic_power_curve(false, 1);
            let first = crate::tests::decided(
                curve
                    .tangent_at_algebraic_parameter(&first_parameter, &policy)
                    .unwrap(),
            );
            let second = crate::tests::decided(
                curve
                    .tangent_at_algebraic_parameter(&second_parameter, &policy)
                    .unwrap(),
            );
            assert_eq!(
                first.status(),
                crate::BezierAlgebraicImageStatus::RetainedRationalExpression
            );
            assert_eq!(
                second.status(),
                crate::BezierAlgebraicImageStatus::RetainedRationalExpression
            );
            // cross((1,3a^2),(1,3b^2))=3(b^2-a^2)>0. Sharing P alone
            // must never make these two derivatives appear equal. The general
            // independent-source path may certify the sign or remain blocked.
            assert!(matches!(
                algebraic_endpoint_tangent_cross_sign(&first, &second, &policy),
                Classification::Decided(RealSign::Positive) | Classification::Uncertain(_)
            ));
            assert!(matches!(
                algebraic_endpoint_tangent_cross_sign(&second, &first, &policy),
                Classification::Decided(RealSign::Negative) | Classification::Uncertain(_)
            ));
        }
    }

    #[test]
    fn scalar_sign_accepts_an_exact_real_arithmetic_result() {
        let evidence = scalar_sign_evidence(
            vec![AlgebraicRootArithmeticReport {
                operation: AlgebraicRootArithmeticOp::Negate,
                status: AlgebraicRootArithmeticStatus::ComputedExactRealWitness,
                exact_result: Some(Real::pi()),
                result_representation: None,
                message: None,
            }],
            &CurveContext::STRICT,
        );

        assert_eq!(evidence.sign, Some(Ordering::Greater));
        let scalar = evidence
            .represented_scalar()
            .expect("the exact Real result is retained");
        assert_eq!(scalar.exact_point_witness(), Some(&Real::pi()));
    }
}
