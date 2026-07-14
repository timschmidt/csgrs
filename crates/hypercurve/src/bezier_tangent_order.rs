//! Certified tangent-order predicates for algebraic Bezier endpoint images.
//!
//! Native arrangement traversal orders branch successors by signs of cross and
//! dot products.  Algebraic endpoint images need the same predicate without
//! collapsing represented coordinates to sampled floats.  This module builds
//! the cross/dot scalars as exact represented algebraic roots with
//! `hypersolve` arithmetic, then reads their signs only from exact rational
//! witnesses or isolating intervals certified away from zero.  This follows
//! the exact-geometric-computation boundary between construction and
//! decision.  The local angular ordering
//! is the standard orientation/dot half-plane ordering used by arrangement
//! kernels.

use std::cmp::Ordering;

use hyperreal::Real;
use hypersolve::{
    AlgebraicRootArithmeticOp, AlgebraicRootArithmeticReport, AlgebraicRootArithmeticStatus,
    AlgebraicRootRepresentation, arithmetic_algebraic_root_representations,
};
#[cfg(feature = "predicates")]
use hypersolve::{
    AlgebraicRootComparisonStatus, AlgebraicRootRefinementComparisonConfig,
    compare_algebraic_root_representations_by_difference,
};

use crate::classify::compare_reals;
use crate::{BezierAlgebraicImageStatus, BezierEndpointTangentImage2, Classification, CurvePolicy};

/// A represented algebraic tangent vector with exact coordinate evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAlgebraicTangentVector2 {
    dx: AlgebraicRootRepresentation,
    dy: AlgebraicRootRepresentation,
}

impl BezierAlgebraicTangentVector2 {
    /// Constructs a tangent vector from represented x/y derivative images.
    pub const fn new(
        dx: AlgebraicRootRepresentation,
        dy: AlgebraicRootRepresentation,
    ) -> BezierAlgebraicTangentVector2 {
        Self { dx, dy }
    }

    /// Extracts a represented vector from a transformed endpoint tangent image.
    pub fn from_endpoint_image(
        image: &BezierEndpointTangentImage2,
    ) -> BezierAlgebraicTangentVectorReport {
        if image.status() != BezierAlgebraicImageStatus::Transformed {
            return BezierAlgebraicTangentVectorReport {
                status: BezierAlgebraicTangentVectorStatus::ImageNotTransformed,
                vector: None,
                message: Some("endpoint tangent image was not transformed".to_owned()),
            };
        }

        let (dx, dy) = match image {
            BezierEndpointTangentImage2::Polynomial(image) => {
                let dx = image
                    .dx()
                    .and_then(|coordinate| coordinate.representation());
                let dy = image
                    .dy()
                    .and_then(|coordinate| coordinate.representation());
                (dx, dy)
            }
            BezierEndpointTangentImage2::Rational(image) => {
                let dx = image
                    .dx()
                    .and_then(|coordinate| coordinate.representation());
                let dy = image
                    .dy()
                    .and_then(|coordinate| coordinate.representation());
                (dx, dy)
            }
        };
        let (Some(dx), Some(dy)) = (dx, dy) else {
            return BezierAlgebraicTangentVectorReport {
                status: BezierAlgebraicTangentVectorStatus::MissingCoordinateImage,
                vector: None,
                message: Some("endpoint tangent image omitted a represented coordinate".to_owned()),
            };
        };

        BezierAlgebraicTangentVectorReport {
            status: BezierAlgebraicTangentVectorStatus::Extracted,
            vector: Some(Self::new(dx.clone(), dy.clone())),
            message: None,
        }
    }

    /// Returns the represented x derivative coordinate.
    pub const fn dx(&self) -> &AlgebraicRootRepresentation {
        &self.dx
    }

    /// Returns the represented y derivative coordinate.
    pub const fn dy(&self) -> &AlgebraicRootRepresentation {
        &self.dy
    }
}

/// Status for extracting represented tangent coordinates from an image.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BezierAlgebraicTangentVectorStatus {
    /// Both derivative coordinates were represented.
    Extracted,
    /// The tangent image did not finish exact construction.
    ImageNotTransformed,
    /// The image status was transformed but a coordinate representation was absent.
    MissingCoordinateImage,
}

/// Extraction report for a represented tangent vector.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAlgebraicTangentVectorReport {
    /// Extraction status.
    pub status: BezierAlgebraicTangentVectorStatus,
    /// Represented tangent vector when extraction succeeds.
    pub vector: Option<BezierAlgebraicTangentVector2>,
    /// Compact diagnostic for failed extraction.
    pub message: Option<String>,
}

/// Certified turn ordering for two candidate tangents around a base tangent.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierTangentTurnOrdering2 {
    /// The first candidate is encountered before the second in counter-clockwise order.
    FirstBeforeSecond,
    /// The second candidate is encountered before the first in counter-clockwise order.
    SecondBeforeFirst,
}

/// Status for algebraic tangent-order comparison.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BezierAlgebraicTangentOrderStatus {
    /// The two candidate turns were ordered.
    Ordered,
    /// The candidates have the same represented direction.
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
pub enum BezierAlgebraicSameTangentOrderStatus {
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

/// Sign construction report for a cross, dot, or norm-squared scalar.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAlgebraicScalarSignReport {
    /// Arithmetic reports produced while constructing the scalar.
    pub arithmetic: Vec<AlgebraicRootArithmeticReport>,
    /// Represented scalar when construction succeeds.
    pub scalar: Option<AlgebraicRootRepresentation>,
    /// Certified sign relative to zero.
    pub sign: Option<Ordering>,
    /// Compact diagnostic for construction or sign failure.
    pub message: Option<String>,
}

/// Report for a certified algebraic tangent-order predicate.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAlgebraicTangentOrderReport {
    /// Final predicate status.
    pub status: BezierAlgebraicTangentOrderStatus,
    /// Certified ordering when `status == Ordered`.
    pub ordering: Option<BezierTangentTurnOrdering2>,
    /// Base/first cross-product sign evidence.
    pub base_first_cross: Option<BezierAlgebraicScalarSignReport>,
    /// Base/second cross-product sign evidence.
    pub base_second_cross: Option<BezierAlgebraicScalarSignReport>,
    /// First/second cross-product sign evidence.
    pub first_second_cross: Option<BezierAlgebraicScalarSignReport>,
    /// Compact diagnostic for unresolved predicates.
    pub message: Option<String>,
}

/// Report for a certified algebraic same-tangent higher-order predicate.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAlgebraicSameTangentOrderReport {
    /// Final predicate status.
    pub status: BezierAlgebraicSameTangentOrderStatus,
    /// Certified ordering when `status == Ordered`.
    pub ordering: Option<BezierTangentTurnOrdering2>,
    /// First candidate `cross(B'(t), B''(t))` sign evidence.
    pub first_curvature_cross: Option<BezierAlgebraicScalarSignReport>,
    /// Second candidate `cross(B'(t), B''(t))` sign evidence.
    pub second_curvature_cross: Option<BezierAlgebraicScalarSignReport>,
    /// Same-side curvature-magnitude difference after clearing speed
    /// denominators.
    pub magnitude_difference: Option<BezierAlgebraicScalarSignReport>,
    /// Compact diagnostic for unresolved predicates.
    pub message: Option<String>,
}

/// Compares two candidate tangent turns from a base tangent.
///
/// The result matches the native branch-order predicate: first classify each
/// candidate into the positive or negative half-turn from `base` using cross
/// and dot signs, then order candidates in the same half by the sign of
/// `first x second`.  Every scalar is represented exactly through
/// `hypersolve` arithmetic; no isolating interval is sampled as a coordinate.
pub fn compare_algebraic_tangent_turn_from_base(
    base: &BezierAlgebraicTangentVector2,
    first: &BezierAlgebraicTangentVector2,
    second: &BezierAlgebraicTangentVector2,
    policy: &CurvePolicy,
) -> Classification<BezierAlgebraicTangentOrderReport> {
    for tangent in [base, first, second] {
        match tangent_nonzero(tangent, policy) {
            AlgebraicTangentNonzero::Nonzero => {}
            AlgebraicTangentNonzero::Zero(report) => {
                return Classification::Decided(order_report(
                    BezierAlgebraicTangentOrderStatus::ZeroTangent,
                    None,
                    None,
                    None,
                    None,
                    Some(format!("zero tangent certified by {:?}", report.sign)),
                ));
            }
            AlgebraicTangentNonzero::Undecided(report) => {
                return Classification::Decided(order_report(
                    BezierAlgebraicTangentOrderStatus::SignUndecided,
                    None,
                    None,
                    None,
                    None,
                    report.message,
                ));
            }
            AlgebraicTangentNonzero::ArithmeticFailed(report) => {
                return Classification::Decided(order_report(
                    BezierAlgebraicTangentOrderStatus::ArithmeticFailed,
                    None,
                    None,
                    None,
                    None,
                    report.message,
                ));
            }
        }
    }

    let (first_half, base_first_cross) = match turn_half(base, first, policy) {
        AlgebraicHalfTurn::Half(half, cross) => (half, cross),
        AlgebraicHalfTurn::ZeroTangent(cross, dot) => {
            return Classification::Decided(order_report(
                BezierAlgebraicTangentOrderStatus::ZeroTangent,
                None,
                Some(cross),
                None,
                Some(dot),
                Some("first tangent has zero direction relative to base".to_owned()),
            ));
        }
        AlgebraicHalfTurn::Undecided(cross, dot) => {
            return Classification::Decided(order_report(
                BezierAlgebraicTangentOrderStatus::SignUndecided,
                None,
                Some(cross),
                None,
                dot,
                Some("could not certify first candidate half-turn".to_owned()),
            ));
        }
        AlgebraicHalfTurn::ArithmeticFailed(cross, dot) => {
            return Classification::Decided(order_report(
                BezierAlgebraicTangentOrderStatus::ArithmeticFailed,
                None,
                Some(cross),
                None,
                dot,
                Some("could not construct first candidate half-turn scalar".to_owned()),
            ));
        }
    };
    let (second_half, base_second_cross) = match turn_half(base, second, policy) {
        AlgebraicHalfTurn::Half(half, cross) => (half, cross),
        AlgebraicHalfTurn::ZeroTangent(cross, dot) => {
            return Classification::Decided(order_report(
                BezierAlgebraicTangentOrderStatus::ZeroTangent,
                None,
                Some(base_first_cross),
                Some(cross),
                Some(dot),
                Some("second tangent has zero direction relative to base".to_owned()),
            ));
        }
        AlgebraicHalfTurn::Undecided(cross, dot) => {
            return Classification::Decided(order_report(
                BezierAlgebraicTangentOrderStatus::SignUndecided,
                None,
                Some(base_first_cross),
                Some(cross),
                dot,
                Some("could not certify second candidate half-turn".to_owned()),
            ));
        }
        AlgebraicHalfTurn::ArithmeticFailed(cross, dot) => {
            return Classification::Decided(order_report(
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
        return Classification::Decided(order_report(
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

    let first_second_cross = cross_sign(first, second, policy);
    match sign_status(&first_second_cross) {
        ScalarSignStatus::Positive => Classification::Decided(order_report(
            BezierAlgebraicTangentOrderStatus::Ordered,
            Some(BezierTangentTurnOrdering2::FirstBeforeSecond),
            Some(base_first_cross),
            Some(base_second_cross),
            Some(first_second_cross),
            None,
        )),
        ScalarSignStatus::Negative => Classification::Decided(order_report(
            BezierAlgebraicTangentOrderStatus::Ordered,
            Some(BezierTangentTurnOrdering2::SecondBeforeFirst),
            Some(base_first_cross),
            Some(base_second_cross),
            Some(first_second_cross),
            None,
        )),
        ScalarSignStatus::Zero => Classification::Decided(order_report(
            BezierAlgebraicTangentOrderStatus::SameDirection,
            None,
            Some(base_first_cross),
            Some(base_second_cross),
            Some(first_second_cross),
            Some("candidate tangent directions are collinear with the same half-turn".to_owned()),
        )),
        ScalarSignStatus::Undecided => Classification::Decided(order_report(
            BezierAlgebraicTangentOrderStatus::SignUndecided,
            None,
            Some(base_first_cross),
            Some(base_second_cross),
            Some(first_second_cross),
            Some("could not certify candidate tangent order sign".to_owned()),
        )),
        ScalarSignStatus::ArithmeticFailed => Classification::Decided(order_report(
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
/// This is the represented-root analogue of the native signed-curvature tie
/// breaker used by retained Bezier traversal. Given two candidates already
/// known to have the same first-order direction, it compares the signs of
/// `cross(B'(t), B''(t))`; branches departing on opposite sides are ordered by
/// that sign. When both depart on the same side it compares
/// `cross^2 / |B'|^6` by clearing positive speed denominators. Every scalar is
/// built through `hypersolve` algebraic arithmetic, following the exactness model's exact
/// geometric computation discipline, and the derivative identities are the
/// standard Bezier endpoint/Taylor formulas described by the Bernstein curve model.
pub fn compare_algebraic_same_tangent_second_order(
    first_tangent: &BezierAlgebraicTangentVector2,
    first_second_derivative: &BezierAlgebraicTangentVector2,
    second_tangent: &BezierAlgebraicTangentVector2,
    second_second_derivative: &BezierAlgebraicTangentVector2,
    policy: &CurvePolicy,
) -> Classification<BezierAlgebraicSameTangentOrderReport> {
    for tangent in [first_tangent, second_tangent] {
        match tangent_nonzero(tangent, policy) {
            AlgebraicTangentNonzero::Nonzero => {}
            AlgebraicTangentNonzero::Zero(report) => {
                return Classification::Decided(same_tangent_report(
                    BezierAlgebraicSameTangentOrderStatus::ZeroTangent,
                    None,
                    None,
                    None,
                    None,
                    Some(format!("zero tangent certified by {:?}", report.sign)),
                ));
            }
            AlgebraicTangentNonzero::Undecided(report) => {
                return Classification::Decided(same_tangent_report(
                    BezierAlgebraicSameTangentOrderStatus::SignUndecided,
                    None,
                    None,
                    None,
                    None,
                    report.message,
                ));
            }
            AlgebraicTangentNonzero::ArithmeticFailed(report) => {
                return Classification::Decided(same_tangent_report(
                    BezierAlgebraicSameTangentOrderStatus::ArithmeticFailed,
                    None,
                    None,
                    None,
                    None,
                    report.message,
                ));
            }
        }
    }

    let first_cross = cross_sign(first_tangent, first_second_derivative, policy);
    let second_cross = cross_sign(second_tangent, second_second_derivative, policy);
    match (sign_status(&first_cross), sign_status(&second_cross)) {
        (ScalarSignStatus::ArithmeticFailed, _) | (_, ScalarSignStatus::ArithmeticFailed) => {
            Classification::Decided(same_tangent_report(
                BezierAlgebraicSameTangentOrderStatus::ArithmeticFailed,
                None,
                Some(first_cross),
                Some(second_cross),
                None,
                Some("could not construct algebraic curvature cross scalar".to_owned()),
            ))
        }
        (ScalarSignStatus::Undecided, _) | (_, ScalarSignStatus::Undecided) => {
            Classification::Decided(same_tangent_report(
                BezierAlgebraicSameTangentOrderStatus::SignUndecided,
                None,
                Some(first_cross),
                Some(second_cross),
                None,
                Some("could not certify algebraic curvature cross sign".to_owned()),
            ))
        }
        (ScalarSignStatus::Zero, ScalarSignStatus::Zero) => {
            Classification::Decided(same_tangent_report(
                BezierAlgebraicSameTangentOrderStatus::SameDirection,
                None,
                Some(first_cross),
                Some(second_cross),
                None,
                Some("both algebraic second-order side witnesses vanished".to_owned()),
            ))
        }
        (ScalarSignStatus::Zero, _) | (_, ScalarSignStatus::Zero) => {
            Classification::Decided(same_tangent_report(
                BezierAlgebraicSameTangentOrderStatus::SameDirection,
                None,
                Some(first_cross),
                Some(second_cross),
                None,
                Some("one algebraic second-order side witness vanished".to_owned()),
            ))
        }
        (ScalarSignStatus::Positive, ScalarSignStatus::Negative) => {
            Classification::Decided(same_tangent_report(
                BezierAlgebraicSameTangentOrderStatus::Ordered,
                Some(BezierTangentTurnOrdering2::FirstBeforeSecond),
                Some(first_cross),
                Some(second_cross),
                None,
                None,
            ))
        }
        (ScalarSignStatus::Negative, ScalarSignStatus::Positive) => {
            Classification::Decided(same_tangent_report(
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
/// departure, and same-side magnitudes are compared as `cross^2 / |B'|^4` by
/// clearing positive speed denominators.  The derivative witness is the
/// standard polynomial Bezier endpoint formula from the Bernstein and de Casteljau curve model, and the predicate follows the exactness model's
/// exact-geometric-computation rule: construct represented algebraic scalars
/// first, then branch only on certified signs.
pub fn compare_algebraic_same_tangent_third_order(
    first_tangent: &BezierAlgebraicTangentVector2,
    first_third_derivative: &BezierAlgebraicTangentVector2,
    second_tangent: &BezierAlgebraicTangentVector2,
    second_third_derivative: &BezierAlgebraicTangentVector2,
    policy: &CurvePolicy,
) -> Classification<BezierAlgebraicSameTangentOrderReport> {
    for tangent in [first_tangent, second_tangent] {
        match tangent_nonzero(tangent, policy) {
            AlgebraicTangentNonzero::Nonzero => {}
            AlgebraicTangentNonzero::Zero(report) => {
                return Classification::Decided(same_tangent_report(
                    BezierAlgebraicSameTangentOrderStatus::ZeroTangent,
                    None,
                    None,
                    None,
                    None,
                    Some(format!("zero tangent certified by {:?}", report.sign)),
                ));
            }
            AlgebraicTangentNonzero::Undecided(report) => {
                return Classification::Decided(same_tangent_report(
                    BezierAlgebraicSameTangentOrderStatus::SignUndecided,
                    None,
                    None,
                    None,
                    None,
                    report.message,
                ));
            }
            AlgebraicTangentNonzero::ArithmeticFailed(report) => {
                return Classification::Decided(same_tangent_report(
                    BezierAlgebraicSameTangentOrderStatus::ArithmeticFailed,
                    None,
                    None,
                    None,
                    None,
                    report.message,
                ));
            }
        }
    }

    let first_cross = cross_sign(first_tangent, first_third_derivative, policy);
    let second_cross = cross_sign(second_tangent, second_third_derivative, policy);
    match (sign_status(&first_cross), sign_status(&second_cross)) {
        (ScalarSignStatus::ArithmeticFailed, _) | (_, ScalarSignStatus::ArithmeticFailed) => {
            Classification::Decided(same_tangent_report(
                BezierAlgebraicSameTangentOrderStatus::ArithmeticFailed,
                None,
                Some(first_cross),
                Some(second_cross),
                None,
                Some("could not construct algebraic third-order cross scalar".to_owned()),
            ))
        }
        (ScalarSignStatus::Undecided, _) | (_, ScalarSignStatus::Undecided) => {
            Classification::Decided(same_tangent_report(
                BezierAlgebraicSameTangentOrderStatus::SignUndecided,
                None,
                Some(first_cross),
                Some(second_cross),
                None,
                Some("could not certify algebraic third-order cross sign".to_owned()),
            ))
        }
        (ScalarSignStatus::Zero, _) | (_, ScalarSignStatus::Zero) => {
            Classification::Decided(same_tangent_report(
                BezierAlgebraicSameTangentOrderStatus::SameDirection,
                None,
                Some(first_cross),
                Some(second_cross),
                None,
                Some("an algebraic third-order side witness vanished".to_owned()),
            ))
        }
        (ScalarSignStatus::Positive, ScalarSignStatus::Negative) => {
            Classification::Decided(same_tangent_report(
                BezierAlgebraicSameTangentOrderStatus::Ordered,
                Some(BezierTangentTurnOrdering2::FirstBeforeSecond),
                Some(first_cross),
                Some(second_cross),
                None,
                None,
            ))
        }
        (ScalarSignStatus::Negative, ScalarSignStatus::Positive) => {
            Classification::Decided(same_tangent_report(
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
                2,
                "third-order",
                policy,
            )
        }
    }
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
    Zero(BezierAlgebraicScalarSignReport),
    Undecided(BezierAlgebraicScalarSignReport),
    ArithmeticFailed(BezierAlgebraicScalarSignReport),
}

enum AlgebraicHalfTurn {
    Half(u8, BezierAlgebraicScalarSignReport),
    ZeroTangent(
        BezierAlgebraicScalarSignReport,
        BezierAlgebraicScalarSignReport,
    ),
    Undecided(
        BezierAlgebraicScalarSignReport,
        Option<BezierAlgebraicScalarSignReport>,
    ),
    ArithmeticFailed(
        BezierAlgebraicScalarSignReport,
        Option<BezierAlgebraicScalarSignReport>,
    ),
}

fn tangent_nonzero(
    tangent: &BezierAlgebraicTangentVector2,
    policy: &CurvePolicy,
) -> AlgebraicTangentNonzero {
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
    policy: &CurvePolicy,
) -> AlgebraicHalfTurn {
    let cross = cross_sign(base, candidate, policy);
    match sign_status(&cross) {
        ScalarSignStatus::Positive => AlgebraicHalfTurn::Half(0, cross),
        ScalarSignStatus::Negative => AlgebraicHalfTurn::Half(1, cross),
        ScalarSignStatus::Zero => {
            let dot = dot_sign(base, candidate, policy);
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
    policy: &CurvePolicy,
) -> BezierAlgebraicScalarSignReport {
    let left_x_right_y = multiply(left.dx(), right.dy());
    let left_y_right_x = multiply(left.dy(), right.dx());
    let scalar = subtract(
        left_x_right_y.result_representation.as_ref(),
        left_x_right_y.exact_result.as_ref(),
        left_y_right_x.result_representation.as_ref(),
        left_y_right_x.exact_result.as_ref(),
    );
    scalar_sign_report(vec![left_x_right_y, left_y_right_x, scalar], policy)
}

fn dot_sign(
    left: &BezierAlgebraicTangentVector2,
    right: &BezierAlgebraicTangentVector2,
    policy: &CurvePolicy,
) -> BezierAlgebraicScalarSignReport {
    let left_x_right_x = multiply(left.dx(), right.dx());
    let left_y_right_y = multiply(left.dy(), right.dy());
    let scalar = add(
        left_x_right_x.result_representation.as_ref(),
        left_x_right_x.exact_result.as_ref(),
        left_y_right_y.result_representation.as_ref(),
        left_y_right_y.exact_result.as_ref(),
    );
    scalar_sign_report(vec![left_x_right_x, left_y_right_y, scalar], policy)
}

fn norm_squared_sign(
    vector: &BezierAlgebraicTangentVector2,
    policy: &CurvePolicy,
) -> BezierAlgebraicScalarSignReport {
    let dx_squared = multiply(vector.dx(), vector.dx());
    let dy_squared = multiply(vector.dy(), vector.dy());
    let scalar = add(
        dx_squared.result_representation.as_ref(),
        dx_squared.exact_result.as_ref(),
        dy_squared.result_representation.as_ref(),
        dy_squared.exact_result.as_ref(),
    );
    scalar_sign_report(vec![dx_squared, dy_squared, scalar], policy)
}

fn compare_algebraic_same_side_curvature_magnitude(
    first_tangent: &BezierAlgebraicTangentVector2,
    first_cross: BezierAlgebraicScalarSignReport,
    second_tangent: &BezierAlgebraicTangentVector2,
    second_cross: BezierAlgebraicScalarSignReport,
    policy: &CurvePolicy,
) -> Classification<BezierAlgebraicSameTangentOrderReport> {
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
    first_cross: BezierAlgebraicScalarSignReport,
    second_tangent: &BezierAlgebraicTangentVector2,
    second_cross: BezierAlgebraicScalarSignReport,
    speed_power: usize,
    witness_name: &str,
    policy: &CurvePolicy,
) -> Classification<BezierAlgebraicSameTangentOrderReport> {
    let first_speed = norm_squared_sign(first_tangent, policy);
    let second_speed = norm_squared_sign(second_tangent, policy);
    if !matches!(sign_status(&first_speed), ScalarSignStatus::Positive)
        || !matches!(sign_status(&second_speed), ScalarSignStatus::Positive)
    {
        return Classification::Decided(same_tangent_report(
            BezierAlgebraicSameTangentOrderStatus::SignUndecided,
            None,
            Some(first_cross),
            Some(second_cross),
            None,
            Some("could not certify positive tangent speeds".to_owned()),
        ));
    }

    let magnitude = same_side_magnitude_difference(
        &first_cross,
        &second_cross,
        &first_speed,
        &second_speed,
        speed_power,
        policy,
    );
    match sign_status(&magnitude) {
        ScalarSignStatus::Negative => Classification::Decided(same_tangent_report(
            BezierAlgebraicSameTangentOrderStatus::Ordered,
            Some(BezierTangentTurnOrdering2::FirstBeforeSecond),
            Some(first_cross),
            Some(second_cross),
            Some(magnitude),
            None,
        )),
        ScalarSignStatus::Positive => Classification::Decided(same_tangent_report(
            BezierAlgebraicSameTangentOrderStatus::Ordered,
            Some(BezierTangentTurnOrdering2::SecondBeforeFirst),
            Some(first_cross),
            Some(second_cross),
            Some(magnitude),
            None,
        )),
        ScalarSignStatus::Zero => Classification::Decided(same_tangent_report(
            BezierAlgebraicSameTangentOrderStatus::SameDirection,
            None,
            Some(first_cross),
            Some(second_cross),
            Some(magnitude),
            Some(format!(
                "same-side algebraic {witness_name} magnitudes are equal"
            )),
        )),
        ScalarSignStatus::Undecided => Classification::Decided(same_tangent_report(
            BezierAlgebraicSameTangentOrderStatus::SignUndecided,
            None,
            Some(first_cross),
            Some(second_cross),
            Some(magnitude),
            Some(format!(
                "could not certify same-side algebraic {witness_name} magnitude"
            )),
        )),
        ScalarSignStatus::ArithmeticFailed => Classification::Decided(same_tangent_report(
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
    first_cross: &BezierAlgebraicScalarSignReport,
    second_cross: &BezierAlgebraicScalarSignReport,
    first_speed: &BezierAlgebraicScalarSignReport,
    second_speed: &BezierAlgebraicScalarSignReport,
    speed_power: usize,
    policy: &CurvePolicy,
) -> BezierAlgebraicScalarSignReport {
    let Some(first_cross_scalar) = first_cross.scalar.as_ref() else {
        return scalar_sign_report(
            vec![missing_operand_report(
                AlgebraicRootArithmeticOp::Multiply,
                "first curvature cross scalar was absent",
            )],
            policy,
        );
    };
    let Some(second_cross_scalar) = second_cross.scalar.as_ref() else {
        return scalar_sign_report(
            vec![missing_operand_report(
                AlgebraicRootArithmeticOp::Multiply,
                "second curvature cross scalar was absent",
            )],
            policy,
        );
    };
    let Some(first_speed_scalar) = first_speed.scalar.as_ref() else {
        return scalar_sign_report(
            vec![missing_operand_report(
                AlgebraicRootArithmeticOp::Multiply,
                "first speed scalar was absent",
            )],
            policy,
        );
    };
    let Some(second_speed_scalar) = second_speed.scalar.as_ref() else {
        return scalar_sign_report(
            vec![missing_operand_report(
                AlgebraicRootArithmeticOp::Multiply,
                "second speed scalar was absent",
            )],
            policy,
        );
    };

    let first_cross_squared = multiply(first_cross_scalar, first_cross_scalar);
    let second_cross_squared = multiply(second_cross_scalar, second_cross_scalar);
    let first_speed_power = power_representation(first_speed_scalar, speed_power);
    let second_speed_power = power_representation(second_speed_scalar, speed_power);
    let first_scaled = multiply_report_results(&first_cross_squared, &second_speed_power);
    let second_scaled = multiply_report_results(&second_cross_squared, &first_speed_power);
    let difference = subtract(
        first_scaled.result_representation.as_ref(),
        first_scaled.exact_result.as_ref(),
        second_scaled.result_representation.as_ref(),
        second_scaled.exact_result.as_ref(),
    );

    let mut arithmetic = Vec::new();
    arithmetic.push(first_cross_squared);
    arithmetic.push(second_cross_squared);
    arithmetic.extend(first_speed_power.arithmetic);
    arithmetic.extend(second_speed_power.arithmetic);
    arithmetic.push(first_scaled);
    arithmetic.push(second_scaled);
    arithmetic.push(difference);
    scalar_sign_report(arithmetic, policy)
}

struct AlgebraicPowerReport {
    arithmetic: Vec<AlgebraicRootArithmeticReport>,
    representation: Option<AlgebraicRootRepresentation>,
    exact: Option<Real>,
}

fn power_representation(value: &AlgebraicRootRepresentation, power: usize) -> AlgebraicPowerReport {
    assert!(power >= 1, "algebraic power must be positive");
    let mut arithmetic = Vec::new();
    let mut representation = Some(value.clone());
    let mut exact = None;
    for _ in 1..power {
        let product = binary_from_report_values(
            representation.as_ref(),
            exact.as_ref(),
            Some(value),
            None,
            AlgebraicRootArithmeticOp::Multiply,
        );
        representation = product.result_representation.clone();
        exact = product.exact_result.clone();
        arithmetic.push(product);
    }
    AlgebraicPowerReport {
        arithmetic,
        representation,
        exact,
    }
}

fn multiply_report_results(
    left: &AlgebraicRootArithmeticReport,
    right: &AlgebraicPowerReport,
) -> AlgebraicRootArithmeticReport {
    binary_from_report_values(
        left.result_representation.as_ref(),
        left.exact_result.as_ref(),
        right.representation.as_ref(),
        right.exact.as_ref(),
        AlgebraicRootArithmeticOp::Multiply,
    )
}

fn multiply(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
) -> AlgebraicRootArithmeticReport {
    arithmetic_algebraic_root_representations(
        left,
        Some(right),
        AlgebraicRootArithmeticOp::Multiply,
    )
}

fn add(
    left_representation: Option<&AlgebraicRootRepresentation>,
    left_exact: Option<&Real>,
    right_representation: Option<&AlgebraicRootRepresentation>,
    right_exact: Option<&Real>,
) -> AlgebraicRootArithmeticReport {
    binary_from_report_values(
        left_representation,
        left_exact,
        right_representation,
        right_exact,
        AlgebraicRootArithmeticOp::Add,
    )
}

fn subtract(
    left_representation: Option<&AlgebraicRootRepresentation>,
    left_exact: Option<&Real>,
    right_representation: Option<&AlgebraicRootRepresentation>,
    right_exact: Option<&Real>,
) -> AlgebraicRootArithmeticReport {
    binary_from_report_values(
        left_representation,
        left_exact,
        right_representation,
        right_exact,
        AlgebraicRootArithmeticOp::Subtract,
    )
}

fn binary_from_report_values(
    left_representation: Option<&AlgebraicRootRepresentation>,
    left_exact: Option<&Real>,
    right_representation: Option<&AlgebraicRootRepresentation>,
    right_exact: Option<&Real>,
    op: AlgebraicRootArithmeticOp,
) -> AlgebraicRootArithmeticReport {
    let left = match representation_or_exact(left_representation, left_exact) {
        Some(value) => value,
        None => return missing_operand_report(op, "left arithmetic operand was absent"),
    };
    let right = match representation_or_exact(right_representation, right_exact) {
        Some(value) => value,
        None => return missing_operand_report(op, "right arithmetic operand was absent"),
    };
    arithmetic_algebraic_root_representations(&left, Some(&right), op)
}

fn representation_or_exact(
    representation: Option<&AlgebraicRootRepresentation>,
    exact: Option<&Real>,
) -> Option<AlgebraicRootRepresentation> {
    representation
        .cloned()
        .or_else(|| exact.map(exact_value_representation))
}

fn exact_value_representation(value: &Real) -> AlgebraicRootRepresentation {
    AlgebraicRootRepresentation {
        constraint_index: 0,
        symbol: hypersolve::SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: vec![-value.clone(), Real::one()],
        interval: hypersolve::IsolatedRootInterval {
            lower: value.clone(),
            upper: value.clone(),
            exact_root: Some(value.clone()),
            distinct_root_count: 1,
        },
        kind: hypersolve::AlgebraicRootKind::ExactRationalWitness,
        validation: hypersolve::AlgebraicRootValidationReport {
            status: hypersolve::AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    }
}

fn missing_operand_report(
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

fn scalar_sign_report(
    arithmetic: Vec<AlgebraicRootArithmeticReport>,
    policy: &CurvePolicy,
) -> BezierAlgebraicScalarSignReport {
    let Some(last) = arithmetic.last() else {
        return BezierAlgebraicScalarSignReport {
            arithmetic,
            scalar: None,
            sign: None,
            message: Some("scalar construction produced no arithmetic reports".to_owned()),
        };
    };
    if !matches!(
        last.status,
        AlgebraicRootArithmeticStatus::ComputedExactRationalWitness
            | AlgebraicRootArithmeticStatus::ComputedRepresentation
    ) {
        return BezierAlgebraicScalarSignReport {
            message: last.message.clone(),
            arithmetic,
            scalar: None,
            sign: None,
        };
    }
    let scalar = match representation_or_exact(
        last.result_representation.as_ref(),
        last.exact_result.as_ref(),
    ) {
        Some(scalar) => scalar,
        None => {
            return BezierAlgebraicScalarSignReport {
                arithmetic,
                scalar: None,
                sign: None,
                message: Some("scalar arithmetic omitted represented result".to_owned()),
            };
        }
    };
    let sign = represented_sign(&scalar, policy);
    let message = sign.is_none().then(|| {
        "represented scalar isolating interval did not certify sign relative to zero".to_owned()
    });
    BezierAlgebraicScalarSignReport {
        arithmetic,
        scalar: Some(scalar),
        sign,
        message,
    }
}

fn represented_sign(value: &AlgebraicRootRepresentation, policy: &CurvePolicy) -> Option<Ordering> {
    if let Some(witness) = value.exact_rational_witness() {
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

#[cfg(feature = "predicates")]
fn refined_represented_sign(
    value: &AlgebraicRootRepresentation,
    policy: &CurvePolicy,
) -> Option<Ordering> {
    let zero = exact_value_representation(&Real::zero());
    let report = compare_algebraic_root_representations_by_difference(
        value,
        &zero,
        AlgebraicRootRefinementComparisonConfig {
            policy: policy.predicate_policy,
            ..AlgebraicRootRefinementComparisonConfig::default()
        },
    );
    (report.comparison.status == AlgebraicRootComparisonStatus::Compared)
        .then_some(report.comparison.ordering)
        .flatten()
}

#[cfg(not(feature = "predicates"))]
fn refined_represented_sign(
    _value: &AlgebraicRootRepresentation,
    _policy: &CurvePolicy,
) -> Option<Ordering> {
    None
}

fn sign_status(report: &BezierAlgebraicScalarSignReport) -> ScalarSignStatus {
    if report.scalar.is_none() {
        return ScalarSignStatus::ArithmeticFailed;
    }
    match report.sign {
        Some(Ordering::Greater) => ScalarSignStatus::Positive,
        Some(Ordering::Less) => ScalarSignStatus::Negative,
        Some(Ordering::Equal) => ScalarSignStatus::Zero,
        None => ScalarSignStatus::Undecided,
    }
}

fn order_report(
    status: BezierAlgebraicTangentOrderStatus,
    ordering: Option<BezierTangentTurnOrdering2>,
    base_first_cross: Option<BezierAlgebraicScalarSignReport>,
    base_second_cross: Option<BezierAlgebraicScalarSignReport>,
    first_second_cross: Option<BezierAlgebraicScalarSignReport>,
    message: Option<String>,
) -> BezierAlgebraicTangentOrderReport {
    BezierAlgebraicTangentOrderReport {
        status,
        ordering,
        base_first_cross,
        base_second_cross,
        first_second_cross,
        message,
    }
}

fn same_tangent_report(
    status: BezierAlgebraicSameTangentOrderStatus,
    ordering: Option<BezierTangentTurnOrdering2>,
    first_curvature_cross: Option<BezierAlgebraicScalarSignReport>,
    second_curvature_cross: Option<BezierAlgebraicScalarSignReport>,
    magnitude_difference: Option<BezierAlgebraicScalarSignReport>,
    message: Option<String>,
) -> BezierAlgebraicSameTangentOrderReport {
    BezierAlgebraicSameTangentOrderReport {
        status,
        ordering,
        first_curvature_cross,
        second_curvature_cross,
        magnitude_difference,
        message,
    }
}
