//! Exact curve points independent of their construction history.

use crate::classify::is_zero;
use crate::{
    Aabb2, Axis2, Classification, CurveContext, CurveOutcome, CurveResult, Point2,
    RationalBezierAlgebraicPointImage2, UncertaintyReason,
};
use hypersolve::AlgebraicRootRepresentation;

/// An exact affine point with shared scalar or selected geometric evidence.
///
/// A point need not have independent `Real` coordinates. Predicates reuse its
/// retained supports, selected roots, and correlations directly.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePoint2(pub(crate) CurvePointData2);

impl CurvePoint2 {
    /// Returns stored `Real` coordinates when they are available.
    ///
    /// This view does not reconstruct selected coordinate images. Absence of
    /// this view does not limit the exact meaning of the point.
    pub const fn coordinates(&self) -> Option<&Point2> {
        match &self.0 {
            CurvePointData2::Exact(point) => Some(point),
            _ => None,
        }
    }

    /// Compares the geometric positions while retaining all selected evidence.
    ///
    /// Structural `PartialEq` is a positive identity check. This query also
    /// compares points created by independent constructions and reports any
    /// predicate that the requested policy cannot certify.
    pub fn coincides_with(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveOutcome<Classification<bool>> {
        crate::policy::resolve_certified_value(policy, |attempt| self.same_point(other, attempt))
    }

    /// Compares one coordinate using retained exact evidence.
    pub fn compare_coordinate(
        &self,
        other: &Self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<std::cmp::Ordering>>> {
        crate::policy::resolve_certified_operation(policy, |attempt| {
            crate::bezier_offset::algebraic_chord_point_coordinate_order(self, other, axis, attempt)
        })
    }

    /// Returns a conservative exact enclosure of this point.
    ///
    /// A selected point may produce a nonzero box. The box encloses the exact
    /// point and does not replace its retained scalar or geometric evidence.
    pub fn bounds(&self, policy: &CurveContext) -> CurveOutcome<Classification<Aabb2>> {
        crate::policy::resolve_certified_value(policy, |attempt| {
            crate::bezier_offset::algebraic_chord_endpoint_bounds_refined(self, 0, attempt)
        })
    }
}

macro_rules! point_from_evidence {
    ($($source:ty => $variant:ident),* $(,)?) => {
        $(impl From<$source> for CurvePoint2 {
            fn from(point: $source) -> Self {
                Self(CurvePointData2::$variant(point))
            }
        })*
    };
}

point_from_evidence! {
    Point2 => Exact,
    RationalBezierAlgebraicPointImage2 => Algebraic,
    crate::BezierAlgebraicChordPairPoint2 => AlgebraicChordPair,
    crate::BezierAlgebraicCuspChordPoint2 => AlgebraicCuspChord,
    crate::BezierAlgebraicCuspChordDerivedPoint2 => AlgebraicCuspChordDerived,
    crate::BezierAlgebraicChordParallelPoint2 => AlgebraicChordParallel,
    crate::BezierAnalyticParallelPoint2 => AnalyticParallel,
    crate::BezierSimilarityPoint2 => Similarity,
}

/// Exact affine point evidence retained for a curve contact.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum CurvePointData2 {
    /// The contact point is represented directly by [`Real`](crate::Real) coordinates.
    Exact(Point2),
    /// The contact point is retained as exact algebraic point evidence.
    ///
    /// A retained rational-expression status may defer coordinate images
    /// while preserving the exact source curve and parameter.
    Algebraic(RationalBezierAlgebraicPointImage2),
    /// A unique nonparallel intersection of two retained algebraic chords.
    ///
    /// The four endpoint fields remain separate and are refined only when a
    /// coordinate comparison or enclosure is requested.
    AlgebraicChordPair(crate::BezierAlgebraicChordPairPoint2),
    /// A selected algebraic-circle contact with a certified axis-aligned
    /// retained chord.  Both selected fields and the square-root branch remain
    /// exact until a terminal predicate policy permits approximation.
    AlgebraicCuspChord(crate::BezierAlgebraicCuspChordPoint2),
    /// An exact affine derivative of a retained selected-circle/axis-chord
    /// contact, such as one endpoint of an axis-aligned parallel.
    AlgebraicCuspChordDerived(crate::BezierAlgebraicCuspChordDerivedPoint2),
    /// One endpoint displaced along an exact unit normal or tangent of a
    /// retained algebraic chord whose normalized direction spans selected
    /// endpoint fields.
    AlgebraicChordParallel(crate::BezierAlgebraicChordParallelPoint2),
    /// One exact point on an analytic Bezier parallel at a retained source
    /// parameter. The normalized direction is evaluated only by predicates.
    AnalyticParallel(crate::BezierAnalyticParallelPoint2),
    /// One exact retained point transported by a certified planar similarity.
    ///
    /// The source evidence remains correlated and is evaluated lazily rather
    /// than flattened into independently reconstructed coordinates.
    Similarity(crate::BezierSimilarityPoint2),
}

impl CurvePoint2 {
    /// Returns a constant-time positive identity certificate for retained
    /// point evidence. Distinct storage may still describe the same point and
    /// must continue through the exact geometric predicates.
    pub(crate) fn shares_storage(&self, other: &Self) -> bool {
        match (self, other) {
            (Self(CurvePointData2::Exact(first)), Self(CurvePointData2::Exact(second))) => {
                first.shares_storage(second)
            }
            (Self(CurvePointData2::Algebraic(first)), Self(CurvePointData2::Algebraic(second))) => {
                first.shares_storage(second)
            }
            (
                Self(CurvePointData2::AlgebraicChordPair(_)),
                Self(CurvePointData2::AlgebraicChordPair(_)),
            ) => false,
            (
                Self(CurvePointData2::AlgebraicCuspChord(first)),
                Self(CurvePointData2::AlgebraicCuspChord(second)),
            ) => first.shares_storage(second),
            (
                Self(CurvePointData2::AlgebraicCuspChordDerived(first)),
                Self(CurvePointData2::AlgebraicCuspChordDerived(second)),
            ) => first.shares_storage(second),
            (
                Self(CurvePointData2::AlgebraicChordParallel(first)),
                Self(CurvePointData2::AlgebraicChordParallel(second)),
            ) => first.shares_storage(second),
            (
                Self(CurvePointData2::AnalyticParallel(first)),
                Self(CurvePointData2::AnalyticParallel(second)),
            ) => first.shares_storage(second),
            (
                Self(CurvePointData2::Similarity(first)),
                Self(CurvePointData2::Similarity(second)),
            ) => first.shares_storage(second),
            _ => false,
        }
    }

    /// Returns the retained algebraic image, when present.
    pub(crate) const fn as_algebraic(&self) -> Option<&RationalBezierAlgebraicPointImage2> {
        match &self.0 {
            CurvePointData2::Algebraic(point) => Some(point),
            _ => None,
        }
    }

    /// Returns retained correlated cusp/chord point evidence, when present.
    pub(crate) const fn as_algebraic_cusp_chord(
        &self,
    ) -> Option<&crate::BezierAlgebraicCuspChordPoint2> {
        match &self.0 {
            CurvePointData2::AlgebraicCuspChord(point) => Some(point),
            _ => None,
        }
    }

    /// Compares two retained affine points without materializing an algebraic
    /// coordinate or sampling either isolating interval.
    ///
    /// Exact points use the canonical [`Point2`] predicate. Algebraic images
    /// first reuse shared parametric provenance and disjoint source bounds,
    /// then compare represented coordinate roots. Correlated chord contacts
    /// retain their defining supports and refine enclosures without composing
    /// endpoint fields. Any predicate that remains unproved stays explicit
    /// under `policy`.
    pub(crate) fn same_point(&self, other: &Self, policy: &CurveContext) -> Classification<bool> {
        if self.shares_storage(other) {
            return Classification::Decided(true);
        }
        let recursive_composite = |point: &Self| {
            matches!(
                point,
                Self(CurvePointData2::AlgebraicChordPair(_))
                    | Self(CurvePointData2::AlgebraicCuspChord(_))
                    | Self(CurvePointData2::AlgebraicCuspChordDerived(_))
                    | Self(CurvePointData2::AlgebraicChordParallel(_))
                    | Self(CurvePointData2::Similarity(_))
            )
        };
        if (recursive_composite(self) || recursive_composite(other))
            && !matches!(self, Self(CurvePointData2::AnalyticParallel(_)))
            && !matches!(other, Self(CurvePointData2::AnalyticParallel(_)))
            && let Ok(Classification::Decided(Some(equal))) =
                crate::bezier_offset::recursive_projective_point_evidence_equality(
                    self, other, policy,
                )
        {
            return Classification::Decided(equal);
        }
        match (self, other) {
            (Self(CurvePointData2::Exact(first)), Self(CurvePointData2::Exact(second))) => {
                match is_zero(&first.distance_squared(second), policy) {
                    Some(equal) => Classification::Decided(equal),
                    None => Classification::Uncertain(UncertaintyReason::RealSign),
                }
            }
            (Self(CurvePointData2::Algebraic(first)), Self(CurvePointData2::Algebraic(second))) => {
                if let Some(classification) =
                    first.same_injective_parametric_source_point(second, policy)
                {
                    return classification;
                }
                if let (
                    Some(Classification::Decided(first_bounds)),
                    Some(Classification::Decided(second_bounds)),
                ) = (
                    first.parametric_source_bounds(policy),
                    second.parametric_source_bounds(policy),
                ) && first_bounds.overlaps(&second_bounds, policy)
                    == Classification::Decided(false)
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "contact-point-equality",
                        "source-bounds-disjoint",
                    );
                    return Classification::Decided(false);
                }
                if let Ok(Some(classification)) = first.same_retained_rational_point(second, policy)
                {
                    return classification;
                }
                let (Some(first), Some(second)) = (first.resolved(policy), second.resolved(policy))
                else {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                };
                let (Some(first_x), Some(first_y), Some(second_x), Some(second_y)) = (
                    first.x().and_then(|image| image.representation()),
                    first.y().and_then(|image| image.representation()),
                    second.x().and_then(|image| image.representation()),
                    second.y().and_then(|image| image.representation()),
                ) else {
                    return if first == second {
                        Classification::Decided(true)
                    } else {
                        Classification::Uncertain(UncertaintyReason::Unsupported)
                    };
                };
                match (
                    crate::bezier_arrangement::represented_roots_equal(first_x, second_x, policy),
                    crate::bezier_arrangement::represented_roots_equal(first_y, second_y, policy),
                ) {
                    (Some(x_equal), Some(y_equal)) => Classification::Decided(x_equal && y_equal),
                    _ => Classification::Uncertain(UncertaintyReason::RealSign),
                }
            }
            (Self(CurvePointData2::Exact(exact)), Self(CurvePointData2::Algebraic(algebraic)))
            | (Self(CurvePointData2::Algebraic(algebraic)), Self(CurvePointData2::Exact(exact))) => {
                if let (Ok(x), Ok(y)) = (
                    algebraic.coordinate_order_to_real(true, exact.x(), policy),
                    algebraic.coordinate_order_to_real(false, exact.y(), policy),
                ) {
                    match (x, y) {
                        (
                            Classification::Decided(std::cmp::Ordering::Equal),
                            Classification::Decided(std::cmp::Ordering::Equal),
                        ) => return Classification::Decided(true),
                        (
                            Classification::Decided(
                                std::cmp::Ordering::Less | std::cmp::Ordering::Greater,
                            ),
                            _,
                        )
                        | (
                            _,
                            Classification::Decided(
                                std::cmp::Ordering::Less | std::cmp::Ordering::Greater,
                            ),
                        ) => return Classification::Decided(false),
                        _ => {}
                    }
                }
                let Some(algebraic) = algebraic.resolved(policy) else {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                };
                let (Some(x), Some(y)) = (
                    algebraic.x().and_then(|image| image.representation()),
                    algebraic.y().and_then(|image| image.representation()),
                ) else {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                };
                let exact_x = AlgebraicRootRepresentation::from_exact_value(exact.x());
                let exact_y = AlgebraicRootRepresentation::from_exact_value(exact.y());
                match (
                    crate::bezier_arrangement::represented_roots_equal(x, &exact_x, policy),
                    crate::bezier_arrangement::represented_roots_equal(y, &exact_y, policy),
                ) {
                    (Some(x_equal), Some(y_equal)) => Classification::Decided(x_equal && y_equal),
                    _ => Classification::Uncertain(UncertaintyReason::RealSign),
                }
            }
            (
                Self(CurvePointData2::AlgebraicChordPair(first)),
                Self(CurvePointData2::AlgebraicChordPair(second)),
            ) => first.same_point(second, policy),
            (Self(CurvePointData2::AlgebraicChordPair(point)), other)
            | (other, Self(CurvePointData2::AlgebraicChordPair(point))) => {
                point.same_point_evidence(other, policy)
            }
            (
                Self(CurvePointData2::AlgebraicCuspChord(first)),
                Self(CurvePointData2::AlgebraicCuspChord(second)),
            ) => first.same_point_evidence(&Self::from(second.clone()), policy),
            (Self(CurvePointData2::AlgebraicCuspChord(point)), other)
            | (other, Self(CurvePointData2::AlgebraicCuspChord(point))) => {
                point.same_point_evidence(other, policy)
            }
            (
                Self(CurvePointData2::AlgebraicCuspChordDerived(first)),
                Self(CurvePointData2::AlgebraicCuspChordDerived(second)),
            ) => first.same_point(second, policy),
            (Self(CurvePointData2::AlgebraicCuspChordDerived(point)), other)
            | (other, Self(CurvePointData2::AlgebraicCuspChordDerived(point))) => {
                point.same_point_evidence(other, policy)
            }
            (Self(CurvePointData2::AlgebraicChordParallel(point)), other)
            | (other, Self(CurvePointData2::AlgebraicChordParallel(point))) => {
                point.same_point_evidence(other, policy)
            }
            (Self(CurvePointData2::AnalyticParallel(point)), other)
            | (other, Self(CurvePointData2::AnalyticParallel(point))) => {
                point.same_point_evidence(other, policy)
            }
            (Self(CurvePointData2::Similarity(point)), other)
            | (other, Self(CurvePointData2::Similarity(point))) => {
                point.same_point_evidence(other, policy)
            }
        }
    }
}
