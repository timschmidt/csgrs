//! Complete-image injectivity certificates for connected native spans.

use std::cmp::Ordering;

use super::Curve2;
use crate::classify::compare_reals;
use crate::{Axis2, Classification, CurveContext, ExactCurveResult, Point2, Real};

impl Curve2 {
    /// Reports whether retained evidence certifies this complete image as injective.
    ///
    /// One native span reuses its existing injectivity certificate. Multiple
    /// spans must meet at exactly equal endpoints and share one strictly
    /// monotone coordinate with the same direction across every span. A source
    /// restriction can inherit injectivity of its complete authored source.
    /// This is a sufficient proof, not a complete self-intersection test.
    ///
    /// `false` means no certificate was established, never that the curve
    /// self-intersects. All decisions are strict, even inside a provisional
    /// scope. Unresolved facts remain retryable; no negative result is cached.
    pub fn has_certified_injective_image(&self) -> ExactCurveResult<bool> {
        if let Some((source, _, _)) = self.source_restriction() {
            return source.has_certified_injective_image();
        }
        let policy = CurveContext::STRICT;
        let Classification::Decided(spans) = self.rational_evaluators_with_policy(&policy)? else {
            return Ok(false);
        };
        let [first, rest @ ..] = spans else {
            return Ok(false);
        };
        if rest.is_empty() {
            return Ok(first.has_certified_injective_axis(&policy));
        }
        // Local injectivity alone cannot rule out overlap between different
        // spans. Exact joins and one common coordinate make their ordered
        // closed coordinate ranges disjoint except at each shared endpoint.
        for pair in spans.windows(2) {
            for axis in [Axis2::X, Axis2::Y] {
                if compare_reals(
                    coordinate(pair[0].end(), axis),
                    coordinate(pair[1].start(), axis),
                    &policy,
                ) != Some(Ordering::Equal)
                {
                    return Ok(false);
                }
            }
        }
        for axis in [Axis2::X, Axis2::Y] {
            let Some(direction @ (Ordering::Less | Ordering::Greater)) = compare_reals(
                coordinate(first.start(), axis),
                coordinate(first.end(), axis),
                &policy,
            ) else {
                continue;
            };
            if spans.iter().all(|span| {
                span.has_certified_injective_axis_on(axis, &policy)
                    && compare_reals(
                        coordinate(span.start(), axis),
                        coordinate(span.end(), axis),
                        &policy,
                    ) == Some(direction)
            }) {
                return Ok(true);
            }
        }
        Ok(false)
    }
}

fn coordinate(point: &Point2, axis: Axis2) -> &Real {
    match axis {
        Axis2::X => point.x(),
        Axis2::Y => point.y(),
    }
}
