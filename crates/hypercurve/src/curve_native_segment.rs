//! Source-bound proofs of complete native line and circular-arc images.

use std::cmp::Ordering;

use crate::bezier_region::{
    materialized_native_subcurve_segment, retained_native_fragment_segment,
};
use crate::policy::resolve_certified_operation;
use crate::{
    BezierParallelSource2, BezierParameter2, BezierSplitFragment2, BezierSubcurve2, Classification,
    Curve2, CurveContext, CurveGeometry2, CurveOperation2, CurveResult, ExactCurveError,
    ExactCurveResult, Real, Segment2, UncertaintyReason,
};

/// A proved native segment image that retains its exact source curve.
///
/// The segment covers the source's complete finite image in traversal order.
/// Its parameter chart need not be the source chart: an algebraic chord, for
/// example, retains a selected coordinate parameter while a native line uses
/// `[0, 1]`. Use [`Self::source`] for original parameters and provenance.
/// This view neither replaces the source nor certifies an earlier approximate
/// construction's fidelity. It contains no persistent cache.
#[derive(Clone, Debug)]
pub struct CurveNativeSegment2<'a> {
    source: &'a Curve2,
    segment: Segment2,
}

impl<'a> CurveNativeSegment2<'a> {
    /// Borrows the original carrier, including its retained parameter evidence.
    pub const fn source(&self) -> &'a Curve2 {
        self.source
    }

    /// Borrows the proved native line or circular arc in its own parameter chart.
    pub const fn segment(&self) -> &Segment2 {
        &self.segment
    }
}

impl Curve2 {
    /// Proves this complete finite curve has a native line or circular-arc image.
    ///
    /// Native line/arc carriers return directly. A recognized single Bezier
    /// span, retained chord, regular line parallel, or supported selected
    /// restriction reuses the shared exact native-image proof. Restricted
    /// Bezier sources additionally need a certified injective unit image and
    /// range containment; a support fit alone cannot justify replacing a
    /// nonmonotone trim by the segment between its endpoints.
    ///
    /// Proof search is strict even inside an ambient approximate scope. A
    /// missing representation or sufficient traversal proof is an unsupported
    /// blocker, not evidence that the curve is nonlinear. Unresolved decisions
    /// remain typed errors. Multiple native spans are not merged by this query.
    pub fn native_segment(&self) -> ExactCurveResult<CurveNativeSegment2<'_>> {
        let segment = resolve_certified_operation(&CurveContext::STRICT, |policy| {
            self.native_segment_raw(policy)
        })?
        .into_value();
        Ok(CurveNativeSegment2 {
            source: self,
            segment,
        })
    }

    fn native_segment_raw(&self, policy: &CurveContext) -> ExactCurveResult<Segment2> {
        match self.geometry() {
            Some(CurveGeometry2::Line(line)) => return Ok(Segment2::Line(line.clone())),
            Some(CurveGeometry2::CircularArc(arc)) => return Ok(Segment2::Arc(arc.clone())),
            _ => {}
        }
        let result = if let Some(fragment) = self.retained_fragment() {
            certified_fragment_segment(fragment, policy)
        } else if let Some(spans) =
            self.restricted_source_spans(policy, CurveOperation2::NativeTopology)?
        {
            let [span] = spans else {
                return Err(self.native_segment_blocker(UncertaintyReason::Unsupported));
            };
            certified_fragment_segment(&span.fragment, policy)
        } else {
            let spans = self
                .native_bezier_fragments_for_operation(policy, CurveOperation2::NativeTopology)?;
            let [span] = spans else {
                return Err(self.native_segment_blocker(UncertaintyReason::Unsupported));
            };
            certified_native_span_segment(span.native_curve(), policy)
        };
        match result.map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::NativeTopology, self.family(), cause)
        })? {
            Classification::Decided(segment) => Ok(segment),
            Classification::Uncertain(reason) => Err(self.native_segment_blocker(reason)),
        }
    }

    fn native_segment_blocker(&self, reason: UncertaintyReason) -> ExactCurveError {
        ExactCurveError::blocked(CurveOperation2::NativeTopology, self.family(), reason)
    }
}

fn certified_native_span_segment(
    source: &BezierSubcurve2,
    policy: &CurveContext,
) -> CurveResult<Classification<Segment2>> {
    match source.certified_injective_image(policy)? {
        Classification::Decided(true) => materialized_native_subcurve_segment(source, policy),
        Classification::Decided(false) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn source_interval_certificate(
    source: &BezierSubcurve2,
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    for (lower, upper) in [
        (&BezierParameter2::Exact(Real::zero()), start),
        (end, &BezierParameter2::Exact(Real::one())),
    ] {
        match lower.cmp_by_refinement_with_policy(upper, policy)? {
            Classification::Decided(Ordering::Less | Ordering::Equal) => {}
            Classification::Decided(Ordering::Greater) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    Ok(match source.certified_injective_image(policy)? {
        Classification::Decided(true) => Classification::Decided(()),
        Classification::Decided(false) => Classification::Uncertain(UncertaintyReason::Unsupported),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

fn certified_fragment_segment(
    fragment: &BezierSplitFragment2,
    policy: &CurveContext,
) -> CurveResult<Classification<Segment2>> {
    let range_proof = match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => {
            return certified_native_span_segment(curve, policy);
        }
        BezierSplitFragment2::RetainedBezier {
            source_curve,
            start,
            end,
            ..
        } => source_interval_certificate(source_curve, start, end, policy)?,
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            let source = match fragment.parallel().source() {
                BezierParallelSource2::Quadratic(curve) => {
                    BezierSubcurve2::Quadratic(curve.clone())
                }
                BezierParallelSource2::Cubic(curve) => BezierSubcurve2::Cubic(curve.clone()),
                BezierParallelSource2::Rational(curve) => BezierSubcurve2::Rational(curve.clone()),
            };
            source_interval_certificate(
                &source,
                fragment.range().start(),
                fragment.range().end(),
                policy,
            )?
        }
        // A chord's defining carrier is exactly the complete segment. The
        // selected-fiber native helper owns its range and monotonicity proof.
        BezierSplitFragment2::AlgebraicChord(_)
        | BezierSplitFragment2::SelectedFiber(_)
        | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => Classification::Decided(()),
    };
    match range_proof {
        Classification::Decided(()) => retained_native_fragment_segment(fragment, policy),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}
