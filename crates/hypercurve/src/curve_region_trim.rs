//! Exact top-level curve clipping against native line/arc regions.

use crate::curve_intersection::{CurveIntersectionContext, split_curve_spans};
use crate::{
    BezierSplitFragment2, Classification, Curve2, CurveIntersectionPairBlockerKind2,
    CurveOperation2, CurvePolicy, CurveSpanRange2, ExactCurveError, ExactCurveResult,
    LineArcRegion2, Real, RegionPointLocation, Segment2, UncertaintyReason,
};

/// One retained region-clipped fragment with its source-curve parameter span.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionTrimFragment2 {
    promoted_span_index: usize,
    span_range: CurveSpanRange2,
    fragment: BezierSplitFragment2,
}

impl CurveRegionTrimFragment2 {
    /// Returns the source curve's promoted Bézier span index.
    pub const fn promoted_span_index(&self) -> usize {
        self.promoted_span_index
    }

    /// Returns the source curve's exact public interval for that span.
    pub const fn span_range(&self) -> &CurveSpanRange2 {
        &self.span_range
    }

    /// Returns the retained native or explicitly algebraic split fragment.
    pub const fn fragment(&self) -> &BezierSplitFragment2 {
        &self.fragment
    }

    /// Consumes this record and returns the retained split fragment.
    pub fn into_fragment(self) -> BezierSplitFragment2 {
        self.fragment
    }

    /// Returns the retained boundaries in the top-level public parameter space
    /// when both promoted-span boundaries are represented by [`Real`].
    pub fn represented_parameter_range(&self) -> Option<(Real, Real)> {
        let (local_start, local_end) = self.fragment.parameter_range();
        let (local_start, local_end) = (local_start.as_exact()?, local_end.as_exact()?);
        let (span_start, span_end) = self.span_range.endpoints();
        let span = span_end - span_start;
        Some((
            span_start + &span * local_start,
            span_start + span * local_end,
        ))
    }
}

impl Curve2 {
    /// Retains the positive-length exact fragments of this curve inside a
    /// native line/arc region.
    ///
    /// Every material and hole boundary is intersected with the curve's
    /// promoted rational-Bézier spans. Certified contacts split the source,
    /// then one exact representative per fragment is classified against the
    /// complete region. Shared boundary components and unresolved endpoint
    /// images remain explicit [`ExactCurveError`] blockers.
    pub fn trim_inside_region(
        &self,
        region: &LineArcRegion2,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Vec<BezierSplitFragment2>> {
        self.trim_inside_region_with_parameters(region, policy)
            .map(|fragments| {
                fragments
                    .into_iter()
                    .map(CurveRegionTrimFragment2::into_fragment)
                    .collect()
            })
    }

    /// Retains positive-length exact fragments together with the top-level
    /// source-curve parameter span that generated each fragment.
    ///
    /// This is the authoritative form for consumers that must transfer a trim
    /// back to a corresponding curve in another parameter space. Algebraic
    /// boundaries remain explicit in the embedded [`BezierSplitFragment2`];
    /// [`CurveRegionTrimFragment2::represented_parameter_range`] succeeds only
    /// when both boundaries are materializable as [`Real`].
    pub fn trim_inside_region_with_parameters(
        &self,
        region: &LineArcRegion2,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Vec<CurveRegionTrimFragment2>> {
        if region.is_empty() {
            return Ok(Vec::new());
        }

        let boundaries = region
            .material_contours()
            .iter()
            .chain(region.hole_contours())
            .flat_map(|contour| contour.segments())
            .map(|segment| match segment {
                Segment2::Line(line) => Curve2::from(line.clone()),
                Segment2::Arc(arc) => Curve2::from(arc.clone()),
            })
            .collect::<Vec<_>>();
        let mut split_parameters = Vec::new();
        for boundary in boundaries {
            let context = CurveIntersectionContext::try_new(self, &boundary, policy)?;
            let result = context.result_view()?;
            if let Some(blocker) = result.blockers().first() {
                let reason = match blocker.kind() {
                    CurveIntersectionPairBlockerKind2::Uncertain(reason) => *reason,
                    CurveIntersectionPairBlockerKind2::IncompleteReplay { .. } => {
                        UncertaintyReason::Predicate
                    }
                    CurveIntersectionPairBlockerKind2::SharedComponent => {
                        UncertaintyReason::Boundary
                    }
                };
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Arrangement,
                    self.family(),
                    reason,
                ));
            }
            if !result.overlaps().is_empty() {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Arrangement,
                    self.family(),
                    UncertaintyReason::Boundary,
                ));
            }
            split_parameters.extend(result.contacts().iter().map(|contact| {
                (
                    contact.first().promoted_span_index(),
                    contact.first().local_parameter().clone(),
                )
            }));
        }

        let materializations = split_curve_spans(self, split_parameters.into_iter(), policy)?;
        let native_fragments = self.native_bezier_fragments()?;
        if materializations.len() != native_fragments.len() {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Arrangement,
                self.family(),
                crate::CurveError::Topology(
                    "curve trim span materializations do not match promoted native spans".into(),
                ),
            ));
        }
        let mut retained = Vec::new();
        for (promoted_span_index, (native, materialization)) in
            native_fragments.iter().zip(&materializations).enumerate()
        {
            for fragment in materialization.fragments() {
                let representative =
                    match fragment.representative_point(policy).map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Arrangement, self.family(), cause)
                    })? {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Arrangement,
                                self.family(),
                                reason,
                            ));
                        }
                    };
                match region.classify_point(&representative, policy) {
                    Classification::Decided(RegionPointLocation::Inside) => {
                        retained.push(CurveRegionTrimFragment2 {
                            promoted_span_index,
                            span_range: native.span_range().clone(),
                            fragment: fragment.clone(),
                        });
                    }
                    Classification::Decided(RegionPointLocation::Outside) => {}
                    Classification::Decided(RegionPointLocation::Boundary) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Arrangement,
                            self.family(),
                            UncertaintyReason::Boundary,
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Arrangement,
                            self.family(),
                            reason,
                        ));
                    }
                }
            }
        }
        Ok(retained)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CircularArc2, Contour2, CurvePolicy, LineSeg2, Point2, Real};

    fn p(x: i32, y: i32) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    fn q(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    fn rectangle(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> Contour2 {
        let points = [
            p(min_x, min_y),
            p(max_x, min_y),
            p(max_x, max_y),
            p(min_x, max_y),
        ];
        Contour2::try_new(
            (0..points.len())
                .map(|index| {
                    Segment2::Line(
                        LineSeg2::try_new(
                            points[index].clone(),
                            points[(index + 1) % points.len()].clone(),
                        )
                        .unwrap(),
                    )
                })
                .collect(),
        )
        .unwrap()
    }

    #[test]
    fn exact_curve_trim_splits_a_line_across_material_and_hole_boundaries() {
        let region = LineArcRegion2::new(vec![rectangle(0, 0, 6, 4)], vec![rectangle(2, 1, 4, 3)]);
        let line = Curve2::from(LineSeg2::try_new(p(-1, 2), p(7, 2)).unwrap());
        let fragments = line
            .trim_inside_region(&region, &CurvePolicy::certified())
            .unwrap();
        assert_eq!(fragments.len(), 2);
        assert_eq!(
            fragments[0]
                .representative_point(&CurvePolicy::certified())
                .unwrap(),
            Classification::Decided(p(1, 2))
        );
        assert_eq!(
            fragments[1]
                .representative_point(&CurvePolicy::certified())
                .unwrap(),
            Classification::Decided(p(5, 2))
        );
    }

    #[test]
    fn exact_curve_trim_retains_a_full_circles_right_semicircle() {
        let region = LineArcRegion2::from_material_contours(vec![rectangle(0, -3, 3, 3)]);
        let circle =
            Curve2::from(CircularArc2::try_from_center(p(2, 0), p(2, 0), p(0, 0), false).unwrap());
        let fragments = circle
            .trim_inside_region(&region, &CurvePolicy::certified())
            .unwrap();
        assert_eq!(fragments.len(), 2);
        for fragment in fragments {
            let Classification::Decided(point) = fragment
                .representative_point(&CurvePolicy::certified())
                .unwrap()
            else {
                panic!("retained conic fragment must have an exact representative");
            };
            assert!(matches!(
                region.classify_point(&point, &CurvePolicy::certified()),
                Classification::Decided(RegionPointLocation::Inside)
            ));
        }
    }

    #[test]
    fn parameter_retaining_trim_maps_nurbs_spans_to_the_public_domain() {
        let region = LineArcRegion2::from_material_contours(vec![rectangle(0, 0, 4, 4)]);
        let curve = Curve2::try_nurbs(
            1,
            vec![p(-1, 2), p(7, 2)],
            vec![Real::one(), Real::one()],
            vec![Real::from(2), Real::from(2), Real::from(4), Real::from(4)],
        )
        .unwrap();
        let fragments = curve
            .trim_inside_region_with_parameters(&region, &CurvePolicy::certified())
            .unwrap();
        assert_eq!(fragments.len(), 1);
        assert_eq!(fragments[0].promoted_span_index(), 0);
        let (start, end) = fragments[0]
            .represented_parameter_range()
            .expect("linear boundary roots are represented exactly");
        assert_eq!(
            crate::classify::compare_reals(&start, &q(9, 4), &CurvePolicy::certified()),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            crate::classify::compare_reals(&end, &q(13, 4), &CurvePolicy::certified()),
            Some(std::cmp::Ordering::Equal)
        );
    }
}
