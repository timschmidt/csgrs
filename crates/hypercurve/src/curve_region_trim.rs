//! Exact top-level curve clipping against unified curve regions.

use crate::curve_intersection::{CurveIntersectionContext, split_curve_spans};
use crate::policy::resolve_certified_operation;
use crate::{
    BezierParameter2, BezierSplitFragment2, Classification, Curve2, CurveContext,
    CurveIntersectionPairBlockerKind2, CurveIntersectionParameter2, CurveOperation2, CurveOutcome,
    CurveRegion2, CurveSpanRange2, ExactCurveError, ExactCurveResult,
    RationalBezierIntersectionPointEvidence2, Real, RegionPointLocation, UncertaintyReason,
};

/// Which authored region boundary owns one exact trim contact.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum CurveRegionBoundaryKind2 {
    /// A material contour boundary.
    Material,
    /// A hole contour boundary.
    Hole,
}

/// Exact evidence that one retained trim endpoint lies on an authored region segment.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionBoundaryContact2 {
    kind: CurveRegionBoundaryKind2,
    contour_index: usize,
    segment_index: usize,
    boundary_parameter: CurveIntersectionParameter2,
    point: RationalBezierIntersectionPointEvidence2,
}

impl CurveRegionBoundaryContact2 {
    /// Returns whether the contacted segment belongs to a material or hole contour.
    pub const fn kind(&self) -> CurveRegionBoundaryKind2 {
        self.kind
    }

    /// Returns the contour index within the selected boundary kind.
    pub const fn contour_index(&self) -> usize {
        self.contour_index
    }

    /// Returns the segment index in the authored contour.
    pub const fn segment_index(&self) -> usize {
        self.segment_index
    }

    /// Returns the exact parameter evidence on the contacted boundary segment.
    pub const fn boundary_parameter(&self) -> &CurveIntersectionParameter2 {
        &self.boundary_parameter
    }

    /// Returns retained exact point evidence for the contact.
    pub const fn point(&self) -> &RationalBezierIntersectionPointEvidence2 {
        &self.point
    }
}

#[derive(Clone)]
struct PendingBoundaryContact {
    promoted_span_index: usize,
    source_parameter: BezierParameter2,
    contact: CurveRegionBoundaryContact2,
}

/// One retained region-clipped fragment with its source-curve parameter span.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionTrimFragment2 {
    promoted_span_index: usize,
    span_range: CurveSpanRange2,
    fragment: BezierSplitFragment2,
    start_boundary_contacts: Vec<CurveRegionBoundaryContact2>,
    end_boundary_contacts: Vec<CurveRegionBoundaryContact2>,
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

    /// Returns every authored region segment proved incident to the fragment start.
    pub fn start_boundary_contacts(&self) -> &[CurveRegionBoundaryContact2] {
        &self.start_boundary_contacts
    }

    /// Returns every authored region segment proved incident to the fragment end.
    pub fn end_boundary_contacts(&self) -> &[CurveRegionBoundaryContact2] {
        &self.end_boundary_contacts
    }

    /// Returns the retained boundaries in the top-level public parameter space
    /// when both promoted-span boundaries are represented by [`Real`].
    pub fn represented_parameter_range(&self) -> Option<(Real, Real)> {
        let (local_start, local_end) = self.fragment.parameter_range()?;
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
    /// Retains the positive-length exact fragments of this curve inside a region.
    ///
    /// Every material and hole boundary is intersected with the curve's
    /// promoted rational-Bézier spans. Certified contacts split the source,
    /// then one exact representative per fragment is classified against the
    /// complete region. Shared boundary components and unresolved endpoint
    /// images remain explicit [`ExactCurveError`] blockers.
    pub fn trim_inside_region(
        &self,
        region: &CurveRegion2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<BezierSplitFragment2>>> {
        resolve_certified_operation(policy, |attempt| {
            self.trim_inside_region_with_parameters_raw(region, attempt)
                .map(|fragments| {
                    fragments
                        .into_iter()
                        .map(CurveRegionTrimFragment2::into_fragment)
                        .collect()
                })
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
        region: &CurveRegion2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveRegionTrimFragment2>>> {
        resolve_certified_operation(policy, |attempt| {
            self.trim_inside_region_with_parameters_raw(region, attempt)
        })
    }

    fn trim_inside_region_with_parameters_raw(
        &self,
        region: &CurveRegion2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveRegionTrimFragment2>> {
        if region.is_empty() {
            return Ok(Vec::new());
        }

        let roles = match region.loop_roles_raw(policy).map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Subdivision, self.family(), cause)
        })? {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Subdivision,
                    self.family(),
                    reason,
                ));
            }
        };
        if roles.len() != region.boundary_loops().len() {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Subdivision,
                self.family(),
                crate::CurveError::Topology(
                    "curve trim region roles do not match its boundary loops".into(),
                ),
            ));
        }

        let mut split_parameters = Vec::new();
        let mut boundary_contacts = Vec::new();
        let mut material_contour_index = 0_usize;
        let mut hole_contour_index = 0_usize;
        for (boundary_loop, role) in region.boundary_loops().iter().zip(roles) {
            let (kind, contour_index) = match role {
                crate::CurveRegionLoopRole::Material => {
                    let index = material_contour_index;
                    material_contour_index += 1;
                    (CurveRegionBoundaryKind2::Material, index)
                }
                crate::CurveRegionLoopRole::Hole => {
                    let index = hole_contour_index;
                    hole_contour_index += 1;
                    (CurveRegionBoundaryKind2::Hole, index)
                }
            };
            for (segment_index, fragment) in boundary_loop.fragments().iter().enumerate() {
                let BezierSplitFragment2::Materialized { curve, .. } = fragment else {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Subdivision,
                        self.family(),
                        UncertaintyReason::Unsupported,
                    ));
                };
                let boundary = Curve2::from(curve.clone());
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
                        CurveOperation2::Subdivision,
                        self.family(),
                        reason,
                    ));
                }
                if !result.overlaps().is_empty() {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Subdivision,
                        self.family(),
                        UncertaintyReason::Boundary,
                    ));
                }
                for contact in result.contacts() {
                    split_parameters.push((
                        contact.first().promoted_span_index(),
                        contact.first().local_parameter().clone(),
                    ));
                    boundary_contacts.push(PendingBoundaryContact {
                        promoted_span_index: contact.first().promoted_span_index(),
                        source_parameter: contact.first().local_parameter().clone(),
                        contact: CurveRegionBoundaryContact2 {
                            kind,
                            contour_index,
                            segment_index,
                            boundary_parameter: contact.second().clone(),
                            point: contact.point().clone(),
                        },
                    });
                }
            }
        }

        let materializations = split_curve_spans(self, split_parameters.into_iter(), policy)?;
        let native_fragments =
            self.native_bezier_fragments_for_operation(policy, CurveOperation2::Subdivision)?;
        if materializations.len() != native_fragments.len() {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Subdivision,
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
                        ExactCurveError::invalid(CurveOperation2::Subdivision, self.family(), cause)
                    })? {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Subdivision,
                                self.family(),
                                reason,
                            ));
                        }
                    };
                match region
                    .classify_point_raw(&representative, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Subdivision, self.family(), cause)
                    })? {
                    Classification::Decided(RegionPointLocation::Inside) => {
                        let Some((start, end)) = fragment.parameter_range() else {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Subdivision,
                                self.family(),
                                UncertaintyReason::Unsupported,
                            ));
                        };
                        retained.push(CurveRegionTrimFragment2 {
                            promoted_span_index,
                            span_range: native.span_range().clone(),
                            fragment: fragment.clone(),
                            start_boundary_contacts: boundary_contacts_at(
                                &boundary_contacts,
                                promoted_span_index,
                                start,
                                self,
                                policy,
                            )?,
                            end_boundary_contacts: boundary_contacts_at(
                                &boundary_contacts,
                                promoted_span_index,
                                end,
                                self,
                                policy,
                            )?,
                        });
                    }
                    Classification::Decided(RegionPointLocation::Outside) => {}
                    Classification::Decided(RegionPointLocation::Boundary) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Subdivision,
                            self.family(),
                            UncertaintyReason::Boundary,
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Subdivision,
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

fn boundary_contacts_at(
    contacts: &[PendingBoundaryContact],
    promoted_span_index: usize,
    parameter: &BezierParameter2,
    source: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<CurveRegionBoundaryContact2>> {
    let mut matched = Vec::new();
    for pending in contacts
        .iter()
        .filter(|pending| pending.promoted_span_index == promoted_span_index)
    {
        match pending
            .source_parameter
            .cmp_by_refinement(parameter, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Subdivision, source.family(), cause)
            })? {
            Classification::Decided(std::cmp::Ordering::Equal) => {
                matched.push(pending.contact.clone());
            }
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Subdivision,
                    source.family(),
                    reason,
                ));
            }
        }
    }
    Ok(matched)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        CircularArc2, Contour2, CurveBoundaryInteriorSide2, CurveCertainty, CurveContext,
        CurvePath2, CurveRegionLoopRole, FillRule, LineSeg2, Point2, QuadraticBezier2, Real,
        Segment2,
    };

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

    fn native_region(material: Vec<Contour2>, holes: Vec<Contour2>) -> CurveRegion2 {
        CurveRegion2::try_from_native_contours(material, holes, &CurveContext::STRICT)
            .unwrap()
            .into_value()
    }

    #[test]
    fn exact_curve_trim_intersects_materialized_quadratic_region_boundaries() {
        let path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(4, 0), p(4, 4)).unwrap()),
            Curve2::from(QuadraticBezier2::new(p(4, 4), p(2, 6), p(0, 4))),
            Curve2::from(LineSeg2::try_new(p(0, 4), p(0, 0)).unwrap()),
        ])
        .unwrap();
        let region = CurveRegion2::try_from_boundary_paths_with_loop_topology(
            &[path],
            &[CurveRegionLoopRole::Material],
            &[FillRule::NonZero],
            &[CurveBoundaryInteriorSide2::Left],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
        let line = Curve2::from(LineSeg2::try_new(p(2, -1), p(2, 6)).unwrap());

        let outcome = line
            .trim_inside_region_with_parameters(&region, &CurveContext::APPROXIMATE_512)
            .unwrap();
        assert_eq!(outcome.certainty, CurveCertainty::Certified);
        assert_eq!(outcome.value.len(), 1);
        assert_eq!(
            outcome.value[0].start_boundary_contacts()[0].segment_index(),
            0
        );
        assert_eq!(
            outcome.value[0].end_boundary_contacts()[0].segment_index(),
            2
        );
    }

    #[test]
    fn exact_curve_trim_splits_a_line_across_material_and_hole_boundaries() {
        let region = native_region(vec![rectangle(0, 0, 6, 4)], vec![rectangle(2, 1, 4, 3)]);
        let line = Curve2::from(LineSeg2::try_new(p(-1, 2), p(7, 2)).unwrap());
        let fragments = line
            .trim_inside_region(&region, &CurveContext::STRICT)
            .unwrap()
            .into_value();
        assert_eq!(fragments.len(), 2);
        assert_eq!(
            fragments[0]
                .representative_point(&CurveContext::STRICT)
                .unwrap(),
            Classification::Decided(p(1, 2))
        );
        assert_eq!(
            fragments[1]
                .representative_point(&CurveContext::STRICT)
                .unwrap(),
            Classification::Decided(p(5, 2))
        );
    }

    #[test]
    fn parameter_retaining_trim_reports_authored_boundary_provenance() {
        let region = native_region(vec![rectangle(0, 0, 6, 4)], vec![rectangle(2, 1, 4, 3)]);
        let line = Curve2::from(LineSeg2::try_new(p(-1, 2), p(7, 2)).unwrap());
        let fragments = line
            .trim_inside_region_with_parameters(&region, &CurveContext::STRICT)
            .unwrap()
            .into_value();

        assert_eq!(fragments.len(), 2);
        let expected = [
            (CurveRegionBoundaryKind2::Material, 3, p(0, 2)),
            (CurveRegionBoundaryKind2::Hole, 3, p(2, 2)),
            (CurveRegionBoundaryKind2::Hole, 1, p(4, 2)),
            (CurveRegionBoundaryKind2::Material, 1, p(6, 2)),
        ];
        let contacts = [
            &fragments[0].start_boundary_contacts()[0],
            &fragments[0].end_boundary_contacts()[0],
            &fragments[1].start_boundary_contacts()[0],
            &fragments[1].end_boundary_contacts()[0],
        ];
        for (contact, (kind, segment_index, point)) in contacts.into_iter().zip(expected) {
            assert_eq!(contact.kind(), kind);
            assert_eq!(contact.contour_index(), 0);
            assert_eq!(contact.segment_index(), segment_index);
            assert_eq!(
                contact.point(),
                &RationalBezierIntersectionPointEvidence2::Exact(point)
            );
            assert!(
                contact
                    .boundary_parameter()
                    .exact_curve_parameter()
                    .is_some()
            );
        }
    }

    #[test]
    fn exact_curve_trim_retains_a_full_circles_right_semicircle() {
        let region = native_region(vec![rectangle(0, -3, 3, 3)], Vec::new());
        let circle =
            Curve2::from(CircularArc2::try_from_center(p(2, 0), p(2, 0), p(0, 0), false).unwrap());
        let fragments = circle
            .trim_inside_region(&region, &CurveContext::STRICT)
            .unwrap()
            .into_value();
        assert_eq!(fragments.len(), 2);
        for fragment in fragments {
            let Classification::Decided(point) = fragment
                .representative_point(&CurveContext::STRICT)
                .unwrap()
            else {
                panic!("retained conic fragment must have an exact representative");
            };
            assert!(matches!(
                region
                    .classify_point(&point, &CurveContext::STRICT)
                    .unwrap()
                    .into_value(),
                Classification::Decided(RegionPointLocation::Inside)
            ));
        }
    }

    #[test]
    fn parameter_retaining_trim_maps_nurbs_spans_to_the_public_domain() {
        let region = native_region(vec![rectangle(0, 0, 4, 4)], Vec::new());
        let curve = Curve2::try_nurbs(
            1,
            vec![p(-1, 2), p(7, 2)],
            vec![Real::one(), Real::one()],
            vec![Real::from(2), Real::from(2), Real::from(4), Real::from(4)],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
        let fragments = curve
            .trim_inside_region_with_parameters(&region, &CurveContext::STRICT)
            .unwrap()
            .into_value();
        assert_eq!(fragments.len(), 1);
        assert_eq!(fragments[0].promoted_span_index(), 0);
        let (start, end) = fragments[0]
            .represented_parameter_range()
            .expect("linear boundary roots are represented exactly");
        assert_eq!(
            crate::classify::compare_reals(&start, &q(9, 4), &CurveContext::STRICT),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            crate::classify::compare_reals(&end, &q(13, 4), &CurveContext::STRICT),
            Some(std::cmp::Ordering::Equal)
        );
    }
}
