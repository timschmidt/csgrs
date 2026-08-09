//! Exact top-level curve clipping against unified curve regions.

use crate::curve_intersection::split_curve_spans;
use crate::policy::resolve_certified_operation;
use crate::{
    BezierParameter2, BezierSplitFragment2, Classification, Curve2, CurveContext,
    CurveIntersectionPairBlockerKind2, CurveOperation2, CurveOutcome, CurvePath2, CurveRegion2,
    CurveRegionParameter2, CurveSpanRange2, ExactCurveError, ExactCurveResult,
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
    boundary_parameter: CurveRegionParameter2,
    point: Option<RationalBezierIntersectionPointEvidence2>,
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
    pub const fn boundary_parameter(&self) -> &CurveRegionParameter2 {
        &self.boundary_parameter
    }

    /// Returns retained exact point evidence when the pair kernel materializes it.
    ///
    /// Some cross-field analytic contacts are represented completely by their
    /// two selected parameters and therefore have no standalone Cartesian point.
    pub const fn point(&self) -> Option<&RationalBezierIntersectionPointEvidence2> {
        self.point.as_ref()
    }
}

#[derive(Clone)]
struct PendingBoundaryContact {
    promoted_span_index: usize,
    source_parameter: BezierParameter2,
    contact: CurveRegionBoundaryContact2,
}

struct PendingBoundaryOverlap {
    promoted_span_index: usize,
    start: BezierParameter2,
    end: BezierParameter2,
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

/// One exact retained fragment from a source curve in a connected path.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePathRegionTrimFragment2 {
    source_curve_index: usize,
    fragment: CurveRegionTrimFragment2,
}

impl CurvePathRegionTrimFragment2 {
    /// Returns the source curve index in the authored path.
    pub const fn source_curve_index(&self) -> usize {
        self.source_curve_index
    }

    /// Returns the retained fragment and its source-parameter evidence.
    pub const fn trim_fragment(&self) -> &CurveRegionTrimFragment2 {
        &self.fragment
    }

    /// Consumes this record and returns the retained fragment evidence.
    pub fn into_trim_fragment(self) -> CurveRegionTrimFragment2 {
        self.fragment
    }
}

/// One maximal connected retained portion of an authored curve path.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePathRegionTrim2 {
    fragments: Box<[CurvePathRegionTrimFragment2]>,
}

impl CurvePathRegionTrim2 {
    /// Returns retained fragments in source traversal order.
    pub fn fragments(&self) -> &[CurvePathRegionTrimFragment2] {
        &self.fragments
    }

    /// Consumes this path and returns its retained fragments.
    pub fn into_fragments(self) -> Vec<CurvePathRegionTrimFragment2> {
        self.fragments.into_vec()
    }
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
    /// Retains the positive-length exact fragments of this curve in the closed
    /// filled set of a region.
    ///
    /// Every material and hole carrier is intersected with the curve's promoted
    /// rational-Bézier spans through the same pair dispatcher used by region
    /// Booleans. Certified contacts split the source, then one exact
    /// representative per fragment is classified against the complete region.
    /// Certified positive-length overlaps retain boundary fragments; an
    /// unexplained boundary classification or unresolved endpoint image remains
    /// an explicit [`ExactCurveError`] blocker.
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

    pub(crate) fn trim_inside_region_with_parameters_raw(
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
        let native_fragments =
            self.native_bezier_fragments_for_operation(policy, CurveOperation2::Subdivision)?;

        let mut loop_boundaries = Vec::with_capacity(roles.len());
        let mut material_contour_index = 0_usize;
        let mut hole_contour_index = 0_usize;
        for role in roles {
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
            loop_boundaries.push((kind, contour_index));
        }

        let result = region.intersect_curve_boundary_carriers_raw(self, policy)?;
        if let Some(blocker) = result.blockers().first() {
            let reason = if let Some(blocker) = blocker.native_blocker() {
                match blocker.kind() {
                    CurveIntersectionPairBlockerKind2::Uncertain(reason) => *reason,
                    CurveIntersectionPairBlockerKind2::IncompleteReplay { .. } => {
                        UncertaintyReason::Predicate
                    }
                    CurveIntersectionPairBlockerKind2::SharedComponent => {
                        UncertaintyReason::Boundary
                    }
                }
            } else if let Some(reason) = blocker.uncertainty_reason() {
                reason
            } else if blocker.is_point_image_parameter_component() {
                UncertaintyReason::Boundary
            } else {
                UncertaintyReason::Predicate
            };
            return Err(ExactCurveError::blocked(
                CurveOperation2::Subdivision,
                self.family(),
                reason,
            ));
        }
        let endpoint_count = result.overlaps().len().saturating_mul(2);
        let mut split_parameters =
            Vec::with_capacity(result.contacts().len().saturating_add(endpoint_count));
        let mut boundary_contacts =
            Vec::with_capacity(result.contacts().len().saturating_add(endpoint_count));
        let mut boundary_overlaps = Vec::with_capacity(result.overlaps().len());
        for contact in result.contacts() {
            let Some(source_parameter) = contact.first_parameter().as_bezier_parameter() else {
                return Err(ExactCurveError::invalid(
                    CurveOperation2::Subdivision,
                    self.family(),
                    crate::CurveError::Topology(
                        "curve trim source carrier lost its Bezier parameter".into(),
                    ),
                ));
            };
            let Some(&(kind, contour_index)) = loop_boundaries.get(contact.second().loop_index())
            else {
                return Err(ExactCurveError::invalid(
                    CurveOperation2::Subdivision,
                    self.family(),
                    crate::CurveError::Topology(
                        "curve trim contact references an unknown region loop".into(),
                    ),
                ));
            };
            let promoted_span_index = contact.first().fragment_index();
            split_parameters.push((promoted_span_index, source_parameter.clone()));
            boundary_contacts.push(PendingBoundaryContact {
                promoted_span_index,
                source_parameter: source_parameter.clone(),
                contact: CurveRegionBoundaryContact2 {
                    kind,
                    contour_index,
                    segment_index: contact.second().fragment_index(),
                    boundary_parameter: contact.second_parameter().clone(),
                    point: contact.point().cloned(),
                },
            });
        }
        for overlap in result.overlaps() {
            let Some((source_start, source_end)) = overlap.first_range().as_bezier_parameters()
            else {
                return Err(ExactCurveError::invalid(
                    CurveOperation2::Subdivision,
                    self.family(),
                    crate::CurveError::Topology(
                        "curve trim source overlap lost its Bezier parameter range".into(),
                    ),
                ));
            };
            let Some(&(kind, contour_index)) = loop_boundaries.get(overlap.second().loop_index())
            else {
                return Err(ExactCurveError::invalid(
                    CurveOperation2::Subdivision,
                    self.family(),
                    crate::CurveError::Topology(
                        "curve trim overlap references an unknown region loop".into(),
                    ),
                ));
            };
            let source_order = compared_parameter_order(source_start, source_end, self, policy)?;
            let (ordered_start, ordered_end) = match source_order {
                std::cmp::Ordering::Less => (source_start.clone(), source_end.clone()),
                std::cmp::Ordering::Greater => (source_end.clone(), source_start.clone()),
                std::cmp::Ordering::Equal => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Subdivision,
                        self.family(),
                        crate::CurveError::Topology(
                            "positive-length curve trim overlap has equal source boundaries".into(),
                        ),
                    ));
                }
            };
            let promoted_span_index = overlap.first().fragment_index();
            let Some(native) = native_fragments.get(promoted_span_index) else {
                return Err(ExactCurveError::invalid(
                    CurveOperation2::Subdivision,
                    self.family(),
                    crate::CurveError::Topology(
                        "curve trim overlap references an unknown source span".into(),
                    ),
                ));
            };
            let source_rational = crate::RationalBezier2::try_from_subcurve(native.curve())
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Subdivision, self.family(), cause)
                })?;
            boundary_overlaps.push(PendingBoundaryOverlap {
                promoted_span_index,
                start: ordered_start,
                end: ordered_end,
            });
            for (source_parameter, boundary_parameter) in [
                (source_start, overlap.second_range().start()),
                (source_end, overlap.second_range().end()),
            ] {
                split_parameters.push((promoted_span_index, source_parameter.clone()));
                boundary_contacts.push(PendingBoundaryContact {
                    promoted_span_index,
                    source_parameter: source_parameter.clone(),
                    contact: CurveRegionBoundaryContact2 {
                        kind,
                        contour_index,
                        segment_index: overlap.second().fragment_index(),
                        boundary_parameter: boundary_parameter.clone(),
                        point: crate::rational_bezier_general::exact_contact_point_evidence(
                            &source_rational,
                            source_parameter,
                            policy,
                        )
                        .map_err(|cause| {
                            ExactCurveError::invalid(
                                CurveOperation2::Subdivision,
                                self.family(),
                                cause,
                            )
                        })?,
                    },
                });
            }
        }

        let materializations = split_curve_spans(self, split_parameters.into_iter(), policy)?;
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
                let Some((start, end)) = fragment.parameter_range() else {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Subdivision,
                        self.family(),
                        UncertaintyReason::Unsupported,
                    ));
                };
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
                let retain =
                    match region
                        .classify_point_raw(&representative, policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(
                                CurveOperation2::Subdivision,
                                self.family(),
                                cause,
                            )
                        })? {
                        Classification::Decided(RegionPointLocation::Inside) => true,
                        Classification::Decided(RegionPointLocation::Outside) => false,
                        Classification::Decided(RegionPointLocation::Boundary) => {
                            if fragment_is_covered_by_boundary_overlap(
                                &boundary_overlaps,
                                promoted_span_index,
                                start,
                                end,
                                self,
                                policy,
                            )? {
                                // Curve-region clipping uses the closed filled set:
                                // positive-length boundary portions are retained,
                                // while an unexplained Boundary representative is
                                // still evidence that subdivision was incomplete.
                                true
                            } else {
                                return Err(ExactCurveError::blocked(
                                    CurveOperation2::Subdivision,
                                    self.family(),
                                    UncertaintyReason::Boundary,
                                ));
                            }
                        }
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Subdivision,
                                self.family(),
                                reason,
                            ));
                        }
                    };
                if retain {
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
            }
        }
        Ok(retained)
    }
}

impl CurvePath2 {
    /// Retains every maximal connected portion of this path inside a region.
    ///
    /// Each source curve is clipped by the same exact [`Curve2`] kernel. The
    /// result keeps represented and algebraic split boundaries in traversal
    /// order instead of coercing them into a lower-family point carrier.
    pub fn trim_inside_region(
        &self,
        region: &CurveRegion2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurvePathRegionTrim2>>> {
        resolve_certified_operation(policy, |attempt| {
            self.trim_inside_region_raw(region, attempt)
        })
    }

    fn trim_inside_region_raw(
        &self,
        region: &CurveRegion2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurvePathRegionTrim2>> {
        validate_trim_path_connectivity(self, policy)?;
        let mut paths = Vec::new();
        let mut current = Vec::new();

        for (source_curve_index, source_curve) in self.curves().iter().enumerate() {
            let fragments = source_curve.trim_inside_region_with_parameters_raw(region, policy)?;
            for fragment in fragments {
                if let Some(previous) = current.last()
                    && !path_trim_fragments_are_contiguous(
                        previous,
                        source_curve_index,
                        &fragment,
                        self.curves(),
                        policy,
                    )?
                {
                    paths.push(std::mem::take(&mut current));
                }
                current.push(CurvePathRegionTrimFragment2 {
                    source_curve_index,
                    fragment,
                });
            }
        }

        if !current.is_empty() {
            paths.push(current);
        }
        merge_closed_trim_path_seam(self, &mut paths, policy)?;
        Ok(paths
            .into_iter()
            .map(|fragments| CurvePathRegionTrim2 {
                fragments: fragments.into_boxed_slice(),
            })
            .collect())
    }
}

fn validate_trim_path_connectivity(
    path: &CurvePath2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    for adjacent in path.curves().windows(2) {
        if !trim_path_points_equal(
            adjacent[0].end(),
            adjacent[1].start(),
            adjacent[1].family(),
            policy,
        )? {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Subdivision,
                adjacent[1].family(),
                crate::CurveError::DisconnectedCurvePath,
            ));
        }
    }
    Ok(())
}

fn merge_closed_trim_path_seam(
    source: &CurvePath2,
    paths: &mut Vec<Vec<CurvePathRegionTrimFragment2>>,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    if paths.len() < 2
        || !trim_path_points_equal(
            source.end(),
            source.start(),
            source.curves()[0].family(),
            policy,
        )?
    {
        return Ok(());
    }
    let last_fragment = paths
        .last()
        .and_then(|path| path.last())
        .expect("nonempty trim paths retain at least one fragment");
    let first_fragment = paths[0]
        .first()
        .expect("nonempty trim paths retain at least one fragment");
    if !trim_fragment_reaches_curve_boundary(
        &last_fragment.fragment,
        source
            .curves()
            .last()
            .expect("validated curve paths are nonempty"),
        false,
        policy,
    )? || !trim_fragment_reaches_curve_boundary(
        &first_fragment.fragment,
        &source.curves()[0],
        true,
        policy,
    )? {
        return Ok(());
    }

    let first = paths.remove(0);
    paths
        .last_mut()
        .expect("at least one trim path remains after removing the first")
        .extend(first);
    Ok(())
}

fn trim_path_points_equal(
    left: &crate::Point2,
    right: &crate::Point2,
    family: crate::CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    if left == right {
        return Ok(true);
    }
    match crate::classify::is_zero(&left.distance_squared(right), policy) {
        Some(equal) => Ok(equal),
        None => Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            family,
            UncertaintyReason::RealSign,
        )),
    }
}

fn path_trim_fragments_are_contiguous(
    previous: &CurvePathRegionTrimFragment2,
    source_curve_index: usize,
    current: &CurveRegionTrimFragment2,
    source_curves: &[Curve2],
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let previous_curve_index = previous.source_curve_index;
    if source_curve_index == previous_curve_index {
        return trim_fragments_touch_on_curve(
            &previous.fragment,
            current,
            &source_curves[source_curve_index],
            policy,
        );
    }
    if source_curve_index != previous_curve_index + 1 {
        return Ok(false);
    }

    Ok(trim_fragment_reaches_curve_boundary(
        &previous.fragment,
        &source_curves[previous_curve_index],
        false,
        policy,
    )? && trim_fragment_reaches_curve_boundary(
        current,
        &source_curves[source_curve_index],
        true,
        policy,
    )?)
}

fn trim_fragments_touch_on_curve(
    previous: &CurveRegionTrimFragment2,
    current: &CurveRegionTrimFragment2,
    source_curve: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let Some((_, previous_end)) = previous.fragment.parameter_range() else {
        return Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            source_curve.family(),
            UncertaintyReason::Unsupported,
        ));
    };
    let Some((current_start, _)) = current.fragment.parameter_range() else {
        return Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            source_curve.family(),
            UncertaintyReason::Unsupported,
        ));
    };

    if previous.promoted_span_index == current.promoted_span_index {
        return compared_parameters_are_equal(previous_end, current_start, source_curve, policy);
    }
    if current.promoted_span_index != previous.promoted_span_index + 1
        || !compared_parameters_are_equal(
            previous_end,
            &BezierParameter2::Exact(Real::one()),
            source_curve,
            policy,
        )?
        || !compared_parameters_are_equal(
            current_start,
            &BezierParameter2::Exact(Real::zero()),
            source_curve,
            policy,
        )?
    {
        return Ok(false);
    }
    compared_reals_are_equal(
        previous.span_range.endpoints().1,
        current.span_range.endpoints().0,
        source_curve,
        policy,
    )
}

fn trim_fragment_reaches_curve_boundary(
    fragment: &CurveRegionTrimFragment2,
    source_curve: &Curve2,
    start: bool,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let Some((fragment_start, fragment_end)) = fragment.fragment.parameter_range() else {
        return Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            source_curve.family(),
            UncertaintyReason::Unsupported,
        ));
    };
    let (local, span, domain, unit) = if start {
        (
            fragment_start,
            fragment.span_range.endpoints().0,
            source_curve.parameter_domain().start(),
            Real::zero(),
        )
    } else {
        (
            fragment_end,
            fragment.span_range.endpoints().1,
            source_curve.parameter_domain().end(),
            Real::one(),
        )
    };
    Ok(
        compared_parameters_are_equal(local, &BezierParameter2::Exact(unit), source_curve, policy)?
            && compared_reals_are_equal(span, domain, source_curve, policy)?,
    )
}

fn compared_parameters_are_equal(
    left: &BezierParameter2,
    right: &BezierParameter2,
    source_curve: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    Ok(compared_parameter_order(left, right, source_curve, policy)?.is_eq())
}

fn compared_parameter_order(
    left: &BezierParameter2,
    right: &BezierParameter2,
    source_curve: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<std::cmp::Ordering> {
    match classified_parameter_order(left, right, source_curve, policy)? {
        Classification::Decided(ordering) => Ok(ordering),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            source_curve.family(),
            reason,
        )),
    }
}

fn classified_parameter_order(
    left: &BezierParameter2,
    right: &BezierParameter2,
    source_curve: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<std::cmp::Ordering>> {
    left.cmp_by_refinement(right, policy).map_err(|cause| {
        ExactCurveError::invalid(CurveOperation2::Subdivision, source_curve.family(), cause)
    })
}

fn fragment_is_covered_by_boundary_overlap(
    overlaps: &[PendingBoundaryOverlap],
    promoted_span_index: usize,
    start: &BezierParameter2,
    end: &BezierParameter2,
    source_curve: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let mut uncertainty = None;
    for overlap in overlaps
        .iter()
        .filter(|overlap| overlap.promoted_span_index == promoted_span_index)
    {
        let start_order = classified_parameter_order(&overlap.start, start, source_curve, policy)?;
        let end_order = classified_parameter_order(end, &overlap.end, source_curve, policy)?;
        match (start_order, end_order) {
            (Classification::Decided(start), Classification::Decided(end))
                if !start.is_gt() && !end.is_gt() =>
            {
                return Ok(true);
            }
            (Classification::Decided(start), _) if start.is_gt() => {}
            (_, Classification::Decided(end)) if end.is_gt() => {}
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                uncertainty.get_or_insert(reason);
            }
            (Classification::Decided(_), Classification::Decided(_)) => {}
        }
    }
    match uncertainty {
        Some(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            source_curve.family(),
            reason,
        )),
        None => Ok(false),
    }
}

fn compared_reals_are_equal(
    left: &Real,
    right: &Real,
    source_curve: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    match crate::classify::compare_reals(left, right, policy) {
        Some(ordering) => Ok(ordering.is_eq()),
        None => Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            source_curve.family(),
            UncertaintyReason::Ordering,
        )),
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
    fn exact_curve_path_trim_groups_adjacent_source_curves() {
        let region = native_region(vec![rectangle(0, 0, 4, 4)], Vec::new());
        let path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p(-1, 2), p(2, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(2, 2), p(5, 2)).unwrap()),
        ])
        .unwrap();

        let retained = path
            .trim_inside_region(&region, &CurveContext::STRICT)
            .unwrap()
            .into_value();
        assert_eq!(retained.len(), 1);
        assert_eq!(retained[0].fragments().len(), 2);
        assert_eq!(retained[0].fragments()[0].source_curve_index(), 0);
        assert_eq!(retained[0].fragments()[1].source_curve_index(), 1);
    }

    #[test]
    fn exact_curve_path_trim_merges_a_closed_path_across_its_authored_seam() {
        let region = native_region(vec![rectangle(-2, -2, 0, 2)], Vec::new());
        let path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p(-1, 1), p(1, 1)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(1, 1), p(1, -1)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(1, -1), p(-1, -1)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(-1, -1), p(-1, 1)).unwrap()),
        ])
        .unwrap();

        let retained = path
            .trim_inside_region(&region, &CurveContext::STRICT)
            .unwrap()
            .into_value();
        assert_eq!(retained.len(), 1);
        assert_eq!(
            retained[0]
                .fragments()
                .iter()
                .map(CurvePathRegionTrimFragment2::source_curve_index)
                .collect::<Vec<_>>(),
            vec![2, 3, 0]
        );
    }

    #[test]
    fn exact_curve_path_trim_merges_a_periodic_curve_across_its_parameter_seam() {
        let region = native_region(vec![rectangle(0, -3, 3, 3)], Vec::new());
        let circle =
            Curve2::from(CircularArc2::try_from_center(p(2, 0), p(2, 0), p(0, 0), false).unwrap());
        let path = CurvePath2::try_new(vec![circle]).unwrap();

        let retained = path
            .trim_inside_region(&region, &CurveContext::STRICT)
            .unwrap()
            .into_value();
        assert_eq!(retained.len(), 1);
        assert_eq!(retained[0].fragments().len(), 2);
        assert!(
            retained[0]
                .fragments()
                .iter()
                .all(|fragment| fragment.source_curve_index() == 0)
        );
    }

    #[test]
    fn exact_curve_path_trim_retains_algebraic_boundary_images() {
        let boundary = CurvePath2::try_new(vec![
            Curve2::from(QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 2))),
            Curve2::from(LineSeg2::try_new(p(2, 2), p(0, 2)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(0, 2), p(0, 0)).unwrap()),
        ])
        .unwrap();
        let region = CurveRegion2::try_from_boundary_paths_with_loop_topology(
            &[boundary],
            &[CurveRegionLoopRole::Material],
            &[FillRule::NonZero],
            &[CurveBoundaryInteriorSide2::Left],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
        let source = CurvePath2::try_new(vec![Curve2::from(
            LineSeg2::try_new(p(-1, 1), p(3, 1)).unwrap(),
        )])
        .unwrap();

        let outcome = source
            .trim_inside_region(&region, &CurveContext::APPROXIMATE_512)
            .unwrap();
        assert_eq!(outcome.certainty, CurveCertainty::Certified);
        assert_eq!(outcome.value.len(), 1);
        let [fragment] = outcome.value[0].fragments() else {
            panic!("parabolic trim must retain one connected fragment");
        };
        assert!(
            fragment
                .trim_fragment()
                .fragment()
                .is_algebraic_endpoint_images()
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
    fn exact_curve_trim_retains_a_positive_length_boundary_overlap() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let region = native_region(vec![rectangle(0, 0, 4, 4)], Vec::new());
            let source = Curve2::from(LineSeg2::try_new(p(-1, 0), p(5, 0)).unwrap());
            let trimmed = source
                .trim_inside_region_with_parameters(&region, &policy)
                .expect("a shared finite boundary must have exact closed-set trim semantics");
            assert_eq!(trimmed.certainty, CurveCertainty::Certified);
            let [trimmed] = trimmed.value.as_slice() else {
                panic!("the shared bottom edge must retain one exact interval");
            };
            let BezierSplitFragment2::Materialized { curve, .. } = trimmed.fragment() else {
                panic!("represented overlap endpoints must materialize the retained line");
            };
            assert_eq!(curve.start(), &p(0, 0));
            assert_eq!(curve.end(), &p(4, 0));
            let start = trimmed
                .start_boundary_contacts()
                .iter()
                .find(|contact| contact.segment_index() == 0)
                .expect("the overlap start must retain bottom-edge provenance");
            let end = trimmed
                .end_boundary_contacts()
                .iter()
                .find(|contact| contact.segment_index() == 0)
                .expect("the overlap end must retain bottom-edge provenance");
            assert_eq!(
                start.point(),
                Some(&RationalBezierIntersectionPointEvidence2::Exact(p(0, 0)))
            );
            assert_eq!(
                end.point(),
                Some(&RationalBezierIntersectionPointEvidence2::Exact(p(4, 0)))
            );
        }
    }

    #[test]
    fn exact_path_trim_connects_inside_fragments_across_a_hole_boundary_overlap() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let region = native_region(vec![rectangle(0, 0, 6, 4)], vec![rectangle(2, 1, 4, 3)]);
            let source = CurvePath2::try_new(vec![Curve2::from(
                LineSeg2::try_new(p(0, 1), p(6, 1)).unwrap(),
            )])
            .unwrap();
            let trimmed = source
                .trim_inside_region(&region, &policy)
                .expect("the closed face includes its hole boundary");
            assert_eq!(trimmed.certainty, CurveCertainty::Certified);
            let [path] = trimmed.value.as_slice() else {
                panic!("inside and boundary intervals must remain one connected path");
            };
            assert_eq!(path.fragments().len(), 3);
        }
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
                Some(&RationalBezierIntersectionPointEvidence2::Exact(point))
            );
            assert!(
                contact
                    .boundary_parameter()
                    .as_bezier_parameter()
                    .and_then(BezierParameter2::as_exact)
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
