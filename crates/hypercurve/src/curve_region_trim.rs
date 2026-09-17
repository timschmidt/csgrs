//! Exact top-level curve clipping against unified curve regions.

use crate::policy::resolve_certified_operation;
use crate::{
    Classification, Curve2, CurveContext, CurveIntersectionPairBlockerKind2, CurveLocation2,
    CurveOperation2, CurveOutcome, CurveParameter2, CurveParameterRange2, CurvePath2, CurvePoint2,
    CurveRegion2, CurveRegionCarrier2, CurveResult, CurveSpanRange2, ExactCurveError,
    ExactCurveResult, Real, RegionPointLocation, UncertaintyReason,
};

/// Which authored region boundary owns one exact trim contact.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum CurveRegionBoundaryKind2 {
    /// A material contour boundary.
    Material,
    /// A hole contour boundary.
    Hole,
}

/// Exact evidence that a trim endpoint lies on a retained region carrier,
/// together with its input boundary provenance.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionBoundaryContact2 {
    kind: CurveRegionBoundaryKind2,
    contour_index: usize,
    carrier: CurveRegionCarrier2,
    boundary_parameter: CurveParameter2,
    point: Option<CurvePoint2>,
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

    /// Returns the evaluable carrier and its input boundary provenance.
    pub const fn carrier(&self) -> &CurveRegionCarrier2 {
        &self.carrier
    }

    /// Returns the exact parameter in [`Self::carrier`]'s curve chart.
    pub const fn boundary_parameter(&self) -> &CurveParameter2 {
        &self.boundary_parameter
    }

    /// Returns retained exact point evidence when the pair kernel materializes it.
    ///
    /// Some cross-field analytic contacts are represented completely by their
    /// two selected parameters and therefore have no standalone Cartesian point.
    pub const fn point(&self) -> Option<&CurvePoint2> {
        self.point.as_ref()
    }
}

#[derive(Clone)]
struct PendingBoundaryContact {
    span_index: usize,
    source_parameter: CurveParameter2,
    contact: CurveRegionBoundaryContact2,
}

struct PendingBoundaryOverlap {
    span_index: usize,
    start: CurveParameter2,
    end: CurveParameter2,
}

/// One retained region-clipped fragment with its source-curve parameter span.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionTrimFragment2 {
    span_index: usize,
    span_range: CurveSpanRange2,
    local_range: CurveParameterRange2,
    curve: Curve2,
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
    /// Returns the source curve's connected span index.
    pub const fn span_index(&self) -> usize {
        self.span_index
    }

    /// Returns the span's affine parameter chart. Its endpoints are the source
    /// parameters corresponding to local zero and one; selected restrictions
    /// can occupy only part of that chart.
    pub const fn span_range(&self) -> &CurveSpanRange2 {
        &self.span_range
    }

    /// Returns the exact retained curve, ready for subsequent operations.
    pub const fn curve(&self) -> &Curve2 {
        &self.curve
    }

    /// Consumes this record and returns the exact retained curve.
    pub fn into_curve(self) -> Curve2 {
        self.curve
    }

    /// Returns the source location of the retained curve's traversal start.
    pub fn start_location(&self) -> CurveLocation2 {
        CurveLocation2::new(
            self.span_index,
            self.span_range.clone(),
            self.local_range.start().clone(),
        )
    }

    /// Returns the source location of the retained curve's traversal end.
    pub fn end_location(&self) -> CurveLocation2 {
        CurveLocation2::new(
            self.span_index,
            self.span_range.clone(),
            self.local_range.end().clone(),
        )
    }

    /// Replays the oriented source parameter range without requiring scalar
    /// coordinates or replacing selected-root evidence.
    pub fn parameter_range(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveParameterRange2>> {
        let start = match self.start_location().parameter(policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(match self.end_location().parameter(policy)? {
            Classification::Decided(end) => {
                Classification::Decided(CurveParameterRange2::new_validated(start, end))
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        })
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
    /// when both local boundaries already have a [`Real`] payload.
    pub fn represented_parameter_range(&self) -> Option<(Real, Real)> {
        let (local_start, local_end) = self.local_range.scalar_endpoints()?;
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
    /// Every material and hole carrier is intersected with the curve's connected
    /// source spans through the same pair dispatcher used by region
    /// Booleans. Certified contacts split the source, then one exact
    /// representative per fragment is classified against the complete region.
    /// Certified positive-length overlaps retain boundary fragments; an
    /// unexplained boundary classification or unresolved endpoint image remains
    /// an explicit [`ExactCurveError`] blocker. Constant source images and
    /// isolated contacts contribute no positive-length piece.
    pub fn trim_inside_region(
        &self,
        region: &CurveRegion2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<Curve2>>> {
        resolve_certified_operation(policy, |attempt| {
            self.trim_inside_region_with_parameters_raw(region, attempt)
                .map(|fragments| {
                    fragments
                        .into_iter()
                        .map(CurveRegionTrimFragment2::into_curve)
                        .collect()
                })
        })
    }

    /// Retains positive-length exact fragments together with the top-level
    /// source-curve parameter span that generated each fragment.
    ///
    /// This is the authoritative form for consumers that must transfer a trim
    /// back to a corresponding curve in another parameter space. Algebraic
    /// boundaries retain their exact [`CurveLocation2`] and [`Curve2`] evidence;
    /// [`CurveRegionTrimFragment2::represented_parameter_range`] succeeds only
    /// when both boundaries already have a [`Real`] payload.
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
        PreparedTrimSource::new(self, policy)?.trim(region, policy)
    }
}

/// One operation owns each span's dimension evidence. Path grouping reuses
/// it for omitted constant spans without preparing or classifying them again.
struct PreparedTrimSource<'a> {
    curve: &'a Curve2,
    spans: std::borrow::Cow<'a, [crate::curve::CurveSourceSpan2]>,
    point_images: Vec<Option<CurvePoint2>>,
}

impl<'a> PreparedTrimSource<'a> {
    fn new(curve: &'a Curve2, policy: &CurveContext) -> ExactCurveResult<Self> {
        let spans = curve.source_spans(policy, CurveOperation2::Subdivision)?;
        let point_images = spans
            .iter()
            .map(|span| span.point_image(policy))
            .collect::<ExactCurveResult<_>>()?;
        Ok(Self {
            curve,
            spans,
            point_images,
        })
    }

    fn constant_spans(
        &self,
        indices: std::ops::Range<usize>,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        for index in indices {
            let Some(image) = &self.point_images[index] else {
                return Ok(false);
            };
            if !trim_path_points_equal(image.clone(), point.clone(), self.curve.family(), policy)? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn trim(
        &self,
        region: &CurveRegion2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveRegionTrimFragment2>> {
        let source_spans = &self.spans;
        let active = source_spans
            .iter()
            .enumerate()
            .filter(|(index, _)| self.point_images[*index].is_none())
            .collect::<Vec<_>>();
        // One-dimensional clipping drops isolated point images, including
        // points on a boundary. They do not enter positive-length pair replay.
        if region.is_empty() || active.is_empty() {
            return Ok(Vec::new());
        }
        let roles = match region.loop_roles_raw(policy).map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Subdivision, self.curve.family(), cause)
        })? {
            Classification::Decided(roles) => roles,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Subdivision,
                    self.curve.family(),
                    reason,
                ));
            }
        };
        if roles.len() != region.boundary_loops().len() {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Subdivision,
                self.curve.family(),
                crate::CurveError::Topology(
                    "curve trim region roles do not match its boundary loops".into(),
                ),
            ));
        }

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

        let context =
            crate::curve_region_boolean::CurveRegionBooleanContext::try_new_curve_boundary(
                &active, region, policy,
            )?;
        let result = context.build_intersection_evidence()?;
        if let Some(blocker) = result.blockers().first() {
            let reason = if let Some(blocker) = blocker.pair_blocker() {
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
                self.curve.family(),
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
            let source_parameter = contact.first_parameter().clone();
            let Some(&(kind, contour_index)) = loop_boundaries.get(contact.second().loop_index())
            else {
                return Err(ExactCurveError::invalid(
                    CurveOperation2::Subdivision,
                    self.curve.family(),
                    crate::CurveError::Topology(
                        "curve trim contact references an unknown region loop".into(),
                    ),
                ));
            };
            let span_index = contact.first().fragment_index();
            split_parameters.push((span_index, source_parameter.clone()));
            boundary_contacts.push(PendingBoundaryContact {
                span_index,
                source_parameter,
                contact: CurveRegionBoundaryContact2 {
                    kind,
                    contour_index,
                    carrier: contact.second().clone(),
                    boundary_parameter: contact.second_parameter().clone(),
                    point: contact.point().cloned(),
                },
            });
        }
        for overlap in result.overlaps() {
            let source_start = overlap.overlap().first_range().start().clone();
            let source_end = overlap.overlap().first_range().end().clone();
            let Some(&(kind, contour_index)) = loop_boundaries.get(overlap.second().loop_index())
            else {
                return Err(ExactCurveError::invalid(
                    CurveOperation2::Subdivision,
                    self.curve.family(),
                    crate::CurveError::Topology(
                        "curve trim overlap references an unknown region loop".into(),
                    ),
                ));
            };
            let source_order =
                compared_parameter_order(&source_start, &source_end, self.curve, policy)?;
            let (ordered_start, ordered_end) = match source_order {
                std::cmp::Ordering::Less => (source_start.clone(), source_end.clone()),
                std::cmp::Ordering::Greater => (source_end.clone(), source_start.clone()),
                std::cmp::Ordering::Equal => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Subdivision,
                        self.curve.family(),
                        crate::CurveError::Topology(
                            "positive-length curve trim overlap has equal source boundaries".into(),
                        ),
                    ));
                }
            };
            let span_index = overlap.first().fragment_index();
            let Some(span) = source_spans.get(span_index) else {
                return Err(ExactCurveError::invalid(
                    CurveOperation2::Subdivision,
                    self.curve.family(),
                    crate::CurveError::Topology(
                        "curve trim overlap references an unknown source span".into(),
                    ),
                ));
            };
            let source_curve = span.curve();
            boundary_overlaps.push(PendingBoundaryOverlap {
                span_index,
                start: ordered_start,
                end: ordered_end,
            });
            for (source_parameter, boundary_parameter) in [
                (&source_start, overlap.overlap().second_range().start()),
                (&source_end, overlap.overlap().second_range().end()),
            ] {
                split_parameters.push((span_index, source_parameter.clone()));
                boundary_contacts.push(PendingBoundaryContact {
                    span_index,
                    source_parameter: source_parameter.clone(),
                    contact: CurveRegionBoundaryContact2 {
                        kind,
                        contour_index,
                        carrier: overlap.second().clone(),
                        boundary_parameter: boundary_parameter.clone(),
                        point: Some(
                            source_curve
                                .point_at(source_parameter, policy)?
                                .into_value(),
                        ),
                    },
                });
            }
        }

        // Preserve each prepared support and its chart through subdivision.
        let mut by_span = vec![Vec::new(); source_spans.len()];
        for (span_index, parameter) in split_parameters {
            let parameters = by_span.get_mut(span_index).ok_or_else(|| {
                ExactCurveError::invalid(
                    CurveOperation2::Subdivision,
                    self.curve.family(),
                    crate::CurveError::Topology(
                        "curve trim cut references an unknown source span".into(),
                    ),
                )
            })?;
            parameters.push(parameter);
        }
        let mut retained = Vec::new();
        for (carrier_index, &(span_index, span)) in active.iter().enumerate() {
            let parameters = std::mem::take(&mut by_span[span_index]);
            let source = span.curve();
            for (range, curve) in source.split_at_parameters(parameters, policy)? {
                let covered = fragment_is_covered_by_boundary_overlap(
                    &boundary_overlaps,
                    span_index,
                    range.start(),
                    range.end(),
                    self.curve,
                    policy,
                )?;
                let retain = if covered {
                    // Closed-set clipping includes positive-length boundary
                    // overlaps; complete pair replay supplies their incidence.
                    true
                } else {
                    match context.trim_piece_location(carrier_index, &curve, &range)? {
                        RegionPointLocation::Inside => true,
                        RegionPointLocation::Outside => false,
                        RegionPointLocation::Boundary => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Subdivision,
                                self.curve.family(),
                                UncertaintyReason::Boundary,
                            ));
                        }
                    }
                };
                if retain {
                    let local_range = if source.source_traversal_is_reversed() {
                        CurveParameterRange2::new_validated(
                            range.end().clone(),
                            range.start().clone(),
                        )
                    } else {
                        range
                    };
                    let start_boundary_contacts = boundary_contacts_at(
                        &boundary_contacts,
                        span_index,
                        local_range.start(),
                        self.curve,
                        policy,
                    )?;
                    let end_boundary_contacts = boundary_contacts_at(
                        &boundary_contacts,
                        span_index,
                        local_range.end(),
                        self.curve,
                        policy,
                    )?;
                    retained.push(CurveRegionTrimFragment2 {
                        span_index,
                        span_range: span.chart(),
                        local_range,
                        curve,
                        start_boundary_contacts,
                        end_boundary_contacts,
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
    /// Constant spans contribute no piece but preserve connectivity between
    /// incident retained pieces. Excluded positive-length excursions break it.
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
        if region.is_empty() {
            return Ok(Vec::new());
        }
        let sources = self
            .curves()
            .iter()
            .map(|curve| PreparedTrimSource::new(curve, policy))
            .collect::<ExactCurveResult<Vec<_>>>()?;
        let mut paths = Vec::new();
        let mut current = Vec::new();

        for (source_curve_index, source) in sources.iter().enumerate() {
            let fragments = source.trim(region, policy)?;
            for fragment in fragments {
                if let Some(previous) = current.last()
                    && !path_trim_fragments_are_contiguous(
                        previous,
                        source_curve_index,
                        &fragment,
                        &sources,
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
        merge_closed_trim_path_seam(self, &sources, &mut paths, policy)?;
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
    sources: &[PreparedTrimSource<'_>],
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
    let last_index = last_fragment.source_curve_index;
    let first_index = first_fragment.source_curve_index;
    let point = last_fragment.fragment.curve.end();
    if !trim_fragment_reaches_curve_boundary(
        &last_fragment.fragment,
        &sources[last_index],
        false,
        policy,
    )? || !trim_fragment_reaches_curve_boundary(
        &first_fragment.fragment,
        &sources[first_index],
        true,
        policy,
    )? || !trim_path_points_equal(
        point.clone(),
        first_fragment.fragment.curve.start(),
        sources[first_index].curve.family(),
        policy,
    )? {
        return Ok(());
    }
    for skipped in sources[last_index + 1..]
        .iter()
        .chain(&sources[..first_index])
    {
        if !skipped.constant_spans(0..skipped.spans.len(), &point, policy)? {
            return Ok(());
        }
    }

    let first = paths.remove(0);
    paths
        .last_mut()
        .expect("at least one trim path remains after removing the first")
        .extend(first);
    Ok(())
}

fn trim_path_points_equal(
    left: CurvePoint2,
    right: CurvePoint2,
    family: crate::CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    if left == right {
        return Ok(true);
    }
    match left.same_point(&right, policy) {
        Classification::Decided(equal) => Ok(equal),
        Classification::Uncertain(_) => Err(ExactCurveError::blocked(
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
    sources: &[PreparedTrimSource<'_>],
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let previous_index = previous.source_curve_index;
    if source_curve_index == previous_index {
        return trim_fragments_touch_on_curve(
            &previous.fragment,
            current,
            &sources[source_curve_index],
            policy,
        );
    }
    if source_curve_index < previous_index {
        return Ok(false);
    }
    if !trim_fragment_reaches_curve_boundary(
        &previous.fragment,
        &sources[previous_index],
        false,
        policy,
    )? || !trim_fragment_reaches_curve_boundary(
        current,
        &sources[source_curve_index],
        true,
        policy,
    )? {
        return Ok(false);
    }
    let point = previous.fragment.curve.end();
    for skipped in &sources[previous_index + 1..source_curve_index] {
        if !skipped.constant_spans(0..skipped.spans.len(), &point, policy)? {
            return Ok(false);
        }
    }
    trim_path_points_equal(
        point,
        current.curve.start(),
        sources[source_curve_index].curve.family(),
        policy,
    )
}

fn trim_fragments_touch_on_curve(
    previous: &CurveRegionTrimFragment2,
    current: &CurveRegionTrimFragment2,
    source: &PreparedTrimSource<'_>,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    if previous.span_index == current.span_index {
        return compared_parameters_are_equal(
            previous.local_range.end(),
            current.local_range.start(),
            source.curve,
            policy,
        );
    }
    if current.span_index < previous.span_index
        || !trim_fragment_reaches_span_boundary(previous, source, false, policy)?
        || !trim_fragment_reaches_span_boundary(current, source, true, policy)?
    {
        return Ok(false);
    }
    let point = previous.curve.end();
    Ok(
        source.constant_spans(previous.span_index + 1..current.span_index, &point, policy)?
            && trim_path_points_equal(point, current.curve.start(), source.curve.family(), policy)?,
    )
}

fn trim_fragment_reaches_span_boundary(
    fragment: &CurveRegionTrimFragment2,
    source: &PreparedTrimSource<'_>,
    start: bool,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let span = source.spans[fragment.span_index].curve();
    let domain = span.parameter_domain();
    let boundary = if start != span.source_traversal_is_reversed() {
        domain.start()
    } else {
        domain.end()
    };
    let parameter = if start {
        fragment.local_range.start()
    } else {
        fragment.local_range.end()
    };
    compared_parameters_are_equal(parameter, boundary, source.curve, policy)
}

fn trim_fragment_reaches_curve_boundary(
    fragment: &CurveRegionTrimFragment2,
    source: &PreparedTrimSource<'_>,
    start: bool,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    if !trim_fragment_reaches_span_boundary(fragment, source, start, policy)? {
        return Ok(false);
    }
    let (indices, point) = if start {
        (0..fragment.span_index, fragment.curve.start())
    } else {
        (
            fragment.span_index + 1..source.spans.len(),
            fragment.curve.end(),
        )
    };
    source.constant_spans(indices, &point, policy)
}

fn compared_parameters_are_equal(
    left: &CurveParameter2,
    right: &CurveParameter2,
    source_curve: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    Ok(compared_parameter_order(left, right, source_curve, policy)?.is_eq())
}

fn compared_parameter_order(
    left: &CurveParameter2,
    right: &CurveParameter2,
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
    left: &CurveParameter2,
    right: &CurveParameter2,
    source_curve: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<std::cmp::Ordering>> {
    left.cmp_by_refinement(right, policy).map_err(|cause| {
        ExactCurveError::invalid(CurveOperation2::Subdivision, source_curve.family(), cause)
    })
}

fn fragment_is_covered_by_boundary_overlap(
    overlaps: &[PendingBoundaryOverlap],
    span_index: usize,
    start: &CurveParameter2,
    end: &CurveParameter2,
    source_curve: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let mut uncertainty = None;
    for overlap in overlaps
        .iter()
        .filter(|overlap| overlap.span_index == span_index)
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

fn boundary_contacts_at(
    contacts: &[PendingBoundaryContact],
    span_index: usize,
    parameter: &CurveParameter2,
    source: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<CurveRegionBoundaryContact2>> {
    let mut matched = Vec::new();
    for pending in contacts
        .iter()
        .filter(|pending| pending.span_index == span_index)
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

    fn representative_point(curve: &Curve2) -> Point2 {
        let Classification::Decided(parameter) = curve
            .parameter_domain()
            .strict_interior_scalar(&CurveContext::STRICT)
            .unwrap()
        else {
            panic!("fixture must have a certified interior parameter");
        };
        curve
            .point_at(&parameter.into(), &CurveContext::STRICT)
            .unwrap()
            .value
            .coordinates()
            .expect("fixture interior has scalar coordinates")
            .clone()
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
            outcome.value[0].start_boundary_contacts()[0]
                .carrier()
                .fragment_index(),
            0
        );
        assert_eq!(
            outcome.value[0].end_boundary_contacts()[0]
                .carrier()
                .fragment_index(),
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
        let curve = fragment.trim_fragment().curve();
        assert_eq!(
            curve
                .start()
                .same_point(&p(0, 1).into(), &CurveContext::STRICT),
            Classification::Decided(true)
        );
        let end = Point2::new(Real::from(2).sqrt().unwrap(), Real::one());
        assert_eq!(
            curve.end().same_point(&end.into(), &CurveContext::STRICT),
            Classification::Decided(true)
        );
        assert_eq!(
            curve
                .trim_inside_region(&region, &CurveContext::STRICT)
                .unwrap()
                .value
                .len(),
            1
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
        assert_eq!(representative_point(&fragments[0]), p(1, 2));
        assert_eq!(representative_point(&fragments[1]), p(5, 2));
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
            assert_eq!(trimmed.curve().start(), p(0, 0).into());
            assert_eq!(trimmed.curve().end(), p(4, 0).into());
            let start = trimmed
                .start_boundary_contacts()
                .iter()
                .find(|contact| contact.carrier().fragment_index() == 0)
                .expect("the overlap start must retain bottom-edge provenance");
            let end = trimmed
                .end_boundary_contacts()
                .iter()
                .find(|contact| contact.carrier().fragment_index() == 0)
                .expect("the overlap end must retain bottom-edge provenance");
            assert_eq!(start.point(), Some(&CurvePoint2::from(p(0, 0))));
            assert_eq!(end.point(), Some(&CurvePoint2::from(p(4, 0))));
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
    fn parameter_retaining_trim_reports_canonical_boundary_provenance() {
        let region = native_region(vec![rectangle(0, 0, 6, 4)], vec![rectangle(2, 1, 4, 3)]);
        let line = Curve2::from(LineSeg2::try_new(p(-1, 2), p(7, 2)).unwrap());
        let fragments = line
            .trim_inside_region_with_parameters(&region, &CurveContext::STRICT)
            .unwrap()
            .into_value();

        assert_eq!(fragments.len(), 2);
        let expected = [
            (CurveRegionBoundaryKind2::Material, 3, p(0, 2)),
            (CurveRegionBoundaryKind2::Hole, 1, p(2, 2)),
            (CurveRegionBoundaryKind2::Hole, 3, p(4, 2)),
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
            assert_eq!(contact.carrier().fragment_index(), segment_index);
            assert_eq!(contact.point(), Some(&CurvePoint2::from(point)));
            assert!(contact.boundary_parameter().as_algebraic_chord().is_some());
            let replay = contact
                .carrier()
                .curve()
                .point_at(contact.boundary_parameter(), &CurveContext::STRICT)
                .unwrap();
            assert_eq!(replay.certainty, crate::CurveCertainty::Certified);
            assert_eq!(
                replay
                    .value
                    .same_point(contact.point().unwrap(), &CurveContext::STRICT),
                Classification::Decided(true),
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
            let point = representative_point(&fragment);
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
        assert_eq!(fragments[0].span_index(), 0);
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

    #[test]
    fn selected_curve_trim_reuses_unprojected_parameters_and_reenters_operations() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let selected = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
                q(1, 2),
                32_768,
                &policy,
            );
            assert!(matches!(
                selected.promoted_bezier_parameter(&policy).unwrap(),
                Classification::Uncertain(_)
            ));
            let parameter = CurveParameter2::from_selected_fiber(selected);
            let original = Curve2::from(QuadraticBezier2::new(p(0, 0), p(2, 0), p(4, 4)));
            let source = original
                .subcurve(parameter.clone(), Real::one().into(), &policy)
                .unwrap();
            assert_eq!(source.certainty, CurveCertainty::Certified);
            let region = native_region(vec![rectangle(-1, -1, 3, 5)], Vec::new());
            for reversed in [false, true] {
                let source = if reversed {
                    source.value.reversed(&policy).unwrap().value
                } else {
                    source.value.clone()
                };
                let result = source
                    .trim_inside_region_with_parameters(&region, &policy)
                    .unwrap();
                assert_eq!(result.certainty, CurveCertainty::Certified);
                let [piece] = result.value.as_slice() else {
                    panic!("one selected interval");
                };
                assert!(piece.represented_parameter_range().is_none());
                let Classification::Decided(range) = piece.parameter_range(&policy).unwrap() else {
                    panic!("retained exact range");
                };
                let expected = if reversed {
                    [CurveParameter2::from(q(3, 4)), parameter.clone()]
                } else {
                    [parameter.clone(), CurveParameter2::from(q(3, 4))]
                };
                for ((actual, expected), endpoint) in [range.start(), range.end()]
                    .into_iter()
                    .zip(expected)
                    .zip([piece.curve().start(), piece.curve().end()])
                {
                    assert_eq!(
                        actual.cmp_by_refinement(&expected, &policy).unwrap(),
                        Classification::Decided(std::cmp::Ordering::Equal)
                    );
                    let replayed = source.point_at(actual, &policy).unwrap();
                    assert_eq!(replayed.certainty, CurveCertainty::Certified);
                    assert_eq!(
                        replayed.value.same_point(&endpoint, &policy),
                        Classification::Decided(true)
                    );
                }
                let cutter = Curve2::from(
                    LineSeg2::try_new(
                        Point2::new(q(5, 2), -Real::one()),
                        Point2::new(q(5, 2), Real::from(5)),
                    )
                    .unwrap(),
                );
                let contacts = piece.curve().intersect_curve(&cutter, &policy).unwrap();
                assert_eq!(contacts.certainty, CurveCertainty::Certified);
                assert!(contacts.value.blockers().is_empty());
                assert_eq!(contacts.value.contacts().len(), 1);
                let split = piece.curve().split_at(q(5, 8).into(), &policy).unwrap();
                assert_eq!(split.certainty, CurveCertainty::Certified);
                assert_eq!(
                    split
                        .value
                        .0
                        .end()
                        .same_point(&split.value.1.start(), &policy),
                    Classification::Decided(true)
                );
                let smaller = native_region(vec![rectangle(-1, -1, 2, 5)], Vec::new());
                let again = piece.curve().trim_inside_region(&smaller, &policy).unwrap();
                assert_eq!(again.certainty, CurveCertainty::Certified);
                let [again] = again.value.as_slice() else {
                    panic!("one repeatedly trimmed interval");
                };
                let endpoint = if reversed { again.start() } else { again.end() };
                assert_eq!(
                    endpoint.same_point(&p(2, 1).into(), &policy),
                    Classification::Decided(true)
                );
            }
        }
    }

    #[test]
    fn path_trim_separates_discontinuous_spline_spans_at_equal_knot_parameters() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let controls = vec![p(0, 0), p(1, 0), p(3, 0), p(4, 0)];
            let knots = [2, 2, 3, 3, 4, 4]
                .into_iter()
                .map(Real::from)
                .collect::<Vec<_>>();
            let polynomial =
                Curve2::try_polynomial_bspline(1, controls.clone(), knots.clone(), &policy)
                    .unwrap()
                    .value;
            let rational = Curve2::try_nurbs(
                1,
                controls,
                vec![Real::one(), Real::from(2), Real::from(3), Real::one()],
                knots,
                &policy,
            )
            .unwrap()
            .value;
            let region = native_region(vec![rectangle(-1, -1, 5, 1)], Vec::new());
            for source in [polynomial, rational] {
                for reversed in [false, true] {
                    let source = if reversed {
                        source.reversed(&policy).unwrap().value
                    } else {
                        source.clone()
                    };
                    let path = CurvePath2::try_new(vec![source]).unwrap();
                    let result = path.trim_inside_region(&region, &policy).unwrap();
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    assert_eq!(result.value.len(), 2, "a knot jump is not a connected path");
                    let pieces = result
                        .value
                        .iter()
                        .map(|path| {
                            let [fragment] = path.fragments() else {
                                panic!("one span per path");
                            };
                            fragment.trim_fragment()
                        })
                        .collect::<Vec<_>>();
                    assert_eq!(
                        pieces[0]
                            .curve()
                            .end()
                            .same_point(&pieces[1].curve().start(), &policy),
                        Classification::Decided(false)
                    );
                    let first = pieces[0].end_location().parameter(&policy).unwrap();
                    let second = pieces[1].start_location().parameter(&policy).unwrap();
                    assert_eq!(first, second, "the discontinuity shares an authored knot");
                }
            }
        }
    }

    #[test]
    fn closed_path_trim_does_not_join_interior_curves_across_an_excluded_seam() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let points = [
                p(-3, 0),
                p(-2, 0),
                p(-2, 1),
                p(2, 1),
                p(2, 0),
                p(3, 0),
                p(3, -1),
                p(-3, -1),
            ];
            let path = CurvePath2::try_new(
                (0..points.len())
                    .map(|index| {
                        Curve2::from(
                            LineSeg2::try_new(
                                points[index].clone(),
                                points[(index + 1) % points.len()].clone(),
                            )
                            .unwrap(),
                        )
                    })
                    .collect(),
            )
            .unwrap();
            let region = native_region(
                vec![rectangle(-2, 0, -1, 1), rectangle(1, 0, 2, 1)],
                Vec::new(),
            );
            let result = path.trim_inside_region(&region, &policy).unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert_eq!(result.value.len(), 2);
            for retained in result.value {
                for pair in retained.fragments().windows(2) {
                    assert_eq!(
                        pair[0]
                            .trim_fragment()
                            .curve()
                            .end()
                            .same_point(&pair[1].trim_fragment().curve().start(), &policy),
                        Classification::Decided(true)
                    );
                }
            }
        }
    }

    fn decided<T: std::fmt::Debug>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => {
                panic!("fixture requires exact evidence: {reason:?}")
            }
        }
    }

    fn sqrt_half_parameter(policy: &CurveContext) -> crate::BezierAlgebraicParameter2 {
        let polynomial = decided(
            crate::BezierParameterPolynomial::try_new_power_basis(
                vec![Real::from(-1), Real::zero(), Real::from(2)],
                policy,
            )
            .unwrap(),
        );
        let interval =
            decided(crate::BezierParameterInterval::try_new(q(2, 3), q(3, 4), policy).unwrap());
        decided(
            crate::BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap(),
        )
    }

    fn assert_trim_replay(
        source: &Curve2,
        piece: &CurveRegionTrimFragment2,
        policy: &CurveContext,
    ) {
        let range = decided(piece.parameter_range(policy).unwrap());
        for (parameter, endpoint) in [
            (range.start(), piece.curve().start()),
            (range.end(), piece.curve().end()),
        ] {
            let replayed = source.point_at(parameter, policy).unwrap();
            assert_eq!(replayed.certainty, CurveCertainty::Certified);
            assert_eq!(
                replayed.value.same_point(&endpoint, policy),
                Classification::Decided(true)
            );
        }
    }

    #[test]
    fn finite_bezier_carrier_preparation_preserves_trim_and_replay() {
        use crate::{
            BezierAlgebraicChord2, BezierParameter2, BezierSplitFragment2, BezierSubcurve2,
            CurveRegionBoundaryLoop2, RationalBezier2,
        };
        let sources = [
            QuadraticBezier2::new(
                Point2::new(q(-1, 4), 0.into()),
                Point2::new(q(-1, 4), q(1, 2)),
                Point2::new(q(-1, 4), 1.into()),
            ),
            // x(u)=u²-3u/4-1/8, y(u)=u has the same cap entry/exit
            // parameters 1/4 and 1/2, and cannot use a line-image shortcut.
            QuadraticBezier2::new(
                Point2::new(q(-1, 8), 0.into()),
                Point2::new(q(-1, 2), q(1, 2)),
                Point2::new(q(1, 8), 1.into()),
            ),
        ];
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for shift in [0, 1, -2] {
                let s = Real::from(shift);
                let cap = QuadraticBezier2::new(
                    Point2::new(-(&s * &s), -&s),
                    Point2::new(-(&s * &s) + &s, q(1, 2) - &s),
                    Point2::new(-((Real::one() - &s) * (Real::one() - &s)), Real::one() - &s),
                );
                for support in [
                    BezierSubcurve2::Quadratic(cap.clone()),
                    BezierSubcurve2::Rational(
                        RationalBezier2::try_from_subcurve(&BezierSubcurve2::Quadratic(
                            cap.clone(),
                        ))
                        .unwrap()
                        .elevated_to_degree(5)
                        .unwrap(),
                    ),
                ] {
                    for reverse_boundary in [false, true] {
                        let endpoints = if reverse_boundary {
                            [p(0, 0), p(-1, 1)]
                        } else {
                            [p(-1, 1), p(0, 0)]
                        };
                        let boundary = CurveRegionBoundaryLoop2::new(
                            vec![
                                BezierSplitFragment2::RetainedBezier {
                                    source_curve: support.clone(),
                                    reversed: reverse_boundary,
                                    start: BezierParameter2::Exact(s.clone()),
                                    end: BezierParameter2::Exact(&s + Real::one()),
                                    start_image: None,
                                    end_image: None,
                                },
                                BezierSplitFragment2::AlgebraicChord(decided(
                                    BezierAlgebraicChord2::try_new(
                                        endpoints[0].clone().into(),
                                        endpoints[1].clone().into(),
                                        &policy,
                                    )
                                    .unwrap(),
                                )),
                            ],
                            &policy,
                        )
                        .unwrap();
                        let region = CurveRegion2::try_new_with_loop_topology(
                            vec![boundary],
                            vec![CurveRegionLoopRole::Material],
                            vec![FillRule::NonZero],
                            vec![if reverse_boundary {
                                CurveBoundaryInteriorSide2::Right
                            } else {
                                CurveBoundaryInteriorSide2::Left
                            }],
                        )
                        .unwrap();
                        let normalized = region.regularized_region(&policy).unwrap();
                        assert_eq!(normalized.certainty, CurveCertainty::Certified);
                        for region in [&region, &normalized.value] {
                            for source in &sources {
                                for reverse_source in [false, true] {
                                    let source = Curve2::from(source.clone());
                                    let source = if reverse_source {
                                        source.reversed(&policy).unwrap().value
                                    } else {
                                        source
                                    };
                                    let result = source
                                        .trim_inside_region_with_parameters(region, &policy)
                                        .unwrap();
                                    assert_eq!(result.certainty, CurveCertainty::Certified);
                                    let [piece] = result.value.as_slice() else {
                                        panic!(
                                            "one finite cap crossing: shift={shift}, reverse_boundary={reverse_boundary}, reverse_source={reverse_source}"
                                        );
                                    };
                                    let expected = if reverse_source {
                                        (q(1, 2), q(3, 4))
                                    } else {
                                        (q(1, 4), q(1, 2))
                                    };
                                    assert_eq!(piece.represented_parameter_range(), Some(expected));
                                    assert_trim_replay(&source, piece, &policy);
                                    for (contacts, endpoint) in [
                                        (piece.start_boundary_contacts(), piece.curve().start()),
                                        (piece.end_boundary_contacts(), piece.curve().end()),
                                    ] {
                                        let [contact] = contacts else {
                                            panic!("one boundary owner");
                                        };
                                        let replay = contact
                                            .carrier()
                                            .curve()
                                            .point_at(contact.boundary_parameter(), &policy)
                                            .unwrap();
                                        assert_eq!(replay.certainty, CurveCertainty::Certified);
                                        assert_eq!(
                                            replay.value.same_point(&endpoint, &policy),
                                            Classification::Decided(true)
                                        );
                                        if let Some(point) = contact.point() {
                                            assert_eq!(
                                                point.same_point(&endpoint, &policy),
                                                Classification::Decided(true)
                                            );
                                        }
                                    }
                                    let repeated =
                                        piece.curve().trim_inside_region(region, &policy).unwrap();
                                    assert_eq!(repeated.certainty, CurveCertainty::Certified);
                                    assert_eq!(repeated.value.len(), 1);
                                    assert_eq!(
                                        repeated.value[0]
                                            .start()
                                            .same_point(&piece.curve().start(), &policy),
                                        Classification::Decided(true)
                                    );
                                    assert_eq!(
                                        repeated.value[0]
                                            .end()
                                            .same_point(&piece.curve().end(), &policy),
                                        Classification::Decided(true)
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn generated_chord_trim_preserves_holes_and_exact_endpoint_replay() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let parameter = sqrt_half_parameter(&policy);
            let endpoint = |x| {
                CurvePoint2::from(
                    crate::RationalBezier2::try_new(vec![p(0, 0), p(x, 0)], vec![Real::one(); 2])
                        .unwrap()
                        .point_at_algebraic_parameter(&parameter, &policy)
                        .unwrap(),
                )
            };
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(endpoint(-4), endpoint(4), &policy).unwrap(),
            );
            let source =
                Curve2::from_retained_fragment(crate::BezierSplitFragment2::AlgebraicChord(chord));
            let region =
                native_region(vec![rectangle(-2, -2, 2, 2)], vec![rectangle(-1, -1, 1, 1)]);
            for reversed in [false, true] {
                let source = if reversed {
                    source.reversed(&policy).unwrap().value
                } else {
                    source.clone()
                };
                let result = source
                    .trim_inside_region_with_parameters(&region, &policy)
                    .unwrap();
                assert_eq!(result.certainty, CurveCertainty::Certified);
                assert_eq!(result.value.len(), 2);
                let endpoints = if reversed {
                    [(2, 1), (-1, -2)]
                } else {
                    [(-2, -1), (1, 2)]
                };
                for (piece, (start, end)) in result.value.iter().zip(endpoints) {
                    assert_eq!(
                        piece
                            .curve()
                            .start()
                            .same_point(&p(start, 0).into(), &policy),
                        Classification::Decided(true)
                    );
                    assert_eq!(
                        piece.curve().end().same_point(&p(end, 0).into(), &policy),
                        Classification::Decided(true)
                    );
                    assert_trim_replay(&source, piece, &policy);
                    let again = piece.curve().trim_inside_region(&region, &policy).unwrap();
                    assert_eq!(again.certainty, CurveCertainty::Certified);
                    assert_eq!(again.value.len(), 1);
                }
            }
        }
    }

    #[test]
    fn generated_circle_trim_preserves_selected_frame_and_reversal() {
        use crate::bezier_offset::{
            BezierAlgebraicCuspSemicircle2, BezierAlgebraicCuspSemicircleFragment2,
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // At t = sqrt(1/2), x = -1 + 2t^2 is exactly zero.
            let support = QuadraticBezier2::new(p(-1, 0), p(-1, 0), p(1, 0))
                .parallel_left(Real::zero())
                .unwrap();
            let circle = decided(
                BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                    support,
                    crate::BezierParameter2::algebraic(sqrt_half_parameter(&policy)),
                    Real::one(),
                    false,
                    &policy,
                )
                .unwrap(),
            )
            .unwrap();
            let source = Curve2::from_retained_fragment(
                crate::BezierSplitFragment2::AlgebraicCuspSemicircle(
                    BezierAlgebraicCuspSemicircleFragment2::full(circle, &policy),
                ),
            );
            let region = native_region(vec![rectangle(-2, 0, 1, 2)], Vec::new());
            for reversed in [false, true] {
                let source = if reversed {
                    source.reversed(&policy).unwrap().value
                } else {
                    source.clone()
                };
                let result = source
                    .trim_inside_region_with_parameters(&region, &policy)
                    .unwrap();
                assert_eq!(result.certainty, CurveCertainty::Certified);
                let [piece] = result.value.as_slice() else {
                    panic!("one upper quarter circle");
                };
                let [start, end] = if reversed {
                    [p(-1, 0), p(0, 1)]
                } else {
                    [p(0, 1), p(-1, 0)]
                };
                assert_eq!(
                    piece.curve().start().same_point(&start.into(), &policy),
                    Classification::Decided(true)
                );
                assert_eq!(
                    piece.curve().end().same_point(&end.into(), &policy),
                    Classification::Decided(true)
                );
                assert_trim_replay(&source, piece, &policy);
                let again = piece.curve().trim_inside_region(&region, &policy).unwrap();
                assert_eq!(again.certainty, CurveCertainty::Certified);
                assert_eq!(again.value.len(), 1);
                let split = piece.curve().split_at(q(1, 4).into(), &policy).unwrap();
                assert_eq!(split.certainty, CurveCertainty::Certified);
                assert_eq!(
                    split
                        .value
                        .0
                        .end()
                        .same_point(&split.value.1.start(), &policy),
                    Classification::Decided(true)
                );
            }
        }
    }

    #[test]
    fn analytic_parallel_trim_keeps_selected_cuts_for_repeated_operations() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 2))
                .parallel_left(q(1, 8))
                .unwrap();
            let fragment = decided(
                crate::BezierParallelFragment2::try_new(
                    parallel,
                    crate::BezierParameterRange2::new_validated(
                        crate::BezierParameter2::Exact(Real::zero()),
                        crate::BezierParameter2::Exact(Real::one()),
                    ),
                    &policy,
                )
                .unwrap(),
            );
            let source = Curve2::from_retained_fragment(
                crate::BezierSplitFragment2::AnalyticParallel(fragment),
            );
            let region = native_region(vec![rectangle(-1, -1, 1, 3)], Vec::new());
            for reversed in [false, true] {
                let source = if reversed {
                    source.reversed(&policy).unwrap().value
                } else {
                    source.clone()
                };
                let result = source
                    .trim_inside_region_with_parameters(&region, &policy)
                    .unwrap();
                assert_eq!(result.certainty, CurveCertainty::Certified);
                let [piece] = result.value.as_slice() else {
                    panic!("one clipped analytic parallel");
                };
                assert_eq!(
                    piece.curve().family(),
                    crate::CurveFamily2::AnalyticParallel
                );
                assert_trim_replay(&source, piece, &policy);
                let contacts = if reversed {
                    piece.start_boundary_contacts()
                } else {
                    piece.end_boundary_contacts()
                };
                assert_eq!(contacts.len(), 1);
                assert_eq!(contacts[0].carrier().fragment_index(), 1);
                let again = piece.curve().trim_inside_region(&region, &policy).unwrap();
                assert_eq!(again.certainty, CurveCertainty::Certified);
                assert_eq!(again.value.len(), 1);
                let split = piece.curve().split_at(q(1, 4).into(), &policy).unwrap();
                assert_eq!(split.certainty, CurveCertainty::Certified);
                assert_eq!(
                    split
                        .value
                        .0
                        .end()
                        .same_point(&split.value.1.start(), &policy),
                    Classification::Decided(true)
                );
            }
        }
    }

    #[test]
    fn isolated_tangency_does_not_create_a_trimmed_curve() {
        let source = Curve2::from(QuadraticBezier2::new(p(-1, 1), p(0, -1), p(1, 1)));
        let region = native_region(vec![rectangle(-2, -2, 2, 0)], Vec::new());
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let trimmed = source.trim_inside_region(&region, &policy).unwrap();
            assert_eq!(trimmed.certainty, CurveCertainty::Certified);
            assert!(trimmed.value.is_empty());
        }
    }

    #[test]
    fn curve_trim_preserves_recursively_nested_islands_and_holes() {
        let region = native_region(
            vec![
                rectangle(-5, -5, 5, 5),
                rectangle(-3, -3, 3, 3),
                rectangle(-1, -1, 1, 1),
            ],
            vec![rectangle(-4, -4, 4, 4), rectangle(-2, -2, 2, 2)],
        );
        let source = Curve2::from(LineSeg2::try_new(p(-6, 0), p(6, 0)).unwrap());
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let trimmed = source.trim_inside_region(&region, &policy).unwrap();
            assert_eq!(trimmed.certainty, CurveCertainty::Certified);
            assert_eq!(trimmed.value.len(), 5);
            for (piece, (start, end)) in
                trimmed
                    .value
                    .iter()
                    .zip([(-5, -4), (-3, -2), (-1, 1), (2, 3), (4, 5)])
            {
                assert_eq!(
                    piece.start().same_point(&p(start, 0).into(), &policy),
                    Classification::Decided(true)
                );
                assert_eq!(
                    piece.end().same_point(&p(end, 0).into(), &policy),
                    Classification::Decided(true)
                );
            }
        }
    }

    fn constant_curves(point: Point2, policy: &CurveContext) -> Vec<Curve2> {
        use crate::{CubicBezier2, HomogeneousControl2, RationalBezier2, RationalQuadraticBezier2};
        let knots = [2, 2, 3, 4, 5, 5]
            .into_iter()
            .map(Real::from)
            .collect::<Vec<_>>();
        vec![
            QuadraticBezier2::new(point.clone(), point.clone(), point.clone()).into(),
            CubicBezier2::new(point.clone(), point.clone(), point.clone(), point.clone()).into(),
            RationalQuadraticBezier2::try_new(
                point.clone(),
                point.clone(),
                point.clone(),
                Real::one(),
                Real::from(2),
                Real::from(3),
            )
            .unwrap()
            .into(),
            RationalBezier2::try_new(
                vec![point.clone(); 4],
                [1, 2, 3, 1].into_iter().map(Real::from).collect(),
            )
            .unwrap()
            .into(),
            decided(
                RationalBezier2::from_homogeneous_controls(
                    vec![
                        HomogeneousControl2::new(point.x().clone(), point.y().clone(), Real::one()),
                        HomogeneousControl2::new(Real::zero(), Real::zero(), Real::zero()),
                        HomogeneousControl2::new(point.x().clone(), point.y().clone(), Real::one()),
                    ],
                    policy,
                )
                .unwrap(),
            )
            .into(),
            Curve2::try_polynomial_bspline(1, vec![point.clone(); 4], knots.clone(), policy)
                .unwrap()
                .value,
            Curve2::try_nurbs(
                1,
                vec![point; 4],
                [1, 2, 3, 1].into_iter().map(Real::from).collect(),
                knots,
                policy,
            )
            .unwrap()
            .value,
        ]
    }

    #[test]
    fn constant_curve_images_trim_to_empty_inside_on_and_outside_a_region() {
        let region = native_region(vec![rectangle(-5, -5, 5, 5)], Vec::new());
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for point in [p(0, 0), p(5, 0), p(5, 5), p(6, 0)] {
                for source in constant_curves(point, &policy) {
                    for reversed in [false, true] {
                        let source = if reversed {
                            source.reversed(&policy).unwrap().value
                        } else {
                            source.clone()
                        };
                        let result = source
                            .trim_inside_region_with_parameters(&region, &policy)
                            .unwrap();
                        assert_eq!(result.certainty, CurveCertainty::Certified);
                        assert!(
                            result.value.is_empty(),
                            "a constant image has no positive-length fragment"
                        );
                        let path = CurvePath2::try_new(vec![source]).unwrap();
                        let result = path.trim_inside_region(&region, &policy).unwrap();
                        assert_eq!(result.certainty, CurveCertainty::Certified);
                        assert!(result.value.is_empty());
                    }
                }
            }
        }
    }

    #[test]
    fn constant_curve_selected_restrictions_do_not_require_parameter_projection() {
        let region = native_region(vec![rectangle(-5, -5, 5, 5)], Vec::new());
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let selected = crate::bezier_offset::degree_nine_selected_fiber_parameter_for_test(
                q(1, 2),
                32_768,
                &policy,
            );
            assert!(matches!(
                selected.promoted_bezier_parameter(&policy).unwrap(),
                Classification::Uncertain(_)
            ));
            let parameter = CurveParameter2::from_selected_fiber(selected);
            for point in [p(0, 0), p(5, 0), p(6, 0)] {
                let original =
                    Curve2::from(QuadraticBezier2::new(point.clone(), point.clone(), point));
                let source = original
                    .subcurve(parameter.clone(), Real::one().into(), &policy)
                    .unwrap()
                    .value;
                for source in [source.clone(), source.reversed(&policy).unwrap().value] {
                    let result = source.trim_inside_region(&region, &policy).unwrap();
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    assert!(result.value.is_empty());
                }
            }
        }
    }

    #[test]
    fn equal_endpoint_images_do_not_collapse_nonconstant_curves() {
        use crate::{CubicBezier2, HomogeneousControl2, RationalBezier2};
        let region = native_region(vec![rectangle(-3, -3, 3, 3)], Vec::new());
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let rational = decided(
                RationalBezier2::from_homogeneous_controls(
                    vec![
                        HomogeneousControl2::new(Real::zero(), Real::zero(), Real::one()),
                        HomogeneousControl2::new(Real::one(), Real::zero(), Real::zero()),
                        HomogeneousControl2::new(Real::zero(), Real::zero(), Real::one()),
                    ],
                    &policy,
                )
                .unwrap(),
            );
            for source in [
                Curve2::from(QuadraticBezier2::new(p(0, 0), p(2, 0), p(0, 0))),
                Curve2::from(CubicBezier2::new(p(0, 0), p(2, 0), p(0, 2), p(0, 0))),
                Curve2::from(rational),
            ] {
                for source in [source.clone(), source.reversed(&policy).unwrap().value] {
                    let result = source.trim_inside_region(&region, &policy).unwrap();
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    assert_eq!(result.value.len(), 1);
                    assert_eq!(
                        result.value[0]
                            .point_at(&q(1, 2).into(), &policy)
                            .unwrap()
                            .value
                            .same_point(&source.start(), &policy),
                        Classification::Decided(false)
                    );
                    let again = result.value[0]
                        .trim_inside_region(&region, &policy)
                        .unwrap();
                    assert_eq!(again.certainty, CurveCertainty::Certified);
                    assert_eq!(again.value.len(), 1);
                }
            }
        }
    }

    #[test]
    fn constant_authored_curves_bridge_incident_retained_path_pieces() {
        let region = native_region(vec![rectangle(-2, -2, 2, 2)], Vec::new());
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for constant in constant_curves(p(0, 0), &policy) {
                let path = CurvePath2::try_new(vec![
                    LineSeg2::try_new(p(-3, 0), p(0, 0)).unwrap().into(),
                    constant,
                    LineSeg2::try_new(p(0, 0), p(3, 0)).unwrap().into(),
                ])
                .unwrap();
                let result = path.trim_inside_region(&region, &policy).unwrap();
                assert_eq!(result.certainty, CurveCertainty::Certified);
                assert_eq!(result.value.len(), 1);
                assert_eq!(
                    result.value[0]
                        .fragments()
                        .iter()
                        .map(|f| f.source_curve_index())
                        .collect::<Vec<_>>(),
                    vec![0, 2]
                );
                let again = CurvePath2::try_new(
                    result.value[0]
                        .fragments()
                        .iter()
                        .map(|f| f.trim_fragment().curve().clone())
                        .collect(),
                )
                .unwrap()
                .trim_inside_region(&region, &policy)
                .unwrap();
                assert_eq!(again.certainty, CurveCertainty::Certified);
                assert_eq!(again.value.len(), 1);
                assert_eq!(again.value[0].fragments().len(), 2);
            }
        }
    }

    #[test]
    fn excluded_nonconstant_excursion_breaks_a_path_despite_equal_endpoints() {
        let region = native_region(vec![rectangle(-2, -2, 0, 2)], Vec::new());
        let path = CurvePath2::try_new(vec![
            LineSeg2::try_new(p(-1, -1), p(0, 0)).unwrap().into(),
            QuadraticBezier2::new(p(0, 0), p(2, 0), p(0, 0)).into(),
            LineSeg2::try_new(p(0, 0), p(-1, 1)).unwrap().into(),
        ])
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let result = path.trim_inside_region(&region, &policy).unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert_eq!(result.value.len(), 2);
            assert_eq!(result.value[0].fragments()[0].source_curve_index(), 0);
            assert_eq!(result.value[1].fragments()[0].source_curve_index(), 2);
        }
    }

    #[test]
    fn constant_spline_spans_preserve_source_indices_charts_and_path_connectivity() {
        let region = native_region(vec![rectangle(-1, -1, 3, 1)], Vec::new());
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let controls = vec![p(0, 0), p(0, 0), p(1, 0), p(1, 0), p(2, 0), p(2, 0)];
            let knots = [2, 2, 3, 4, 5, 6, 7, 7]
                .into_iter()
                .map(Real::from)
                .collect::<Vec<_>>();
            for source in [
                Curve2::try_polynomial_bspline(1, controls.clone(), knots.clone(), &policy)
                    .unwrap()
                    .value,
                Curve2::try_nurbs(
                    1,
                    controls,
                    [1, 2, 3, 4, 5, 6].into_iter().map(Real::from).collect(),
                    knots,
                    &policy,
                )
                .unwrap()
                .value,
            ] {
                let restricted = source
                    .subcurve(q(5, 2).into(), q(13, 2).into(), &policy)
                    .unwrap()
                    .value;
                for source in [
                    source.clone(),
                    source.reversed(&policy).unwrap().value,
                    restricted.clone(),
                    restricted.reversed(&policy).unwrap().value,
                ] {
                    let path = CurvePath2::try_new(vec![source.clone()]).unwrap();
                    let result = path.trim_inside_region(&region, &policy).unwrap();
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    assert_eq!(result.value.len(), 1);
                    let pieces = result.value[0].fragments();
                    assert_eq!(pieces.len(), 2);
                    assert_eq!(
                        pieces
                            .iter()
                            .map(|f| f.trim_fragment().span_index())
                            .collect::<Vec<_>>(),
                        vec![1, 3]
                    );
                    for piece in pieces {
                        assert_trim_replay(&source, piece.trim_fragment(), &policy);
                    }
                    assert_eq!(
                        pieces[0]
                            .trim_fragment()
                            .curve()
                            .end()
                            .same_point(&pieces[1].trim_fragment().curve().start(), &policy),
                        Classification::Decided(true)
                    );
                }
            }
        }
    }

    #[test]
    fn constant_spline_jumps_do_not_join_coincident_retained_endpoints() {
        let region = native_region(vec![rectangle(-1, -1, 4, 1)], Vec::new());
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let controls = vec![p(0, 0), p(1, 0), p(2, 0), p(2, 0), p(1, 0), p(3, 0)];
            let knots = [2, 2, 3, 3, 4, 4, 5, 5]
                .into_iter()
                .map(Real::from)
                .collect::<Vec<_>>();
            for source in [
                Curve2::try_polynomial_bspline(1, controls.clone(), knots.clone(), &policy)
                    .unwrap()
                    .value,
                Curve2::try_nurbs(1, controls, vec![Real::one(); 6], knots, &policy)
                    .unwrap()
                    .value,
            ] {
                for source in [source.clone(), source.reversed(&policy).unwrap().value] {
                    let result = CurvePath2::try_new(vec![source])
                        .unwrap()
                        .trim_inside_region(&region, &policy)
                        .unwrap();
                    assert_eq!(result.certainty, CurveCertainty::Certified);
                    assert_eq!(result.value.len(), 2);
                    assert!(result.value.iter().all(|path| path.fragments().len() == 1));
                    assert_eq!(
                        result.value[0].fragments()[0]
                            .trim_fragment()
                            .curve()
                            .end()
                            .same_point(
                                &result.value[1].fragments()[0]
                                    .trim_fragment()
                                    .curve()
                                    .start(),
                                &policy
                            ),
                        Classification::Decided(true)
                    );
                }
            }
        }
    }

    #[test]
    fn constant_prefix_and_suffix_preserve_a_closed_path_seam() {
        let region = native_region(vec![rectangle(-2, -2, 0, 2)], Vec::new());
        let constant = Curve2::from(QuadraticBezier2::new(p(-1, 1), p(-1, 1), p(-1, 1)));
        let path = CurvePath2::try_new(vec![
            constant.clone(),
            LineSeg2::try_new(p(-1, 1), p(1, 1)).unwrap().into(),
            LineSeg2::try_new(p(1, 1), p(1, -1)).unwrap().into(),
            LineSeg2::try_new(p(1, -1), p(-1, -1)).unwrap().into(),
            LineSeg2::try_new(p(-1, -1), p(-1, 1)).unwrap().into(),
            constant,
        ])
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let result = path.trim_inside_region(&region, &policy).unwrap();
            assert_eq!(result.certainty, CurveCertainty::Certified);
            assert_eq!(result.value.len(), 1);
            assert_eq!(
                result.value[0]
                    .fragments()
                    .iter()
                    .map(|f| f.source_curve_index())
                    .collect::<Vec<_>>(),
                vec![3, 4, 1]
            );
        }
    }

    #[test]
    fn constant_zero_distance_and_collapsed_circular_parallels_trim_to_empty() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let circle = crate::RationalBezier2::try_new(
                vec![p(1, 0), p(1, 1), p(0, 1)],
                vec![Real::one(), q(1, 2).sqrt().unwrap(), Real::one()],
            )
            .unwrap();
            for parallel in [
                QuadraticBezier2::new(p(0, 0), p(0, 0), p(0, 0))
                    .parallel_left(Real::zero())
                    .unwrap(),
                circle.parallel_left(Real::one()).unwrap(),
            ] {
                // A circular parallel that collapses has no regular analytic
                // span. Its certified homogeneous image is the admitted curve.
                let source =
                    match decided(parallel.exact_circular_parallel_component(&policy).unwrap()) {
                        Some(component) => {
                            assert_eq!(
                                crate::BezierSubcurve2::Rational(component.clone())
                                    .point_image(&policy),
                                Classification::Decided(Some(p(0, 0)))
                            );
                            Curve2::from(component)
                        }
                        None => {
                            let fragment = decided(
                                crate::BezierParallelFragment2::try_new(
                                    parallel,
                                    crate::BezierParameterRange2::new_validated(
                                        crate::BezierParameter2::Exact(Real::zero()),
                                        crate::BezierParameter2::Exact(Real::one()),
                                    ),
                                    &policy,
                                )
                                .unwrap(),
                            );
                            Curve2::from_retained_fragment(
                                crate::BezierSplitFragment2::AnalyticParallel(fragment),
                            )
                        }
                    };
                for region in [
                    native_region(vec![rectangle(-2, -2, 2, 2)], Vec::new()),
                    native_region(vec![rectangle(-2, -2, 0, 2)], Vec::new()),
                    native_region(vec![rectangle(1, 1, 2, 2)], Vec::new()),
                ] {
                    for source in [source.clone(), source.reversed(&policy).unwrap().value] {
                        let result = source.trim_inside_region(&region, &policy).unwrap();
                        assert_eq!(result.certainty, CurveCertainty::Certified);
                        assert!(result.value.is_empty());
                    }
                }
            }
        }
    }

    #[test]
    fn nonconstant_parallel_certificate_survives_coincident_endpoints_and_a_midpoint_cusp() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let parallel = crate::RationalBezier2::try_new(
                vec![p(0, 0), p(1, 0), p(1, 1), p(-1, 0), p(0, 0)],
                vec![Real::one(); 5],
            )
            .unwrap()
            .parallel_left(q(2, 3))
            .unwrap();
            let derivative = decided(parallel.derivative_at(&q(1, 2), &policy).unwrap());
            assert_eq!(
                crate::classify::is_zero(derivative.dx(), &policy),
                Some(true)
            );
            assert_eq!(
                crate::classify::is_zero(derivative.dy(), &policy),
                Some(true)
            );
            assert_eq!(
                parallel.nonconstant_image_certificate(&policy).unwrap(),
                Classification::Decided(())
            );
            assert_eq!(
                decided(parallel.point_at(&Real::zero(), &policy).unwrap()),
                decided(parallel.point_at(&Real::one(), &policy).unwrap()),
            );
            // The whole span contains an interior cusp. Its nonconstancy is
            // a support theorem; regular-span construction must still reject it.
            assert!(
                crate::BezierParallelFragment2::try_new(
                    parallel,
                    crate::BezierParameterRange2::new_validated(
                        crate::BezierParameter2::Exact(Real::zero()),
                        crate::BezierParameter2::Exact(Real::one()),
                    ),
                    &policy,
                )
                .is_err()
            );
        }
    }
}
