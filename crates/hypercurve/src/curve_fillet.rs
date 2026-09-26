//! Retained fillet families and explicit exact contact selection.

use super::*;

/// Isolated exact fillet edits and retained continuous contact families.
///
/// Each family owns the source evidence needed to select its two contacts.
/// Selection performs the same cut validation and reconstruction as an isolated
/// edit. No representative point is chosen for a continuous family.
#[derive(Clone, Debug)]
pub struct CurveFilletSolutions2<T, F> {
    isolated: CurveCornerSolutions2<T>,
    pub(crate) families: Vec<F>,
}

impl<T, F> CurveFilletSolutions2<T, F> {
    /// Returns the isolated, fully reconstructed edits in deterministic order.
    pub fn isolated_solutions(&self) -> &[T] {
        match &self.isolated {
            CurveCornerSolutions2::NoSolution(_) => &[],
            CurveCornerSolutions2::Unique(value) => std::slice::from_ref(value),
            CurveCornerSolutions2::Multiple(values) => values,
        }
    }

    /// Returns the retained contact families in deterministic order.
    pub fn families(&self) -> &[F] {
        &self.families
    }

    /// Returns a reason only when neither an isolated edit nor a family exists.
    pub fn no_solution_reason(&self) -> Option<CurveCornerNoSolution2> {
        if self.families.is_empty() {
            self.isolated.no_solution_reason()
        } else {
            None
        }
    }

    /// Consumes the result without discarding either kind of candidate.
    pub fn into_parts(self) -> (Vec<T>, Vec<F>) {
        let isolated = match self.isolated {
            CurveCornerSolutions2::NoSolution(_) => Vec::new(),
            CurveCornerSolutions2::Unique(value) => vec![value],
            CurveCornerSolutions2::Multiple(values) => values,
        };
        (isolated, self.families)
    }

    pub(crate) fn empty(reason: CurveCornerNoSolution2) -> Self {
        Self::from_isolated(CurveCornerSolutions2::NoSolution(reason), Vec::new())
    }

    pub(crate) fn from_isolated(isolated: CurveCornerSolutions2<T>, families: Vec<F>) -> Self {
        Self { isolated, families }
    }

    pub(crate) fn try_map_isolated<U>(
        self,
        map: impl FnMut(T) -> ExactCurveResult<Option<U>>,
    ) -> ExactCurveResult<CurveFilletSolutions2<U, F>> {
        let isolated =
            compact_optional_corner_solutions(try_map_corner_solutions(self.isolated, map)?);
        Ok(CurveFilletSolutions2 {
            isolated,
            families: self.families,
        })
    }

    pub(crate) fn map_families<G>(self, map: impl FnMut(F) -> G) -> CurveFilletSolutions2<T, G> {
        CurveFilletSolutions2 {
            isolated: self.isolated,
            families: self.families.into_iter().map(map).collect(),
        }
    }
}

#[derive(Clone, Debug)]
enum OwnedFilletParallelSource2 {
    Direct(Curve2),
    NativeSpan(NativeBezierFragment2),
    Retained(crate::BezierParallelFragment2),
    Selected(crate::bezier_split::BezierSelectedFiberFragment2),
}

impl OwnedFilletParallelSource2 {
    fn retain(source: FilletParallelSource2<'_>) -> Self {
        match source {
            FilletParallelSource2::Direct(ExactCornerBezier2::Direct(source)) => {
                Self::Direct(source.clone())
            }
            FilletParallelSource2::Direct(ExactCornerBezier2::NativeSpan(source)) => {
                Self::NativeSpan(source.clone())
            }
            FilletParallelSource2::Retained(source) => Self::Retained(source.clone()),
            FilletParallelSource2::Selected(source) => Self::Selected(source.clone()),
        }
    }

    fn borrowed(&self) -> FilletParallelSource2<'_> {
        match self {
            Self::Direct(source) => {
                FilletParallelSource2::Direct(ExactCornerBezier2::Direct(source))
            }
            Self::NativeSpan(source) => {
                FilletParallelSource2::Direct(ExactCornerBezier2::NativeSpan(source))
            }
            Self::Retained(source) => FilletParallelSource2::Retained(source),
            Self::Selected(source) => FilletParallelSource2::Selected(source),
        }
    }

    fn curve(&self) -> Curve2 {
        match self {
            Self::Direct(source) => source.clone(),
            Self::NativeSpan(source) => source.clone().into_curve(),
            Self::Retained(source) => Curve2::from_retained_fragment(
                crate::BezierSplitFragment2::AnalyticParallel(source.clone()),
            ),
            Self::Selected(source) => Curve2::from_retained_fragment(
                crate::BezierSplitFragment2::SelectedFiber(source.clone()),
            ),
        }
    }
}

#[derive(Clone, Debug)]
struct FilletFamilyGeometry2 {
    sources: [OwnedFilletParallelSource2; 2],
    centers: [BezierParallel2; 2],
    contact_curves: [Curve2; 2],
    radius: Real,
    clockwise: bool,
    retain_selected_circle_endpoints: bool,
    domains: [FilletContactDomain2; 2],
    families: [CurveFamily2; 2],
    policy: CurveContext,
}

#[derive(Clone, Debug)]
pub(crate) struct FilletCornerFamily2 {
    geometry: Arc<FilletFamilyGeometry2>,
    component: crate::bezier_offset::CurveParameterComponent2,
}

impl FilletCornerFamily2 {
    #[allow(clippy::too_many_arguments)]
    pub(super) fn retain(
        components: Vec<crate::bezier_offset::CurveParameterComponent2>,
        offsets: [&FilletOffsetCarrier2<'_, '_>; 2],
        radius: &Real,
        clockwise: bool,
        retain_selected_circle_endpoints: bool,
        domains: [FilletContactDomain2; 2],
        families: [CurveFamily2; 2],
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<Self>> {
        if components.is_empty() {
            return Ok(Vec::new());
        }
        let [
            FilletOffsetCarrier2::Parallel {
                source: first,
                support: first_center,
            },
            FilletOffsetCarrier2::Parallel {
                source: second,
                support: second_center,
            },
        ] = offsets
        else {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                families[0],
                crate::UncertaintyReason::Unsupported,
            ));
        };
        let sources = [
            OwnedFilletParallelSource2::retain(*first),
            OwnedFilletParallelSource2::retain(*second),
        ];
        let contact_curves = sources.each_ref().map(OwnedFilletParallelSource2::curve);
        let geometry = Arc::new(FilletFamilyGeometry2 {
            sources,
            centers: [first_center.clone(), second_center.clone()],
            contact_curves,
            radius: radius.clone(),
            clockwise,
            retain_selected_circle_endpoints,
            domains,
            families,
            policy: policy.retained_object_policy(),
        });
        Ok(components
            .into_iter()
            .map(|component| Self {
                geometry: Arc::clone(&geometry),
                component,
            })
            .collect())
    }

    pub(crate) fn contact_curves(&self) -> &[Curve2; 2] {
        &self.geometry.contact_curves
    }
    pub(crate) fn radius(&self) -> &Real {
        &self.geometry.radius
    }
    pub(crate) fn clockwise(&self) -> bool {
        self.geometry.clockwise
    }

    pub(crate) fn select(
        &self,
        previous: &CurveParameter2,
        next: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<FilletCorner2>> {
        let data = &self.geometry;
        if !policy.accepts_retained_policy(data.policy)
            || (data.policy.selects_approximate_512() && !policy.permits_approximate_512())
        {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                data.families[0],
                crate::UncertaintyReason::Predicate,
            ));
        }
        if data.policy.selects_approximate_512() {
            policy.observe_approximate_512();
        }
        match self
            .component
            .contains_pair(previous, next, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, data.families[0], cause)
            })? {
            Classification::Decided(true) => (),
            Classification::Decided(false) => return Ok(None),
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    data.families[0],
                    reason,
                ));
            }
        }
        let point = match self.component.point_image() {
            Some(point) => point.clone(),
            None => analytic_parallel_point_evidence(
                &data.centers[0],
                previous,
                CurveOperation2::Fillet,
                data.families[0],
                policy,
            )?,
        };
        let center = FilletCenterWitness2 {
            point,
            previous_parameter: Some(previous.clone()),
            next_parameter: Some(next.clone()),
            retained_anchor_evidence: None,
        };
        let offsets: [_; 2] = std::array::from_fn(|axis| FilletOffsetCarrier2::Parallel {
            source: data.sources[axis].borrowed(),
            support: data.centers[axis].clone(),
        });
        Ok(
            match fillet_corner_from_center(
                &offsets[0],
                &offsets[1],
                &center,
                data.clockwise,
                data.retain_selected_circle_endpoints,
                data.domains,
                data.families[0],
                data.families[1],
                policy,
            )? {
                FilletCornerSelection2::Selected(corner) => Some(corner),
                FilletCornerSelection2::Outside | FilletCornerSelection2::Degenerate => None,
            },
        )
    }
}

/// One continuous fillet family bound to an immutable source path.
///
/// Contact parameters use the exact charts of [`Self::contact_curves`], in
/// previous/next order. They may lie on permitted extensions of those charts.
/// Selection validates trim ownership and nondegeneracy before publishing a path.
#[derive(Clone, Debug)]
pub struct CurvePathFilletFamily2 {
    placement: Arc<PathFilletPlacement2<'static>>,
    native: FilletCornerFamily2,
}

impl CurvePathFilletFamily2 {
    /// Returns the two exact contact curves and their parameter charts.
    pub fn contact_curves(&self) -> &[Curve2; 2] {
        self.native.contact_curves()
    }

    /// Returns the requested exact radius.
    pub fn radius(&self) -> &Real {
        self.native.radius()
    }

    /// Returns the orientation of the inserted circular arc.
    pub fn is_clockwise(&self) -> bool {
        self.native.clockwise()
    }

    /// Selects two exact contacts and reconstructs the edited path.
    ///
    /// Returns `None` when the pair is outside this family, violates the trim
    /// domain, or collapses the inserted arc. Undecided predicates remain errors.
    pub fn select(
        &self,
        previous: &CurveParameter2,
        next: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Option<CurvePath2>>> {
        resolve_certified_operation(policy, |attempt| {
            let Some(corner) = self.native.select(previous, next, attempt)? else {
                return Ok(None);
            };
            self.placement
                .publish(corner, self.native.radius(), attempt)
        })
    }
}

#[derive(Debug)]
pub(super) struct PathFilletPlacement2<'a> {
    source: std::borrow::Cow<'a, CurvePath2>,
    vertex: usize,
    indices: [usize; 2],
    source_maps: [Option<(Real, Real)>; 2],
    strict_authored_bounds: [bool; 2],
    arcs: [Option<Arc<RetainedRationalCornerArc2>>; 2],
    promoted: [Option<std::borrow::Cow<'a, crate::BezierParallelFragment2>>; 2],
    circular_domains:
        [Option<Arc<PolicyEvaluationCache<curve_corner_domain::AuthoredCircularDomain2>>>; 2],
}

impl<'a> PathFilletPlacement2<'a> {
    #[allow(clippy::too_many_arguments)]
    pub(super) fn new(
        source: &'a CurvePath2,
        vertex: usize,
        indices: [usize; 2],
        preparation_maps: [Option<(&Real, &Real)>; 2],
        authored_maps: [Option<&(Real, Real)>; 2],
        arcs: [Option<Arc<RetainedRationalCornerArc2>>; 2],
        promoted: [Option<&'a crate::BezierParallelFragment2>; 2],
        circular_domains: [Option<Arc<PolicyEvaluationCache<curve_corner_domain::AuthoredCircularDomain2>>>;
            2],
    ) -> Self {
        let source_maps = std::array::from_fn(|axis| {
            let inner =
                preparation_maps[axis].map(|(scale, offset)| (scale.clone(), offset.clone()));
            match (inner, authored_maps[axis]) {
                (Some((scale, offset)), Some((outer_scale, outer_offset))) => {
                    Some((outer_scale * scale, outer_scale * offset + outer_offset))
                }
                (None, Some(map)) => Some(map.clone()),
                (inner, None) => inner,
            }
        });
        Self {
            source: std::borrow::Cow::Borrowed(source),
            vertex,
            indices,
            source_maps,
            strict_authored_bounds: authored_maps.map(|map| map.is_some()),
            arcs,
            promoted: promoted.map(|source| source.map(std::borrow::Cow::Borrowed)),
            circular_domains,
        }
    }

    fn into_owned(self) -> PathFilletPlacement2<'static> {
        PathFilletPlacement2 {
            source: std::borrow::Cow::Owned(self.source.into_owned()),
            vertex: self.vertex,
            indices: self.indices,
            source_maps: self.source_maps,
            strict_authored_bounds: self.strict_authored_bounds,
            arcs: self.arcs,
            promoted: self
                .promoted
                .map(|source| source.map(|source| std::borrow::Cow::Owned(source.into_owned()))),
            circular_domains: self.circular_domains,
        }
    }

    pub(super) fn bind<T>(
        self,
        solutions: CurveFilletSolutions2<T, FilletCornerFamily2>,
    ) -> CurveFilletSolutions2<T, CurvePathFilletFamily2> {
        let shared = (!solutions.families.is_empty()).then(|| Arc::new(self.into_owned()));
        solutions.map_families(|native| CurvePathFilletFamily2 {
            placement: Arc::clone(shared.as_ref().expect("family reconstruction context")),
            native,
        })
    }

    pub(super) fn publish(
        &self,
        mut corner: FilletCorner2,
        radius: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CurvePath2>> {
        for (axis, cut) in [&mut corner.previous, &mut corner.next]
            .into_iter()
            .enumerate()
        {
            let authored = &self.source.data.curves[self.indices[axis]];
            let family = authored.family();
            cut.map_source_parameter(
                self.source_maps[axis].as_ref().map(|(s, o)| (s, o)),
                CurveOperation2::Fillet,
                family,
                policy,
            )?;
            if !self.strict_authored_bounds[axis] {
                continue;
            }
            if cut.placement != CornerPlacement2::Extension {
                let parameter = cut
                    .parameter
                    .as_ref()
                    .expect("finite source-chart parameter");
                let range = authored.parameter_domain();
                if !curve_corner_domain::parameter_order(
                    parameter,
                    range.start(),
                    CurveOperation2::Fillet,
                    family,
                    policy,
                )?
                .is_gt()
                    || !curve_corner_domain::parameter_order(
                        parameter,
                        range.end(),
                        CurveOperation2::Fillet,
                        family,
                        policy,
                    )?
                    .is_lt()
                {
                    return Ok(None);
                }
            } else if let Some(circle) = &self.arcs[axis] {
                let domain = match resolve_cached_evaluation(
                    self.circular_domains[axis]
                        .as_deref()
                        .expect("an incident circular chart retains its domain cache"),
                    policy,
                    |attempt| match curve_corner_domain::AuthoredCircularDomain2::new(
                        authored,
                        circle.support(),
                        CurveOperation2::Fillet,
                        attempt,
                    ) {
                        Ok(domain) => Ok(Classification::Decided(domain)),
                        Err(ExactCurveError::Blocked(blocker)) => {
                            Ok(Classification::Uncertain(blocker.reason()))
                        }
                        Err(error) => Err(error),
                    },
                )? {
                    Classification::Decided(domain) => domain,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            family,
                            reason,
                        ));
                    }
                };
                if domain.contains_incident_point(
                    &cut.point,
                    CurveOperation2::Fillet,
                    family,
                    policy,
                )? {
                    return Ok(None);
                }
            }
        }
        self.source.publish_fillet_corner(
            self.vertex,
            self.indices[0],
            self.indices[1],
            corner,
            radius,
            self.arcs.each_ref().map(|arc| arc.as_deref()),
            self.promoted.each_ref().map(|source| source.as_deref()),
            policy,
        )
    }
}

pub(super) enum FilletCornerSelection2 {
    Selected(FilletCorner2),
    Outside,
    Degenerate,
}

#[allow(clippy::too_many_arguments)]
pub(super) fn fillet_corner_from_center(
    previous_offset: &FilletOffsetCarrier2<'_, '_>,
    next_offset: &FilletOffsetCarrier2<'_, '_>,
    center: &FilletCenterWitness2,
    clockwise: bool,
    retain_selected_circle_endpoints: bool,
    domains: [FilletContactDomain2; 2],
    previous_family: CurveFamily2,
    next_family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<FilletCornerSelection2> {
    let deferred_arc_is_previous = center
        .retained_anchor_evidence
        .as_ref()
        .and_then(|evidence| evidence.deferred_arc_contact.as_ref())
        .map(|deferred| deferred.arc_is_previous);
    let Some(previous_cut) = fillet_cut_from_center(
        previous_offset,
        &center.point,
        center.parameter(true),
        deferred_arc_is_previous == Some(true),
        true,
        retain_selected_circle_endpoints,
        domains[0],
        previous_family,
        policy,
    )?
    else {
        return Ok(FilletCornerSelection2::Outside);
    };
    let Some(next_cut) = fillet_cut_from_center(
        next_offset,
        &center.point,
        center.parameter(false),
        deferred_arc_is_previous == Some(false),
        false,
        retain_selected_circle_endpoints,
        domains[1],
        next_family,
        policy,
    )?
    else {
        return Ok(FilletCornerSelection2::Outside);
    };
    let cut_point_relation = if center
        .retained_anchor_evidence
        .as_ref()
        .and_then(|evidence| evidence.cross)
        .is_some_and(|cross| matches!(cross, RealSign::Positive | RealSign::Negative))
    {
        // Two contacts on one nonzero-radius circle cannot occupy the
        // same point with nonparallel tangents: both tangents would be
        // perpendicular to the same radial vector. The pair replay's
        // exact nonzero tangent cross is therefore also a constant-
        // time distinct-cut certificate and avoids constructing a
        // potentially high-degree Cartesian compositum solely for
        // this degeneracy test.
        Classification::Decided(false)
    } else if center.point.coordinates().is_none()
        && center
            .retained_anchor_evidence
            .as_ref()
            .is_some_and(|evidence| evidence.deferred_arc_contact.is_some())
    {
        // One circular cut is only a transient marker until the
        // retained fillet circle is intersected with the authored arc.
        // Its marker stores the center, so it cannot participate in
        // the ordinary two-contact degeneracy predicate.
        Classification::Decided(false)
    } else {
        previous_cut.point.same_point(&next_cut.point, policy)
    };
    match cut_point_relation {
        Classification::Decided(true) => Ok(FilletCornerSelection2::Degenerate),
        Classification::Decided(false) => {
            let previous_is_cusp =
                matches!(previous_offset, FilletOffsetCarrier2::AlgebraicCusp { .. });
            let next_is_cusp = matches!(next_offset, FilletOffsetCarrier2::AlgebraicCusp { .. });
            let previous_chord_anchors_on_next_arc = matches!(
                (previous_offset, next_offset),
                (
                    FilletOffsetCarrier2::AlgebraicChord { .. },
                    FilletOffsetCarrier2::Arc { .. }
                )
            );
            let cusp_and_line = matches!(
                (previous_offset, next_offset),
                (
                    FilletOffsetCarrier2::AlgebraicCusp { .. },
                    FilletOffsetCarrier2::Line { .. }
                )
            ) || matches!(
                (previous_offset, next_offset),
                (
                    FilletOffsetCarrier2::Line { .. },
                    FilletOffsetCarrier2::AlgebraicCusp { .. }
                )
            );
            let line_precedes_chord = matches!(
                (previous_offset, next_offset),
                (
                    FilletOffsetCarrier2::Line { .. },
                    FilletOffsetCarrier2::AlgebraicChord { .. }
                )
            );
            let (first, first_is_previous, first_family, second, second_family) = if cusp_and_line {
                // The line is transiently lowered to a chord for
                // center incidence. Keep the selected circle as
                // the reconstruction anchor so its mapped radial
                // field remains authoritative.
                if previous_is_cusp {
                    (
                        previous_offset,
                        true,
                        previous_family,
                        next_offset,
                        next_family,
                    )
                } else {
                    (
                        next_offset,
                        false,
                        next_family,
                        previous_offset,
                        previous_family,
                    )
                }
            } else if (previous_is_cusp && !next_is_cusp)
                || previous_chord_anchors_on_next_arc
                || line_precedes_chord
            {
                (
                    next_offset,
                    false,
                    next_family,
                    previous_offset,
                    previous_family,
                )
            } else {
                (
                    previous_offset,
                    true,
                    previous_family,
                    next_offset,
                    next_family,
                )
            };
            let deferred_arc_frame = center
                .retained_anchor_evidence
                .as_ref()
                .and_then(|evidence| evidence.deferred_arc_contact.as_ref())
                .map(|deferred| (deferred.arc_is_previous, deferred.contact_seed.is_some()));
            let prefer_parallel_frame =
                center
                    .retained_anchor_evidence
                    .as_ref()
                    .is_some_and(|evidence| {
                        evidence.center_parallel.is_some() || evidence.source_direction.is_some()
                    });
            let force_chord_normal = matches!(
                (previous_offset, next_offset),
                (
                    FilletOffsetCarrier2::AlgebraicChord { .. },
                    FilletOffsetCarrier2::Line { .. }
                ) | (
                    FilletOffsetCarrier2::Line { .. },
                    FilletOffsetCarrier2::AlgebraicChord { .. }
                )
            );
            #[cfg(feature = "dispatch-trace")]
            {
                let carrier_kind = |carrier: &FilletOffsetCarrier2<'_, '_>| match carrier {
                    FilletOffsetCarrier2::Line {
                        source: FilletLinearSource2::Native { .. },
                        ..
                    } => "line-native",
                    FilletOffsetCarrier2::Line {
                        source: FilletLinearSource2::AlgebraicChord(_),
                        ..
                    } => "line-chord",
                    FilletOffsetCarrier2::Arc { .. } => "arc",
                    FilletOffsetCarrier2::Point { .. } => "point",
                    FilletOffsetCarrier2::Parallel { .. } => "parallel",
                    FilletOffsetCarrier2::AlgebraicCusp { .. } => "algebraic-cusp",
                    FilletOffsetCarrier2::AlgebraicChord { .. } => "algebraic-chord",
                };
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-fillet-previous-carrier",
                    carrier_kind(previous_offset),
                );
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-fillet-next-carrier",
                    carrier_kind(next_offset),
                );
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-fillet-force-chord-normal",
                    if force_chord_normal { "yes" } else { "no" },
                );
            }
            let first_frame = first.retained_fillet_frame(
                first_is_previous,
                center.parameter(first_is_previous),
                center.retained_anchor_evidence.clone(),
                force_chord_normal,
                first_family,
                policy,
            )?;
            let frame_is_preferred = |frame: &RetainedFilletFrame2| {
                if let Some((arc_is_previous, contact_is_preselected)) = deferred_arc_frame {
                    return (frame.anchor_is_previous == arc_is_previous) == contact_is_preselected;
                }
                if force_chord_normal {
                    return matches!(
                        &frame.radial_frame,
                        RetainedFilletRadialFrame2::ChordNormal { .. }
                    );
                }
                if matches!(
                    &frame.radial_frame,
                    RetainedFilletRadialFrame2::ChordNormal { .. }
                ) {
                    return prefer_parallel_frame
                        && frame
                            .anchor_evidence
                            .as_ref()
                            .and_then(|evidence| evidence.center_parallel.as_ref())
                            .and_then(|center| center.parameter.as_ref())
                            .is_some_and(CurveParameter2::is_retained_scalar);
                }
                matches!(
                    &frame.radial_frame,
                    RetainedFilletRadialFrame2::ParallelNormal { .. }
                ) == prefer_parallel_frame
            };
            let retained_frame = if first_frame.as_ref().is_some_and(frame_is_preferred) {
                first_frame
            } else {
                let second_frame = second.retained_fillet_frame(
                    !first_is_previous,
                    center.parameter(!first_is_previous),
                    center.retained_anchor_evidence.clone(),
                    force_chord_normal,
                    second_family,
                    policy,
                )?;
                if second_frame.as_ref().is_some_and(frame_is_preferred) {
                    second_frame
                } else {
                    first_frame.or(second_frame)
                }
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-fillet-retained-frame",
                match retained_frame.as_ref().map(|frame| &frame.radial_frame) {
                    Some(RetainedFilletRadialFrame2::RepresentedUnitNormal(_)) => {
                        "represented-unit-normal"
                    }
                    Some(RetainedFilletRadialFrame2::ChordNormal { .. }) => "chord-normal",
                    Some(RetainedFilletRadialFrame2::ConcentricArc { .. }) => "concentric-arc",
                    Some(RetainedFilletRadialFrame2::SelectedConcentric { .. }) => {
                        "selected-concentric"
                    }
                    Some(RetainedFilletRadialFrame2::ParallelNormal { .. }) => "parallel-normal",
                    None => "none",
                },
            );
            Ok(FilletCornerSelection2::Selected(FilletCorner2 {
                previous: previous_cut,
                next: next_cut,
                center: center.point.clone(),
                clockwise,
                retained_frame,
            }))
        }
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            previous_family,
            reason,
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    fn joined_parallel_path(policy: &CurveContext, regular_ends: bool) -> CurvePath2 {
        // P(t)=(3t/8,9t²/64). These reversed offsets join exactly at
        // (0,41/64), while radius 1/128 gives a continuous center family.
        let source = QuadraticBezier2::new(
            Point2::from_values(0, 0),
            Point2::new(q(3, 16), Real::zero()),
            Point2::new(q(3, 8), q(9, 64)),
        )
        .parallel_left(Real::zero())
        .unwrap();
        let curves = [(q(41, 64), true), (q(5, 8), false)].map(|(distance, previous)| {
            let parallel = source.with_distance(distance);
            let point = |t: &CurveParameter2| {
                analytic_parallel_point_evidence(
                    &parallel,
                    t,
                    CurveOperation2::Fillet,
                    CurveFamily2::AnalyticParallel,
                    policy,
                )
                .unwrap()
            };
            let range = if regular_ends {
                if previous {
                    CurveParameterRange2::new_validated(
                        Real::zero().into(),
                        (q(5, 9) + q(1, 10000)).into(),
                    )
                } else {
                    CurveParameterRange2::new_validated(
                        (q(5, 9) - q(1, 10000)).into(),
                        Real::one().into(),
                    )
                }
            } else {
                CurveParameterRange2::unit()
            };
            let start = point(range.start());
            let end = point(range.end());
            crate::bezier_split::BezierSelectedFiberFragment2::new(
                crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(parallel),
                range,
                start,
                end,
            )
            .reversed()
            .into()
        });
        CurvePath2::try_new_with_policy(curves.into(), policy)
            .unwrap()
            .value
    }

    fn same(actual: &CurvePoint2, expected: &CurvePoint2, policy: &CurveContext) {
        let result = actual.coincides_with(expected, policy);
        assert_eq!(result.certainty, crate::CurveCertainty::Certified);
        assert_eq!(result.value, Classification::Decided(true));
    }

    #[test]
    fn joined_path_selects_and_replays_a_continuous_fillet_family() {
        let parameter = CurveParameter2::from(q(5, 9));
        let contacts = [
            CurvePoint2::from(Point2::new(q(-95, 2496), q(4753, 7488))),
            CurvePoint2::from(Point2::new(q(-5, 156), q(4645, 7488))),
        ];
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = joined_parallel_path(&policy, false);
            for reversed in [false, true] {
                let source = if reversed {
                    source.reversed(&policy).unwrap().value
                } else {
                    source.clone()
                };
                let outcome = source
                    .fillet_vertex_by_radius(1, q(1, 128), CurveCornerMode2::TrimOnly, &policy)
                    .unwrap_or_else(|error| panic!("full-domain family query: reversed={reversed}, policy={policy:?}: {error:?}"));
                assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
                assert!(!outcome.value.families().is_empty());
                assert!(outcome.value.no_solution_reason().is_none());
                let mut selected = Vec::new();
                for family in outcome.value.families() {
                    for excluded in [Real::zero(), Real::one()] {
                        let excluded = CurveParameter2::from(excluded);
                        let result = family.select(&excluded, &excluded, &policy).unwrap();
                        assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                        assert!(result.value.is_none());
                    }
                    let result = family.select(&parameter, &parameter, &policy).unwrap();
                    assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                    if let Some(edited) = result.value {
                        assert_eq!(family.is_clockwise(), !reversed);
                        assert_eq!(family.radius(), &q(1, 128));
                        for (axis, curve) in family.contact_curves().iter().enumerate() {
                            same(
                                &curve.point_at(&parameter, &policy).unwrap().value,
                                &contacts[if reversed { 1 - axis } else { axis }],
                                &policy,
                            );
                        }
                        same(&edited.start(), &source.start(), &policy);
                        same(&edited.end(), &source.end(), &policy);
                        for pair in edited.curves().windows(2) {
                            same(&pair[0].end(), &pair[1].start(), &policy);
                        }
                        assert!(edited.curves().len() >= 3);
                        for inserted in &edited.curves()[1..edited.curves().len() - 1] {
                            let mut preparation =
                                crate::bezier_region::CornerCarrierPreparation2::from_curve(
                                    inserted, true,
                                );
                            preparation
                                .prepare(CurveOperation2::Fillet, &policy)
                                .unwrap();
                            let support = match preparation
                                .exact_carrier(true, CurveOperation2::Fillet, &policy)
                                .unwrap()
                            {
                                ExactCornerCarrier2::Arc(circle) => circle.clone(),
                                ExactCornerCarrier2::RetainedRationalArc(circle) => {
                                    circle.support().clone()
                                }
                                _ => panic!("the inserted span must retain an exact circle proof"),
                            };
                            assert_eq!(support.radius_squared(), q(1, 16384));
                            assert_eq!(
                                support.center(),
                                &Point2::new(q(-175, 4992), q(4699, 7488))
                            );
                            assert_eq!(support.is_clockwise(), !reversed);
                        }
                        same(
                            &edited.curves()[0].end(),
                            &contacts[usize::from(reversed)],
                            &policy,
                        );
                        same(
                            &edited.curves().last().unwrap().start(),
                            &contacts[usize::from(!reversed)],
                            &policy,
                        );
                        let replay = family
                            .clone()
                            .select(&parameter, &parameter, &CurveContext::STRICT)
                            .unwrap();
                        assert_eq!(replay.certainty, crate::CurveCertainty::Certified);
                        assert!(replay.value.is_some());
                        selected.push(edited);
                    }
                }
                assert_eq!(
                    selected.len(),
                    1,
                    "one component owns the selected contact pair"
                );
            }
        }
    }

    #[test]
    fn normalized_region_selects_and_reuses_a_continuous_fillet_family() {
        let parameter = CurveParameter2::from(q(5, 9));
        let join = CurvePoint2::from(Point2::new(Real::zero(), q(41, 64)));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let path = joined_parallel_path(&policy, true);
            let mut curves = path.curves().to_vec();
            let Classification::Decided(closing) =
                crate::BezierAlgebraicChord2::try_new(path.end(), path.start(), &policy).unwrap()
            else {
                panic!("the exact closing chord must be certified");
            };
            curves.push(closing.into());
            let closed = CurvePath2::try_new_with_policy(curves, &policy)
                .unwrap()
                .value;
            let source = crate::CurveRegion2::try_from_boundary_paths(&[closed], &policy).unwrap();
            assert_eq!(source.certainty, crate::CurveCertainty::Certified);
            let source = source.value.regularized_region(&policy).unwrap();
            assert_eq!(source.certainty, crate::CurveCertainty::Certified);
            let source = source.value;
            let mut edited = Vec::new();
            let mut corners = 0;
            let mut families = 0;
            for (loop_index, boundary) in source.boundary_loops().iter().enumerate() {
                for (vertex, curve) in boundary.curves().iter().enumerate() {
                    if curve.start().coincides_with(&join, &policy).value
                        != Classification::Decided(true)
                    {
                        continue;
                    }
                    corners += 1;
                    let result = source
                        .fillet_loop_vertex_by_radius(
                            loop_index,
                            vertex,
                            q(1, 128),
                            CurveCornerMode2::TrimOnly,
                            &policy,
                        )
                        .unwrap();
                    assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                    families += result.value.families().len();
                    for family in result.value.families() {
                        let selected = family.select(&parameter, &parameter, &policy).unwrap();
                        assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                        if let Some(region) = selected.value {
                            assert!(!region.is_empty());
                            let normalized = region.regularized_region(&policy).unwrap();
                            assert_eq!(normalized.certainty, crate::CurveCertainty::Certified);
                            let boolean =
                                region.boolean_regions(&normalized.value, &policy).unwrap();
                            assert_eq!(boolean.certainty, crate::CurveCertainty::Certified);
                            assert!(boolean.value.xor().is_empty());
                            // Continue through the other corner and set operations
                            // with the selected output as their actual input.
                            let apex = CurvePoint2::from(Point2::new(
                                q(-175, 4992) + q(12, 1664),
                                q(4699, 7488) + q(5, 1664),
                            ));
                            let (arc_loop, arc_vertex) = region
                                .boundary_loops()
                                .iter()
                                .enumerate()
                                .find_map(|(loop_index, boundary)| {
                                    boundary
                                        .curves()
                                        .iter()
                                        .position(|curve| {
                                            curve.start().coincides_with(&apex, &policy).value
                                                == Classification::Decided(true)
                                        })
                                        .map(|vertex| (loop_index, vertex))
                                })
                                .expect(
                                    "the semicircle's two rational charts retain their common apex",
                                );
                            let chamfered = region
                                .chamfer_loop_vertex_by_setbacks(
                                    arc_loop,
                                    arc_vertex,
                                    q(1, 4096),
                                    q(1, 4096),
                                    CurveCornerMode2::TrimOnly,
                                    &policy,
                                )
                                .unwrap();
                            assert_eq!(chamfered.certainty, crate::CurveCertainty::Certified);
                            let CurveCornerSolutions2::Unique(chamfered) = chamfered.value else {
                                panic!("one exact circular-seam chamfer");
                            };
                            let offset = chamfered
                                .offset(q(1, 4096), &crate::OffsetCornerStyle2::Round, &policy)
                                .unwrap();
                            assert_eq!(offset.certainty, crate::CurveCertainty::Certified);
                            let vertices = [
                                Point2::new(q(-175, 4992), Real::zero()),
                                Point2::from_values(1, 0),
                                Point2::from_values(1, 1),
                                Point2::new(q(-175, 4992), Real::one()),
                            ];
                            let clip = CurvePath2::try_new(
                                (0..4)
                                    .map(|i| {
                                        LineSeg2::try_new(
                                            vertices[i].clone(),
                                            vertices[(i + 1) % 4].clone(),
                                        )
                                        .unwrap()
                                        .into()
                                    })
                                    .collect(),
                            )
                            .unwrap();
                            let clip =
                                crate::CurveRegion2::try_from_boundary_paths(&[clip], &policy)
                                    .unwrap();
                            assert_eq!(clip.certainty, crate::CurveCertainty::Certified);
                            let clipped =
                                offset.value.boolean_regions(&clip.value, &policy).unwrap();
                            assert_eq!(clipped.certainty, crate::CurveCertainty::Certified);
                            assert!(!clipped.value.intersection().is_empty());
                            assert!(!clipped.value.difference().is_empty());
                            edited.push(region);
                        }
                    }
                }
            }
            assert_eq!(
                edited.len(),
                1,
                "one normalized boundary owns the selected fillet; corners={corners}, families={families}"
            );
        }
    }
}
