//! Corner contacts in a complete authored parameter domain.

use super::*;
use crate::CurveParameterRange2;
use std::borrow::Cow;

fn decided<T>(
    value: Classification<T>,
    operation: CurveOperation2,
    family: CurveFamily2,
) -> ExactCurveResult<T> {
    match value {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => {
            Err(ExactCurveError::blocked(operation, family, reason))
        }
    }
}

pub(super) fn parameter_order(
    first: &CurveParameter2,
    second: &CurveParameter2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<std::cmp::Ordering> {
    match first
        .cmp_by_refinement(second, policy)
        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?
    {
        Classification::Decided(order) => Ok(order),
        Classification::Uncertain(reason) => {
            Err(ExactCurveError::blocked(operation, family, reason))
        }
    }
}

/// Closed authored sweeps on one incident circular support. A chord's side
/// selects either the minor or major directed sweep without inverse parameter
/// reconstruction. The original selected endpoints remain its exact authority.
struct AuthoredCircularDomain2 {
    sweeps: Vec<(crate::BezierAlgebraicChord2, LineSide)>,
}

impl AuthoredCircularDomain2 {
    fn new(
        authored: &Curve2,
        incident: &CircularArc2,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let family = authored.family();
        let source = authored
            .source_range()
            .map_or(authored, |range| &range.source);
        let charts = source.native_bezier_fragments_for_operation(policy, operation)?;
        let mut sweeps = Vec::new();
        for chart in charts {
            let (start, end) = chart.parameter_range();
            let range =
                CurveParameterRange2::new_validated(start.clone().into(), end.clone().into());
            let Some([start, end]) = decided(
                crate::bezier_split::intersect_parameter_ranges(
                    authored.parameter_domain(),
                    &range,
                    policy,
                )
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?,
                operation,
                family,
            )?
            else {
                continue;
            };
            let Some(support) = native_span_circular_arc(chart, operation, family, policy)? else {
                continue;
            };
            // A crossing on a different support has a distinct source
            // preimage. Only another chart of this same circle owns a ray cut.
            if !same_circular_support(incident, &support, operation, family, policy)? {
                continue;
            }
            let start = authored
                .point_at_parameter_with_policy(&start, CurveParameterSide2::Right, policy)
                .map_err(|error| error.with_operation(operation))?;
            let end = authored
                .point_at_parameter_with_policy(&end, CurveParameterSide2::Left, policy)
                .map_err(|error| error.with_operation(operation))?;
            let chord = decided(
                crate::BezierAlgebraicChord2::try_new(start, end, policy)
                    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?,
                operation,
                family,
            )?;
            sweeps.push((
                chord,
                if support.is_clockwise() {
                    LineSide::Left
                } else {
                    LineSide::Right
                },
            ));
        }
        Ok(Self { sweeps })
    }

    /// The corner solve has already certified incidence on this circle.
    /// Closed finite ownership also excludes extensions at authored endpoints,
    /// even though those endpoints are not themselves admissible trim cuts.
    fn contains_incident_point(
        &self,
        point: &CurvePoint2,
        operation: CurveOperation2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        for (chord, interior) in &self.sweeps {
            let side = decided(
                chord
                    .oriented_support_side(point, policy)
                    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?,
                operation,
                family,
            )?;
            if side == LineSide::On || side == *interior {
                return Ok(true);
            }
        }
        Ok(false)
    }
}

fn same_circular_support(
    first: &CircularArc2,
    second: &CircularArc2,
    operation: CurveOperation2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    for (first, second) in [
        (first.center().x(), second.center().x()),
        (first.center().y(), second.center().y()),
        (first.radius_squared_ref(), second.radius_squared_ref()),
    ] {
        match crate::classify::compare_reals(first, second, policy) {
            Some(std::cmp::Ordering::Equal) => {}
            Some(_) => return Ok(false),
            None => {
                return Err(ExactCurveError::blocked(
                    operation,
                    family,
                    crate::UncertaintyReason::Ordering,
                ));
            }
        }
    }
    Ok(true)
}

impl Curve2 {
    /// Two cuts on one closed authored curve must leave a nonempty interval
    /// between them. Enumeration over several charts can also produce the
    /// opposite pairing, whose trims have already passed each other.
    pub(super) fn corner_cuts_leave_authored_interval(
        &self,
        previous: &CornerCut2,
        next: &CornerCut2,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        if (self.source_range().is_none()
            && !matches!(
                self.geometry(),
                Some(CurveGeometry2::PolynomialBSpline(_) | CurveGeometry2::Nurbs(_))
            ))
            || previous.placement == CornerPlacement2::Extension
            || next.placement == CornerPlacement2::Extension
        {
            return Ok(true);
        }
        let previous = previous.parameter.as_ref().expect("authored cut parameter");
        let next = next.parameter.as_ref().expect("authored cut parameter");
        let ordering = parameter_order(next, previous, operation, self.family(), policy)?;
        Ok(if self.source_range().is_some_and(|range| range.reversed) {
            ordering.is_gt()
        } else {
            ordering.is_lt()
        })
    }

    /// A rational chart partitions enumeration, never the authored trim domain.
    /// The incident chart still owns an extension ray. Every finite chart is
    /// searched from the same exact corner location before global placement.
    pub(super) fn chamfer_cuts_in_authored_domain(
        &self,
        incident: ExactCornerCarrier2<'_>,
        incident_chart: Option<(&Real, &Real)>,
        setback: &Real,
        setback_sign: RealSign,
        previous: bool,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CornerCuts2> {
        let operation = CurveOperation2::Chamfer;
        let family = self.family();
        let incident_cuts = || {
            let mut cuts = corner_chamfer_cuts(
                incident,
                setback,
                setback_sign,
                previous,
                mode,
                false,
                operation,
                family,
                policy,
            )?;
            for cut in cuts.iter_mut() {
                cut.map_source_parameter(incident_chart, operation, family, policy)?;
            }
            Ok(cuts)
        };
        let source = match self.source_range() {
            Some(range) => &range.source,
            None if matches!(
                self.geometry(),
                Some(CurveGeometry2::PolynomialBSpline(_) | CurveGeometry2::Nurbs(_))
            ) =>
            {
                self
            }
            _ => return incident_cuts(),
        };
        let charts = source.native_bezier_fragments_for_operation(policy, operation)?;
        if charts.len() == 1 || setback_sign == RealSign::Zero {
            return incident_cuts();
        }
        let range = self.parameter_domain();
        let corner_is_upper = previous != self.source_range().is_some_and(|range| range.reversed);
        let corner_parameter = if corner_is_upper {
            range.end()
        } else {
            range.start()
        };
        let mut center_chart = None;
        for (index, chart) in charts.iter().enumerate() {
            let (start, end) = chart.parameter_range();
            let lower = parameter_order(
                corner_parameter,
                &start.clone().into(),
                operation,
                family,
                policy,
            )?;
            let upper = parameter_order(
                corner_parameter,
                &end.clone().into(),
                operation,
                family,
                policy,
            )?;
            if lower.is_lt()
                || upper.is_gt()
                || (lower.is_eq() && corner_is_upper && index > 0)
                || (upper.is_eq() && !corner_is_upper && index + 1 < charts.len())
            {
                continue;
            }
            let inverse = (Real::one() / (end - start))
                .map_err(|cause| ExactCurveError::invalid(operation, family, cause.into()))?;
            let local = decided(
                corner_parameter
                    .affine_image_unbounded(&inverse, &(-start * &inverse), policy)
                    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?,
                operation,
                family,
            )?;
            center_chart = Some((
                exact_corner_bezier_parallel(
                    ExactCornerBezier2::NativeSpan(chart),
                    Real::zero(),
                    operation,
                    family,
                )?,
                local,
                index,
            ));
            break;
        }
        let (center, center_parameter, center_index) = center_chart.ok_or_else(|| {
            ExactCurveError::invalid(operation, family, CurveError::InvalidCurveParameter)
        })?;
        let incident_circle =
            native_span_circular_arc(&charts[center_index], operation, family, policy)?;
        let corner = self.endpoint(!previous);
        let circular_contacts = match (&incident_circle, corner.coordinates()) {
            (Some(circle), Some(corner)) => Some(circular_setback_points(
                circle, corner, setback, operation, family, policy,
            )?),
            _ => None,
        };
        let unit_range = CurveParameterRange2::unit();
        let mut cuts = CornerCuts2::default();
        let mut seam: Option<(CurveParameter2, CurvePoint2)> = None;
        for chart in charts {
            let previous_seam = seam.take();
            let (start, end) = chart.parameter_range();
            if !parameter_order(
                range.end(),
                &start.clone().into(),
                operation,
                family,
                policy,
            )?
            .is_gt()
                || !parameter_order(
                    range.start(),
                    &end.clone().into(),
                    operation,
                    family,
                    policy,
                )?
                .is_lt()
            {
                continue;
            }
            let circular = if circular_contacts.is_some() {
                match native_span_circular_arc(chart, operation, family, policy)? {
                    Some(circle) => same_circular_support(
                        incident_circle.as_ref().expect("circular contacts"),
                        &circle,
                        operation,
                        family,
                        policy,
                    )?,
                    None => false,
                }
            } else {
                false
            };
            let mut contact_points = Vec::new();
            let parallel;
            let parameters = if circular {
                parallel = None;
                let evaluator = RationalBezier2::try_from_subcurve(chart.curve())
                    .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?;
                let mut parameters = Vec::new();
                for point in circular_contacts
                    .as_ref()
                    .expect("circular contacts")
                    .iter()
                    .flatten()
                {
                    for parameter in decided(
                        evaluator
                            .retained_circle_point_parameters(point, policy)
                            .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?,
                        operation,
                        family,
                    )? {
                        parameters.push(CurveParameter2::from(parameter));
                        contact_points.push(point);
                    }
                }
                parameters
            } else {
                parallel = Some(exact_corner_bezier_parallel(
                    ExactCornerBezier2::NativeSpan(chart),
                    Real::zero(),
                    operation,
                    family,
                )?);
                decided(
                    parallel
                        .as_ref()
                        .expect("generic contact carrier")
                        .fixed_distance_incidence(
                            &center,
                            &center_parameter,
                            setback,
                            &unit_range,
                            None,
                            policy,
                        )
                        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?,
                    operation,
                    family,
                )?
            };
            for (index, chart_parameter) in parameters.into_iter().enumerate() {
                let parameter = decided(
                    chart_parameter
                        .affine_image_unbounded(&(end - start), start, policy)
                        .map_err(|cause| ExactCurveError::invalid(operation, family, cause))?,
                    operation,
                    family,
                )?;
                if !parameter_order(&parameter, range.start(), operation, family, policy)?.is_gt()
                    || !parameter_order(&parameter, range.end(), operation, family, policy)?.is_lt()
                {
                    continue;
                }
                let point = if let Some(point) = contact_points.get(index) {
                    (*point).clone().into()
                } else {
                    analytic_parallel_point_evidence(
                        parallel.as_ref().expect("generic contact carrier"),
                        &chart_parameter,
                        operation,
                        family,
                        policy,
                    )?
                };
                // Only adjacent closed cells can duplicate a source location.
                // Retain that seam witness instead of comparing every new
                // root with every previously published cut. Distinct source
                // parameters at a self-contact remain distinct solutions.
                if let Some((previous_parameter, previous_point)) = &previous_seam
                    && parameter_order(&parameter, previous_parameter, operation, family, policy)?
                        .is_eq()
                    && decided(point.same_point(previous_point, policy), operation, family)?
                {
                    continue;
                }
                if parameter_order(&parameter, &end.clone().into(), operation, family, policy)?
                    .is_eq()
                {
                    seam = Some((parameter.clone(), point.clone()));
                }
                cuts.push(CornerCut2 {
                    parameter: Some(parameter),
                    point,
                    placement: CornerPlacement2::Trim,
                });
            }
        }
        if mode == CurveCornerMode2::TrimOrExtend {
            let incident = incident_cuts()?;
            if incident
                .iter()
                .any(|cut| cut.placement == CornerPlacement2::Extension)
            {
                let circle_domain = incident_circle
                    .as_ref()
                    .map(|circle| AuthoredCircularDomain2::new(self, circle, operation, policy))
                    .transpose()?;
                for cut in incident.iter() {
                    if cut.placement != CornerPlacement2::Extension {
                        continue;
                    }
                    if let Some(domain) = &circle_domain
                        && domain.contains_incident_point(&cut.point, operation, family, policy)?
                    {
                        continue;
                    }
                    cuts.push(cut.clone());
                }
            }
        }
        Ok(cuts)
    }
}

/// One finite source chart. The optional map names its location in the authored
/// curve; an unpartitioned carrier keeps its established support machinery.
struct FilletSourceChart2<'a> {
    curve: Cow<'a, Curve2>,
    source_map: Option<(Real, Real)>,
}

impl FilletSourceChart2<'_> {
    fn prepare(
        &self,
        previous: bool,
        policy: &CurveContext,
    ) -> ExactCurveResult<crate::bezier_region::CornerCarrierPreparation2<'_>> {
        let mut preparation =
            crate::bezier_region::CornerCarrierPreparation2::from_curve(&self.curve, previous);
        preparation.prepare(CurveOperation2::Fillet, policy)?;
        Ok(preparation)
    }

    fn domain(&self, mode: CurveCornerMode2) -> FilletContactDomain2 {
        if self.source_map.is_some() {
            FilletContactDomain2::SourceChart(mode)
        } else {
            FilletContactDomain2::AuthoredCurve(mode)
        }
    }

    fn place_cut(
        &self,
        authored: &Curve2,
        preparation: &crate::bezier_region::CornerCarrierPreparation2<'_>,
        cut: &mut CornerCut2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        let operation = CurveOperation2::Fillet;
        cut.map_source_parameter(
            preparation.source_chart(),
            operation,
            authored.family(),
            policy,
        )?;
        let Some((scale, offset)) = &self.source_map else {
            return Ok(true);
        };
        cut.map_source_parameter(Some((scale, offset)), operation, authored.family(), policy)?;
        if cut.placement == CornerPlacement2::Extension {
            return Ok(true);
        }
        let parameter = cut
            .parameter
            .as_ref()
            .expect("a finite chart retains its source parameter");
        let range = authored.parameter_domain();
        Ok(parameter_order(
            parameter,
            range.start(),
            operation,
            authored.family(),
            policy,
        )?
        .is_gt()
            && parameter_order(parameter, range.end(), operation, authored.family(), policy)?
                .is_lt())
    }
}

impl Curve2 {
    fn finite_fillet_charts(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<FilletSourceChart2<'_>>> {
        let operation = CurveOperation2::Fillet;
        if let Some(spans) = self.restricted_source_spans(policy, operation)? {
            if spans.len() > 1 {
                return Ok(spans
                    .iter()
                    .map(|span| FilletSourceChart2 {
                        curve: Cow::Owned(Curve2::from_retained_fragment(span.fragment.clone())),
                        source_map: Some((span.source_scale.clone(), span.source_offset.clone())),
                    })
                    .collect());
            }
        } else if matches!(
            self.geometry(),
            Some(CurveGeometry2::PolynomialBSpline(_) | CurveGeometry2::Nurbs(_))
        ) {
            let spans = self.native_bezier_fragments_for_operation(policy, operation)?;
            if spans.len() > 1 {
                return Ok(spans
                    .iter()
                    .map(|span| {
                        let (start, end) = span.parameter_range();
                        FilletSourceChart2 {
                            curve: Cow::Owned(span.clone().into_curve()),
                            source_map: Some((end - start, start.clone())),
                        }
                    })
                    .collect());
            }
        }
        Ok(vec![FilletSourceChart2 {
            curve: Cow::Borrowed(self),
            source_map: None,
        }])
    }
}

impl CurvePath2 {
    /// Enumerates all finite chart pairs while keeping the outer trim domain
    /// authoritative. Each preparation is reused across the opposite charts.
    /// Internal endpoint contacts belong to the chart that survives the cut,
    /// so a seam needs neither duplicate publication nor all-pairs deduplication.
    pub(super) fn fillets_in_authored_domain(
        &self,
        vertex_index: usize,
        previous_index: usize,
        next_index: usize,
        radius: &Real,
        mode: CurveCornerMode2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CurveCornerSolutions2<Self>>> {
        let operation = CurveOperation2::Fillet;
        let previous = &self.data.curves[previous_index];
        let next = &self.data.curves[next_index];
        let can_partition = |curve: &Curve2| {
            curve.source_range().is_some()
                || matches!(
                    curve.geometry(),
                    Some(CurveGeometry2::PolynomialBSpline(_) | CurveGeometry2::Nurbs(_))
                )
        };
        if !can_partition(previous) && !can_partition(next) {
            return Ok(None);
        }
        let previous_charts = previous.finite_fillet_charts(policy)?;
        let next_charts = next.finite_fillet_charts(policy)?;
        if previous_charts.len() == 1 && next_charts.len() == 1 {
            return Ok(None);
        }
        let previous_sources = previous_charts
            .iter()
            .map(|chart| chart.prepare(true, policy))
            .collect::<ExactCurveResult<Vec<_>>>()?;
        let next_sources = next_charts
            .iter()
            .map(|chart| chart.prepare(false, policy))
            .collect::<ExactCurveResult<Vec<_>>>()?;
        let mut candidates = [Vec::new(), Vec::new()];
        let mut previous_circle_domain = None;
        let mut next_circle_domain = None;
        for (previous_chart_index, (previous_chart, previous_source)) in
            previous_charts.iter().zip(&previous_sources).enumerate()
        {
            for (next_chart_index, (next_chart, next_source)) in
                next_charts.iter().zip(&next_sources).enumerate()
            {
                if previous_index == next_index && previous_chart_index < next_chart_index {
                    // The surviving interval runs from the next cut to the
                    // previous cut. These disjoint charts have already crossed
                    // before any contact equation needs to be constructed.
                    continue;
                }
                let previous_carrier = previous_source.exact_carrier(true, operation, policy)?;
                let next_carrier = next_source.exact_carrier(false, operation, policy)?;
                let previous_arc = previous_carrier.retained_rational_arc_support().cloned();
                let next_arc = next_carrier.retained_rational_arc_support().cloned();
                let domains = [
                    previous_chart.domain(if previous_chart_index + 1 == previous_charts.len() {
                        mode
                    } else {
                        CurveCornerMode2::TrimOnly
                    }),
                    next_chart.domain(if next_chart_index == 0 {
                        mode
                    } else {
                        CurveCornerMode2::TrimOnly
                    }),
                ];
                // These charts need not meet at the authored vertex. The
                // connected-line shortcut therefore does not apply here.
                let solutions = solve_carrier_fillet_corner(
                    previous_carrier,
                    next_carrier,
                    radius,
                    false,
                    domains,
                    previous.family(),
                    next.family(),
                    policy,
                )?;
                try_map_corner_solutions(solutions, |mut solution| {
                    if !previous_chart.place_cut(
                        previous,
                        previous_source,
                        &mut solution.previous,
                        policy,
                    )? || !next_chart.place_cut(next, next_source, &mut solution.next, policy)?
                    {
                        return Ok(());
                    }
                    for (chart, authored, cut, circle, cached) in [
                        (
                            previous_chart,
                            previous,
                            &solution.previous,
                            previous_arc.as_ref(),
                            &mut previous_circle_domain,
                        ),
                        (
                            next_chart,
                            next,
                            &solution.next,
                            next_arc.as_ref(),
                            &mut next_circle_domain,
                        ),
                    ] {
                        if chart.source_map.is_some()
                            && cut.placement == CornerPlacement2::Extension
                            && let Some(circle) = circle
                        {
                            if cached.is_none() {
                                *cached = Some(AuthoredCircularDomain2::new(
                                    authored, circle, operation, policy,
                                )?);
                            }
                            if cached
                                .as_ref()
                                .expect("authored circle domain")
                                .contains_incident_point(
                                    &cut.point,
                                    operation,
                                    authored.family(),
                                    policy,
                                )?
                            {
                                return Ok(());
                            }
                        }
                    }
                    let clockwise = solution.clockwise;
                    if let Some(path) = self.publish_fillet_corner(
                        vertex_index,
                        previous_index,
                        next_index,
                        solution,
                        radius,
                        mode,
                        [previous_arc.as_ref(), next_arc.as_ref()],
                        [
                            previous_source.promoted_parallel(),
                            next_source.promoted_parallel(),
                        ],
                        policy,
                    )? {
                        candidates[usize::from(clockwise)].push(path);
                    }
                    Ok(())
                })?;
            }
        }
        let mut solutions = CornerSolutionAccumulator::Empty;
        for candidate in candidates.into_iter().flatten() {
            solutions.push(candidate);
        }
        Ok(Some(
            solutions.finish(CurveCornerNoSolution2::OutsideTrimDomain),
        ))
    }
}
