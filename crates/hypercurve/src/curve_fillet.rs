//! Exact fillet requests, constraint replay, and corner reconstruction.

use super::*;

/// An exact tangency constraint on one of the incident curves.
#[derive(Clone, Debug, PartialEq)]
pub enum CurveFilletContact2 {
    /// Selects one location in the incident input curve's parameter chart.
    /// Retained algebraic parameters keep their source and root evidence.
    Parameter(CurveParameter2),
    /// Selects every incident source location at this exact point.
    /// This also names circular extension contacts outside the authored chart.
    Point(CurvePoint2),
}

/// Exact design data for a circular fillet.
///
/// A radius may leave a continuous family. Add a center or contact constraint
/// until the admissible solutions are finite. Constraints are conjunctive;
/// contacts are in previous/next traversal order and use the input curves' charts.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveFillet2 {
    /// The nonnegative exact radius.
    pub radius: Real,
    /// An optional exact center.
    pub center: Option<CurvePoint2>,
    /// Optional exact previous and next tangency contacts.
    pub contacts: [Option<CurveFilletContact2>; 2],
}

impl CurveFillet2 {
    /// Requests every admissible fillet of this radius.
    pub fn new(radius: Real) -> Self {
        Self {
            radius,
            center: None,
            contacts: [None, None],
        }
    }

    pub(crate) fn has_constraints(&self) -> bool {
        self.center.is_some() || self.contacts.iter().any(Option::is_some)
    }
}

fn constraint_required(family: CurveFamily2) -> ExactCurveError {
    ExactCurveError::invalid(
        CurveOperation2::Fillet,
        family,
        CurveError::FilletConstraintRequired,
    )
}

fn fillet_point_matches(
    actual: &CurvePoint2,
    requested: &CurvePoint2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    match actual.same_point(requested, policy) {
        Classification::Decided(matches) => Ok(matches),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    }
}

fn inverse_fillet_parameter(
    parameter: &CurveParameter2,
    scale: &Real,
    offset: &Real,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<CurveParameter2> {
    let inverse = (Real::one() / scale)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause.into()))?;
    match parameter
        .affine_image_unbounded(&inverse, &(-offset * &inverse), policy)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
    {
        Classification::Decided(parameter) => Ok(parameter),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Fillet,
            family,
            reason,
        )),
    }
}

#[derive(Clone, Copy)]
pub(crate) enum FilletContactChart2 {
    Parameter,
    RationalCircle,
    CircularSweep,
}

impl FilletContactChart2 {
    pub(crate) fn for_curve(curve: &Curve2, rational_circle: bool) -> Self {
        if rational_circle {
            Self::RationalCircle
        } else if curve.family() == CurveFamily2::CircularArc {
            Self::CircularSweep
        } else {
            Self::Parameter
        }
    }

    fn retains_parameter(self, cut: &CornerCut2) -> bool {
        match self {
            Self::Parameter => true,
            // Finite rational circle cuts retain source parameters. Their
            // geometric points can repeat in different authored spline spans.
            // Circular extensions and native sweeps may keep endpoint markers.
            Self::RationalCircle => cut.placement != CornerPlacement2::Extension,
            Self::CircularSweep => false,
        }
    }
}

pub(crate) struct FilletConstraintBinding2<'a> {
    pub(crate) request: &'a CurveFillet2,
    pub(crate) sources: [&'a Curve2; 2],
    pub(crate) maps: [Option<(&'a Real, &'a Real)>; 2],
    pub(crate) charts: [FilletContactChart2; 2],
}

impl FilletConstraintBinding2<'_> {
    fn source_parameter(
        &self,
        axis: usize,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveParameter2> {
        match self.maps[axis] {
            Some((scale, offset)) => inverse_fillet_parameter(
                parameter,
                scale,
                offset,
                self.sources[axis].family(),
                policy,
            ),
            None => Ok(parameter.clone()),
        }
    }

    fn parameter_point(
        &self,
        axis: usize,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePoint2> {
        let input = self.sources[axis];
        let source = input.source_range().map_or(input, |range| &range.source);
        // A promoted chord's monotone location is not the authored Bezier
        // parameter. Evaluate that chart directly, including permitted
        // extensions, and compare the exact contact points instead.
        let rationalize = |curve: BezierSubcurve2| {
            RationalBezier2::try_from_subcurve(&curve).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Fillet, source.family(), cause)
            })
        };
        let rational = match source.geometry() {
            Some(CurveGeometry2::Line(line)) => Some(rationalize(BezierSubcurve2::Quadratic(
                QuadraticBezier2::from_line_segment(line.clone()),
            ))?),
            Some(CurveGeometry2::QuadraticBezier(curve)) => {
                Some(rationalize(BezierSubcurve2::Quadratic(curve.clone()))?)
            }
            Some(CurveGeometry2::CubicBezier(curve)) => {
                Some(rationalize(BezierSubcurve2::Cubic(curve.clone()))?)
            }
            Some(CurveGeometry2::RationalQuadraticBezier(curve)) => {
                Some(RationalBezier2::from(curve.clone()))
            }
            Some(CurveGeometry2::RationalBezier(curve)) => Some(curve.clone()),
            _ => match source.retained_fragment() {
                Some(crate::BezierSplitFragment2::RetainedBezier { source_curve, .. }) => Some(
                    RationalBezier2::try_from_subcurve(source_curve).map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, source.family(), cause)
                    })?,
                ),
                Some(crate::BezierSplitFragment2::SelectedFiber(source)) => {
                    source.rational_curve().cloned()
                }
                _ => None,
            },
        };
        if let Some(rational) = rational {
            return curve_evaluation::rational_point(&rational, parameter, input.family(), policy)
                .map_err(|error| error.with_operation(CurveOperation2::Fillet));
        }
        input
            .point_at(parameter, policy)
            .map(|outcome| outcome.value)
            .map_err(|error| error.with_operation(CurveOperation2::Fillet))
    }

    fn parallel_contact_parameters(
        &self,
        axis: usize,
        source: FilletParallelSource2<'_>,
        center_support: &BezierParallel2,
        domain: FilletContactDomain2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<FilletContactSelection2> {
        Ok(match &self.request.contacts[axis] {
            Some(CurveFilletContact2::Parameter(parameter)) => {
                let parameter = self.source_parameter(axis, parameter, policy)?;
                let parameter = match source {
                    FilletParallelSource2::Direct(source) => {
                        let (start, end) = source.parameter_range();
                        inverse_fillet_parameter(&parameter, &(end - start), start, family, policy)?
                    }
                    _ => parameter,
                };
                FilletContactSelection2::Parameters(vec![parameter])
            }
            Some(CurveFilletContact2::Point(point)) => {
                let support = match source {
                    FilletParallelSource2::Direct(_) => center_support.with_distance(Real::zero()),
                    FilletParallelSource2::Retained(source) => source.parallel().clone(),
                    FilletParallelSource2::Selected(source) => source.parallel_carrier(),
                };
                let incident = (domain.mode() == CurveCornerMode2::TrimOrExtend)
                    .then(|| source.incident_domain(center_support, axis == 0, family, policy))
                    .transpose()?;
                fillet_point_parameters(
                    &support,
                    point,
                    &source.curve_parameter_range(),
                    incident.as_ref(),
                    false,
                    family,
                    policy,
                )?
            }
            None => FilletContactSelection2::Any,
        })
    }

    pub(super) fn matches(
        &self,
        corner: &FilletCorner2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        let family = self.sources[0].family();
        if let Some(center) = &self.request.center
            && !fillet_point_matches(&corner.center, center, family, policy)?
        {
            return Ok(false);
        }
        for (axis, cut) in [&corner.previous, &corner.next].into_iter().enumerate() {
            if !self.matches_contact(axis, cut, policy)? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn matches_contact(
        &self,
        axis: usize,
        cut: &CornerCut2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        let Some(requested) = &self.request.contacts[axis] else {
            return Ok(true);
        };
        let family = self.sources[axis].family();
        match requested {
            CurveFilletContact2::Point(point) => {
                if !fillet_point_matches(&cut.point, point, family, policy)? {
                    return Ok(false);
                }
            }
            CurveFilletContact2::Parameter(parameter) => {
                if self.charts[axis].retains_parameter(cut)
                    && let Some(actual) = &cut.parameter
                    && (actual.as_algebraic_chord().is_none()
                        || parameter.as_algebraic_chord().is_some())
                {
                    let parameter = self.source_parameter(axis, parameter, policy)?;
                    match actual.same_value(&parameter, policy).map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                    })? {
                        Classification::Decided(true) => (),
                        Classification::Decided(false) => return Ok(false),
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                family,
                                reason,
                            ));
                        }
                    }
                } else {
                    let point = self.parameter_point(axis, parameter, policy)?;
                    if !fillet_point_matches(&cut.point, &point, family, policy)? {
                        return Ok(false);
                    }
                }
            }
        }
        Ok(true)
    }
}

#[allow(clippy::too_many_arguments)]
fn constrained_coincident_fillet_at_center(
    offsets: [&FilletOffsetCarrier2<'_, '_>; 2],
    point: CurvePoint2,
    clockwise: bool,
    retain_selected_circle_endpoints: bool,
    domains: [FilletContactDomain2; 2],
    families: [CurveFamily2; 2],
    binding: Option<&FilletConstraintBinding2<'_>>,
    policy: &CurveContext,
) -> ExactCurveResult<CurveCornerSolutions2<FilletCorner2>> {
    let center = FilletCenterWitness2 {
        source_frames: [None, None],
        point,
        previous_parameter: None,
        next_parameter: None,
        // Both family resolvers have certified opposed source tangents.
        // Preserve that semicircle proof for reconstruction instead of
        // asking a general parameter map to rediscover it from the points.
        retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
            cross: Some(RealSign::Zero),
            dot: Some(RealSign::Negative),
            center_parallel: None,
            source_direction: None,
            canonical_anchor_curve: None,
            deferred_arc_contact: None,
        }),
    };
    let empty = |reason| Ok(CurveCornerSolutions2::NoSolution(reason));
    match fillet_corner_from_center(
        offsets[0],
        offsets[1],
        &center,
        clockwise,
        retain_selected_circle_endpoints,
        domains,
        families[0],
        families[1],
        policy,
    )? {
        FilletCornerSelection2::Selected(corner) => {
            if let Some(binding) = binding
                && !binding.matches(&corner, policy)?
            {
                return empty(CurveCornerNoSolution2::UnsatisfiedConstraints);
            }
            Ok(CurveCornerSolutions2::Unique(corner))
        }
        FilletCornerSelection2::Outside => empty(CurveCornerNoSolution2::OutsideTrimDomain),
        FilletCornerSelection2::Degenerate => empty(CurveCornerNoSolution2::DegenerateCandidate),
    }
}

/// Coincident noncollapsed circles retain a radial contact correspondence.
/// A supplied center or contact fixes it; otherwise only domain intersection
/// and degeneracy are decided, without publishing a representative fillet.
#[allow(clippy::too_many_arguments)]
pub(super) fn constrained_coincident_circular_fillet(
    offsets: [&FilletOffsetCarrier2<'_, '_>; 2],
    clockwise: bool,
    retain_selected_circle_endpoints: bool,
    domains: [FilletContactDomain2; 2],
    families: [CurveFamily2; 2],
    binding: Option<&FilletConstraintBinding2<'_>>,
    policy: &CurveContext,
) -> ExactCurveResult<CurveCornerSolutions2<FilletCorner2>> {
    use crate::segment::ArcSweepPointLocation2;
    let arc = |axis: usize| match offsets[axis] {
        FilletOffsetCarrier2::Arc {
            source,
            source_radius,
            signed_radius,
        } => (*source, *source_radius, signed_radius),
        _ => unreachable!("coincident circular fillets retain both source arcs"),
    };
    let arcs = [arc(0), arc(1)];
    let empty = |reason| Ok(CurveCornerSolutions2::NoSolution(reason));
    let invalid = |cause| ExactCurveError::invalid(CurveOperation2::Fillet, families[0], cause);
    let blocked = |reason| ExactCurveError::blocked(CurveOperation2::Fillet, families[0], reason);
    // Contact_i = origin + (source_radius_i / signed_radius_i) * (center-origin).
    // The common center support is nonzero. Equal radial factors therefore
    // identify the same contact everywhere, and only those families collapse.
    match crate::classify::is_zero(&(arcs[0].1 * arcs[1].2 - arcs[1].1 * arcs[0].2), policy) {
        Some(true) => return empty(CurveCornerNoSolution2::DegenerateCandidate),
        Some(false) => (),
        None => return Err(blocked(crate::UncertaintyReason::RealSign)),
    }
    let mut center = binding.and_then(|binding| binding.request.center.clone());
    if center.is_none()
        && let Some(binding) = binding
    {
        for (axis, contact) in binding.request.contacts.iter().enumerate() {
            let Some(contact) = contact else { continue };
            let point = match contact {
                CurveFilletContact2::Point(point) => point.clone(),
                CurveFilletContact2::Parameter(parameter) => {
                    binding.parameter_point(axis, parameter, policy)?
                }
            };
            let (source, source_radius, signed_radius) = arcs[axis];
            match crate::bezier_offset::retained_point_circle_incidence_sign(
                &point,
                source.support().center(),
                source.support().radius_squared_ref(),
                policy,
            )
            .map_err(invalid)?
            {
                Classification::Decided(RealSign::Zero) => (),
                Classification::Decided(_) => {
                    return empty(CurveCornerNoSolution2::UnsatisfiedConstraints);
                }
                Classification::Uncertain(reason) => return Err(blocked(reason)),
            }
            let scale = (signed_radius / source_radius).map_err(|cause| invalid(cause.into()))?;
            center = Some(
                match crate::BezierAlgebraicChord2::scaled_about_point_endpoint(
                    &point,
                    source.support().center(),
                    &scale,
                    policy,
                )
                .map_err(invalid)?
                {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => return Err(blocked(reason)),
                },
            );
            break;
        }
    }
    if let Some(center) = center {
        if binding.is_some_and(|binding| binding.request.center.is_some())
            && !point_on_fillet_offset(&center, offsets[0], true, domains[0], families[0], policy)?
        {
            return empty(CurveCornerNoSolution2::UnsatisfiedConstraints);
        }
        return constrained_coincident_fillet_at_center(
            offsets,
            center,
            clockwise,
            retain_selected_circle_endpoints,
            domains,
            families,
            binding,
            policy,
        );
    }
    // Circular extension supplies the full center circle apart from excluded
    // endpoints. Intersecting it with a nonzero authored arc leaves a family.
    if domains
        .iter()
        .any(|domain| domain.mode() == CurveCornerMode2::TrimOrExtend)
    {
        return Err(constraint_required(families[0]));
    }
    let center_scales = [
        (arcs[0].2 / arcs[0].1).map_err(|cause| invalid(cause.into()))?,
        (arcs[1].2 / arcs[1].1).map_err(|cause| invalid(cause.into()))?,
    ];
    let source_scales = [
        (arcs[0].1 / arcs[0].2).map_err(|cause| invalid(cause.into()))?,
        (arcs[1].1 / arcs[1].2).map_err(|cause| invalid(cause.into()))?,
    ];
    let radial_image = |axis: usize, point: &CurvePoint2, scale: &Real| {
        match crate::BezierAlgebraicChord2::scaled_about_point_endpoint(
            point,
            arcs[axis].0.support().center(),
            scale,
            policy,
        )
        .map_err(invalid)?
        {
            Classification::Decided(point) => Ok(point),
            Classification::Uncertain(reason) => Err(blocked(reason)),
        }
    };
    let location = |axis: usize, center: &CurvePoint2| {
        let point = radial_image(axis, center, &source_scales[axis])?;
        match arcs[axis].0 {
            ExactCornerArc2::Native(source) => match source
                .strict_incident_point_evidence_location(&point, policy)
                .map_err(invalid)?
            {
                Classification::Decided(location) => Ok(location),
                Classification::Uncertain(reason) => Err(blocked(reason)),
            },
            ExactCornerArc2::RetainedRational(source) => {
                let Some(parameter) = RetainedRationalCornerArc2::parameter_at_incident_point(
                    source
                        .fragment
                        .rational_curve()
                        .expect("a rational circular chart"),
                    &point,
                    CurveOperation2::Fillet,
                    families[axis],
                    policy,
                )?
                else {
                    return Ok(ArcSweepPointLocation2::Outside);
                };
                let compare = |boundary| {
                    curve_corner_domain::parameter_order(
                        &parameter,
                        boundary,
                        CurveOperation2::Fillet,
                        families[axis],
                        policy,
                    )
                };
                let start = compare(source.fragment.range().start())?;
                let end = compare(source.fragment.range().end())?;
                Ok(if start.is_lt() || end.is_gt() {
                    ArcSweepPointLocation2::Outside
                } else if start.is_eq() || end.is_eq() {
                    ArcSweepPointLocation2::Endpoint
                } else {
                    ArcSweepPointLocation2::Interior
                })
            }
        }
    };
    let mut endpoints: Vec<CurvePoint2> = Vec::new();
    for axis in 0..2 {
        // A rational chart's supporting arc can be wider than its surviving
        // interval. Map the actual endpoints and classify in that interval,
        // rather than inferring a family from the untrimmed parent sweep.
        let boundaries = match arcs[axis].0 {
            ExactCornerArc2::Native(source) => {
                [source.start().clone().into(), source.end().clone().into()]
            }
            ExactCornerArc2::RetainedRational(source) => [
                source.fragment.start_point().clone(),
                source.fragment.end_point().clone(),
            ],
        };
        for point in &boundaries {
            let center = radial_image(axis, point, &center_scales[axis])?;
            match location(1 - axis, &center)? {
                ArcSweepPointLocation2::Interior => return Err(constraint_required(families[0])),
                ArcSweepPointLocation2::Outside => continue,
                ArcSweepPointLocation2::Endpoint => (),
            }
            let mut duplicate = false;
            for known in &endpoints {
                if fillet_point_matches(known, &center, families[0], policy)? {
                    duplicate = true;
                    break;
                }
            }
            if !duplicate {
                endpoints.push(center);
            }
        }
    }
    if endpoints.is_empty() {
        return empty(CurveCornerNoSolution2::OutsideTrimDomain);
    }
    // With no endpoint strictly inside the other sweep, an open overlap can
    // only have coincident boundaries (including full circles). One certified
    // interior point distinguishes identical sweeps from complementary sweeps.
    // It is a domain witness only, never a selected center returned to callers.
    let interior = match match arcs[0].0 {
        ExactCornerArc2::Native(source) => source.representative_point(policy),
        ExactCornerArc2::RetainedRational(source) => source.fragment.representative_point(policy),
    }
    .map_err(invalid)?
    {
        Classification::Decided(point) => point.into(),
        Classification::Uncertain(reason) => return Err(blocked(reason)),
    };
    let interior = radial_image(0, &interior, &center_scales[0])?;
    if location(1, &interior)? != ArcSweepPointLocation2::Outside {
        return Err(constraint_required(families[0]));
    }
    let mut candidates = CornerSolutionAccumulator::Empty;
    for center in endpoints {
        candidates.append(constrained_coincident_fillet_at_center(
            offsets,
            center,
            clockwise,
            retain_selected_circle_endpoints,
            domains,
            families,
            binding,
            policy,
        )?);
    }
    Ok(candidates.finish(CurveCornerNoSolution2::OutsideTrimDomain))
}

/// Recovers a represented solve chart from a certified coincidence sample.
/// The chord keeps its independent endpoint fields and all domain ownership.
pub(super) fn coincident_linear_source_chart(
    source: &crate::BezierAlgebraicChord2,
    parallel: &crate::BezierParallel2,
    sample: &Real,
    signed_distance: &Real,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<LineSeg2> {
    let invalid = |cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause);
    let blocked = |reason| ExactCurveError::blocked(CurveOperation2::Fillet, family, reason);
    let point = match parallel.point_at(sample, policy).map_err(invalid)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Err(blocked(reason)),
    };
    let derivative = match parallel.derivative_at(sample, policy).map_err(invalid)? {
        Classification::Decided(derivative) => derivative,
        Classification::Uncertain(reason) => return Err(blocked(reason)),
    };
    let mut tangent = (derivative.dx().clone(), derivative.dy().clone());
    if crate::classify::real_sign(&(&tangent.0 * &tangent.0 + &tangent.1 * &tangent.1), policy)
        == Some(RealSign::Zero)
    {
        // A collapsed parallel contributes a point, not an affine line chart.
        return Err(blocked(crate::UncertaintyReason::Boundary));
    }
    let (mut unit_x, mut unit_y, _) = line_unit_direction(
        &tangent.0,
        &tangent.1,
        CurveOperation2::Fillet,
        family,
        policy,
    )?;
    match source
        .tangent_dot_vector_sign(&tangent, policy)
        .map_err(invalid)?
    {
        Classification::Decided(RealSign::Positive) => (),
        Classification::Decided(RealSign::Negative) => {
            unit_x = -unit_x;
            unit_y = -unit_y;
            tangent.0 = -tangent.0;
            tangent.1 = -tangent.1;
        }
        Classification::Decided(RealSign::Zero) => {
            return Err(invalid(CurveError::Topology(
                "a coincident parallel had a transverse source tangent".into(),
            )));
        }
        Classification::Uncertain(reason) => return Err(blocked(reason)),
    }
    let start = point.translated(signed_distance * &unit_y, -(signed_distance * &unit_x));
    // Preserve the companion's derivative scale in the solve chart. Only the
    // normal displacement needs a unit vector; spreading its radical into the
    // parameter equations can destroy a compact source correspondence.
    let line = LineSeg2::new_unchecked(start.clone(), start.translated(tangent.0, tangent.1));
    match policy
        .strict_predicate_pass(|| source.has_non_collinear_support_with_exact_line(&line, policy))
        .map_err(invalid)?
    {
        Classification::Decided(false) => Ok(line),
        Classification::Decided(true) => Err(invalid(CurveError::Topology(
            "a coincident parallel sample did not recover its original chord support".into(),
        ))),
        Classification::Uncertain(reason) => Err(blocked(reason)),
    }
}

/// Replays a mixed coincident support through the shared parameter kernel.
/// A retained chord contributes an affine solve chart with its exact original
/// endpoints; the nonlinear companion keeps every source parameter and sheet.
#[allow(clippy::too_many_arguments)]
pub(super) fn replay_coincident_linear_parallel_fillet(
    prepared: [&PreparedFilletCarrier2<'_>; 2],
    linear_support: Option<&LineSeg2>,
    radius: &Real,
    retain_selected_circle_endpoints: bool,
    domains: [FilletContactDomain2; 2],
    families: [CurveFamily2; 2],
    binding: Option<&FilletConstraintBinding2<'_>>,
    policy: &CurveContext,
) -> ExactCurveResult<CurveCornerSolutions2<FilletCorner2>> {
    let (linear, parallel, axis) = match prepared {
        [
            linear @ (PreparedFilletCarrier2::Line { .. }
            | PreparedFilletCarrier2::AlgebraicChord { .. }),
            PreparedFilletCarrier2::Parallel { source, .. },
        ] => (linear, *source, 0),
        [
            PreparedFilletCarrier2::Parallel { source, .. },
            linear @ (PreparedFilletCarrier2::Line { .. }
            | PreparedFilletCarrier2::AlgebraicChord { .. }),
        ] => (linear, *source, 1),
        _ => unreachable!("a mixed coincident support retains its line and parallel sources"),
    };
    let (source, chord_support) = match linear {
        PreparedFilletCarrier2::Line {
            source,
            chord_support,
            ..
        } => (*source, chord_support.as_ref()),
        PreparedFilletCarrier2::AlgebraicChord { source } => {
            (FilletLinearSource2::AlgebraicChord(source), linear_support)
        }
        _ => unreachable!(),
    };
    let invalid = |cause| ExactCurveError::invalid(CurveOperation2::Fillet, families[axis], cause);
    let blocked =
        |reason| ExactCurveError::blocked(CurveOperation2::Fillet, families[axis], reason);
    let line = match &source {
        FilletLinearSource2::Native {
            source,
            parameterization,
            ..
        } => parameterization.map_or_else(
            || Curve2::from(QuadraticBezier2::from_line_segment((*source).clone())),
            |curve| Curve2::from(curve.clone()),
        ),
        FilletLinearSource2::AlgebraicChord(chord) => {
            let support = chord_support.expect("a represented chord support is prepared once");
            let parameter =
                |point| match crate::bezier_offset::affine_line_parameter_at_incident_point(
                    support, point, policy,
                )
                .map_err(invalid)?
                {
                    Classification::Decided(parameter) => Ok(parameter),
                    Classification::Uncertain(reason) => Err(blocked(reason)),
                };
            let reversed = match chord
                .tangent_dot_vector_sign(&support.delta(), policy)
                .map_err(invalid)?
            {
                Classification::Decided(RealSign::Positive) => false,
                Classification::Decided(RealSign::Negative) => true,
                Classification::Decided(RealSign::Zero) => {
                    return Err(invalid(CurveError::Topology(
                        "a chord and its affine support had orthogonal directions".into(),
                    )));
                }
                Classification::Uncertain(reason) => return Err(blocked(reason)),
            };
            let [start, end] = if reversed {
                [chord.end(), chord.start()]
            } else {
                [chord.start(), chord.end()]
            };
            let curve = RationalBezier2::try_from_subcurve(&BezierSubcurve2::Quadratic(
                QuadraticBezier2::from_line_segment(support.clone()),
            ))
            .map_err(invalid)?;
            let chart = crate::bezier_split::BezierSelectedFiberFragment2::new(
                crate::bezier_split::BezierSelectedFiberSource2::Rational(curve),
                CurveParameterRange2::new_validated(parameter(start)?, parameter(end)?),
                start.clone(),
                end.clone(),
            );
            Curve2::from_retained_fragment(crate::BezierSplitFragment2::SelectedFiber(
                if reversed { chart.reversed() } else { chart },
            ))
        }
    };
    // Chord locations name points on an injective affine source. Transport a
    // requested location through that point, leaving the companion's possibly
    // noninjective parameter constraint unchanged.
    let mut request = source
        .algebraic_chord()
        .and(binding)
        .map(|binding| binding.request.clone());
    if let Some(request) = &mut request
        && let Some(CurveFilletContact2::Parameter(parameter)) = &request.contacts[axis]
    {
        request.contacts[axis] = Some(CurveFilletContact2::Point(
            binding
                .expect("a transported request retains its binding")
                .parameter_point(axis, parameter, policy)?,
        ));
    }
    let transported =
        request
            .as_ref()
            .zip(binding)
            .map(|(request, binding)| FilletConstraintBinding2 {
                request,
                sources: binding.sources,
                maps: binding.maps,
                charts: binding.charts,
            });
    let linear = match line.retained_fragment() {
        Some(crate::BezierSplitFragment2::SelectedFiber(chart)) => {
            ExactCornerCarrier2::SelectedFiber(chart)
        }
        _ => ExactCornerCarrier2::Bezier(&line),
    };
    let parallel = parallel.corner_carrier();
    let [previous, next] = if axis == 0 {
        [linear, parallel]
    } else {
        [parallel, linear]
    };
    let solutions = solve_carrier_fillet_corner(
        previous,
        next,
        radius,
        retain_selected_circle_endpoints,
        domains,
        families[0],
        families[1],
        transported.as_ref().or(binding),
        policy,
    )?;
    let Some(chord) = source.algebraic_chord() else {
        return Ok(solutions);
    };
    let empty_reason = solutions
        .no_solution_reason()
        .unwrap_or(CurveCornerNoSolution2::UnsatisfiedConstraints);
    let mut candidates = CornerSolutionAccumulator::Empty;
    for mut corner in solutions.into_solutions() {
        let cut = if axis == 0 {
            &mut corner.previous
        } else {
            &mut corner.next
        };
        // The affine chart certified this exact location and domain. Publish
        // the original chord's monotone location instead of its solve scalar.
        cut.parameter = Some(CurveParameter2::from_algebraic_chord(
            chord
                .parameter_at_certified_support_point(cut.point.clone(), policy)
                .map_err(invalid)?,
        ));
        if let Some(binding) = binding
            && !binding.matches(&corner, policy)?
        {
            continue;
        }
        candidates.push(corner);
    }
    Ok(candidates.finish(empty_reason))
}

/// Resolves coincident linear center supports without selecting a representative.
/// Equal source directions collapse every inserted arc. Opposed directions give
/// semicircles, subject to the exact source domains and supplied constraints.
#[allow(clippy::too_many_arguments)]
pub(super) fn constrained_coincident_linear_fillet(
    offsets: [&FilletOffsetCarrier2<'_, '_>; 2],
    signed_distance: &Real,
    clockwise: bool,
    retain_selected_circle_endpoints: bool,
    domains: [FilletContactDomain2; 2],
    families: [CurveFamily2; 2],
    binding: Option<&FilletConstraintBinding2<'_>>,
    policy: &CurveContext,
) -> ExactCurveResult<CurveCornerSolutions2<FilletCorner2>> {
    let source = |axis: usize| match offsets[axis] {
        FilletOffsetCarrier2::Line { source, .. } => match source {
            FilletLinearSource2::Native { source, .. } => algebraic_chord_from_line_support(
                source,
                CurveOperation2::Fillet,
                families[axis],
                policy,
            ),
            FilletLinearSource2::AlgebraicChord(source) => Ok((*source).clone()),
        },
        FilletOffsetCarrier2::AlgebraicChord { source, .. } => Ok((*source).clone()),
        _ => unreachable!("coincident linear fillets retain both original lines"),
    };
    let sources = [source(0)?, source(1)?];
    let empty = |reason| Ok(CurveCornerSolutions2::NoSolution(reason));
    let invalid = |cause| ExactCurveError::invalid(CurveOperation2::Fillet, families[0], cause);
    let blocked = |reason| ExactCurveError::blocked(CurveOperation2::Fillet, families[0], reason);
    match sources[0]
        .tangent_dot_sign(&sources[1], policy)
        .map_err(invalid)?
    {
        Classification::Decided(RealSign::Positive) => {
            return empty(CurveCornerNoSolution2::DegenerateCandidate);
        }
        Classification::Decided(RealSign::Negative) => (),
        Classification::Decided(RealSign::Zero) => {
            return Err(invalid(CurveError::Topology(
                "coincident linear fillet supports had orthogonal source directions".into(),
            )));
        }
        Classification::Uncertain(reason) => return Err(blocked(reason)),
    }

    let on_support = |support: &crate::BezierAlgebraicChord2, point: &CurvePoint2| match support
        .oriented_support_side(point, policy)
        .map_err(invalid)?
    {
        Classification::Decided(side) => Ok(side == LineSide::On),
        Classification::Uncertain(reason) => Err(blocked(reason)),
    };
    let mut center = binding.and_then(|binding| binding.request.center.clone());
    if center.is_none()
        && let Some(binding) = binding
    {
        for (axis, contact) in binding.request.contacts.iter().enumerate() {
            let Some(contact) = contact else { continue };
            let point = match contact {
                CurveFilletContact2::Point(point) => point.clone(),
                CurveFilletContact2::Parameter(parameter) => {
                    binding.parameter_point(axis, parameter, policy)?
                }
            };
            if !on_support(&sources[axis], &point)? {
                return empty(CurveCornerNoSolution2::UnsatisfiedConstraints);
            }
            center = Some(
                sources[axis]
                    .normal_displaced_point_evidence(point, signed_distance.clone(), policy)
                    .map_err(invalid)?,
            );
            break;
        }
    }
    if let Some(center) = center {
        // Coincidence certifies the entire common support, including permitted
        // extensions. Finite contact ownership is checked by the shared cuts.
        if binding.is_some_and(|binding| binding.request.center.is_some()) {
            let support = match offsets[0] {
                FilletOffsetCarrier2::Line { support, .. } => algebraic_chord_from_line_support(
                    support,
                    CurveOperation2::Fillet,
                    families[0],
                    policy,
                )?,
                FilletOffsetCarrier2::AlgebraicChord { support, .. } => support.clone(),
                _ => unreachable!("linear center support"),
            };
            if !on_support(&support, &center)? {
                return empty(CurveCornerNoSolution2::UnsatisfiedConstraints);
            }
        }
        return constrained_coincident_fillet_at_center(
            offsets,
            center,
            clockwise,
            retain_selected_circle_endpoints,
            domains,
            families,
            binding,
            policy,
        );
    }

    // In the previous line's increasing coordinate, both remote endpoints
    // are strict lower bounds. A TrimOnly corner endpoint adds an upper bound;
    // extending either incident line removes just that upper bound. Its own
    // nonzero source interval already lies strictly above its remote endpoint,
    // so only the two cross-endpoint comparisons remain. Equal bounds are
    // empty even for SourceChart, whose remote endpoints remain excluded.
    if domains
        .iter()
        .any(|domain| domain.mode() == CurveCornerMode2::TrimOnly)
    {
        let centers = [
            sources[0]
                .parallel_left_retained(signed_distance.clone(), policy)
                .map_err(invalid)?,
            sources[1]
                .parallel_left_retained(signed_distance.clone(), policy)
                .map_err(invalid)?,
        ];
        let before =
            |first: &crate::bezier_offset::BezierAlgebraicChordParameter2,
             second: &crate::bezier_offset::BezierAlgebraicChordParameter2| {
                match first.cmp_by_refinement(second, policy).map_err(invalid)? {
                    Classification::Decided(order) => Ok(order.is_lt()),
                    Classification::Uncertain(reason) => Err(blocked(reason)),
                }
            };
        if domains[0].mode() == CurveCornerMode2::TrimOnly {
            let lower = centers[0]
                .parameter_at_certified_support_point(centers[1].end().clone(), policy)
                .map_err(invalid)?;
            if !before(&lower, &centers[0].end_parameter())? {
                return empty(CurveCornerNoSolution2::OutsideTrimDomain);
            }
        }
        if domains[1].mode() == CurveCornerMode2::TrimOnly {
            let upper = centers[0]
                .parameter_at_certified_support_point(centers[1].start().clone(), policy)
                .map_err(invalid)?;
            if !before(&centers[0].start_parameter(), &upper)? {
                return empty(CurveCornerNoSolution2::OutsideTrimDomain);
            }
        }
    }
    Err(constraint_required(families[0]))
}

/// Resolves a collapsed center while its original source and chart still live.
/// The fixed center does not select any contact on the collapsed circle.
#[allow(clippy::too_many_arguments)]
pub(super) fn constrained_collapsed_fillet(
    prepared: [&PreparedFilletCarrier2<'_>; 2],
    offsets: [&FilletOffsetCarrier2<'_, '_>; 2],
    clockwise: bool,
    signed_distance: &Real,
    retain_selected_circle_endpoints: bool,
    domains: [FilletContactDomain2; 2],
    families: [CurveFamily2; 2],
    binding: Option<&FilletConstraintBinding2<'_>>,
    policy: &CurveContext,
) -> ExactCurveResult<CurveCornerSolutions2<FilletCorner2>> {
    let (center, other_axis) = match offsets {
        [FilletOffsetCarrier2::Point { point, .. }, _] => (point, 1),
        [_, FilletOffsetCarrier2::Point { point, .. }] => (point, 0),
        _ => unreachable!("a collapsed offset retains its fixed center"),
    };
    let empty = |reason| Ok(CurveCornerSolutions2::NoSolution(reason));
    if let Some(requested) = binding.and_then(|binding| binding.request.center.as_ref())
        && !fillet_point_matches(center, requested, families[0], policy)?
    {
        return empty(CurveCornerNoSolution2::UnsatisfiedConstraints);
    }
    if !matches!(offsets[other_axis], FilletOffsetCarrier2::Parallel { .. })
        && !point_on_fillet_offset(
            center,
            offsets[other_axis],
            other_axis == 0,
            domains[other_axis],
            families[other_axis],
            policy,
        )?
    {
        return empty(CurveCornerNoSolution2::NoTangentCircle);
    }
    let mut cuts: [Option<CurveCornerSolutions2<FilletContactWitness2>>; 2] = [None, None];
    for axis in 0..2 {
        let cut = match offsets[axis] {
            FilletOffsetCarrier2::Point { source, .. } => {
                let Some((binding, contact)) = binding.and_then(|binding| {
                    binding.request.contacts[axis]
                        .as_ref()
                        .map(|c| (binding, c))
                }) else {
                    continue;
                };
                let point = match contact {
                    CurveFilletContact2::Point(point) => point.clone(),
                    CurveFilletContact2::Parameter(parameter) => {
                        binding.parameter_point(axis, parameter, policy)?
                    }
                };
                let incidence = match source {
                    PreparedFilletCarrier2::Arc { source, .. } => {
                        crate::bezier_offset::retained_point_circle_incidence_sign(
                            &point,
                            source.support().center(),
                            source.support().radius_squared_ref(),
                            policy,
                        )
                    }
                    PreparedFilletCarrier2::AlgebraicCusp { source } => source
                        .semicircle()
                        .retained_point_incidence_sign(&point, policy),
                    _ => unreachable!("only a circle produces a collapsed Point offset"),
                }
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, families[axis], cause)
                })?;
                match incidence {
                    Classification::Decided(RealSign::Zero) => (),
                    Classification::Decided(_) => {
                        return empty(CurveCornerNoSolution2::UnsatisfiedConstraints);
                    }
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            families[axis],
                            reason,
                        ));
                    }
                }
                if let PreparedFilletCarrier2::AlgebraicCusp { source } = source {
                    let parameter = if let CurveFilletContact2::Parameter(parameter) = contact {
                        let parameter = binding.source_parameter(axis, parameter, policy)?;
                        if let Some(value) = parameter.scalar() {
                            CurveParameter2::from_algebraic_cusp(
                                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(value.clone()),
                            )
                        } else {
                            parameter
                        }
                    } else {
                        match source
                            .semicircle()
                            .parameter_at_certified_incident_point(&point, policy)
                            .map_err(|cause| {
                                ExactCurveError::invalid(
                                    CurveOperation2::Fillet,
                                    families[axis],
                                    cause,
                                )
                            })? {
                            Classification::Decided(parameter) => parameter,
                            Classification::Uncertain(reason) => {
                                return Err(ExactCurveError::blocked(
                                    CurveOperation2::Fillet,
                                    families[axis],
                                    reason,
                                ));
                            }
                        }
                    };
                    let cut = selected_circle_cut_at_incident_point(
                        source,
                        parameter,
                        point,
                        axis == 0,
                        domains[axis],
                        families[axis],
                        policy,
                    )?;
                    let Some(cut) = cut else {
                        return empty(CurveCornerNoSolution2::OutsideTrimDomain);
                    };
                    cuts[axis] = Some(CurveCornerSolutions2::Unique(FilletContactWitness2 {
                        center_parameter: cut.parameter.clone(),
                        cut,
                    }));
                    continue;
                }
                let PreparedFilletCarrier2::Arc { source, .. } = source else {
                    unreachable!()
                };
                match source {
                    ExactCornerArc2::RetainedRational(source) => source.cut_at_incident_point(
                        point,
                        axis == 0,
                        domains[axis].mode(),
                        matches!(domains[axis], FilletContactDomain2::SourceChart(_)),
                        CurveOperation2::Fillet,
                        families[axis],
                        policy,
                    )?,
                    ExactCornerArc2::Native(_) => arc_fillet_cut_from_incident_point(
                        source,
                        point,
                        false,
                        axis == 0,
                        domains[axis],
                        families[axis],
                        policy,
                    )?,
                }
            }
            offset @ (FilletOffsetCarrier2::Line { .. }
            | FilletOffsetCarrier2::Arc { .. }
            | FilletOffsetCarrier2::AlgebraicChord { .. }) => fillet_cut_from_center(
                offset,
                center,
                None,
                None,
                false,
                axis == 0,
                retain_selected_circle_endpoints,
                domains[axis],
                families[axis],
                policy,
            )?,
            offset @ FilletOffsetCarrier2::AlgebraicCusp { support, .. } => {
                let parameter = match support
                    .semicircle()
                    .parameter_at_certified_incident_point(center, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, families[axis], cause)
                    })? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            families[axis],
                            reason,
                        ));
                    }
                };
                fillet_cut_from_center(
                    offset,
                    center,
                    Some(&parameter),
                    None,
                    false,
                    axis == 0,
                    retain_selected_circle_endpoints,
                    domains[axis],
                    families[axis],
                    policy,
                )?
            }
            FilletOffsetCarrier2::Parallel { source, support } => {
                let requested = binding
                    .map(|binding| {
                        binding.parallel_contact_parameters(
                            axis,
                            *source,
                            support,
                            domains[axis],
                            families[axis],
                            policy,
                        )
                    })
                    .transpose()?
                    .unwrap_or(FilletContactSelection2::Any);
                let (parameters, certified_center) =
                    if matches!(requested, FilletContactSelection2::Any) {
                        let incident = (domains[axis].mode() == CurveCornerMode2::TrimOrExtend)
                            .then(|| {
                                source.incident_domain(support, axis == 0, families[axis], policy)
                            })
                            .transpose()?;
                        (
                            fillet_point_parameters(
                                support,
                                center,
                                &source.curve_parameter_range(),
                                incident.as_ref(),
                                false,
                                families[axis],
                                policy,
                            )?,
                            true,
                        )
                    } else {
                        // Exact contact constraints narrow this fiber before any
                        // general center inversion, preserving supplied root identity.
                        (requested, false)
                    };
                let FilletContactSelection2::Parameters(parameters) = parameters else {
                    // A constant center image leaves a free contact only on
                    // the original source's admissible signed normal branch.
                    if !prepared[axis].accepts_collapsed_offset(
                        offsets[axis],
                        signed_distance,
                        families[axis],
                        policy,
                    )? {
                        return empty(CurveCornerNoSolution2::NoTangentCircle);
                    }
                    continue;
                };
                let mut contacts = CornerSolutionAccumulator::Empty;
                let mut saw_outside = false;
                for parameter in parameters {
                    if source
                        .parameter_placement(
                            &parameter,
                            axis == 0,
                            domains[axis],
                            families[axis],
                            policy,
                        )?
                        .is_none()
                    {
                        saw_outside = true;
                        continue;
                    }
                    if !prepared[axis].accepts_offset_contact(
                        offsets[axis],
                        Some(&parameter),
                        signed_distance,
                        families[axis],
                        policy,
                    )? {
                        continue;
                    }
                    if !certified_center {
                        let selected_center = analytic_parallel_point_evidence(
                            support,
                            &parameter,
                            CurveOperation2::Fillet,
                            families[axis],
                            policy,
                        )?;
                        if !fillet_point_matches(&selected_center, center, families[axis], policy)?
                        {
                            continue;
                        }
                    }
                    let Some(cut) = fillet_cut_from_center(
                        offsets[axis],
                        center,
                        Some(&parameter),
                        None,
                        false,
                        axis == 0,
                        retain_selected_circle_endpoints,
                        domains[axis],
                        families[axis],
                        policy,
                    )?
                    else {
                        continue;
                    };
                    if let Some(binding) = binding
                        && !binding.matches_contact(axis, &cut, policy)?
                    {
                        continue;
                    }
                    contacts.push(FilletContactWitness2 {
                        cut,
                        center_parameter: Some(parameter),
                    });
                }
                let reason =
                    if binding.is_some_and(|binding| binding.request.contacts[axis].is_some()) {
                        CurveCornerNoSolution2::UnsatisfiedConstraints
                    } else if saw_outside {
                        CurveCornerNoSolution2::OutsideTrimDomain
                    } else {
                        CurveCornerNoSolution2::NoTangentCircle
                    };
                cuts[axis] = Some(contacts.finish(reason));
                continue;
            }
        };
        let Some(cut) = cut else {
            return empty(CurveCornerNoSolution2::OutsideTrimDomain);
        };
        if let Some(binding) = binding
            && !binding.matches_contact(axis, &cut, policy)?
        {
            return empty(CurveCornerNoSolution2::UnsatisfiedConstraints);
        }
        cuts[axis] = Some(CurveCornerSolutions2::Unique(FilletContactWitness2 {
            center_parameter: cut.parameter.clone(),
            cut,
        }));
    }
    for contacts in cuts.iter().flatten() {
        if let Some(reason) = contacts.no_solution_reason() {
            return empty(reason);
        }
    }
    let [Some(previous), Some(next)] = cuts else {
        return Err(constraint_required(families[0]));
    };
    let mut candidates = CornerSolutionAccumulator::Empty;
    for previous in previous.solutions() {
        for next in next.solutions() {
            let witness = FilletCenterWitness2 {
                source_frames: [None, None],
                point: center.clone(),
                previous_parameter: previous.center_parameter.clone(),
                next_parameter: next.center_parameter.clone(),
                retained_anchor_evidence: None,
            };
            match fillet_corner_from_cuts(
                offsets,
                &witness,
                [previous.cut.clone(), next.cut.clone()],
                clockwise,
                families,
                policy,
            )? {
                FilletCornerSelection2::Selected(corner) => candidates.push(corner),
                FilletCornerSelection2::Outside | FilletCornerSelection2::Degenerate => (),
            }
        }
    }
    Ok(candidates.finish(CurveCornerNoSolution2::DegenerateCandidate))
}

/// Places a certified selected-circle contact without changing its exact point
/// or inverse parameter. The source's finite interval owns endpoint admission.
fn selected_circle_cut_at_incident_point(
    source: &crate::BezierAlgebraicCuspSemicircleFragment2,
    parameter: CurveParameter2,
    point: CurvePoint2,
    previous: bool,
    domain: FilletContactDomain2,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CornerCut2>> {
    use crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::{
        End, Exterior, Interior, Start,
    };
    let location = if parameter.is_algebraic_cusp_complement() {
        Exterior
    } else {
        let local = parameter.as_algebraic_cusp().ok_or_else(|| {
            ExactCurveError::invalid(
                CurveOperation2::Fillet,
                family,
                CurveError::InvalidCurveParameter,
            )
        })?;
        match source
            .certified_incident_point_evidence_location(local, &point, policy)
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        {
            Classification::Decided(location) => location,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    family,
                    reason,
                ));
            }
        }
    };
    let placement = match location {
        Interior => CornerPlacement2::Trim,
        Start | End
            if matches!(domain, FilletContactDomain2::SourceChart(_))
                && ((location == End) == previous) =>
        {
            CornerPlacement2::Corner
        }
        Exterior if domain.mode() == CurveCornerMode2::TrimOrExtend => CornerPlacement2::Extension,
        Start | End | Exterior => return Ok(None),
    };
    Ok(Some(CornerCut2 {
        parameter: Some(parameter),
        point,
        placement,
    }))
}

struct FilletContactWitness2 {
    cut: CornerCut2,
    // Raw center-support coordinates must survive separately from the cut's
    // authored/chart-mapped parameter so retained normal frames replay exactly.
    center_parameter: Option<CurveParameter2>,
}

fn fillet_point_parameters(
    support: &BezierParallel2,
    point: &CurvePoint2,
    range: &CurveParameterRange2,
    incident: Option<&crate::bezier_offset::BezierParallelIncidentDomain2>,
    regular_domain: bool,
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<FilletContactSelection2> {
    let mut all = false;
    let mut parameters = Vec::new();
    let visited = support
        .visit_point_incidence_evidence(
            point,
            range,
            incident,
            regular_domain,
            policy,
            &mut |parameter| {
                if let Some(parameter) = parameter {
                    parameters.push(parameter.clone());
                } else {
                    all = true;
                }
                std::ops::ControlFlow::Continue(())
            },
        )
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?;
    match visited {
        Classification::Decided(std::ops::ControlFlow::Continue(())) => (),
        Classification::Decided(std::ops::ControlFlow::Break(())) => {
            unreachable!("complete constraint visit")
        }
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            ));
        }
    }
    if all {
        return Ok(FilletContactSelection2::Any);
    }
    let mut unique: Vec<CurveParameter2> = Vec::new();
    'parameters: for parameter in parameters {
        for previous in &unique {
            match previous
                .same_value(&parameter, policy)
                .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
            {
                Classification::Decided(true) => continue 'parameters,
                Classification::Decided(false) => (),
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        family,
                        reason,
                    ));
                }
            }
        }
        unique.push(parameter);
    }
    Ok(FilletContactSelection2::Parameters(unique))
}

enum FilletContactSelection2 {
    Any,
    Parameters(Vec<CurveParameter2>),
}

impl FilletContactSelection2 {
    fn alternatives(&self) -> impl Iterator<Item = Option<&CurveParameter2>> {
        let (any, parameters): (_, &[CurveParameter2]) = match self {
            Self::Any => (true, &[]),
            Self::Parameters(parameters) => (false, parameters),
        };
        any.then_some(None)
            .into_iter()
            .chain(parameters.iter().map(Some))
    }
}

/// Discovers exact pair centers while the original incident sources remain
/// available for cut reconstruction and constrained component replay.
pub(super) fn parallel_pair_centers(
    sources: [FilletParallelSource2<'_>; 2],
    supports: [&BezierParallel2; 2],
    domains: [FilletContactDomain2; 2],
    families: [CurveFamily2; 2],
    normal_constraints: Option<&[crate::bezier_offset::BezierParallelDerivativeConstraint2; 2]>,
    policy: &CurveContext,
) -> ExactCurveResult<FilletCenters2> {
    let mut centers = FilletCenters2::default();
    let invalid = |cause| ExactCurveError::invalid(CurveOperation2::Fillet, families[0], cause);
    let blocked = |reason| ExactCurveError::blocked(CurveOperation2::Fillet, families[0], reason);
    let identical_contact_curves = supports[0] == supports[1]
        && sources[0].parallel_distance() == sources[1].parallel_distance();
    let original_ranges = sources.map(|source| source.curve_parameter_range());
    let mut incidents = [None, None];
    let mut expanded = [None, None];
    let mut partitions = [None, None];
    for axis in 0..2 {
        if domains[axis].mode() == CurveCornerMode2::TrimOrExtend {
            let incident =
                sources[axis].incident_domain(supports[axis], axis == 0, families[axis], policy)?;
            expanded[axis] = Some(
                match incident
                    .expanded_range(&original_ranges[axis], policy)
                    .map_err(invalid)?
                {
                    Classification::Decided(range) => range,
                    Classification::Uncertain(reason) => return Err(blocked(reason)),
                },
            );
            incidents[axis] = Some(incident);
        }
        let range = expanded[axis].as_ref().unwrap_or(&original_ranges[axis]);
        let source = supports[axis].with_distance(sources[axis].parallel_distance());
        let analysis = match source
            .singularity_analysis(range, policy)
            .map_err(invalid)?
        {
            Classification::Decided(analysis) => analysis,
            Classification::Uncertain(reason) => return Err(blocked(reason)),
        };
        if !analysis.source_is_regular() || !analysis.parallel_is_cusp_free() {
            partitions[axis] = Some(match analysis.regular_subranges(policy).map_err(invalid)? {
                Classification::Decided(cells) => cells,
                Classification::Uncertain(reason) => return Err(blocked(reason)),
            });
        }
    }
    let finite = [0, 1].map(|axis| expanded[axis].as_ref().unwrap_or(&original_ranges[axis]));
    let cells = [0, 1].map(|axis| {
        partitions[axis]
            .as_deref()
            .unwrap_or(std::slice::from_ref(finite[axis]))
    });
    let mut selected_cells = [Vec::new(), Vec::new()];
    if let Some(constraints) = normal_constraints {
        for axis in 0..2 {
            for range in cells[axis] {
                selected_cells[axis].push(
                    match constraints[axis]
                        .selects_regular_range(range, policy)
                        .map_err(invalid)?
                    {
                        Classification::Decided(selected) => selected,
                        Classification::Uncertain(reason) => return Err(blocked(reason)),
                    },
                );
            }
        }
    }
    let regular_frames = partitions.iter().any(Option::is_some);
    for first in 0..cells[0].len() {
        'second: for second in 0..cells[1].len() {
            let indices = [first, second];
            let ranges = [0, 1].map(|axis| &cells[axis][indices[axis]]);
            let mut parameter_domains = ranges.map(|range| CurveParameterDomain2::new(range, None));
            for axis in 0..2 {
                if partitions[axis].is_some() {
                    let retains_lower = (axis == 0) != sources[axis].is_reversed();
                    parameter_domains[axis].inclusion = [!retains_lower, retains_lower];
                }
                if let Some(incident) = &incidents[axis] {
                    let ray = incident.parameter_ray();
                    let last_cell = match ray.direction {
                        crate::BezierParameterRayDirection2::Increasing => {
                            indices[axis] + 1 == cells[axis].len()
                        }
                        crate::BezierParameterRayDirection2::Decreasing => indices[axis] == 0,
                    };
                    if last_cell {
                        match ray.is_empty(policy).map_err(invalid)? {
                            Classification::Decided(true) => {}
                            Classification::Decided(false) => {
                                parameter_domains[axis].extension = Some(ray)
                            }
                            Classification::Uncertain(reason) => return Err(blocked(reason)),
                        }
                    }
                }
            }
            if normal_constraints.is_some() {
                for axis in 0..2 {
                    if parameter_domains[axis].extension.is_none()
                        && !selected_cells[axis][indices[axis]]
                    {
                        continue 'second;
                    }
                }
            }
            // The source analysis already proves a constant nonzero
            // derivative scale on each finite cell. Its selected normal also
            // owns the cell's stationary boundary. Rays can cross additional
            // original-offset cusps and keep their algebraic normal selector.
            let constraints = parameter_domains
                .iter()
                .any(|domain| domain.extension.is_some())
                .then_some(normal_constraints)
                .flatten();
            let query = crate::bezier_offset::ParameterComponentQuery2::AllComponents(constraints);
            let result = if identical_contact_curves {
                supports[0].self_intersections_in_domain(
                    parameter_domains,
                    query,
                    regular_frames,
                    policy,
                )
            } else if regular_frames {
                supports[0].parallel_intersections_on_regular_domains(
                    supports[1],
                    parameter_domains,
                    query,
                    policy,
                )
            } else {
                supports[0].parallel_intersections_in_domain(
                    supports[1],
                    parameter_domains,
                    query,
                    policy,
                )
            }
            .map_err(invalid)?;
            let (intersections, components) = match result {
                Classification::Decided(result) => result.into_parts(),
                Classification::Uncertain(reason) => return Err(blocked(reason)),
            };
            if !intersections.is_complete() {
                return Err(blocked(crate::UncertaintyReason::Predicate));
            }
            centers.components.extend(components);
            'contacts: for contact in intersections.contacts() {
                let parameters = [contact.first_parameter(), contact.second_parameter()];
                // The domain owns stationary seams before either isolated
                // contacts or positive-dimensional families are replayed.
                for component in &centers.components {
                    match component
                        .contains_pair(parameters[0], parameters[1], policy)
                        .map_err(invalid)?
                    {
                        Classification::Decided(true) => continue 'contacts,
                        Classification::Decided(false) => {}
                        Classification::Uncertain(reason) => return Err(blocked(reason)),
                    }
                }
                for axis in 0..2 {
                    if !sources[axis].parameter_is_admissible(
                        parameters[axis],
                        axis == 0,
                        domains[axis],
                        incidents[axis].as_ref(),
                        families[axis],
                        policy,
                    )? {
                        continue 'contacts;
                    }
                }
                let center = if regular_frames {
                    regular_parallel_pair_center(
                        sources, supports, parameters, ranges, families, policy,
                    )?
                } else {
                    let point = analytic_parallel_point_evidence(
                        supports[0],
                        parameters[0],
                        CurveOperation2::Fillet,
                        families[0],
                        policy,
                    )?;
                    let reversed = sources[0]
                        .support_reverses_source_at(
                            supports[0],
                            parameters[0],
                            families[0],
                            policy,
                        )?
                        .zip(sources[1].support_reverses_source_at(
                            supports[1],
                            parameters[1],
                            families[1],
                            policy,
                        )?)
                        .map(|(first, second)| first != second);
                    let orient = |sign| {
                        reversed.map(|reverse| {
                            if reverse {
                                reverse_fillet_sign(sign)
                            } else {
                                sign
                            }
                        })
                    };
                    FilletCenterWitness2 {
                        source_frames: [None, None],
                        point,
                        previous_parameter: Some(parameters[0].clone()),
                        next_parameter: Some(parameters[1].clone()),
                        retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
                            cross: contact.tangent_cross_sign().and_then(orient),
                            dot: contact.tangent_dot_sign().and_then(orient),
                            center_parallel: None,
                            source_direction: None,
                            canonical_anchor_curve: None,
                            deferred_arc_contact: None,
                        }),
                    }
                };
                centers.push(center);
            }
        }
    }
    Ok(centers)
}

/// Retains the source side that survives the cut. The center support's own
/// derivative scale does not determine the original curve's orientation.
fn regular_parallel_pair_center(
    sources: [FilletParallelSource2<'_>; 2],
    supports: [&BezierParallel2; 2],
    parameters: [&CurveParameter2; 2],
    ranges: [&CurveParameterRange2; 2],
    families: [CurveFamily2; 2],
    policy: &CurveContext,
) -> ExactCurveResult<FilletCenterWitness2> {
    let invalid = |cause| ExactCurveError::invalid(CurveOperation2::Fillet, families[0], cause);
    let blocked = |reason| ExactCurveError::blocked(CurveOperation2::Fillet, families[0], reason);
    let mut point = None;
    let mut frames = [None, None];
    let mut limiting_frames = [false; 2];
    let mut reversed = false;
    for axis in 0..2 {
        let source = supports[axis].with_distance(sources[axis].parallel_distance());
        let pointwise_scale = source
            .parallel_derivative_scale_sign(parameters[axis], policy)
            .map_err(invalid)?;
        let stationary = pointwise_scale == Classification::Decided(RealSign::Zero);
        let scale = match if stationary {
            source
                .parallel_derivative_scale_sign_on_regular_range(
                    parameters[axis],
                    ranges[axis],
                    policy,
                )
                .map_err(invalid)?
        } else {
            pointwise_scale
        } {
            Classification::Decided(RealSign::Zero) => {
                return Err(blocked(crate::UncertaintyReason::Boundary));
            }
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => return Err(blocked(reason)),
        };
        let (center, tangent) = match supports[axis]
            .regular_source_point_and_tangent_support(
                supports[axis],
                parameters[axis],
                ranges[axis],
                RealSign::Positive,
                policy,
            )
            .map_err(invalid)?
        {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => return Err(blocked(reason)),
        };
        if axis == 0 {
            point = Some(center);
        }
        limiting_frames[axis] = stationary
            || !matches!(
                source.source_tangent_nonzero_at(parameters[axis], policy),
                Ok(Classification::Decided(true))
            );
        reversed ^= sources[axis].is_reversed() ^ (scale == RealSign::Negative);
        frames[axis] = Some(FilletSourceFrame2 {
            tangent,
            derivative_scale: scale,
        });
    }
    let [first, second] = frames
        .each_ref()
        .map(|frame| &frame.as_ref().unwrap().tangent);
    let orient = |sign| {
        if reversed {
            reverse_fillet_sign(sign)
        } else {
            sign
        }
    };
    let cross = match first.tangent_cross_sign(second, policy).map_err(invalid)? {
        Classification::Decided(sign) => orient(sign),
        Classification::Uncertain(reason) => return Err(blocked(reason)),
    };
    let dot = match first.tangent_dot_sign(second, policy).map_err(invalid)? {
        Classification::Decided(sign) => orient(sign),
        Classification::Uncertain(reason) => return Err(blocked(reason)),
    };
    // Ordinary cuts retain their native point and support evidence. A
    // limiting frame is needed only at a stationary contact, or when an
    // ordinary tangent certificate is unavailable; forcing it on every
    // contact needlessly changes the representation used by later edits.
    for (frame, needed) in frames.iter_mut().zip(limiting_frames) {
        if !needed {
            *frame = None;
        }
    }
    Ok(FilletCenterWitness2 {
        source_frames: frames,
        point: point.unwrap(),
        previous_parameter: Some(parameters[0].clone()),
        next_parameter: Some(parameters[1].clone()),
        retained_anchor_evidence: Some(RetainedFilletAnchorEvidence2 {
            cross: Some(cross),
            dot: Some(dot),
            center_parallel: None,
            source_direction: None,
            canonical_anchor_curve: None,
            deferred_arc_contact: None,
        }),
    })
}

/// Replays component constraints while the prepared offsets remain borrowed.
/// No family result or copied center support escapes the carrier solver.
pub(super) struct FilletComponentReplay2<'a, 'b> {
    offsets: [&'b FilletOffsetCarrier2<'a, 'b>; 2],
    clockwise: bool,
    retain_selected_circle_endpoints: bool,
    domains: [FilletContactDomain2; 2],
    families: [CurveFamily2; 2],
}

impl FilletComponentReplay2<'_, '_> {
    #[allow(clippy::too_many_arguments)]
    pub(super) fn solve(
        components: &[crate::bezier_offset::CurveParameterComponent2],
        offsets: [&FilletOffsetCarrier2<'_, '_>; 2],
        clockwise: bool,
        retain_selected_circle_endpoints: bool,
        domains: [FilletContactDomain2; 2],
        families: [CurveFamily2; 2],
        binding: Option<&FilletConstraintBinding2<'_>>,
        candidates: &mut CornerSolutionAccumulator<FilletCorner2>,
        policy: &CurveContext,
    ) -> ExactCurveResult<()> {
        if components.is_empty() {
            return Ok(());
        }
        if !offsets
            .iter()
            .all(|offset| matches!(offset, FilletOffsetCarrier2::Parallel { .. }))
        {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                families[0],
                crate::UncertaintyReason::Unsupported,
            ));
        }
        let binding = binding
            .filter(|binding| binding.request.has_constraints())
            .ok_or_else(|| constraint_required(families[0]))?;
        let replay = FilletComponentReplay2 {
            offsets,
            clockwise,
            retain_selected_circle_endpoints,
            domains,
            families,
        };
        let mut selected_pairs = Vec::new();
        for component in components {
            if let (Some(requested), Some(image)) =
                (&binding.request.center, component.point_image())
                && !fillet_point_matches(requested, image, families[0], policy)?
            {
                continue;
            }
            let constraints = replay.contact_constraints(binding, component, policy)?;
            for first in constraints[0].alternatives() {
                for second in constraints[1].alternatives() {
                    if let Some(corner) =
                        replay.select(component, [first, second], &mut selected_pairs, policy)?
                        && binding.matches(&corner, policy)?
                    {
                        candidates.push(corner);
                    }
                }
            }
        }
        Ok(())
    }

    fn parallel(&self, axis: usize) -> (FilletParallelSource2<'_>, &BezierParallel2) {
        let FilletOffsetCarrier2::Parallel { source, support } = self.offsets[axis] else {
            unreachable!("parameter components retain parallel center supports")
        };
        (*source, support)
    }

    fn contact_constraints(
        &self,
        binding: &FilletConstraintBinding2<'_>,
        component: &crate::bezier_offset::CurveParameterComponent2,
        policy: &CurveContext,
    ) -> ExactCurveResult<[FilletContactSelection2; 2]> {
        let data = self;
        let mut constraints = [FilletContactSelection2::Any, FilletContactSelection2::Any];
        for (axis, constraint) in constraints.iter_mut().enumerate() {
            let (source, support) = data.parallel(axis);
            *constraint =
                if let Some(CurveFilletContact2::Point(point)) = &binding.request.contacts[axis] {
                    self.point_parameters(
                        component,
                        axis,
                        &support.with_distance(source.parallel_distance()),
                        point,
                        policy,
                    )?
                } else {
                    binding.parallel_contact_parameters(
                        axis,
                        source,
                        support,
                        data.domains[axis],
                        data.families[axis],
                        policy,
                    )?
                };
        }
        if binding.request.contacts.iter().all(Option::is_none)
            && let Some(center) = &binding.request.center
        {
            // A correspondence transports one selected contact to the other.
            // A constant center leaves both contact axes free and still needs
            // contact constraints; it cannot select an arbitrary representative.
            constraints[0] =
                self.point_parameters(component, 0, self.parallel(0).1, center, policy)?;
            if matches!(constraints[0], FilletContactSelection2::Any) {
                constraints[1] =
                    self.point_parameters(component, 1, self.parallel(1).1, center, policy)?;
            }
        }
        Ok(constraints)
    }

    fn point_parameters(
        &self,
        component: &crate::bezier_offset::CurveParameterComponent2,
        axis: usize,
        support: &BezierParallel2,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<FilletContactSelection2> {
        let family = self.families[axis];
        let invalid = |cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause);
        let blocked = |reason| ExactCurveError::blocked(CurveOperation2::Fillet, family, reason);
        let (range, extended) = match component
            .source_chart_range(axis, policy)
            .map_err(invalid)?
        {
            Classification::Decided(chart) => chart,
            Classification::Uncertain(reason) => return Err(blocked(reason)),
        };
        let (source, center_support) = self.parallel(axis);
        let incident = extended
            .then(|| source.incident_domain(center_support, axis == 0, family, policy))
            .transpose()?;
        fillet_point_parameters(
            support,
            point,
            &range,
            incident.as_ref(),
            true,
            family,
            policy,
        )
    }

    fn select(
        &self,
        component: &crate::bezier_offset::CurveParameterComponent2,
        constraints: [Option<&CurveParameter2>; 2],
        selected_pairs: &mut Vec<[CurveParameter2; 2]>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<FilletCorner2>> {
        let data = self;
        let selected = match component.constrain(constraints, policy).map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Fillet, data.families[0], cause)
        })? {
            Classification::Decided(
                crate::bezier_offset::CurveParameterComponentSelection2::Selected(pair),
            ) => pair,
            Classification::Decided(
                crate::bezier_offset::CurveParameterComponentSelection2::Empty,
            ) => return Ok(None),
            Classification::Decided(
                crate::bezier_offset::CurveParameterComponentSelection2::NeedsConstraint,
            ) => {
                return Err(constraint_required(data.families[0]));
            }
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    data.families[0],
                    reason,
                ));
            }
        };
        // Finite and incident charts can cover the same selected pair. Keep
        // each authored preimage once, before reconstructing its circle;
        // coincident points at different parameters remain distinct cuts.
        for prior in selected_pairs.iter() {
            let mut same = true;
            for axis in 0..2 {
                match prior[axis]
                    .same_value(&selected[axis], policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(
                            CurveOperation2::Fillet,
                            data.families[axis],
                            cause,
                        )
                    })? {
                    Classification::Decided(true) => (),
                    Classification::Decided(false) => {
                        same = false;
                        break;
                    }
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            data.families[axis],
                            reason,
                        ));
                    }
                }
            }
            if same {
                return Ok(None);
            }
        }
        selected_pairs.push(selected.clone());
        let pairs = [self.parallel(0), self.parallel(1)];
        let mut ranges = [None, None];
        for (axis, range) in ranges.iter_mut().enumerate() {
            *range = Some(
                match component
                    .source_chart_range(axis, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(
                            CurveOperation2::Fillet,
                            data.families[axis],
                            cause,
                        )
                    })? {
                    Classification::Decided((range, _)) => range,
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            data.families[axis],
                            reason,
                        ));
                    }
                },
            );
        }
        let mut center = regular_parallel_pair_center(
            pairs.map(|(source, _)| source),
            pairs.map(|(_, support)| support),
            [&selected[0], &selected[1]],
            ranges.each_ref().map(|range| range.as_ref().unwrap()),
            data.families,
            policy,
        )?;
        if let Some(point) = component.point_image() {
            center.point = point.clone();
        }
        Ok(
            match fillet_corner_from_center(
                data.offsets[0],
                data.offsets[1],
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

#[derive(Debug)]
pub(super) struct PathFilletPlacement2<'a> {
    source: &'a CurvePath2,
    vertex: usize,
    indices: [usize; 2],
    source_maps: [Option<(Real, Real)>; 2],
    strict_authored_bounds: [bool; 2],
    arcs: [Option<Arc<RetainedRationalCornerArc2>>; 2],
    promoted: [Option<&'a crate::BezierParallelFragment2>; 2],
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
            source,
            vertex,
            indices,
            source_maps,
            strict_authored_bounds: authored_maps.map(|map| map.is_some()),
            arcs,
            promoted,
            circular_domains,
        }
    }

    pub(super) fn constraints<'b>(
        &'b self,
        request: &'b CurveFillet2,
    ) -> FilletConstraintBinding2<'b> {
        FilletConstraintBinding2 {
            request,
            sources: self.indices.map(|index| &self.source.data.curves[index]),
            maps: self
                .source_maps
                .each_ref()
                .map(|map| map.as_ref().map(|(scale, offset)| (scale, offset))),
            charts: std::array::from_fn(|axis| {
                FilletContactChart2::for_curve(
                    &self.source.data.curves[self.indices[axis]],
                    self.arcs[axis].is_some(),
                )
            }),
        }
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
            self.promoted,
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
        center.source_frames[0].as_ref(),
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
        center.source_frames[1].as_ref(),
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
    fillet_corner_from_cuts(
        [previous_offset, next_offset],
        center,
        [previous_cut, next_cut],
        clockwise,
        [previous_family, next_family],
        policy,
    )
}

fn fillet_corner_from_cuts(
    offsets: [&FilletOffsetCarrier2<'_, '_>; 2],
    center: &FilletCenterWitness2,
    cuts: [CornerCut2; 2],
    clockwise: bool,
    families: [CurveFamily2; 2],
    policy: &CurveContext,
) -> ExactCurveResult<FilletCornerSelection2> {
    let [previous_offset, next_offset] = offsets;
    let [previous_cut, next_cut] = cuts;
    let [previous_family, next_family] = families;
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
            if let Some((axis, source_frame)) = center
                .source_frames
                .iter()
                .enumerate()
                .find_map(|(axis, frame)| frame.as_ref().map(|frame| (axis, frame)))
            {
                let FilletOffsetCarrier2::Parallel { source, support } = offsets[axis] else {
                    unreachable!("a retained source frame belongs to a parallel contact");
                };
                return Ok(FilletCornerSelection2::Selected(FilletCorner2 {
                    previous: previous_cut,
                    next: next_cut,
                    center: center.point.clone(),
                    clockwise,
                    retained_frame: Some(RetainedFilletFrame2 {
                        anchor_is_previous: axis == 0,
                        radial_frame: RetainedFilletRadialFrame2::ChordNormal {
                            anchor: source_frame.tangent.clone(),
                            policy: *policy,
                        },
                        radial_distance: source.parallel_distance() - support.distance(),
                        anchor_evidence: center.retained_anchor_evidence.clone(),
                    }),
                }));
            }

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

    fn closed_region(path: &CurvePath2, policy: &CurveContext) -> crate::CurveRegion2 {
        let mut curves = path.curves().to_vec();
        let Classification::Decided(closing) =
            crate::BezierAlgebraicChord2::try_new(path.end(), path.start(), policy).unwrap()
        else {
            panic!("exact closing chord")
        };
        curves.push(Curve2::from_retained_fragment(
            crate::BezierSplitFragment2::AlgebraicChord(closing),
        ));
        crate::CurveRegion2::try_from_boundary_paths(
            &[CurvePath2::try_new(curves).unwrap()],
            policy,
        )
        .unwrap()
        .value
    }

    #[test]
    fn coincident_circular_fillet_constraints_complete_the_authored_chart_solutions() {
        let p = Point2::from_values;
        for past_center in [false, true] {
            let clockwise = !past_center;
            let remote = if past_center { p(-2, 0) } else { p(0, -2) };
            let control = if past_center {
                vec![
                    p(1, 0),
                    Point2::new(q(1, 2), Real::one()),
                    p(0, 2),
                    p(-2, 2),
                    remote.clone(),
                ]
            } else {
                vec![
                    p(1, 0),
                    Point2::new(q(3, 2), Real::zero()),
                    p(2, 0),
                    p(2, -2),
                    remote.clone(),
                ]
            };
            let spline = Curve2::try_nurbs(
                2,
                control,
                vec![
                    Real::one(),
                    Real::one(),
                    Real::one(),
                    q(1, 2).sqrt().unwrap(),
                    Real::one(),
                ],
                [0, 0, 0, 1, 1, 2, 2, 2]
                    .into_iter()
                    .map(Real::from)
                    .collect(),
                &CurveContext::STRICT,
            )
            .unwrap()
            .value;
            let source = CurvePath2::try_new(vec![
                CircularArc2::try_from_center(p(0, -1), p(1, 0), p(0, 0), false)
                    .unwrap()
                    .into(),
                spline,
            ])
            .unwrap();
            let inner = Point2::new(q(3, 5), q(-4, 5));
            let outer = if past_center {
                Point2::new(q(-6, 5), q(8, 5))
            } else {
                Point2::new(q(6, 5), q(-8, 5))
            };
            let center = if past_center {
                Point2::new(q(-3, 10), q(2, 5))
            } else {
                Point2::new(q(9, 10), q(-6, 5))
            };
            // Independent nonzero fillet in a continuum, including the sheet
            // where the inner arc's offset passes through its source center.
            let witness = CurvePath2::try_new(vec![
                CircularArc2::try_from_center(p(0, -1), inner.clone(), p(0, 0), false)
                    .unwrap()
                    .into(),
                CircularArc2::try_from_center(
                    inner.clone(),
                    outer.clone(),
                    center.clone(),
                    clockwise,
                )
                .unwrap()
                .into(),
                CircularArc2::try_from_center(outer.clone(), remote.clone(), p(0, 0), clockwise)
                    .unwrap()
                    .into(),
            ])
            .unwrap();
            for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
                let retained = |point: &Point2| {
                    let Classification::Decided(chord) = crate::BezierAlgebraicChord2::try_new(
                        point.clone().into(),
                        inner.clone().into(),
                        &policy,
                    )
                    .unwrap() else {
                        panic!("exact contact chord")
                    };
                    let chord = chord
                        .parallel_left_retained(Real::one(), &policy)
                        .unwrap()
                        .parallel_left_retained(-Real::one(), &policy)
                        .unwrap();
                    assert!(chord.start().coordinates().is_none());
                    chord.start().clone()
                };
                let retained_outer = retained(&outer);
                let retained_center = retained(&center);
                for reversed in [false, true] {
                    let path = if reversed {
                        source.reversed(&policy).unwrap().value
                    } else {
                        source.clone()
                    };
                    let spline_axis = usize::from(!reversed);
                    for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                        let mut request =
                            CurveFillet2::new(if past_center { q(3, 2) } else { q(1, 2) });
                        match path.fillet_vertex(1, &request, mode, &policy) {
                            Err(ExactCurveError::Invalid {
                                cause: CurveError::FilletConstraintRequired,
                                ..
                            }) => (),
                            Err(error) => panic!("unexpected circular family error: {error}"),
                            Ok(outcome) => panic!(
                                "circular family omitted: count={}, reason={:?}",
                                outcome.value.candidate_count(),
                                outcome.value.no_solution_reason()
                            ),
                        }
                        for constraint in 0..6 {
                            request.center =
                                (constraint == 0 || constraint == 2).then(|| center.clone().into());
                            if constraint == 5 {
                                request.center = Some(retained_center.clone());
                            }
                            request.contacts = [None, None];
                            if constraint == 4 {
                                request.contacts[spline_axis] =
                                    Some(CurveFilletContact2::Point(retained_outer.clone()));
                            }
                            if constraint == 1 || constraint == 2 {
                                request.contacts[spline_axis] =
                                    Some(CurveFilletContact2::Point(outer.clone().into()));
                            }
                            let contacts: [CurvePoint2; 2] = if constraint == 3 {
                                request.contacts[spline_axis] =
                                    Some(CurveFilletContact2::Parameter(
                                        (if reversed { q(1, 2) } else { q(3, 2) }).into(),
                                    ));
                                let unit = q(1, 2).sqrt().unwrap();
                                let outer_x = Real::from(if past_center { -2 } else { 2 }) * &unit;
                                [
                                    Point2::new(unit.clone(), -unit).into(),
                                    Point2::new(outer_x.clone(), -outer_x).into(),
                                ]
                            } else {
                                [inner.clone().into(), outer.clone().into()]
                            };
                            let selected = path.fillet_vertex(1, &request, mode, &policy).unwrap_or_else(|error| panic!("circular constraint failed: constraint={constraint}, reversed={reversed}, mode={mode:?}, past_center={past_center}, error={error}"));
                            assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                            assert_eq!(
                                selected.value.candidate_count(),
                                1,
                                "constraint={constraint}, reversed={reversed}, mode={mode:?}, past_center={past_center}"
                            );
                            let edited = &selected.value.solutions()[0];
                            same(&edited.start(), &path.start(), &policy);
                            same(&edited.end(), &path.end(), &policy);
                            same(
                                &edited.curves().first().unwrap().end(),
                                &contacts[usize::from(reversed)],
                                &policy,
                            );
                            same(
                                &edited.curves().last().unwrap().start(),
                                &contacts[spline_axis],
                                &policy,
                            );
                            for pair in edited.curves().windows(2) {
                                same(&pair[0].end(), &pair[1].start(), &policy);
                            }
                            if matches!(constraint, 0 | 4 | 5)
                                && !reversed
                                && mode == CurveCornerMode2::TrimOnly
                            {
                                let region = closed_region(edited, &policy);
                                let difference = region
                                    .boolean_regions(&closed_region(&witness, &policy), &policy)
                                    .unwrap();
                                assert_eq!(difference.certainty, crate::CurveCertainty::Certified);
                                assert!(difference.value.xor().is_empty());
                                let offset = region
                                    .offset(q(1, 100), &crate::OffsetCornerStyle2::Bevel, &policy)
                                    .unwrap();
                                assert_eq!(offset.certainty, crate::CurveCertainty::Certified);
                                let outside =
                                    offset.value.classify_point(&p(10, 10), &policy).unwrap();
                                assert_eq!(outside.certainty, crate::CurveCertainty::Certified);
                                assert_eq!(
                                    outside.value,
                                    Classification::Decided(crate::RegionPointLocation::Outside)
                                );
                            }
                        }
                        request.contacts = [None, None];
                        request.center = Some(p(0, 0).into());
                        assert!(
                            path.fillet_vertex(1, &request, mode, &policy)
                                .unwrap()
                                .value
                                .solutions()
                                .is_empty()
                        );
                        request.center = None;
                        request.contacts[spline_axis] =
                            Some(CurveFilletContact2::Point(remote.clone().into()));
                        assert!(
                            path.fillet_vertex(1, &request, mode, &policy)
                                .unwrap()
                                .value
                                .solutions()
                                .is_empty()
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn coincident_circular_fillet_domains_distinguish_families_and_isolated_contacts() {
        let p = Point2::from_values;
        let arc = |start, end, clockwise| {
            CircularArc2::try_from_center(start, end, p(0, 0), clockwise).unwrap()
        };
        // Disjoint quarters, complementary halves, coincident full circles,
        // two-component major-arc overlap, and an isolated incident contact
        // after crossing the inner circle's center.
        let fixtures = [
            (
                arc(p(0, -1), p(1, 0), false),
                arc(p(-2, 0), p(0, 2), true),
                false,
                false,
            ),
            (
                arc(p(-1, 0), p(1, 0), false),
                arc(p(-2, 0), p(2, 0), true),
                false,
                false,
            ),
            (
                arc(p(1, 0), p(1, 0), false),
                arc(p(2, 0), p(2, 0), true),
                true,
                false,
            ),
            (
                arc(p(1, 0), p(0, -1), false),
                arc(p(0, 2), p(-2, 0), true),
                true,
                false,
            ),
            (
                arc(p(0, -1), p(1, 0), false),
                arc(p(-2, 0), p(0, -2), false),
                false,
                true,
            ),
        ];
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (index, (previous, next, overlap, isolated)) in fixtures.iter().enumerate() {
                for reversed in [false, true] {
                    let arcs = if reversed {
                        [next.reversed(), previous.reversed()]
                    } else {
                        [previous.clone(), next.clone()]
                    };
                    for chart in [false, true] {
                        for extend_axis in [None, Some(0), Some(1)] {
                            let domains = [0, 1].map(|axis| {
                                let mode = if extend_axis == Some(axis) {
                                    CurveCornerMode2::TrimOrExtend
                                } else {
                                    CurveCornerMode2::TrimOnly
                                };
                                if chart {
                                    FilletContactDomain2::SourceChart(mode)
                                } else {
                                    FilletContactDomain2::AuthoredCurve(mode)
                                }
                            });
                            let radius = if *isolated { q(3, 2) } else { q(1, 2) };
                            let outcome = solve_carrier_fillet_corner(
                                ExactCornerCarrier2::Arc(&arcs[0]),
                                ExactCornerCarrier2::Arc(&arcs[1]),
                                &radius,
                                false,
                                domains,
                                CurveFamily2::CircularArc,
                                CurveFamily2::CircularArc,
                                None,
                                &policy,
                            );
                            if *overlap || extend_axis.is_some() {
                                assert!(
                                    matches!(
                                        outcome,
                                        Err(ExactCurveError::Invalid {
                                            cause: CurveError::FilletConstraintRequired,
                                            ..
                                        })
                                    ),
                                    "expected family: fixture={index}, chart={chart}, reversed={reversed}, extend_axis={extend_axis:?}"
                                );
                            } else {
                                let solutions = outcome.unwrap();
                                assert_eq!(
                                    solutions.candidate_count(),
                                    usize::from(*isolated && chart),
                                    "fixture={index}, chart={chart}, reversed={reversed}"
                                );
                                for corner in solutions.solutions() {
                                    same(
                                        &corner.center,
                                        &Point2::new(q(-1, 2), Real::zero()).into(),
                                        &policy,
                                    );
                                    same(
                                        &corner.previous.point,
                                        &arcs[0].end().clone().into(),
                                        &policy,
                                    );
                                    same(
                                        &corner.next.point,
                                        &arcs[1].start().clone().into(),
                                        &policy,
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
    fn coincident_circular_fillet_families_use_the_surviving_rational_intervals() {
        let p = Point2::from_values;
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for upper in [q(1, 4), q(1, 2), q(3, 4), q(1, 8).sqrt().unwrap()] {
                let sources = [
                    vec![p(1, 0), p(1, 1), p(0, 1)],
                    vec![p(0, 2), p(2, 2), p(2, 0)],
                ]
                .map(|controls| {
                    let curve = RationalBezier2::try_new(
                        controls,
                        vec![Real::one(), q(1, 2).sqrt().unwrap(), Real::one()],
                    )
                    .unwrap();
                    let Classification::Decided(start) =
                        curve.point_at_classified(&Real::zero(), &policy)
                    else {
                        panic!("exact chart start")
                    };
                    let Classification::Decided(end) = curve.point_at_classified(&upper, &policy)
                    else {
                        panic!("exact chart end")
                    };
                    crate::bezier_split::BezierSelectedFiberFragment2::new(
                        crate::bezier_split::BezierSelectedFiberSource2::Rational(curve),
                        CurveParameterRange2::new_validated(
                            Real::zero().into(),
                            upper.clone().into(),
                        ),
                        start.into(),
                        end.into(),
                    )
                });
                let open_overlap = upper == q(3, 4);
                for reversed in [false, true] {
                    let sources = if reversed {
                        [sources[1].reversed(), sources[0].reversed()]
                    } else {
                        sources.clone()
                    };
                    let carriers = sources.each_ref().map(|source| {
                        RetainedRationalCornerArc2::from_selected(
                            source,
                            CurveOperation2::Fillet,
                            &policy,
                        )
                        .unwrap()
                        .unwrap()
                    });
                    for chart in [false, true] {
                        for extend_axis in [None, Some(0), Some(1)] {
                            let domains = [0, 1].map(|axis| {
                                let mode = if extend_axis == Some(axis) {
                                    CurveCornerMode2::TrimOrExtend
                                } else {
                                    CurveCornerMode2::TrimOnly
                                };
                                if chart {
                                    FilletContactDomain2::SourceChart(mode)
                                } else {
                                    FilletContactDomain2::AuthoredCurve(mode)
                                }
                            });
                            let outcome = solve_carrier_fillet_corner(
                                ExactCornerCarrier2::RetainedRationalArc(carriers[0].clone()),
                                ExactCornerCarrier2::RetainedRationalArc(carriers[1].clone()),
                                &q(1, 2),
                                false,
                                domains,
                                CurveFamily2::CircularArc,
                                CurveFamily2::CircularArc,
                                None,
                                &policy,
                            );
                            if open_overlap || extend_axis.is_some() {
                                assert!(matches!(
                                    outcome,
                                    Err(ExactCurveError::Invalid {
                                        cause: CurveError::FilletConstraintRequired,
                                        ..
                                    })
                                ));
                            } else {
                                assert_eq!(outcome.unwrap().candidate_count(), 0);
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn coincident_line_fillet_constraints_complete_the_authored_chart_solutions() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for radius in [Real::one(), Real::from(2).sqrt().unwrap()] {
                let p = |x, y| Point2::new(Real::from(x) * &radius, Real::from(y) * &radius);
                let spline = Curve2::try_nurbs(
                    1,
                    vec![p(0, 0), p(0, 2), p(-3, 2)],
                    vec![Real::one(); 3],
                    [2, 2, 5, 9, 9].into_iter().map(Real::from).collect(),
                    &policy,
                )
                .unwrap()
                .value;
                let source = CurvePath2::try_new(vec![
                    LineSeg2::try_new(p(-3, 0), p(0, 0)).unwrap().into(),
                    spline,
                ])
                .unwrap();
                let witness = CurvePath2::try_new(vec![
                    LineSeg2::try_new(p(-3, 0), p(-1, 0)).unwrap().into(),
                    CircularArc2::try_from_center(p(-1, 0), p(-1, 2), p(-1, 1), false)
                        .unwrap()
                        .into(),
                    LineSeg2::try_new(p(-1, 2), p(-3, 2)).unwrap().into(),
                ])
                .unwrap();
                for reversed in [false, true] {
                    let path = if reversed {
                        source.reversed(&policy).unwrap().value
                    } else {
                        source.clone()
                    };
                    let line_axis = usize::from(reversed);
                    let spline_axis = 1 - line_axis;
                    for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                        let mut request = CurveFillet2::new(radius.clone());
                        match path.fillet_vertex(1, &request, mode, &policy) {
                            Err(ExactCurveError::Invalid {
                                cause: CurveError::FilletConstraintRequired,
                                ..
                            }) => (),
                            Err(ExactCurveError::Blocked(blocker)) => panic!(
                                "free linear contacts blocked: {:?}, reversed={reversed}, mode={mode:?}",
                                blocker.reason()
                            ),
                            Err(ExactCurveError::Invalid { cause, .. }) => panic!(
                                "free linear contacts invalid: {cause}, reversed={reversed}, mode={mode:?}"
                            ),
                            Ok(outcome) => panic!(
                                "free linear contacts omitted: count={}, reason={:?}, reversed={reversed}, mode={mode:?}",
                                outcome.value.candidate_count(),
                                outcome.value.no_solution_reason()
                            ),
                        }
                        // One center fixes both the isolated quarter circle and
                        // the semicircle on the later, opposing source chart.
                        request.center = Some(p(-1, 1).into());
                        let centered = path.fillet_vertex(1, &request, mode, &policy).unwrap();
                        assert_eq!(centered.certainty, crate::CurveCertainty::Certified);
                        assert_eq!(centered.value.candidate_count(), 2);
                        request.center = None;
                        request.contacts[line_axis] = Some(CurveFilletContact2::Parameter(
                            (if reversed { q(1, 3) } else { q(2, 3) }).into(),
                        ));
                        assert_eq!(
                            path.fillet_vertex(1, &request, mode, &policy)
                                .unwrap()
                                .value
                                .candidate_count(),
                            2
                        );
                        request.contacts[line_axis] = None;
                        for contact in [
                            CurveFilletContact2::Point(p(-1, 2).into()),
                            CurveFilletContact2::Parameter(
                                (if reversed { q(14, 3) } else { q(19, 3) }).into(),
                            ),
                        ] {
                            request.contacts[spline_axis] = Some(contact);
                            let selected = path.fillet_vertex(1, &request, mode, &policy).unwrap();
                            assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                            assert_eq!(selected.value.candidate_count(), 1);
                            let edited = &selected.value.solutions()[0];
                            same(&edited.start(), &path.start(), &policy);
                            same(&edited.end(), &path.end(), &policy);
                            let contacts: [CurvePoint2; 2] = [p(-1, 0).into(), p(-1, 2).into()];
                            same(
                                &edited.curves().first().unwrap().end(),
                                &contacts[line_axis],
                                &policy,
                            );
                            same(
                                &edited.curves().last().unwrap().start(),
                                &contacts[spline_axis],
                                &policy,
                            );
                            for pair in edited.curves().windows(2) {
                                same(&pair[0].end(), &pair[1].start(), &policy);
                            }
                            assert_eq!(
                                edited.reversed(&CurveContext::STRICT).unwrap().certainty,
                                crate::CurveCertainty::Certified
                            );
                            let difference = closed_region(edited, &policy)
                                .boolean_regions(&closed_region(&witness, &policy), &policy)
                                .unwrap();
                            assert_eq!(difference.certainty, crate::CurveCertainty::Certified);
                            assert!(difference.value.xor().is_empty());
                        }
                        request.contacts[line_axis] = Some(CurveFilletContact2::Point(
                            Point2::new(-&radius * q(1, 2), Real::zero()).into(),
                        ));
                        assert!(
                            path.fillet_vertex(1, &request, mode, &policy)
                                .unwrap()
                                .value
                                .solutions()
                                .is_empty()
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn nonlinear_linear_fillet_components_retain_unique_contacts_and_tangents() {
        for rotated in [false, true] {
            let p = |x: i64, y: i64| {
                if rotated {
                    Point2::new(q(3 * x - 4 * y, 5), q(4 * x + 3 * y, 5))
                } else {
                    Point2::from_values(x, y)
                }
            };
            let spline = Curve2::try_nurbs(
                2,
                vec![p(0, 0), p(0, 1), p(0, 2), p(-1, 2), p(-3, 2)],
                vec![Real::one(); 5],
                [0, 0, 0, 1, 1, 2, 2, 2]
                    .into_iter()
                    .map(Real::from)
                    .collect(),
                &CurveContext::STRICT,
            )
            .unwrap()
            .value;
            let witness = CurvePath2::try_new(vec![
                LineSeg2::try_new(p(-3, 0), p(-2, 0)).unwrap().into(),
                CircularArc2::try_from_center(p(-2, 0), p(-2, 2), p(-2, 1), false)
                    .unwrap()
                    .into(),
                LineSeg2::try_new(p(-2, 2), p(-3, 2)).unwrap().into(),
            ])
            .unwrap();
            for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
                let line = LineSeg2::try_new(p(-3, 0), p(0, 0)).unwrap();
                let chord = algebraic_chord_from_line_support(
                    &line,
                    CurveOperation2::Fillet,
                    CurveFamily2::Line,
                    &policy,
                )
                .unwrap();
                let retained = chord
                    .parallel_left_retained(Real::one(), &policy)
                    .unwrap()
                    .parallel_left_retained(-Real::one(), &policy)
                    .unwrap();
                assert!(retained.start().coordinates().is_none());
                assert!(retained.end().coordinates().is_none());
                for (representation, first) in [
                    Curve2::from(line),
                    Curve2::from(chord),
                    Curve2::from(retained),
                ]
                .into_iter()
                .enumerate()
                {
                    let source = CurvePath2::try_new(vec![first, spline.clone()]).unwrap();
                    for reversed in [false, true] {
                        let path = if reversed {
                            source.reversed(&policy).unwrap().value
                        } else {
                            source.clone()
                        };
                        let spline_axis = usize::from(!reversed);
                        for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                            assert!(matches!(
                                path.fillet_vertex(
                                    1,
                                    &CurveFillet2::new(Real::one()),
                                    mode,
                                    &policy
                                ),
                                Err(ExactCurveError::Invalid {
                                    cause: CurveError::FilletConstraintRequired,
                                    ..
                                })
                            ));
                            for selection in 0..5 {
                                let mut request = CurveFillet2::new(Real::one());
                                match selection {
                                    0 => request.center = Some(p(-2, 1).into()),
                                    1 => {
                                        request.contacts[spline_axis] =
                                            Some(CurveFilletContact2::Point(p(-2, 2).into()))
                                    }
                                    2 => {
                                        // On the second span x(u)=-2u-u². Keep its
                                        // authored knot coordinate, including reversal.
                                        let parameter = Real::from(3).sqrt().unwrap();
                                        request.contacts[spline_axis] =
                                            Some(CurveFilletContact2::Parameter(
                                                (if reversed {
                                                    Real::from(2) - parameter
                                                } else {
                                                    parameter
                                                })
                                                .into(),
                                            ));
                                    }
                                    3 => {
                                        request.contacts[1 - spline_axis] =
                                            Some(CurveFilletContact2::Point(p(-2, 0).into()))
                                    }
                                    _ => {
                                        let parameter = match path.curves()[1 - spline_axis]
                                            .retained_fragment()
                                        {
                                            Some(crate::BezierSplitFragment2::AlgebraicChord(
                                                chord,
                                            )) => CurveParameter2::from_algebraic_chord(
                                                chord
                                                    .parameter_at_certified_support_point(
                                                        p(-2, 0).into(),
                                                        &policy,
                                                    )
                                                    .unwrap(),
                                            ),
                                            _ => (if reversed { q(2, 3) } else { q(1, 3) }).into(),
                                        };
                                        request.contacts[1 - spline_axis] =
                                            Some(CurveFilletContact2::Parameter(parameter));
                                    }
                                }
                                let selected = path.fillet_vertex(1, &request, mode, &policy)
                            .unwrap_or_else(|error| panic!("nonlinear line fillet: {error}; rotated={rotated}, representation={representation}, reversed={reversed}, mode={mode:?}, selection={selection}"));
                                assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                                assert_eq!(
                                    selected.value.candidate_count(),
                                    1,
                                    "rotated={rotated}, representation={representation}, reversed={reversed}, mode={mode:?}, selection={selection}"
                                );
                                let edited = &selected.value.solutions()[0];
                                same(&edited.start(), &path.start(), &policy);
                                same(&edited.end(), &path.end(), &policy);
                                let contacts = [p(-2, 0).into(), p(-2, 2).into()];
                                same(
                                    &edited.curves().first().unwrap().end(),
                                    &contacts[usize::from(reversed)],
                                    &policy,
                                );
                                same(
                                    &edited.curves().last().unwrap().start(),
                                    &contacts[spline_axis],
                                    &policy,
                                );
                                for pair in edited.curves().windows(2) {
                                    same(&pair[0].end(), &pair[1].start(), &policy);
                                }
                                let difference = closed_region(edited, &policy)
                                    .boolean_regions(&closed_region(&witness, &policy), &policy)
                                    .unwrap();
                                assert_eq!(difference.certainty, crate::CurveCertainty::Certified);
                                assert!(difference.value.xor().is_empty());
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn coincident_linear_fillet_domains_exclude_remote_endpoints() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for left in [true, false] {
                let previous =
                    LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(1, 0))
                        .unwrap();
                let next = if left {
                    LineSeg2::try_new(Point2::from_values(0, 2), Point2::from_values(-1, 2))
                        .unwrap()
                } else {
                    LineSeg2::try_new(Point2::from_values(2, 2), Point2::from_values(1, 2)).unwrap()
                };
                let retained = |line: &LineSeg2| {
                    let chord = algebraic_chord_from_line_support(
                        line,
                        CurveOperation2::Fillet,
                        CurveFamily2::Line,
                        &policy,
                    )
                    .unwrap();
                    let chord = chord
                        .parallel_left_retained(Real::one(), &policy)
                        .unwrap()
                        .parallel_left_retained(-Real::one(), &policy)
                        .unwrap();
                    assert!(chord.start().coordinates().is_none());
                    assert!(chord.end().coordinates().is_none());
                    chord
                };
                let previous_chord = retained(&previous);
                let next_chord = retained(&next);
                for use_retained in [false, true] {
                    for reversed in [false, true] {
                        let lines = if reversed {
                            [next.reversed(), previous.reversed()]
                        } else {
                            [previous.clone(), next.clone()]
                        };
                        let chords = if reversed {
                            [next_chord.reversed(), previous_chord.reversed()]
                        } else {
                            [previous_chord.clone(), next_chord.clone()]
                        };
                        let curves = if use_retained {
                            chords.clone().map(|chord| {
                                Curve2::from_retained_fragment(
                                    crate::BezierSplitFragment2::AlgebraicChord(chord),
                                )
                            })
                        } else {
                            lines.clone().map(Curve2::from)
                        };
                        for previous_mode in
                            [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend]
                        {
                            for next_mode in
                                [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend]
                            {
                                let has_family = if left { next_mode } else { previous_mode }
                                    == CurveCornerMode2::TrimOrExtend;
                                let modes = if reversed {
                                    [next_mode, previous_mode]
                                } else {
                                    [previous_mode, next_mode]
                                };
                                for chart in [false, true] {
                                    let domains = modes.map(|mode| {
                                        if chart {
                                            FilletContactDomain2::SourceChart(mode)
                                        } else {
                                            FilletContactDomain2::AuthoredCurve(mode)
                                        }
                                    });
                                    let mut request = CurveFillet2::new(Real::one());
                                    let solve = |request: &CurveFillet2| {
                                        let binding = FilletConstraintBinding2 {
                                            request,
                                            sources: curves.each_ref(),
                                            maps: [None, None],
                                            charts: [FilletContactChart2::Parameter; 2],
                                        };
                                        let [previous, next] = if use_retained {
                                            chords
                                                .each_ref()
                                                .map(ExactCornerCarrier2::AlgebraicChord)
                                        } else {
                                            lines.each_ref().map(ExactCornerCarrier2::Line)
                                        };
                                        solve_carrier_fillet_corner(
                                            previous,
                                            next,
                                            &request.radius,
                                            false,
                                            domains,
                                            CurveFamily2::Line,
                                            CurveFamily2::Line,
                                            Some(&binding),
                                            &policy,
                                        )
                                    };
                                    let unconstrained = solve(&request);
                                    if has_family {
                                        assert!(matches!(
                                            unconstrained,
                                            Err(ExactCurveError::Invalid {
                                                cause: CurveError::FilletConstraintRequired,
                                                ..
                                            })
                                        ));
                                    } else {
                                        assert_eq!(
                                            unconstrained.unwrap().no_solution_reason(),
                                            Some(CurveCornerNoSolution2::OutsideTrimDomain)
                                        );
                                    }
                                    let x = if left { q(1, 2) } else { q(3, 2) };
                                    request.center = Some(Point2::new(x, Real::one()).into());
                                    assert_eq!(
                                        solve(&request).unwrap().candidate_count(),
                                        usize::from(has_family)
                                    );
                                    // The touching endpoint is always remote
                                    // on one source. Internal chart ownership
                                    // cannot turn it into an admissible cut.
                                    request.center =
                                        Some(Point2::from_values(i64::from(!left), 1).into());
                                    assert!(solve(&request).unwrap().solutions().is_empty());
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn coincident_retained_line_fillet_keeps_independent_endpoint_fields() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let selected = |end, denominator| {
                let Classification::Decided(polynomial) =
                    crate::BezierParameterPolynomial::try_new_power_basis(
                        vec![-Real::one(), Real::zero(), Real::from(denominator)],
                        &policy,
                    )
                    .unwrap()
                else {
                    panic!("inverse-square polynomial")
                };
                let Classification::Decided(roots) =
                    polynomial.isolate_unit_interval_roots(&policy).unwrap()
                else {
                    panic!("selected inverse-square parameter")
                };
                assert_eq!(roots.len(), 1);
                let source = RationalBezier2::try_new(
                    vec![Point2::from_values(0, 0), end],
                    vec![Real::one(); 2],
                )
                .unwrap();
                crate::rational_bezier_general::exact_contact_point_evidence(
                    &source, &roots[0], &policy,
                )
                .unwrap()
                .unwrap()
            };
            let Classification::Decided(previous) = crate::BezierAlgebraicChord2::try_new(
                selected(Point2::from_values(0, 1), 3),
                selected(Point2::from_values(1, 0), 2),
                &policy,
            )
            .unwrap() else {
                panic!("independently selected line endpoints")
            };
            assert!(previous.exact_line().is_none());
            assert!(previous.strict_provenance_support_line(&policy).is_none());
            assert!(previous.certified_unit_tangent().is_none());
            let next = previous
                .parallel_left_retained(Real::from(2), &policy)
                .unwrap()
                .reversed();
            let Classification::Decided(first_contact) = previous
                .endpoint_at_signed_tangent_distance(false, q(1, 4), &policy)
                .unwrap()
            else {
                panic!("retained exact tangent displacement")
            };
            let second_contact = previous
                .normal_displaced_point_evidence(first_contact.clone(), Real::from(2), &policy)
                .unwrap();
            let center = previous
                .normal_displaced_point_evidence(first_contact.clone(), Real::one(), &policy)
                .unwrap();
            assert!(center.coordinates().is_none());
            for reversed in [false, true] {
                let chords = if reversed {
                    [next.reversed(), previous.reversed()]
                } else {
                    [previous.clone(), next.clone()]
                };
                let curves = chords.clone().map(|chord| {
                    Curve2::from_retained_fragment(crate::BezierSplitFragment2::AlgebraicChord(
                        chord,
                    ))
                });
                let mut request = CurveFillet2::new(Real::one());
                let solve = |request: &CurveFillet2| {
                    let binding = FilletConstraintBinding2 {
                        request,
                        sources: curves.each_ref(),
                        maps: [None, None],
                        charts: [FilletContactChart2::Parameter; 2],
                    };
                    solve_carrier_fillet_corner(
                        ExactCornerCarrier2::AlgebraicChord(&chords[0]),
                        ExactCornerCarrier2::AlgebraicChord(&chords[1]),
                        &request.radius,
                        false,
                        [FilletContactDomain2::AuthoredCurve(CurveCornerMode2::TrimOnly); 2],
                        CurveFamily2::Line,
                        CurveFamily2::Line,
                        Some(&binding),
                        &policy,
                    )
                };
                assert!(matches!(
                    solve(&request),
                    Err(ExactCurveError::Invalid {
                        cause: CurveError::FilletConstraintRequired,
                        ..
                    })
                ));
                for select_center in [false, true] {
                    request.center = select_center.then(|| center.clone());
                    request.contacts = [None, None];
                    if !select_center {
                        request.contacts[usize::from(reversed)] =
                            Some(CurveFilletContact2::Point(first_contact.clone()));
                    }
                    let result = solve(&request).unwrap();
                    let [corner] = result.solutions() else {
                        panic!("one retained semicircle")
                    };
                    let expected = if reversed {
                        [&second_contact, &first_contact]
                    } else {
                        [&first_contact, &second_contact]
                    };
                    same(&corner.previous.point, expected[0], &policy);
                    same(&corner.next.point, expected[1], &policy);
                    same(&corner.center, &center, &policy);
                    assert_eq!(corner.clockwise, reversed);
                    assert!(corner.retained_frame.is_some());
                }
            }
        }
    }

    #[test]
    fn collapsed_parallel_fillet_requires_the_original_circle_normal() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = RationalQuadraticBezier2::try_unit_end_weights(
                Point2::from_values(1, 0),
                Point2::from_values(1, 1),
                Point2::from_values(0, 1),
                (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap(),
            )
            .unwrap()
            .parallel_left(q(1, 4))
            .unwrap();
            let Classification::Decided(fragment) = crate::BezierParallelFragment2::try_new(
                source,
                BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                ),
                &policy,
            )
            .unwrap() else {
                panic!("regular circular parallel");
            };
            // These coincident circles run in opposite directions. Their
            // radius-offset images can coincide only on the wrong normal sheet.
            let arc = CircularArc2::try_from_center(
                Point2::new(Real::zero(), q(3, 4)),
                Point2::new(q(3, 4), Real::zero()),
                Point2::from_values(0, 0),
                true,
            )
            .unwrap();
            let curves = [Curve2::from(arc.clone()), Curve2::from(fragment.clone())];
            let mut request = CurveFillet2::new(q(3, 4));
            request.center = Some(Point2::from_values(0, 0).into());
            let binding = FilletConstraintBinding2 {
                request: &request,
                sources: curves.each_ref(),
                maps: [None, None],
                charts: [
                    FilletContactChart2::CircularSweep,
                    FilletContactChart2::Parameter,
                ],
            };
            // Exercise the retained carrier directly, independently of the
            // optional promotion of a represented circular parallel to an arc.
            let result = solve_carrier_fillet_corner(
                ExactCornerCarrier2::Arc(&arc),
                ExactCornerCarrier2::AnalyticParallel(&fragment),
                &request.radius,
                false,
                [FilletContactDomain2::AuthoredCurve(CurveCornerMode2::TrimOrExtend); 2],
                curves[0].family(),
                curves[1].family(),
                Some(&binding),
                &policy,
            )
            .unwrap();
            assert!(result.solutions().is_empty());
        }
    }

    #[test]
    fn collapsed_circle_parallel_fillet_replays_contacts_and_normal_sheets() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for representation in 0..3 {
                let (other, center, radius, arc_start, circle_contact, parallel_contact) =
                    if representation == 0 {
                        (
                            Curve2::from(QuadraticBezier2::new(
                                Point2::from_values(-1, 0),
                                Point2::from_values(0, 2),
                                Point2::from_values(1, 0),
                            )),
                            Point2::from_values(0, 0),
                            Real::one(),
                            Point2::from_values(0, -1),
                            Point2::new(q(-3, 5), q(-4, 5)),
                            Point2::from_values(0, 1),
                        )
                    } else {
                        // P(x)=(x,1-x²), -3/8 <= x <= 3/8, offset by -1.
                        // Its derivative scale is negative throughout this range.
                        // At x=0 the contact is (0,0), traversed toward the left.
                        let parallel = QuadraticBezier2::new(
                            Point2::new(q(-3, 8), q(55, 64)),
                            Point2::new(Real::zero(), q(73, 64)),
                            Point2::new(q(3, 8), q(55, 64)),
                        )
                        .parallel_left(Real::from(-1))
                        .unwrap();
                        let other = if representation == 1 {
                            let Classification::Decided(fragment) =
                                crate::BezierParallelFragment2::try_new(
                                    parallel,
                                    BezierParameterRange2::new_validated(
                                        BezierParameter2::Exact(Real::zero()),
                                        BezierParameter2::Exact(Real::one()),
                                    ),
                                    &policy,
                                )
                                .unwrap()
                            else {
                                panic!("regular negative-scale parallel");
                            };
                            Curve2::from(fragment)
                        } else {
                            crate::bezier_split::BezierSelectedFiberFragment2::new(
                                crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(
                                    parallel,
                                ),
                                CurveParameterRange2::unit(),
                                Point2::new(q(9, 40), q(19, 320)).into(),
                                Point2::new(q(-9, 40), q(19, 320)).into(),
                            )
                            .into()
                        };
                        let radius = q(1109, 2432);
                        (
                            other,
                            Point2::new(Real::zero(), radius.clone()),
                            radius.clone(),
                            Point2::new(Real::zero(), Real::from(2) * &radius),
                            Point2::new(radius.clone(), radius),
                            Point2::from_values(0, 0),
                        )
                    };
                let join = if representation == 0 {
                    Point2::from_values(-1, 0)
                } else {
                    Point2::new(q(9, 40), q(19, 320))
                };
                same(&other.start(), &join.clone().into(), &policy);
                let arc =
                    CircularArc2::try_from_center(arc_start, join, center.clone(), true).unwrap();
                let source = CurvePath2::try_new_with_policy(vec![arc.into(), other], &policy)
                    .unwrap()
                    .value;
                for reversed in [false, true] {
                    let path = if reversed {
                        source.reversed(&policy).unwrap().value
                    } else {
                        source.clone()
                    };
                    let circle_axis = usize::from(reversed);
                    let parallel_axis = 1 - circle_axis;
                    for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                        let mut request = CurveFillet2::new(radius.clone());
                        request.center = Some(center.clone().into());
                        assert!(matches!(
                            path.fillet_vertex(1, &request, mode, &policy),
                            Err(ExactCurveError::Invalid {
                                cause: CurveError::FilletConstraintRequired,
                                ..
                            })
                        ));
                        request.contacts[circle_axis] =
                            Some(CurveFilletContact2::Point(circle_contact.clone().into()));
                        for contact in [
                            None,
                            Some(CurveFilletContact2::Parameter(q(1, 2).into())),
                            Some(CurveFilletContact2::Point(parallel_contact.clone().into())),
                        ] {
                            request.contacts[parallel_axis] = contact;
                            let selected = path.fillet_vertex(1, &request, mode, &policy).unwrap();
                            assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                            assert_eq!(selected.value.candidate_count(), 1);
                            let edited = &selected.value.solutions()[0];
                            let contacts = if reversed {
                                [&parallel_contact, &circle_contact]
                            } else {
                                [&circle_contact, &parallel_contact]
                            };
                            same(
                                &edited.curves().first().unwrap().end(),
                                &contacts[0].clone().into(),
                                &policy,
                            );
                            same(
                                &edited.curves().last().unwrap().start(),
                                &contacts[1].clone().into(),
                                &policy,
                            );
                            for pair in edited.curves().windows(2) {
                                same(&pair[0].end(), &pair[1].start(), &policy);
                            }
                            let replay = edited.reversed(&CurveContext::STRICT).unwrap();
                            assert_eq!(replay.certainty, crate::CurveCertainty::Certified);
                        }
                        request.contacts[parallel_axis] =
                            Some(CurveFilletContact2::Parameter(q(1, 3).into()));
                        let excluded = path.fillet_vertex(1, &request, mode, &policy).unwrap();
                        assert_eq!(excluded.certainty, crate::CurveCertainty::Certified);
                        assert!(excluded.value.solutions().is_empty());
                    }
                }
            }
        }
    }

    #[test]
    fn collapsed_fillet_point_contacts_preserve_distinct_bezier_preimages() {
        // x(t)=(32/3)(t-1/4)(t-1/2)(t-3/4), y(t)=1-x(t)^2.
        // The first and third visits to (0,1) have the required orientation;
        // the middle visit traverses in the opposite direction.
        let points = vec![
            Point2::from_values(-1, 0),
            Point2::new(q(2, 9), q(22, 9)),
            Point2::new(q(17, 45), q(-112, 135)),
            Point2::new(Real::zero(), q(134, 45)),
            Point2::new(q(-17, 45), q(-112, 135)),
            Point2::new(q(-2, 9), q(22, 9)),
            Point2::from_values(1, 0),
        ];
        let curve = RationalBezier2::try_new(points, vec![Real::one(); 7]).unwrap();
        let source = CurvePath2::try_new(vec![
            CircularArc2::try_from_center(
                Point2::from_values(0, -1),
                Point2::from_values(-1, 0),
                Point2::from_values(0, 0),
                true,
            )
            .unwrap()
            .into(),
            curve.into(),
        ])
        .unwrap();
        let cutter = CurvePath2::try_new(vec![
            LineSeg2::try_new(
                Point2::new(q(-1, 20), q(399, 400) - q(1, 10000)),
                Point2::new(q(-1, 20), q(399, 400) + q(1, 10000)),
            )
            .unwrap()
            .into(),
        ])
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for reversed in [false, true] {
                let path = if reversed {
                    source.reversed(&policy).unwrap().value
                } else {
                    source.clone()
                };
                let circle_axis = usize::from(reversed);
                let curve_axis = 1 - circle_axis;
                let mut request = CurveFillet2::new(Real::one());
                request.center = Some(Point2::from_values(0, 0).into());
                request.contacts[circle_axis] = Some(CurveFilletContact2::Point(
                    Point2::new(q(-3, 5), q(-4, 5)).into(),
                ));
                for contact in [
                    None,
                    Some(CurveFilletContact2::Point(Point2::from_values(0, 1).into())),
                ] {
                    request.contacts[curve_axis] = contact;
                    let selected = path
                        .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                        .unwrap();
                    assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(selected.value.candidate_count(), 2);
                }
                for (visit, parameter) in [q(1, 4), q(3, 4)].into_iter().enumerate() {
                    request.contacts[curve_axis] =
                        Some(CurveFilletContact2::Parameter(parameter.into()));
                    let selected = path
                        .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                        .unwrap();
                    assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(selected.value.candidate_count(), 1);
                    let crossings = selected.value.solutions()[0]
                        .intersect_path(&cutter, &policy)
                        .unwrap();
                    assert_eq!(crossings.certainty, crate::CurveCertainty::Certified);
                    assert!(crossings.value.blockers().is_empty());
                    assert_eq!(
                        crossings.value.contacts().len(),
                        if (visit == 0) != reversed { 2 } else { 0 }
                    );
                }
                request.contacts[curve_axis] = Some(CurveFilletContact2::Parameter(q(1, 2).into()));
                let reversed_normal = path
                    .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                    .unwrap();
                assert_eq!(reversed_normal.certainty, crate::CurveCertainty::Certified);
                assert!(reversed_normal.value.solutions().is_empty());
            }
        }
    }

    fn generated_selected_fillet_circle(policy: &CurveContext) -> Curve2 {
        let p = Point2::from_values;
        let source = CurvePath2::try_new(vec![
            LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap().into(),
            QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2)).into(),
        ])
        .unwrap();
        let generated = source
            .fillet_vertex(
                1,
                &CurveFillet2::new(Real::one()),
                CurveCornerMode2::TrimOnly,
                policy,
            )
            .unwrap();
        assert_eq!(generated.certainty, crate::CurveCertainty::Certified);
        assert_eq!(generated.value.candidate_count(), 1);
        generated.value.solutions()[0]
            .curves()
            .iter()
            .find(|curve| curve.family() == CurveFamily2::CircularArc && curve.geometry().is_none())
            .expect("a generated selected circle")
            .clone()
    }

    #[test]
    fn collapsed_selected_circle_fillet_reuses_constrained_contacts_and_support() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let circle = generated_selected_fillet_circle(&policy);
            let arc = circle
                .subcurve(q(1, 16).into(), q(3, 16).into(), &policy)
                .unwrap()
                .value;
            let (first, second) = arc.split_at(q(1, 8).into(), &policy).unwrap().value;
            let path = CurvePath2::try_new(vec![first, second]).unwrap();
            let region = |path: &CurvePath2| {
                let Classification::Decided(closing) =
                    crate::BezierAlgebraicChord2::try_new(path.end(), path.start(), &policy)
                        .unwrap()
                else {
                    panic!("a nonzero exact closing chord")
                };
                let mut curves = path.curves().to_vec();
                curves.push(Curve2::from_retained_fragment(
                    crate::BezierSplitFragment2::AlgebraicChord(closing),
                ));
                let result = crate::CurveRegion2::try_from_boundary_paths(
                    &[CurvePath2::try_new(curves).unwrap()],
                    &policy,
                )
                .unwrap();
                assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                result.value
            };
            let expected_region = region(&CurvePath2::try_new(vec![arc]).unwrap());
            let parameters: Vec<_> = path
                .curves()
                .iter()
                .map(|curve| {
                    let (start, end) = curve.parameter_domain().scalar_endpoints().unwrap();
                    CurveParameter2::from((start + end) * q(1, 2))
                })
                .collect();
            let contacts: Vec<_> = path
                .curves()
                .iter()
                .zip(&parameters)
                .map(|(curve, parameter)| curve.point_at(parameter, &policy).unwrap().value)
                .collect();
            for (curve, contact) in path.curves().iter().zip(&contacts) {
                assert!(curve.geometry().is_none());
                assert!(contact.coordinates().is_none());
                assert_eq!(
                    contact.same_point(&curve.start(), &policy),
                    Classification::Decided(false)
                );
                assert_eq!(
                    contact.same_point(&curve.end(), &policy),
                    Classification::Decided(false)
                );
            }
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                let mut contacts = [contacts[0].clone(), contacts[1].clone()];
                let mut parameters = [parameters[0].clone(), parameters[1].clone()];
                if reversed {
                    contacts.reverse();
                    parameters.reverse();
                }
                for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                    let mut request = CurveFillet2::new(Real::one());
                    for stage in 0..2 {
                        if stage == 1 {
                            request.contacts[0] =
                                Some(CurveFilletContact2::Point(contacts[0].clone()));
                        }
                        assert!(matches!(
                            path.fillet_vertex(1, &request, mode, &policy),
                            Err(ExactCurveError::Invalid {
                                cause: CurveError::FilletConstraintRequired,
                                ..
                            })
                        ));
                    }
                    for by_parameter in [false, true] {
                        eprintln!(
                            "selected circle: policy={policy:?}, reversed={reversed}, mode={mode:?}, parameter={by_parameter}"
                        );
                        request.contacts = if by_parameter {
                            parameters
                                .clone()
                                .map(|parameter| Some(CurveFilletContact2::Parameter(parameter)))
                        } else {
                            contacts
                                .clone()
                                .map(|point| Some(CurveFilletContact2::Point(point)))
                        };
                        let result = path.fillet_vertex(1, &request, mode, &policy).unwrap_or_else(|error| {
                            panic!("selected circle: policy={policy:?}, reversed={reversed}, mode={mode:?}, parameter={by_parameter}: {error}")
                        });
                        assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                        assert_eq!(result.value.candidate_count(), 1);
                        let edited = &result.value.solutions()[0];
                        same(&edited.start(), &path.start(), &policy);
                        same(&edited.end(), &path.end(), &policy);
                        for pair in edited.curves().windows(2) {
                            same(&pair[0].end(), &pair[1].start(), &policy);
                        }
                        for contact in &contacts {
                            assert!(
                                edited
                                    .curves()
                                    .iter()
                                    .any(|curve| curve.start().same_point(contact, &policy)
                                        == Classification::Decided(true))
                            );
                        }
                        let edited_region = region(edited);
                        let difference = edited_region
                            .boolean_regions(&expected_region, &policy)
                            .unwrap();
                        assert_eq!(difference.certainty, crate::CurveCertainty::Certified);
                        assert!(difference.value.xor().is_empty());
                    }
                }
            }
        }
    }

    #[test]
    fn collapsed_selected_circle_fillet_crosses_seams_and_reenters_region_editing() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let generated = generated_selected_fillet_circle(&policy);
            let Some(crate::BezierSplitFragment2::AlgebraicCuspSemicircle(fragment)) =
                generated.retained_fragment()
            else {
                panic!("a selected fillet circle")
            };
            let circle = fragment.semicircle();
            let halves = [circle.clone(), circle.complementary_half()].map(|half| {
                Curve2::from_retained_fragment(
                    crate::BezierSplitFragment2::AlgebraicCuspSemicircle(
                        crate::BezierAlgebraicCuspSemicircleFragment2::full(half, &policy),
                    ),
                )
            });
            let source = CurvePath2::try_new(halves.to_vec()).unwrap();
            let Classification::Decided(center) = circle.center_point_evidence(&policy).unwrap()
            else {
                panic!("a retained center")
            };
            let region = |path: &CurvePath2| {
                let region = crate::CurveRegion2::try_from_boundary_paths(
                    std::slice::from_ref(path),
                    &policy,
                )
                .unwrap();
                assert_eq!(region.certainty, crate::CurveCertainty::Certified);
                assert!(!region.value.is_empty());
                region.value
            };
            let expected = region(&source);
            for reversed in [false, true] {
                let path = if reversed {
                    source.reversed(&policy).unwrap().value
                } else {
                    source.clone()
                };
                let mut contacts = [
                    halves[0].point_at(&q(1, 4).into(), &policy).unwrap().value,
                    halves[1].point_at(&q(3, 4).into(), &policy).unwrap().value,
                ];
                if reversed {
                    contacts.reverse();
                }
                for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                    eprintln!(
                        "selected-circle seam: policy={policy:?}, reversed={reversed}, mode={mode:?}"
                    );
                    let mut request = CurveFillet2::new(Real::one());
                    request.center = Some(center.clone());
                    assert!(matches!(
                        path.fillet_vertex(1, &request, mode, &policy),
                        Err(ExactCurveError::Invalid {
                            cause: CurveError::FilletConstraintRequired,
                            ..
                        })
                    ));
                    request.contacts = contacts
                        .clone()
                        .map(|point| Some(CurveFilletContact2::Point(point)));
                    let result = path.fillet_vertex(1, &request, mode, &policy).unwrap();
                    assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(result.value.candidate_count(), 1);
                    let edited = &result.value.solutions()[0];
                    let edited_region = region(edited);
                    let difference = edited_region.boolean_regions(&expected, &policy).unwrap();
                    assert_eq!(difference.certainty, crate::CurveCertainty::Certified);
                    assert!(difference.value.xor().is_empty());
                    // A remote endpoint is never an incident extension. This
                    // remains true when its evidence uses the opposite half.
                    request.contacts[0] = Some(CurveFilletContact2::Point(path.start()));
                    let excluded = path.fillet_vertex(1, &request, mode, &policy).unwrap();
                    assert_eq!(excluded.certainty, crate::CurveCertainty::Certified);
                    assert!(excluded.value.solutions().is_empty());
                    request.contacts = [None, Some(CurveFilletContact2::Point(center.clone()))];
                    let excluded = path.fillet_vertex(1, &request, mode, &policy).unwrap();
                    assert_eq!(excluded.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(
                        excluded.value.no_solution_reason(),
                        Some(CurveCornerNoSolution2::UnsatisfiedConstraints)
                    );
                    if mode == CurveCornerMode2::TrimOnly {
                        eprintln!("selected-circle seam: chamfer");
                        let chamfered = edited
                            .chamfer_vertex_by_setbacks(1, q(1, 16), q(1, 16), mode, &policy)
                            .unwrap();
                        assert_eq!(chamfered.certainty, crate::CurveCertainty::Certified);
                        assert_eq!(chamfered.value.candidate_count(), 1);
                        let chamfered = region(&chamfered.value.solutions()[0]);
                        let clipped = chamfered.boolean_regions(&expected, &policy).unwrap();
                        assert_eq!(clipped.certainty, crate::CurveCertainty::Certified);
                        assert!(clipped.value.difference().is_empty());
                        assert!(!clipped.value.xor().is_empty());
                        eprintln!("selected-circle seam: offset");
                        let expanded = chamfered
                            .offset(q(1, 32), &crate::OffsetCornerStyle2::Round, &policy)
                            .unwrap();
                        assert_eq!(expanded.certainty, crate::CurveCertainty::Certified);
                        assert!(!expanded.value.is_empty());
                        let contained =
                            chamfered.boolean_regions(&expanded.value, &policy).unwrap();
                        assert_eq!(contained.certainty, crate::CurveCertainty::Certified);
                        assert!(contained.value.difference().is_empty());
                        assert!(!contained.value.xor().is_empty());
                    }
                }
            }
        }
    }

    #[test]
    fn native_circle_fillet_retains_selected_contacts_through_reconstruction() {
        let p = Point2::from_values;
        let arc = |start, end| {
            Curve2::from(CircularArc2::try_from_center(start, end, p(0, 0), false).unwrap())
        };
        let quarter_path =
            || CurvePath2::try_new(vec![arc(p(1, 0), p(0, 1)), arc(p(0, 1), p(-1, 0))]).unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let contact = |denominator: i64, turns: usize| {
                let Classification::Decided(polynomial) =
                    crate::BezierParameterPolynomial::try_new_power_basis(
                        vec![-Real::one(), Real::zero(), Real::from(denominator)],
                        &policy,
                    )
                    .unwrap()
                else {
                    panic!("selected circle contact polynomial")
                };
                let Classification::Decided(roots) =
                    polynomial.isolate_unit_interval_roots(&policy).unwrap()
                else {
                    panic!("selected circle contact root")
                };
                assert_eq!(roots.len(), 1);
                let rotate = |mut point: Point2| {
                    for _ in 0..turns {
                        point = Point2::new(-point.y(), point.x().clone());
                    }
                    point
                };
                let source: Curve2 = RationalBezier2::try_new(
                    [p(1, 0), p(1, 1), p(0, 1)]
                        .into_iter()
                        .map(rotate)
                        .collect(),
                    vec![Real::one(), Real::one(), Real::from(2)],
                )
                .unwrap()
                .into();
                let point = source
                    .point_at(&roots[0].clone().into(), &policy)
                    .unwrap()
                    .value;
                assert!(point.coordinates().is_none());
                // An independently represented point is used only for the
                // expected set, never to supply the retained contact request.
                let expected = rotate(Point2::new(
                    q(denominator - 1, denominator + 1),
                    Real::from(denominator).sqrt().unwrap() * q(2, denominator + 1),
                ));
                same(&point, &expected.clone().into(), &policy);
                (point, expected)
            };
            for shape in 0..6 {
                let source = match shape {
                    0 | 3 => quarter_path(),
                    1 => CurvePath2::try_new(vec![arc(p(1, 0), p(0, -1)), arc(p(0, -1), p(1, 0))])
                        .unwrap(),
                    _ => CurvePath2::try_new(vec![arc(p(1, 0), p(1, 0))]).unwrap(),
                };
                let turns = match shape {
                    0 => [0, 1],
                    1 => [2, 3],
                    2 => [3, 0],
                    3 => [2, 1],
                    _ => [0, 3],
                };
                let [first, second] = [contact(2, turns[0]), contact(3, turns[1])];
                let witness = if shape == 3 {
                    CurvePath2::try_new(vec![
                        arc(p(1, 0), first.1.clone()),
                        arc(first.1.clone(), second.1.clone()),
                        arc(second.1.clone(), p(-1, 0)),
                    ])
                    .unwrap()
                } else {
                    source.clone()
                };
                for reversed in [false, true] {
                    let path = if reversed {
                        source.reversed(&policy).unwrap().value
                    } else {
                        source.clone()
                    };
                    let mut contacts = if shape == 5 {
                        [first.1.clone().into(), second.1.clone().into()]
                    } else {
                        [first.0.clone(), second.0.clone()]
                    };
                    if reversed {
                        contacts.reverse();
                    }
                    let mut request = CurveFillet2::new(Real::one());
                    request.contacts = contacts
                        .clone()
                        .map(|point| Some(CurveFilletContact2::Point(point)));
                    for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                        let result = path
                            .fillet_vertex(usize::from(matches!(shape, 0 | 1 | 3)), &request, mode, &policy)
                            .unwrap_or_else(|error| {
                                panic!("circle shape={shape}, reversed={reversed}, mode={mode:?}: {error}")
                            });
                        assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                        if shape >= 4 || (shape == 3 && mode == CurveCornerMode2::TrimOnly) {
                            assert!(
                                result.value.solutions().is_empty(),
                                "excluded circle cuts: shape={shape}, reversed={reversed}, mode={mode:?}"
                            );
                            continue;
                        }
                        assert_eq!(result.value.candidate_count(), 1);
                        let edited = &result.value.solutions()[0];
                        for pair in edited.curves().windows(2) {
                            same(&pair[0].end(), &pair[1].start(), &policy);
                        }
                        for contact in &contacts {
                            assert!(edited.curves().iter().any(|curve| {
                                curve.start().same_point(contact, &policy)
                                    == Classification::Decided(true)
                            }));
                        }
                        assert_eq!(
                            edited.reversed(&CurveContext::STRICT).unwrap().certainty,
                            crate::CurveCertainty::Certified
                        );
                        let region = |path: &CurvePath2, label| {
                            let mut curves = path.curves().to_vec();
                            match path.start().same_point(&path.end(), &policy) {
                                Classification::Decided(true) => (),
                                Classification::Decided(false) => {
                                    let Classification::Decided(closing) =
                                        crate::BezierAlgebraicChord2::try_new(
                                            path.end(),
                                            path.start(),
                                            &policy,
                                        )
                                        .unwrap()
                                    else {
                                        panic!("exact closing chord")
                                    };
                                    curves.push(Curve2::from_retained_fragment(
                                        crate::BezierSplitFragment2::AlgebraicChord(closing),
                                    ));
                                }
                                Classification::Uncertain(reason) => panic!("closure: {reason:?}"),
                            }
                            let outcome = crate::CurveRegion2::try_from_boundary_paths(
                                &[CurvePath2::try_new(curves).unwrap()],
                                &policy,
                            )
                            .unwrap();
                            assert_eq!(
                                outcome.certainty,
                                crate::CurveCertainty::Certified,
                                "normalization={label}, policy={policy:?}, shape={shape}, reversed={reversed}, mode={mode:?}"
                            );
                            outcome.value
                        };
                        let difference = region(edited, "edited")
                            .boolean_regions(&region(&witness, "witness"), &policy)
                            .unwrap();
                        assert_eq!(
                            difference.certainty,
                            crate::CurveCertainty::Certified,
                            "Boolean policy={policy:?}, shape={shape}, reversed={reversed}, mode={mode:?}"
                        );
                        assert!(difference.value.xor().is_empty());
                    }
                }
            }
        }
    }

    #[test]
    fn collapsed_circle_fillet_requires_each_free_contact_and_preserves_exact_trims() {
        let center = Point2::from_values(0, 0);
        let path = CurvePath2::try_new(vec![
            CircularArc2::try_from_center(
                Point2::from_values(1, 0),
                Point2::from_values(0, 1),
                center.clone(),
                false,
            )
            .unwrap()
            .into(),
            CircularArc2::try_from_center(
                Point2::from_values(0, 1),
                Point2::from_values(-1, 0),
                center.clone(),
                false,
            )
            .unwrap()
            .into(),
        ])
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                let mut contacts = [
                    CurvePoint2::from(Point2::new(q(3, 5), q(4, 5))),
                    CurvePoint2::from(Point2::new(q(-3, 5), q(4, 5))),
                ];
                if reversed {
                    contacts.reverse();
                }
                for mode in [CurveCornerMode2::TrimOnly, CurveCornerMode2::TrimOrExtend] {
                    let mut request = CurveFillet2::new(Real::one());
                    for stage in 0..3 {
                        if stage == 1 {
                            request.center = Some(center.clone().into());
                        }
                        if stage == 2 {
                            request.contacts[0] =
                                Some(CurveFilletContact2::Point(contacts[0].clone()));
                        }
                        assert!(matches!(
                            path.fillet_vertex(1, &request, mode, &policy),
                            Err(ExactCurveError::Invalid {
                                cause: CurveError::FilletConstraintRequired,
                                ..
                            })
                        ));
                    }
                    request.contacts[1] = Some(CurveFilletContact2::Point(contacts[1].clone()));
                    let result = path.fillet_vertex(1, &request, mode, &policy).unwrap();
                    assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(result.value.candidate_count(), 1);
                    let edited = &result.value.solutions()[0];
                    assert_eq!(edited.curves().len(), 3);
                    same(&edited.start(), &path.start(), &policy);
                    same(&edited.end(), &path.end(), &policy);
                    same(&edited.curves()[0].end(), &contacts[0], &policy);
                    same(&edited.curves()[1].start(), &contacts[0], &policy);
                    same(&edited.curves()[1].end(), &contacts[1], &policy);
                    same(&edited.curves()[2].start(), &contacts[1], &policy);
                    let Some(CurveGeometry2::CircularArc(arc)) = edited.curves()[1].geometry()
                    else {
                        panic!("native exact fillet circle");
                    };
                    assert_eq!(arc.center(), &center);
                    assert_eq!(arc.radius_squared(), Real::one());
                    assert_eq!(arc.is_clockwise(), reversed);

                    // Closing the same half-disk tests the regularized set
                    // independently of how the edited circle is partitioned.
                    let region = |path: &CurvePath2| {
                        let mut curves = path.curves().to_vec();
                        curves.push(
                            LineSeg2::try_new(
                                path.end().coordinates().unwrap().clone(),
                                path.start().coordinates().unwrap().clone(),
                            )
                            .unwrap()
                            .into(),
                        );
                        crate::CurveRegion2::try_from_boundary_paths(
                            &[CurvePath2::try_new(curves).unwrap()],
                            &policy,
                        )
                        .unwrap()
                        .value
                    };
                    let difference = region(&path)
                        .boolean_regions(&region(edited), &policy)
                        .unwrap();
                    assert_eq!(difference.certainty, crate::CurveCertainty::Certified);
                    assert!(difference.value.xor().is_empty());

                    request.center = Some(Point2::from_values(1, 0).into());
                    let excluded = path.fillet_vertex(1, &request, mode, &policy).unwrap();
                    assert_eq!(
                        excluded.value.no_solution_reason(),
                        Some(CurveCornerNoSolution2::UnsatisfiedConstraints)
                    );
                    request.center = Some(center.clone().into());
                    request.contacts = [
                        None,
                        Some(CurveFilletContact2::Point(Point2::from_values(0, 0).into())),
                    ];
                    // Contradiction is decided before demanding the free axis.
                    let excluded = path.fillet_vertex(1, &request, mode, &policy).unwrap();
                    assert_eq!(
                        excluded.value.no_solution_reason(),
                        Some(CurveCornerNoSolution2::UnsatisfiedConstraints)
                    );
                }
            }
        }
    }

    #[test]
    fn collapsed_circular_spline_fillets_keep_authored_contact_parameters() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let half_root_two = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
            let quarter = |points: [(i32, i32); 3], start: i32, end: i32| {
                Curve2::try_nurbs(
                    2,
                    points
                        .into_iter()
                        .map(|(x, y)| Point2::from_values(x, y))
                        .collect(),
                    vec![Real::one(), half_root_two.clone(), Real::one()],
                    vec![
                        Real::from(start),
                        Real::from(start),
                        Real::from(start),
                        Real::from(end),
                        Real::from(end),
                        Real::from(end),
                    ],
                    &policy,
                )
                .unwrap()
                .value
            };
            let path = CurvePath2::try_new(vec![
                quarter([(1, 0), (1, 1), (0, 1)], 2, 4),
                quarter([(0, 1), (-1, 1), (-1, 0)], 7, 9),
            ])
            .unwrap();
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                let mut parameters = [Real::from(3), Real::from(8)];
                let mut contacts = [
                    CurvePoint2::from(Point2::new(half_root_two.clone(), half_root_two.clone())),
                    CurvePoint2::from(Point2::new(-half_root_two.clone(), half_root_two.clone())),
                ];
                if reversed {
                    parameters.reverse();
                    contacts.reverse();
                }
                for by_parameter in [false, true] {
                    let mut request = CurveFillet2::new(Real::one());
                    request.contacts = std::array::from_fn(|axis| {
                        Some(if by_parameter {
                            CurveFilletContact2::Parameter(parameters[axis].clone().into())
                        } else {
                            CurveFilletContact2::Point(contacts[axis].clone())
                        })
                    });
                    let selected = path
                        .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                        .unwrap();
                    assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(selected.value.candidate_count(), 1);
                    let edited = &selected.value.solutions()[0];
                    same(
                        &edited.curves().first().unwrap().end(),
                        &contacts[0],
                        &policy,
                    );
                    same(
                        &edited.curves().last().unwrap().start(),
                        &contacts[1],
                        &policy,
                    );
                    for pair in edited.curves().windows(2) {
                        same(&pair[0].end(), &pair[1].start(), &policy);
                    }
                }
            }
        }
    }

    #[test]
    fn isolated_fillet_constraints_are_conjunctive_and_use_input_parameters() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let path = CurvePath2::try_new(vec![
                LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(2, 0))
                    .unwrap()
                    .into(),
                LineSeg2::try_new(Point2::from_values(2, 0), Point2::from_values(2, 2))
                    .unwrap()
                    .into(),
            ])
            .unwrap();
            let mut request = CurveFillet2::new(Real::one());
            request.center = Some(Point2::from_values(1, 1).into());
            request.contacts = [
                Some(CurveFilletContact2::Parameter(q(1, 2).into())),
                Some(CurveFilletContact2::Point(Point2::from_values(2, 1).into())),
            ];
            let selected = path
                .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                .unwrap();
            assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
            assert_eq!(selected.value.candidate_count(), 1);
            request.contacts[0] = Some(CurveFilletContact2::Parameter(q(1, 4).into()));
            let excluded = path
                .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                .unwrap();
            assert_eq!(excluded.certainty, crate::CurveCertainty::Certified);
            assert_eq!(
                excluded.value.no_solution_reason(),
                Some(CurveCornerNoSolution2::UnsatisfiedConstraints)
            );
            request.contacts = [None, None];
            request.center = Some(Point2::from_values(1, 2).into());
            let excluded = path
                .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                .unwrap();
            assert_eq!(
                excluded.value.no_solution_reason(),
                Some(CurveCornerNoSolution2::UnsatisfiedConstraints)
            );
        }
    }

    #[test]
    fn fillet_contacts_use_authored_spline_knots_across_multiple_charts() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let spline = Curve2::try_nurbs(
                1,
                vec![
                    Point2::from_values(0, 0),
                    Point2::from_values(2, 0),
                    Point2::from_values(4, 0),
                ],
                vec![Real::one(); 3],
                [2, 2, 3, 4, 4].into_iter().map(Real::from).collect(),
                &policy,
            )
            .unwrap()
            .value;
            let path = CurvePath2::try_new(vec![
                spline,
                LineSeg2::try_new(Point2::from_values(4, 0), Point2::from_values(4, 4))
                    .unwrap()
                    .into(),
            ])
            .unwrap();
            for reversed in [false, true] {
                let source = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                let parameters = if reversed {
                    [q(1, 4), q(7, 2)]
                } else {
                    [q(5, 2), q(3, 4)]
                };
                let mut request = CurveFillet2::new(Real::from(3));
                request.contacts =
                    parameters.map(|p| Some(CurveFilletContact2::Parameter(p.into())));
                let selected = source
                    .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                    .unwrap();
                assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                assert_eq!(selected.value.candidate_count(), 1);
                let edited = &selected.value.solutions()[0];
                let contacts: [CurvePoint2; 2] = [
                    Point2::from_values(1, 0).into(),
                    Point2::from_values(4, 3).into(),
                ];
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
            }
        }
    }

    #[test]
    fn circular_spline_contact_parameters_distinguish_repeated_point_visits() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let points = [
                (1, 0),
                (1, 1),
                (0, 1),
                (-1, 1),
                (-1, 0),
                (-1, -1),
                (0, -1),
                (1, -1),
                (1, 0),
                (1, 1),
                (0, 1),
                (-1, 1),
                (-1, 0),
                (-1, -1),
                (0, -1),
                (1, -1),
                (1, 0),
            ]
            .into_iter()
            .map(|(x, y)| Point2::from_values(x, y))
            .collect();
            let weights = (0..17).map(|i| Real::from(1_i64 << (i / 2))).collect();
            let mut knots = vec![Real::zero(); 3];
            for i in 1..8 {
                knots.extend([Real::from(i), Real::from(i)]);
            }
            knots.extend([Real::from(8), Real::from(8), Real::from(8)]);
            let circle = Curve2::try_nurbs(2, points, weights, knots, &policy)
                .unwrap()
                .value;
            let path = CurvePath2::try_new(vec![
                circle,
                LineSeg2::try_new(Point2::from_values(1, 0), Point2::from_values(6, 0))
                    .unwrap()
                    .into(),
            ])
            .unwrap();
            for reversed in [false, true] {
                let source = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                let axis = usize::from(reversed);
                let mut request = CurveFillet2::new(Real::from(4));
                request.contacts[axis] = Some(CurveFilletContact2::Point(
                    Point2::new(q(3, 5), -q(4, 5)).into(),
                ));
                let at_point = source
                    .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                    .unwrap();
                assert_eq!(at_point.certainty, crate::CurveCertainty::Certified);
                assert_eq!(
                    at_point.value.candidate_count(),
                    2,
                    "both visits to the constrained point survive"
                );
                let cutter = CurvePath2::try_new(vec![
                    LineSeg2::try_new(
                        Point2::new(q(3, 5), q(1, 2)),
                        Point2::new(q(3, 5), Real::one()),
                    )
                    .unwrap()
                    .into(),
                ])
                .unwrap();
                for (visit, parameter) in [q(10, 3), q(22, 3)].into_iter().enumerate() {
                    let parameter = CurveParameter2::from(if reversed {
                        Real::from(8) - parameter
                    } else {
                        parameter
                    });
                    request.contacts[axis] =
                        Some(CurveFilletContact2::Parameter(parameter.clone()));
                    let selected = source
                        .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                        .unwrap();
                    assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(
                        selected.value.candidate_count(),
                        1,
                        "a parameter selects one authored visit"
                    );
                    let edited = &selected.value.solutions()[0];
                    // Count passages through the upper right quadrant. The
                    // longer result keeps one additional exact circle traversal,
                    // regardless of its reconstructed chart partition.
                    let crossings = edited.intersect_path(&cutter, &policy).unwrap();
                    assert_eq!(crossings.certainty, crate::CurveCertainty::Certified);
                    assert!(crossings.value.blockers().is_empty());
                    assert_eq!(crossings.value.contacts().len(), visit + 1);
                }
            }
        }
    }

    #[test]
    fn promoted_line_contact_constraints_keep_the_nonlinear_source_chart() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // The incoming curve is x=4t^2 on [1/4,1]. Its contact at x=1
            // has t=1/2, independent of the chord's affine parameter.
            let source =
                Curve2::from_retained_fragment(crate::BezierSplitFragment2::RetainedBezier {
                    reversed: false,
                    start: BezierParameter2::Exact(q(1, 4)),
                    end: BezierParameter2::Exact(Real::one()),
                    source_curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                        Point2::from_values(0, 0),
                        Point2::from_values(0, 0),
                        Point2::from_values(4, 0),
                    )),
                    start_image: None,
                    end_image: None,
                });
            let path = CurvePath2::try_new_with_policy(
                vec![
                    source,
                    LineSeg2::try_new(Point2::from_values(4, 0), Point2::from_values(4, 4))
                        .unwrap()
                        .into(),
                ],
                &policy,
            )
            .unwrap()
            .value;
            let mut request = CurveFillet2::new(Real::from(3));
            request.contacts[0] = Some(CurveFilletContact2::Parameter(q(1, 2).into()));
            let selected = path
                .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                .unwrap();
            assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
            assert_eq!(selected.value.candidate_count(), 1);
            same(
                &selected.value.solutions()[0].curves()[0].end(),
                &Point2::from_values(1, 0).into(),
                &policy,
            );
            request.contacts[0] = Some(CurveFilletContact2::Parameter(q(3, 4).into()));
            let excluded = path
                .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                .unwrap();
            assert!(excluded.value.solutions().is_empty());
        }
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
                let request = CurveFillet2::new(q(1, 128));
                let unconstrained =
                    source.fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy);
                assert!(
                    matches!(
                        unconstrained,
                        Err(ExactCurveError::Invalid {
                            cause: CurveError::FilletConstraintRequired,
                            ..
                        })
                    ),
                    "radius-only family: error={:?}, solutions={:?}",
                    unconstrained.as_ref().err(),
                    unconstrained.as_ref().ok().map(|result| (
                        result.value.candidate_count(),
                        result.value.no_solution_reason()
                    ))
                );
                for excluded in [Real::zero(), Real::one()] {
                    let mut request = request.clone();
                    request.contacts[0] = Some(CurveFilletContact2::Parameter(excluded.into()));
                    let result = source
                        .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                        .unwrap();
                    assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                    assert!(result.value.solutions().is_empty());
                }
                let mut contradictory = request.clone();
                contradictory.contacts = [
                    Some(CurveFilletContact2::Parameter(parameter.clone())),
                    Some(CurveFilletContact2::Parameter(q(1, 2).into())),
                ];
                let excluded = source
                    .fillet_vertex(1, &contradictory, CurveCornerMode2::TrimOnly, &policy)
                    .unwrap();
                assert_eq!(excluded.certainty, crate::CurveCertainty::Certified);
                assert_eq!(
                    excluded.value.no_solution_reason(),
                    Some(CurveCornerNoSolution2::UnsatisfiedConstraints)
                );
                contradictory.contacts[1] = None;
                contradictory.center = Some(Point2::from_values(0, 0).into());
                let excluded = source
                    .fillet_vertex(1, &contradictory, CurveCornerMode2::TrimOnly, &policy)
                    .unwrap();
                assert_eq!(excluded.certainty, crate::CurveCertainty::Certified);
                assert_eq!(
                    excluded.value.no_solution_reason(),
                    Some(CurveCornerNoSolution2::UnsatisfiedConstraints)
                );
                let mut requests = Vec::new();
                let mut centered = request.clone();
                centered.center = Some(Point2::new(q(-175, 4992), q(4699, 7488)).into());
                requests.push(centered);
                for axis in 0..2 {
                    for contact in [
                        CurveFilletContact2::Parameter(parameter.clone()),
                        CurveFilletContact2::Point(
                            contacts[if reversed { 1 - axis } else { axis }].clone(),
                        ),
                    ] {
                        let mut constrained = request.clone();
                        constrained.contacts[axis] = Some(contact);
                        requests.push(constrained);
                    }
                }
                let mut paired = request;
                paired.contacts = [
                    Some(CurveFilletContact2::Parameter(parameter.clone())),
                    Some(CurveFilletContact2::Parameter(parameter.clone())),
                ];
                requests.push(paired);
                for request in requests {
                    let result = source
                        .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                        .unwrap();
                    assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(
                        result.value.candidate_count(),
                        1,
                        "one component owns the constrained contacts"
                    );
                    for edited in result.value.into_solutions() {
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
                        let replay = source
                            .fillet_vertex(
                                1,
                                &request,
                                CurveCornerMode2::TrimOnly,
                                &CurveContext::STRICT,
                            )
                            .unwrap();
                        assert_eq!(replay.certainty, crate::CurveCertainty::Certified);
                        assert_eq!(replay.value.candidate_count(), 1);
                    }
                }
            }
        }
    }

    #[test]
    fn continuous_fillet_constraints_accept_arbitrary_exact_real_contacts() {
        let parameter =
            CurveParameter2::from((Real::from(5).sqrt().unwrap() / Real::from(4)).unwrap());
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let path = joined_parallel_path(&policy, false);
            let contacts = path
                .curves()
                .iter()
                .map(|curve| curve.point_at(&parameter, &policy).unwrap().value)
                .collect::<Vec<_>>();
            for axis in 0..2 {
                let mut request = CurveFillet2::new(q(1, 128));
                request.contacts[axis] = Some(CurveFilletContact2::Parameter(parameter.clone()));
                let selected = path
                    .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                    .unwrap();
                assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                assert_eq!(selected.value.candidate_count(), 1);
                let edited = &selected.value.solutions()[0];
                same(&edited.curves()[0].end(), &contacts[0], &policy);
                same(
                    &edited.curves().last().unwrap().start(),
                    &contacts[1],
                    &policy,
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
            for (loop_index, boundary) in source.boundary_loops().iter().enumerate() {
                for (vertex, curve) in boundary.curves().iter().enumerate() {
                    if curve.start().coincides_with(&join, &policy).value
                        != Classification::Decided(true)
                    {
                        continue;
                    }
                    corners += 1;
                    let mut request = CurveFillet2::new(q(1, 128));
                    assert!(matches!(
                        source.fillet_loop_vertex(
                            loop_index,
                            vertex,
                            &request,
                            CurveCornerMode2::TrimOnly,
                            &policy
                        ),
                        Err(ExactCurveError::Invalid {
                            cause: CurveError::FilletConstraintRequired,
                            ..
                        })
                    ));
                    request.contacts[0] = Some(CurveFilletContact2::Parameter(parameter.clone()));
                    let selected = source
                        .fillet_loop_vertex(
                            loop_index,
                            vertex,
                            &request,
                            CurveCornerMode2::TrimOnly,
                            &policy,
                        )
                        .unwrap();
                    assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                    for region in selected.value.into_solutions() {
                        assert!(!region.is_empty());
                        let normalized = region.regularized_region(&policy).unwrap();
                        assert_eq!(normalized.certainty, crate::CurveCertainty::Certified);
                        let boolean = region.boolean_regions(&normalized.value, &policy).unwrap();
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
                            crate::CurveRegion2::try_from_boundary_paths(&[clip], &policy).unwrap();
                        assert_eq!(clip.certainty, crate::CurveCertainty::Certified);
                        let clipped = offset.value.boolean_regions(&clip.value, &policy).unwrap();
                        assert_eq!(clipped.certainty, crate::CurveCertainty::Certified);
                        assert!(!clipped.value.intersection().is_empty());
                        assert!(!clipped.value.difference().is_empty());
                        edited.push(region);
                    }
                }
            }
            assert_eq!(
                edited.len(),
                1,
                "one normalized boundary owns the selected fillet; corners={corners}"
            );
        }
    }
}

#[cfg(test)]
mod stationary_continuous_family_regression {
    use super::*;

    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    #[test]
    fn continuous_fillets_preserve_stationary_reparameterization() {
        // P(u)=(3u²/8,9u⁴/64) is the same parabola as the rational
        // continuous-family counterexample, with a stationary source endpoint.
        let base = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                Point2::new(q(1, 16), Real::zero()),
                Point2::new(q(3, 16), Real::zero()),
                Point2::new(q(3, 8), q(9, 64)),
            ],
            vec![Real::one(); 5],
        )
        .unwrap()
        .parallel_left(Real::zero())
        .unwrap();
        let parameter =
            CurveParameter2::from((Real::from(5).sqrt().unwrap() / Real::from(3)).unwrap());
        let center = CurvePoint2::from(Point2::new(q(-175, 4992), q(4699, 7488)));
        let contacts = [
            CurvePoint2::from(Point2::new(q(-95, 2496), q(4753, 7488))),
            CurvePoint2::from(Point2::new(q(-5, 156), q(4645, 7488))),
        ];
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let curves = [q(41, 64), q(5, 8)].map(|d| {
                let start = CurvePoint2::from(Point2::new(Real::zero(), d.clone()));
                let end =
                    CurvePoint2::from(Point2::new(q(3, 8) - q(3, 5) * &d, q(9, 64) + q(4, 5) * &d));
                Curve2::from(
                    crate::bezier_split::BezierSelectedFiberFragment2::new(
                        crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(
                            base.with_distance(d),
                        ),
                        CurveParameterRange2::unit(),
                        start,
                        end,
                    )
                    .reversed(),
                )
            });
            let path = CurvePath2::try_new_with_policy(curves.into(), &policy)
                .unwrap()
                .value;
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                let request = CurveFillet2::new(q(1, 128));
                let unconstrained =
                    path.fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy);
                assert!(
                    matches!(
                        unconstrained,
                        Err(ExactCurveError::Invalid {
                            cause: CurveError::FilletConstraintRequired,
                            ..
                        })
                    ),
                    "radius-only family error={:?}",
                    unconstrained.as_ref().err()
                );
                let mut requests = Vec::new();
                let mut centered = request.clone();
                centered.center = Some(center.clone());
                requests.push(centered);
                for axis in 0..2 {
                    for contact in [
                        CurveFilletContact2::Parameter(parameter.clone()),
                        CurveFilletContact2::Point(
                            contacts[if reversed { 1 - axis } else { axis }].clone(),
                        ),
                    ] {
                        let mut selected = request.clone();
                        selected.contacts[axis] = Some(contact);
                        requests.push(selected);
                    }
                }
                for selected in requests {
                    let outcome = path
                        .fillet_vertex(1, &selected, CurveCornerMode2::TrimOnly, &policy)
                        .unwrap();
                    assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(outcome.value.candidate_count(), 1);
                    let edited = &outcome.value.solutions()[0];
                    for (actual, expected) in [
                        (edited.curves()[0].end(), &contacts[usize::from(reversed)]),
                        (
                            edited.curves().last().unwrap().start(),
                            &contacts[usize::from(!reversed)],
                        ),
                    ] {
                        let same = actual.coincides_with(expected, &policy);
                        assert_eq!(same.certainty, crate::CurveCertainty::Certified);
                        assert_eq!(same.value, Classification::Decided(true));
                    }
                    for inserted in &edited.curves()[1..edited.curves().len() - 1] {
                        let mut preparation =
                            crate::bezier_region::CornerCarrierPreparation2::from_curve(
                                inserted, true,
                            );
                        preparation
                            .prepare(CurveOperation2::Fillet, &policy)
                            .unwrap();
                        let (actual_center, radius_squared, clockwise) = match preparation
                            .exact_carrier(true, CurveOperation2::Fillet, &policy)
                            .unwrap()
                        {
                            ExactCornerCarrier2::Arc(circle) => (
                                CurvePoint2::from(circle.center().clone()),
                                circle.radius_squared(),
                                circle.is_clockwise(),
                            ),
                            ExactCornerCarrier2::RetainedRationalArc(circle) => {
                                let circle = circle.support();
                                (
                                    CurvePoint2::from(circle.center().clone()),
                                    circle.radius_squared(),
                                    circle.is_clockwise(),
                                )
                            }
                            ExactCornerCarrier2::AlgebraicCusp(fragment) => {
                                let circle = fragment.semicircle();
                                let actual_center =
                                    match circle.center_point_evidence(&policy).unwrap() {
                                        Classification::Decided(center) => center,
                                        Classification::Uncertain(reason) => {
                                            panic!("circle center must replay: {reason:?}")
                                        }
                                    };
                                (
                                    actual_center,
                                    circle.radial_distance() * circle.radial_distance(),
                                    circle.is_clockwise() ^ fragment.is_reversed(),
                                )
                            }
                            _ => panic!("the independently known fillet must retain its circle"),
                        };
                        assert_eq!(
                            crate::classify::real_sign(&(radius_squared - q(1, 16384)), &policy),
                            Some(RealSign::Zero)
                        );
                        let center_match = actual_center.coincides_with(&center, &policy);
                        assert_eq!(center_match.certainty, crate::CurveCertainty::Certified);
                        assert_eq!(center_match.value, Classification::Decided(true));
                        assert_eq!(clockwise, !reversed);
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod algebraic_bridge_fillet_regression {
    use super::*;
    use crate::bezier_split::{BezierSelectedFiberFragment2, BezierSelectedFiberSource2};

    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    #[test]
    fn fillet_extension_owns_the_algebraic_endpoint_bridge() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(polynomial) =
                crate::BezierParameterPolynomial::try_new_power_basis(
                    vec![-Real::one(), Real::zero(), Real::from(2)],
                    &policy,
                )
                .unwrap()
            else {
                panic!("the endpoint polynomial must be exact");
            };
            let Classification::Decided(mut roots) =
                polynomial.isolate_unit_interval_roots(&policy).unwrap()
            else {
                panic!("the positive endpoint root must be isolated");
            };
            assert_eq!(roots.len(), 1);
            let endpoint: CurveParameter2 = roots.pop().unwrap().into();
            assert!(matches!(
                endpoint.as_bezier_parameter(),
                Some(BezierParameter2::Algebraic(_))
            ));
            let parallel = QuadraticBezier2::new(
                Point2::from_values(0, 0),
                Point2::new(q(1, 2), Real::zero()),
                Point2::from_values(1, 1),
            )
            .parallel_left(Real::zero())
            .unwrap();
            let Classification::Decided(incident) = parallel
                .incident_domain_from_parameter(
                    &endpoint,
                    crate::BezierParameterRayDirection2::Increasing,
                    &policy,
                )
                .unwrap()
            else {
                panic!("the parabola has an unbounded regular continuation");
            };
            assert_eq!(
                crate::classify::real_sign(&(incident.anchor() - q(23, 32)), &policy),
                Some(RealSign::Positive),
                "the independently chosen contact must lie before the chart anchor"
            );
            assert_eq!(
                incident
                    .contains_extension_parameter(&q(23, 32).into(), &policy)
                    .unwrap(),
                Classification::Decided(true)
            );
            let join = Point2::new(q(1, 2).sqrt().unwrap(), q(1, 2));
            let previous: Curve2 = BezierSelectedFiberFragment2::new(
                BezierSelectedFiberSource2::AnalyticParallel(parallel),
                CurveParameterRange2::new_validated(Real::zero().into(), endpoint),
                Point2::from_values(0, 0).into(),
                join.clone().into(),
            )
            .into();
            let next = LineSeg2::try_new(join, Point2::new(-Real::one(), q(1, 2)))
                .unwrap()
                .into();
            let path = CurvePath2::try_new_with_policy(vec![previous, next], &policy)
                .unwrap()
                .value;
            // At P(23/32) the right normal is (23,-16)/sqrt(785).
            // Equating its circle-center height with the line's y=1/2+r
            // gives this exact, strictly positive radius.
            let root = Real::from(785).sqrt().unwrap();
            let radius =
                (Real::from(17) * &root / (Real::from(1024) * (&root + Real::from(16)))).unwrap();
            let center = Point2::new(
                q(23, 32) + (Real::from(23) * &radius / &root).unwrap(),
                q(1, 2) + &radius,
            );
            let contacts = [
                CurvePoint2::from(Point2::new(q(23, 32), q(529, 1024))),
                CurvePoint2::from(Point2::new(center.x().clone(), q(1, 2))),
            ];
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                let mut request = CurveFillet2::new(radius.clone());
                request.center = Some(center.clone().into());
                let result = match path.fillet_vertex(
                    1,
                    &request,
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                ) {
                    Ok(result) => result,
                    Err(ExactCurveError::Blocked(blocker)) => {
                        panic!("known bridge fillet blocked: {:?}", blocker.reason())
                    }
                    Err(_) => panic!("known bridge fillet rejected"),
                };
                assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                assert_eq!(
                    result.value.candidate_count(),
                    1,
                    "the bridge owns one independently known exact center"
                );
                let solution = result.value.into_solutions().pop().unwrap();
                for expected in &contacts {
                    assert!(
                        solution.curves().windows(2).any(|pair| {
                            pair[0].end().coincides_with(expected, &policy).value
                                == Classification::Decided(true)
                                && pair[1].start().coincides_with(expected, &policy).value
                                    == Classification::Decided(true)
                        }),
                        "each independently known contact must be a retained path junction"
                    );
                }
            }
        }
    }
}

#[cfg(test)]
mod stationary_family_composition_regression {
    use super::*;
    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }
    fn joined_parallel_path(policy: &CurveContext) -> CurvePath2 {
        // P(u)=(3u²/8,9u⁴/64). These reversed offsets join exactly at
        // (0,41/64), while radius 1/128 gives a continuous center family.
        let source = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                Point2::new(q(1, 16), Real::zero()),
                Point2::new(q(3, 16), Real::zero()),
                Point2::new(q(3, 8), q(9, 64)),
            ],
            vec![Real::one(); 5],
        )
        .unwrap()
        .parallel_left(Real::zero())
        .unwrap();
        let curves = [(q(41, 64), true), (q(5, 8), false)].map(|(distance, previous)| {
            let parallel = source.with_distance(distance);
            let point = |t: &CurveParameter2| {
                if matches!(t.as_bezier_parameter(), Some(BezierParameter2::Exact(value)) if value == &Real::zero()) {
                    return CurvePoint2::from(Point2::new(Real::zero(), parallel.distance().clone()));
                }
                analytic_parallel_point_evidence(
                    &parallel,
                    t,
                    CurveOperation2::Fillet,
                    CurveFamily2::AnalyticParallel,
                    policy,
                )
                .unwrap()
            };
            let range = if previous {
                CurveParameterRange2::new_validated(
                    Real::zero().into(),
                    (q(5, 9) + q(1, 10000)).sqrt().unwrap().into(),
                )
            } else {
                CurveParameterRange2::new_validated(
                    (q(5, 9) - q(1, 10000)).sqrt().unwrap().into(),
                    Real::one().into(),
                )
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

    #[test]
    fn normalized_region_selects_and_reuses_a_stationary_fillet_family() {
        let parameter = CurveParameter2::from(q(5, 9).sqrt().unwrap());
        let join = CurvePoint2::from(Point2::new(Real::zero(), q(41, 64)));
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let path = joined_parallel_path(&policy);
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
            for (loop_index, boundary) in source.boundary_loops().iter().enumerate() {
                for (vertex, curve) in boundary.curves().iter().enumerate() {
                    if curve.start().coincides_with(&join, &policy).value
                        != Classification::Decided(true)
                    {
                        continue;
                    }
                    corners += 1;
                    let mut request = CurveFillet2::new(q(1, 128));
                    assert!(matches!(
                        source.fillet_loop_vertex(
                            loop_index,
                            vertex,
                            &request,
                            CurveCornerMode2::TrimOnly,
                            &policy
                        ),
                        Err(ExactCurveError::Invalid {
                            cause: CurveError::FilletConstraintRequired,
                            ..
                        })
                    ));
                    request.contacts[0] = Some(CurveFilletContact2::Parameter(parameter.clone()));
                    let selected = source
                        .fillet_loop_vertex(
                            loop_index,
                            vertex,
                            &request,
                            CurveCornerMode2::TrimOnly,
                            &policy,
                        )
                        .unwrap();
                    assert_eq!(selected.certainty, crate::CurveCertainty::Certified);
                    for region in selected.value.into_solutions() {
                        assert!(!region.is_empty());
                        let normalized = region.regularized_region(&policy).unwrap();
                        assert_eq!(normalized.certainty, crate::CurveCertainty::Certified);
                        let boolean = region.boolean_regions(&normalized.value, &policy).unwrap();
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
                            crate::CurveRegion2::try_from_boundary_paths(&[clip], &policy).unwrap();
                        assert_eq!(clip.certainty, crate::CurveCertainty::Certified);
                        let clipped = offset.value.boolean_regions(&clip.value, &policy).unwrap();
                        assert_eq!(clipped.certainty, crate::CurveCertainty::Certified);
                        assert!(!clipped.value.intersection().is_empty());
                        assert!(!clipped.value.difference().is_empty());
                        edited.push(region);
                    }
                }
            }
            assert_eq!(
                edited.len(),
                1,
                "one normalized boundary owns the selected fillet; corners={corners}"
            );
        }
    }
}

#[cfg(test)]
mod stationary_retained_point_constraint_regression {
    use super::*;

    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("fixture must be certified: {reason:?}"),
        }
    }

    fn retained_point(point: &Point2, policy: &CurveContext) -> CurvePoint2 {
        // Q(s) = point + (2s² - 1) * (1, 2), selected at 2s² - 1 = 0.
        // This independent curve retains the same exact point without storing
        // its coordinates or borrowing the queried parallel's source identity.
        let origin = Point2::new(point.x() - Real::one(), point.y() - Real::from(2));
        let end = Point2::new(point.x() + Real::one(), point.y() + Real::from(2));
        let source = Curve2::from(QuadraticBezier2::new(origin.clone(), origin, end));
        let polynomial = decided(
            crate::BezierParameterPolynomial::try_new_power_basis(
                vec![-Real::one(), Real::zero(), Real::from(2)],
                policy,
            )
            .unwrap(),
        );
        let interval =
            decided(crate::BezierParameterInterval::try_new(q(1, 2), Real::one(), policy).unwrap());
        let parameter = CurveParameter2::from(BezierParameter2::Algebraic(decided(
            crate::BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap(),
        )));
        let result = source.point_at(&parameter, policy).unwrap();
        assert_eq!(result.certainty, crate::CurveCertainty::Certified);
        assert!(result.value.coordinates().is_none());
        let same = result
            .value
            .coincides_with(&CurvePoint2::from(point.clone()), policy);
        assert_eq!(same.certainty, crate::CurveCertainty::Certified);
        assert_eq!(same.value, Classification::Decided(true));
        result.value
    }

    #[test]
    fn stationary_fillet_family_accepts_independent_retained_point_constraints() {
        let base = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                Point2::new(q(1, 16), Real::zero()),
                Point2::new(q(3, 16), Real::zero()),
                Point2::new(q(3, 8), q(9, 64)),
            ],
            vec![Real::one(); 5],
        )
        .unwrap()
        .parallel_left(Real::zero())
        .unwrap();
        let center = Point2::new(q(-175, 4992), q(4699, 7488));
        let contacts = [
            Point2::new(q(-95, 2496), q(4753, 7488)),
            Point2::new(q(-5, 156), q(4645, 7488)),
        ];
        let mut failures = Vec::new();
        for (policy_index, policy) in [CurveContext::STRICT, CurveContext::APPROXIMATE_512]
            .into_iter()
            .enumerate()
        {
            let curves = [q(41, 64), q(5, 8)].map(|distance| {
                let start = CurvePoint2::from(Point2::new(Real::zero(), distance.clone()));
                let end = CurvePoint2::from(Point2::new(
                    q(3, 8) - q(3, 5) * &distance,
                    q(9, 64) + q(4, 5) * &distance,
                ));
                Curve2::from(
                    crate::bezier_split::BezierSelectedFiberFragment2::new(
                        crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(
                            base.with_distance(distance),
                        ),
                        CurveParameterRange2::unit(),
                        start,
                        end,
                    )
                    .reversed(),
                )
            });
            let path = CurvePath2::try_new_with_policy(curves.into(), &policy)
                .unwrap()
                .value;
            let retained_center = retained_point(&center, &policy);
            let retained_contacts = contacts
                .each_ref()
                .map(|point| retained_point(point, &policy));
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                for selection in 0..3 {
                    let mut request = CurveFillet2::new(q(1, 128));
                    if selection == 0 {
                        request.center = Some(retained_center.clone());
                    } else {
                        let axis = selection - 1;
                        request.contacts[axis] = Some(CurveFilletContact2::Point(
                            retained_contacts[if reversed { 1 - axis } else { axis }].clone(),
                        ));
                    }
                    let result = match path.fillet_vertex(
                        1,
                        &request,
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    ) {
                        Ok(result) => result,
                        Err(error) => {
                            failures.push((
                                policy_index,
                                reversed,
                                selection,
                                format!("{error:?}"),
                            ));
                            continue;
                        }
                    };
                    assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(result.value.candidate_count(), 1);
                    let result = &result.value.solutions()[0];
                    for (actual, expected) in [
                        (result.curves()[0].end(), &contacts[usize::from(reversed)]),
                        (
                            result.curves().last().unwrap().start(),
                            &contacts[usize::from(!reversed)],
                        ),
                    ] {
                        let same =
                            actual.coincides_with(&CurvePoint2::from(expected.clone()), &policy);
                        assert_eq!(same.certainty, crate::CurveCertainty::Certified);
                        assert_eq!(same.value, Classification::Decided(true));
                    }
                }
            }
        }
        assert!(
            failures.is_empty(),
            "retained exact point constraints must select the same fillets: {failures:?}"
        );
    }
}
#[cfg(test)]
mod stationary_recursive_point_constraint_regression {
    use super::*;

    fn q(n: i64, d: i64) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    fn decided<T>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("fixture must be certified: {reason:?}"),
        }
    }

    fn retained_point(point: &Point2, policy: &CurveContext) -> CurvePoint2 {
        // A chord with direction (3,4) has left unit normal (-4/5,3/5).
        // Its independently authored first endpoint plus that normal is
        // exactly the requested point, retained as a procedural displacement.
        let origin = Point2::new(point.x() + q(4, 5), point.y() - q(3, 5));
        let end = Point2::new(origin.x() + Real::from(3), origin.y() + Real::from(4));
        let source = decided(
            crate::BezierAlgebraicChord2::try_new(origin.into(), end.into(), policy).unwrap(),
        );
        let (point_image, _) = crate::BezierAlgebraicChordParallelPoint2::new_pair(
            source,
            Real::one(),
            Real::zero(),
            Real::zero(),
            policy,
        );
        let result = CurvePoint2::from(point_image);
        assert!(result.coordinates().is_none());
        let same = result.coincides_with(&CurvePoint2::from(point.clone()), policy);
        assert_eq!(same.certainty, crate::CurveCertainty::Certified);
        assert_eq!(same.value, Classification::Decided(true));
        result
    }

    #[test]
    fn stationary_fillet_family_accepts_independent_recursive_point_constraints() {
        let base = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(0, 0),
                Point2::new(q(1, 16), Real::zero()),
                Point2::new(q(3, 16), Real::zero()),
                Point2::new(q(3, 8), q(9, 64)),
            ],
            vec![Real::one(); 5],
        )
        .unwrap()
        .parallel_left(Real::zero())
        .unwrap();
        let center = Point2::new(q(-175, 4992), q(4699, 7488));
        let contacts = [
            Point2::new(q(-95, 2496), q(4753, 7488)),
            Point2::new(q(-5, 156), q(4645, 7488)),
        ];
        let mut failures = Vec::new();
        for (policy_index, policy) in [CurveContext::STRICT, CurveContext::APPROXIMATE_512]
            .into_iter()
            .enumerate()
        {
            let curves = [q(41, 64), q(5, 8)].map(|distance| {
                let start = CurvePoint2::from(Point2::new(Real::zero(), distance.clone()));
                let end = CurvePoint2::from(Point2::new(
                    q(3, 8) - q(3, 5) * &distance,
                    q(9, 64) + q(4, 5) * &distance,
                ));
                Curve2::from(
                    crate::bezier_split::BezierSelectedFiberFragment2::new(
                        crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(
                            base.with_distance(distance),
                        ),
                        CurveParameterRange2::unit(),
                        start,
                        end,
                    )
                    .reversed(),
                )
            });
            let path = CurvePath2::try_new_with_policy(curves.into(), &policy)
                .unwrap()
                .value;
            let retained_center = retained_point(&center, &policy);
            let retained_contacts = contacts
                .each_ref()
                .map(|point| retained_point(point, &policy));
            for reversed in [false, true] {
                let path = if reversed {
                    path.reversed(&policy).unwrap().value
                } else {
                    path.clone()
                };
                for selection in 0..3 {
                    let mut request = CurveFillet2::new(q(1, 128));
                    if selection == 0 {
                        request.center = Some(retained_center.clone());
                    } else {
                        let axis = selection - 1;
                        request.contacts[axis] = Some(CurveFilletContact2::Point(
                            retained_contacts[if reversed { 1 - axis } else { axis }].clone(),
                        ));
                    }
                    let result = match path.fillet_vertex(
                        1,
                        &request,
                        CurveCornerMode2::TrimOnly,
                        &policy,
                    ) {
                        Ok(result) => result,
                        Err(error) => {
                            failures.push((
                                policy_index,
                                reversed,
                                selection,
                                format!("{error:?}"),
                            ));
                            continue;
                        }
                    };
                    assert_eq!(result.certainty, crate::CurveCertainty::Certified);
                    assert_eq!(result.value.candidate_count(), 1);
                    let result = &result.value.solutions()[0];
                    for (actual, expected) in [
                        (result.curves()[0].end(), &contacts[usize::from(reversed)]),
                        (
                            result.curves().last().unwrap().start(),
                            &contacts[usize::from(!reversed)],
                        ),
                    ] {
                        let same =
                            actual.coincides_with(&CurvePoint2::from(expected.clone()), &policy);
                        assert_eq!(same.certainty, crate::CurveCertainty::Certified);
                        assert_eq!(same.value, Classification::Decided(true));
                    }
                }
            }
        }
        assert!(
            failures.is_empty(),
            "retained exact point constraints must select the same fillets: {failures:?}"
        );
    }
}
