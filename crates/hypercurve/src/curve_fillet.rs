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
        let center = FilletCenterWitness2 {
            point: center,
            previous_parameter: None,
            next_parameter: None,
            retained_anchor_evidence: None,
        };
        return match fillet_corner_from_center(
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
            FilletCornerSelection2::Degenerate => {
                empty(CurveCornerNoSolution2::DegenerateCandidate)
            }
        };
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
                let PreparedFilletCarrier2::Arc { source, .. } = source else {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Fillet,
                        families[axis],
                        crate::UncertaintyReason::Unsupported,
                    ));
                };
                match crate::bezier_offset::retained_point_circle_incidence_sign(
                    &point,
                    source.support().center(),
                    source.support().radius_squared_ref(),
                    policy,
                )
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, families[axis], cause)
                })? {
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
                false,
                axis == 0,
                retain_selected_circle_endpoints,
                domains[axis],
                families[axis],
                policy,
            )?,
            FilletOffsetCarrier2::AlgebraicCusp { support, .. } => {
                // The concentric offset retains the source's angular range.
                // Incidence was certified above; strict chord-side exclusion
                // proves there is no authored contact even if the other axis
                // is still free. No inverse angular parameter is needed.
                if domains[axis] == FilletContactDomain2::AuthoredCurve(CurveCornerMode2::TrimOnly)
                    && matches!(
                        support
                            .certified_incident_point_evidence_is_strict_interior(center, policy)
                            .map_err(|cause| ExactCurveError::invalid(
                                CurveOperation2::Fillet,
                                families[axis],
                                cause
                            ))?,
                        Classification::Decided(false)
                    )
                {
                    return empty(CurveCornerNoSolution2::OutsideTrimDomain);
                }
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    families[axis],
                    crate::UncertaintyReason::Unsupported,
                ));
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
    family: CurveFamily2,
    policy: &CurveContext,
) -> ExactCurveResult<FilletContactSelection2> {
    let mut all = false;
    let mut parameters = Vec::new();
    let visited = support
        .visit_point_incidence_evidence(point, range, incident, policy, &mut |parameter| {
            if let Some(parameter) = parameter {
                parameters.push(parameter.clone());
            } else {
                all = true;
            }
            std::ops::ControlFlow::Continue(())
        })
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
        let mut prepared = None;
        for component in components {
            if let (Some(requested), Some(image)) =
                (&binding.request.center, component.point_image())
                && !fillet_point_matches(requested, image, families[0], policy)?
            {
                continue;
            }
            if prepared.is_none() {
                prepared = Some(replay.contact_constraints(binding, policy)?);
            }
            let constraints = prepared
                .as_ref()
                .expect("constraints for this center support");
            for first in constraints[0].alternatives() {
                for second in constraints[1].alternatives() {
                    if let Some(corner) = replay.select(component, [first, second], policy)?
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
        policy: &CurveContext,
    ) -> ExactCurveResult<[FilletContactSelection2; 2]> {
        let data = self;
        let mut constraints = [FilletContactSelection2::Any, FilletContactSelection2::Any];
        for (axis, constraint) in constraints.iter_mut().enumerate() {
            let (source, support) = data.parallel(axis);
            *constraint = binding.parallel_contact_parameters(
                axis,
                source,
                support,
                data.domains[axis],
                data.families[axis],
                policy,
            )?;
        }
        if binding.request.contacts.iter().all(Option::is_none)
            && let Some(center) = &binding.request.center
        {
            // A correspondence transports one selected contact to the other.
            // A constant center leaves both contact axes free and still needs
            // contact constraints; it cannot select an arbitrary representative.
            constraints[0] = self.center_parameters(0, center, policy)?;
            if matches!(constraints[0], FilletContactSelection2::Any) {
                constraints[1] = self.center_parameters(1, center, policy)?;
            }
        }
        Ok(constraints)
    }

    fn center_parameters(
        &self,
        axis: usize,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<FilletContactSelection2> {
        let data = self;
        let family = data.families[axis];
        let (source, support) = data.parallel(axis);
        let incident = (data.domains[axis].mode() == CurveCornerMode2::TrimOrExtend)
            .then(|| source.incident_domain(support, axis == 0, family, policy))
            .transpose()?;
        fillet_point_parameters(
            support,
            point,
            &source.curve_parameter_range(),
            incident.as_ref(),
            family,
            policy,
        )
    }

    fn select(
        &self,
        component: &crate::bezier_offset::CurveParameterComponent2,
        constraints: [Option<&CurveParameter2>; 2],
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
        let [previous, next] = &selected;
        let point = match component.point_image() {
            Some(point) => point.clone(),
            None => analytic_parallel_point_evidence(
                data.parallel(0).1,
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
                            let region = |path: &CurvePath2| {
                                let mut curves = path.curves().to_vec();
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
                                crate::CurveRegion2::try_from_boundary_paths(
                                    &[CurvePath2::try_new(curves).unwrap()],
                                    &policy,
                                )
                                .unwrap()
                                .value
                            };
                            let difference = region(edited)
                                .boolean_regions(&region(&witness), &policy)
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
                assert!(matches!(
                    source.fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy),
                    Err(ExactCurveError::Invalid {
                        cause: CurveError::FilletConstraintRequired,
                        ..
                    })
                ));
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
