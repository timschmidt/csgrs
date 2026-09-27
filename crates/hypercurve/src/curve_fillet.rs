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

#[derive(Clone, Debug)]
pub(crate) struct FilletCandidates2<'a> {
    isolated: CurveCornerSolutions2<FilletCorner2>,
    families: Vec<FilletCornerFamily2<'a>>,
}

impl<'a> FilletCandidates2<'a> {
    pub(super) fn empty(reason: CurveCornerNoSolution2) -> Self {
        Self::from_isolated(CurveCornerSolutions2::NoSolution(reason), Vec::new())
    }

    pub(super) fn from_isolated(
        isolated: CurveCornerSolutions2<FilletCorner2>,
        families: Vec<FilletCornerFamily2<'a>>,
    ) -> Self {
        Self { isolated, families }
    }

    pub(crate) fn require_finite(self) -> ExactCurveResult<CurveCornerSolutions2<FilletCorner2>> {
        if let Some(family) = self.families.first() {
            return Err(constraint_required(family.families[0]));
        }
        Ok(self.isolated)
    }

    pub(crate) fn resolve(
        self,
        binding: &FilletConstraintBinding2<'_>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveCornerSolutions2<FilletCorner2>> {
        if !binding.request.has_constraints() {
            return self.require_finite();
        }
        let empty = if self.families.is_empty() {
            self.isolated
                .no_solution_reason()
                .unwrap_or(CurveCornerNoSolution2::UnsatisfiedConstraints)
        } else {
            CurveCornerNoSolution2::UnsatisfiedConstraints
        };
        let mut candidates = CornerSolutionAccumulator::Empty;
        for corner in self.isolated.into_solutions() {
            if binding.matches(&corner, policy)? {
                candidates.push(corner);
            }
        }
        for family in self.families {
            let mut prepared = None;
            for component in &family.components {
                if let (Some(requested), Some(image)) =
                    (&binding.request.center, component.point_image())
                    && !fillet_point_matches(requested, image, family.families[0], policy)?
                {
                    continue;
                }
                if prepared.is_none() {
                    prepared = Some(family.contact_constraints(binding, policy)?);
                }
                let constraints = prepared
                    .as_ref()
                    .expect("constraints for this center support");
                for first in constraints[0].alternatives() {
                    for second in constraints[1].alternatives() {
                        if let Some(corner) = family.select(component, [first, second], policy)?
                            && binding.matches(&corner, policy)?
                        {
                            candidates.push(corner);
                        }
                    }
                }
            }
        }
        Ok(candidates.finish(empty))
    }

    #[cfg(test)]
    pub(crate) fn solutions(&self) -> &[FilletCorner2] {
        self.isolated.solutions()
    }
    #[cfg(test)]
    pub(crate) fn families(&self) -> &[FilletCornerFamily2<'a>] {
        &self.families
    }
    #[cfg(test)]
    pub(crate) fn into_parts(self) -> (Vec<FilletCorner2>, Vec<FilletCornerFamily2<'a>>) {
        (self.isolated.into_solutions(), self.families)
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

    fn matches(&self, corner: &FilletCorner2, policy: &CurveContext) -> ExactCurveResult<bool> {
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

/// Resolves a collapsed center while its original source and chart still live.
/// The fixed center does not select any contact on the collapsed circle.
pub(super) fn constrained_collapsed_fillet(
    offsets: [&FilletOffsetCarrier2<'_, '_>; 2],
    clockwise: bool,
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
    if !point_on_fillet_offset(
        center,
        offsets[other_axis],
        other_axis == 0,
        domains[other_axis],
        families[other_axis],
        policy,
    )? {
        return empty(CurveCornerNoSolution2::NoTangentCircle);
    }
    let mut cuts = [None, None];
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
                    ExactCornerArc2::Native(_) => {
                        let point = point.coordinates().ok_or_else(|| {
                            ExactCurveError::blocked(
                                CurveOperation2::Fillet,
                                families[axis],
                                crate::UncertaintyReason::Unsupported,
                            )
                        })?;
                        arc_fillet_cut_from_incident_point(
                            source,
                            point.clone(),
                            false,
                            axis == 0,
                            domains[axis],
                            families[axis],
                            policy,
                        )?
                    }
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
                let incident = (domains[axis].mode() == CurveCornerMode2::TrimOrExtend)
                    .then(|| source.incident_domain(support, axis == 0, families[axis], policy))
                    .transpose()?;
                let mut admitted = false;
                let mut failure = None;
                let visited = support
                    .visit_point_incidence_evidence(
                        center,
                        &source.curve_parameter_range(),
                        incident.as_ref(),
                        policy,
                        &mut |parameter| {
                            let placement = parameter
                                .map(|parameter| {
                                    source.parameter_placement(
                                        parameter,
                                        axis == 0,
                                        domains[axis],
                                        families[axis],
                                        policy,
                                    )
                                })
                                .transpose();
                            match placement {
                                Ok(Some(None)) => std::ops::ControlFlow::Continue(()),
                                Ok(_) => {
                                    admitted = true;
                                    std::ops::ControlFlow::Break(())
                                }
                                Err(error) => {
                                    failure = Some(error);
                                    std::ops::ControlFlow::Break(())
                                }
                            }
                        },
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Fillet, families[axis], cause)
                    })?;
                if let Some(error) = failure {
                    return Err(error);
                }
                match visited {
                    Classification::Decided(_) if !admitted => {
                        return empty(CurveCornerNoSolution2::OutsideTrimDomain);
                    }
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Fillet,
                            families[axis],
                            reason,
                        ));
                    }
                    Classification::Decided(_) => (),
                }
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Fillet,
                    families[axis],
                    crate::UncertaintyReason::Unsupported,
                ));
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
        cuts[axis] = Some(cut);
    }
    let [Some(previous), Some(next)] = cuts else {
        return Err(constraint_required(families[0]));
    };
    let witness = FilletCenterWitness2 {
        point: center.clone(),
        previous_parameter: previous.parameter.clone(),
        next_parameter: next.parameter.clone(),
        retained_anchor_evidence: None,
    };
    match fillet_corner_from_cuts(
        offsets,
        &witness,
        [previous, next],
        clockwise,
        families,
        policy,
    )? {
        FilletCornerSelection2::Selected(corner) => Ok(CurveCornerSolutions2::Unique(corner)),
        FilletCornerSelection2::Outside => empty(CurveCornerNoSolution2::OutsideTrimDomain),
        FilletCornerSelection2::Degenerate => empty(CurveCornerNoSolution2::DegenerateCandidate),
    }
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

#[derive(Clone, Debug)]
pub(crate) struct FilletCornerFamily2<'a> {
    components: Vec<crate::bezier_offset::CurveParameterComponent2>,
    sources: [FilletParallelSource2<'a>; 2],
    centers: [BezierParallel2; 2],
    clockwise: bool,
    retain_selected_circle_endpoints: bool,
    domains: [FilletContactDomain2; 2],
    families: [CurveFamily2; 2],
}

impl<'a> FilletCornerFamily2<'a> {
    pub(super) fn retain(
        components: Vec<crate::bezier_offset::CurveParameterComponent2>,
        offsets: [&FilletOffsetCarrier2<'a, '_>; 2],
        clockwise: bool,
        retain_selected_circle_endpoints: bool,
        domains: [FilletContactDomain2; 2],
        families: [CurveFamily2; 2],
    ) -> ExactCurveResult<Option<Self>> {
        if components.is_empty() {
            return Ok(None);
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
        let sources = [*first, *second];
        Ok(Some(Self {
            components,
            sources,
            centers: [first_center.clone(), second_center.clone()],
            clockwise,
            retain_selected_circle_endpoints,
            domains,
            families,
        }))
    }

    fn contact_constraints(
        &self,
        binding: &FilletConstraintBinding2<'_>,
        policy: &CurveContext,
    ) -> ExactCurveResult<[FilletContactSelection2; 2]> {
        let data = self;
        let mut constraints = [FilletContactSelection2::Any, FilletContactSelection2::Any];
        for (axis, constraint) in constraints.iter_mut().enumerate() {
            let source = data.sources[axis];
            let family = data.families[axis];
            *constraint = match &binding.request.contacts[axis] {
                Some(CurveFilletContact2::Parameter(parameter)) => {
                    let parameter = binding.source_parameter(axis, parameter, policy)?;
                    let parameter = match source {
                        FilletParallelSource2::Direct(source) => {
                            let (start, end) = source.parameter_range();
                            inverse_fillet_parameter(
                                &parameter,
                                &(end - start),
                                start,
                                family,
                                policy,
                            )?
                        }
                        _ => parameter,
                    };
                    FilletContactSelection2::Parameters(vec![parameter])
                }
                Some(CurveFilletContact2::Point(point)) => {
                    let support = match source {
                        FilletParallelSource2::Direct(_) => {
                            data.centers[axis].with_distance(Real::zero())
                        }
                        FilletParallelSource2::Retained(source) => source.parallel().clone(),
                        FilletParallelSource2::Selected(source) => source.parallel_carrier(),
                    };
                    self.point_constraints(axis, &support, point, policy)?
                }
                None => FilletContactSelection2::Any,
            };
        }
        if binding.request.contacts.iter().all(Option::is_none)
            && let Some(center) = &binding.request.center
        {
            // A correspondence transports one selected contact to the other.
            // A constant center leaves both contact axes free and still needs
            // contact constraints; it cannot select an arbitrary representative.
            constraints[0] = self.point_constraints(0, &data.centers[0], center, policy)?;
            if matches!(constraints[0], FilletContactSelection2::Any) {
                constraints[1] = self.point_constraints(1, &data.centers[1], center, policy)?;
            }
        }
        Ok(constraints)
    }

    fn point_constraints(
        &self,
        axis: usize,
        support: &BezierParallel2,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<FilletContactSelection2> {
        let data = self;
        let family = data.families[axis];
        let source = data.sources[axis];
        let incident = (data.domains[axis].mode() == CurveCornerMode2::TrimOrExtend)
            .then(|| source.incident_domain(&data.centers[axis], axis == 0, family, policy))
            .transpose()?;
        let mut all = false;
        let mut parameters = Vec::new();
        let visited = support
            .visit_point_incidence_evidence(
                point,
                &source.curve_parameter_range(),
                incident.as_ref(),
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
                match previous.same_value(&parameter, policy).map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Fillet, family, cause)
                })? {
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
            source: data.sources[axis],
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
