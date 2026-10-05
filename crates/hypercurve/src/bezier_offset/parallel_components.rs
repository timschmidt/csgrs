//! Parallel-pair candidate systems, parameter components and implicit cells.

use super::*;

pub(super) fn parallel_intersection_candidate_system(
    equations: [BivariatePolynomial; 2],
    candidates: CurveIntersectionCandidates2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParallelIntersectionCandidateSystem2>> {
    if matches!(
        candidates,
        CurveIntersectionCandidates2::DegenerateResultant
    ) && let Some(reduced) = hypersolve::saturate_rootless_bivariate_axis_factors(
        &equations,
        [[&Real::zero(), &Real::one()]; 2],
    ) {
        let candidates = match project_parallel_intersection_system(
            &reduced[0],
            &reduced[1],
            [CurveParameterDomain2::new(&CurveParameterRange2::unit(), None); 2],
            policy,
        )? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        return Ok(Classification::Decided(parallel_candidate_system(
            candidates, reduced,
        )));
    }
    Ok(Classification::Decided(parallel_candidate_system(
        candidates, equations,
    )))
}

pub(super) fn parallel_candidate_system(
    candidates: CurveIntersectionCandidates2,
    equations: [BivariatePolynomial; 2],
) -> BezierParallelIntersectionCandidateSystem2 {
    let replay_equations =
        (!matches!(&candidates, CurveIntersectionCandidates2::NoIntersection)).then_some(equations);
    BezierParallelIntersectionCandidateSystem2::projected(candidates, replay_equations)
}

pub(super) fn project_parallel_intersection_system(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
    domains: [CurveParameterDomain2<'_>; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<CurveIntersectionCandidates2>> {
    // Exact saturation can leave (c,0). Its nonzero constant equation
    // proves the whole domain empty even though its resultant is zero.
    for equation in [first_equation, second_equation] {
        if let [row] = equation.coefficients.as_slice()
            && let [constant] = row.as_slice()
            && matches!(
                real_sign(constant, &policy.strict_counterpart()),
                Some(RealSign::Positive | RealSign::Negative)
            )
        {
            return Ok(Classification::Decided(
                CurveIntersectionCandidates2::NoIntersection,
            ));
        }
    }
    let config = CurveIntersectionResultantConfig {
        min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
        max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
    };
    let parallel_report = resultant_bivariate_polynomial_system_complete(
        first_equation,
        second_equation,
        CurveResultantParameter::First,
        config,
    );
    let parallel = match resultant_parameter_projection(parallel_report, domains[0], policy)? {
        Classification::Decided(projection) => projection,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    match parallel {
        ResultantParameterProjection::Empty => {
            // One empty projection proves the complete pair domain empty.
            // Avoid constructing or refining the other resultant.
            return Ok(Classification::Decided(
                CurveIntersectionCandidates2::NoIntersection,
            ));
        }
        ResultantParameterProjection::Degenerate => {
            return Ok(Classification::Decided(
                CurveIntersectionCandidates2::DegenerateResultant,
            ));
        }
        ResultantParameterProjection::Parameters(_)
        | ResultantParameterProjection::SelectedParameters(_) => {}
    }
    let other_report = resultant_bivariate_polynomial_system_complete(
        first_equation,
        second_equation,
        CurveResultantParameter::Second,
        config,
    );
    let other = match resultant_parameter_projection(other_report, domains[1], policy)? {
        Classification::Decided(projection) => projection,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(match (parallel, other) {
        (ResultantParameterProjection::Empty, _) | (_, ResultantParameterProjection::Empty) => {
            CurveIntersectionCandidates2::NoIntersection
        }
        (ResultantParameterProjection::Degenerate, _)
        | (_, ResultantParameterProjection::Degenerate) => {
            CurveIntersectionCandidates2::DegenerateResultant
        }
        (
            ResultantParameterProjection::Parameters(parallel_parameters)
            | ResultantParameterProjection::SelectedParameters(parallel_parameters),
            ResultantParameterProjection::Parameters(other_parameters)
            | ResultantParameterProjection::SelectedParameters(other_parameters),
        ) => CurveIntersectionCandidates2::Candidates {
            first_parameters: parallel_parameters,
            second_parameters: other_parameters,
        },
    }))
}

pub(super) fn bivariate_system_may_have_component(equations: &[BivariatePolynomial; 2]) -> bool {
    bivariate_pair_may_have_component(&equations[0], &equations[1])
}

pub(super) fn bivariate_pair_may_have_component(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
) -> bool {
    for retained_value in [Real::from(2_i8), Real::from(3_i8), Real::from(5_i8)] {
        let first = bivariate_specialize_first(first_equation, &retained_value);
        let second = bivariate_specialize_first(second_equation, &retained_value);
        // A modular gcd certifies coprimality cheaply. When it instead finds
        // a likely common factor, this conservative filter may answer "may
        // have a component" without the exact chain, whose rational
        // coefficients can grow far past the inputs; callers certify any
        // component independently.
        match hypersolve::univariate_polynomials_modular_coprimality(&first, &second) {
            hypersolve::ModularCoprimality::Coprime => return false,
            hypersolve::ModularCoprimality::CommonFactorLikely => continue,
            hypersolve::ModularCoprimality::Inconclusive => {}
        }
        let Ok(report) = subresultant_chain_univariate_polynomials(
            &first,
            &second,
            PARALLEL_INTERSECTION_RESULTANT_PRECISION,
        ) else {
            continue;
        };
        if !report.has_nonconstant_common_factor {
            return false;
        }
    }
    true
}

pub(super) struct ParameterComponentSystem2 {
    pub(super) overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
    pub(super) component_overlaps: Arc<[BezierParameterComponentOverlap2]>,
    pub(super) component_pairs: Arc<[BezierParallelIntersectionParameterPair2]>,
    pub(super) selected_component_pair_count: usize,
    pub(super) residual_equations: [BivariatePolynomial; 2],
}

/// Exact cell selector for a positive-dimensional parameter component.
///
/// The ordinary analytic-parallel/rational system selects `branch > 0`.  A
/// pair of analytic parallels instead has a piecewise radical predicate: two
/// projection signs select the transverse case, while a tangent-parallel
/// point uses the norm and normal-side signs.  Keeping that distinction in one
/// small selector lets both systems reuse the same rational-map and implicit
/// cylindrical decomposition without flattening either predicate to samples.
pub(super) enum ParameterComponentSelector2<'a> {
    Positive(
        &'a BivariatePolynomial,
        Option<&'a [BezierParallelDerivativePolynomials2; 2]>,
    ),
    ParallelPair {
        system: &'a BezierParallelPairEquationSystem2,
        parameter_filter: Option<&'a BivariatePolynomial>,
        normal_constraints: Option<&'a [BezierParallelDerivativePolynomials2; 2]>,
    },
}

impl ParameterComponentSelector2<'_> {
    pub(super) fn normal_constraints(&self) -> Option<&[BezierParallelDerivativePolynomials2; 2]> {
        match self {
            Self::Positive(_, constraints) => *constraints,
            Self::ParallelPair {
                normal_constraints, ..
            } => *normal_constraints,
        }
    }

    pub(super) fn boundary_polynomials(&self) -> Vec<BivariatePolynomial> {
        let mut boundaries = match self {
            Self::Positive(branch, _) => vec![(*branch).clone()],
            Self::ParallelPair {
                system,
                parameter_filter,
                ..
            } => {
                let mut boundaries = vec![
                    system.weight_product.clone(),
                    system.tangent_cross.clone(),
                    system.first_projection.clone(),
                    system.second_projection.clone(),
                    system.tangent_dot.clone(),
                    system.norm_residual.clone(),
                    system.first_normal_projection.clone(),
                ];
                if let Some(filter) = parameter_filter {
                    boundaries.push((*filter).clone());
                }
                boundaries
            }
        };
        if let Some(constraints) = self.normal_constraints() {
            boundaries.extend(
                constraints
                    .iter()
                    .flat_map(|constraint| constraint.boundary_polynomials())
                    .cloned(),
            );
        }
        boundaries
    }

    pub(super) fn selected_at(
        &self,
        retained_parameter: CurveResultantParameter,
        retained: &BezierParameter2,
        lifted: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let (first, second) = match retained_parameter {
            CurveResultantParameter::First => (retained, lifted),
            CurveResultantParameter::Second => (lifted, retained),
        };
        if let Some(constraints) = self.normal_constraints() {
            for constraint in constraints {
                match constraint.selected_at(first, second, policy)? {
                    Classification::Decided(true) => {}
                    other => return Ok(other),
                }
            }
        }
        match self {
            Self::Positive(branch, _) => Ok(
                match signed_bivariate_at_parameter_pair(branch, first, second, policy)? {
                    Classification::Decided(RealSign::Positive) => Classification::Decided(true),
                    Classification::Decided(RealSign::Negative | RealSign::Zero) => {
                        Classification::Decided(false)
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                },
            ),
            Self::ParallelPair {
                system,
                parameter_filter,
                ..
            } => {
                if let Some(filter) = parameter_filter {
                    match signed_bivariate_at_parameter_pair(filter, first, second, policy)? {
                        Classification::Decided(RealSign::Positive) => {}
                        Classification::Decided(RealSign::Zero | RealSign::Negative) => {
                            return Ok(Classification::Decided(false));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                parallel_pair_component_selected_at(system, first, second, policy)
            }
        }
    }
}

impl ParameterComponentSystem2 {
    pub(super) fn from_partitioned_pairs(
        overlaps: Vec<RationalBezierIntersectionOverlap2>,
        component_overlaps: Vec<BezierParameterComponentOverlap2>,
        mut selected_pairs: Vec<BezierParallelIntersectionParameterPair2>,
        excluded_pairs: Vec<BezierParallelIntersectionParameterPair2>,
        residual_equations: [BivariatePolynomial; 2],
    ) -> Self {
        let selected_component_pair_count = selected_pairs.len();
        if selected_pairs.is_empty() {
            selected_pairs = excluded_pairs;
        } else {
            selected_pairs.extend(excluded_pairs);
        }
        Self {
            overlaps: overlaps.into(),
            component_overlaps: component_overlaps.into(),
            component_pairs: selected_pairs.into(),
            selected_component_pair_count,
            residual_equations,
        }
    }

    #[cfg(test)]
    pub(super) fn selected_pairs(&self) -> &[BezierParallelIntersectionParameterPair2] {
        &self.component_pairs[..self.selected_component_pair_count]
    }

    #[cfg(test)]
    pub(super) fn excluded_pairs(&self) -> &[BezierParallelIntersectionParameterPair2] {
        &self.component_pairs[self.selected_component_pair_count..]
    }
}

#[derive(Clone, Debug)]
pub(crate) struct BezierParameterComponentOverlap2 {
    pub(in crate::bezier_offset) overlap: RationalBezierIntersectionOverlap2,
    pub(in crate::bezier_offset) support: Arc<BivariatePolynomial>,
    pub(in crate::bezier_offset) fiber_root_ranks: [usize; 2],
    pub(in crate::bezier_offset) witness: BezierParallelIntersectionParameterPair2,
    pub(in crate::bezier_offset) parameter_charts: Option<Arc<ParameterComponentAffineCharts2>>,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct ParameterComponentAffineCharts2 {
    pub(super) to_source: [ParameterComponentAffineMap2; 2],
    pub(super) to_local: [ParameterComponentAffineMap2; 2],
    pub(super) overlap: RationalBezierIntersectionOverlap2,
}

pub(super) const UNKNOWN_PARAMETER_COMPONENT_FIBER_ROOT_RANK: usize = usize::MAX;

impl PartialEq for BezierParameterComponentOverlap2 {
    fn eq(&self, other: &Self) -> bool {
        self.overlap == other.overlap
            && self.support == other.support
            && self.fiber_root_ranks == other.fiber_root_ranks
            && self.witness == other.witness
            && self.parameter_charts == other.parameter_charts
    }
}

impl BezierParameterComponentOverlap2 {
    pub(crate) fn overlap(&self) -> &RationalBezierIntersectionOverlap2 {
        self.parameter_charts
            .as_ref()
            .map_or(&self.overlap, |charts| &charts.overlap)
    }

    /// Publishes the original parameters of a component certified in a finite
    /// affine chart. Its support, fiber ranks and witness remain in the chart
    /// where they were proved. Repeated transport composes two affine maps;
    /// it never nests correspondences or reconstructs the source curves.
    pub(super) fn in_parameter_charts(
        mut self,
        mut maps: [ParameterComponentAffineMap2; 2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if maps
            .iter()
            .all(|map| map.scale == Real::one() && map.offset == Real::zero())
        {
            return Ok(Classification::Decided(self));
        }
        if let Some(charts) = &self.parameter_charts {
            for (map, inner) in maps.iter_mut().zip(&charts.to_source) {
                map.offset = &map.scale * &inner.offset + &map.offset;
                map.scale = &map.scale * &inner.scale;
            }
        }
        for map in &maps {
            match real_sign(&map.scale, &policy.strict_counterpart()) {
                Some(RealSign::Positive) => {}
                Some(_) => return Err(CurveError::InvalidBezierRange),
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        let inverse = |map: &ParameterComponentAffineMap2| -> CurveResult<_> {
            let scale = (Real::one() / &map.scale)?;
            Ok(ParameterComponentAffineMap2 {
                offset: -(&map.offset * &scale),
                scale,
            })
        };
        let to_local = [inverse(&maps[0])?, inverse(&maps[1])?];
        let mut endpoints: [[Option<BezierParameter2>; 2]; 2] =
            std::array::from_fn(|_| std::array::from_fn(|_| None));
        for ((range, map), endpoints) in [self.overlap.first_range(), self.overlap.second_range()]
            .into_iter()
            .zip(&maps)
            .zip(&mut endpoints)
        {
            for (parameter, endpoint) in [range.start(), range.end()].into_iter().zip(endpoints) {
                match parameter.affine_image_unbounded(&map.scale, &map.offset, policy)? {
                    Classification::Decided(parameter) => *endpoint = Some(parameter),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        let [[first_start, first_end], [second_start, second_end]] = endpoints.map(|pair| {
            pair.map(|parameter| parameter.expect("every component endpoint was transported"))
        });
        self.parameter_charts = Some(Arc::new(ParameterComponentAffineCharts2 {
            to_source: maps,
            to_local,
            overlap: RationalBezierIntersectionOverlap2::from_certified_parameters(
                first_start,
                first_end,
                second_start,
                second_end,
                self.overlap.orientation(),
                [self.overlap.includes_start(), self.overlap.includes_end()],
            ),
        }));
        Ok(Classification::Decided(self))
    }

    /// Retains closed rectangle-boundary contacts when the selected component
    /// has no positive span in that rectangle. The monotone correspondence
    /// and endpoint inclusion already certify incidence and branch selection.
    /// Its exact parameter images need not be standalone polynomial roots.
    pub(super) fn closed_boundary_contacts(
        &self,
        first: &CurveParameterRange2,
        second: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierParallelPairIntersectionContact2>>> {
        let mut contacts = Vec::new();
        for (axis, range) in [
            (CurveResultantParameter::First, first),
            (CurveResultantParameter::Second, second),
        ] {
            for parameter in [range.start(), range.end()] {
                let mapped = match self.map_curve_parameter(axis, parameter, policy)? {
                    Classification::Decided(Some(mapped)) => mapped,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let (first_parameter, second_parameter) = match axis {
                    CurveResultantParameter::First => (parameter.clone(), mapped),
                    CurveResultantParameter::Second => (mapped, parameter.clone()),
                };
                let mut included = true;
                for (parameter, range) in [(&first_parameter, first), (&second_parameter, second)] {
                    match CurveParameterDomain2::new(range, None)
                        .contains_finite_parameter(parameter, policy)?
                    {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => {
                            included = false;
                            break;
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                if !included {
                    continue;
                }
                for (boundary, owns_boundary) in [
                    (
                        self.overlap().first_range().start(),
                        self.overlap().includes_start(),
                    ),
                    (
                        self.overlap().first_range().end(),
                        self.overlap().includes_end(),
                    ),
                ] {
                    if !owns_boundary {
                        match first_parameter.same_value(&boundary.clone().into(), policy)? {
                            Classification::Decided(true) => included = false,
                            Classification::Decided(false) => {}
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                }
                if !included {
                    continue;
                }
                match parallel_pair_contact_parameters_are_retained(
                    &contacts,
                    &first_parameter,
                    &second_parameter,
                    policy,
                )? {
                    Classification::Decided(true) => continue,
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                contacts.push(BezierParallelPairIntersectionContact2 {
                    first_parameter,
                    second_parameter,
                    certified_transverse: false,
                    tangent_cross_sign: None,
                    tangent_dot_sign: None,
                });
            }
        }
        Ok(Classification::Decided(contacts))
    }

    pub(crate) fn map_curve_parameter(
        &self,
        retained_parameter: CurveResultantParameter,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        let Some(charts) = &self.parameter_charts else {
            return self.map_local_curve_parameter(retained_parameter, parameter, policy);
        };
        let axis = match retained_parameter {
            CurveResultantParameter::First => 0,
            CurveResultantParameter::Second => 1,
        };
        let ranges = [charts.overlap.first_range(), charts.overlap.second_range()];
        for (retained, lifted) in [
            (ranges[axis].start(), ranges[1 - axis].start()),
            (ranges[axis].end(), ranges[1 - axis].end()),
        ] {
            if parameter.same_value(&retained.clone().into(), policy)?
                == Classification::Decided(true)
            {
                return Ok(Classification::Decided(Some(lifted.clone().into())));
            }
        }
        let map = &charts.to_local[axis];
        let parameter = match parameter.affine_image_unbounded(&map.scale, &map.offset, policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let parameter =
            match self.map_local_curve_parameter(retained_parameter, &parameter, policy)? {
                Classification::Decided(Some(parameter)) => parameter,
                other => return Ok(other),
            };
        let map = &charts.to_source[1 - axis];
        Ok(parameter
            .affine_image_unbounded(&map.scale, &map.offset, policy)?
            .map(Some))
    }

    pub(super) fn map_local_curve_parameter(
        &self,
        retained_parameter: CurveResultantParameter,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        let parameter = if let Some(parameter) = parameter.as_bezier_parameter() {
            parameter.clone()
        } else if let Some(parameter) = parameter.as_selected_fiber() {
            return policy.strict_predicate_pass(|| {
                self.map_selected_fiber_parameter(retained_parameter, parameter, policy)
            });
        } else if let Some(parameter) = parameter.as_recursive_projective() {
            return policy.strict_predicate_pass(|| {
                self.map_recursive_projective_parameter(retained_parameter, parameter, policy)
            });
        } else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(self
            .map_parameter(retained_parameter, &parameter, policy)?
            .map(|parameter| parameter.map(CurveParameter2::from)))
    }

    /// Maps one compact selected-fiber scalar through this exact component.
    ///
    /// If the retained coordinate is selected by `F(alpha, u) = 0`, the
    /// component support `S(u, v) = 0` is projected only as far as a local
    /// image relation `H(alpha, v) = 0`.  Every isolated image root is replayed
    /// against `S` at the authored `(u, v)` tuple before the component's
    /// certified fiber rank selects the branch.  This method is entered under
    /// [`CurveContext::strict_predicate_pass`], because both isolation and
    /// rank selection become construction evidence.
    pub(super) fn map_selected_fiber_parameter(
        &self,
        retained_parameter: CurveResultantParameter,
        parameter: &BezierAlgebraicSelectedFiberParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        parameter.validate_policy(policy)?;
        let (retained_range, lifted_range) = match retained_parameter {
            CurveResultantParameter::First => {
                (self.overlap.first_range(), self.overlap.second_range())
            }
            CurveResultantParameter::Second => {
                (self.overlap.second_range(), self.overlap.first_range())
            }
        };
        for (retained_endpoint, lifted_endpoint) in [
            (retained_range.start(), lifted_range.start()),
            (retained_range.end(), lifted_range.end()),
        ] {
            match parameter.cmp_bezier_parameter(retained_endpoint, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(Some(CurveParameter2::from(
                        lifted_endpoint.clone(),
                    ))));
                }
                Classification::Decided(_) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        match overlap_parameter_is_in_range(
            &CurveParameter2::from_selected_fiber(parameter.clone()),
            retained_range,
            false,
            policy,
        )? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        let swapped_support;
        let support = match retained_parameter {
            CurveResultantParameter::First => self.support.as_ref(),
            CurveResultantParameter::Second => {
                swapped_support = bivariate_swap_parameters(&self.support);
                &swapped_support
            }
        };
        // The stored fiber rank counts every root on the normalized chart.
        // Clip to lifted_range only after selecting that ranked branch.
        let support_roots = match selected_fiber_polynomial_relation_parameters(
            parameter,
            support,
            &CurveParameterRange2::unit(),
            policy,
        )? {
            Classification::Decided(Some(candidates)) => candidates,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        for pair in support_roots.windows(2) {
            match pair[0].cmp_by_refinement(&pair[1], policy)? {
                Classification::Decided(std::cmp::Ordering::Less) => {}
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Err(CurveError::Topology(
                        "a selected component image isolated one root twice".into(),
                    ));
                }
                Classification::Decided(std::cmp::Ordering::Greater) => {
                    return Err(CurveError::Topology(
                        "selected component image roots were not ordered".into(),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let rank = match self.resolved_fiber_root_rank(retained_parameter, policy)? {
            Classification::Decided(rank) => rank,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let Some(mapped) = support_roots.into_iter().nth(rank) else {
            return Ok(Classification::Decided(None));
        };
        match overlap_parameter_is_in_range(
            &CurveParameter2::from_selected_fiber(mapped.clone()),
            lifted_range,
            true,
            policy,
        )? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "parameter-component-map",
            "selected-fiber-local-image",
        );
        Ok(Classification::Decided(Some(
            mapped.represented_value().map_or_else(
                || CurveParameter2::from_selected_fiber(mapped.clone()),
                |value| CurveParameter2::from(BezierParameter2::Exact(value.clone())),
            ),
        )))
    }

    /// Maps one recursively retained scalar through the authored component
    /// without first constructing its global univariate image. The component
    /// is specialized directly in the recursive coefficient tower; its
    /// linear/quadratic image stays there, while higher degree uses the shared
    /// dense projection only for candidate enumeration and exact sheet replay.
    pub(super) fn map_recursive_projective_parameter(
        &self,
        retained_parameter: CurveResultantParameter,
        parameter: &BezierRecursiveProjectiveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        parameter.validate_policy(policy)?;
        let (retained_range, lifted_range) = match retained_parameter {
            CurveResultantParameter::First => {
                (self.overlap.first_range(), self.overlap.second_range())
            }
            CurveResultantParameter::Second => {
                (self.overlap.second_range(), self.overlap.first_range())
            }
        };
        for (retained_endpoint, lifted_endpoint) in [
            (retained_range.start(), lifted_range.start()),
            (retained_range.end(), lifted_range.end()),
        ] {
            match parameter.cmp_bezier_parameter(retained_endpoint, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(Some(CurveParameter2::from(
                        lifted_endpoint.clone(),
                    ))));
                }
                Classification::Decided(_) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        match overlap_parameter_is_in_range(
            &CurveParameter2::from_recursive_projective(parameter.clone()),
            retained_range,
            false,
            policy,
        )? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        let swapped_support;
        let support = match retained_parameter {
            CurveResultantParameter::First => self.support.as_ref(),
            CurveResultantParameter::Second => {
                swapped_support = bivariate_swap_parameters(&self.support);
                &swapped_support
            }
        };
        let Some(equation) = recursive_projective_bivariate_first_parameter_polynomial(
            support,
            parameter,
            bivariate_first_active_degree(support),
        ) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(scalar) = parameter.projective_scalar() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let candidates = match recursive_projective_polynomial_parameters(
            &scalar.numerator.field(),
            equation,
            SelectedThirdAxisDomain2::Finite(&CurveParameterRange2::unit()),
            policy,
        )? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let rank = match self.resolved_fiber_root_rank(retained_parameter, policy)? {
            Classification::Decided(rank) => rank,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let Some(mapped) = candidates.into_iter().nth(rank) else {
            return Ok(Classification::Decided(None));
        };
        match overlap_parameter_is_in_range(&mapped, lifted_range, true, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "parameter-component-map",
            "recursive-projective-local-image",
        );
        Ok(Classification::Decided(Some(mapped)))
    }

    pub(super) fn resolved_fiber_root_rank(
        &self,
        retained_parameter: CurveResultantParameter,
        policy: &CurveContext,
    ) -> CurveResult<Classification<usize>> {
        let stored = self.fiber_root_ranks[match retained_parameter {
            CurveResultantParameter::First => 0,
            CurveResultantParameter::Second => 1,
        }];
        if stored != UNKNOWN_PARAMETER_COMPONENT_FIBER_ROOT_RANK {
            return Ok(Classification::Decided(stored));
        }
        parameter_component_fiber_root_rank(
            &self.support,
            &self.witness,
            retained_parameter,
            policy,
        )
    }

    pub(super) fn map_parameter(
        &self,
        retained_parameter: CurveResultantParameter,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierParameter2>>> {
        let (retained_range, lifted_range) = match retained_parameter {
            CurveResultantParameter::First => {
                (self.overlap.first_range(), self.overlap.second_range())
            }
            CurveResultantParameter::Second => {
                (self.overlap.second_range(), self.overlap.first_range())
            }
        };
        for (retained_endpoint, lifted_endpoint) in [
            (retained_range.start(), lifted_range.start()),
            (retained_range.end(), lifted_range.end()),
        ] {
            match parameter.same_value(retained_endpoint, policy)? {
                Classification::Decided(true) => {
                    return Ok(Classification::Decided(Some(lifted_endpoint.clone())));
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        match overlap_parameter_is_in_range(
            &parameter.clone().into(),
            retained_range,
            false,
            policy,
        )? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        let rank = self.fiber_root_ranks[match retained_parameter {
            CurveResultantParameter::First => 0,
            CurveResultantParameter::Second => 1,
        }];
        if let BezierParameter2::Exact(parameter) = parameter {
            let coefficients = match retained_parameter {
                CurveResultantParameter::First => {
                    bivariate_specialize_first(&self.support, parameter)
                }
                CurveResultantParameter::Second => {
                    bivariate_specialize_second(&self.support, parameter)
                }
            };
            let roots = match polynomial_unit_interval_roots(&coefficients, policy)? {
                Classification::Decided(Some(roots)) => roots,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if rank != UNKNOWN_PARAMETER_COMPONENT_FIBER_ROOT_RANK {
                let Some(mapped) = roots.get(rank).cloned() else {
                    return Ok(Classification::Decided(None));
                };
                return match overlap_parameter_is_in_range(
                    &mapped.clone().into(),
                    lifted_range,
                    true,
                    policy,
                )? {
                    Classification::Decided(true) => Ok(Classification::Decided(Some(mapped))),
                    Classification::Decided(false) => Ok(Classification::Decided(None)),
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                };
            }
            let mut candidates = Vec::new();
            for (root_rank, root) in roots.into_iter().enumerate() {
                match overlap_parameter_is_in_range(
                    &root.clone().into(),
                    lifted_range,
                    true,
                    policy,
                )? {
                    Classification::Decided(true) => candidates.push((root_rank, root)),
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            if candidates.len() <= 1 {
                return Ok(Classification::Decided(
                    candidates.pop().map(|(_, root)| root),
                ));
            }
            let desired_rank = match self.resolved_fiber_root_rank(retained_parameter, policy)? {
                Classification::Decided(rank) => rank,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            return Ok(Classification::Decided(
                candidates
                    .into_iter()
                    .find_map(|(rank, root)| (rank == desired_rank).then_some(root)),
            ));
        }

        let BezierParameter2::Algebraic(algebraic) = parameter else {
            unreachable!("Bezier parameters are exact or algebraic")
        };
        let defining = algebraic.polynomial().coefficients();
        let axis_equation = match retained_parameter {
            CurveResultantParameter::First => bivariate_outer_product(defining, &[Real::one()]),
            CurveResultantParameter::Second => bivariate_outer_product(&[Real::one()], defining),
        };
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };
        let pairs = match bivariate_system_unit_square_solution_pairs(
            &self.support,
            &axis_equation,
            policy,
            config,
        )? {
            Classification::Decided(pairs) => pairs,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut candidate_pairs = Vec::new();
        for pair in pairs {
            let (candidate_retained, candidate_lifted) = match retained_parameter {
                CurveResultantParameter::First => (&pair.parallel_parameter, &pair.other_parameter),
                CurveResultantParameter::Second => {
                    (&pair.other_parameter, &pair.parallel_parameter)
                }
            };
            match candidate_retained.same_value(parameter, policy)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            match overlap_parameter_is_in_range(
                &candidate_lifted.clone().into(),
                lifted_range,
                true,
                policy,
            )? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            if rank != UNKNOWN_PARAMETER_COMPONENT_FIBER_ROOT_RANK {
                let candidate_rank = match parameter_component_fiber_root_rank(
                    &self.support,
                    &pair,
                    retained_parameter,
                    policy,
                )? {
                    Classification::Decided(candidate_rank) => candidate_rank,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if candidate_rank != rank {
                    continue;
                }
            }
            let candidate_lifted = candidate_lifted.clone();
            candidate_pairs.push((pair, candidate_lifted));
        }
        if candidate_pairs.len() <= 1 {
            return Ok(Classification::Decided(
                candidate_pairs.pop().map(|(_, parameter)| parameter),
            ));
        }
        if rank == UNKNOWN_PARAMETER_COMPONENT_FIBER_ROOT_RANK {
            let desired_rank = match self.resolved_fiber_root_rank(retained_parameter, policy)? {
                Classification::Decided(rank) => rank,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let mut mapped = None;
            for (pair, parameter) in candidate_pairs {
                let candidate_rank = match parameter_component_fiber_root_rank(
                    &self.support,
                    &pair,
                    retained_parameter,
                    policy,
                )? {
                    Classification::Decided(rank) => rank,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if candidate_rank == desired_rank && mapped.replace(parameter).is_some() {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
            }
            return Ok(Classification::Decided(mapped));
        }
        Ok(Classification::Uncertain(UncertaintyReason::Boundary))
    }

    pub(crate) fn clipped_ranges(
        &self,
        first_fragment: &CurveParameterRange2,
        second_fragment: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(CurveParameterRange2, CurveParameterRange2)>>> {
        let first_overlap =
            CurveParameterRange2::from_bezier_range(self.overlap().first_range().clone());
        let second_overlap =
            CurveParameterRange2::from_bezier_range(self.overlap().second_range().clone());
        crate::bezier_split::clip_corresponding_parameter_ranges(
            &first_overlap,
            &second_overlap,
            first_fragment,
            second_fragment,
            policy,
            |parameter| self.map_curve_parameter(CurveResultantParameter::First, parameter, policy),
            |parameter| {
                self.map_curve_parameter(CurveResultantParameter::Second, parameter, policy)
            },
        )
    }

    /// Decides component existence without constructing inverse cuts.
    pub(crate) fn has_positive_overlap(
        &self,
        first_fragment: &CurveParameterRange2,
        second_fragment: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let first_overlap =
            CurveParameterRange2::from_bezier_range(self.overlap().first_range().clone());
        let second_overlap =
            CurveParameterRange2::from_bezier_range(self.overlap().second_range().clone());
        crate::bezier_split::corresponding_parameter_ranges_are_positive(
            &first_overlap,
            &second_overlap,
            first_fragment,
            second_fragment,
            policy,
            |parameter| self.map_curve_parameter(CurveResultantParameter::First, parameter, policy),
        )
    }
}

#[cfg(test)]
pub(crate) fn nonlinear_parameter_component_overlap_for_test(
    policy: &CurveContext,
) -> BezierParameterComponentOverlap2 {
    // u=t^2 is monotone on the authored square but is not an affine or
    // projective parameter correspondence.
    let support = BivariatePolynomial::new(vec![
        vec![Real::zero(), Real::one()],
        vec![Real::zero()],
        vec![Real::from(-1_i8)],
    ]);
    let branch = BivariatePolynomial::new(vec![vec![Real::one()]]);
    let config = CurveIntersectionResultantConfig {
        min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
        max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
    };
    let Classification::Decided(Some(component)) =
        parameter_component_system(&[support.clone(), support], &branch, policy, config).unwrap()
    else {
        panic!("the nonlinear parameter component was not certified");
    };
    let [overlap] = component.component_overlaps.as_ref() else {
        panic!("the nonlinear component omitted its exact support evidence");
    };
    overlap.clone()
}

pub(super) struct ParameterComponentOverlapDraft2 {
    pub(super) overlap: RationalBezierIntersectionOverlap2,
    pub(super) witness: BezierParallelIntersectionParameterPair2,
}

pub(super) fn parallel_candidate_system_from_parameter_components(
    component: ParameterComponentSystem2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParallelIntersectionCandidateSystem2>> {
    for equation in &component.residual_equations {
        if bivariate_unit_square_has_strict_bernstein_sign(equation, policy)? {
            let mut candidate_system = BezierParallelIntersectionCandidateSystem2::projected(
                CurveIntersectionCandidates2::NoIntersection,
                None,
            );
            candidate_system.overlaps = component.overlaps;
            candidate_system.component_overlaps = component.component_overlaps;
            candidate_system.component_pairs = component.component_pairs;
            candidate_system.selected_component_pair_count =
                component.selected_component_pair_count;
            return Ok(Classification::Decided(candidate_system));
        }
    }
    let candidates = match project_parallel_intersection_system(
        &component.residual_equations[0],
        &component.residual_equations[1],
        [CurveParameterDomain2::new(&CurveParameterRange2::unit(), None); 2],
        policy,
    )? {
        Classification::Decided(candidates) => candidates,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    match parallel_intersection_candidate_system(component.residual_equations, candidates, policy)?
    {
        Classification::Decided(mut candidate_system) => {
            candidate_system.overlaps = component.overlaps;
            candidate_system.component_overlaps = component.component_overlaps;
            candidate_system.component_pairs = component.component_pairs;
            candidate_system.selected_component_pair_count =
                component.selected_component_pair_count;
            Ok(Classification::Decided(candidate_system))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

/// Proves one bivariate polynomial nonzero throughout the authored parameter square.
///
/// Tensor-product Bernstein basis functions are nonnegative and sum to one on
/// `[0, 1]^2`, so controls with one strict sign certify that the polynomial has
/// that sign everywhere. A mixed, zero, or undecidable control merely declines
/// this acceleration and leaves the complete resultant path authoritative.
/// Inspect only already-retained scalar facts here: walking or refining a large
/// lazy expression graph would make this optional shortcut unbounded.
pub(super) fn bivariate_unit_square_strict_bernstein_sign(
    polynomial: &BivariatePolynomial,
    _policy: &CurveContext,
) -> CurveResult<Option<RealSign>> {
    let first_degree = polynomial.coefficients.len().saturating_sub(1);
    let second_degree = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or_default()
        .saturating_sub(1);
    if polynomial.coefficients.is_empty() || polynomial.coefficients.iter().all(Vec::is_empty) {
        return Ok(None);
    }

    let second_controls = polynomial
        .coefficients
        .iter()
        .map(|row| power_to_bernstein_coefficients(row, second_degree))
        .collect::<CurveResult<Vec<_>>>()?;
    let mut strict_sign = None;
    for second_index in 0..=second_degree {
        let first_power = second_controls
            .iter()
            .map(|row| row[second_index].clone())
            .collect::<Vec<_>>();
        for control in power_to_bernstein_coefficients(&first_power, first_degree)? {
            let Some(sign @ (RealSign::Positive | RealSign::Negative)) = control.immediate_sign()
            else {
                return Ok(None);
            };
            match strict_sign {
                Some(previous) if previous != sign => return Ok(None),
                Some(_) => {}
                None => strict_sign = Some(sign),
            }
        }
    }
    Ok(strict_sign)
}

pub(super) fn bivariate_unit_square_has_strict_bernstein_sign(
    polynomial: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<bool> {
    Ok(bivariate_unit_square_strict_bernstein_sign(polynomial, policy)?.is_some())
}

pub(super) fn bivariate_restrict_to_parameter_box(
    polynomial: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
) -> BivariatePolynomial {
    let first = real_interval_from_parameter(first_parameter);
    let second = real_interval_from_parameter(second_parameter);
    polynomial.substitute_affine(
        &(&first.upper - &first.lower),
        &first.lower,
        &(&second.upper - &second.lower),
        &second.lower,
    )
}

pub(crate) fn bivariate_fiber_strict_sign_on_parameter_range(
    polynomial: &BivariatePolynomial,
    retained: &BezierAlgebraicParameter2,
    fiber_range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Option<RealSign>> {
    policy.bounded_exact_predicate_pass(|| {
        let retained_parameter = BezierParameter2::Algebraic(retained.clone());
        let envelope = |range: &CurveParameterRange2| {
            CurveParameterDomain2::new(range, None)
                .finite_envelope(policy)
                .map(|bounds| bounds.map(|(_, [lower, upper])| [lower.clone(), upper.clone()]))
        };
        let [fiber_lower, fiber_upper] = match envelope(fiber_range)? {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(_) => return Ok(None),
        };
        let restricted = polynomial.substitute_affine(
            &(retained.interval().end() - retained.interval().start()),
            retained.interval().start(),
            &(&fiber_upper - &fiber_lower),
            &fiber_lower,
        );
        if let Some(sign) = bivariate_unit_square_strict_bernstein_sign(&restricted, policy)? {
            return Ok(Some(sign));
        }
        // Refine only scheduling bounds. The selected endpoint authorities remain
        // unchanged, and construction decisions retain the caller's policy.
        let mut endpoints = Vec::with_capacity(2);
        for endpoint in [fiber_range.start(), fiber_range.end()] {
            match endpoint.refined_for_finite_envelope(16, policy)? {
                Classification::Decided(endpoint) => endpoints.push(endpoint),
                Classification::Uncertain(_) => return Ok(None),
            }
        }
        let refined = CurveParameterRange2::new_validated(endpoints.remove(0), endpoints.remove(0));
        let [fiber_lower, fiber_upper] = match envelope(&refined)? {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(_) => return Ok(None),
        };
        let restricted = polynomial.substitute_affine(
            &Real::one(),
            &Real::zero(),
            &(&fiber_upper - &fiber_lower),
            &fiber_lower,
        );
        let fiber_degree = restricted
            .coefficients
            .iter()
            .map(Vec::len)
            .max()
            .unwrap_or_default()
            .saturating_sub(1);
        let controls_by_retained_power = restricted
            .coefficients
            .iter()
            .map(|row| power_to_bernstein_coefficients(row, fiber_degree))
            .collect::<CurveResult<Vec<_>>>()?;
        let mut controls = Vec::with_capacity(fiber_degree + 1);
        for fiber_index in 0..=fiber_degree {
            let coefficient = controls_by_retained_power
                .iter()
                .map(|row| row[fiber_index].clone())
                .collect::<Vec<_>>();
            let coefficient = match retained
                .polynomial()
                .reduce_power_basis(coefficient, policy)?
            {
                Classification::Decided(coefficient) => coefficient,
                Classification::Uncertain(_) => return Ok(None),
            };
            controls.push(coefficient);
        }

        // Enclose all selected-center control values once, then perform target
        // subdivision using rational interval averages only. Tightening both the
        // center bracket and scalar precision preserves a fully exact proof while
        // avoiding one nested algebraic sign/GCD calculation per child control.
        let strict = policy.strict_counterpart();
        let mut interval_refinement = BezierParameterRefinement2::new(&retained_parameter, &strict);
        for (target_steps, precision) in
            [(8, -64), (16, -128), (32, -256), (64, -512), (128, -1024)]
        {
            let parameter = interval_refinement.refine_to(target_steps);
            let intervals = controls
                .iter()
                .map(|control| {
                    coefficients_value_interval_on_parameter_interval(control, parameter, precision)
                })
                .collect::<CurveResult<Option<Vec<_>>>>()?;
            if let Some(sign) = intervals.and_then(rational_interval_bernstein_strict_sign) {
                return Ok(Some(sign));
            }
        }

        // All target-Bernstein controls share the same selected center. Refine
        // that isolator once for the complete batch before invoking any
        // polynomial GCD. This proves the overwhelmingly common strict-sign case
        // without rebuilding the same high-degree center Sturm sequence for each
        // control independently.
        let mut refinement = BezierParameterRefinement2::new(&retained_parameter, &strict);
        for target_steps in [0, 1, 2, 4, 8, 16, 32, 64] {
            let parameter = refinement.refine_to(target_steps);
            let mut common_sign = None;
            let mut all_decided = true;
            let mut mixed = false;
            for control in &controls {
                let Some(sign) =
                    strict_coefficients_sign_on_parameter_interval(control, parameter, &strict)?
                else {
                    all_decided = false;
                    break;
                };
                if common_sign.is_some_and(|common| common != sign) {
                    mixed = true;
                } else if common_sign.is_none() {
                    common_sign = Some(sign);
                }
            }
            if all_decided {
                if !mixed {
                    return Ok(common_sign);
                }
                break;
            }
        }

        // Mixed controls can still bound a one-sign polynomial. Repeated
        // de Casteljau subdivision converges to the exact fiber image; each
        // accepted child requires weakly one-sign controls and strict endpoint
        // controls, which also excludes a root on the closed child boundary.
        // Bound the certificate work so genuinely intersecting fibers promptly
        // return to the authoritative projection path.
        let refined_retained_parameter = refinement.refine_to(64).clone();
        let mut pending = vec![(controls, 0_u8)];
        let mut certified_sign = None;
        let mut visited = 0_usize;
        while let Some((controls, depth)) = pending.pop() {
            visited += 1;
            let mut signs = Vec::with_capacity(controls.len());
            for control in &controls {
                let sign = if let Some(sign) = strict_coefficients_sign_on_parameter_interval(
                    control,
                    &refined_retained_parameter,
                    &strict,
                )? {
                    sign
                } else {
                    match signed_coefficients_at_parameter(control, &retained_parameter, policy)? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(_) => return Ok(None),
                    }
                };
                signs.push(sign);
            }
            let endpoints_are_positive = signs.first() == Some(&RealSign::Positive)
                && signs.last() == Some(&RealSign::Positive);
            let endpoints_are_negative = signs.first() == Some(&RealSign::Negative)
                && signs.last() == Some(&RealSign::Negative);
            if signs.first() == Some(&RealSign::Zero)
                || signs.last() == Some(&RealSign::Zero)
                || matches!(
                    (signs.first(), signs.last()),
                    (Some(RealSign::Positive), Some(RealSign::Negative))
                        | (Some(RealSign::Negative), Some(RealSign::Positive))
                )
            {
                // A zero endpoint or opposite strict endpoint signs certify an
                // actual root on this closed child, so further subdivision cannot
                // prove the fiber root-free.
                return Ok(None);
            }
            let segment_sign = if endpoints_are_positive
                && signs.iter().all(|sign| *sign != RealSign::Negative)
            {
                Some(RealSign::Positive)
            } else if endpoints_are_negative && signs.iter().all(|sign| *sign != RealSign::Positive)
            {
                Some(RealSign::Negative)
            } else {
                None
            };
            if let Some(segment_sign) = segment_sign {
                match certified_sign {
                    Some(previous) if previous != segment_sign => return Ok(None),
                    Some(_) => {}
                    None => certified_sign = Some(segment_sign),
                }
                continue;
            }
            if depth == 8 || visited >= 64 {
                return Ok(None);
            }

            let mut work = controls;
            let degree = work.len().saturating_sub(1);
            let mut left = Vec::with_capacity(work.len());
            let mut right = Vec::with_capacity(work.len());
            left.push(work[0].clone());
            right.push(work[degree].clone());
            let half = (Real::one() / Real::from(2_i8))?;
            for level in 1..=degree {
                for index in 0..=degree - level {
                    work[index] =
                        polynomial_scale(&polynomial_add(&work[index], &work[index + 1]), &half);
                }
                left.push(work[0].clone());
                right.push(work[degree - level].clone());
            }
            right.reverse();
            pending.push((right, depth + 1));
            pending.push((left, depth + 1));
        }
        Ok(certified_sign)
    })
}

pub(super) fn bivariate_parameter_box_strict_sign(
    polynomial: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Option<RealSign>> {
    // Horner enclosures need only linear work in the coefficient grid. Try
    // them before expanding a fresh polynomial in the isolator's affine chart.
    // An unresolved enclosure leaves Bernstein and algebraic replay intact.
    if let Some(sign) = policy.bounded_exact_predicate_pass(|| {
        RealInterval::evaluate_bivariate_power_basis(
            polynomial,
            &real_interval_from_parameter(first_parameter),
            &real_interval_from_parameter(second_parameter),
        )
        .and_then(|interval| interval.strict_nonzero_sign())
    }) {
        return Ok(Some(sign));
    }
    bivariate_unit_square_strict_bernstein_sign(
        &bivariate_restrict_to_parameter_box(polynomial, first_parameter, second_parameter),
        policy,
    )
}

/// Certifies a common zero on `[0, 1]^2` by the two-dimensional
/// Poincare-Miranda theorem.
///
/// Strict Bernstein signs prove the first equation has opposite signs on the
/// two vertical faces and the second has opposite signs on the horizontal
/// faces. Swapping the equations is equivalent. This is only an existence
/// certificate; its caller is responsible for associating the zero with an
/// isolated pair of resultant roots.
pub(super) fn bivariate_unit_square_has_poincare_miranda_root(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<bool> {
    let zero = Real::zero();
    let one = Real::one();
    for (vertical, horizontal) in [(first, second), (second, first)] {
        let left = univariate_unit_interval_strict_bernstein_sign(
            &bivariate_specialize_first(vertical, &zero),
            policy,
        )?;
        let right = univariate_unit_interval_strict_bernstein_sign(
            &bivariate_specialize_first(vertical, &one),
            policy,
        )?;
        if !strict_signs_are_opposite(left, right) {
            continue;
        }
        let bottom = univariate_unit_interval_strict_bernstein_sign(
            &bivariate_specialize_second(horizontal, &zero),
            policy,
        )?;
        let top = univariate_unit_interval_strict_bernstein_sign(
            &bivariate_specialize_second(horizontal, &one),
            policy,
        )?;
        if strict_signs_are_opposite(bottom, top) {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Applies an exact midpoint-Jacobian row preconditioner before the
/// Poincare-Miranda test. At a transverse zero the transformed equations have
/// axis-aligned first derivatives at the midpoint, so sufficiently refined
/// isolating boxes acquire the required strict face signs. A nonzero
/// determinant proves that the row transform preserves the common-zero set.
pub(super) fn bivariate_unit_square_has_preconditioned_poincare_miranda_root(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<bool> {
    if bivariate_unit_square_has_poincare_miranda_root(first, second, policy)? {
        return Ok(true);
    }
    let half = (Real::one() / Real::from(2_i8))?;
    let evaluate_midpoint = |polynomial: &BivariatePolynomial| {
        Real::eval_poly(&bivariate_specialize_first(polynomial, &half), &half)
    };
    let first_first = evaluate_midpoint(&bivariate_parameter_derivative(
        first,
        CurveResultantParameter::First,
    ));
    let first_second = evaluate_midpoint(&bivariate_parameter_derivative(
        first,
        CurveResultantParameter::Second,
    ));
    let second_first = evaluate_midpoint(&bivariate_parameter_derivative(
        second,
        CurveResultantParameter::First,
    ));
    let second_second = evaluate_midpoint(&bivariate_parameter_derivative(
        second,
        CurveResultantParameter::Second,
    ));
    let determinant = &first_first * &second_second - &first_second * &second_first;
    if !matches!(
        real_sign(&determinant, policy),
        Some(RealSign::Positive | RealSign::Negative)
    ) {
        return Ok(false);
    }
    let preconditioned_first = bivariate_subtract(
        &bivariate_scale(first.clone(), &second_second),
        &bivariate_scale(second.clone(), &first_second),
    );
    let preconditioned_second = bivariate_subtract(
        &bivariate_scale(second.clone(), &first_first),
        &bivariate_scale(first.clone(), &second_first),
    );
    bivariate_unit_square_has_poincare_miranda_root(
        &preconditioned_first,
        &preconditioned_second,
        policy,
    )
}

/// Proves that one Cartesian pair of isolated resultant roots is a common
/// bivariate zero. Each coordinate interval isolates exactly one root of the
/// corresponding projection polynomial; Poincare-Miranda supplies a common
/// zero in their rectangle, so its coordinates must be the represented roots.
///
/// This helper is sound only when `first_parameter` and `second_parameter`
/// came from the two resultant projections of `first` and `second`.
pub(super) fn projected_bivariate_parameter_pair_has_box_root(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<bool> {
    if !matches!(first_parameter, BezierParameter2::Algebraic(_))
        || !matches!(second_parameter, BezierParameter2::Algebraic(_))
    {
        return Ok(false);
    }
    Ok(refine_parameter_pair_for_certificate(
        first_parameter,
        second_parameter,
        policy,
        |first_parameter, second_parameter| {
            let restricted_first =
                bivariate_restrict_to_parameter_box(first, first_parameter, second_parameter);
            let restricted_second =
                bivariate_restrict_to_parameter_box(second, first_parameter, second_parameter);
            Ok(
                bivariate_unit_square_has_preconditioned_poincare_miranda_root(
                    &restricted_first,
                    &restricted_second,
                    policy,
                )?
                .then_some(()),
            )
        },
    )?
    .is_some())
}

pub(crate) fn bivariate_parameter_pair_strict_sign_by_refinement(
    polynomial: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Option<RealSign>> {
    refine_parameter_pair_for_certificate(
        first_parameter,
        second_parameter,
        policy,
        |first, second| bivariate_parameter_box_strict_sign(polynomial, first, second, policy),
    )
}

/// Shares incremental parameter refinement across the predicates of one
/// certificate query. Declining this finite schedule leaves exact replay
/// authoritative; an unchanged box is never evaluated a second time.
pub(super) fn refine_parameter_pair_for_certificate<T>(
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
    mut evaluate: impl FnMut(&BezierParameter2, &BezierParameter2) -> CurveResult<Option<T>>,
) -> CurveResult<Option<T>> {
    let mut first_refinement = BezierParameterRefinement2::new(first_parameter, policy);
    let mut second_refinement = BezierParameterRefinement2::new(second_parameter, policy);
    let mut previous_box = None;
    for target_steps in [0, 2, 4, 8, 16, 32] {
        let refined_first = first_refinement.refine_to(target_steps).clone();
        let refined_second = second_refinement.refine_to(target_steps).clone();
        if previous_box
            .as_ref()
            .is_some_and(|(first, second)| first == &refined_first && second == &refined_second)
        {
            break;
        }
        previous_box = Some((refined_first.clone(), refined_second.clone()));
        if let Some(certificate) = evaluate(&refined_first, &refined_second)? {
            return Ok(Some(certificate));
        }
    }
    Ok(None)
}

pub(super) fn parameter_component_system(
    equations: &[BivariatePolynomial; 2],
    branch: &BivariatePolynomial,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<Option<ParameterComponentSystem2>>> {
    parameter_component_system_with_selector(
        equations,
        &ParameterComponentSelector2::Positive(branch, None),
        policy,
        config,
    )
}

pub(super) fn parameter_component_system_with_selector(
    equations: &[BivariatePolynomial; 2],
    selector: &ParameterComponentSelector2<'_>,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<Option<ParameterComponentSystem2>>> {
    let mut residual_equations = equations.clone();
    let mut overlaps = Vec::new();
    let mut component_overlaps = Vec::new();
    let mut isolated_pairs = Vec::new();
    let mut excluded_pairs = Vec::new();
    let mut extracted_component = false;
    loop {
        let mut blocker = None;
        let mut next_component = None;
        for retained_parameter in [
            CurveResultantParameter::Second,
            CurveResultantParameter::First,
        ] {
            let report = parameter_component_bivariate_polynomial_system_complete(
                &residual_equations[0],
                &residual_equations[1],
                retained_parameter,
                config,
            );
            match report.status {
                BivariatePolynomialComponentStatus::Rational => {
                    let Some(reduced_equations) = report.reduced_equations else {
                        continue;
                    };
                    let map = CurveIntersectionParameterLiftMap {
                        cofactor_row: 0,
                        numerator_coefficients: report.numerator_coefficients,
                        denominator_coefficients: report.denominator_coefficients,
                    };
                    match certify_rational_parameter_component_map_with_selector(
                        &residual_equations,
                        selector,
                        retained_parameter,
                        &map,
                        policy,
                    )? {
                        Classification::Decided(Some(evidence)) => {
                            next_component = Some((evidence, reduced_equations));
                            break;
                        }
                        Classification::Decided(None) => {}
                        Classification::Uncertain(reason) => blocker = Some(reason),
                    }
                }
                BivariatePolynomialComponentStatus::UndecidedCoefficient => {
                    blocker = Some(UncertaintyReason::RealSign);
                }
                BivariatePolynomialComponentStatus::Implicit => {
                    let (Some(component), Some(reduced_equations)) =
                        (report.implicit_component, report.reduced_equations)
                    else {
                        blocker = Some(UncertaintyReason::Boundary);
                        continue;
                    };
                    match certify_regular_implicit_parameter_component_with_selector(
                        &component,
                        selector,
                        retained_parameter,
                        policy,
                        config,
                    )? {
                        Classification::Decided(Some(evidence)) => {
                            next_component = Some((evidence, reduced_equations));
                            break;
                        }
                        Classification::Decided(None) => {
                            blocker = Some(UncertaintyReason::Boundary)
                        }
                        Classification::Uncertain(reason) => blocker = Some(reason),
                    }
                }
                BivariatePolynomialComponentStatus::EmptyEquation
                | BivariatePolynomialComponentStatus::UnsupportedLiftedDegree
                | BivariatePolynomialComponentStatus::DegreeBoundExceeded
                | BivariatePolynomialComponentStatus::NoSupportedComponent
                | BivariatePolynomialComponentStatus::DeterminantError
                | BivariatePolynomialComponentStatus::InterpolationFailed
                | BivariatePolynomialComponentStatus::DivisionFailed => {}
            }
        }
        let Some((evidence, reduced_equations)) = next_component else {
            if !extracted_component {
                return Ok(blocker
                    .map_or_else(|| Classification::Decided(None), Classification::Uncertain));
            }
            return Ok(Classification::Decided(Some(
                ParameterComponentSystem2::from_partitioned_pairs(
                    overlaps,
                    component_overlaps,
                    isolated_pairs,
                    excluded_pairs,
                    residual_equations,
                ),
            )));
        };
        for overlap in evidence.overlaps.iter() {
            if !overlaps.contains(overlap) {
                overlaps.push(overlap.clone());
            }
        }
        for overlap in evidence.component_overlaps.iter() {
            if !component_overlaps.contains(overlap) {
                component_overlaps.push(overlap.clone());
            }
        }
        for pair in evidence.selected_pairs() {
            if !isolated_pairs.contains(pair) {
                isolated_pairs.push(pair.clone());
            }
        }
        for pair in evidence.excluded_pairs() {
            if !excluded_pairs.contains(pair) {
                excluded_pairs.push(pair.clone());
            }
        }
        residual_equations = reduced_equations;
        extracted_component = true;
        for equation in &residual_equations {
            if bivariate_unit_square_has_strict_bernstein_sign(equation, policy)? {
                return Ok(Classification::Decided(Some(
                    ParameterComponentSystem2::from_partitioned_pairs(
                        overlaps,
                        component_overlaps,
                        isolated_pairs,
                        excluded_pairs,
                        residual_equations,
                    ),
                )));
            }
        }
    }
}

/// Certifies the complete unit-square image of one finite-event implicit component.
///
/// A globally graphical component takes the ordered-fiber fast path. The
/// general path partitions at both projection derivatives and all authored
/// square boundaries, proves every event incidence through isolated exact
/// fiber tubes, and emits one doubly monotone oriented overlap cell per edge.
/// Projection folds, transverse domain crossings, isolated boundary touches,
/// and isolated singular vertices are accepted. The parent intersection engine
/// replays axis-wide and boundary-coincident components against constant-image
/// geometry. When ordinary certification fails, a cold fallback first removes
/// component multiplicity. Exact projection degeneration then gates removal of
/// selected-branch-zero factors before the same topology proof is retried.
pub(super) fn certify_regular_implicit_parameter_component_with_selector(
    component: &BivariatePolynomial,
    selector: &ParameterComponentSelector2<'_>,
    retained_parameter: CurveResultantParameter,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<Option<ParameterComponentEvidence2>>> {
    match selector {
        ParameterComponentSelector2::Positive(branch, None) => {
            certify_regular_implicit_parameter_component(
                component,
                branch,
                retained_parameter,
                policy,
                config,
            )
        }
        ParameterComponentSelector2::Positive(_, Some(_))
        | ParameterComponentSelector2::ParallelPair { .. } => {
            certify_parallel_pair_implicit_parameter_component(
                component,
                selector,
                retained_parameter,
                policy,
                config,
            )
        }
    }
}

/// Certifies every exact factor of a reducible selected-branch component.
///
/// A parallel-pair selector is piecewise: a selector boundary can vanish on
/// one component while the remaining exact predicates still select or reject
/// that complete component. Sending the unreduced product to the cell builder
/// makes `component = boundary = 0` look positive-dimensional and prevents an
/// otherwise regular sibling from being published. Split only after exact
/// division, certify both supports independently, and merge their correlated
/// evidence. This is deliberately separate from the ordinary `branch > 0`
/// path, where a branch-zero factor is always excluded rather than selected by
/// another predicate.
#[cold]
#[inline(never)]
pub(super) fn certify_parallel_pair_implicit_parameter_component(
    component: &BivariatePolynomial,
    selector: &ParameterComponentSelector2<'_>,
    retained_parameter: CurveResultantParameter,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<Option<ParameterComponentEvidence2>>> {
    let boundaries = selector.boundary_polynomials();
    let Some([first, second]) = split_parameter_component_at_selector_boundary(
        component,
        &boundaries,
        retained_parameter,
        config,
    ) else {
        return certify_implicit_parameter_component_once_with_selector(
            component,
            selector,
            retained_parameter,
            policy,
            config,
        );
    };

    let first = match certify_parallel_pair_implicit_parameter_component(
        &first,
        selector,
        retained_parameter,
        policy,
        config,
    )? {
        Classification::Decided(Some(evidence)) => evidence,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let second = match certify_parallel_pair_implicit_parameter_component(
        &second,
        selector,
        retained_parameter,
        policy,
        config,
    )? {
        Classification::Decided(Some(evidence)) => evidence,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(Some(
        merge_parameter_component_evidence(first, second),
    )))
}

/// Finds one proper exact factor shared by a component and selector boundary.
///
/// Direct division covers the common and cheapest case where the selector
/// boundary itself is a factor. Hypersolve's component report supplies the
/// exact GCD quotient when the boundary has additional factors. Both axis
/// orientations are attempted, and every accepted split is replayed through
/// exact bivariate division before topology sees either child.
pub(super) fn split_parameter_component_at_selector_boundary(
    component: &BivariatePolynomial,
    boundaries: &[BivariatePolynomial],
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> Option<[BivariatePolynomial; 2]> {
    let component_degree = bivariate_storage_bidegree_sum(component);
    if component_degree <= 1 {
        return None;
    }
    let alternate_parameter = match retained_parameter {
        CurveResultantParameter::First => CurveResultantParameter::Second,
        CurveResultantParameter::Second => CurveResultantParameter::First,
    };
    for boundary in boundaries {
        let boundary_degree = bivariate_storage_bidegree_sum(boundary);
        if boundary_degree == 0 || divide_bivariate_polynomial_exact(boundary, component).is_some()
        {
            continue;
        }
        if let Some(quotient) = divide_bivariate_polynomial_exact(component, boundary)
            && proper_parameter_component_split(component_degree, boundary, &quotient)
        {
            return Some([boundary.clone(), quotient]);
        }
        for parameter in [retained_parameter, alternate_parameter] {
            let report = parameter_component_bivariate_polynomial_system_complete(
                component, boundary, parameter, config,
            );
            if !matches!(
                report.status,
                BivariatePolynomialComponentStatus::Rational
                    | BivariatePolynomialComponentStatus::Implicit
            ) {
                continue;
            }
            let Some([quotient, _]) = report.reduced_equations else {
                continue;
            };
            let Some(factor) = divide_bivariate_polynomial_exact(component, &quotient) else {
                continue;
            };
            if proper_parameter_component_split(component_degree, &factor, &quotient) {
                return Some([factor, quotient]);
            }
        }
    }
    None
}

pub(super) fn certify_regular_implicit_parameter_component(
    component: &BivariatePolynomial,
    branch: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<Option<ParameterComponentEvidence2>>> {
    let initial = certify_implicit_parameter_component_once(
        component,
        branch,
        retained_parameter,
        policy,
        config,
    )?;
    if matches!(initial, Classification::Decided(Some(_))) {
        return Ok(initial);
    }
    certify_regular_implicit_parameter_component_fallback(
        component,
        branch,
        retained_parameter,
        policy,
        config,
        initial,
    )
}

#[cold]
#[inline(never)]
pub(super) fn certify_regular_implicit_parameter_component_fallback(
    component: &BivariatePolynomial,
    branch: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
    initial: Classification<Option<ParameterComponentEvidence2>>,
) -> CurveResult<Classification<Option<ParameterComponentEvidence2>>> {
    if divide_bivariate_polynomial_exact(branch, component).is_some() {
        return Ok(Classification::Decided(Some(
            ParameterComponentEvidence2::default(),
        )));
    }
    let mut fallback = initial;
    let mut multiplicity_reduced = None;
    if let Some(reduced) =
        reduce_implicit_parameter_component_multiplicity(component, retained_parameter, config)
        && reduced != *component
    {
        fallback = certify_implicit_parameter_component_once(
            &reduced,
            branch,
            retained_parameter,
            policy,
            config,
        )?;
        if matches!(fallback, Classification::Decided(Some(_))) {
            return Ok(fallback);
        }
        multiplicity_reduced = Some(reduced);
    }

    let support = multiplicity_reduced.as_ref().unwrap_or(component);
    match bivariate_system_has_positive_dimensional_relation(support, branch, policy)? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(fallback),
        // The gate is only an optimization. A capped equality decision must
        // not suppress the exact component extractor that the prior path ran.
        Classification::Uncertain(_) => {}
    }
    let Some(reduced) = remove_implicit_parameter_component_zero_branch_factors(
        support,
        branch,
        retained_parameter,
        config,
    ) else {
        return Ok(fallback);
    };
    if bivariate_storage_bidegree_sum(&reduced) == 0 {
        return Ok(Classification::Decided(Some(
            ParameterComponentEvidence2::default(),
        )));
    }
    fallback = certify_implicit_parameter_component_once(
        &reduced,
        branch,
        retained_parameter,
        policy,
        config,
    )?;
    if matches!(fallback, Classification::Decided(Some(_))) {
        return Ok(fallback);
    }

    let Some(square_free) =
        reduce_implicit_parameter_component_multiplicity(&reduced, retained_parameter, config)
    else {
        return Ok(fallback);
    };
    if square_free == reduced {
        return Ok(fallback);
    }
    certify_implicit_parameter_component_once(
        &square_free,
        branch,
        retained_parameter,
        policy,
        config,
    )
}

/// Removes every positive-dimensional factor on which `branch > 0` is false.
///
/// Direct exact division catches a branch that vanishes on the complete
/// component. For a reducible component, the existing Hypersolve common-factor
/// authority repeatedly divides factors shared by the component and branch.
/// Only a strict bidegree decrease is accepted. The remaining quotient retains
/// the authored branch polynomial, so its sign on every surviving component is
/// still certified rather than canceled or inferred.
#[cold]
#[inline(never)]
pub(super) fn remove_implicit_parameter_component_zero_branch_factors(
    component: &BivariatePolynomial,
    branch: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> Option<BivariatePolynomial> {
    let alternate_parameter = match retained_parameter {
        CurveResultantParameter::First => CurveResultantParameter::Second,
        CurveResultantParameter::Second => CurveResultantParameter::First,
    };
    let mut reduced = component.clone();
    let mut changed = false;
    loop {
        if divide_bivariate_polynomial_exact(branch, &reduced).is_some() {
            return Some(BivariatePolynomial::new(vec![vec![Real::one()]]));
        }
        let degree = bivariate_storage_bidegree_sum(&reduced);
        let mut next = None;
        for parameter in [retained_parameter, alternate_parameter] {
            let report = parameter_component_bivariate_polynomial_system_complete(
                &reduced, branch, parameter, config,
            );
            if !matches!(
                report.status,
                BivariatePolynomialComponentStatus::Rational
                    | BivariatePolynomialComponentStatus::Implicit
            ) {
                continue;
            }
            let Some([candidate, _]) = report.reduced_equations else {
                continue;
            };
            if bivariate_storage_bidegree_sum(&candidate) < degree {
                next = Some(candidate);
                break;
            }
        }
        let Some(next) = next else {
            return changed.then_some(reduced);
        };
        reduced = next;
        changed = true;
    }
}

/// Removes every exactly extractable repeated factor before topology replay.
///
/// For a primitive characteristic-zero component `H`, a repeated geometric
/// factor divides both `H` and its derivative in the lifted parameter. The
/// Hypersolve component extractor remains the algebraic authority: each exact
/// report supplies `H / gcd_factor` as its first residual equation. Repeating
/// only while that exact division lowers the bidegree yields the geometric
/// square-free support without copying multiplicity into overlap topology.
#[cold]
#[inline(never)]
pub(super) fn reduce_implicit_parameter_component_multiplicity(
    component: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    config: CurveIntersectionResultantConfig,
) -> Option<BivariatePolynomial> {
    let differentiated_parameter = match retained_parameter {
        CurveResultantParameter::First => CurveResultantParameter::Second,
        CurveResultantParameter::Second => CurveResultantParameter::First,
    };
    let mut reduced = component.clone();
    loop {
        let derivative = bivariate_parameter_derivative(&reduced, differentiated_parameter);
        let report = parameter_component_bivariate_polynomial_system_complete(
            &reduced,
            &derivative,
            retained_parameter,
            config,
        );
        if !matches!(
            report.status,
            BivariatePolynomialComponentStatus::Rational
                | BivariatePolynomialComponentStatus::Implicit
        ) {
            return Some(reduced);
        }
        let [next, _] = report.reduced_equations?;
        if bivariate_storage_bidegree_sum(&next) >= bivariate_storage_bidegree_sum(&reduced) {
            return None;
        }
        reduced = next;
    }
}

pub(super) fn certify_implicit_parameter_component_once(
    component: &BivariatePolynomial,
    branch: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<Option<ParameterComponentEvidence2>>> {
    certify_implicit_parameter_component_once_with_selector(
        component,
        &ParameterComponentSelector2::Positive(branch, None),
        retained_parameter,
        policy,
        config,
    )
}

pub(super) fn certify_implicit_parameter_component_once_with_selector(
    component: &BivariatePolynomial,
    selector: &ParameterComponentSelector2<'_>,
    retained_parameter: CurveResultantParameter,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<Option<ParameterComponentEvidence2>>> {
    let swapped_component;
    let component = match retained_parameter {
        CurveResultantParameter::First => component,
        CurveResultantParameter::Second => {
            swapped_component = bivariate_swap_parameters(component);
            &swapped_component
        }
    };
    let mut selection_boundaries = selector.boundary_polynomials();
    if retained_parameter == CurveResultantParameter::Second {
        for boundary in &mut selection_boundaries {
            *boundary = bivariate_swap_parameters(boundary);
        }
    }
    selection_boundaries.retain(|boundary| {
        // An identically zero selector term cannot partition this support. It
        // remains visible to `selected_at`, where the other exact predicates
        // decide the tangent-parallel or strict-positive case.
        divide_bivariate_polynomial_exact(boundary, component).is_none()
    });

    if let Classification::Decided(Some(evidence)) =
        certify_regular_implicit_parameter_graph_with_selector(
            component,
            selector,
            &selection_boundaries,
            retained_parameter,
            policy,
            config,
        )?
    {
        return Ok(Classification::Decided(Some(evidence)));
    }
    certify_regular_implicit_parameter_cells_with_selector(
        component,
        selector,
        &selection_boundaries,
        retained_parameter,
        policy,
        config,
    )
}

/// Fast path for a component that is globally a graph over one parameter.
pub(super) fn certify_regular_implicit_parameter_graph_with_selector(
    component: &BivariatePolynomial,
    selector: &ParameterComponentSelector2<'_>,
    selection_boundaries: &[BivariatePolynomial],
    retained_parameter: CurveResultantParameter,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<Option<ParameterComponentEvidence2>>> {
    let start_roots = match polynomial_unit_interval_roots(
        &bivariate_specialize_first(component, &Real::zero()),
        policy,
    )? {
        Classification::Decided(Some(roots)) if !roots.is_empty() => roots,
        Classification::Decided(_) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let branch_count = start_roots.len();
    let end_roots = match polynomial_unit_interval_roots(
        &bivariate_specialize_first(component, &Real::one()),
        policy,
    )? {
        Classification::Decided(Some(roots)) if roots.len() == branch_count => roots,
        Classification::Decided(_) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let retained_derivative =
        bivariate_parameter_derivative(component, CurveResultantParameter::First);
    let lifted_derivative =
        bivariate_parameter_derivative(component, CurveResultantParameter::Second);
    match bivariate_system_has_unit_square_solution(component, &lifted_derivative, policy, config)?
    {
        Classification::Decided(false) => {}
        Classification::Decided(true) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let turning_points = match bivariate_system_unit_square_solution_pairs(
        component,
        &retained_derivative,
        policy,
        config,
    )? {
        Classification::Decided(points) => points,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    for boundary_value in [Real::zero(), Real::one()] {
        let roots = match polynomial_unit_interval_roots(
            &bivariate_specialize_second(component, &boundary_value),
            policy,
        )? {
            Classification::Decided(Some(roots)) => roots,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match lifted_boundary_roots_are_turning_events(
            &roots,
            &boundary_value,
            &turning_points,
            policy,
        )? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }

    for boundary in selection_boundaries {
        match bivariate_system_has_unit_square_solution(component, boundary, policy, config)? {
            Classification::Decided(false) => {}
            Classification::Decided(true) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let mut component_branches = start_roots
        .into_iter()
        .zip(end_roots)
        .map(|(lifted_start, lifted_end)| {
            vec![
                ParameterComponentPoint {
                    retained_parameter: BezierParameter2::Exact(Real::zero()),
                    lifted_parameter: lifted_start,
                },
                ParameterComponentPoint {
                    retained_parameter: BezierParameter2::Exact(Real::one()),
                    lifted_parameter: lifted_end,
                },
            ]
        })
        .collect::<Vec<_>>();
    for point in turning_points {
        let rank =
            match parameter_component_point_root_rank(component, &point, branch_count, policy)? {
                Classification::Decided(Some(rank)) => rank,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        match insert_parameter_component_point(
            &mut component_branches[rank],
            ParameterComponentPoint {
                retained_parameter: point.parallel_parameter,
                lifted_parameter: point.other_parameter,
            },
            policy,
        )? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }

    let mut overlap_drafts = Vec::new();
    for (branch_rank, boundaries) in component_branches.iter().enumerate() {
        for window in boundaries.windows(2) {
            let [start, end] = window else {
                unreachable!("component boundaries are visited in pairs")
            };
            let direction = match start
                .lifted_parameter
                .cmp_by_refinement_with_policy(&end.lifted_parameter, policy)?
            {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Decided(direction) => direction,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let retained_sample = match start
                .retained_parameter
                .strict_scalar_between_ordered(&end.retained_parameter, policy)?
            {
                Classification::Decided(sample) => sample,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let sample_roots = match polynomial_unit_interval_roots(
                &bivariate_specialize_first(component, &retained_sample),
                policy,
            )? {
                Classification::Decided(Some(roots)) if roots.len() == branch_count => roots,
                Classification::Decided(_) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let lifted_sample = sample_roots[branch_rank].clone();
            let selected_branch = match selector.selected_at(
                retained_parameter,
                &BezierParameter2::Exact(retained_sample.clone()),
                &lifted_sample,
                policy,
            )? {
                Classification::Decided(selected) => selected,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if selected_branch {
                overlap_drafts.push(ParameterComponentOverlapDraft2 {
                    overlap: parameter_component_overlap_from_domain(
                        retained_parameter,
                        ParameterComponentDomain {
                            retained_start: start.retained_parameter.clone(),
                            retained_end: end.retained_parameter.clone(),
                            lifted_start: start.lifted_parameter.clone(),
                            lifted_end: end.lifted_parameter.clone(),
                        },
                        direction,
                    ),
                    witness: rational_parameter_component_pair(
                        retained_parameter,
                        ParameterComponentPoint {
                            retained_parameter: BezierParameter2::Exact(retained_sample),
                            lifted_parameter: lifted_sample,
                        },
                    ),
                });
            }
        }
    }
    let support = if retained_parameter == CurveResultantParameter::Second {
        bivariate_swap_parameters(component)
    } else {
        component.clone()
    };
    Ok(
        match parameter_component_evidence_from_drafts(
            overlap_drafts,
            &support,
            Vec::new(),
            Vec::new(),
            policy,
        )? {
            Classification::Decided(evidence) => Classification::Decided(Some(evidence)),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

/// General exact cell decomposition for a finite-event implicit component.
///
/// The retained-axis resultant critical fibers, lifted-axis turning fibers,
/// and all four authored-domain boundaries form a cylindrical decomposition.
/// Between consecutive retained fibers every unit-square root is a simple
/// ordered graph. Exact fiber counts isolate each event, and sufficiently near
/// rational side fibers certify its incidences without sampling topology.
pub(super) fn certify_regular_implicit_parameter_cells_with_selector(
    component: &BivariatePolynomial,
    selector: &ParameterComponentSelector2<'_>,
    selection_boundaries: &[BivariatePolynomial],
    retained_parameter: CurveResultantParameter,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<Option<ParameterComponentEvidence2>>> {
    for boundary in [Real::zero(), Real::one()] {
        for specialized in [
            bivariate_specialize_first(component, &boundary),
            bivariate_specialize_second(component, &boundary),
        ] {
            match polynomial_coefficients_are_identically_zero(&specialized, policy) {
                Classification::Decided(true) => return Ok(Classification::Decided(None)),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }
    let retained_derivative =
        bivariate_parameter_derivative(component, CurveResultantParameter::First);
    let lifted_derivative =
        bivariate_parameter_derivative(component, CurveResultantParameter::Second);
    let folds = match bivariate_system_unit_square_solution_pairs(
        component,
        &lifted_derivative,
        policy,
        config,
    )? {
        Classification::Decided(points) => points,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mut critical_points = Vec::with_capacity(folds.len());
    for point in folds {
        let singular = match signed_bivariate_at_parameter_pair(
            &retained_derivative,
            &point.parallel_parameter,
            &point.other_parameter,
            policy,
        )? {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => false,
            Classification::Decided(RealSign::Zero) => true,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        critical_points.push((point, singular));
    }
    let turns = match bivariate_system_unit_square_solution_pairs(
        component,
        &retained_derivative,
        policy,
        config,
    )? {
        Classification::Decided(points) => points,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    critical_points.reserve(turns.len());
    for point in turns {
        let singular = match signed_bivariate_at_parameter_pair(
            &lifted_derivative,
            &point.parallel_parameter,
            &point.other_parameter,
            policy,
        )? {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => false,
            Classification::Decided(RealSign::Zero) => true,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        critical_points.push((point, singular));
    }
    let mut selection_events = Vec::new();
    for boundary in selection_boundaries {
        match bivariate_system_has_unit_square_solution(component, boundary, policy, config)? {
            Classification::Decided(false) => continue,
            Classification::Decided(true) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let points =
            match bivariate_system_unit_square_solution_pairs(component, boundary, policy, config)?
            {
                Classification::Decided(points) => points,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        for point in points {
            if !selection_events.contains(&point) {
                selection_events.push(point);
            }
        }
    }

    let mut fibers = Vec::new();
    for retained_boundary in [Real::zero(), Real::one()] {
        let retained_boundary = BezierParameter2::Exact(retained_boundary);
        match ensure_implicit_parameter_component_fiber(&mut fibers, retained_boundary, policy)? {
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    for (point, singular) in critical_points {
        let event = ImplicitParameterComponentEvent {
            point: ParameterComponentPoint {
                retained_parameter: point.parallel_parameter,
                lifted_parameter: point.other_parameter,
            },
            domain_boundary: false,
            singular,
            selection_boundary: false,
        };
        match insert_implicit_parameter_component_event(&mut fibers, event, policy)? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    for point in selection_events {
        let event = ImplicitParameterComponentEvent {
            point: ParameterComponentPoint {
                retained_parameter: point.parallel_parameter,
                lifted_parameter: point.other_parameter,
            },
            domain_boundary: false,
            singular: false,
            selection_boundary: true,
        };
        match insert_implicit_parameter_component_event(&mut fibers, event, policy)? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    for retained_boundary in [Real::zero(), Real::one()] {
        let roots = match polynomial_unit_interval_roots(
            &bivariate_specialize_first(component, &retained_boundary),
            policy,
        )? {
            Classification::Decided(Some(roots)) => roots,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        for lifted_parameter in roots {
            let event = ImplicitParameterComponentEvent {
                point: ParameterComponentPoint {
                    retained_parameter: BezierParameter2::Exact(retained_boundary.clone()),
                    lifted_parameter,
                },
                domain_boundary: true,
                singular: false,
                selection_boundary: false,
            };
            match insert_implicit_parameter_component_event(&mut fibers, event, policy)? {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }
    for lifted_boundary in [Real::zero(), Real::one()] {
        let roots = match polynomial_unit_interval_roots(
            &bivariate_specialize_second(component, &lifted_boundary),
            policy,
        )? {
            Classification::Decided(Some(roots)) => roots,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        for retained_parameter in roots {
            let event = ImplicitParameterComponentEvent {
                point: ParameterComponentPoint {
                    retained_parameter,
                    lifted_parameter: BezierParameter2::Exact(lifted_boundary.clone()),
                },
                domain_boundary: true,
                singular: false,
                selection_boundary: false,
            };
            match insert_implicit_parameter_component_event(&mut fibers, event, policy)? {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }

    let mut overlap_drafts = Vec::new();
    let mut selected_pairs = Vec::new();
    let mut excluded_pairs = Vec::new();
    let mut active_tracks: Vec<Option<ImplicitParameterTrack>> = Vec::new();
    for fiber_index in 0..fibers.len() {
        let neighborhoods = match implicit_parameter_event_neighborhoods(
            component,
            &fibers[fiber_index],
            policy,
        )? {
            Classification::Decided(Some(neighborhoods)) => neighborhoods,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let incidence = match implicit_parameter_fiber_incidence(
            component,
            &fibers,
            fiber_index,
            &neighborhoods,
            policy,
        )? {
            Classification::Decided(Some(incidence)) => incidence,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let left_root_count = incidence.left.as_ref().map_or(0, |side| side.roots.len());
        if active_tracks.len() != left_root_count {
            return Ok(Classification::Decided(None));
        }
        let right_root_count = incidence.right.as_ref().map_or(0, |side| side.roots.len());
        let mut next_tracks: Vec<Option<ImplicitParameterTrack>> = std::iter::repeat_with(|| None)
            .take(right_root_count)
            .collect();

        if let (Some(left), Some(right)) = (&incidence.left, &incidence.right) {
            for (left_ranks, right_ranks) in left.gap_ranks.iter().zip(&right.gap_ranks) {
                if left_ranks.len() != right_ranks.len() {
                    return Ok(Classification::Decided(None));
                }
                for (&left_rank, &right_rank) in left_ranks.iter().zip(right_ranks) {
                    let Some(track) = active_tracks[left_rank].take() else {
                        return Ok(Classification::Decided(None));
                    };
                    if next_tracks[right_rank].replace(track).is_some() {
                        return Ok(Classification::Decided(None));
                    }
                }
            }
        }

        for (event_index, event) in fibers[fiber_index].events.iter().enumerate() {
            let event_selected = match selector.selected_at(
                retained_parameter,
                &event.point.retained_parameter,
                &event.point.lifted_parameter,
                policy,
            )? {
                Classification::Decided(selected) => selected,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if event.selection_boundary {
                push_unique_parameter_component_pair(
                    if event_selected {
                        &mut selected_pairs
                    } else {
                        &mut excluded_pairs
                    },
                    retained_parameter,
                    event.point.clone(),
                );
            }
            let left_ranks = incidence
                .left
                .as_ref()
                .map_or(&[][..], |side| side.event_ranks[event_index].as_slice());
            let right_ranks = incidence
                .right
                .as_ref()
                .map_or(&[][..], |side| side.event_ranks[event_index].as_slice());
            for &rank in left_ranks {
                let Some(track) = active_tracks[rank].take() else {
                    return Ok(Classification::Decided(None));
                };
                match finish_implicit_parameter_track(
                    track,
                    &event.point,
                    event_selected,
                    retained_parameter,
                    &mut overlap_drafts,
                    policy,
                )? {
                    Classification::Decided(Some(())) => {}
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            for &rank in right_ranks {
                let Some(right) = incidence.right.as_ref() else {
                    return Ok(Classification::Decided(None));
                };
                let track = match implicit_parameter_track_from_side(
                    selector,
                    retained_parameter,
                    &event.point,
                    right,
                    rank,
                    event_selected,
                    policy,
                )? {
                    Classification::Decided(Some(track)) => track,
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if next_tracks[rank].replace(track).is_some() {
                    return Ok(Classification::Decided(None));
                }
            }
            if left_ranks.is_empty() && right_ranks.is_empty() {
                if event.selection_boundary {
                    continue;
                }
                if event_selected {
                    push_unique_parameter_component_pair(
                        &mut selected_pairs,
                        retained_parameter,
                        event.point.clone(),
                    );
                }
            }
        }
        if active_tracks.iter().any(Option::is_some) || next_tracks.iter().any(Option::is_none) {
            return Ok(Classification::Decided(None));
        }
        active_tracks = next_tracks;
    }
    if !active_tracks.is_empty() {
        return Ok(Classification::Decided(None));
    }
    let support = if retained_parameter == CurveResultantParameter::Second {
        bivariate_swap_parameters(component)
    } else {
        component.clone()
    };
    Ok(
        match parameter_component_evidence_from_drafts(
            overlap_drafts,
            &support,
            selected_pairs,
            excluded_pairs,
            policy,
        )? {
            Classification::Decided(evidence) => Classification::Decided(Some(evidence)),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

pub(super) fn polynomial_coefficients_are_identically_zero(
    coefficients: &[Real],
    policy: &CurveContext,
) -> Classification<bool> {
    let mut uncertain = false;
    for coefficient in coefficients {
        match real_sign(coefficient, policy) {
            Some(RealSign::Zero) => {}
            Some(RealSign::Positive | RealSign::Negative) => {
                return Classification::Decided(false);
            }
            None => uncertain = true,
        }
    }
    if uncertain {
        Classification::Uncertain(UncertaintyReason::RealSign)
    } else {
        Classification::Decided(true)
    }
}

pub(super) fn ensure_implicit_parameter_component_fiber(
    fibers: &mut Vec<ImplicitParameterComponentFiber>,
    retained_parameter: BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<usize>> {
    for index in 0..fibers.len() {
        match retained_parameter
            .cmp_by_refinement_with_policy(&fibers[index].retained_parameter, policy)?
        {
            Classification::Decided(std::cmp::Ordering::Less) => {
                fibers.insert(
                    index,
                    ImplicitParameterComponentFiber {
                        retained_parameter,
                        events: Vec::new(),
                    },
                );
                return Ok(Classification::Decided(index));
            }
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return Ok(Classification::Decided(index));
            }
            Classification::Decided(std::cmp::Ordering::Greater) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    fibers.push(ImplicitParameterComponentFiber {
        retained_parameter,
        events: Vec::new(),
    });
    Ok(Classification::Decided(fibers.len() - 1))
}

pub(super) fn insert_implicit_parameter_component_event(
    fibers: &mut Vec<ImplicitParameterComponentFiber>,
    mut event: ImplicitParameterComponentEvent,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let fiber_index = match ensure_implicit_parameter_component_fiber(
        fibers,
        event.point.retained_parameter.clone(),
        policy,
    )? {
        Classification::Decided(index) => index,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    event.point.retained_parameter = fibers[fiber_index].retained_parameter.clone();
    let events = &mut fibers[fiber_index].events;
    for index in 0..events.len() {
        match event
            .point
            .lifted_parameter
            .cmp_by_refinement_with_policy(&events[index].point.lifted_parameter, policy)?
        {
            Classification::Decided(std::cmp::Ordering::Less) => {
                events.insert(index, event);
                return Ok(Classification::Decided(()));
            }
            Classification::Decided(std::cmp::Ordering::Equal) => {
                events[index].domain_boundary |= event.domain_boundary;
                events[index].singular |= event.singular;
                events[index].selection_boundary |= event.selection_boundary;
                return Ok(Classification::Decided(()));
            }
            Classification::Decided(std::cmp::Ordering::Greater) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    events.push(event);
    Ok(Classification::Decided(()))
}

pub(super) fn implicit_parameter_event_neighborhoods(
    component: &BivariatePolynomial,
    fiber: &ImplicitParameterComponentFiber,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<ImplicitParameterEventNeighborhood>>>> {
    let mut neighborhoods = Vec::with_capacity(fiber.events.len());
    for event in &fiber.events {
        match implicit_parameter_event_neighborhood(&event.point.lifted_parameter, policy)? {
            Classification::Decided(Some(neighborhood)) => neighborhoods.push(neighborhood),
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    loop {
        for neighborhood in &mut neighborhoods {
            match refine_implicit_parameter_event_neighborhood(neighborhood, policy)? {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let mut disjoint = true;
        for pair in neighborhoods.windows(2) {
            if compare_reals(&pair[0].upper, &pair[1].lower, policy)
                != Some(std::cmp::Ordering::Less)
            {
                disjoint = false;
                break;
            }
        }
        if !disjoint {
            continue;
        }
        let mut isolated = true;
        for neighborhood in &neighborhoods {
            match implicit_parameter_fiber_root_count(
                component,
                &fiber.retained_parameter,
                &neighborhood.lower,
                &neighborhood.upper,
                policy,
            )? {
                Classification::Decided(Some(1)) => {}
                Classification::Decided(Some(_)) => isolated = false,
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if isolated {
            return Ok(Classification::Decided(Some(neighborhoods)));
        }
    }
}

pub(super) fn implicit_parameter_event_neighborhood(
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<ImplicitParameterEventNeighborhood>>> {
    match parameter {
        BezierParameter2::Exact(_) => Ok(Classification::Decided(Some(
            ImplicitParameterEventNeighborhood {
                parameter: parameter.clone(),
                lower: Real::zero(),
                upper: Real::one(),
            },
        ))),
        BezierParameter2::Algebraic(algebraic) => {
            let lower = algebraic.interval().start().clone();
            let upper = algebraic.interval().end().clone();
            if lower.exact_rational_ref().is_none() || upper.exact_rational_ref().is_none() {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            if compare_reals(&lower, &upper, policy) != Some(std::cmp::Ordering::Less) {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
            Ok(Classification::Decided(Some(
                ImplicitParameterEventNeighborhood {
                    parameter: parameter.clone(),
                    lower,
                    upper,
                },
            )))
        }
    }
}

pub(super) fn refine_implicit_parameter_event_neighborhood(
    neighborhood: &mut ImplicitParameterEventNeighborhood,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    match &neighborhood.parameter {
        BezierParameter2::Exact(value) => {
            match compare_reals(&neighborhood.lower, value, policy) {
                Some(std::cmp::Ordering::Less) => {
                    neighborhood.lower = ((&neighborhood.lower + value) / Real::from(2_i8))?;
                }
                Some(std::cmp::Ordering::Equal) => {}
                Some(std::cmp::Ordering::Greater) | None => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
                }
            }
            match compare_reals(value, &neighborhood.upper, policy) {
                Some(std::cmp::Ordering::Less) => {
                    neighborhood.upper = ((value + &neighborhood.upper) / Real::from(2_i8))?;
                }
                Some(std::cmp::Ordering::Equal) => {}
                Some(std::cmp::Ordering::Greater) | None => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
                }
            }
        }
        BezierParameter2::Algebraic(_) => {
            neighborhood.parameter = neighborhood
                .parameter
                .clone()
                .refined_isolating_interval(8, policy);
            match &neighborhood.parameter {
                BezierParameter2::Exact(_) => {}
                BezierParameter2::Algebraic(algebraic) => {
                    neighborhood.lower = algebraic.interval().start().clone();
                    neighborhood.upper = algebraic.interval().end().clone();
                }
            }
        }
    }
    Ok(Classification::Decided(()))
}

pub(super) fn implicit_parameter_fiber_root_count(
    component: &BivariatePolynomial,
    retained_parameter: &BezierParameter2,
    lifted_lower: &Real,
    lifted_upper: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<usize>>> {
    if let BezierParameter2::Exact(retained_parameter) = retained_parameter {
        let roots = match polynomial_unit_interval_roots(
            &bivariate_specialize_first(component, retained_parameter),
            policy,
        )? {
            Classification::Decided(Some(roots)) => roots,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let lower = BezierParameter2::Exact(lifted_lower.clone());
        let upper = BezierParameter2::Exact(lifted_upper.clone());
        let mut count = 0;
        for root in roots {
            let lower_order = match root.cmp_by_refinement_with_policy(&lower, policy)? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let upper_order = match root.cmp_by_refinement_with_policy(&upper, policy)? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if lower_order != std::cmp::Ordering::Less && upper_order != std::cmp::Ordering::Greater
            {
                count += 1;
            }
        }
        return Ok(Classification::Decided(Some(count)));
    }

    let BezierParameter2::Algebraic(retained_parameter) = retained_parameter else {
        unreachable!("exact retained fibers returned above")
    };
    if lifted_lower.exact_rational_ref().is_none() || lifted_upper.exact_rational_ref().is_none() {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let representation = parameter_representation(retained_parameter, policy);
    let report = count_bivariate_fiber_roots_at_algebraic_parameter_closed(
        component,
        CurveResultantParameter::First,
        &representation,
        lifted_lower,
        lifted_upper,
        policy.predicate_policy(),
    );
    if report.certainty == PredicateCertainty::Approximate {
        policy.observe_approximate_512();
    }
    Ok(match report.status {
        AlgebraicFiberRootCountStatus::Counted => {
            Classification::Decided(report.distinct_root_count)
        }
        AlgebraicFiberRootCountStatus::IdenticallyZeroFiber => Classification::Decided(None),
        AlgebraicFiberRootCountStatus::EndpointRoot
        | AlgebraicFiberRootCountStatus::InvalidEvidence
        | AlgebraicFiberRootCountStatus::InvalidInterval
        | AlgebraicFiberRootCountStatus::UnsupportedCoefficient
        | AlgebraicFiberRootCountStatus::Undecided => {
            Classification::Uncertain(UncertaintyReason::Predicate)
        }
    })
}

pub(super) fn implicit_parameter_fiber_incidence(
    component: &BivariatePolynomial,
    fibers: &[ImplicitParameterComponentFiber],
    fiber_index: usize,
    neighborhoods: &[ImplicitParameterEventNeighborhood],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<ImplicitParameterFiberIncidence>>> {
    let fiber = &fibers[fiber_index];
    let mut left_sample = if fiber_index == 0 {
        None
    } else {
        match fibers[fiber_index - 1]
            .retained_parameter
            .strict_scalar_between_ordered(&fiber.retained_parameter, policy)?
        {
            Classification::Decided(sample) => Some(sample),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    };
    let mut right_sample = if fiber_index + 1 == fibers.len() {
        None
    } else {
        match fiber
            .retained_parameter
            .strict_scalar_between_ordered(&fibers[fiber_index + 1].retained_parameter, policy)?
        {
            Classification::Decided(sample) => Some(sample),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    };

    let mut retained_refinement =
        BezierParameterRefinement2::new(&fiber.retained_parameter, policy);
    let mut retained_refinement_steps = 0_usize;
    loop {
        let left = match left_sample.as_ref() {
            Some(sample) => match implicit_parameter_fiber_side(
                component,
                &fiber.retained_parameter,
                sample,
                neighborhoods,
                policy,
            )? {
                Classification::Decided(ImplicitParameterFiberSideAttempt::Certified(side)) => {
                    Some(side)
                }
                Classification::Decided(ImplicitParameterFiberSideAttempt::Retry) => None,
                Classification::Decided(ImplicitParameterFiberSideAttempt::Boundary) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
            None => None,
        };
        let right = match right_sample.as_ref() {
            Some(sample) => match implicit_parameter_fiber_side(
                component,
                &fiber.retained_parameter,
                sample,
                neighborhoods,
                policy,
            )? {
                Classification::Decided(ImplicitParameterFiberSideAttempt::Certified(side)) => {
                    Some(side)
                }
                Classification::Decided(ImplicitParameterFiberSideAttempt::Retry) => None,
                Classification::Decided(ImplicitParameterFiberSideAttempt::Boundary) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
            None => None,
        };
        let left_ready = left_sample.is_none() || left.is_some();
        let right_ready = right_sample.is_none() || right.is_some();
        if left_ready
            && right_ready
            && implicit_parameter_fiber_incidence_counts_are_valid(fiber, &left, &right)
        {
            return Ok(Classification::Decided(Some(
                ImplicitParameterFiberIncidence { left, right },
            )));
        }

        retained_refinement_steps = retained_refinement_steps.saturating_add(8);
        let refined_retained = retained_refinement
            .refine_to(retained_refinement_steps)
            .clone();
        if let Some(sample) = left_sample.take() {
            left_sample = match BezierParameter2::Exact(sample)
                .strict_scalar_between_ordered(&refined_retained, policy)?
            {
                Classification::Decided(sample) => Some(sample),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        }
        if let Some(sample) = right_sample.take() {
            right_sample = match refined_retained
                .strict_scalar_between_ordered(&BezierParameter2::Exact(sample), policy)?
            {
                Classification::Decided(sample) => Some(sample),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        }
    }
}

pub(super) fn implicit_parameter_fiber_side(
    component: &BivariatePolynomial,
    retained_parameter: &BezierParameter2,
    sample: &Real,
    neighborhoods: &[ImplicitParameterEventNeighborhood],
    policy: &CurveContext,
) -> CurveResult<Classification<ImplicitParameterFiberSideAttempt>> {
    let sample_parameter = BezierParameter2::Exact(sample.clone());
    let (range_start, range_end) =
        match sample_parameter.cmp_by_refinement_with_policy(retained_parameter, policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => {
                (&sample_parameter, retained_parameter)
            }
            Classification::Decided(std::cmp::Ordering::Greater) => {
                (retained_parameter, &sample_parameter)
            }
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    for neighborhood in neighborhoods {
        for lifted_boundary in [&neighborhood.lower, &neighborhood.upper] {
            match polynomial_is_rootless_on_open_parameter_range(
                &bivariate_specialize_second(component, lifted_boundary),
                range_start,
                range_end,
                policy,
            )? {
                Classification::Decided(Some(true)) => {}
                Classification::Decided(Some(false)) => {
                    return Ok(Classification::Decided(
                        ImplicitParameterFiberSideAttempt::Retry,
                    ));
                }
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(
                        ImplicitParameterFiberSideAttempt::Boundary,
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }

    let roots = match polynomial_unit_interval_roots(
        &bivariate_specialize_first(component, sample),
        policy,
    )? {
        Classification::Decided(Some(roots)) => roots,
        Classification::Decided(None) => {
            return Ok(Classification::Decided(
                ImplicitParameterFiberSideAttempt::Boundary,
            ));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let zero = BezierParameter2::Exact(Real::zero());
    let one = BezierParameter2::Exact(Real::one());
    let mut event_ranks = vec![Vec::new(); neighborhoods.len()];
    let mut gap_ranks = vec![Vec::new(); neighborhoods.len() + 1];
    for (rank, root) in roots.iter().enumerate() {
        for boundary in [&zero, &one] {
            match root.cmp_by_refinement_with_policy(boundary, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(
                        ImplicitParameterFiberSideAttempt::Boundary,
                    ));
                }
                Classification::Decided(_) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let mut placed = false;
        for (event_index, neighborhood) in neighborhoods.iter().enumerate() {
            let lower = BezierParameter2::Exact(neighborhood.lower.clone());
            match root.cmp_by_refinement_with_policy(&lower, policy)? {
                Classification::Decided(std::cmp::Ordering::Less) => {
                    gap_ranks[event_index].push(rank);
                    placed = true;
                    break;
                }
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(
                        ImplicitParameterFiberSideAttempt::Retry,
                    ));
                }
                Classification::Decided(std::cmp::Ordering::Greater) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let upper = BezierParameter2::Exact(neighborhood.upper.clone());
            match root.cmp_by_refinement_with_policy(&upper, policy)? {
                Classification::Decided(std::cmp::Ordering::Less) => {
                    event_ranks[event_index].push(rank);
                    placed = true;
                    break;
                }
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(
                        ImplicitParameterFiberSideAttempt::Retry,
                    ));
                }
                Classification::Decided(std::cmp::Ordering::Greater) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if !placed {
            gap_ranks[neighborhoods.len()].push(rank);
        }
    }
    Ok(Classification::Decided(
        ImplicitParameterFiberSideAttempt::Certified(ImplicitParameterFiberSide {
            sample: sample.clone(),
            roots,
            event_ranks,
            gap_ranks,
        }),
    ))
}

pub(super) fn implicit_parameter_fiber_incidence_counts_are_valid(
    fiber: &ImplicitParameterComponentFiber,
    left: &Option<ImplicitParameterFiberSide>,
    right: &Option<ImplicitParameterFiberSide>,
) -> bool {
    for event_index in 0..fiber.events.len() {
        let incidence_count = left
            .as_ref()
            .map_or(0, |side| side.event_ranks[event_index].len())
            + right
                .as_ref()
                .map_or(0, |side| side.event_ranks[event_index].len());
        if fiber.events[event_index].singular {
            if !fiber.events[event_index].domain_boundary && !incidence_count.is_multiple_of(2) {
                return false;
            }
            continue;
        }
        if if fiber.events[event_index].domain_boundary {
            incidence_count > 2
        } else {
            incidence_count != 2
        } {
            return false;
        }
    }
    match (left, right) {
        (Some(left), Some(right)) => left
            .gap_ranks
            .iter()
            .zip(&right.gap_ranks)
            .all(|(left, right)| left.len() == right.len()),
        (Some(side), None) | (None, Some(side)) => side.gap_ranks.iter().all(Vec::is_empty),
        (None, None) => true,
    }
}

pub(super) fn implicit_parameter_track_from_side(
    selector: &ParameterComponentSelector2<'_>,
    retained_parameter: CurveResultantParameter,
    start: &ParameterComponentPoint,
    side: &ImplicitParameterFiberSide,
    rank: usize,
    start_included: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<ImplicitParameterTrack>>> {
    let retained_sample = BezierParameter2::Exact(side.sample.clone());
    let selected = match selector.selected_at(
        retained_parameter,
        &retained_sample,
        &side.roots[rank],
        policy,
    )? {
        Classification::Decided(selected) => selected,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(Some(ImplicitParameterTrack {
        start: start.clone(),
        witness: rational_parameter_component_pair(
            retained_parameter,
            ParameterComponentPoint {
                retained_parameter: retained_sample,
                lifted_parameter: side.roots[rank].clone(),
            },
        ),
        selected,
        start_included,
    })))
}

pub(super) fn finish_implicit_parameter_track(
    track: ImplicitParameterTrack,
    end: &ParameterComponentPoint,
    end_included: bool,
    retained_parameter: CurveResultantParameter,
    overlaps: &mut Vec<ParameterComponentOverlapDraft2>,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<()>>> {
    match track
        .start
        .retained_parameter
        .cmp_by_refinement_with_policy(&end.retained_parameter, policy)?
    {
        Classification::Decided(std::cmp::Ordering::Less) => {}
        Classification::Decided(_) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    let direction = match track
        .start
        .lifted_parameter
        .cmp_by_refinement_with_policy(&end.lifted_parameter, policy)?
    {
        Classification::Decided(std::cmp::Ordering::Less) => std::cmp::Ordering::Less,
        Classification::Decided(std::cmp::Ordering::Greater) => std::cmp::Ordering::Greater,
        Classification::Decided(std::cmp::Ordering::Equal) => {
            return Ok(Classification::Decided(None));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if track.selected {
        overlaps.push(ParameterComponentOverlapDraft2 {
            overlap: parameter_component_overlap_from_domain_with_endpoint_inclusion(
                retained_parameter,
                ParameterComponentDomain {
                    retained_start: track.start.retained_parameter,
                    retained_end: end.retained_parameter.clone(),
                    lifted_start: track.start.lifted_parameter,
                    lifted_end: end.lifted_parameter.clone(),
                },
                direction,
                [track.start_included, end_included],
            ),
            witness: track.witness,
        });
    }
    Ok(Classification::Decided(Some(())))
}

pub(super) fn polynomial_is_rootless_on_open_parameter_range(
    coefficients: &[Real],
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<bool>>> {
    let polynomial = match polynomial_from_coefficients(coefficients.to_vec(), policy)? {
        Classification::Decided(Some(polynomial)) => polynomial,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let roots = match polynomial.isolate_unit_interval_roots_with_policy(policy)? {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    for root in roots {
        let start_order = match root.cmp_by_refinement_with_policy(start, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let end_order = match root.cmp_by_refinement_with_policy(end, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if start_order == std::cmp::Ordering::Greater && end_order == std::cmp::Ordering::Less {
            return Ok(Classification::Decided(Some(false)));
        }
    }
    Ok(Classification::Decided(Some(true)))
}

pub(super) fn parameter_component_point_root_rank(
    component: &BivariatePolynomial,
    point: &BezierParallelIntersectionParameterPair2,
    branch_count: usize,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<usize>>> {
    if let BezierParameter2::Exact(retained_parameter) = &point.parallel_parameter {
        let roots = match polynomial_unit_interval_roots(
            &bivariate_specialize_first(component, retained_parameter),
            policy,
        )? {
            Classification::Decided(Some(roots)) if roots.len() == branch_count => roots,
            Classification::Decided(_) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let mut blocker = None;
        for (rank, root) in roots.into_iter().enumerate() {
            match root.cmp_by_refinement_with_policy(&point.other_parameter, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(Some(rank)));
                }
                Classification::Decided(_) => {}
                Classification::Uncertain(reason) => blocker = Some(reason),
            }
        }
        return Ok(blocker.map_or(Classification::Decided(None), Classification::Uncertain));
    }

    let BezierParameter2::Algebraic(retained_parameter) = &point.parallel_parameter else {
        unreachable!("exact retained parameters returned above")
    };
    let retained_representation = parameter_representation(retained_parameter, policy);
    let rank = match &point.other_parameter {
        BezierParameter2::Exact(lifted_parameter) => {
            match compare_reals(lifted_parameter, &Real::zero(), policy) {
                Some(std::cmp::Ordering::Equal) => return Ok(Classification::Decided(Some(0))),
                Some(std::cmp::Ordering::Less) | None => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                }
                Some(std::cmp::Ordering::Greater) => {}
            }
            match compare_reals(lifted_parameter, &Real::one(), policy) {
                Some(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(Some(branch_count - 1)));
                }
                Some(std::cmp::Ordering::Greater) | None => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                }
                Some(std::cmp::Ordering::Less) => {}
            }
            if lifted_parameter.exact_rational_ref().is_none() {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            let mut lower = (lifted_parameter / Real::from(2_i8))?;
            let mut upper = ((lifted_parameter + Real::one()) / Real::from(2_i8))?;
            loop {
                match algebraic_fiber_root_rank_in_isolator(
                    component,
                    &retained_representation,
                    &lower,
                    &upper,
                    branch_count,
                    policy,
                )? {
                    Classification::Decided(Some(rank)) => break rank,
                    Classification::Decided(None) => {
                        lower = ((lower + lifted_parameter) / Real::from(2_i8))?;
                        upper = ((upper + lifted_parameter) / Real::from(2_i8))?;
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        BezierParameter2::Algebraic(_) => {
            let mut refinement = BezierParameterRefinement2::new(&point.other_parameter, policy);
            let mut refinement_steps = 0_usize;
            loop {
                let refined = refinement.refine_to(refinement_steps);
                match refined {
                    BezierParameter2::Exact(lifted_parameter) => {
                        return parameter_component_point_root_rank(
                            component,
                            &BezierParallelIntersectionParameterPair2 {
                                parallel_parameter: point.parallel_parameter.clone(),
                                other_parameter: BezierParameter2::Exact(lifted_parameter.clone()),
                            },
                            branch_count,
                            policy,
                        );
                    }
                    BezierParameter2::Algebraic(lifted_parameter) => {
                        match algebraic_fiber_root_rank_in_isolator(
                            component,
                            &retained_representation,
                            lifted_parameter.interval().start(),
                            lifted_parameter.interval().end(),
                            branch_count,
                            policy,
                        )? {
                            Classification::Decided(Some(rank)) => break rank,
                            Classification::Decided(None) => {
                                refinement_steps = refinement_steps.saturating_add(8);
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                }
            }
        }
    };
    Ok(Classification::Decided(Some(rank)))
}

pub(super) fn algebraic_fiber_root_rank_in_isolator(
    component: &BivariatePolynomial,
    retained_parameter: &hypersolve::AlgebraicRootRepresentation,
    lifted_lower: &Real,
    lifted_upper: &Real,
    branch_count: usize,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<usize>>> {
    if !matches!(
        compare_reals(&Real::zero(), lifted_lower, policy),
        Some(std::cmp::Ordering::Less)
    ) || !matches!(
        compare_reals(lifted_upper, &Real::one(), policy),
        Some(std::cmp::Ordering::Less)
    ) {
        return Ok(Classification::Decided(None));
    }
    let isolator_report = count_bivariate_fiber_roots_at_algebraic_parameter(
        component,
        CurveResultantParameter::First,
        retained_parameter,
        lifted_lower,
        lifted_upper,
        policy.predicate_policy(),
    );
    if isolator_report.certainty == PredicateCertainty::Approximate {
        policy.observe_approximate_512();
    }
    match isolator_report.status {
        AlgebraicFiberRootCountStatus::Counted => match isolator_report.distinct_root_count {
            Some(1) => {}
            Some(_) => return Ok(Classification::Decided(None)),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Predicate)),
        },
        AlgebraicFiberRootCountStatus::EndpointRoot => {
            return Ok(Classification::Decided(None));
        }
        AlgebraicFiberRootCountStatus::IdenticallyZeroFiber => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        AlgebraicFiberRootCountStatus::InvalidEvidence
        | AlgebraicFiberRootCountStatus::InvalidInterval
        | AlgebraicFiberRootCountStatus::UnsupportedCoefficient
        | AlgebraicFiberRootCountStatus::Undecided => {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
    }

    let rank_report = count_bivariate_fiber_roots_at_algebraic_parameter_closed(
        component,
        CurveResultantParameter::First,
        retained_parameter,
        &Real::zero(),
        lifted_lower,
        policy.predicate_policy(),
    );
    if rank_report.certainty == PredicateCertainty::Approximate {
        policy.observe_approximate_512();
    }
    Ok(match rank_report.status {
        AlgebraicFiberRootCountStatus::Counted => match rank_report.distinct_root_count {
            Some(rank) if rank < branch_count => Classification::Decided(Some(rank)),
            Some(_) => Classification::Decided(None),
            None => Classification::Uncertain(UncertaintyReason::Predicate),
        },
        AlgebraicFiberRootCountStatus::IdenticallyZeroFiber => {
            Classification::Uncertain(UncertaintyReason::Boundary)
        }
        AlgebraicFiberRootCountStatus::EndpointRoot
        | AlgebraicFiberRootCountStatus::InvalidEvidence
        | AlgebraicFiberRootCountStatus::InvalidInterval
        | AlgebraicFiberRootCountStatus::UnsupportedCoefficient
        | AlgebraicFiberRootCountStatus::Undecided => {
            Classification::Uncertain(UncertaintyReason::Predicate)
        }
    })
}

pub(super) fn insert_parameter_component_point(
    points: &mut Vec<ParameterComponentPoint>,
    point: ParameterComponentPoint,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let mut index = 0;
    while index < points.len() {
        match point
            .retained_parameter
            .cmp_by_refinement_with_policy(&points[index].retained_parameter, policy)?
        {
            Classification::Decided(std::cmp::Ordering::Less) => break,
            Classification::Decided(std::cmp::Ordering::Greater) => index += 1,
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return match point
                    .lifted_parameter
                    .cmp_by_refinement_with_policy(&points[index].lifted_parameter, policy)?
                {
                    Classification::Decided(std::cmp::Ordering::Equal) => {
                        Ok(Classification::Decided(()))
                    }
                    Classification::Decided(_) => {
                        Ok(Classification::Uncertain(UncertaintyReason::Boundary))
                    }
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                };
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    points.insert(index, point);
    Ok(Classification::Decided(()))
}

pub(super) fn lifted_boundary_roots_are_turning_events(
    roots: &[BezierParameter2],
    lifted_boundary: &Real,
    turning_points: &[BezierParallelIntersectionParameterPair2],
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let zero = BezierParameter2::Exact(Real::zero());
    let one = BezierParameter2::Exact(Real::one());
    let lifted_boundary = BezierParameter2::Exact(lifted_boundary.clone());
    for root in roots {
        match root.cmp_by_refinement_with_policy(&zero, policy)? {
            Classification::Decided(std::cmp::Ordering::Equal) => continue,
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        match root.cmp_by_refinement_with_policy(&one, policy)? {
            Classification::Decided(std::cmp::Ordering::Equal) => continue,
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        let mut blocker = None;
        let mut matched = false;
        for point in turning_points {
            match root.cmp_by_refinement_with_policy(&point.parallel_parameter, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    match lifted_boundary
                        .cmp_by_refinement_with_policy(&point.other_parameter, policy)?
                    {
                        Classification::Decided(std::cmp::Ordering::Equal) => {
                            matched = true;
                            break;
                        }
                        Classification::Decided(_) => {}
                        Classification::Uncertain(reason) => blocker = Some(reason),
                    }
                }
                Classification::Decided(_) => {}
                Classification::Uncertain(reason) => blocker = Some(reason),
            }
        }
        if !matched {
            return Ok(blocker.map_or(Classification::Decided(false), Classification::Uncertain));
        }
    }
    Ok(Classification::Decided(true))
}

pub(super) fn bivariate_polynomial_is_independent_of_parameter(
    polynomial: &BivariatePolynomial,
    parameter: CurveResultantParameter,
    policy: &CurveContext,
) -> Classification<bool> {
    let mut uncertain = false;
    for (first_power, row) in polynomial.coefficients.iter().enumerate() {
        for (second_power, coefficient) in row.iter().enumerate() {
            let depends = match parameter {
                CurveResultantParameter::First => first_power != 0,
                CurveResultantParameter::Second => second_power != 0,
            };
            if !depends {
                continue;
            }
            match real_sign(coefficient, policy) {
                Some(RealSign::Zero) => {}
                Some(RealSign::Positive | RealSign::Negative) => {
                    return Classification::Decided(false);
                }
                None => uncertain = true,
            }
        }
    }
    if uncertain {
        Classification::Uncertain(UncertaintyReason::RealSign)
    } else {
        Classification::Decided(true)
    }
}

pub(super) fn bivariate_system_has_unit_square_solution(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<bool>> {
    if bivariate_unit_square_has_strict_bernstein_sign(second, policy)?
        || bivariate_unit_square_has_strict_bernstein_sign(first, policy)?
    {
        return Ok(Classification::Decided(false));
    }
    let candidates = match project_parallel_intersection_system(
        first,
        second,
        [CurveParameterDomain2::new(&CurveParameterRange2::unit(), None); 2],
        policy,
    )? {
        Classification::Decided(candidates) => candidates,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let (parallel_parameters, other_parameters) = match candidates {
        CurveIntersectionCandidates2::Candidates {
            first_parameters: parallel_parameters,
            second_parameters: other_parameters,
        } => (parallel_parameters, other_parameters),
        CurveIntersectionCandidates2::NoIntersection => {
            return Ok(Classification::Decided(false));
        }
        CurveIntersectionCandidates2::DegenerateResultant => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
    };

    let mut blocker = None;
    let mut parameter_lifts = [None, None];
    for first_parameter in &parallel_parameters {
        for second_parameter in &other_parameters {
            match replay_bivariate_parameter_pair(
                first,
                second,
                first_parameter,
                second_parameter,
                policy,
                config,
                &mut parameter_lifts,
            )? {
                Classification::Decided(BivariateParameterPairReplay::Rejected) => {}
                Classification::Decided(
                    BivariateParameterPairReplay::Direct
                    | BivariateParameterPairReplay::LinearLift(_, _),
                ) => return Ok(Classification::Decided(true)),
                Classification::Uncertain(reason) => blocker = Some(reason),
            }
        }
    }
    Ok(blocker.map_or(Classification::Decided(false), Classification::Uncertain))
}

#[cold]
#[inline(never)]
pub(super) fn bivariate_system_has_positive_dimensional_relation(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    if bivariate_unit_square_has_strict_bernstein_sign(second, policy)?
        || bivariate_unit_square_has_strict_bernstein_sign(first, policy)?
    {
        return Ok(Classification::Decided(false));
    }
    Ok(
        match project_parallel_intersection_system(
            first,
            second,
            [CurveParameterDomain2::new(&CurveParameterRange2::unit(), None); 2],
            policy,
        )? {
            Classification::Decided(CurveIntersectionCandidates2::DegenerateResultant) => {
                Classification::Decided(true)
            }
            Classification::Decided(_) => Classification::Decided(false),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

pub(super) fn bivariate_system_unit_square_solution_pairs(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<Vec<BezierParallelIntersectionParameterPair2>>> {
    if bivariate_unit_square_has_strict_bernstein_sign(second, policy)?
        || bivariate_unit_square_has_strict_bernstein_sign(first, policy)?
    {
        return Ok(Classification::Decided(Vec::new()));
    }
    let candidates = match project_parallel_intersection_system(
        first,
        second,
        [CurveParameterDomain2::new(&CurveParameterRange2::unit(), None); 2],
        policy,
    )? {
        Classification::Decided(candidates) => candidates,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let (parallel_parameters, other_parameters) = match candidates {
        CurveIntersectionCandidates2::Candidates {
            first_parameters: parallel_parameters,
            second_parameters: other_parameters,
        } => (parallel_parameters, other_parameters),
        CurveIntersectionCandidates2::NoIntersection => {
            return Ok(Classification::Decided(Vec::new()));
        }
        CurveIntersectionCandidates2::DegenerateResultant => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
    };

    let mut blocker = None;
    let mut pairs = Vec::new();
    let mut parameter_lifts = [None, None];
    for first_parameter in parallel_parameters {
        for second_parameter in &other_parameters {
            match replay_bivariate_parameter_pair(
                first,
                second,
                &first_parameter,
                second_parameter,
                policy,
                config,
                &mut parameter_lifts,
            )? {
                Classification::Decided(BivariateParameterPairReplay::Rejected) => {}
                Classification::Decided(
                    BivariateParameterPairReplay::Direct
                    | BivariateParameterPairReplay::LinearLift(_, _),
                ) => {
                    let pair = BezierParallelIntersectionParameterPair2 {
                        parallel_parameter: first_parameter.clone(),
                        other_parameter: second_parameter.clone(),
                    };
                    if !pairs.contains(&pair) {
                        pairs.push(pair);
                    }
                }
                Classification::Uncertain(reason) => blocker = Some(reason),
            }
        }
    }
    Ok(blocker.map_or(Classification::Decided(pairs), Classification::Uncertain))
}

#[cfg(test)]
pub(super) fn certify_rational_parameter_component_map(
    equations: &[BivariatePolynomial; 2],
    branch: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    map: &CurveIntersectionParameterLiftMap,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<ParameterComponentEvidence2>>> {
    certify_rational_parameter_component_map_with_selector(
        equations,
        &ParameterComponentSelector2::Positive(branch, None),
        retained_parameter,
        map,
        policy,
    )
}

pub(super) fn certify_rational_parameter_component_map_with_selector(
    equations: &[BivariatePolynomial; 2],
    selector: &ParameterComponentSelector2<'_>,
    retained_parameter: CurveResultantParameter,
    map: &CurveIntersectionParameterLiftMap,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<ParameterComponentEvidence2>>> {
    let Some(support) = rational_parameter_component_support(
        retained_parameter,
        &map.numerator_coefficients,
        &map.denominator_coefficients,
    ) else {
        return Ok(Classification::Decided(None));
    };
    for equation in equations {
        let (cleared, _) = bivariate_on_parameter_lift_cleared(equation, retained_parameter, map);
        match polynomial_from_coefficients(cleared, policy)? {
            Classification::Decided(None) => {}
            Classification::Decided(Some(_)) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }

    let derivative_numerator = polynomial_subtract(
        &polynomial_multiply(
            &polynomial_derivative(&map.numerator_coefficients),
            &map.denominator_coefficients,
        ),
        &polynomial_multiply(
            &map.numerator_coefficients,
            &polynomial_derivative(&map.denominator_coefficients),
        ),
    );
    let mut parameter_image_map = RationalParameterImageMap2::new(
        map.numerator_coefficients.clone(),
        map.denominator_coefficients.clone(),
        policy,
    );
    let partition = match rational_parameter_component_domains(
        map,
        &derivative_numerator,
        &mut parameter_image_map,
        policy,
    )? {
        Classification::Decided(Some(partition)) => partition,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if partition.domains.is_empty() && partition.isolated_points.is_empty() {
        return Ok(Classification::Decided(Some(
            ParameterComponentEvidence2::default(),
        )));
    }
    let mut selection_events = Vec::new();
    for boundary in selector.boundary_polynomials() {
        let (coefficients, _) =
            bivariate_on_parameter_lift_cleared(&boundary, retained_parameter, map);
        let polynomial = match polynomial_from_coefficients(coefficients, policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            // A predicate identically zero on this component is not an event:
            // the selector's remaining exact signs decide every cell.  This is
            // essential for tangent-parallel pair components, where the cross
            // product vanishes along the complete correspondence.
            Classification::Decided(None) => continue,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let roots = match polynomial.isolate_unit_interval_roots_with_policy(policy)? {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        for root in roots {
            match insert_ordered_parameter_component_boundary(&mut selection_events, root, policy)?
            {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }

    let mut overlap_drafts = Vec::with_capacity(
        partition
            .domains
            .len()
            .saturating_add(selection_events.len()),
    );
    let mut selected_pairs = Vec::with_capacity(selection_events.len());
    let mut excluded_pairs = Vec::with_capacity(selection_events.len());
    for (domain, direction) in partition.domains {
        let mut segment_start = ParameterComponentPoint {
            retained_parameter: domain.retained_start.clone(),
            lifted_parameter: domain.lifted_start.clone(),
        };
        let mut start_included = true;
        let mut end_included = true;
        for root in &selection_events {
            match root.cmp_by_refinement_with_policy(&domain.retained_start, policy)? {
                Classification::Decided(std::cmp::Ordering::Less) => continue,
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    start_included = match partition_parameter_component_selector_point(
                        selector,
                        retained_parameter,
                        &ParameterComponentPoint {
                            retained_parameter: domain.retained_start.clone(),
                            lifted_parameter: domain.lifted_start.clone(),
                        },
                        &mut selected_pairs,
                        &mut excluded_pairs,
                        policy,
                    )? {
                        Classification::Decided(selected) => selected,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    continue;
                }
                Classification::Decided(std::cmp::Ordering::Greater) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            match root.cmp_by_refinement_with_policy(&domain.retained_end, policy)? {
                Classification::Decided(std::cmp::Ordering::Greater) => break,
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    end_included = match partition_parameter_component_selector_point(
                        selector,
                        retained_parameter,
                        &ParameterComponentPoint {
                            retained_parameter: domain.retained_end.clone(),
                            lifted_parameter: domain.lifted_end.clone(),
                        },
                        &mut selected_pairs,
                        &mut excluded_pairs,
                        policy,
                    )? {
                        Classification::Decided(selected) => selected,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    break;
                }
                Classification::Decided(std::cmp::Ordering::Less) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let lifted_parameter = match parameter_image_map.image(root)? {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let segment_end = ParameterComponentPoint {
                retained_parameter: root.clone(),
                lifted_parameter,
            };
            let event_included = match partition_parameter_component_selector_point(
                selector,
                retained_parameter,
                &segment_end,
                &mut selected_pairs,
                &mut excluded_pairs,
                policy,
            )? {
                Classification::Decided(selected) => selected,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match append_selected_rational_parameter_component_domain(
                &mut overlap_drafts,
                selector,
                retained_parameter,
                map,
                segment_start,
                segment_end.clone(),
                direction,
                [start_included, event_included],
                policy,
            )? {
                Classification::Decided(Some(())) => {}
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            segment_start = segment_end;
            start_included = event_included;
        }
        match append_selected_rational_parameter_component_domain(
            &mut overlap_drafts,
            selector,
            retained_parameter,
            map,
            segment_start,
            ParameterComponentPoint {
                retained_parameter: domain.retained_end,
                lifted_parameter: domain.lifted_end,
            },
            direction,
            [start_included, end_included],
            policy,
        )? {
            Classification::Decided(Some(())) => {}
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    for point in partition.isolated_points {
        match partition_parameter_component_selector_point(
            selector,
            retained_parameter,
            &point,
            &mut selected_pairs,
            &mut excluded_pairs,
            policy,
        )? {
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(
        match parameter_component_evidence_from_drafts(
            overlap_drafts,
            &support,
            selected_pairs,
            excluded_pairs,
            policy,
        )? {
            Classification::Decided(evidence) => Classification::Decided(Some(evidence)),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

pub(super) fn append_selected_rational_parameter_component_domain(
    overlaps: &mut Vec<ParameterComponentOverlapDraft2>,
    selector: &ParameterComponentSelector2<'_>,
    retained_parameter: CurveResultantParameter,
    map: &CurveIntersectionParameterLiftMap,
    start: ParameterComponentPoint,
    end: ParameterComponentPoint,
    direction: std::cmp::Ordering,
    endpoint_inclusion: [bool; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<()>>> {
    let retained_sample = match start
        .retained_parameter
        .strict_scalar_between_ordered(&end.retained_parameter, policy)?
    {
        Classification::Decided(sample) => sample,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let lifted_sample = match rational_parameter_map_at(map, &retained_sample, policy)? {
        Classification::Decided(sample) => sample,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let retained_sample = BezierParameter2::Exact(retained_sample);
    let lifted_sample = BezierParameter2::Exact(lifted_sample);
    let selected =
        match selector.selected_at(retained_parameter, &retained_sample, &lifted_sample, policy)? {
            Classification::Decided(selected) => selected,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    if selected {
        let witness = rational_parameter_component_pair(
            retained_parameter,
            ParameterComponentPoint {
                retained_parameter: retained_sample,
                lifted_parameter: lifted_sample,
            },
        );
        overlaps.push(ParameterComponentOverlapDraft2 {
            overlap: parameter_component_overlap_from_domain_with_endpoint_inclusion(
                retained_parameter,
                ParameterComponentDomain {
                    retained_start: start.retained_parameter,
                    retained_end: end.retained_parameter,
                    lifted_start: start.lifted_parameter,
                    lifted_end: end.lifted_parameter,
                },
                direction,
                endpoint_inclusion,
            ),
            witness,
        });
    }
    Ok(Classification::Decided(Some(())))
}

pub(super) fn insert_ordered_parameter_component_boundary(
    boundaries: &mut Vec<BezierParameter2>,
    boundary: BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    for index in 0..boundaries.len() {
        match boundary.cmp_by_refinement_with_policy(&boundaries[index], policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => {
                boundaries.insert(index, boundary);
                return Ok(Classification::Decided(()));
            }
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return Ok(Classification::Decided(()));
            }
            Classification::Decided(std::cmp::Ordering::Greater) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    boundaries.push(boundary);
    Ok(Classification::Decided(()))
}

pub(super) fn partition_parameter_component_selector_point(
    selector: &ParameterComponentSelector2<'_>,
    retained_parameter: CurveResultantParameter,
    point: &ParameterComponentPoint,
    selected_pairs: &mut Vec<BezierParallelIntersectionParameterPair2>,
    excluded_pairs: &mut Vec<BezierParallelIntersectionParameterPair2>,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let selected = match selector.selected_at(
        retained_parameter,
        &point.retained_parameter,
        &point.lifted_parameter,
        policy,
    )? {
        Classification::Decided(selected) => selected,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    push_unique_parameter_component_pair(
        if selected {
            selected_pairs
        } else {
            excluded_pairs
        },
        retained_parameter,
        point.clone(),
    );
    Ok(Classification::Decided(selected))
}

#[derive(Default)]
pub(super) struct ParameterComponentEvidence2 {
    pub(super) overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
    pub(super) component_overlaps: Arc<[BezierParameterComponentOverlap2]>,
    pub(super) component_pairs: Arc<[BezierParallelIntersectionParameterPair2]>,
    pub(super) selected_component_pair_count: usize,
}

impl ParameterComponentEvidence2 {
    pub(super) fn from_partitioned_pairs(
        overlaps: Vec<RationalBezierIntersectionOverlap2>,
        mut selected_pairs: Vec<BezierParallelIntersectionParameterPair2>,
        excluded_pairs: Vec<BezierParallelIntersectionParameterPair2>,
    ) -> Self {
        let selected_component_pair_count = selected_pairs.len();
        if selected_pairs.is_empty() {
            selected_pairs = excluded_pairs;
        } else {
            selected_pairs.extend(excluded_pairs);
        }
        Self {
            overlaps: overlaps.into(),
            component_overlaps: Arc::from([]),
            component_pairs: selected_pairs.into(),
            selected_component_pair_count,
        }
    }

    pub(super) fn selected_pairs(&self) -> &[BezierParallelIntersectionParameterPair2] {
        &self.component_pairs[..self.selected_component_pair_count]
    }

    pub(super) fn excluded_pairs(&self) -> &[BezierParallelIntersectionParameterPair2] {
        &self.component_pairs[self.selected_component_pair_count..]
    }
}

pub(super) fn merge_parameter_component_evidence(
    first: ParameterComponentEvidence2,
    second: ParameterComponentEvidence2,
) -> ParameterComponentEvidence2 {
    let mut overlaps = first.overlaps.to_vec();
    for overlap in second.overlaps.iter() {
        if !overlaps.contains(overlap) {
            overlaps.push(overlap.clone());
        }
    }
    let mut component_overlaps = first.component_overlaps.to_vec();
    for overlap in second.component_overlaps.iter() {
        if !component_overlaps.contains(overlap) {
            component_overlaps.push(overlap.clone());
        }
    }
    let mut selected_pairs = first.selected_pairs().to_vec();
    for pair in second.selected_pairs() {
        if !selected_pairs.contains(pair) {
            selected_pairs.push(pair.clone());
        }
    }
    let mut excluded_pairs = first.excluded_pairs().to_vec();
    for pair in second.excluded_pairs() {
        if !excluded_pairs.contains(pair) {
            excluded_pairs.push(pair.clone());
        }
    }
    let mut evidence = ParameterComponentEvidence2::from_partitioned_pairs(
        overlaps,
        selected_pairs,
        excluded_pairs,
    );
    evidence.component_overlaps = component_overlaps.into();
    evidence
}

pub(super) fn parameter_component_evidence_from_drafts(
    drafts: Vec<ParameterComponentOverlapDraft2>,
    support: &BivariatePolynomial,
    selected_pairs: Vec<BezierParallelIntersectionParameterPair2>,
    excluded_pairs: Vec<BezierParallelIntersectionParameterPair2>,
    policy: &CurveContext,
) -> CurveResult<Classification<ParameterComponentEvidence2>> {
    let support = Arc::new(support.clone());
    let mut overlaps = Vec::with_capacity(drafts.len());
    let mut component_overlaps = Vec::with_capacity(drafts.len());
    for draft in drafts {
        let first_rank = if draft.witness.parallel_parameter.scalar().is_some() {
            match parameter_component_fiber_root_rank(
                support.as_ref(),
                &draft.witness,
                CurveResultantParameter::First,
                policy,
            )? {
                Classification::Decided(rank) => rank,
                Classification::Uncertain(_) => UNKNOWN_PARAMETER_COMPONENT_FIBER_ROOT_RANK,
            }
        } else {
            UNKNOWN_PARAMETER_COMPONENT_FIBER_ROOT_RANK
        };
        let second_rank = if draft.witness.other_parameter.scalar().is_some() {
            match parameter_component_fiber_root_rank(
                support.as_ref(),
                &draft.witness,
                CurveResultantParameter::Second,
                policy,
            )? {
                Classification::Decided(rank) => rank,
                Classification::Uncertain(_) => UNKNOWN_PARAMETER_COMPONENT_FIBER_ROOT_RANK,
            }
        } else {
            UNKNOWN_PARAMETER_COMPONENT_FIBER_ROOT_RANK
        };
        overlaps.push(draft.overlap.clone());
        component_overlaps.push(BezierParameterComponentOverlap2 {
            overlap: draft.overlap,
            support: support.clone(),
            fiber_root_ranks: [first_rank, second_rank],
            witness: draft.witness,
            parameter_charts: None,
        });
    }
    let mut evidence = ParameterComponentEvidence2::from_partitioned_pairs(
        overlaps,
        selected_pairs,
        excluded_pairs,
    );
    evidence.component_overlaps = component_overlaps.into();
    Ok(Classification::Decided(evidence))
}

pub(super) fn parameter_component_fiber_root_rank(
    component: &BivariatePolynomial,
    witness: &BezierParallelIntersectionParameterPair2,
    retained_parameter: CurveResultantParameter,
    policy: &CurveContext,
) -> CurveResult<Classification<usize>> {
    let swapped_component;
    let swapped_witness;
    let (component, witness) = match retained_parameter {
        CurveResultantParameter::First => (component, witness),
        CurveResultantParameter::Second => {
            swapped_component = bivariate_swap_parameters(component);
            swapped_witness = BezierParallelIntersectionParameterPair2 {
                parallel_parameter: witness.other_parameter.clone(),
                other_parameter: witness.parallel_parameter.clone(),
            };
            (&swapped_component, &swapped_witness)
        }
    };
    let branch_count = match implicit_parameter_fiber_root_count(
        component,
        &witness.parallel_parameter,
        &Real::zero(),
        &Real::one(),
        policy,
    )? {
        Classification::Decided(Some(count)) if count != 0 => count,
        Classification::Decided(_) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    match parameter_component_point_root_rank(component, witness, branch_count, policy)? {
        Classification::Decided(Some(rank)) => Ok(Classification::Decided(rank)),
        Classification::Decided(None) => Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

#[derive(Clone)]
pub(super) struct ParameterComponentDomain {
    pub(super) retained_start: BezierParameter2,
    pub(super) retained_end: BezierParameter2,
    pub(super) lifted_start: BezierParameter2,
    pub(super) lifted_end: BezierParameter2,
}

#[derive(Clone)]
pub(super) struct ParameterComponentPoint {
    pub(super) retained_parameter: BezierParameter2,
    pub(super) lifted_parameter: BezierParameter2,
}

#[derive(Clone)]
pub(super) struct ImplicitParameterComponentEvent {
    pub(super) point: ParameterComponentPoint,
    pub(super) domain_boundary: bool,
    pub(super) singular: bool,
    pub(super) selection_boundary: bool,
}

pub(super) struct ImplicitParameterComponentFiber {
    pub(super) retained_parameter: BezierParameter2,
    pub(super) events: Vec<ImplicitParameterComponentEvent>,
}

pub(super) struct ImplicitParameterEventNeighborhood {
    pub(super) parameter: BezierParameter2,
    pub(super) lower: Real,
    pub(super) upper: Real,
}

pub(super) struct ImplicitParameterFiberSide {
    pub(super) sample: Real,
    pub(super) roots: Vec<BezierParameter2>,
    pub(super) event_ranks: Vec<Vec<usize>>,
    pub(super) gap_ranks: Vec<Vec<usize>>,
}

pub(super) enum ImplicitParameterFiberSideAttempt {
    Certified(ImplicitParameterFiberSide),
    Retry,
    Boundary,
}

pub(super) struct ImplicitParameterFiberIncidence {
    pub(super) left: Option<ImplicitParameterFiberSide>,
    pub(super) right: Option<ImplicitParameterFiberSide>,
}

pub(super) struct ImplicitParameterTrack {
    pub(super) start: ParameterComponentPoint,
    pub(super) witness: BezierParallelIntersectionParameterPair2,
    pub(super) selected: bool,
    pub(super) start_included: bool,
}

pub(super) struct RationalParameterComponentPartition {
    pub(super) domains: Vec<(ParameterComponentDomain, std::cmp::Ordering)>,
    pub(super) isolated_points: Vec<ParameterComponentPoint>,
}

#[derive(Clone)]
pub(super) struct RationalParameterComponentBoundary {
    pub(super) parameter: BezierParameter2,
    pub(super) denominator_root: bool,
    pub(super) zero_image: bool,
    pub(super) unit_image: bool,
}

pub(super) fn rational_parameter_component_domains(
    map: &CurveIntersectionParameterLiftMap,
    derivative_numerator: &[Real],
    parameter_image_map: &mut RationalParameterImageMap2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<RationalParameterComponentPartition>>> {
    let denominator_roots =
        match polynomial_unit_interval_roots(&map.denominator_coefficients, policy)? {
            Classification::Decided(Some(roots)) => roots,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let derivative_roots = match polynomial_unit_interval_roots(derivative_numerator, policy)? {
        Classification::Decided(Some(roots)) => roots,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let zero_roots = match polynomial_unit_interval_roots(&map.numerator_coefficients, policy)? {
        Classification::Decided(Some(roots)) => roots,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let unit_coefficients =
        polynomial_subtract(&map.numerator_coefficients, &map.denominator_coefficients);
    let unit_roots = match polynomial_unit_interval_roots(&unit_coefficients, policy)? {
        Classification::Decided(Some(roots)) => roots,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let mut boundaries = Vec::with_capacity(
        denominator_roots.len() + derivative_roots.len() + zero_roots.len() + unit_roots.len() + 2,
    );
    for boundary in [Real::zero(), Real::one()] {
        boundaries.push(RationalParameterComponentBoundary {
            parameter: BezierParameter2::Exact(boundary),
            denominator_root: false,
            zero_image: false,
            unit_image: false,
        });
    }
    for (roots, denominator_root, zero_image, unit_image) in [
        (denominator_roots, true, false, false),
        (derivative_roots, false, false, false),
        (zero_roots, false, true, false),
        (unit_roots, false, false, true),
    ] {
        for parameter in roots {
            let boundary = RationalParameterComponentBoundary {
                parameter,
                denominator_root,
                zero_image,
                unit_image,
            };
            if let Classification::Uncertain(reason) =
                insert_rational_parameter_component_boundary(&mut boundaries, boundary, policy)?
            {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }

    let zero = Real::zero();
    let one = Real::one();
    let mut runs: Vec<(usize, usize, std::cmp::Ordering)> = Vec::new();
    for index in 0..boundaries.len().saturating_sub(1) {
        let sample = match boundaries[index]
            .parameter
            .strict_scalar_between_ordered(&boundaries[index + 1].parameter, policy)?
        {
            Classification::Decided(sample) => sample,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let direction = match real_sign(&Real::eval_poly(derivative_numerator, &sample), policy) {
            Some(RealSign::Positive) => std::cmp::Ordering::Less,
            Some(RealSign::Negative) => std::cmp::Ordering::Greater,
            Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let lifted = match rational_parameter_map_at(map, &sample, policy)? {
            Classification::Decided(lifted) => lifted,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let Some(zero_order) = compare_reals(&lifted, &zero, policy) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        };
        let Some(unit_order) = compare_reals(&lifted, &one, policy) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        };
        if zero_order != std::cmp::Ordering::Greater || unit_order != std::cmp::Ordering::Less {
            continue;
        }
        if let Some((_, end, previous_direction)) = runs.last_mut()
            && *end == index
            && *previous_direction == direction
            && !boundaries[index].denominator_root
        {
            *end = index + 1;
        } else {
            runs.push((index, index + 1, direction));
        }
    }

    let mut covered_boundaries = vec![false; boundaries.len()];
    for (start, end, _) in &runs {
        covered_boundaries[*start..=*end].fill(true);
    }
    let mut domains = Vec::with_capacity(runs.len());
    for (start, end, direction) in runs {
        let lifted_start = match rational_parameter_component_boundary_image(
            &boundaries[start],
            parameter_image_map,
        )? {
            Classification::Decided(Some(parameter)) => parameter,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let lifted_end = match rational_parameter_component_boundary_image(
            &boundaries[end],
            parameter_image_map,
        )? {
            Classification::Decided(Some(parameter)) => parameter,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match lifted_start.cmp_by_refinement_with_policy(&lifted_end, policy)? {
            Classification::Decided(order) if order == direction => {}
            Classification::Decided(_) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        domains.push((
            ParameterComponentDomain {
                retained_start: boundaries[start].parameter.clone(),
                retained_end: boundaries[end].parameter.clone(),
                lifted_start,
                lifted_end,
            },
            direction,
        ));
    }

    let mut isolated_points = Vec::new();
    let last_boundary = boundaries.len().saturating_sub(1);
    for (index, boundary) in boundaries.iter().enumerate() {
        if covered_boundaries[index]
            || (!boundary.zero_image
                && !boundary.unit_image
                && index != 0
                && index != last_boundary)
        {
            continue;
        }
        let lifted_parameter =
            match rational_parameter_component_boundary_image(boundary, parameter_image_map)? {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => continue,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        isolated_points.push(ParameterComponentPoint {
            retained_parameter: boundary.parameter.clone(),
            lifted_parameter,
        });
    }
    Ok(Classification::Decided(Some(
        RationalParameterComponentPartition {
            domains,
            isolated_points,
        },
    )))
}

pub(super) fn polynomial_unit_interval_roots(
    coefficients: &[Real],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<BezierParameter2>>>> {
    let polynomial = match polynomial_from_coefficients(coefficients.to_vec(), policy)? {
        Classification::Decided(Some(polynomial)) => polynomial,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(polynomial
        .isolate_unit_interval_roots_with_policy(policy)?
        .map(Some))
}

pub(super) fn insert_rational_parameter_component_boundary(
    boundaries: &mut Vec<RationalParameterComponentBoundary>,
    boundary: RationalParameterComponentBoundary,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let mut index = 0;
    while index < boundaries.len() {
        match boundary
            .parameter
            .cmp_by_refinement_with_policy(&boundaries[index].parameter, policy)?
        {
            Classification::Decided(std::cmp::Ordering::Less) => break,
            Classification::Decided(std::cmp::Ordering::Equal) => {
                boundaries[index].denominator_root |= boundary.denominator_root;
                boundaries[index].zero_image |= boundary.zero_image;
                boundaries[index].unit_image |= boundary.unit_image;
                return Ok(Classification::Decided(()));
            }
            Classification::Decided(std::cmp::Ordering::Greater) => index += 1,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    boundaries.insert(index, boundary);
    Ok(Classification::Decided(()))
}

pub(super) fn rational_parameter_component_boundary_image(
    boundary: &RationalParameterComponentBoundary,
    parameter_image_map: &mut RationalParameterImageMap2,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    if boundary.denominator_root {
        return Ok(Classification::Decided(None));
    }
    if boundary.zero_image {
        return Ok(Classification::Decided(Some(BezierParameter2::Exact(
            Real::zero(),
        ))));
    }
    if boundary.unit_image {
        return Ok(Classification::Decided(Some(BezierParameter2::Exact(
            Real::one(),
        ))));
    }
    parameter_image_map.image(&boundary.parameter)
}

pub(super) fn parameter_component_overlap_from_domain(
    retained_parameter: CurveResultantParameter,
    domain: ParameterComponentDomain,
    direction: std::cmp::Ordering,
) -> RationalBezierIntersectionOverlap2 {
    parameter_component_overlap_from_domain_with_endpoint_inclusion(
        retained_parameter,
        domain,
        direction,
        [true, true],
    )
}

pub(super) fn parameter_component_overlap_from_domain_with_endpoint_inclusion(
    retained_parameter: CurveResultantParameter,
    domain: ParameterComponentDomain,
    direction: std::cmp::Ordering,
    endpoint_inclusion: [bool; 2],
) -> RationalBezierIntersectionOverlap2 {
    let orientation = match direction {
        std::cmp::Ordering::Less => CurveOverlapOrientation2::Same,
        std::cmp::Ordering::Greater => CurveOverlapOrientation2::Reversed,
        std::cmp::Ordering::Equal => unreachable!("a component domain has nonzero derivative"),
    };
    match (retained_parameter, direction) {
        (CurveResultantParameter::First, _) => {
            RationalBezierIntersectionOverlap2::from_certified_parameters(
                domain.retained_start,
                domain.retained_end,
                domain.lifted_start,
                domain.lifted_end,
                orientation,
                endpoint_inclusion,
            )
        }
        (CurveResultantParameter::Second, std::cmp::Ordering::Less) => {
            RationalBezierIntersectionOverlap2::from_certified_parameters(
                domain.lifted_start,
                domain.lifted_end,
                domain.retained_start,
                domain.retained_end,
                orientation,
                endpoint_inclusion,
            )
        }
        (CurveResultantParameter::Second, std::cmp::Ordering::Greater) => {
            RationalBezierIntersectionOverlap2::from_certified_parameters(
                domain.lifted_end,
                domain.lifted_start,
                domain.retained_end,
                domain.retained_start,
                orientation,
                [endpoint_inclusion[1], endpoint_inclusion[0]],
            )
        }
        (CurveResultantParameter::Second, std::cmp::Ordering::Equal) => {
            unreachable!("a component domain has nonzero derivative")
        }
    }
}

pub(super) fn rational_parameter_component_pair(
    retained_parameter: CurveResultantParameter,
    point: ParameterComponentPoint,
) -> BezierParallelIntersectionParameterPair2 {
    match retained_parameter {
        CurveResultantParameter::First => BezierParallelIntersectionParameterPair2 {
            parallel_parameter: point.retained_parameter,
            other_parameter: point.lifted_parameter,
        },
        CurveResultantParameter::Second => BezierParallelIntersectionParameterPair2 {
            parallel_parameter: point.lifted_parameter,
            other_parameter: point.retained_parameter,
        },
    }
}

pub(super) fn push_unique_parameter_component_pair(
    pairs: &mut Vec<BezierParallelIntersectionParameterPair2>,
    retained_parameter: CurveResultantParameter,
    point: ParameterComponentPoint,
) {
    let pair = rational_parameter_component_pair(retained_parameter, point);
    if !pairs.contains(&pair) {
        pairs.push(pair);
    }
}

#[cfg(test)]
pub(super) fn polynomial_is_rootless_on_parameter_range(
    coefficients: &[Real],
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let polynomial = match polynomial_from_coefficients(coefficients.to_vec(), policy)? {
        Classification::Decided(Some(polynomial)) => polynomial,
        Classification::Decided(None) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let roots = match polynomial.isolate_unit_interval_roots_with_policy(policy)? {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    parameter_roots_are_outside_range(&roots, start, end, policy)
}

#[cfg(test)]
pub(super) fn parameter_roots_are_outside_range(
    roots: &[BezierParameter2],
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    for root in roots {
        let start_order = match root.cmp_by_refinement_with_policy(start, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end_order = match root.cmp_by_refinement_with_policy(end, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if matches!(
            start_order,
            std::cmp::Ordering::Equal | std::cmp::Ordering::Greater
        ) && matches!(
            end_order,
            std::cmp::Ordering::Equal | std::cmp::Ordering::Less
        ) {
            return Ok(Classification::Decided(false));
        }
    }
    Ok(Classification::Decided(true))
}

pub(super) fn rational_parameter_map_at(
    map: &CurveIntersectionParameterLiftMap,
    parameter: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Real>> {
    let denominator = Real::eval_poly(&map.denominator_coefficients, parameter);
    match real_sign(&denominator, policy) {
        Some(RealSign::Positive | RealSign::Negative) => {}
        Some(RealSign::Zero) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    Ok(Classification::Decided(
        (Real::eval_poly(&map.numerator_coefficients, parameter) / denominator)?,
    ))
}

#[derive(Clone)]
pub(super) struct BezierParallelPairEquationSystem2 {
    pub(super) first_equation: BivariatePolynomial,
    pub(super) second_equation: BivariatePolynomial,
    pub(super) norm_equation: BivariatePolynomial,
    pub(super) first_projection: BivariatePolynomial,
    pub(super) second_projection: BivariatePolynomial,
    pub(super) tangent_cross: BivariatePolynomial,
    pub(super) tangent_dot: BivariatePolynomial,
    pub(super) norm_residual: BivariatePolynomial,
    pub(super) first_normal_projection: BivariatePolynomial,
    pub(super) first_distance: Real,
    pub(super) second_distance: Real,
    pub(super) first_distance_sign: RealSign,
    pub(super) second_distance_sign: RealSign,
    pub(super) weight_product: BivariatePolynomial,
}

pub(super) struct BezierParallelFixedDistanceSystem2 {
    pub(super) incidence: BivariatePolynomial,
    pub(super) center_speed_squared: BivariatePolynomial,
    pub(super) candidate_speed_squared: BivariatePolynomial,
    pub(super) squared_branch: BezierAlgebraicCuspTwoTermExpression2,
    pub(super) circle: BezierParallelTwoNormalExpression2,
}

/// One regular affine endpoint-extension domain for an analytic parallel.
///
/// `endpoint` remains the authored exact or algebraic boundary. `anchor` is a
/// represented scalar in the same regular cell and starts the compact open-ray
/// chart; for an algebraic endpoint the certified rootless interval between
/// the two is covered by the caller's expanded finite projection. `barrier`
/// is the first source pole or tangent-speed zero beyond that anchor.
#[derive(Clone, Debug)]
pub(crate) struct BezierParallelIncidentDomain2 {
    pub(in crate::bezier_offset) endpoint: CurveParameter2,
    pub(in crate::bezier_offset) bridge: Option<BezierParameterInterval>,
    pub(in crate::bezier_offset) anchor: Real,
    pub(in crate::bezier_offset) direction: BezierParameterRayDirection2,
    pub(in crate::bezier_offset) barrier: Option<BezierParameter2>,
}

impl BezierParallelIncidentDomain2 {
    pub(crate) fn parameter_ray(&self) -> BezierParameterRay2<'_> {
        BezierParameterRay2 {
            anchor: &self.anchor,
            direction: self.direction,
            barrier: self.barrier.as_ref(),
        }
    }

    pub(crate) const fn endpoint(&self) -> &CurveParameter2 {
        &self.endpoint
    }

    pub(crate) const fn anchor(&self) -> &Real {
        &self.anchor
    }

    pub(crate) const fn direction(&self) -> BezierParameterRayDirection2 {
        self.direction
    }

    pub(crate) const fn barrier(&self) -> Option<&BezierParameter2> {
        self.barrier.as_ref()
    }

    pub(super) const fn bridge(&self) -> Option<&BezierParameterInterval> {
        self.bridge.as_ref()
    }

    pub(super) fn reversed(&self) -> Self {
        Self {
            endpoint: self
                .endpoint
                .unit_complement()
                .expect("an analytic incident endpoint has a source-parameter complement"),
            bridge: self
                .bridge
                .as_ref()
                .map(BezierParameterInterval::unit_complement),
            anchor: Real::one() - &self.anchor,
            direction: match self.direction {
                BezierParameterRayDirection2::Decreasing => {
                    BezierParameterRayDirection2::Increasing
                }
                BezierParameterRayDirection2::Increasing => {
                    BezierParameterRayDirection2::Decreasing
                }
            },
            barrier: self.barrier.as_ref().map(BezierParameter2::unit_complement),
        }
    }

    /// Enlarges an authored retained range only through the certified
    /// rootless bridge between an algebraic endpoint and the ray chart.
    pub(crate) fn expanded_range(
        &self,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveParameterRange2>> {
        match CurveParameterDomain2::new(range, None)
            .contains_finite_parameter(&self.endpoint, policy)?
        {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Err(CurveError::Topology(
                    "an incident bridge must meet the finite parameter domain".into(),
                ));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        let [lower, upper] = match range.ordered_endpoints(policy)? {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let anchor = CurveParameter2::from(self.anchor.clone());
        let lower = match anchor.cmp_by_refinement(lower, policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => anchor.clone(),
            Classification::Decided(_) => lower.clone(),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let upper = match anchor.cmp_by_refinement(upper, policy)? {
            Classification::Decided(std::cmp::Ordering::Greater) => anchor,
            Classification::Decided(_) => upper.clone(),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(Classification::Decided(
            CurveParameterRange2::new_validated(lower, upper),
        ))
    }

    pub(crate) fn contains_extension_parameter(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let endpoint_order = match parameter.cmp_by_refinement(&self.endpoint, policy)? {
            Classification::Decided(ordering) => ordering,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let beyond_endpoint = match self.direction {
            BezierParameterRayDirection2::Decreasing => endpoint_order == std::cmp::Ordering::Less,
            BezierParameterRayDirection2::Increasing => {
                endpoint_order == std::cmp::Ordering::Greater
            }
        };
        if !beyond_endpoint {
            return Ok(Classification::Decided(false));
        }
        let Some(barrier) = self.barrier.as_ref() else {
            return Ok(Classification::Decided(true));
        };
        Ok(parameter
            .cmp_by_refinement(&CurveParameter2::from(barrier.clone()), policy)?
            .map(|ordering| match self.direction {
                BezierParameterRayDirection2::Decreasing => ordering == std::cmp::Ordering::Greater,
                BezierParameterRayDirection2::Increasing => ordering == std::cmp::Ordering::Less,
            }))
    }
}

/// Exact expression in the two positive source speeds selected by a pair of
/// analytic parallels.
///
/// With `x=sqrt(center_speed_squared)` and
/// `y=sqrt(candidate_speed_squared)`, the represented value is
/// `product*x*y + center*x + candidate*y + rational`.
#[derive(Clone, Debug)]
pub(super) struct BezierParallelTwoNormalExpression2 {
    pub(super) product: BivariatePolynomial,
    pub(super) center: BivariatePolynomial,
    pub(super) candidate: BivariatePolynomial,
    pub(super) rational: BivariatePolynomial,
}

pub(super) struct BezierSelectedParallelNormalCircleParallelSystem2 {
    pub(super) incidence: BivariatePolynomial,
    pub(super) squared_branch: BezierAlgebraicCuspTwoTermExpression2,
    pub(super) circle: BezierParallelTwoNormalExpression2,
    pub(super) selected_half_plane: BezierAlgebraicCuspTwoTermExpression2,
    pub(super) diameter: BezierParallelTwoNormalExpression2,
    pub(super) radius_squared_denominator: BivariatePolynomial,
    pub(super) tangent_cross_source: BezierAlgebraicCuspTwoTermExpression2,
    pub(super) tangent_dot_source: BezierParallelTwoNormalExpression2,
    pub(super) center_speed_squared: BivariatePolynomial,
    pub(super) candidate_speed_squared: BivariatePolynomial,
}

pub(super) enum BezierSelectedParallelNormalPositiveProjection2 {
    Candidates(Vec<BezierAlgebraicSelectedFiberParameter2>),
    /// Candidates projected directly from the authored equation after both
    /// positive speed radicals collapsed polynomially. Pair replay of this
    /// incidence certifies the unsquared sheet without reconstructing either
    /// speed root globally.
    AuthoredPolynomialCandidates {
        parameters: Vec<BezierAlgebraicSelectedFiberParameter2>,
        incidence: BivariatePolynomial,
    },
    CoincidentCircleComponent,
    Degenerate,
}

/// The parameter relation requested by a parallel-pair query. Finite self
/// queries retain both operand orders; unit self queries publish one order.
#[derive(Clone, Copy)]
pub(super) enum BezierParallelPairParameterSelection2 {
    All,
    OffDiagonal,
    Increasing,
}

impl BezierParallelPairParameterSelection2 {
    pub(super) fn admits(
        self,
        first: &BezierParameter2,
        second: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        match self {
            Self::All => Ok(Classification::Decided(true)),
            Self::Increasing => Ok(first
                .cmp_by_refinement_with_policy(second, policy)?
                .map(|order| order.is_lt())),
            Self::OffDiagonal => {
                // Saturated residuals can still meet the removed diagonal.
                // Exclude certified equal parameters before replaying their
                // equations and tangent data. An inconclusive optional proof
                // preserves general replay and its final contact filter.
                let order = policy.bounded_exact_predicate_pass(|| {
                    first.cmp_by_refinement_with_policy(second, policy)
                })?;
                Ok(Classification::Decided(!matches!(
                    order,
                    Classification::Decided(std::cmp::Ordering::Equal)
                )))
            }
        }
    }
}

#[derive(Clone, Copy)]
pub(super) enum BezierParallelPairProjectionBasis2 {
    ProjectionEquations,
    FirstAndNorm,
}

pub(super) struct BezierParallelPairProjection2 {
    pub(super) candidates: CurveIntersectionCandidates2,
    pub(super) basis: BezierParallelPairProjectionBasis2,
    pub(super) overlap: Option<RationalBezierIntersectionOverlap2>,
    pub(super) component_overlap_evidence: Arc<[BezierParameterComponentOverlap2]>,
    pub(super) component_pairs: Arc<[BezierParallelIntersectionParameterPair2]>,
    pub(super) selected_component_pair_count: usize,
    pub(super) residual_equations: Option<Box<[BivariatePolynomial; 2]>>,
    pub(super) radical_component_projection: Option<Box<BezierParallelPairProjection2>>,
}

pub(super) fn parallel_source_equality_equations(
    first: &BezierParallel2,
    second: &BezierParallel2,
) -> CurveResult<[BivariatePolynomial; 2]> {
    let first_source = first.source_power_basis()?;
    let second_source = second.source_power_basis()?;
    let unit_weight = [Real::one()];
    let first_weight = first_source.weight.unwrap_or(&unit_weight);
    let second_weight = second_source.weight.unwrap_or(&unit_weight);
    Ok([
        bivariate_parameter_difference(
            first_weight,
            second_source.x_numerator,
            first_source.x_numerator,
            second_weight,
        ),
        bivariate_parameter_difference(
            first_weight,
            second_source.y_numerator,
            first_source.y_numerator,
            second_weight,
        ),
    ])
}

/// Extracts only exact source-coordinate correspondences.
///
/// The former pair saturation divided every common factor of the squared
/// offset equations.  That could erase an unrelated radical component before
/// its geometric branch was replayed.  Source equality is the narrower
/// authority: Hypersolve publishes each exact factor and residual, and only
/// those published factors may subsequently be removed from the pair system.
pub(super) fn parallel_source_parameter_components_from_equations(
    mut residual: [BivariatePolynomial; 2],
    config: CurveIntersectionResultantConfig,
) -> Classification<Vec<BivariatePolynomial>> {
    let mut components = Vec::new();
    loop {
        match extract_bivariate_axis_components(&residual) {
            Classification::Decided(Some(axis)) => {
                components.extend(axis.supports);
                residual = axis.residual_equations;
                continue;
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Classification::Uncertain(reason);
            }
        }
        let previous_degree = residual
            .iter()
            .map(bivariate_storage_bidegree_sum)
            .sum::<usize>();
        let mut next = None;
        let mut blocker = None;
        for retained_parameter in [
            CurveResultantParameter::First,
            CurveResultantParameter::Second,
        ] {
            let report = parameter_component_bivariate_polynomial_system_complete(
                &residual[0],
                &residual[1],
                retained_parameter,
                config,
            );
            match report.status {
                BivariatePolynomialComponentStatus::Rational => {
                    let (Some(component), Some(reduced)) = (
                        rational_parameter_component_support(
                            retained_parameter,
                            &report.numerator_coefficients,
                            &report.denominator_coefficients,
                        ),
                        report.reduced_equations,
                    ) else {
                        blocker = Some(UncertaintyReason::Boundary);
                        continue;
                    };
                    if residual.iter().all(|equation| {
                        divide_bivariate_polynomial_exact(equation, &component).is_some()
                    }) {
                        next = Some((component, reduced));
                        break;
                    }
                    blocker = Some(UncertaintyReason::Boundary);
                }
                BivariatePolynomialComponentStatus::Implicit => {
                    let (Some(component), Some(reduced)) =
                        (report.implicit_component, report.reduced_equations)
                    else {
                        blocker = Some(UncertaintyReason::Boundary);
                        continue;
                    };
                    next = Some((component, reduced));
                    break;
                }
                BivariatePolynomialComponentStatus::UndecidedCoefficient => {
                    blocker = Some(UncertaintyReason::RealSign)
                }
                BivariatePolynomialComponentStatus::EmptyEquation
                | BivariatePolynomialComponentStatus::DegreeBoundExceeded
                | BivariatePolynomialComponentStatus::DeterminantError
                | BivariatePolynomialComponentStatus::InterpolationFailed => {
                    blocker = Some(UncertaintyReason::Boundary)
                }
                BivariatePolynomialComponentStatus::UnsupportedLiftedDegree
                | BivariatePolynomialComponentStatus::NoSupportedComponent => {}
                // Hypersolve reaches `DivisionFailed` when the exact first
                // nonzero subresultant is not a common divisor. That is the
                // coprime residual case, not an uncertain coefficient: a
                // genuine generic source component would divide both source
                // coordinate equations exactly.
                BivariatePolynomialComponentStatus::DivisionFailed => {}
            }
        }
        let Some((component, next_residual)) = next else {
            return blocker.map_or_else(
                || Classification::Decided(components),
                Classification::Uncertain,
            );
        };
        let next_degree = next_residual
            .iter()
            .map(bivariate_storage_bidegree_sum)
            .sum::<usize>();
        if next_degree >= previous_degree {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        components.push(component);
        residual = next_residual;
    }
}

pub(super) fn parallel_source_parameter_components(
    first: &BezierParallel2,
    second: &BezierParallel2,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<Vec<BivariatePolynomial>>> {
    Ok(parallel_source_parameter_components_from_equations(
        parallel_source_equality_equations(first, second)?,
        config,
    ))
}

pub(super) fn structural_parallel_source_parameter_component(
    first: &BezierParallel2,
    second: &BezierParallel2,
) -> Option<BivariatePolynomial> {
    if first.source() == second.source() {
        return Some(BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::one()],
            vec![Real::from(-1_i8)],
        ]));
    }
    (first.source() == &second.source().reversed()).then(|| {
        BivariatePolynomial::new(vec![
            vec![Real::from(-1_i8), Real::one()],
            vec![Real::one()],
        ])
    })
}

pub(super) struct ParameterDomainConstraint2 {
    pub(super) component_support: Option<BivariatePolynomial>,
    pub(super) isolated_projection: Option<BezierParallelPairProjection2>,
}

pub(super) fn parameter_domain_constraint(
    support: BivariatePolynomial,
    constraint: &BivariatePolynomial,
    domains: [CurveParameterDomain2<'_>; 2],
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<ParameterDomainConstraint2>> {
    let mut residual_equations = [support, constraint.clone()];
    let mut component_support = None;
    let project = |equations: &[BivariatePolynomial; 2]| {
        project_parallel_intersection_system(&equations[0], &equations[1], domains, policy)
    };
    match extract_bivariate_axis_components(&residual_equations) {
        Classification::Decided(Some(axis)) => {
            component_support = parameter_component_union_support(&axis.supports);
            residual_equations = axis.residual_equations;
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let initial_candidates = match project(&residual_equations)? {
        Classification::Decided(candidates) => candidates,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let candidates = if matches!(
        initial_candidates,
        CurveIntersectionCandidates2::DegenerateResultant
    ) {
        let extracted = match extract_bivariate_system_components(residual_equations, config) {
            Classification::Decided(extracted) => extracted,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        component_support = merge_parameter_component_support(component_support, extracted.support);
        residual_equations = extracted.residual_equations;
        match project(&residual_equations)? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    } else {
        initial_candidates
    };
    let isolated_projection = match candidates {
        CurveIntersectionCandidates2::NoIntersection => None,
        CurveIntersectionCandidates2::DegenerateResultant => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        candidates @ CurveIntersectionCandidates2::Candidates { .. } => {
            Some(BezierParallelPairProjection2 {
                candidates,
                basis: BezierParallelPairProjectionBasis2::ProjectionEquations,
                overlap: None,
                component_overlap_evidence: Arc::from([]),
                component_pairs: Arc::from([]),
                selected_component_pair_count: 0,
                residual_equations: Some(Box::new(residual_equations)),
                radical_component_projection: None,
            })
        }
    };
    Ok(Classification::Decided(ParameterDomainConstraint2 {
        component_support,
        isolated_projection,
    }))
}

pub(super) fn prepend_parallel_pair_projection(
    projection: &mut BezierParallelPairProjection2,
    mut preceding: BezierParallelPairProjection2,
) {
    preceding.radical_component_projection = projection.radical_component_projection.take();
    projection.radical_component_projection = Some(Box::new(preceding));
}

pub(super) fn retain_parameter_component_pairs(
    projection: Option<BezierParallelPairProjection2>,
    pairs: Vec<BezierParallelIntersectionParameterPair2>,
    overlaps: Vec<BezierParameterComponentOverlap2>,
) -> Option<BezierParallelPairProjection2> {
    if pairs.is_empty() && overlaps.is_empty() {
        return projection;
    }
    let mut projection = projection.unwrap_or(BezierParallelPairProjection2 {
        candidates: CurveIntersectionCandidates2::NoIntersection,
        basis: BezierParallelPairProjectionBasis2::ProjectionEquations,
        overlap: None,
        component_overlap_evidence: Arc::from([]),
        component_pairs: Arc::from([]),
        selected_component_pair_count: 0,
        residual_equations: None,
        radical_component_projection: None,
    });
    if !overlaps.is_empty() {
        let mut evidence = projection.component_overlap_evidence.to_vec();
        evidence.extend(overlaps);
        projection.component_overlap_evidence = evidence.into();
    }
    let mut retained =
        projection.component_pairs[..projection.selected_component_pair_count].to_vec();
    for pair in pairs {
        if !retained.contains(&pair) {
            retained.push(pair);
        }
    }
    let selected_count = retained.len();
    for pair in &projection.component_pairs[projection.selected_component_pair_count..] {
        if !retained.contains(pair) {
            retained.push(pair.clone());
        }
    }
    projection.component_pairs = retained.into();
    projection.selected_component_pair_count = selected_count;
    Some(projection)
}

pub(super) fn parallel_nonstructural_source_parameter_components(
    first: &BezierParallel2,
    second: &BezierParallel2,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<Vec<BivariatePolynomial>>> {
    let equations = parallel_source_equality_equations(first, second)?;
    let residual =
        if let Some(structural) = structural_parallel_source_parameter_component(first, second) {
            let Some(residual) = divide_bivariate_system_component(&equations, &structural) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            };
            residual
        } else {
            equations
        };
    Ok(parallel_source_parameter_components_from_equations(
        residual, config,
    ))
}

pub(super) fn parallel_pair_equations_without_source_components(
    equations: &[BivariatePolynomial; 2],
    first: &BezierParallel2,
    second: &BezierParallel2,
    source_overlap: &Classification<CertifiedParallelSourceOverlap2>,
) -> CurveResult<Option<[BivariatePolynomial; 2]>> {
    if !matches!(source_overlap, Classification::Decided(_)) {
        return Ok(None);
    }
    let config = CurveIntersectionResultantConfig {
        min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
        max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
    };
    let source_equations = parallel_source_equality_equations(first, second)?;
    let (mut residual_equations, source_residual_equations) =
        if let Some(component) = structural_parallel_source_parameter_component(first, second) {
            let (Some(pair_residual), Some(source_residual)) = (
                divide_bivariate_system_component(equations, &component),
                divide_bivariate_system_component(&source_equations, &component),
            ) else {
                return Ok(None);
            };
            (Some(pair_residual), source_residual)
        } else {
            (None, source_equations)
        };
    let source_components = match parallel_source_parameter_components_from_equations(
        source_residual_equations,
        config,
    ) {
        Classification::Decided(components) => components,
        Classification::Uncertain(_) => return Ok(None),
    };
    for component in source_components {
        let remaining = residual_equations.as_ref().unwrap_or(equations);
        if let Some(reduced) = divide_bivariate_system_component(remaining, &component) {
            residual_equations = Some(reduced);
        }
    }
    Ok(residual_equations)
}

pub(super) enum BezierParallelPairDomainProjection2 {
    Enumerated {
        projection: BezierParallelPairProjection2,
        retained_contacts: Vec<BezierParallelPairIntersectionContact2>,
        components: Vec<CurveParameterComponent2>,
    },
    Components(Vec<CurveParameterComponent2>),
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct ParameterComponentAffineMap2 {
    pub(super) scale: Real,
    pub(super) offset: Real,
}

#[derive(Clone, Copy)]
pub(super) enum ParameterComponentMap2<'a> {
    Identity,
    Affine(&'a ParameterComponentAffineMap2),
    Incident(BezierParameterRay2<'a>),
}

#[derive(Clone, Copy)]
pub(super) struct ParameterComponentChart2<'a> {
    pub(super) domain: CurveParameterDomain2<'a>,
    pub(super) mapping: ParameterComponentMap2<'a>,
    pub(super) range: &'a std::cell::OnceCell<CurveParameterRange2>,
}

impl<'a> ParameterComponentChart2<'a> {
    pub(super) fn authored(
        domain: CurveParameterDomain2<'a>,
        map: &'a std::cell::OnceCell<Option<ParameterComponentAffineMap2>>,
        range: &'a std::cell::OnceCell<CurveParameterRange2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if map.get().is_none() {
            let (_, [lower, upper]) = match domain.finite_envelope(policy)? {
                Classification::Decided(envelope) => envelope,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            // The unit topology is authoritative only for a contained domain.
            // Exterior finite intervals use an invertible affine chart, while
            // the original selected boundaries remain the clipping authority.
            let contained = matches!(
                compare_reals(lower, &Real::zero(), &CurveContext::STRICT),
                Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater)
            ) && matches!(
                compare_reals(upper, &Real::one(), &CurveContext::STRICT),
                Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Less)
            );
            let mapping = (!contained).then(|| ParameterComponentAffineMap2 {
                scale: upper - lower,
                offset: lower.clone(),
            });
            let _ = map.set(mapping);
        }
        Ok(Classification::Decided(Self {
            domain,
            mapping: match map.get().expect("the finite chart map was prepared") {
                Some(mapping) => ParameterComponentMap2::Affine(mapping),
                None => ParameterComponentMap2::Identity,
            },
            range,
        }))
    }

    pub(super) fn compact_range(
        self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<&'a CurveParameterRange2>> {
        if let Some(range) = self.range.get() {
            return Ok(Classification::Decided(range));
        }
        let range = match self.mapping {
            ParameterComponentMap2::Identity | ParameterComponentMap2::Affine(_) => {
                let [lower, upper] = match self.domain.finite.ordered_endpoints(policy)? {
                    Classification::Decided(endpoints) => endpoints,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                match self.mapping {
                    ParameterComponentMap2::Identity => {
                        CurveParameterRange2::new_validated(lower.clone(), upper.clone())
                    }
                    ParameterComponentMap2::Affine(mapping) => {
                        let scale = (Real::one() / &mapping.scale)?;
                        let offset = -(&mapping.offset * &scale);
                        let lower = match lower.affine_image_unbounded(&scale, &offset, policy)? {
                            Classification::Decided(parameter) => parameter,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        let upper = match upper.affine_image_unbounded(&scale, &offset, policy)? {
                            Classification::Decided(parameter) => parameter,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        CurveParameterRange2::new_validated(lower, upper)
                    }
                    ParameterComponentMap2::Incident(_) => unreachable!(),
                }
            }
            ParameterComponentMap2::Incident(extension) => {
                let end = match extension.barrier {
                    None => BezierParameter2::Exact(Real::one()),
                    Some(barrier) => match barrier.incident_ray_compact_parameter(
                        extension.anchor,
                        extension.direction,
                        policy,
                    )? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    },
                };
                let range = match BezierParameterRange2::try_new_with_policy(
                    BezierParameter2::Exact(Real::zero()),
                    end,
                    policy,
                )? {
                    Classification::Decided(range) => range,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                CurveParameterRange2::from_bezier_range(range)
            }
        };
        Ok(Classification::Decided(self.range.get_or_init(|| range)))
    }

    pub(super) fn contains_component_event(
        self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let range = match self.compact_range(policy)? {
            Classification::Decided(range) => range,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let inclusion = if matches!(self.mapping, ParameterComponentMap2::Incident(_)) {
            [false; 2]
        } else {
            self.domain.inclusion
        };
        let after_start = match parameter.cmp_by_refinement(range.start(), policy)? {
            Classification::Decided(ordering) => {
                ordering.is_gt() || (inclusion[0] && ordering.is_eq())
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if !after_start {
            return Ok(Classification::Decided(false));
        }
        Ok(parameter
            .cmp_by_refinement(range.end(), policy)?
            .map(|ordering| ordering.is_lt() || (inclusion[1] && ordering.is_eq())))
    }

    /// Finite roots keep their original ownership. Overlapping ray events
    /// still partition selector signs, but only the finite chart publishes
    /// those contacts. This rule follows the actual domain, not the unit span.
    pub(super) fn owned_original_parameter(
        self,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierParameter2>>> {
        match self.contains_component_event(&parameter.clone().into(), policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        let mapped = match self.mapping {
            ParameterComponentMap2::Identity => {
                return Ok(Classification::Decided(Some(parameter.clone())));
            }
            ParameterComponentMap2::Affine(mapping) => {
                return Ok(parameter
                    .affine_image_unbounded(&mapping.scale, &mapping.offset, policy)?
                    .map(Some));
            }
            ParameterComponentMap2::Incident(extension) => {
                match parameter.incident_ray_parameter(
                    extension.anchor,
                    extension.direction,
                    policy,
                )? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        };
        Ok(self
            .domain
            .contains_finite_parameter(&mapped.clone().into(), policy)?
            .map(|inside| (!inside).then_some(mapped)))
    }

    pub(super) fn owned_retained_parameter(
        self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        if let Some(parameter) = parameter.as_bezier_parameter() {
            return Ok(self
                .owned_original_parameter(parameter, policy)?
                .map(|parameter| parameter.map(CurveParameter2::from)));
        }
        match self.contains_component_event(parameter, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        let mapped = match self.mapping {
            ParameterComponentMap2::Identity => {
                return Ok(Classification::Decided(Some(parameter.clone())));
            }
            ParameterComponentMap2::Affine(mapping) => {
                return Ok(parameter
                    .affine_image_unbounded(&mapping.scale, &mapping.offset, policy)?
                    .map(Some));
            }
            ParameterComponentMap2::Incident(extension) => {
                let direction = match extension.direction {
                    BezierParameterRayDirection2::Increasing => Real::one(),
                    BezierParameterRayDirection2::Decreasing => -Real::one(),
                };
                match parameter.projective_image_unbounded(
                    &[extension.anchor.clone(), direction - extension.anchor],
                    &[Real::one(), -Real::one()],
                    policy,
                )? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        };
        Ok(self
            .domain
            .contains_finite_parameter(&mapped, policy)?
            .map(|inside| (!inside).then_some(mapped)))
    }
}

pub(super) fn bivariate_compose_incident_parameter(
    polynomial: &BivariatePolynomial,
    axis: CurveResultantParameter,
    anchor: &Real,
    direction: BezierParameterRayDirection2,
) -> Option<BivariatePolynomial> {
    let direction_sign = match direction {
        BezierParameterRayDirection2::Decreasing => -Real::one(),
        BezierParameterRayDirection2::Increasing => Real::one(),
    };
    // t = anchor +/- x/(1-x)
    let numerator = vec![anchor.clone(), &direction_sign - anchor];
    let denominator = vec![Real::one(), -Real::one()];
    let transformed = match axis {
        CurveResultantParameter::First => {
            let degree = polynomial.coefficients.len().saturating_sub(1);
            let numerator_powers = polynomial_powers(&numerator, degree);
            let denominator_powers = polynomial_powers(&denominator, degree);
            let second_count = polynomial
                .coefficients
                .iter()
                .map(Vec::len)
                .max()
                .unwrap_or(0);
            let mut coefficients = vec![vec![Real::zero(); second_count]; degree + 1];
            for (power, row) in polynomial.coefficients.iter().enumerate() {
                let homogeneous = polynomial_multiply(
                    &numerator_powers[power],
                    &denominator_powers[degree - power],
                );
                for (compact_power, scale) in homogeneous.iter().enumerate() {
                    for (second_power, coefficient) in row.iter().enumerate() {
                        coefficients[compact_power][second_power] += coefficient * scale;
                    }
                }
            }
            BivariatePolynomial::new(coefficients)
        }
        CurveResultantParameter::Second => {
            let degree = polynomial
                .coefficients
                .iter()
                .map(|row| row.len().saturating_sub(1))
                .max()
                .unwrap_or(0);
            let numerator_powers = polynomial_powers(&numerator, degree);
            let denominator_powers = polynomial_powers(&denominator, degree);
            let coefficients = polynomial
                .coefficients
                .iter()
                .map(|row| {
                    let mut transformed = vec![Real::zero(); degree + 1];
                    for (power, coefficient) in row.iter().enumerate() {
                        let homogeneous = polynomial_multiply(
                            &numerator_powers[power],
                            &denominator_powers[degree - power],
                        );
                        for (compact_power, scale) in homogeneous.iter().enumerate() {
                            transformed[compact_power] += coefficient * scale;
                        }
                    }
                    transformed
                })
                .collect();
            BivariatePolynomial::new(coefficients)
        }
    };
    bivariate_trim_exact(transformed)
}

pub(super) fn transform_parameter_component_chart_polynomial<'a>(
    polynomial: &'a BivariatePolynomial,
    first_chart: ParameterComponentChart2<'_>,
    second_chart: ParameterComponentChart2<'_>,
) -> Option<Cow<'a, BivariatePolynomial>> {
    let mut transformed = Cow::Borrowed(polynomial);
    for (axis, chart) in [
        (CurveResultantParameter::First, first_chart),
        (CurveResultantParameter::Second, second_chart),
    ] {
        match chart.mapping {
            ParameterComponentMap2::Identity => {}
            ParameterComponentMap2::Affine(mapping) => {
                let polynomial = match axis {
                    CurveResultantParameter::First => transformed.substitute_affine(
                        &mapping.scale,
                        &mapping.offset,
                        &Real::one(),
                        &Real::zero(),
                    ),
                    CurveResultantParameter::Second => transformed.substitute_affine(
                        &Real::one(),
                        &Real::zero(),
                        &mapping.scale,
                        &mapping.offset,
                    ),
                };
                transformed = Cow::Owned(bivariate_trim_exact(polynomial)?);
            }
            ParameterComponentMap2::Incident(extension) => {
                transformed = Cow::Owned(bivariate_compose_incident_parameter(
                    &transformed,
                    axis,
                    extension.anchor,
                    extension.direction,
                )?);
            }
        }
    }
    Some(transformed)
}

pub(super) fn transform_parallel_pair_system_for_component_chart<'a>(
    system: &'a BezierParallelPairEquationSystem2,
    first_chart: ParameterComponentChart2<'_>,
    second_chart: ParameterComponentChart2<'_>,
) -> Option<Cow<'a, BezierParallelPairEquationSystem2>> {
    if matches!(first_chart.mapping, ParameterComponentMap2::Identity)
        && matches!(second_chart.mapping, ParameterComponentMap2::Identity)
    {
        return Some(Cow::Borrowed(system));
    }
    let transform = |polynomial: &BivariatePolynomial| {
        transform_parameter_component_chart_polynomial(polynomial, first_chart, second_chart)
            .map(Cow::into_owned)
    };
    Some(Cow::Owned(BezierParallelPairEquationSystem2 {
        first_equation: transform(&system.first_equation)?,
        second_equation: transform(&system.second_equation)?,
        norm_equation: transform(&system.norm_equation)?,
        first_projection: transform(&system.first_projection)?,
        second_projection: transform(&system.second_projection)?,
        tangent_cross: transform(&system.tangent_cross)?,
        tangent_dot: transform(&system.tangent_dot)?,
        norm_residual: transform(&system.norm_residual)?,
        first_normal_projection: transform(&system.first_normal_projection)?,
        first_distance: system.first_distance.clone(),
        second_distance: system.second_distance.clone(),
        first_distance_sign: system.first_distance_sign,
        second_distance_sign: system.second_distance_sign,
        weight_product: transform(&system.weight_product)?,
    }))
}

#[derive(Clone, Copy)]
pub(crate) enum ParameterComponentQuery2<'a> {
    /// Stop at the first selected component; contacts and other components
    /// are intentionally not enumerated after that point. Optional constraints
    /// select the original contact curves' normal sheets.
    FirstComponent(Option<&'a [BezierParallelDerivativeConstraint2; 2]>),
    /// Retain every finite or incident component together with residual contacts.
    AllComponents(Option<&'a [BezierParallelDerivativeConstraint2; 2]>),
    /// Finite intersections retain every correspondence and residual contact.
    RetainFinite,
}

impl<'a> ParameterComponentQuery2<'a> {
    pub(super) fn normal_constraints(self) -> Option<&'a [BezierParallelDerivativeConstraint2; 2]> {
        match self {
            Self::FirstComponent(constraints) | Self::AllComponents(constraints) => constraints,
            Self::RetainFinite => None,
        }
    }

    pub(super) fn without_identity_constraints(self, policy: &CurveContext) -> Self {
        let identity = self.normal_constraints().is_some_and(|constraints| {
            constraints.iter().all(|constraint| {
                constraint.expected == RealSign::Positive
                    && real_sign(constraint.parallel.distance(), policy) == Some(RealSign::Zero)
            })
        });
        if !identity {
            return self;
        }
        match self {
            Self::FirstComponent(_) => Self::FirstComponent(None),
            Self::AllComponents(_) => Self::AllComponents(None),
            Self::RetainFinite => self,
        }
    }
}

/// Polynomial sign evidence for the orientation of one original parallel.
/// The center support's own derivative does not select the fillet normal.
pub(crate) struct BezierParallelDerivativeConstraint2 {
    pub(in crate::bezier_offset) parallel: BezierParallel2,
    pub(in crate::bezier_offset) axis: CurveResultantParameter,
    pub(in crate::bezier_offset) expected: RealSign,
    pub(in crate::bezier_offset) side: Option<BezierParameterRayDirection2>,
    pub(in crate::bezier_offset) polynomials:
        OnceLock<CurveResult<BezierParallelDerivativePolynomials2>>,
}

impl BezierParallelDerivativeConstraint2 {
    /// Replays the constant orientation on a source cell already partitioned
    /// at every source singularity and original-parallel cusp. The cell's
    /// endpoints use its one-sided orientation, including a stationary seam.
    pub(crate) fn selects_regular_range(
        &self,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let strict = policy.strict_counterpart();
        let interior = match range.strict_interior_scalar(&strict)? {
            Classification::Decided(interior) => interior,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(self
            .parallel
            .parallel_derivative_scale_sign(&interior.into(), &strict)?
            .map(|sign| sign == self.expected))
    }

    pub(super) fn polynomials(&self) -> CurveResult<&BezierParallelDerivativePolynomials2> {
        match self.polynomials.get_or_init(|| {
            self.parallel
                .derivative_scale_polynomials(self.axis, self.expected, self.side)
        }) {
            Ok(polynomials) => Ok(polynomials),
            Err(error) => Err(error.clone()),
        }
    }
}

pub(super) struct BezierParallelDerivativePolynomials2 {
    pub(super) axis: CurveResultantParameter,
    pub(super) offset_distance: Real,
    pub(super) speed_squared: BivariatePolynomial,
    pub(super) signed_curvature: BivariatePolynomial,
    pub(super) cusp_norm: BivariatePolynomial,
    pub(super) expected: RealSign,
    pub(super) side: Option<BezierParameterRayDirection2>,
}

impl BezierParallelDerivativePolynomials2 {
    pub(super) fn boundary_polynomials(&self) -> [&BivariatePolynomial; 3] {
        [&self.speed_squared, &self.signed_curvature, &self.cusp_norm]
    }

    pub(super) fn selected_at(
        &self,
        first: &BezierParameter2,
        second: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        if real_sign(&self.offset_distance, policy) == Some(RealSign::Zero) {
            return Ok(Classification::Decided(self.expected == RealSign::Positive));
        }
        let sign =
            |polynomial| signed_bivariate_at_parameter_pair(polynomial, first, second, policy);
        match sign(&self.speed_squared)? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero) => return Ok(Classification::Decided(false)),
            Classification::Decided(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "parallel source speed squared was certified negative".into(),
                ));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        let scale =
            parallel_derivative_scale_from_curvature_sign(sign(&self.signed_curvature)?, || {
                sign(&self.cusp_norm)
            })?;
        if scale != Classification::Decided(RealSign::Zero) || self.side.is_none() {
            return Ok(scale.map(|scale| scale == self.expected));
        }
        let side_sign = |polynomial: &BivariatePolynomial| {
            // These normal predicates depend on only the original source
            // axis. Separate affine/incident charts preserve that structure.
            let (parameter, coefficients) = match self.axis {
                CurveResultantParameter::First => {
                    debug_assert!(polynomial.coefficients.iter().all(|row| row.len() <= 1));
                    (
                        first,
                        Cow::Owned(
                            polynomial
                                .coefficients
                                .iter()
                                .map(|row| row.first().cloned().unwrap_or_else(Real::zero))
                                .collect::<Vec<_>>(),
                        ),
                    )
                }
                CurveResultantParameter::Second => {
                    debug_assert!(polynomial.coefficients.len() <= 1);
                    (
                        second,
                        Cow::Borrowed(
                            polynomial
                                .coefficients
                                .first()
                                .map(Vec::as_slice)
                                .unwrap_or(&[]),
                        ),
                    )
                }
            };
            parameter_polynomial_side_sign(
                &coefficients,
                &parameter.clone().into(),
                self.side.unwrap(),
                &policy.strict_counterpart(),
            )
        };
        Ok(parallel_derivative_scale_from_curvature_sign(
            side_sign(&self.signed_curvature)?,
            || side_sign(&self.cusp_norm),
        )?
        .map(|scale| scale == self.expected))
    }

    pub(super) fn in_charts(
        &self,
        first: ParameterComponentChart2<'_>,
        second: ParameterComponentChart2<'_>,
    ) -> Option<Self> {
        let transform = |polynomial| {
            transform_parameter_component_chart_polynomial(polynomial, first, second)
                .map(Cow::into_owned)
        };
        let side = if let Some(side) = self.side {
            let chart = match self.axis {
                CurveResultantParameter::First => first,
                CurveResultantParameter::Second => second,
            };
            let reversed = match chart.mapping {
                ParameterComponentMap2::Identity => false,
                ParameterComponentMap2::Affine(mapping) => {
                    match real_sign(&mapping.scale, &CurveContext::STRICT)? {
                        RealSign::Positive => false,
                        RealSign::Negative => true,
                        RealSign::Zero => return None,
                    }
                }
                ParameterComponentMap2::Incident(ray) => {
                    ray.direction == BezierParameterRayDirection2::Decreasing
                }
            };
            Some(
                if (side == BezierParameterRayDirection2::Increasing) != reversed {
                    BezierParameterRayDirection2::Increasing
                } else {
                    BezierParameterRayDirection2::Decreasing
                },
            )
        } else {
            None
        };
        Some(Self {
            axis: self.axis,
            side,
            offset_distance: self.offset_distance.clone(),
            speed_squared: transform(&self.speed_squared)?,
            signed_curvature: transform(&self.signed_curvature)?,
            cusp_norm: transform(&self.cusp_norm)?,
            expected: self.expected,
        })
    }
}

pub(super) struct ParameterComponentSelection2 {
    pub(super) components: Vec<CurveParameterComponent2>,
    pub(super) component_overlaps: Vec<BezierParameterComponentOverlap2>,
    pub(super) selected_pairs: Vec<BezierParallelIntersectionParameterPair2>,
    pub(super) retained_contacts: Vec<BezierParallelPairIntersectionContact2>,
}

impl ParameterComponentSelection2 {
    pub(super) fn has_components(&self) -> bool {
        !self.components.is_empty() || !self.component_overlaps.is_empty()
    }
}

pub(super) type RetainedComponentDomains2 = (
    Arc<[ComponentParameterChart2; 2]>,
    [Vec<ComponentParameterInterval2>; 2],
);

pub(super) fn retained_component_domains<'a>(
    slot: &'a std::cell::OnceCell<RetainedComponentDomains2>,
    charts: [ParameterComponentChart2<'_>; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<&'a RetainedComponentDomains2>> {
    if let Some(retained) = slot.get() {
        return Ok(Classification::Decided(retained));
    }
    let charts = match retain_component_charts(charts, policy)? {
        Classification::Decided(charts) => charts,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mut owned = [Vec::new(), Vec::new()];
    for (chart, owned) in charts.iter().zip(&mut owned) {
        match chart.owned_intervals(policy)? {
            Classification::Decided(intervals) => *owned = intervals,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    Ok(Classification::Decided(
        slot.get_or_init(|| (charts, owned)),
    ))
}

#[allow(clippy::too_many_arguments)]
pub(super) fn select_axis_parameter_components_on_chart(
    factor: &[Real],
    fixed_axis: CurveResultantParameter,
    selector: &ParameterComponentSelector2<'_>,
    first_chart: ParameterComponentChart2<'_>,
    second_chart: ParameterComponentChart2<'_>,
    selected_pairs: &mut Vec<BezierParallelIntersectionParameterPair2>,
    components: &mut Vec<CurveParameterComponent2>,
    query: ParameterComponentQuery2<'_>,
    retained_charts: &std::cell::OnceCell<RetainedComponentDomains2>,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    if factor.len() <= 1 {
        return Ok(Classification::Decided(false));
    }
    let polynomial = match polynomial_from_coefficients(factor.to_vec(), policy)? {
        Classification::Decided(Some(polynomial)) => polynomial,
        Classification::Decided(None) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let fixed_parameters = match polynomial.isolate_unit_interval_roots_with_policy(policy)? {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let fixed_chart = match fixed_axis {
        CurveResultantParameter::First => first_chart,
        CurveResultantParameter::Second => second_chart,
    };
    let mut found = false;
    for fixed in fixed_parameters {
        match fixed_chart.contains_component_event(&fixed.clone().into(), policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => continue,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        // Select complete sign cells before clipping. An arbitrary retained
        // domain end inside one cell inherits that cell's certified signs;
        // it must not need a standalone root reconstruction.
        let mut events = vec![
            BezierParameter2::Exact(Real::zero()),
            BezierParameter2::Exact(Real::one()),
        ];
        for boundary in selector.boundary_polynomials() {
            let boundary = match fixed_axis {
                CurveResultantParameter::First => boundary,
                CurveResultantParameter::Second => bivariate_swap_parameters(&boundary),
            };
            let projection = match selected_parameter_fiber_parameters(
                &boundary,
                &fixed,
                MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
                MAX_SELECTED_FIBER_QUOTIENT_DEGREE,
                &CurveParameterRange2::unit(),
                policy,
            )? {
                Classification::Decided(projection) => projection,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let roots = match projection {
                BezierAlgebraicFiberProjection2::Parameters(roots) => roots,
                BezierAlgebraicFiberProjection2::IdenticallyZero => continue,
                BezierAlgebraicFiberProjection2::Degenerate => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
            };
            for root in roots {
                match insert_ordered_parameter_component_boundary(&mut events, root, policy)? {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }

        let pair = |free: BezierParameter2| match fixed_axis {
            CurveResultantParameter::First => BezierParallelIntersectionParameterPair2 {
                parallel_parameter: fixed.clone(),
                other_parameter: free,
            },
            CurveResultantParameter::Second => BezierParallelIntersectionParameterPair2 {
                parallel_parameter: free,
                other_parameter: fixed.clone(),
            },
        };
        let mut event_inclusion = Vec::with_capacity(events.len());
        for event in &events {
            let event_pair = pair(event.clone());
            let selected = match selector.selected_at(
                CurveResultantParameter::First,
                &event_pair.parallel_parameter,
                &event_pair.other_parameter,
                policy,
            )? {
                Classification::Decided(selected) => selected,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            event_inclusion.push(selected);
            if selected {
                let mapped = match owned_parameter_component_pair_from_chart(
                    &event_pair,
                    first_chart,
                    second_chart,
                    policy,
                )? {
                    Classification::Decided(Some(pair)) => pair,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if !selected_pairs.contains(&mapped) {
                    selected_pairs.push(mapped);
                }
            }
        }

        let boundaries: Vec<_> = events.into_iter().map(CurveParameter2::from).collect();
        for (boundaries, inclusion) in boundaries.windows(2).zip(event_inclusion.windows(2)) {
            match boundaries[0].cmp_by_refinement(&boundaries[1], policy)? {
                Classification::Decided(std::cmp::Ordering::Less) => {}
                Classification::Decided(std::cmp::Ordering::Equal) => continue,
                Classification::Decided(std::cmp::Ordering::Greater) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let sample =
                match boundaries[0].strict_scalar_between_ordered(&boundaries[1], policy)? {
                    Classification::Decided(sample) => sample,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            let sample_pair = pair(BezierParameter2::Exact(sample));
            match selector.selected_at(
                CurveResultantParameter::First,
                &sample_pair.parallel_parameter,
                &sample_pair.other_parameter,
                policy,
            )? {
                Classification::Decided(true) => {
                    let (charts, owned) = match retained_component_domains(
                        retained_charts,
                        [first_chart, second_chart],
                        policy,
                    )? {
                        Classification::Decided(retained) => retained,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let fixed_axis_index = match fixed_axis {
                        CurveResultantParameter::First => 0,
                        CurveResultantParameter::Second => 1,
                    };
                    let free_axis_index = 1 - fixed_axis_index;
                    let free_interval = ComponentParameterInterval2 {
                        range: CurveParameterRange2::new_validated(
                            boundaries[0].clone(),
                            boundaries[1].clone(),
                        ),
                        inclusion: [inclusion[0], inclusion[1]],
                    };
                    for fixed_interval in &owned[fixed_axis_index] {
                        match fixed_interval.contains(&fixed.clone().into(), policy)? {
                            Classification::Decided(true) => {}
                            Classification::Decided(false) => continue,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                        for owned_free in &owned[free_axis_index] {
                            let free_interval =
                                match free_interval.intersection(owned_free, policy)? {
                                    Classification::Decided(Some(interval)) => interval,
                                    Classification::Decided(None) => continue,
                                    Classification::Uncertain(reason) => {
                                        return Ok(Classification::Uncertain(reason));
                                    }
                                };
                            if matches!(query, ParameterComponentQuery2::RetainFinite) {
                                return Ok(Classification::Decided(true));
                            }
                            let intervals = if fixed_axis_index == 0 {
                                [fixed_interval.clone(), free_interval]
                            } else {
                                [free_interval, fixed_interval.clone()]
                            };
                            let fixed = if fixed_axis_index == 0 {
                                [Some(fixed.clone().into()), None]
                            } else {
                                [None, Some(fixed.clone().into())]
                            };
                            components.push(CurveParameterComponent2::product(
                                fixed,
                                Arc::clone(charts),
                                intervals,
                            ));
                            found = true;
                            if matches!(query, ParameterComponentQuery2::FirstComponent(_)) {
                                return Ok(Classification::Decided(true));
                            }
                        }
                    }
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }
    Ok(Classification::Decided(found))
}

pub(super) fn owned_parameter_component_pair_from_chart(
    pair: &BezierParallelIntersectionParameterPair2,
    first_chart: ParameterComponentChart2<'_>,
    second_chart: ParameterComponentChart2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParallelIntersectionParameterPair2>>> {
    let first = match first_chart.owned_original_parameter(&pair.parallel_parameter, policy)? {
        Classification::Decided(Some(parameter)) => parameter,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let second = match second_chart.owned_original_parameter(&pair.other_parameter, policy)? {
        Classification::Decided(Some(parameter)) => parameter,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(Some(
        BezierParallelIntersectionParameterPair2 {
            parallel_parameter: first,
            other_parameter: second,
        },
    )))
}

pub(super) fn select_parameter_component_in_domain(
    support: &BivariatePolynomial,
    selector: &ParameterComponentSelector2<'_>,
    domains: [CurveParameterDomain2<'_>; 2],
    query: ParameterComponentQuery2<'_>,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<ParameterComponentSelection2>> {
    debug_assert!(
        !matches!(query, ParameterComponentQuery2::RetainFinite)
            || domains.iter().all(|domain| domain.extension.is_none())
    );
    let mut components = Vec::new();
    let mut component_overlaps = Vec::new();
    let mut selected_pairs = Vec::new();
    let mut retained_contacts = Vec::new();
    // Each query has one policy and at most four chart domains. Retain only
    // decided ranges, on demand: a finite selected component must not force
    // an unused extension's barrier conversion or refinement.
    let ranges: [[_; 2]; 2] =
        std::array::from_fn(|_| std::array::from_fn(|_| std::cell::OnceCell::new()));
    let finite_maps: [_; 2] = std::array::from_fn(|_| std::cell::OnceCell::new());
    let chart = |axis: usize,
                 extended: bool|
     -> CurveResult<Classification<Option<ParameterComponentChart2<'_>>>> {
        if extended {
            let Some(extension) = domains[axis].extension else {
                return Ok(Classification::Decided(None));
            };
            match extension.is_empty(policy)? {
                Classification::Decided(true) => return Ok(Classification::Decided(None)),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
            Ok(Classification::Decided(Some(ParameterComponentChart2 {
                domain: domains[axis],
                mapping: ParameterComponentMap2::Incident(extension),
                range: &ranges[axis][1],
            })))
        } else {
            Ok(ParameterComponentChart2::authored(
                domains[axis],
                &finite_maps[axis],
                &ranges[axis][0],
                policy,
            )?
            .map(Some))
        }
    };
    for second_extended in [false, true] {
        let second_chart = match chart(1, second_extended)? {
            Classification::Decided(Some(chart)) => chart,
            Classification::Decided(None) => continue,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        for first_extended in [false, true] {
            let first_chart = match chart(0, first_extended)? {
                Classification::Decided(Some(chart)) => chart,
                Classification::Decided(None) => continue,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let Some(chart_support) =
                transform_parameter_component_chart_polynomial(support, first_chart, second_chart)
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
            };
            let normal_constraints = match query.normal_constraints() {
                Some(constraints) => {
                    Some([constraints[0].polynomials()?, constraints[1].polynomials()?])
                }
                None => selector
                    .normal_constraints()
                    .map(|constraints| constraints.each_ref()),
            };
            let chart_constraints = if let Some(constraints) = normal_constraints {
                let Some(first) = constraints[0].in_charts(first_chart, second_chart) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                };
                let Some(second) = constraints[1].in_charts(first_chart, second_chart) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                };
                Some([first, second])
            } else {
                None
            };
            let chart_branch;
            let chart_system;
            let chart_filter;
            let selector = match selector {
                ParameterComponentSelector2::Positive(branch, _) => {
                    chart_branch = match transform_parameter_component_chart_polynomial(
                        branch,
                        first_chart,
                        second_chart,
                    ) {
                        Some(branch) => branch,
                        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                    };
                    ParameterComponentSelector2::Positive(&chart_branch, chart_constraints.as_ref())
                }
                ParameterComponentSelector2::ParallelPair {
                    normal_constraints: _,
                    system,
                    parameter_filter,
                } => {
                    chart_system = match transform_parallel_pair_system_for_component_chart(
                        system,
                        first_chart,
                        second_chart,
                    ) {
                        Some(system) => system,
                        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                    };
                    chart_filter = match parameter_filter {
                        Some(filter) => match transform_parameter_component_chart_polynomial(
                            filter,
                            first_chart,
                            second_chart,
                        ) {
                            Some(filter) => Some(filter),
                            None => {
                                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                            }
                        },
                        None => None,
                    };
                    ParameterComponentSelector2::ParallelPair {
                        normal_constraints: chart_constraints.as_ref(),
                        system: &chart_system,
                        parameter_filter: chart_filter.as_deref(),
                    }
                }
            };
            let retained_charts = std::cell::OnceCell::new();
            let axis_report =
                extract_bivariate_polynomial_system_axis_factors(&chart_support, &chart_support);
            let chart_support = if axis_report.status
                == BivariatePolynomialAxisFactorStatus::Reduced
            {
                for (factor, axis) in [
                    (
                        &axis_report.first_parameter_factor,
                        CurveResultantParameter::First,
                    ),
                    (
                        &axis_report.second_parameter_factor,
                        CurveResultantParameter::Second,
                    ),
                ] {
                    match select_axis_parameter_components_on_chart(
                        factor,
                        axis,
                        &selector,
                        first_chart,
                        second_chart,
                        &mut selected_pairs,
                        &mut components,
                        query,
                        &retained_charts,
                        policy,
                    )? {
                        Classification::Decided(true) => {
                            // Full finite rational queries remove constant
                            // images first and exclude poles in the selector.
                            // An axis component here has no bijective map.
                            if matches!(query, ParameterComponentQuery2::RetainFinite) {
                                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                            }
                            if matches!(query, ParameterComponentQuery2::FirstComponent(_)) {
                                return Ok(Classification::Decided(ParameterComponentSelection2 {
                                    components,
                                    component_overlaps,
                                    selected_pairs,
                                    retained_contacts,
                                }));
                            }
                        }
                        Classification::Decided(false) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                let Some([reduced, _]) = axis_report.reduced_equations else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                };
                Cow::Owned(reduced)
            } else {
                chart_support
            };
            if bivariate_unit_square_has_strict_bernstein_sign(&chart_support, policy)? {
                continue;
            }
            let chart_support = chart_support.into_owned();
            let component = match parameter_component_system_with_selector(
                &[chart_support.clone(), chart_support],
                &selector,
                policy,
                config,
            )? {
                Classification::Decided(Some(component)) => component,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if component.overlaps.iter().any(|overlap| {
                !component
                    .component_overlaps
                    .iter()
                    .any(|evidence| evidence.overlap() == overlap)
            }) {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            let first_range = match first_chart.compact_range(policy)? {
                Classification::Decided(range) => range,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let second_range = match second_chart.compact_range(policy)? {
                Classification::Decided(range) => range,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            for overlap in component.component_overlaps.iter() {
                match overlap.has_positive_overlap(first_range, second_range, policy)? {
                    Classification::Decided(true) => {
                        if !matches!(query, ParameterComponentQuery2::RetainFinite) {
                            let (charts, owned) = match retained_component_domains(
                                &retained_charts,
                                [first_chart, second_chart],
                                policy,
                            )? {
                                Classification::Decided(retained) => retained,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            };
                            for first in &owned[0] {
                                for second in &owned[1] {
                                    match overlap.has_positive_overlap(
                                        &first.range,
                                        &second.range,
                                        policy,
                                    )? {
                                        Classification::Decided(true) => {}
                                        Classification::Decided(false) => continue,
                                        Classification::Uncertain(reason) => {
                                            return Ok(Classification::Uncertain(reason));
                                        }
                                    }
                                    components.push(CurveParameterComponent2::implicit(
                                        overlap.clone(),
                                        Arc::clone(charts),
                                        [first.clone(), second.clone()],
                                    ));
                                    if matches!(query, ParameterComponentQuery2::FirstComponent(_))
                                    {
                                        return Ok(Classification::Decided(
                                            ParameterComponentSelection2 {
                                                components,
                                                component_overlaps,
                                                selected_pairs,
                                                retained_contacts,
                                            },
                                        ));
                                    }
                                }
                            }
                            continue;
                        }
                        let maps = [first_chart, second_chart].map(|chart| match chart.mapping {
                            ParameterComponentMap2::Identity => ParameterComponentAffineMap2 {
                                scale: Real::one(),
                                offset: Real::zero(),
                            },
                            ParameterComponentMap2::Affine(map) => map.clone(),
                            ParameterComponentMap2::Incident(_) => {
                                unreachable!("finite component discovery has no incident chart")
                            }
                        });
                        match overlap.clone().in_parameter_charts(maps, policy)? {
                            Classification::Decided(overlap) => component_overlaps.push(overlap),
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                        continue;
                    }
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                let contacts =
                    match overlap.closed_boundary_contacts(first_range, second_range, policy)? {
                        Classification::Decided(contacts) => contacts,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                for mut contact in contacts {
                    let mut owned = [None, None];
                    for ((chart, parameter), slot) in [
                        (first_chart, &contact.first_parameter),
                        (second_chart, &contact.second_parameter),
                    ]
                    .into_iter()
                    .zip(&mut owned)
                    {
                        match chart.owned_retained_parameter(parameter, policy)? {
                            Classification::Decided(Some(parameter)) => *slot = Some(parameter),
                            Classification::Decided(None) => break,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    let [Some(first), Some(second)] = owned else {
                        continue;
                    };
                    contact.first_parameter = first;
                    contact.second_parameter = second;
                    match parallel_pair_contact_parameters_are_retained(
                        &retained_contacts,
                        &contact.first_parameter,
                        &contact.second_parameter,
                        policy,
                    )? {
                        Classification::Decided(true) => continue,
                        Classification::Decided(false) => retained_contacts.push(contact),
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
            for pair in &component.component_pairs[..component.selected_component_pair_count] {
                let pair = match owned_parameter_component_pair_from_chart(
                    pair,
                    first_chart,
                    second_chart,
                    policy,
                )? {
                    Classification::Decided(Some(pair)) => pair,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if !selected_pairs.contains(&pair) {
                    selected_pairs.push(pair);
                }
            }
        }
    }
    Ok(Classification::Decided(ParameterComponentSelection2 {
        components,
        component_overlaps,
        selected_pairs,
        retained_contacts,
    }))
}

pub(super) fn project_parallel_pair_without_components_in_domain(
    system: &BezierParallelPairEquationSystem2,
    first: &BezierParallel2,
    second: &BezierParallel2,
    source_overlap: &Classification<CertifiedParallelSourceOverlap2>,
    domains: [CurveParameterDomain2<'_>; 2],
    query: ParameterComponentQuery2<'_>,
    parameter_filter: Option<&BivariatePolynomial>,
    policy: &CurveContext,
) -> CurveResult<Option<BezierParallelPairDomainProjection2>> {
    if !matches!(source_overlap, Classification::Decided(_)) {
        return Ok(None);
    }
    let original_equations = [
        system.first_equation.clone(),
        system.second_equation.clone(),
    ];
    let source_residual = parallel_pair_equations_without_source_components(
        &original_equations,
        first,
        second,
        source_overlap,
    )?;
    let source_component_removed = source_residual.is_some();
    let mut residual_equations = source_residual.unwrap_or(original_equations);
    let project = |equations: &[BivariatePolynomial; 2]| {
        project_parallel_intersection_system(&equations[0], &equations[1], domains, policy)
    };
    let config = CurveIntersectionResultantConfig {
        min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
        max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
    };
    let mut pair_component_support = None;
    let mut residual_was_saturated = false;
    match extract_bivariate_axis_components(&residual_equations) {
        Classification::Decided(Some(axis)) => {
            pair_component_support = parameter_component_union_support(&axis.supports);
            residual_equations = axis.residual_equations;
            residual_was_saturated = true;
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(_) => return Ok(None),
    }
    let initial_candidates = match project(&residual_equations)? {
        Classification::Decided(candidates) => candidates,
        Classification::Uncertain(_) => return Ok(None),
    };
    let candidates = if matches!(
        initial_candidates,
        CurveIntersectionCandidates2::DegenerateResultant
    ) {
        let extracted = match extract_bivariate_system_components(residual_equations, config) {
            Classification::Decided(extracted) => extracted,
            Classification::Uncertain(_) => return Ok(None),
        };
        if extracted.support.is_none() {
            return Ok(None);
        }
        pair_component_support =
            merge_parameter_component_support(pair_component_support, extracted.support);
        residual_equations = extracted.residual_equations;
        residual_was_saturated = true;
        match project(&residual_equations)? {
            Classification::Decided(candidates)
                if !matches!(
                    candidates,
                    CurveIntersectionCandidates2::DegenerateResultant
                ) =>
            {
                candidates
            }
            _ => return Ok(None),
        }
    } else {
        initial_candidates
    };
    let mut retained_contacts = Vec::new();
    let mut components = Vec::new();
    let mut component_overlaps = Vec::new();
    let radical_component_projection = if let Some(support) = pair_component_support {
        let constraint = match parameter_domain_constraint(
            support,
            &system.norm_equation,
            domains,
            policy,
            config,
        )? {
            Classification::Decided(constraint) => constraint,
            Classification::Uncertain(_) => return Ok(None),
        };
        let mut selected_pairs = Vec::new();
        if let Some(component_support) = constraint.component_support {
            let selection = match select_parameter_component_in_domain(
                &component_support,
                &ParameterComponentSelector2::ParallelPair {
                    normal_constraints: None,
                    system,
                    parameter_filter,
                },
                domains,
                query,
                policy,
                config,
            )? {
                Classification::Decided(selection) => selection,
                Classification::Uncertain(_) => return Ok(None),
            };
            if selection.has_components()
                && matches!(query, ParameterComponentQuery2::FirstComponent(_))
            {
                return Ok(Some(BezierParallelPairDomainProjection2::Components(
                    selection.components,
                )));
            }
            components.extend(selection.components);
            selected_pairs = selection.selected_pairs;
            component_overlaps = selection.component_overlaps;
            retained_contacts.extend(selection.retained_contacts);
        }
        retain_parameter_component_pairs(constraint.isolated_projection, selected_pairs, Vec::new())
            .map(Box::new)
    } else {
        None
    };
    let residual_equations =
        (source_component_removed || residual_was_saturated).then(|| Box::new(residual_equations));
    Ok(Some(BezierParallelPairDomainProjection2::Enumerated {
        projection: BezierParallelPairProjection2 {
            candidates,
            basis: BezierParallelPairProjectionBasis2::ProjectionEquations,
            overlap: match source_overlap {
                Classification::Decided(source) => source.selected_overlap().cloned(),
                Classification::Uncertain(_) => None,
            },
            component_overlap_evidence: component_overlaps.into(),
            component_pairs: match source_overlap {
                Classification::Decided(source) => source.contacts.clone(),
                Classification::Uncertain(_) => Arc::from([]),
            },
            selected_component_pair_count: match source_overlap {
                Classification::Decided(source) => source.contacts.len(),
                Classification::Uncertain(_) => 0,
            },
            residual_equations,
            radical_component_projection,
        },
        retained_contacts,
        components,
    }))
}

pub(super) fn project_unit_parallel_pair_intersection_system(
    system: &BezierParallelPairEquationSystem2,
    first: &BezierParallel2,
    second: &BezierParallel2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParallelPairDomainProjection2>> {
    let unit = CurveParameterRange2::unit();
    let domains = [CurveParameterDomain2::new(&unit, None); 2];
    let project_without_components =
        |source_overlap: &Classification<CertifiedParallelSourceOverlap2>| {
            project_parallel_pair_without_components_in_domain(
                system,
                first,
                second,
                source_overlap,
                domains,
                ParameterComponentQuery2::RetainFinite,
                None,
                policy,
            )
        };
    let enumerated = |projection| BezierParallelPairDomainProjection2::Enumerated {
        projection,
        retained_contacts: Vec::new(),
        components: Vec::new(),
    };
    let may_component =
        bivariate_pair_may_have_component(&system.first_equation, &system.second_equation);
    // A source correspondence certifies its component, not the absence of
    // other contacts. Unit and retained domains share axis extraction,
    // component selection and residual projection before claiming completeness.
    let structural_overlap = may_component
        .then(|| structural_parallel_overlap(first, second, policy))
        .transpose()?
        .flatten();
    let mut source_overlap = if let Some(overlap) = structural_overlap {
        Some(Classification::Decided(
            CertifiedParallelSourceOverlap2::without_contacts(
                CertifiedParallelSourceOverlapKind2::Selected(overlap),
            ),
        ))
    } else {
        may_component
            .then(|| certified_parallel_source_overlap(first, second, policy))
            .transpose()?
    };
    if let Some(source_overlap) = source_overlap.as_ref()
        && let Some(projection) = project_without_components(source_overlap)?
    {
        return Ok(Classification::Decided(projection));
    }

    let projected = match project_parallel_intersection_system(
        &system.first_equation,
        &system.second_equation,
        domains,
        policy,
    )? {
        Classification::Decided(projected) => projected,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if !matches!(projected, CurveIntersectionCandidates2::DegenerateResultant) {
        return Ok(Classification::Decided(enumerated(
            BezierParallelPairProjection2 {
                candidates: projected,
                basis: BezierParallelPairProjectionBasis2::ProjectionEquations,
                overlap: None,
                component_overlap_evidence: Arc::from([]),
                component_pairs: Arc::from([]),
                selected_component_pair_count: 0,
                residual_equations: None,
                radical_component_projection: None,
            },
        )));
    }
    if source_overlap.is_none() {
        source_overlap = Some(certified_parallel_source_overlap(first, second, policy)?);
    }
    let source_overlap = source_overlap.expect("degenerate projection classified its source");
    if let Some(projection) = project_without_components(&source_overlap)? {
        return Ok(Classification::Decided(projection));
    }
    let fallback = match project_parallel_intersection_system(
        &system.first_equation,
        &system.norm_equation,
        domains,
        policy,
    )? {
        Classification::Decided(projected) => projected,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if matches!(fallback, CurveIntersectionCandidates2::DegenerateResultant)
        && let Classification::Uncertain(reason) = source_overlap
    {
        return Ok(Classification::Uncertain(reason));
    }
    Ok(Classification::Decided(enumerated(
        BezierParallelPairProjection2 {
            candidates: fallback,
            basis: BezierParallelPairProjectionBasis2::FirstAndNorm,
            overlap: None,
            component_overlap_evidence: Arc::from([]),
            component_pairs: match &source_overlap {
                Classification::Decided(source) => source.contacts.clone(),
                Classification::Uncertain(_) => Arc::from([]),
            },
            selected_component_pair_count: match &source_overlap {
                Classification::Decided(source) => source.contacts.len(),
                Classification::Uncertain(_) => 0,
            },
            residual_equations: None,
            radical_component_projection: None,
        },
    )))
}

/// Constructs the polynomial candidate relation for points on two selected
/// analytic parallels at a prescribed squared distance.
///
/// The first parameter is the retained center and the second is the candidate.
/// With source-point numerator difference `Delta`, speed squares `Su,St`,
/// source-weight product `W`, signed offset distances `du,dv`, and target
/// square `r2`, the exact relation after multiplying by both positive speeds
/// is
///
/// `A sqrt(Su St) + B sqrt(Su) + C sqrt(St) + E = 0`.
///
/// Squaring first in `sqrt(St)` gives `F0 + F1 sqrt(Su) = 0`; squaring once
/// more gives the projected incidence `F0^2-Su F1^2=0`. The returned radical
/// expressions retain both unsquared relations for selected-branch replay.
pub(super) fn parallel_source_fixed_distance_incidence(
    center: &BezierParallel2,
    candidate: &BezierParallel2,
    radius_squared: &Real,
    center_parameter: &CurveParameter2,
    candidate_range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<BivariatePolynomial>> {
    let center_source = center.source_power_basis()?;
    let candidate_source = candidate.source_power_basis()?;
    if let Some(weight) = center_source.weight {
        match policy.strict_predicate_pass(|| center_parameter.polynomial_sign(weight, policy))? {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    if let Some(weight) = candidate_source.weight {
        match polynomial_is_nonzero_on_parameter_range(weight, candidate_range, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    let unit = [Real::one()];
    let center_weight = center_source.weight.unwrap_or(&unit);
    let candidate_weight = candidate_source.weight.unwrap_or(&unit);
    let delta_x = bivariate_parameter_difference(
        center_weight,
        candidate_source.x_numerator,
        center_source.x_numerator,
        candidate_weight,
    );
    let delta_y = bivariate_parameter_difference(
        center_weight,
        candidate_source.y_numerator,
        center_source.y_numerator,
        candidate_weight,
    );
    let weight_product = bivariate_outer_product(center_weight, candidate_weight);
    Ok(Classification::Decided(bivariate_subtract(
        &bivariate_add(
            &bivariate_multiply(&delta_x, &delta_x),
            &bivariate_multiply(&delta_y, &delta_y),
        ),
        &bivariate_scale(
            bivariate_multiply(&weight_product, &weight_product),
            radius_squared,
        ),
    )))
}

pub(super) fn parallel_fixed_distance_system(
    center: &BezierParallel2,
    candidate: &BezierParallel2,
    radius_squared: &Real,
    candidate_range: &CurveParameterRange2,
    center_parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParallelFixedDistanceSystem2>> {
    let center_source = center.source_power_basis()?;
    let candidate_source = candidate.source_power_basis()?;
    let center_differential = center.differential()?;
    let candidate_differential = candidate.differential()?;
    // Distance replay needs a finite selected source frame, not a whole
    // native source or a nonzero parallel derivative. Circle construction
    // separately certifies any oriented tangent it retains.
    if let Classification::Uncertain(reason) =
        center.certify_source_frame_at(center_parameter, policy)?
    {
        return Ok(Classification::Uncertain(reason));
    }
    let candidate_speed = parallel_speed_squared_polynomial(candidate_differential);
    for coefficients in candidate_source
        .weight
        .into_iter()
        .chain(Some(candidate_speed.as_slice()))
    {
        match polynomial_is_nonzero_on_parameter_range(coefficients, candidate_range, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }

    let unit = [Real::one()];
    let center_weight = center_source.weight.unwrap_or(&unit);
    let candidate_weight = candidate_source.weight.unwrap_or(&unit);
    let delta_x = bivariate_parameter_difference(
        center_weight,
        candidate_source.x_numerator,
        center_source.x_numerator,
        candidate_weight,
    );
    let delta_y = bivariate_parameter_difference(
        center_weight,
        candidate_source.y_numerator,
        center_source.y_numerator,
        candidate_weight,
    );
    let center_speed_squared = bivariate_outer_product(
        &parallel_speed_squared_polynomial(center_differential),
        &unit,
    );
    let candidate_speed_squared = bivariate_outer_product(&unit, &candidate_speed);
    let candidate_tangent_x = bivariate_outer_product(&unit, &candidate_differential.tangent_x);
    let candidate_tangent_y = bivariate_outer_product(&unit, &candidate_differential.tangent_y);
    let candidate_normal_projection = bivariate_subtract(
        &bivariate_multiply(&delta_y, &candidate_tangent_x),
        &bivariate_multiply(&delta_x, &candidate_tangent_y),
    );
    let center_normal_projection = bivariate_subtract(
        &bivariate_multiply_first_parameter(&delta_y, &center_differential.tangent_x),
        &bivariate_multiply_first_parameter(&delta_x, &center_differential.tangent_y),
    );
    let tangent_dot = bivariate_add(
        &bivariate_outer_product(
            &center_differential.tangent_x,
            &candidate_differential.tangent_x,
        ),
        &bivariate_outer_product(
            &center_differential.tangent_y,
            &candidate_differential.tangent_y,
        ),
    );
    let weight = bivariate_outer_product(center_weight, candidate_weight);
    let weight_squared = bivariate_multiply(&weight, &weight);
    let squared_delta = bivariate_add(
        &bivariate_multiply(&delta_x, &delta_x),
        &bivariate_multiply(&delta_y, &delta_y),
    );
    let a = bivariate_add(
        &squared_delta,
        &bivariate_scale(
            weight_squared.clone(),
            &(center.distance() * center.distance() + candidate.distance() * candidate.distance()
                - radius_squared),
        ),
    );
    let b = bivariate_scale(
        bivariate_multiply(&weight, &candidate_normal_projection),
        &(Real::from(2_u8) * candidate.distance()),
    );
    let c = bivariate_scale(
        bivariate_multiply(&weight, &center_normal_projection),
        &(Real::from(-2_i8) * center.distance()),
    );
    let e = bivariate_scale(
        bivariate_multiply(&weight_squared, &tangent_dot),
        &(Real::from(-2_i8) * center.distance() * candidate.distance()),
    );

    let f0 = bivariate_subtract(
        &bivariate_add(
            &bivariate_multiply(&e, &e),
            &bivariate_multiply(&bivariate_multiply(&b, &b), &center_speed_squared),
        ),
        &bivariate_add(
            &bivariate_multiply(&candidate_speed_squared, &bivariate_multiply(&c, &c)),
            &bivariate_multiply(
                &bivariate_multiply(&candidate_speed_squared, &bivariate_multiply(&a, &a)),
                &center_speed_squared,
            ),
        ),
    );
    let f1 = bivariate_scale(
        bivariate_subtract(
            &bivariate_multiply(&e, &b),
            &bivariate_multiply(&candidate_speed_squared, &bivariate_multiply(&c, &a)),
        ),
        &Real::from(2_u8),
    );
    // A zero displacement removes its speed radical. Isolate the smaller
    // norm directly; squaring it again only adds multiplicity and carries an
    // irrelevant source-speed factor into every later selected-fiber replay.
    // Both source speeds have already been certified nonzero on their domains.
    let center_is_zero =
        real_sign(center.distance(), &CurveContext::STRICT) == Some(RealSign::Zero);
    let candidate_is_zero =
        real_sign(candidate.distance(), &CurveContext::STRICT) == Some(RealSign::Zero);
    let incidence = match (center_is_zero, candidate_is_zero) {
        (true, true) => a.clone(),
        (true, false) => bivariate_subtract(
            &bivariate_multiply(&bivariate_multiply(&a, &a), &candidate_speed_squared),
            &bivariate_multiply(&b, &b),
        ),
        (false, true) => bivariate_subtract(
            &bivariate_multiply(&bivariate_multiply(&a, &a), &center_speed_squared),
            &bivariate_multiply(&c, &c),
        ),
        (false, false) => bivariate_subtract(
            &bivariate_multiply(&f0, &f0),
            &bivariate_multiply(&bivariate_multiply(&f1, &f1), &center_speed_squared),
        ),
    };
    Ok(Classification::Decided(
        BezierParallelFixedDistanceSystem2 {
            incidence,
            center_speed_squared,
            candidate_speed_squared,
            squared_branch: BezierAlgebraicCuspTwoTermExpression2 {
                rational: f0,
                radical: f1,
            },
            circle: BezierParallelTwoNormalExpression2 {
                product: a,
                center: b,
                candidate: c,
                rational: e,
            },
        },
    ))
}

pub(super) fn parallel_pair_equation_system(
    first: &BezierParallel2,
    second: &BezierParallel2,
    unit_domain: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParallelPairEquationSystem2>>> {
    parallel_pair_equation_system_with_tangent_fields(
        first,
        second,
        None,
        None,
        unit_domain,
        policy,
    )
}
