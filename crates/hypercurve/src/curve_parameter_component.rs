//! Retained parameter components in their certified finite or incident charts.
//!
//! A projective chart is retained as a single fractional-linear map. Its
//! compact endpoint at infinity remains excluded; no affine scalar is made
//! for it. The finite chart keeps ownership where an incident ray overlaps it.

use super::*;

#[derive(Clone, Debug)]
pub(super) struct ComponentParameterInterval2 {
    pub(super) range: CurveParameterRange2,
    pub(super) inclusion: [bool; 2],
}

impl ComponentParameterInterval2 {
    pub(super) fn intersection(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        let mut ends = [None, None];
        let mut inclusion = [false; 2];
        for (index, (a, b)) in [
            (self.range.start(), other.range.start()),
            (self.range.end(), other.range.end()),
        ]
        .into_iter()
        .enumerate()
        {
            match a.cmp_by_refinement(b, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    ends[index] = Some(a.clone());
                    inclusion[index] = self.inclusion[index] && other.inclusion[index];
                }
                Classification::Decided(order) => {
                    let use_first = if index == 0 {
                        order.is_gt()
                    } else {
                        order.is_lt()
                    };
                    ends[index] = Some(if use_first { a } else { b }.clone());
                    inclusion[index] = if use_first {
                        self.inclusion[index]
                    } else {
                        other.inclusion[index]
                    };
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        let [start, end] = ends.map(|end| end.expect("both interval bounds were selected"));
        match start.cmp_by_refinement(&end, policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => {
                Ok(Classification::Decided(Some(Self {
                    range: CurveParameterRange2::new_validated(start, end),
                    inclusion,
                })))
            }
            Classification::Decided(_) => Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(super) fn contains(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        for (bound, included, lower) in [
            (self.range.start(), self.inclusion[0], true),
            (self.range.end(), self.inclusion[1], false),
        ] {
            match parameter.cmp_by_refinement(bound, policy)? {
                Classification::Decided(order) => {
                    let inside = if lower { order.is_gt() } else { order.is_lt() }
                        || (included && order.is_eq());
                    if !inside {
                        return Ok(Classification::Decided(false));
                    }
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        Ok(Classification::Decided(true))
    }
}

#[derive(Clone, Debug)]
pub(super) struct ComponentParameterChart2 {
    numerator: [Real; 2],
    denominator: [Real; 2],
    interval: ComponentParameterInterval2,
    finite_owner: Option<(
        CurveParameterRange2,
        [bool; 2],
        BezierParameterRayDirection2,
    )>,
}

impl ComponentParameterChart2 {
    pub(super) fn retain(
        chart: ParameterComponentChart2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let range = match chart.compact_range(policy)? {
            Classification::Decided(range) => range.clone(),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let (numerator, denominator, finite_owner) = match chart.mapping {
            ParameterComponentMap2::Identity => (
                [Real::zero(), Real::one()],
                [Real::one(), Real::zero()],
                None,
            ),
            ParameterComponentMap2::Affine(map) => (
                [map.offset.clone(), map.scale.clone()],
                [Real::one(), Real::zero()],
                None,
            ),
            ParameterComponentMap2::Incident(ray) => {
                let direction = match ray.direction {
                    BezierParameterRayDirection2::Increasing => Real::one(),
                    BezierParameterRayDirection2::Decreasing => -Real::one(),
                };
                (
                    [ray.anchor.clone(), direction - ray.anchor],
                    [Real::one(), -Real::one()],
                    Some((
                        chart.domain.finite.clone(),
                        chart.domain.inclusion,
                        ray.direction,
                    )),
                )
            }
        };
        let inclusion = if finite_owner.is_none() {
            chart.domain.inclusion
        } else {
            [false; 2]
        };
        Ok(Classification::Decided(Self {
            numerator,
            denominator,
            interval: ComponentParameterInterval2 { range, inclusion },
            finite_owner,
        }))
    }

    /// Partitions the compact chart after subtracting its finite owner.
    /// Each cut keeps exactly the complement of that owner's endpoint inclusion.
    pub(super) fn owned_intervals(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<ComponentParameterInterval2>>> {
        let Some((finite, inclusion, direction)) = &self.finite_owner else {
            return Ok(Classification::Decided(vec![self.interval.clone()]));
        };
        let endpoints = match finite.ordered_endpoints(policy)? {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let (endpoints, inclusion) = if *direction == BezierParameterRayDirection2::Increasing {
            (endpoints, *inclusion)
        } else {
            ([endpoints[1], endpoints[0]], [inclusion[1], inclusion[0]])
        };
        let anchor = CurveParameter2::from(self.numerator[0].clone());
        let inverse_numerator = [-self.numerator[0].clone(), self.denominator[0].clone()];
        let inverse_denominator = [self.numerator[1].clone(), -self.denominator[1].clone()];
        let range = &self.interval.range;
        let mut cuts = [None, None];
        for ((endpoint, included), cut) in endpoints.into_iter().zip(inclusion).zip(&mut cuts) {
            let on_ray = match endpoint.cmp_by_refinement(&anchor, policy)? {
                Classification::Decided(order) => {
                    if *direction == BezierParameterRayDirection2::Increasing {
                        order.is_gt()
                    } else {
                        order.is_lt()
                    }
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let mapped = if on_ray {
                match Self::image(endpoint, &inverse_numerator, &inverse_denominator, policy)? {
                    Classification::Decided(Some(mapped)) => mapped,
                    Classification::Decided(None) => {
                        return Err(CurveError::Topology(
                            "an incident chart inverse has a pole on its certified ray".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            } else {
                range.start().clone()
            };
            // A finite endpoint outside the open ray must not close the
            // ray's anchor or regularity barrier when its cut is clamped.
            let included = if included {
                false
            } else {
                match self.interval.contains(&mapped, policy)? {
                    Classification::Decided(inside) => inside,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            };
            let mapped = match mapped.cmp_by_refinement(range.start(), policy)? {
                Classification::Decided(order) if order.is_le() => range.start().clone(),
                Classification::Decided(_) => {
                    match mapped.cmp_by_refinement(range.end(), policy)? {
                        Classification::Decided(order) if order.is_ge() => range.end().clone(),
                        Classification::Decided(_) => mapped,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            *cut = Some((mapped, included));
        }
        let [(lower, lower_included), (upper, upper_included)] =
            cuts.map(|cut| cut.expect("both finite boundaries were transported"));
        let mut intervals = Vec::with_capacity(2);
        for (start, end, inclusion) in [
            (
                range.start(),
                &lower,
                [self.interval.inclusion[0], lower_included],
            ),
            (
                &upper,
                range.end(),
                [upper_included, self.interval.inclusion[1]],
            ),
        ] {
            match start.cmp_by_refinement(end, policy)? {
                Classification::Decided(std::cmp::Ordering::Less) => {
                    intervals.push(ComponentParameterInterval2 {
                        range: CurveParameterRange2::new_validated(start.clone(), end.clone()),
                        inclusion,
                    })
                }
                Classification::Decided(std::cmp::Ordering::Equal) => {}
                Classification::Decided(std::cmp::Ordering::Greater) => {
                    return Err(CurveError::InvalidCurveRange);
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        Ok(Classification::Decided(intervals))
    }

    fn image(
        parameter: &CurveParameter2,
        numerator: &[Real; 2],
        denominator: &[Real; 2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        if denominator == &[Real::one(), Real::zero()] {
            if numerator == &[Real::zero(), Real::one()] {
                return Ok(Classification::Decided(Some(parameter.clone())));
            }
            return Ok(parameter
                .affine_image_unbounded(&numerator[1], &numerator[0], policy)?
                .map(Some));
        }
        match parameter.polynomial_sign(denominator, policy)? {
            Classification::Decided(RealSign::Zero) => return Ok(Classification::Decided(None)),
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        Ok(parameter
            .projective_image_unbounded(numerator, denominator, policy)?
            .map(Some))
    }

    pub(super) fn local_parameter(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        if let Some((finite, inclusion, _)) = &self.finite_owner {
            match CurveParameterDomain2::new(finite, None)
                .with_finite_inclusion(*inclusion)
                .contains_finite_parameter(parameter, policy)?
            {
                Classification::Decided(true) => return Ok(Classification::Decided(None)),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        let inverse_numerator = [-self.numerator[0].clone(), self.denominator[0].clone()];
        let inverse_denominator = [self.numerator[1].clone(), -self.denominator[1].clone()];
        let parameter =
            match Self::image(parameter, &inverse_numerator, &inverse_denominator, policy)? {
                Classification::Decided(Some(parameter)) => parameter,
                other => return Ok(other),
            };
        Ok(self
            .interval
            .contains(&parameter, policy)?
            .map(|inside| inside.then_some(parameter)))
    }
}

pub(super) fn retain_component_charts(
    charts: [ParameterComponentChart2<'_>; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<Arc<[ComponentParameterChart2; 2]>>> {
    let mut retained = [None, None];
    for (chart, retained) in charts.into_iter().zip(&mut retained) {
        match ComponentParameterChart2::retain(chart, policy)? {
            Classification::Decided(chart) => *retained = Some(chart),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    Ok(Classification::Decided(Arc::new(retained.map(|chart| {
        chart.expect("both component charts were retained")
    }))))
}

/// A retained locus and its exact clipping domains. Correspondences preserve
/// the solver's local selected roots and fiber ranks. Products retain either
/// fixed parameters or complete interval axes, including point-image families.
#[derive(Clone, Debug)]
pub(crate) struct CurveParameterComponent2 {
    charts: Arc<[ComponentParameterChart2; 2]>,
    intervals: [ComponentParameterInterval2; 2],
    locus: ComponentParameterLocus2,
    swapped: bool,
    point_image: Option<CurvePoint2>,
}

/// A constrained component is empty, a selected pair, or still has a free axis.
/// The last case is an input requirement, never evidence of no solution.
#[derive(Clone, Debug)]
pub(crate) enum CurveParameterComponentSelection2 {
    Empty,
    Selected([CurveParameter2; 2]),
    NeedsConstraint,
}

#[derive(Clone, Debug)]
enum ComponentParameterLocus2 {
    Correspondence {
        source: crate::curve_intersection::CurveOverlapCorrespondence2,
        range: CurveParameterRange2,
        inclusion: [bool; 2],
    },
    Product([Option<CurveParameter2>; 2]),
}

impl CurveParameterComponent2 {
    pub(super) fn correspondence(
        source: crate::curve_intersection::CurveOverlapCorrespondence2,
        range: CurveParameterRange2,
        inclusion: [bool; 2],
        charts: Arc<[ComponentParameterChart2; 2]>,
        intervals: [ComponentParameterInterval2; 2],
    ) -> Self {
        Self {
            charts,
            intervals,
            locus: ComponentParameterLocus2::Correspondence {
                source,
                range,
                inclusion,
            },
            swapped: false,
            point_image: None,
        }
    }

    pub(super) fn implicit(
        source: BezierParameterComponentOverlap2,
        charts: Arc<[ComponentParameterChart2; 2]>,
        intervals: [ComponentParameterInterval2; 2],
    ) -> Self {
        let overlap = source.overlap();
        let range = CurveParameterRange2::from_bezier_range(overlap.first_range().clone());
        let inclusion = [overlap.includes_start(), overlap.includes_end()];
        Self::correspondence(
            crate::curve_intersection::CurveOverlapCorrespondence2::ParameterComponent {
                source,
                swapped: false,
            },
            range,
            inclusion,
            charts,
            intervals,
        )
    }

    pub(super) fn product(
        fixed: [Option<CurveParameter2>; 2],
        charts: Arc<[ComponentParameterChart2; 2]>,
        intervals: [ComponentParameterInterval2; 2],
    ) -> Self {
        debug_assert!(fixed.iter().any(Option::is_none));
        Self {
            charts,
            intervals,
            locus: ComponentParameterLocus2::Product(fixed),
            swapped: false,
            point_image: None,
        }
    }

    pub(super) fn with_point_image(mut self, point: CurvePoint2) -> Self {
        self.point_image = Some(point);
        self
    }

    pub(crate) fn point_image(&self) -> Option<&CurvePoint2> {
        self.point_image.as_ref()
    }

    /// Returns the original finite chart that selects this axis's source
    /// frame, and whether the component uses its incident extension.
    pub(crate) fn source_chart_range(
        &self,
        axis: usize,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(CurveParameterRange2, bool)>> {
        let chart = &self.charts[if self.swapped { 1 - axis } else { axis }];
        if let Some((finite, _, _)) = &chart.finite_owner {
            return Ok(Classification::Decided((finite.clone(), true)));
        }
        let mut endpoints = [None, None];
        for (parameter, endpoint) in [chart.interval.range.start(), chart.interval.range.end()]
            .into_iter()
            .zip(&mut endpoints)
        {
            *endpoint = Some(
                match ComponentParameterChart2::image(
                    parameter,
                    &chart.numerator,
                    &chart.denominator,
                    policy,
                )? {
                    Classification::Decided(Some(parameter)) => parameter,
                    Classification::Decided(None) => {
                        return Err(CurveError::Topology(
                            "a finite component chart has a pole at its boundary".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            );
        }
        let [start, end] = endpoints.map(|endpoint| endpoint.unwrap());
        Ok(Classification::Decided((
            CurveParameterRange2::new_validated(start, end),
            false,
        )))
    }

    pub(super) fn swapped(mut self) -> Self {
        self.swapped = !self.swapped;
        self
    }

    pub(crate) fn contains_pair(
        &self,
        first: &CurveParameter2,
        second: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        Ok(self
            .constrain([Some(first), Some(second)], policy)?
            .map(|selection| match selection {
                CurveParameterComponentSelection2::Empty => false,
                CurveParameterComponentSelection2::Selected(_) => true,
                CurveParameterComponentSelection2::NeedsConstraint => {
                    unreachable!("two specified parameters determine a pair")
                }
            }))
    }

    /// Selects a pair using zero, one or two exact source-chart constraints.
    /// A correspondence determines its opposite contact without reconstructing
    /// a Cartesian point. A product must have every free axis constrained.
    pub(crate) fn constrain(
        &self,
        mut constraints: [Option<&CurveParameter2>; 2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveParameterComponentSelection2>> {
        use CurveParameterComponentSelection2::{Empty, NeedsConstraint, Selected};
        if self.swapped {
            constraints.swap(0, 1);
        }
        let mut local = [None, None];
        for (axis, constraint) in constraints.iter().enumerate() {
            let Some(parameter) = constraint else {
                continue;
            };
            let parameter = match self.charts[axis].local_parameter(parameter, policy)? {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => return Ok(Classification::Decided(Empty)),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            match self.intervals[axis].contains(&parameter, policy)? {
                Classification::Decided(true) => local[axis] = Some(parameter),
                Classification::Decided(false) => return Ok(Classification::Decided(Empty)),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        match &self.locus {
            ComponentParameterLocus2::Correspondence {
                source,
                range,
                inclusion,
            } => {
                let axis = if local[0].is_some() {
                    0
                } else if local[1].is_some() {
                    1
                } else {
                    return Ok(Classification::Decided(NeedsConstraint));
                };
                let contains_first =
                    |first: &CurveParameter2| -> CurveResult<Classification<bool>> {
                        match CurveParameterDomain2::new(range, None)
                            .contains_finite_parameter(first, policy)?
                        {
                            Classification::Decided(true) => (),
                            other => return Ok(other),
                        }
                        for (boundary, included) in
                            [(range.start(), inclusion[0]), (range.end(), inclusion[1])]
                        {
                            if !included {
                                match first.same_value(boundary, policy)? {
                                    Classification::Decided(true) => {
                                        return Ok(Classification::Decided(false));
                                    }
                                    Classification::Decided(false) => (),
                                    Classification::Uncertain(reason) => {
                                        return Ok(Classification::Uncertain(reason));
                                    }
                                }
                            }
                        }
                        Ok(Classification::Decided(true))
                    };
                // Exclude a first-axis constraint before asking the algebraic
                // correspondence to transport it outside its certified range.
                if let Some(first) = &local[0] {
                    match contains_first(first)? {
                        Classification::Decided(true) => (),
                        Classification::Decided(false) => {
                            return Ok(Classification::Decided(Empty));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                let mapped = match source.map_parameter(
                    local[axis].as_ref().expect("a constrained contact"),
                    axis == 0,
                    policy,
                )? {
                    Classification::Decided(Some(mapped)) => mapped,
                    Classification::Decided(None) => return Ok(Classification::Decided(Empty)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if let Some(other) = &local[1 - axis] {
                    match mapped.same_value(other, policy)? {
                        Classification::Decided(true) => (),
                        Classification::Decided(false) => {
                            return Ok(Classification::Decided(Empty));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                } else {
                    local[1 - axis] = Some(mapped);
                }
                let first = local[0]
                    .as_ref()
                    .expect("a correspondence determines both contacts");
                match if axis == 0 {
                    Classification::Decided(true)
                } else {
                    contains_first(first)?
                } {
                    Classification::Decided(true) => (),
                    Classification::Decided(false) => return Ok(Classification::Decided(Empty)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            ComponentParameterLocus2::Product(fixed) => {
                for (parameter, fixed) in local.iter_mut().zip(fixed) {
                    if let Some(fixed) = fixed {
                        if let Some(parameter) = parameter {
                            match parameter.same_value(fixed, policy)? {
                                Classification::Decided(true) => (),
                                Classification::Decided(false) => {
                                    return Ok(Classification::Decided(Empty));
                                }
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        } else {
                            *parameter = Some(fixed.clone());
                        }
                    }
                }
                if local.iter().any(Option::is_none) {
                    return Ok(Classification::Decided(NeedsConstraint));
                }
            }
        }
        let local = local.map(|parameter| parameter.expect("every contact is determined"));
        let mut selected = [None, None];
        for (axis, parameter) in local.iter().enumerate() {
            match self.intervals[axis].contains(parameter, policy)? {
                Classification::Decided(true) => (),
                Classification::Decided(false) => return Ok(Classification::Decided(Empty)),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
            selected[axis] = Some(if let Some(original) = constraints[axis] {
                original.clone()
            } else {
                let chart = &self.charts[axis];
                match ComponentParameterChart2::image(
                    parameter,
                    &chart.numerator,
                    &chart.denominator,
                    policy,
                )? {
                    Classification::Decided(Some(parameter)) => {
                        // Recheck chart ownership after forward transport. A
                        // computed contact cannot claim a finite owner's point
                        // through an overlapping incident chart.
                        match chart.local_parameter(&parameter, policy)? {
                            Classification::Decided(Some(_)) => parameter,
                            Classification::Decided(None) => {
                                return Ok(Classification::Decided(Empty));
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    Classification::Decided(None) => return Ok(Classification::Decided(Empty)),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            });
        }
        let mut selected = selected.map(|parameter| parameter.expect("both source contacts"));
        if self.swapped {
            selected.swap(0, 1);
        }
        Ok(Classification::Decided(Selected(selected)))
    }
}

/// Retains an already-complete finite kernel result without discarding its
/// source maps, selected component evidence or constant point-image identity.
pub(super) fn retain_finite_parallel_components(
    first: &BezierParallel2,
    second: &BezierParallel2,
    intersections: &BezierParallelPairIntersectionSet2,
    domains: [CurveParameterDomain2<'_>; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<CurveParameterComponent2>>> {
    debug_assert!(domains.iter().all(|domain| domain.extension.is_none()));
    if intersections.overlaps().is_empty() && intersections.parameter_components().is_empty() {
        return Ok(Classification::Decided(Vec::new()));
    }
    let mut charts = [None, None];
    for (domain, chart) in domains.into_iter().zip(&mut charts) {
        let [lower, upper] = match domain.finite.ordered_endpoints(policy)? {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        *chart = Some(ComponentParameterChart2 {
            numerator: [Real::zero(), Real::one()],
            denominator: [Real::one(), Real::zero()],
            interval: ComponentParameterInterval2 {
                range: CurveParameterRange2::new_validated(lower.clone(), upper.clone()),
                inclusion: domain.inclusion,
            },
            finite_owner: None,
        });
    }
    let charts = charts.map(|chart| chart.expect("both finite source domains were retained"));
    let intervals = charts.each_ref().map(|chart| chart.interval.clone());
    let charts = Arc::new(charts);
    let mut components = Vec::new();
    let mut rational_images = None;
    for overlap in intersections.overlaps() {
        let mut selected = false;
        for source in intersections
            .component_overlaps()
            .iter()
            .filter(|source| source.overlap() == overlap)
        {
            components.push(CurveParameterComponent2::implicit(
                source.clone(),
                Arc::clone(&charts),
                intervals.clone(),
            ));
            selected = true;
        }
        if selected {
            continue;
        }
        if rational_images.is_none() {
            let mut images = [None, None];
            for ((parallel, domain), image) in
                [first, second].into_iter().zip(domains).zip(&mut images)
            {
                *image = Some(
                    match parallel
                        .exact_rational_parallel_component_on_regular_range(domain.finite, policy)?
                    {
                        Classification::Decided(Some(component)) => component.curve().clone(),
                        Classification::Decided(None) => parallel.source().to_rational_bezier()?,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    },
                );
            }
            rational_images =
                Some(images.map(|image| image.expect("both exact source maps were retained")));
        }
        let images = rational_images
            .as_ref()
            .expect("rational source correspondence");
        let correspondence = crate::curve_intersection::CurveOverlapCorrespondence2::rational(
            RationalBezierOverlapParameterCorrespondence2::for_overlap(
                &images[0], &images[1], overlap, policy,
            ),
            overlap,
        );
        components.push(CurveParameterComponent2::correspondence(
            correspondence,
            CurveParameterRange2::from_bezier_range(overlap.first_range().clone()),
            [overlap.includes_start(), overlap.includes_end()],
            Arc::clone(&charts),
            intervals.clone(),
        ));
    }
    for component in intersections.parameter_components() {
        components.push(
            CurveParameterComponent2::product(
                [component.first_parameter(), component.second_parameter()]
                    .map(|parameter| parameter.cloned().map(CurveParameter2::from)),
                Arc::clone(&charts),
                intervals.clone(),
            )
            .with_point_image(component.point().clone()),
        );
    }
    Ok(Classification::Decided(components))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn q(n: i32, d: i32) -> Real {
        (Real::from(n) / Real::from(d)).unwrap()
    }

    fn decided<T>(value: CurveResult<Classification<T>>) -> T {
        match value.unwrap() {
            Classification::Decided(value) => value,
            Classification::Uncertain(_) => {
                panic!("exact retained component decision was uncertain")
            }
        }
    }

    #[test]
    fn incident_charts_keep_poles_and_finite_ownership_excluded() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for direction in [
                BezierParameterRayDirection2::Increasing,
                BezierParameterRayDirection2::Decreasing,
            ] {
                let sign = if direction == BezierParameterRayDirection2::Increasing {
                    1
                } else {
                    -1
                };
                let finite = CurveParameterRange2::new_validated(
                    Real::from(2 * sign).into(),
                    Real::from(4 * sign).into(),
                );
                let anchor = Real::zero();
                let ray = BezierParameterRay2 {
                    anchor: &anchor,
                    direction,
                    barrier: None,
                };
                let range = std::cell::OnceCell::new();
                let chart = decided(ComponentParameterChart2::retain(
                    ParameterComponentChart2 {
                        domain: CurveParameterDomain2::new(&finite, Some(ray)),
                        mapping: ParameterComponentMap2::Incident(ray),
                        range: &range,
                    },
                    &policy,
                ));
                let owned = decided(chart.owned_intervals(&policy));
                assert_eq!(owned.len(), 2);
                for (value, count) in [
                    (q(0, 1), 0),
                    (q(1, 2), 1),
                    (q(2, 3), 0),
                    (q(3, 4), 0),
                    (q(4, 5), 0),
                    (q(9, 10), 1),
                    (q(1, 1), 0),
                ] {
                    let value = CurveParameter2::from(value);
                    assert_eq!(
                        owned
                            .iter()
                            .filter(|interval| decided(interval.contains(&value, &policy)))
                            .count(),
                        count
                    );
                }
                for (source, expected) in [
                    (sign, Some(q(1, 2))),
                    (2 * sign, None),
                    (3 * sign, None),
                    (4 * sign, None),
                    (9 * sign, Some(q(9, 10))),
                    (-sign, None),
                    (0, None),
                ] {
                    let local = decided(chart.local_parameter(&Real::from(source).into(), &policy));
                    match (local, expected) {
                        (Some(local), Some(expected)) => {
                            assert!(decided(local.same_value(&expected.into(), &policy)))
                        }
                        (None, None) => (),
                        _ => panic!("incident chart violated finite ownership"),
                    }
                }
                assert!(
                    decided(ComponentParameterChart2::image(
                        &Real::one().into(),
                        &chart.numerator,
                        &chart.denominator,
                        &policy,
                    ))
                    .is_none()
                );
            }
        }
    }

    #[test]
    fn product_components_keep_fixed_axes_and_operand_roles() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let interval = ComponentParameterInterval2 {
                range: CurveParameterRange2::unit(),
                inclusion: [true, false],
            };
            let chart = ComponentParameterChart2 {
                numerator: [Real::zero(), Real::one()],
                denominator: [Real::one(), Real::zero()],
                interval: interval.clone(),
                finite_owner: None,
            };
            let charts = Arc::new([chart.clone(), chart]);
            let point = CurvePoint2::from(Point2::new(Real::zero(), Real::zero()));
            let fixed = CurveParameter2::from(q(1, 2));
            let component = CurveParameterComponent2::product(
                [Some(fixed.clone()), None],
                charts.clone(),
                [interval.clone(), interval.clone()],
            )
            .with_point_image(point.clone());
            assert!(component.point_image.is_some());
            for (first, second, expected) in [
                (q(1, 2), q(0, 1), true),
                (q(1, 2), q(1, 1), false),
                (q(1, 3), q(1, 2), false),
                (q(1, 2), q(1, 3), true),
            ] {
                let first = CurveParameter2::from(first);
                let second = CurveParameter2::from(second);
                assert_eq!(
                    decided(component.contains_pair(&first, &second, &policy)),
                    expected
                );
                assert_eq!(
                    decided(
                        component
                            .clone()
                            .swapped()
                            .contains_pair(&second, &first, &policy)
                    ),
                    expected
                );
            }
            let product = CurveParameterComponent2::product(
                [None, None],
                charts,
                [interval.clone(), interval],
            )
            .with_point_image(point);
            assert!(decided(product.contains_pair(
                &q(1, 3).into(),
                &q(2, 3).into(),
                &policy
            )));
            assert!(!decided(product.contains_pair(
                &Real::one().into(),
                &q(2, 3).into(),
                &policy
            )));
        }
    }
    #[test]
    fn contact_constraints_distinguish_free_axes_from_empty_components() {
        use CurveParameterComponentSelection2::{Empty, NeedsConstraint, Selected};
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let interval = ComponentParameterInterval2 {
                range: CurveParameterRange2::unit(),
                inclusion: [true, false],
            };
            let chart = ComponentParameterChart2 {
                numerator: [Real::from(2), Real::from(3)],
                denominator: [Real::one(), Real::zero()],
                interval: interval.clone(),
                finite_owner: None,
            };
            let charts = Arc::new([chart.clone(), chart]);
            let fixed = CurveParameter2::from(q(1, 2));
            let free = CurveParameter2::from(Real::from(2).sqrt().unwrap() + Real::from(2));
            let fixed_source = CurveParameter2::from(q(7, 2));
            for swapped in [false, true] {
                let mut component = CurveParameterComponent2::product(
                    [Some(fixed.clone()), None],
                    charts.clone(),
                    [interval.clone(), interval.clone()],
                );
                if swapped {
                    component = component.swapped();
                }
                fn inputs(
                    mut pair: [Option<&CurveParameter2>; 2],
                    swapped: bool,
                ) -> [Option<&CurveParameter2>; 2] {
                    if swapped {
                        pair.swap(0, 1);
                    }
                    pair
                }
                assert!(matches!(
                    decided(component.constrain([None, None], &policy)),
                    NeedsConstraint
                ));
                assert!(matches!(
                    decided(
                        component.constrain(inputs([Some(&fixed_source), None], swapped), &policy)
                    ),
                    NeedsConstraint
                ));
                let Selected(pair) =
                    decided(component.constrain(inputs([None, Some(&free)], swapped), &policy))
                else {
                    panic!("the remaining free axis was constrained");
                };
                assert_eq!(pair[usize::from(!swapped)], free);
                assert!(decided(
                    pair[usize::from(swapped)].same_value(&fixed_source, &policy)
                ));
                assert!(decided(
                    component.contains_pair(&pair[0], &pair[1], &policy)
                ));
                assert!(matches!(
                    decided(component.constrain(inputs([Some(&free), None], swapped), &policy)),
                    Empty
                ));
                let excluded = CurveParameter2::from(Real::from(5));
                assert!(matches!(
                    decided(component.constrain(inputs([None, Some(&excluded)], swapped), &policy)),
                    Empty
                ));
            }
            let product = CurveParameterComponent2::product(
                [None, None],
                charts,
                [interval.clone(), interval],
            );
            assert!(matches!(
                decided(product.constrain([Some(&free), None], &policy)),
                NeedsConstraint
            ));
            assert!(matches!(
                decided(product.constrain([Some(&free), Some(&fixed_source)], &policy)),
                Selected(_)
            ));
        }
    }

    #[test]
    fn one_contact_replays_correspondence_through_both_source_charts() {
        use CurveParameterComponentSelection2::{Empty, NeedsConstraint, Selected};
        let equation =
            BivariatePolynomial::new(vec![vec![Real::zero(), -Real::one()], vec![Real::one()]]);
        let positive = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let selection = decided(select_parameter_component_in_domain(
                &equation,
                &ParameterComponentSelector2::Positive(&positive, None),
                [CurveParameterDomain2::new(&CurveParameterRange2::unit(), None); 2],
                ParameterComponentQuery2::AllComponents(None),
                &policy,
                config,
            ));
            assert_eq!(selection.components.len(), 1);
            let mut component = selection.components.into_iter().next().unwrap();
            let interval = ComponentParameterInterval2 {
                range: CurveParameterRange2::unit(),
                inclusion: [false, false],
            };
            component.charts = Arc::new([
                ComponentParameterChart2 {
                    numerator: [Real::from(2), Real::from(3)],
                    denominator: [Real::one(), Real::zero()],
                    interval: interval.clone(),
                    finite_owner: None,
                },
                ComponentParameterChart2 {
                    numerator: [Real::from(5), Real::from(-2)],
                    denominator: [Real::one(), Real::zero()],
                    interval: interval.clone(),
                    finite_owner: None,
                },
            ]);
            component.intervals = [interval.clone(), interval.clone()];
            let local = Real::from(2).sqrt().unwrap() / Real::from(2);
            let local = local.unwrap();
            let first = CurveParameter2::from(Real::from(2) + Real::from(3) * &local);
            let second = CurveParameter2::from(Real::from(5) - Real::from(2) * &local);
            for swapped in [false, true] {
                let component = if swapped {
                    component.clone().swapped()
                } else {
                    component.clone()
                };
                let expected = if swapped {
                    [&second, &first]
                } else {
                    [&first, &second]
                };
                assert!(matches!(
                    decided(component.constrain([None, None], &policy)),
                    NeedsConstraint
                ));
                for axis in 0..2 {
                    let mut constraints = [None, None];
                    constraints[axis] = Some(expected[axis]);
                    let Selected(pair) = decided(component.constrain(constraints, &policy)) else {
                        panic!("one contact determines the corresponding contact");
                    };
                    assert_eq!(&pair[axis], expected[axis]);
                    assert!(decided(
                        pair[1 - axis].same_value(expected[1 - axis], &policy)
                    ));
                    assert!(decided(
                        component.contains_pair(&pair[0], &pair[1], &policy)
                    ));
                }
                let wrong = CurveParameter2::from(Real::from(4));
                assert!(matches!(
                    decided(component.constrain([Some(expected[0]), Some(&wrong)], &policy)),
                    Empty
                ));
                let endpoint = CurveParameter2::from(Real::from(5));
                assert!(matches!(
                    decided(component.constrain([Some(&endpoint), None], &policy)),
                    Empty
                ));
            }
            // A reversed correspondence range keeps inclusion attached to
            // its stored endpoints, independently of the chart orientation.
            let mut reversed_range = component.clone();
            for chart in Arc::make_mut(&mut reversed_range.charts) {
                chart.interval.inclusion = [true, true];
            }
            for interval in &mut reversed_range.intervals {
                interval.inclusion = [true, true];
            }
            let ComponentParameterLocus2::Correspondence {
                range, inclusion, ..
            } = &mut reversed_range.locus
            else {
                panic!("retained diagonal correspondence");
            };
            *range = CurveParameterRange2::new_validated(Real::one().into(), Real::zero().into());
            *inclusion = [true, false];
            let included = CurveParameter2::from(Real::from(5));
            let excluded = CurveParameter2::from(Real::from(2));
            assert!(matches!(
                decided(reversed_range.constrain([Some(&included), None], &policy)),
                Selected(_)
            ));
            assert!(matches!(
                decided(reversed_range.constrain([Some(&excluded), None], &policy)),
                Empty
            ));
            let included_other = CurveParameter2::from(Real::from(3));
            let excluded_other = CurveParameter2::from(Real::from(5));
            assert!(matches!(
                decided(reversed_range.constrain([None, Some(&included_other)], &policy)),
                Selected(_)
            ));
            assert!(matches!(
                decided(reversed_range.constrain([None, Some(&excluded_other)], &policy)),
                Empty
            ));
            // Forward transport onto a compact incident chart must still
            // subtract that chart's finite owner. Here u maps to u/(1-u),
            // whose points [2,4] belong to a separate finite source chart.
            component.charts = Arc::new([
                ComponentParameterChart2 {
                    numerator: [Real::zero(), Real::one()],
                    denominator: [Real::one(), Real::zero()],
                    interval: interval.clone(),
                    finite_owner: None,
                },
                ComponentParameterChart2 {
                    numerator: [Real::zero(), Real::one()],
                    denominator: [Real::one(), -Real::one()],
                    interval: interval.clone(),
                    finite_owner: Some((
                        CurveParameterRange2::new_validated(
                            Real::from(2).into(),
                            Real::from(4).into(),
                        ),
                        [true; 2],
                        BezierParameterRayDirection2::Increasing,
                    )),
                },
            ]);
            for (local, expected) in [
                (q(1, 2), Some(Real::one())),
                (q(2, 3), None),
                (q(3, 4), None),
                (q(4, 5), None),
                (q(9, 10), Some(Real::from(9))),
            ] {
                let parameter = CurveParameter2::from(local);
                let result = decided(component.constrain([Some(&parameter), None], &policy));
                match (result, expected) {
                    (Selected(pair), Some(expected)) => {
                        assert_eq!(pair[0], parameter);
                        assert!(decided(pair[1].same_value(&expected.into(), &policy)));
                    }
                    (Empty, None) => (),
                    _ => panic!("transported contact violated finite chart ownership"),
                }
            }
        }
    }
}
