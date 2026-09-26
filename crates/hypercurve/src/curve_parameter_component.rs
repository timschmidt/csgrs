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
    finite_owner: Option<(CurveParameterRange2, BezierParameterRayDirection2)>,
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
                    Some((chart.domain.finite.clone(), ray.direction)),
                )
            }
        };
        let closed = finite_owner.is_none();
        Ok(Classification::Decided(Self {
            numerator,
            denominator,
            interval: ComponentParameterInterval2 {
                range,
                inclusion: [closed; 2],
            },
            finite_owner,
        }))
    }

    /// Partitions the compact chart after subtracting the closed finite owner.
    /// Open cut ends prevent an incident component from republishing that owner.
    pub(super) fn owned_intervals(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<ComponentParameterInterval2>>> {
        let Some((finite, direction)) = &self.finite_owner else {
            return Ok(Classification::Decided(vec![self.interval.clone()]));
        };
        let endpoints = match finite.ordered_endpoints(policy)? {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let endpoints = if *direction == BezierParameterRayDirection2::Increasing {
            endpoints
        } else {
            [endpoints[1], endpoints[0]]
        };
        let anchor = CurveParameter2::from(self.numerator[0].clone());
        let inverse_numerator = [-self.numerator[0].clone(), self.denominator[0].clone()];
        let inverse_denominator = [self.numerator[1].clone(), -self.denominator[1].clone()];
        let range = &self.interval.range;
        let mut cuts = [None, None];
        for (endpoint, cut) in endpoints.into_iter().zip(&mut cuts) {
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
            *cut = Some(mapped);
        }
        let [lower, upper] = cuts.map(|cut| cut.expect("both finite boundaries were transported"));
        let mut intervals = Vec::with_capacity(2);
        for (start, end, inclusion) in [
            (range.start(), &lower, [self.interval.inclusion[0], false]),
            (&upper, range.end(), [false, self.interval.inclusion[1]]),
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
        if let Some((finite, _)) = &self.finite_owner {
            match CurveParameterDomain2::new(finite, None)
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
        let (first, second) = if self.swapped {
            (second, first)
        } else {
            (first, second)
        };
        let mut local = [None, None];
        for (axis, parameter) in [first, second].into_iter().enumerate() {
            let parameter = match self.charts[axis].local_parameter(parameter, policy)? {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => return Ok(Classification::Decided(false)),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            match self.intervals[axis].contains(&parameter, policy)? {
                Classification::Decided(true) => local[axis] = Some(parameter),
                other => return Ok(other),
            }
        }
        let [first, second] =
            local.map(|parameter| parameter.expect("both component parameters are owned"));
        match &self.locus {
            ComponentParameterLocus2::Correspondence {
                source,
                range,
                inclusion,
            } => {
                match CurveParameterDomain2::new(range, None)
                    .contains_finite_parameter(&first, policy)?
                {
                    Classification::Decided(true) => {}
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
                            Classification::Decided(false) => {}
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                }
                match source.map_parameter(&first, true, policy)? {
                    Classification::Decided(Some(mapped)) => mapped.same_value(&second, policy),
                    Classification::Decided(None) => Ok(Classification::Decided(false)),
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                }
            }
            ComponentParameterLocus2::Product(fixed) => {
                for (parameter, fixed) in [first, second].iter().zip(fixed) {
                    if let Some(fixed) = fixed {
                        match parameter.same_value(fixed, policy)? {
                            Classification::Decided(true) => {}
                            other => return Ok(other),
                        }
                    }
                }
                Ok(Classification::Decided(true))
            }
        }
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
                inclusion: [true, true],
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
}
