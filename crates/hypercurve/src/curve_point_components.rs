//! Complete parameter loci and their intersection in the retained source atlas.
use super::*;
use std::cmp::Ordering;

fn decided<T>(value: Classification<T>) -> ExactCurveResult<T> {
    ExactCurveError::decided(CurveOperation2::Intersection, value)
}
fn compare(
    a: &CurveParameter2,
    b: &CurveParameter2,
    policy: &CurveContext,
) -> ExactCurveResult<Ordering> {
    decided(a.cmp_by_refinement(b, policy).map_err(|cause| {
        ExactCurveError::invalid_unattributed(CurveOperation2::Intersection, cause)
    })?)
}
impl CurveLocationComponent2 {
    pub(super) fn new(
        span_index: usize,
        span_range: CurveSpanRange2,
        parameters: CurveParameterSet2,
        endpoint_inclusion: [bool; 2],
    ) -> Self {
        Self {
            span_index,
            span_range,
            parameters,
            endpoint_inclusion,
        }
    }
    /// Original source span index, including distinct one-sided knot charts.
    pub const fn span_index(&self) -> usize {
        self.span_index
    }
    /// Original affine chart from local to authored parameters.
    pub const fn span_range(&self) -> &CurveSpanRange2 {
        &self.span_range
    }
    /// Local component closure, without materializing selected endpoints.
    /// Apply `endpoint_inclusion` to recover the represented set.
    pub const fn local_parameters(&self) -> &CurveParameterSet2 {
        &self.parameters
    }
    /// Inclusion at the local range's start and end; singles include their point.
    pub const fn endpoint_inclusion(&self) -> [bool; 2] {
        self.endpoint_inclusion
    }
    /// The component closure in the authored parameter chart. Range orientation and
    /// endpoint inclusion correspond to the original local range.
    pub fn parameters(&self) -> ExactCurveResult<CurveParameterSet2> {
        let map = |p: &CurveParameter2| {
            CurveLocation2::new(self.span_index, self.span_range.clone(), p.clone()).parameter()
        };
        Ok(match &self.parameters {
            CurveParameterSet2::Single(p) => CurveParameterSet2::Single(map(p)?),
            CurveParameterSet2::Range(r) => CurveParameterSet2::Range(
                CurveParameterRange2::try_new(map(r.start())?, map(r.end())?)?,
            ),
        })
    }
    fn global_bounds(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<([CurveParameter2; 2], [bool; 2])> {
        let (a, b) = match &self.parameters {
            CurveParameterSet2::Single(p) => (p, p),
            CurveParameterSet2::Range(r) => (r.start(), r.end()),
        };
        let map = |p: &CurveParameter2| {
            decided(
                CurveLocation2::new(self.span_index, self.span_range.clone(), p.clone())
                    .parameter_with_policy(policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid_unattributed(CurveOperation2::Intersection, cause)
                    })?,
            )
        };
        let mut bounds = [map(a)?, map(b)?];
        let mut inclusion = self.endpoint_inclusion;
        if compare(&bounds[0], &bounds[1], policy)?.is_gt() {
            bounds.swap(0, 1);
            inclusion.swap(0, 1);
        }
        Ok((bounds, inclusion))
    }
    fn local_at(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveParameter2> {
        let (a, b) = self.span_range.endpoints();
        let inverse = (Real::one() / (b - a)).map_err(|cause| {
            ExactCurveError::invalid_unattributed(
                CurveOperation2::Intersection,
                CurveError::from(cause),
            )
        })?;
        decided(
            parameter
                .affine_image_unbounded(&inverse, &(-a * &inverse), policy)
                .map_err(|cause| {
                    ExactCurveError::invalid_unattributed(CurveOperation2::Intersection, cause)
                })?,
        )
    }
    fn contains(&self, p: &CurveParameter2, policy: &CurveContext) -> ExactCurveResult<bool> {
        let ([a, b], [ia, ib]) = self.global_bounds(policy)?;
        let lower = compare(p, &a, policy)?;
        let upper = compare(p, &b, policy)?;
        Ok((lower.is_gt() || (lower.is_eq() && ia)) && (upper.is_lt() || (upper.is_eq() && ib)))
    }
    fn from_location(location: CurveLocation2) -> Self {
        Self::new(
            location.span_index,
            location.span_range,
            CurveParameterSet2::Single(location.local_parameter),
            [true, true],
        )
    }
}
impl CurvePointLocations2 {
    fn components(&self) -> Vec<CurveLocationComponent2> {
        match self {
            Self::EntireCurve => {
                unreachable!("whole-domain intersections use the identity directly")
            }
            Self::Locations(locations) => locations
                .iter()
                .cloned()
                .map(CurveLocationComponent2::from_location)
                .collect(),
            Self::Components(components) => components.clone(),
        }
    }
}
impl Curve2 {
    fn same_component_source(
        &self,
        a: &CurveLocationComponent2,
        b: &CurveLocationComponent2,
        p: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        if a.span_index == b.span_index && a.span_range == b.span_range {
            return Ok(true);
        }
        let spans = self.source_spans(policy, CurveOperation2::Intersection)?;
        let point = |c: &CurveLocationComponent2| -> ExactCurveResult<CurvePoint2> {
            let span = spans.get(c.span_index).ok_or_else(|| {
                ExactCurveError::invalid_unattributed(
                    CurveOperation2::Intersection,
                    CurveError::InvalidCurveParameter,
                )
            })?;
            Ok(span
                .curve()
                .point_at_with_policy(&c.local_at(p, policy)?, policy)?
                .into_value())
        };
        decided(point(a)?.same_point(&point(b)?, policy))
    }
    /// Intersects two complete point-preimage results from this same unchanged
    /// curve and source atlas. Parameter equality alone does not identify
    /// distinct source germs at a discontinuous knot. Original open endpoints
    /// and selected parameter evidence survive the intersection.
    pub fn intersect_point_locations(
        &self,
        first: &CurvePointLocations2,
        second: &CurvePointLocations2,
    ) -> ExactCurveResult<CurvePointLocations2> {
        resolve_certified_operation(&crate::policy::principal_context(), |policy| {
            if matches!(first, CurvePointLocations2::EntireCurve) {
                return Ok(second.clone());
            }
            if matches!(second, CurvePointLocations2::EntireCurve) {
                return Ok(first.clone());
            }
            let mut result = Vec::new();
            for a in first.components() {
                let (ab, ai) = a.global_bounds(policy)?;
                for b in second.components() {
                    let (bb, bi) = b.global_bounds(policy)?;
                    let (lo, li) = match compare(&ab[0], &bb[0], policy)? {
                        Ordering::Less => (bb[0].clone(), bi[0]),
                        Ordering::Greater => (ab[0].clone(), ai[0]),
                        Ordering::Equal => (ab[0].clone(), ai[0] && bi[0]),
                    };
                    let (hi, hi_included) = match compare(&ab[1], &bb[1], policy)? {
                        Ordering::Less => (ab[1].clone(), ai[1]),
                        Ordering::Greater => (bb[1].clone(), bi[1]),
                        Ordering::Equal => (ab[1].clone(), ai[1] && bi[1]),
                    };
                    let set = match compare(&lo, &hi, policy)? {
                        Ordering::Greater => continue,
                        Ordering::Equal => {
                            if !li
                                || !hi_included
                                || !self.same_component_source(&a, &b, &lo, policy)?
                            {
                                continue;
                            }
                            CurveParameterSet2::Single(a.local_at(&lo, policy)?)
                        }
                        Ordering::Less => {
                            // Native source atlas interiors are disjoint. A
                            // cross-chart common endpoint is handled above.
                            if a.span_index != b.span_index || a.span_range != b.span_range {
                                return Err(ExactCurveError::blocked(
                                    CurveOperation2::Intersection,
                                    self.family(),
                                    UncertaintyReason::Unsupported,
                                ));
                            }
                            CurveParameterSet2::Range(CurveParameterRange2::new_validated(
                                a.local_at(&lo, policy)?,
                                a.local_at(&hi, policy)?,
                            ))
                        }
                    };
                    result.push(CurveLocationComponent2::new(
                        a.span_index,
                        a.span_range.clone(),
                        set,
                        [li, hi_included],
                    ));
                }
            }
            self.normalize_point_components(result, policy)
        })
        .map(CurveOutcome::into_value)
    }
    pub(super) fn normalize_point_components(
        &self,
        components: Vec<CurveLocationComponent2>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePointLocations2> {
        let mut ranges = Vec::new();
        let mut points = Vec::new();
        for component in components {
            match component.parameters {
                CurveParameterSet2::Single(_) => points.push(component),
                CurveParameterSet2::Range(_) => ranges.push(component),
            }
        }
        let mut result: Vec<CurveLocationComponent2> = Vec::new();
        for point in points {
            let p = point.global_bounds(policy)?.0[0].clone();
            let mut duplicate = false;
            for previous in ranges.iter().chain(&result) {
                if previous.contains(&p, policy)?
                    && self.same_component_source(&point, previous, &p, policy)?
                {
                    duplicate = true;
                    break;
                }
            }
            if !duplicate {
                result.push(point);
            }
        }
        result.extend(ranges);
        // Exact insertion sorting propagates undecided comparisons. No rounded
        // endpoint or selected-root payload is used as an ordering key.
        let mut sorted: Vec<CurveLocationComponent2> = Vec::new();
        for component in result {
            let p = component.global_bounds(policy)?.0[0].clone();
            let mut index = 0;
            while index < sorted.len()
                && !compare(&sorted[index].global_bounds(policy)?.0[0], &p, policy)?.is_gt()
            {
                index += 1;
            }
            sorted.insert(index, component);
        }
        if sorted
            .iter()
            .all(|c| matches!(c.parameters, CurveParameterSet2::Single(_)))
        {
            return Ok(CurvePointLocations2::Locations(
                sorted
                    .into_iter()
                    .map(|c| {
                        let CurveParameterSet2::Single(p) = c.parameters else {
                            unreachable!()
                        };
                        CurveLocation2::new(c.span_index, c.span_range, p)
                    })
                    .collect(),
            ));
        }
        Ok(CurvePointLocations2::Components(sorted))
    }
}
