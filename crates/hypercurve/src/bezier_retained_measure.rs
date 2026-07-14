//! Exact measurements over retained Bezier/conic carriers.
//!
//! Retained regions may contain algebraic endpoint-image fragments that are not
//! native Bezier subcurves yet.  This module therefore exposes measurements
//! whose scope is explicit.  An endpoint envelope bounds retained boundary
//! endpoints only: native endpoints contribute exact point coordinates, and
//! algebraic endpoint images contribute the certified isolating intervals of
//! their represented coordinates.  It never samples an algebraic root and it
//! does not claim curve-interior extrema.
//!
//! A curve envelope is stronger: it consumes materialized native Bezier/conic
//! carriers and includes exact coordinate extrema from derivative roots.
//! Polynomial Bezier extrema use the Bernstein derivative identities described
//! by the Bernstein and de Casteljau curve model, and the
//! rational-quadratic path reuses the crate's quotient-derivative conic bounds.
//! Algebraic endpoint-image fragments can also contribute when they retain the
//! source curve that generated the algebraic split: the envelope materializes
//! the source subcurve over the certified parameter-interval hull and then
//! includes that exact native bound as a conservative overbound of the true
//! algebraic subrange.  Fragments without source-curve evidence remain
//! unsupported. This preserves the construction/decision split. The
//! broad-phase role mirrors sweep-line candidate filtering.

use hyperreal::Real;
use hypersolve::AlgebraicRootRepresentation;

use crate::classify::compare_reals;
use crate::{
    Aabb2, Axis2, BezierEndpointPointImage2, BezierParameter2, BezierSplitFragment2,
    BezierSubcurve2, Classification, CurvePolicy, CurveRegion2, CurveRegionBoundaryLoop2, Point2,
    UncertaintyReason,
};

/// Exact curve-interior envelope for retained Bezier/conic carriers.
///
/// Native subcurves contribute endpoint and derivative-root extrema. Algebraic
/// endpoint-image fragments contribute only when they carry their source
/// curve; in that case the certified parameter intervals choose a native
/// source subcurve whose bounds conservatively overbound the algebraic
/// subrange. Endpoint images alone are still rejected because they do not prove
/// any interior extrema.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierRetainedCurveEnvelope2 {
    envelope: Aabb2,
    exact_fragment_count: usize,
    native_fragment_count: usize,
    algebraic_fragment_count: usize,
    fragment_source_kinds: Vec<BezierRetainedEnvelopeSourceKind>,
}

/// Source class of one retained envelope witness.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierRetainedEnvelopeSourceKind {
    /// A materialized native Bezier/conic object contributed the witness.
    Native,
    /// A retained algebraic endpoint-image carrier contributed the witness.
    Algebraic,
}

impl BezierRetainedCurveEnvelope2 {
    /// Constructs a curve-interior envelope for a retained region.
    ///
    /// Empty regions are unsupported because there is no finite neutral
    /// envelope. A retained algebraic endpoint-image fragment must carry its
    /// source curve; endpoint-only evidence is unsupported.
    pub fn from_region(region: &CurveRegion2, policy: &CurvePolicy) -> Classification<Self> {
        let mut accumulator = CurveEnvelopeAccumulator::default();
        for boundary_loop in region.boundary_loops() {
            match accumulator.include_loop(boundary_loop, policy) {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
        accumulator.finish()
    }

    /// Constructs a curve-interior envelope for one retained boundary loop.
    pub fn from_loop(
        boundary_loop: &CurveRegionBoundaryLoop2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let mut accumulator = CurveEnvelopeAccumulator::default();
        match accumulator.include_loop(boundary_loop, policy) {
            Classification::Decided(()) => accumulator.finish(),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Returns the exact curve-interior envelope.
    pub const fn envelope(&self) -> &Aabb2 {
        &self.envelope
    }

    /// Returns how many retained fragments contributed certified curve bounds.
    pub const fn exact_fragment_count(&self) -> usize {
        self.exact_fragment_count
    }

    /// Returns how many materialized native fragments contributed exact bounds.
    pub const fn native_fragment_count(&self) -> usize {
        self.native_fragment_count
    }

    /// Returns how many algebraic endpoint-image fragments contributed source-curve bounds.
    pub const fn algebraic_fragment_count(&self) -> usize {
        self.algebraic_fragment_count
    }

    /// Returns true when algebraic source-curve evidence contributed to the envelope.
    pub const fn has_algebraic_fragments(&self) -> bool {
        self.algebraic_fragment_count > 0
    }

    /// Returns one source-kind witness per retained fragment that contributed bounds.
    pub fn fragment_source_kinds(&self) -> &[BezierRetainedEnvelopeSourceKind] {
        &self.fragment_source_kinds
    }
}

/// Exact endpoint envelope for a retained Bezier region or loop.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierRetainedEndpointEnvelope2 {
    envelope: Aabb2,
    native_endpoint_count: usize,
    algebraic_endpoint_count: usize,
    endpoint_source_kinds: Vec<BezierRetainedEnvelopeSourceKind>,
}

impl BezierRetainedEndpointEnvelope2 {
    /// Constructs an endpoint envelope for a retained region.
    ///
    /// Empty regions are unsupported because there is no finite neutral
    /// envelope. Retained algebraic fragments must provide endpoint point
    /// images for every endpoint they contribute; otherwise the envelope is
    /// explicit boundary uncertainty rather than a partial box.
    pub fn from_region(region: &CurveRegion2, policy: &CurvePolicy) -> Classification<Self> {
        let mut accumulator = EndpointEnvelopeAccumulator::default();
        for boundary_loop in region.boundary_loops() {
            match accumulator.include_loop(boundary_loop, policy) {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
        accumulator.finish(policy)
    }

    /// Constructs an endpoint envelope for one retained boundary loop.
    pub fn from_loop(
        boundary_loop: &CurveRegionBoundaryLoop2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let mut accumulator = EndpointEnvelopeAccumulator::default();
        match accumulator.include_loop(boundary_loop, policy) {
            Classification::Decided(()) => accumulator.finish(policy),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Returns the conservative endpoint envelope.
    pub const fn envelope(&self) -> &Aabb2 {
        &self.envelope
    }

    /// Returns how many native endpoint points contributed to this envelope.
    pub const fn native_endpoint_count(&self) -> usize {
        self.native_endpoint_count
    }

    /// Returns how many algebraic endpoint images contributed to this envelope.
    pub const fn algebraic_endpoint_count(&self) -> usize {
        self.algebraic_endpoint_count
    }

    /// Returns true when at least one represented algebraic endpoint image
    /// contributed interval evidence.
    pub const fn has_algebraic_endpoints(&self) -> bool {
        self.algebraic_endpoint_count > 0
    }

    /// Returns one source-kind witness per endpoint image that contributed bounds.
    pub fn endpoint_source_kinds(&self) -> &[BezierRetainedEnvelopeSourceKind] {
        &self.endpoint_source_kinds
    }
}

#[derive(Clone, Debug)]
struct CoordinateInterval {
    lower: Real,
    upper: Real,
}

#[derive(Clone, Debug)]
struct EndpointInterval {
    x: CoordinateInterval,
    y: CoordinateInterval,
    kind: BezierRetainedEnvelopeSourceKind,
}

#[derive(Default)]
struct EndpointEnvelopeAccumulator {
    min_x: Option<Real>,
    min_y: Option<Real>,
    max_x: Option<Real>,
    max_y: Option<Real>,
    native_endpoint_count: usize,
    algebraic_endpoint_count: usize,
    endpoint_source_kinds: Vec<BezierRetainedEnvelopeSourceKind>,
}

#[derive(Default)]
struct CurveEnvelopeAccumulator {
    envelope: Option<Aabb2>,
    exact_fragment_count: usize,
    native_fragment_count: usize,
    algebraic_fragment_count: usize,
    fragment_source_kinds: Vec<BezierRetainedEnvelopeSourceKind>,
}

impl CurveEnvelopeAccumulator {
    fn include_loop(
        &mut self,
        boundary_loop: &CurveRegionBoundaryLoop2,
        policy: &CurvePolicy,
    ) -> Classification<()> {
        for fragment in boundary_loop.fragments() {
            match self.include_fragment(fragment, policy) {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
        Classification::Decided(())
    }

    fn include_fragment(
        &mut self,
        fragment: &BezierSplitFragment2,
        policy: &CurvePolicy,
    ) -> Classification<()> {
        let (curve_box, kind) = match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => {
                match retained_curve_bounds(curve, policy) {
                    Classification::Decided(curve_box) => {
                        (curve_box, BezierRetainedEnvelopeSourceKind::Native)
                    }
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
            BezierSplitFragment2::AlgebraicEndpointImages {
                start,
                end,
                source_curve,
                start_image,
                end_image,
                ..
            } => {
                let Some(source_curve) = source_curve else {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                };
                match retained_algebraic_source_bounds(
                    source_curve,
                    start,
                    end,
                    start_image.as_ref(),
                    end_image.as_ref(),
                    policy,
                ) {
                    Classification::Decided(curve_box) => {
                        (curve_box, BezierRetainedEnvelopeSourceKind::Algebraic)
                    }
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
            BezierSplitFragment2::Unresolved { .. } => {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
        };
        self.envelope = match self.envelope.take() {
            Some(envelope) => match envelope.union(&curve_box, policy) {
                Classification::Decided(merged) => Some(merged),
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            },
            None => Some(curve_box),
        };
        self.exact_fragment_count += 1;
        match kind {
            BezierRetainedEnvelopeSourceKind::Native => self.native_fragment_count += 1,
            BezierRetainedEnvelopeSourceKind::Algebraic => self.algebraic_fragment_count += 1,
        }
        self.fragment_source_kinds.push(kind);
        Classification::Decided(())
    }

    fn finish(self) -> Classification<BezierRetainedCurveEnvelope2> {
        let Some(envelope) = self.envelope else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        Classification::Decided(BezierRetainedCurveEnvelope2 {
            envelope,
            exact_fragment_count: self.exact_fragment_count,
            native_fragment_count: self.native_fragment_count,
            algebraic_fragment_count: self.algebraic_fragment_count,
            fragment_source_kinds: self.fragment_source_kinds,
        })
    }
}

fn retained_algebraic_source_interval_bounds(
    source_curve: &BezierSubcurve2,
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurvePolicy,
) -> Classification<Aabb2> {
    let (range_start, range_end) = match parameter_interval_hull(start, end, policy) {
        Classification::Decided(range) => range,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let subcurve = match subcurve_between_exact(source_curve, &range_start, &range_end, policy) {
        Classification::Decided(subcurve) => subcurve,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    retained_curve_bounds(&subcurve, policy)
}

fn retained_algebraic_source_bounds(
    source_curve: &BezierSubcurve2,
    start: &BezierParameter2,
    end: &BezierParameter2,
    start_image: Option<&crate::BezierAlgebraicEndpointImage2>,
    end_image: Option<&crate::BezierAlgebraicEndpointImage2>,
    policy: &CurvePolicy,
) -> Classification<Aabb2> {
    match retained_algebraic_source_extrema_bounds(
        source_curve,
        start,
        end,
        start_image,
        end_image,
        policy,
    ) {
        Classification::Decided(Some(bounds)) => Classification::Decided(bounds),
        Classification::Decided(None) => {
            retained_algebraic_source_interval_bounds(source_curve, start, end, policy)
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

/// Builds a retained algebraic-fragment envelope from endpoint images and
/// certified source-curve extrema.
///
/// This is stronger than the interval-hull fallback because it keeps the
/// algebraic endpoint coordinates as constructed exact objects and admits only
/// derivative roots whose exact parameter is certified inside the retained
/// range.  That is the exactness model's object/predicate boundary: construct endpoint and
/// extremum evidence first, then branch only on certified ordering.  The
/// derivative-root extrema are the standard Bezier bounds from the Bernstein curve model.
fn retained_algebraic_source_extrema_bounds(
    source_curve: &BezierSubcurve2,
    start: &BezierParameter2,
    end: &BezierParameter2,
    start_image: Option<&crate::BezierAlgebraicEndpointImage2>,
    end_image: Option<&crate::BezierAlgebraicEndpointImage2>,
    policy: &CurvePolicy,
) -> Classification<Option<Aabb2>> {
    let Some(start_endpoint) =
        parameter_endpoint_interval(source_curve, start, start_image, policy)
    else {
        return Classification::Decided(None);
    };
    let Some(end_endpoint) = parameter_endpoint_interval(source_curve, end, end_image, policy)
    else {
        return Classification::Decided(None);
    };

    let mut accumulator = EndpointEnvelopeAccumulator::default();
    match accumulator.include_endpoint(start_endpoint, policy) {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }
    match accumulator.include_endpoint(end_endpoint, policy) {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    let monotone_parameters = match retained_curve_monotone_parameters(source_curve, policy) {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    for parameter in monotone_parameters {
        match exact_parameter_inside_retained_range(start, end, &parameter, policy) {
            Some(true) => {
                let point = match source_curve_point_at(source_curve, parameter, policy) {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                };
                match accumulator.include_endpoint(native_endpoint_interval(&point), policy) {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
            Some(false) => {}
            None => return Classification::Decided(None),
        }
    }

    match accumulator.finish(policy) {
        Classification::Decided(envelope) => Classification::Decided(Some(envelope.envelope)),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

/// Returns endpoint interval evidence for an exact or algebraic fragment
/// boundary.
///
/// Exact parameters are evaluated directly on the source curve. Algebraic
/// parameters consume their retained endpoint image; if that image is absent,
/// the caller must fall back to a coarser retained source envelope.
fn parameter_endpoint_interval(
    source_curve: &BezierSubcurve2,
    parameter: &BezierParameter2,
    image: Option<&crate::BezierAlgebraicEndpointImage2>,
    policy: &CurvePolicy,
) -> Option<EndpointInterval> {
    match parameter {
        BezierParameter2::Exact(value) => {
            match source_curve_point_at(source_curve, value.clone(), policy) {
                Classification::Decided(point) => Some(native_endpoint_interval(&point)),
                Classification::Uncertain(_) => None,
            }
        }
        BezierParameter2::Algebraic(_) => {
            image.and_then(|image| algebraic_endpoint_interval(image.point()))
        }
    }
}

/// Returns unique exact source parameters where x or y can have a local
/// extremum.
fn retained_curve_monotone_parameters(
    source_curve: &BezierSubcurve2,
    policy: &CurvePolicy,
) -> Classification<Vec<Real>> {
    let mut parameters = Vec::new();
    for axis in [Axis2::X, Axis2::Y] {
        let axis_parameters = match source_curve {
            BezierSubcurve2::Quadratic(curve) => curve.axis_monotone_parameters(axis, policy),
            BezierSubcurve2::Cubic(curve) => curve.axis_monotone_parameters(axis, policy),
            BezierSubcurve2::RationalQuadratic(curve) => {
                curve.axis_monotone_parameters(axis, policy)
            }
            BezierSubcurve2::Rational(_) => {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            }
        };
        let axis_parameters = match axis_parameters {
            Classification::Decided(axis_parameters) => axis_parameters,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        for parameter in axis_parameters {
            if push_unique_real(&mut parameters, parameter, policy).is_none() {
                return Classification::Uncertain(UncertaintyReason::Ordering);
            }
        }
    }
    Classification::Decided(parameters)
}

/// Certifies whether an exact source parameter lies inside an ordered retained
/// fragment range.
///
/// The comparison uses [`BezierParameter2::cmp_by_interval`], so overlapping
/// isolating intervals deliberately produce `None` and force the conservative
/// interval-hull fallback instead of sampling the algebraic root.
fn exact_parameter_inside_retained_range(
    start: &BezierParameter2,
    end: &BezierParameter2,
    parameter: &Real,
    policy: &CurvePolicy,
) -> Option<bool> {
    let parameter = BezierParameter2::Exact(parameter.clone());
    let start_cmp = match start.cmp_by_interval(&parameter, policy).ok()? {
        Classification::Decided(ordering) => ordering,
        Classification::Uncertain(_) => return None,
    };
    let end_cmp = match parameter.cmp_by_interval(end, policy).ok()? {
        Classification::Decided(ordering) => ordering,
        Classification::Uncertain(_) => return None,
    };
    Some(start_cmp != std::cmp::Ordering::Greater && end_cmp != std::cmp::Ordering::Greater)
}

/// Evaluates a retained source curve at an exact Bezier parameter.
fn source_curve_point_at(
    source_curve: &BezierSubcurve2,
    parameter: Real,
    policy: &CurvePolicy,
) -> Classification<Point2> {
    match source_curve {
        BezierSubcurve2::Quadratic(curve) => Classification::Decided(curve.point_at(parameter)),
        BezierSubcurve2::Cubic(curve) => Classification::Decided(curve.point_at(parameter)),
        BezierSubcurve2::RationalQuadratic(curve) => curve.point_at(parameter, policy),
        BezierSubcurve2::Rational(curve) => curve.point_at_classified(&parameter, policy),
    }
}

/// Pushes an exact parameter unless an equal one is already present.
fn push_unique_real(values: &mut Vec<Real>, value: Real, policy: &CurvePolicy) -> Option<()> {
    if values
        .iter()
        .any(|existing| compare_reals(existing, &value, policy) == Some(std::cmp::Ordering::Equal))
    {
        return Some(());
    }
    values.push(value);
    Some(())
}

fn parameter_interval_hull(
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurvePolicy,
) -> Classification<(Real, Real)> {
    let start_interval = match start.known_interval(policy) {
        Ok(Classification::Decided(interval)) => interval,
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    let end_interval = match end.known_interval(policy) {
        Ok(Classification::Decided(interval)) => interval,
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };

    let lower = match compare_reals(start_interval.start(), end_interval.start(), policy) {
        Some(std::cmp::Ordering::Greater) => end_interval.start().clone(),
        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal) => {
            start_interval.start().clone()
        }
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    let upper = match compare_reals(start_interval.end(), end_interval.end(), policy) {
        Some(std::cmp::Ordering::Less) => end_interval.end().clone(),
        Some(std::cmp::Ordering::Greater | std::cmp::Ordering::Equal) => {
            start_interval.end().clone()
        }
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    if compare_reals(&lower, &upper, policy) == Some(std::cmp::Ordering::Greater) {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    }
    Classification::Decided((lower, upper))
}

fn subcurve_between_exact(
    curve: &BezierSubcurve2,
    start: &Real,
    end: &Real,
    policy: &CurvePolicy,
) -> Classification<BezierSubcurve2> {
    let result = match curve {
        BezierSubcurve2::Quadratic(curve) => curve
            .subcurve_between_exact(start, end, policy)
            .map(BezierSubcurve2::Quadratic),
        BezierSubcurve2::Cubic(curve) => curve
            .subcurve_between_exact(start, end, policy)
            .map(BezierSubcurve2::Cubic),
        BezierSubcurve2::RationalQuadratic(curve) => curve
            .subcurve_between_exact(start, end, policy)
            .map(BezierSubcurve2::RationalQuadratic),
        BezierSubcurve2::Rational(curve) => {
            match curve.subcurve_between_exact(start, end, policy) {
                Ok(Classification::Decided(curve)) => Ok(BezierSubcurve2::Rational(curve)),
                Ok(Classification::Uncertain(_)) => {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                }
                Err(error) => Err(error),
            }
        }
    };
    match result {
        Ok(curve) => Classification::Decided(curve),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

fn retained_curve_bounds(curve: &BezierSubcurve2, policy: &CurvePolicy) -> Classification<Aabb2> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::Cubic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(policy),
    }
}

impl EndpointEnvelopeAccumulator {
    fn include_loop(
        &mut self,
        boundary_loop: &CurveRegionBoundaryLoop2,
        policy: &CurvePolicy,
    ) -> Classification<()> {
        for fragment in boundary_loop.fragments() {
            match self.include_fragment(fragment, policy) {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
        Classification::Decided(())
    }

    fn include_fragment(
        &mut self,
        fragment: &BezierSplitFragment2,
        policy: &CurvePolicy,
    ) -> Classification<()> {
        match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => {
                let (start, end) = curve.endpoints();
                match self.include_endpoint(native_endpoint_interval(&start), policy) {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
                self.include_endpoint(native_endpoint_interval(&end), policy)
            }
            BezierSplitFragment2::AlgebraicEndpointImages {
                start_image,
                end_image,
                ..
            } => {
                let Some(start_image) = start_image else {
                    return Classification::Uncertain(UncertaintyReason::Boundary);
                };
                let Some(end_image) = end_image else {
                    return Classification::Uncertain(UncertaintyReason::Boundary);
                };
                let Some(start) = algebraic_endpoint_interval(start_image.point()) else {
                    return Classification::Uncertain(UncertaintyReason::Boundary);
                };
                let Some(end) = algebraic_endpoint_interval(end_image.point()) else {
                    return Classification::Uncertain(UncertaintyReason::Boundary);
                };
                match self.include_endpoint(start, policy) {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
                self.include_endpoint(end, policy)
            }
            BezierSplitFragment2::Unresolved { .. } => {
                Classification::Uncertain(UncertaintyReason::Boundary)
            }
        }
    }

    fn include_endpoint(
        &mut self,
        endpoint: EndpointInterval,
        policy: &CurvePolicy,
    ) -> Classification<()> {
        if self
            .include_coordinate(&endpoint.x.lower, &endpoint.x.upper, Axis::X, policy)
            .is_none()
            || self
                .include_coordinate(&endpoint.y.lower, &endpoint.y.upper, Axis::Y, policy)
                .is_none()
        {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        }
        match endpoint.kind {
            BezierRetainedEnvelopeSourceKind::Native => self.native_endpoint_count += 1,
            BezierRetainedEnvelopeSourceKind::Algebraic => self.algebraic_endpoint_count += 1,
        }
        self.endpoint_source_kinds.push(endpoint.kind);
        Classification::Decided(())
    }

    fn include_coordinate(
        &mut self,
        lower: &Real,
        upper: &Real,
        axis: Axis,
        policy: &CurvePolicy,
    ) -> Option<()> {
        if compare_reals(lower, upper, policy)? == std::cmp::Ordering::Greater {
            return None;
        }
        let (min, max) = match axis {
            Axis::X => (&mut self.min_x, &mut self.max_x),
            Axis::Y => (&mut self.min_y, &mut self.max_y),
        };
        match (min.as_mut(), max.as_mut()) {
            (Some(min), Some(max)) => {
                if compare_reals(lower, min, policy)? == std::cmp::Ordering::Less {
                    *min = lower.clone();
                }
                if compare_reals(upper, max, policy)? == std::cmp::Ordering::Greater {
                    *max = upper.clone();
                }
            }
            (None, None) => {
                *min = Some(lower.clone());
                *max = Some(upper.clone());
            }
            _ => return None,
        }
        Some(())
    }

    fn finish(self, policy: &CurvePolicy) -> Classification<BezierRetainedEndpointEnvelope2> {
        let (Some(min_x), Some(min_y), Some(max_x), Some(max_y)) =
            (self.min_x, self.min_y, self.max_x, self.max_y)
        else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let min = Point2::new(min_x, min_y);
        let max = Point2::new(max_x, max_y);
        let envelope = match Aabb2::from_points([&min, &max], policy) {
            Classification::Decided(envelope) => envelope,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        Classification::Decided(BezierRetainedEndpointEnvelope2 {
            envelope,
            native_endpoint_count: self.native_endpoint_count,
            algebraic_endpoint_count: self.algebraic_endpoint_count,
            endpoint_source_kinds: self.endpoint_source_kinds,
        })
    }
}

#[derive(Clone, Copy)]
enum Axis {
    X,
    Y,
}

fn native_endpoint_interval(point: &Point2) -> EndpointInterval {
    EndpointInterval {
        x: CoordinateInterval {
            lower: point.x().clone(),
            upper: point.x().clone(),
        },
        y: CoordinateInterval {
            lower: point.y().clone(),
            upper: point.y().clone(),
        },
        kind: BezierRetainedEnvelopeSourceKind::Native,
    }
}

fn algebraic_endpoint_interval(point: &BezierEndpointPointImage2) -> Option<EndpointInterval> {
    match point {
        BezierEndpointPointImage2::Polynomial(point) => Some(EndpointInterval {
            x: polynomial_coordinate_interval(
                point.x()?,
                point.parameter().polynomial_coefficients.as_slice(),
            )?,
            y: polynomial_coordinate_interval(
                point.y()?,
                point.parameter().polynomial_coefficients.as_slice(),
            )?,
            kind: BezierRetainedEnvelopeSourceKind::Algebraic,
        }),
        BezierEndpointPointImage2::Rational(point) => Some(EndpointInterval {
            x: represented_coordinate_interval(point.x()?.representation()?),
            y: represented_coordinate_interval(point.y()?.representation()?),
            kind: BezierRetainedEnvelopeSourceKind::Algebraic,
        }),
    }
}

/// Returns the tightest interval currently available for a polynomial
/// coordinate image.
///
/// When the coordinate polynomial has constant remainder modulo the algebraic
/// parameter's defining polynomial, the endpoint coordinate is that exact
/// rational constant.  This is the elementary quotient-ring identity
/// `p(alpha) = c` whenever `p(t) - c` is a multiple of the minimal replay
/// polynomial for `alpha`; the construction stays symbolic in the sense of the exactness model
/// and avoids widening to the parameter isolating interval.  Otherwise
/// the represented-root isolating interval remains the conservative evidence.
fn polynomial_coordinate_interval(
    coordinate: &crate::BezierAlgebraicCoordinateImage,
    parameter_polynomial: &[Real],
) -> Option<CoordinateInterval> {
    if let Some(exact) =
        polynomial_image_constant_remainder(coordinate.coefficients(), parameter_polynomial)
    {
        return Some(CoordinateInterval {
            lower: exact.clone(),
            upper: exact,
        });
    }
    Some(represented_coordinate_interval(
        coordinate.representation()?,
    ))
}

fn polynomial_image_constant_remainder(coefficients: &[Real], modulus: &[Real]) -> Option<Real> {
    let remainder = polynomial_remainder(coefficients, modulus)?;
    match remainder.as_slice() {
        [] => Some(Real::zero()),
        [constant] => Some(constant.clone()),
        _ => None,
    }
}

fn polynomial_remainder(coefficients: &[Real], modulus: &[Real]) -> Option<Vec<Real>> {
    let mut remainder = trim_polynomial(coefficients)?;
    let modulus = trim_polynomial(modulus)?;
    if modulus.len() < 2 {
        return None;
    }
    let leading = modulus.last()?;
    while remainder.len() >= modulus.len() {
        let shift = remainder.len() - modulus.len();
        let factor = (remainder.last()?.clone() / leading.clone()).ok()?;
        for (index, coefficient) in modulus.iter().enumerate() {
            let target = shift + index;
            remainder[target] = &remainder[target] - &(&factor * coefficient);
        }
        trim_polynomial_in_place(&mut remainder)?;
    }
    Some(remainder)
}

fn trim_polynomial(coefficients: &[Real]) -> Option<Vec<Real>> {
    let mut trimmed = coefficients.to_vec();
    trim_polynomial_in_place(&mut trimmed)?;
    Some(trimmed)
}

fn trim_polynomial_in_place(coefficients: &mut Vec<Real>) -> Option<()> {
    while coefficients.last().is_some_and(|coefficient| {
        compare_reals(coefficient, &Real::zero(), &CurvePolicy::certified())
            == Some(std::cmp::Ordering::Equal)
    }) {
        coefficients.pop();
    }
    Some(())
}

fn represented_coordinate_interval(root: &AlgebraicRootRepresentation) -> CoordinateInterval {
    if let Some(witness) = root.exact_rational_witness() {
        return CoordinateInterval {
            lower: witness.clone(),
            upper: witness.clone(),
        };
    }
    CoordinateInterval {
        lower: root.interval.lower.clone(),
        upper: root.interval.upper.clone(),
    }
}
