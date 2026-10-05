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
use hypersolve::RealInterval;

use crate::classify::compare_reals;
use crate::{
    Aabb2, Axis2, BezierParameter2, BezierSplitFragment2, BezierSubcurve2, Classification,
    CurveContext, CurveOutcome, CurveRegion2, CurveRegionBoundaryLoop2, CurveResult, Point2,
    RationalBezierAlgebraicPointImage2, UncertaintyReason,
};

impl CurveRegion2 {
    /// Returns a certified exact boundary envelope for the unified region, or
    /// `None` for the empty region, which has no envelope.
    ///
    /// Native line/arc topology uses the compact private bounds fast path. All
    /// other retained carriers use derivative-root and algebraic-source
    /// evidence without segmentation. Carriers lacking sufficient exact
    /// interior evidence report a blocker.
    pub fn bounds(&self) -> crate::ExactCurveResult<Option<Aabb2>> {
        if self.boundary_loops().is_empty() {
            return Ok(None);
        }
        crate::ExactCurveError::principal_query(
            crate::CurveOperation2::Classification,
            self.bounds_with_policy(&crate::policy::principal_context()),
        )
        .map(Some)
    }

    /// [`Self::bounds`] under an explicit predicate policy.
    pub(crate) fn bounds_with_policy(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Aabb2>>> {
        crate::policy::resolve_certified_operation(policy, |attempt| self.bounds_raw(attempt))
    }

    pub(crate) fn bounds_raw(&self, policy: &CurveContext) -> CurveResult<Classification<Aabb2>> {
        let certified = match self.native_line_arc_region(policy)? {
            Classification::Decided(native) => Aabb2::from_region(native)?,
            Classification::Uncertain(_) => BezierRetainedCurveEnvelope2::from_region(self, policy)
                .map(|envelope| envelope.envelope().clone()),
        };
        match certified {
            Classification::Uncertain(UncertaintyReason::Ordering)
                if policy.permits_approximate_512() =>
            {
                Ok(self.boundary_curve_bounds_ordered_by(policy))
            }
            certified => Ok(certified),
        }
    }

    /// Merges the certified boxes of every boundary curve, ordering their
    /// corners under `policy`.
    ///
    /// Box construction itself orders only by certified decisions. An
    /// approximate region query may still decide the order of coordinates that
    /// approximate coincidence left in different exact representations; the
    /// policy observes that consumption.
    fn boundary_curve_bounds_ordered_by(&self, policy: &CurveContext) -> Classification<Aabb2> {
        let mut corners: Option<[Real; 4]> = None;
        for boundary_loop in self.boundary_loops() {
            for curve in boundary_loop.curves() {
                let Ok(bounds) = curve.bounds() else {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                };
                let next = [
                    bounds.min_x().clone(),
                    bounds.min_y().clone(),
                    bounds.max_x().clone(),
                    bounds.max_y().clone(),
                ];
                let Some(current) = corners.as_mut() else {
                    corners = Some(next);
                    continue;
                };
                for (index, value) in next.into_iter().enumerate() {
                    let Some(order) = compare_reals(&value, &current[index], policy) else {
                        return Classification::Uncertain(UncertaintyReason::Ordering);
                    };
                    let extends = if index < 2 {
                        order.is_lt()
                    } else {
                        order.is_gt()
                    };
                    if extends {
                        current[index] = value;
                    }
                }
            }
        }
        let Some([min_x, min_y, max_x, max_y]) = corners else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        // Each curve box is ordered, so the merged extremes are too.
        Classification::Decided(Aabb2::new_unchecked(
            Point2::new(min_x, min_y),
            Point2::new(max_x, max_y),
        ))
    }
}

/// Exact curve-interior envelope for retained Bezier/conic carriers.
///
/// Native subcurves contribute endpoint and derivative-root extrema. Algebraic
/// endpoint-image fragments contribute only when they carry their source
/// curve; in that case the certified parameter intervals choose a native
/// source subcurve whose bounds conservatively overbound the algebraic
/// subrange. Endpoint images alone are still rejected because they do not prove
/// any interior extrema.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierRetainedCurveEnvelope2 {
    envelope: Aabb2,
    exact_fragment_count: usize,
    native_fragment_count: usize,
    algebraic_fragment_count: usize,
    fragment_source_kinds: Vec<BezierRetainedEnvelopeSourceKind>,
}

/// Source class of one retained envelope witness.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BezierRetainedEnvelopeSourceKind {
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
    pub fn from_region(region: &CurveRegion2, policy: &CurveContext) -> Classification<Self> {
        let mut accumulator = CurveEnvelopeAccumulator::default();
        for boundary_loop in region.boundary_loops() {
            match accumulator.include_loop(boundary_loop, policy) {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
        accumulator.finish()
    }

    /// Returns the exact curve-interior envelope.
    pub const fn envelope(&self) -> &Aabb2 {
        &self.envelope
    }
}

/// Exact endpoint envelope for a retained Bezier region or loop.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierRetainedEndpointEnvelope2 {
    envelope: Aabb2,
    native_endpoint_count: usize,
    algebraic_endpoint_count: usize,
    endpoint_source_kinds: Vec<BezierRetainedEnvelopeSourceKind>,
}

#[derive(Clone, Debug)]
struct EndpointInterval {
    x: RealInterval,
    y: RealInterval,
    kind: BezierRetainedEnvelopeSourceKind,
}

#[derive(Default)]
struct EndpointEnvelopeAccumulator {
    envelope: Option<Aabb2>,
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
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> Classification<()> {
        let kind = if matches!(fragment, BezierSplitFragment2::Materialized { .. }) {
            BezierRetainedEnvelopeSourceKind::Native
        } else {
            BezierRetainedEnvelopeSourceKind::Algebraic
        };
        let bounds = match fragment {
            BezierSplitFragment2::Materialized { curve, .. } => retained_curve_bounds(curve),
            BezierSplitFragment2::RetainedBezier {
                start,
                end,
                source_curve,
                start_image,
                end_image,
                ..
            } => {
                let unit = crate::CurveParameterRange2::unit();
                let bounds = if matches!(
                    crate::bezier_split::CurveParameterDomain2::new(&unit, None)
                        .contains_finite_range(&fragment.curve_region_parameter_range(), policy),
                    Ok(Classification::Decided(true))
                ) {
                    retained_algebraic_source_bounds(
                        source_curve,
                        start,
                        end,
                        start_image.as_ref(),
                        end_image.as_ref(),
                        policy,
                    )
                } else {
                    Classification::Uncertain(UncertaintyReason::Unsupported)
                };
                match bounds {
                    Classification::Decided(_) => bounds,
                    Classification::Uncertain(_) => {
                        crate::bezier_region::retained_fragment_query_bounds(fragment, policy)
                    }
                }
            }
            BezierSplitFragment2::AlgebraicChord(chord) => chord
                .conservative_bounds(policy)
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
            BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) => fragment
                .conservative_bounds()
                .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
            BezierSplitFragment2::AnalyticParallel(_) | BezierSplitFragment2::SelectedFiber(_) => {
                crate::bezier_region::retained_fragment_query_bounds(fragment, policy)
            }
        };
        let curve_box = match bounds {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        self.envelope = match self.envelope.take() {
            Some(envelope) => match envelope.union(&curve_box) {
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
    policy: &CurveContext,
) -> Classification<Aabb2> {
    let (range_start, range_end) = match parameter_interval_hull(start, end, policy) {
        Classification::Decided(range) => range,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let subcurve = match source_curve.subcurve_between_exact(&range_start, &range_end, policy) {
        Ok(Classification::Decided(subcurve)) => subcurve,
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    retained_curve_bounds(&subcurve)
}

fn retained_algebraic_source_bounds(
    source_curve: &BezierSubcurve2,
    start: &BezierParameter2,
    end: &BezierParameter2,
    start_image: Option<&crate::BezierAlgebraicEndpointImage2>,
    end_image: Option<&crate::BezierAlgebraicEndpointImage2>,
    policy: &CurveContext,
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
    policy: &CurveContext,
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
    match accumulator.include_endpoint(start_endpoint) {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }
    match accumulator.include_endpoint(end_endpoint) {
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
                match accumulator.include_endpoint(native_endpoint_interval(&point)) {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
            Some(false) => {}
            None => return Classification::Decided(None),
        }
    }

    match accumulator.finish() {
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
    policy: &CurveContext,
) -> Option<EndpointInterval> {
    match parameter {
        BezierParameter2::Exact(value) => {
            match source_curve_point_at(source_curve, value.clone(), policy) {
                Classification::Decided(point) => Some(native_endpoint_interval(&point)),
                Classification::Uncertain(_) => None,
            }
        }
        BezierParameter2::Algebraic(_) => image.and_then(|image| match image.point().ok()? {
            Classification::Decided(point) => algebraic_endpoint_interval(point),
            Classification::Uncertain(_) => None,
        }),
    }
}

/// Returns unique exact source parameters where x or y can have a local
/// extremum.
fn retained_curve_monotone_parameters(
    source_curve: &BezierSubcurve2,
    policy: &CurveContext,
) -> Classification<Vec<Real>> {
    let mut parameters = Vec::new();
    for axis in [Axis2::X, Axis2::Y] {
        let axis_parameters = match source_curve {
            BezierSubcurve2::Quadratic(curve) => {
                curve.axis_monotone_parameters_with_policy(axis, policy)
            }
            BezierSubcurve2::Cubic(curve) => {
                curve.axis_monotone_parameters_with_policy(axis, policy)
            }
            BezierSubcurve2::RationalQuadratic(curve) => {
                curve.axis_monotone_parameters_with_policy(axis, policy)
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
            if push_unique_real(&mut parameters, parameter).is_none() {
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
    policy: &CurveContext,
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
    policy: &CurveContext,
) -> Classification<Point2> {
    match source_curve {
        BezierSubcurve2::Quadratic(curve) => Classification::Decided(curve.point_at(parameter)),
        BezierSubcurve2::Cubic(curve) => Classification::Decided(curve.point_at(parameter)),
        BezierSubcurve2::RationalQuadratic(curve) => curve.point_at_with_policy(parameter, policy),
        BezierSubcurve2::Rational(curve) => curve.point_at_classified(&parameter, policy),
    }
}

/// Pushes an exact parameter unless a certified-equal one is already present.
/// Envelope candidates are deduplicated only on STRICT equality: an
/// approximate merge could drop a distinct extremum.
fn push_unique_real(values: &mut Vec<Real>, value: Real) -> Option<()> {
    if values.iter().any(|existing| {
        compare_reals(existing, &value, &CurveContext::STRICT) == Some(std::cmp::Ordering::Equal)
    }) {
        return Some(());
    }
    values.push(value);
    Some(())
}

fn parameter_interval_hull(
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurveContext,
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

    // The hull is an enclosure, so its endpoints are chosen by certified
    // STRICT order only; the caller's policy governs predicates on it.
    let strict = &CurveContext::STRICT;
    let lower = match compare_reals(start_interval.start(), end_interval.start(), strict) {
        Some(std::cmp::Ordering::Greater) => end_interval.start().clone(),
        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal) => {
            start_interval.start().clone()
        }
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    let upper = match compare_reals(start_interval.end(), end_interval.end(), strict) {
        Some(std::cmp::Ordering::Less) => end_interval.end().clone(),
        Some(std::cmp::Ordering::Greater | std::cmp::Ordering::Equal) => {
            start_interval.end().clone()
        }
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    if compare_reals(&lower, &upper, strict) == Some(std::cmp::Ordering::Greater) {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    }
    Classification::Decided((lower, upper))
}

fn retained_curve_bounds(curve: &BezierSubcurve2) -> Classification<Aabb2> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => curve.certified_bounds(),
        BezierSubcurve2::Cubic(curve) => curve.certified_bounds(),
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(),
    }
}

impl EndpointEnvelopeAccumulator {
    fn include_endpoint(&mut self, endpoint: EndpointInterval) -> Classification<()> {
        let min = Point2::new(endpoint.x.lower, endpoint.y.lower);
        let max = Point2::new(endpoint.x.upper, endpoint.y.upper);
        let endpoint_envelope = match Aabb2::from_points([&min, &max]) {
            Classification::Decided(envelope) => envelope,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        self.envelope = match self.envelope.take() {
            Some(envelope) => match envelope.union(&endpoint_envelope) {
                Classification::Decided(envelope) => Some(envelope),
                Classification::Uncertain(reason) => {
                    return Classification::Uncertain(reason);
                }
            },
            None => Some(endpoint_envelope),
        };
        match endpoint.kind {
            BezierRetainedEnvelopeSourceKind::Native => self.native_endpoint_count += 1,
            BezierRetainedEnvelopeSourceKind::Algebraic => self.algebraic_endpoint_count += 1,
        }
        self.endpoint_source_kinds.push(endpoint.kind);
        Classification::Decided(())
    }

    fn finish(self) -> Classification<BezierRetainedEndpointEnvelope2> {
        let Some(envelope) = self.envelope else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        Classification::Decided(BezierRetainedEndpointEnvelope2 {
            envelope,
            native_endpoint_count: self.native_endpoint_count,
            algebraic_endpoint_count: self.algebraic_endpoint_count,
            endpoint_source_kinds: self.endpoint_source_kinds,
        })
    }
}

fn native_endpoint_interval(point: &Point2) -> EndpointInterval {
    EndpointInterval {
        x: RealInterval {
            lower: point.x().clone(),
            upper: point.x().clone(),
        },
        y: RealInterval {
            lower: point.y().clone(),
            upper: point.y().clone(),
        },
        kind: BezierRetainedEnvelopeSourceKind::Native,
    }
}

fn algebraic_endpoint_interval(
    point: &RationalBezierAlgebraicPointImage2,
) -> Option<EndpointInterval> {
    Some(EndpointInterval {
        x: algebraic_coordinate_interval(point.x()?, &point.parameter().polynomial_coefficients)?,
        y: algebraic_coordinate_interval(point.y()?, &point.parameter().polynomial_coefficients)?,
        kind: BezierRetainedEnvelopeSourceKind::Algebraic,
    })
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
fn algebraic_coordinate_interval(
    coordinate: &crate::BezierAlgebraicRationalCoordinateImage,
    parameter_polynomial: &[Real],
) -> Option<RealInterval> {
    if coordinate.denominator_coefficients() == [Real::one()]
        && let Some(exact) = polynomial_image_constant_remainder(
            coordinate.numerator_coefficients(),
            parameter_polynomial,
        )
    {
        return Some(RealInterval {
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
        compare_reals(coefficient, &Real::zero(), &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Equal)
    }) {
        coefficients.pop();
    }
    Some(())
}

fn represented_coordinate_interval(root: &AlgebraicRootRepresentation) -> RealInterval {
    if let Some(witness) = root.exact_point_witness() {
        return RealInterval {
            lower: witness.clone(),
            upper: witness.clone(),
        };
    }
    RealInterval {
        lower: root.interval.lower.clone(),
        upper: root.interval.upper.clone(),
    }
}
