//! Polynomial approximations of exact analytic parallels: Blend2D and
//! Levien-style candidates, the conservative same-parameter verifier, and
//! the certificates it publishes.

use super::*;

/// Blend2D quadratic parallel candidate with an exact radial-excursion bound.
///
/// `radial_error_bound` bounds the excess of `|candidate(t)-source(t)|` over
/// the requested distance. It is deliberately not advertised as a Hausdorff
/// bound to the exact analytic parallel; callers must use a parallel verifier
/// before promoting this candidate to a certified approximation.
#[derive(Clone, Debug, PartialEq)]
pub struct Blend2dQuadraticOffsetCandidate2 {
    pub(super) curve: QuadraticBezier2,
    pub(super) radial_error_bound: Real,
    pub(super) tangent_cosine: Real,
    pub(super) distance: Real,
}

impl Blend2dQuadraticOffsetCandidate2 {
    /// Returns the exact-scalar quadratic candidate.
    pub const fn curve(&self) -> &QuadraticBezier2 {
        &self.curve
    }

    /// Returns the Blend2D radial-excursion error bound.
    pub const fn radial_error_bound(&self) -> &Real {
        &self.radial_error_bound
    }

    /// Returns the exact cosine between the endpoint tangent directions.
    pub const fn tangent_cosine(&self) -> &Real {
        &self.tangent_cosine
    }

    /// Returns the signed left-offset distance.
    pub const fn distance(&self) -> &Real {
        &self.distance
    }
}

/// Deterministic two-quadratic reduction of one cubic Bezier span.
#[derive(Clone, Debug, PartialEq)]
pub struct Blend2dCubicQuadraticReduction2 {
    pub(super) first: QuadraticBezier2,
    pub(super) second: QuadraticBezier2,
    pub(super) same_parameter_error_bound: Real,
}

/// Endpoint-tangent cubic candidate in the style of Levien's offset fitter.
///
/// The candidate always interpolates both exact parallel endpoints and tangent
/// directions. When the endpoint tangents are independent and the solved arms
/// are positive, their two scalar lengths additionally interpolate the exact
/// parallel midpoint. Acceptance still comes exclusively from the conservative
/// verifier; this construction is an optimization, not a certificate.
#[derive(Clone, Debug, PartialEq)]
pub struct LevienCubicOffsetCandidate2 {
    pub(super) curve: CubicBezier2,
    pub(super) matched_midpoint: bool,
    pub(super) distance: Real,
}

impl LevienCubicOffsetCandidate2 {
    /// Returns the cubic fitting candidate.
    pub const fn curve(&self) -> &CubicBezier2 {
        &self.curve
    }

    /// Returns whether positive tangent-arm solving also matched the exact midpoint.
    pub const fn matched_midpoint(&self) -> bool {
        self.matched_midpoint
    }

    /// Returns the signed left-offset distance.
    pub const fn distance(&self) -> &Real {
        &self.distance
    }
}

/// Polynomial Bezier candidate accepted by the parallel verifier.
#[derive(Clone, Debug, PartialEq)]
pub enum BezierParallelApproximationCurve2 {
    /// Quadratic candidate.
    Quadratic(QuadraticBezier2),
    /// Cubic candidate.
    Cubic(CubicBezier2),
}

/// Options for conservative same-parameter parallel verification.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierParallelVerificationOptions {
    pub(super) max_error: Real,
    pub(super) max_depth: usize,
}

impl BezierParallelVerificationOptions {
    /// Constructs verification options after certifying a positive tolerance and recursion budget.
    pub fn try_new(max_error: Real, max_depth: usize, policy: &CurveContext) -> CurveResult<Self> {
        if max_depth == 0 || real_sign(&max_error, policy) != Some(RealSign::Positive) {
            return Err(CurveError::InvalidBezierOffsetOptions);
        }
        Ok(Self {
            max_error,
            max_depth,
        })
    }

    /// Returns the requested same-parameter Euclidean error bound.
    pub const fn max_error(&self) -> &Real {
        &self.max_error
    }

    /// Returns the maximum exact bisection depth.
    pub const fn max_depth(&self) -> usize {
        self.max_depth
    }
}

/// Certificate proving a polynomial candidate remains within an exact analytic parallel tube.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierParallelApproximation2 {
    pub(super) curve: BezierParallelApproximationCurve2,
    pub(super) error_bound: Real,
    pub(super) leaf_count: usize,
    pub(super) maximum_depth: usize,
    pub(super) distance: Real,
}

/// One source-parameter span and its certified polynomial parallel approximation.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierParallelSpan2 {
    pub(super) source_start: Real,
    pub(super) source_end: Real,
    pub(super) approximation: CertifiedBezierParallelApproximation2,
}

impl CertifiedBezierParallelSpan2 {
    /// Returns the inclusive source parameter at the beginning of this span.
    pub const fn source_start(&self) -> &Real {
        &self.source_start
    }

    /// Returns the inclusive source parameter at the end of this span.
    pub const fn source_end(&self) -> &Real {
        &self.source_end
    }

    /// Returns the certified polynomial approximation for this span.
    pub const fn approximation(&self) -> &CertifiedBezierParallelApproximation2 {
        &self.approximation
    }
}

/// Connected sequence of certified polynomial approximations to one exact parallel.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierParallelPath2 {
    pub(super) spans: Vec<CertifiedBezierParallelSpan2>,
    pub(super) error_bound: Real,
    pub(super) construction_maximum_depth: usize,
    pub(super) verification_leaf_count: usize,
}

/// Certified primitive-parallel image of an ordered top-level curve path.
///
/// Lines, circular arcs, and Pythagorean-hodograph Beziers remain exact. Other
/// regular polynomial Beziers are replaced by independently verified quadratic
/// spans. Construction succeeds only when all produced primitive endpoints are
/// exactly connected; authored corners therefore remain a higher-level join
/// decision instead of being silently bridged.
#[derive(Clone, Debug)]
pub struct CertifiedCurvePathParallel2 {
    pub(super) path: CurvePath2,
    pub(super) max_parallel_error: Real,
    pub(super) source_curve_count: usize,
    pub(super) output_curve_count: usize,
    pub(super) exact_source_curve_count: usize,
    pub(super) approximated_source_curve_count: usize,
    pub(super) verification_leaf_count: usize,
}

impl CertifiedCurvePathParallel2 {
    /// Returns the exact/native and certified-polynomial parallel path.
    pub const fn path(&self) -> &CurvePath2 {
        &self.path
    }

    /// Returns the per-span parallel Hausdorff bound.
    pub const fn max_parallel_error(&self) -> &Real {
        &self.max_parallel_error
    }

    /// Returns the number of authored source curves.
    pub const fn source_curve_count(&self) -> usize {
        self.source_curve_count
    }

    /// Returns the number of exact and fitted output curves.
    pub const fn output_curve_count(&self) -> usize {
        self.output_curve_count
    }

    /// Returns the number of source curves whose parallel stayed exact.
    pub const fn exact_source_curve_count(&self) -> usize {
        self.exact_source_curve_count
    }

    /// Returns the number of source curves replaced by verified polynomial spans.
    pub const fn approximated_source_curve_count(&self) -> usize {
        self.approximated_source_curve_count
    }

    /// Returns the aggregate verifier leaf count for all fitted spans.
    pub const fn verification_leaf_count(&self) -> usize {
        self.verification_leaf_count
    }

    /// Consumes the certificate and returns its connected path.
    pub fn into_path(self) -> CurvePath2 {
        self.path
    }
}

impl CertifiedBezierParallelPath2 {
    /// Returns the ordered certified source spans.
    pub fn spans(&self) -> &[CertifiedBezierParallelSpan2] {
        &self.spans
    }

    /// Returns the requested bound proved independently for every span.
    pub const fn error_bound(&self) -> &Real {
        &self.error_bound
    }

    /// Returns the deepest candidate-generation subdivision.
    pub const fn construction_maximum_depth(&self) -> usize {
        self.construction_maximum_depth
    }

    /// Returns the aggregate verifier leaf count across all spans.
    pub const fn verification_leaf_count(&self) -> usize {
        self.verification_leaf_count
    }
}

impl CertifiedBezierParallelApproximation2 {
    /// Returns the certified polynomial Bezier candidate.
    pub const fn curve(&self) -> &BezierParallelApproximationCurve2 {
        &self.curve
    }

    /// Returns the proven same-parameter Euclidean bound.
    ///
    /// This is also a conservative Hausdorff bound because the shared
    /// parameter supplies a continuous correspondence in both directions.
    pub const fn error_bound(&self) -> &Real {
        &self.error_bound
    }

    /// Returns the number of accepted verification leaves.
    pub const fn leaf_count(&self) -> usize {
        self.leaf_count
    }

    /// Returns the deepest exact bisection used by verification.
    pub const fn maximum_depth(&self) -> usize {
        self.maximum_depth
    }

    /// Returns the signed left-offset distance.
    pub const fn distance(&self) -> &Real {
        &self.distance
    }
}

impl Blend2dCubicQuadraticReduction2 {
    /// Returns the quadratic covering source parameters `[0, 1/2]`.
    pub const fn first(&self) -> &QuadraticBezier2 {
        &self.first
    }

    /// Returns the quadratic covering source parameters `[1/2, 1]`.
    pub const fn second(&self) -> &QuadraticBezier2 {
        &self.second
    }

    /// Returns the exact same-parameter Euclidean error bound for either half.
    pub const fn same_parameter_error_bound(&self) -> &Real {
        &self.same_parameter_error_bound
    }
}

impl From<QuadraticBezier2> for BezierParallelApproximationCurve2 {
    fn from(value: QuadraticBezier2) -> Self {
        Self::Quadratic(value)
    }
}

impl From<CubicBezier2> for BezierParallelApproximationCurve2 {
    fn from(value: CubicBezier2) -> Self {
        Self::Cubic(value)
    }
}

#[derive(Clone)]
pub(super) enum PolynomialBezierNode2 {
    Quadratic(QuadraticBezier2),
    Cubic(CubicBezier2),
}

impl PolynomialBezierNode2 {
    pub(super) fn from_candidate(candidate: &BezierParallelApproximationCurve2) -> Self {
        match candidate {
            BezierParallelApproximationCurve2::Quadratic(curve) => Self::Quadratic(curve.clone()),
            BezierParallelApproximationCurve2::Cubic(curve) => Self::Cubic(curve.clone()),
        }
    }

    pub(super) fn point_at_half(&self) -> Point2 {
        let half = (Real::one() / Real::from(2_i8)).expect("division by two is exact");
        match self {
            Self::Quadratic(curve) => curve.point_at(half),
            Self::Cubic(curve) => curve.point_at(half),
        }
    }

    pub(super) fn split_half(&self) -> (Self, Self) {
        let half = (Real::one() / Real::from(2_i8)).expect("division by two is exact");
        match self {
            Self::Quadratic(curve) => {
                let (left, right) = curve.split_at_exact(half);
                (Self::Quadratic(left), Self::Quadratic(right))
            }
            Self::Cubic(curve) => {
                let (left, right) = curve.split_at_exact(half);
                (Self::Cubic(left), Self::Cubic(right))
            }
        }
    }

    pub(super) fn derivative_controls(&self) -> Vec<(Real, Real)> {
        let controls: Vec<&Point2> = match self {
            Self::Quadratic(curve) => curve.control_points().into_iter().collect(),
            Self::Cubic(curve) => curve.control_points().into_iter().collect(),
        };
        let degree = Real::from((controls.len() - 1) as u64);
        controls
            .windows(2)
            .map(|pair| {
                let delta = pair[1].delta_from(pair[0]);
                (&degree * delta.0, &degree * delta.1)
            })
            .collect()
    }

    pub(super) fn second_derivative_controls(&self) -> Vec<(Real, Real)> {
        let derivative = self.derivative_controls();
        let degree = derivative.len().saturating_sub(1);
        if degree == 0 {
            return Vec::new();
        }
        let scale = Real::from(degree as u64);
        derivative
            .windows(2)
            .map(|pair| {
                (
                    &scale * (&pair[1].0 - &pair[0].0),
                    &scale * (&pair[1].1 - &pair[0].1),
                )
            })
            .collect()
    }

    pub(super) fn exact_parallel_midpoint(
        &self,
        distance: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Point2>> {
        let source = match self {
            Self::Quadratic(curve) => curve.parallel_left(distance.clone())?,
            Self::Cubic(curve) => curve.parallel_left(distance.clone())?,
        };
        let half = (Real::one() / Real::from(2_i8))?;
        source.point_at(&half, policy)
    }
}

#[derive(Default)]
pub(super) struct ParallelVerificationTrace {
    pub(super) leaf_count: usize,
    pub(super) maximum_depth: usize,
}

pub(super) fn verify_parallel_node(
    source: PolynomialBezierNode2,
    candidate: PolynomialBezierNode2,
    distance: &Real,
    options: &BezierParallelVerificationOptions,
    policy: &CurveContext,
    depth: usize,
    trace: &mut ParallelVerificationTrace,
) -> CurveResult<Classification<()>> {
    trace.maximum_depth = trace.maximum_depth.max(depth);
    let exact_midpoint = match source.exact_parallel_midpoint(distance, policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let candidate_midpoint = candidate.point_at_half();
    let midpoint_error = exact_midpoint
        .distance_squared(&candidate_midpoint)
        .sqrt()?;
    let source_derivatives = source.derivative_controls();
    let source_accelerations = source.second_derivative_controls();
    let candidate_derivatives = candidate.derivative_controls();
    let minimum_source_speed = match derivative_hull_minimum_speed(&source_derivatives, policy)? {
        Classification::Decided(Some(speed)) => speed,
        Classification::Decided(None) => {
            if depth >= options.max_depth {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            return subdivide_parallel_verification(
                source, candidate, distance, options, policy, depth, trace,
            );
        }
        Classification::Uncertain(reason) => {
            if depth >= options.max_depth {
                return Ok(Classification::Uncertain(reason));
            }
            return subdivide_parallel_verification(
                source, candidate, distance, options, policy, depth, trace,
            );
        }
    };
    let source_acceleration_upper = vector_control_norm_sum(&source_accelerations)?;
    let derivative_difference_upper = vector_control_norm_sum(&derivative_control_differences(
        &source_derivatives,
        &candidate_derivatives,
    )?)?;
    let normal_derivative_upper = (source_acceleration_upper / minimum_source_speed)?;
    let error_derivative_upper =
        derivative_difference_upper + distance.abs() * normal_derivative_upper;
    let leaf_error_upper = midpoint_error + (error_derivative_upper / Real::from(2_i8))?;
    match compare_reals(&leaf_error_upper, &options.max_error, policy) {
        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal) => {
            trace.leaf_count += 1;
            Ok(Classification::Decided(()))
        }
        Some(std::cmp::Ordering::Greater) if depth < options.max_depth => {
            subdivide_parallel_verification(
                source, candidate, distance, options, policy, depth, trace,
            )
        }
        Some(std::cmp::Ordering::Greater) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
}

pub(super) fn subdivide_parallel_verification(
    source: PolynomialBezierNode2,
    candidate: PolynomialBezierNode2,
    distance: &Real,
    options: &BezierParallelVerificationOptions,
    policy: &CurveContext,
    depth: usize,
    trace: &mut ParallelVerificationTrace,
) -> CurveResult<Classification<()>> {
    let (source_left, source_right) = source.split_half();
    let (candidate_left, candidate_right) = candidate.split_half();
    match verify_parallel_node(
        source_left,
        candidate_left,
        distance,
        options,
        policy,
        depth + 1,
        trace,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    verify_parallel_node(
        source_right,
        candidate_right,
        distance,
        options,
        policy,
        depth + 1,
        trace,
    )
}

pub(super) fn vector_control_norm_sum(controls: &[(Real, Real)]) -> CurveResult<Real> {
    let mut sum = Real::zero();
    for (x, y) in controls {
        sum = &sum + (x * x + y * y).sqrt()?;
    }
    Ok(sum)
}

pub(super) fn derivative_control_differences(
    first: &[(Real, Real)],
    second: &[(Real, Real)],
) -> CurveResult<Vec<(Real, Real)>> {
    let target_degree = first.len().max(second.len()).saturating_sub(1);
    let first = elevate_vector_bernstein(first, target_degree)?;
    let second = elevate_vector_bernstein(second, target_degree)?;
    Ok(first
        .into_iter()
        .zip(second)
        .map(|(first, second)| (first.0 - second.0, first.1 - second.1))
        .collect())
}

pub(super) fn elevate_vector_bernstein(
    controls: &[(Real, Real)],
    target_degree: usize,
) -> CurveResult<Vec<(Real, Real)>> {
    if controls.is_empty() {
        return Ok(Vec::new());
    }
    let mut elevated = controls.to_vec();
    while elevated.len() - 1 < target_degree {
        let degree = elevated.len() - 1;
        let next_degree = degree + 1;
        let mut next = Vec::with_capacity(next_degree + 1);
        next.push(elevated[0].clone());
        for index in 1..next_degree {
            let left_weight = (Real::from(index as u64) / Real::from(next_degree as u64))?;
            let right_weight = Real::one() - &left_weight;
            next.push((
                &elevated[index - 1].0 * &left_weight + &elevated[index].0 * &right_weight,
                &elevated[index - 1].1 * &left_weight + &elevated[index].1 * &right_weight,
            ));
        }
        next.push(elevated[degree].clone());
        elevated = next;
    }
    Ok(elevated)
}

pub(super) fn derivative_hull_minimum_speed(
    controls: &[(Real, Real)],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Real>>> {
    if controls.is_empty() {
        return Ok(Classification::Decided(None));
    }
    let (minimum_x, maximum_x) =
        match coordinate_extrema(controls.iter().map(|control| &control.0), policy)? {
            Classification::Decided(extrema) => extrema,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let (minimum_y, maximum_y) =
        match coordinate_extrema(controls.iter().map(|control| &control.1), policy)? {
            Classification::Decided(extrema) => extrema,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let minimum_abs_x = match interval_minimum_absolute(&minimum_x, &maximum_x, policy) {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let minimum_abs_y = match interval_minimum_absolute(&minimum_y, &maximum_y, policy) {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let lower_squared = &minimum_abs_x * &minimum_abs_x + &minimum_abs_y * &minimum_abs_y;
    match real_sign(&lower_squared, policy) {
        Some(RealSign::Positive) => Ok(Classification::Decided(Some(lower_squared.sqrt()?))),
        Some(RealSign::Zero) => Ok(Classification::Decided(None)),
        Some(RealSign::Negative) => Err(CurveError::Topology(
            "derivative hull lower squared speed was certified negative".to_owned(),
        )),
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

pub(super) fn coordinate_extrema<'a>(
    mut values: impl Iterator<Item = &'a Real>,
    policy: &CurveContext,
) -> CurveResult<Classification<(Real, Real)>> {
    let first = values
        .next()
        .ok_or_else(|| CurveError::Topology("empty derivative control hull".to_owned()))?;
    let mut minimum = first.clone();
    let mut maximum = first.clone();
    for value in values {
        match compare_reals(value, &minimum, policy) {
            Some(std::cmp::Ordering::Less) => minimum = value.clone(),
            Some(_) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        match compare_reals(value, &maximum, policy) {
            Some(std::cmp::Ordering::Greater) => maximum = value.clone(),
            Some(_) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
    }
    Ok(Classification::Decided((minimum, maximum)))
}

pub(super) fn interval_minimum_absolute(
    minimum: &Real,
    maximum: &Real,
    policy: &CurveContext,
) -> Classification<Real> {
    match (real_sign(minimum, policy), real_sign(maximum, policy)) {
        (Some(RealSign::Positive), Some(_)) => Classification::Decided(minimum.clone()),
        (Some(_), Some(RealSign::Negative)) => Classification::Decided(maximum.abs()),
        (Some(_), Some(_)) => Classification::Decided(Real::zero()),
        _ => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}
