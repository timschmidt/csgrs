//! Certified staged offsets for polynomial Bezier curves.
//!
//! General parallels remain retained analytic expressions because they are not
//! finite polynomial Beziers. Exact source and offset cusps are isolated before
//! construction. Line images and Pythagorean hodographs materialize exactly;
//! other regular spans use Blend2D degree reduction and Levien-style tangent
//! cubics only as candidates, with a conservative same-parameter/Hausdorff
//! verifier controlling acceptance. Connected smooth paths and `CurveRegion2`
//! expose this lane while keeping corner joins and weaker chord fallback
//! explicit in their evidence.
//!
//! Candidate construction follows Raph Levien's parallel-curve and path-
//! simplification analyses and Blend2D's exact same-parameter degree-reduction
//! identities. Hypercurve deliberately replaces their sampling/error heuristics
//! with exact-scalar interval certification at the acceptance boundary.

use hyperreal::{RealSign, ZeroKnowledge as ZeroStatus};

use crate::bezier_parameter::{bernstein_to_power_coefficients, power_to_bernstein_coefficients};
use crate::classify::{compare_reals, in_closed_unit_interval, real_sign};
use crate::{
    BezierCuspClassification, BezierDegree, BezierEndpoint, BezierInflectionClassification,
    BezierLineImageFitRelation, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, CertifiedBezierLineImageOffset2, Classification, CubicBezier2,
    Curve2, CurveContext, CurveDerivative2, CurveError, CurveGeometry2, CurveOperation2,
    CurvePath2, CurveResult, ExactCurveError, ExactCurveResult, Point2, QuadraticBezier2,
    RationalBezier2, RationalQuadraticBezier2, Real, UncertaintyReason,
};

/// Polynomial Bezier source retained by an exact parallel evaluator.
#[derive(Clone, Debug, PartialEq)]
enum BezierParallelSource2 {
    Quadratic(QuadraticBezier2),
    Cubic(CubicBezier2),
}

/// Exact analytic parallel of a polynomial Bezier curve.
///
/// A general parallel is not itself a polynomial Bezier. This carrier retains
/// the exact expression `P(t) + d * left_normal(P'(t))`; fitted Beziers are
/// separate approximation products and can therefore be verified against this
/// object without confusing exact scalar coordinates with exact curve image.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierParallel2 {
    source: BezierParallelSource2,
    distance: Real,
    source_x: Vec<Real>,
    source_y: Vec<Real>,
    derivative_x: Vec<Real>,
    derivative_y: Vec<Real>,
    second_derivative_x: Vec<Real>,
    second_derivative_y: Vec<Real>,
}

/// Exact rational parallel certified from a polynomial Pythagorean hodograph.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedPythagoreanHodographOffset2 {
    curve: RationalBezier2,
    speed_polynomial: Vec<Real>,
    source_degree: usize,
    rational_degree: usize,
    distance: Real,
}

/// Blend2D quadratic parallel candidate with an exact radial-excursion bound.
///
/// `radial_error_bound` bounds the excess of `|candidate(t)-source(t)|` over
/// the requested distance. It is deliberately not advertised as a Hausdorff
/// bound to the exact analytic parallel; callers must use a parallel verifier
/// before promoting this candidate to a certified approximation.
#[derive(Clone, Debug, PartialEq)]
pub struct Blend2dQuadraticOffsetCandidate2 {
    curve: QuadraticBezier2,
    radial_error_bound: Real,
    tangent_cosine: Real,
    distance: Real,
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
    first: QuadraticBezier2,
    second: QuadraticBezier2,
    same_parameter_error_bound: Real,
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
    curve: CubicBezier2,
    matched_midpoint: bool,
    distance: Real,
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
    max_error: Real,
    max_depth: usize,
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
    curve: BezierParallelApproximationCurve2,
    error_bound: Real,
    leaf_count: usize,
    maximum_depth: usize,
    distance: Real,
}

/// One source-parameter span and its certified polynomial parallel approximation.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierParallelSpan2 {
    source_start: Real,
    source_end: Real,
    approximation: CertifiedBezierParallelApproximation2,
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
    spans: Vec<CertifiedBezierParallelSpan2>,
    error_bound: Real,
    construction_maximum_depth: usize,
    verification_leaf_count: usize,
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
    path: CurvePath2,
    max_parallel_error: Real,
    source_curve_count: usize,
    output_curve_count: usize,
    exact_source_curve_count: usize,
    approximated_source_curve_count: usize,
    verification_leaf_count: usize,
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

impl CertifiedPythagoreanHodographOffset2 {
    /// Returns the exact rational Bezier carrying the parallel image.
    pub const fn curve(&self) -> &RationalBezier2 {
        &self.curve
    }

    /// Returns the polynomial whose square is exactly `P' dot P'`.
    pub fn speed_polynomial(&self) -> &[Real] {
        &self.speed_polynomial
    }

    /// Returns the polynomial source degree.
    pub const fn source_degree(&self) -> usize {
        self.source_degree
    }

    /// Returns the homogeneous degree of the exact rational parallel.
    pub const fn rational_degree(&self) -> usize {
        self.rational_degree
    }

    /// Returns the signed left-offset distance.
    pub const fn distance(&self) -> &Real {
        &self.distance
    }
}

/// Exact singularity evidence for a retained Bezier parallel.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierParallelSingularityAnalysis2 {
    source_singularities: Vec<BezierParameter2>,
    parallel_cusps: Vec<BezierParameter2>,
    source_speed_squared_degree: usize,
    parallel_cusp_polynomial_degree: Option<usize>,
}

impl BezierParallelSingularityAnalysis2 {
    /// Returns every isolated parameter where the source derivative vanishes.
    pub fn source_singularities(&self) -> &[BezierParameter2] {
        &self.source_singularities
    }

    /// Returns every isolated regular-source parameter where the parallel derivative vanishes.
    pub fn parallel_cusps(&self) -> &[BezierParameter2] {
        &self.parallel_cusps
    }

    /// Returns whether the source normal is defined over the complete parameter domain.
    pub fn source_is_regular(&self) -> bool {
        self.source_singularities.is_empty()
    }

    /// Returns whether the retained parallel is cusp-free over every regular source span.
    pub fn parallel_is_cusp_free(&self) -> bool {
        self.parallel_cusps.is_empty()
    }

    /// Returns the degree of the exact source speed-squared polynomial.
    pub const fn source_speed_squared_degree(&self) -> usize {
        self.source_speed_squared_degree
    }

    /// Returns the degree of the squared parallel-cusp polynomial when nonconstant.
    pub const fn parallel_cusp_polynomial_degree(&self) -> Option<usize> {
        self.parallel_cusp_polynomial_degree
    }
}

impl BezierParallel2 {
    fn from_controls(
        source: BezierParallelSource2,
        controls: &[&Point2],
        distance: Real,
    ) -> CurveResult<Self> {
        let x = bernstein_to_power_coefficients(
            controls.iter().map(|point| point.x().clone()).collect(),
        )?;
        let y = bernstein_to_power_coefficients(
            controls.iter().map(|point| point.y().clone()).collect(),
        )?;
        let derivative_x = polynomial_derivative(&x);
        let derivative_y = polynomial_derivative(&y);
        let second_derivative_x = polynomial_derivative(&derivative_x);
        let second_derivative_y = polynomial_derivative(&derivative_y);
        Ok(Self {
            source,
            distance,
            source_x: x,
            source_y: y,
            derivative_x,
            derivative_y,
            second_derivative_x,
            second_derivative_y,
        })
    }

    /// Returns the signed distance measured along the source's left normal.
    pub const fn distance(&self) -> &Real {
        &self.distance
    }

    /// Returns the retained quadratic source when this is a quadratic parallel.
    pub const fn quadratic_source(&self) -> Option<&QuadraticBezier2> {
        match &self.source {
            BezierParallelSource2::Quadratic(source) => Some(source),
            BezierParallelSource2::Cubic(_) => None,
        }
    }

    /// Returns the retained cubic source when this is a cubic parallel.
    pub const fn cubic_source(&self) -> Option<&CubicBezier2> {
        match &self.source {
            BezierParallelSource2::Quadratic(_) => None,
            BezierParallelSource2::Cubic(source) => Some(source),
        }
    }

    /// Evaluates the exact analytic parallel at one represented parameter.
    pub fn point_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Point2>> {
        match in_closed_unit_interval(parameter, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let source_point = self.source_point_at(parameter.clone());
        if real_sign(&self.distance, policy) == Some(RealSign::Zero) {
            return Ok(Classification::Decided(source_point));
        }
        let derivative = self.source_derivative_at(parameter);
        let speed_squared = derivative.dx() * derivative.dx() + derivative.dy() * derivative.dy();
        match real_sign(&speed_squared, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "Bezier derivative squared norm was certified negative".to_owned(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let speed = speed_squared.sqrt()?;
        let normal_x = ((Real::zero() - derivative.dy()) / &speed)?;
        let normal_y = (derivative.dx().clone() / &speed)?;
        Ok(Classification::Decided(source_point.translated(
            &self.distance * normal_x,
            &self.distance * normal_y,
        )))
    }

    /// Evaluates the exact first derivative of the analytic parallel.
    pub fn derivative_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveDerivative2>> {
        match in_closed_unit_interval(parameter, policy) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let derivative = self.source_derivative_at(parameter);
        if real_sign(&self.distance, policy) == Some(RealSign::Zero) {
            return Ok(Classification::Decided(derivative));
        }
        let second = self.source_second_derivative_at(parameter);
        let speed_squared = derivative.dx() * derivative.dx() + derivative.dy() * derivative.dy();
        match real_sign(&speed_squared, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "Bezier derivative squared norm was certified negative".to_owned(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let speed = speed_squared.clone().sqrt()?;
        let speed_cubed = &speed_squared * &speed;
        let velocity_acceleration_dot =
            derivative.dx() * second.dx() + derivative.dy() * second.dy();
        let normal_derivative_x = ((Real::zero() - second.dy()) / &speed)?
            + ((derivative.dy() * &velocity_acceleration_dot) / &speed_cubed)?;
        let normal_derivative_y = (second.dx().clone() / &speed)?
            - ((derivative.dx() * velocity_acceleration_dot) / speed_cubed)?;
        Ok(Classification::Decided(CurveDerivative2::new(
            derivative.dx() + &self.distance * normal_derivative_x,
            derivative.dy() + &self.distance * normal_derivative_y,
        )))
    }

    /// Isolates source singularities and distance-dependent parallel cusps exactly.
    ///
    /// On regular source spans a parallel cusp satisfies
    /// `d * (P'' x P') + |P'|^3 = 0`. The root scheduler isolates the polynomial
    /// obtained by squaring that equation, then rejects source singularities and
    /// the opposite-sign roots introduced by squaring.
    pub fn singularity_analysis(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelSingularityAnalysis2>> {
        let speed_squared = polynomial_add(
            &polynomial_multiply(&self.derivative_x, &self.derivative_x),
            &polynomial_multiply(&self.derivative_y, &self.derivative_y),
        );
        let speed_polynomial = match polynomial_from_coefficients(speed_squared.clone(), policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let source_singularities = match speed_polynomial.isolate_unit_interval_roots(policy)? {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if real_sign(&self.distance, policy) == Some(RealSign::Zero) {
            return Ok(Classification::Decided(
                BezierParallelSingularityAnalysis2 {
                    source_singularities,
                    parallel_cusps: Vec::new(),
                    source_speed_squared_degree: speed_polynomial.degree(),
                    parallel_cusp_polynomial_degree: None,
                },
            ));
        }

        let curvature_cross = polynomial_subtract(
            &polynomial_multiply(&self.second_derivative_x, &self.derivative_y),
            &polynomial_multiply(&self.second_derivative_y, &self.derivative_x),
        );
        let signed_curvature_term = polynomial_scale(&curvature_cross, &self.distance);
        let squared_cusp_polynomial = polynomial_subtract(
            &polynomial_multiply(&signed_curvature_term, &signed_curvature_term),
            &polynomial_power(&speed_squared, 3),
        );
        let cusp_polynomial = match polynomial_from_coefficients(squared_cusp_polynomial, policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let candidates = match cusp_polynomial.isolate_unit_interval_roots(policy)? {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let curvature_term_polynomial =
            match polynomial_from_coefficients(signed_curvature_term, policy)? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        let mut parallel_cusps = Vec::new();
        for candidate in candidates {
            if parameter_matches_any(&candidate, &source_singularities, policy)? {
                continue;
            }
            let sign = match signed_polynomial_at_root(
                curvature_term_polynomial.as_ref(),
                &candidate,
                policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if sign == RealSign::Negative {
                parallel_cusps.push(candidate);
            }
        }
        Ok(Classification::Decided(
            BezierParallelSingularityAnalysis2 {
                source_singularities,
                parallel_cusps,
                source_speed_squared_degree: speed_polynomial.degree(),
                parallel_cusp_polynomial_degree: Some(cusp_polynomial.degree()),
            },
        ))
    }

    /// Materializes this parallel exactly when the source hodograph is Pythagorean.
    ///
    /// If `P' dot P' = sigma^2` for a polynomial `sigma` with certified nonzero
    /// sign over `[0, 1]`, the unit normal is rational and the complete parallel
    /// is converted to an arbitrary-degree [`RationalBezier2`]. `None` means the
    /// exact polynomial-square identity was disproved; unresolved scalar signs
    /// remain explicit [`Classification::Uncertain`].
    pub fn exact_pythagorean_hodograph_offset(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CertifiedPythagoreanHodographOffset2>>> {
        let speed_squared = polynomial_add(
            &polynomial_multiply(&self.derivative_x, &self.derivative_x),
            &polynomial_multiply(&self.derivative_y, &self.derivative_y),
        );
        let mut speed = match polynomial_square_root(&speed_squared, policy)? {
            Classification::Decided(Some(speed)) => speed,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let speed_at_start = polynomial_evaluate(&speed, &Real::zero());
        match real_sign(&speed_at_start, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Negative) => {
                speed = polynomial_scale(&speed, &Real::from(-1_i8));
            }
            Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let speed_polynomial = match polynomial_from_coefficients(speed.clone(), policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let speed_roots = match speed_polynomial.isolate_unit_interval_roots(policy)? {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if !speed_roots.is_empty() {
            return Ok(Classification::Decided(None));
        }

        let numerator_x = polynomial_subtract(
            &polynomial_multiply(&self.source_x, &speed),
            &polynomial_scale(&self.derivative_y, &self.distance),
        );
        let numerator_y = polynomial_add(
            &polynomial_multiply(&self.source_y, &speed),
            &polynomial_scale(&self.derivative_x, &self.distance),
        );
        let source_degree = self.source_x.len() - 1;
        let base_degree = numerator_x
            .len()
            .max(numerator_y.len())
            .max(speed.len())
            .saturating_sub(1);
        for rational_degree in base_degree..=base_degree.saturating_add(32) {
            let weights = power_to_bernstein_coefficients(&speed, rational_degree)?;
            let mut all_positive = true;
            for weight in &weights {
                match real_sign(weight, policy) {
                    Some(RealSign::Positive) => {}
                    Some(RealSign::Zero | RealSign::Negative) => {
                        all_positive = false;
                        break;
                    }
                    None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                }
            }
            if !all_positive {
                continue;
            }
            let x_homogeneous = power_to_bernstein_coefficients(&numerator_x, rational_degree)?;
            let y_homogeneous = power_to_bernstein_coefficients(&numerator_y, rational_degree)?;
            let controls = x_homogeneous
                .into_iter()
                .zip(y_homogeneous)
                .zip(weights.iter())
                .map(|((x, y), weight)| Ok(Point2::new((x / weight)?, (y / weight)?)))
                .collect::<CurveResult<Vec<_>>>()?;
            let curve = RationalBezier2::try_new(controls, weights)?;
            return Ok(Classification::Decided(Some(
                CertifiedPythagoreanHodographOffset2 {
                    curve,
                    speed_polynomial: speed,
                    source_degree,
                    rational_degree,
                    distance: self.distance.clone(),
                },
            )));
        }
        Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
    }

    /// Builds a Levien-style endpoint-tangent cubic for later verification.
    ///
    /// Independent endpoint tangents provide two scalar arm lengths, solved so
    /// the cubic also passes through the exact analytic parallel at `t=1/2`.
    /// Parallel, negative-arm, or undecidable cases use exact Hermite endpoint
    /// derivatives instead. Neither lane is accepted without
    /// [`Self::verify_polynomial_candidate`].
    pub fn levien_cubic_candidate(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<LevienCubicOffsetCandidate2>> {
        let zero = Real::zero();
        let one = Real::one();
        let half = (Real::one() / Real::from(2_i8))?;
        let start = match self.point_at(&zero, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let end = match self.point_at(&one, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let midpoint = match self.point_at(&half, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let start_derivative = match self.derivative_at(&zero, policy)? {
            Classification::Decided(derivative) => derivative,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let end_derivative = match self.derivative_at(&one, policy)? {
            Classification::Decided(derivative) => derivative,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if start_derivative.zero_status() != ZeroStatus::NonZero
            || end_derivative.zero_status() != ZeroStatus::NonZero
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }

        let tangent_cross = start_derivative.dx() * end_derivative.dy()
            - start_derivative.dy() * end_derivative.dx();
        let midpoint_base = start.lerp(&end, half);
        let midpoint_delta = midpoint.delta_from(&midpoint_base);
        let rhs_scale = (Real::from(8_i8) / Real::from(3_i8))?;
        let rhs_x = midpoint_delta.0 * &rhs_scale;
        let rhs_y = midpoint_delta.1 * rhs_scale;
        let solved_arms = match real_sign(&tangent_cross, policy) {
            Some(RealSign::Positive | RealSign::Negative) => {
                let start_arm = ((&rhs_x * end_derivative.dy() - &rhs_y * end_derivative.dx())
                    / &tangent_cross)?;
                let end_arm = ((start_derivative.dy() * &rhs_x - start_derivative.dx() * &rhs_y)
                    / &tangent_cross)?;
                match (real_sign(&start_arm, policy), real_sign(&end_arm, policy)) {
                    (Some(RealSign::Positive), Some(RealSign::Positive)) => {
                        Some((start_arm, end_arm))
                    }
                    _ => None,
                }
            }
            Some(RealSign::Zero) | None => None,
        };
        let (control1, control2, matched_midpoint) = if let Some((start_arm, end_arm)) = solved_arms
        {
            (
                start.translated(
                    start_derivative.dx() * &start_arm,
                    start_derivative.dy() * start_arm,
                ),
                end.translated(
                    Real::zero() - end_derivative.dx() * &end_arm,
                    Real::zero() - end_derivative.dy() * end_arm,
                ),
                true,
            )
        } else {
            let one_third = (Real::one() / Real::from(3_i8))?;
            (
                start.translated(
                    start_derivative.dx() * &one_third,
                    start_derivative.dy() * &one_third,
                ),
                end.translated(
                    Real::zero() - end_derivative.dx() * &one_third,
                    Real::zero() - end_derivative.dy() * one_third,
                ),
                false,
            )
        };
        Ok(Classification::Decided(LevienCubicOffsetCandidate2 {
            curve: CubicBezier2::new(start, control1, control2, end),
            matched_midpoint,
            distance: self.distance.clone(),
        }))
    }

    /// Conservatively verifies a polynomial Bezier candidate against this exact parallel.
    ///
    /// Each dyadic leaf bounds the midpoint error plus a Lipschitz remainder.
    /// Source and candidate derivative control hulls bound speed, while
    /// `|n'| <= |P''| / |P'|` bounds variation of the exact unit normal. No
    /// sampled normal ray or floating conversion participates in acceptance.
    pub fn verify_polynomial_candidate(
        &self,
        candidate: BezierParallelApproximationCurve2,
        options: &BezierParallelVerificationOptions,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CertifiedBezierParallelApproximation2>> {
        let analysis = match self.singularity_analysis(policy)? {
            Classification::Decided(analysis) => analysis,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if !analysis.source_is_regular() {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        let source = PolynomialBezierNode2::from_source(&self.source);
        let candidate_node = PolynomialBezierNode2::from_candidate(&candidate);
        let mut trace = ParallelVerificationTrace::default();
        match verify_parallel_node(
            source,
            candidate_node,
            &self.distance,
            options,
            policy,
            0,
            &mut trace,
        )? {
            Classification::Decided(()) => Ok(Classification::Decided(
                CertifiedBezierParallelApproximation2 {
                    curve: candidate,
                    error_bound: options.max_error.clone(),
                    leaf_count: trace.leaf_count,
                    maximum_depth: trace.maximum_depth,
                    distance: self.distance.clone(),
                },
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    fn source_point_at(&self, parameter: Real) -> Point2 {
        match &self.source {
            BezierParallelSource2::Quadratic(source) => source.point_at(parameter),
            BezierParallelSource2::Cubic(source) => source.point_at(parameter),
        }
    }

    fn source_derivative_at(&self, parameter: &Real) -> CurveDerivative2 {
        CurveDerivative2::new(
            polynomial_evaluate(&self.derivative_x, parameter),
            polynomial_evaluate(&self.derivative_y, parameter),
        )
    }

    fn source_second_derivative_at(&self, parameter: &Real) -> CurveDerivative2 {
        CurveDerivative2::new(
            polynomial_evaluate(&self.second_derivative_x, parameter),
            polynomial_evaluate(&self.second_derivative_y, parameter),
        )
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
enum PolynomialBezierNode2 {
    Quadratic(QuadraticBezier2),
    Cubic(CubicBezier2),
}

impl PolynomialBezierNode2 {
    fn from_source(source: &BezierParallelSource2) -> Self {
        match source {
            BezierParallelSource2::Quadratic(curve) => Self::Quadratic(curve.clone()),
            BezierParallelSource2::Cubic(curve) => Self::Cubic(curve.clone()),
        }
    }

    fn from_candidate(candidate: &BezierParallelApproximationCurve2) -> Self {
        match candidate {
            BezierParallelApproximationCurve2::Quadratic(curve) => Self::Quadratic(curve.clone()),
            BezierParallelApproximationCurve2::Cubic(curve) => Self::Cubic(curve.clone()),
        }
    }

    fn point_at_half(&self) -> Point2 {
        let half = (Real::one() / Real::from(2_i8)).expect("division by two is exact");
        match self {
            Self::Quadratic(curve) => curve.point_at(half),
            Self::Cubic(curve) => curve.point_at(half),
        }
    }

    fn split_half(&self) -> (Self, Self) {
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

    fn derivative_controls(&self) -> Vec<(Real, Real)> {
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

    fn second_derivative_controls(&self) -> Vec<(Real, Real)> {
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

    fn exact_parallel_midpoint(
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
struct ParallelVerificationTrace {
    leaf_count: usize,
    maximum_depth: usize,
}

fn verify_parallel_node(
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

fn subdivide_parallel_verification(
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

fn vector_control_norm_sum(controls: &[(Real, Real)]) -> CurveResult<Real> {
    let mut sum = Real::zero();
    for (x, y) in controls {
        sum = &sum + (x * x + y * y).sqrt()?;
    }
    Ok(sum)
}

fn derivative_control_differences(
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

fn elevate_vector_bernstein(
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

fn derivative_hull_minimum_speed(
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

fn coordinate_extrema<'a>(
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

fn interval_minimum_absolute(
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

/// Exact source-curve hazard that must be resolved before a Bezier offset is
/// treated as a topology product.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BezierOffsetRisk {
    /// The entire source curve is certified to be one point.
    DegeneratePoint,
    /// The source has at least one certified cusp where the normal is undefined.
    Cusp,
    /// A cubic has certified inflection parameters where the normal field can flip.
    Inflection,
    /// The curvature numerator is structurally zero over the whole cubic.
    AllCurvatureZero,
    /// The first derivative is certified zero at the given endpoint.
    UndefinedEndpointNormal {
        /// Endpoint whose first derivative is zero.
        endpoint: BezierEndpoint,
    },
    /// Structural inspection could not prove whether the endpoint derivative is nonzero.
    UnresolvedEndpointNormal {
        /// Endpoint whose first derivative status is unknown.
        endpoint: BezierEndpoint,
    },
    /// The source endpoints are structurally coincident.
    CoincidentEndpoints,
    /// A rational Bezier denominator can cross or touch zero on the affine interval.
    ProjectiveDenominatorBoundary,
}

/// Exact source analysis retained before a staged Bezier offset candidate is built.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierOffsetPreflight2 {
    degree: BezierDegree,
    cusp_classification: BezierCuspClassification,
    inflection_classification: BezierInflectionClassification,
    start_tangent_status: ZeroStatus,
    end_tangent_status: ZeroStatus,
    endpoint_coincidence: ZeroStatus,
    risks: Vec<BezierOffsetRisk>,
    construction_policy: CurveContext,
}

/// Result of a staged Bezier/conic offset attempt.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum BezierOffsetCandidate2 {
    /// The source was certified to be one endpoint line image and offset exactly.
    ExactLineImage {
        /// Exact primitive offset of the certified endpoint line image.
        offset: CertifiedBezierLineImageOffset2,
        /// Exact source analysis retained from the staged preflight.
        preflight: BezierOffsetPreflight2,
    },
    /// The source hodograph is Pythagorean, so its complete parallel is exact rational geometry.
    ExactPythagoreanHodograph {
        /// Exact arbitrary-degree rational Bezier parallel.
        offset: CertifiedPythagoreanHodographOffset2,
        /// Exact source analysis retained from the staged preflight.
        preflight: BezierOffsetPreflight2,
    },
    /// The source is not yet supported by a certified analytic/fitted offset.
    Unresolved {
        /// Exact source analysis for the unresolved curve.
        preflight: BezierOffsetPreflight2,
        /// Signed distance along the curve's left normal.
        distance: Real,
    },
}

impl BezierOffsetPreflight2 {
    /// Returns the Bezier degree covered by this preflight.
    pub const fn degree(&self) -> BezierDegree {
        self.degree
    }

    /// Returns the exact cusp classification used by offset preflight.
    pub const fn cusp_classification(&self) -> &BezierCuspClassification {
        &self.cusp_classification
    }

    /// Returns the exact inflection classification used by offset preflight.
    pub const fn inflection_classification(&self) -> &BezierInflectionClassification {
        &self.inflection_classification
    }

    /// Returns structural zero knowledge for the start endpoint derivative.
    pub const fn start_tangent_status(&self) -> ZeroStatus {
        self.start_tangent_status
    }

    /// Returns structural zero knowledge for the end endpoint derivative.
    pub const fn end_tangent_status(&self) -> ZeroStatus {
        self.end_tangent_status
    }

    /// Returns structural zero knowledge for source endpoint coincidence.
    pub const fn endpoint_coincidence(&self) -> ZeroStatus {
        self.endpoint_coincidence
    }

    /// Returns the exact or unresolved risks detected before offset fitting.
    pub fn risks(&self) -> &[BezierOffsetRisk] {
        &self.risks
    }

    /// Returns true when no currently implemented exact preflight risk remains.
    pub fn is_clear(&self) -> bool {
        self.risks.is_empty()
    }

    /// Returns the policy used to prove this preflight.
    pub const fn construction_policy(&self) -> &CurveContext {
        &self.construction_policy
    }
}

impl BezierOffsetCandidate2 {
    /// Returns the preflight retained by this staged candidate.
    pub const fn preflight(&self) -> &BezierOffsetPreflight2 {
        match self {
            Self::ExactLineImage { preflight, .. }
            | Self::ExactPythagoreanHodograph { preflight, .. }
            | Self::Unresolved { preflight, .. } => preflight,
        }
    }

    /// Returns the exact primitive offset, if one was constructed.
    pub const fn exact_line_image_offset(&self) -> Option<&CertifiedBezierLineImageOffset2> {
        match self {
            Self::ExactLineImage { offset, .. } => Some(offset),
            Self::ExactPythagoreanHodograph { .. } | Self::Unresolved { .. } => None,
        }
    }

    /// Returns the exact rational PH parallel, if one was constructed.
    pub const fn exact_pythagorean_hodograph_offset(
        &self,
    ) -> Option<&CertifiedPythagoreanHodographOffset2> {
        match self {
            Self::ExactPythagoreanHodograph { offset, .. } => Some(offset),
            Self::ExactLineImage { .. } | Self::Unresolved { .. } => None,
        }
    }

    /// Returns the unresolved preflight when no primitive offset was constructed.
    pub const fn unresolved_preflight(&self) -> Option<&BezierOffsetPreflight2> {
        match self {
            Self::ExactLineImage { .. } | Self::ExactPythagoreanHodograph { .. } => None,
            Self::Unresolved { preflight, .. } => Some(preflight),
        }
    }

    /// Returns the signed distance along the curve's left normal.
    pub const fn distance(&self) -> &Real {
        match self {
            Self::ExactLineImage { offset, .. } => offset.distance(),
            Self::ExactPythagoreanHodograph { offset, .. } => offset.distance(),
            Self::Unresolved { distance, .. } => distance,
        }
    }
}

impl QuadraticBezier2 {
    /// Retains this quadratic's exact analytic left parallel.
    pub fn parallel_left(&self, distance: Real) -> CurveResult<BezierParallel2> {
        BezierParallel2::from_controls(
            BezierParallelSource2::Quadratic(self.clone()),
            &self.control_points(),
            distance,
        )
    }

    /// Retains this quadratic's exact analytic right parallel.
    pub fn parallel_right(&self, distance: Real) -> CurveResult<BezierParallel2> {
        self.parallel_left(-distance)
    }

    /// Builds the deterministic Blend2D quadratic left-parallel candidate.
    pub fn blend2d_offset_left_candidate(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Blend2dQuadraticOffsetCandidate2>> {
        if real_sign(&distance, policy) == Some(RealSign::Zero) {
            return Ok(Classification::Decided(Blend2dQuadraticOffsetCandidate2 {
                curve: self.clone(),
                radial_error_bound: Real::zero(),
                tangent_cosine: Real::one(),
                distance,
            }));
        }
        let start_delta = self.control().delta_from(self.start());
        let end_delta = self.end().delta_from(self.control());
        let start_length_squared =
            &start_delta.0 * &start_delta.0 + &start_delta.1 * &start_delta.1;
        let end_length_squared = &end_delta.0 * &end_delta.0 + &end_delta.1 * &end_delta.1;
        for length_squared in [&start_length_squared, &end_length_squared] {
            match real_sign(length_squared, policy) {
                Some(RealSign::Positive) => {}
                Some(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Some(RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "Bezier tangent squared norm was certified negative".to_owned(),
                    ));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        let start_length = start_length_squared.sqrt()?;
        let end_length = end_length_squared.sqrt()?;
        let start_normal = (
            ((Real::zero() - &start_delta.1) / &start_length)?,
            (start_delta.0.clone() / &start_length)?,
        );
        let end_normal = (
            ((Real::zero() - &end_delta.1) / &end_length)?,
            (end_delta.0.clone() / &end_length)?,
        );
        let normal_sum = (
            &start_normal.0 + &end_normal.0,
            &start_normal.1 + &end_normal.1,
        );
        let normal_sum_squared = &normal_sum.0 * &normal_sum.0 + &normal_sum.1 * &normal_sum.1;
        match real_sign(&normal_sum_squared, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "summed unit-normal squared norm was certified negative".to_owned(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let tangent_cosine = ((&start_delta.0 * &end_delta.0 + &start_delta.1 * &end_delta.1)
            / (&start_length * &end_length))?;
        let one_plus_cosine = Real::one() + &tangent_cosine;
        match real_sign(&one_plus_cosine, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "endpoint tangent cosine was certified below -1".to_owned(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let middle_scale = ((&distance * Real::from(2_i8)) / &normal_sum_squared)?;
        let candidate = QuadraticBezier2::new(
            self.start()
                .translated(&distance * &start_normal.0, &distance * &start_normal.1),
            self.control()
                .translated(&middle_scale * &normal_sum.0, &middle_scale * &normal_sum.1),
            self.end()
                .translated(&distance * &end_normal.0, &distance * &end_normal.1),
        );
        let half_secant = ((Real::from(2_i8) / &one_plus_cosine)?).sqrt()?;
        let maximum_radial_distance = ((distance.abs() * (Real::from(3_i8) + &tangent_cosine))
            / Real::from(4_i8))?
            * half_secant;
        let radial_error_bound = maximum_radial_distance - distance.abs();
        Ok(Classification::Decided(Blend2dQuadraticOffsetCandidate2 {
            curve: candidate,
            radial_error_bound,
            tangent_cosine,
            distance,
        }))
    }

    /// Builds the deterministic Blend2D quadratic right-parallel candidate.
    pub fn blend2d_offset_right_candidate(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Blend2dQuadraticOffsetCandidate2>> {
        self.blend2d_offset_left_candidate(-distance, policy)
    }

    /// Adaptively constructs and certifies a connected Blend2D quadratic parallel path.
    pub fn approximate_parallel_blend2d_certified(
        &self,
        distance: Real,
        options: &BezierParallelVerificationOptions,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CertifiedBezierParallelPath2>> {
        let analysis = match self
            .parallel_left(distance.clone())?
            .singularity_analysis(policy)?
        {
            Classification::Decided(analysis) => analysis,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if !analysis.source_is_regular() {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        let mut trace = ParallelPathConstructionTrace::default();
        match construct_quadratic_parallel_spans(
            self.clone(),
            Real::zero(),
            Real::one(),
            &distance,
            options,
            policy,
            0,
            &mut trace,
        )? {
            Classification::Decided(()) => {
                Ok(Classification::Decided(CertifiedBezierParallelPath2 {
                    spans: trace.spans,
                    error_bound: options.max_error.clone(),
                    construction_maximum_depth: trace.maximum_depth,
                    verification_leaf_count: trace.verification_leaf_count,
                }))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Runs exact source analysis for later offset adapters.
    pub fn offset_preflight(
        &self,
        policy: &CurveContext,
    ) -> Classification<BezierOffsetPreflight2> {
        let cusp_classification = match self.cusp_classification(policy) {
            Classification::Decided(classification) => classification,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let inflection_classification = self.inflection_classification();
        let start_tangent_status = self.endpoint_tangent(BezierEndpoint::Start).zero_status();
        let end_tangent_status = self.endpoint_tangent(BezierEndpoint::End).zero_status();
        let endpoint_coincidence = self.endpoints_coincident_status();
        Classification::Decided(build_preflight(
            BezierDegree::Quadratic,
            cusp_classification,
            inflection_classification,
            start_tangent_status,
            end_tangent_status,
            endpoint_coincidence,
            policy,
        ))
    }

    /// Attempts a staged certified left offset of this quadratic Bezier.
    pub fn offset_left_staged(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierOffsetCandidate2>> {
        staged_offset_left(self, distance, policy)
    }

    /// Attempts a staged certified right offset of this quadratic Bezier.
    pub fn offset_right_staged(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierOffsetCandidate2>> {
        staged_offset_left(self, -distance, policy)
    }
}

impl CubicBezier2 {
    /// Retains this cubic's exact analytic left parallel.
    pub fn parallel_left(&self, distance: Real) -> CurveResult<BezierParallel2> {
        BezierParallel2::from_controls(
            BezierParallelSource2::Cubic(self.clone()),
            &self.control_points(),
            distance,
        )
    }

    /// Retains this cubic's exact analytic right parallel.
    pub fn parallel_right(&self, distance: Real) -> CurveResult<BezierParallel2> {
        self.parallel_left(-distance)
    }

    /// Reduces this cubic to two joined quadratics using the Blend2D construction.
    pub fn blend2d_two_quadratic_reduction(&self) -> CurveResult<Blend2dCubicQuadraticReduction2> {
        let one_quarter = (Real::one() / Real::from(4_i8))?;
        let three_quarters = &one_quarter * Real::from(3_i8);
        let one_half = (Real::one() / Real::from(2_i8))?;
        let first_control = Point2::new(
            self.start().x() * &one_quarter + self.control1().x() * &three_quarters,
            self.start().y() * &one_quarter + self.control1().y() * &three_quarters,
        );
        let second_control = Point2::new(
            self.end().x() * &one_quarter + self.control2().x() * &three_quarters,
            self.end().y() * &one_quarter + self.control2().y() * &three_quarters,
        );
        let midpoint = first_control.lerp(&second_control, one_half);
        let third_difference_x = self.start().x() - self.control1().x() * Real::from(3_i8)
            + self.control2().x() * Real::from(3_i8)
            - self.end().x();
        let third_difference_y = self.start().y() - self.control1().y() * Real::from(3_i8)
            + self.control2().y() * Real::from(3_i8)
            - self.end().y();
        let third_difference_norm = (&third_difference_x * &third_difference_x
            + &third_difference_y * &third_difference_y)
            .sqrt()?;
        Ok(Blend2dCubicQuadraticReduction2 {
            first: QuadraticBezier2::new(self.start().clone(), first_control, midpoint.clone()),
            second: QuadraticBezier2::new(midpoint, second_control, self.end().clone()),
            same_parameter_error_bound: (third_difference_norm / Real::from(54_i8))?,
        })
    }

    /// Adaptively reduces, offsets, and certifies this cubic through Blend2D quadratics.
    pub fn approximate_parallel_blend2d_certified(
        &self,
        distance: Real,
        options: &BezierParallelVerificationOptions,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CertifiedBezierParallelPath2>> {
        let analysis = match self
            .parallel_left(distance.clone())?
            .singularity_analysis(policy)?
        {
            Classification::Decided(analysis) => analysis,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if !analysis.source_is_regular() {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        let mut trace = ParallelPathConstructionTrace::default();
        match construct_cubic_parallel_spans(
            self.clone(),
            Real::zero(),
            Real::one(),
            &distance,
            options,
            policy,
            0,
            &mut trace,
        )? {
            Classification::Decided(()) => {
                Ok(Classification::Decided(CertifiedBezierParallelPath2 {
                    spans: trace.spans,
                    error_bound: options.max_error.clone(),
                    construction_maximum_depth: trace.maximum_depth,
                    verification_leaf_count: trace.verification_leaf_count,
                }))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Runs exact source analysis for later offset adapters.
    pub fn offset_preflight(
        &self,
        policy: &CurveContext,
    ) -> Classification<BezierOffsetPreflight2> {
        let cusp_classification = match self.cusp_classification(policy) {
            Classification::Decided(classification) => classification,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let inflection_classification = match self.inflection_classification(policy) {
            Classification::Decided(classification) => classification,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let start_tangent_status = self.endpoint_tangent(BezierEndpoint::Start).zero_status();
        let end_tangent_status = self.endpoint_tangent(BezierEndpoint::End).zero_status();
        let endpoint_coincidence = self.endpoints_coincident_status();
        Classification::Decided(build_preflight(
            BezierDegree::Cubic,
            cusp_classification,
            inflection_classification,
            start_tangent_status,
            end_tangent_status,
            endpoint_coincidence,
            policy,
        ))
    }

    /// Attempts a staged certified left offset of this cubic Bezier.
    pub fn offset_left_staged(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierOffsetCandidate2>> {
        staged_offset_left(self, distance, policy)
    }

    /// Attempts a staged certified right offset of this cubic Bezier.
    pub fn offset_right_staged(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierOffsetCandidate2>> {
        staged_offset_left(self, -distance, policy)
    }
}

impl RationalQuadraticBezier2 {
    /// Runs exact source analysis for later rational-conic offset adapters.
    pub fn offset_preflight(
        &self,
        policy: &CurveContext,
    ) -> Classification<BezierOffsetPreflight2> {
        let denominator_risk =
            match weights_known_same_nonzero_sign(self.weights().as_slice(), policy) {
                Some(true) => false,
                Some(false) => true,
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            };
        let start_tangent_status = rational_endpoint_delta_status(self.start(), self.control());
        let end_tangent_status = rational_endpoint_delta_status(self.control(), self.end());
        let endpoint_coincidence = self.start().distance_squared(self.end()).zero_status();
        let mut preflight = build_preflight(
            BezierDegree::Quadratic,
            BezierCuspClassification::None,
            BezierInflectionClassification::NotApplicable,
            start_tangent_status,
            end_tangent_status,
            endpoint_coincidence,
            policy,
        );
        if denominator_risk {
            preflight
                .risks
                .push(BezierOffsetRisk::ProjectiveDenominatorBoundary);
        }
        if rational_collapsed_point_status(self) == ZeroStatus::Zero
            && !preflight.risks.contains(&BezierOffsetRisk::DegeneratePoint)
        {
            preflight.risks.insert(0, BezierOffsetRisk::DegeneratePoint);
        }
        Classification::Decided(preflight)
    }

    /// Attempts a staged certified left offset of this rational quadratic conic.
    pub fn offset_left_staged(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierOffsetCandidate2>> {
        staged_offset_left(self, distance, policy)
    }

    /// Attempts a staged certified right offset of this rational quadratic conic.
    pub fn offset_right_staged(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierOffsetCandidate2>> {
        staged_offset_left(self, -distance, policy)
    }
}

impl CurvePath2 {
    /// Constructs a connected certified left parallel for supported smooth paths.
    ///
    /// Native lines/arcs and polynomial PH offsets remain exact. General
    /// quadratic/cubic spans use the adaptive Blend2D construction followed by
    /// the conservative verifier. If adjacent primitive parallels do not meet
    /// exactly (the usual case at an authored corner), this returns
    /// `Unsupported`; selecting a miter, round, or bevel join belongs to the
    /// region/string offset layer.
    pub fn approximate_parallel_blend2d_certified(
        &self,
        distance: Real,
        options: &BezierParallelVerificationOptions,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<CertifiedCurvePathParallel2>> {
        let source_curve_count = self.curves().len();
        if real_sign(&distance, policy) == Some(RealSign::Zero) {
            return Ok(Classification::Decided(CertifiedCurvePathParallel2 {
                path: self.clone(),
                max_parallel_error: Real::zero(),
                source_curve_count,
                output_curve_count: source_curve_count,
                exact_source_curve_count: source_curve_count,
                approximated_source_curve_count: 0,
                verification_leaf_count: 0,
            }));
        }
        if real_sign(&distance, policy).is_none() {
            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
        }

        let mut output = Vec::new();
        let mut exact_source_curve_count = 0;
        let mut approximated_source_curve_count = 0;
        let mut verification_leaf_count = 0;
        for source in self.curves() {
            match source.geometry() {
                CurveGeometry2::Line(line) => {
                    output.push(Curve2::from(
                        line.offset_left(distance.clone())
                            .map_err(|cause| parallel_path_error(source, cause))?,
                    ));
                    exact_source_curve_count += 1;
                }
                CurveGeometry2::CircularArc(arc) => {
                    let offset = match arc
                        .offset_left(distance.clone(), policy)
                        .map_err(|cause| parallel_path_error(source, cause))?
                    {
                        Classification::Decided(offset) => offset,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    output.push(Curve2::from(offset));
                    exact_source_curve_count += 1;
                }
                CurveGeometry2::QuadraticBezier(curve) => {
                    match append_polynomial_parallel(
                        curve
                            .parallel_left(distance.clone())
                            .map_err(|cause| parallel_path_error(source, cause))?,
                        || {
                            curve.approximate_parallel_blend2d_certified(
                                distance.clone(),
                                options,
                                policy,
                            )
                        },
                        policy,
                        &mut output,
                        &mut verification_leaf_count,
                    )
                    .map_err(|cause| parallel_path_error(source, cause))?
                    {
                        Classification::Decided(true) => exact_source_curve_count += 1,
                        Classification::Decided(false) => approximated_source_curve_count += 1,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                CurveGeometry2::CubicBezier(curve) => {
                    match append_polynomial_parallel(
                        curve
                            .parallel_left(distance.clone())
                            .map_err(|cause| parallel_path_error(source, cause))?,
                        || {
                            curve.approximate_parallel_blend2d_certified(
                                distance.clone(),
                                options,
                                policy,
                            )
                        },
                        policy,
                        &mut output,
                        &mut verification_leaf_count,
                    )
                    .map_err(|cause| parallel_path_error(source, cause))?
                    {
                        Classification::Decided(true) => exact_source_curve_count += 1,
                        Classification::Decided(false) => approximated_source_curve_count += 1,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                CurveGeometry2::RationalQuadraticBezier(_)
                | CurveGeometry2::RationalBezier(_)
                | CurveGeometry2::PolynomialBSpline(_)
                | CurveGeometry2::Nurbs(_) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
            }
        }

        if !parallel_curves_are_connected(&output) {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        let source_closed = self.start().distance_squared(self.end()).zero_status();
        let output_closed = output
            .first()
            .zip(output.last())
            .map_or(ZeroStatus::NonZero, |(first, last)| {
                first.start().distance_squared(last.end()).zero_status()
            });
        match (source_closed, output_closed) {
            (ZeroStatus::Zero, ZeroStatus::NonZero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            (ZeroStatus::Zero, ZeroStatus::Unknown) | (ZeroStatus::Unknown, _) => {
                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
            }
            _ => {}
        }
        let output_curve_count = output.len();
        let path = CurvePath2::try_new(output)?;
        Ok(Classification::Decided(CertifiedCurvePathParallel2 {
            path,
            max_parallel_error: options.max_error().clone(),
            source_curve_count,
            output_curve_count,
            exact_source_curve_count,
            approximated_source_curve_count,
            verification_leaf_count,
        }))
    }
}

fn append_polynomial_parallel<F>(
    parallel: BezierParallel2,
    approximate: F,
    policy: &CurveContext,
    output: &mut Vec<Curve2>,
    verification_leaf_count: &mut usize,
) -> CurveResult<Classification<bool>>
where
    F: FnOnce() -> CurveResult<Classification<CertifiedBezierParallelPath2>>,
{
    let singularities = match parallel.singularity_analysis(policy)? {
        Classification::Decided(singularities) => singularities,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if !singularities.source_is_regular() || !singularities.parallel_is_cusp_free() {
        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
    }
    match parallel.exact_pythagorean_hodograph_offset(policy)? {
        Classification::Decided(Some(exact)) => {
            output.push(Curve2::from(exact.curve().clone()));
            return Ok(Classification::Decided(true));
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    let fitted = match approximate()? {
        Classification::Decided(fitted) => fitted,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    *verification_leaf_count += fitted.verification_leaf_count();
    output.extend(
        fitted
            .spans()
            .iter()
            .map(|span| match span.approximation().curve() {
                BezierParallelApproximationCurve2::Quadratic(curve) => Curve2::from(curve.clone()),
                BezierParallelApproximationCurve2::Cubic(curve) => Curve2::from(curve.clone()),
            }),
    );
    Ok(Classification::Decided(false))
}

fn parallel_curves_are_connected(curves: &[Curve2]) -> bool {
    curves.windows(2).all(|pair| {
        pair[0].end() == pair[1].start()
            || pair[0]
                .end()
                .distance_squared(pair[1].start())
                .zero_status()
                == ZeroStatus::Zero
    })
}

fn parallel_path_error(source: &Curve2, cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(CurveOperation2::Offset, source.family(), cause)
}

trait StagedBezierOffset {
    fn offset_preflight(&self, policy: &CurveContext) -> Classification<BezierOffsetPreflight2>;
    fn fit_exact_line_image(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierLineImageFitRelation>>;

    fn exact_pythagorean_hodograph_offset(
        &self,
        _distance: Real,
        _policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CertifiedPythagoreanHodographOffset2>>> {
        Ok(Classification::Decided(None))
    }
}

impl StagedBezierOffset for QuadraticBezier2 {
    fn offset_preflight(&self, policy: &CurveContext) -> Classification<BezierOffsetPreflight2> {
        QuadraticBezier2::offset_preflight(self, policy)
    }

    fn fit_exact_line_image(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierLineImageFitRelation>> {
        QuadraticBezier2::fit_exact_line_image(self, policy)
    }

    fn exact_pythagorean_hodograph_offset(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CertifiedPythagoreanHodographOffset2>>> {
        self.parallel_left(distance)?
            .exact_pythagorean_hodograph_offset(policy)
    }
}

impl StagedBezierOffset for CubicBezier2 {
    fn offset_preflight(&self, policy: &CurveContext) -> Classification<BezierOffsetPreflight2> {
        CubicBezier2::offset_preflight(self, policy)
    }

    fn fit_exact_line_image(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierLineImageFitRelation>> {
        CubicBezier2::fit_exact_line_image(self, policy)
    }

    fn exact_pythagorean_hodograph_offset(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CertifiedPythagoreanHodographOffset2>>> {
        self.parallel_left(distance)?
            .exact_pythagorean_hodograph_offset(policy)
    }
}

impl StagedBezierOffset for RationalQuadraticBezier2 {
    fn offset_preflight(&self, policy: &CurveContext) -> Classification<BezierOffsetPreflight2> {
        RationalQuadraticBezier2::offset_preflight(self, policy)
    }

    fn fit_exact_line_image(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierLineImageFitRelation>> {
        RationalQuadraticBezier2::fit_exact_line_image(self, policy)
    }
}

fn staged_offset_left<C>(
    curve: &C,
    distance: Real,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierOffsetCandidate2>>
where
    C: StagedBezierOffset,
{
    let preflight = match curve.offset_preflight(policy) {
        Classification::Decided(preflight) => preflight,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let line_image_fit = match curve.fit_exact_line_image(policy) {
        Ok(relation) => relation,
        Err(CurveError::ZeroLengthLine)
            if preflight.risks.contains(&BezierOffsetRisk::DegeneratePoint) =>
        {
            return Ok(Classification::Decided(
                BezierOffsetCandidate2::Unresolved {
                    preflight,
                    distance,
                },
            ));
        }
        Err(error) => return Err(error),
    };
    match line_image_fit {
        Classification::Decided(BezierLineImageFitRelation::Fit(fit)) => Ok(
            Classification::Decided(BezierOffsetCandidate2::ExactLineImage {
                offset: fit.offset_left_exact(distance)?,
                preflight,
            }),
        ),
        Classification::Decided(BezierLineImageFitRelation::NotLine) => {
            match curve.exact_pythagorean_hodograph_offset(distance.clone(), policy)? {
                Classification::Decided(Some(offset)) => Ok(Classification::Decided(
                    BezierOffsetCandidate2::ExactPythagoreanHodograph { offset, preflight },
                )),
                Classification::Decided(None) => Ok(Classification::Decided(
                    BezierOffsetCandidate2::Unresolved {
                        preflight,
                        distance,
                    },
                )),
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            }
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn rational_endpoint_delta_status(first: &Point2, second: &Point2) -> ZeroStatus {
    first.distance_squared(second).zero_status()
}

fn rational_collapsed_point_status(curve: &RationalQuadraticBezier2) -> ZeroStatus {
    let start_control = curve
        .start()
        .distance_squared(curve.control())
        .zero_status();
    let control_end = curve.control().distance_squared(curve.end()).zero_status();
    match (start_control, control_end) {
        (ZeroStatus::Zero, ZeroStatus::Zero) => ZeroStatus::Zero,
        (ZeroStatus::NonZero, _) | (_, ZeroStatus::NonZero) => ZeroStatus::NonZero,
        _ => ZeroStatus::Unknown,
    }
}

fn weights_known_same_nonzero_sign(weights: &[&Real], policy: &CurveContext) -> Option<bool> {
    let mut expected = None;
    for weight in weights {
        let sign = real_sign(weight, policy)?;
        match sign {
            RealSign::Positive | RealSign::Negative => {
                if let Some(expected) = expected {
                    if expected != sign {
                        return Some(false);
                    }
                } else {
                    expected = Some(sign);
                }
            }
            RealSign::Zero => return Some(false),
        }
    }
    Some(expected.is_some())
}

fn build_preflight(
    degree: BezierDegree,
    cusp_classification: BezierCuspClassification,
    inflection_classification: BezierInflectionClassification,
    start_tangent_status: ZeroStatus,
    end_tangent_status: ZeroStatus,
    endpoint_coincidence: ZeroStatus,
    policy: &CurveContext,
) -> BezierOffsetPreflight2 {
    let mut risks = Vec::new();
    match &cusp_classification {
        BezierCuspClassification::DegeneratePoint => risks.push(BezierOffsetRisk::DegeneratePoint),
        BezierCuspClassification::Cusps { .. } => risks.push(BezierOffsetRisk::Cusp),
        BezierCuspClassification::Unresolved => risks.push(BezierOffsetRisk::Cusp),
        BezierCuspClassification::None => {}
    }
    match &inflection_classification {
        BezierInflectionClassification::Inflections { .. } => {
            risks.push(BezierOffsetRisk::Inflection);
        }
        BezierInflectionClassification::AllCurvatureZero => {
            risks.push(BezierOffsetRisk::AllCurvatureZero);
        }
        BezierInflectionClassification::Unresolved => risks.push(BezierOffsetRisk::Inflection),
        BezierInflectionClassification::NotApplicable | BezierInflectionClassification::None => {}
    }
    push_endpoint_normal_risk(&mut risks, BezierEndpoint::Start, start_tangent_status);
    push_endpoint_normal_risk(&mut risks, BezierEndpoint::End, end_tangent_status);
    if endpoint_coincidence == ZeroStatus::Zero {
        risks.push(BezierOffsetRisk::CoincidentEndpoints);
    }
    BezierOffsetPreflight2 {
        degree,
        cusp_classification,
        inflection_classification,
        start_tangent_status,
        end_tangent_status,
        endpoint_coincidence,
        risks,
        construction_policy: *policy,
    }
}

fn push_endpoint_normal_risk(
    risks: &mut Vec<BezierOffsetRisk>,
    endpoint: BezierEndpoint,
    zero_status: ZeroStatus,
) {
    match zero_status {
        ZeroStatus::Zero => risks.push(BezierOffsetRisk::UndefinedEndpointNormal { endpoint }),
        ZeroStatus::Unknown => risks.push(BezierOffsetRisk::UnresolvedEndpointNormal { endpoint }),
        ZeroStatus::NonZero => {}
    }
}

fn polynomial_derivative(coefficients: &[Real]) -> Vec<Real> {
    coefficients
        .iter()
        .enumerate()
        .skip(1)
        .map(|(degree, coefficient)| coefficient * Real::from(degree as u64))
        .collect()
}

#[derive(Default)]
struct ParallelPathConstructionTrace {
    spans: Vec<CertifiedBezierParallelSpan2>,
    maximum_depth: usize,
    verification_leaf_count: usize,
}

#[allow(clippy::too_many_arguments)]
fn construct_quadratic_parallel_spans(
    source: QuadraticBezier2,
    source_start: Real,
    source_end: Real,
    distance: &Real,
    options: &BezierParallelVerificationOptions,
    policy: &CurveContext,
    depth: usize,
    trace: &mut ParallelPathConstructionTrace,
) -> CurveResult<Classification<()>> {
    trace.maximum_depth = trace.maximum_depth.max(depth);
    if let Classification::Decided(candidate) =
        source.blend2d_offset_left_candidate(distance.clone(), policy)?
    {
        let parallel = source.parallel_left(distance.clone())?;
        if let Classification::Decided(approximation) = parallel.verify_polynomial_candidate(
            candidate.curve().clone().into(),
            options,
            policy,
        )? {
            trace.verification_leaf_count += approximation.leaf_count();
            trace.spans.push(CertifiedBezierParallelSpan2 {
                source_start,
                source_end,
                approximation,
            });
            return Ok(Classification::Decided(()));
        }
    }
    if depth >= options.max_depth {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let half = (Real::one() / Real::from(2_i8))?;
    let midpoint = (&source_start + &source_end) * &half;
    let (left, right) = source.split_at_exact(half);
    match construct_quadratic_parallel_spans(
        left,
        source_start,
        midpoint.clone(),
        distance,
        options,
        policy,
        depth + 1,
        trace,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    construct_quadratic_parallel_spans(
        right,
        midpoint,
        source_end,
        distance,
        options,
        policy,
        depth + 1,
        trace,
    )
}

#[allow(clippy::too_many_arguments)]
fn construct_cubic_parallel_spans(
    source: CubicBezier2,
    source_start: Real,
    source_end: Real,
    distance: &Real,
    options: &BezierParallelVerificationOptions,
    policy: &CurveContext,
    depth: usize,
    trace: &mut ParallelPathConstructionTrace,
) -> CurveResult<Classification<()>> {
    trace.maximum_depth = trace.maximum_depth.max(depth);
    let parallel = source.parallel_left(distance.clone())?;
    let candidate = match parallel.levien_cubic_candidate(policy) {
        Ok(Classification::Decided(candidate)) => Some(candidate),
        Ok(Classification::Uncertain(_)) | Err(CurveError::Real(_)) => None,
        Err(error) => return Err(error),
    };
    if let Some(candidate) = candidate {
        let approximation = match parallel.verify_polynomial_candidate(
            candidate.curve().clone().into(),
            options,
            policy,
        ) {
            Ok(Classification::Decided(approximation)) => Some(approximation),
            Ok(Classification::Uncertain(_)) | Err(CurveError::Real(_)) => None,
            Err(error) => return Err(error),
        };
        if let Some(approximation) = approximation {
            trace.verification_leaf_count += approximation.leaf_count();
            trace.spans.push(CertifiedBezierParallelSpan2 {
                source_start,
                source_end,
                approximation,
            });
            return Ok(Classification::Decided(()));
        }
    }
    if depth >= options.max_depth {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let half = (Real::one() / Real::from(2_i8))?;
    let midpoint = (&source_start + &source_end) * &half;
    let (source_left, source_right) = source.split_at_exact(half.clone());
    let reduction = source.blend2d_two_quadratic_reduction()?;
    match construct_cubic_reduced_half(
        source_left,
        reduction.first().clone(),
        source_start,
        midpoint.clone(),
        distance,
        options,
        policy,
        depth + 1,
        trace,
    )? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    construct_cubic_reduced_half(
        source_right,
        reduction.second().clone(),
        midpoint,
        source_end,
        distance,
        options,
        policy,
        depth + 1,
        trace,
    )
}

#[allow(clippy::too_many_arguments)]
fn construct_cubic_reduced_half(
    source: CubicBezier2,
    reduced: QuadraticBezier2,
    source_start: Real,
    source_end: Real,
    distance: &Real,
    options: &BezierParallelVerificationOptions,
    policy: &CurveContext,
    depth: usize,
    trace: &mut ParallelPathConstructionTrace,
) -> CurveResult<Classification<()>> {
    trace.maximum_depth = trace.maximum_depth.max(depth);
    let candidate = match reduced.blend2d_offset_left_candidate(distance.clone(), policy) {
        Ok(Classification::Decided(candidate)) => Some(candidate),
        Ok(Classification::Uncertain(_)) | Err(CurveError::Real(_)) => None,
        Err(error) => return Err(error),
    };
    if let Some(candidate) = candidate {
        let parallel = source.parallel_left(distance.clone())?;
        let approximation = match parallel.verify_polynomial_candidate(
            candidate.curve().clone().into(),
            options,
            policy,
        ) {
            Ok(Classification::Decided(approximation)) => Some(approximation),
            Ok(Classification::Uncertain(_)) | Err(CurveError::Real(_)) => None,
            Err(error) => return Err(error),
        };
        if let Some(approximation) = approximation {
            trace.verification_leaf_count += approximation.leaf_count();
            trace.spans.push(CertifiedBezierParallelSpan2 {
                source_start,
                source_end,
                approximation,
            });
            return Ok(Classification::Decided(()));
        }
    }
    if depth >= options.max_depth {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    construct_cubic_parallel_spans(
        source,
        source_start,
        source_end,
        distance,
        options,
        policy,
        depth,
        trace,
    )
}

fn polynomial_evaluate(coefficients: &[Real], parameter: &Real) -> Real {
    coefficients
        .iter()
        .rev()
        .fold(Real::zero(), |accumulator, coefficient| {
            accumulator * parameter + coefficient
        })
}

fn polynomial_add(first: &[Real], second: &[Real]) -> Vec<Real> {
    let length = first.len().max(second.len());
    (0..length)
        .map(|index| {
            first.get(index).cloned().unwrap_or_else(Real::zero)
                + second.get(index).cloned().unwrap_or_else(Real::zero)
        })
        .collect()
}

fn polynomial_subtract(first: &[Real], second: &[Real]) -> Vec<Real> {
    let length = first.len().max(second.len());
    (0..length)
        .map(|index| {
            first.get(index).cloned().unwrap_or_else(Real::zero)
                - second.get(index).cloned().unwrap_or_else(Real::zero)
        })
        .collect()
}

fn polynomial_multiply(first: &[Real], second: &[Real]) -> Vec<Real> {
    if first.is_empty() || second.is_empty() {
        return Vec::new();
    }
    let mut product = vec![Real::zero(); first.len() + second.len() - 1];
    for (first_degree, first_coefficient) in first.iter().enumerate() {
        for (second_degree, second_coefficient) in second.iter().enumerate() {
            let term = first_coefficient * second_coefficient;
            product[first_degree + second_degree] = &product[first_degree + second_degree] + term;
        }
    }
    product
}

fn polynomial_scale(coefficients: &[Real], scale: &Real) -> Vec<Real> {
    coefficients
        .iter()
        .map(|coefficient| coefficient * scale)
        .collect()
}

fn polynomial_power(coefficients: &[Real], exponent: usize) -> Vec<Real> {
    let mut result = vec![Real::one()];
    for _ in 0..exponent {
        result = polynomial_multiply(&result, coefficients);
    }
    result
}

fn polynomial_square_root(
    coefficients: &[Real],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<Real>>>> {
    let mut normalized = coefficients.to_vec();
    while let Some(coefficient) = normalized.last() {
        match real_sign(coefficient, policy) {
            Some(RealSign::Zero) => {
                normalized.pop();
            }
            Some(RealSign::Positive | RealSign::Negative) => break,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
    if normalized.is_empty() {
        return Ok(Classification::Decided(Some(vec![Real::zero()])));
    }
    let degree = normalized.len() - 1;
    if !degree.is_multiple_of(2) {
        return Ok(Classification::Decided(None));
    }
    let root_degree = degree / 2;
    let leading = normalized[degree].clone();
    match real_sign(&leading, policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Zero | RealSign::Negative) => {
            return Ok(Classification::Decided(None));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let leading_root = leading.sqrt()?;
    let mut root = vec![Real::zero(); root_degree + 1];
    root[root_degree] = leading_root.clone();
    for index in (0..root_degree).rev() {
        let power = root_degree + index;
        let mut known = Real::zero();
        for left in (index + 1)..=root_degree {
            let Some(right) = power.checked_sub(left) else {
                continue;
            };
            if right > root_degree || right <= index {
                continue;
            }
            known = &known + &root[left] * &root[right];
        }
        let residual = normalized.get(power).cloned().unwrap_or_else(Real::zero) - known;
        root[index] = (residual / (&leading_root * Real::from(2_i8)))?;
    }
    let replay = polynomial_multiply(&root, &root);
    let difference = polynomial_subtract(&replay, &normalized);
    for coefficient in difference {
        match real_sign(&coefficient, policy) {
            Some(RealSign::Zero) => {}
            Some(RealSign::Positive | RealSign::Negative) => {
                return Ok(Classification::Decided(None));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
    Ok(Classification::Decided(Some(root)))
}

fn polynomial_from_coefficients(
    coefficients: Vec<Real>,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameterPolynomial>>> {
    match BezierParameterPolynomial::try_new_power_basis(coefficients, policy) {
        Ok(Classification::Decided(polynomial)) => Ok(Classification::Decided(Some(polynomial))),
        Err(CurveError::InvalidBezierPolynomial) => Ok(Classification::Decided(None)),
        Ok(Classification::Uncertain(reason)) => Ok(Classification::Uncertain(reason)),
        Err(error) => Err(error),
    }
}

fn parameter_matches_any(
    candidate: &BezierParameter2,
    parameters: &[BezierParameter2],
    policy: &CurveContext,
) -> CurveResult<bool> {
    for parameter in parameters {
        match candidate.cmp_by_interval(parameter, policy)? {
            Classification::Decided(std::cmp::Ordering::Equal) => return Ok(true),
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => {
                return Err(CurveError::Topology(format!(
                    "parallel cusp/source singularity equality remained uncertain: {reason:?}"
                )));
            }
        }
    }
    Ok(false)
}

fn signed_polynomial_at_root(
    polynomial: Option<&BezierParameterPolynomial>,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let Some(polynomial) = polynomial else {
        return Ok(Classification::Decided(RealSign::Zero));
    };
    match parameter {
        BezierParameter2::Exact(parameter) => {
            match real_sign(&polynomial.evaluate(parameter), policy) {
                Some(sign) => Ok(Classification::Decided(sign)),
                None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        BezierParameter2::Algebraic(parameter) => signed_polynomial_on_isolating_interval(
            polynomial,
            parameter.polynomial(),
            parameter.interval(),
            policy,
            0,
        ),
    }
}

fn signed_polynomial_on_isolating_interval(
    filter: &BezierParameterPolynomial,
    defining: &BezierParameterPolynomial,
    interval: &BezierParameterInterval,
    policy: &CurveContext,
    depth: usize,
) -> CurveResult<Classification<RealSign>> {
    let count = match filter.root_count_in_interval(interval, policy) {
        Ok(Classification::Decided(count)) => count,
        Ok(Classification::Uncertain(reason)) => return Ok(Classification::Uncertain(reason)),
        Err(CurveError::InvalidBezierAlgebraicParameter) => usize::MAX,
        Err(error) => return Err(error),
    };
    if count == 0 {
        return match real_sign(&filter.evaluate(interval.start()), policy) {
            Some(sign) => Ok(Classification::Decided(sign)),
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
    }
    if depth >= 256 {
        return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
    }
    let midpoint = ((interval.start() + interval.end()) / Real::from(2_i8))?;
    if real_sign(&defining.evaluate(&midpoint), policy) == Some(RealSign::Zero) {
        return match real_sign(&filter.evaluate(&midpoint), policy) {
            Some(sign) => Ok(Classification::Decided(sign)),
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
    }
    let left =
        match BezierParameterInterval::try_new(interval.start().clone(), midpoint.clone(), policy)?
        {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let right = match BezierParameterInterval::try_new(midpoint, interval.end().clone(), policy)? {
        Classification::Decided(interval) => interval,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let left_count = match defining.root_count_in_interval(&left, policy)? {
        Classification::Decided(count) => count,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let next = if left_count == 1 {
        left
    } else {
        let right_count = match defining.root_count_in_interval(&right, policy)? {
            Classification::Decided(count) => count,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if right_count != 1 {
            return Err(CurveError::InvalidBezierAlgebraicParameter);
        }
        right
    };
    signed_polynomial_on_isolating_interval(filter, defining, &next, policy, depth + 1)
}

#[cfg(test)]
mod conversion_tests {
    use super::*;

    #[test]
    fn power_to_bernstein_remains_exact_beyond_u64_binomials() {
        let degree = 80_usize;
        let coefficients =
            power_to_bernstein_coefficients(&[Real::zero(), Real::one()], degree).unwrap();
        let degree_real = Real::from(u64::try_from(degree).unwrap());
        let policy = CurveContext::STRICT;

        assert_eq!(coefficients.len(), degree + 1);
        for (index, coefficient) in coefficients.iter().enumerate() {
            let expected = (Real::from(u64::try_from(index).unwrap()) / &degree_real).unwrap();
            assert_eq!(
                compare_reals(coefficient, &expected, &policy),
                Some(std::cmp::Ordering::Equal)
            );
        }
    }
}
