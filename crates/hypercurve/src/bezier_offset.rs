//! Certified staged offsets for polynomial and rational Bezier curves.
//!
//! General parallels remain retained analytic expressions because they are not
//! generally finite rational Beziers. Exact source and offset cusps are isolated before
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

use std::sync::{Arc, OnceLock};

#[cfg(feature = "predicates")]
use crate::bezier_algebraic_image::parameter_representation;
use crate::bezier_parameter::{bernstein_to_power_coefficients, power_to_bernstein_coefficients};
use crate::classify::{compare_reals, in_closed_unit_interval, real_sign};
use crate::rational_bezier_general::{
    ResultantParameterProjection, rational_parameter_image, resultant_parameter_projection,
};
use crate::{
    Aabb2, BezierCuspClassification, BezierDegree, BezierEndpoint, BezierInflectionClassification,
    BezierLineImageFitRelation, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, CertifiedBezierLineImageOffset2, Classification, CubicBezier2,
    Curve2, CurveContext, CurveDerivative2, CurveError, CurveGeometry2, CurveOperation2,
    CurvePath2, CurveResult, ExactCurveError, ExactCurveResult, LineSeg2, Point2, QuadraticBezier2,
    RationalBezier2, RationalBezierIntersectionCandidates2, RationalBezierIntersectionContacts2,
    RationalBezierIntersectionOverlap2, RationalBezierOverlapOrientation2,
    RationalQuadraticBezier2, Real, UncertaintyReason,
};
use hyperreal::{RealSign, ZeroKnowledge as ZeroStatus};
#[cfg(feature = "predicates")]
use hypersolve::{
    AlgebraicFiberRootCountStatus, PredicateCertainty,
    count_bivariate_common_fiber_roots_at_algebraic_parameter,
};
use hypersolve::{
    BivariatePolynomial, BivariatePolynomialAxisFactorStatus, BivariatePolynomialComponentStatus,
    CurveIntersectionParameterLiftMap, CurveIntersectionParameterLiftReport,
    CurveIntersectionParameterLiftStatus, CurveIntersectionResultantConfig,
    CurveResultantParameter, RationalParametricCurve2,
    extract_bivariate_polynomial_system_axis_factors,
    linear_parameter_lifts_bivariate_polynomial_system,
    parameter_component_bivariate_polynomial_system, resultant_bivariate_polynomial_system,
    subresultant_chain_univariate_polynomials,
};

#[derive(Clone, Debug, PartialEq)]
enum BezierParallelSource2 {
    Quadratic(QuadraticBezier2),
    Cubic(CubicBezier2),
    Rational(RationalBezier2),
}

impl BezierParallelSource2 {
    fn reversed(&self) -> Self {
        match self {
            Self::Quadratic(source) => {
                let reversed = if source.retained_exact_line_image().is_some() {
                    QuadraticBezier2::with_retained_exact_line_image(
                        source.end().clone(),
                        source.control().clone(),
                        source.start().clone(),
                    )
                    .expect("reversing a retained exact line preserves distinct endpoints")
                } else {
                    QuadraticBezier2::new(
                        source.end().clone(),
                        source.control().clone(),
                        source.start().clone(),
                    )
                };
                Self::Quadratic(reversed)
            }
            Self::Cubic(source) => Self::Cubic(CubicBezier2::new(
                source.end().clone(),
                source.control2().clone(),
                source.control1().clone(),
                source.start().clone(),
            )),
            Self::Rational(source) => Self::Rational(source.reversed()),
        }
    }

    fn split_at_exact(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(Self, Self)>> {
        match self {
            Self::Quadratic(source) => {
                let (left, right) = source.split_at_exact(parameter.clone());
                Ok(Classification::Decided((
                    Self::Quadratic(left),
                    Self::Quadratic(right),
                )))
            }
            Self::Cubic(source) => {
                let (left, right) = source.split_at_exact(parameter.clone());
                Ok(Classification::Decided((
                    Self::Cubic(left),
                    Self::Cubic(right),
                )))
            }
            Self::Rational(source) => source.split_at_exact(parameter, policy).map(|split| {
                split.map(|(left, right)| (Self::Rational(left), Self::Rational(right)))
            }),
        }
    }

    fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match self {
            Self::Quadratic(source) => source
                .subcurve_between_exact(start, end, policy)
                .map(Self::Quadratic)
                .map(Classification::Decided),
            Self::Cubic(source) => source
                .subcurve_between_exact(start, end, policy)
                .map(Self::Cubic)
                .map(Classification::Decided),
            Self::Rational(source) => source
                .subcurve_between_exact(start, end, policy)
                .map(|subcurve| subcurve.map(Self::Rational)),
        }
    }

    fn certified_bounds(&self, policy: &CurveContext) -> Classification<Aabb2> {
        match self {
            Self::Quadratic(source) => source.certified_bounds(policy),
            Self::Cubic(source) => source.certified_bounds(policy),
            Self::Rational(source) => source.certified_bounds_classified(policy),
        }
    }

    fn to_rational_bezier(&self) -> CurveResult<RationalBezier2> {
        match self {
            Self::Quadratic(source) => RationalBezier2::try_new(
                source.control_points().into_iter().cloned().collect(),
                vec![Real::one(); 3],
            ),
            Self::Cubic(source) => RationalBezier2::try_new(
                source.control_points().into_iter().cloned().collect(),
                vec![Real::one(); 4],
            ),
            Self::Rational(source) => Ok(source.clone()),
        }
    }
}

#[derive(Debug)]
struct BezierParallelData2 {
    source: BezierParallelSource2,
    distance: Real,
    polynomial_power_basis: OnceLock<(Vec<Real>, Vec<Real>)>,
    differential: OnceLock<BezierParallelDifferential2>,
    certified_ph_offset: OnceLock<Option<Arc<CertifiedPythagoreanHodographOffset2>>>,
}

#[derive(Debug)]
struct BezierParallelDifferential2 {
    tangent_x: Vec<Real>,
    tangent_y: Vec<Real>,
    tangent_derivative_x: Vec<Real>,
    tangent_derivative_y: Vec<Real>,
}

struct BezierParallelPowerBasisRef<'a> {
    x_numerator: &'a [Real],
    y_numerator: &'a [Real],
    weight: Option<&'a [Real]>,
}

/// Exact analytic parallel of a polynomial or rational Bezier curve.
///
/// A general parallel is not itself a finite rational Bezier. This compact,
/// clone-shared carrier retains
/// the exact expression `P(t) + d * left_normal(P'(t))`; fitted Beziers are
/// separate approximation products and can therefore be verified against this
/// object without confusing exact scalar coordinates with exact curve image.
/// Polynomial sources retain their native compact representation; rational
/// sources use homogeneous coordinates. The tangent numerator and its
/// derivative are built lazily and shared by every clone.
#[derive(Clone)]
pub struct BezierParallel2 {
    data: Arc<BezierParallelData2>,
}

/// Complete exact parameter evidence for incidence on an analytic parallel.
#[derive(Clone, Debug, PartialEq)]
pub enum BezierParallelIncidence2 {
    /// Every defined parameter satisfies the incidence query.
    EntireCurve,
    /// The complete ordered set of represented or isolated algebraic parameters.
    Parameters(Vec<BezierParameter2>),
}

/// Complete resultant projections for an analytic parallel and rational Bezier pair.
#[derive(Clone, Debug, PartialEq)]
pub enum BezierParallelIntersectionCandidates2 {
    /// Exact elimination proves that no finite parameter pair can intersect.
    NoIntersection,
    /// Both projections contain every possible finite contact parameter.
    Candidates {
        /// Ordered represented or algebraically isolated parallel parameters.
        parallel_parameters: Vec<BezierParameter2>,
        /// Ordered represented or algebraically isolated rational-curve parameters.
        other_parameters: Vec<BezierParameter2>,
    },
    /// A projection vanished identically and requires shared-component replay.
    DegenerateResultant,
}

struct BezierParallelIntersectionCandidateSystem2 {
    candidates: BezierParallelIntersectionCandidates2,
    replay_equations: Option<[BivariatePolynomial; 2]>,
    overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
    component_pairs: Arc<[BezierParallelIntersectionParameterPair2]>,
}

#[derive(Clone, PartialEq)]
struct BezierParallelIntersectionParameterPair2 {
    parallel_parameter: BezierParameter2,
    other_parameter: BezierParameter2,
}

impl BezierParallelIntersectionCandidateSystem2 {
    fn projected(
        candidates: BezierParallelIntersectionCandidates2,
        replay_equations: Option<[BivariatePolynomial; 2]>,
    ) -> Self {
        Self {
            candidates,
            replay_equations,
            overlaps: Arc::from([]),
            component_pairs: Arc::from([]),
        }
    }

    fn overlaps(overlaps: Arc<[RationalBezierIntersectionOverlap2]>) -> Self {
        Self {
            candidates: BezierParallelIntersectionCandidates2::NoIntersection,
            replay_equations: None,
            overlaps,
            component_pairs: Arc::from([]),
        }
    }
}

/// One exactly replayed contact between an analytic parallel and a rational Bezier.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierParallelIntersectionContact2 {
    parallel_parameter: BezierParameter2,
    other_parameter: BezierParameter2,
    point: crate::RationalBezierIntersectionPointEvidence2,
    certified_transverse: bool,
}

impl BezierParallelIntersectionContact2 {
    /// Returns the exact parameter on the analytic parallel.
    pub const fn parallel_parameter(&self) -> &BezierParameter2 {
        &self.parallel_parameter
    }

    /// Returns the exact parameter on the rational Bezier.
    pub const fn other_parameter(&self) -> &BezierParameter2 {
        &self.other_parameter
    }

    /// Returns retained affine point evidence evaluated on the rational Bezier.
    pub const fn point(&self) -> &crate::RationalBezierIntersectionPointEvidence2 {
        &self.point
    }

    /// Returns whether exact first derivatives certify a transverse contact.
    pub const fn is_certified_transverse(&self) -> bool {
        self.certified_transverse
    }
}

/// Complete or explicitly incomplete analytic-parallel/rational-Bezier intersection set.
///
/// Isolated contacts and positive-dimensional overlaps are independent slices:
/// both may occur for one authored pair. When replay cannot complete, the
/// original complete parameter projections remain behind one shared pointer
/// instead of inflating every successful result.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierParallelIntersectionSet2 {
    contacts: Arc<[BezierParallelIntersectionContact2]>,
    overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
    incomplete_candidates: Option<Arc<BezierParallelIntersectionCandidates2>>,
}

impl BezierParallelIntersectionSet2 {
    fn complete(
        contacts: Arc<[BezierParallelIntersectionContact2]>,
        overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
    ) -> Self {
        Self {
            contacts,
            overlaps,
            incomplete_candidates: None,
        }
    }

    fn incomplete(
        contacts: Arc<[BezierParallelIntersectionContact2]>,
        overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
        candidates: BezierParallelIntersectionCandidates2,
    ) -> Self {
        Self {
            contacts,
            overlaps,
            incomplete_candidates: Some(Arc::new(candidates)),
        }
    }

    /// Returns every exactly replayed isolated selected-branch contact.
    pub fn contacts(&self) -> &[BezierParallelIntersectionContact2] {
        &self.contacts
    }

    /// Returns every exactly certified positive-length overlap.
    ///
    /// Each overlap uses its first range for the analytic parallel and its
    /// second range for the rational operand.
    pub fn overlaps(&self) -> &[RationalBezierIntersectionOverlap2] {
        &self.overlaps
    }

    /// Returns whether all possible finite contacts and components were decided.
    pub const fn is_complete(&self) -> bool {
        self.incomplete_candidates.is_none()
    }

    /// Returns complete unpaired projections retained after incomplete replay.
    pub fn incomplete_candidates(&self) -> Option<&BezierParallelIntersectionCandidates2> {
        self.incomplete_candidates.as_deref()
    }

    /// Returns whether a complete result proves the two finite images disjoint.
    pub fn is_empty(&self) -> bool {
        self.is_complete() && self.contacts.is_empty() && self.overlaps.is_empty()
    }
}

fn parallel_contact_pair_is_retained(
    contacts: &[BezierParallelIntersectionContact2],
    pair: &BezierParallelIntersectionParameterPair2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let mut uncertain = None;
    for contact in contacts {
        let parallel_equal = contact
            .parallel_parameter
            .same_value(&pair.parallel_parameter, policy)?;
        let other_equal = contact
            .other_parameter
            .same_value(&pair.other_parameter, policy)?;
        match (parallel_equal, other_equal) {
            (Classification::Decided(true), Classification::Decided(true)) => {
                return Ok(Classification::Decided(true));
            }
            (Classification::Decided(false), _) | (_, Classification::Decided(false)) => {}
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                uncertain = Some(reason)
            }
        }
    }
    Ok(uncertain.map_or(Classification::Decided(false), Classification::Uncertain))
}

fn parallel_candidates_from_rational(
    candidates: RationalBezierIntersectionCandidates2,
) -> BezierParallelIntersectionCandidates2 {
    match candidates {
        RationalBezierIntersectionCandidates2::NoIntersection => {
            BezierParallelIntersectionCandidates2::NoIntersection
        }
        RationalBezierIntersectionCandidates2::Candidates {
            first_parameters,
            second_parameters,
        } => BezierParallelIntersectionCandidates2::Candidates {
            parallel_parameters: first_parameters,
            other_parameters: second_parameters,
        },
        RationalBezierIntersectionCandidates2::DegenerateResultant => {
            BezierParallelIntersectionCandidates2::DegenerateResultant
        }
    }
}

impl std::fmt::Debug for BezierParallel2 {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("BezierParallel2")
            .field("source", &self.data.source)
            .field("distance", &self.data.distance)
            .finish()
    }
}

impl PartialEq for BezierParallel2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.data.source == other.data.source && self.data.distance == other.data.distance)
    }
}

/// Exact rational parallel certified from a homogeneous Pythagorean hodograph.
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

    /// Returns `sigma`, where the homogeneous tangent numerator satisfies `H dot H = sigma^2`.
    pub fn speed_polynomial(&self) -> &[Real] {
        &self.speed_polynomial
    }

    /// Returns the homogeneous source degree.
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
    fn from_source(source: BezierParallelSource2, distance: Real) -> Self {
        Self {
            data: Arc::new(BezierParallelData2 {
                source,
                distance,
                polynomial_power_basis: OnceLock::new(),
                differential: OnceLock::new(),
                certified_ph_offset: OnceLock::new(),
            }),
        }
    }

    /// Returns the degree of the retained source span.
    pub fn source_degree(&self) -> usize {
        match &self.data.source {
            BezierParallelSource2::Quadratic(_) => 2,
            BezierParallelSource2::Cubic(_) => 3,
            BezierParallelSource2::Rational(source) => source.degree(),
        }
    }

    /// Returns the same exact parallel image with traversal direction reversed.
    ///
    /// Reversal flips the source tangent and therefore its left normal. The
    /// signed distance is negated so this carrier still traces the original
    /// parallel image, now from end to start.
    pub fn reversed(&self) -> Self {
        Self::from_source(self.data.source.reversed(), -self.distance().clone())
    }

    /// Splits this exact parallel at one represented interior parameter.
    ///
    /// The two returned carriers use local `[0, 1]` parameters and retain the
    /// same signed left distance. Endpoint splits are rejected as boundaries
    /// because a zero-width source span has no defined unit normal.
    pub fn split_at_exact(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(Self, Self)>> {
        match strict_interior_unit_parameter(parameter, policy) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        Ok(self
            .data
            .source
            .split_at_exact(parameter, policy)?
            .map(|(left, right)| {
                (
                    Self::from_source(left, self.distance().clone()),
                    Self::from_source(right, self.distance().clone()),
                )
            }))
    }

    /// Restricts this exact parallel to an ordered, nonempty represented range.
    ///
    /// The returned carrier is reparameterized to `[0, 1]` and retains the
    /// source orientation and signed left distance.
    pub fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if in_closed_unit_interval(start, policy) != Some(true)
            || in_closed_unit_interval(end, policy) != Some(true)
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        }
        match compare_reals(start, end, policy) {
            Some(std::cmp::Ordering::Less) => {}
            Some(std::cmp::Ordering::Equal) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Some(std::cmp::Ordering::Greater) | None => {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
        }
        Ok(self
            .data
            .source
            .subcurve_between_exact(start, end, policy)?
            .map(|source| Self::from_source(source, self.distance().clone())))
    }

    /// Returns a conservative exact box for every defined point of this parallel.
    ///
    /// A unit normal changes either source coordinate by at most `|distance|`,
    /// so expanding a certified source box by that amount is exact broad-phase
    /// evidence without sampling the parallel or materializing a finite curve.
    pub fn conservative_bounds(&self, policy: &CurveContext) -> CurveResult<Classification<Aabb2>> {
        let source = match self.data.source.certified_bounds(policy) {
            Classification::Decided(source) => source,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radius = self.distance().abs();
        Ok(Classification::Decided(Aabb2::new_unchecked(
            Point2::new(source.min_x() - &radius, source.min_y() - &radius),
            Point2::new(source.max_x() + &radius, source.max_y() + radius),
        )))
    }

    /// Returns complete exact parameter evidence where this parallel contains `point`.
    ///
    /// For a rational source `P=(X/W,Y/W)` with homogeneous tangent numerator
    /// `H`, a point `C` lies on one of the two unsigned parallels exactly when
    /// `(CW-(X,Y)) dot H = 0` and
    /// `|CW-(X,Y)|^2-d^2 W^2 = 0`. Their polynomial GCD supplies every real
    /// candidate. The sign of
    /// `(CW-(X,Y)) dot rotate90(H) * W * d` then removes the opposite normal
    /// branch without evaluating an approximate square root. Polynomial curves
    /// are the `W=1` specialization.
    pub fn point_incidence(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIncidence2>> {
        let distance_sign = match real_sign(self.distance(), policy) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let source = self.source_power_basis()?;
        let differential = self.differential()?;

        if let Classification::Uncertain(reason) = Self::certify_finite_source(&source, policy)? {
            return Ok(Classification::Uncertain(reason));
        }
        if distance_sign != RealSign::Zero
            && let Classification::Uncertain(reason) =
                Self::certify_regular_differential(differential, policy)?
        {
            return Ok(Classification::Uncertain(reason));
        }

        let weighted_target = |coordinate: &Real| match source.weight {
            Some(weight) => polynomial_scale(weight, coordinate),
            None => vec![coordinate.clone()],
        };
        let delta_x = polynomial_subtract(&weighted_target(point.x()), source.x_numerator);
        let delta_y = polynomial_subtract(&weighted_target(point.y()), source.y_numerator);
        let orthogonality = polynomial_add(
            &polynomial_multiply(&delta_x, &differential.tangent_x),
            &polynomial_multiply(&delta_y, &differential.tangent_y),
        );
        let weighted_distance = match source.weight {
            Some(weight) => polynomial_scale(weight, self.distance()),
            None => vec![self.distance().clone()],
        };
        let distance_relation = polynomial_subtract(
            &polynomial_add(
                &polynomial_multiply(&delta_x, &delta_x),
                &polynomial_multiply(&delta_y, &delta_y),
            ),
            &polynomial_multiply(&weighted_distance, &weighted_distance),
        );
        let incidence =
            match common_unit_polynomial_roots(orthogonality, distance_relation, policy)? {
                Classification::Decided(incidence) => incidence,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        if distance_sign == RealSign::Zero {
            return Ok(Classification::Decided(incidence));
        }

        let orientation = polynomial_subtract(
            &polynomial_multiply(&delta_y, &differential.tangent_x),
            &polynomial_multiply(&delta_x, &differential.tangent_y),
        );
        let orientation = match source.weight {
            Some(weight) => polynomial_multiply(&orientation, weight),
            None => orientation,
        };
        let branch = match polynomial_from_coefficients(
            polynomial_scale(&orientation, self.distance()),
            policy,
        )? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        match incidence {
            BezierParallelIncidence2::EntireCurve => {
                match branch.isolate_unit_interval_roots(policy)? {
                    Classification::Decided(roots) if roots.is_empty() => {}
                    Classification::Decided(_) => {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                match real_sign(&branch.evaluate(&Real::zero()), policy) {
                    Some(RealSign::Positive) => Ok(Classification::Decided(
                        BezierParallelIncidence2::EntireCurve,
                    )),
                    Some(RealSign::Negative) => Ok(Classification::Decided(
                        BezierParallelIncidence2::Parameters(Vec::new()),
                    )),
                    Some(RealSign::Zero) => {
                        Ok(Classification::Uncertain(UncertaintyReason::Boundary))
                    }
                    None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                }
            }
            BezierParallelIncidence2::Parameters(candidates) => {
                let mut retained = Vec::with_capacity(candidates.len());
                for candidate in candidates {
                    match signed_polynomial_at_root(Some(&branch), &candidate, policy)? {
                        Classification::Decided(RealSign::Positive) => retained.push(candidate),
                        Classification::Decided(RealSign::Negative) => {}
                        Classification::Decided(RealSign::Zero) => {
                            return Err(CurveError::Topology(
                                "parallel branch vanished at a regular nonzero-distance incidence"
                                    .to_owned(),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Ok(Classification::Decided(
                    BezierParallelIncidence2::Parameters(retained),
                ))
            }
        }
    }

    /// Classifies whether `point` belongs to this exact analytic parallel.
    pub fn contains_point(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        Ok(self
            .point_incidence(point, policy)?
            .map(|incidence| match incidence {
                BezierParallelIncidence2::EntireCurve => true,
                BezierParallelIncidence2::Parameters(parameters) => !parameters.is_empty(),
            }))
    }

    /// Returns complete exact parameters where this parallel meets a supporting line.
    ///
    /// The finite endpoints of `line` define its nonzero direction and support;
    /// they do not clip the result to the segment. For homogeneous source
    /// `(X/W,Y/W)`, signed line numerator `L`, tangent numerator `H`, and line
    /// direction `V`, candidates are the roots of
    /// `L^2(H dot H)-d^2(V dot H)^2W^2`. Exact signs of `L` and
    /// `d(V dot H)W` remove the branch introduced by squaring. A zero-distance
    /// carrier takes the direct unsquared source-line route and remains valid
    /// at source stationary parameters.
    pub fn supporting_line_incidence(
        &self,
        line: &LineSeg2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIncidence2>> {
        let distance_sign = match real_sign(self.distance(), policy) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let source = self.source_power_basis()?;
        if let Classification::Uncertain(reason) = Self::certify_finite_source(&source, policy)? {
            return Ok(Classification::Uncertain(reason));
        }
        let differential = self.differential()?;
        let weighted_start = |coordinate: &Real| match source.weight {
            Some(weight) => polynomial_scale(weight, coordinate),
            None => vec![coordinate.clone()],
        };
        let source_from_start_x =
            polynomial_subtract(source.x_numerator, &weighted_start(line.start().x()));
        let source_from_start_y =
            polynomial_subtract(source.y_numerator, &weighted_start(line.start().y()));
        let (line_x, line_y) = line.delta();
        let line_numerator = polynomial_subtract(
            &polynomial_scale(&source_from_start_y, &line_x),
            &polynomial_scale(&source_from_start_x, &line_y),
        );
        if distance_sign == RealSign::Zero {
            return common_unit_polynomial_roots(line_numerator, vec![Real::zero()], policy);
        }
        if let Classification::Uncertain(reason) =
            Self::certify_regular_differential(differential, policy)?
        {
            return Ok(Classification::Uncertain(reason));
        }

        let normal_projection = polynomial_add(
            &polynomial_scale(&differential.tangent_x, &line_x),
            &polynomial_scale(&differential.tangent_y, &line_y),
        );
        let signed_normal_term = polynomial_scale(&normal_projection, self.distance());
        let signed_normal_term = match source.weight {
            Some(weight) => polynomial_multiply(&signed_normal_term, weight),
            None => signed_normal_term,
        };
        let line_polynomial = match polynomial_from_coefficients(line_numerator.clone(), policy)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let normal_polynomial =
            match polynomial_from_coefficients(signed_normal_term.clone(), policy)? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let (line_polynomial, normal_polynomial) = match (line_polynomial, normal_polynomial) {
            (None, None) => {
                return Ok(Classification::Decided(
                    BezierParallelIncidence2::EntireCurve,
                ));
            }
            (Some(polynomial), None) | (None, Some(polynomial)) => {
                return Ok(polynomial
                    .isolate_unit_interval_roots(policy)?
                    .map(BezierParallelIncidence2::Parameters));
            }
            (Some(line_polynomial), Some(normal_polynomial)) => {
                (line_polynomial, normal_polynomial)
            }
        };
        let speed_squared = polynomial_add(
            &polynomial_multiply(&differential.tangent_x, &differential.tangent_x),
            &polynomial_multiply(&differential.tangent_y, &differential.tangent_y),
        );
        let squared_relation = polynomial_subtract(
            &polynomial_multiply(
                &polynomial_multiply(&line_numerator, &line_numerator),
                &speed_squared,
            ),
            &polynomial_multiply(&signed_normal_term, &signed_normal_term),
        );
        let incidence =
            match common_unit_polynomial_roots(squared_relation, vec![Real::zero()], policy)? {
                Classification::Decided(incidence) => incidence,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };

        match incidence {
            BezierParallelIncidence2::EntireCurve => {
                let line_sign = match polynomial_right_origin_sign(&line_polynomial, policy) {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let normal_sign = match polynomial_right_origin_sign(&normal_polynomial, policy) {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if real_signs_are_opposite(line_sign, normal_sign) {
                    return Ok(Classification::Decided(
                        BezierParallelIncidence2::EntireCurve,
                    ));
                }
                Ok(line_polynomial
                    .isolate_unit_interval_roots(policy)?
                    .map(BezierParallelIncidence2::Parameters))
            }
            BezierParallelIncidence2::Parameters(candidates) => {
                let mut retained = Vec::with_capacity(candidates.len());
                for candidate in candidates {
                    let line_sign = match signed_polynomial_at_root(
                        Some(&line_polynomial),
                        &candidate,
                        policy,
                    )? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    let normal_sign = match signed_polynomial_at_root(
                        Some(&normal_polynomial),
                        &candidate,
                        policy,
                    )? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    match (line_sign, normal_sign) {
                        (RealSign::Zero, RealSign::Zero) => retained.push(candidate),
                        (RealSign::Positive, RealSign::Negative)
                        | (RealSign::Negative, RealSign::Positive) => retained.push(candidate),
                        (RealSign::Positive, RealSign::Positive)
                        | (RealSign::Negative, RealSign::Negative) => {}
                        (RealSign::Zero, RealSign::Positive | RealSign::Negative)
                        | (RealSign::Positive | RealSign::Negative, RealSign::Zero) => {
                            return Err(CurveError::Topology(
                                "squared parallel-line candidate lost its exact mate".to_owned(),
                            ));
                        }
                    }
                }
                Ok(Classification::Decided(
                    BezierParallelIncidence2::Parameters(retained),
                ))
            }
        }
    }

    /// Constructs complete parameter projections for intersections with a rational Bezier.
    ///
    /// For target `Q(u)=A(u)/B(u)`, source `P(t)=(X(t)/W(t),Y(t)/W(t))`,
    /// homogeneous tangent numerator `H(t)`, and
    /// `Delta=(A_x W-XB,A_y W-YB)`, every unsigned parallel contact satisfies
    /// `Delta dot H=0` and `Delta dot Delta-d^2 W^2 B^2=0`. Hypersolve
    /// eliminates each parameter from that one bivariate system. The returned
    /// projections are candidate evidence: exact contact replay must still
    /// pair roots and reject the opposite normal branch introduced by the
    /// squared distance equation.
    pub fn intersection_candidates(
        &self,
        other: &RationalBezier2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIntersectionCandidates2>> {
        if let Some(Some(offset)) = self.data.certified_ph_offset.get() {
            return Ok(
                match offset
                    .curve()
                    .intersection_candidates_classified(other, policy)?
                {
                    Classification::Decided(candidates) => {
                        Classification::Decided(parallel_candidates_from_rational(candidates))
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                },
            );
        }
        Ok(self
            .intersection_candidate_system(other, policy)?
            .map(|system| {
                if system.overlaps.is_empty() {
                    system.candidates
                } else {
                    BezierParallelIntersectionCandidates2::DegenerateResultant
                }
            }))
    }

    fn intersection_candidate_system(
        &self,
        other: &RationalBezier2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIntersectionCandidateSystem2>> {
        let distance_sign = match real_sign(self.distance(), policy) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        if let Some(Some(offset)) = self.data.certified_ph_offset.get() {
            match offset
                .curve()
                .intersection_candidates_classified(other, policy)?
            {
                Classification::Decided(candidates) => {
                    let candidates = parallel_candidates_from_rational(candidates);
                    if !matches!(
                        candidates,
                        BezierParallelIntersectionCandidates2::DegenerateResultant
                    ) {
                        return Ok(Classification::Decided(
                            BezierParallelIntersectionCandidateSystem2::projected(candidates, None),
                        ));
                    }
                    if let Classification::Decided(RationalBezierIntersectionContacts2::Overlap(
                        overlap,
                    )) = offset
                        .curve()
                        .intersection_contacts_classified(other, policy)?
                    {
                        return Ok(Classification::Decided(
                            BezierParallelIntersectionCandidateSystem2::overlaps(Arc::from([
                                overlap,
                            ])),
                        ));
                    }
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let source = self.source_power_basis()?;
        if let Classification::Uncertain(reason) = Self::certify_finite_source(&source, policy)? {
            return Ok(Classification::Uncertain(reason));
        }
        let other_power = other.homogeneous_power_basis()?;
        if let Classification::Uncertain(reason) =
            Self::certify_finite_weight(Some(&other_power.weight), policy)?
        {
            return Ok(Classification::Uncertain(reason));
        }
        let differential = self.differential()?;
        if distance_sign != RealSign::Zero
            && let Classification::Uncertain(reason) =
                Self::certify_regular_differential(differential, policy)?
        {
            return Ok(Classification::Uncertain(reason));
        }

        let other_bounds = other.certified_bounds_classified(policy);
        if let (Classification::Decided(parallel_bounds), Classification::Decided(other_bounds)) =
            (self.conservative_bounds(policy)?, other_bounds)
            && matches!(
                parallel_bounds.overlaps(&other_bounds, policy),
                Classification::Decided(false)
            )
        {
            return Ok(Classification::Decided(
                BezierParallelIntersectionCandidateSystem2::projected(
                    BezierParallelIntersectionCandidates2::NoIntersection,
                    None,
                ),
            ));
        }

        let (orthogonality, distance_relation) = parallel_rational_intersection_equations(
            &source,
            differential,
            self.distance(),
            other_power,
        );
        let equations = [orthogonality, distance_relation];
        if other.degree() >= 4
            && distance_sign != RealSign::Zero
            && bivariate_system_may_have_component(&equations)
        {
            let reduced = rootless_axis_primitive_system(&equations, policy)?;
            let component_equations = reduced.as_ref().unwrap_or(&equations);
            if bivariate_system_may_have_component(component_equations) {
                if let Classification::Decided(Some(exact_parallel)) =
                    self.exact_rational_parallel_component(policy)?
                    && let Classification::Decided(RationalBezierIntersectionContacts2::Overlap(
                        overlap,
                    )) = exact_parallel.intersection_contacts_classified(other, policy)?
                {
                    return Ok(Classification::Decided(
                        BezierParallelIntersectionCandidateSystem2::overlaps(Arc::from([overlap])),
                    ));
                }
                let branch = parallel_rational_selected_branch(
                    &source,
                    differential,
                    self.distance(),
                    other_power,
                );
                let config = CurveIntersectionResultantConfig {
                    min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
                    max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
                };
                if let Classification::Decided(Some(component)) =
                    parameter_component_system(component_equations, &branch, policy, config)?
                {
                    return parallel_candidate_system_from_parameter_components(component, policy);
                }
            }
        }
        let candidates =
            match project_parallel_intersection_system(&equations[0], &equations[1], policy)? {
                Classification::Decided(candidates) => candidates,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        if matches!(
            candidates,
            BezierParallelIntersectionCandidates2::DegenerateResultant
        ) {
            match self.exact_rational_parallel_component(policy)? {
                Classification::Decided(Some(exact_parallel)) => {
                    match exact_parallel.intersection_candidates_classified(other, policy)? {
                        Classification::Decided(candidates) => {
                            let candidates = parallel_candidates_from_rational(candidates);
                            if !matches!(
                                candidates,
                                BezierParallelIntersectionCandidates2::DegenerateResultant
                            ) {
                                return Ok(Classification::Decided(
                                    BezierParallelIntersectionCandidateSystem2::projected(
                                        candidates, None,
                                    ),
                                ));
                            }
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let candidate_system =
            match parallel_intersection_candidate_system(equations, candidates, policy)? {
                Classification::Decided(candidate_system) => candidate_system,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        if matches!(
            candidate_system.candidates,
            BezierParallelIntersectionCandidates2::DegenerateResultant
        ) && distance_sign != RealSign::Zero
            && let Some(component_equations) = candidate_system.replay_equations.as_ref()
        {
            let branch = parallel_rational_selected_branch(
                &source,
                differential,
                self.distance(),
                other_power,
            );
            let config = CurveIntersectionResultantConfig {
                min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
                max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
            };
            if let Classification::Decided(Some(component)) =
                parameter_component_system(component_equations, &branch, policy, config)?
            {
                return parallel_candidate_system_from_parameter_components(component, policy);
            }
        }
        Ok(Classification::Decided(candidate_system))
    }

    /// Replays every resultant candidate into exact selected-branch contacts.
    ///
    /// Directly represented pairs and pairs with one isolated algebraic
    /// parameter are substituted into both original equations exactly. Pairs
    /// of algebraic parameters use univariate/identity/reversal fast paths,
    /// followed by Hypersolve's exact nullity-one Sylvester lift or one
    /// specialization-first common-fiber GCD and local-field Sturm count for
    /// genuinely coupled systems. Even-multiplicity roots and specialized
    /// degree drops are retained exactly. A degenerate resultant first
    /// delegates to the rational shared-component authority whenever zero
    /// distance or a certified Pythagorean hodograph supplies an exact
    /// rational parallel. Otherwise a primitive first subresultant may expose
    /// every extractable parameter component; rational maps retain their
    /// partitioned fast path, while one regular implicit graph is accepted only
    /// after exact closed-domain, critical-point, monotonicity, and selected-
    /// branch certification. Both authored residual equations remain in the
    /// same recursive engine. Unsupported coefficient towers and multi-cell or
    /// singular implicit topology remain explicit
    /// [`BezierParallelIntersectionSet2::incomplete_candidates`] evidence; no
    /// projected root is promoted without exact replay.
    pub fn intersections(
        &self,
        other: &RationalBezier2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIntersectionSet2>> {
        let candidate_system = match self.intersection_candidate_system(other, policy)? {
            Classification::Decided(candidate_system) => candidate_system,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let BezierParallelIntersectionCandidateSystem2 {
            candidates,
            replay_equations,
            overlaps,
            component_pairs,
        } = candidate_system;
        let residual_degenerate = matches!(
            candidates,
            BezierParallelIntersectionCandidates2::DegenerateResultant
        );
        let empty_parameters: &[BezierParameter2] = &[];
        let (parallel_parameters, other_parameters) = match &candidates {
            BezierParallelIntersectionCandidates2::NoIntersection => {
                if component_pairs.is_empty() {
                    return Ok(Classification::Decided(
                        BezierParallelIntersectionSet2::complete(Arc::from([]), overlaps),
                    ));
                }
                (empty_parameters, empty_parameters)
            }
            BezierParallelIntersectionCandidates2::DegenerateResultant => {
                if component_pairs.is_empty() {
                    if !overlaps.is_empty() {
                        return Ok(Classification::Decided(
                            BezierParallelIntersectionSet2::incomplete(
                                Arc::from([]),
                                overlaps,
                                BezierParallelIntersectionCandidates2::DegenerateResultant,
                            ),
                        ));
                    }
                    return self.replay_degenerate_component(other, policy);
                }
                (empty_parameters, empty_parameters)
            }
            BezierParallelIntersectionCandidates2::Candidates {
                parallel_parameters,
                other_parameters,
            } => (parallel_parameters.as_slice(), other_parameters.as_slice()),
        };

        let source = self.source_power_basis()?;
        let differential = self.differential()?;
        let other_power = other.homogeneous_power_basis()?;
        let [orthogonality, distance_relation] = replay_equations.unwrap_or_else(|| {
            let (orthogonality, distance_relation) = parallel_rational_intersection_equations(
                &source,
                differential,
                self.distance(),
                other_power,
            );
            [orthogonality, distance_relation]
        });
        let distance_sign = match real_sign(self.distance(), policy) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let branch = (distance_sign != RealSign::Zero).then(|| {
            parallel_rational_selected_branch(&source, differential, self.distance(), other_power)
        });

        let mut contacts = Vec::new();
        let mut incomplete = residual_degenerate;
        let mut parameter_lifts: [Option<CurveIntersectionParameterLiftReport>; 2] = [None, None];
        let lift_config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };
        for parallel_parameter in parallel_parameters {
            for other_parameter in other_parameters {
                let replay = match replay_bivariate_parameter_pair(
                    &orthogonality,
                    &distance_relation,
                    parallel_parameter,
                    other_parameter,
                    policy,
                    lift_config,
                    &mut parameter_lifts,
                )? {
                    Classification::Decided(replay) => replay,
                    Classification::Uncertain(_) => {
                        incomplete = true;
                        continue;
                    }
                };
                if replay == BivariateParameterPairReplay::Rejected {
                    continue;
                }
                if let Some(branch) = branch.as_ref() {
                    let sign = match replay {
                        BivariateParameterPairReplay::Rejected => {
                            unreachable!("rejected parameter pairs continue above")
                        }
                        BivariateParameterPairReplay::Direct => signed_bivariate_at_parameter_pair(
                            branch,
                            parallel_parameter,
                            other_parameter,
                            policy,
                        )?,
                        BivariateParameterPairReplay::LinearLift(axis, map_index) => {
                            let (report_index, retained_parameter) = match axis {
                                CurveResultantParameter::First => (0, parallel_parameter),
                                CurveResultantParameter::Second => (1, other_parameter),
                            };
                            signed_bivariate_on_parameter_lift(
                                branch,
                                retained_parameter,
                                axis,
                                &parameter_lifts[report_index]
                                    .as_ref()
                                    .expect("a lifted replay retains its report")
                                    .maps[map_index],
                                policy,
                            )?
                        }
                    };
                    match sign {
                        Classification::Decided(RealSign::Positive) => {}
                        Classification::Decided(RealSign::Negative) => continue,
                        Classification::Decided(RealSign::Zero) => {
                            return Err(CurveError::Topology(
                                "parallel branch vanished at a regular nonzero-distance contact"
                                    .to_owned(),
                            ));
                        }
                        Classification::Uncertain(_) => {
                            incomplete = true;
                            continue;
                        }
                    }
                }
                let Some(point) = crate::rational_bezier_general::exact_contact_point_evidence(
                    other,
                    other_parameter,
                    policy,
                )?
                else {
                    incomplete = true;
                    continue;
                };
                contacts.push(BezierParallelIntersectionContact2 {
                    parallel_parameter: parallel_parameter.clone(),
                    other_parameter: other_parameter.clone(),
                    point,
                    certified_transverse: self.certified_transverse_contact(
                        other,
                        parallel_parameter,
                        other_parameter,
                        policy,
                    ),
                });
            }
        }
        for pair in component_pairs.iter() {
            match parallel_contact_pair_is_retained(&contacts, pair, policy)? {
                Classification::Decided(true) => continue,
                Classification::Decided(false) => {}
                Classification::Uncertain(_) => {
                    incomplete = true;
                    continue;
                }
            }
            let Some(point) = crate::rational_bezier_general::exact_contact_point_evidence(
                other,
                &pair.other_parameter,
                policy,
            )?
            else {
                incomplete = true;
                continue;
            };
            contacts.push(BezierParallelIntersectionContact2 {
                parallel_parameter: pair.parallel_parameter.clone(),
                other_parameter: pair.other_parameter.clone(),
                point,
                certified_transverse: self.certified_transverse_contact(
                    other,
                    &pair.parallel_parameter,
                    &pair.other_parameter,
                    policy,
                ),
            });
        }
        let contacts: Arc<[BezierParallelIntersectionContact2]> = contacts.into();
        if incomplete {
            return Ok(Classification::Decided(
                BezierParallelIntersectionSet2::incomplete(contacts, overlaps, candidates),
            ));
        }
        Ok(Classification::Decided(
            BezierParallelIntersectionSet2::complete(contacts, overlaps),
        ))
    }

    fn replay_degenerate_component(
        &self,
        other: &RationalBezier2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParallelIntersectionSet2>> {
        let exact_parallel = match self.exact_rational_parallel_component(policy)? {
            Classification::Decided(Some(exact_parallel)) => Some(exact_parallel),
            Classification::Decided(None) => None,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let Some(exact_parallel) = exact_parallel else {
            return Ok(Classification::Decided(
                BezierParallelIntersectionSet2::incomplete(
                    Arc::from([]),
                    Arc::from([]),
                    BezierParallelIntersectionCandidates2::DegenerateResultant,
                ),
            ));
        };
        let replay = match exact_parallel.intersection_contacts_classified(other, policy)? {
            Classification::Decided(replay) => replay,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let map_contacts = |contacts: &[crate::RationalBezierIntersectionContact2]| {
            contacts
                .iter()
                .map(|contact| BezierParallelIntersectionContact2 {
                    parallel_parameter: contact.first_parameter().clone(),
                    other_parameter: contact.second_parameter().clone(),
                    point: contact.point().clone(),
                    certified_transverse: contact.is_certified_transverse(),
                })
                .collect::<Arc<[_]>>()
        };
        Ok(Classification::Decided(match replay {
            RationalBezierIntersectionContacts2::NoIntersection => {
                BezierParallelIntersectionSet2::complete(Arc::from([]), Arc::from([]))
            }
            RationalBezierIntersectionContacts2::Contacts(contacts) => {
                BezierParallelIntersectionSet2::complete(map_contacts(&contacts), Arc::from([]))
            }
            RationalBezierIntersectionContacts2::Overlap(overlap) => {
                BezierParallelIntersectionSet2::complete(Arc::from([]), Arc::from([overlap]))
            }
            RationalBezierIntersectionContacts2::Incomplete { contacts, .. } => {
                BezierParallelIntersectionSet2::incomplete(
                    map_contacts(&contacts),
                    Arc::from([]),
                    BezierParallelIntersectionCandidates2::DegenerateResultant,
                )
            }
            RationalBezierIntersectionContacts2::DegenerateResultant => {
                BezierParallelIntersectionSet2::incomplete(
                    Arc::from([]),
                    Arc::from([]),
                    BezierParallelIntersectionCandidates2::DegenerateResultant,
                )
            }
        }))
    }

    fn exact_rational_parallel_component(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RationalBezier2>>> {
        match real_sign(self.distance(), policy) {
            Some(RealSign::Zero) => Ok(Classification::Decided(Some(
                self.data.source.to_rational_bezier()?,
            ))),
            Some(RealSign::Positive | RealSign::Negative) => {
                if let Some(cached) = self.data.certified_ph_offset.get() {
                    return Ok(Classification::Decided(
                        cached.as_deref().map(|offset| offset.curve().clone()),
                    ));
                }
                Ok(Classification::Decided(
                    match self.exact_pythagorean_hodograph_offset(policy)? {
                        Classification::Decided(Some(offset)) => Some(offset.curve().clone()),
                        Classification::Decided(None) | Classification::Uncertain(_) => None,
                    },
                ))
            }
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }

    fn certified_transverse_contact(
        &self,
        other: &RationalBezier2,
        parallel_parameter: &BezierParameter2,
        other_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> bool {
        let (Some(parallel_parameter), Some(other_parameter)) =
            (parallel_parameter.as_exact(), other_parameter.as_exact())
        else {
            return false;
        };
        let Ok(Classification::Decided(parallel_derivative)) =
            self.derivative_at(parallel_parameter, policy)
        else {
            return false;
        };
        let Classification::Decided(other_derivatives) =
            other.derivatives_at_classified(other_parameter, 1, policy)
        else {
            return false;
        };
        let Some(other_derivative) = other_derivatives.first() else {
            return false;
        };
        !matches!(
            real_sign(
                &(parallel_derivative.dx() * other_derivative.dy()
                    - parallel_derivative.dy() * other_derivative.dx()),
                policy,
            ),
            Some(RealSign::Zero) | None
        )
    }

    fn polynomial_power_basis(&self) -> CurveResult<&(Vec<Real>, Vec<Real>)> {
        if let Some(power_basis) = self.data.polynomial_power_basis.get() {
            return Ok(power_basis);
        }
        let power_basis = match &self.data.source {
            BezierParallelSource2::Quadratic(source) => {
                polynomial_control_power_basis(&source.control_points())?
            }
            BezierParallelSource2::Cubic(source) => {
                polynomial_control_power_basis(&source.control_points())?
            }
            BezierParallelSource2::Rational(_) => {
                return Err(CurveError::Topology(
                    "rational parallel requested a polynomial source basis".to_owned(),
                ));
            }
        };
        let _ = self.data.polynomial_power_basis.set(power_basis);
        Ok(self
            .data
            .polynomial_power_basis
            .get()
            .expect("parallel polynomial basis was initialized"))
    }

    fn source_power_basis(&self) -> CurveResult<BezierParallelPowerBasisRef<'_>> {
        match &self.data.source {
            BezierParallelSource2::Quadratic(_) | BezierParallelSource2::Cubic(_) => {
                let (x_numerator, y_numerator) = self.polynomial_power_basis()?;
                Ok(BezierParallelPowerBasisRef {
                    x_numerator,
                    y_numerator,
                    weight: None,
                })
            }
            BezierParallelSource2::Rational(source) => {
                let source = source.homogeneous_power_basis()?;
                Ok(BezierParallelPowerBasisRef {
                    x_numerator: &source.x_numerator,
                    y_numerator: &source.y_numerator,
                    weight: Some(&source.weight),
                })
            }
        }
    }

    fn certify_finite_source(
        source: &BezierParallelPowerBasisRef<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<()>> {
        Self::certify_finite_weight(source.weight, policy)
    }

    fn certify_finite_weight(
        weight: Option<&[Real]>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<()>> {
        let Some(weight) = weight else {
            return Ok(Classification::Decided(()));
        };
        let weight_polynomial = match polynomial_from_coefficients(weight.to_vec(), policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(
            match weight_polynomial.isolate_unit_interval_roots(policy)? {
                Classification::Decided(roots) if roots.is_empty() => Classification::Decided(()),
                Classification::Decided(_) => {
                    Classification::Uncertain(UncertaintyReason::Boundary)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    fn certify_regular_differential(
        differential: &BezierParallelDifferential2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<()>> {
        let speed_squared = polynomial_add(
            &polynomial_multiply(&differential.tangent_x, &differential.tangent_x),
            &polynomial_multiply(&differential.tangent_y, &differential.tangent_y),
        );
        let speed_polynomial = match polynomial_from_coefficients(speed_squared, policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match speed_polynomial.isolate_unit_interval_roots(policy)? {
            Classification::Decided(roots) if roots.is_empty() => {}
            Classification::Decided(_) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        match real_sign(&speed_polynomial.evaluate(&Real::zero()), policy) {
            Some(RealSign::Positive) => Ok(Classification::Decided(())),
            Some(RealSign::Zero) => Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            Some(RealSign::Negative) => Err(CurveError::Topology(
                "Bezier tangent squared norm was certified negative".to_owned(),
            )),
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }

    fn differential(&self) -> CurveResult<&BezierParallelDifferential2> {
        if let Some(differential) = self.data.differential.get() {
            return Ok(differential);
        }
        let (tangent_x, tangent_y) = match &self.data.source {
            BezierParallelSource2::Quadratic(_) | BezierParallelSource2::Cubic(_) => {
                let (source_x, source_y) = self.polynomial_power_basis()?;
                (
                    polynomial_derivative(source_x),
                    polynomial_derivative(source_y),
                )
            }
            BezierParallelSource2::Rational(source) => {
                let source = source.homogeneous_power_basis()?;
                let x_numerator = polynomial_trim_structural_zeros(source.x_numerator.clone());
                let y_numerator = polynomial_trim_structural_zeros(source.y_numerator.clone());
                let weight = polynomial_trim_structural_zeros(source.weight.clone());
                let x_derivative = polynomial_derivative(&x_numerator);
                let y_derivative = polynomial_derivative(&y_numerator);
                let weight_derivative = polynomial_derivative(&weight);
                (
                    polynomial_subtract(
                        &polynomial_multiply(&x_derivative, &weight),
                        &polynomial_multiply(&x_numerator, &weight_derivative),
                    ),
                    polynomial_subtract(
                        &polynomial_multiply(&y_derivative, &weight),
                        &polynomial_multiply(&y_numerator, &weight_derivative),
                    ),
                )
            }
        };
        let differential = BezierParallelDifferential2 {
            tangent_derivative_x: polynomial_derivative(&tangent_x),
            tangent_derivative_y: polynomial_derivative(&tangent_y),
            tangent_x,
            tangent_y,
        };
        let _ = self.data.differential.set(differential);
        Ok(self
            .data
            .differential
            .get()
            .expect("parallel differential was initialized"))
    }

    /// Returns the signed distance measured along the source's left normal.
    pub fn distance(&self) -> &Real {
        &self.data.distance
    }

    fn rational_source(&self) -> Option<&RationalBezier2> {
        match &self.data.source {
            BezierParallelSource2::Rational(source) => Some(source),
            BezierParallelSource2::Quadratic(_) | BezierParallelSource2::Cubic(_) => None,
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
        if real_sign(self.distance(), policy) == Some(RealSign::Zero) {
            return Ok(match &self.data.source {
                BezierParallelSource2::Quadratic(source) => {
                    Classification::Decided(source.point_at(parameter.clone()))
                }
                BezierParallelSource2::Cubic(source) => {
                    Classification::Decided(source.point_at(parameter.clone()))
                }
                BezierParallelSource2::Rational(source) => {
                    source.point_at_classified(parameter, policy)
                }
            });
        }
        let differential = self.differential()?;
        let source_point = if let Some(source) = self.rational_source() {
            let source = source.homogeneous_power_basis()?;
            let weight = polynomial_evaluate(&source.weight, parameter);
            match real_sign(&weight, policy) {
                Some(RealSign::Positive | RealSign::Negative) => {}
                Some(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            Point2::new(
                (polynomial_evaluate(&source.x_numerator, parameter) / &weight)?,
                (polynomial_evaluate(&source.y_numerator, parameter) / weight)?,
            )
        } else {
            let (source_x, source_y) = self.polynomial_power_basis()?;
            Point2::new(
                polynomial_evaluate(source_x, parameter),
                polynomial_evaluate(source_y, parameter),
            )
        };
        let tangent_x = polynomial_evaluate(&differential.tangent_x, parameter);
        let tangent_y = polynomial_evaluate(&differential.tangent_y, parameter);
        let speed_squared = &tangent_x * &tangent_x + &tangent_y * &tangent_y;
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
        let normal_x = ((Real::zero() - &tangent_y) / &speed)?;
        let normal_y = (tangent_x / &speed)?;
        Ok(Classification::Decided(source_point.translated(
            self.distance() * normal_x,
            self.distance() * normal_y,
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
        if real_sign(self.distance(), policy) == Some(RealSign::Zero) {
            return match &self.data.source {
                BezierParallelSource2::Quadratic(_) | BezierParallelSource2::Cubic(_) => {
                    let differential = self.differential()?;
                    Ok(Classification::Decided(CurveDerivative2::new(
                        polynomial_evaluate(&differential.tangent_x, parameter),
                        polynomial_evaluate(&differential.tangent_y, parameter),
                    )))
                }
                BezierParallelSource2::Rational(source) => {
                    Ok(source.derivative_at_classified(parameter, policy))
                }
            };
        }
        let differential = self.differential()?;
        let weight = if let Some(source) = self.rational_source() {
            let source = source.homogeneous_power_basis()?;
            let weight = polynomial_evaluate(&source.weight, parameter);
            match real_sign(&weight, policy) {
                Some(RealSign::Positive | RealSign::Negative) => {}
                Some(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            Some(weight)
        } else {
            None
        };
        let tangent_x = polynomial_evaluate(&differential.tangent_x, parameter);
        let tangent_y = polynomial_evaluate(&differential.tangent_y, parameter);
        let tangent_derivative_x =
            polynomial_evaluate(&differential.tangent_derivative_x, parameter);
        let tangent_derivative_y =
            polynomial_evaluate(&differential.tangent_derivative_y, parameter);
        let speed_squared = &tangent_x * &tangent_x + &tangent_y * &tangent_y;
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
        let tangent_dot_derivative =
            &tangent_x * &tangent_derivative_x + &tangent_y * &tangent_derivative_y;
        let normal_derivative_x = ((Real::zero() - &tangent_derivative_y) / &speed)?
            + ((&tangent_y * &tangent_dot_derivative) / &speed_cubed)?;
        let normal_derivative_y = (tangent_derivative_x / &speed)?
            - ((&tangent_x * tangent_dot_derivative) / speed_cubed)?;
        let (source_derivative_x, source_derivative_y) = if let Some(weight) = weight {
            let weight_squared = &weight * &weight;
            (
                (&tangent_x / &weight_squared)?,
                (&tangent_y / weight_squared)?,
            )
        } else {
            (tangent_x.clone(), tangent_y.clone())
        };
        Ok(Classification::Decided(CurveDerivative2::new(
            source_derivative_x + self.distance() * normal_derivative_x,
            source_derivative_y + self.distance() * normal_derivative_y,
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
        let source = match self.rational_source() {
            Some(source) => Some(source.homogeneous_power_basis()?),
            None => None,
        };
        let differential = self.differential()?;
        let weight = if let Some(source) = source {
            let weight = polynomial_trim_structural_zeros(source.weight.clone());
            let weight_polynomial = match polynomial_from_coefficients(weight.clone(), policy)? {
                Classification::Decided(Some(polynomial)) => polynomial,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match weight_polynomial.isolate_unit_interval_roots(policy)? {
                Classification::Decided(roots) if roots.is_empty() => {}
                Classification::Decided(_) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            Some(weight)
        } else {
            None
        };
        let speed_squared = polynomial_add(
            &polynomial_multiply(&differential.tangent_x, &differential.tangent_x),
            &polynomial_multiply(&differential.tangent_y, &differential.tangent_y),
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
        if real_sign(self.distance(), policy) == Some(RealSign::Zero) {
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
            &polynomial_multiply(&differential.tangent_derivative_x, &differential.tangent_y),
            &polynomial_multiply(&differential.tangent_derivative_y, &differential.tangent_x),
        );
        let signed_curvature_term = if let Some(weight) = &weight {
            polynomial_scale(
                &polynomial_multiply(&polynomial_multiply(weight, weight), &curvature_cross),
                self.distance(),
            )
        } else {
            polynomial_scale(&curvature_cross, self.distance())
        };
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

    /// Materializes this parallel exactly when the homogeneous hodograph is Pythagorean.
    ///
    /// For a rational source `(X/W, Y/W)`, let
    /// `H = (X'W-XW', Y'W-YW')`. If `H dot H = sigma^2` for a polynomial
    /// `sigma` with certified nonzero sign over `[0, 1]`, the unit normal is
    /// rational and the complete parallel is converted to an arbitrary-degree
    /// [`RationalBezier2`]. Polynomial PH curves are the `W=1` specialization.
    /// `None` means the exact polynomial-square identity was disproved;
    /// unresolved scalar signs remain explicit [`Classification::Uncertain`].
    pub fn exact_pythagorean_hodograph_offset(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CertifiedPythagoreanHodographOffset2>>> {
        if let Some(cached) = self.data.certified_ph_offset.get() {
            return Ok(Classification::Decided(cached.as_deref().cloned()));
        }
        if policy.permits_approximate_512() {
            match self.compute_pythagorean_hodograph_offset(&CurveContext::STRICT)? {
                Classification::Decided(offset) => {
                    return Ok(Classification::Decided(
                        self.retain_certified_ph_offset(offset),
                    ));
                }
                Classification::Uncertain(_) => {
                    return self.compute_pythagorean_hodograph_offset(policy);
                }
            }
        }
        match self.compute_pythagorean_hodograph_offset(policy)? {
            Classification::Decided(offset) => Ok(Classification::Decided(
                self.retain_certified_ph_offset(offset),
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    fn retain_certified_ph_offset(
        &self,
        offset: Option<CertifiedPythagoreanHodographOffset2>,
    ) -> Option<CertifiedPythagoreanHodographOffset2> {
        let _ = self.data.certified_ph_offset.set(offset.map(Arc::new));
        self.data
            .certified_ph_offset
            .get()
            .expect("a certified PH result was retained")
            .as_deref()
            .cloned()
    }

    #[cold]
    fn compute_pythagorean_hodograph_offset(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CertifiedPythagoreanHodographOffset2>>> {
        let rational_source = self.rational_source();
        let rational_power_basis = match rational_source {
            Some(source) => Some(source.homogeneous_power_basis()?),
            None => None,
        };
        let polynomial_power_basis = if rational_source.is_none() {
            Some(self.polynomial_power_basis()?)
        } else {
            None
        };
        let (source_x, source_y) = match (rational_power_basis, polynomial_power_basis) {
            (Some(source), None) => (&source.x_numerator, &source.y_numerator),
            (None, Some((source_x, source_y))) => (source_x, source_y),
            _ => unreachable!("parallel source has exactly one power basis"),
        };
        let differential = self.differential()?;
        let weight = if let Some(source) = rational_power_basis {
            let weight = polynomial_trim_structural_zeros(source.weight.clone());
            let weight_polynomial = match polynomial_from_coefficients(weight.clone(), policy)? {
                Classification::Decided(Some(polynomial)) => polynomial,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match weight_polynomial.isolate_unit_interval_roots(policy)? {
                Classification::Decided(roots) if roots.is_empty() => {}
                Classification::Decided(_) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            Some(weight)
        } else {
            None
        };
        let speed_squared = polynomial_add(
            &polynomial_multiply(&differential.tangent_x, &differential.tangent_x),
            &polynomial_multiply(&differential.tangent_y, &differential.tangent_y),
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

        if let Some(source) = rational_source
            && let Classification::Decided(Some(arc)) =
                crate::arc_bezier::rational_bezier_circular_arc(source, policy)?
        {
            let radius_scale = arc.left_offset_radius_scale(self.distance())?;
            let controls = source
                .control_points()
                .iter()
                .map(|point| crate::offset::scale_from_center(point, arc.center(), &radius_scale))
                .collect();
            let weights = source.weights().to_vec();
            let curve = if matches!(
                real_sign(&radius_scale, &CurveContext::STRICT),
                Some(RealSign::Positive | RealSign::Negative)
            ) {
                let two = Real::from(2_i8);
                let radius_squared = arc.radius_squared_ref() * &radius_scale * &radius_scale;
                let implicit = Arc::new([
                    Real::one(),
                    Real::zero(),
                    Real::one(),
                    -(&two * arc.center().x()),
                    -(&two * arc.center().y()),
                    arc.center().x() * arc.center().x() + arc.center().y() * arc.center().y()
                        - &radius_squared,
                ]);
                let circle = Arc::new(crate::rational_bezier::RationalQuadraticCircle2 {
                    center: arc.center().clone(),
                    radius_squared,
                });
                RationalBezier2::try_new_with_implicit_quadratic_conic(
                    controls,
                    weights,
                    implicit,
                    Some(circle),
                )?
            } else {
                RationalBezier2::try_new(controls, weights)?
            };
            let source_degree = self.source_degree();
            return Ok(Classification::Decided(Some(
                CertifiedPythagoreanHodographOffset2 {
                    curve,
                    speed_polynomial: speed,
                    source_degree,
                    rational_degree: source_degree,
                    distance: self.distance().clone(),
                },
            )));
        }

        let (normal_x_term, normal_y_term, mut denominator) = if let Some(weight) = &weight {
            let weighted_distance = polynomial_scale(weight, self.distance());
            (
                polynomial_multiply(&weighted_distance, &differential.tangent_y),
                polynomial_multiply(&weighted_distance, &differential.tangent_x),
                polynomial_multiply(weight, &speed),
            )
        } else {
            (
                polynomial_scale(&differential.tangent_y, self.distance()),
                polynomial_scale(&differential.tangent_x, self.distance()),
                speed.clone(),
            )
        };
        let mut numerator_x =
            polynomial_subtract(&polynomial_multiply(source_x, &speed), &normal_x_term);
        let mut numerator_y =
            polynomial_add(&polynomial_multiply(source_y, &speed), &normal_y_term);
        if weight.is_some() {
            match real_sign(&polynomial_evaluate(&denominator, &Real::zero()), policy) {
                Some(RealSign::Positive) => {}
                Some(RealSign::Negative) => {
                    let negative_one = Real::from(-1_i8);
                    numerator_x = polynomial_scale(&numerator_x, &negative_one);
                    numerator_y = polynomial_scale(&numerator_y, &negative_one);
                    denominator = polynomial_scale(&denominator, &negative_one);
                }
                Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        let source_degree = self.source_degree();
        let base_degree = numerator_x
            .len()
            .max(numerator_y.len())
            .max(denominator.len())
            .saturating_sub(1);
        let mut rational_degree = base_degree;
        let mut weights = power_to_bernstein_coefficients(&denominator, rational_degree)?;
        // A polynomial certified strictly positive on the compact unit interval
        // has all-positive Bernstein coefficients after finitely many degree
        // elevations. Keep that exact guarantee instead of imposing a search cap.
        loop {
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
                let Some(next_degree) = rational_degree.checked_add(1) else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                };
                weights = elevate_scalar_bernstein_once(&weights)?;
                rational_degree = next_degree;
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
                    distance: self.distance().clone(),
                },
            )));
        }
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
            distance: self.distance().clone(),
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
        let source = match &self.data.source {
            BezierParallelSource2::Quadratic(source) => {
                PolynomialBezierNode2::Quadratic(source.clone())
            }
            BezierParallelSource2::Cubic(source) => PolynomialBezierNode2::Cubic(source.clone()),
            BezierParallelSource2::Rational(_) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        };
        let candidate_node = PolynomialBezierNode2::from_candidate(&candidate);
        let mut trace = ParallelVerificationTrace::default();
        match verify_parallel_node(
            source,
            candidate_node,
            self.distance(),
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
                    distance: self.distance().clone(),
                },
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
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
        Ok(BezierParallel2::from_source(
            BezierParallelSource2::Quadratic(self.clone()),
            distance,
        ))
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
        Ok(BezierParallel2::from_source(
            BezierParallelSource2::Cubic(self.clone()),
            distance,
        ))
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
    /// Retains this rational quadratic's exact analytic left parallel.
    pub fn parallel_left(&self, distance: Real) -> CurveResult<BezierParallel2> {
        Ok(BezierParallel2::from_source(
            BezierParallelSource2::Rational(RationalBezier2::from(self.clone())),
            distance,
        ))
    }

    /// Retains this rational quadratic's exact analytic right parallel.
    pub fn parallel_right(&self, distance: Real) -> CurveResult<BezierParallel2> {
        self.parallel_left(-distance)
    }

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

impl RationalBezier2 {
    /// Retains this arbitrary-degree rational Bezier's exact analytic left parallel.
    pub fn parallel_left(&self, distance: Real) -> CurveResult<BezierParallel2> {
        Ok(BezierParallel2::from_source(
            BezierParallelSource2::Rational(self.clone()),
            distance,
        ))
    }

    /// Retains this arbitrary-degree rational Bezier's exact analytic right parallel.
    pub fn parallel_right(&self, distance: Real) -> CurveResult<BezierParallel2> {
        self.parallel_left(-distance)
    }
}

impl CurvePath2 {
    /// Constructs a connected certified left parallel for supported smooth paths.
    ///
    /// Native lines/arcs and polynomial or rational PH offsets remain exact.
    /// General quadratic/cubic spans use the adaptive Blend2D construction
    /// followed by the conservative verifier. If adjacent primitive parallels do not meet
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
                CurveGeometry2::RationalQuadraticBezier(curve) => {
                    match append_exact_rational_parallel(
                        curve
                            .parallel_left(distance.clone())
                            .map_err(|cause| parallel_path_error(source, cause))?,
                        policy,
                        &mut output,
                    )
                    .map_err(|cause| parallel_path_error(source, cause))?
                    {
                        Classification::Decided(()) => exact_source_curve_count += 1,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                CurveGeometry2::RationalBezier(curve) => {
                    match append_exact_rational_parallel(
                        curve
                            .parallel_left(distance.clone())
                            .map_err(|cause| parallel_path_error(source, cause))?,
                        policy,
                        &mut output,
                    )
                    .map_err(|cause| parallel_path_error(source, cause))?
                    {
                        Classification::Decided(()) => exact_source_curve_count += 1,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                CurveGeometry2::PolynomialBSpline(_) | CurveGeometry2::Nurbs(_) => {
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

fn append_exact_rational_parallel(
    parallel: BezierParallel2,
    policy: &CurveContext,
    output: &mut Vec<Curve2>,
) -> CurveResult<Classification<()>> {
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
            Ok(Classification::Decided(()))
        }
        Classification::Decided(None) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
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

    fn exact_pythagorean_hodograph_offset(
        &self,
        distance: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CertifiedPythagoreanHodographOffset2>>> {
        self.parallel_left(distance)?
            .exact_pythagorean_hodograph_offset(policy)
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

fn polynomial_control_power_basis(controls: &[&Point2]) -> CurveResult<(Vec<Real>, Vec<Real>)> {
    Ok((
        bernstein_to_power_coefficients(controls.iter().map(|point| point.x().clone()).collect())?,
        bernstein_to_power_coefficients(controls.iter().map(|point| point.y().clone()).collect())?,
    ))
}

fn strict_interior_unit_parameter(parameter: &Real, policy: &CurveContext) -> Classification<()> {
    if in_closed_unit_interval(parameter, policy) != Some(true) {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    }
    match (
        compare_reals(parameter, &Real::zero(), policy),
        compare_reals(parameter, &Real::one(), policy),
    ) {
        (Some(std::cmp::Ordering::Greater), Some(std::cmp::Ordering::Less)) => {
            Classification::Decided(())
        }
        (Some(std::cmp::Ordering::Equal), _) | (_, Some(std::cmp::Ordering::Equal)) => {
            Classification::Uncertain(UncertaintyReason::Boundary)
        }
        (Some(_), Some(_)) => Classification::Uncertain(UncertaintyReason::Ordering),
        _ => Classification::Uncertain(UncertaintyReason::Ordering),
    }
}

fn elevate_scalar_bernstein_once(controls: &[Real]) -> CurveResult<Vec<Real>> {
    let degree = controls.len().saturating_sub(1);
    let next_degree = degree + 1;
    let denominator = Real::from(next_degree as u64);
    let mut elevated = Vec::with_capacity(next_degree + 1);
    elevated.push(controls[0].clone());
    for index in 1..next_degree {
        let left_weight = (Real::from(index as u64) / &denominator)?;
        elevated.push(
            &controls[index - 1] * &left_weight + &controls[index] * (Real::one() - left_weight),
        );
    }
    elevated.push(controls[degree].clone());
    Ok(elevated)
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

fn polynomial_trim_structural_zeros(mut coefficients: Vec<Real>) -> Vec<Real> {
    while coefficients.len() > 1
        && coefficients
            .last()
            .is_some_and(|coefficient| coefficient.zero_status() == ZeroStatus::Zero)
    {
        coefficients.pop();
    }
    coefficients
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
    let mut valuation = 0;
    while valuation < normalized.len() {
        match real_sign(&normalized[valuation], policy) {
            Some(RealSign::Zero) => valuation += 1,
            Some(RealSign::Positive | RealSign::Negative) => break,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
    if !valuation.is_multiple_of(2) {
        return Ok(Classification::Decided(None));
    }
    let reduced = &normalized[valuation..];
    let degree = reduced.len() - 1;
    if !degree.is_multiple_of(2) {
        return Ok(Classification::Decided(None));
    }
    let root_degree = degree / 2;
    let constant = reduced[0].clone();
    match real_sign(&constant, policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Zero | RealSign::Negative) => {
            return Ok(Classification::Decided(None));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let constant_root = constant.sqrt()?;
    let mut root = vec![Real::zero(); valuation / 2 + root_degree + 1];
    root[valuation / 2] = constant_root.clone();
    for index in 1..=root_degree {
        let mut known = Real::zero();
        for left in 1..index {
            let right = index - left;
            known = &known + &root[valuation / 2 + left] * &root[valuation / 2 + right];
        }
        let residual = &reduced[index] - known;
        root[valuation / 2 + index] = (residual / (&constant_root * Real::from(2_i8)))?;
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

const MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE: usize = 128;
const PARALLEL_INTERSECTION_RESULTANT_PRECISION: i32 = -128;

fn parallel_intersection_candidate_system(
    equations: [BivariatePolynomial; 2],
    candidates: BezierParallelIntersectionCandidates2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParallelIntersectionCandidateSystem2>> {
    if matches!(
        candidates,
        BezierParallelIntersectionCandidates2::DegenerateResultant
    ) && let Some(reduced) = rootless_axis_primitive_system(&equations, policy)?
    {
        let candidates =
            match project_parallel_intersection_system(&reduced[0], &reduced[1], policy)? {
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

fn parallel_candidate_system(
    candidates: BezierParallelIntersectionCandidates2,
    equations: [BivariatePolynomial; 2],
) -> BezierParallelIntersectionCandidateSystem2 {
    let replay_equations = (!matches!(
        &candidates,
        BezierParallelIntersectionCandidates2::NoIntersection
    ))
    .then_some(equations);
    BezierParallelIntersectionCandidateSystem2::projected(candidates, replay_equations)
}

fn project_parallel_intersection_system(
    first_equation: &BivariatePolynomial,
    second_equation: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParallelIntersectionCandidates2>> {
    let config = CurveIntersectionResultantConfig {
        min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
        max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
    };
    let parallel = match resultant_parameter_projection(
        resultant_bivariate_polynomial_system(
            first_equation,
            second_equation,
            CurveResultantParameter::First,
            config,
        ),
        policy,
    )? {
        Classification::Decided(projection) => projection,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if matches!(parallel, ResultantParameterProjection::Degenerate) {
        return Ok(Classification::Decided(
            BezierParallelIntersectionCandidates2::DegenerateResultant,
        ));
    }
    let other = match resultant_parameter_projection(
        resultant_bivariate_polynomial_system(
            first_equation,
            second_equation,
            CurveResultantParameter::Second,
            config,
        ),
        policy,
    )? {
        Classification::Decided(projection) => projection,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(match (parallel, other) {
        (ResultantParameterProjection::Empty, _) | (_, ResultantParameterProjection::Empty) => {
            BezierParallelIntersectionCandidates2::NoIntersection
        }
        (ResultantParameterProjection::Degenerate, _)
        | (_, ResultantParameterProjection::Degenerate) => {
            BezierParallelIntersectionCandidates2::DegenerateResultant
        }
        (
            ResultantParameterProjection::Parameters(parallel_parameters),
            ResultantParameterProjection::Parameters(other_parameters),
        ) => BezierParallelIntersectionCandidates2::Candidates {
            parallel_parameters,
            other_parameters,
        },
    }))
}

/// Returns the axis-primitive system only after proving saturation preserves
/// the complete solution set on the closed parameter square.
fn rootless_axis_primitive_system(
    equations: &[BivariatePolynomial; 2],
    policy: &CurveContext,
) -> CurveResult<Option<[BivariatePolynomial; 2]>> {
    let report = extract_bivariate_polynomial_system_axis_factors(&equations[0], &equations[1]);
    if report.status != BivariatePolynomialAxisFactorStatus::Reduced {
        return Ok(None);
    }
    for factor in [
        &report.first_parameter_factor,
        &report.second_parameter_factor,
    ] {
        if factor.len() <= 1 {
            continue;
        }
        let polynomial = match polynomial_from_coefficients(factor.clone(), policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
        };
        match polynomial.isolate_unit_interval_roots(policy)? {
            Classification::Decided(roots) if roots.is_empty() => {}
            Classification::Decided(_) | Classification::Uncertain(_) => return Ok(None),
        }
    }
    Ok(report.reduced_equations)
}

fn bivariate_system_may_have_component(equations: &[BivariatePolynomial; 2]) -> bool {
    for retained_value in [Real::from(2_i8), Real::from(3_i8), Real::from(5_i8)] {
        let first = bivariate_specialize_first(&equations[0], &retained_value);
        let second = bivariate_specialize_first(&equations[1], &retained_value);
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

struct ParameterComponentSystem2 {
    overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
    isolated_pairs: Arc<[BezierParallelIntersectionParameterPair2]>,
    residual_equations: [BivariatePolynomial; 2],
}

fn parallel_candidate_system_from_parameter_components(
    component: ParameterComponentSystem2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParallelIntersectionCandidateSystem2>> {
    for equation in &component.residual_equations {
        if bivariate_unit_square_has_strict_bernstein_sign(equation, policy)? {
            let mut candidate_system = BezierParallelIntersectionCandidateSystem2::projected(
                BezierParallelIntersectionCandidates2::NoIntersection,
                None,
            );
            candidate_system.overlaps = component.overlaps;
            candidate_system.component_pairs = component.isolated_pairs;
            return Ok(Classification::Decided(candidate_system));
        }
    }
    let candidates = match project_parallel_intersection_system(
        &component.residual_equations[0],
        &component.residual_equations[1],
        policy,
    )? {
        Classification::Decided(candidates) => candidates,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    match parallel_intersection_candidate_system(component.residual_equations, candidates, policy)?
    {
        Classification::Decided(mut candidate_system) => {
            candidate_system.overlaps = component.overlaps;
            candidate_system.component_pairs = component.isolated_pairs;
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
fn bivariate_unit_square_has_strict_bernstein_sign(
    polynomial: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<bool> {
    let first_degree = polynomial.coefficients.len().saturating_sub(1);
    let second_degree = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or_default()
        .saturating_sub(1);
    if polynomial.coefficients.is_empty() || polynomial.coefficients.iter().all(Vec::is_empty) {
        return Ok(false);
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
            let Some(sign @ (RealSign::Positive | RealSign::Negative)) =
                real_sign(&control, policy)
            else {
                return Ok(false);
            };
            match strict_sign {
                Some(previous) if previous != sign => return Ok(false),
                Some(_) => {}
                None => strict_sign = Some(sign),
            }
        }
    }
    Ok(strict_sign.is_some())
}

fn parameter_component_system(
    equations: &[BivariatePolynomial; 2],
    branch: &BivariatePolynomial,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<Option<ParameterComponentSystem2>>> {
    let mut residual_equations = equations.clone();
    let mut overlaps = Vec::new();
    let mut isolated_pairs = Vec::new();
    let mut extracted_component = false;
    loop {
        let mut blocker = None;
        let mut next_component = None;
        for retained_parameter in [
            CurveResultantParameter::Second,
            CurveResultantParameter::First,
        ] {
            let report = parameter_component_bivariate_polynomial_system(
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
                    match certify_rational_parameter_component_map(
                        &residual_equations,
                        branch,
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
                    match certify_regular_implicit_parameter_component(
                        &component,
                        branch,
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
            return Ok(Classification::Decided(Some(ParameterComponentSystem2 {
                overlaps: overlaps.into(),
                isolated_pairs: isolated_pairs.into(),
                residual_equations,
            })));
        };
        for overlap in evidence.overlaps.iter() {
            if !overlaps.contains(overlap) {
                overlaps.push(overlap.clone());
            }
        }
        for pair in evidence.isolated_pairs.iter() {
            if !isolated_pairs.contains(pair) {
                isolated_pairs.push(pair.clone());
            }
        }
        residual_equations = reduced_equations;
        extracted_component = true;
        for equation in &residual_equations {
            if bivariate_unit_square_has_strict_bernstein_sign(equation, policy)? {
                return Ok(Classification::Decided(Some(ParameterComponentSystem2 {
                    overlaps: overlaps.into(),
                    isolated_pairs: isolated_pairs.into(),
                    residual_equations,
                })));
            }
        }
    }
}

/// Certifies the complete unit-square image of one regular implicit component.
///
/// This is the first algebraic-function transport cut. It accepts exactly one
/// graph over the retained unit interval, proves that no root enters through
/// an open box edge, excludes vertical and horizontal critical points, and
/// proves the selected-branch predicate has no zero on the graph. More general
/// multi-cell or singular implicit topology remains an explicit boundary.
fn certify_regular_implicit_parameter_component(
    component: &BivariatePolynomial,
    branch: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
) -> CurveResult<Classification<Option<ParameterComponentEvidence2>>> {
    let swapped_component;
    let swapped_branch;
    let (component, branch) = match retained_parameter {
        CurveResultantParameter::First => (component, branch),
        CurveResultantParameter::Second => {
            swapped_component = bivariate_swap_parameters(component);
            swapped_branch = bivariate_swap_parameters(branch);
            (&swapped_component, &swapped_branch)
        }
    };

    let start_roots = match polynomial_unit_interval_roots(
        &bivariate_specialize_first(component, &Real::zero()),
        policy,
    )? {
        Classification::Decided(Some(roots)) if roots.len() == 1 => roots,
        Classification::Decided(_) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end_roots = match polynomial_unit_interval_roots(
        &bivariate_specialize_first(component, &Real::one()),
        policy,
    )? {
        Classification::Decided(Some(roots)) if roots.len() == 1 => roots,
        Classification::Decided(_) => return Ok(Classification::Decided(None)),
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
        match parameters_are_unit_endpoints(&roots, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }

    for derivative in [
        bivariate_parameter_derivative(component, CurveResultantParameter::First),
        bivariate_parameter_derivative(component, CurveResultantParameter::Second),
    ] {
        match bivariate_system_has_unit_square_solution(component, &derivative, policy, config)? {
            Classification::Decided(false) => {}
            Classification::Decided(true) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }

    let half = (Real::one() / Real::from(2_i8))?;
    let midpoint_root = match polynomial_unit_interval_roots(
        &bivariate_specialize_first(component, &half),
        policy,
    )? {
        Classification::Decided(Some(mut roots)) if roots.len() == 1 => roots.pop().unwrap(),
        Classification::Decided(_) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    match bivariate_system_has_unit_square_solution(component, branch, policy, config)? {
        Classification::Decided(false) => {}
        Classification::Decided(true) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    let branch_sign = match signed_bivariate_at_parameter_pair(
        branch,
        &BezierParameter2::Exact(half),
        &midpoint_root,
        policy,
    )? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let lifted_start = start_roots.into_iter().next().unwrap();
    let lifted_end = end_roots.into_iter().next().unwrap();
    let direction = match lifted_start.cmp_by_refinement(&lifted_end, policy)? {
        Classification::Decided(std::cmp::Ordering::Equal) => {
            return Ok(Classification::Decided(None));
        }
        Classification::Decided(direction) => direction,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let overlaps = match branch_sign {
        RealSign::Positive => Arc::from([parameter_component_overlap_from_domain(
            retained_parameter,
            ParameterComponentDomain {
                retained_start: BezierParameter2::Exact(Real::zero()),
                retained_end: BezierParameter2::Exact(Real::one()),
                lifted_start,
                lifted_end,
            },
            direction,
        )]),
        RealSign::Negative => Arc::from([]),
        RealSign::Zero => return Ok(Classification::Decided(None)),
    };
    Ok(Classification::Decided(Some(ParameterComponentEvidence2 {
        overlaps,
        isolated_pairs: Arc::from([]),
    })))
}

fn parameters_are_unit_endpoints(
    parameters: &[BezierParameter2],
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let zero = BezierParameter2::Exact(Real::zero());
    let one = BezierParameter2::Exact(Real::one());
    for parameter in parameters {
        match parameter.cmp_by_refinement(&zero, policy)? {
            Classification::Decided(std::cmp::Ordering::Equal) => continue,
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        match parameter.cmp_by_refinement(&one, policy)? {
            Classification::Decided(std::cmp::Ordering::Equal) => {}
            Classification::Decided(_) => return Ok(Classification::Decided(false)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(true))
}

fn bivariate_parameter_derivative(
    polynomial: &BivariatePolynomial,
    parameter: CurveResultantParameter,
) -> BivariatePolynomial {
    let coefficients = match parameter {
        CurveResultantParameter::First => polynomial
            .coefficients
            .iter()
            .enumerate()
            .skip(1)
            .map(|(power, row)| {
                let scale = Real::from(power as u64);
                row.iter().map(|coefficient| coefficient * &scale).collect()
            })
            .collect(),
        CurveResultantParameter::Second => polynomial
            .coefficients
            .iter()
            .map(|row| {
                row.iter()
                    .enumerate()
                    .skip(1)
                    .map(|(power, coefficient)| coefficient * Real::from(power as u64))
                    .collect()
            })
            .collect(),
    };
    BivariatePolynomial::new(coefficients)
}

fn bivariate_system_has_unit_square_solution(
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
    let candidates = match project_parallel_intersection_system(first, second, policy)? {
        Classification::Decided(candidates) => candidates,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let (parallel_parameters, other_parameters) = match candidates {
        BezierParallelIntersectionCandidates2::Candidates {
            parallel_parameters,
            other_parameters,
        } => (parallel_parameters, other_parameters),
        BezierParallelIntersectionCandidates2::NoIntersection => {
            return Ok(Classification::Decided(false));
        }
        BezierParallelIntersectionCandidates2::DegenerateResultant => {
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

fn certify_rational_parameter_component_map(
    equations: &[BivariatePolynomial; 2],
    branch: &BivariatePolynomial,
    retained_parameter: CurveResultantParameter,
    map: &CurveIntersectionParameterLiftMap,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<ParameterComponentEvidence2>>> {
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
    let partition = match rational_parameter_component_domains(map, &derivative_numerator, policy)?
    {
        Classification::Decided(Some(partition)) => partition,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let (branch_coefficients, _) =
        bivariate_on_parameter_lift_cleared(branch, retained_parameter, map);
    let mut overlaps = Vec::with_capacity(partition.domains.len());
    for (domain, direction) in partition.domains {
        match polynomial_is_rootless_on_parameter_range(
            &branch_coefficients,
            &domain.retained_start,
            &domain.retained_end,
            policy,
        )? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let branch_sample = match domain
            .retained_start
            .strict_rational_between_ordered(&domain.retained_end, policy)?
        {
            Classification::Decided(sample) => BezierParameter2::Exact(sample),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let branch_sign = match signed_bivariate_on_parameter_lift(
            branch,
            &branch_sample,
            retained_parameter,
            map,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match branch_sign {
            RealSign::Positive => overlaps.push(parameter_component_overlap_from_domain(
                retained_parameter,
                domain,
                direction,
            )),
            RealSign::Negative => {}
            RealSign::Zero => return Ok(Classification::Decided(None)),
        }
    }
    let mut isolated_pairs = Vec::with_capacity(partition.isolated_points.len());
    for point in partition.isolated_points {
        let sign = match signed_bivariate_on_parameter_lift(
            branch,
            &point.retained_parameter,
            retained_parameter,
            map,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match sign {
            RealSign::Positive => {
                isolated_pairs.push(rational_parameter_component_pair(retained_parameter, point))
            }
            RealSign::Negative => {}
            RealSign::Zero => return Ok(Classification::Decided(None)),
        }
    }
    Ok(Classification::Decided(Some(ParameterComponentEvidence2 {
        overlaps: overlaps.into(),
        isolated_pairs: isolated_pairs.into(),
    })))
}

struct ParameterComponentEvidence2 {
    overlaps: Arc<[RationalBezierIntersectionOverlap2]>,
    isolated_pairs: Arc<[BezierParallelIntersectionParameterPair2]>,
}

#[derive(Clone)]
struct ParameterComponentDomain {
    retained_start: BezierParameter2,
    retained_end: BezierParameter2,
    lifted_start: BezierParameter2,
    lifted_end: BezierParameter2,
}

struct RationalParameterComponentPoint {
    retained_parameter: BezierParameter2,
    lifted_parameter: BezierParameter2,
}

struct RationalParameterComponentPartition {
    domains: Vec<(ParameterComponentDomain, std::cmp::Ordering)>,
    isolated_points: Vec<RationalParameterComponentPoint>,
}

#[derive(Clone)]
struct RationalParameterComponentBoundary {
    parameter: BezierParameter2,
    denominator_root: bool,
    zero_image: bool,
    unit_image: bool,
}

fn rational_parameter_component_domains(
    map: &CurveIntersectionParameterLiftMap,
    derivative_numerator: &[Real],
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
            .strict_rational_between_ordered(&boundaries[index + 1].parameter, policy)?
        {
            Classification::Decided(sample) => sample,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let direction = match real_sign(&polynomial_evaluate(derivative_numerator, &sample), policy)
        {
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
        let lifted_start =
            match rational_parameter_component_boundary_image(&boundaries[start], map, policy)? {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        let lifted_end =
            match rational_parameter_component_boundary_image(&boundaries[end], map, policy)? {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        match lifted_start.cmp_by_refinement(&lifted_end, policy)? {
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
            match rational_parameter_component_boundary_image(boundary, map, policy)? {
                Classification::Decided(Some(parameter)) => parameter,
                Classification::Decided(None) => continue,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        isolated_points.push(RationalParameterComponentPoint {
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

fn polynomial_unit_interval_roots(
    coefficients: &[Real],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<BezierParameter2>>>> {
    let polynomial = match polynomial_from_coefficients(coefficients.to_vec(), policy)? {
        Classification::Decided(Some(polynomial)) => polynomial,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(polynomial.isolate_unit_interval_roots(policy)?.map(Some))
}

fn insert_rational_parameter_component_boundary(
    boundaries: &mut Vec<RationalParameterComponentBoundary>,
    boundary: RationalParameterComponentBoundary,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let mut index = 0;
    while index < boundaries.len() {
        match boundary
            .parameter
            .cmp_by_refinement(&boundaries[index].parameter, policy)?
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

fn rational_parameter_component_boundary_image(
    boundary: &RationalParameterComponentBoundary,
    map: &CurveIntersectionParameterLiftMap,
    policy: &CurveContext,
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
    rational_parameter_image(
        &boundary.parameter,
        &map.numerator_coefficients,
        &map.denominator_coefficients,
        policy,
    )
}

fn parameter_component_overlap_from_domain(
    retained_parameter: CurveResultantParameter,
    domain: ParameterComponentDomain,
    direction: std::cmp::Ordering,
) -> RationalBezierIntersectionOverlap2 {
    let orientation = match direction {
        std::cmp::Ordering::Less => RationalBezierOverlapOrientation2::Same,
        std::cmp::Ordering::Greater => RationalBezierOverlapOrientation2::Reversed,
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
            )
        }
        (CurveResultantParameter::Second, std::cmp::Ordering::Less) => {
            RationalBezierIntersectionOverlap2::from_certified_parameters(
                domain.lifted_start,
                domain.lifted_end,
                domain.retained_start,
                domain.retained_end,
                orientation,
            )
        }
        (CurveResultantParameter::Second, std::cmp::Ordering::Greater) => {
            RationalBezierIntersectionOverlap2::from_certified_parameters(
                domain.lifted_end,
                domain.lifted_start,
                domain.retained_end,
                domain.retained_start,
                orientation,
            )
        }
        (CurveResultantParameter::Second, std::cmp::Ordering::Equal) => {
            unreachable!("a component domain has nonzero derivative")
        }
    }
}

fn rational_parameter_component_pair(
    retained_parameter: CurveResultantParameter,
    point: RationalParameterComponentPoint,
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

fn polynomial_is_rootless_on_parameter_range(
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
    let roots = match polynomial.isolate_unit_interval_roots(policy)? {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    for root in roots {
        let start_order = match root.cmp_by_refinement(start, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end_order = match root.cmp_by_refinement(end, policy)? {
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

fn rational_parameter_map_at(
    map: &CurveIntersectionParameterLiftMap,
    parameter: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Real>> {
    let denominator = polynomial_evaluate(&map.denominator_coefficients, parameter);
    match real_sign(&denominator, policy) {
        Some(RealSign::Positive | RealSign::Negative) => {}
        Some(RealSign::Zero) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    Ok(Classification::Decided(
        (polynomial_evaluate(&map.numerator_coefficients, parameter) / denominator)?,
    ))
}

fn parallel_rational_intersection_equations(
    source: &BezierParallelPowerBasisRef<'_>,
    differential: &BezierParallelDifferential2,
    distance: &Real,
    other: &RationalParametricCurve2,
) -> (BivariatePolynomial, BivariatePolynomial) {
    let unit_weight = [Real::one()];
    let source_weight = source.weight.unwrap_or(&unit_weight);
    let delta_x = bivariate_parameter_difference(
        source_weight,
        &other.x_numerator,
        source.x_numerator,
        &other.weight,
    );
    let delta_y = bivariate_parameter_difference(
        source_weight,
        &other.y_numerator,
        source.y_numerator,
        &other.weight,
    );
    let orthogonality = bivariate_add(
        &bivariate_multiply_first_parameter(&delta_x, &differential.tangent_x),
        &bivariate_multiply_first_parameter(&delta_y, &differential.tangent_y),
    );
    let squared_delta = bivariate_add(
        &bivariate_multiply(&delta_x, &delta_x),
        &bivariate_multiply(&delta_y, &delta_y),
    );
    let weighted_distance_squared = bivariate_scale(
        bivariate_outer_product(
            &polynomial_multiply(source_weight, source_weight),
            &polynomial_multiply(&other.weight, &other.weight),
        ),
        &(distance * distance),
    );
    (
        orthogonality,
        bivariate_subtract(&squared_delta, &weighted_distance_squared),
    )
}

fn parallel_rational_selected_branch(
    source: &BezierParallelPowerBasisRef<'_>,
    differential: &BezierParallelDifferential2,
    distance: &Real,
    other: &RationalParametricCurve2,
) -> BivariatePolynomial {
    let unit_weight = [Real::one()];
    let source_weight = source.weight.unwrap_or(&unit_weight);
    let delta_x = bivariate_parameter_difference(
        source_weight,
        &other.x_numerator,
        source.x_numerator,
        &other.weight,
    );
    let delta_y = bivariate_parameter_difference(
        source_weight,
        &other.y_numerator,
        source.y_numerator,
        &other.weight,
    );
    let orientation = bivariate_subtract(
        &bivariate_multiply_first_parameter(&delta_y, &differential.tangent_x),
        &bivariate_multiply_first_parameter(&delta_x, &differential.tangent_y),
    );
    bivariate_scale(
        bivariate_multiply(
            &orientation,
            &bivariate_outer_product(source_weight, &other.weight),
        ),
        distance,
    )
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BivariateParameterPairReplay {
    Rejected,
    Direct,
    LinearLift(CurveResultantParameter, usize),
}

fn replay_bivariate_parameter_pair(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
    config: CurveIntersectionResultantConfig,
    parameter_lifts: &mut [Option<CurveIntersectionParameterLiftReport>; 2],
) -> CurveResult<Classification<BivariateParameterPairReplay>> {
    match bivariate_pair_satisfies_system(first, second, first_parameter, second_parameter, policy)?
    {
        Classification::Decided(true) => {
            return Ok(Classification::Decided(
                BivariateParameterPairReplay::Direct,
            ));
        }
        Classification::Decided(false) => {
            return Ok(Classification::Decided(
                BivariateParameterPairReplay::Rejected,
            ));
        }
        Classification::Uncertain(_) => {}
    }

    let first_lifts = parameter_lifts[0].get_or_insert_with(|| {
        linear_parameter_lifts_bivariate_polynomial_system(
            first,
            second,
            CurveResultantParameter::First,
            config,
        )
    });
    match parameter_pair_matches_linear_lift(
        first_lifts,
        first_parameter,
        second_parameter,
        policy,
    )? {
        Classification::Decided(replay) => return Ok(Classification::Decided(replay)),
        Classification::Uncertain(_) => {}
    }

    for (axis, retained_parameter, fiber_parameter) in [
        (
            CurveResultantParameter::First,
            first_parameter,
            second_parameter,
        ),
        (
            CurveResultantParameter::Second,
            second_parameter,
            first_parameter,
        ),
    ] {
        match parameter_pair_matches_specialized_fiber(
            first,
            second,
            axis,
            retained_parameter,
            fiber_parameter,
            policy,
        )? {
            Classification::Decided(replay) => return Ok(Classification::Decided(replay)),
            Classification::Uncertain(UncertaintyReason::Boundary) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(_) => {}
        }
    }

    let second_lifts = parameter_lifts[1].get_or_insert_with(|| {
        linear_parameter_lifts_bivariate_polynomial_system(
            first,
            second,
            CurveResultantParameter::Second,
            config,
        )
    });
    match parameter_pair_matches_linear_lift(
        second_lifts,
        second_parameter,
        first_parameter,
        policy,
    )? {
        Classification::Decided(replay) => Ok(Classification::Decided(replay)),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn bivariate_pair_satisfies_system(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let first_sign =
        signed_bivariate_at_parameter_pair(first, first_parameter, second_parameter, policy)?;
    if matches!(
        first_sign,
        Classification::Decided(RealSign::Positive | RealSign::Negative)
    ) {
        return Ok(Classification::Decided(false));
    }
    let mut second_sign =
        signed_bivariate_at_parameter_pair(second, first_parameter, second_parameter, policy)?;
    if matches!(
        second_sign,
        Classification::Decided(RealSign::Positive | RealSign::Negative)
    ) {
        return Ok(Classification::Decided(false));
    }
    if matches!(first_sign, Classification::Decided(RealSign::Zero))
        && matches!(second_sign, Classification::Uncertain(_))
        && let Classification::Decided(Some(reduced)) =
            reduce_bivariate_by_single_axis_equation(first, second, policy)?
    {
        second_sign = signed_bivariate_at_parameter_pair(
            &reduced,
            first_parameter,
            second_parameter,
            policy,
        )?;
        if matches!(
            second_sign,
            Classification::Decided(RealSign::Positive | RealSign::Negative)
        ) {
            return Ok(Classification::Decided(false));
        }
    }
    let mut first_sign = first_sign;
    if matches!(second_sign, Classification::Decided(RealSign::Zero))
        && matches!(first_sign, Classification::Uncertain(_))
        && let Classification::Decided(Some(reduced)) =
            reduce_bivariate_by_single_axis_equation(second, first, policy)?
    {
        first_sign = signed_bivariate_at_parameter_pair(
            &reduced,
            first_parameter,
            second_parameter,
            policy,
        )?;
        if matches!(
            first_sign,
            Classification::Decided(RealSign::Positive | RealSign::Negative)
        ) {
            return Ok(Classification::Decided(false));
        }
    }
    match (first_sign, second_sign) {
        (Classification::Decided(RealSign::Zero), Classification::Decided(RealSign::Zero)) => {
            Ok(Classification::Decided(true))
        }
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
            Ok(Classification::Uncertain(reason))
        }
        _ => unreachable!("nonzero bivariate signs returned above"),
    }
}

fn reduce_bivariate_by_single_axis_equation(
    vanishing: &BivariatePolynomial,
    target: &BivariatePolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BivariatePolynomial>>> {
    for axis in [
        CurveResultantParameter::First,
        CurveResultantParameter::Second,
    ] {
        let coefficients = match bivariate_single_axis_coefficients(vanishing, axis, policy)? {
            Classification::Decided(Some(coefficients)) => coefficients,
            Classification::Decided(None) => continue,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let modulus = match polynomial_from_coefficients(coefficients, policy)? {
            Classification::Decided(Some(modulus)) if modulus.degree() > 0 => modulus,
            Classification::Decided(_) => continue,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        return Ok(bivariate_reduce_axis(target, &modulus, axis, policy)?.map(Some));
    }
    Ok(Classification::Decided(None))
}

fn parameter_pair_matches_linear_lift(
    report: &CurveIntersectionParameterLiftReport,
    retained_parameter: &BezierParameter2,
    lifted_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<BivariateParameterPairReplay>> {
    if report.status != CurveIntersectionParameterLiftStatus::Constructed
        || report.retained_parameter == report.lifted_parameter
    {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let mut blocker = UncertaintyReason::Predicate;
    for (map_index, map) in report.maps.iter().enumerate() {
        match crate::rational_bezier_general::rational_parameter_image_matches(
            retained_parameter,
            lifted_parameter,
            &map.numerator_coefficients,
            &map.denominator_coefficients,
            policy,
        )? {
            Classification::Decided(true) => {
                return Ok(Classification::Decided(
                    BivariateParameterPairReplay::LinearLift(report.retained_parameter, map_index),
                ));
            }
            Classification::Decided(false) => {
                return Ok(Classification::Decided(
                    BivariateParameterPairReplay::Rejected,
                ));
            }
            Classification::Uncertain(reason) => {
                blocker = reason;
                continue;
            }
        }
    }
    Ok(Classification::Uncertain(blocker))
}

fn parameter_pair_matches_specialized_fiber(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    retained_axis: CurveResultantParameter,
    retained_parameter: &BezierParameter2,
    fiber_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<BivariateParameterPairReplay>> {
    #[cfg(feature = "predicates")]
    {
        let (
            BezierParameter2::Algebraic(retained_parameter),
            BezierParameter2::Algebraic(fiber_parameter),
        ) = (retained_parameter, fiber_parameter)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
            first,
            second,
            retained_axis,
            &parameter_representation(retained_parameter, policy),
            fiber_parameter.interval().start(),
            fiber_parameter.interval().end(),
            policy.predicate_policy(),
        );
        if report.certainty == PredicateCertainty::Approximate {
            policy.observe_approximate_512();
        }
        Ok(match report.status {
            AlgebraicFiberRootCountStatus::Counted => match report.distinct_root_count {
                Some(0) => Classification::Decided(BivariateParameterPairReplay::Rejected),
                Some(_) => Classification::Decided(BivariateParameterPairReplay::Direct),
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

    #[cfg(not(feature = "predicates"))]
    {
        let _ = (
            first,
            second,
            retained_axis,
            retained_parameter,
            fiber_parameter,
            policy,
        );
        Ok(Classification::Uncertain(UncertaintyReason::Predicate))
    }
}

fn signed_bivariate_on_parameter_lift(
    polynomial: &BivariatePolynomial,
    retained_parameter: &BezierParameter2,
    retained_axis: CurveResultantParameter,
    map: &CurveIntersectionParameterLiftMap,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let (cleared, lifted_degree) =
        bivariate_on_parameter_lift_cleared(polynomial, retained_axis, map);
    let cleared_sign = match signed_coefficients_at_parameter(cleared, retained_parameter, policy)?
    {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if cleared_sign == RealSign::Zero || lifted_degree.is_multiple_of(2) {
        return Ok(Classification::Decided(cleared_sign));
    }
    let denominator_sign = match signed_coefficients_at_parameter(
        map.denominator_coefficients.clone(),
        retained_parameter,
        policy,
    )? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if denominator_sign == RealSign::Zero {
        return Err(CurveError::Topology(
            "selected bivariate parameter lift denominator vanished".to_owned(),
        ));
    }
    Ok(Classification::Decided(
        match (cleared_sign, denominator_sign) {
            (RealSign::Positive, RealSign::Positive) | (RealSign::Negative, RealSign::Negative) => {
                RealSign::Positive
            }
            (RealSign::Positive, RealSign::Negative) | (RealSign::Negative, RealSign::Positive) => {
                RealSign::Negative
            }
            (RealSign::Zero, _) | (_, RealSign::Zero) => {
                unreachable!("zero signs returned before multiplication")
            }
        },
    ))
}

fn bivariate_on_parameter_lift_cleared(
    polynomial: &BivariatePolynomial,
    retained_axis: CurveResultantParameter,
    map: &CurveIntersectionParameterLiftMap,
) -> (Vec<Real>, usize) {
    let swapped;
    let polynomial = match retained_axis {
        CurveResultantParameter::First => polynomial,
        CurveResultantParameter::Second => {
            swapped = bivariate_swap_parameters(polynomial);
            &swapped
        }
    };
    let lifted_degree = polynomial
        .coefficients
        .iter()
        .map(|row| row.len().saturating_sub(1))
        .max()
        .unwrap_or(0);
    let numerator_powers = polynomial_powers(&map.numerator_coefficients, lifted_degree);
    let denominator_powers = polynomial_powers(&map.denominator_coefficients, lifted_degree);
    let retained_degree = polynomial.coefficients.len().saturating_sub(1);
    let map_degree = map
        .numerator_coefficients
        .len()
        .max(map.denominator_coefficients.len())
        .saturating_sub(1);
    let mut cleared =
        vec![Real::zero(); retained_degree + lifted_degree.saturating_mul(map_degree) + 1];
    for (retained_power, row) in polynomial.coefficients.iter().enumerate() {
        for (lifted_power, coefficient) in row.iter().enumerate() {
            let factor = polynomial_multiply(
                &numerator_powers[lifted_power],
                &denominator_powers[lifted_degree - lifted_power],
            );
            for (power, factor) in factor.iter().enumerate() {
                cleared[retained_power + power] += coefficient * factor;
            }
        }
    }
    (cleared, lifted_degree)
}

fn bivariate_swap_parameters(polynomial: &BivariatePolynomial) -> BivariatePolynomial {
    let first_count = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    let second_count = polynomial.coefficients.len();
    let mut coefficients = vec![vec![Real::zero(); second_count]; first_count];
    for (first, row) in polynomial.coefficients.iter().enumerate() {
        for (second, coefficient) in row.iter().enumerate() {
            coefficients[second][first] = coefficient.clone();
        }
    }
    BivariatePolynomial::new(coefficients)
}

fn polynomial_powers(polynomial: &[Real], maximum: usize) -> Vec<Vec<Real>> {
    let mut powers = Vec::with_capacity(maximum + 1);
    powers.push(vec![Real::one()]);
    for power in 1..=maximum {
        powers.push(polynomial_multiply(&powers[power - 1], polynomial));
    }
    powers
}

fn signed_bivariate_at_parameter_pair(
    polynomial: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    match (first_parameter, second_parameter) {
        (BezierParameter2::Exact(first), BezierParameter2::Exact(second)) => {
            match real_sign(
                &polynomial_evaluate(&bivariate_specialize_first(polynomial, first), second),
                policy,
            ) {
                Some(sign) => Ok(Classification::Decided(sign)),
                None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        (BezierParameter2::Exact(first), second) => signed_coefficients_at_parameter(
            bivariate_specialize_first(polynomial, first),
            second,
            policy,
        ),
        (first, BezierParameter2::Exact(second)) => signed_coefficients_at_parameter(
            bivariate_specialize_second(polynomial, second),
            first,
            policy,
        ),
        (
            first @ BezierParameter2::Algebraic(first_algebraic),
            second @ BezierParameter2::Algebraic(second_algebraic),
        ) => {
            let mut blocker = UncertaintyReason::Predicate;
            match bivariate_single_axis_coefficients(
                polynomial,
                CurveResultantParameter::First,
                policy,
            )? {
                Classification::Decided(Some(coefficients)) => {
                    return signed_coefficients_at_parameter(coefficients, first, policy);
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => blocker = reason,
            }
            match bivariate_single_axis_coefficients(
                polynomial,
                CurveResultantParameter::Second,
                policy,
            )? {
                Classification::Decided(Some(coefficients)) => {
                    return signed_coefficients_at_parameter(coefficients, second, policy);
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => blocker = reason,
            }
            match signed_rank_one_bivariate_at_parameter_pair(polynomial, first, second, policy)? {
                Classification::Decided(Some(sign)) => {
                    return Ok(Classification::Decided(sign));
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => blocker = reason,
            }
            if matches!(
                first.same_value(second, policy)?,
                Classification::Decided(true)
            ) {
                return signed_coefficients_at_parameter(
                    bivariate_substitute_second_equal_first(polynomial),
                    first,
                    policy,
                );
            }
            let complemented = first.clone().unit_complement();
            if matches!(
                complemented.same_value(second, policy)?,
                Classification::Decided(true)
            ) {
                return signed_coefficients_at_parameter(
                    bivariate_substitute_second_equal_one_minus_first(polynomial),
                    first,
                    policy,
                );
            }
            let reduced = match bivariate_reduce_parameter_polynomials(
                polynomial,
                first_algebraic.polynomial(),
                second_algebraic.polynomial(),
                policy,
            )? {
                Classification::Decided(reduced) => Some(reduced),
                Classification::Uncertain(reason) => {
                    blocker = reason;
                    None
                }
            };
            if let Some(reduced) = reduced.as_ref() {
                match bivariate_single_axis_coefficients(
                    reduced,
                    CurveResultantParameter::First,
                    policy,
                )? {
                    Classification::Decided(Some(coefficients)) => {
                        return signed_coefficients_at_parameter(coefficients, first, policy);
                    }
                    Classification::Decided(None) => {}
                    Classification::Uncertain(reason) => blocker = reason,
                }
                match bivariate_single_axis_coefficients(
                    reduced,
                    CurveResultantParameter::Second,
                    policy,
                )? {
                    Classification::Decided(Some(coefficients)) => {
                        return signed_coefficients_at_parameter(coefficients, second, policy);
                    }
                    Classification::Decided(None) => {}
                    Classification::Uncertain(reason) => blocker = reason,
                }
            }
            Ok(Classification::Uncertain(blocker))
        }
    }
}

fn signed_rank_one_bivariate_at_parameter_pair(
    polynomial: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<RealSign>>> {
    let column_count = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    let mut pivot = None;
    let mut unknown = false;
    for (row_index, row) in polynomial.coefficients.iter().enumerate() {
        for column_index in 0..column_count {
            let coefficient = row.get(column_index).cloned().unwrap_or_else(Real::zero);
            match real_sign(&coefficient, policy) {
                Some(RealSign::Zero) => {}
                Some(sign @ (RealSign::Positive | RealSign::Negative)) => {
                    pivot = Some((row_index, column_index, coefficient, sign));
                    break;
                }
                None => unknown = true,
            }
        }
        if pivot.is_some() {
            break;
        }
    }
    let Some((pivot_row_index, pivot_column_index, pivot_value, pivot_sign)) = pivot else {
        return Ok(if unknown {
            Classification::Uncertain(UncertaintyReason::RealSign)
        } else {
            Classification::Decided(Some(RealSign::Zero))
        });
    };
    let pivot_row = &polynomial.coefficients[pivot_row_index];
    for row in &polynomial.coefficients {
        let column_value = row
            .get(pivot_column_index)
            .cloned()
            .unwrap_or_else(Real::zero);
        for column_index in 0..column_count {
            let coefficient = row.get(column_index).cloned().unwrap_or_else(Real::zero);
            let row_value = pivot_row
                .get(column_index)
                .cloned()
                .unwrap_or_else(Real::zero);
            match real_sign(
                &(coefficient * &pivot_value - &column_value * row_value),
                policy,
            ) {
                Some(RealSign::Zero) => {}
                Some(RealSign::Positive | RealSign::Negative) => {
                    return Ok(Classification::Decided(None));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
    }

    let first_sign = signed_coefficients_at_parameter(
        polynomial
            .coefficients
            .iter()
            .map(|row| {
                row.get(pivot_column_index)
                    .cloned()
                    .unwrap_or_else(Real::zero)
            })
            .collect(),
        first_parameter,
        policy,
    )?;
    let second_sign = signed_coefficients_at_parameter(
        (0..column_count)
            .map(|column_index| {
                pivot_row
                    .get(column_index)
                    .cloned()
                    .unwrap_or_else(Real::zero)
            })
            .collect(),
        second_parameter,
        policy,
    )?;
    Ok(match (first_sign, second_sign) {
        (Classification::Decided(RealSign::Zero), _)
        | (_, Classification::Decided(RealSign::Zero)) => {
            Classification::Decided(Some(RealSign::Zero))
        }
        (
            Classification::Decided(first @ (RealSign::Positive | RealSign::Negative)),
            Classification::Decided(second @ (RealSign::Positive | RealSign::Negative)),
        ) => Classification::Decided(Some(
            if (first == second) == (pivot_sign == RealSign::Positive) {
                RealSign::Positive
            } else {
                RealSign::Negative
            },
        )),
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
            Classification::Uncertain(reason)
        }
    })
}

fn signed_coefficients_at_parameter(
    coefficients: Vec<Real>,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let polynomial = match polynomial_from_coefficients(coefficients, policy)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    signed_polynomial_at_root(polynomial.as_ref(), parameter, policy)
}

fn bivariate_specialize_first(polynomial: &BivariatePolynomial, value: &Real) -> Vec<Real> {
    let second_count = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    (0..second_count)
        .map(|second_power| {
            polynomial
                .coefficients
                .iter()
                .rev()
                .fold(Real::zero(), |accumulator, row| {
                    accumulator * value + row.get(second_power).cloned().unwrap_or_else(Real::zero)
                })
        })
        .collect()
}

fn bivariate_specialize_second(polynomial: &BivariatePolynomial, value: &Real) -> Vec<Real> {
    polynomial
        .coefficients
        .iter()
        .map(|row| polynomial_evaluate(row, value))
        .collect()
}

fn bivariate_reduce_parameter_polynomials(
    polynomial: &BivariatePolynomial,
    first: &BezierParameterPolynomial,
    second: &BezierParameterPolynomial,
    policy: &CurveContext,
) -> CurveResult<Classification<BivariatePolynomial>> {
    let first_reduced =
        match bivariate_reduce_axis(polynomial, first, CurveResultantParameter::First, policy)? {
            Classification::Decided(reduced) => reduced,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    bivariate_reduce_axis(
        &first_reduced,
        second,
        CurveResultantParameter::Second,
        policy,
    )
}

fn bivariate_reduce_axis(
    polynomial: &BivariatePolynomial,
    modulus: &BezierParameterPolynomial,
    axis: CurveResultantParameter,
    policy: &CurveContext,
) -> CurveResult<Classification<BivariatePolynomial>> {
    let axis_degree = match axis {
        CurveResultantParameter::First => polynomial.coefficients.len().saturating_sub(1),
        CurveResultantParameter::Second => polynomial
            .coefficients
            .iter()
            .map(|row| row.len().saturating_sub(1))
            .max()
            .unwrap_or(0),
    };
    if modulus.degree() > axis_degree {
        return Ok(Classification::Decided(polynomial.clone()));
    }
    match axis {
        CurveResultantParameter::First => {
            let second_count = polynomial
                .coefficients
                .iter()
                .map(Vec::len)
                .max()
                .unwrap_or(0);
            let mut columns = Vec::with_capacity(second_count);
            for second_power in 0..second_count {
                let coefficients = polynomial
                    .coefficients
                    .iter()
                    .map(|row| row.get(second_power).cloned().unwrap_or_else(Real::zero))
                    .collect();
                match modulus.reduce_power_basis(coefficients, policy)? {
                    Classification::Decided(coefficients) => columns.push(coefficients),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            let first_count = columns.iter().map(Vec::len).max().unwrap_or(0);
            let coefficients = (0..first_count)
                .map(|first_power| {
                    columns
                        .iter()
                        .map(|column| column.get(first_power).cloned().unwrap_or_else(Real::zero))
                        .collect()
                })
                .collect();
            Ok(Classification::Decided(BivariatePolynomial::new(
                coefficients,
            )))
        }
        CurveResultantParameter::Second => {
            let mut coefficients = Vec::with_capacity(polynomial.coefficients.len());
            for row in &polynomial.coefficients {
                match modulus.reduce_power_basis(row.clone(), policy)? {
                    Classification::Decided(row) => coefficients.push(row),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Ok(Classification::Decided(BivariatePolynomial::new(
                coefficients,
            )))
        }
    }
}

fn bivariate_single_axis_coefficients(
    polynomial: &BivariatePolynomial,
    axis: CurveResultantParameter,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<Real>>>> {
    let mut unknown = false;
    match axis {
        CurveResultantParameter::First => {
            for row in &polynomial.coefficients {
                for coefficient in row.iter().skip(1) {
                    match real_sign(coefficient, policy) {
                        Some(RealSign::Zero) => {}
                        Some(RealSign::Positive | RealSign::Negative) => {
                            return Ok(Classification::Decided(None));
                        }
                        None => unknown = true,
                    }
                }
            }
            if unknown {
                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
            }
            Ok(Classification::Decided(Some(
                polynomial
                    .coefficients
                    .iter()
                    .map(|row| row.first().cloned().unwrap_or_else(Real::zero))
                    .collect(),
            )))
        }
        CurveResultantParameter::Second => {
            for row in polynomial.coefficients.iter().skip(1) {
                for coefficient in row {
                    match real_sign(coefficient, policy) {
                        Some(RealSign::Zero) => {}
                        Some(RealSign::Positive | RealSign::Negative) => {
                            return Ok(Classification::Decided(None));
                        }
                        None => unknown = true,
                    }
                }
            }
            if unknown {
                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
            }
            Ok(Classification::Decided(Some(
                polynomial.coefficients.first().cloned().unwrap_or_default(),
            )))
        }
    }
}

fn bivariate_substitute_second_equal_first(polynomial: &BivariatePolynomial) -> Vec<Real> {
    let degree = polynomial
        .coefficients
        .iter()
        .enumerate()
        .flat_map(|(first, row)| {
            row.iter()
                .enumerate()
                .map(move |(second, _)| first + second)
        })
        .max()
        .unwrap_or(0);
    let mut coefficients = vec![Real::zero(); degree + 1];
    for (first, row) in polynomial.coefficients.iter().enumerate() {
        for (second, coefficient) in row.iter().enumerate() {
            coefficients[first + second] += coefficient;
        }
    }
    coefficients
}

fn bivariate_substitute_second_equal_one_minus_first(
    polynomial: &BivariatePolynomial,
) -> Vec<Real> {
    let second_degree = polynomial
        .coefficients
        .iter()
        .map(|row| row.len().saturating_sub(1))
        .max()
        .unwrap_or(0);
    let mut complement_powers = Vec::with_capacity(second_degree + 1);
    complement_powers.push(vec![Real::one()]);
    for degree in 1..=second_degree {
        complement_powers.push(polynomial_multiply(
            &complement_powers[degree - 1],
            &[Real::one(), Real::from(-1_i8)],
        ));
    }
    let degree = polynomial.coefficients.len().saturating_sub(1) + second_degree;
    let mut coefficients = vec![Real::zero(); degree + 1];
    for (first, row) in polynomial.coefficients.iter().enumerate() {
        for (second, coefficient) in row.iter().enumerate() {
            for (power, factor) in complement_powers[second].iter().enumerate() {
                coefficients[first + power] += coefficient * factor;
            }
        }
    }
    coefficients
}

fn bivariate_parameter_difference(
    first_left: &[Real],
    second_left: &[Real],
    first_right: &[Real],
    second_right: &[Real],
) -> BivariatePolynomial {
    bivariate_subtract(
        &bivariate_outer_product(first_left, second_left),
        &bivariate_outer_product(first_right, second_right),
    )
}

fn bivariate_outer_product(first: &[Real], second: &[Real]) -> BivariatePolynomial {
    let mut coefficients = vec![vec![Real::zero(); second.len()]; first.len()];
    for (first_power, first_coefficient) in first.iter().enumerate() {
        for (second_power, second_coefficient) in second.iter().enumerate() {
            coefficients[first_power][second_power] = first_coefficient * second_coefficient;
        }
    }
    BivariatePolynomial::new(coefficients)
}

fn bivariate_add(first: &BivariatePolynomial, second: &BivariatePolynomial) -> BivariatePolynomial {
    bivariate_combine(first, second, false)
}

fn bivariate_subtract(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
) -> BivariatePolynomial {
    bivariate_combine(first, second, true)
}

fn bivariate_combine(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    subtract_second: bool,
) -> BivariatePolynomial {
    let first_count = first.coefficients.len().max(second.coefficients.len());
    let second_count = first
        .coefficients
        .iter()
        .chain(&second.coefficients)
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    let mut coefficients = vec![vec![Real::zero(); second_count]; first_count];
    for (target, source) in coefficients.iter_mut().zip(&first.coefficients) {
        for (target, source) in target.iter_mut().zip(source) {
            *target += source;
        }
    }
    for (target, source) in coefficients.iter_mut().zip(&second.coefficients) {
        for (target, source) in target.iter_mut().zip(source) {
            if subtract_second {
                *target -= source;
            } else {
                *target += source;
            }
        }
    }
    BivariatePolynomial::new(coefficients)
}

fn bivariate_multiply_first_parameter(
    polynomial: &BivariatePolynomial,
    factor: &[Real],
) -> BivariatePolynomial {
    let first_count = polynomial.coefficients.len() + factor.len() - 1;
    let second_count = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    let mut coefficients = vec![vec![Real::zero(); second_count]; first_count];
    for (first_power, row) in polynomial.coefficients.iter().enumerate() {
        for (factor_power, factor) in factor.iter().enumerate() {
            for (second_power, coefficient) in row.iter().enumerate() {
                coefficients[first_power + factor_power][second_power] += coefficient * factor;
            }
        }
    }
    BivariatePolynomial::new(coefficients)
}

fn bivariate_multiply(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
) -> BivariatePolynomial {
    let first_second_count = first.coefficients.iter().map(Vec::len).max().unwrap_or(0);
    let second_second_count = second.coefficients.iter().map(Vec::len).max().unwrap_or(0);
    let mut coefficients = vec![
        vec![Real::zero(); first_second_count + second_second_count - 1];
        first.coefficients.len() + second.coefficients.len() - 1
    ];
    for (first_power, first_row) in first.coefficients.iter().enumerate() {
        for (second_power, second_row) in second.coefficients.iter().enumerate() {
            for (first_column, first_coefficient) in first_row.iter().enumerate() {
                for (second_column, second_coefficient) in second_row.iter().enumerate() {
                    coefficients[first_power + second_power][first_column + second_column] +=
                        first_coefficient * second_coefficient;
                }
            }
        }
    }
    BivariatePolynomial::new(coefficients)
}

fn bivariate_scale(mut polynomial: BivariatePolynomial, scale: &Real) -> BivariatePolynomial {
    for coefficient in polynomial.coefficients.iter_mut().flatten() {
        *coefficient *= scale;
    }
    polynomial
}

fn common_unit_polynomial_roots(
    first: Vec<Real>,
    second: Vec<Real>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParallelIncidence2>> {
    let first = match polynomial_from_coefficients(first, policy)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let second = match polynomial_from_coefficients(second, policy)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let polynomial = match (first, second) {
        (None, None) => {
            return Ok(Classification::Decided(
                BezierParallelIncidence2::EntireCurve,
            ));
        }
        (Some(polynomial), None) | (None, Some(polynomial)) => polynomial,
        (Some(first), Some(second)) => match first.greatest_common_divisor(&second, policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => {
                return Ok(Classification::Decided(
                    BezierParallelIncidence2::Parameters(Vec::new()),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        },
    };
    Ok(match polynomial.isolate_unit_interval_roots(policy)? {
        Classification::Decided(parameters) => {
            Classification::Decided(BezierParallelIncidence2::Parameters(parameters))
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

fn polynomial_right_origin_sign(
    polynomial: &BezierParameterPolynomial,
    policy: &CurveContext,
) -> Classification<RealSign> {
    for coefficient in polynomial.coefficients() {
        match real_sign(coefficient, policy) {
            Some(RealSign::Zero) => {}
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => {
                return Classification::Decided(sign);
            }
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }
    unreachable!("a normalized Bezier parameter polynomial is nonzero")
}

const fn real_signs_are_opposite(first: RealSign, second: RealSign) -> bool {
    matches!(
        (first, second),
        (RealSign::Positive, RealSign::Negative) | (RealSign::Negative, RealSign::Positive)
    )
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
        ),
    }
}

fn signed_polynomial_on_isolating_interval(
    filter: &BezierParameterPolynomial,
    defining: &BezierParameterPolynomial,
    interval: &BezierParameterInterval,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    match defining.greatest_common_divisor(filter, policy)? {
        Classification::Decided(Some(common)) => {
            match common.root_count_in_interval(interval, policy)? {
                Classification::Decided(0) => {}
                Classification::Decided(1) => {
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                Classification::Decided(_) => {
                    return Err(CurveError::InvalidBezierAlgebraicParameter);
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    let mut interval = interval.clone();
    loop {
        match filter.root_count_in_interval(&interval, policy) {
            Ok(Classification::Decided(0)) => {
                return match real_sign(&filter.evaluate(interval.start()), policy) {
                    Some(sign @ (RealSign::Positive | RealSign::Negative)) => {
                        Ok(Classification::Decided(sign))
                    }
                    Some(RealSign::Zero) => Err(CurveError::InvalidBezierAlgebraicParameter),
                    None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                };
            }
            Ok(Classification::Decided(_)) | Err(CurveError::InvalidBezierAlgebraicParameter) => {}
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            Err(error) => return Err(error),
        }

        let midpoint = ((interval.start() + interval.end()) / Real::from(2_i8))?;
        match real_sign(&defining.evaluate(&midpoint), policy) {
            Some(RealSign::Zero) => {
                return match real_sign(&filter.evaluate(&midpoint), policy) {
                    Some(sign) => Ok(Classification::Decided(sign)),
                    None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                };
            }
            Some(RealSign::Positive | RealSign::Negative) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let left = match BezierParameterInterval::try_new(
            interval.start().clone(),
            midpoint.clone(),
            policy,
        )? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let right =
            match BezierParameterInterval::try_new(midpoint, interval.end().clone(), policy)? {
                Classification::Decided(interval) => interval,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let left_count = match defining.root_count_in_interval(&left, policy)? {
            Classification::Decided(count) => count,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        interval = if left_count == 1 {
            left
        } else {
            let right_count = match defining.root_count_in_interval(&right, policy)? {
                Classification::Decided(count) => count,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if right_count != 1 {
                return Err(CurveError::InvalidBezierAlgebraicParameter);
            }
            right
        };
    }
}

#[cfg(test)]
mod conversion_tests {
    use super::*;

    #[cfg(feature = "predicates")]
    fn algebraic_parameter(coefficients: Vec<Real>) -> BezierParameter2 {
        let polynomial = match BezierParameterPolynomial::try_new_power_basis(
            coefficients,
            &CurveContext::STRICT,
        )
        .unwrap()
        {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => panic!("parameter polynomial: {reason:?}"),
        };
        let parameters = match polynomial
            .isolate_unit_interval_roots(&CurveContext::STRICT)
            .unwrap()
        {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => panic!("parameter isolation: {reason:?}"),
        };
        let [parameter] = parameters.as_slice() else {
            panic!("expected one unit-interval algebraic parameter");
        };
        assert!(matches!(parameter, BezierParameter2::Algebraic(_)));
        parameter.clone()
    }

    #[test]
    fn exact_parallel_is_one_word_and_clones_share_lazy_differential() {
        let source = QuadraticBezier2::new(
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(Real::one(), Real::one()),
            Point2::new(Real::from(2_i8), Real::zero()),
        );
        let parallel = source.parallel_left(Real::one()).unwrap();
        let clone = parallel.clone();

        assert_eq!(
            std::mem::size_of::<BezierParallel2>(),
            std::mem::size_of::<usize>()
        );
        assert!(Arc::ptr_eq(&parallel.data, &clone.data));
        assert!(parallel.data.differential.get().is_none());
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        assert!(matches!(
            clone.point_at(&half, &CurveContext::STRICT).unwrap(),
            Classification::Decided(_)
        ));
        assert!(parallel.data.differential.get().is_some());
    }

    fn parameter_lift_map(
        numerator: Vec<Real>,
        denominator: Vec<Real>,
    ) -> CurveIntersectionParameterLiftMap {
        CurveIntersectionParameterLiftMap {
            cofactor_row: 0,
            numerator_coefficients: numerator,
            denominator_coefficients: denominator,
        }
    }

    #[test]
    fn rational_component_domain_clips_increasing_and_decreasing_maps_exactly() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let increasing = parameter_lift_map(
                vec![Real::from(-1_i8), Real::from(4_i8)],
                vec![Real::from(2_i8)],
            );
            let derivative = polynomial_subtract(
                &polynomial_multiply(
                    &polynomial_derivative(&increasing.numerator_coefficients),
                    &increasing.denominator_coefficients,
                ),
                &polynomial_multiply(
                    &increasing.numerator_coefficients,
                    &polynomial_derivative(&increasing.denominator_coefficients),
                ),
            );
            let Classification::Decided(Some(increasing)) =
                rational_parameter_component_domains(&increasing, &derivative, &policy).unwrap()
            else {
                panic!("increasing rational map did not clip");
            };
            assert!(increasing.isolated_points.is_empty());
            let [(increasing, std::cmp::Ordering::Less)] = increasing.domains.as_slice() else {
                panic!("increasing map did not produce one increasing domain");
            };
            assert_eq!(
                increasing.retained_start.as_exact(),
                Some(&(Real::one() / Real::from(4_i8)).unwrap())
            );
            assert_eq!(
                increasing.retained_end.as_exact(),
                Some(&(Real::from(3_i8) / Real::from(4_i8)).unwrap())
            );
            assert_eq!(increasing.lifted_start.as_exact(), Some(&Real::zero()));
            assert_eq!(increasing.lifted_end.as_exact(), Some(&Real::one()));

            let decreasing = parameter_lift_map(
                vec![Real::from(3_i8), Real::from(-4_i8)],
                vec![Real::from(2_i8)],
            );
            let derivative = polynomial_subtract(
                &polynomial_multiply(
                    &polynomial_derivative(&decreasing.numerator_coefficients),
                    &decreasing.denominator_coefficients,
                ),
                &polynomial_multiply(
                    &decreasing.numerator_coefficients,
                    &polynomial_derivative(&decreasing.denominator_coefficients),
                ),
            );
            let Classification::Decided(Some(decreasing)) =
                rational_parameter_component_domains(&decreasing, &derivative, &policy).unwrap()
            else {
                panic!("decreasing rational map did not clip");
            };
            assert!(decreasing.isolated_points.is_empty());
            let [(decreasing, std::cmp::Ordering::Greater)] = decreasing.domains.as_slice() else {
                panic!("decreasing map did not produce one decreasing domain");
            };
            assert_eq!(
                decreasing.retained_start.as_exact(),
                Some(&(Real::one() / Real::from(4_i8)).unwrap())
            );
            assert_eq!(
                decreasing.retained_end.as_exact(),
                Some(&(Real::from(3_i8) / Real::from(4_i8)).unwrap())
            );
            assert_eq!(decreasing.lifted_start.as_exact(), Some(&Real::one()));
            assert_eq!(decreasing.lifted_end.as_exact(), Some(&Real::zero()));
        }
    }

    #[test]
    fn rational_component_monotonicity_allows_stationary_points_without_sign_reversal() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // 8*((s-1/2)^3+1/2) = 3+6s-12s^2+8s^3 has a
            // positive derivative with one double zero at s=1/2.
            let map = parameter_lift_map(
                vec![
                    Real::from(3_i8),
                    Real::from(6_i8),
                    Real::from(-12_i8),
                    Real::from(8_i8),
                ],
                vec![Real::from(8_i8)],
            );
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
            let Classification::Decided(Some(domains)) =
                rational_parameter_component_domains(&map, &derivative_numerator, &policy).unwrap()
            else {
                panic!("weakly monotone full-domain map was rejected");
            };
            assert!(domains.isolated_points.is_empty());
            let [(domain, std::cmp::Ordering::Less)] = domains.domains.as_slice() else {
                panic!("stationary increasing map was not merged into one domain");
            };
            assert_eq!(domain.retained_start.as_exact(), Some(&Real::zero()));
            assert_eq!(domain.retained_end.as_exact(), Some(&Real::one()));
        }
    }

    #[test]
    fn rational_component_domains_split_noninjective_and_pole_separated_maps() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let noninjective = parameter_lift_map(
                vec![Real::zero(), Real::from(4_i8), Real::from(-4_i8)],
                vec![Real::one()],
            );
            let derivative = polynomial_subtract(
                &polynomial_multiply(
                    &polynomial_derivative(&noninjective.numerator_coefficients),
                    &noninjective.denominator_coefficients,
                ),
                &polynomial_multiply(
                    &noninjective.numerator_coefficients,
                    &polynomial_derivative(&noninjective.denominator_coefficients),
                ),
            );
            let Classification::Decided(Some(domains)) =
                rational_parameter_component_domains(&noninjective, &derivative, &policy).unwrap()
            else {
                panic!("noninjective finite map did not partition");
            };
            let [
                (ascending, std::cmp::Ordering::Less),
                (descending, std::cmp::Ordering::Greater),
            ] = domains.domains.as_slice()
            else {
                panic!("noninjective map did not produce two oriented domains");
            };
            let half = (Real::one() / Real::from(2_i8)).unwrap();
            assert_eq!(ascending.retained_start.as_exact(), Some(&Real::zero()));
            assert_eq!(ascending.retained_end.as_exact(), Some(&half));
            assert_eq!(ascending.lifted_start.as_exact(), Some(&Real::zero()));
            assert_eq!(ascending.lifted_end.as_exact(), Some(&Real::one()));
            assert_eq!(descending.retained_start.as_exact(), Some(&half));
            assert_eq!(descending.retained_end.as_exact(), Some(&Real::one()));
            assert_eq!(descending.lifted_start.as_exact(), Some(&Real::one()));
            assert_eq!(descending.lifted_end.as_exact(), Some(&Real::zero()));

            let quarter = (Real::one() / Real::from(4_i8)).unwrap();
            let pole_split = parameter_lift_map(
                vec![-quarter.clone(), Real::one()],
                vec![Real::from(-1_i8), Real::from(2_i8)],
            );
            let derivative = polynomial_subtract(
                &polynomial_multiply(
                    &polynomial_derivative(&pole_split.numerator_coefficients),
                    &pole_split.denominator_coefficients,
                ),
                &polynomial_multiply(
                    &pole_split.numerator_coefficients,
                    &polynomial_derivative(&pole_split.denominator_coefficients),
                ),
            );
            let Classification::Decided(Some(domains)) =
                rational_parameter_component_domains(&pole_split, &derivative, &policy).unwrap()
            else {
                panic!("pole-separated finite branches did not partition");
            };
            let [
                (first, std::cmp::Ordering::Greater),
                (second, std::cmp::Ordering::Greater),
            ] = domains.domains.as_slice()
            else {
                panic!("pole-separated map did not retain both finite branches");
            };
            let three_quarters = (Real::from(3_i8) / Real::from(4_i8)).unwrap();
            assert_eq!(first.retained_start.as_exact(), Some(&Real::zero()));
            assert_eq!(first.retained_end.as_exact(), Some(&quarter));
            assert_eq!(first.lifted_start.as_exact(), Some(&quarter));
            assert_eq!(first.lifted_end.as_exact(), Some(&Real::zero()));
            assert_eq!(second.retained_start.as_exact(), Some(&three_quarters));
            assert_eq!(second.retained_end.as_exact(), Some(&Real::one()));
            assert_eq!(second.lifted_start.as_exact(), Some(&Real::one()));
            assert_eq!(second.lifted_end.as_exact(), Some(&three_quarters));
        }
    }

    #[test]
    fn rational_component_domains_retain_isolated_closed_square_touches() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let tangent_touch = parameter_lift_map(
                vec![Real::from(-1_i8), Real::from(4_i8), Real::from(-4_i8)],
                vec![Real::one()],
            );
            let derivative = polynomial_subtract(
                &polynomial_multiply(
                    &polynomial_derivative(&tangent_touch.numerator_coefficients),
                    &tangent_touch.denominator_coefficients,
                ),
                &polynomial_multiply(
                    &tangent_touch.numerator_coefficients,
                    &polynomial_derivative(&tangent_touch.denominator_coefficients),
                ),
            );
            let Classification::Decided(Some(partition)) =
                rational_parameter_component_domains(&tangent_touch, &derivative, &policy).unwrap()
            else {
                panic!("isolated closed-square component touch was not partitioned");
            };
            assert!(partition.domains.is_empty());
            let [point] = partition.isolated_points.as_slice() else {
                panic!("isolated component touch was not retained exactly once");
            };
            assert_eq!(
                point.retained_parameter.as_exact(),
                Some(&(Real::one() / Real::from(2_i8)).unwrap())
            );
            assert_eq!(point.lifted_parameter.as_exact(), Some(&Real::zero()));
        }
    }

    #[test]
    fn rational_component_range_checks_reject_only_relevant_roots() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let quarter = BezierParameter2::Exact((Real::one() / Real::from(4_i8)).unwrap());
            let three_quarters =
                BezierParameter2::Exact((Real::from(3_i8) / Real::from(4_i8)).unwrap());
            assert_eq!(
                polynomial_is_rootless_on_parameter_range(
                    &[Real::from(-1_i8), Real::from(10_i8)],
                    &quarter,
                    &three_quarters,
                    &policy,
                )
                .unwrap(),
                Classification::Decided(true)
            );
            assert_eq!(
                polynomial_is_rootless_on_parameter_range(
                    &[Real::from(-1_i8), Real::from(2_i8)],
                    &quarter,
                    &three_quarters,
                    &policy,
                )
                .unwrap(),
                Classification::Decided(false)
            );
            let zero = BezierParameter2::Exact(Real::zero());
            let one = BezierParameter2::Exact(Real::one());
            assert_eq!(
                polynomial_is_rootless_on_parameter_range(
                    &[Real::from(-1_i8), Real::from(2_i8)],
                    &zero,
                    &one,
                    &policy,
                )
                .unwrap(),
                Classification::Decided(false)
            );
        }
    }

    #[test]
    fn rational_component_certificate_replays_equations_branch_and_poles() {
        let identity_equation = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::one()],
            vec![Real::from(-1_i8)],
        ]);
        let identity = parameter_lift_map(vec![Real::zero(), Real::one()], vec![Real::one()]);
        let selected_branch = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let opposite_branch = BivariatePolynomial::new(vec![vec![Real::from(-1_i8)]]);
        let vanishing_branch =
            BivariatePolynomial::new(vec![vec![Real::from(-1_i8)], vec![Real::from(2_i8)]]);
        let pole_equation = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::one()],
            vec![Real::from(-1_i8), Real::from(-2_i8)],
        ]);
        let pole_map = parameter_lift_map(
            vec![Real::zero(), Real::one()],
            vec![Real::one(), Real::from(-2_i8)],
        );

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let equations = [identity_equation.clone(), identity_equation.clone()];
            let Classification::Decided(Some(evidence)) = certify_rational_parameter_component_map(
                &equations,
                &selected_branch,
                CurveResultantParameter::First,
                &identity,
                &policy,
            )
            .unwrap() else {
                panic!("selected identity component was not certified");
            };
            assert!(evidence.isolated_pairs.is_empty());
            let [overlap] = evidence.overlaps.as_ref() else {
                panic!("identity component did not produce exactly one overlap");
            };
            assert_eq!(
                overlap.first_range().exact_endpoints(),
                Some((&Real::zero(), &Real::one()))
            );
            assert_eq!(
                overlap.second_range().exact_endpoints(),
                Some((&Real::zero(), &Real::one()))
            );
            let Classification::Decided(Some(opposite)) = certify_rational_parameter_component_map(
                &equations,
                &opposite_branch,
                CurveResultantParameter::First,
                &identity,
                &policy,
            )
            .unwrap() else {
                panic!("opposite component was not decided");
            };
            assert!(opposite.overlaps.is_empty());
            assert!(opposite.isolated_pairs.is_empty());
            assert!(matches!(
                certify_rational_parameter_component_map(
                    &equations,
                    &vanishing_branch,
                    CurveResultantParameter::First,
                    &identity,
                    &policy,
                )
                .unwrap(),
                Classification::Decided(None)
            ));

            let pole_equations = [pole_equation.clone(), pole_equation.clone()];
            let Classification::Decided(Some(evidence)) = certify_rational_parameter_component_map(
                &pole_equations,
                &selected_branch,
                CurveResultantParameter::First,
                &pole_map,
                &policy,
            )
            .unwrap() else {
                panic!("finite component domain before a pole was not retained");
            };
            assert!(evidence.isolated_pairs.is_empty());
            let [overlap] = evidence.overlaps.as_ref() else {
                panic!("pole-split map did not produce one finite overlap");
            };
            assert_eq!(
                overlap.first_range().exact_endpoints(),
                Some((&Real::zero(), &(Real::one() / Real::from(3_i8)).unwrap()))
            );
            assert_eq!(
                overlap.second_range().exact_endpoints(),
                Some((&Real::zero(), &Real::one()))
            );
        }
    }

    #[test]
    fn rational_component_system_retains_residual_isolated_candidates() {
        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        let three_quarters = (Real::from(3_i8) / Real::from(4_i8)).unwrap();
        let component = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::from(-1_i8)],
            vec![Real::one()],
        ]);
        let first_residual =
            BivariatePolynomial::new(vec![vec![-quarter.clone()], vec![Real::one()]]);
        let second_residual =
            BivariatePolynomial::new(vec![vec![-three_quarters.clone(), Real::one()]]);
        let equations = [
            bivariate_multiply(&component, &first_residual),
            bivariate_multiply(&component, &second_residual),
        ];
        let branch = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(Some(system)) =
                parameter_component_system(&equations, &branch, &policy, config).unwrap()
            else {
                panic!("shared component plus residual point was not decomposed");
            };
            assert_eq!(system.overlaps.len(), 1);
            let Classification::Decided(BezierParallelIntersectionCandidates2::Candidates {
                parallel_parameters,
                other_parameters,
            }) = project_parallel_intersection_system(
                &system.residual_equations[0],
                &system.residual_equations[1],
                &policy,
            )
            .unwrap()
            else {
                panic!("residual isolated solution was discarded");
            };
            assert_eq!(
                parallel_parameters,
                vec![BezierParameter2::Exact(quarter.clone())]
            );
            assert_eq!(
                other_parameters,
                vec![BezierParameter2::Exact(three_quarters.clone())]
            );
        }
    }

    #[test]
    fn rational_component_system_enumerates_split_quadratic_components() {
        let first_component = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::one()],
            vec![Real::from(-1_i8)],
        ]);
        let second_component = BivariatePolynomial::new(vec![
            vec![Real::from(-1_i8), Real::one()],
            vec![Real::one()],
        ]);
        let common = bivariate_multiply(&first_component, &second_component);
        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let first_residual =
            BivariatePolynomial::new(vec![vec![-quarter.clone()], vec![Real::one()]]);
        let second_residual = BivariatePolynomial::new(vec![vec![-half.clone(), Real::one()]]);
        let equations = [
            bivariate_multiply(&common, &first_residual),
            bivariate_multiply(&common, &second_residual),
        ];
        let branch = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(Some(system)) =
                parameter_component_system(&equations, &branch, &policy, config).unwrap()
            else {
                panic!("two rational components were not enumerated");
            };
            assert_eq!(system.overlaps.len(), 2);
            assert!(system.isolated_pairs.is_empty());
            assert!(system.overlaps.iter().any(|overlap| {
                overlap.orientation() == RationalBezierOverlapOrientation2::Same
            }));
            assert!(system.overlaps.iter().any(|overlap| {
                overlap.orientation() == RationalBezierOverlapOrientation2::Reversed
            }));
            let Classification::Decided(candidate_system) =
                parallel_candidate_system_from_parameter_components(system, &policy).unwrap()
            else {
                panic!("multiple component residual projection was uncertain");
            };
            assert_eq!(candidate_system.overlaps.len(), 2);
            let BezierParallelIntersectionCandidates2::Candidates {
                parallel_parameters,
                other_parameters,
            } = candidate_system.candidates
            else {
                panic!("isolated residual beside two components was discarded");
            };
            assert_eq!(
                parallel_parameters,
                vec![BezierParameter2::Exact(quarter.clone())]
            );
            assert_eq!(
                other_parameters,
                vec![BezierParameter2::Exact(half.clone())]
            );
        }
    }

    #[test]
    fn rational_component_system_deduplicates_repeated_components() {
        let component = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::one()],
            vec![Real::from(-1_i8)],
        ]);
        let repeated = bivariate_multiply(&component, &component);
        let equations = [
            bivariate_multiply(
                &repeated,
                &BivariatePolynomial::new(vec![vec![Real::from(2_i8)], vec![Real::one()]]),
            ),
            bivariate_multiply(
                &repeated,
                &BivariatePolynomial::new(vec![vec![Real::from(2_i8), Real::one()]]),
            ),
        ];
        let branch = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(Some(system)) =
                parameter_component_system(&equations, &branch, &policy, config).unwrap()
            else {
                panic!("repeated rational component was not extracted");
            };
            assert_eq!(system.overlaps.len(), 1);
            assert!(system.isolated_pairs.is_empty());
        }
    }

    #[test]
    fn rational_component_system_enumerates_repeated_cubic_fibers() {
        let repeated = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::one()],
            vec![Real::from(-1_i8)],
        ]);
        let distinct = BivariatePolynomial::new(vec![
            vec![Real::from(-1_i8), Real::one()],
            vec![Real::one()],
        ]);
        let common = bivariate_multiply(&bivariate_multiply(&repeated, &repeated), &distinct);
        let equations = [
            bivariate_multiply(
                &common,
                &BivariatePolynomial::new(vec![vec![Real::from(2_i8)], vec![Real::one()]]),
            ),
            bivariate_multiply(
                &common,
                &BivariatePolynomial::new(vec![vec![Real::from(2_i8), Real::one()]]),
            ),
        ];
        let branch = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(Some(system)) =
                parameter_component_system(&equations, &branch, &policy, config).unwrap()
            else {
                panic!("repeated cubic common fiber was not enumerated");
            };
            assert_eq!(system.overlaps.len(), 2);
            assert!(system.isolated_pairs.is_empty());
            assert!(system.overlaps.iter().any(|overlap| {
                overlap.orientation() == RationalBezierOverlapOrientation2::Same
            }));
            assert!(system.overlaps.iter().any(|overlap| {
                overlap.orientation() == RationalBezierOverlapOrientation2::Reversed
            }));
        }
    }

    #[test]
    fn rational_component_system_deduplicates_triple_cubic_fibers() {
        let component = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::one()],
            vec![Real::from(-1_i8)],
        ]);
        let common = bivariate_multiply(&bivariate_multiply(&component, &component), &component);
        let equations = [
            bivariate_multiply(
                &common,
                &BivariatePolynomial::new(vec![vec![Real::from(2_i8)], vec![Real::one()]]),
            ),
            bivariate_multiply(
                &common,
                &BivariatePolynomial::new(vec![vec![Real::from(2_i8), Real::one()]]),
            ),
        ];
        let branch = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(Some(system)) =
                parameter_component_system(&equations, &branch, &policy, config).unwrap()
            else {
                panic!("triple cubic common fiber was not enumerated");
            };
            assert_eq!(system.overlaps.len(), 1);
            assert!(system.isolated_pairs.is_empty());
            assert_eq!(
                system.overlaps[0].orientation(),
                RationalBezierOverlapOrientation2::Same
            );
        }
    }

    #[test]
    fn implicit_parameter_component_remains_an_explicit_boundary() {
        let component = BivariatePolynomial::new(vec![
            vec![Real::from(-1_i8), Real::zero(), Real::one()],
            vec![Real::zero()],
            vec![Real::one()],
        ]);
        let equations = [
            bivariate_multiply(
                &component,
                &BivariatePolynomial::new(vec![vec![Real::from(2_i8)], vec![Real::one()]]),
            ),
            bivariate_multiply(
                &component,
                &BivariatePolynomial::new(vec![vec![Real::from(2_i8), Real::one()]]),
            ),
        ];
        let branch = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            assert!(matches!(
                parameter_component_system(&equations, &branch, &policy, config).unwrap(),
                Classification::Uncertain(UncertaintyReason::Boundary)
            ));
        }
    }

    #[test]
    fn regular_implicit_parameter_components_transport_monotone_graphs() {
        let components = [
            (
                BivariatePolynomial::new(vec![
                    vec![Real::zero(), Real::from(2_i8), Real::from(2_i8)],
                    vec![Real::from(-3_i8)],
                    vec![Real::from(-1_i8)],
                ]),
                RationalBezierOverlapOrientation2::Same,
            ),
            (
                BivariatePolynomial::new(vec![
                    vec![Real::from(4_i8), Real::from(-6_i8), Real::from(2_i8)],
                    vec![Real::from(-3_i8)],
                    vec![Real::from(-1_i8)],
                ]),
                RationalBezierOverlapOrientation2::Reversed,
            ),
            (
                BivariatePolynomial::new(vec![
                    vec![Real::zero(), Real::one(), Real::zero(), Real::one()],
                    vec![Real::from(-1_i8)],
                    vec![Real::from(-1_i8)],
                ]),
                RationalBezierOverlapOrientation2::Same,
            ),
            (
                BivariatePolynomial::new(vec![
                    vec![
                        Real::zero(),
                        Real::one(),
                        Real::zero(),
                        Real::zero(),
                        Real::one(),
                    ],
                    vec![Real::from(-1_i8)],
                    vec![Real::zero()],
                    vec![Real::from(-1_i8)],
                ]),
                RationalBezierOverlapOrientation2::Same,
            ),
        ];
        let branch = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (component, expected_orientation) in &components {
                let equations = [
                    component.clone(),
                    bivariate_scale(component.clone(), &Real::from(2_i8)),
                ];
                let Classification::Decided(Some(system)) =
                    parameter_component_system(&equations, &branch, &policy, config).unwrap()
                else {
                    panic!("regular implicit graph was not transported");
                };
                let [overlap] = system.overlaps.as_ref() else {
                    panic!("one regular implicit graph must produce one overlap");
                };
                assert_eq!(overlap.orientation(), *expected_orientation);
                assert_eq!(
                    overlap.first_range().start().as_exact(),
                    Some(&Real::zero())
                );
                assert_eq!(overlap.first_range().end().as_exact(), Some(&Real::one()));
                match expected_orientation {
                    RationalBezierOverlapOrientation2::Same => {
                        assert_eq!(
                            overlap.second_range().start().as_exact(),
                            Some(&Real::zero())
                        );
                        assert_eq!(overlap.second_range().end().as_exact(), Some(&Real::one()));
                    }
                    RationalBezierOverlapOrientation2::Reversed => {
                        assert_eq!(
                            overlap.second_range().start().as_exact(),
                            Some(&Real::one())
                        );
                        assert_eq!(overlap.second_range().end().as_exact(), Some(&Real::zero()));
                    }
                }
            }
        }
    }

    #[test]
    fn regular_implicit_parameter_component_rejects_a_branch_zero() {
        let component = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::from(2_i8), Real::from(2_i8)],
            vec![Real::from(-3_i8)],
            vec![Real::from(-1_i8)],
        ]);
        let equations = [
            component.clone(),
            bivariate_scale(component, &Real::from(2_i8)),
        ];
        let branch = BivariatePolynomial::new(vec![vec![
            (Real::from(-1_i8) / Real::from(2_i8)).unwrap(),
            Real::one(),
        ]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            assert!(matches!(
                parameter_component_system(&equations, &branch, &policy, config).unwrap(),
                Classification::Uncertain(UncertaintyReason::Boundary)
            ));
        }
    }

    #[test]
    fn regular_implicit_parameter_component_discards_the_opposite_branch() {
        let component = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::from(2_i8), Real::from(2_i8)],
            vec![Real::from(-3_i8)],
            vec![Real::from(-1_i8)],
        ]);
        let equations = [
            component.clone(),
            bivariate_scale(component, &Real::from(2_i8)),
        ];
        let branch = BivariatePolynomial::new(vec![vec![Real::from(-1_i8)]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(Some(system)) =
                parameter_component_system(&equations, &branch, &policy, config).unwrap()
            else {
                panic!("opposite regular implicit graph was not decided");
            };
            assert!(system.overlaps.is_empty());
            assert!(system.isolated_pairs.is_empty());
        }
    }

    #[test]
    fn regular_implicit_component_excludes_off_component_critical_points() {
        // H=16u^2-8u-3-2t-3t^2 has one unit-square branch from
        // (0,3/4) to (1,1). H_u vanishes on u=1/4, but H does not vanish
        // there; the exact two-equation fallback must distinguish that from a
        // critical point on the transported component.
        let component = BivariatePolynomial::new(vec![
            vec![Real::from(-3_i8), Real::from(-8_i8), Real::from(16_i8)],
            vec![Real::from(-2_i8)],
            vec![Real::from(-3_i8)],
        ]);
        let equations = [
            component.clone(),
            bivariate_scale(component, &Real::from(2_i8)),
        ];
        let branch = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(Some(system)) =
                parameter_component_system(&equations, &branch, &policy, config).unwrap()
            else {
                panic!("off-component derivative zero blocked a regular graph");
            };
            let [overlap] = system.overlaps.as_ref() else {
                panic!("one regular implicit graph must produce one overlap");
            };
            assert_eq!(
                overlap.first_range().exact_endpoints(),
                Some((&Real::zero(), &Real::one()))
            );
            let three_quarters = (Real::from(3_i8) / Real::from(4_i8)).unwrap();
            assert_eq!(
                overlap.second_range().exact_endpoints(),
                Some((&three_quarters, &Real::one()))
            );
        }
    }

    #[test]
    fn regular_implicit_component_retains_residual_isolated_candidates() {
        let component = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::from(2_i8), Real::from(2_i8)],
            vec![Real::from(-3_i8)],
            vec![Real::from(-1_i8)],
        ]);
        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        let three_quarters = (Real::from(3_i8) / Real::from(4_i8)).unwrap();
        let equations = [
            bivariate_multiply(
                &component,
                &BivariatePolynomial::new(vec![vec![-quarter.clone()], vec![Real::one()]]),
            ),
            bivariate_multiply(
                &component,
                &BivariatePolynomial::new(vec![vec![-three_quarters.clone(), Real::one()]]),
            ),
        ];
        let branch = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(Some(system)) =
                parameter_component_system(&equations, &branch, &policy, config).unwrap()
            else {
                panic!("implicit component plus isolated residual was not decomposed");
            };
            assert_eq!(system.overlaps.len(), 1);
            let Classification::Decided(BezierParallelIntersectionCandidates2::Candidates {
                parallel_parameters,
                other_parameters,
            }) = project_parallel_intersection_system(
                &system.residual_equations[0],
                &system.residual_equations[1],
                &policy,
            )
            .unwrap()
            else {
                panic!("residual isolated solution was discarded");
            };
            assert_eq!(
                parallel_parameters,
                vec![BezierParameter2::Exact(quarter.clone())]
            );
            assert_eq!(
                other_parameters,
                vec![BezierParameter2::Exact(three_quarters.clone())]
            );
        }
    }

    #[test]
    fn rational_component_system_filters_split_components_independently() {
        // u=t/4 lies wholly on the negative branch of 2u-1, while
        // u=1-t/4 lies wholly on its positive branch.
        let opposite = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::from(4_i8)],
            vec![Real::from(-1_i8)],
        ]);
        let selected = BivariatePolynomial::new(vec![
            vec![Real::from(-4_i8), Real::from(4_i8)],
            vec![Real::one()],
        ]);
        let common = bivariate_multiply(&opposite, &selected);
        let equations = [
            bivariate_multiply(
                &common,
                &BivariatePolynomial::new(vec![vec![Real::from(2_i8)], vec![Real::one()]]),
            ),
            bivariate_multiply(
                &common,
                &BivariatePolynomial::new(vec![vec![Real::from(2_i8), Real::one()]]),
            ),
        ];
        let branch = BivariatePolynomial::new(vec![vec![Real::from(-1_i8), Real::from(2_i8)]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(Some(system)) =
                parameter_component_system(&equations, &branch, &policy, config).unwrap()
            else {
                panic!("split selected and opposite components were not decided");
            };
            let [overlap] = system.overlaps.as_ref() else {
                panic!("only the selected component should survive branch replay");
            };
            assert_eq!(
                overlap.orientation(),
                RationalBezierOverlapOrientation2::Reversed
            );
            assert_eq!(
                overlap.second_range().exact_endpoints(),
                Some((
                    &Real::one(),
                    &(Real::from(3_i8) / Real::from(4_i8)).unwrap()
                ))
            );
        }
    }

    #[test]
    fn bivariate_bernstein_sign_excludes_only_strict_unit_square_misses() {
        let positive = BivariatePolynomial::new(vec![
            vec![Real::one(), Real::one()],
            vec![Real::one(), Real::one()],
        ]);
        let negative = bivariate_scale(positive.clone(), &Real::from(-1_i8));
        let boundary_zero =
            BivariatePolynomial::new(vec![vec![Real::zero(), Real::one()], vec![Real::one()]]);
        let sign_change = BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::from(-1_i8)],
            vec![Real::one()],
        ]);

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            assert!(bivariate_unit_square_has_strict_bernstein_sign(&positive, &policy).unwrap());
            assert!(bivariate_unit_square_has_strict_bernstein_sign(&negative, &policy).unwrap());
            assert!(
                !bivariate_unit_square_has_strict_bernstein_sign(&boundary_zero, &policy).unwrap()
            );
            assert!(
                !bivariate_unit_square_has_strict_bernstein_sign(&sign_change, &policy).unwrap()
            );
        }
    }

    #[test]
    fn rational_component_probe_is_not_an_authoritative_negative() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        // D=(t-2)(t-3)(t-5) makes every fixed fast probe see the genuine
        // component D(t)u-N(t) collapse to a nonzero constant. On [0,1],
        // N/D=t/2-1/D is finite, increasing, and lies strictly inside [0,1].
        let component = BivariatePolynomial::new(vec![
            vec![Real::one(), Real::from(-30_i8)],
            vec![Real::from(15_i8), Real::from(31_i8)],
            vec![Real::from(-31_i8) * half.clone(), Real::from(-10_i8)],
            vec![Real::from(5_i8), Real::one()],
            vec![-half, Real::zero()],
        ]);
        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        let three_quarters = (Real::from(3_i8) / Real::from(4_i8)).unwrap();
        let first_residual = BivariatePolynomial::new(vec![vec![-quarter], vec![Real::one()]]);
        let second_residual = BivariatePolynomial::new(vec![vec![-three_quarters, Real::one()]]);
        let equations = [
            bivariate_multiply(&component, &first_residual),
            bivariate_multiply(&component, &second_residual),
        ];
        assert!(!bivariate_system_may_have_component(&equations));
        let branch = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(Some(system)) =
                parameter_component_system(&equations, &branch, &policy, config).unwrap()
            else {
                panic!("exact component extraction trusted a speculative negative probe");
            };
            let [overlap] = system.overlaps.as_ref() else {
                panic!("finite component hidden from the fixed probe was not retained");
            };
            assert_eq!(
                overlap.first_range().exact_endpoints(),
                Some((&Real::zero(), &Real::one()))
            );
            assert_eq!(
                overlap.second_range().exact_endpoints(),
                Some((
                    &(Real::one() / Real::from(30_i8)).unwrap(),
                    &(Real::from(5_i8) / Real::from(8_i8)).unwrap()
                ))
            );
        }
    }

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

    #[test]
    fn polynomial_square_root_preserves_even_origin_valuation() {
        let coefficients = [
            Real::zero(),
            Real::zero(),
            Real::one(),
            Real::from(2_i8),
            Real::one(),
        ];
        assert_eq!(
            polynomial_square_root(&coefficients, &CurveContext::STRICT).unwrap(),
            Classification::Decided(Some(vec![Real::zero(), Real::one(), Real::one()]))
        );
    }

    #[test]
    fn polynomial_square_root_rejects_odd_origin_valuation() {
        assert_eq!(
            polynomial_square_root(
                &[Real::zero(), Real::one(), Real::one()],
                &CurveContext::STRICT,
            )
            .unwrap(),
            Classification::Decided(None)
        );
    }

    #[test]
    fn axis_saturation_requires_every_removed_factor_to_be_rootless() {
        let rootless = [
            BivariatePolynomial::new(vec![
                vec![Real::from(2_i8), Real::from(2_i8)],
                vec![Real::one(), Real::one()],
            ]),
            BivariatePolynomial::new(vec![
                vec![Real::from(4_i8), Real::from(2_i8)],
                vec![Real::from(2_i8), Real::one()],
            ]),
        ];
        let rootful = [
            BivariatePolynomial::new(vec![
                vec![Real::from(-1_i8), Real::from(-1_i8)],
                vec![Real::from(2_i8), Real::from(2_i8)],
            ]),
            BivariatePolynomial::new(vec![
                vec![Real::from(-2_i8), Real::from(-1_i8)],
                vec![Real::from(4_i8), Real::from(2_i8)],
            ]),
        ];
        let expected = [
            BivariatePolynomial::new(vec![vec![Real::one(), Real::one()]]),
            BivariatePolynomial::new(vec![vec![Real::from(2_i8), Real::one()]]),
        ];

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            assert_eq!(
                rootless_axis_primitive_system(&rootless, &policy).unwrap(),
                Some(expected.clone())
            );
            assert_eq!(
                rootless_axis_primitive_system(&rootful, &policy).unwrap(),
                None
            );
        }
    }

    #[test]
    #[cfg(feature = "predicates")]
    fn specialized_common_fiber_counts_even_multiplicity_in_a_coupled_system() {
        // alpha = cbrt(1/2), beta = alpha^2 = cbrt(1/4). The two equations
        // differ by A(alpha)=2*alpha^3-1 and specialize to
        // (beta-alpha^2)^2, so the selected fiber root has even multiplicity
        // and cannot be recovered from endpoint sign change.
        let alpha = algebraic_parameter(vec![
            Real::from(-1_i8),
            Real::zero(),
            Real::zero(),
            Real::from(2_i8),
        ]);
        let beta = algebraic_parameter(vec![
            Real::from(-1_i8),
            Real::zero(),
            Real::zero(),
            Real::from(4_i8),
        ]);
        let first = BivariatePolynomial::new(vec![
            vec![Real::from(-1_i8), Real::zero(), Real::one()],
            vec![],
            vec![Real::zero(), Real::from(-2_i8)],
            vec![Real::from(2_i8)],
            vec![Real::one()],
        ]);
        let second = BivariatePolynomial::new(vec![
            vec![Real::from(-2_i8), Real::zero(), Real::one()],
            vec![],
            vec![Real::zero(), Real::from(-2_i8)],
            vec![Real::from(4_i8)],
            vec![Real::one()],
        ]);
        let config = CurveIntersectionResultantConfig {
            min_precision: PARALLEL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        };
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let mut parameter_lifts = [None, None];
            assert_eq!(
                replay_bivariate_parameter_pair(
                    &first,
                    &second,
                    &alpha,
                    &beta,
                    &policy,
                    config,
                    &mut parameter_lifts,
                )
                .unwrap(),
                Classification::Decided(BivariateParameterPairReplay::Direct)
            );
        }
    }

    #[test]
    #[cfg(feature = "predicates")]
    fn specialized_common_fiber_replays_degree_drops_in_either_orientation() {
        let alpha = algebraic_parameter(vec![
            Real::from(-1_i8),
            Real::zero(),
            Real::zero(),
            Real::from(2_i8),
        ]);
        let beta = algebraic_parameter(vec![
            Real::from(-1_i8),
            Real::zero(),
            Real::zero(),
            Real::from(4_i8),
        ]);
        // A=2*a^3-1 and B=4*b^3-1 vanish at the represented pair. The A*B
        // term supplies generic degree three in both orientations, then drops
        // out exactly. Both specialized equations retain b-a^2 as their GCD.
        let first = BivariatePolynomial::new(vec![
            vec![Real::one(), Real::one(), Real::zero(), Real::from(-4_i8)],
            vec![],
            vec![Real::from(-1_i8)],
            vec![
                Real::from(-2_i8),
                Real::zero(),
                Real::zero(),
                Real::from(8_i8),
            ],
        ]);
        let second = BivariatePolynomial::new(vec![
            vec![
                Real::one(),
                Real::from(2_i8),
                Real::zero(),
                Real::from(-4_i8),
            ],
            vec![],
            vec![Real::from(-2_i8)],
            vec![
                Real::from(-2_i8),
                Real::zero(),
                Real::zero(),
                Real::from(8_i8),
            ],
        ]);
        let rootless_after_drop = BivariatePolynomial::new(vec![
            vec![Real::one(), Real::zero(), Real::from(-1_i8)],
            vec![],
            vec![],
            vec![Real::zero(), Real::zero(), Real::from(2_i8)],
        ]);
        let beta_defining = BivariatePolynomial::new(vec![vec![
            Real::from(-1_i8),
            Real::zero(),
            Real::zero(),
            Real::from(4_i8),
        ]]);
        let alpha_defining = BivariatePolynomial::new(vec![
            vec![Real::from(-1_i8)],
            vec![],
            vec![],
            vec![Real::from(2_i8)],
        ]);

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (retained_axis, retained, fiber) in [
                (CurveResultantParameter::First, &alpha, &beta),
                (CurveResultantParameter::Second, &beta, &alpha),
            ] {
                assert_eq!(
                    parameter_pair_matches_specialized_fiber(
                        &first,
                        &second,
                        retained_axis,
                        retained,
                        fiber,
                        &policy,
                    )
                    .unwrap(),
                    Classification::Decided(BivariateParameterPairReplay::Direct)
                );
            }
            assert_eq!(
                parameter_pair_matches_specialized_fiber(
                    &rootless_after_drop,
                    &beta_defining,
                    CurveResultantParameter::First,
                    &alpha,
                    &beta,
                    &policy,
                )
                .unwrap(),
                Classification::Decided(BivariateParameterPairReplay::Rejected)
            );
            assert_eq!(
                parameter_pair_matches_specialized_fiber(
                    &alpha_defining,
                    &alpha_defining,
                    CurveResultantParameter::First,
                    &alpha,
                    &beta,
                    &policy,
                )
                .unwrap(),
                Classification::Uncertain(UncertaintyReason::Boundary)
            );
        }
    }

    #[test]
    fn algebraic_sign_filter_has_no_fixed_refinement_cap() {
        // The 104th continued-fraction convergent to sqrt(2) lies below the
        // exact value by roughly 2^-264. Distinguishing its half from the root
        // of `2t^2-1` therefore requires more than the retired 256 bisections.
        let mut numerator = num::BigUint::from(1_u8);
        let mut denominator = num::BigUint::from(1_u8);
        for _ in 0..104 {
            let next_numerator = &numerator + &denominator * num::BigUint::from(2_u8);
            let next_denominator = &numerator + &denominator;
            numerator = next_numerator;
            denominator = next_denominator;
        }
        let nearby_root = Real::new(
            hyperreal::Rational::from_bigint_fraction(
                num::BigInt::from(numerator),
                denominator * num::BigUint::from(2_u8),
            )
            .unwrap(),
        );
        let policy = CurveContext::STRICT;
        let defining = match BezierParameterPolynomial::try_new_power_basis(
            vec![Real::from(-1_i8), Real::zero(), Real::from(2_i8)],
            &policy,
        )
        .unwrap()
        {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => panic!("defining polynomial: {reason:?}"),
        };
        let filter = match BezierParameterPolynomial::try_new_power_basis(
            vec![-nearby_root, Real::one()],
            &policy,
        )
        .unwrap()
        {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => panic!("filter polynomial: {reason:?}"),
        };
        let interval = match BezierParameterInterval::try_new(
            (Real::one() / Real::from(2_i8)).unwrap(),
            Real::one(),
            &policy,
        )
        .unwrap()
        {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(reason) => panic!("isolating interval: {reason:?}"),
        };

        assert_eq!(
            signed_polynomial_on_isolating_interval(&filter, &defining, &interval, &policy)
                .unwrap(),
            Classification::Decided(RealSign::Positive)
        );
    }
}
