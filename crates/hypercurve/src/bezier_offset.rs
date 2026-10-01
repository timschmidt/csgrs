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

mod algebraic_chord;
mod approximation;
mod bivariate_replay;
mod cusp_semicircle;
mod dense_sign;
mod fiber_sign;
mod multivariate_sign;
mod parallel;
mod parallel_components;
mod parallel_construction;
mod parallel_pair_sets;
#[path = "curve_parameter_component.rs"]
mod parameter_component;
mod recursive_field;
mod recursive_quadratic;
mod represented;
mod selected_dense;
mod selected_fiber;

pub use approximation::*;
pub(crate) use bivariate_replay::*;
use dense_sign::*;
pub(crate) use fiber_sign::*;
use multivariate_sign::*;
pub(crate) use parallel_components::*;
pub use parallel_construction::*;
pub(crate) use parallel_pair_sets::*;
use parameter_component::{
    ComponentParameterChart2, ComponentParameterInterval2, retain_component_charts,
    retain_finite_parallel_components,
};
pub(crate) use parameter_component::{CurveParameterComponent2, CurveParameterComponentSelection2};
pub(crate) use recursive_quadratic::*;
use represented::*;
use selected_dense::*;

use crate::CurvePointData2;
use crate::classify::product_sign;
use hypersolve::RealInterval;
use hypersolve::bivariate_arithmetic::{
    bivariate_add, bivariate_multiply, bivariate_multiply_first_parameter, bivariate_outer_product,
    bivariate_parameter_difference, bivariate_scale, bivariate_scaled_difference,
    bivariate_specialize_first, bivariate_specialize_second,
    bivariate_substitute_second_equal_affine_first, bivariate_substitute_second_equal_first,
    bivariate_substitute_second_equal_one_minus_first, bivariate_subtract,
    bivariate_swap_parameters, polynomial_add, polynomial_multiply, polynomial_power,
    polynomial_powers, polynomial_scale, polynomial_subtract, try_bivariate_multiply,
    try_zero_bivariate_coefficients,
};
use hypersolve::bivariate_arithmetic::{
    bivariate_complement_second_parameter, bivariate_first_active_degree,
    bivariate_parameter_derivative, bivariate_projective_second_parameter,
    bivariate_second_parameter_coefficient, bivariate_storage_bidegree_sum,
    bivariate_tensor_product, deflate_bivariate_parameter_diagonal_exact, polynomial_derivative,
    polynomial_trim_structural_zeros,
};
#[cfg(test)]
use hypersolve::exact_factor::bivariate_bilinear_factor_from_roots;
use hypersolve::exact_factor::{
    bivariate_add_scaled_assign, bivariate_bilinear_factorizations_bounded,
    bivariate_evaluate_exact, bivariate_exact_nonzero_metadata, bivariate_exact_square_root,
    bivariate_linear_root_resultant, bivariate_quadratic_constraint_resultant,
    bivariate_trim_exact, cubic_specialization_rejects_repeated_factor,
    polynomial_restrict_to_interval,
};
use hypersolve::exact_factor::{
    rational_multi_affine_lift_factor_coefficients, rational_multi_affine_lift_scale,
    rational_multi_affine_lift_scale_from_anchor_pair, trivariate_axis_lift_degree,
    trivariate_axis_lift_power_slice, trivariate_axis_lift_taylor_slice,
};
use hypersolve::represented_root::{
    NEGATIVE_UNIT_SCALE, POSITIVE_UNIT_SCALE, dense_tensor_interval,
    dense_tensor_interval_with_coefficient_precision,
    dense_tensor_interval_with_coefficient_precision_and_source_witnesses,
    refined_represented_root, represented_affine_coordinate, represented_affine_tensor_basis,
    represented_dense_value_refined, represented_dense_value_with_coefficient_precision,
    represented_ratio, represented_roots_strictly_equal, represented_strict_order,
    represented_strict_sign, represented_tensor_coordinate, represented_tensor_coordinate_refined,
    represented_tensor_nested_interval, represented_tensor_nested_value_refined,
    represented_univariate_coordinate, represented_vector_dot_cross,
    represented_zero_offset_unit_scales,
};
use hypersolve::tensor_support::{
    bivariate_dense_tensor, bivariate_tensor_with_output_axis,
    dense_reduce_selected_tuple_relations, dense_tensor_with_output_axis, try_clone_dense_tensor,
};
use hypersolve::tensor_support::{
    dense_last_axis_coefficient, dense_last_axis_derivative, dense_reduce_selected_root_relations,
    dense_specialize_last_axis, dense_tensor_embed_axes, dense_tensor_from_polynomial_coefficients,
};
use hypersolve::trivariate_arithmetic::try_zero_trivariate_coefficients;
use std::borrow::Cow;
use std::ops::ControlFlow;
use std::sync::{Arc, Mutex, OnceLock, Weak};

use crate::bezier_algebraic_image::{
    RationalBezierAlgebraicPointPredicate2, compare_algebraic_representations_with_policy,
};
use crate::bezier_algebraic_image::{
    certified_parameter_representation, parameter_representation,
    rational_point_image_from_power_basis, rational_tangent_image_from_power_basis,
};
use crate::bezier_parameter::{
    BezierParameterRay2, BezierParameterRefinement2, bernstein_to_power_coefficients,
    coefficients_value_interval_on_parameter_interval,
    coefficients_value_interval_on_real_interval, deep_exact_coefficients_sign_at_parameter,
    divide_by_linear_root, power_to_bernstein_coefficients, restrict_power_basis_to_interval,
    signed_coefficients_at_parameter, strict_coefficients_sign_on_parameter_interval,
    univariate_unit_interval_strict_bernstein_sign,
};
use crate::bezier_split::CurveParameterDomain2;
use crate::classify::{classify_oriented_line, compare_reals, in_closed_unit_interval, real_sign};
use crate::rational_bezier_general::{
    RationalBezierOverlapParameterCorrespondence2, RationalParameterImageMap2,
    ResultantParameterProjection, exact_contact_point_evidence,
    resultant_bivariate_polynomial_system_complete, resultant_parameter_polynomial,
    resultant_parameter_projection,
};
use crate::{
    Aabb2, Axis2, BezierAlgebraicParameter2, BezierLineContact, BezierLineContactKind,
    BezierLineContactRelation, BezierLineCrossingDirection, BezierLineImageFitRelation,
    BezierParameter2, BezierParameterInterval, BezierParameterPolynomial, BezierParameterRange2,
    BezierParameterRayDirection2, Classification, CubicBezier2, Curve2, CurveContext,
    CurveDerivative2, CurveError, CurveGeometry2, CurveIntersectionCandidates2, CurveOperation2,
    CurveOverlapOrientation2, CurveParameter2, CurveParameterRange2, CurvePath2, CurvePoint2,
    CurveResult, ExactCurveError, ExactCurveResult, LineCircleRelation, LineSeg2, Point2,
    QuadraticBezier2, RationalBezier2, RationalBezierAlgebraicPointImage2,
    RationalBezierAlgebraicTangentImage2, RationalBezierIntersectionContacts2,
    RationalBezierIntersectionOverlap2, RationalQuadraticBezier2, Real, Similarity2,
    UncertaintyReason,
};
use hyperreal::{Rational as HyperRational, RealSign, ZeroKnowledge};
use hypersolve::{
    AlgebraicFiberDiagonalDeflationStatus, AlgebraicFiberPolynomialImageProjectionConfig,
    AlgebraicFiberPolynomialImageProjectionStatus, AlgebraicFiberProjectionStatus,
    AlgebraicFiberRationalReductionStatus, AlgebraicFiberRootCountStatus,
    AlgebraicFiberRootIsolationConfig, AlgebraicFiberRootIsolationReport,
    AlgebraicFiberRootIsolationStatus, PredicateCertainty,
    count_bivariate_common_fiber_roots_at_algebraic_parameter,
    count_bivariate_fiber_roots_at_algebraic_parameter,
    count_bivariate_fiber_roots_at_algebraic_parameter_closed,
    count_bivariate_fiber_roots_at_algebraic_parameter_intervals,
    deflate_bivariate_fiber_diagonal_root_at_algebraic_parameter,
    isolate_bivariate_fiber_roots_at_algebraic_parameter, project_algebraic_fiber_polynomial_image,
    project_algebraic_fiber_polynomial_image_relation,
    project_bivariate_fiber_at_algebraic_parameter,
    reduce_bivariate_rational_function_at_algebraic_parameter,
};
use hypersolve::{
    AlgebraicRootComparisonStatus, AlgebraicRootMobiusTransformStatus,
    AlgebraicRootRefinementComparisonConfig, AlgebraicRootRepresentation,
    AlgebraicRootSquareRootStatus, DenseTensorPolynomial, IsolatedRootInterval,
    OrderedFieldPolynomialContext, OrderedFieldRootIsolationConfig,
    OrderedFieldRootIsolationStatus, algebraic_root_affine_relation,
    compare_algebraic_root_representations_with_refinement, divide_univariate_polynomial_exact,
    greatest_common_divisor_univariate_polynomials_exact, isolate_ordered_field_polynomial_roots,
    project_selected_tensor_fiber_via_tagged_norm, square_root_algebraic_root_representation,
    transform_algebraic_root_mobius, validate_algebraic_root_representation,
};
use hypersolve::{
    BivariatePolynomial, BivariatePolynomialAxisFactorStatus, BivariatePolynomialComponentReport,
    BivariatePolynomialComponentStatus, CurveIntersectionParameterLiftMap,
    CurveIntersectionParameterLiftReport, CurveIntersectionParameterLiftStatus,
    CurveIntersectionResultantConfig, CurveIntersectionResultantStatus, CurveResultantParameter,
    RationalParametricCurve2, divide_bivariate_polynomial_exact,
    extract_bivariate_polynomial_system_axis_factors,
    linear_parameter_lifts_bivariate_polynomial_system,
    parameter_component_bivariate_polynomial_system, resultant_bivariate_polynomial_system,
    subresultant_chain_univariate_polynomials,
};
use hypersolve::{
    TrivariateConstraintResultantStatus, TrivariateConstraintSubresultantStatus,
    TrivariatePolynomial, TrivariatePolynomialAxis,
    resultant_trivariate_polynomial_univariate_constraint,
    subresultant_trivariate_polynomial_univariate_constraint,
};

/// Exact source representation retained by an analytic Bezier parallel.
///
/// This structural view is the carrier's lossless serialization and
/// diagnostic boundary: together with the signed distance it reconstructs the
/// complete procedural curve without exposing or materializing lazy caches.
#[derive(Clone, Debug, PartialEq)]
pub enum BezierParallelSource2 {
    /// Polynomial quadratic Bezier source.
    Quadratic(QuadraticBezier2),
    /// Polynomial cubic Bezier source.
    Cubic(CubicBezier2),
    /// Arbitrary-degree rational Bezier source.
    Rational(RationalBezier2),
}

impl BezierParallelSource2 {
    /// Returns whether the source parameter itself is an exact affine line
    /// coordinate. Every parallel of such a source is a translation with the
    /// same nonzero derivative, so its derivative scale is identically
    /// positive and needs no curvature/speed predicate.
    fn has_exact_affine_line_parameterization(&self) -> bool {
        match self {
            Self::Quadratic(source) => {
                if source.retained_exact_line_image().is_some() {
                    return true;
                }
                let Ok(line) = LineSeg2::try_new(source.start().clone(), source.end().clone())
                else {
                    return false;
                };
                let half =
                    (Real::one() / Real::from(2_i8)).expect("two is a nonzero exact denominator");
                source.control() == &line.point_at(half)
            }
            Self::Cubic(source) => {
                let Ok(line) = LineSeg2::try_new(source.start().clone(), source.end().clone())
                else {
                    return false;
                };
                let third =
                    (Real::one() / Real::from(3_i8)).expect("three is a nonzero exact denominator");
                source.control1() == &line.point_at(third.clone())
                    && source.control2() == &line.point_at(Real::from(2_i8) * third)
            }
            Self::Rational(source) => source.exact_linear_parameterization_line().is_some(),
        }
    }

    fn is_reversal_of(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Quadratic(first), Self::Quadratic(second)) => first
                .control_points()
                .into_iter()
                .eq(second.control_points().into_iter().rev()),
            (Self::Cubic(first), Self::Cubic(second)) => first
                .control_points()
                .into_iter()
                .eq(second.control_points().into_iter().rev()),
            (Self::Rational(first), Self::Rational(second)) => first
                .homogeneous_controls()
                .iter()
                .eq(second.homogeneous_controls().iter().rev()),
            (Self::Quadratic(_) | Self::Cubic(_) | Self::Rational(_), _) => false,
        }
    }

    fn reversed(&self) -> Self {
        match self {
            Self::Quadratic(source) => Self::Quadratic(
                source
                    .reversed_with_retained_provenance()
                    .expect("reversing a retained exact line preserves distinct endpoints"),
            ),
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

    fn certified_bounds(&self) -> Classification<Aabb2> {
        match self {
            Self::Quadratic(source) => source.certified_bounds(),
            Self::Cubic(source) => source.certified_bounds(),
            Self::Rational(source) => source.certified_bounds_classified(),
        }
    }

    pub(crate) fn to_rational_bezier(&self) -> CurveResult<RationalBezier2> {
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

    fn transform_similarity(&self, transform: &Similarity2) -> CurveResult<Self> {
        let transformed = match self {
            Self::Quadratic(source) => {
                Self::Quadratic(source.transform_similarity_with_retained_provenance(transform)?)
            }
            Self::Cubic(source) => {
                let points = source
                    .control_points()
                    .map(|point| transform.transform_point(point));
                Self::Cubic(CubicBezier2::new(
                    points[0].clone(),
                    points[1].clone(),
                    points[2].clone(),
                    points[3].clone(),
                ))
            }
            Self::Rational(source) => Self::Rational(source.transform_similarity(transform)),
        };
        Ok(transformed)
    }
}

#[derive(Debug)]
struct BezierParallelSourceData2 {
    source: BezierParallelSource2,
    polynomial_power_basis: OnceLock<(Vec<Real>, Vec<Real>)>,
    differential: OnceLock<BezierParallelDifferential2>,
    primitive_tangent: OnceLock<Option<BezierParallelPrimitiveTangent2>>,
    unit_ph_speed: OnceLock<Option<Arc<BezierParameterPolynomial>>>,
}

#[derive(Debug)]
struct BezierParallelData2 {
    source: Arc<BezierParallelSourceData2>,
    distance: Real,
    certified_ph_offset: OnceLock<Option<Arc<CertifiedPythagoreanHodographOffset2>>>,
}

#[derive(Debug)]
struct BezierParallelDifferential2 {
    tangent_x: Vec<Real>,
    tangent_y: Vec<Real>,
    tangent_derivative_x: Vec<Real>,
    tangent_derivative_y: Vec<Real>,
}

/// Successful exact factorization of the source hodograph. The factor sign
/// selects its normal sheet; the primitive field is shared across ranges,
/// retained point witnesses, and all offsets of this source.
#[derive(Debug)]
struct BezierParallelPrimitiveTangent2 {
    factor: Vec<Real>,
    field: Arc<BezierAnalyticParallelTangentField2>,
    reversed_field: OnceLock<Arc<BezierAnalyticParallelTangentField2>>,
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

/// Clone-shared rational frame at one certified algebraic parallel cusp.
///
/// The frame retains the source point `P`, unit left normal `N`, and common
/// denominator as power-basis polynomials in the cusp parameter. Every point
/// `P + d N` on a parallel of the shared source is then one affine combination
/// of these five vectors. Keeping the frame once avoids rebuilding the same
/// high-degree products for a cusp join's center and both endpoints.
#[derive(Clone, Debug)]
pub(crate) struct BezierParallelAlgebraicCuspFrame2 {
    data: Arc<BezierParallelAlgebraicCuspFrameData2>,
}

/// Exact center/radial frame shared by the authoritative selected-circle
/// carrier. Rational one-field frames remain the compact fast path; a general
/// selected parallel normal retains its speed radical procedurally instead of
/// flattening it into a primitive element.
#[derive(Clone, Debug)]
enum BezierSelectedCircleFrame2 {
    Rational(BezierParallelAlgebraicCuspFrame2),
    ParallelNormal(Arc<BezierSelectedParallelNormalFrameData2>),
    /// The center is retained point evidence and the parameter-zero radius is
    /// the exact unit left normal of one algebraic chord.  Points in this
    /// frame remain lazy chord-normal/tangent displacements, so independent
    /// endpoint and center fields never need a primitive-element compositum.
    ChordNormal(Arc<BezierSelectedChordNormalFrameData2>),
    /// The circle center is one mapped point on an existing selected circle,
    /// and its parameter-zero radius is that source circle's certified radial
    /// direction at the same point.  This keeps a genuinely correlated
    /// circle-pair fillet in its existing pair field instead of constructing a
    /// primitive element for the two selected centers and contact radical.
    SelectedRadial(Arc<BezierSelectedRadialFrameData2>),
}

#[derive(Debug, PartialEq)]
struct BezierSelectedParallelNormalFrameData2 {
    center_support: BezierParallel2,
    center_parameter: CurveParameter2,
    policy: CurveContext,
}

#[derive(Debug, PartialEq)]
struct BezierSelectedChordNormalFrameData2 {
    anchor: BezierAlgebraicChord2,
    center: CurvePoint2,
    policy: CurveContext,
}

#[derive(Debug)]
struct BezierSelectedRadialFrameData2 {
    center_parameter: Arc<BezierAlgebraicCuspSemicircleMappedParameterData2>,
    /// Signed source-circle radius whose quotient with the contact radial is
    /// the parameter-zero unit direction of this frame.
    normal_denominator: Real,
    /// Ultimate pre-similarity frame. One shared pointer preserves the exact
    /// authored coefficient field and replaces much larger independently
    /// rebuilt composita on transformed line/circle predicates.
    similarity_source: Option<Arc<BezierSelectedRadialSimilaritySource2>>,
    policy: CurveContext,
}

#[derive(Debug)]
struct BezierCircleParallelSystemCacheEntry2 {
    target: Weak<BezierParallelData2>,
    policy: CurveContext,
    permits_approximate_512: bool,
    system: Arc<BezierRecursiveCircleTargetSystem2>,
}

#[derive(Debug)]
struct BezierSelectedRadialSimilaritySource2 {
    frame: Arc<BezierSelectedRadialFrameData2>,
    radial_distance: Real,
    clockwise: bool,
}

#[derive(Debug)]
struct BezierParallelAlgebraicCuspFrameData2 {
    /// Analytic source provenance when this frame was built at a parallel
    /// cusp. A direct retained-center frame has no source parallel; all circle
    /// equations use the shared polynomial frame below in either case.
    parallel: Option<BezierParallel2>,
    /// Fixed unit normal for a direct axis-aligned retained center. This is a
    /// structural tangent certificate for adjacent round-join chords; general
    /// transformed and analytic-cusp frames leave it absent.
    cardinal_normal: Option<(i8, i8)>,
    /// Direct certified unit-normal provenance when this one-field frame was
    /// authored from a represented vector. Keeping one shared pointer avoids
    /// trying to factor that vector back out of expanded algebraic
    /// homogeneous coefficients at every endpoint query.
    represented_unit_normal: Option<Arc<(Real, Real)>>,
    /// Original retained center image for a directly framed circle. This is a
    /// cache/provenance handle only; the polynomial frame remains authoritative.
    direct_center: Option<RationalBezierAlgebraicPointImage2>,
    parameter: BezierAlgebraicParameter2,
    source_x_numerator: Vec<Real>,
    source_y_numerator: Vec<Real>,
    normal_x_numerator: Vec<Real>,
    normal_y_numerator: Vec<Real>,
    denominator: Vec<Real>,
}

impl PartialEq for BezierParallelAlgebraicCuspFrame2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.data.parallel == other.data.parallel
                && self.data.cardinal_normal == other.data.cardinal_normal
                && self.data.represented_unit_normal == other.data.represented_unit_normal
                && self.data.parameter == other.data.parameter
                && self.data.source_x_numerator == other.data.source_x_numerator
                && self.data.source_y_numerator == other.data.source_y_numerator
                && self.data.normal_x_numerator == other.data.normal_x_numerator
                && self.data.normal_y_numerator == other.data.normal_y_numerator
                && self.data.denominator == other.data.denominator)
    }
}

impl PartialEq for BezierSelectedCircleFrame2 {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Rational(first), Self::Rational(second)) => first == second,
            (Self::ParallelNormal(first), Self::ParallelNormal(second)) => {
                Arc::ptr_eq(first, second) || first == second
            }
            (Self::ChordNormal(first), Self::ChordNormal(second)) => {
                Arc::ptr_eq(first, second) || first == second
            }
            (Self::SelectedRadial(first), Self::SelectedRadial(second)) => {
                Arc::ptr_eq(first, second)
            }
            (
                Self::Rational(_),
                Self::ParallelNormal(_) | Self::ChordNormal(_) | Self::SelectedRadial(_),
            )
            | (
                Self::ParallelNormal(_),
                Self::Rational(_) | Self::ChordNormal(_) | Self::SelectedRadial(_),
            )
            | (
                Self::ChordNormal(_),
                Self::Rational(_) | Self::ParallelNormal(_) | Self::SelectedRadial(_),
            )
            | (
                Self::SelectedRadial(_),
                Self::Rational(_) | Self::ParallelNormal(_) | Self::ChordNormal(_),
            ) => false,
        }
    }
}

/// Exact semicircular join centered at an algebraic cusp of an analytic parallel.
///
/// The start point is `C + r N`, the end point is `C - r N`, and `clockwise`
/// selects which half of the circle is traversed. The carrier is intentionally
/// one word: its center, radius direction, parameter proof, and all polynomial
/// products are shared through the cusp frame.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspSemicircle2 {
    data: Arc<BezierAlgebraicCuspSemicircleData2>,
}

#[derive(Debug)]
struct BezierAlgebraicCuspSemicircleData2 {
    frame: BezierSelectedCircleFrame2,
    radial_distance: Real,
    clockwise: bool,
    /// A query reuses its circle/target field across incidence, parameter order
    /// and tangent replay. The weak target key does not retain the curve.
    parallel_system_cache: Mutex<Option<BezierCircleParallelSystemCacheEntry2>>,
}

impl PartialEq for BezierAlgebraicCuspSemicircle2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.data.frame == other.data.frame
                && self.data.radial_distance == other.data.radial_distance
                && self.data.clockwise == other.data.clockwise)
    }
}

#[derive(Debug)]
pub(crate) struct BezierAlgebraicCuspCircleRationalSystem2 {
    /// Circle incidence, equal to zero at every contact.
    pub(crate) incidence: BivariatePolynomial,
    /// Nonnegative exactly on the selected closed semicircle.
    pub(crate) selected_half_plane: BivariatePolynomial,
    /// Positive at the start diameter endpoint and negative at the end.
    pub(crate) diameter_side: BivariatePolynomial,
    /// The squared radius over the same positive squared common denominator.
    pub(crate) radius_squared_denominator: BivariatePolynomial,
    /// Signed tangent cross product after multiplication by a positive square.
    pub(crate) tangent_cross: BivariatePolynomial,
    /// Signed angular velocity `cross(Q-C,Q')` after multiplication by a
    /// positive square. Its selected-fiber roots partition every coincident
    /// circle component into regular monotone parameter cells.
    pub(crate) angular_tangent: BivariatePolynomial,
}

/// A certified rational bracket or represented witness for one semicircle
/// contact parameter.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum BezierAlgebraicCuspSemicircleParameterBracket2 {
    Exact(Real),
    Interval(BezierParameterInterval),
}

/// Pair-shared local-field map from rational-curve contacts to the algebraic
/// semicircle's monotone parameter.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspSemicircleRationalParameterMap2 {
    data: Arc<BezierAlgebraicCuspSemicircleRationalParameterMapData2>,
}

/// Shared exact map for a rational-curve contact whose source parameter stays
/// in the selected circle-center fiber rather than a global norm field.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspSemicircleSelectedFiberRationalParameterMap2 {
    data: Arc<BezierAlgebraicCuspSemicircleSelectedFiberRationalParameterMapData2>,
}

#[derive(Debug)]
struct BezierAlgebraicCuspSemicircleSelectedFiberRationalParameterMapData2 {
    semicircle: BezierAlgebraicCuspSemicircle2,
    curve: RationalBezier2,
    /// A singleton of the complete circle incidence, excluding overlap
    /// boundary equations and candidate-only tangent constraints.
    isolated_incidence: Option<BezierAlgebraicSelectedFiberAuthority2>,
    diameter: BezierAlgebraicCuspTwoTermExpression2,
    radius_squared_denominator: BivariatePolynomial,
    speed_squared: BivariatePolynomial,
    /// Circle tangent crossed with the rational tangent, under the map's
    /// common positive selected-fiber scale.
    tangent_cross: BezierAlgebraicCuspTwoTermExpression2,
    /// Circle radius crossed with the rational tangent. The circle turn sign
    /// converts this into the corresponding tangent dot product.
    angular_tangent: BezierAlgebraicCuspTwoTermExpression2,
    policy: CurveContext,
    parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2,
}

/// Shared local-field map for a genuinely analytic parallel contact with a
/// general selected circle. The target parameter remains in `Q(alpha)` and
/// both source-speed radicals are replayed from the pair-owned expressions.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspSemicircleSelectedFiberParallelParameterMap2 {
    data: Arc<BezierAlgebraicCuspSemicircleSelectedFiberParallelParameterMapData2>,
}

#[derive(Debug)]
struct BezierAlgebraicCuspSemicircleSelectedFiberParallelParameterMapData2 {
    semicircle: BezierAlgebraicCuspSemicircle2,
    parallel: BezierParallel2,
    diameter: BezierParallelTwoNormalExpression2,
    radius_squared_denominator: BivariatePolynomial,
    tangent_cross_source: BezierAlgebraicCuspTwoTermExpression2,
    tangent_dot_source: BezierParallelTwoNormalExpression2,
    center_speed_squared: BivariatePolynomial,
    candidate_speed_squared: BivariatePolynomial,
    policy: CurveContext,
}

#[derive(Debug)]
struct BezierAlgebraicCuspSemicircleRationalParameterMapData2 {
    /// Stable exact carrier identity for comparisons against contacts replayed
    /// through a correlated retained chord.
    semicircle: BezierAlgebraicCuspSemicircle2,
    curve: RationalBezier2,
    system: BezierAlgebraicCuspSemicircleRationalParameterMapSystem2,
    policy: CurveContext,
    parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2,
}

/// Each fact carries its applicable key. Ordinary scalar projections keep
/// their compact Bezier parameter; inverse correspondences accept every exact
/// curve parameter without a dummy key, an extra tag, or a boxed key.
#[derive(Clone, Debug)]
enum BezierAlgebraicCuspSemicircleParameterCacheEntry2 {
    ScalarValue {
        parameter: BezierParameter2,
        value: Option<Real>,
    },
    Approximate512ScalarValue {
        parameter: BezierParameter2,
        value: Box<Option<Real>>,
    },
    RepresentedDiameterCoordinate {
        parameter: BezierParameter2,
        evidence: Box<(AlgebraicRootRepresentation, CurveContext)>,
    },
    ParameterBracket {
        parameter: BezierParameter2,
        evidence: Box<BezierAlgebraicCuspSemicircleCachedParameterBracket2>,
    },
    /// The correspondence survives expiry of its weak source handle; the
    /// overlap can replay the certified incidence without an ownership cycle.
    RetainedCusp {
        parameter: CurveParameter2,
        cusp: Weak<BezierAlgebraicCuspSemicircleMappedParameterData2>,
        policy: CurveContext,
    },
}

#[derive(Clone, Debug)]
struct BezierAlgebraicCuspSemicircleCachedParameterBracket2 {
    location: BezierAlgebraicCuspSemicircleContactLocation2,
    refinement_steps: usize,
    bracket: BezierAlgebraicCuspSemicircleParameterBracket2,
    policy: CurveContext,
}

#[derive(Debug, Default)]
struct BezierAlgebraicCuspSemicircleParameterCache2 {
    entries: Mutex<Vec<BezierAlgebraicCuspSemicircleParameterCacheEntry2>>,
}

/// Exact parameter-order authority behind the common rational-contact result.
///
/// Ordinary and selected-parallel-normal circles keep the compact two-axis
/// fast path.  A pair-radial fillet circle retains the two source roots and
/// the target parameter as three independent axes, replaying the authored
/// pair-contact radical instead of flattening it into a primitive element.
#[derive(Debug)]
enum BezierAlgebraicCuspSemicircleRationalParameterMapSystem2 {
    OneField {
        cusp_parameter: BezierParameter2,
        incidence: BivariatePolynomial,
        diameter: BezierAlgebraicCuspSemicircleRationalDiameter2,
        radius_squared_denominator: BivariatePolynomial,
    },
    SelectedRadial {
        pair_map: BezierAlgebraicCuspSemicirclePairParameterMap2,
        branch: i8,
        discriminant: TrivariatePolynomial,
        diameter: BezierAlgebraicCuspTrivariateSquareRootExpression2,
        radius_squared_denominator: TrivariatePolynomial,
        tangent_cross: BezierAlgebraicCuspTrivariateSquareRootExpression2,
        angular_tangent: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    },
    /// Arbitrary-depth selected-radial circle whose rational target parameter
    /// is embedded into the existing recursive quadratic tower. Rational
    /// incidence uses unit procedural speed, so singular target parameters
    /// remain valid contacts rather than becoming spurious speed roots.
    Recursive {
        system: Arc<BezierRecursiveCircleTargetSystem2>,
    },
    /// Direct homogeneous chord-pair center with two retained positive speed
    /// radicals. Its rank-independent tensor is the sole angular authority;
    /// immutable coefficients are allocation-shared by contacts.
    ChordNormalProjective {
        system: Arc<BezierChordNormalDenseMapSystem2>,
    },
    /// Rank-independent Cartesian frame selected under STRICT.  The target
    /// curve parameter remains its own retained algebraic root; angular order
    /// is evaluated from the exact represented contact point on demand.
    Represented {
        frame: BezierRepresentedSelectedRadialCircleFrame2,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BezierAlgebraicCuspSemicircleContactLocation2 {
    Interior,
    Start,
    End,
}

/// Exact finite-chord endpoint incidence retained by an owning boundary
/// construction. This says only that the named chord endpoint lies on the
/// supporting circle; it makes no tangency, selected-half, or uniqueness
/// claim, so the common quadratic kernel must still classify every root.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BezierCertifiedFiniteChordEndpointIncidence2 {
    Start,
    End,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierAlgebraicCuspSemicircleRationalContact2 {
    pub(crate) other_parameter: CurveParameter2,
    pub(crate) point: CurvePoint2,
    pub(crate) tangent_cross_sign: RealSign,
    /// Exact circle-tangent dot rational-tangent sign when the intersection
    /// kernel already evaluated the corresponding angular predicate.
    pub(crate) tangent_dot_sign: Option<RealSign>,
    pub(crate) location: BezierAlgebraicCuspSemicircleContactLocation2,
}

/// One exact general selected-circle/rational contact retained wholly in the
/// selected center fiber. The contact and both carrier parameters share one
/// mapped-data allocation.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspSemicircleSelectedFiberContact2 {
    data: Arc<BezierAlgebraicCuspSemicircleMappedParameterData2>,
}

impl PartialEq for BezierAlgebraicCuspSemicircleSelectedFiberContact2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
    }
}

/// One positive-length rational-circle component interval retained in the
/// selected center field. Both rational endpoints are one-word local scalars;
/// no norm polynomial or algebraic subcurve is constructed.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2 {
    other_start: BezierAlgebraicSelectedFiberParameter2,
    other_end: BezierAlgebraicSelectedFiberParameter2,
    cusp_start: BezierAlgebraicCuspSemicircleParameter2,
    cusp_end: BezierAlgebraicCuspSemicircleParameter2,
    orientation: CurveOverlapOrientation2,
    map: BezierAlgebraicCuspSemicircleSelectedFiberRationalParameterMap2,
}

impl PartialEq for BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2 {
    fn eq(&self, other: &Self) -> bool {
        self.other_start == other.other_start
            && self.other_end == other.other_end
            && self.cusp_start.shares_exact_evidence(&other.cusp_start)
            && self.cusp_end.shares_exact_evidence(&other.cusp_end)
            && self.orientation == other.orientation
    }
}

#[derive(Clone, Debug, PartialEq)]
enum BezierAlgebraicCuspSemicircleRationalCorrelation2 {
    /// Use the relation retained by the pair-shared parameter map.
    Map,
    /// The represented line map is only a compact kernel for this retained
    /// chord. Preserve its tangent authority without enlarging every mapped
    /// contact with a second optional payload.
    MapWithChordTangent {
        chord: BezierAlgebraicChord2,
        circle_cross_chord: RealSign,
    },
    /// The other parameter is independently represented, normally an exact
    /// authored range endpoint.
    Independent,
    /// Use a component-partition relation specific to this parameter, such as
    /// a stationary angular-velocity equation.
    Relation(Arc<BivariatePolynomial>),
}

/// One source-parameter boundary of a coincident rational-circle component.
/// The same compact record serves one-field and pair-radial maps; the owning
/// exact signer interprets the retained correlation.
#[derive(Clone, Debug)]
struct BezierAlgebraicCuspSemicircleRationalComponentBoundary2 {
    parameter: BezierParameter2,
    correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2,
    selected_relation: bool,
}

/// Minimal parameter evidence retained by a mapped cusp cut. Intersection
/// point and tangent evidence stay on the transient contact instead of being
/// duplicated in every range endpoint.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierAlgebraicCuspSemicircleRationalMapContact2 {
    other_parameter: CurveParameter2,
    location: BezierAlgebraicCuspSemicircleContactLocation2,
    correlation: BezierAlgebraicCuspSemicircleRationalCorrelation2,
}

/// Isolated parameter visits and monotone overlap cells can coexist, including
/// distinct visits to the same geometric point. Both retain their original
/// mapping authority; a cell already owns its closed boundary visits.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum BezierAlgebraicCuspSemicircleRationalIntersections2 {
    Mapped {
        contacts: Vec<BezierAlgebraicCuspSemicircleRationalContact2>,
        overlaps: Vec<BezierAlgebraicCuspSemicircleMappedOverlap2>,
    },
    SelectedFiber {
        contacts: Vec<BezierAlgebraicCuspSemicircleSelectedFiberContact2>,
        overlaps: Vec<BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2>,
    },
    DegenerateProjection,
}

#[derive(Clone, Debug)]
pub(crate) enum BezierAlgebraicFiberProjection2 {
    Parameters(Vec<BezierParameter2>),
    IdenticallyZero,
    Degenerate,
}

#[derive(Debug)]
enum BezierAlgebraicCuspParallelComponentReplay2 {
    Resolved(BezierAlgebraicCuspSemicircleParallelIntersections2),
    IsolatedCandidates {
        incidence: BivariatePolynomial,
        parameters: Vec<BezierParameter2>,
    },
    Degenerate,
}

#[derive(Clone, Debug, PartialEq)]
struct BezierAlgebraicCuspTwoTermExpression2 {
    /// Rational term `A`.
    rational: BivariatePolynomial,
    /// Radical coefficient `B`; the owning system determines whether the term
    /// is `B/sqrt(S)` or `B sqrt(K)`.
    radical: BivariatePolynomial,
}

/// Directed diameter coordinate retained by a rational-curve/circle map.
///
/// A one-field selected circle has a polynomial coordinate. A general
/// parallel-normal frame contributes exactly one source-speed radical. The
/// enum keeps the polynomial fast path allocation-identical while allowing
/// both representations to reuse the same local circle parameter map.
#[derive(Clone, Debug, PartialEq)]
enum BezierAlgebraicCuspSemicircleRationalDiameter2 {
    Rational(BivariatePolynomial),
    ParallelNormal {
        coordinate: BezierAlgebraicCuspTwoTermExpression2,
        speed_squared: BivariatePolynomial,
    },
}

/// Exact one-radical incidence between a general selected circle and a finite
/// rational Bezier. The first parameter selects the circle center on its
/// source parallel and the second parameter selects the rational point.
#[derive(Debug)]
struct BezierSelectedParallelNormalCircleRationalSystem2 {
    incidence: BivariatePolynomial,
    circle: BezierAlgebraicCuspTwoTermExpression2,
    selected_half_plane: BivariatePolynomial,
    diameter: BezierAlgebraicCuspTwoTermExpression2,
    radius_squared_denominator: BivariatePolynomial,
    speed_squared: BivariatePolynomial,
    tangent_cross: BezierAlgebraicCuspTwoTermExpression2,
    angular_tangent: BezierAlgebraicCuspTwoTermExpression2,
}

/// Simple radial-alignment incidence used when concentric support and signed
/// radii already certify a circle/circle tangency.  The unsquared relation and
/// radial-dot sign remove the conjugate normal and antipodal roots introduced
/// by squaring, while retaining the compact selected-fiber scalar.
#[derive(Debug)]
struct BezierSelectedParallelNormalRationalTangentCandidate2 {
    incidence: BivariatePolynomial,
    radial_alignment: BezierAlgebraicCuspTwoTermExpression2,
    radial_dot: BezierAlgebraicCuspTwoTermExpression2,
    speed_squared: BivariatePolynomial,
    expected_radial_dot_sign: RealSign,
}

#[derive(Debug)]
struct BezierAlgebraicCuspSemicircleParallelSystem2 {
    /// Squared circle incidence used only for complete candidate projection.
    incidence: BivariatePolynomial,
    /// Unsquared circle equation; zero selects the geometric radical branch.
    circle: BezierAlgebraicCuspTwoTermExpression2,
    /// Nonnegative exactly on the selected closed semicircle.
    selected_half_plane: BezierAlgebraicCuspTwoTermExpression2,
    /// Positive at the traversal start diameter endpoint and negative at end.
    diameter_side: BezierAlgebraicCuspTwoTermExpression2,
    /// Squared radius over the positive common denominator.
    radius_squared_denominator: BivariatePolynomial,
    /// Squared speed of the analytic parallel's source tangent.
    speed_squared: BivariatePolynomial,
    /// Tangent cross sign before the parallel derivative-scale correction.
    tangent_cross_source: BivariatePolynomial,
    /// Circle-tangent dot source-tangent sign before the parallel
    /// derivative-scale correction.
    tangent_dot_source: BezierAlgebraicCuspTwoTermExpression2,
}

#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspSemicircleParallelParameterMap2 {
    data: Arc<BezierAlgebraicCuspSemicircleParallelParameterMapData2>,
}

#[derive(Debug)]
struct BezierAlgebraicCuspSemicircleParallelParameterMapData2 {
    /// Stable exact carrier identity for rejecting mapped point evidence that
    /// was retained across a transform without transforming this correlation.
    semicircle: BezierAlgebraicCuspSemicircle2,
    parallel: BezierParallel2,
    system: BezierAlgebraicCuspSemicircleParallelParameterMapSystem2,
    policy: CurveContext,
    parameter_cache: BezierAlgebraicCuspSemicircleParameterCache2,
}

/// Exact parameter-order authority behind a common analytic-parallel contact.
/// Ordinary one-field circles retain the compact bivariate path. Selected
/// frames retain their shared quadratic tower and add only the exact target
/// root plus its procedural positive speed radical during predicate replay.
/// Directly represented frames keep the coordinate-based path.
#[derive(Debug)]
enum BezierAlgebraicCuspSemicircleParallelParameterMapSystem2 {
    OneField {
        cusp_parameter: BezierParameter2,
        /// A nondegenerate relation selecting correlated parallel parameters
        /// jointly with the cusp root. Independently authored range endpoints
        /// retain `correlated == false` and never consume it.
        incidence: BivariatePolynomial,
        diameter: BezierAlgebraicCuspTwoTermExpression2,
        radius_squared_denominator: BivariatePolynomial,
        speed_squared: BivariatePolynomial,
    },
    Recursive {
        system: Arc<BezierRecursiveCircleTargetSystem2>,
    },
    Represented {
        system: Arc<BezierRepresentedCircleParallelSystem2>,
    },
}

#[derive(Clone, Debug)]
enum BezierAlgebraicCuspSemicircleParallelCorrelation2 {
    Map,
    Independent,
    /// A clipped component already owns this circle parameter. Reuse its
    /// authority instead of solving a zero incidence polynomial again.
    Retained(BezierAlgebraicCuspSemicircleParameter2),
}

impl PartialEq for BezierAlgebraicCuspSemicircleParallelCorrelation2 {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Map, Self::Map) | (Self::Independent, Self::Independent) => true,
            (Self::Retained(first), Self::Retained(second)) => first.shares_exact_evidence(second),
            _ => false,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierAlgebraicCuspSemicircleParallelContact2 {
    pub(crate) parallel_parameter: BezierParameter2,
    pub(crate) tangent_cross_sign: Option<RealSign>,
    pub(crate) location: BezierAlgebraicCuspSemicircleContactLocation2,
    correlation: BezierAlgebraicCuspSemicircleParallelCorrelation2,
}

/// One circle/analytic contact whose analytic source coordinate remains in a
/// compact retained scalar field. This is the line-valued counterpart of a
/// selected-fiber contact: the authoritative circle/chord solve already owns
/// the point, cusp parameter, and tangent predicates, while the exact affine
/// line coordinate is also the analytic carrier parameter.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspSemicircleRetainedParallelContact2 {
    retained: BezierAlgebraicCuspSemicircleRetainedChordContact2,
    other_parameter: CurveParameter2,
    tangent_dot_sign: RealSign,
    circle_side_of_parallel: Option<crate::classify::LineSide>,
}

impl PartialEq for BezierAlgebraicCuspSemicircleRetainedParallelContact2 {
    fn eq(&self, other: &Self) -> bool {
        self.retained
            .cusp_parameter
            .shares_exact_evidence(&other.retained.cusp_parameter)
            && self.retained.chord_parameter == other.retained.chord_parameter
            && self.retained.point == other.retained.point
            && self.retained.tangent_cross_sign == other.retained.tangent_cross_sign
            && self.other_parameter == other.other_parameter
            && self.tangent_dot_sign == other.tangent_dot_sign
            && self.circle_side_of_parallel == other.circle_side_of_parallel
    }
}

#[derive(Clone, Debug)]
enum BezierAlgebraicCuspSemicircleMappedOverlapMap2 {
    Rational(BezierAlgebraicCuspSemicircleRationalParameterMap2),
    Parallel(BezierAlgebraicCuspSemicircleParallelParameterMap2),
}

/// One selected positive-length cusp-circle/mapped-curve overlap.
///
/// The other source range remains in ascending Bezier-parameter order.
/// Its endpoint locations identify exact cusp endpoints when the overlap is
/// clipped by the selected diameter; an interior endpoint is mapped lazily by
/// the pair-shared parameter authority. `orientation` compares
/// increasing parameters on the two supporting carriers, independently of
/// either boundary fragment's traversal bit.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspSemicircleMappedOverlap2 {
    other_range: CurveParameterRange2,
    cusp_start: BezierAlgebraicCuspSemicircleParameter2,
    cusp_end: BezierAlgebraicCuspSemicircleParameter2,
    orientation: CurveOverlapOrientation2,
    parameter_map: BezierAlgebraicCuspSemicircleMappedOverlapMap2,
    map_reversed: bool,
}

impl PartialEq for BezierAlgebraicCuspSemicircleMappedOverlap2 {
    fn eq(&self, other: &Self) -> bool {
        self.other_range == other.other_range
            && self.cusp_start.shares_exact_evidence(&other.cusp_start)
            && self.cusp_end.shares_exact_evidence(&other.cusp_end)
            && self.orientation == other.orientation
    }
}

fn rational_overlap_parameter_for_exact_cusp(
    map: &BezierAlgebraicCuspSemicircleRationalParameterMap2,
    parameter: &Real,
    range: &CurveParameterRange2,
    map_reversed: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<CurveParameter2>> {
    let data = &map.data;
    let map_range = if map_reversed {
        CurveParameterRange2::new_validated(
            range
                .start()
                .unit_complement()
                .ok_or(CurveError::InvalidCurveParameter)?,
            range
                .end()
                .unit_complement()
                .ok_or(CurveError::InvalidCurveParameter)?,
        )
    } else {
        range.clone()
    };
    if let BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Represented { frame } =
        &data.system
    {
        match in_closed_unit_interval(parameter, &policy.strict_counterpart()) {
            Some(true) => {}
            Some(false) => return Err(CurveError::InvalidBezierParameter),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
        let one_minus = Real::one() - parameter;
        let denominator = &one_minus * &one_minus + parameter * parameter;
        match real_sign(&denominator, &CurveContext::STRICT) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "represented semicircle inverse-map denominator was not positive".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radial = Real::one() - Real::from(2_i8) * parameter;
        let candidates = if let [Some(center_x), Some(center_y), Some(unit_x), Some(unit_y)] = [
            frame.center[0].exact_point_witness(),
            frame.center[1].exact_point_witness(),
            frame.unit_radial[0].exact_point_witness(),
            frame.unit_radial[1].exact_point_witness(),
        ] {
            let tangent = data.semicircle.turn_sign() * Real::from(2_i8) * parameter * &one_minus;
            let radius_scale = (&frame.signed_radius / &denominator)?;
            let point = Point2::new(
                center_x + &radius_scale * (&radial * unit_x - &tangent * unit_y),
                center_y + &radius_scale * (&radial * unit_y + &tangent * unit_x),
            );
            match policy.strict_predicate_pass(|| {
                data.curve
                    .point_incidence_on_range(&point, &map_range, policy)
            })? {
                Classification::Decided(
                    crate::rational_bezier_general::RationalBezierPointIncidence2::Parameters(
                        candidates,
                    ),
                ) => candidates,
                Classification::Decided(
                    crate::rational_bezier_general::RationalBezierPointIncidence2::EntireCurve,
                ) => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else {
            // Keep the angular inverse in the frame's selected coefficient
            // field. A represented frame need not have Cartesian Real witnesses.
            let system = match data.semicircle.represented_rational_component_system(
                &data.curve,
                &map_range,
                frame,
                policy,
            )? {
                Classification::Decided(system) => system,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let Some(predicate) = system.diameter.scale(&denominator).and_then(|diameter| {
                system
                    .radius_squared_denominator
                    .scale(&radial)
                    .and_then(|radius| diameter.subtract(&radius))
            }) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            match system.parameters(&predicate, &map_range, policy)? {
                Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                    candidates,
                )) => candidates,
                Classification::Decided(_) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        };
        let retained = retain_unique_overlap_parameter(
            curve_region_parameters_from_bezier(candidates),
            range,
            map_reversed,
            true,
            policy,
            |_| Ok(Classification::Decided(RealSign::Zero)),
        )?;
        if let Classification::Decided(candidate) = &retained
            && let Some(candidate) = candidate.as_bezier_parameter()
        {
            let map_parameter = if map_reversed {
                candidate.unit_complement()
            } else {
                candidate.clone()
            };
            map.data.parameter_cache.retain_scalar_value(
                map_parameter,
                Some(parameter.clone()),
                policy,
            );
        }
        return Ok(retained);
    }
    if let BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::ChordNormalProjective {
        system,
    } = &data.system
    {
        let one_minus = Real::one() - parameter;
        // (1-t)^2+t^2 is strictly positive for every real t.
        let denominator = &one_minus * &one_minus + parameter * parameter;
        let radial = Real::one() - Real::from(2_i8) * parameter;
        let Some(expression) = system.diameter_parameter_expression(&denominator, &radial) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let candidates = match chord_normal_dense_expression_parameters(
            system,
            &expression,
            SelectedThirdAxisDomain2::Finite(&map_range),
            policy,
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
                candidates
            }
            Classification::Decided(_) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        return retain_unique_overlap_parameter(
            curve_region_parameters_from_bezier(candidates),
            range,
            map_reversed,
            true,
            policy,
            |_| Ok(Classification::Decided(RealSign::Zero)),
        );
    }
    if let BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::Recursive { system } =
        &data.system
    {
        if !system.unit_target_speed {
            return Err(CurveError::Topology(
                "a recursive rational inverse map retained analytic target speed".into(),
            ));
        }
        let one_minus = Real::one() - parameter;
        let denominator = &one_minus * &one_minus + parameter * parameter;
        match real_sign(&denominator, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "semicircle inverse-map denominator was not positive".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radial_coefficient = Real::one() - Real::from(2_i8) * parameter;
        let Some(projection) =
            system.diameter_parameter_projection(&denominator, &radial_coefficient)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let candidates = match system.expression_parameters(
            &projection,
            SelectedThirdAxisDomain2::Finite(&map_range),
            policy,
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
                candidates
            }
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let retained = retain_unique_overlap_parameter(
            curve_region_parameters_from_bezier(candidates),
            range,
            map_reversed,
            true,
            policy,
            |candidate| {
                policy.strict_predicate_pass(|| {
                    system.diameter_parameter_sign(
                        candidate,
                        &denominator,
                        &radial_coefficient,
                        policy,
                    )
                })
            },
        )?;
        if let Classification::Decided(candidate) = &retained
            && let Some(candidate) = candidate.as_bezier_parameter()
        {
            let map_parameter = if map_reversed {
                candidate.unit_complement()
            } else {
                candidate.clone()
            };
            map.data.parameter_cache.retain_scalar_value(
                map_parameter,
                Some(parameter.clone()),
                policy,
            );
        }
        return Ok(retained);
    }
    if let BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::SelectedRadial {
        pair_map,
        branch,
        discriminant,
        diameter,
        radius_squared_denominator,
        ..
    } = &data.system
    {
        let one_minus = Real::one() - parameter;
        let denominator = &one_minus * &one_minus + parameter * parameter;
        match real_sign(&denominator, policy) {
            Some(RealSign::Positive) => {}
            Some(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "semicircle inverse-map denominator was not positive".into(),
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radial_coefficient = Real::one() - Real::from(2_i8) * parameter;
        let Some(rational) = TrivariatePolynomial::linear_combination(&[
            (&diameter.rational, &denominator),
            (radius_squared_denominator, &(-radial_coefficient)),
        ]) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(radical) = diameter.radical.scale(&denominator) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let expression = BezierAlgebraicCuspTrivariateSquareRootExpression2 { rational, radical };
        let Some([first_cusp_parameter, second_cusp_parameter]) =
            pair_map.compact_source_parameters()
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let (first_cusp_parameter, second_cusp_parameter) =
            (&first_cusp_parameter, &second_cusp_parameter);
        let candidates = match selected_pair_square_root_expression_third_axis_parameters(
            &expression,
            discriminant,
            first_cusp_parameter,
            second_cusp_parameter,
            *branch,
            SelectedThirdAxisDomain2::Finite(&map_range),
            policy,
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
                candidates
            }
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let retained = retain_unique_overlap_parameter(
            curve_region_parameters_from_bezier(candidates),
            range,
            map_reversed,
            true,
            policy,
            |_| Ok(Classification::Decided(RealSign::Zero)),
        )?;
        if let Classification::Decided(candidate) = &retained
            && let Some(candidate) = candidate.as_bezier_parameter()
        {
            let map_parameter = if map_reversed {
                candidate.unit_complement()
            } else {
                candidate.clone()
            };
            map.data.parameter_cache.retain_scalar_value(
                map_parameter,
                Some(parameter.clone()),
                policy,
            );
        }
        return Ok(retained);
    }
    let Some((cusp_parameter, _, diameter, radius_squared_denominator)) = data.one_field_system()
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let BezierParameter2::Algebraic(cusp_parameter) = cusp_parameter else {
        return Err(CurveError::Topology(
            "cusp/rational map lost its retained cusp root".into(),
        ));
    };
    let one_minus = Real::one() - parameter;
    let denominator = &one_minus * &one_minus + parameter * parameter;
    match real_sign(&denominator, policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Zero | RealSign::Negative) => {
            return Err(CurveError::Topology(
                "semicircle inverse-map denominator was not positive".into(),
            ));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let radial_coefficient = Real::one() - Real::from(2_i8) * parameter;
    let incidence = match diameter {
        BezierAlgebraicCuspSemicircleRationalDiameter2::Rational(diameter) => {
            bivariate_scaled_difference(
                diameter,
                &denominator,
                radius_squared_denominator,
                &radial_coefficient,
            )
        }
        BezierAlgebraicCuspSemicircleRationalDiameter2::ParallelNormal {
            coordinate,
            speed_squared,
        } => {
            let rational = bivariate_scaled_difference(
                &coordinate.rational,
                &denominator,
                radius_squared_denominator,
                &radial_coefficient,
            );
            let radical = bivariate_scale(coordinate.radical.clone(), &denominator);
            bivariate_subtract(
                &bivariate_multiply(&bivariate_multiply(&rational, &rational), speed_squared),
                &bivariate_multiply(&radical, &radical),
            )
        }
    };
    let incidence = match reduce_algebraic_cusp_bivariate(incidence, cusp_parameter, policy)? {
        Classification::Decided(incidence) => incidence,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let candidates = match algebraic_selected_reduced_fiber_parameters(
        &incidence,
        cusp_parameter,
        &map_range,
        policy,
    )? {
        Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
            candidates
        }
        Classification::Decided(
            BezierAlgebraicFiberProjection2::IdenticallyZero
            | BezierAlgebraicFiberProjection2::Degenerate,
        ) => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let retained = retain_unique_overlap_parameter(
        curve_region_parameters_from_bezier(candidates),
        range,
        map_reversed,
        true,
        policy,
        |_| Ok(Classification::Decided(RealSign::Zero)),
    )?;
    if let Classification::Decided(candidate) = &retained
        && let Some(candidate) = candidate.as_bezier_parameter()
    {
        let map_parameter = if map_reversed {
            candidate.unit_complement()
        } else {
            candidate.clone()
        };
        map.data.parameter_cache.retain_scalar_value(
            map_parameter,
            Some(parameter.clone()),
            policy,
        );
    }
    Ok(retained)
}

fn parallel_overlap_parameter_for_exact_cusp(
    map: &BezierAlgebraicCuspSemicircleParallelParameterMap2,
    parameter: &Real,
    range: &CurveParameterRange2,
    map_reversed: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<CurveParameter2>> {
    let map_range = if map_reversed {
        CurveParameterRange2::new_validated(
            range
                .start()
                .unit_complement()
                .ok_or(CurveError::InvalidCurveParameter)?,
            range
                .end()
                .unit_complement()
                .ok_or(CurveError::InvalidCurveParameter)?,
        )
    } else {
        range.clone()
    };
    let map = &map.data;
    let Some((cusp_parameter, _, diameter, radius_squared_denominator, speed_squared)) =
        map.one_field_system()
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let cusp_map_parameter = cusp_parameter;
    let BezierParameter2::Algebraic(cusp_parameter) = cusp_map_parameter else {
        return Err(CurveError::Topology(
            "cusp/parallel map lost its retained cusp root".into(),
        ));
    };
    let one_minus = Real::one() - parameter;
    let denominator = &one_minus * &one_minus + parameter * parameter;
    match real_sign(&denominator, policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Zero | RealSign::Negative) => {
            return Err(CurveError::Topology(
                "semicircle inverse-map denominator was not positive".into(),
            ));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let radial_coefficient = Real::one() - Real::from(2_i8) * parameter;
    let predicate = BezierAlgebraicCuspTwoTermExpression2 {
        rational: bivariate_scaled_difference(
            &diameter.rational,
            &denominator,
            radius_squared_denominator,
            &radial_coefficient,
        ),
        radical: bivariate_scale(diameter.radical.clone(), &denominator),
    };
    let incidence = bivariate_subtract(
        &bivariate_multiply(
            &bivariate_multiply(&predicate.rational, &predicate.rational),
            speed_squared,
        ),
        &bivariate_multiply(&predicate.radical, &predicate.radical),
    );
    let incidence = match reduce_algebraic_cusp_bivariate(incidence, cusp_parameter, policy)? {
        Classification::Decided(incidence) => incidence,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let cusp_root = parameter_representation(cusp_parameter, policy);
    let quotient = algebraic_selected_quotient_ring_fiber_projection_with_max_degree(
        &incidence,
        &cusp_root,
        MAX_SELECTED_FIBER_QUOTIENT_DEGREE,
        &map_range,
        policy,
    )?;
    // The quotient norm includes candidates contributed by conjugate cusp
    // roots. The unsquared replay below evaluates every candidate at the
    // selected cusp root, so it rejects both those foreign candidates and
    // the opposite radical branch in one exact predicate. Running the
    // general selected-fiber membership pass first would duplicate that
    // proof and can dominate reversal-heavy clipping.
    let candidates = match quotient {
        Classification::Decided(ResultantParameterProjection::Empty) => Vec::new(),
        Classification::Decided(
            ResultantParameterProjection::Parameters(candidates)
            | ResultantParameterProjection::SelectedParameters(candidates),
        ) => candidates,
        Classification::Decided(ResultantParameterProjection::Degenerate)
        | Classification::Uncertain(_) => {
            match selected_fiber_parameters(
                &incidence,
                &BezierParameter2::Algebraic(cusp_parameter.clone()),
                &map_range,
                policy,
            )? {
                Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                    candidates,
                )) => candidates,
                Classification::Decided(
                    BezierAlgebraicFiberProjection2::IdenticallyZero
                    | BezierAlgebraicFiberProjection2::Degenerate,
                ) => return Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    };
    retain_unique_overlap_parameter(
        curve_region_parameters_from_bezier(candidates),
        range,
        map_reversed,
        true,
        policy,
        |map_parameter| {
            let map_parameter = map_parameter.as_bezier_parameter().ok_or_else(|| {
                CurveError::Topology("a parallel inverse produced a non-Bezier candidate".into())
            })?;
            algebraic_cusp_correlated_radical_sum_sign(
                &incidence,
                &predicate,
                speed_squared,
                cusp_map_parameter,
                map_parameter,
                policy,
            )
        },
    )
}

fn promote_curve_region_bezier_parameter(
    parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParameter2>> {
    if let Some(parameter) = parameter.as_bezier_parameter() {
        return Ok(Classification::Decided(parameter.clone()));
    }
    policy.strict_predicate_pass(|| parameter.promoted_bezier_parameter_complete(policy))
}

fn rational_mapped_cusp_scalar_value(
    map: &BezierAlgebraicCuspSemicircleRationalParameterMap2,
    contact: &BezierAlgebraicCuspSemicircleRationalMapContact2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Real>>> {
    let BezierAlgebraicCuspSemicircleRationalParameterMapSystem2::OneField {
        cusp_parameter,
        diameter,
        radius_squared_denominator,
        ..
    } = &map.data.system
    else {
        // Projecting this pair-radial angular value would require eliminating
        // both selected source roots. Keep its exact mapped representation
        // until a caller needs that scalar projection.
        return Ok(Classification::Decided(None));
    };
    let other_parameter =
        match promote_curve_region_bezier_parameter(&contact.other_parameter, policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    if let Some(value) = map
        .data
        .parameter_cache
        .cached_scalar_value(&other_parameter, policy)
    {
        return Ok(Classification::Decided(value));
    }
    let represented_other_parameter = match other_parameter
        .clone()
        .promote_represented_exact_point(policy)?
    {
        Classification::Decided(BezierParameter2::Exact(parameter)) => parameter,
        Classification::Decided(BezierParameter2::Algebraic(_)) => {
            return Ok(Classification::Decided(None));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let radius_squared_denominator =
        bivariate_specialize_second(radius_squared_denominator, &represented_other_parameter);
    let denominator = [Real::one(), Real::from(-2_i8), Real::from(2_i8)];
    let radial = [Real::one(), Real::from(-2_i8)];
    let incidence = match diameter {
        BezierAlgebraicCuspSemicircleRationalDiameter2::Rational(diameter) => {
            let diameter = bivariate_specialize_second(diameter, &represented_other_parameter);
            bivariate_subtract(
                &bivariate_tensor_product(&diameter, &denominator),
                &bivariate_tensor_product(&radius_squared_denominator, &radial),
            )
        }
        BezierAlgebraicCuspSemicircleRationalDiameter2::ParallelNormal {
            coordinate,
            speed_squared,
        } => {
            let diameter_rational =
                bivariate_specialize_second(&coordinate.rational, &represented_other_parameter);
            let diameter_radical =
                bivariate_specialize_second(&coordinate.radical, &represented_other_parameter);
            let speed_squared =
                bivariate_specialize_second(speed_squared, &represented_other_parameter);
            let rational = bivariate_subtract(
                &bivariate_tensor_product(&diameter_rational, &denominator),
                &bivariate_tensor_product(&radius_squared_denominator, &radial),
            );
            let radical = bivariate_tensor_product(&diameter_radical, &denominator);
            bivariate_subtract(
                &bivariate_multiply(
                    &bivariate_multiply(&rational, &rational),
                    &bivariate_tensor_product(&speed_squared, &[Real::one()]),
                ),
                &bivariate_multiply(&radical, &radical),
            )
        }
    };
    let result =
        mapped_cusp_scalar_value_from_incidence(incidence, cusp_parameter, policy, |parameter| {
            map.mapped_contact_order_to_real(contact, parameter, policy)
        })?;
    if let Classification::Decided(value) = &result {
        map.data
            .parameter_cache
            .retain_scalar_value(other_parameter, value.clone(), policy);
    }
    Ok(result)
}

fn parallel_mapped_cusp_scalar_value(
    map: &BezierAlgebraicCuspSemicircleParallelParameterMap2,
    contact: &BezierAlgebraicCuspSemicircleParallelContact2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Real>>> {
    if let Some(value) = map
        .data
        .parameter_cache
        .cached_scalar_value(&contact.parallel_parameter, policy)
    {
        return Ok(Classification::Decided(value));
    }
    let Some((cusp_parameter, _, diameter, radius_squared_denominator, speed_squared)) =
        map.data.one_field_system()
    else {
        // A pair-radial map remains an exact procedural scalar. An independent
        // scalar witness would require a second three-axis projection with
        // the compact cusp parameter as its target; retain the exact map
        // until that projection is needed.
        return Ok(Classification::Decided(None));
    };
    let other_parameter = match contact
        .parallel_parameter
        .clone()
        .promote_represented_exact_point(policy)?
    {
        Classification::Decided(BezierParameter2::Exact(parameter)) => parameter,
        Classification::Decided(BezierParameter2::Algebraic(_)) => {
            return Ok(Classification::Decided(None));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let diameter_rational = bivariate_specialize_second(&diameter.rational, &other_parameter);
    let diameter_radical = bivariate_specialize_second(&diameter.radical, &other_parameter);
    let radius_squared_denominator =
        bivariate_specialize_second(radius_squared_denominator, &other_parameter);
    let speed_squared = bivariate_specialize_second(speed_squared, &other_parameter);
    let denominator = [Real::one(), Real::from(-2_i8), Real::from(2_i8)];
    let rational = bivariate_subtract(
        &bivariate_tensor_product(&diameter_rational, &denominator),
        &bivariate_tensor_product(
            &radius_squared_denominator,
            &[Real::one(), Real::from(-2_i8)],
        ),
    );
    let radical = bivariate_tensor_product(&diameter_radical, &denominator);
    let incidence = bivariate_subtract(
        &bivariate_multiply(
            &bivariate_multiply(&rational, &rational),
            &bivariate_tensor_product(&speed_squared, &[Real::one()]),
        ),
        &bivariate_multiply(&radical, &radical),
    );
    let result =
        mapped_cusp_scalar_value_from_incidence(incidence, cusp_parameter, policy, |parameter| {
            map.contact_order_to_real(contact, parameter, policy)
        })?;
    if let Classification::Decided(value) = &result {
        map.data.parameter_cache.retain_scalar_value(
            contact.parallel_parameter.clone(),
            value.clone(),
            policy,
        );
    }
    Ok(result)
}

fn mapped_cusp_scalar_value_from_incidence(
    incidence: BivariatePolynomial,
    cusp_parameter: &BezierParameter2,
    policy: &CurveContext,
    mut order_to_real: impl FnMut(&Real) -> CurveResult<Classification<std::cmp::Ordering>>,
) -> CurveResult<Classification<Option<Real>>> {
    let BezierParameter2::Algebraic(cusp_parameter) = cusp_parameter else {
        return Ok(Classification::Decided(None));
    };
    let incidence = match reduce_algebraic_cusp_bivariate(incidence, cusp_parameter, policy)? {
        Classification::Decided(incidence) => incidence,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let candidates = match algebraic_selected_reduced_fiber_parameters(
        &incidence,
        cusp_parameter,
        &crate::CurveParameterRange2::unit(),
        policy,
    )? {
        Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
            candidates
        }
        Classification::Decided(
            BezierAlgebraicFiberProjection2::IdenticallyZero
            | BezierAlgebraicFiberProjection2::Degenerate,
        ) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let mut retained = None;
    for candidate in candidates {
        let candidate = match candidate.promote_represented_exact_point(policy)? {
            Classification::Decided(BezierParameter2::Exact(candidate)) => candidate,
            Classification::Decided(BezierParameter2::Algebraic(_)) => continue,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match order_to_real(&candidate)? {
            Classification::Decided(std::cmp::Ordering::Equal) if retained.is_none() => {
                retained = Some(candidate);
            }
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return Err(CurveError::Topology(
                    "mapped cusp cut had multiple scalar value witnesses".into(),
                ));
            }
            Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Greater) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    }
    Ok(Classification::Decided(retained))
}

/// Tests same and complemented native parameters by the original unsquared
/// diameter coordinate. This is a per-cut proof: an isolated equality is
/// sufficient and cannot be mistaken for a global map equivalence.
fn rational_parallel_parameter_orientation_at_cut(
    rational: &BezierAlgebraicCuspSemicircleRationalParameterMap2,
    parallel: &BezierAlgebraicCuspSemicircleParallelParameterMap2,
    source_parameter: &BezierParameter2,
    source_is_rational: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<CurveOverlapOrientation2>>> {
    if !policy.accepts_retained_policy(rational.data.policy)
        || !policy.accepts_retained_policy(parallel.data.policy)
    {
        return Err(CurveError::Topology(
            "cross-map cusp parameterization comparison used a different predicate policy".into(),
        ));
    }
    let Some((rational_cusp_parameter, _, _, _)) = rational.data.one_field_system() else {
        return Ok(Classification::Decided(None));
    };
    let Some((parallel_cusp_parameter, _, _, _, _)) = parallel.data.one_field_system() else {
        return Ok(Classification::Decided(None));
    };
    match rational_cusp_parameter.same_value(parallel_cusp_parameter, policy)? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let mut uncertain = None;
    for orientation in [
        CurveOverlapOrientation2::Same,
        CurveOverlapOrientation2::Reversed,
    ] {
        let rational_parameter =
            if source_is_rational || orientation == CurveOverlapOrientation2::Same {
                source_parameter.clone()
            } else {
                source_parameter.unit_complement()
            };
        match rational_parallel_parameters_match_at_cut(
            rational,
            parallel,
            &rational_parameter,
            orientation,
            policy,
        )? {
            Classification::Decided(true) => {
                return Ok(Classification::Decided(Some(orientation)));
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => uncertain = Some(reason),
        }
    }
    Ok(uncertain.map_or(Classification::Decided(None), Classification::Uncertain))
}

fn rational_parallel_parameters_match_at_cut(
    rational: &BezierAlgebraicCuspSemicircleRationalParameterMap2,
    parallel: &BezierAlgebraicCuspSemicircleParallelParameterMap2,
    rational_parameter: &BezierParameter2,
    orientation: CurveOverlapOrientation2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    rational_parallel_diameter_relation_at_cut(
        rational,
        parallel,
        rational_parameter,
        orientation,
        false,
        policy,
    )
}

fn algebraic_cusp_independent_two_radical_sum_is_zero(
    rational: &BivariatePolynomial,
    first_radical: &BivariatePolynomial,
    first_radicand: &BivariatePolynomial,
    second_radical: &BivariatePolynomial,
    second_radicand: &BivariatePolynomial,
    cusp_parameter: &BezierParameter2,
    other_parameter: &BezierParameter2,
) -> CurveResult<Classification<bool>> {
    let exact = &CurveContext::STRICT;
    let sign = |polynomial: &BivariatePolynomial| {
        signed_bivariate_at_parameter_pair(polynomial, cusp_parameter, other_parameter, exact)
    };
    let rational_sign = match sign(rational)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let first_sign = match sign(first_radical)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let second_sign = match sign(second_radical)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    for radicand in [first_radicand, second_radicand] {
        match sign(radicand)? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero | RealSign::Negative) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }

    let signs = [rational_sign, first_sign, second_sign];
    let mut nonzero = [0_usize; 3];
    let mut nonzero_count = 0_usize;
    for (index, sign) in signs.into_iter().enumerate() {
        if sign != RealSign::Zero {
            nonzero[nonzero_count] = index;
            nonzero_count += 1;
        }
    }
    match nonzero_count {
        0 => return Ok(Classification::Decided(true)),
        1 => return Ok(Classification::Decided(false)),
        2 => {
            let first = nonzero[0];
            let second = nonzero[1];
            if signs[first] == signs[second] {
                return Ok(Classification::Decided(false));
            }
        }
        3 if rational_sign == first_sign && first_sign == second_sign => {
            return Ok(Classification::Decided(false));
        }
        3 => {}
        _ => unreachable!("the radical sum has exactly three terms"),
    }

    // Multiplication by both positive square roots changes
    //
    //     A + B/sqrt(S) + C/sqrt(T)
    //
    // into X+Y+Z, where X=A*sqrt(S*T), Y=B*sqrt(T), and
    // Z=C*sqrt(S). Their signs are the coefficient signs above and their
    // squares are ordinary bivariate polynomials. Exact cancellation of two
    // terms is equality of their squares with opposite signs. With three
    // terms, isolate the uniquely signed term L from the two same-signed
    // terms M,N and certify
    //
    //     D=L^2-M^2-N^2 > 0,   D^2=4*M^2*N^2.
    //
    // The sign precondition rejects every conjugate introduced by squaring.
    let square = |index| match index {
        0 => bivariate_multiply(
            &bivariate_multiply(rational, rational),
            &bivariate_multiply(first_radicand, second_radicand),
        ),
        1 => bivariate_multiply(
            &bivariate_multiply(first_radical, first_radical),
            second_radicand,
        ),
        2 => bivariate_multiply(
            &bivariate_multiply(second_radical, second_radical),
            first_radicand,
        ),
        _ => unreachable!("the radical sum has exactly three terms"),
    };
    if nonzero_count == 2 {
        return Ok(
            match sign(&bivariate_subtract(
                &square(nonzero[0]),
                &square(nonzero[1]),
            ))? {
                Classification::Decided(RealSign::Zero) => Classification::Decided(true),
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                    Classification::Decided(false)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        );
    }
    let squares = [square(0), square(1), square(2)];
    let odd = if rational_sign != first_sign && rational_sign != second_sign {
        0
    } else if first_sign != rational_sign && first_sign != second_sign {
        1
    } else {
        2
    };
    let [first_same, second_same] = match odd {
        0 => [1, 2],
        1 => [0, 2],
        2 => [0, 1],
        _ => unreachable!(),
    };
    let magnitude_difference = bivariate_subtract(
        &bivariate_subtract(&squares[odd], &squares[first_same]),
        &squares[second_same],
    );
    match sign(&magnitude_difference)? {
        Classification::Decided(RealSign::Positive) => {}
        Classification::Decided(RealSign::Zero | RealSign::Negative) => {
            return Ok(Classification::Decided(false));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let conjugate_residual = bivariate_subtract(
        &bivariate_multiply(&magnitude_difference, &magnitude_difference),
        &bivariate_scale(
            bivariate_multiply(&squares[first_same], &squares[second_same]),
            &Real::from(4_i8),
        ),
    );
    Ok(match sign(&conjugate_residual)? {
        Classification::Decided(RealSign::Zero) => Classification::Decided(true),
        Classification::Decided(RealSign::Positive | RealSign::Negative) => {
            Classification::Decided(false)
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

fn parallel_parameters_are_complementary_at_cut(
    first: &BezierAlgebraicCuspSemicircleParallelParameterMap2,
    second: &BezierAlgebraicCuspSemicircleParallelParameterMap2,
    shared_parameter: &BezierParameter2,
    orientation: CurveOverlapOrientation2,
    construction_policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    if !construction_policy.accepts_retained_policy(first.data.policy)
        || !construction_policy.accepts_retained_policy(second.data.policy)
    {
        return Err(CurveError::Topology(
            "parallel cusp diameter comparison used a different predicate policy".into(),
        ));
    }

    // This relation creates reusable axis evidence, so every equality is
    // proved under STRICT even when the enclosing construction was authored
    // with APPROXIMATE_512. The construction policy above validates the
    // retained maps; it is not permission to turn a terminal equality into a
    // cardinal chord certificate.
    let exact = &CurveContext::STRICT;
    let Some((first_cusp, _, first_diameter, first_radius, first_speed)) =
        first.data.one_field_system()
    else {
        return Ok(Classification::Decided(false));
    };
    let Some((second_cusp, _, second_diameter, second_radius, second_speed)) =
        second.data.one_field_system()
    else {
        return Ok(Classification::Decided(false));
    };
    match first_cusp.same_value(second_cusp, exact)? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }

    // Each directed diameter coordinate has the form
    //
    //     (A + B/sqrt(S)) / D,       D > 0.
    //
    // Cross-multiply the positive denominators. Equal speed roots reduce to
    // the cheaper two-term radical signer; otherwise the exact two-radical
    // norm identity below rejects every conjugate introduced by squaring. A
    // zero sum is precisely the unit-complement relation on the selected
    // semicircle.
    let second_speed_squared = bivariate_orient_second_parameter(second_speed, orientation);
    let second_radius_squared_denominator =
        bivariate_orient_second_parameter(second_radius, orientation);
    let second_diameter_rational =
        bivariate_orient_second_parameter(&second_diameter.rational, orientation);
    let second_diameter_radical =
        bivariate_orient_second_parameter(&second_diameter.radical, orientation);
    let common_denominator = first_radius == second_radius_squared_denominator.as_ref();
    let rational = if common_denominator {
        bivariate_add(&first_diameter.rational, second_diameter_rational.as_ref())
    } else {
        bivariate_add(
            &bivariate_multiply(
                &first_diameter.rational,
                second_radius_squared_denominator.as_ref(),
            ),
            &bivariate_multiply(second_diameter_rational.as_ref(), first_radius),
        )
    };
    let first_radical = if common_denominator {
        Cow::Borrowed(&first_diameter.radical)
    } else {
        Cow::Owned(bivariate_multiply(
            &first_diameter.radical,
            second_radius_squared_denominator.as_ref(),
        ))
    };
    let second_radical = if common_denominator {
        Cow::Borrowed(second_diameter_radical.as_ref())
    } else {
        Cow::Owned(bivariate_multiply(
            second_diameter_radical.as_ref(),
            first_radius,
        ))
    };
    let speeds_equal = first_speed == second_speed_squared.as_ref() || {
        let speed_difference = bivariate_subtract(first_speed, second_speed_squared.as_ref());
        matches!(
            signed_bivariate_at_parameter_pair(
                &speed_difference,
                first_cusp,
                shared_parameter,
                exact,
            )?,
            Classification::Decided(RealSign::Zero)
        )
    };
    if speeds_equal {
        let expression = BezierAlgebraicCuspTwoTermExpression2 {
            rational,
            radical: bivariate_add(first_radical.as_ref(), second_radical.as_ref()),
        };
        return Ok(
            match algebraic_cusp_independent_radical_sum_sign(
                &expression,
                first_speed,
                first_cusp,
                shared_parameter,
                exact,
            )? {
                Classification::Decided(RealSign::Zero) => Classification::Decided(true),
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                    Classification::Decided(false)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        );
    }
    algebraic_cusp_independent_two_radical_sum_is_zero(
        &rational,
        first_radical.as_ref(),
        first_speed,
        second_radical.as_ref(),
        second_speed_squared.as_ref(),
        first_cusp,
        shared_parameter,
    )
}

fn scaled_oriented_rational_diameter(
    diameter: &BezierAlgebraicCuspSemicircleRationalDiameter2,
    scale: &BivariatePolynomial,
    orientation: CurveOverlapOrientation2,
    negate: bool,
) -> (
    BivariatePolynomial,
    Option<(BivariatePolynomial, BivariatePolynomial)>,
) {
    let signed = |polynomial: BivariatePolynomial| {
        if negate {
            bivariate_scale(polynomial, &Real::from(-1_i8))
        } else {
            polynomial
        }
    };
    match diameter {
        BezierAlgebraicCuspSemicircleRationalDiameter2::Rational(coordinate) => {
            let coordinate = bivariate_orient_second_parameter(coordinate, orientation);
            (signed(bivariate_multiply(coordinate.as_ref(), scale)), None)
        }
        BezierAlgebraicCuspSemicircleRationalDiameter2::ParallelNormal {
            coordinate,
            speed_squared,
        } => {
            let rational = bivariate_orient_second_parameter(&coordinate.rational, orientation);
            let radical = bivariate_orient_second_parameter(&coordinate.radical, orientation);
            let speed_squared =
                bivariate_orient_second_parameter(speed_squared, orientation).into_owned();
            (
                signed(bivariate_multiply(rational.as_ref(), scale)),
                Some((
                    signed(bivariate_multiply(radical.as_ref(), scale)),
                    speed_squared,
                )),
            )
        }
    }
}

fn independent_diameter_sum_is_zero(
    rational: BivariatePolynomial,
    first_radical: Option<(BivariatePolynomial, BivariatePolynomial)>,
    second_radical: Option<(BivariatePolynomial, BivariatePolynomial)>,
    cusp_parameter: &BezierParameter2,
    other_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let zero = |sign| match sign {
        Classification::Decided(RealSign::Zero) => Classification::Decided(true),
        Classification::Decided(RealSign::Positive | RealSign::Negative) => {
            Classification::Decided(false)
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    };
    match (first_radical, second_radical) {
        (None, None) => Ok(zero(signed_bivariate_at_parameter_pair(
            &rational,
            cusp_parameter,
            other_parameter,
            policy,
        )?)),
        (Some((radical, speed_squared)), None) | (None, Some((radical, speed_squared))) => {
            Ok(zero(algebraic_cusp_independent_radical_sum_sign(
                &BezierAlgebraicCuspTwoTermExpression2 { rational, radical },
                &speed_squared,
                cusp_parameter,
                other_parameter,
                policy,
            )?))
        }
        (
            Some((first_radical, first_speed_squared)),
            Some((second_radical, second_speed_squared)),
        ) => {
            let speeds_equal = first_speed_squared == second_speed_squared || {
                let difference = bivariate_subtract(&first_speed_squared, &second_speed_squared);
                matches!(
                    signed_bivariate_at_parameter_pair(
                        &difference,
                        cusp_parameter,
                        other_parameter,
                        &CurveContext::STRICT,
                    )?,
                    Classification::Decided(RealSign::Zero)
                )
            };
            if speeds_equal {
                return Ok(zero(algebraic_cusp_independent_radical_sum_sign(
                    &BezierAlgebraicCuspTwoTermExpression2 {
                        rational,
                        radical: bivariate_add(&first_radical, &second_radical),
                    },
                    &first_speed_squared,
                    cusp_parameter,
                    other_parameter,
                    &CurveContext::STRICT,
                )?));
            }
            algebraic_cusp_independent_two_radical_sum_is_zero(
                &rational,
                &first_radical,
                &first_speed_squared,
                &second_radical,
                &second_speed_squared,
                cusp_parameter,
                other_parameter,
            )
        }
    }
}

fn rational_parameters_are_complementary_at_cut(
    first: &BezierAlgebraicCuspSemicircleRationalParameterMap2,
    second: &BezierAlgebraicCuspSemicircleRationalParameterMap2,
    shared_parameter: &BezierParameter2,
    orientation: CurveOverlapOrientation2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    if !policy.accepts_retained_policy(first.data.policy)
        || !policy.accepts_retained_policy(second.data.policy)
    {
        return Err(CurveError::Topology(
            "rational cusp diameter comparison used a different predicate policy".into(),
        ));
    }
    let (
        Some((first_cusp_parameter, _, first_diameter, first_radius)),
        Some((second_cusp_parameter, _, second_diameter, second_radius)),
    ) = (
        first.data.one_field_system(),
        second.data.one_field_system(),
    )
    else {
        return Ok(Classification::Decided(false));
    };
    match first_cusp_parameter.same_value(second_cusp_parameter, policy)? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let oriented_second_radius = bivariate_orient_second_parameter(second_radius, orientation);
    let (first_rational, first_radical) = scaled_oriented_rational_diameter(
        first_diameter,
        oriented_second_radius.as_ref(),
        CurveOverlapOrientation2::Same,
        false,
    );
    let (second_rational, second_radical) =
        scaled_oriented_rational_diameter(second_diameter, first_radius, orientation, false);
    independent_diameter_sum_is_zero(
        bivariate_add(&first_rational, &second_rational),
        first_radical,
        second_radical,
        first_cusp_parameter,
        shared_parameter,
        policy,
    )
}

fn rational_parallel_diameter_relation_at_cut(
    rational: &BezierAlgebraicCuspSemicircleRationalParameterMap2,
    parallel: &BezierAlgebraicCuspSemicircleParallelParameterMap2,
    rational_parameter: &BezierParameter2,
    orientation: CurveOverlapOrientation2,
    opposite: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    if !policy.accepts_retained_policy(rational.data.policy)
        || !policy.accepts_retained_policy(parallel.data.policy)
    {
        return Err(CurveError::Topology(
            "cross-map cusp diameter comparison used a different predicate policy".into(),
        ));
    }
    let Some((rational_cusp_parameter, _, rational_diameter, rational_radius)) =
        rational.data.one_field_system()
    else {
        return Ok(Classification::Decided(false));
    };
    let Some((parallel_cusp_parameter, _, parallel_diameter, parallel_radius, parallel_speed)) =
        parallel.data.one_field_system()
    else {
        return Ok(Classification::Decided(false));
    };
    match rational_cusp_parameter.same_value(parallel_cusp_parameter, policy)? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let parallel_radius_squared_denominator =
        bivariate_orient_second_parameter(parallel_radius, orientation);
    let parallel_diameter = BezierAlgebraicCuspSemicircleRationalDiameter2::ParallelNormal {
        coordinate: parallel_diameter.clone(),
        speed_squared: parallel_speed.clone(),
    };
    let (parallel_rational, parallel_radical) =
        scaled_oriented_rational_diameter(&parallel_diameter, rational_radius, orientation, false);
    let (rational_term, rational_radical) = scaled_oriented_rational_diameter(
        rational_diameter,
        parallel_radius_squared_denominator.as_ref(),
        CurveOverlapOrientation2::Same,
        !opposite,
    );
    independent_diameter_sum_is_zero(
        bivariate_add(&parallel_rational, &rational_term),
        parallel_radical,
        rational_radical,
        rational_cusp_parameter,
        rational_parameter,
        policy,
    )
}

fn bivariate_orient_second_parameter<'a>(
    polynomial: &'a BivariatePolynomial,
    orientation: CurveOverlapOrientation2,
) -> Cow<'a, BivariatePolynomial> {
    if orientation == CurveOverlapOrientation2::Same {
        Cow::Borrowed(polynomial)
    } else {
        Cow::Owned(bivariate_complement_second_parameter(polynomial))
    }
}

fn retain_unique_overlap_parameter<F>(
    candidates: Vec<CurveParameter2>,
    range: &CurveParameterRange2,
    map_reversed: bool,
    include_boundaries: bool,
    policy: &CurveContext,
    mut predicate_sign: F,
) -> CurveResult<Classification<CurveParameter2>>
where
    F: FnMut(&CurveParameter2) -> CurveResult<Classification<RealSign>>,
{
    let mut retained = None;
    for map_parameter in candidates {
        let sign = match predicate_sign(&map_parameter)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if sign != RealSign::Zero {
            continue;
        }
        let candidate = if map_reversed {
            map_parameter.unit_complement().ok_or_else(|| {
                CurveError::Topology(
                    "a mapped overlap candidate had no scalar unit-complement".into(),
                )
            })?
        } else {
            map_parameter
        };
        let mut in_range = match CurveParameterDomain2::new(range, None)
            .contains_finite_parameter(&candidate, policy)?
        {
            Classification::Decided(in_range) => in_range,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if in_range && !include_boundaries {
            for endpoint in [range.start(), range.end()] {
                match candidate.same_value(endpoint, policy)? {
                    Classification::Decided(true) => in_range = false,
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        if in_range && retained.replace(candidate).is_some() {
            return Err(CurveError::Topology(
                "one cusp cut mapped to multiple parameters in one regular overlap cell".into(),
            ));
        }
    }
    match retained {
        Some(parameter) => Ok(Classification::Decided(parameter)),
        None => Err(CurveError::Topology(
            "cusp cut had no parameter on its published mapped overlap".into(),
        )),
    }
}

fn curve_region_parameters_from_bezier(parameters: Vec<BezierParameter2>) -> Vec<CurveParameter2> {
    parameters.into_iter().map(CurveParameter2::from).collect()
}

fn retain_direct_overlap_parameter(
    parameter: CurveParameter2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurveParameter2>> {
    match CurveParameterDomain2::new(range, None).contains_finite_parameter(&parameter, policy)? {
        Classification::Decided(true) => Ok(Classification::Decided(parameter)),
        Classification::Decided(false) => Err(CurveError::Topology(
            "cusp cut had no parameter on its published mapped overlap".into(),
        )),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(crate) fn overlap_parameter_is_in_range(
    parameter: &CurveParameter2,
    range: &BezierParameterRange2,
    include_boundaries: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let start_parameter = CurveParameter2::from(range.start().clone());
    let end_parameter = CurveParameter2::from(range.end().clone());
    let orientation = match start_parameter.cmp_by_refinement(&end_parameter, policy)? {
        Classification::Decided(std::cmp::Ordering::Less) => std::cmp::Ordering::Less,
        Classification::Decided(std::cmp::Ordering::Greater) => std::cmp::Ordering::Greater,
        Classification::Decided(std::cmp::Ordering::Equal) => {
            return Err(CurveError::DegenerateOverlapRange);
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let start = match parameter.cmp_by_refinement(&start_parameter, policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end = match parameter.cmp_by_refinement(&end_parameter, policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(match orientation {
        std::cmp::Ordering::Less => {
            (start.is_gt() || (include_boundaries && start.is_eq()))
                && (end.is_lt() || (include_boundaries && end.is_eq()))
        }
        std::cmp::Ordering::Greater => {
            (start.is_lt() || (include_boundaries && start.is_eq()))
                && (end.is_gt() || (include_boundaries && end.is_eq()))
        }
        std::cmp::Ordering::Equal => unreachable!("the overlap range is positive-length"),
    }))
}

fn bezier_parameter_is_in_curve_region_range(
    parameter: &BezierParameter2,
    range: &CurveParameterRange2,
    include_boundaries: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let parameter = CurveParameter2::from(parameter.clone());
    let start = match parameter.cmp_by_refinement(range.start(), policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let end = match parameter.cmp_by_refinement(range.end(), policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(
        (start.is_gt() || (include_boundaries && start.is_eq()))
            && (end.is_lt() || (include_boundaries && end.is_eq())),
    ))
}

fn mapped_parameters_for_cusp_endpoint(
    contacts: impl Iterator<
        Item = (
            BezierAlgebraicCuspSemicircleContactLocation2,
            CurveParameter2,
        ),
    >,
    overlaps: Vec<BezierAlgebraicCuspSemicircleMappedOverlap2>,
    parameter: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<CurveParameter2>>> {
    let expected_location = if parameter == &Real::zero() {
        BezierAlgebraicCuspSemicircleContactLocation2::Start
    } else if parameter == &Real::one() {
        BezierAlgebraicCuspSemicircleContactLocation2::End
    } else {
        return Err(CurveError::InvalidCurveParameter);
    };
    let parameter = BezierAlgebraicCuspSemicircleParameter2::Exact(parameter.clone());
    let mut candidates: Vec<CurveParameter2> = contacts
        .filter_map(|(location, other)| (location == expected_location).then_some(other))
        .collect();
    for overlap in overlaps {
        let after_start = match parameter.cmp_by_refinement(&overlap.cusp_start, policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => false,
            Classification::Decided(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater) => {
                true
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let before_end = match parameter.cmp_by_refinement(&overlap.cusp_end, policy)? {
            Classification::Decided(std::cmp::Ordering::Greater) => false,
            Classification::Decided(std::cmp::Ordering::Equal | std::cmp::Ordering::Less) => true,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if !after_start || !before_end {
            continue;
        }
        let candidate = match overlap.other_parameter_for_cusp(&parameter, policy)? {
            Classification::Decided(candidate) => candidate,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut duplicate = false;
        for existing in &candidates {
            match existing.same_value(&candidate, policy)? {
                Classification::Decided(true) => {
                    duplicate = true;
                    break;
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if !duplicate {
            candidates.push(candidate);
        }
    }
    Ok(Classification::Decided(candidates))
}

fn rational_parameters_for_cusp_endpoint(
    source: &BezierAlgebraicCuspSemicircle2,
    parameter: &Real,
    target: &RationalBezier2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<CurveParameter2>>> {
    match source.rational_intersections(target, range, policy)? {
        Classification::Decided(BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped {
            contacts,
            overlaps,
        }) => mapped_parameters_for_cusp_endpoint(
            contacts
                .into_iter()
                .map(|contact| (contact.location, contact.other_parameter)),
            overlaps,
            parameter,
            policy,
        ),
        Classification::Decided(
            BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection
            | BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber { .. },
        ) => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn parallel_parameters_for_cusp_endpoint(
    source: &BezierAlgebraicCuspSemicircle2,
    parameter: &Real,
    target: &BezierParallel2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<CurveParameter2>>> {
    match source.parallel_intersections(target, range, None, policy)? {
        Classification::Decided(BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped {
            contacts,
            overlaps,
        }) => mapped_parameters_for_cusp_endpoint(
            contacts.into_iter().map(|contact| {
                (
                    contact.location,
                    CurveParameter2::from(contact.parallel_parameter),
                )
            }),
            overlaps,
            parameter,
            policy,
        ),
        Classification::Decided(
            BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent
            | BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection
            | BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiber { .. }
            | BezierAlgebraicCuspSemicircleParallelIntersections2::RetainedContacts(_),
        ) => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

/// Projects one bivariate relation through a compact selected source scalar.
/// Every local image candidate is replayed against the authored source/image
/// pair on the requested finite range, so conjugate roots introduced by
/// elimination never become geometry. Selected endpoints keep their policy
/// identity and perform the final exact clipping.
fn selected_fiber_polynomial_relation_parameters(
    source: &BezierAlgebraicSelectedFiberParameter2,
    relation: &BivariatePolynomial,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<BezierAlgebraicSelectedFiberParameter2>>>> {
    policy.strict_predicate_pass(|| {
        source.validate_policy(policy)?;
        let image = match source.retained_polynomial_image_relation(relation, policy)? {
            Classification::Decided(Some(image)) => image,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if let Some(factor) = image.identically_zero_source_factor {
            match source.predicate_sign(&factor, policy)? {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if image.identically_zero_image_relation {
            return Ok(Classification::Decided(None));
        }
        let Some(image_relation) = image.relation else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        let candidates = match selected_fiber_parameters_in_range(
            &image_relation,
            &source.data.authority.data.retained_parameter,
            range,
            policy,
        )? {
            Classification::Decided(Some(candidates)) => candidates,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut retained = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            let sign =
                algebraic_selected_fiber_pair_projected_root(source, &candidate, relation, policy)?;
            match sign {
                Classification::Decided(true) => retained.push(candidate),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(Some(retained)))
    })
}

/// Maps one interior point between exact carriers of the same selected circle
/// by their tangent line. On a circle, a tangent line identifies only the
/// point and its antipode; one published semicircle overlap range contains at
/// most one of those interior points. `range` is expressed in the target's
/// own chart, before any reversal in the overlap correspondence.
fn mapped_circle_tangent_parameter_candidates(
    source_parameter: BezierAlgebraicCuspSemicircleMappedPointParameterRef2<'_>,
    source_tangent: &[Vec<Real>; 2],
    target_tangent: &[Vec<Real>; 2],
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<CurveParameter2>>> {
    let incidence = bivariate_subtract(
        &bivariate_outer_product(&source_tangent[0], &target_tangent[1]),
        &bivariate_outer_product(&source_tangent[1], &target_tangent[0]),
    );
    let candidates = match source_parameter {
        BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Ordinary(parameter) => {
            // Tangent equations can carry radical coefficients from a
            // transported chamfer. Keep the direct resultant first for an
            // algebraic source: eager quotient-ring reduction can expand
            // those coefficients before the small projection is available.
            let projection = selected_fiber_parameters(&incidence, parameter, range, policy)?;
            match projection {
                Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(
                    parameters,
                )) => Classification::Decided(parameters),
                Classification::Decided(
                    BezierAlgebraicFiberProjection2::IdenticallyZero
                    | BezierAlgebraicFiberProjection2::Degenerate,
                ) => Classification::Uncertain(UncertaintyReason::Unsupported),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            }
        }
        BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Selected(parameter) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "mapped-circle-tangent-inverse",
                "selected-fiber-local-image",
            );
            return Ok(
                match selected_fiber_polynomial_relation_parameters(
                    parameter, &incidence, range, policy,
                )? {
                    Classification::Decided(Some(parameters)) => Classification::Decided(
                        parameters
                            .into_iter()
                            .map(CurveParameter2::from_selected_fiber)
                            .collect(),
                    ),
                    Classification::Decided(None) => {
                        Classification::Uncertain(UncertaintyReason::Unsupported)
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                },
            );
        }
    };
    Ok(candidates.map(curve_region_parameters_from_bezier))
}

/// Analytic replay preserves both component kinds in the same parameter space
/// as rational replay, without projecting selected evidence to scalar roots.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum BezierAlgebraicCuspSemicircleParallelIntersections2 {
    Mapped {
        contacts: Vec<BezierAlgebraicCuspSemicircleParallelContact2>,
        overlaps: Vec<BezierAlgebraicCuspSemicircleMappedOverlap2>,
    },
    SelectedFiber {
        contacts: Vec<BezierAlgebraicCuspSemicircleSelectedFiberContact2>,
        overlaps: Vec<BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2>,
    },
    RetainedContacts(Vec<BezierAlgebraicCuspSemicircleRetainedParallelContact2>),
    /// The analytic carrier follows this circle's complete supporting
    /// component on a regular incident parameter cell. No isolated fillet
    /// center exists, and an unbounded overlap chart is intentionally not
    /// materialized merely to report that fact.
    CoincidentCircleComponent,
    DegenerateProjection,
}

/// One-word pair-shared maps from a retained circle-circle contact to both
/// algebraic semicircle parameters.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspSemicirclePairParameterMap2 {
    data: Arc<BezierAlgebraicCuspSemicirclePairParameterMapData2>,
}

#[derive(Debug)]
struct BezierAlgebraicCuspSemicirclePairParameterMapData2 {
    first_semicircle: BezierAlgebraicCuspSemicircle2,
    second_semicircle: BezierAlgebraicCuspSemicircle2,
    system: BezierCirclePairParameterMapSystem2,
    /// Canonical compact field for the untransformed authored pair. Direct
    /// pair points and circles centered at the same contact must share this
    /// allocation so descendant quadratic generators can lift the point as
    /// an ancestor instead of treating an equivalent reconstruction as an
    /// unrelated field.
    recursive_field: OnceLock<BezierRecursiveQuadraticField2>,
    policy: CurveContext,
}

#[derive(Debug)]
enum BezierCirclePairParameterMapSystem2 {
    Represented(BezierRepresentedCirclePairParameterMapSystem2),
    Recursive(BezierRecursiveCirclePairParameterMapSystem2),
}

#[derive(Debug)]
struct BezierRepresentedCirclePairParameterMapSystem2 {
    first_center: [AlgebraicRootRepresentation; 2],
    second_center: [AlgebraicRootRepresentation; 2],
    contacts: Vec<BezierRepresentedCirclePairContactData2>,
}

#[derive(Debug)]
struct BezierRepresentedCirclePairContactData2 {
    branch: i8,
    point: [AlgebraicRootRepresentation; 2],
    first_parameter: BezierRepresentedCircleContactParameterData2,
    second_parameter: BezierRepresentedCircleContactParameterData2,
    tangent_cross: AlgebraicRootRepresentation,
    tangent_dot: AlgebraicRootRepresentation,
    /// Stable recursive contact and both participating support centers.
    /// Descendants must reuse this allocation so either side of the pair
    /// enters the identical coefficient tower.
    recursive_contact_frame: OnceLock<BezierRecursiveQuadraticPairContactFrame2>,
}

#[derive(Debug)]
struct BezierRecursiveCirclePairParameterMapSystem2 {
    contacts: Vec<BezierRecursiveCirclePairContactData2>,
}

#[derive(Debug)]
struct BezierRecursiveCirclePairContactData2 {
    branch: i8,
    frame: BezierRecursiveQuadraticPairContactFrame2,
    angular: [BezierRecursiveCirclePairAngularData2; 2],
    tangent_cross: BezierRecursiveQuadraticValue2,
    tangent_dot: BezierRecursiveQuadraticValue2,
    /// Independent sign certificate for the zero-discriminant branch.  Its
    /// tangent cross is identically zero, while the two nonzero radial line
    /// factors and positive center-distance square decide the dot sign.
    tangent_dot_sign: Option<RealSign>,
}

#[derive(Debug)]
struct BezierRecursiveCirclePairAngularData2 {
    diameter: BezierRecursiveQuadraticValue2,
    radius_squared_denominator: BezierRecursiveQuadraticValue2,
}

struct BezierRecursiveCirclePairContactSide2 {
    location: BezierAlgebraicCuspSemicircleContactLocation2,
    angular: BezierRecursiveCirclePairAngularData2,
}

#[derive(Clone, Debug)]
enum BezierRepresentedCircleContactParameterData2 {
    Materialized(BezierParameter2),
    ExactContactRadial([Real; 2]),
    Retained {
        parameter: BezierAlgebraicCuspSemicircleParameter2,
        unit_complement: bool,
    },
    AuthoredPairAngular {
        map: BezierAlgebraicCuspSemicirclePairParameterMap2,
        contact: BezierAlgebraicCuspSemicirclePairContact2,
        dot_scale: Real,
        oriented_cross_scale: Real,
        radius_squared: Real,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierAlgebraicCuspSemicirclePairContact2 {
    /// `-1` and `+1` select the two transverse circle intersections; zero is
    /// the unique tangent branch.
    branch: i8,
    pub(crate) first_location: BezierAlgebraicCuspSemicircleContactLocation2,
    pub(crate) second_location: BezierAlgebraicCuspSemicircleContactLocation2,
    pub(crate) tangent_cross_sign: RealSign,
}

/// Reduced exact line-circle system for one certified axis-aligned retained
/// chord.  The first parameter is the selected circle-center field and the
/// second is one selected point on the chord's stable supporting line.
#[derive(Debug)]
struct BezierAlgebraicCuspSemicircleChordSystem2 {
    incidence: BivariatePolynomial,
    discriminant: BivariatePolynomial,
    selected_half_plane: BezierAlgebraicCuspTwoTermExpression2,
    diameter_side: BezierAlgebraicCuspTwoTermExpression2,
    radius_squared_denominator: BivariatePolynomial,
    common_denominator: BivariatePolynomial,
    center_x: BivariatePolynomial,
    center_y: BivariatePolynomial,
    point_x: BezierAlgebraicCuspTwoTermExpression2,
    point_y: BezierAlgebraicCuspTwoTermExpression2,
    point_minus_support_axis: BezierAlgebraicCuspTwoTermExpression2,
    cusp_parameter: BezierParameter2,
    support_parameter: BezierParameter2,
    direction: BezierAlgebraicChordAxisDirection2,
}

struct BezierAlgebraicCuspNormalizedCircleFrame2 {
    center_x: Vec<Real>,
    center_y: Vec<Real>,
    normal_x: Vec<Real>,
    normal_y: Vec<Real>,
    denominator: Vec<Real>,
    cusp_parameter: BezierParameter2,
}

struct BezierAlgebraicCuspPositivePointField2 {
    x: Vec<Real>,
    y: Vec<Real>,
    denominator: Vec<Real>,
    parameter: BezierParameter2,
}

/// Pair-shared exact map from a selected circle/axis-chord contact to the
/// semicircle parameter and correlated affine point.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspSemicircleChordParameterMap2 {
    data: Arc<BezierAlgebraicCuspSemicircleChordParameterMapData2>,
}

#[derive(Debug)]
struct BezierAlgebraicCuspSemicircleChordParameterMapData2 {
    semicircle: BezierAlgebraicCuspSemicircle2,
    chord: BezierAlgebraicChord2,
    system: BezierAlgebraicCuspSemicircleChordParameterMapSystem2,
    finite_chord_domain: bool,
    policy: CurveContext,
    /// Shared coefficient authority for historical compact maps imported
    /// into the recursive quadratic tower. Retaining the field—not merely
    /// reconstructed coordinates—preserves each positive radical's identity
    /// when several topology paths consume the same contact.
    recursive_import_field: OnceLock<BezierRecursiveQuadraticField2>,
}

#[derive(Debug)]
enum BezierAlgebraicCuspSemicircleChordParameterMapSystem2 {
    Axis(BezierAlgebraicCuspSemicircleAxisChordParameterMapSystem2),
    Oblique(BezierAlgebraicCuspSemicircleObliqueChordParameterMapSystem2),
    RepresentedOblique(BezierRepresentedCircleChordParameterMapSystem2),
    RetainedOffset(BezierAlgebraicCuspSemicircleRetainedOffsetChordParameterMapSystem2),
    RecursiveQuadraticLine(BezierRecursiveQuadraticLineParameterMapSystem2),
    SelectedRadial(BezierSelectedRadialCircleChordParameterMapSystem2),
    ChordNormalProjective(BezierChordNormalDenseChordParameterMapSystem2),
}

/// Rank-independent exact authority for a selected circle against an affine
/// chord whose endpoints are themselves retained algebraic constructions.
///
/// The ordinary axis, two-field oblique, and procedural-offset systems remain
/// the hot representations.  This cold complete path materializes each
/// already-selected endpoint coordinate under STRICT, performs the analytic
/// line/circle solve in their common dense tensor, and retains only the two
/// final contact coordinates and scalar predicates.  APPROXIMATE_512 may
/// terminate a later comparison, but never chooses a square-root sheet here.
#[derive(Debug)]
struct BezierRepresentedCircleChordParameterMapSystem2 {
    center: [AlgebraicRootRepresentation; 2],
    contacts: Vec<BezierRepresentedCircleChordContactData2>,
}

#[derive(Debug)]
struct BezierRepresentedCircleChordContactData2 {
    branch: i8,
    point: [AlgebraicRootRepresentation; 2],
    cusp_parameter: BezierRepresentedCircleChordAngularParameter2,
    chord_parameter: AlgebraicRootRepresentation,
    tangent_cross: AlgebraicRootRepresentation,
    tangent_dot: AlgebraicRootRepresentation,
}

/// Exact angular parameter retained by the rank-independent represented
/// circle/chord map.  The ordinary cold path still publishes a selected
/// algebraic scalar.  A chord-normal frame instead keeps the two correlated
/// normalization/contact radicals in the shared recursive field and exposes
/// only projective comparisons, avoiding a degree-multiplied global norm.
#[derive(Debug)]
enum BezierRepresentedCircleChordAngularParameter2 {
    Materialized(BezierParameter2),
    Recursive(BezierRecursiveQuadraticProjectiveScalar2),
}

/// Shared exact authority for scalar roots retained directly in one selected
/// algebraic fiber `F(alpha, u) = 0`.
///
/// This is the canonical representation for high-degree local curve
/// parameters.  It keeps the already selected root `alpha` and reduced
/// incidence once, while each scalar stores only its isolating interval and a
/// pointer to this authority.  It deliberately does not construct the global
/// norm of `u`.
#[derive(Clone, Debug)]
struct BezierAlgebraicSelectedFiberAuthority2 {
    data: Arc<BezierAlgebraicSelectedFiberAuthorityData2>,
}

#[derive(Debug)]
struct BezierAlgebraicSelectedFiberAuthorityData2 {
    incidence: BivariatePolynomial,
    retained_parameter: BezierAlgebraicParameter2,
    policy: CurveContext,
    retained_refinement_64: OnceLock<Arc<BezierParameter2>>,
    incidence_has_parameter_diagonal: OnceLock<bool>,
    root_refiner: Mutex<Option<Box<hypersolve::AlgebraicFiberRootRefiner>>>,
}

impl PartialEq for BezierAlgebraicSelectedFiberAuthorityData2 {
    fn eq(&self, other: &Self) -> bool {
        self.incidence == other.incidence
            && self.retained_parameter == other.retained_parameter
            && self.policy == other.policy
    }
}

impl PartialEq for BezierAlgebraicSelectedFiberAuthority2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data) || self.data == other.data
    }
}

/// One exact scalar root in a selected algebraic fiber.
///
/// Clones are one word and share all polynomial evidence.  The interval is a
/// certified singleton in `Q(alpha)`, not an approximate coordinate.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicSelectedFiberParameter2 {
    data: Arc<BezierAlgebraicSelectedFiberParameterData2>,
}

/// Exact local image of one selected-fiber scalar.
///
/// `relation` is the residual image after source components on which the
/// authored image equation vanishes identically have been saturated. The
/// removed factor remains explicit so the selected source root, rather than
/// an unrelated conjugate, decides whether the image is positive-dimensional.
#[derive(Debug)]
struct BezierSelectedPolynomialImage2 {
    relation: Option<BivariatePolynomial>,
    global_schedule: Option<Vec<Real>>,
    identically_zero_source_factor: Option<BivariatePolynomial>,
    identically_zero_image_relation: bool,
}

#[derive(Debug, Default)]
struct BezierSelectedFiberRepresentations2 {
    bezier: OnceLock<BezierParameter2>,
    projective: OnceLock<BezierRecursiveProjectiveParameter2>,
}

#[derive(Debug)]
struct BezierAlgebraicSelectedFiberParameterData2 {
    authority: BezierAlgebraicSelectedFiberAuthority2,
    root: IsolatedRootInterval,
    representations: Arc<BezierSelectedFiberRepresentations2>,
}

impl PartialEq for BezierAlgebraicSelectedFiberParameterData2 {
    fn eq(&self, other: &Self) -> bool {
        self.authority == other.authority && self.root == other.root
    }
}

impl PartialEq for BezierAlgebraicSelectedFiberParameter2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data) || self.data == other.data
    }
}

#[cfg(test)]
pub(crate) fn exact_selected_fiber_parameter_for_test(
    retained_parameter: BezierAlgebraicParameter2,
    value: Real,
    policy: &CurveContext,
) -> BezierAlgebraicSelectedFiberParameter2 {
    BezierAlgebraicSelectedFiberAuthority2::exact_parameter(retained_parameter, value, policy)
}

#[cfg(test)]
pub(crate) fn degree_nine_selected_fiber_parameter_for_test(
    retained_constant: Real,
    fiber_scale: i32,
    policy: &CurveContext,
) -> BezierAlgebraicSelectedFiberParameter2 {
    let mut retained_coefficients = vec![Real::zero(); 10];
    retained_coefficients[0] = -retained_constant;
    retained_coefficients[9] = Real::one();
    let Classification::Decided(retained_polynomial) =
        BezierParameterPolynomial::try_new_power_basis(retained_coefficients, policy).unwrap()
    else {
        panic!("the degree-nine retained polynomial must construct");
    };
    let Classification::Decided(retained_interval) =
        BezierParameterInterval::try_new(Real::zero(), Real::one(), policy).unwrap()
    else {
        panic!("the degree-nine retained interval must construct");
    };
    let Classification::Decided(retained) =
        BezierAlgebraicParameter2::try_isolate(retained_polynomial, retained_interval, policy)
            .unwrap()
    else {
        panic!("the degree-nine retained root must isolate");
    };

    // `fiber_scale * u^15 - alpha = 0` makes the global projection
    // degree 9*15=135, beyond the bounded degree-128 resultant and the
    // degree-eight quotient-ring fallback. The local selected fiber is
    // nevertheless a single exact positive root.
    let mut fiber = vec![Real::zero(); 16];
    fiber[15] = Real::from(fiber_scale);
    let incidence = BivariatePolynomial::new(vec![fiber, vec![Real::from(-1_i8)]]);
    let report = isolate_bivariate_fiber_roots_at_algebraic_parameter(
        &incidence,
        CurveResultantParameter::First,
        &parameter_representation(&retained, policy),
        &Real::zero(),
        &Real::one(),
        AlgebraicFiberRootIsolationConfig {
            max_subdivision_depth: 512,
            refinement_steps: 8,
        },
        hypersolve::PredicatePolicy::STRICT,
    );
    assert_eq!(report.status, AlgebraicFiberRootIsolationStatus::Isolated);
    let [root] = report.intervals.as_slice() else {
        panic!("the odd local fiber must have one unit-interval root");
    };
    BezierAlgebraicSelectedFiberAuthority2::new(incidence, retained, policy).parameter(root.clone())
}

#[cfg(test)]
pub(crate) fn high_degree_quadratic_selected_fiber_parameter_for_test(
    retained_constant: Real,
    policy: &CurveContext,
) -> BezierAlgebraicSelectedFiberParameter2 {
    let mut coefficients = vec![Real::zero(); 66];
    coefficients[0] = -retained_constant;
    coefficients[65] = Real::one();
    let Classification::Decided(polynomial) =
        BezierParameterPolynomial::try_new_power_basis(coefficients, policy).unwrap()
    else {
        panic!("the degree-65 retained carrier must construct")
    };
    let Classification::Decided(interval) =
        BezierParameterInterval::try_new(Real::zero(), Real::one(), policy).unwrap()
    else {
        panic!("the degree-65 retained interval must construct")
    };
    let Classification::Decided(retained) =
        BezierAlgebraicParameter2::try_isolate(polynomial, interval, policy).unwrap()
    else {
        panic!("the degree-65 retained root must isolate")
    };
    BezierAlgebraicSelectedFiberAuthority2::new(
        BivariatePolynomial::new(vec![
            vec![Real::zero(), Real::zero(), Real::from(4_i8)],
            vec![Real::from(-1_i8)],
        ]),
        retained.clone(),
        policy,
    )
    .parameter(IsolatedRootInterval {
        lower: Real::zero(),
        upper: (Real::one() / Real::from(2_i8)).unwrap(),
        exact_root: None,
        distinct_root_count: 1,
    })
}

#[derive(Debug)]
struct BezierAlgebraicCuspSemicircleAxisChordParameterMapSystem2 {
    incidence: BivariatePolynomial,
    discriminant: BivariatePolynomial,
    diameter_side: BezierAlgebraicCuspTwoTermExpression2,
    radius_squared_denominator: BivariatePolynomial,
    common_denominator: BivariatePolynomial,
    center_x: BivariatePolynomial,
    center_y: BivariatePolynomial,
    point_x: BezierAlgebraicCuspTwoTermExpression2,
    point_y: BezierAlgebraicCuspTwoTermExpression2,
    cusp_parameter: BezierParameter2,
    support_parameter: BezierParameter2,
    direction: BezierAlgebraicChordAxisDirection2,
}

#[derive(Clone, Debug)]
struct BezierAlgebraicCuspTrivariateSquareRootExpression2 {
    rational: TrivariatePolynomial,
    radical: TrivariatePolynomial,
}

/// Exact expression in the retained circle-pair radical and one analytic
/// parallel's positive source-speed radical.
///
/// With `k=sqrt(pair_discriminant)` on the authored pair branch and
/// `s=sqrt(candidate_speed_squared) > 0`, the represented value is
/// `product*k*s + pair*k + candidate*s + rational`.
#[derive(Clone, Debug)]
struct BezierAlgebraicCuspTrivariateTwoSquareRootExpression2 {
    product: TrivariatePolynomial,
    pair: TrivariatePolynomial,
    candidate: TrivariatePolynomial,
    rational: TrivariatePolynomial,
}

/// Target-independent pair-field geometry of a selected-radial circle.
///
/// The first two axes are the two authored source-circle roots.  Center and
/// parameter-zero radial numerators share `denominator`; their radical terms
/// use the retained `branch*sqrt(discriminant)` sheet.  Keeping this compact
/// frame authoritative avoids rebuilding the circle-pair construction for
/// rational, analytic-parallel, and later chord targets.
#[derive(Debug)]
struct BezierSelectedRadialCircleFrameSystem2 {
    pair_map: BezierAlgebraicCuspSemicirclePairParameterMap2,
    /// Similarity-free systems may reuse the pair map's canonical recursive
    /// base. A transformed frame has different coefficient polynomials and
    /// therefore must retain a distinct base allocation.
    canonical_pair_field: bool,
    branch: i8,
    discriminant: TrivariatePolynomial,
    denominator: TrivariatePolynomial,
    center_x: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    center_y: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    radial_x: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    radial_y: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    normal_denominator: Real,
}

/// Generic exact frame for a recursively authored selected-radial circle.
/// `center` and `unit_radial` are independently isolated algebraic coordinates
/// of the authored center and normalized parameter-zero direction. Keeping the
/// exact signed radius separate makes reversal and concentric scaling scalar
/// operations instead of new high-degree coordinate eliminants. Together with
/// the retained turn on the owning semicircle they define every circle
/// predicate without a fixed source-field rank.
#[derive(Clone, Debug, PartialEq)]
struct BezierRepresentedSelectedRadialCircleFrame2 {
    center: [AlgebraicRootRepresentation; 2],
    unit_radial: [AlgebraicRootRepresentation; 2],
    signed_radius: Real,
}

struct BezierRepresentedCommonSourceCenter2 {
    sources: Vec<AlgebraicRootRepresentation>,
    transforms: [[Real; 6]; 2],
}

struct BezierRepresentedAuthoredCenterRelation2 {
    distance_squared: Real,
    pair_map: BezierAlgebraicCuspSemicirclePairParameterMap2,
    pair_contact: BezierAlgebraicCuspSemicirclePairContact2,
    candidate_first: bool,
    radial_support_first: bool,
    center_support_first: bool,
    normal_denominator: Real,
    source_similarity: Option<Similarity2>,
    direct_parameter_evidence: bool,
}

struct BezierRepresentedCircleContactParameterEvidence2 {
    location: BezierAlgebraicCuspSemicircleContactLocation2,
    parameter: BezierRepresentedCircleContactParameterData2,
}

enum BezierRepresentedAuthoredTangent2 {
    Contact {
        parameters: [BezierRepresentedCircleContactParameterEvidence2; 2],
        point: Option<CurvePoint2>,
    },
    OutsideRetainedHalf,
}

struct BezierSelectedRadialCircleFrameSource2<'a> {
    frame: &'a BezierSelectedRadialFrameData2,
    pair_map: &'a BezierAlgebraicCuspSemicirclePairParameterMap2,
    pair_contact: &'a BezierAlgebraicCuspSemicirclePairContact2,
    support_first: bool,
    similarity: Option<Similarity2>,
}

/// Three-axis exact incidence for a rational curve against a circle whose
/// center and start radial are one retained contact of two selected circles.
/// The first two axes are the original circle-center roots and the third is
/// the rational curve parameter.  `incidence_projection` is squared only for
/// candidate enumeration; every accepted root is replayed against the signed
/// one-radical `incidence` expression.
#[derive(Debug)]
struct BezierSelectedRadialCircleRationalSystem2 {
    pair_map: BezierAlgebraicCuspSemicirclePairParameterMap2,
    branch: i8,
    discriminant: TrivariatePolynomial,
    incidence: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    incidence_projection: TrivariatePolynomial,
    selected_half_plane: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    diameter: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    radius_squared_denominator: TrivariatePolynomial,
    tangent_cross: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    /// Signed angular velocity `cross(Q-C,Q')`. Its exact zeros partition a
    /// coincident rational-circle component into monotone parameter cells.
    angular_tangent: BezierAlgebraicCuspTrivariateSquareRootExpression2,
}

/// Minimal-degree direct pair-radial backend for the authoritative recursive
/// circle/analytic-parallel kernel. It enumerates and replays the common
/// two-root case without independently materializing frame coordinates; the
/// owning recursive system handles components and every higher carrier.
#[derive(Debug)]
struct BezierDirectPairRadialParallelFastPath2 {
    pair_map: BezierAlgebraicCuspSemicirclePairParameterMap2,
    branch: i8,
    pair_discriminant: TrivariatePolynomial,
    candidate_speed_squared: TrivariatePolynomial,
    incidence: BezierAlgebraicCuspTrivariateTwoSquareRootExpression2,
    incidence_candidate_norm: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    incidence_projection: TrivariatePolynomial,
    selected_half_plane: BezierAlgebraicCuspTrivariateTwoSquareRootExpression2,
    diameter: BezierAlgebraicCuspTrivariateTwoSquareRootExpression2,
    radius_squared_denominator: BezierAlgebraicCuspTrivariateTwoSquareRootExpression2,
    tangent_cross_source: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    tangent_dot_source: BezierAlgebraicCuspTrivariateTwoSquareRootExpression2,
    target_weight: Vec<Real>,
    target_speed_squared: Vec<Real>,
}

/// Rank-independent exact incidence of a represented selected-circle frame
/// with one rational Bezier.  Only the center coordinates participate in
/// candidate projection; the unit radial is retained in `frame` and enters
/// angular predicates after an incidence root has survived exact replay.
#[derive(Debug)]
struct BezierRepresentedCircleRationalSystem2 {
    frame: BezierRepresentedSelectedRadialCircleFrame2,
    sources: Vec<AlgebraicRootRepresentation>,
    incidence: DenseTensorPolynomial,
    tangent_cross: DenseTensorPolynomial,
}

/// Cold angular partition for a rational curve already proved to lie on a
/// rank-independent represented circle. The discrete intersection fast path
/// deliberately omits the unit-radial axes; they are joined only after its
/// incidence has proved identically zero.
#[derive(Debug)]
struct BezierRepresentedCircleRationalComponentSystem2 {
    sources: Vec<AlgebraicRootRepresentation>,
    selected_half_plane: DenseTensorPolynomial,
    diameter: DenseTensorPolynomial,
    radius_squared_denominator: DenseTensorPolynomial,
    angular_tangent: DenseTensorPolynomial,
    quadratic_selected_parameters: Option<Vec<BezierParameter2>>,
}

/// One exact finite interior inverse chart for a retained rational quadratic,
/// together with the represented selected-circle frame needed to evaluate its
/// two antipodal boundary points.
#[derive(Debug)]
struct BezierRepresentedQuadraticConicInverse2 {
    center: [AlgebraicRootRepresentation; 2],
    unit_radial: [AlgebraicRootRepresentation; 2],
    signed_radius: Real,
    numerator: [Real; 3],
    denominator: [Real; 3],
}

impl BezierRepresentedCircleRationalComponentSystem2 {
    fn parameters(
        &self,
        polynomial: &DenseTensorPolynomial,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        let projection = match selected_dense_last_axis_parameters(
            polynomial,
            &self.sources,
            SelectedThirdAxisDomain2::Finite(range),
            policy,
        )? {
            Classification::Decided(projection) => projection,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let BezierAlgebraicFiberProjection2::Parameters(candidates) = projection else {
            return Ok(Classification::Decided(projection));
        };

        // Dense elimination projects every algebraic conjugate of the frame
        // sources. Only candidates that replay to zero on the authored tuple
        // are component boundaries; admitting the other conjugate roots
        // would subdivide a regular overlap into fictitious monotone cells.
        let mut retained = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            match projected_selected_dense_candidate_box_incidence(
                polynomial,
                &self.sources,
                &candidate,
                64,
                64,
            ) {
                Some(BezierDenseCandidateBoxIncidence2::Root(_)) => {
                    retained.push(candidate);
                    continue;
                }
                Some(BezierDenseCandidateBoxIncidence2::Disjoint(_)) => continue,
                None => {}
            }
            match self.sign(polynomial, &candidate, policy)? {
                Classification::Decided(RealSign::Zero) => retained.push(candidate),
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(
            BezierAlgebraicFiberProjection2::Parameters(retained),
        ))
    }

    fn sign(
        &self,
        polynomial: &DenseTensorPolynomial,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let mut selected = Vec::with_capacity(self.sources.len() + 1);
        selected.extend(self.sources.iter().cloned());
        selected.push(match parameter {
            BezierParameter2::Exact(parameter) => {
                AlgebraicRootRepresentation::from_exact_value(parameter)
            }
            BezierParameter2::Algebraic(parameter) => parameter_representation(parameter, policy),
        });
        dense_polynomial_tuple_sign(polynomial, &selected, policy)
    }

    /// A finite rational quadratic with one common weight sign traces one
    /// regular circle arc shorter than a semicircle. Its selected-half
    /// predicate therefore has no interior root when the endpoint signs
    /// agree and exactly one when they differ. The latter is retained through
    /// a linear projective conic inverse, avoiding a degree-multiplied global
    /// projection and any approximate construction decision.
    fn quadratic_selected_parameters(
        &self,
        inverse: &BezierRepresentedQuadraticConicInverse2,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierParameter2>>> {
        let (lower, upper) = range
            .scalar_endpoints()
            .ok_or(CurveError::InvalidCurveParameter)?;
        let zero = BezierParameter2::Exact(lower.clone());
        let one = BezierParameter2::Exact(upper.clone());
        let lower_sign = match self.sign(&self.selected_half_plane, &zero, policy)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let upper_sign = match self.sign(&self.selected_half_plane, &one, policy)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if lower_sign == RealSign::Zero && upper_sign == RealSign::Zero {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        if lower_sign == RealSign::Zero {
            return Ok(Classification::Decided(vec![zero]));
        }
        if upper_sign == RealSign::Zero {
            return Ok(Classification::Decided(vec![one]));
        }
        if lower_sign == upper_sign {
            return Ok(Classification::Decided(Vec::new()));
        }

        inverse
            .interior_parameter(policy)
            .map(|parameter| parameter.map(|parameter| vec![parameter]))
    }
}

/// Retains the interior projective inverse chart for a rational quadratic.
/// In the homogeneous control frame `H0,H1,H2`, a conic point's dual
/// coordinates are proportional to `((1-t)^2, 2t(1-t), t^2)`, giving
/// `t=2c2/(c1+2c2)`. Its denominator is nonzero for every interior `t`;
/// target endpoints are handled before this construction is needed.
fn represented_quadratic_conic_inverse(
    frame: &BezierRepresentedSelectedRadialCircleFrame2,
    controls: &[[Real; 3]; 3],
) -> BezierRepresentedQuadraticConicInverse2 {
    let cross = |first: &[Real; 3], second: &[Real; 3]| {
        [
            &first[1] * &second[2] - &first[2] * &second[1],
            &first[2] * &second[0] - &first[0] * &second[2],
            &first[0] * &second[1] - &first[1] * &second[0],
        ]
    };
    let dual = [
        cross(&controls[1], &controls[2]),
        cross(&controls[2], &controls[0]),
        cross(&controls[0], &controls[1]),
    ];
    let two = Real::from(2_i8);
    BezierRepresentedQuadraticConicInverse2 {
        center: frame.center.clone(),
        unit_radial: frame.unit_radial.clone(),
        signed_radius: frame.signed_radius.clone(),
        numerator: std::array::from_fn(|axis| &two * &dual[2][axis]),
        denominator: std::array::from_fn(|axis| &dual[1][axis] + &two * &dual[2][axis]),
    }
}

/// Constructs `(a*x+b*y+c)/(d*x+e*y+f)` directly in the compositum of two
/// selected algebraic roots. Eliminating the affine relation before
/// interpolation keeps the final degree at `deg(x)*deg(y)`; independently
/// materializing numerator and denominator would square that degree and lose
/// their shared-source correlation.
fn represented_biaffine_ratio(
    x: &AlgebraicRootRepresentation,
    y: &AlgebraicRootRepresentation,
    numerator: &[Real; 3],
    denominator: &[Real; 3],
) -> Classification<AlgebraicRootRepresentation> {
    if !x.is_valid() || !y.is_valid() {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    }
    let x_degree = x.polynomial_coefficients.len().saturating_sub(1);
    let y_degree = y.polynomial_coefficients.len().saturating_sub(1);
    if x_degree == 0 || y_degree == 0 {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    }

    // For output `u`, the inverse relation is
    //
    //   (u*d-a)x + (u*e-b)y + (u*f-c) = 0.
    //
    // Eliminating x analytically gives
    // `A(u)^m P(-(B(u)y+C(u))/A(u))`; its resultant with Q(y)
    // is the exact output eliminant, including degree-drop specializations.
    let a = BivariatePolynomial::new(vec![vec![-numerator[0].clone(), denominator[0].clone()]]);
    let negative_tail = BivariatePolynomial::new(vec![
        vec![numerator[2].clone(), -denominator[2].clone()],
        vec![numerator[1].clone(), -denominator[1].clone()],
    ]);
    let powers = |base: &BivariatePolynomial| {
        let mut values = Vec::with_capacity(x_degree + 1);
        values.push(BivariatePolynomial::new(vec![vec![Real::one()]]));
        for exponent in 1..=x_degree {
            values.push(bivariate_multiply(&values[exponent - 1], base));
        }
        values
    };
    let negative_tail_powers = powers(&negative_tail);
    let a_powers = powers(&a);
    let mut eliminated_x = BivariatePolynomial::new(vec![vec![Real::zero()]]);
    for (x_power, coefficient) in x.polynomial_coefficients.iter().enumerate() {
        let term = bivariate_multiply(
            &negative_tail_powers[x_power],
            &a_powers[x_degree - x_power],
        );
        eliminated_x = bivariate_add(&eliminated_x, &bivariate_scale(term, coefficient));
    }
    let y_constraint = BivariatePolynomial::new(
        y.polynomial_coefficients
            .iter()
            .cloned()
            .map(|coefficient| vec![coefficient])
            .collect(),
    );
    let report = resultant_bivariate_polynomial_system_complete(
        &eliminated_x,
        &y_constraint,
        CurveResultantParameter::Second,
        CurveIntersectionResultantConfig {
            min_precision: hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION,
            max_resultant_degree: MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        },
    );
    if report.status != CurveIntersectionResultantStatus::Constructed {
        return Classification::Uncertain(match report.status {
            CurveIntersectionResultantStatus::UndecidedCoefficient => UncertaintyReason::Predicate,
            _ => UncertaintyReason::Unsupported,
        });
    }
    let Some(coefficients) = hypersolve::square_free_part(
        report.resultant_coefficients,
        hypersolve::PredicatePolicy::STRICT,
    ) else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };

    let affine_interval = |first: &AlgebraicRootRepresentation,
                           second: &AlgebraicRootRepresentation,
                           coefficients: &[Real; 3]| {
        let mut value = RealInterval {
            lower: coefficients[2].clone(),
            upper: coefficients[2].clone(),
        };
        for (source, scale) in [(first, &coefficients[0]), (second, &coefficients[1])] {
            let source = RealInterval {
                lower: source.interval.lower.clone(),
                upper: source.interval.upper.clone(),
            };
            let scale = RealInterval {
                lower: scale.clone(),
                upper: scale.clone(),
            };
            value = value.add(&source.multiply(&scale)?);
        }
        Some(value)
    };
    let mut last_reason = UncertaintyReason::Predicate;
    for refinement_steps in [0_usize, 4, 8, 16, 32, 64, 128, 256, 512] {
        let x = refined_represented_root(x, refinement_steps);
        let y = refined_represented_root(y, refinement_steps);
        let (Some(numerator), Some(denominator)) = (
            affine_interval(&x, &y, numerator),
            affine_interval(&x, &y, denominator),
        ) else {
            continue;
        };
        let Some(interval) = numerator.divide(&denominator) else {
            continue;
        };
        match Classification::from(represented_univariate_coordinate(
            &coefficients,
            &interval.lower,
            &interval.upper,
            &x,
        )) {
            Classification::Decided(parameter) => return Classification::Decided(parameter),
            Classification::Uncertain(reason) => last_reason = reason,
        }
    }
    Classification::Uncertain(last_reason)
}

impl BezierRepresentedQuadraticConicInverse2 {
    /// Recovers the unique selected-diameter crossing on a common-sign
    /// rational quadratic. Each intermediate construction combines at most
    /// two already isolated algebraic values, avoiding one high-rank tensor
    /// eliminant while retaining exact singleton-root evidence throughout.
    fn interior_parameter(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierParameter2>> {
        let strict = policy.strict_counterpart();
        let mut last_reason = UncertaintyReason::Predicate;
        for endpoint_sign in [Real::one(), Real::from(-1_i8)] {
            let radial_scale = &self.signed_radius * &endpoint_sign;
            let coordinate = |axis: usize| {
                Classification::from(represented_affine_coordinate(
                    &[
                        (&self.center[axis], &Real::one()),
                        (&self.unit_radial[axis], &radial_scale),
                    ],
                    &Real::zero(),
                ))
            };
            let (x, y) = match (coordinate(0), coordinate(1)) {
                (Classification::Decided(x), Classification::Decided(y)) => (x, y),
                (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                    last_reason = reason;
                    continue;
                }
            };
            let parameter =
                match represented_biaffine_ratio(&x, &y, &self.numerator, &self.denominator) {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        continue;
                    }
                };
            let parameter = match represented_strict_interior_bezier_parameter(&parameter)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    last_reason = reason;
                    continue;
                }
            };
            let zero = BezierParameter2::Exact(Real::zero());
            let one = BezierParameter2::Exact(Real::one());
            let after_zero = parameter.cmp_by_refinement(&zero, &strict)?;
            let before_one = parameter.cmp_by_refinement(&one, &strict)?;
            match (after_zero, before_one) {
                (
                    Classification::Decided(std::cmp::Ordering::Greater),
                    Classification::Decided(std::cmp::Ordering::Less),
                ) => return Ok(Classification::Decided(parameter)),
                (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                    last_reason = reason
                }
                _ => {}
            }
        }
        Ok(Classification::Uncertain(last_reason))
    }
}

/// One exact value `R + A*sqrt(S)` over a rank-independent represented
/// selected-circle frame and one analytic-parallel parameter. `S` is the
/// target source-speed square and its positive square root is procedural:
/// candidate isolation may square this expression, but every admitted root is
/// replayed on the authored positive sheet.
#[derive(Clone, Debug)]
struct BezierRepresentedCircleParallelExpression2 {
    rational: DenseTensorPolynomial,
    radical: DenseTensorPolynomial,
}

/// Rank-independent selected-circle/analytic-parallel authority.
///
/// The represented center and start unit radial occupy only the algebraic
/// axes they actually require; the last tensor axis is the target Bezier
/// parameter. Multiplication by the positive factor `W^2*sqrt(S)` removes all
/// target divisions without changing a predicate sign. This leaves exactly
/// one procedural target-speed radical, even when the circle center and chord
/// normal came from unrelated endpoint fields.
#[derive(Debug)]
struct BezierRepresentedCircleParallelSystem2 {
    sources: Vec<AlgebraicRootRepresentation>,
    projection: DenseTensorPolynomial,
    circle: BezierRepresentedCircleParallelExpression2,
    selected_half_plane: BezierRepresentedCircleParallelExpression2,
    diameter: BezierRepresentedCircleParallelExpression2,
    radius_squared_denominator: BezierRepresentedCircleParallelExpression2,
    tangent_cross_source: DenseTensorPolynomial,
    tangent_dot_source: BezierRepresentedCircleParallelExpression2,
    speed_squared: DenseTensorPolynomial,
    weight: DenseTensorPolynomial,
}

/// Shared represented-center construction for analytic circle incidence.
///
/// Candidate enumeration depends only on the center. Full represented-circle
/// topology appends the selected unit radial to the same affine tensor basis,
/// while recursive frames may use the smaller center-only projection merely
/// as a root schedule and replay every candidate in their retained tower.
struct BezierRepresentedCenterParallelSystem2 {
    sources: Vec<AlgebraicRootRepresentation>,
    coordinates: Vec<DenseTensorPolynomial>,
    projection: DenseTensorPolynomial,
    circle: BezierRepresentedCircleParallelExpression2,
    radial_x: DenseTensorPolynomial,
    radial_y: DenseTensorPolynomial,
    normal_x: DenseTensorPolynomial,
    normal_y: DenseTensorPolynomial,
    tangent_x: DenseTensorPolynomial,
    tangent_y: DenseTensorPolynomial,
    speed_squared: DenseTensorPolynomial,
    weight: DenseTensorPolynomial,
    weight_squared: DenseTensorPolynomial,
}

/// One expression A(t)+B(t)*sqrt(S(t)) over a retained quadratic field.
/// Related expressions share S; enumeration and replay share the exact norm
/// A²-B²S only when requested. Every admitted parameter still replays the
/// authored positive root, independently of that squared equation.
#[derive(Clone, Debug)]
struct BezierRecursiveQuadraticParallelExpression2 {
    rational: Vec<BezierRecursiveQuadraticValue2>,
    radical: Vec<BezierRecursiveQuadraticValue2>,
    /// The radicand belongs to the expression. Related predicates share it,
    /// so a caller cannot replay the expression with another speed sheet.
    speed_squared: Arc<[BezierRecursiveQuadraticValue2]>,
    /// Successful exact arithmetic only; no policy-dependent sign is cached.
    squared_magnitude: Arc<OnceLock<Vec<BezierRecursiveQuadraticValue2>>>,
}

/// Arbitrary-depth retained circle/analytic-parallel authority.
///
/// The center and parameter-zero radial retain their shared recursive field.
/// Global enumeration eliminates only the final target parameter. Replay can
/// keep local polynomial roots in that field; native candidates embed as a
/// base axis with a positive speed root. Both routes preserve the authored
/// speed sheet without independently materializing Cartesian coordinates.
#[derive(Debug)]
struct BezierRecursiveCircleTargetSystem2 {
    field: BezierRecursiveQuadraticField2,
    base: Arc<BezierRecursiveQuadraticBaseFieldData2>,
    direct_pair_fast_path: Option<Arc<BezierDirectPairRadialParallelFastPath2>>,
    /// Rational-curve incidence is the zero-distance specialization with an
    /// exact unit procedural speed. It neither adjoins nor divides by the
    /// target derivative norm, preserving singular rational contacts.
    unit_target_speed: bool,
    projection: Option<DenseTensorPolynomial>,
    incidence_univariate: OnceLock<BezierSelectedDenseLastAxisUnivariate2>,
    represented_center_schedule: OnceLock<BezierRepresentedCenterParallelSchedule2>,
    circle: BezierRecursiveQuadraticParallelExpression2,
    selected_half_plane: BezierRecursiveQuadraticParallelExpression2,
    diameter: BezierRecursiveQuadraticParallelExpression2,
    radius_squared_denominator: BezierRecursiveQuadraticParallelExpression2,
    tangent_cross_source: Vec<BezierRecursiveQuadraticValue2>,
    tangent_dot_source: BezierRecursiveQuadraticParallelExpression2,
    weight: Vec<BezierRecursiveQuadraticValue2>,
}

/// Embedding of one isolated analytic-curve parameter into the selected
/// circle's recursive coefficient tower.  Existing positive quadratic
/// generators are replayed over the enlarged dense base in their original
/// order; the target parameter therefore stays independent until the exact
/// authored-sheet incidence accepts it.
struct BezierRecursiveQuadraticTargetEmbedding2 {
    field: BezierRecursiveQuadraticField2,
    parameter: BezierRecursiveQuadraticValue2,
    source_base: Arc<BezierRecursiveQuadraticBaseFieldData2>,
    target_base: Arc<BezierRecursiveQuadraticBaseFieldData2>,
    source_axes: Vec<usize>,
    target_axis: usize,
    extensions: Vec<BezierRecursiveQuadraticExtensionEmbedding2>,
}

/// One retained chord support and rational target over the endpoints' least
/// shared recursive quadratic field. Linear and quadratic incidence roots
/// stay in that local projective field; only higher-degree targets use dense
/// projection for enumeration. Every candidate is replayed on the authored
/// endpoint sheet before finite containment or tangent orientation becomes
/// topology evidence.
#[derive(Debug)]
struct BezierRecursiveProjectiveChordRationalSystem2 {
    field: BezierRecursiveQuadraticField2,
    /// Finite endpoints are retained only when incidence already uses their
    /// field. A chord with an older authoritative support clips candidates
    /// through its point classifier instead of importing unrelated boundary
    /// fields eagerly.
    start: Option<BezierRecursiveQuadraticProjectivePoint2>,
    end: Option<BezierRecursiveQuadraticProjectivePoint2>,
    source_x: Vec<BezierRecursiveQuadraticValue2>,
    source_y: Vec<BezierRecursiveQuadraticValue2>,
    source_weight: Vec<BezierRecursiveQuadraticValue2>,
    source_weight_sign: Option<RealSign>,
    incidence: Vec<BezierRecursiveQuadraticValue2>,
    tangent_cross: Vec<BezierRecursiveQuadraticValue2>,
    affine_preimage_incidence_factor_sign: Option<RealSign>,
    tangent_from_incidence_derivative_sign: Option<RealSign>,
}

/// One retained chord support and analytic parallel target over the
/// endpoints' least shared recursive quadratic field. The target source-speed
/// root remains procedural and positive; projection only enumerates target
/// parameters, while exact replay owns incidence, finite containment, and
/// tangent orientation.
#[derive(Debug)]
struct BezierRecursiveProjectiveChordParallelSystem2 {
    field: BezierRecursiveQuadraticField2,
    base: Arc<BezierRecursiveQuadraticBaseFieldData2>,
    /// Global elimination is a demand-driven fallback. Local roots retain
    /// their selected coefficient field and leave this cache empty.
    projection: OnceLock<DenseTensorPolynomial>,
    incidence: BezierRecursiveQuadraticParallelExpression2,
    source_weight: Vec<BezierRecursiveQuadraticValue2>,
    tangent_cross: Vec<BezierRecursiveQuadraticValue2>,
    tangent_dot: Vec<BezierRecursiveQuadraticValue2>,
    /// Selected-axis differences from the two finite chord endpoints.  The
    /// monotone root authority first clips through exact geometric envelopes
    /// and constructs these substantially larger expressions only when that
    /// certificate overlaps an endpoint.
    coordinate_differences: Option<[BezierRecursiveQuadraticParallelExpression2; 2]>,
}

#[derive(Clone, Copy)]
enum BezierRecursiveParallelCandidateEvidence2<'a> {
    /// Local isolation, including certified deflation, proves the selected
    /// norm vanishes. The caller has certified positive speed on the domain;
    /// the unsquared component signs must still select the authored sheet.
    SelectedNorm,
    Projected(&'a DenseTensorPolynomial),
    Replay,
}

#[derive(Debug)]
struct BezierRecursiveProjectiveChordParallelIntervalSystem2 {
    incidence_rational: Vec<RealInterval>,
    incidence_radical: Vec<RealInterval>,
    speed_squared: Vec<RealInterval>,
    precision: i32,
}

/// Fixed-distance incidence from one recursively retained analytic-parallel
/// point. The center parameter and its positive speed remain in `field`; only
/// the independent candidate parameter is projected and every norm root is
/// replayed against the authored candidate-speed sheet.
#[derive(Debug)]
struct BezierRecursiveFixedDistanceSystem2 {
    field: BezierRecursiveQuadraticField2,
    base: Arc<BezierRecursiveQuadraticBaseFieldData2>,
    projection: DenseTensorPolynomial,
    incidence: BezierRecursiveQuadraticParallelExpression2,
    source_weight: Vec<BezierRecursiveQuadraticValue2>,
    unit_target_speed: bool,
}

struct BezierRecursiveQuadraticParallelEvaluation2 {
    embedding: BezierRecursiveQuadraticTargetEmbedding2,
    speed_field: BezierRecursiveQuadraticField2,
    speed: BezierRecursiveQuadraticValue2,
}

/// Rank-independent chord-normal projective authority.
/// Every selected endpoint field occupies one tensor axis and the final axis
/// is the affine target parameter. The two positive support-speed radicals
/// remain procedural and are never flattened into Cartesian coordinates.
#[derive(Clone, Debug)]
struct BezierChordNormalDenseMapSystem2 {
    source_representations: Vec<AlgebraicRootRepresentation>,
    first_speed_squared: DenseTensorPolynomial,
    second_speed_squared: DenseTensorPolynomial,
    diameter: BezierDenseTwoSquareRootExpression2,
    radius_squared_denominator: BezierDenseTwoSquareRootExpression2,
}

#[derive(Debug)]
struct BezierChordNormalDenseIntersectionSystem2 {
    map: Arc<BezierChordNormalDenseMapSystem2>,
    incidence: BezierDenseTwoSquareRootExpression2,
    selected_half_plane: BezierDenseTwoSquareRootExpression2,
    tangent_cross: BezierDenseTwoSquareRootExpression2,
    angular_tangent: Option<BezierDenseTwoSquareRootExpression2>,
    geometry: Option<BezierChordNormalDenseTargetGeometry2>,
}

#[derive(Clone, Debug)]
struct BezierChordNormalProjectiveFrameSource2 {
    first_support: BezierChordParallelSupportSource2,
    second_support: BezierChordParallelSupportSource2,
    anchor_speed: usize,
    coordinates: [AlgebraicRootRepresentation; 12],
}

#[derive(Debug)]
struct BezierChordNormalDenseChordParameterMapSystem2 {
    projective: Arc<BezierChordNormalDenseMapSystem2>,
    geometry: BezierChordNormalDenseTargetGeometry2,
    tangent_cross: BezierDenseTwoSquareRootExpression2,
    angular_tangent: BezierDenseTwoSquareRootExpression2,
    recursive_contact_fields: [std::sync::OnceLock<BezierRecursiveQuadraticField2>; 3],
}

#[derive(Clone, Debug)]
struct BezierDenseTwoSquareRootExpression2 {
    rational: DenseTensorPolynomial,
    first: DenseTensorPolynomial,
    second: DenseTensorPolynomial,
    product: DenseTensorPolynomial,
}

/// One exact coefficient field retained by recursively composed line/circle
/// contacts.  The base is the existing dense selected-root/two-speed-radical
/// authority.  Every later contact appends only its positive quadratic
/// discriminant, so depth grows linearly while each level shares its parent.
#[derive(Clone, Debug)]
enum BezierRecursiveQuadraticField2 {
    Base(Arc<BezierRecursiveQuadraticBaseFieldData2>),
    Extension(Arc<BezierRecursiveQuadraticExtensionFieldData2>),
}

#[derive(Debug)]
struct BezierRecursiveQuadraticBaseFieldData2 {
    sources: Vec<AlgebraicRootRepresentation>,
    source_real_witnesses: Vec<Option<Real>>,
    source_refinement: OnceLock<Mutex<BezierRecursiveQuadraticSourceRefinement2>>,
    first_speed_squared: DenseTensorPolynomial,
    second_speed_squared: DenseTensorPolynomial,
}

struct BezierRecursiveQuadraticSourceRefinement2 {
    parameters: Vec<Option<BezierParameterRefinement2<'static>>>,
}

impl std::fmt::Debug for BezierRecursiveQuadraticSourceRefinement2 {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("BezierRecursiveQuadraticSourceRefinement2")
            .field("axes", &self.parameters.len())
            .finish_non_exhaustive()
    }
}

#[derive(Debug)]
struct BezierRecursiveQuadraticExtensionFieldData2 {
    parent: BezierRecursiveQuadraticField2,
    /// Strictly positive at the retained parent-field tuple.
    radicand: BezierRecursiveQuadraticValue2,
}

/// One exact embedding of a source quadratic generator into a transient
/// common tower. The target generator has the source radicand embedded in its
/// own parent, so both name the same positive square root.
struct BezierRecursiveQuadraticExtensionEmbedding2 {
    source: Arc<BezierRecursiveQuadraticExtensionFieldData2>,
    target: Arc<BezierRecursiveQuadraticExtensionFieldData2>,
}

/// One normalized value in a retained recursive quadratic field.  Extension
/// values are `retained + radical * sqrt(radicand)` with the positive square
/// root.  Both coefficients live in the shared parent field.
#[derive(Clone, Debug)]
struct BezierRecursiveQuadraticValue2 {
    data: Arc<BezierRecursiveQuadraticValueData2>,
}

#[derive(Debug)]
enum BezierRecursiveQuadraticValueData2 {
    Base {
        field: Arc<BezierRecursiveQuadraticBaseFieldData2>,
        expression: BezierDenseTwoSquareRootExpression2,
        real_witness: std::sync::OnceLock<Real>,
    },
    Extension {
        field: Arc<BezierRecursiveQuadraticExtensionFieldData2>,
        retained: BezierRecursiveQuadraticValue2,
        radical: BezierRecursiveQuadraticValue2,
        real_witness: std::sync::OnceLock<Real>,
    },
}

#[derive(Clone, Debug)]
struct BezierRecursiveQuadraticProjectiveScalar2 {
    numerator: BezierRecursiveQuadraticValue2,
    /// Certified strictly positive by construction.
    denominator: BezierRecursiveQuadraticValue2,
}

/// Exact root of one strictly monotone authored chord/parallel incidence.
///
/// `source_lower` and `source_upper` are represented parameters carrying
/// opposite strict signs of `system.incidence`.  They are construction
/// evidence, not approximations to the selected root.  `map_*` retains an
/// exact Mobius chart from that source parameter to the scalar currently
/// exposed by [`BezierRecursiveProjectiveParameter2`].
#[derive(Clone, Debug)]
struct BezierRecursiveMonotoneParameter2 {
    system: Arc<BezierRecursiveProjectiveChordParallelSystem2>,
    side_chord: BezierAlgebraicChord2,
    side_parallel: BezierParallel2,
    source_lower: Real,
    source_upper: Real,
    source_lower_sign: RealSign,
    source_upper_sign: RealSign,
    source_refinement_steps: usize,
    refinement_cache: Arc<Mutex<BezierRecursiveMonotoneRefinementCache2>>,
    map_numerator: [Real; 2],
    map_denominator: [Real; 2],
}

#[derive(Clone, Debug)]
struct BezierRecursiveMonotoneRefinementCache2 {
    source_lower: Real,
    source_upper: Real,
    source_lower_sign: RealSign,
    source_upper_sign: RealSign,
    steps: usize,
    interval_system: Option<Arc<BezierRecursiveProjectiveChordParallelIntervalSystem2>>,
    interval_system_attempted: bool,
}

#[derive(Clone, Debug)]
enum BezierRecursiveProjectiveParameterAuthority2 {
    Projective(BezierRecursiveQuadraticProjectiveScalar2),
    Monotone(BezierRecursiveMonotoneParameter2),
    Polynomial {
        authority: Arc<BezierRecursivePolynomialParameterAuthority2>,
        // These signs belong to the parameter's immutable bounds, not to
        // the shared polynomial. Ordering may need only the lower sign.
        endpoint_signs: [OnceLock<RealSign>; 2],
    },
}

/// Shared exact authority for roots selected directly over one recursive
/// quadratic coefficient field.  Simple roots remain local Bernstein fibers;
/// a global norm is constructed only by cold scalar promotion or by the
/// complete repeated-root fallback.
#[derive(Debug)]
struct BezierRecursivePolynomialParameterAuthority2 {
    field: BezierRecursiveQuadraticField2,
    coefficients: Vec<BezierRecursiveQuadraticValue2>,
}

/// One exact scalar retained directly in its recursive quadratic field.
///
/// This is the compact construction authority for projective images that
/// already own every selected root and positive radical sheet.  The stored
/// interval is only an outward finite envelope; equality and ordering replay
/// the recursive field expression itself.  A global algebraic polynomial is
/// constructed only when a later carrier genuinely requires an ordinary
/// [`BezierParameter2`].
#[derive(Clone, Debug)]
pub(crate) struct BezierRecursiveProjectiveParameter2 {
    data: Arc<BezierRecursiveProjectiveParameterData2>,
}

#[derive(Debug)]
struct BezierRecursiveProjectiveParameterData2 {
    authority: BezierRecursiveProjectiveParameterAuthority2,
    /// Successful strict projection belongs to this selected root, including
    /// its refinements and exact field embeddings. Distinct roots and changed
    /// parameter charts own separate cells; uncertainty is never retained.
    projection: Arc<BezierRecursiveParameterProjection2>,
    lower: Real,
    upper: Real,
    refinement_steps: usize,
    identity: Option<Arc<BezierRecursiveProjectiveParameterIdentity2>>,
    line_branch: i8,
    policy: CurveContext,
}

#[derive(Debug, Default)]
struct BezierRecursiveParameterProjection2 {
    parameter: OnceLock<BezierParameter2>,
    /// One composed chart back to an unmapped selected root. Refinements
    /// share this identity; a later projection replays the map rather than
    /// eliminating an independently reconstructed image polynomial.
    chart: Option<Arc<BezierRecursiveParameterChart2>>,
}

#[derive(Debug)]
struct BezierRecursiveParameterChart2 {
    source: BezierRecursiveProjectiveParameter2,
    numerator: [Real; 2],
    denominator: [Real; 2],
}

#[derive(Clone, Debug)]
struct BezierRecursiveLineParameterIdentity2 {
    source_frame: Arc<BezierSelectedRadialFrameData2>,
    source_radial_distance: Real,
    source_clockwise: bool,
    line: LineSeg2,
    /// Certified map from the canonical source carrier to `line`.
    transform: Option<Similarity2>,
}

#[derive(Clone, Debug)]
struct BezierRecursiveChordRationalTangentIdentity2 {
    chord: BezierAlgebraicChord2,
    source: RationalBezier2,
    /// `chord` traversal crossed with increasing target traversal.
    tangent_cross_sign: RealSign,
    /// Finite-chord location certified while the recursive contact was
    /// filtered. This byte occupies existing alignment padding and lets a
    /// later Boolean endpoint recover contact-to-endpoint orientation without
    /// comparing independently selected Cartesian coordinates.
    chord_location: BezierRecursiveChordContactLocation2,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BezierRecursiveChordContactLocation2 {
    Start,
    Interior,
    End,
}

#[derive(Clone, Debug)]
enum BezierRecursiveProjectiveParameterIdentity2 {
    Line(BezierRecursiveLineParameterIdentity2),
    ChordRationalTangent(BezierRecursiveChordRationalTangentIdentity2),
}

impl PartialEq for BezierRecursiveProjectiveParameter2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
    }
}

#[derive(Clone, Debug)]
struct BezierRecursiveQuadraticProjectivePoint2 {
    x: BezierRecursiveQuadraticValue2,
    y: BezierRecursiveQuadraticValue2,
    /// Certified strictly positive by construction.
    denominator: BezierRecursiveQuadraticValue2,
}

/// Circle frame embedded in one recursive coefficient field. The unit
/// direction `(center - support_center) / normal_denominator` selects
/// parameter zero; the support may be a parent circle center or a synthetic
/// anchor retaining the authored normal.
#[derive(Clone, Debug)]
struct BezierRecursiveCircleFrame2 {
    field: BezierRecursiveQuadraticField2,
    center: BezierRecursiveQuadraticProjectivePoint2,
    support_center: BezierRecursiveQuadraticProjectivePoint2,
    normal_denominator: Real,
}

/// One chord contact and its supporting circle center represented in the
/// recursive quadratic authority. Both points share the same strictly
/// positive projective denominator and field.
#[derive(Clone)]
struct BezierRecursiveQuadraticChordContactFrame2 {
    field: BezierRecursiveQuadraticField2,
    point: BezierRecursiveQuadraticProjectivePoint2,
    center: BezierRecursiveQuadraticProjectivePoint2,
}

/// One circle-pair contact and both support centers in their least shared
/// recursive quadratic tower.
#[derive(Clone, Debug)]
struct BezierRecursiveQuadraticPairContactFrame2 {
    field: BezierRecursiveQuadraticField2,
    point: BezierRecursiveQuadraticProjectivePoint2,
    centers: [BezierRecursiveQuadraticProjectivePoint2; 2],
}

enum BezierRecursiveProjectivePointSource2 {
    Exact(Point2),
    Algebraic(RationalBezierAlgebraicPointImage2),
    Recursive(BezierRecursiveQuadraticProjectivePoint2),
}

#[derive(Debug)]
struct BezierRecursiveQuadraticLineContactSystem2 {
    branch: i8,
    /// Exact affine parameter on the authored line, retained in the same
    /// recursive tower as `point` and every contact predicate.
    parameter: BezierRecursiveProjectiveParameter2,
    /// Circle-tangent/chord-tangent dot sign reduced before adjoining this
    /// contact discriminant. Both branches receive the same parent-field
    /// certificate; storing its byte beside `branch` consumes existing
    /// alignment padding instead of enlarging the owning map allocation.
    tangent_dot_sign: Option<RealSign>,
    point: BezierRecursiveQuadraticProjectivePoint2,
    /// Dot of the authored parameter-zero radial with this contact radial,
    /// multiplied by the same strictly positive projective scale as
    /// `radius_squared_denominator`.
    diameter: BezierRecursiveQuadraticValue2,
    radius_squared_denominator: BezierRecursiveQuadraticValue2,
    tangent_cross: BezierRecursiveQuadraticValue2,
    angular_tangent: BezierRecursiveQuadraticValue2,
}

/// Arbitrary-depth retained line/circle map.  It is the recursive continuation
/// of the dense chord-normal map, not a second intersection engine: the same
/// exact circle predicates are stored in a quadratic tower instead of being
/// flattened to a degree-multiplied univariate resultant.
#[derive(Debug)]
struct BezierRecursiveQuadraticLineParameterMapSystem2 {
    /// Every constructor normalizes this point and `center` to strictly
    /// positive projective denominators before publishing the map. Contact
    /// replay may therefore lift either value without re-signing it.
    center: BezierRecursiveQuadraticProjectivePoint2,
    contacts: Vec<BezierRecursiveQuadraticLineContactSystem2>,
}

#[derive(Debug)]
struct BezierChordNormalDenseTarget2 {
    x: DenseTensorPolynomial,
    y: DenseTensorPolynomial,
    weight: DenseTensorPolynomial,
    tangent_x: DenseTensorPolynomial,
    tangent_y: DenseTensorPolynomial,
}

#[derive(Clone, Debug)]
struct BezierChordNormalDenseTargetGeometry2 {
    point_x: BezierDenseTwoSquareRootExpression2,
    point_y: BezierDenseTwoSquareRootExpression2,
    center_x: BezierDenseTwoSquareRootExpression2,
    center_y: BezierDenseTwoSquareRootExpression2,
    common_denominator: DenseTensorPolynomial,
}

impl BezierChordNormalDenseIntersectionSystem2 {
    fn contact_parameters(
        &self,
        domain: SelectedThirdAxisDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        chord_normal_dense_expression_parameters(&self.map, &self.incidence, domain, policy)
    }

    fn selected_half_plane_sign(
        &self,
        target: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.map
            .expression_sign(&self.selected_half_plane, target, policy)
    }

    fn selected_half_plane_parameters(
        &self,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        chord_normal_dense_expression_parameters(
            &self.map,
            &self.selected_half_plane,
            SelectedThirdAxisDomain2::Finite(range),
            policy,
        )
    }

    fn diameter_sign(
        &self,
        target: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.map.expression_sign(&self.map.diameter, target, policy)
    }

    fn tangent_cross_sign(
        &self,
        target: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        self.map
            .expression_sign(&self.tangent_cross, target, policy)
    }

    fn angular_tangent_parameters(
        &self,
        range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        let Some(angular_tangent) = self.angular_tangent.as_ref() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        chord_normal_dense_expression_parameters(
            &self.map,
            angular_tangent,
            SelectedThirdAxisDomain2::Finite(range),
            policy,
        )
    }

    fn angular_tangent_sign(
        &self,
        target: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let Some(angular_tangent) = self.angular_tangent.as_ref() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        self.map.expression_sign(angular_tangent, target, policy)
    }

    fn rational_parameter_map_system(&self) -> Arc<BezierChordNormalDenseMapSystem2> {
        self.map.clone()
    }

    fn into_parameter_map_system(self) -> BezierChordNormalDenseChordParameterMapSystem2 {
        BezierChordNormalDenseChordParameterMapSystem2 {
            projective: self.map,
            geometry: self
                .geometry
                .expect("a chord target retains its dense point map"),
            tangent_cross: self.tangent_cross,
            angular_tangent: self
                .angular_tangent
                .expect("a chord target retains its dense angular tangent"),
            recursive_contact_fields: std::array::from_fn(|_| std::sync::OnceLock::new()),
        }
    }
}

impl BezierChordNormalDenseMapSystem2 {
    fn diameter_parameter_expression(
        &self,
        denominator: &Real,
        radial_coefficient: &Real,
    ) -> Option<BezierDenseTwoSquareRootExpression2> {
        self.diameter.scale(denominator).and_then(|diameter| {
            self.radius_squared_denominator
                .scale(radial_coefficient)
                .and_then(|radius| diameter.subtract(&radius))
        })
    }

    fn diameter_parameter_sign(
        &self,
        target: &BezierParameter2,
        denominator: &Real,
        radial_coefficient: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let Some(predicate) = self.diameter_parameter_expression(denominator, radial_coefficient)
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        self.expression_sign(&predicate, target, policy)
    }
}

/// One value `R + B*sqrt(K)` in the four selected roots of a pair-radial
/// circle and an independent algebraic chord.
#[derive(Clone, Debug)]
struct BezierAlgebraicCuspQuadrivariateSquareRootExpression2 {
    rational: QuadrivariatePolynomial2,
    radical: QuadrivariatePolynomial2,
}

/// One nested value `X + branch*Y*sqrt(S)` where `X`, `Y`, and the
/// nonnegative chord discriminant `S` all live in the retained pair-radical
/// extension.  This is the complete radical tower for a line contact; no
/// primitive element or additional selected root is needed.
#[derive(Clone, Debug)]
struct BezierSelectedRadialCircleChordNestedExpression2 {
    retained: BezierAlgebraicCuspQuadrivariateSquareRootExpression2,
    candidate: BezierAlgebraicCuspQuadrivariateSquareRootExpression2,
}

impl BezierSelectedRadialCircleChordNestedExpression2 {
    fn from_retained(
        retained: BezierAlgebraicCuspQuadrivariateSquareRootExpression2,
    ) -> Option<Self> {
        Some(Self {
            retained,
            candidate: BezierAlgebraicCuspQuadrivariateSquareRootExpression2::from_rational(
                QuadrivariatePolynomial2::zero([1; 4])?,
            )?,
        })
    }

    fn from_rational(rational: QuadrivariatePolynomial2) -> Option<Self> {
        Self::from_retained(
            BezierAlgebraicCuspQuadrivariateSquareRootExpression2::from_rational(rational)?,
        )
    }

    fn subtract(&self, other: &Self) -> Option<Self> {
        Some(Self {
            retained: self.retained.subtract(&other.retained)?,
            candidate: self.candidate.subtract(&other.candidate)?,
        })
    }

    fn scale(&self, scale: &Real) -> Option<Self> {
        Some(Self {
            retained: self.retained.scale(scale)?,
            candidate: self.candidate.scale(scale)?,
        })
    }

    fn multiply_rational(&self, polynomial: &QuadrivariatePolynomial2) -> Option<Self> {
        Some(Self {
            retained: self.retained.multiply_rational(polynomial)?,
            candidate: self.candidate.multiply_rational(polynomial)?,
        })
    }

    fn linear_combination(terms: &[(&Self, &Real)]) -> Option<Self> {
        let retained_terms = terms
            .iter()
            .map(|(expression, scale)| (&expression.retained, *scale))
            .collect::<Vec<_>>();
        let candidate_terms = terms
            .iter()
            .map(|(expression, scale)| (&expression.candidate, *scale))
            .collect::<Vec<_>>();
        Some(Self {
            retained: BezierAlgebraicCuspQuadrivariateSquareRootExpression2::linear_combination(
                &retained_terms,
            )?,
            candidate: BezierAlgebraicCuspQuadrivariateSquareRootExpression2::linear_combination(
                &candidate_terms,
            )?,
        })
    }
}

/// Four-root exact line-circle system for a pair-radial selected circle and a
/// genuinely algebraic chord support.
#[derive(Debug)]
struct BezierSelectedRadialCircleChordSystem2 {
    retained: BezierSelectedRadialCircleChordParameterMapSystem2,
    selected_half_plane: BezierSelectedRadialCircleChordNestedExpression2,
    point_minus_start: BezierSelectedRadialCircleChordNestedExpression2,
    point_minus_end: BezierSelectedRadialCircleChordNestedExpression2,
}

#[derive(Debug)]
struct BezierSelectedRadialCircleChordParameterMapSystem2 {
    pair_map: BezierAlgebraicCuspSemicirclePairParameterMap2,
    pair_branch: i8,
    pair_discriminant: QuadrivariatePolynomial2,
    chord_discriminant: BezierAlgebraicCuspQuadrivariateSquareRootExpression2,
    diameter: BezierSelectedRadialCircleChordNestedExpression2,
    radius_squared_denominator: QuadrivariatePolynomial2,
    common_denominator: QuadrivariatePolynomial2,
    center_x: BezierAlgebraicCuspQuadrivariateSquareRootExpression2,
    center_y: BezierAlgebraicCuspQuadrivariateSquareRootExpression2,
    point_x: BezierSelectedRadialCircleChordNestedExpression2,
    point_y: BezierSelectedRadialCircleChordNestedExpression2,
    first_parameter: BezierParameter2,
    second_parameter: BezierParameter2,
}

/// Exact analytic intersection system for one selected algebraic circle and
/// an oblique chord whose endpoint images remain in two independent fields.
/// The endpoint roots and circle-center root occupy the three tensor axes; the
/// line parameter is eliminated analytically, leaving only one selected square
/// root and at most two finite contacts.
#[derive(Debug)]
struct BezierAlgebraicCuspSemicircleObliqueChordSystem2 {
    retained: BezierAlgebraicCuspSemicircleObliqueChordParameterMapSystem2,
    selected_half_plane: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    point_minus_start: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    point_minus_end: BezierAlgebraicCuspTrivariateSquareRootExpression2,
}

#[derive(Debug)]
struct BezierAlgebraicCuspSemicircleObliqueChordParameterMapSystem2 {
    discriminant: TrivariatePolynomial,
    diameter_side: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    radius_squared_denominator: TrivariatePolynomial,
    common_denominator: TrivariatePolynomial,
    center_x: TrivariatePolynomial,
    center_y: TrivariatePolynomial,
    point_x: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    point_y: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    first_parameter: BezierParameter2,
    second_parameter: BezierParameter2,
    cusp_parameter: BezierParameter2,
}

/// One value `X + branch*Y*sqrt(E)` where `X`, `Y`, and the nonnegative
/// contact discriminant `E` live in the positive chord-speed extension.
/// This is the exact tower needed by a procedural retained chord offset:
/// endpoint fields remain independent, `sqrt(D dot D)` is never materialized,
/// and the outer branch is the ordinary pair of circle/line contacts.
#[derive(Clone, Debug)]
struct BezierAlgebraicCuspRetainedOffsetChordNestedExpression2 {
    retained: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    candidate: BezierAlgebraicCuspTrivariateSquareRootExpression2,
}

#[derive(Debug)]
struct BezierAlgebraicCuspSemicircleRetainedOffsetChordSystem2 {
    retained: BezierAlgebraicCuspSemicircleRetainedOffsetChordParameterMapSystem2,
    selected_half_plane: BezierAlgebraicCuspRetainedOffsetChordNestedExpression2,
    point_minus_start: BezierAlgebraicCuspRetainedOffsetChordNestedExpression2,
    point_minus_end: BezierAlgebraicCuspRetainedOffsetChordNestedExpression2,
}

#[derive(Debug)]
struct BezierAlgebraicCuspSemicircleRetainedOffsetChordParameterMapSystem2 {
    speed_squared: TrivariatePolynomial,
    contact_discriminant: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    diameter_side: BezierAlgebraicCuspRetainedOffsetChordNestedExpression2,
    tangent_dot: BezierAlgebraicCuspTrivariateSquareRootExpression2,
    radius_squared_denominator: TrivariatePolynomial,
    common_denominator: TrivariatePolynomial,
    center_x: TrivariatePolynomial,
    center_y: TrivariatePolynomial,
    point_x: BezierAlgebraicCuspRetainedOffsetChordNestedExpression2,
    point_y: BezierAlgebraicCuspRetainedOffsetChordNestedExpression2,
    first_parameter: BezierParameter2,
    second_parameter: BezierParameter2,
    cusp_parameter: BezierParameter2,
}

/// One finite contact between a selected algebraic semicircle and a
/// certified axis-aligned retained chord.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierAlgebraicCuspSemicircleChordContact2 {
    /// `-1` and `+1` select the two transverse support contacts; zero is the
    /// unique tangent contact.  Branch order follows the chord traversal.
    branch: i8,
    /// Exact affine parameter on a retained algebraic chord handled by the
    /// common chord-normal projective kernel.
    projective_parameter: Option<BezierParameter2>,
    pub(crate) cusp_location: BezierAlgebraicCuspSemicircleContactLocation2,
    pub(crate) chord_location: BezierAlgebraicCuspSemicircleContactLocation2,
    pub(crate) tangent_cross_sign: RealSign,
}

#[derive(Clone, Debug)]
pub(crate) enum BezierAlgebraicCuspSemicircleChordIntersections2 {
    NoContacts,
    Contacts {
        contacts: Vec<BezierAlgebraicCuspSemicircleChordContact2>,
        parameter_map: BezierAlgebraicCuspSemicircleChordParameterMap2,
    },
}

/// One fully retained finite contact produced by the authoritative selected
/// semicircle/chord kernel.  Fast paths may use different local elimination
/// systems, but callers receive the same exact carrier parameters and point
/// evidence.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspSemicircleRetainedChordContact2 {
    pub(crate) cusp_parameter: BezierAlgebraicCuspSemicircleParameter2,
    pub(crate) chord_parameter: BezierAlgebraicChordParameter2,
    pub(crate) point: CurvePoint2,
    pub(crate) tangent_cross_sign: RealSign,
}

/// Exact affine point retained by a selected algebraic-circle/axis-chord
/// contact.  The one-word carrier shares the same map/contact allocation as
/// the corresponding mapped semicircle parameter.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspChordPoint2 {
    data: Arc<BezierAlgebraicCuspSemicircleMappedParameterData2>,
}

/// One-word exact point derived from a correlated selected-circle contact
/// without flattening its selected fields.
///
/// Chord contacts reuse their original radical system. Other mapped contacts
/// retain their exact source point and selected-circle parameter proof, then
/// refine those two existing fields independently for nonstructural queries.
/// No primitive element or approximate construction coordinate is introduced.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspChordDerivedPoint2 {
    data: Arc<BezierAlgebraicCuspChordDerivedPointData2>,
}

#[derive(Debug)]
struct BezierAlgebraicCuspChordDerivedPointData2 {
    source: BezierAlgebraicCuspDerivedPointSource2,
    radial_scale: Real,
    /// Coefficient of the counterclockwise perpendicular source radius.
    /// Existing concentric offsets keep this zero; retained cusp chamfers use
    /// it to rotate a mapped endpoint without constructing a compositum.
    perpendicular_scale: Real,
    translation_x: Real,
    translation_y: Real,
}

#[derive(Clone, Debug)]
enum BezierAlgebraicCuspDerivedPointSource2 {
    Chord(BezierAlgebraicCuspChordPoint2),
    Mapped {
        parameter: Arc<BezierAlgebraicCuspSemicircleMappedParameterData2>,
        /// A one-field point image when the mapped carrier can publish one.
        /// General analytic parallels retain `None` and evaluate their exact
        /// point bounds lazily from the native parameter and unit normal.
        point: Option<CurvePoint2>,
    },
}

impl PartialEq for BezierAlgebraicCuspDerivedPointSource2 {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Chord(first), Self::Chord(second)) => first == second,
            (
                Self::Mapped {
                    parameter: first_parameter,
                    point: first_point,
                },
                Self::Mapped {
                    parameter: second_parameter,
                    point: second_point,
                },
            ) => Arc::ptr_eq(first_parameter, second_parameter) && first_point == second_point,
            (Self::Chord(_), Self::Mapped { .. }) | (Self::Mapped { .. }, Self::Chord(_)) => false,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BezierAlgebraicCuspSemicirclePairEndpoint2 {
    FirstStart,
    FirstEnd,
    SecondStart,
    SecondEnd,
}

/// One-word exact positive-length overlap between two selected algebraic
/// semicircles on the same supporting circle.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspSemicirclePairOverlap2 {
    data: Arc<BezierAlgebraicCuspSemicirclePairOverlapData2>,
}

/// Exact local parameter on one retained algebraic-cusp semicircle.
///
/// Interior values retain the pair/contact predicate that orders the value
/// against any represented rational parameter. This deliberately avoids a
/// primitive element for the independent selected roots. Exact values remain
/// inline, while mapped values share their larger proof through one pointer.
#[derive(Clone, Debug)]
pub(crate) enum BezierAlgebraicCuspSemicircleParameter2 {
    Exact(Real),
    Mapped(Arc<BezierAlgebraicCuspSemicircleMappedParameterData2>),
}

#[derive(Debug)]
pub(crate) enum BezierSelectedChordNormalAnchor2 {
    Represented((Real, Real)),
    RetainedChord(BezierAlgebraicChord2),
    RetainedCircleChord {
        map: BezierAlgebraicCuspSemicircleChordParameterMap2,
        contact: BezierAlgebraicCuspSemicircleChordContact2,
    },
    RetainedCircleRationalChord {
        map: BezierAlgebraicCuspSemicircleRationalParameterMap2,
        contact: BezierAlgebraicCuspSemicircleRationalMapContact2,
    },
}

#[derive(Debug)]
pub(crate) enum BezierAlgebraicCuspSemicircleMappedParameterData2 {
    Rational {
        map: BezierAlgebraicCuspSemicircleRationalParameterMap2,
        contact: BezierAlgebraicCuspSemicircleRationalMapContact2,
    },
    SelectedFiberRational {
        map: BezierAlgebraicCuspSemicircleSelectedFiberRationalParameterMap2,
        other_parameter: BezierAlgebraicSelectedFiberParameter2,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
        tangent_cross_sign: RealSign,
    },
    SelectedFiberParallel {
        map: BezierAlgebraicCuspSemicircleSelectedFiberParallelParameterMap2,
        other_parameter: BezierAlgebraicSelectedFiberParameter2,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
        tangent_cross_sign: RealSign,
    },
    Parallel {
        map: BezierAlgebraicCuspSemicircleParallelParameterMap2,
        contact: BezierAlgebraicCuspSemicircleParallelContact2,
    },
    /// A fillet contact retained directly in the two selected source-tangent
    /// fields that define its angular position.
    ///
    /// The circle start radius is a signed left normal of its selected frame;
    /// the contact radius is a signed left normal of `parallel`.  Their sign
    /// product plus the two unnormalized source tangents orders the contact
    /// against every represented circle parameter without a square root,
    /// resultant, or primitive element.
    SelectedParallelContact {
        semicircle: BezierAlgebraicCuspSemicircle2,
        parallel: BezierParallel2,
        parameter: CurveParameter2,
        location: BezierAlgebraicCuspSemicircleContactLocation2,
        radial_product_sign: RealSign,
        tangent_cross_sign: RealSign,
        tangent_dot_sign: RealSign,
        policy: CurveContext,
    },
    /// One round-join endpoint whose contact radial is the signed left normal
    /// of a retained selected-circle traversal tangent.
    ///
    /// The companion endpoint already owns the two-normal circle/parallel
    /// contact map. Reusing that map to order this angular parameter avoids a
    /// four-field circle-circle or oblique-chord reconstruction.
    SelectedCircularTangentContact {
        semicircle: BezierAlgebraicCuspSemicircle2,
        companion: BezierAlgebraicCuspSemicircleFragment2,
        companion_at_start: bool,
        parallel: BezierParallel2,
        parameter: CurveParameter2,
        source_direction: RealSign,
        radial_product_sign: RealSign,
        point: CurvePoint2,
        policy: CurveContext,
    },
    /// One fillet endpoint retained directly from the independent selected
    /// circle-pair contact that solved the fillet center.
    ///
    /// `anchor_first` records which pair carrier supplies this circle's start
    /// radial. The pair map then signs every angular comparison from its
    /// shared dot/discriminant representation, without constructing a
    /// primitive element for the two selected fields.
    SelectedPairContact {
        semicircle: BezierAlgebraicCuspSemicircle2,
        map: BezierAlgebraicCuspSemicirclePairParameterMap2,
        contact: BezierAlgebraicCuspSemicirclePairContact2,
        anchor_first: bool,
        radial_product_sign: RealSign,
        point: CurvePoint2,
        policy: CurveContext,
    },
    /// One exact round-join endpoint whose radial direction is the signed
    /// left normal of a retained algebraic chord.
    ///
    /// `anchor_tangent` and `chord` are the covariant local angular authority
    /// from the authored join. The anchor is either a represented vector or
    /// the exact retained circle/chord tangent map that produced the center.
    /// `radial_product_sign` also covers a contact on the complementary half
    /// without changing that authority. A similarity preserves their angular
    /// relation; represented anchors transform in place, while retained maps
    /// use the generic exact transport. This avoids adjoining the chord's
    /// endpoint fields and normalization radical to the selected-circle field.
    SelectedChordNormalContact {
        semicircle: BezierAlgebraicCuspSemicircle2,
        anchor_tangent: BezierSelectedChordNormalAnchor2,
        chord: BezierAlgebraicChord2,
        radial_product_sign: RealSign,
        point: CurvePoint2,
        policy: CurveContext,
    },
    /// One exact round-join endpoint whose radial is a retained chord normal
    /// and whose circle chart starts on an analytic-parallel normal.
    ///
    /// The selected center parameter and two independent chord endpoint fields
    /// remain separate. Angular predicates sign only the required
    /// chord/parallel tangent linear combination at the decision boundary.
    SelectedChordParallelNormalContact {
        semicircle: BezierAlgebraicCuspSemicircle2,
        parallel: BezierParallel2,
        parallel_parameter: CurveParameter2,
        chord: BezierAlgebraicChord2,
        radial_product_sign: RealSign,
        point: CurvePoint2,
        policy: CurveContext,
    },
    Pair {
        map: BezierAlgebraicCuspSemicirclePairParameterMap2,
        contact: BezierAlgebraicCuspSemicirclePairContact2,
        first: bool,
    },
    Chord {
        map: BezierAlgebraicCuspSemicircleChordParameterMap2,
        contact: BezierAlgebraicCuspSemicircleChordContact2,
    },
    PairOverlap {
        overlap: BezierAlgebraicCuspSemicirclePairOverlap2,
        endpoint: BezierAlgebraicCuspSemicirclePairEndpoint2,
        first: bool,
    },
    PairOverlapMap {
        overlap: BezierAlgebraicCuspSemicirclePairOverlap2,
        source: BezierAlgebraicCuspSemicircleParameter2,
        source_first: bool,
    },
    /// A mapped local parameter transported by one certified similarity.
    ///
    /// The selected-circle chart is similarity covariant: its scalar
    /// parameter is unchanged even under reflection because the transformed
    /// frame, signed radius, and traversal turn are reflected together.  The
    /// source parameter therefore remains the exact ordering authority while
    /// `point` retains the transformed Cartesian evidence without constructing
    /// a compositum for either selected field.
    SimilarityTransport {
        semicircle: BezierAlgebraicCuspSemicircle2,
        source: BezierAlgebraicCuspSemicircleParameter2,
        point: CurvePoint2,
        policy: CurveContext,
    },
    /// Exact angular transport on one selected supporting circle chart.
    ///
    /// `half_angle` is the coefficient of the positive-orientation Mobius
    /// transform of `source`. On the source half it is the physical
    /// `tan(theta / 2)`. On the complementary half it is that value's negative
    /// reciprocal; zero names the diameter antipode. `point` retains the
    /// physical center-relative rotation independently of this compact chart
    /// coordinate.
    Chamfer {
        semicircle: BezierAlgebraicCuspSemicircle2,
        source: BezierAlgebraicCuspSemicircleParameter2,
        half_angle: Real,
        point: CurvePoint2,
        policy: CurveContext,
    },
}

enum BezierAlgebraicCuspSemicircleMappedTangentSource2<'a> {
    Rational {
        curve: &'a RationalBezier2,
        parameter: &'a BezierParameter2,
        policy: CurveContext,
    },
    Parallel {
        parallel: &'a BezierParallel2,
        parameter: &'a BezierParameter2,
        policy: CurveContext,
    },
}

#[derive(Clone)]
enum BezierAlgebraicCuspSemicircleMappedPointParameter2 {
    Ordinary(BezierParameter2),
    Selected(BezierAlgebraicSelectedFiberParameter2),
}

enum BezierAlgebraicCuspSemicircleMappedPointSource2 {
    Rational {
        curve: RationalBezier2,
        parameter: BezierAlgebraicCuspSemicircleMappedPointParameter2,
        policy: CurveContext,
    },
    Parallel {
        parallel: BezierParallel2,
        parameter: BezierAlgebraicCuspSemicircleMappedPointParameter2,
        policy: CurveContext,
    },
}

#[derive(Clone, Copy)]
enum BezierAlgebraicCuspSemicircleMappedPointParameterRef2<'a> {
    Ordinary(&'a BezierParameter2),
    Selected(&'a BezierAlgebraicSelectedFiberParameter2),
}

impl BezierAlgebraicCuspSemicircleMappedPointParameterRef2<'_> {
    fn polynomial_sign(
        self,
        coefficients: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        match self {
            Self::Ordinary(parameter) => {
                signed_coefficients_at_parameter(coefficients, parameter, policy)
            }
            Self::Selected(parameter) => parameter.predicate_sign(
                &bivariate_outer_product(&[Real::one()], coefficients),
                policy,
            ),
        }
    }

    fn to_curve_region_parameter(self) -> CurveParameter2 {
        match self {
            Self::Ordinary(parameter) => CurveParameter2::from(parameter.clone()),
            Self::Selected(parameter) => CurveParameter2::from_selected_fiber(parameter.clone()),
        }
    }

    fn matching_target_parameters(
        self,
        pairs: impl IntoIterator<Item = (CurveParameter2, CurveParameter2)>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<CurveParameter2>>> {
        let mut retained = Vec::new();
        for (candidate_source, candidate_target) in pairs {
            let same = self
                .to_curve_region_parameter()
                .same_value(&candidate_source, policy)?;
            match same {
                Classification::Decided(true) => retained.push(candidate_target),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(retained))
    }
}

/// Collapses duplicate parameter evidence before it becomes a topology event.
/// Distinct intersection authorities may certify the same image through an
/// isolated contact, a structural overlap, and an extracted component. Only a
/// strict exact equality may remove one candidate; an undecided comparison
/// conservatively retains both.
fn decided_mapped_point_parameters(
    mut parameters: Vec<CurveParameter2>,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
    let mut candidate = 1_usize;
    while candidate < parameters.len() {
        let duplicate = policy.strict_predicate_pass(|| -> CurveResult<bool> {
            for retained in &parameters[..candidate] {
                if matches!(
                    retained.same_value(&parameters[candidate], policy)?,
                    Classification::Decided(true)
                ) {
                    return Ok(true);
                }
            }
            Ok(false)
        })?;
        if duplicate {
            parameters.remove(candidate);
        } else {
            candidate += 1;
        }
    }
    Ok(Classification::Decided(Some(parameters)))
}

impl BezierAlgebraicCuspSemicircleMappedTangentSource2<'_> {
    fn policy(&self) -> CurveContext {
        match self {
            Self::Rational { policy, .. } | Self::Parallel { policy, .. } => *policy,
        }
    }

    fn parameter(&self) -> &BezierParameter2 {
        match self {
            Self::Rational { parameter, .. } | Self::Parallel { parameter, .. } => parameter,
        }
    }

    fn tangent_power_basis(&self) -> CurveResult<[Vec<Real>; 2]> {
        match self {
            Self::Rational { curve, .. } => Ok(rational_parametric_tangent_numerator(
                curve.homogeneous_power_basis()?,
            )),
            Self::Parallel { parallel, .. } => {
                let differential = parallel.differential()?;
                Ok([
                    differential.tangent_x.clone(),
                    differential.tangent_y.clone(),
                ])
            }
        }
    }
}

/// Isolating bounds schedule a local carrier query; the retained parameter
/// still owns its point. Pointwise nonvanishing guarantees that a regular
/// neighborhood exists, even when the unused native chart has poles or cusps.
fn mapped_point_regular_source_range(
    parallel: &BezierParallel2,
    parameter: BezierAlgebraicCuspSemicircleMappedPointParameterRef2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<CurveParameterRange2>> {
    policy.strict_predicate_pass(|| {
        let source = parallel.source_power_basis()?;
        let speed = parallel_speed_squared_polynomial(parallel.differential()?);
        for polynomial in source.weight.into_iter().chain(std::iter::once(&speed[..])) {
            match parameter.polynomial_sign(polynomial, policy)? {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
                Classification::Decided(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        let parameter = parameter.to_curve_region_parameter();
        let mut steps = 0_usize;
        let mut radius = Real::one();
        loop {
            let refined = match parameter.refined_for_finite_envelope(steps, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let Some((lower, upper)) = refined.finite_envelope_bounds() else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let (lower, upper) = match compare_reals(lower, upper, policy) {
                Some(std::cmp::Ordering::Less) => (lower.clone(), upper.clone()),
                Some(std::cmp::Ordering::Equal) => (lower - &radius, upper + &radius),
                Some(std::cmp::Ordering::Greater) => return Err(CurveError::InvalidBezierRange),
                None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
            };
            let range = CurveParameterRange2::new_validated(lower.into(), upper.into());
            let mut regular = true;
            for polynomial in source.weight.into_iter().chain(std::iter::once(&speed[..])) {
                if !matches!(
                    polynomial_is_nonzero_on_parameter_range(polynomial, &range, policy)?,
                    Classification::Decided(true)
                ) {
                    regular = false;
                    break;
                }
            }
            if regular {
                return Ok(Classification::Decided(range));
            }
            radius = (radius / Real::from(2)).expect("two is nonzero");
            steps = steps
                .checked_mul(2)
                .and_then(|steps| steps.checked_add(1))
                .ok_or_else(|| {
                    CurveError::Topology("local point carrier refinement overflow".into())
                })?;
        }
    })
}

fn append_mapped_point_parameters(
    candidates: &mut Vec<CurveParameter2>,
    mapped: Classification<Option<Vec<CurveParameter2>>>,
) -> Classification<bool> {
    match mapped {
        Classification::Decided(Some(mut mapped)) => {
            candidates.append(&mut mapped);
            Classification::Decided(true)
        }
        Classification::Decided(None) => Classification::Decided(false),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

fn finish_mapped_point_parameter_candidates(
    mut candidates: Vec<CurveParameter2>,
    component_overlaps: &[BezierParameterComponentOverlap2],
    parameter_components: &[BezierParallelPairIntersectionParameterComponent2],
    overlaps: &[RationalBezierIntersectionOverlap2],
    overlap_curves: Option<(&RationalBezier2, &RationalBezier2)>,
    parameter: BezierAlgebraicCuspSemicircleMappedPointParameterRef2<'_>,
    complete: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
    if !complete {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let component_overlap_decided = match append_mapped_point_parameters(
        &mut candidates,
        mapped_point_parameters_through_component_overlaps(
            component_overlaps,
            CurveResultantParameter::First,
            parameter,
            policy,
        )?,
    ) {
        Classification::Decided(decided) => decided,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let has_parameter_components = !parameter_components.is_empty();
    let parameter_component_decided = match append_mapped_point_parameters(
        &mut candidates,
        mapped_point_parameters_through_parameter_components(
            parameter_components
                .iter()
                .map(|component| (component.first_parameter(), component.second_parameter())),
            parameter,
            policy,
        )?,
    ) {
        Classification::Decided(decided) => decided,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if has_parameter_components && !parameter_component_decided {
        return Ok(Classification::Decided(None));
    }
    let has_overlap = !overlaps.is_empty();
    let overlap_decided = if has_overlap {
        let Some((source_curve, target_curve)) = overlap_curves else {
            return Ok(Classification::Decided(None));
        };
        match append_mapped_point_parameters(
            &mut candidates,
            mapped_point_parameters_through_rational_overlaps(
                overlaps,
                source_curve,
                target_curve,
                CurveResultantParameter::First,
                parameter,
                policy,
            )?,
        ) {
            Classification::Decided(decided) => decided,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    } else {
        false
    };
    if has_overlap && !overlap_decided {
        return Ok(Classification::Decided(None));
    }
    if !candidates.is_empty()
        || component_overlap_decided
        || parameter_component_decided
        || overlap_decided
    {
        decided_mapped_point_parameters(candidates, policy)
    } else if complete {
        Ok(Classification::Decided(Some(Vec::new())))
    } else {
        Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
    }
}

fn mapped_point_parameters_through_component_overlaps(
    overlaps: &[BezierParameterComponentOverlap2],
    retained_parameter: CurveResultantParameter,
    parameter: BezierAlgebraicCuspSemicircleMappedPointParameterRef2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
    if overlaps.is_empty() {
        return Ok(Classification::Decided(None));
    }
    let parameter = parameter.to_curve_region_parameter();
    let mut mapped = Vec::with_capacity(overlaps.len());
    for overlap in overlaps {
        match policy.strict_predicate_pass(|| {
            overlap.map_curve_parameter(retained_parameter, &parameter, policy)
        })? {
            Classification::Decided(Some(parameter)) => mapped.push(parameter),
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "mapped-circle-point-inverse",
        "parameter-component-map",
    );
    Ok(Classification::Decided(Some(mapped)))
}

fn mapped_point_parameters_through_parameter_components<'a>(
    components: impl IntoIterator<Item = (Option<&'a BezierParameter2>, Option<&'a BezierParameter2>)>,
    parameter: BezierAlgebraicCuspSemicircleMappedPointParameterRef2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
    policy.strict_predicate_pass(|| {
        let mut saw_component = false;
        let mut mapped = Vec::new();
        for (source, target) in components {
            saw_component = true;
            match (source, target) {
                (None, Some(target)) => {
                    mapped.push(CurveParameter2::from(target.clone()));
                }
                (Some(source), Some(target)) => {
                    match parameter
                        .to_curve_region_parameter()
                        .same_value(&source.clone().into(), policy)?
                    {
                        Classification::Decided(true) => {
                            mapped.push(CurveParameter2::from(target.clone()));
                        }
                        Classification::Decided(false) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                (Some(source), None) => {
                    match parameter
                        .to_curve_region_parameter()
                        .same_value(&source.clone().into(), policy)?
                    {
                        Classification::Decided(true) => {
                            return Ok(Classification::Decided(None));
                        }
                        Classification::Decided(false) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                (None, None) => return Ok(Classification::Decided(None)),
            }
        }
        if !saw_component {
            return Ok(Classification::Decided(None));
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "mapped-circle-point-inverse",
            "point-parameter-component",
        );
        Ok(Classification::Decided(Some(mapped)))
    })
}

fn mapped_point_parameters_through_rational_overlaps(
    overlaps: &[RationalBezierIntersectionOverlap2],
    source_curve: &RationalBezier2,
    target_curve: &RationalBezier2,
    source_parameter: CurveResultantParameter,
    parameter: BezierAlgebraicCuspSemicircleMappedPointParameterRef2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
    if overlaps.is_empty() {
        return Ok(Classification::Decided(None));
    }
    let source = parameter.to_curve_region_parameter();
    let mut mapped = Vec::with_capacity(overlaps.len());
    for authored_overlap in overlaps {
        let swapped;
        let overlap = match source_parameter {
            CurveResultantParameter::First => authored_overlap,
            CurveResultantParameter::Second => {
                swapped = swapped_parallel_overlap(authored_overlap);
                &swapped
            }
        };
        let correspondence = policy.strict_predicate_pass(|| {
            RationalBezierOverlapParameterCorrespondence2::for_overlap(
                source_curve,
                target_curve,
                overlap,
                policy,
            )
        });
        if matches!(
            (&correspondence, parameter),
            (
                RationalBezierOverlapParameterCorrespondence2::General { .. },
                BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Selected(_),
            )
        ) {
            return Ok(Classification::Decided(None));
        }
        match policy.strict_predicate_pass(|| {
            overlap_parameter_is_in_range(&source, overlap.first_range(), true, policy)
        })? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => continue,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        match policy.strict_predicate_pass(|| {
            correspondence.map_first_to_second_region_parameter(
                &source,
                overlap.first_range(),
                overlap.second_range(),
                policy,
            )
        })? {
            Classification::Decided(Some(parameter)) => mapped.push(parameter),
            Classification::Decided(None) => {
                return Err(CurveError::Topology(
                    "a certified overlap omitted its retained point correspondence".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "mapped-circle-point-inverse",
        "rational-overlap-parameter-map",
    );
    Ok(Classification::Decided(Some(mapped)))
}

/// Encloses one exact analytic-parallel point without adjoining its normalized
/// tangent field to the retained source parameter.
fn analytic_parallel_point_bounds_refined(
    parallel: &BezierParallel2,
    parameter: &BezierParameter2,
    tangent_distance: &Real,
    translation_x: &Real,
    translation_y: &Real,
    refinement_steps: usize,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    let parameter = parameter
        .clone()
        .refined_isolating_interval(refinement_steps, policy);
    let parameter = real_interval_from_parameter(&parameter);
    analytic_parallel_point_bounds_over_interval(
        parallel,
        &parameter,
        tangent_distance,
        translation_x,
        translation_y,
    )
}

fn retained_analytic_parallel_point_bounds_at_bezier_parameter(
    point: &BezierAnalyticParallelPoint2,
    parameter: &BezierParameter2,
) -> Classification<Aabb2> {
    analytic_parallel_point_bounds_over_interval_with_tangent(
        &point.data.parallel,
        &real_interval_from_parameter(parameter),
        point
            .data
            .frame_tangent
            .as_ref()
            .map(|tangent| (&tangent.x[..], &tangent.y[..])),
        &point.data.tangent_distance,
        &point.data.translation_x,
        &point.data.translation_y,
    )
}

fn rational_bezier_point_bounds_refined(
    curve: &RationalBezier2,
    parameter: &BezierParameter2,
    refinement_steps: usize,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    let parameter = parameter
        .clone()
        .refined_isolating_interval(refinement_steps, policy);
    rational_bezier_point_bounds_over_interval(curve, &real_interval_from_parameter(&parameter))
}

fn rational_bezier_point_bounds_over_interval(
    curve: &RationalBezier2,
    parameter: &RealInterval,
) -> Classification<Aabb2> {
    let source = match curve.homogeneous_power_basis() {
        Ok(source) => source,
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    let evaluate = |polynomial: &[Real]| RealInterval::evaluate_power_basis(polynomial, parameter);
    let (Some(x), Some(y), Some(weight)) = (
        evaluate(&source.x_numerator),
        evaluate(&source.y_numerator),
        evaluate(&source.weight),
    ) else {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    };
    let (Some(x), Some(y)) = (x.divide(&weight), y.divide(&weight)) else {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    };
    Classification::Decided(Aabb2::new_unchecked(
        Point2::new(x.lower, y.lower),
        Point2::new(x.upper, y.upper),
    ))
}

fn analytic_parallel_point_bounds_over_interval(
    parallel: &BezierParallel2,
    parameter: &RealInterval,
    tangent_distance: &Real,
    translation_x: &Real,
    translation_y: &Real,
) -> Classification<Aabb2> {
    analytic_parallel_point_bounds_over_interval_with_tangent(
        parallel,
        parameter,
        None,
        tangent_distance,
        translation_x,
        translation_y,
    )
}

fn analytic_parallel_point_bounds_over_interval_with_tangent(
    parallel: &BezierParallel2,
    parameter: &RealInterval,
    frame_tangent: Option<(&[Real], &[Real])>,
    tangent_distance: &Real,
    translation_x: &Real,
    translation_y: &Real,
) -> Classification<Aabb2> {
    let evaluate = |polynomial: &[Real]| RealInterval::evaluate_power_basis(polynomial, parameter);
    let source = match parallel.source_power_basis() {
        Ok(source) => source,
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    let (Some(mut point_x), Some(mut point_y)) =
        (evaluate(source.x_numerator), evaluate(source.y_numerator))
    else {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    };
    if let Some(weight) = source.weight {
        let Some(weight) = evaluate(weight) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let (Some(x), Some(y)) = (point_x.divide(&weight), point_y.divide(&weight)) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        point_x = x;
        point_y = y;
    }
    // A zero displacement is the source point even at a stationary
    // parameter. Only nonzero normal or tangent offsets need a unit frame.
    if parallel.distance().zero_status() != ZeroKnowledge::Zero
        || tangent_distance.zero_status() != ZeroKnowledge::Zero
    {
        let differential;
        let (frame_tangent_x, frame_tangent_y) = match frame_tangent {
            Some(tangent) => tangent,
            None => {
                differential = match parallel.differential() {
                    Ok(differential) => differential,
                    Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
                };
                (&differential.tangent_x[..], &differential.tangent_y[..])
            }
        };
        let (Some(tangent_x), Some(tangent_y)) =
            (evaluate(frame_tangent_x), evaluate(frame_tangent_y))
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(speed) = tangent_x
            .square()
            .and_then(|x| tangent_y.square().map(|y| x.add(&y)))
            .and_then(|speed_squared| speed_squared.nonnegative_square_root(None))
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(normal_x) = (RealInterval {
            lower: -tangent_y.upper.clone(),
            upper: -tangent_y.lower.clone(),
        })
        .divide(&speed) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(normal_y) = tangent_x.divide(&speed) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let distance = RealInterval {
            lower: parallel.distance().clone(),
            upper: parallel.distance().clone(),
        };
        let tangent_distance = RealInterval {
            lower: tangent_distance.clone(),
            upper: tangent_distance.clone(),
        };
        let (
            Some(normal_offset_x),
            Some(normal_offset_y),
            Some(tangent_offset_x),
            Some(tangent_offset_y),
        ) = (
            normal_x.multiply(&distance),
            normal_y.multiply(&distance),
            tangent_x
                .divide(&speed)
                .and_then(|unit| unit.multiply(&tangent_distance)),
            tangent_y
                .divide(&speed)
                .and_then(|unit| unit.multiply(&tangent_distance)),
        )
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        point_x = point_x.add(&normal_offset_x).add(&tangent_offset_x);
        point_y = point_y.add(&normal_offset_y).add(&tangent_offset_y);
    }
    let point_x = point_x.add(&RealInterval {
        lower: translation_x.clone(),
        upper: translation_x.clone(),
    });
    let point_y = point_y.add(&RealInterval {
        lower: translation_y.clone(),
        upper: translation_y.clone(),
    });
    Classification::Decided(Aabb2::new_unchecked(
        Point2::new(point_x.lower, point_y.lower),
        Point2::new(point_x.upper, point_y.upper),
    ))
}

/// Recovers every common zero at one parameter already represented in a
/// single selected algebraic field.
///
/// Either equation is a complete candidate projection. The other is replayed
/// in the same correlated fiber, rejecting conjugate and repeated-coordinate
/// roots without constructing a Cartesian primitive element. An optional
/// positive branch predicate selects one sheet of an implicit carrier.
fn one_field_common_zero_parameter_candidates(
    parameter: BezierAlgebraicCuspSemicircleMappedPointParameterRef2<'_>,
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    positive_branch: Option<&BivariatePolynomial>,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
    let mut last_reason = UncertaintyReason::Unsupported;
    let mut identically_zero = 0;
    for (incidence, replay) in [(first, second), (second, first)] {
        let projection = match parameter {
            BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Ordinary(
                BezierParameter2::Algebraic(parameter),
            ) => selected_fiber_parameters_in_range(incidence, parameter, range, policy)?,
            BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Selected(parameter) => {
                selected_fiber_polynomial_relation_parameters(parameter, incidence, range, policy)?
            }
            BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Ordinary(
                BezierParameter2::Exact(_),
            ) => unreachable!("represented parameters use scalar point incidence"),
        };
        let candidates = match projection {
            Classification::Decided(Some(candidates)) => candidates,
            Classification::Decided(None) => {
                identically_zero += 1;
                continue;
            }
            Classification::Uncertain(reason) => {
                last_reason = reason;
                continue;
            }
        };
        let mut retained = Vec::with_capacity(candidates.len());
        let mut retry_reason = None;
        for candidate in candidates {
            // The desired sheet is strictly positive. Reject an unsigned
            // candidate before asking whether its remaining incidence is zero.
            let sign = |predicate: &BivariatePolynomial| match parameter {
                BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Ordinary(_) => {
                    candidate.predicate_sign(predicate, policy)
                }
                BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Selected(source) => {
                    algebraic_selected_fiber_pair_predicate_sign(
                        source, &candidate, predicate, policy,
                    )
                }
            };
            if let Some(branch) = positive_branch {
                let branch_sign = sign(branch)?;
                match branch_sign {
                    Classification::Decided(RealSign::Positive) => {}
                    Classification::Decided(RealSign::Negative | RealSign::Zero) => continue,
                    Classification::Uncertain(reason) => {
                        retry_reason = Some(reason);
                        break;
                    }
                }
            }
            let replay_zero =
                if let BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Selected(source) =
                    parameter
                    && matches!(
                        sign(&bivariate_parameter_derivative(
                            incidence,
                            CurveResultantParameter::Second
                        ))?,
                        Classification::Decided(RealSign::Negative | RealSign::Positive)
                    )
                {
                    // Projection already proved this incidence at the selected
                    // pair. Its nonzero derivative makes it a simple local root
                    // authority, avoiding the larger conjugate image norm.
                    algebraic_selected_fiber_pair_projected_root_via_subresultants(
                        source,
                        &candidate,
                        replay,
                        Some(incidence),
                        policy,
                    )?
                } else {
                    sign(replay)?.map(|sign| sign == RealSign::Zero)
                };
            match replay_zero {
                Classification::Decided(true) => {}
                Classification::Decided(false) => continue,
                Classification::Uncertain(reason) => {
                    retry_reason = Some(reason);
                    break;
                }
            }
            retained.push(CurveParameter2::from_selected_fiber(candidate));
        }
        if let Some(reason) = retry_reason {
            last_reason = reason;
            continue;
        }
        return Ok(Classification::Decided(Some(retained)));
    }
    if identically_zero == 2 {
        if let Some(branch) = positive_branch {
            // The caller proved a finite, regular source frame. With both
            // incidence equations identically zero and nonzero displacement,
            // the normal branch cannot vanish anywhere in this interval.
            // Its sign at one interior point therefore owns the whole fiber.
            let sample = match range.strict_interior_scalar(policy)? {
                Classification::Decided(sample) => sample,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            match parameter
                .polynomial_sign(&bivariate_specialize_second(branch, &sample), policy)?
            {
                Classification::Decided(RealSign::Positive) => {}
                Classification::Decided(RealSign::Negative) => {
                    return Ok(Classification::Decided(Some(Vec::new())));
                }
                Classification::Decided(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "implicit carrier branch vanished on a certified regular component".into(),
                    ));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        // An empty inverse and an entire parameter fiber are distinct.
        return Ok(Classification::Decided(None));
    }
    Ok(Classification::Uncertain(last_reason))
}

/// Inverts an exact or singly-selected point on a finite target domain.
/// Rational curves use the zero-distance specialization: unsquared coordinate
/// incidence needs no normal, even at stationary or constant source points.
/// Nonzero offsets retain correlated incidence and signed normal-sheet replay.
fn one_field_point_parameter_candidates(
    point: &CurvePoint2,
    target: &BezierParallel2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
    if let CurvePoint2(CurvePointData2::Exact(point)) = point {
        // Keep the scalar GCD path for an already represented point.
        return Ok(target.point_incidence(point, range, policy)?.map(
            |incidence| match incidence {
                BezierParallelIncidence2::Parameters(parameters) => {
                    Some(curve_region_parameters_from_bezier(parameters))
                }
                BezierParallelIncidence2::EntireCurve => None,
            },
        ));
    }
    let point = match algebraic_axis_point_coordinates(point, policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    parametric_point_parameter_candidates(
        BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Ordinary(&point.parameter),
        [&point.x, &point.y, &point.denominator],
        target,
        range,
        policy,
    )
}

/// Homogeneous point incidence retains the source scalar's native authority.
/// Only its local denominator sign is required, including negative weights.
fn parametric_point_parameter_candidates(
    parameter: BezierAlgebraicCuspSemicircleMappedPointParameterRef2<'_>,
    [x, y, weight]: [&[Real]; 3],
    target: &BezierParallel2,
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<Vec<CurveParameter2>>>> {
    let weight_sign = match parameter.polynomial_sign(weight, policy)? {
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let represented = match parameter {
        BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Ordinary(
            BezierParameter2::Exact(value),
        ) => Some(value),
        BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Ordinary(
            BezierParameter2::Algebraic(_),
        ) => None,
        BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Selected(value) => {
            value.represented_value()
        }
    };
    if let Some(value) = represented {
        let evaluate = |coefficients: &[Real]| {
            coefficients
                .iter()
                .rev()
                .fold(Real::zero(), |sum, coefficient| sum * value + coefficient)
        };
        let denominator = evaluate(weight);
        let point = Point2::new(
            (evaluate(x) / &denominator).expect("the selected denominator is nonzero"),
            (evaluate(y) / &denominator).expect("the selected denominator is nonzero"),
        );
        return one_field_point_parameter_candidates(&point.into(), target, range, policy);
    }
    let recover = |first: &BivariatePolynomial,
                   second: &BivariatePolynomial,
                   positive_branch: Option<&BivariatePolynomial>| {
        one_field_common_zero_parameter_candidates(
            parameter,
            first,
            second,
            positive_branch,
            range,
            policy,
        )
    };
    let distance_sign = match real_sign(target.distance(), policy) {
        Some(sign) => sign,
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    let source = target.source_power_basis()?;
    if let Classification::Uncertain(reason) = target.certify_source_frame_in_domain(
        SelectedThirdAxisDomain2::Finite(range),
        None,
        policy,
    )? {
        return Ok(Classification::Uncertain(reason));
    }

    let unit_weight = [Real::one()];
    let source_weight = source.weight.unwrap_or(&unit_weight);
    let delta_x = bivariate_subtract(
        &bivariate_outer_product(x, source_weight),
        &bivariate_outer_product(weight, source.x_numerator),
    );
    let delta_y = bivariate_subtract(
        &bivariate_outer_product(y, source_weight),
        &bivariate_outer_product(weight, source.y_numerator),
    );
    if distance_sign == RealSign::Zero {
        return recover(&delta_x, &delta_y, None);
    }
    let differential = target.differential()?;
    let tangent_x = bivariate_outer_product(&[Real::one()], &differential.tangent_x);
    let tangent_y = bivariate_outer_product(&[Real::one()], &differential.tangent_y);
    let orthogonality = bivariate_add(
        &bivariate_multiply(&delta_x, &tangent_x),
        &bivariate_multiply(&delta_y, &tangent_y),
    );
    let common_weight = bivariate_outer_product(weight, source_weight);
    let distance_squared = target.distance() * target.distance();
    let distance_relation = bivariate_subtract(
        &bivariate_add(
            &bivariate_multiply(&delta_x, &delta_x),
            &bivariate_multiply(&delta_y, &delta_y),
        ),
        &bivariate_scale(
            bivariate_multiply(&common_weight, &common_weight),
            &distance_squared,
        ),
    );
    let orientation = bivariate_subtract(
        &bivariate_multiply(&delta_y, &tangent_x),
        &bivariate_multiply(&delta_x, &tangent_y),
    );
    let orientation = if source.weight.is_some() {
        bivariate_multiply(
            &orientation,
            &bivariate_outer_product(&[Real::one()], source_weight),
        )
    } else {
        orientation
    };
    let signed_distance = if weight_sign == RealSign::Negative {
        -target.distance()
    } else {
        target.distance().clone()
    };
    let branch = bivariate_scale(orientation, &signed_distance);
    recover(&orthogonality, &distance_relation, Some(&branch))
}

/// One-word exact subfragment of an algebraic-cusp semicircle.
///
/// The retained local range is ascending; `reversed` records traversal. The
/// range may use exact represented endpoints or mapped contact cuts without
/// materializing the selected cusp fields into a high-degree scalar tower.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicCuspSemicircleFragment2 {
    data: Arc<BezierAlgebraicCuspSemicircleFragmentData2>,
}

/// Exact location of a caller-certified incident point on one retained
/// selected-circle fragment. Endpoint names follow fragment traversal rather
/// than the ascending local parameter range.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BezierAlgebraicCuspSemicircleIncidentLocation2 {
    Exterior,
    Start,
    Interior,
    End,
}

/// One-word exact straight chord whose endpoints may remain algebraic.
///
/// The endpoint images are retained independently. In particular, this does
/// not construct a primitive element when a chamfer joins cuts from two
/// unrelated curve fields. Exact predicates combine those fields only at the
/// decision boundary, and `policy` remains part of the carrier so replay
/// cannot silently weaken a STRICT construction.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicChord2 {
    data: Arc<BezierAlgebraicChordData2>,
}

/// One endpoint displaced along an exact unit direction of a retained chord.
///
/// The shared carrier keeps the two independently selected source endpoints
/// separate and lazily evaluates either the left-normal image
/// `P + d*(-dy, dx)/sqrt(dx^2 + dy^2)` or the tangent image
/// `P + d*(dx, dy)/sqrt(dx^2 + dy^2)`. This is the general counterpart of the
/// represented-unit-tangent fast path: no primitive element, rounded
/// coordinate, or approximate construction fact is introduced.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicChordParallelPoint2 {
    data: Arc<BezierAlgebraicChordParallelData2>,
    at_end: bool,
}

/// One exact point on an analytic Bezier parallel at a retained parameter.
///
/// The point evaluates `P(t) + d*(-H_y,H_x)/sqrt(H dot H)` lazily. Its one-word
/// handle keeps the selected parameter and normalized source expression
/// together without adjoining the speed square root or rounding coordinates.
#[derive(Clone, Debug)]
pub(crate) struct BezierAnalyticParallelPoint2 {
    data: Arc<BezierAnalyticParallelPointData2>,
}

/// One exact retained point transported by a certified planar similarity.
///
/// The source evidence remains authoritative and clone-shared.  In particular,
/// two adjacent selected-fiber fragments that shared one algebraic contact
/// before transformation still share that contact afterward; no independent
/// coordinate reconstruction or primitive-element field is introduced.
/// Compatible transforms compose at construction, so repeated transport keeps
/// one view of the original field rather than a history of similarity layers.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierSimilarityPoint2 {
    data: Arc<BezierSimilarityPointData2>,
}

#[derive(Debug, PartialEq)]
struct BezierSimilarityPointData2 {
    source: CurvePoint2,
    transform: Similarity2,
    policy: CurveContext,
}

#[derive(Debug)]
struct BezierAnalyticParallelPointData2 {
    parallel: BezierParallel2,
    parameter: BezierAnalyticParallelPointParameter2,
    /// Optional source-oriented tangent field after cancelling a common
    /// hodograph factor on one regular branch.  Ordinary parallel points use
    /// the carrier differential directly; source-cusp limit points retain
    /// this field so their unit normal and tangent remain defined at the
    /// singular parameter without changing the open-branch curve carrier.
    frame_tangent: Option<Arc<BezierAnalyticParallelTangentField2>>,
    /// Signed displacement along the source's unit tangent. The parallel
    /// distance already stores the orthogonal unit-normal displacement.
    tangent_distance: Real,
    translation_x: Real,
    translation_y: Real,
    policy: CurveContext,
    bounds_cache: Mutex<Option<(CurveContext, usize, Aabb2)>>,
    recursive_projective_point: OnceLock<BezierRecursiveQuadraticProjectivePoint2>,
}

#[derive(Debug, PartialEq)]
struct BezierAnalyticParallelTangentField2 {
    x: Vec<Real>,
    y: Vec<Real>,
}

#[derive(Clone, Debug, PartialEq)]
enum BezierAnalyticParallelPointParameter2 {
    Bezier(BezierParameter2),
    SelectedFiber(BezierAlgebraicSelectedFiberParameter2),
    RecursiveProjective(BezierRecursiveProjectiveParameter2),
}

impl BezierAnalyticParallelPointParameter2 {
    fn curve_parameter(&self) -> CurveParameter2 {
        match self {
            Self::Bezier(parameter) => parameter.clone().into(),
            Self::SelectedFiber(parameter) => {
                CurveParameter2::from_selected_fiber(parameter.clone())
            }
            Self::RecursiveProjective(parameter) => {
                CurveParameter2::from_recursive_projective(parameter.clone())
            }
        }
    }

    fn matches_region_parameter(&self, parameter: &CurveParameter2) -> bool {
        match self {
            Self::Bezier(retained) => parameter
                .as_bezier_parameter()
                .is_some_and(|parameter| parameter == retained),
            Self::SelectedFiber(retained) => parameter
                .as_selected_fiber()
                .is_some_and(|parameter| parameter == retained),
            Self::RecursiveProjective(retained) => parameter
                .as_recursive_projective()
                .is_some_and(|parameter| parameter == retained),
        }
    }
}

#[derive(Debug)]
struct BezierAlgebraicChordParallelData2 {
    source: BezierAlgebraicChord2,
    /// Optional arbitrary origin for a displaced interior/contact point.
    /// Endpoint pairs leave this empty and share one allocation selected by
    /// `at_end`, preserving their compact two-word handles.
    source_point: Option<Arc<CurvePoint2>>,
    distance: Real,
    translation_x: Real,
    translation_y: Real,
    /// Reuses one lazy normalized-expression carrier for offset normals and
    /// corner-setback tangents.
    direction: BezierAlgebraicChordUnitDisplacement2,
    policy: CurveContext,
    recursive_points: OnceLock<[BezierRecursiveQuadraticProjectivePoint2; 2]>,
}

struct BezierAlgebraicChordParallelRecursiveFrame2 {
    displaced: [BezierRecursiveQuadraticProjectivePoint2; 2],
    direction_endpoints: [BezierRecursiveQuadraticProjectivePoint2; 2],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BezierAlgebraicChordUnitDisplacement2 {
    LeftNormal,
    Tangent,
}

/// Certified traversal direction of an axis-aligned retained algebraic chord.
///
/// Cardinal directions contain no normalized algebraic scalar, so an exact
/// signed parallel can translate each retained endpoint in its existing
/// selected field. Certified similarities may retain the corresponding exact
/// unit tangent for an oblique image; an uncertified general direction remains
/// on the algebraic parallel-carrier boundary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BezierAlgebraicChordAxisDirection2 {
    PositiveX,
    NegativeX,
    PositiveY,
    NegativeY,
}

impl BezierAlgebraicChordAxisDirection2 {
    const fn cardinal_components(self) -> (i8, i8) {
        match self {
            Self::PositiveX => (1, 0),
            Self::NegativeX => (-1, 0),
            Self::PositiveY => (0, 1),
            Self::NegativeY => (0, -1),
        }
    }

    pub(crate) const fn axis(self) -> Axis2 {
        match self {
            Self::PositiveX | Self::NegativeX => Axis2::X,
            Self::PositiveY | Self::NegativeY => Axis2::Y,
        }
    }

    /// Converts the coordinate order of a point against this oriented axis
    /// support into the corresponding exact line side.
    ///
    /// `order` compares the point coordinate with the support coordinate on
    /// the axis perpendicular to this direction.
    const fn line_side_from_perpendicular_order(
        self,
        order: std::cmp::Ordering,
    ) -> crate::classify::LineSide {
        use crate::classify::LineSide;
        use std::cmp::Ordering;

        match order {
            Ordering::Equal => LineSide::On,
            Ordering::Greater => match self {
                Self::PositiveX | Self::NegativeY => LineSide::Left,
                Self::NegativeX | Self::PositiveY => LineSide::Right,
            },
            Ordering::Less => match self {
                Self::PositiveX | Self::NegativeY => LineSide::Right,
                Self::NegativeX | Self::PositiveY => LineSide::Left,
            },
        }
    }

    pub(crate) fn unit_tangent(self) -> (Real, Real) {
        match self {
            Self::PositiveX => (Real::one(), Real::zero()),
            Self::NegativeX => (-Real::one(), Real::zero()),
            Self::PositiveY => (Real::zero(), Real::one()),
            Self::NegativeY => (Real::zero(), -Real::one()),
        }
    }

    const fn parameter_axis(self) -> BezierAlgebraicChordParameterAxis2 {
        BezierAlgebraicChordParameterAxis2 {
            axis: self.axis(),
            coordinate_increases: matches!(self, Self::PositiveX | Self::PositiveY),
        }
    }
}

/// Compact exact local parameter on one retained algebraic chord.
///
/// The parameter retains the affine point itself and orders it on a certified
/// monotone coordinate of the chord.  This avoids constructing an unrelated
/// primitive element merely to materialize the normalized scalar in `[0, 1]`.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicChordParameter2 {
    data: BezierAlgebraicChordParameterStorage2,
}

#[derive(Clone, Debug)]
enum BezierAlgebraicChordParameterStorage2 {
    Endpoint {
        chord: BezierAlgebraicChord2,
        at_end: bool,
    },
    Interior(Arc<BezierAlgebraicChordParameterData2>),
}

#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicChordRationalContact2 {
    chord_parameter: BezierAlgebraicChordParameter2,
    /// Exact source parameter, including local selected-fiber and recursive
    /// carriers that must not be materialized merely to publish the contact.
    other_parameter: CurveParameter2,
    point: CurvePoint2,
    tangent_cross_sign: RealSign,
}

/// Exact positive-length overlap between one retained algebraic chord and one
/// monotone branch of a rational line image.
///
/// The chord range follows the chord's local parameter order.  The source
/// range is oriented to the same image traversal and can therefore be
/// descending when the source runs in the opposite direction.  Retaining the
/// source and chord once also supplies exact endpoint transport when a
/// CurveRegion carrier clips the overlap to an authored subrange.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicChordRationalOverlap2 {
    chord: BezierAlgebraicChord2,
    source: RationalBezier2,
    chord_range: [BezierAlgebraicChordParameter2; 2],
    source_range: CurveParameterRange2,
    orientation: CurveOverlapOrientation2,
}

#[derive(Clone, Debug)]
pub(crate) enum BezierAlgebraicChordRationalIntersections2 {
    Contacts(Vec<BezierAlgebraicChordRationalContact2>),
    Overlaps(Vec<BezierAlgebraicChordRationalOverlap2>),
    ContactsAndOverlaps {
        contacts: Vec<BezierAlgebraicChordRationalContact2>,
        overlaps: Vec<BezierAlgebraicChordRationalOverlap2>,
    },
    DegenerateProjection,
    NotSourceRelated,
}

#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicChordParallelContact2 {
    chord_parameter: BezierAlgebraicChordParameter2,
    parallel_parameter: CurveParameter2,
    point: CurvePoint2,
    tangent_cross_sign: RealSign,
    tangent_dot_sign: RealSign,
}

/// A unique chord/parallel contact selected by a strict monotonicity
/// certificate without projecting its recursive coefficient tower.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicChordRetainedParallelContact2 {
    chord_parameter: BezierAlgebraicChordParameter2,
    parallel_parameter: CurveParameter2,
    point: CurvePoint2,
    tangent_cross_sign: RealSign,
}

#[derive(Clone, Debug)]
pub(crate) enum BezierAlgebraicChordParallelIntersections2 {
    Contacts(Vec<BezierAlgebraicChordParallelContact2>),
    /// The analytic carrier follows the chord's complete supporting line on
    /// the selected regular parameter cell. Retain the exact sheet-selection
    /// sample so consumers can recover its affine support without rebuilding
    /// the chord's independent endpoint fields.
    CoincidentSupportComponent {
        sample: Real,
    },
    DegenerateProjection,
}

#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicChordPairContact2 {
    first_parameter: BezierAlgebraicChordParameter2,
    second_parameter: BezierAlgebraicChordParameter2,
    point: CurvePoint2,
    tangent_cross_sign: RealSign,
}

/// Exact affine point defined by the unique nonparallel intersection of two
/// retained algebraic chord supports.
///
/// The point keeps the four endpoint fields separate.  Exact predicates use
/// the certified side relations and refine only the coordinate boxes needed
/// by a consumer; no primitive-element tower or rounded coordinate is stored.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicChordPairPoint2 {
    data: Arc<BezierAlgebraicChordPairPointData2>,
}

#[derive(Debug)]
struct BezierAlgebraicChordPairPointData2 {
    first: BezierAlgebraicChord2,
    second: BezierAlgebraicChord2,
    location: BezierAlgebraicChordPairPointLocation2,
    policy: CurveContext,
    /// Lazily retained compact homogeneous lines for the two defining
    /// supports. The `Arc` keeps the always-present pair allocation to one
    /// additional word; the six recursive values are allocated only if a
    /// cold three-support predicate actually needs them.
    recursive_support_lines: OnceLock<Arc<[BezierRecursiveQuadraticProjectivePoint2; 2]>>,
    recursive_point: OnceLock<BezierRecursiveQuadraticProjectivePoint2>,
}

#[derive(Clone, Copy, Debug)]
enum BezierAlgebraicChordPairPointLocation2 {
    EndpointSides {
        first: [crate::classify::LineSide; 2],
        second: [crate::classify::LineSide; 2],
        tangent_cross_sign: RealSign,
    },
    /// Exact offset construction already knows the intersection order from
    /// each adjacent endpoint: for nonparallel unit directions `u,v`, equal
    /// signed left offsets meet at parameters whose signs are respectively
    /// `-sign(d * cross(u,v))` and `sign(d * cross(u,v))`.
    AnchorOrders {
        first_at_end: bool,
        first: std::cmp::Ordering,
        second_at_end: bool,
        second: std::cmp::Ordering,
        tangent_cross_sign: RealSign,
    },
}

impl PartialEq for BezierAlgebraicChordPairPoint2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.data.policy == other.data.policy
                && ((self.data.first == other.data.first && self.data.second == other.data.second)
                    || (self.data.first == other.data.second
                        && self.data.second == other.data.first)))
    }
}

/// Paired overlap boundaries in the first chord's traversal order. The
/// second range descends when the chords have opposite orientations.
#[derive(Clone, Debug)]
pub(crate) struct BezierAlgebraicChordPairOverlap2 {
    first_range: [BezierAlgebraicChordParameter2; 2],
    second_range: [BezierAlgebraicChordParameter2; 2],
    orientation: CurveOverlapOrientation2,
}

#[derive(Clone, Debug)]
pub(crate) enum BezierAlgebraicChordPairIntersections2 {
    Contacts(Vec<BezierAlgebraicChordPairContact2>),
    Overlaps(Vec<BezierAlgebraicChordPairOverlap2>),
}

/// Exact outcome of the shared four-endpoint side kernel.
///
/// A strict same-side certificate for either finite chord already proves
/// disjointness. Preserve that result without manufacturing the two unused
/// endpoint sides merely to satisfy the complete-arrangement representation.
enum BezierAlgebraicChordPairSides2 {
    Disjoint,
    Complete(
        [crate::classify::LineSide; 2],
        [crate::classify::LineSide; 2],
    ),
}

#[derive(Debug)]
struct BezierAlgebraicChordRationalBoundary2 {
    chord_parameter: BezierAlgebraicChordParameter2,
    source_parameter: CurveParameter2,
    point: CurvePoint2,
}

/// One source-domain boundary in a collinear noninjective partition.
///
/// A source parameter projected from a selected chord endpoint retains which
/// endpoint supplied it.  Reusing that selected-root evidence avoids asking a
/// later independent point comparison to rediscover an equality that the
/// fiber projection has already proved.
#[derive(Debug)]
struct BezierAlgebraicChordRationalPartitionBoundary2 {
    source_parameter: CurveParameter2,
    chord_endpoint_at_end: Option<bool>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct BezierAlgebraicChordParameterAxis2 {
    axis: Axis2,
    coordinate_increases: bool,
}

#[derive(Debug)]
struct BezierAlgebraicChordParameterData2 {
    chord: BezierAlgebraicChord2,
    point: CurvePoint2,
    axis: BezierAlgebraicChordParameterAxis2,
    /// The constructing kernel proved both strict finite-domain inequalities.
    /// Support-only parameters deliberately leave this false: the represented
    /// point can then lie on either exterior ray even though it uses the same
    /// monotone affine coordinate. This bit occupies existing alignment
    /// padding and lets topology consumers retain a proof without rebuilding
    /// an unrelated Cartesian comparison.
    certified_strict_interior: bool,
}

#[derive(Debug)]
struct BezierAlgebraicChordSourceIncidence2 {
    incidence: BivariatePolynomial,
    line_x: Vec<Real>,
    line_y: Vec<Real>,
    chord_denominator_sign: RealSign,
}

/// Target-independent two-field tangent support for one retained chord.
///
/// The first and second tensor axes retain the independently selected chord
/// endpoints. Consumers add only the vector or analytic-parameter axis needed
/// to sign an oriented tangent relation.
#[derive(Debug)]
struct BezierAlgebraicChordIndependentSupport2 {
    line_x: BivariatePolynomial,
    line_y: BivariatePolynomial,
    first_parameter: BezierParameter2,
    second_parameter: BezierParameter2,
    chord_denominator_sign: RealSign,
}

/// Prepared independent-field predicate data for one retained chord.
#[derive(Debug)]
pub(crate) struct BezierAlgebraicChordAlgebraicRay2 {
    start: RationalBezierAlgebraicPointImage2,
    end: RationalBezierAlgebraicPointImage2,
}

/// Exact algebraic-query ray evaluator for one retained analytic parallel.
///
/// The source parameter range remains in its native selected/Bezier domain.
/// Line incidence uses a squared projection only to enumerate candidates; all
/// topology decisions replay the authored `A*sqrt(S)+B` normal branch.
#[derive(Debug)]
pub(crate) struct BezierParallelAlgebraicRay2 {
    parallel: BezierParallel2,
    range: CurveParameterRange2,
    reversed: bool,
    endpoints: [CurvePoint2; 2],
}

struct BezierParallelAlgebraicIncidenceSystem2 {
    incidence: BivariatePolynomial,
    expression: BezierAlgebraicCuspTwoTermExpression2,
    speed_squared: BivariatePolynomial,
}

/// Prepared supporting-line predicate for one retained chord.
///
/// Native endpoints stay on the two-coordinate `LineSeg2` path. Mixed and
/// algebraic pairs normalize only the queried point into the selected field
/// already retained by the algebraic support, avoiding an artificial field
/// (and an unsupported exact/exact normalization) for represented chords.
#[derive(Debug)]
enum BezierAlgebraicChordSupportPredicate2 {
    Exact {
        chord: BezierAlgebraicChord2,
        line: LineSeg2,
    },
    Algebraic {
        chord: BezierAlgebraicChord2,
        ray: BezierAlgebraicChordAlgebraicRay2,
    },
    CertifiedTangent {
        chord: BezierAlgebraicChord2,
    },
    /// General exact fallback for endpoint carriers that deliberately do not
    /// flatten into one algebraic field. Strict interval refinement decides
    /// every nonzero side; equality remains structural or policy-terminal.
    RefinedEndpoint {
        chord: BezierAlgebraicChord2,
    },
}

impl PartialEq for BezierAlgebraicChord2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.data.start == other.data.start
                && self.data.end == other.data.end
                && self.data.reversed == other.data.reversed
                && (self.data.policy == other.data.policy
                    || matches!(
                        (&self.data.start, &self.data.end),
                        (
                            CurvePoint2(CurvePointData2::Exact(_)),
                            CurvePoint2(CurvePointData2::Exact(_))
                        )
                    )))
    }
}

impl PartialEq for BezierAlgebraicChordParallelPoint2 {
    fn eq(&self, other: &Self) -> bool {
        self.at_end == other.at_end
            && (Arc::ptr_eq(&self.data, &other.data)
                || (self.data.source == other.data.source
                    && self.data.source_point == other.data.source_point
                    && self.data.distance == other.data.distance
                    && self.data.translation_x == other.data.translation_x
                    && self.data.translation_y == other.data.translation_y
                    && self.data.direction == other.data.direction
                    && self.data.policy == other.data.policy))
    }
}

impl PartialEq for BezierAnalyticParallelPoint2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.data.parallel == other.data.parallel
                && self.data.parameter == other.data.parameter
                && self.data.frame_tangent == other.data.frame_tangent
                && self.data.tangent_distance == other.data.tangent_distance
                && self.data.translation_x == other.data.translation_x
                && self.data.translation_y == other.data.translation_y
                && self.data.policy == other.data.policy)
    }
}

/// Recovers a reusable unit direction when independently selected endpoint
/// fields retain the same exact affine support line.
///
/// Each endpoint certificate is coefficientwise and STRICT-only.  Exact
/// proportionality of the two line equations then proves the chord direction
/// without adjoining the selected fields merely to normalize their delta.
fn strict_common_retained_line_coefficients(
    start: &CurvePoint2,
    end: &CurvePoint2,
) -> Option<[Real; 3]> {
    let first = start
        .as_algebraic()?
        .strict_retained_affine_line_coefficients()?;
    let second = end
        .as_algebraic()?
        .strict_retained_affine_line_coefficients()?;
    let strict = &CurveContext::STRICT;
    for residual in [
        Real::diff_of_products(&first[0], &second[1], &first[1], &second[0]),
        Real::diff_of_products(&first[0], &second[2], &first[2], &second[0]),
        Real::diff_of_products(&first[1], &second[2], &first[2], &second[1]),
    ] {
        if real_sign(&residual, strict) != Some(RealSign::Zero) {
            return None;
        }
    }
    Some(first)
}

fn strict_common_retained_line_unit_tangent(
    start: &CurvePoint2,
    end: &CurvePoint2,
    parameter_axis: BezierAlgebraicChordParameterAxis2,
    policy: &CurveContext,
) -> Option<(Real, Real)> {
    if let (
        CurvePoint2(CurvePointData2::AlgebraicChordParallel(start)),
        CurvePoint2(CurvePointData2::AlgebraicChordParallel(end)),
    ) = (start, end)
        && start.data.source_point.is_none()
        && end.data.source_point.is_none()
        && start.at_end == end.at_end
        && start.data.direction == end.data.direction
        && start.data.source == end.data.source
        && compare_reals(
            &start.data.translation_x,
            &end.data.translation_x,
            &CurveContext::STRICT,
        ) == Some(std::cmp::Ordering::Equal)
        && compare_reals(
            &start.data.translation_y,
            &end.data.translation_y,
            &CurveContext::STRICT,
        ) == Some(std::cmp::Ordering::Equal)
    {
        let distance_order = compare_reals(
            &start.data.distance,
            &end.data.distance,
            &CurveContext::STRICT,
        )?;
        if distance_order != std::cmp::Ordering::Equal {
            let (tangent_x, tangent_y) = start.data.source.certified_unit_tangent()?;
            let (mut direction_x, mut direction_y) = match start.data.direction {
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => (-tangent_y, tangent_x),
                BezierAlgebraicChordUnitDisplacement2::Tangent => (tangent_x, tangent_y),
            };
            if distance_order == std::cmp::Ordering::Greater {
                direction_x = -direction_x;
                direction_y = -direction_y;
            }
            let selected = match parameter_axis.axis {
                Axis2::X => &direction_x,
                Axis2::Y => &direction_y,
            };
            let selected_increases = match real_sign(selected, &CurveContext::STRICT)? {
                RealSign::Positive => true,
                RealSign::Negative => false,
                RealSign::Zero => return None,
            };
            if selected_increases == parameter_axis.coordinate_increases {
                return Some((direction_x, direction_y));
            }
        }
    }
    let exact_coordinate = |point: &CurvePoint2, axis: Axis2| match point {
        CurvePoint2(CurvePointData2::Exact(point)) => Some(match axis {
            Axis2::X => point.x().clone(),
            Axis2::Y => point.y().clone(),
        }),
        CurvePoint2(CurvePointData2::Algebraic(point)) => {
            point.exact_coordinate(axis == Axis2::X, policy)
        }
        CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
        | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
        | CurvePoint2(CurvePointData2::AnalyticParallel(_))
        | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => None,
    };
    let constant_axis = match parameter_axis.axis {
        Axis2::X => Axis2::Y,
        Axis2::Y => Axis2::X,
    };
    if let (Some(first), Some(second)) = (
        exact_coordinate(start, constant_axis),
        exact_coordinate(end, constant_axis),
    ) && compare_reals(&first, &second, &CurveContext::STRICT) == Some(std::cmp::Ordering::Equal)
    {
        let direction = Real::from(if parameter_axis.coordinate_increases {
            1_i8
        } else {
            -1_i8
        });
        return Some(match parameter_axis.axis {
            Axis2::X => (direction, Real::zero()),
            Axis2::Y => (Real::zero(), direction),
        });
    }

    if let (CurvePoint2(CurvePointData2::Exact(start)), CurvePoint2(CurvePointData2::Exact(end))) =
        (start, end)
    {
        let (direction_x, direction_y) = end.delta_from(start);
        let norm = Real::dot2_refs([&direction_x, &direction_y], [&direction_x, &direction_y])
            .sqrt()
            .ok()?;
        return Some(((&direction_x / &norm).ok()?, (&direction_y / norm).ok()?));
    }

    let first = strict_common_retained_line_coefficients(start, end)?;
    let strict = &CurveContext::STRICT;

    // `(b, -a)` is tangent to `a*x + b*y + c = 0`.
    let mut direction_x = first[1].clone();
    let mut direction_y = -first[0].clone();
    let selected_component = match parameter_axis.axis {
        Axis2::X => &direction_x,
        Axis2::Y => &direction_y,
    };
    let selected_increases = match real_sign(selected_component, strict)? {
        RealSign::Positive => true,
        RealSign::Negative => false,
        RealSign::Zero => return None,
    };
    if selected_increases != parameter_axis.coordinate_increases {
        direction_x = -direction_x;
        direction_y = -direction_y;
    }
    let norm = Real::dot2_refs([&direction_x, &direction_y], [&direction_x, &direction_y])
        .sqrt()
        .ok()?;
    Some(((&direction_x / &norm).ok()?, (&direction_y / norm).ok()?))
}

#[derive(Clone, Debug)]
struct BezierAlgebraicChordData2 {
    start: CurvePoint2,
    end: CurvePoint2,
    parameter_axis: BezierAlgebraicChordParameterAxis2,
    /// Proof that the other coordinate is constant, so `parameter_axis`
    /// determines the complete cardinal traversal direction.
    certified_axis_aligned: bool,
    /// Exact unit traversal tangent retained when a certified chord is mapped
    /// away from a cardinal direction. Axis-aligned chords derive the same
    /// evidence without this allocation.
    certified_unit_tangent: Option<Arc<[Real; 2]>>,
    /// Bit 0/1 proves that the traversal start/end direction is transverse to
    /// the retained circle tangent carried by that endpoint. The two bits fit
    /// existing alignment padding and avoid replaying a multi-field predicate
    /// when a later parallel uses the fact only to make Cauchy strict.
    certified_circle_transverse_endpoints: u8,
    /// Authored exact endpoint tangencies to analytic parallels. Straight
    /// offset joins use these to deflate known double roots while retaining
    /// the complete residual supporting-line solve.
    parallel_tangent_contacts: Option<Arc<[crate::bezier::BezierParallelLineTangentContact2]>>,
    source: Option<BezierAlgebraicChord2>,
    reversed: bool,
    policy: CurveContext,
}

impl PartialEq for BezierAlgebraicChordParameter2 {
    fn eq(&self, other: &Self) -> bool {
        match (&self.data, &other.data) {
            (
                BezierAlgebraicChordParameterStorage2::Endpoint {
                    chord: first,
                    at_end: first_at_end,
                },
                BezierAlgebraicChordParameterStorage2::Endpoint {
                    chord: second,
                    at_end: second_at_end,
                },
            ) if Arc::ptr_eq(&first.data, &second.data) => first_at_end == second_at_end,
            (
                BezierAlgebraicChordParameterStorage2::Interior(first),
                BezierAlgebraicChordParameterStorage2::Interior(second),
            ) if Arc::ptr_eq(first, second) => true,
            _ => self.axis() == other.axis() && self.point() == other.point(),
        }
    }
}

impl PartialEq for BezierAlgebraicCuspSemicircleFragment2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.data.semicircle == other.data.semicircle
                && self.data.start.shares_exact_evidence(&other.data.start)
                && self.data.end.shares_exact_evidence(&other.data.end)
                && self.data.reversed == other.data.reversed
                && self.data.policy == other.data.policy)
    }
}

#[derive(Clone, Debug)]
struct BezierAlgebraicCuspSemicircleFragmentData2 {
    semicircle: BezierAlgebraicCuspSemicircle2,
    start: BezierAlgebraicCuspSemicircleParameter2,
    end: BezierAlgebraicCuspSemicircleParameter2,
    /// Exact endpoint images are constructed at most once and shared by every
    /// topology, validation, and adjacency consumer of this fragment.
    start_point_image: OnceLock<Option<RationalBezierAlgebraicPointImage2>>,
    end_point_image: OnceLock<Option<RationalBezierAlgebraicPointImage2>>,
    /// Source-parameter endpoint bits whose authored boundary neighbor is
    /// certified tangent. Bit zero names `start`; bit one names `end`.
    certified_tangent_endpoints: u8,
    reversed: bool,
    policy: CurveContext,
}

/// One selected-circle endpoint tangent reduced to the pair branch and
/// participating support that authored its direction. `orientation` is the
/// fragment traversal tangent relative to that support's intrinsic tangent.
struct BezierRetainedPairTangentProvenance2<'a> {
    map: &'a BezierAlgebraicCuspSemicirclePairParameterMap2,
    contact: &'a BezierAlgebraicCuspSemicirclePairContact2,
    first: bool,
    orientation: RealSign,
}

fn cloned_once_lock<T: Clone>(source: &OnceLock<T>) -> OnceLock<T> {
    let cloned = OnceLock::new();
    if let Some(value) = source.get() {
        let _ = cloned.set(value.clone());
    }
    cloned
}

/// Cached local-field geometry for classifying an algebraic ray against one
/// cusp subarc. Endpoints normally share the cusp field; a nonrational mapped
/// cut can instead lend its carrier field to an exactly rational companion.
/// Distinct endpoint fields stay separate and use the cold three-field chord
/// predicate, so no path constructs a primitive element.
#[derive(Debug)]
pub(crate) struct BezierAlgebraicCuspSemicircleAlgebraicRay2 {
    start: RationalBezierAlgebraicPointImage2,
    end: RationalBezierAlgebraicPointImage2,
    center: RationalBezierAlgebraicPointImage2,
    radius_squared: Real,
    clockwise: bool,
}

#[derive(Debug)]
struct BezierAlgebraicCuspSemicirclePairOverlapData2 {
    parameter_map: BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2,
    first_boundaries: [BezierAlgebraicCuspSemicirclePairEndpoint2; 2],
    second_boundaries: [BezierAlgebraicCuspSemicirclePairEndpoint2; 2],
    orientation: CurveOverlapOrientation2,
    policy: CurveContext,
}

#[derive(Debug)]
enum BezierAlgebraicCuspSemicirclePairOverlapParameterMapData2 {
    ExactEndpoints {
        first_semicircle: BezierAlgebraicCuspSemicircle2,
        second_semicircle: BezierAlgebraicCuspSemicircle2,
    },
    /// Rank-independent coincident support. Dot and cross are exact selected
    /// images of the two parameter-zero radials; the exact squared radii
    /// complete every angular correspondence predicate without coordinates.
    Represented {
        first_semicircle: BezierAlgebraicCuspSemicircle2,
        second_semicircle: BezierAlgebraicCuspSemicircle2,
        radial_dot: AlgebraicRootRepresentation,
        radial_cross: AlgebraicRootRepresentation,
        first_radius_squared: Real,
        second_radius_squared: Real,
        first_clockwise: bool,
    },
    /// Similarity-transformed carriers sharing the source overlap's invariant
    /// parameter correspondence. Uniform similarities leave both local
    /// parameters unchanged; retaining the source proof avoids cloning its
    /// correlated polynomial system for every transformed boundary cut.
    SimilarityTransport {
        first_semicircle: BezierAlgebraicCuspSemicircle2,
        second_semicircle: BezierAlgebraicCuspSemicircle2,
        source: BezierAlgebraicCuspSemicirclePairOverlap2,
    },
}

/// Operation-local sharing for similarity-transformed selected circles and
/// their mapped cuts. Arrangement partitions commonly clone one carrier and
/// one cut across adjacent fragments; retaining that identity avoids repeating
/// exact polynomial-frame transforms and proof-wrapper allocations.
#[derive(Default)]
pub(crate) struct BezierAlgebraicCuspSemicircleSimilarityCache2 {
    frames: Vec<(BezierSelectedCircleFrame2, BezierSelectedCircleFrame2)>,
    semicircles: Vec<(
        BezierAlgebraicCuspSemicircle2,
        BezierAlgebraicCuspSemicircle2,
    )>,
    overlaps: Vec<(
        BezierAlgebraicCuspSemicirclePairOverlap2,
        BezierAlgebraicCuspSemicirclePairOverlap2,
    )>,
    parameters: Vec<(
        Arc<BezierAlgebraicCuspSemicircleMappedParameterData2>,
        BezierAlgebraicCuspSemicircle2,
        BezierAlgebraicCuspSemicircleParameter2,
    )>,
    chords: Vec<(BezierAlgebraicChord2, BezierAlgebraicChord2)>,
}

#[derive(Clone, Debug)]
pub(crate) enum BezierAlgebraicCuspSemicirclePairIntersections2 {
    NoContacts,
    Contacts {
        contacts: Vec<BezierAlgebraicCuspSemicirclePairContact2>,
        parameter_map: BezierAlgebraicCuspSemicirclePairParameterMap2,
    },
    EndpointContacts(Vec<BezierAlgebraicCuspSemicirclePairContact2>),
    Overlap(BezierAlgebraicCuspSemicirclePairOverlap2),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BezierRecursiveCirclePairSupportRelation2 {
    Concentric,
    Discriminant(RealSign),
}

impl BezierSelectedCircleFrame2 {
    fn evidence_policy(&self) -> Option<CurveContext> {
        match self {
            Self::Rational(_) => None,
            Self::ParallelNormal(frame) => Some(frame.policy),
            Self::ChordNormal(frame) => Some(frame.policy),
            Self::SelectedRadial(frame) => Some(frame.policy),
        }
    }

    fn shares_storage(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Rational(first), Self::Rational(second)) => {
                Arc::ptr_eq(&first.data, &second.data)
            }
            (Self::ParallelNormal(first), Self::ParallelNormal(second)) => {
                Arc::ptr_eq(first, second)
            }
            (Self::ChordNormal(first), Self::ChordNormal(second)) => Arc::ptr_eq(first, second),
            (Self::SelectedRadial(first), Self::SelectedRadial(second)) => {
                Arc::ptr_eq(first, second)
            }
            (
                Self::Rational(_),
                Self::ParallelNormal(_) | Self::ChordNormal(_) | Self::SelectedRadial(_),
            )
            | (
                Self::ParallelNormal(_),
                Self::Rational(_) | Self::ChordNormal(_) | Self::SelectedRadial(_),
            )
            | (
                Self::ChordNormal(_),
                Self::Rational(_) | Self::ParallelNormal(_) | Self::SelectedRadial(_),
            )
            | (
                Self::SelectedRadial(_),
                Self::Rational(_) | Self::ParallelNormal(_) | Self::ChordNormal(_),
            ) => false,
        }
    }

    fn rational(&self) -> Option<&BezierParallelAlgebraicCuspFrame2> {
        match self {
            Self::Rational(frame) => Some(frame),
            Self::ParallelNormal(_) | Self::ChordNormal(_) | Self::SelectedRadial(_) => None,
        }
    }

    fn parallel_normal(&self) -> Option<&BezierSelectedParallelNormalFrameData2> {
        match self {
            Self::Rational(_) | Self::ChordNormal(_) | Self::SelectedRadial(_) => None,
            Self::ParallelNormal(frame) => Some(frame),
        }
    }

    fn chord_normal(&self) -> Option<&BezierSelectedChordNormalFrameData2> {
        match self {
            Self::ChordNormal(frame) => Some(frame),
            Self::Rational(_) | Self::ParallelNormal(_) | Self::SelectedRadial(_) => None,
        }
    }

    fn selected_radial(&self) -> Option<&BezierSelectedRadialFrameData2> {
        match self {
            Self::SelectedRadial(frame) => Some(frame),
            Self::Rational(_) | Self::ParallelNormal(_) | Self::ChordNormal(_) => None,
        }
    }

    fn rational_required(&self) -> CurveResult<&BezierParallelAlgebraicCuspFrame2> {
        self.rational().ok_or_else(|| {
            CurveError::Topology(
                "a general selected-circle frame entered a rational-frame-only operation".into(),
            )
        })
    }

    fn center_parallel_distance(&self) -> Real {
        match self {
            Self::Rational(frame) => frame.center_parallel_distance(),
            Self::ParallelNormal(frame) => frame.center_support.distance().clone(),
            Self::ChordNormal(_) | Self::SelectedRadial(_) => Real::zero(),
        }
    }

    fn source_parallel(&self) -> Option<&BezierParallel2> {
        match self {
            Self::Rational(frame) => frame.data.parallel.as_ref(),
            Self::ParallelNormal(frame) => Some(&frame.center_support),
            Self::ChordNormal(_) | Self::SelectedRadial(_) => None,
        }
    }

    fn transform_similarity(&self, transform: &Similarity2) -> CurveResult<Self> {
        match self {
            Self::Rational(frame) => Ok(Self::Rational(frame.transform_similarity(transform)?)),
            Self::ParallelNormal(frame) => Ok(Self::ParallelNormal(Arc::new(
                BezierSelectedParallelNormalFrameData2 {
                    center_support: frame.center_support.transform_similarity(transform)?,
                    center_parameter: frame.center_parameter.clone(),
                    policy: frame.policy,
                },
            ))),
            Self::ChordNormal(_) | Self::SelectedRadial(_) => Err(CurveError::Topology(
                "a correlated selected circle requires cached similarity transport".into(),
            )),
        }
    }

    fn point_numerators_at_parallel_distance(
        &self,
        distance: &Real,
    ) -> CurveResult<(Vec<Real>, Vec<Real>)> {
        Ok(self
            .rational_required()?
            .point_numerators_at_parallel_distance(distance))
    }

    fn point_image_at_parallel_distance(
        &self,
        distance: &Real,
        policy: &CurveContext,
    ) -> CurveResult<RationalBezierAlgebraicPointImage2> {
        self.rational_required()?
            .point_image_at_parallel_distance(distance, policy)
    }

    fn certified_cardinal_normal(&self) -> CurveResult<Option<(i8, i8)>> {
        match self.rational() {
            Some(frame) => frame.certified_cardinal_normal(),
            None => Ok(None),
        }
    }

    fn represented_unit_normal(&self) -> CurveResult<Option<(Real, Real)>> {
        match self.rational() {
            Some(frame) => frame.represented_unit_normal(),
            None => Ok(None),
        }
    }
}

fn algebraic_cusp_semicircle_endpoint_parameter(
    location: BezierAlgebraicCuspSemicircleContactLocation2,
) -> Option<BezierAlgebraicCuspSemicircleParameter2> {
    match location {
        BezierAlgebraicCuspSemicircleContactLocation2::Start => {
            Some(BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()))
        }
        BezierAlgebraicCuspSemicircleContactLocation2::End => {
            Some(BezierAlgebraicCuspSemicircleParameter2::Exact(Real::one()))
        }
        BezierAlgebraicCuspSemicircleContactLocation2::Interior => None,
    }
}

fn algebraic_cusp_semicircle_endpoint_contact_order(
    location: BezierAlgebraicCuspSemicircleContactLocation2,
    parameter: &Real,
    policy: &CurveContext,
) -> Option<Classification<std::cmp::Ordering>> {
    let endpoint = match location {
        BezierAlgebraicCuspSemicircleContactLocation2::Start => Real::zero(),
        BezierAlgebraicCuspSemicircleContactLocation2::End => Real::one(),
        BezierAlgebraicCuspSemicircleContactLocation2::Interior => {
            if parameter == &Real::zero() {
                return Some(Classification::Decided(std::cmp::Ordering::Greater));
            }
            if parameter == &Real::one() {
                return Some(Classification::Decided(std::cmp::Ordering::Less));
            }
            return None;
        }
    };
    Some(
        compare_reals(&endpoint, parameter, policy)
            .map(Classification::Decided)
            .unwrap_or(Classification::Uncertain(UncertaintyReason::Ordering)),
    )
}

fn refine_algebraic_cusp_semicircle_parameter_bracket(
    bracket: Option<&BezierAlgebraicCuspSemicircleParameterBracket2>,
    refinement_steps: usize,
    mut contact_order_to_real: impl FnMut(&Real) -> CurveResult<Classification<std::cmp::Ordering>>,
) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParameterBracket2>> {
    let (mut start, mut end) = match bracket {
        Some(exact @ BezierAlgebraicCuspSemicircleParameterBracket2::Exact(_)) => {
            return Ok(Classification::Decided(exact.clone()));
        }
        Some(BezierAlgebraicCuspSemicircleParameterBracket2::Interval(bounds)) => {
            (bounds.start().clone(), bounds.end().clone())
        }
        None => (Real::zero(), Real::one()),
    };
    for _ in 0..refinement_steps {
        let midpoint = ((&start + &end) / Real::from(2_i8))?;
        match contact_order_to_real(&midpoint)? {
            Classification::Decided(std::cmp::Ordering::Less) => end = midpoint,
            Classification::Decided(std::cmp::Ordering::Greater) => start = midpoint,
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicCuspSemicircleParameterBracket2::Exact(midpoint),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(
        match BezierParameterInterval::try_new(start, end, &CurveContext::STRICT)? {
            Classification::Decided(interval) => Classification::Decided(
                BezierAlgebraicCuspSemicircleParameterBracket2::Interval(interval),
            ),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

impl PartialEq for BezierAlgebraicCuspChordPoint2 {
    fn eq(&self, other: &Self) -> bool {
        if self.shares_storage(other) {
            return true;
        }
        let (first_map, first_contact) = self.map_contact();
        let (second_map, second_contact) = other.map_contact();
        Arc::ptr_eq(&first_map.data, &second_map.data) && first_contact == second_contact
    }
}

impl PartialEq for BezierAlgebraicCuspChordDerivedPoint2 {
    fn eq(&self, other: &Self) -> bool {
        self.shares_storage(other)
            || (self.data.source == other.data.source
                && self.data.radial_scale == other.data.radial_scale
                && self.data.perpendicular_scale == other.data.perpendicular_scale
                && self.data.translation_x == other.data.translation_x
                && self.data.translation_y == other.data.translation_y)
    }
}

fn algebraic_constant_point_image(
    point: &Point2,
    parameter: &BezierAlgebraicParameter2,
    policy: &CurveContext,
) -> RationalBezierAlgebraicPointImage2 {
    RationalBezierAlgebraicPointImage2::from_retained_expression(
        parameter.clone(),
        parameter_representation(parameter, policy),
        vec![point.x().clone()],
        vec![point.y().clone()],
        vec![Real::one()],
        "embedded an exact cusp endpoint in a retained local field",
    )
}

fn cusp_semicircle_parameter_bracket_bounds(
    bracket: &BezierAlgebraicCuspSemicircleParameterBracket2,
) -> (&Real, &Real) {
    match bracket {
        BezierAlgebraicCuspSemicircleParameterBracket2::Exact(parameter) => (parameter, parameter),
        BezierAlgebraicCuspSemicircleParameterBracket2::Interval(interval) => {
            (interval.start(), interval.end())
        }
    }
}

fn cusp_chamfer_parameter_denominator(source: &Real, half_angle: &Real) -> Real {
    Real::one() + half_angle * (Real::one() - Real::from(2_i8) * source)
}

fn cusp_chamfer_parameter_value(
    source: &Real,
    half_angle: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Real>> {
    let denominator = cusp_chamfer_parameter_denominator(source, half_angle);
    match real_sign(&denominator, policy) {
        Some(RealSign::Positive | RealSign::Negative) => {}
        Some(RealSign::Zero) => return Err(CurveError::InvalidBezierRange),
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let numerator = source + half_angle * (Real::one() - source);
    Ok(Classification::Decided((numerator / denominator)?))
}

/// Chooses the finite half-circle chart for one physical angular transport.
///
/// The source and target chart coordinates are related projectively. A pole
/// in the source-half affine coordinate is an ordinary finite point on the
/// complementary chart, so chart selection must sign the retained Mobius
/// denominator before asking the ordinary parameter-order predicate.
fn cusp_chamfer_parameter_uses_complement(
    source: &BezierAlgebraicCuspSemicircleParameter2,
    physical_half_angle: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let denominator_coefficient = Real::from(-2_i8) * physical_half_angle;
    let denominator_constant = Real::one() + physical_half_angle;
    let denominator = match cusp_parameter_affine_expression_sign(
        source,
        &denominator_coefficient,
        &denominator_constant,
        policy,
    )? {
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Decided(true));
        }
        Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let numerator = match cusp_parameter_affine_expression_sign(
        source,
        &(Real::one() - physical_half_angle),
        physical_half_angle,
        policy,
    )? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let denominator_minus_numerator = match cusp_parameter_affine_expression_sign(
        source,
        &(-(Real::one() + physical_half_angle)),
        &Real::one(),
        policy,
    )? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let agrees_with_denominator = |sign| sign == RealSign::Zero || sign == denominator;
    Ok(Classification::Decided(
        !agrees_with_denominator(numerator)
            || !agrees_with_denominator(denominator_minus_numerator),
    ))
}

fn cusp_chamfer_parameter_bracket(
    source: &BezierAlgebraicCuspSemicircleParameter2,
    half_angle: &Real,
    refinement_steps: usize,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParameterBracket2>> {
    let source = match source.parameter_bracket(refinement_steps, policy)? {
        Classification::Decided(source) => source,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let (start, end) = cusp_semicircle_parameter_bracket_bounds(&source);
    let strict = &CurveContext::STRICT;
    let start_denominator = cusp_chamfer_parameter_denominator(start, half_angle);
    let end_denominator = cusp_chamfer_parameter_denominator(end, half_angle);
    let (Some(start_sign), Some(end_sign)) = (
        real_sign(&start_denominator, strict),
        real_sign(&end_denominator, strict),
    ) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
    };
    if start_sign == RealSign::Zero || start_sign != end_sign {
        return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
    }
    let start = match cusp_chamfer_parameter_value(start, half_angle, strict)? {
        Classification::Decided(start) => start,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let end = match cusp_chamfer_parameter_value(end, half_angle, strict)? {
        Classification::Decided(end) => end,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    // A coarse source bracket may extend past the subset on which this
    // authored angular transport remains in the half-circle chart.  The
    // retained target itself was certified strictly inside [0, 1] when the
    // cut was created, so intersecting its monotone image with that chart is
    // an exact enclosure operation.  This matters for nested chamfers: at
    // refinement zero the outer transport commonly maps source endpoint 1
    // past 1 even though the selected source root and its image are interior.
    let zero = Real::zero();
    let one = Real::one();
    let start = match compare_reals(&start, &zero, strict) {
        Some(std::cmp::Ordering::Less) => zero,
        Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater) => start,
        None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    };
    let end = match compare_reals(&end, &one, strict) {
        Some(std::cmp::Ordering::Greater) => one,
        Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Less) => end,
        None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    };
    if start == end {
        return Ok(Classification::Decided(
            BezierAlgebraicCuspSemicircleParameterBracket2::Exact(start),
        ));
    }
    Ok(
        match BezierParameterInterval::try_new(start, end, policy)? {
            Classification::Decided(interval) => Classification::Decided(
                BezierAlgebraicCuspSemicircleParameterBracket2::Interval(interval),
            ),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

fn cusp_parameter_affine_expression_sign(
    parameter: &BezierAlgebraicCuspSemicircleParameter2,
    coefficient: &Real,
    constant: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let coefficient_sign = match real_sign(coefficient, policy) {
        Some(sign) => sign,
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    };
    if coefficient_sign == RealSign::Zero {
        return Ok(real_sign(constant, policy).map_or(
            Classification::Uncertain(UncertaintyReason::RealSign),
            Classification::Decided,
        ));
    }
    let root = ((-constant.clone()) / coefficient.clone())?;
    let parameter_order =
        if compare_reals(&root, &Real::zero(), policy) == Some(std::cmp::Ordering::Less) {
            Classification::Decided(std::cmp::Ordering::Greater)
        } else if compare_reals(&root, &Real::one(), policy) == Some(std::cmp::Ordering::Greater) {
            Classification::Decided(std::cmp::Ordering::Less)
        } else {
            parameter.order_to_real(&root, policy)?
        };
    Ok(parameter_order.map(|order| {
        let sign = match order {
            std::cmp::Ordering::Less => RealSign::Negative,
            std::cmp::Ordering::Equal => RealSign::Zero,
            std::cmp::Ordering::Greater => RealSign::Positive,
        };
        if coefficient_sign == RealSign::Negative {
            match sign {
                RealSign::Negative => RealSign::Positive,
                RealSign::Zero => RealSign::Zero,
                RealSign::Positive => RealSign::Negative,
            }
        } else {
            sign
        }
    }))
}

fn cusp_chamfer_parameter_order_to_real(
    source: &BezierAlgebraicCuspSemicircleParameter2,
    half_angle: &Real,
    target: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<std::cmp::Ordering>> {
    // f(v)-t = (a v+b)/(1+q-2qv), where f is the exact
    // tan-half-angle Mobius transport.
    let numerator_coefficient =
        Real::one() + half_angle * (Real::from(2_i8) * target - Real::one());
    let numerator_constant = half_angle - target * (Real::one() + half_angle);
    let denominator_coefficient = Real::from(-2_i8) * half_angle;
    let denominator_constant = Real::one() + half_angle;
    let numerator = match cusp_parameter_affine_expression_sign(
        source,
        &numerator_coefficient,
        &numerator_constant,
        policy,
    )? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let denominator = match cusp_parameter_affine_expression_sign(
        source,
        &denominator_coefficient,
        &denominator_constant,
        policy,
    )? {
        Classification::Decided(RealSign::Zero) => {
            return Err(CurveError::InvalidBezierRange);
        }
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let sign = if denominator == RealSign::Negative {
        match numerator {
            RealSign::Negative => RealSign::Positive,
            RealSign::Zero => RealSign::Zero,
            RealSign::Positive => RealSign::Negative,
        }
    } else {
        numerator
    };
    Ok(Classification::Decided(match sign {
        RealSign::Negative => std::cmp::Ordering::Less,
        RealSign::Zero => std::cmp::Ordering::Equal,
        RealSign::Positive => std::cmp::Ordering::Greater,
    }))
}

fn complement_cusp_parameter_bracket(
    bracket: BezierAlgebraicCuspSemicircleParameterBracket2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicCuspSemicircleParameterBracket2>> {
    match bracket {
        BezierAlgebraicCuspSemicircleParameterBracket2::Exact(parameter) => {
            Ok(Classification::Decided(
                BezierAlgebraicCuspSemicircleParameterBracket2::Exact(Real::one() - parameter),
            ))
        }
        BezierAlgebraicCuspSemicircleParameterBracket2::Interval(interval) => {
            let start = Real::one() - interval.end();
            let end = Real::one() - interval.start();
            Ok(
                match BezierParameterInterval::try_new(start, end, policy)? {
                    Classification::Decided(interval) => Classification::Decided(
                        BezierAlgebraicCuspSemicircleParameterBracket2::Interval(interval),
                    ),
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                },
            )
        }
    }
}

fn positive_algebraic_point_field(
    point: &RationalBezierAlgebraicPointImage2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicCuspPositivePointField2>> {
    let point = match point.predicate_evaluator(policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let (x, y, denominator) = point.coordinate_polynomials();
    let scale = match point.denominator_sign() {
        RealSign::Positive => Real::one(),
        RealSign::Negative => Real::from(-1_i8),
        RealSign::Zero => {
            return Err(CurveError::Topology(
                "retained algebraic point had a zero affine denominator".into(),
            ));
        }
    };
    Ok(Classification::Decided(
        BezierAlgebraicCuspPositivePointField2 {
            x: polynomial_scale(x, &scale),
            y: polynomial_scale(y, &scale),
            denominator: polynomial_scale(denominator, &scale),
            parameter: point.retained_parameter().clone(),
        },
    ))
}

fn algebraic_point_linear_numerator(
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    x_factor: &Real,
    y_factor: &Real,
) -> Vec<Real> {
    let (x, y, _) = point.coordinate_polynomials();
    polynomial_add(
        &polynomial_scale(x, x_factor),
        &polynomial_scale(y, y_factor),
    )
}

fn selected_bivariate_parameter_pair_sign(
    polynomial: &BivariatePolynomial,
    first: &RationalBezierAlgebraicPointPredicate2<'_>,
    second: &RationalBezierAlgebraicPointPredicate2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    // Retained cusp points frequently share the identical selected local
    // field. Collapse that certified diagonal before interval boxes, rank-one
    // detection, or multi-field reduction; this is both exact and avoids a
    // zero-resultant proof for supporting-circle incidence.
    if first.retained_parameter() == second.retained_parameter() {
        return signed_coefficients_at_parameter(
            &bivariate_substitute_second_equal_first(polynomial),
            first.retained_parameter(),
            policy,
        );
    }
    if let Some(sign) = bivariate_parameter_pair_strict_sign_by_refinement(
        polynomial,
        first.retained_parameter(),
        second.retained_parameter(),
        policy,
    )? {
        return Ok(Classification::Decided(sign));
    }
    signed_bivariate_at_parameter_pair(
        polynomial,
        first.retained_parameter(),
        second.retained_parameter(),
        policy,
    )
}

fn signed_algebraic_point_linear_difference(
    first: &RationalBezierAlgebraicPointPredicate2<'_>,
    second: &RationalBezierAlgebraicPointPredicate2<'_>,
    x_factor: &Real,
    y_factor: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let (_, _, first_denominator) = first.coordinate_polynomials();
    let (_, _, second_denominator) = second.coordinate_polynomials();
    let first_numerator = algebraic_point_linear_numerator(first, x_factor, y_factor);
    let second_numerator = algebraic_point_linear_numerator(second, x_factor, y_factor);
    let difference = bivariate_subtract(
        &bivariate_outer_product(&first_numerator, second_denominator),
        &bivariate_outer_product(first_denominator, &second_numerator),
    );
    Ok(
        selected_bivariate_parameter_pair_sign(&difference, first, second, policy)?.map(|sign| {
            product_sign(
                sign,
                product_sign(first.denominator_sign(), second.denominator_sign()),
            )
        }),
    )
}

fn algebraic_point_linear_order(
    first: &RationalBezierAlgebraicPointPredicate2<'_>,
    second: &RationalBezierAlgebraicPointPredicate2<'_>,
    x_factor: &Real,
    y_factor: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<std::cmp::Ordering>> {
    Ok(
        signed_algebraic_point_linear_difference(first, second, x_factor, y_factor, policy)?.map(
            |sign| match sign {
                RealSign::Negative => std::cmp::Ordering::Less,
                RealSign::Zero => std::cmp::Ordering::Equal,
                RealSign::Positive => std::cmp::Ordering::Greater,
            },
        ),
    )
}

/// The specialized symbolic factor replay remains a bounded fast path. Larger
/// tensors continue through the rank-independent Hypersolve image authority;
/// this threshold is never a construction or correctness limit.
const MAX_TRIVARIATE_BOUNDED_FAST_PATH_CONTROLS: usize = 16_384;

/// A balanced product of 24 multi-affine factors occupies 25^3 controls. Keep
/// that measured-safe symbolic recursion envelope while sending larger exact
/// products to the complete rank-independent projection below.
const MAX_TRIVARIATE_EXACT_FACTOR_SPLITS: usize = 24;

const MAX_TRIVARIATE_EXACT_FACTOR_COEFFICIENTS: usize = MAX_TRIVARIATE_EXACT_FACTOR_SPLITS + 1;

const MAX_EXHAUSTIVE_MULTI_AFFINE_COEFFICIENTS: usize = 9;

const MAX_BOUNDED_BILINEAR_FACTORIZATIONS: usize = MAX_TRIVARIATE_EXACT_FACTOR_SPLITS;

const MAX_FIRST_BILINEAR_FACTOR_PROPOSALS: usize = 64;

/// Higher-degree slices receive a bounded proposal pass. A proposal can only
/// be accepted by exact division, so exhaustion loses capability rather than
/// exactness.
const MAX_BOUNDED_BILINEAR_FACTOR_PROPOSALS: usize = 256;

/// Compact dense polynomial on four independently selected scalar roots.
///
/// Coefficients are flat and row-major, with the fourth axis contiguous.  The
/// only symbolic elimination exposed for this tensor constrains that fourth
/// axis and returns trivariate coefficients; all other operations are the
/// small affine/product vocabulary needed by the pair-radial chord kernel.
#[derive(Clone, Debug)]
struct QuadrivariatePolynomial2 {
    dimensions: [usize; 4],
    coefficients: Vec<Real>,
}

impl QuadrivariatePolynomial2 {
    fn try_new(dimensions: [usize; 4], coefficients: Vec<Real>) -> Option<Self> {
        let count = dimensions
            .into_iter()
            .try_fold(1_usize, usize::checked_mul)?;
        if dimensions.contains(&0) || count == 0 || coefficients.len() != count {
            return None;
        }
        Some(Self {
            dimensions,
            coefficients,
        })
    }

    fn zero(dimensions: [usize; 4]) -> Option<Self> {
        let count = dimensions
            .into_iter()
            .try_fold(1_usize, usize::checked_mul)?;
        let mut coefficients = Vec::new();
        coefficients.try_reserve_exact(count).ok()?;
        coefficients.resize_with(count, Real::zero);
        Self::try_new(dimensions, coefficients)
    }

    fn flat_index(dimensions: [usize; 4], exponents: [usize; 4]) -> usize {
        (((exponents[0] * dimensions[1] + exponents[1]) * dimensions[2] + exponents[2])
            * dimensions[3])
            + exponents[3]
    }

    fn exponents(dimensions: [usize; 4], mut index: usize) -> [usize; 4] {
        let fourth = index % dimensions[3];
        index /= dimensions[3];
        let third = index % dimensions[2];
        index /= dimensions[2];
        let second = index % dimensions[1];
        index /= dimensions[1];
        [index, second, third, fourth]
    }

    fn coefficient(&self, exponents: [usize; 4]) -> Option<&Real> {
        exponents
            .into_iter()
            .zip(self.dimensions)
            .all(|(exponent, count)| exponent < count)
            .then(|| &self.coefficients[Self::flat_index(self.dimensions, exponents)])
    }

    fn from_axis_polynomial(coefficients: &[Real], axis: usize) -> Option<Self> {
        if coefficients.is_empty() || axis >= 4 {
            return None;
        }
        let mut dimensions = [1; 4];
        dimensions[axis] = coefficients.len();
        let mut polynomial = Self::zero(dimensions)?;
        for (power, coefficient) in coefficients.iter().enumerate() {
            let mut exponents = [0; 4];
            exponents[axis] = power;
            polynomial.coefficients[Self::flat_index(dimensions, exponents)] = coefficient.clone();
        }
        Some(polynomial)
    }

    fn lift_trivariate(polynomial: &TrivariatePolynomial, axes: [usize; 3]) -> Option<Self> {
        if axes.into_iter().any(|axis| axis >= 4)
            || axes[0] == axes[1]
            || axes[0] == axes[2]
            || axes[1] == axes[2]
        {
            return None;
        }
        let source = polynomial.dimensions();
        let mut dimensions = [1; 4];
        for (source_axis, target_axis) in axes.into_iter().enumerate() {
            dimensions[target_axis] = [source.0, source.1, source.2][source_axis];
        }
        let mut lifted = Self::zero(dimensions)?;
        for (first, rows) in polynomial.coefficients.iter().enumerate() {
            for (second, row) in rows.iter().enumerate() {
                for (third, coefficient) in row.iter().enumerate() {
                    let source_exponents = [first, second, third];
                    let mut target_exponents = [0; 4];
                    for source_axis in 0..3 {
                        target_exponents[axes[source_axis]] = source_exponents[source_axis];
                    }
                    lifted.coefficients[Self::flat_index(dimensions, target_exponents)] =
                        coefficient.clone();
                }
            }
        }
        Some(lifted)
    }

    fn combine(&self, other: &Self, subtract: bool) -> Option<Self> {
        let dimensions =
            std::array::from_fn(|axis| self.dimensions[axis].max(other.dimensions[axis]));
        let mut result = Self::zero(dimensions)?;
        for (source, subtract_source) in [(self, false), (other, subtract)] {
            for (index, coefficient) in source.coefficients.iter().enumerate() {
                let target =
                    Self::flat_index(dimensions, Self::exponents(source.dimensions, index));
                if subtract_source {
                    result.coefficients[target] -= coefficient;
                } else {
                    result.coefficients[target] += coefficient;
                }
            }
        }
        Some(result)
    }

    fn add(&self, other: &Self) -> Option<Self> {
        self.combine(other, false)
    }

    fn subtract(&self, other: &Self) -> Option<Self> {
        self.combine(other, true)
    }

    fn scale(&self, scale: &Real) -> Option<Self> {
        let mut result = Self::zero(self.dimensions)?;
        for (target, source) in result.coefficients.iter_mut().zip(&self.coefficients) {
            *target = source * scale;
        }
        Some(result)
    }

    fn linear_combination(terms: &[(&Self, &Real)]) -> Option<Self> {
        let dimensions = terms.iter().fold([0; 4], |mut dimensions, (term, _)| {
            for (axis, count) in term.dimensions.into_iter().enumerate() {
                dimensions[axis] = dimensions[axis].max(count);
            }
            dimensions
        });
        let mut result = Self::zero(dimensions)?;
        for (term, scale) in terms {
            if scale.zero_status() == ZeroKnowledge::Zero {
                continue;
            }
            for (index, coefficient) in term.coefficients.iter().enumerate() {
                let target = Self::flat_index(dimensions, Self::exponents(term.dimensions, index));
                result.coefficients[target] += coefficient * *scale;
            }
        }
        Some(result)
    }

    fn multiply(&self, other: &Self) -> Option<Self> {
        Self::sum_products(&[(self, other, false)])
    }

    fn sum_products(terms: &[(&Self, &Self, bool)]) -> Option<Self> {
        let dimensions = terms
            .iter()
            .try_fold([0; 4], |mut dimensions, (left, right, _)| {
                for (axis, dimension) in dimensions.iter_mut().enumerate() {
                    *dimension = (*dimension).max(
                        left.dimensions[axis]
                            .checked_add(right.dimensions[axis])?
                            .checked_sub(1)?,
                    );
                }
                Some(dimensions)
            })?;
        let mut result = Self::zero(dimensions)?;
        for (left, right, subtract) in terms {
            for (left_index, left_coefficient) in left.coefficients.iter().enumerate() {
                let left_exponents = Self::exponents(left.dimensions, left_index);
                for (right_index, right_coefficient) in right.coefficients.iter().enumerate() {
                    let right_exponents = Self::exponents(right.dimensions, right_index);
                    let exponents =
                        std::array::from_fn(|axis| left_exponents[axis] + right_exponents[axis]);
                    let target = Self::flat_index(dimensions, exponents);
                    if *subtract {
                        result.coefficients[target] -= left_coefficient * right_coefficient;
                    } else {
                        result.coefficients[target] += left_coefficient * right_coefficient;
                    }
                }
            }
        }
        Some(result)
    }

    fn to_dense_polynomial(&self) -> Option<DenseTensorPolynomial> {
        let mut coefficients = Vec::new();
        coefficients
            .try_reserve_exact(self.coefficients.len())
            .ok()?;
        coefficients.extend(self.coefficients.iter().cloned());
        DenseTensorPolynomial::try_new(self.dimensions.to_vec(), coefficients)
    }
}

impl BezierDenseTwoSquareRootExpression2 {
    fn zero(rank: usize) -> Option<DenseTensorPolynomial> {
        DenseTensorPolynomial::zero(vec![1; rank])
    }

    fn from_rational(rational: DenseTensorPolynomial) -> Option<Self> {
        let zero = Self::zero(rational.dimensions().len())?;
        Some(Self {
            rational,
            first: zero.clone(),
            second: zero.clone(),
            product: zero,
        })
    }

    fn from_first_radical(first: DenseTensorPolynomial) -> Option<Self> {
        let zero = Self::zero(first.dimensions().len())?;
        Some(Self {
            rational: zero.clone(),
            first,
            second: zero.clone(),
            product: zero,
        })
    }

    fn from_second_radical(second: DenseTensorPolynomial) -> Option<Self> {
        let zero = Self::zero(second.dimensions().len())?;
        Some(Self {
            rational: zero.clone(),
            first: zero.clone(),
            second,
            product: zero,
        })
    }

    fn polynomial_is_stored_zero(polynomial: &DenseTensorPolynomial) -> bool {
        polynomial.coefficients().iter().all(|coefficient| {
            coefficient
                .exact_rational_ref()
                .is_some_and(|value| value.is_zero())
        })
    }

    fn is_stored_zero(&self) -> bool {
        [&self.rational, &self.first, &self.second, &self.product]
            .into_iter()
            .all(Self::polynomial_is_stored_zero)
    }

    fn is_stored_one(&self) -> bool {
        let Some((constant, remainder)) = self.rational.coefficients().split_first() else {
            return false;
        };
        constant
            .exact_rational_ref()
            .is_some_and(|value| value.is_one())
            && remainder.iter().all(|value| {
                value
                    .exact_rational_ref()
                    .is_some_and(|value| value.is_zero())
            })
            && [&self.first, &self.second, &self.product]
                .into_iter()
                .all(Self::polynomial_is_stored_zero)
    }

    fn combine(&self, other: &Self, subtract: bool) -> Option<Self> {
        let combine = |first: &DenseTensorPolynomial, second: &DenseTensorPolynomial| {
            if subtract {
                first.subtract(second)
            } else {
                first.add(second)
            }
        };
        Some(Self {
            rational: combine(&self.rational, &other.rational)?,
            first: combine(&self.first, &other.first)?,
            second: combine(&self.second, &other.second)?,
            product: combine(&self.product, &other.product)?,
        })
    }

    fn add(&self, other: &Self) -> Option<Self> {
        self.combine(other, false)
    }

    fn subtract(&self, other: &Self) -> Option<Self> {
        self.combine(other, true)
    }

    fn scale(&self, scale: &Real) -> Option<Self> {
        Some(Self {
            rational: self.rational.scale(scale)?,
            first: self.first.scale(scale)?,
            second: self.second.scale(scale)?,
            product: self.product.scale(scale)?,
        })
    }

    fn multiply_rational(&self, polynomial: &DenseTensorPolynomial) -> Option<Self> {
        Some(Self {
            rational: self.rational.multiply(polynomial)?,
            first: self.first.multiply(polynomial)?,
            second: self.second.multiply(polynomial)?,
            product: self.product.multiply(polynomial)?,
        })
    }

    fn multiply(
        &self,
        other: &Self,
        first_speed_squared: &DenseTensorPolynomial,
        second_speed_squared: &DenseTensorPolynomial,
    ) -> Option<Self> {
        let speed_product = first_speed_squared.multiply(second_speed_squared)?;
        let rational = self
            .rational
            .multiply(&other.rational)?
            .add(
                &self
                    .first
                    .multiply(&other.first)?
                    .multiply(first_speed_squared)?,
            )?
            .add(
                &self
                    .second
                    .multiply(&other.second)?
                    .multiply(second_speed_squared)?,
            )?
            .add(
                &self
                    .product
                    .multiply(&other.product)?
                    .multiply(&speed_product)?,
            )?;
        let first = self
            .rational
            .multiply(&other.first)?
            .add(&self.first.multiply(&other.rational)?)?
            .add(
                &self
                    .second
                    .multiply(&other.product)?
                    .multiply(second_speed_squared)?,
            )?
            .add(
                &self
                    .product
                    .multiply(&other.second)?
                    .multiply(second_speed_squared)?,
            )?;
        let second = self
            .rational
            .multiply(&other.second)?
            .add(&self.second.multiply(&other.rational)?)?
            .add(
                &self
                    .first
                    .multiply(&other.product)?
                    .multiply(first_speed_squared)?,
            )?
            .add(
                &self
                    .product
                    .multiply(&other.first)?
                    .multiply(first_speed_squared)?,
            )?;
        let product = self
            .rational
            .multiply(&other.product)?
            .add(&self.product.multiply(&other.rational)?)?
            .add(&self.first.multiply(&other.second)?)?
            .add(&self.second.multiply(&other.first)?)?;
        Some(Self {
            rational,
            first,
            second,
            product,
        })
    }

    fn square(
        &self,
        first_speed_squared: &DenseTensorPolynomial,
        second_speed_squared: &DenseTensorPolynomial,
    ) -> Option<Self> {
        let speed_product = first_speed_squared.multiply(second_speed_squared)?;
        let rational = self
            .rational
            .multiply(&self.rational)?
            .add(
                &self
                    .first
                    .multiply(&self.first)?
                    .multiply(first_speed_squared)?,
            )?
            .add(
                &self
                    .second
                    .multiply(&self.second)?
                    .multiply(second_speed_squared)?,
            )?
            .add(
                &self
                    .product
                    .multiply(&self.product)?
                    .multiply(&speed_product)?,
            )?;
        let two = Real::from(2_i8);
        let first = self
            .rational
            .multiply(&self.first)?
            .add(
                &self
                    .second
                    .multiply(&self.product)?
                    .multiply(second_speed_squared)?,
            )?
            .scale(&two)?;
        let second = self
            .rational
            .multiply(&self.second)?
            .add(
                &self
                    .first
                    .multiply(&self.product)?
                    .multiply(first_speed_squared)?,
            )?
            .scale(&two)?;
        let product = self
            .rational
            .multiply(&self.product)?
            .add(&self.first.multiply(&self.second)?)?
            .scale(&two)?;
        Some(Self {
            rational,
            first,
            second,
            product,
        })
    }

    fn reduced(&self, sources: &[AlgebraicRootRepresentation]) -> Option<Self> {
        Some(Self {
            rational: dense_reduce_selected_root_relations(self.rational.clone(), sources)?,
            first: dense_reduce_selected_root_relations(self.first.clone(), sources)?,
            second: dense_reduce_selected_root_relations(self.second.clone(), sources)?,
            product: dense_reduce_selected_root_relations(self.product.clone(), sources)?,
        })
    }

    /// Canonicalizes a coefficient-field value at the retained source tuple.
    /// Unlike [`Self::reduced`], these tensors have no free output axis.
    fn reduced_at_source_tuple(&self, sources: &[AlgebraicRootRepresentation]) -> Option<Self> {
        Some(Self {
            rational: dense_reduce_selected_tuple_relations(self.rational.clone(), sources)?,
            first: dense_reduce_selected_tuple_relations(self.first.clone(), sources)?,
            second: dense_reduce_selected_tuple_relations(self.second.clone(), sources)?,
            product: dense_reduce_selected_tuple_relations(self.product.clone(), sources)?,
        })
    }

    /// Enumerates conjugate-sheet zeros without multiplicities introduced only
    /// by absent generators. Callers must still replay the authored sheet.
    fn projection(
        &self,
        first_speed_squared: &DenseTensorPolynomial,
        second_speed_squared: &DenseTensorPolynomial,
        sources: &[AlgebraicRootRepresentation],
    ) -> Option<DenseTensorPolynomial> {
        let expression = self.reduced(sources)?;
        let first_speed_squared =
            dense_reduce_selected_root_relations(first_speed_squared.clone(), sources)?;
        let second_speed_squared =
            dense_reduce_selected_root_relations(second_speed_squared.clone(), sources)?;
        let reduce = |polynomial| dense_reduce_selected_root_relations(polynomial, sources);
        let (retained_rational, retained_radical) =
            if Self::polynomial_is_stored_zero(&expression.second)
                && Self::polynomial_is_stored_zero(&expression.product)
            {
                // An absent second generator would only square the retained
                // first-generator equation, doubling its eventual norm degree.
                (expression.rational, expression.first)
            } else {
                let rational_squared = reduce(expression.rational.multiply(&expression.rational)?)?;
                let first_squared = reduce(expression.first.multiply(&expression.first)?)?;
                let second_squared = reduce(expression.second.multiply(&expression.second)?)?;
                let product_squared = reduce(expression.product.multiply(&expression.product)?)?;
                let retained_rational = reduce(
                    rational_squared
                        .add(&reduce(first_squared.multiply(&first_speed_squared)?)?)?
                        .subtract(&reduce(
                            second_speed_squared.multiply(&second_squared.add(&reduce(
                                product_squared.multiply(&first_speed_squared)?,
                            )?)?)?,
                        )?)?,
                )?;
                let retained_radical = reduce(
                    expression
                        .rational
                        .multiply(&expression.first)?
                        .subtract(&reduce(
                            second_speed_squared
                                .multiply(&expression.second.multiply(&expression.product)?)?,
                        )?)?
                        .scale(&Real::from(2_i8))?,
                )?;
                (retained_rational, retained_radical)
            };
        if Self::polynomial_is_stored_zero(&retained_radical) {
            // The final norm would be a square. Keep its base, including any
            // radicand factors: their zeros have not been proved absent.
            return Some(retained_rational);
        }
        reduce(
            retained_rational
                .multiply(&retained_rational)?
                .subtract(&reduce(
                    retained_radical
                        .multiply(&retained_radical)?
                        .multiply(&first_speed_squared)?,
                )?)?,
        )
    }
}

/// Projects the third-axis zeros of one exact `A + branch*B*sqrt(K)`
/// expression at a retained pair of selected source roots, then rejects every
/// conjugate or opposite-branch norm root by exact unsquared replay.
fn selected_pair_square_root_expression_third_axis_parameters(
    expression: &BezierAlgebraicCuspTrivariateSquareRootExpression2,
    radicand: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    branch: i8,
    domain: SelectedThirdAxisDomain2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    let Some(projection) = expression.projection(radicand) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let projection =
        trivariate_reduce_parameter_pair_relations(&projection, first_parameter, second_parameter)
            .unwrap_or(projection);
    selected_pair_square_root_expression_third_axis_parameters_from_projection(
        expression,
        radicand,
        &projection,
        first_parameter,
        second_parameter,
        branch,
        domain,
        policy,
    )
}

#[allow(clippy::too_many_arguments)]
fn selected_pair_square_root_expression_third_axis_parameters_from_projection(
    expression: &BezierAlgebraicCuspTrivariateSquareRootExpression2,
    radicand: &TrivariatePolynomial,
    projection: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    branch: i8,
    domain: SelectedThirdAxisDomain2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    selected_projected_trivariate_third_axis_parameters(
        projection,
        first_parameter,
        second_parameter,
        domain,
        policy,
        |candidate, projected_root_certified| {
            algebraic_cusp_projected_trivariate_square_root_sum_sign(
                expression,
                radicand,
                projection,
                first_parameter,
                second_parameter,
                candidate,
                branch,
                projected_root_certified,
                policy,
            )
        },
    )
}

/// Certifies that one projected third-axis candidate belongs to the two
/// selected source roots, rather than to a conjugate pair introduced by the
/// sequential resultants.
///
/// The two defining polynomials and `incidence` form a square system in the
/// three parameter axes.  On a sufficiently refined product isolator, strict
/// opposite signs on each pair of coordinate faces give a Poincare--Miranda
/// existence proof.  Each source box contains exactly its selected root and
/// the third box contains exactly one root of the authored projection, so the
/// certified zero is necessarily the represented parameter triple.  This is
/// a division-free fast authority for transverse roots; multiple roots fall
/// through to the general exact signer.
fn projected_selected_trivariate_candidate_has_box_root(
    incidence: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    third_parameter: &BezierParameter2,
    maximum_steps: usize,
) -> CurveResult<bool> {
    let strict = &CurveContext::STRICT;
    match (first_parameter, second_parameter, third_parameter) {
        (
            BezierParameter2::Exact(first),
            second @ BezierParameter2::Algebraic(second_algebraic),
            third @ BezierParameter2::Algebraic(_),
        ) => {
            let Some((incidence, [1, 2])) =
                trivariate_specialize_axis_bivariate(incidence, 0, first)
            else {
                return Ok(false);
            };
            let defining = BivariatePolynomial::new(
                second_algebraic
                    .polynomial()
                    .coefficients()
                    .iter()
                    .map(|coefficient| vec![coefficient.clone()])
                    .collect(),
            );
            return projected_bivariate_parameter_pair_has_box_root(
                &defining, &incidence, second, third, strict,
            );
        }
        (
            first @ BezierParameter2::Algebraic(first_algebraic),
            BezierParameter2::Exact(second),
            third @ BezierParameter2::Algebraic(_),
        ) => {
            let Some((incidence, [0, 2])) =
                trivariate_specialize_axis_bivariate(incidence, 1, second)
            else {
                return Ok(false);
            };
            let defining = BivariatePolynomial::new(
                first_algebraic
                    .polynomial()
                    .coefficients()
                    .iter()
                    .map(|coefficient| vec![coefficient.clone()])
                    .collect(),
            );
            return projected_bivariate_parameter_pair_has_box_root(
                &defining, &incidence, first, third, strict,
            );
        }
        (
            first @ BezierParameter2::Algebraic(_),
            second @ BezierParameter2::Algebraic(_),
            BezierParameter2::Exact(third),
        ) => {
            let Some((incidence, [0, 1])) =
                trivariate_specialize_axis_bivariate(incidence, 2, third)
            else {
                return Ok(false);
            };
            return Ok(matches!(
                signed_bivariate_at_parameter_pair_exact_first(&incidence, first, second,)?,
                Classification::Decided(RealSign::Zero)
            ));
        }
        (BezierParameter2::Exact(first), BezierParameter2::Exact(second), third) => {
            let Some((incidence, [1, 2])) =
                trivariate_specialize_axis_bivariate(incidence, 0, first)
            else {
                return Ok(false);
            };
            return Ok(matches!(
                signed_coefficients_at_parameter(
                    &bivariate_specialize_first(&incidence, second),
                    third,
                    strict,
                )?,
                Classification::Decided(RealSign::Zero)
            ));
        }
        (first, BezierParameter2::Exact(second), BezierParameter2::Exact(third)) => {
            let Some((incidence, [0, 2])) =
                trivariate_specialize_axis_bivariate(incidence, 1, second)
            else {
                return Ok(false);
            };
            return Ok(matches!(
                signed_coefficients_at_parameter(
                    &bivariate_specialize_second(&incidence, third),
                    first,
                    strict,
                )?,
                Classification::Decided(RealSign::Zero)
            ));
        }
        (BezierParameter2::Exact(first), second, BezierParameter2::Exact(third)) => {
            let Some((incidence, [1, 2])) =
                trivariate_specialize_axis_bivariate(incidence, 0, first)
            else {
                return Ok(false);
            };
            return Ok(matches!(
                signed_coefficients_at_parameter(
                    &bivariate_specialize_second(&incidence, third),
                    second,
                    strict,
                )?,
                Classification::Decided(RealSign::Zero)
            ));
        }
        _ => {}
    }
    if !matches!(first_parameter, BezierParameter2::Algebraic(_))
        || !matches!(second_parameter, BezierParameter2::Algebraic(_))
        || !matches!(third_parameter, BezierParameter2::Algebraic(_))
    {
        return Ok(false);
    }

    let mut first_refinement = BezierParameterRefinement2::new(first_parameter, strict);
    let mut second_refinement = BezierParameterRefinement2::new(second_parameter, strict);
    let mut third_refinement = BezierParameterRefinement2::new(third_parameter, strict);
    let mut previous_box = None;
    for target_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        if target_steps > maximum_steps {
            break;
        }
        // The incidence faces lie one target-box width away from the root.
        // Refine the retained source axes ahead of that width so their
        // coefficient variation cannot mask the transverse third-axis sign.
        let source_steps = target_steps.saturating_add(64);
        let first = first_refinement.refine_to(source_steps).clone();
        let second = second_refinement.refine_to(source_steps).clone();
        let third = third_refinement.refine_to(target_steps).clone();
        if previous_box
            .as_ref()
            .is_some_and(|(old_first, old_second, old_third)| {
                old_first == &first && old_second == &second && old_third == &third
            })
        {
            break;
        }
        previous_box = Some((first.clone(), second.clone(), third.clone()));

        let defining_face_signs = |parameter: &BezierParameter2| {
            let BezierParameter2::Algebraic(parameter) = parameter else {
                return None;
            };
            let lower = Real::eval_poly(
                parameter.polynomial().coefficients(),
                parameter.interval().start(),
            );
            let upper = Real::eval_poly(
                parameter.polynomial().coefficients(),
                parameter.interval().end(),
            );
            Some((real_sign(&lower, strict)?, real_sign(&upper, strict)?))
        };
        let Some((first_lower, first_upper)) = defining_face_signs(&first) else {
            continue;
        };
        let Some((second_lower, second_upper)) = defining_face_signs(&second) else {
            continue;
        };
        if !strict_signs_are_opposite(Some(first_lower), Some(first_upper))
            || !strict_signs_are_opposite(Some(second_lower), Some(second_upper))
        {
            continue;
        }

        let restricted = trivariate_restrict_to_parameter_box(incidence, &first, &second, &third);
        let Some((third_coefficients, remaining)) =
            trivariate_axis_bivariate_coefficients(&restricted, 2)
        else {
            continue;
        };
        if remaining != [0, 1] || third_coefficients.is_empty() {
            continue;
        }
        let lower_face = &third_coefficients[0];
        let upper_face = third_coefficients
            .iter()
            .skip(1)
            .fold(lower_face.clone(), |sum, coefficient| {
                bivariate_add(&sum, coefficient)
            });
        let lower_sign = bivariate_unit_square_strict_bernstein_sign(lower_face, strict)?;
        let upper_sign = bivariate_unit_square_strict_bernstein_sign(&upper_face, strict)?;
        if strict_signs_are_opposite(lower_sign, upper_sign) {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Returns a defining polynomial whose selected root is simple.
///
/// A resultant carrier may retain an even-multiplicity factor. Exact
/// square-free reduction removes that multiplicity in one step; derivative
/// replay is retained only as an exact fallback when the generic reduction is
/// unavailable. No approximate equality is allowed to choose this construction
/// polynomial.
fn selected_parameter_simple_constraint(
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<Real>>> {
    let BezierParameter2::Algebraic(parameter) = parameter else {
        let BezierParameter2::Exact(parameter) = parameter else {
            unreachable!()
        };
        return Ok(Classification::Decided(vec![
            -parameter.clone(),
            Real::one(),
        ]));
    };
    let selected = BezierParameter2::Algebraic(parameter.clone());
    let strict = policy.strict_counterpart();
    let original = parameter.polynomial().coefficients().to_vec();
    if let Some(square_free) =
        hypersolve::square_free_part(original.clone(), hypersolve::PredicatePolicy::STRICT)
    {
        return Ok(Classification::Decided(square_free));
    }
    let mut constraint = original;
    loop {
        if constraint.len() <= 1 {
            return Err(CurveError::InvalidBezierAlgebraicParameter);
        }
        let derivative = polynomial_derivative(&constraint);
        match signed_coefficients_at_parameter(&derivative, &selected, &strict)? {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                return Ok(Classification::Decided(constraint));
            }
            Classification::Decided(RealSign::Zero) => constraint = derivative,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
}

/// Replaces one projected parameter carrier by an exact lower-degree common
/// factor when that factor owns the already isolated root.
///
/// The proposal is accepted only after exact GCD construction, exact
/// square-free reduction, and a STRICT singleton root count in the original
/// isolating interval. Failure to find a smaller carrier is only a scheduling
/// miss; the original parameter remains authoritative.
fn selected_parameter_reduced_by_constraint(
    parameter: &BezierParameter2,
    constraint: &[Real],
) -> CurveResult<BezierParameter2> {
    let BezierParameter2::Algebraic(parameter) = parameter else {
        return Ok(parameter.clone());
    };
    let original = parameter.polynomial().coefficients();
    let Some(common) = greatest_common_divisor_univariate_polynomials_exact(original, constraint)
    else {
        return Ok(BezierParameter2::Algebraic(parameter.clone()));
    };
    let Some(common) = hypersolve::square_free_part(common, hypersolve::PredicatePolicy::STRICT)
    else {
        return Ok(BezierParameter2::Algebraic(parameter.clone()));
    };
    if common.len() <= 1 || common.len() >= original.len() {
        return Ok(BezierParameter2::Algebraic(parameter.clone()));
    }
    let polynomial =
        match BezierParameterPolynomial::try_new_power_basis(common, &CurveContext::STRICT)? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(_) => {
                return Ok(BezierParameter2::Algebraic(parameter.clone()));
            }
        };
    match polynomial.root_count_in_interval(parameter.interval(), &CurveContext::STRICT)? {
        Classification::Decided(0) | Classification::Uncertain(_) => {
            Ok(BezierParameter2::Algebraic(parameter.clone()))
        }
        Classification::Decided(1) => Ok(BezierParameter2::Algebraic(
            BezierAlgebraicParameter2::from_certified_singleton(
                polynomial,
                parameter.interval().clone(),
            ),
        )),
        Classification::Decided(_) => Err(CurveError::Topology(
            "an exact common factor introduced multiple roots into a singleton interval".into(),
        )),
    }
}

/// Returns a square-free univariate carrier for the algebraic roots of one
/// selected-pair trivariate projection. This is optional scheduling evidence:
/// exact projection or isolation uncertainty leaves the caller's existing
/// carrier unchanged.
fn selected_trivariate_third_axis_constraint(
    polynomial: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
) -> CurveResult<Option<Vec<Real>>> {
    let projection = match selected_trivariate_third_axis_parameters(
        polynomial,
        first_parameter,
        second_parameter,
        SelectedThirdAxisDomain2::Finite(&CurveParameterRange2::unit()),
        &CurveContext::STRICT,
    )? {
        Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(parameters)) => {
            parameters
        }
        Classification::Decided(
            BezierAlgebraicFiberProjection2::IdenticallyZero
            | BezierAlgebraicFiberProjection2::Degenerate,
        )
        | Classification::Uncertain(_) => return Ok(None),
    };
    let Some(coefficients) = projection
        .into_iter()
        .find_map(|parameter| match parameter {
            BezierParameter2::Algebraic(parameter) => {
                Some(parameter.polynomial().coefficients().to_vec())
            }
            BezierParameter2::Exact(_) => None,
        })
    else {
        return Ok(None);
    };
    Ok(hypersolve::square_free_part(
        coefficients,
        hypersolve::PredicatePolicy::STRICT,
    ))
}

fn independent_parameter_pair_incidence(second: &BezierParameter2) -> BivariatePolynomial {
    let coefficients = match second {
        BezierParameter2::Exact(parameter) => vec![-parameter.clone(), Real::one()],
        BezierParameter2::Algebraic(parameter) => parameter.polynomial().coefficients().to_vec(),
    };
    BivariatePolynomial::new(vec![coefficients])
}

/// Certifies projected-root membership through the first nonzero exact
/// subresultant in the selected source-pair fiber.
///
/// The ordinary resultant proves that the target constraint and projected
/// incidence share some root at the two selected source values. Scanning the
/// constrained subresultants then recovers their fiber GCD without a primitive
/// element. The target constraint is simple at the represented candidate, so
/// that GCD is transverse there even when the original geometric incidence
/// has even multiplicity; the existing product-box authority can therefore
/// correlate the exact root tuple.
fn projected_selected_trivariate_candidate_has_subresultant_root(
    projected_incidence: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    third_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    projected_selected_trivariate_candidate_has_subresultant_root_with_resultant_limit(
        projected_incidence,
        first_parameter,
        second_parameter,
        third_parameter,
        MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
        policy,
    )
}

fn projected_selected_trivariate_candidate_has_subresultant_root_with_resultant_limit(
    projected_incidence: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    third_parameter: &BezierParameter2,
    max_resultant_degree: usize,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let strict = policy.strict_counterpart();
    let pair_incidence = independent_parameter_pair_incidence(second_parameter);
    let pair_sign = |polynomial: &BivariatePolynomial| {
        algebraic_selected_correlated_predicate_sign(
            &pair_incidence,
            polynomial,
            first_parameter,
            second_parameter,
            &strict,
        )
    };

    // Remove target-axis coefficients that vanish only in the selected pair
    // field. Retaining their globally nonzero conjugate values would give the
    // symbolic subresultant the wrong specialized degree.
    let Some((mut target_coefficients, [0, 1])) =
        trivariate_axis_bivariate_coefficients(projected_incidence, 2)
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    while let Some(coefficient) = target_coefficients.last() {
        match pair_sign(coefficient)? {
            Classification::Decided(RealSign::Zero) if target_coefficients.len() > 1 => {
                target_coefficients.pop();
            }
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Decided(RealSign::Positive | RealSign::Negative) => break,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let Some(projected_incidence) =
        trivariate_from_axis_bivariate_coefficients(&target_coefficients, 2, [0, 1])
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let constraint = match selected_parameter_simple_constraint(third_parameter, &strict)? {
        Classification::Decided(constraint) => constraint,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let mut config = CurveIntersectionResultantConfig {
        min_precision: hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION,
        max_resultant_degree,
    };
    let mut resultant = resultant_trivariate_polynomial_univariate_constraint(
        &projected_incidence,
        &constraint,
        TrivariatePolynomialAxis::Third,
        config,
    );
    if resultant.status == TrivariateConstraintResultantStatus::DegreeBoundExceeded
        && config.max_resultant_degree != usize::MAX
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "selected-pair-constrained-elimination",
            "unbounded-cold-continuation",
        );
        config.max_resultant_degree = usize::MAX;
        resultant = resultant_trivariate_polynomial_univariate_constraint(
            &projected_incidence,
            &constraint,
            TrivariatePolynomialAxis::Third,
            config,
        );
    }
    let resultant = match resultant.status {
        TrivariateConstraintResultantStatus::Constructed => resultant
            .resultant
            .expect("a constructed constrained resultant retains its polynomial"),
        TrivariateConstraintResultantStatus::UndecidedCoefficient => {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        TrivariateConstraintResultantStatus::EmptyPolynomial
        | TrivariateConstraintResultantStatus::InvalidConstraint
        | TrivariateConstraintResultantStatus::DegreeBoundExceeded
        | TrivariateConstraintResultantStatus::ResultantError
        | TrivariateConstraintResultantStatus::InterpolationDivisionFailed => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
    };
    match pair_sign(&resultant)? {
        Classification::Decided(RealSign::Positive | RealSign::Negative) => {
            return Ok(Classification::Decided(false));
        }
        Classification::Decided(RealSign::Zero) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }

    let maximum_order = target_coefficients
        .len()
        .saturating_sub(1)
        .min(constraint.len().saturating_sub(1));
    for order in 1..=maximum_order {
        let mut report = subresultant_trivariate_polynomial_univariate_constraint(
            &projected_incidence,
            &constraint,
            TrivariatePolynomialAxis::Third,
            order,
            config,
        );
        if report.status == TrivariateConstraintSubresultantStatus::DegreeBoundExceeded
            && config.max_resultant_degree != usize::MAX
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "selected-pair-constrained-elimination",
                "unbounded-cold-continuation",
            );
            config.max_resultant_degree = usize::MAX;
            report = subresultant_trivariate_polynomial_univariate_constraint(
                &projected_incidence,
                &constraint,
                TrivariatePolynomialAxis::Third,
                order,
                config,
            );
        }
        match report.status {
            TrivariateConstraintSubresultantStatus::Constructed => {}
            TrivariateConstraintSubresultantStatus::UndecidedCoefficient => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            TrivariateConstraintSubresultantStatus::EmptyPolynomial
            | TrivariateConstraintSubresultantStatus::InvalidConstraint
            | TrivariateConstraintSubresultantStatus::InvalidOrder
            | TrivariateConstraintSubresultantStatus::DegreeBoundExceeded
            | TrivariateConstraintSubresultantStatus::DeterminantError
            | TrivariateConstraintSubresultantStatus::InterpolationDivisionFailed => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
        }
        let mut any_nonzero = false;
        let mut uncertainty = None;
        for coefficient in &report.coefficients {
            match pair_sign(coefficient)? {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                    any_nonzero = true;
                    break;
                }
                Classification::Uncertain(reason) => uncertainty = Some(reason),
            }
        }
        if !any_nonzero {
            if let Some(reason) = uncertainty {
                return Ok(Classification::Uncertain(reason));
            }
            continue;
        }
        let Some(gcd) =
            trivariate_from_axis_bivariate_coefficients(&report.coefficients, 2, [0, 1])
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if projected_selected_trivariate_candidate_has_box_root(
            &gcd,
            first_parameter,
            second_parameter,
            third_parameter,
            64,
        )? {
            return Ok(Classification::Decided(true));
        }
        return Ok(
            match trivariate_parameter_triple_sign_by_refinement(
                &gcd,
                first_parameter,
                second_parameter,
                third_parameter,
                &strict,
            )? {
                Classification::Decided(RealSign::Zero) => Classification::Decided(true),
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                    Classification::Decided(false)
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        );
    }
    Err(CurveError::Topology(
        "a constrained subresultant sequence lost its nonzero terminal polynomial".into(),
    ))
}

fn bezier_parameter_root_representation(
    parameter: &BezierParameter2,
) -> AlgebraicRootRepresentation {
    match parameter {
        BezierParameter2::Exact(parameter) => {
            AlgebraicRootRepresentation::from_exact_value(parameter)
        }
        BezierParameter2::Algebraic(parameter) => {
            certified_parameter_representation(parameter, &CurveContext::STRICT)
        }
    }
}

/// Applies one exact final-axis enumerator to the authored finite span and,
/// when requested, its regular incident ray.  Circle/parallel coefficient
/// authorities differ, but domain clipping and ordering are identical.
fn selected_axis_parameters_in_domain(
    domain: CurveParameterDomain2<'_>,
    policy: &CurveContext,
    mut parameters_in_domain: impl FnMut(
        SelectedThirdAxisDomain2<'_>,
    )
        -> CurveResult<Classification<BezierAlgebraicFiberProjection2>>,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    let mut parameters =
        match parameters_in_domain(SelectedThirdAxisDomain2::Finite(domain.finite))? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(parameters)) => {
                parameters
            }
            Classification::Decided(projection) => return Ok(Classification::Decided(projection)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    if domain.inclusion != [true; 2] {
        let mut owned = Vec::with_capacity(parameters.len());
        for parameter in parameters {
            match domain.contains_finite_parameter(&parameter.clone().into(), policy)? {
                Classification::Decided(true) => owned.push(parameter),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        parameters = owned;
    }
    if let Some(extension) = domain.extension {
        match parameters_in_domain(SelectedThirdAxisDomain2::IncidentRay {
            anchor: extension.anchor,
            direction: extension.direction,
            barrier: extension.barrier,
        })? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(exterior)) => {
                parameters.reserve(exterior.len());
                for parameter in exterior {
                    // The finite cell owns shared roots, including its closed
                    // endpoints. Keep the original range authorities for this
                    // decision; outward root-isolation bounds are only a schedule.
                    match domain.contains_finite_parameter(&parameter.clone().into(), policy)? {
                        Classification::Decided(true) => {}
                        Classification::Decided(false) => parameters.push(parameter),
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
            }
            Classification::Decided(projection) => return Ok(Classification::Decided(projection)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    Ok(Classification::Decided(
        BezierAlgebraicFiberProjection2::Parameters(parameters),
    ))
}

impl BezierDirectPairRadialParallelFastPath2 {
    fn compact_source_parameters(&self) -> Option<[BezierParameter2; 2]> {
        self.pair_map.compact_source_parameters()
    }

    fn pair_expression_sign(
        &self,
        expression: &BezierAlgebraicCuspTrivariateSquareRootExpression2,
        target_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let Some([first_parameter, second_parameter]) = self.compact_source_parameters() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        algebraic_cusp_trivariate_square_root_sum_sign(
            expression,
            &self.pair_discriminant,
            &first_parameter,
            &second_parameter,
            target_parameter,
            self.branch,
            policy,
        )
    }

    fn expression_sign(
        &self,
        expression: &BezierAlgebraicCuspTrivariateTwoSquareRootExpression2,
        target_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let Some([first_parameter, second_parameter]) = self.compact_source_parameters() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        algebraic_cusp_trivariate_two_square_root_sum_sign(
            expression,
            &self.pair_discriminant,
            &self.candidate_speed_squared,
            &first_parameter,
            &second_parameter,
            target_parameter,
            self.branch,
            policy,
        )
    }

    fn expression_parameters(
        &self,
        domain: SelectedThirdAxisDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        let Some([first_parameter, second_parameter]) = self.compact_source_parameters() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        selected_projected_trivariate_third_axis_parameters(
            &self.incidence_projection,
            &first_parameter,
            &second_parameter,
            domain,
            policy,
            |candidate, projected_root_certified| {
                algebraic_cusp_projected_trivariate_two_square_root_sum_sign(
                    &self.incidence,
                    Some(&self.incidence_candidate_norm),
                    &self.pair_discriminant,
                    &self.candidate_speed_squared,
                    &self.incidence_projection,
                    &first_parameter,
                    &second_parameter,
                    candidate,
                    self.branch,
                    projected_root_certified,
                    policy,
                )
            },
        )
    }

    fn parameters_with_incident_domain(
        &self,
        domain: CurveParameterDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        selected_axis_parameters_in_domain(domain, policy, |domain| {
            self.expression_parameters(domain, policy)
        })
    }

    fn target_is_regular(
        &self,
        target_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        match signed_coefficients_at_parameter(&self.target_weight, target_parameter, policy)? {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Decided(false));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        Ok(
            match signed_coefficients_at_parameter(
                &self.target_speed_squared,
                target_parameter,
                policy,
            )? {
                Classification::Decided(RealSign::Positive) => Classification::Decided(true),
                Classification::Decided(RealSign::Zero) => Classification::Decided(false),
                Classification::Decided(RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "a pair-radial circle/parallel target had negative speed squared".into(),
                    ));
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    fn contact_location(
        &self,
        target_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleContactLocation2>>> {
        let selected =
            match self.expression_sign(&self.selected_half_plane, target_parameter, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        Ok(match selected {
            RealSign::Negative => Classification::Decided(None),
            RealSign::Positive => Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            )),
            RealSign::Zero => {
                match self.expression_sign(&self.diameter, target_parameter, policy)? {
                    Classification::Decided(RealSign::Positive) => Classification::Decided(Some(
                        BezierAlgebraicCuspSemicircleContactLocation2::Start,
                    )),
                    Classification::Decided(RealSign::Negative) => Classification::Decided(Some(
                        BezierAlgebraicCuspSemicircleContactLocation2::End,
                    )),
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a pair-radial nonzero circle contact had zero local diameter".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }
            }
        })
    }

    fn tangent_cross_dot_source_sign(
        &self,
        target_parameter: &BezierParameter2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let Some(expression) = (|| {
            Some(BezierAlgebraicCuspTrivariateTwoSquareRootExpression2 {
                product: TrivariatePolynomial::linear_combination(&[
                    (&self.tangent_cross_source.radical, cross_scale),
                    (&self.tangent_dot_source.product, dot_scale),
                ])?,
                pair: self.tangent_dot_source.pair.scale(dot_scale)?,
                candidate: TrivariatePolynomial::linear_combination(&[
                    (&self.tangent_cross_source.rational, cross_scale),
                    (&self.tangent_dot_source.candidate, dot_scale),
                ])?,
                rational: self.tangent_dot_source.rational.scale(dot_scale)?,
            })
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        self.expression_sign(&expression, target_parameter, policy)
    }

    fn diameter_parameter_sign(
        &self,
        target_parameter: &BezierParameter2,
        denominator: &Real,
        radial_coefficient: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let Some(expression) = self.diameter.scale(denominator).and_then(|diameter| {
            self.radius_squared_denominator
                .scale(radial_coefficient)
                .and_then(|radius| diameter.subtract(&radius))
        }) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        self.expression_sign(&expression, target_parameter, policy)
    }
}

impl BezierRepresentedCircleParallelSystem2 {
    fn sources_with_target(
        &self,
        target_parameter: &BezierParameter2,
    ) -> Vec<AlgebraicRootRepresentation> {
        let mut sources = self.sources.clone();
        sources.push(bezier_parameter_root_representation(target_parameter));
        sources
    }

    fn polynomial_sign(
        &self,
        polynomial: &DenseTensorPolynomial,
        target_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        dense_polynomial_tuple_sign(
            polynomial,
            &self.sources_with_target(target_parameter),
            policy,
        )
    }

    fn expression_sign(
        &self,
        expression: &BezierRepresentedCircleParallelExpression2,
        target_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        dense_positive_square_root_sum_sign(
            &expression.rational,
            &expression.radical,
            &self.speed_squared,
            &self.sources_with_target(target_parameter),
            policy,
        )
    }

    fn parameters_with_incident_domain(
        &self,
        polynomial: &DenseTensorPolynomial,
        domain: CurveParameterDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        selected_dense_last_axis_parameters_with_incident_domain(
            polynomial,
            &self.sources,
            domain,
            policy,
        )
    }

    fn target_is_regular(
        &self,
        target_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        match self.polynomial_sign(&self.weight, target_parameter, policy)? {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Decided(false));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        Ok(
            match self.polynomial_sign(&self.speed_squared, target_parameter, policy)? {
                Classification::Decided(RealSign::Positive) => Classification::Decided(true),
                Classification::Decided(RealSign::Zero) => Classification::Decided(false),
                Classification::Decided(RealSign::Negative) => {
                    return Err(CurveError::Topology(
                        "a represented circle/parallel target had negative speed squared".into(),
                    ));
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    fn contact_location(
        &self,
        target_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicCuspSemicircleContactLocation2>>> {
        let selected =
            match self.expression_sign(&self.selected_half_plane, target_parameter, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        Ok(match selected {
            RealSign::Negative => Classification::Decided(None),
            RealSign::Positive => Classification::Decided(Some(
                BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            )),
            RealSign::Zero => {
                match self.expression_sign(&self.diameter, target_parameter, policy)? {
                    Classification::Decided(RealSign::Positive) => Classification::Decided(Some(
                        BezierAlgebraicCuspSemicircleContactLocation2::Start,
                    )),
                    Classification::Decided(RealSign::Negative) => Classification::Decided(Some(
                        BezierAlgebraicCuspSemicircleContactLocation2::End,
                    )),
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a represented nonzero circle contact had zero local diameter".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }
            }
        })
    }

    fn tangent_cross_dot_source_sign(
        &self,
        target_parameter: &BezierParameter2,
        cross_scale: &Real,
        dot_scale: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let Some(expression) = (|| {
            Some(BezierRepresentedCircleParallelExpression2 {
                rational: self.tangent_dot_source.rational.scale(dot_scale)?,
                radical: self
                    .tangent_cross_source
                    .scale(cross_scale)?
                    .add(&self.tangent_dot_source.radical.scale(dot_scale)?)?,
            })
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        self.expression_sign(&expression, target_parameter, policy)
    }

    fn diameter_parameter_sign(
        &self,
        target_parameter: &BezierParameter2,
        denominator: &Real,
        radial_coefficient: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let Some(expression) = (|| {
            Some(BezierRepresentedCircleParallelExpression2 {
                rational: self.diameter.rational.scale(denominator)?.subtract(
                    &self
                        .radius_squared_denominator
                        .rational
                        .scale(radial_coefficient)?,
                )?,
                radical: self.diameter.radical.scale(denominator)?.subtract(
                    &self
                        .radius_squared_denominator
                        .radical
                        .scale(radial_coefficient)?,
                )?,
            })
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        self.expression_sign(&expression, target_parameter, policy)
    }
}

impl BezierChordNormalDenseChordParameterMapSystem2 {
    fn recursive_contact_field(
        &self,
        contact: &BezierAlgebraicCuspSemicircleChordContact2,
    ) -> CurveResult<Option<BezierRecursiveQuadraticField2>> {
        let index = match contact.branch {
            -1 => 0,
            0 => 1,
            1 => 2,
            _ => {
                return Err(CurveError::Topology(
                    "a dense chord contact retained an invalid radical branch".into(),
                ));
            }
        };
        let slot = &self.recursive_contact_fields[index];
        if let Some(field) = slot.get() {
            return Ok(Some(field.clone()));
        }
        let target = contact.projective_parameter.as_ref().ok_or_else(|| {
            CurveError::Topology(
                "a dense chord contact lost its target parameter while entering the recursive field"
                    .into(),
            )
        })?;
        let Some(field) = BezierRecursiveQuadraticField2::base(
            self.projective.sources_with_target(target),
            self.projective.first_speed_squared.clone(),
            self.projective.second_speed_squared.clone(),
        ) else {
            return Ok(None);
        };
        let _ = slot.set(field);
        Ok(slot.get().cloned())
    }
}

impl BezierChordNormalDenseMapSystem2 {
    fn sources_with_target(
        &self,
        target_parameter: &BezierParameter2,
    ) -> Vec<AlgebraicRootRepresentation> {
        let mut sources = self.source_representations.clone();
        sources.push(bezier_parameter_root_representation(target_parameter));
        sources
    }

    fn expression_sign(
        &self,
        expression: &BezierDenseTwoSquareRootExpression2,
        target_parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let strict = &CurveContext::STRICT;
        let mut target_refinement = BezierParameterRefinement2::new(target_parameter, strict);
        let mut refined_sources = None;
        for refinement_steps in [0_usize, 2, 4, 8, 16, 32, 64] {
            let mut sources = self
                .source_representations
                .iter()
                .map(|source| refined_represented_root(source, refinement_steps))
                .collect::<Vec<_>>();
            sources.push(bezier_parameter_root_representation(
                target_refinement.refine_to(refinement_steps),
            ));
            if let Some(sign) = dense_two_positive_square_root_interval(
                expression,
                &self.first_speed_squared,
                &self.second_speed_squared,
                &sources,
            )
            .as_ref()
            .and_then(dense_strict_interval_sign)
            {
                return Ok(Classification::Decided(sign));
            }
            refined_sources = Some(sources);
        }
        dense_two_positive_square_root_sum_sign(
            expression,
            &self.first_speed_squared,
            &self.second_speed_squared,
            &refined_sources.unwrap_or_else(|| self.sources_with_target(target_parameter)),
            policy,
        )
    }
}

fn dense_expression_last_axis_degree(
    expression: &BezierDenseTwoSquareRootExpression2,
) -> Option<usize> {
    [
        &expression.rational,
        &expression.first,
        &expression.second,
        &expression.product,
    ]
    .into_iter()
    .map(|polynomial| polynomial.dimensions().last().copied())
    .collect::<Option<Vec<_>>>()?
    .into_iter()
    .max()
    .and_then(|count| count.checked_sub(1))
}

/// Converts one coefficient of the final affine-parameter axis into the
/// existing retained two-radical base field. The unit final axis is removed
/// structurally; no source root is eliminated or materialized.
fn recursive_quadratic_base_expression_coefficient(
    expression: &BezierDenseTwoSquareRootExpression2,
    power: usize,
    base: &Arc<BezierRecursiveQuadraticBaseFieldData2>,
) -> Option<BezierRecursiveQuadraticValue2> {
    let coefficient = |polynomial: &DenseTensorPolynomial| {
        let coefficient = dense_last_axis_coefficient(polynomial, power)?;
        let axis = coefficient.dimensions().len().checked_sub(1)?;
        coefficient.remove_certified_independent_axis(
            axis,
            hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION,
        )
    };
    BezierRecursiveQuadraticValue2::from_base(
        base.clone(),
        BezierDenseTwoSquareRootExpression2 {
            rational: coefficient(&expression.rational)?,
            first: coefficient(&expression.first)?,
            second: coefficient(&expression.second)?,
            product: coefficient(&expression.product)?,
        },
    )
}

/// Evaluates `E(n/d)` homogeneously as `d^degree E(n/d)` in a retained
/// recursive field. The caller supplies one common degree when expressions
/// must later be added, so their positive projective scales remain identical.
fn recursive_quadratic_expression_projective_numerator(
    expression: &BezierDenseTwoSquareRootExpression2,
    base: &Arc<BezierRecursiveQuadraticBaseFieldData2>,
    field: &BezierRecursiveQuadraticField2,
    numerator: &BezierRecursiveQuadraticValue2,
    denominator: &BezierRecursiveQuadraticValue2,
    degree: usize,
) -> Option<BezierRecursiveQuadraticValue2> {
    let coefficient = |power| {
        field.lift(&recursive_quadratic_base_expression_coefficient(
            expression, power, base,
        )?)
    };
    let mut value = coefficient(degree)?;
    let mut denominator_power = field.constant(Real::one())?;
    for power in (0..degree).rev() {
        denominator_power = denominator_power.multiply(denominator)?;
        value = value
            .multiply(numerator)?
            .add(&coefficient(power)?.multiply(&denominator_power)?)?;
    }
    Some(value)
}

fn chord_normal_dense_expression_is_identically_zero(
    map: &BezierChordNormalDenseMapSystem2,
    expression: &BezierDenseTwoSquareRootExpression2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let target_count = [
        expression
            .rational
            .dimensions()
            .last()
            .copied()
            .unwrap_or(1),
        expression.first.dimensions().last().copied().unwrap_or(1),
        expression.second.dimensions().last().copied().unwrap_or(1),
        expression.product.dimensions().last().copied().unwrap_or(1),
    ]
    .into_iter()
    .max()
    .unwrap_or(1);
    for power in 0..target_count {
        let Some(coefficient) = (|| {
            Some(BezierDenseTwoSquareRootExpression2 {
                rational: dense_last_axis_coefficient(&expression.rational, power)?,
                first: dense_last_axis_coefficient(&expression.first, power)?,
                second: dense_last_axis_coefficient(&expression.second, power)?,
                product: dense_last_axis_coefficient(&expression.product, power)?,
            })
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        match map.expression_sign(&coefficient, &BezierParameter2::Exact(Real::zero()), policy)? {
            Classification::Decided(RealSign::Zero) => {}
            Classification::Decided(RealSign::Negative | RealSign::Positive) => {
                return Ok(Classification::Decided(false));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(true))
}

fn dense_two_positive_square_root_transverse_root(
    expression: &BezierDenseTwoSquareRootExpression2,
    first_speed_squared: &DenseTensorPolynomial,
    second_speed_squared: &DenseTensorPolynomial,
    source_representations: &[AlgebraicRootRepresentation],
    target_parameter: &BezierParameter2,
) -> Option<bool> {
    let strict = &CurveContext::STRICT;
    let mut target_refinement = BezierParameterRefinement2::new(target_parameter, strict);
    for target_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let source_steps = target_steps.saturating_add(64);
        let mut sources = source_representations
            .iter()
            .map(|source| refined_represented_root(source, source_steps))
            .collect::<Vec<_>>();
        let target =
            bezier_parameter_root_representation(target_refinement.refine_to(target_steps));
        sources.push(target.clone());
        let value = dense_two_positive_square_root_interval(
            expression,
            first_speed_squared,
            second_speed_squared,
            &sources,
        );
        if matches!(
            value.as_ref().and_then(dense_strict_interval_sign),
            Some(RealSign::Negative | RealSign::Positive)
        ) {
            return Some(false);
        }
        let mut lower_sources = sources.clone();
        let mut upper_sources = sources;
        *lower_sources.last_mut()? =
            AlgebraicRootRepresentation::from_exact_value(&target.interval.lower);
        *upper_sources.last_mut()? =
            AlgebraicRootRepresentation::from_exact_value(&target.interval.upper);
        let lower = dense_two_positive_square_root_interval(
            expression,
            first_speed_squared,
            second_speed_squared,
            &lower_sources,
        )
        .as_ref()
        .and_then(dense_strict_interval_sign);
        let upper = dense_two_positive_square_root_interval(
            expression,
            first_speed_squared,
            second_speed_squared,
            &upper_sources,
        )
        .as_ref()
        .and_then(dense_strict_interval_sign);
        if strict_signs_are_opposite(lower, upper) {
            return Some(true);
        }
    }
    None
}

fn dense_positive_square_root_transverse_root(
    rational: &DenseTensorPolynomial,
    radical: &DenseTensorPolynomial,
    radicand: &DenseTensorPolynomial,
    source_representations: &[AlgebraicRootRepresentation],
    target_parameter: &BezierParameter2,
) -> Option<bool> {
    let strict = &CurveContext::STRICT;
    let mut target_refinement = BezierParameterRefinement2::new(target_parameter, strict);
    for target_steps in [0_usize, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let source_steps = target_steps.saturating_add(64);
        let mut sources = source_representations
            .iter()
            .map(|source| refined_represented_root(source, source_steps))
            .collect::<Vec<_>>();
        let target =
            bezier_parameter_root_representation(target_refinement.refine_to(target_steps));
        sources.push(target.clone());
        let value = dense_positive_square_root_interval(rational, radical, radicand, &sources);
        if matches!(
            value.as_ref().and_then(dense_strict_interval_sign),
            Some(RealSign::Negative | RealSign::Positive)
        ) {
            return Some(false);
        }
        let mut lower_sources = sources.clone();
        let mut upper_sources = sources;
        *lower_sources.last_mut()? =
            AlgebraicRootRepresentation::from_exact_value(&target.interval.lower);
        *upper_sources.last_mut()? =
            AlgebraicRootRepresentation::from_exact_value(&target.interval.upper);
        let lower =
            dense_positive_square_root_interval(rational, radical, radicand, &lower_sources)
                .as_ref()
                .and_then(dense_strict_interval_sign);
        let upper =
            dense_positive_square_root_interval(rational, radical, radicand, &upper_sources)
                .as_ref()
                .and_then(dense_strict_interval_sign);
        if strict_signs_are_opposite(lower, upper) {
            return Some(true);
        }
    }
    None
}

fn chord_normal_dense_expression_parameters(
    map: &BezierChordNormalDenseMapSystem2,
    expression: &BezierDenseTwoSquareRootExpression2,
    domain: SelectedThirdAxisDomain2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
    let Some(projection) = expression.projection(
        &map.first_speed_squared,
        &map.second_speed_squared,
        &map.source_representations,
    ) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let candidates = match selected_dense_last_axis_parameters(
        &projection,
        &map.source_representations,
        domain,
        policy,
    )? {
        Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(candidates)) => {
            candidates
        }
        Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero) => {
            return Ok(
                match chord_normal_dense_expression_is_identically_zero(map, expression, policy)? {
                    Classification::Decided(true) => {
                        Classification::Decided(BezierAlgebraicFiberProjection2::IdenticallyZero)
                    }
                    Classification::Decided(false) => {
                        Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate)
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                },
            );
        }
        Classification::Decided(BezierAlgebraicFiberProjection2::Degenerate) => {
            return Ok(Classification::Decided(
                BezierAlgebraicFiberProjection2::Degenerate,
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let mut retained = Vec::with_capacity(candidates.len());
    for candidate in candidates {
        let transverse = dense_two_positive_square_root_transverse_root(
            expression,
            &map.first_speed_squared,
            &map.second_speed_squared,
            &map.source_representations,
            &candidate,
        );
        match transverse {
            Some(true) => retained.push(candidate),
            Some(false) => {}
            None => {
                let sign = map.expression_sign(expression, &candidate, policy)?;
                match sign {
                    Classification::Decided(RealSign::Zero) => retained.push(candidate),
                    Classification::Decided(RealSign::Negative | RealSign::Positive) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
    }
    Ok(Classification::Decided(
        BezierAlgebraicFiberProjection2::Parameters(retained),
    ))
}

fn selected_radial_chord_parameters(
    system: &BezierSelectedRadialCircleChordParameterMapSystem2,
) -> Option<[BezierParameter2; 4]> {
    let [first, second] = system.pair_map.compact_source_parameters()?;
    Some([
        first,
        second,
        system.first_parameter.clone(),
        system.second_parameter.clone(),
    ])
}

fn selected_radial_chord_pair_expression_sign(
    system: &BezierSelectedRadialCircleChordParameterMapSystem2,
    expression: &BezierAlgebraicCuspQuadrivariateSquareRootExpression2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let Some(owned_parameters) = selected_radial_chord_parameters(system) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let parameters = [
        &owned_parameters[0],
        &owned_parameters[1],
        &owned_parameters[2],
        &owned_parameters[3],
    ];
    let rational = match quadrivariate_parameter_tuple_sign_by_refinement(
        &expression.rational,
        parameters,
        policy,
    )? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if system.pair_branch == 0 {
        return Ok(Classification::Decided(rational));
    }
    let radical = match quadrivariate_parameter_tuple_sign_by_refinement(
        &expression.radical,
        parameters,
        policy,
    )? {
        Classification::Decided(sign) if system.pair_branch < 0 => match sign {
            RealSign::Negative => RealSign::Positive,
            RealSign::Zero => RealSign::Zero,
            RealSign::Positive => RealSign::Negative,
        },
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    match (rational, radical) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
            return Ok(Classification::Decided(sign));
        }
        (first, second) if first == second => {
            return Ok(Classification::Decided(first));
        }
        _ => {}
    }
    let radical_squared = expression
        .radical
        .multiply(&expression.radical)
        .ok_or_else(|| {
            CurveError::Topology("a pair-radical magnitude exceeded its tensor budget".into())
        })?;
    let Some(magnitude) = QuadrivariatePolynomial2::sum_products(&[
        (&expression.rational, &expression.rational, false),
        (&radical_squared, &system.pair_discriminant, true),
    ]) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    Ok(
        match quadrivariate_parameter_tuple_sign_by_refinement(&magnitude, parameters, policy)? {
            Classification::Decided(RealSign::Positive) => Classification::Decided(rational),
            Classification::Decided(RealSign::Negative) => Classification::Decided(radical),
            Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

fn selected_radial_chord_nested_expression_sign(
    system: &BezierSelectedRadialCircleChordParameterMapSystem2,
    expression: &BezierSelectedRadialCircleChordNestedExpression2,
    branch: i8,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    debug_assert!((-1..=1).contains(&branch));
    let retained =
        match selected_radial_chord_pair_expression_sign(system, &expression.retained, policy)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    if branch == 0 {
        return Ok(Classification::Decided(retained));
    }
    let candidate =
        match selected_radial_chord_pair_expression_sign(system, &expression.candidate, policy)? {
            Classification::Decided(sign) if branch < 0 => match sign {
                RealSign::Negative => RealSign::Positive,
                RealSign::Zero => RealSign::Zero,
                RealSign::Positive => RealSign::Negative,
            },
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    match (retained, candidate) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
            return Ok(Classification::Decided(sign));
        }
        (first, second) if first == second => {
            return Ok(Classification::Decided(first));
        }
        _ => {}
    }
    let Some(magnitude) = expression
        .retained
        .square(&system.pair_discriminant)
        .and_then(|retained_squared| {
            expression
                .candidate
                .square(&system.pair_discriminant)
                .and_then(|candidate_squared| {
                    candidate_squared
                        .multiply(&system.chord_discriminant, &system.pair_discriminant)
                        .and_then(|candidate_term| retained_squared.subtract(&candidate_term))
                })
        })
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    Ok(
        match selected_radial_chord_pair_expression_sign(system, &magnitude, policy)? {
            Classification::Decided(RealSign::Positive) => Classification::Decided(retained),
            Classification::Decided(RealSign::Negative) => Classification::Decided(candidate),
            Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

fn retained_offset_chord_speed_expression_sign(
    system: &BezierAlgebraicCuspSemicircleRetainedOffsetChordParameterMapSystem2,
    expression: &BezierAlgebraicCuspTrivariateSquareRootExpression2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    algebraic_cusp_trivariate_square_root_components_sign(
        &expression.rational,
        &expression.radical,
        &system.speed_squared,
        &system.first_parameter,
        &system.second_parameter,
        &system.cusp_parameter,
        1,
        policy,
    )
}

fn retained_offset_chord_nested_expression_sign(
    system: &BezierAlgebraicCuspSemicircleRetainedOffsetChordParameterMapSystem2,
    expression: &BezierAlgebraicCuspRetainedOffsetChordNestedExpression2,
    branch: i8,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    debug_assert!((-1..=1).contains(&branch));
    let retained =
        match retained_offset_chord_speed_expression_sign(system, &expression.retained, policy)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    if branch == 0 {
        return Ok(Classification::Decided(retained));
    }
    let candidate =
        match retained_offset_chord_speed_expression_sign(system, &expression.candidate, policy)? {
            Classification::Decided(sign) if branch < 0 => product_sign(sign, RealSign::Negative),
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    match (retained, candidate) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
            return Ok(Classification::Decided(sign));
        }
        (first, second) if first == second => {
            return Ok(Classification::Decided(first));
        }
        _ => {}
    }
    let Some(magnitude) =
        expression
            .retained
            .square(&system.speed_squared)
            .and_then(|retained_squared| {
                expression
                    .candidate
                    .square(&system.speed_squared)
                    .and_then(|candidate_squared| {
                        candidate_squared
                            .multiply(&system.contact_discriminant, &system.speed_squared)
                            .and_then(|candidate_term| retained_squared.subtract(&candidate_term))
                    })
            })
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    Ok(
        match retained_offset_chord_speed_expression_sign(system, &magnitude, policy)? {
            Classification::Decided(RealSign::Positive) => Classification::Decided(retained),
            Classification::Decided(RealSign::Negative) => Classification::Decided(candidate),
            Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

fn algebraic_point_oriented_line_side_distinct_fields(
    start: &RationalBezierAlgebraicPointPredicate2<'_>,
    end: &RationalBezierAlgebraicPointPredicate2<'_>,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<crate::classify::LineSide>> {
    let one = Real::one();
    let zero = Real::zero();
    for endpoint in [start, end] {
        let x = signed_algebraic_point_linear_difference(endpoint, point, &one, &zero, policy)?;
        if x == Classification::Decided(RealSign::Zero)
            && signed_algebraic_point_linear_difference(endpoint, point, &zero, &one, policy)?
                == Classification::Decided(RealSign::Zero)
        {
            return Ok(Classification::Decided(crate::classify::LineSide::On));
        }
    }

    let (start_x, start_y, start_denominator) = start.coordinate_polynomials();
    let (end_x, end_y, end_denominator) = end.coordinate_polynomials();
    let (point_x, point_y, point_denominator) = point.coordinate_polynomials();
    let line_x = bivariate_subtract(
        &bivariate_outer_product(start_denominator, end_x),
        &bivariate_outer_product(start_x, end_denominator),
    );
    let line_y = bivariate_subtract(
        &bivariate_outer_product(start_denominator, end_y),
        &bivariate_outer_product(start_y, end_denominator),
    );
    let point_delta_x = bivariate_subtract(
        &bivariate_outer_product(start_denominator, point_x),
        &bivariate_outer_product(start_x, point_denominator),
    );
    let point_delta_y = bivariate_subtract(
        &bivariate_outer_product(start_denominator, point_y),
        &bivariate_outer_product(start_y, point_denominator),
    );
    let Some(determinant) =
        TrivariatePolynomial::ab_ac_determinant(&line_x, &point_delta_y, &line_y, &point_delta_x)
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let denominator_sign = product_sign(end.denominator_sign(), point.denominator_sign());
    Ok(trivariate_parameter_triple_sign_by_refinement(
        &determinant,
        start.retained_parameter(),
        end.retained_parameter(),
        point.retained_parameter(),
        policy,
    )?
    .map(|sign| crate::classify::LineSide::from_real_sign(product_sign(sign, denominator_sign))))
}

fn algebraic_point_oriented_line_side(
    start: &RationalBezierAlgebraicPointPredicate2<'_>,
    end: &RationalBezierAlgebraicPointPredicate2<'_>,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<crate::classify::LineSide>> {
    if start.retained_parameter() != end.retained_parameter() {
        return algebraic_point_oriented_line_side_distinct_fields(start, end, point, policy);
    }
    let (start_x, start_y, start_denominator) = start.coordinate_polynomials();
    let (end_x, end_y, end_denominator) = end.coordinate_polynomials();
    let (point_x, point_y, point_denominator) = point.coordinate_polynomials();

    // Start and end are rational images in one retained selected-root field. Clear
    // that shared field first, leaving one bivariate expression against the
    // independent query field.
    let line_x = polynomial_subtract(
        &polynomial_multiply(end_x, start_denominator),
        &polynomial_multiply(start_x, end_denominator),
    );
    let line_y = polynomial_subtract(
        &polynomial_multiply(end_y, start_denominator),
        &polynomial_multiply(start_y, end_denominator),
    );
    let point_delta_x = bivariate_subtract(
        &bivariate_outer_product(start_denominator, point_x),
        &bivariate_outer_product(start_x, point_denominator),
    );
    let point_delta_y = bivariate_subtract(
        &bivariate_outer_product(start_denominator, point_y),
        &bivariate_outer_product(start_y, point_denominator),
    );
    let determinant = bivariate_subtract(
        &bivariate_multiply_first_parameter(&point_delta_y, &line_x),
        &bivariate_multiply_first_parameter(&point_delta_x, &line_y),
    );
    let denominator_sign = product_sign(end.denominator_sign(), point.denominator_sign());
    Ok(
        selected_bivariate_parameter_pair_sign(&determinant, start, point, policy)?.map(|sign| {
            crate::classify::LineSide::from_real_sign(product_sign(sign, denominator_sign))
        }),
    )
}

fn algebraic_point_circle_residual_sign(
    center: &RationalBezierAlgebraicPointPredicate2<'_>,
    point: &RationalBezierAlgebraicPointPredicate2<'_>,
    radius_squared: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let (center_x, center_y, center_denominator) = center.coordinate_polynomials();
    let (point_x, point_y, point_denominator) = point.coordinate_polynomials();
    let delta_x = bivariate_subtract(
        &bivariate_outer_product(center_denominator, point_x),
        &bivariate_outer_product(center_x, point_denominator),
    );
    let delta_y = bivariate_subtract(
        &bivariate_outer_product(center_denominator, point_y),
        &bivariate_outer_product(center_y, point_denominator),
    );
    let common_denominator = bivariate_outer_product(center_denominator, point_denominator);
    let residual = bivariate_subtract(
        &bivariate_add(
            &bivariate_multiply(&delta_x, &delta_x),
            &bivariate_multiply(&delta_y, &delta_y),
        ),
        &bivariate_scale(
            bivariate_multiply(&common_denominator, &common_denominator),
            radius_squared,
        ),
    );
    // Both affine denominators are squared, so the cleared residual has the
    // physical squared-distance sign regardless of projective orientation.
    selected_bivariate_parameter_pair_sign(&residual, center, point, policy)
}

impl BezierAlgebraicChordParameter2 {
    /// Returns which endpoint of this identical chord the parameter is, if
    /// it is one.
    pub(crate) fn endpoint_of(&self, chord: &BezierAlgebraicChord2) -> Option<bool> {
        match &self.data {
            BezierAlgebraicChordParameterStorage2::Endpoint {
                chord: endpoint_chord,
                at_end,
            } if Arc::ptr_eq(&endpoint_chord.data, &chord.data) => Some(*at_end),
            _ => None,
        }
    }

    /// Returns whether this is the named traversal endpoint of `chord` by
    /// retained identity. No coordinate equality or policy-terminal
    /// approximation participates in this topology query.
    pub(crate) fn is_endpoint_of(&self, chord: &BezierAlgebraicChord2, at_start: bool) -> bool {
        matches!(
            &self.data,
            BezierAlgebraicChordParameterStorage2::Endpoint {
                chord: parameter_chord,
                at_end,
            } if Arc::ptr_eq(&parameter_chord.data, &chord.data) && *at_end == !at_start
        )
    }

    pub(crate) fn chord(&self) -> &BezierAlgebraicChord2 {
        match &self.data {
            BezierAlgebraicChordParameterStorage2::Endpoint { chord, .. } => chord,
            BezierAlgebraicChordParameterStorage2::Interior(data) => &data.chord,
        }
    }

    /// Returns true only when the parameter's constructing kernel certified
    /// both strict inequalities against its retained chord endpoints.
    pub(crate) fn has_certified_strict_interior_location(&self) -> bool {
        matches!(
            &self.data,
            BezierAlgebraicChordParameterStorage2::Interior(data)
                if data.certified_strict_interior
        )
    }

    /// Returns true only when this exact parameter was constructed after the
    /// authoritative pair kernel proved strict containment in this identical
    /// finite chord. A shared infinite support is insufficient because a
    /// later split can have narrower bounds.
    pub(crate) fn is_certified_strict_interior_of(&self, chord: &BezierAlgebraicChord2) -> bool {
        matches!(
            &self.data,
            BezierAlgebraicChordParameterStorage2::Interior(data)
                if data.certified_strict_interior && Arc::ptr_eq(&data.chord.data, &chord.data)
        )
    }

    fn axis(&self) -> BezierAlgebraicChordParameterAxis2 {
        match &self.data {
            BezierAlgebraicChordParameterStorage2::Endpoint { chord, .. } => {
                chord.data.parameter_axis
            }
            BezierAlgebraicChordParameterStorage2::Interior(data) => data.axis,
        }
    }

    pub(crate) fn point(&self) -> &CurvePoint2 {
        match &self.data {
            BezierAlgebraicChordParameterStorage2::Endpoint { chord, at_end } => {
                if *at_end {
                    chord.end()
                } else {
                    chord.start()
                }
            }
            BezierAlgebraicChordParameterStorage2::Interior(data) => &data.point,
        }
    }

    /// Publishes this local chord coordinate as the source parameter of an
    /// exact affine-line parameterization.
    ///
    /// The chord parameter normally retains only point/order evidence: that
    /// is both smaller and stronger for chord topology.  A line-valued
    /// rational or analytic carrier, however, uses the identical normalized
    /// affine coordinate as its source parameter.  Recover that scalar in the
    /// point's existing recursive tower instead of projecting the contact
    /// through a second rational-curve resultant.
    pub(crate) fn exact_line_curve_parameter(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<CurveParameter2>> {
        self.chord().validate_policy(policy)?;
        let Some(line) = self.chord().exact_line() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if self.is_endpoint_of(self.chord(), true) {
            return Ok(Classification::Decided(CurveParameter2::from(
                BezierParameter2::Exact(Real::zero()),
            )));
        }
        if self.is_endpoint_of(self.chord(), false) {
            return Ok(Classification::Decided(CurveParameter2::from(
                BezierParameter2::Exact(Real::one()),
            )));
        }
        if let CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) = self.point() {
            let (map, contact) = point.map_contact();
            if Arc::ptr_eq(&map.data.chord.data, &self.chord().data)
                && let Some(system) = map.recursive_quadratic_line_system()
            {
                return Ok(Classification::Decided(
                    CurveParameter2::from_recursive_projective(
                        system.contact(contact.branch)?.parameter.clone(),
                    ),
                ));
            }
        }
        affine_line_parameter_at_incident_point(&line, self.point(), policy)
    }

    pub(crate) fn cmp_by_refinement(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<std::cmp::Ordering>> {
        if let (
            BezierAlgebraicChordParameterStorage2::Endpoint {
                chord: first,
                at_end: first_at_end,
            },
            BezierAlgebraicChordParameterStorage2::Endpoint {
                chord: second,
                at_end: second_at_end,
            },
        ) = (&self.data, &other.data)
            && Arc::ptr_eq(&first.data, &second.data)
        {
            return Ok(Classification::Decided(first_at_end.cmp(second_at_end)));
        }
        let certified_interior_to_endpoint =
            |interior: &Self, endpoint: &Self| -> Option<std::cmp::Ordering> {
                let chord = endpoint.chord();
                if !interior.is_certified_strict_interior_of(chord) {
                    return None;
                }
                if endpoint.is_endpoint_of(chord, true) {
                    Some(std::cmp::Ordering::Greater)
                } else if endpoint.is_endpoint_of(chord, false) {
                    Some(std::cmp::Ordering::Less)
                } else {
                    None
                }
            };
        if let Some(order) = certified_interior_to_endpoint(self, other) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-parameter-order",
                "authored-strict-interior",
            );
            return Ok(Classification::Decided(order));
        }
        if let Some(order) = certified_interior_to_endpoint(other, self) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-parameter-order",
                "authored-strict-interior",
            );
            return Ok(Classification::Decided(order.reverse()));
        }
        {
            if let (
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(first)),
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(second)),
            ) = (self.point(), other.point())
                && first.same_point(second, policy) == Classification::Decided(true)
            {
                return Ok(Classification::Decided(std::cmp::Ordering::Equal));
            }
            if let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = self.point() {
                return point.cmp_on_chord_to_evidence(self.chord(), other.point(), policy);
            }
            if let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = other.point() {
                return Ok(point
                    .cmp_on_chord_to_evidence(other.chord(), self.point(), policy)?
                    .map(std::cmp::Ordering::reverse));
            }
            if let CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) = self.point() {
                return point.cmp_on_chord_to_parameter(self.chord(), other, policy);
            }
            if let CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) = other.point() {
                return Ok(point
                    .cmp_on_chord_to_parameter(other.chord(), self, policy)?
                    .map(std::cmp::Ordering::reverse));
            }
            if matches!(
                (self.point(), other.point()),
                (
                    CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)),
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)),
                ) | (
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)),
                    CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)),
                )
            ) && policy.strict_predicate_pass(|| self.point().same_point(other.point(), policy))
                == Classification::Decided(true)
            {
                // The concentric-circle and accumulated chord-normal
                // constructions can name the same endpoint without sharing a
                // coordinate field. Consume that exact identity before the
                // monotone-axis comparison, whose independent boxes cannot
                // prove equality under STRICT.
                return Ok(Classification::Decided(std::cmp::Ordering::Equal));
            }
        }
        let axis = self.axis();
        if axis != other.axis() {
            return Err(CurveError::Topology(
                "cannot compare parameters from distinct algebraic-chord domains".into(),
            ));
        }
        let order =
            algebraic_chord_point_coordinate_order(self.point(), other.point(), axis.axis, policy)?;
        Ok(if axis.coordinate_increases {
            order
        } else {
            order.map(std::cmp::Ordering::reverse)
        })
    }
}

fn rational_point_evidence_at_parameter(
    source: &RationalBezier2,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    match exact_contact_point_evidence(source, parameter, policy)? {
        Classification::Decided(point) => Ok(Classification::Decided(point)),
        Classification::Uncertain(UncertaintyReason::Boundary) => {
            Ok(Classification::Uncertain(UncertaintyReason::Boundary))
        }
        Classification::Uncertain(reason) => match parameter {
            BezierParameter2::Algebraic(parameter) => {
                Ok(Classification::Decided(CurvePoint2::from(
                    // These contact callers own a finite source-domain proof.
                    // Coordinate projection may be unavailable, but a certified
                    // zero denominator above must never be replaced by this source.
                    RationalBezierAlgebraicPointImage2::from_parametric_source(
                        source.clone(),
                        parameter.clone(),
                        policy,
                    ),
                )))
            }
            BezierParameter2::Exact(_) => Ok(Classification::Uncertain(reason)),
        },
    }
}

/// Evaluates a rational source at its native retained parameter carrier.
/// Recursive and selected-fiber parameters remain zero-distance analytic
/// points so their defining field, branch, and parameter identity survive
/// later finite-chord and Boolean predicates.
fn rational_point_evidence_at_region_parameter(
    source: &RationalBezier2,
    parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    if let Some(parameter @ BezierParameter2::Algebraic(_)) = parameter.as_bezier_parameter() {
        return rational_point_evidence_at_parameter(source, parameter, policy);
    }
    // Even a represented parameter may be irrational. Keep the source map
    // until a predicate has compared its coefficients with the other point;
    // eager coordinate evaluation can hide exact cancellations with opaque
    // Real coefficients in the rationalized parallel component.
    let parallel = source.parallel_left(Real::zero())?;
    Ok(
        BezierAnalyticParallelPoint2::new_with_region_parameter_and_tangent_distance(
            parallel,
            parameter,
            Real::zero(),
            policy,
        )
        .map(CurvePoint2::from)
        .map(Classification::Decided)
        .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
    )
}

fn sort_and_dedup_collinear_partition_boundaries(
    mut boundaries: Vec<BezierAlgebraicChordRationalPartitionBoundary2>,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<BezierAlgebraicChordRationalPartitionBoundary2>>> {
    for index in 1..boundaries.len() {
        let mut cursor = index;
        while cursor > 0 {
            match boundaries[cursor]
                .source_parameter
                .cmp_by_refinement(&boundaries[cursor - 1].source_parameter, policy)?
            {
                Classification::Decided(std::cmp::Ordering::Less) => {
                    boundaries.swap(cursor, cursor - 1);
                    cursor -= 1;
                }
                Classification::Decided(_) => break,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }
    let mut unique: Vec<BezierAlgebraicChordRationalPartitionBoundary2> =
        Vec::with_capacity(boundaries.len());
    for boundary in boundaries {
        let Some(previous) = unique.last() else {
            unique.push(boundary);
            continue;
        };
        match boundary
            .source_parameter
            .cmp_by_refinement(&previous.source_parameter, policy)?
        {
            Classification::Decided(std::cmp::Ordering::Equal) => {
                let retained_endpoint = previous.chord_endpoint_at_end;
                match (retained_endpoint, boundary.chord_endpoint_at_end) {
                    (Some(first), Some(second)) if first != second => {
                        return Err(CurveError::Topology(
                            "one source parameter represented both endpoints of a nondegenerate chord"
                                .into(),
                        ));
                    }
                    (None, Some(_)) => {
                        unique
                            .last_mut()
                            .expect("the retained duplicate boundary exists")
                            .chord_endpoint_at_end = boundary.chord_endpoint_at_end;
                    }
                    _ => {}
                }
            }
            Classification::Decided(std::cmp::Ordering::Greater) => unique.push(boundary),
            Classification::Decided(std::cmp::Ordering::Less) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    Ok(Classification::Decided(unique))
}

impl BezierAlgebraicChordRationalContact2 {
    pub(crate) fn chord_parameter(&self) -> &BezierAlgebraicChordParameter2 {
        &self.chord_parameter
    }

    pub(crate) fn other_parameter(&self) -> &CurveParameter2 {
        &self.other_parameter
    }

    pub(crate) fn point(&self) -> &CurvePoint2 {
        &self.point
    }

    pub(crate) const fn tangent_cross_sign(&self) -> RealSign {
        self.tangent_cross_sign
    }
}

impl BezierAlgebraicChordParallelContact2 {
    pub(crate) fn chord_parameter(&self) -> &BezierAlgebraicChordParameter2 {
        &self.chord_parameter
    }

    pub(crate) fn parallel_parameter(&self) -> &CurveParameter2 {
        &self.parallel_parameter
    }

    pub(crate) fn point(&self) -> &CurvePoint2 {
        &self.point
    }

    pub(crate) const fn tangent_cross_sign(&self) -> RealSign {
        self.tangent_cross_sign
    }

    pub(crate) const fn tangent_dot_sign(&self) -> RealSign {
        self.tangent_dot_sign
    }
}

impl BezierAlgebraicChordRetainedParallelContact2 {
    pub(crate) fn chord_parameter(&self) -> &BezierAlgebraicChordParameter2 {
        &self.chord_parameter
    }

    pub(crate) fn parallel_parameter(&self) -> &CurveParameter2 {
        &self.parallel_parameter
    }

    pub(crate) fn point(&self) -> &CurvePoint2 {
        &self.point
    }

    pub(crate) const fn tangent_cross_sign(&self) -> RealSign {
        self.tangent_cross_sign
    }
}

impl BezierAlgebraicChordPairContact2 {
    pub(crate) fn first_parameter(&self) -> &BezierAlgebraicChordParameter2 {
        &self.first_parameter
    }

    pub(crate) fn second_parameter(&self) -> &BezierAlgebraicChordParameter2 {
        &self.second_parameter
    }

    pub(crate) fn point(&self) -> &CurvePoint2 {
        &self.point
    }

    pub(crate) const fn tangent_cross_sign(&self) -> RealSign {
        self.tangent_cross_sign
    }
}

impl BezierAlgebraicChordPairOverlap2 {
    pub(crate) fn first_range(&self) -> [&BezierAlgebraicChordParameter2; 2] {
        [&self.first_range[0], &self.first_range[1]]
    }

    pub(crate) fn second_range(&self) -> [&BezierAlgebraicChordParameter2; 2] {
        [&self.second_range[0], &self.second_range[1]]
    }

    pub(crate) const fn orientation(&self) -> CurveOverlapOrientation2 {
        self.orientation
    }
}

impl BezierAlgebraicChordRationalOverlap2 {
    /// The caller has certified a finite monotone rational branch on this chord.
    /// Both ranges must retain corresponding endpoints, in the same order.
    pub(crate) fn from_certified_ranges(
        chord: BezierAlgebraicChord2,
        source: RationalBezier2,
        chord_range: [BezierAlgebraicChordParameter2; 2],
        source_range: CurveParameterRange2,
        orientation: CurveOverlapOrientation2,
    ) -> Self {
        Self {
            chord,
            source,
            chord_range,
            source_range,
            orientation,
        }
    }

    /// Clips either operand through the original monotone correspondence.
    pub(crate) fn clipped_ranges(
        &self,
        chord_range: &CurveParameterRange2,
        source_range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(CurveParameterRange2, CurveParameterRange2)>>> {
        let original_chord = CurveParameterRange2::new_validated(
            CurveParameter2::from_algebraic_chord(self.chord_range[0].clone()),
            CurveParameter2::from_algebraic_chord(self.chord_range[1].clone()),
        );
        crate::bezier_split::clip_corresponding_parameter_ranges(
            &original_chord,
            &self.source_range,
            chord_range,
            source_range,
            policy,
            |parameter| {
                self.source_parameter_at_chord_parameter(
                    parameter
                        .as_algebraic_chord()
                        .ok_or(CurveError::InvalidCurveParameter)?,
                    policy,
                )
            },
            |parameter| {
                Ok(self
                    .chord_parameter_at_source_parameter(parameter, policy)?
                    .map(|parameter| parameter.map(CurveParameter2::from_algebraic_chord)))
            },
        )
    }

    pub(crate) fn source_parameter_at_chord_parameter(
        &self,
        parameter: &BezierAlgebraicChordParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        let mut orders = [std::cmp::Ordering::Equal; 2];
        for (index, (chord, source)) in [
            (&self.chord_range[0], self.source_range.start()),
            (&self.chord_range[1], self.source_range.end()),
        ]
        .into_iter()
        .enumerate()
        {
            match parameter.cmp_by_refinement(chord, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(Some(source.clone())));
                }
                Classification::Decided(order) => orders[index] = order,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        if orders[0] == orders[1] {
            return Ok(Classification::Decided(None));
        }
        let domain = CurveParameterDomain2::new(&self.source_range, None);
        if let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = parameter.point()
            && let BezierParallelSource2::Rational(source) = point.data.parallel.source()
            && source == &self.source
            && [
                point.data.parallel.distance(),
                &point.data.tangent_distance,
                &point.data.translation_x,
                &point.data.translation_y,
            ]
            .into_iter()
            .all(|value| value == &Real::zero())
        {
            let retained = match &point.data.parameter {
                BezierAnalyticParallelPointParameter2::Bezier(parameter) => {
                    CurveParameter2::from(parameter.clone())
                }
                BezierAnalyticParallelPointParameter2::SelectedFiber(parameter) => {
                    CurveParameter2::from_selected_fiber(parameter.clone())
                }
                BezierAnalyticParallelPointParameter2::RecursiveProjective(parameter) => {
                    CurveParameter2::from_recursive_projective(parameter.clone())
                }
            };
            match domain.contains_finite_parameter(&retained, policy)? {
                Classification::Decided(true) => {
                    return Ok(Classification::Decided(Some(retained)));
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        if let Classification::Decided(parameters) =
            self.chord.collinear_source_parameters_at_chord_endpoint(
                &self.source,
                parameter.point(),
                &self.source_range,
                policy,
            )?
        {
            for candidate in parameters {
                match domain.contains_finite_parameter(&candidate, policy)? {
                    Classification::Decided(true) => {
                        return Ok(Classification::Decided(Some(candidate)));
                    }
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        let [low, high] = match self.source_range.ordered_endpoints(policy)? {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let source_order = if self.chord.data.parameter_axis.coordinate_increases
            == (self.orientation == CurveOverlapOrientation2::Same)
        {
            std::cmp::Ordering::Less
        } else {
            std::cmp::Ordering::Greater
        };
        self.chord
            .collinear_monotone_source_parameter_at_chord_endpoint(
                &self.source,
                parameter.point(),
                &[low.clone(), high.clone()],
                source_order,
                policy,
            )
    }

    pub(crate) fn chord_range(&self) -> [&BezierAlgebraicChordParameter2; 2] {
        [&self.chord_range[0], &self.chord_range[1]]
    }

    pub(crate) const fn source_range(&self) -> &CurveParameterRange2 {
        &self.source_range
    }

    pub(crate) const fn orientation(&self) -> CurveOverlapOrientation2 {
        self.orientation
    }

    pub(crate) fn chord_parameter_at_source_parameter(
        &self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierAlgebraicChordParameter2>>> {
        let mut orders = [std::cmp::Ordering::Equal; 2];
        for (index, (source_boundary, chord_boundary)) in [
            (self.source_range.start(), &self.chord_range[0]),
            (self.source_range.end(), &self.chord_range[1]),
        ]
        .into_iter()
        .enumerate()
        {
            match parameter.cmp_by_refinement(source_boundary, policy)? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(Some(chord_boundary.clone())));
                }
                Classification::Decided(order) => orders[index] = order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if orders[0] == orders[1] {
            return Ok(Classification::Decided(None));
        }
        let point =
            match rational_point_evidence_at_region_parameter(&self.source, parameter, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        // The retained monotone correspondence proves that every source
        // parameter strictly between these paired boundaries lies in the
        // finite chord's interior. Reuse that proof instead of reconstructing
        // independent Cartesian fields to rediscover finite membership.
        self.chord.validate_policy(policy)?;
        Ok(Classification::Decided(Some(
            self.chord.parameter_at_certified_interior_point(point),
        )))
    }
}

/// Extracts the affine parameter of a point whose incidence on this line
/// is already certified. Its coordinates stay in their existing shared field.
pub(crate) fn affine_line_parameter_at_incident_point(
    line: &LineSeg2,
    point: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurveParameter2>> {
    let (dx, dy) = line.delta();
    let Some((axis, direction)) =
        [(Axis2::X, dx), (Axis2::Y, dy)]
            .into_iter()
            .find_map(
                |(axis, delta)| match real_sign(&delta, &policy.strict_counterpart()) {
                    Some(direction @ (RealSign::Positive | RealSign::Negative)) => {
                        Some((axis, direction))
                    }
                    _ => None,
                },
            )
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
    };
    if let CurvePoint2(CurvePointData2::Exact(point)) = point {
        let (coordinate, start, delta) = match axis {
            Axis2::X => (
                point.x(),
                line.start().x(),
                line.end().x() - line.start().x(),
            ),
            Axis2::Y => (
                point.y(),
                line.start().y(),
                line.end().y() - line.start().y(),
            ),
        };
        return Ok(Classification::Decided(CurveParameter2::from(
            BezierParameter2::Exact(((coordinate - start) / delta)?),
        )));
    }
    let points = match recursive_projective_evidence_points(&[point], policy)? {
        Classification::Decided(Some(points)) => points,
        Classification::Decided(None) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let point = match positive_recursive_projective_point(
        points
            .into_iter()
            .next()
            .expect("one affine parameter imports one projective point"),
    )? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let (coordinate, start, delta) = match axis {
        Axis2::X => (
            &point.x,
            line.start().x(),
            line.end().x() - line.start().x(),
        ),
        Axis2::Y => (
            &point.y,
            line.start().y(),
            line.end().y() - line.start().y(),
        ),
    };
    let Some(mut numerator) =
        coordinate.subtract(&point.denominator.scale(start).ok_or_else(|| {
            CurveError::Topology(
                "an affine line parameter exceeded its recursive field budget".into(),
            )
        })?)
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let Some(mut denominator) = point.denominator.scale(&delta) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    if direction == RealSign::Negative {
        let negative = Real::from(-1_i8);
        let Some(reversed_numerator) = numerator.scale(&negative) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(reversed_denominator) = denominator.scale(&negative) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        numerator = reversed_numerator;
        denominator = reversed_denominator;
    }
    Ok(BezierRecursiveProjectiveParameter2::new(
        BezierRecursiveQuadraticProjectiveScalar2 {
            numerator,
            denominator,
        },
        policy,
    )?
    .map(CurveParameter2::from_recursive_projective))
}

fn algebraic_chord_parameter_axis(
    start: &CurvePoint2,
    end: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicChordParameterAxis2>> {
    let mut uncertainty = None;
    let mut axes = [Axis2::X, Axis2::Y];
    if let (Classification::Decided(start), Classification::Decided(end)) = (
        algebraic_chord_endpoint_bounds_refined(start, 0, policy),
        algebraic_chord_endpoint_bounds_refined(end, 0, policy),
    ) {
        let midpoint = |bounds: &Aabb2, axis| {
            let (lower, upper) = match axis {
                Axis2::X => (bounds.min().x(), bounds.max().x()),
                Axis2::Y => (bounds.min().y(), bounds.max().y()),
            };
            Some((lower.to_f64_lossy()? + upper.to_f64_lossy()?) * 0.5)
        };
        let separation = |axis| Some((midpoint(&end, axis)? - midpoint(&start, axis)?).abs());
        if matches!(
            (separation(Axis2::X), separation(Axis2::Y)),
            (Some(x), Some(y)) if y > x
        ) {
            // This is scheduling only. The chosen coordinate still has to
            // pass the exact order predicate below; no approximate value is
            // retained by the chord.
            axes.swap(0, 1);
        }
    }
    // Search every coordinate for a strict interval separation before asking
    // either coordinate to prove equality. A nonzero multi-field vector may
    // have one exactly equal component; solving that component first can build
    // a large compositum even though the other component is already separated
    // in the two carriers' native interval fields.
    for refinement_steps in [0, 2, 4, 8, 16, 32, 64] {
        let (Classification::Decided(start_bounds), Classification::Decided(end_bounds)) = (
            algebraic_chord_endpoint_bounds_refined(start, refinement_steps, policy),
            algebraic_chord_endpoint_bounds_refined(end, refinement_steps, policy),
        ) else {
            continue;
        };
        for axis in axes {
            let Some(order @ (std::cmp::Ordering::Less | std::cmp::Ordering::Greater)) =
                algebraic_chord_bounds_axis_order(&start_bounds, &end_bounds, axis)
            else {
                continue;
            };
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-parameter-axis",
                "cross-axis-interval-separated",
            );
            return Ok(Classification::Decided(
                BezierAlgebraicChordParameterAxis2 {
                    axis,
                    coordinate_increases: order == std::cmp::Ordering::Less,
                },
            ));
        }
    }
    for axis in axes {
        match algebraic_chord_point_coordinate_order(start, end, axis, policy)? {
            Classification::Decided(std::cmp::Ordering::Less) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicChordParameterAxis2 {
                        axis,
                        coordinate_increases: true,
                    },
                ));
            }
            Classification::Decided(std::cmp::Ordering::Greater) => {
                return Ok(Classification::Decided(
                    BezierAlgebraicChordParameterAxis2 {
                        axis,
                        coordinate_increases: false,
                    },
                ));
            }
            Classification::Decided(std::cmp::Ordering::Equal) => {}
            Classification::Uncertain(reason) => {
                uncertainty.get_or_insert(reason);
            }
        }
    }
    Ok(uncertainty.map_or(
        Classification::Uncertain(UncertaintyReason::Boundary),
        Classification::Uncertain,
    ))
}

pub(crate) fn algebraic_chord_point_coordinate_order(
    first: &CurvePoint2,
    second: &CurvePoint2,
    axis: Axis2,
    policy: &CurveContext,
) -> CurveResult<Classification<std::cmp::Ordering>> {
    if first.shares_storage(second) {
        return Ok(Classification::Decided(std::cmp::Ordering::Equal));
    }
    // A cardinal support already owns one coordinate of its intersection.
    // Reuse that endpoint for every point family before refining or promoting
    // the intersection's independent Cartesian coordinates.
    for (candidate, other, reverse) in [(first, second, false), (second, first, true)] {
        if let CurvePoint2(CurvePointData2::AlgebraicChordPair(candidate)) = candidate
            && let Classification::Decided(support) =
                candidate.constant_axis_support_point(axis, policy)?
            && let Classification::Decided(order) =
                algebraic_chord_point_coordinate_order(support, other, axis, policy)?
        {
            return Ok(Classification::Decided(if reverse {
                order.reverse()
            } else {
                order
            }));
        }
    }
    if let (
        CurvePoint2(CurvePointData2::AnalyticParallel(first)),
        CurvePoint2(CurvePointData2::AnalyticParallel(second)),
    ) = (first, second)
        && let Some(order) = first.same_zero_distance_source_axis_order(second, axis, policy)?
    {
        return Ok(order);
    }
    if let (
        CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(derived)),
        CurvePoint2(CurvePointData2::AlgebraicChordParallel(parallel)),
    ) = (first, second)
        && let Some(order) =
            derived.selected_chord_normal_parallel_axis_order(parallel, axis, policy)
    {
        return order;
    }
    if let (
        CurvePoint2(CurvePointData2::AlgebraicChordParallel(parallel)),
        CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(derived)),
    ) = (first, second)
        && let Some(order) =
            derived.selected_chord_normal_parallel_axis_order(parallel, axis, policy)
    {
        return Ok(order?.map(std::cmp::Ordering::reverse));
    }
    let cusp_parallel_order = |cusp: &BezierAlgebraicCuspChordPoint2,
                               parallel: &BezierAlgebraicChordParallelPoint2|
     -> CurveResult<Classification<std::cmp::Ordering>> {
        let coordinate = match parallel.exact_axis_coordinate(axis, policy)? {
            Classification::Decided(Some(coordinate)) => coordinate,
            Classification::Decided(None) | Classification::Uncertain(_) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
        };
        let order = cusp.axis_coordinate_order_to_real(axis, &coordinate, policy)?;
        #[cfg(feature = "dispatch-trace")]
        if matches!(order, Classification::Decided(_)) {
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-parallel-axis-order",
                "exact-cardinal-coordinate",
            );
        }
        Ok(order)
    };
    match (first, second) {
        (
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(cusp)),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(parallel)),
        ) => {
            if let order @ Classification::Decided(_) = cusp_parallel_order(cusp, parallel)? {
                return Ok(order);
            }
        }
        (
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(parallel)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(cusp)),
        ) => {
            if let Classification::Decided(order) = cusp_parallel_order(cusp, parallel)? {
                return Ok(Classification::Decided(order.reverse()));
            }
        }
        _ => {}
    }
    if matches!(
        (first, second),
        (
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)),
            CurvePoint2(CurvePointData2::Algebraic(_)),
        ) | (
            CurvePoint2(CurvePointData2::Algebraic(_)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)),
        )
    ) && let (
        Classification::Decided(first_coordinates),
        Classification::Decided(second_coordinates),
    ) = (
        represented_point_evidence_coordinates(first, policy)?,
        represented_point_evidence_coordinates(second, policy)?,
    ) {
        let coordinate = usize::from(axis == Axis2::Y);
        if let Some(order) = represented_strict_order(
            &first_coordinates[coordinate],
            &second_coordinates[coordinate],
        ) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-point-axis-order",
                "direct-represented-coordinate",
            );
            return Ok(Classification::Decided(order));
        }
    }
    if let (
        CurvePoint2(CurvePointData2::AlgebraicChordParallel(first)),
        CurvePoint2(CurvePointData2::AlgebraicChordParallel(second)),
    ) = (first, second)
        && let Some(order) = first.cardinal_axis_order_to_parallel(second, axis, policy)
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-parallel-axis-order",
            "cardinal-frame-cancellation",
        );
        return order;
    }
    if let CurvePoint2(CurvePointData2::AlgebraicChordParallel(displaced)) = first
        && let Some(source) = displaced.strict_preserved_axis_source(axis, policy)
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-parallel-axis-order",
            "preserved-source-coordinate",
        );
        return algebraic_chord_point_coordinate_order(source, second, axis, policy);
    }
    if let CurvePoint2(CurvePointData2::AlgebraicChordParallel(displaced)) = second
        && let Some(source) = displaced.strict_preserved_axis_source(axis, policy)
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-parallel-axis-order",
            "preserved-source-coordinate",
        );
        return algebraic_chord_point_coordinate_order(first, source, axis, policy);
    }
    if let CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(derived)) = first
        && !matches!(
            second,
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
        )
        && let Some(source) = derived.identity_source_point(policy)
    {
        return algebraic_chord_point_coordinate_order(&source, second, axis, policy);
    }
    if let CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(derived)) = second
        && !matches!(
            first,
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
        )
        && let Some(source) = derived.identity_source_point(policy)
    {
        return algebraic_chord_point_coordinate_order(first, &source, axis, policy);
    }
    let use_x = axis == Axis2::X;
    match (first, second) {
        (
            CurvePoint2(CurvePointData2::Exact(first)),
            CurvePoint2(CurvePointData2::Exact(second)),
        ) => Ok(compare_reals(
            if use_x { first.x() } else { first.y() },
            if use_x { second.x() } else { second.y() },
            policy,
        )
        .map_or(
            Classification::Uncertain(UncertaintyReason::Ordering),
            Classification::Decided,
        )),
        (
            CurvePoint2(CurvePointData2::Algebraic(first)),
            CurvePoint2(CurvePointData2::Exact(second)),
        ) => first.coordinate_order_to_real(
            use_x,
            if use_x { second.x() } else { second.y() },
            policy,
        ),
        (
            CurvePoint2(CurvePointData2::Exact(first)),
            CurvePoint2(CurvePointData2::Algebraic(second)),
        ) => Ok(second
            .coordinate_order_to_real(use_x, if use_x { first.x() } else { first.y() }, policy)?
            .map(std::cmp::Ordering::reverse)),
        (
            CurvePoint2(CurvePointData2::Algebraic(first)),
            CurvePoint2(CurvePointData2::Algebraic(second)),
        ) => {
            if first == second {
                return Ok(Classification::Decided(std::cmp::Ordering::Equal));
            }
            {
                let first = match first.predicate_evaluator(policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let second = match second.predicate_evaluator(policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let one = Real::one();
                let zero = Real::zero();
                algebraic_point_linear_order(
                    &first,
                    &second,
                    if use_x { &one } else { &zero },
                    if use_x { &zero } else { &one },
                    policy,
                )
            }
        }
        (
            CurvePoint2(CurvePointData2::AlgebraicChordPair(first)),
            CurvePoint2(CurvePointData2::AlgebraicChordPair(second)),
        ) if first == second => Ok(Classification::Decided(std::cmp::Ordering::Equal)),
        (
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(first)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(second)),
        ) if first == second => Ok(Classification::Decided(std::cmp::Ordering::Equal)),
        (
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(first)),
            CurvePoint2(CurvePointData2::Exact(second)),
        ) => {
            if first
                .map_contact()
                .0
                .recursive_quadratic_line_system()
                .is_some()
            {
                return first.axis_coordinate_order_to_real(
                    axis,
                    if use_x { second.x() } else { second.y() },
                    policy,
                );
            }
            if first.same_point_evidence(&CurvePoint2::from(second.clone()), policy)
                == Classification::Decided(true)
            {
                return Ok(Classification::Decided(std::cmp::Ordering::Equal));
            }
            first.axis_coordinate_order_to_real(
                axis,
                if use_x { second.x() } else { second.y() },
                policy,
            )
        }
        (
            CurvePoint2(CurvePointData2::Exact(first)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(second)),
        ) => {
            if second
                .map_contact()
                .0
                .recursive_quadratic_line_system()
                .is_some()
            {
                return Ok(second
                    .axis_coordinate_order_to_real(
                        axis,
                        if use_x { first.x() } else { first.y() },
                        policy,
                    )?
                    .map(std::cmp::Ordering::reverse));
            }
            if second.same_point_evidence(&CurvePoint2::from(first.clone()), policy)
                == Classification::Decided(true)
            {
                return Ok(Classification::Decided(std::cmp::Ordering::Equal));
            }
            Ok(second
                .axis_coordinate_order_to_real(
                    axis,
                    if use_x { first.x() } else { first.y() },
                    policy,
                )?
                .map(std::cmp::Ordering::reverse))
        }
        (
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(first)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(second)),
        ) => {
            if let Some(order) =
                first.recursive_projective_axis_order_to_chord(second, axis, policy)?
            {
                return Ok(order);
            }
            Ok(algebraic_chord_point_coordinate_order_fallback(
                &CurvePoint2::from(first.clone()),
                &CurvePoint2::from(second.clone()),
                axis,
                policy,
            ))
        }
        (
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(first)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(second)),
        ) => {
            if let Some(order) =
                second.recursive_projective_axis_order_to_chord(first, axis, policy)?
            {
                return Ok(order.map(std::cmp::Ordering::reverse));
            }
            Ok(algebraic_chord_point_coordinate_order_fallback(
                &CurvePoint2::from(first.clone()),
                &CurvePoint2::from(second.clone()),
                axis,
                policy,
            ))
        }
        (
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(first)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(second)),
        ) if first == second => Ok(Classification::Decided(std::cmp::Ordering::Equal)),
        (
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(first)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(second)),
        ) => {
            if let Some(order) = first.complementary_mapped_axis_order(second, axis, policy)? {
                return Ok(Classification::Decided(order));
            }
            if let Some(order) = first.common_radial_source_axis_order(second, axis, policy)? {
                return Ok(Classification::Decided(order));
            }
            if let Some(order) = first.recursive_projective_axis_order(second, axis, policy)? {
                return Ok(order);
            }
            if let (Some(first), Some(second)) = (
                first.identity_source_point(policy),
                second.identity_source_point(policy),
            ) {
                return algebraic_chord_point_coordinate_order(&first, &second, axis, policy);
            }
            let first = CurvePoint2::from(first.clone());
            let second = CurvePoint2::from(second.clone());
            Ok(algebraic_chord_point_coordinate_order_fallback(
                &first, &second, axis, policy,
            ))
        }
        (
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(first)),
            CurvePoint2(CurvePointData2::Exact(second)),
        ) => first.axis_coordinate_order_to_real(
            axis,
            if use_x { second.x() } else { second.y() },
            policy,
        ),
        (
            CurvePoint2(CurvePointData2::Exact(first)),
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(second)),
        ) => Ok(second
            .axis_coordinate_order_to_real(axis, if use_x { first.x() } else { first.y() }, policy)?
            .map(std::cmp::Ordering::reverse)),
        (
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(first)),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(second)),
        ) if first.shares_carrier(second) => {
            if first.at_end == second.at_end {
                return Ok(Classification::Decided(std::cmp::Ordering::Equal));
            }
            algebraic_chord_point_coordinate_order(
                first.source_endpoint(),
                second.source_endpoint(),
                axis,
                policy,
            )
        }
        (
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(first)),
            CurvePoint2(CurvePointData2::Exact(second)),
        ) => Ok(first.axis_coordinate_order_to_real(
            axis,
            if use_x { second.x() } else { second.y() },
            policy,
        )),
        (
            CurvePoint2(CurvePointData2::Exact(first)),
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(second)),
        ) => Ok(second
            .axis_coordinate_order_to_real(axis, if use_x { first.x() } else { first.y() }, policy)
            .map(std::cmp::Ordering::reverse)),
        (
            CurvePoint2(CurvePointData2::AnalyticParallel(first)),
            CurvePoint2(CurvePointData2::AnalyticParallel(second)),
        ) if first == second => Ok(Classification::Decided(std::cmp::Ordering::Equal)),
        (
            CurvePoint2(CurvePointData2::AnalyticParallel(first)),
            CurvePoint2(CurvePointData2::Exact(second)),
        ) => Ok(first.axis_coordinate_order_to_real(
            axis,
            if use_x { second.x() } else { second.y() },
            policy,
        )),
        (
            CurvePoint2(CurvePointData2::Exact(first)),
            CurvePoint2(CurvePointData2::AnalyticParallel(second)),
        ) => Ok(second
            .axis_coordinate_order_to_real(axis, if use_x { first.x() } else { first.y() }, policy)
            .map(std::cmp::Ordering::reverse)),
        (CurvePoint2(CurvePointData2::AlgebraicChordPair(_)), _)
        | (_, CurvePoint2(CurvePointData2::AlgebraicChordPair(_)))
        | (CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)), _)
        | (_, CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)))
        | (CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)), _)
        | (_, CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)))
        | (CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)), _)
        | (_, CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)))
        | (CurvePoint2(CurvePointData2::AnalyticParallel(_)), _)
        | (_, CurvePoint2(CurvePointData2::AnalyticParallel(_)))
        | (CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)), _)
        | (_, CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_))) => Ok(
            algebraic_chord_point_coordinate_order_fallback(first, second, axis, policy),
        ),
    }
}

fn algebraic_chord_bounds_axis_order(
    first: &Aabb2,
    second: &Aabb2,
    axis: Axis2,
) -> Option<std::cmp::Ordering> {
    let (first_min, first_max, second_min, second_max) = match axis {
        Axis2::X => (
            first.min().x(),
            first.max().x(),
            second.min().x(),
            second.max().x(),
        ),
        Axis2::Y => (
            first.min().y(),
            first.max().y(),
            second.min().y(),
            second.max().y(),
        ),
    };
    if compare_reals(first_max, second_min, &CurveContext::STRICT) == Some(std::cmp::Ordering::Less)
    {
        return Some(std::cmp::Ordering::Less);
    }
    if compare_reals(second_max, first_min, &CurveContext::STRICT) == Some(std::cmp::Ordering::Less)
    {
        return Some(std::cmp::Ordering::Greater);
    }
    (compare_reals(first_min, first_max, &CurveContext::STRICT) == Some(std::cmp::Ordering::Equal)
        && compare_reals(second_min, second_max, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Equal)
        && compare_reals(first_min, second_min, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Equal))
    .then_some(std::cmp::Ordering::Equal)
}

fn algebraic_chord_point_coordinate_order_from_bounds(
    first: &CurvePoint2,
    second: &CurvePoint2,
    axis: Axis2,
    refinement_steps: usize,
    policy: &CurveContext,
) -> Option<Option<std::cmp::Ordering>> {
    let (Classification::Decided(first), Classification::Decided(second)) = (
        algebraic_chord_endpoint_bounds_refined(first, refinement_steps, policy),
        algebraic_chord_endpoint_bounds_refined(second, refinement_steps, policy),
    ) else {
        return None;
    };
    Some(algebraic_chord_bounds_axis_order(&first, &second, axis))
}

fn algebraic_chord_point_coordinate_order_fallback(
    first: &CurvePoint2,
    second: &CurvePoint2,
    axis: Axis2,
    policy: &CurveContext,
) -> Classification<std::cmp::Ordering> {
    for refinement_steps in [0, 2, 4, 8, 16] {
        if policy.has_bounded_exact_predicate_budget() && refinement_steps > 8 {
            break;
        }
        if let Some(Some(order)) = algebraic_chord_point_coordinate_order_from_bounds(
            first,
            second,
            axis,
            refinement_steps,
            policy,
        ) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-point-axis-order",
                "interval-separated",
            );
            return Classification::Decided(order);
        }
    }
    for (candidate, other, reverse) in [(first, second, false), (second, first, true)] {
        if let CurvePoint2(CurvePointData2::AnalyticParallel(candidate)) = candidate
            && let Ok(Classification::Decided(Some(order))) = policy.strict_predicate_pass(|| {
                candidate.retained_parameter_axis_order_to_point(other, axis, policy)
            })
        {
            return Classification::Decided(if reverse { order.reverse() } else { order });
        }
    }
    if policy.has_bounded_exact_predicate_budget() {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    }
    // Every point family can carry reusable homogeneous evidence. Compare
    // only the requested coordinate in its least shared retained field before
    // publishing independent Cartesian roots. The source authority preserves
    // denominator signs, selected generators and the positive speed sheet.
    if let Ok(Classification::Decided(Some(order))) =
        recursive_projective_point_evidence_axis_order(first, second, axis, policy)
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-point-axis-order",
            "retained-field",
        );
        return Classification::Decided(order);
    }
    if let (
        Ok(Classification::Decided(first_coordinates)),
        Ok(Classification::Decided(second_coordinates)),
    ) = (
        represented_point_evidence_coordinates(first, policy),
        represented_point_evidence_coordinates(second, policy),
    ) {
        let coordinate_index = usize::from(axis == Axis2::Y);
        if let Classification::Decided(difference) =
            Classification::from(represented_affine_coordinate(
                &[
                    (&first_coordinates[coordinate_index], &Real::one()),
                    (&second_coordinates[coordinate_index], &Real::from(-1_i8)),
                ],
                &Real::zero(),
            ))
            && let Some(sign) = represented_strict_sign(&difference)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-point-axis-order",
                "represented-cold-fallback",
            );
            return Classification::Decided(match sign {
                RealSign::Negative => std::cmp::Ordering::Less,
                RealSign::Zero => std::cmp::Ordering::Equal,
                RealSign::Positive => std::cmp::Ordering::Greater,
            });
        }
    }
    let mut terminal_refined = false;
    for refinement_steps in [32, 64, 128, 256, 512] {
        match algebraic_chord_point_coordinate_order_from_bounds(
            first,
            second,
            axis,
            refinement_steps,
            policy,
        ) {
            Some(Some(order)) => return Classification::Decided(order),
            Some(None) => terminal_refined |= refinement_steps == 512,
            None => {}
        }
    }
    if terminal_refined && policy.permits_approximate_512() {
        policy.observe_approximate_512();
        Classification::Decided(std::cmp::Ordering::Equal)
    } else {
        Classification::Uncertain(UncertaintyReason::Ordering)
    }
}

fn algebraic_chord_owned_coordinate_polynomials(
    point: &RationalBezierAlgebraicPointImage2,
    policy: &CurveContext,
) -> CurveResult<Classification<[Vec<Real>; 3]>> {
    if let Some((x, y, weight)) = point.retained_coordinate_polynomials() {
        return Ok(Classification::Decided([
            x.to_vec(),
            y.to_vec(),
            weight.to_vec(),
        ]));
    }
    let point = match point.predicate_evaluator(policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let (x, y, weight) = point.coordinate_polynomials();
    Ok(Classification::Decided([
        x.to_vec(),
        y.to_vec(),
        weight.to_vec(),
    ]))
}

#[derive(Debug)]
struct BezierAlgebraicAxisPointCoordinates2 {
    parameter: BezierParameter2,
    x: Vec<Real>,
    y: Vec<Real>,
    denominator: Vec<Real>,
}

/// Normalizes one represented or singly-selected point to a positive affine
/// denominator. Correlated multi-carrier points deliberately stay outside this
/// two-field construction.
fn algebraic_axis_point_coordinates(
    point: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicAxisPointCoordinates2>> {
    let (parameter, mut x, mut y, mut denominator) = match point {
        CurvePoint2(CurvePointData2::Exact(point)) => (
            BezierParameter2::Exact(Real::zero()),
            vec![point.x().clone()],
            vec![point.y().clone()],
            vec![Real::one()],
        ),
        CurvePoint2(CurvePointData2::Algebraic(point)) => {
            let parameter = match algebraic_chord_image_parameter(point, policy)? {
                Classification::Decided(parameter) => BezierParameter2::Algebraic(parameter),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let [x, y, denominator] =
                match algebraic_chord_owned_coordinate_polynomials(point, policy)? {
                    Classification::Decided(coordinates) => coordinates,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            (parameter, x, y, denominator)
        }
        CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
        | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
        | CurvePoint2(CurvePointData2::AnalyticParallel(_))
        | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
    };
    let denominator_sign = match signed_coefficients_at_parameter(&denominator, &parameter, policy)?
    {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    match denominator_sign {
        RealSign::Positive => {}
        RealSign::Negative => {
            x = polynomial_scale(&x, &Real::from(-1_i8));
            y = polynomial_scale(&y, &Real::from(-1_i8));
            denominator = polynomial_scale(&denominator, &Real::from(-1_i8));
        }
        RealSign::Zero => {
            return Err(CurveError::Topology(
                "retained axis point had a zero affine denominator".into(),
            ));
        }
    }
    Ok(Classification::Decided(
        BezierAlgebraicAxisPointCoordinates2 {
            parameter,
            x,
            y,
            denominator,
        },
    ))
}

fn algebraic_chord_point_coordinate_representation(
    point: &CurvePoint2,
    axis: Axis2,
    policy: &CurveContext,
) -> Option<hypersolve::AlgebraicRootRepresentation> {
    match point {
        CurvePoint2(CurvePointData2::Exact(point)) => {
            Some(AlgebraicRootRepresentation::from_exact_value(match axis {
                Axis2::X => point.x(),
                Axis2::Y => point.y(),
            }))
        }
        CurvePoint2(CurvePointData2::Algebraic(point)) => {
            let coordinates = point.represented_coordinates(policy)?;
            Some(match axis {
                Axis2::X => coordinates[0].clone(),
                Axis2::Y => coordinates[1].clone(),
            })
        }
        CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) => {
            let Classification::Decided(Some(coordinates)) =
                point.represented_coordinates(policy).ok()?
            else {
                return None;
            };
            Some(match axis {
                Axis2::X => coordinates[0].clone(),
                Axis2::Y => coordinates[1].clone(),
            })
        }
        CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
        | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
        | CurvePoint2(CurvePointData2::AnalyticParallel(_))
        | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => None,
    }
}

fn represented_point_evidence_coordinates(
    point: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
    if let CurvePoint2(CurvePointData2::Algebraic(point)) = point {
        return Ok(point
            .represented_coordinates(policy)
            .map(Classification::Decided)
            .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)));
    }
    if let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point {
        return point.represented_coordinates(policy);
    }
    if let CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) = point {
        return point.represented_coordinates(policy);
    }
    if let CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) = point {
        return point.represented_coordinates(policy);
    }
    if let CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) = point {
        return Ok(match point.represented_coordinates(policy)? {
            Classification::Decided(Some(coordinates)) => Classification::Decided(coordinates),
            Classification::Decided(None) => {
                Classification::Uncertain(UncertaintyReason::Unsupported)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        });
    }
    if let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = point {
        return match point.predicate_point_evidence(policy)? {
            Classification::Decided(Some(point)) => {
                represented_point_evidence_coordinates(&point, policy)
            }
            Classification::Decided(None) => point.represented_coordinates(policy),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        };
    }
    if let CurvePoint2(CurvePointData2::Similarity(point)) = point {
        if !policy.accepts_retained_policy(point.data.policy) {
            return Err(CurveError::Topology(
                "similarity point entered a represented predicate under a different policy".into(),
            ));
        }
        let source = match represented_point_evidence_coordinates(&point.data.source, policy)? {
            Classification::Decided(source) => source,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        return Ok(represented_similarity_point(&source, &point.data.transform));
    }
    let Some(x) = algebraic_chord_point_coordinate_representation(point, Axis2::X, policy) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let Some(y) = algebraic_chord_point_coordinate_representation(point, Axis2::Y, policy) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    Ok(Classification::Decided([x, y]))
}

/// Reconstructs one exact point from a retained chord and its certified affine
/// parameter. The endpoint coordinates and the parameter occupy only their
/// independent algebraic axes; the Cartesian coordinates are the tensor
/// images `A + u(B - A)`. This is the rank-independent coordinate authority
/// for procedural chords whose construction-frame radicals are irrelevant
/// once the chord and its selected parameter are known.
fn represented_chord_parameter_coordinates(
    chord: &BezierAlgebraicChord2,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
    chord.validate_policy(policy)?;
    match represented_parallel_chord_support(chord, policy)? {
        Classification::Decided(Some(parallel)) => {
            return Ok(represented_parallel_chord_parameter_coordinates(
                chord, parameter, parallel,
            ));
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let start = match represented_point_evidence_coordinates(chord.start(), policy)? {
        Classification::Decided(start) => start,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let end = match represented_point_evidence_coordinates(chord.end(), policy)? {
        Classification::Decided(end) => end,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let parameter = bezier_parameter_root_representation(parameter);
    let represented = [
        start[0].clone(),
        start[1].clone(),
        end[0].clone(),
        end[1].clone(),
        parameter,
    ];
    let Some((sources, coordinates)) = represented_affine_tensor_basis(&represented) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let coordinate = |start: usize, end: usize| {
        coordinates[end]
            .subtract(&coordinates[start])?
            .multiply(&coordinates[4])?
            .add(&coordinates[start])
    };
    let (Some(x), Some(y)) = (coordinate(0, 2), coordinate(1, 3)) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let x = Classification::from(represented_dense_value_refined(&x, &sources));
    let y = Classification::from(represented_dense_value_refined(&y, &sources));
    Ok(match (x, y) {
        (Classification::Decided(x), Classification::Decided(y)) => {
            Classification::Decided([x, y].map(|coordinate| {
                hypersolve::compact_algebraic_root_low_degree_witness(&coordinate)
                    .unwrap_or(coordinate)
            }))
        }
        (Classification::Uncertain(UncertaintyReason::Unsupported), _)
        | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        _ => Classification::Uncertain(UncertaintyReason::Predicate),
    })
}

/// Represents a chord direction and speed in one source field when one
/// endpoint is exact and the other is a rational map `(X, Y) / W` of one
/// algebraic parameter. Then `dx = (e_x W - X) / W`, `dy` likewise, and
/// `dx^2 + dy^2 = (D_x^2 + D_y^2) / W^2` are all rational images of that one
/// root. Independent coordinate roots would instead need a two-root
/// resultant to recover the identity that couples them.
fn represented_single_field_chord_direction_speed(
    chord: &BezierAlgebraicChord2,
) -> Option<(
    [AlgebraicRootRepresentation; 2],
    AlgebraicRootRepresentation,
)> {
    let (exact, algebraic, sign) = match (chord.start(), chord.end()) {
        (
            CurvePoint2(CurvePointData2::Algebraic(start)),
            CurvePoint2(CurvePointData2::Exact(end)),
        ) => (end, start, Real::one()),
        (
            CurvePoint2(CurvePointData2::Exact(start)),
            CurvePoint2(CurvePointData2::Algebraic(end)),
        ) => (start, end, Real::from(-1_i8)),
        _ => return None,
    };
    let (x, y, weight) = algebraic.retained_coordinate_polynomials()?;
    let root = algebraic.parameter();
    // Orient as end - start: `sign * (exact * W - X)`.
    let difference = |coordinate: &[Real], value: &Real| {
        let length = coordinate.len().max(weight.len());
        (0..length)
            .map(|index| {
                let scaled = weight.get(index).map_or_else(Real::zero, |w| w * value);
                let source = coordinate.get(index).cloned().unwrap_or_else(Real::zero);
                (scaled - source) * &sign
            })
            .collect::<Vec<_>>()
    };
    let dx = difference(x, exact.x());
    let dy = difference(y, exact.y());
    let norm_squared = polynomial_add(
        &polynomial_multiply(&dx, &dx),
        &polynomial_multiply(&dy, &dy),
    );
    let weight_squared = polynomial_multiply(weight, weight);
    let strict = hypersolve::PredicatePolicy::STRICT;
    let [dx, dy] =
        hypersolve::transform_algebraic_root_rational_images(root, [&dx, &dy], weight, strict);
    let [norm_squared] = hypersolve::transform_algebraic_root_rational_images(
        root,
        [&norm_squared],
        &weight_squared,
        strict,
    );
    let transformed = |report: hypersolve::AlgebraicRootRationalImageReport| {
        (report.status == hypersolve::AlgebraicRootRationalImageStatus::Transformed)
            .then_some(report.representation)
            .flatten()
    };
    let (dx, dy, norm_squared) = (
        transformed(dx)?,
        transformed(dy)?,
        transformed(norm_squared)?,
    );
    let speed = square_root_algebraic_root_representation(&norm_squared, 1);
    (speed.status == AlgebraicRootSquareRootStatus::Transformed)
        .then_some(speed.representation)
        .flatten()
        .map(|speed| ([dx, dy], speed))
}

fn represented_chord_direction_speed(
    chord: &BezierAlgebraicChord2,
    policy: &CurveContext,
) -> CurveResult<
    Classification<(
        [AlgebraicRootRepresentation; 2],
        AlgebraicRootRepresentation,
    )>,
> {
    chord.validate_policy(policy)?;
    #[cfg(feature = "dispatch-trace")]
    let evidence_kind = |point: &CurvePoint2| match point {
        CurvePoint2(CurvePointData2::Exact(_)) => "exact",
        CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
        CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "chord-pair",
        CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp-chord",
        CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "cusp-chord-derived",
        CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "chord-parallel",
        CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic-parallel",
        CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => "similarity",
    };
    if let Some(direction_speed) = represented_single_field_chord_direction_speed(chord) {
        return Ok(Classification::Decided(direction_speed));
    }
    let start = match represented_point_evidence_coordinates(chord.start(), policy)? {
        Classification::Decided(start) => start,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            {
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-chord-direction-speed-blocker",
                    "start",
                );
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-chord-direction-speed-start-kind",
                    evidence_kind(chord.start()),
                );
            }
            return Ok(Classification::Uncertain(reason));
        }
    };
    let end = match represented_point_evidence_coordinates(chord.end(), policy)? {
        Classification::Decided(end) => end,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            {
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-chord-direction-speed-blocker",
                    "end",
                );
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "represented-chord-direction-speed-end-kind",
                    evidence_kind(chord.end()),
                );
            }
            return Ok(Classification::Uncertain(reason));
        }
    };
    let difference = |end: &AlgebraicRootRepresentation, start: &AlgebraicRootRepresentation| {
        Classification::from(represented_affine_coordinate(
            &[(end, &Real::one()), (start, &Real::from(-1_i8))],
            &Real::zero(),
        ))
    };
    let dx = match difference(&end[0], &start[0]) {
        Classification::Decided(dx) => dx,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-chord-direction-speed-blocker",
                "dx",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    let dy = match difference(&end[1], &start[1]) {
        Classification::Decided(dy) => dy,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-chord-direction-speed-blocker",
                "dy",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    let norm_squared = match Classification::from(represented_vector_dot_cross(
        &[dx.clone(), dy.clone()],
        &[dx.clone(), dy.clone()],
    )) {
        Classification::Decided([norm_squared, _]) => norm_squared,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-chord-direction-speed-blocker",
                "norm-squared",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    let speed = square_root_algebraic_root_representation(&norm_squared, 1);
    let speed = match speed.status {
        AlgebraicRootSquareRootStatus::Transformed => speed
            .representation
            .expect("a transformed chord speed retains its representation"),
        AlgebraicRootSquareRootStatus::UndecidedSign => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-chord-direction-speed-blocker",
                "speed-predicate",
            );
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        AlgebraicRootSquareRootStatus::InvalidEvidence
        | AlgebraicRootSquareRootStatus::InvalidBranch
        | AlgebraicRootSquareRootStatus::NegativeRadicand
        | AlgebraicRootSquareRootStatus::NonzeroZeroBranch
        | AlgebraicRootSquareRootStatus::InvalidTransformedEvidence => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-chord-direction-speed-blocker",
                "speed-unsupported",
            );
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
    };
    Ok(Classification::Decided(([dx, dy], speed)))
}

/// Represents any nonzero direction on a chord's authoritative affine
/// support, oriented with the finite chord traversal.
///
/// A trimmed retained chord can end at a correlated multi-support point whose
/// standalone coordinate field is intentionally unavailable. Its root support
/// still owns two representable endpoints and the same unit tangent. Reusing
/// that support avoids materializing the trim point and transports orientation
/// through the chord's retained parameter axis exactly.
fn represented_chord_support_direction_speed(
    chord: &BezierAlgebraicChord2,
    policy: &CurveContext,
) -> CurveResult<
    Classification<(
        [AlgebraicRootRepresentation; 2],
        AlgebraicRootRepresentation,
    )>,
> {
    chord.validate_policy(policy)?;
    let support = chord.retained_support();
    let ([mut dx, mut dy], speed) = match represented_chord_direction_speed(support, policy)? {
        Classification::Decided(direction_speed) => direction_speed,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if !Arc::ptr_eq(&support.data, &chord.data) && chord.retained_support_orientation_is_reversed()
    {
        let negate = |coordinate: &AlgebraicRootRepresentation| {
            Classification::from(represented_affine_coordinate(
                &[(coordinate, &Real::from(-1_i8))],
                &Real::zero(),
            ))
        };
        dx = match negate(&dx) {
            Classification::Decided(dx) => dx,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        dy = match negate(&dy) {
            Classification::Decided(dy) => dy,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    }
    Ok(Classification::Decided(([dx, dy], speed)))
}

struct BezierRepresentedChordDirectionTensor2 {
    sources: Vec<AlgebraicRootRepresentation>,
    direction: [DenseTensorPolynomial; 2],
    speed_squared: DenseTensorPolynomial,
    speed: AlgebraicRootRepresentation,
    trailing: Vec<DenseTensorPolynomial>,
}

/// Keeps one chord's authoritative displacement, its positive speed, and any
/// caller-supplied selected scalars in a shared exact tensor.
fn represented_chord_direction_tensor(
    chord: &BezierAlgebraicChord2,
    trailing: &[AlgebraicRootRepresentation],
    known_speed: Option<AlgebraicRootRepresentation>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierRepresentedChordDirectionTensor2>> {
    chord.validate_policy(policy)?;
    let speed = if let Some(speed) = known_speed {
        speed
    } else {
        match represented_chord_support_direction_speed(chord, policy)? {
            Classification::Decided((_, speed)) => speed,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    };
    let support = chord.retained_support();
    let start = match represented_point_evidence_coordinates(support.start(), policy)? {
        Classification::Decided(start) => start,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let end = match represented_point_evidence_coordinates(support.end(), policy)? {
        Classification::Decided(end) => end,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let represented = start
        .into_iter()
        .chain(end)
        .chain(trailing.iter().cloned())
        .collect::<Vec<_>>();
    let Some((sources, coordinates)) = represented_affine_tensor_basis(&represented) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let mut coordinates = coordinates.into_iter();
    let [start_x, start_y, end_x, end_y] = [(); 4].map(|_| {
        coordinates
            .next()
            .expect("a chord direction tensor retains both endpoints")
    });
    let trailing = coordinates.collect::<Vec<_>>();
    let reduce = |polynomial| dense_reduce_selected_root_relations(polynomial, &sources);
    let Some((mut dx, mut dy)) = end_x
        .subtract(&start_x)
        .and_then(&reduce)
        .zip(end_y.subtract(&start_y).and_then(&reduce))
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    if !Arc::ptr_eq(&support.data, &chord.data) && chord.retained_support_orientation_is_reversed()
    {
        let Some((reversed_x, reversed_y)) = dx
            .scale(&Real::from(-1_i8))
            .and_then(&reduce)
            .zip(dy.scale(&Real::from(-1_i8)).and_then(&reduce))
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        dx = reversed_x;
        dy = reversed_y;
    }
    let Some(speed_squared) = dx
        .multiply(&dx)
        .and_then(|square_x| {
            dy.multiply(&dy)
                .and_then(|square_y| square_x.add(&square_y))
        })
        .and_then(reduce)
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    Ok(Classification::Decided(
        BezierRepresentedChordDirectionTensor2 {
            sources,
            direction: [dx, dy],
            speed_squared,
            speed,
            trailing,
        },
    ))
}

fn represented_chord_unit_direction(
    chord: &BezierAlgebraicChord2,
    direction: BezierAlgebraicChordUnitDisplacement2,
    policy: &CurveContext,
) -> CurveResult<Classification<[AlgebraicRootRepresentation; 2]>> {
    chord.validate_policy(policy)?;
    if let Some((x, y)) = chord.certified_unit_tangent() {
        let components = match direction {
            BezierAlgebraicChordUnitDisplacement2::Tangent => [x, y],
            BezierAlgebraicChordUnitDisplacement2::LeftNormal => [-y, x],
        };
        return Ok(Classification::Decided(components.map(|value| {
            AlgebraicRootRepresentation::from_exact_value(&value)
        })));
    }
    let (support, reversed) = chord.smallest_incidence_support();
    if let (
        CurvePoint2(CurvePointData2::AnalyticParallel(start)),
        CurvePoint2(CurvePointData2::AnalyticParallel(end)),
    ) = (support.start(), support.end())
        && let Classification::Decided(Some(line)) = start.recursive_tangent_line_to(end, policy)?
    {
        // The oriented line owns (-dy, dx) up to a positive scale. Its
        // shared source point, normal displacement, and translation have
        // already cancelled. Normalize in that field before publishing any
        // coordinate roots; projecting the two endpoints first duplicates
        // their source and speed fields merely to subtract them again.
        let normalized = (|| -> CurveResult<Option<[AlgebraicRootRepresentation; 2]>> {
            let Some(speed_squared) = line.x.square().and_then(|x| x.add(&line.y.square()?)) else {
                return Ok(None);
            };
            if speed_squared.sign_with_nonzero_certificate()?
                != Classification::Decided(RealSign::Positive)
            {
                return Ok(None);
            }
            let parent = speed_squared.field();
            let (field, speed) =
                if let Some(speed) = parent.retained_positive_square_root(&speed_squared) {
                    (parent, speed)
                } else {
                    let Some(field) = parent.extension(speed_squared) else {
                        return Ok(None);
                    };
                    let Some(speed) = parent
                        .constant(Real::zero())
                        .and_then(|zero| field.element(zero, parent.constant(Real::one())?))
                    else {
                        return Ok(None);
                    };
                    (field, speed)
                };
            let orientation = Real::from(if reversed { -1_i8 } else { 1_i8 });
            let components = match direction {
                BezierAlgebraicChordUnitDisplacement2::Tangent => {
                    [line.y.scale(&orientation), line.x.scale(&(-orientation))]
                }
                BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
                    [line.x.scale(&orientation), line.y.scale(&orientation)]
                }
            };
            let [Some(x), Some(y)] =
                components.map(|component| component.and_then(|value| field.lift(&value)))
            else {
                return Ok(None);
            };
            let mut represented = Vec::with_capacity(2);
            for numerator in [x, y] {
                match (BezierRecursiveQuadraticProjectiveScalar2 {
                    numerator,
                    denominator: speed.clone(),
                })
                .represented_value(policy)?
                {
                    Classification::Decided(value) => represented.push(value),
                    Classification::Uncertain(_) => return Ok(None),
                }
            }
            Ok(Some(
                represented
                    .try_into()
                    .expect("a unit direction has two coordinates"),
            ))
        })()?;
        if let Some(direction) = normalized {
            return Ok(Classification::Decided(direction));
        }
    }
    let ([dx, dy], speed) = match represented_chord_support_direction_speed(chord, policy)? {
        Classification::Decided(direction_speed) => direction_speed,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-chord-unit-direction-blocker",
                "support-direction",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    let (direction_x, direction_y) = match direction {
        BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
            let direction_x = match Classification::from(represented_affine_coordinate(
                &[(&dy, &Real::from(-1_i8))],
                &Real::zero(),
            )) {
                Classification::Decided(value) => value,
                Classification::Uncertain(reason) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "represented-chord-unit-direction-blocker",
                        "normal-negation",
                    );
                    return Ok(Classification::Uncertain(reason));
                }
            };
            (direction_x, dx)
        }
        BezierAlgebraicChordUnitDisplacement2::Tangent => (dx, dy),
    };
    let x = Classification::from(represented_ratio(&direction_x, &speed));
    let y = Classification::from(represented_ratio(&direction_y, &speed));
    let independent = match (x, y) {
        (Classification::Decided(x), Classification::Decided(y)) => {
            Classification::Decided([x, y].map(|coordinate| {
                hypersolve::compact_algebraic_root_low_degree_witness(&coordinate)
                    .unwrap_or(coordinate)
            }))
        }
        (Classification::Uncertain(UncertaintyReason::Unsupported), _)
        | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        _ => Classification::Uncertain(UncertaintyReason::Predicate),
    };
    if matches!(independent, Classification::Decided(_)) {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "represented-chord-unit-direction",
            "independent-quotients",
        );
        return Ok(independent);
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "represented-chord-unit-direction-fallback",
        match independent {
            Classification::Uncertain(UncertaintyReason::Unsupported) => "independent-unsupported",
            Classification::Uncertain(_) => "independent-predicate",
            Classification::Decided(_) => unreachable!(),
        },
    );

    // Dividing two independently materialized roots can forget the defining
    // identity `speed^2 = dx^2 + dy^2`.  Preserve the original endpoint axes
    // and eliminate both normalized coordinates from that one correlated
    // tensor when the smaller quotient authority cannot isolate a sheet.
    let BezierRepresentedChordDirectionTensor2 {
        sources,
        direction: [dx, dy],
        speed_squared,
        speed,
        trailing,
    } = match represented_chord_direction_tensor(chord, &[], Some(speed), policy)? {
        Classification::Decided(tensor) => tensor,
        Classification::Uncertain(reason) => {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "represented-chord-unit-direction-blocker",
                "correlated-direction-tensor",
            );
            return Ok(Classification::Uncertain(reason));
        }
    };
    debug_assert!(trailing.is_empty());
    let (numerator_x, numerator_y) = match direction {
        BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
            let Some(normal_x) = dy.scale(&Real::from(-1_i8)) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            (normal_x, dx)
        }
        BezierAlgebraicChordUnitDisplacement2::Tangent => (dx, dy),
    };
    let rank = sources.len() + 1;
    let Some((zero, one)) =
        DenseTensorPolynomial::from_axis_polynomial(rank, 0, std::slice::from_ref(&Real::zero()))
            .zip(DenseTensorPolynomial::from_axis_polynomial(
                rank,
                0,
                std::slice::from_ref(&Real::one()),
            ))
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let x = represented_tensor_nested_ratio(
        &numerator_x,
        &zero,
        &zero,
        &one,
        &speed_squared,
        &sources,
        &speed,
    );
    let y = represented_tensor_nested_ratio(
        &numerator_y,
        &zero,
        &zero,
        &one,
        &speed_squared,
        &sources,
        &speed,
    );
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "represented-chord-unit-direction-fallback",
        match (&x, &y) {
            (Classification::Decided(_), Classification::Decided(_)) => "correlated-quotients",
            (Classification::Uncertain(UncertaintyReason::Unsupported), _)
            | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
                "correlated-unsupported"
            }
            _ => "correlated-predicate",
        },
    );
    Ok(match (x, y) {
        (Classification::Decided(x), Classification::Decided(y)) => {
            Classification::Decided([x, y].map(|coordinate| {
                hypersolve::compact_algebraic_root_low_degree_witness(&coordinate)
                    .unwrap_or(coordinate)
            }))
        }
        (Classification::Uncertain(UncertaintyReason::Unsupported), _)
        | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        _ => Classification::Uncertain(UncertaintyReason::Predicate),
    })
}

/// Shared retained field for the angular half of a represented
/// chord-normal-circle/line solve.  The target contact radical is appended to
/// the anchor normalization field instead of first materializing unrelated X
/// and Y roots for the same point.
#[derive(Debug)]
struct BezierRepresentedChordNormalLineAngularSystem2 {
    field: BezierRecursiveQuadraticField2,
    anchor_direction: [BezierRecursiveQuadraticValue2; 2],
    target_direction: [BezierRecursiveQuadraticValue2; 2],
    direction_squared: BezierRecursiveQuadraticValue2,
    radial_retained: [BezierRecursiveQuadraticValue2; 2],
    discriminant: BezierRecursiveQuadraticValue2,
    anchor_speed: BezierRecursiveQuadraticValue2,
    signed_radius: Real,
    turn: Real,
}

fn represented_chord_normal_line_angular_system(
    anchor: &BezierAlgebraicChord2,
    line_start: &[AlgebraicRootRepresentation; 2],
    line_end: &[AlgebraicRootRepresentation; 2],
    center: &[AlgebraicRootRepresentation; 2],
    signed_radius: &Real,
    turn: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierRepresentedChordNormalLineAngularSystem2>> {
    let trailing = line_start
        .iter()
        .chain(line_end)
        .chain(center)
        .cloned()
        .collect::<Vec<_>>();
    let BezierRepresentedChordDirectionTensor2 {
        sources,
        direction: anchor_direction,
        speed_squared,
        speed: _,
        trailing,
    } = match represented_chord_direction_tensor(anchor, &trailing, None, policy)? {
        Classification::Decided(tensor) => tensor,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let [start_x, start_y, end_x, end_y, center_x, center_y]: [DenseTensorPolynomial; 6] = trailing
        .try_into()
        .expect("a chord-normal angular tensor retains its line and center coordinates");
    let reduce = |polynomial| dense_reduce_selected_root_relations(polynomial, &sources);
    let Some((target_direction, direction_squared, radial_retained, discriminant, one)) = (|| {
        let dx = reduce(end_x.subtract(&start_x)?)?;
        let dy = reduce(end_y.subtract(&start_y)?)?;
        let vx = reduce(start_x.subtract(&center_x)?)?;
        let vy = reduce(start_y.subtract(&center_y)?)?;
        let direction_squared = reduce(dx.multiply(&dx)?.add(&dy.multiply(&dy)?)?)?;
        let projection = reduce(vx.multiply(&dx)?.add(&vy.multiply(&dy)?)?)?;
        let radius_squared = signed_radius * signed_radius;
        let rank = sources.len() + 1;
        let radius_squared = DenseTensorPolynomial::from_axis_polynomial(
            rank,
            0,
            std::slice::from_ref(&radius_squared),
        )?;
        let radial_residual = reduce(
            vx.multiply(&vx)?
                .add(&vy.multiply(&vy)?)?
                .subtract(&radius_squared)?,
        )?;
        let discriminant = reduce(
            projection
                .multiply(&projection)?
                .subtract(&direction_squared.multiply(&radial_residual)?)?,
        )?;
        let point_retained_x = reduce(
            start_x
                .multiply(&direction_squared)?
                .subtract(&dx.multiply(&projection)?)?,
        )?;
        let point_retained_y = reduce(
            start_y
                .multiply(&direction_squared)?
                .subtract(&dy.multiply(&projection)?)?,
        )?;
        let radial_retained_x =
            reduce(point_retained_x.subtract(&center_x.multiply(&direction_squared)?)?)?;
        let radial_retained_y =
            reduce(point_retained_y.subtract(&center_y.multiply(&direction_squared)?)?)?;
        let one = DenseTensorPolynomial::from_axis_polynomial(
            rank,
            0,
            std::slice::from_ref(&Real::one()),
        )?;
        Some((
            [dx, dy],
            direction_squared,
            [radial_retained_x, radial_retained_y],
            discriminant,
            one,
        ))
    })() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let remove_output_axis = |polynomial: DenseTensorPolynomial| {
        polynomial.remove_certified_independent_axis(
            sources.len(),
            hypersolve::PredicatePolicy::MAX_REFINEMENT_PRECISION,
        )
    };
    let Some((
        anchor_direction,
        target_direction,
        speed_squared,
        direction_squared,
        radial_retained,
        discriminant,
        one,
    )) = (|| {
        let [anchor_x, anchor_y] = anchor_direction;
        let [target_x, target_y] = target_direction;
        let [radial_x, radial_y] = radial_retained;
        Some((
            [remove_output_axis(anchor_x)?, remove_output_axis(anchor_y)?],
            [remove_output_axis(target_x)?, remove_output_axis(target_y)?],
            remove_output_axis(speed_squared)?,
            remove_output_axis(direction_squared)?,
            [remove_output_axis(radial_x)?, remove_output_axis(radial_y)?],
            remove_output_axis(discriminant)?,
            remove_output_axis(one)?,
        ))
    })()
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let Some(field) = BezierRecursiveQuadraticField2::base(sources, speed_squared, one.clone())
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let BezierRecursiveQuadraticField2::Base(base) = &field else {
        unreachable!("a represented chord-normal angular field begins at its dense base")
    };
    let rational = |polynomial| {
        BezierDenseTwoSquareRootExpression2::from_rational(polynomial).and_then(|expression| {
            BezierRecursiveQuadraticValue2::from_base(base.clone(), expression)
        })
    };
    let Some((
        anchor_direction,
        target_direction,
        direction_squared,
        radial_retained,
        discriminant,
        anchor_speed,
    )) = (|| {
        let [anchor_x, anchor_y] = anchor_direction;
        let [target_x, target_y] = target_direction;
        let [radial_x, radial_y] = radial_retained;
        Some((
            [rational(anchor_x)?, rational(anchor_y)?],
            [rational(target_x)?, rational(target_y)?],
            rational(direction_squared)?,
            [rational(radial_x)?, rational(radial_y)?],
            rational(discriminant)?,
            BezierDenseTwoSquareRootExpression2::from_first_radical(one).and_then(
                |expression| BezierRecursiveQuadraticValue2::from_base(base.clone(), expression),
            )?,
        ))
    })()
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    Ok(Classification::Decided(
        BezierRepresentedChordNormalLineAngularSystem2 {
            field,
            anchor_direction,
            target_direction,
            direction_squared,
            radial_retained,
            discriminant,
            anchor_speed,
            signed_radius: signed_radius.clone(),
            turn: turn.clone(),
        },
    ))
}

impl BezierRepresentedChordNormalLineAngularSystem2 {
    fn contact_location_parameter(
        &self,
        branch: i8,
    ) -> CurveResult<
        Classification<
            Option<(
                BezierAlgebraicCuspSemicircleContactLocation2,
                BezierRepresentedCircleChordAngularParameter2,
            )>,
        >,
    > {
        if !matches!(branch, -1..=1) {
            return Err(CurveError::Topology(
                "a represented chord-normal contact retained an invalid radical branch".into(),
            ));
        }
        let discriminant_sign = match self.discriminant.sign(&CurveContext::STRICT)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if (branch == 0 && discriminant_sign != RealSign::Zero)
            || (branch != 0 && discriminant_sign != RealSign::Positive)
        {
            return Err(CurveError::Topology(
                "a represented chord-normal contact branch disagreed with its discriminant".into(),
            ));
        }
        let field = if branch == 0 {
            self.field.clone()
        } else {
            self.field
                .extension(self.discriminant.clone())
                .ok_or_else(|| {
                    CurveError::Topology(
                        "a represented chord-normal contact could not extend its base field".into(),
                    )
                })?
        };
        let radial = if branch == 0 {
            self.radial_retained.clone()
        } else {
            let coefficient = Real::from(branch);
            let Some(radial) = [0, 1]
                .map(|axis| {
                    field.element(
                        self.radial_retained[axis].clone(),
                        self.target_direction[axis].scale(&coefficient)?,
                    )
                })
                .into_iter()
                .collect::<Option<Vec<_>>>()
                .and_then(|values| values.try_into().ok())
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            radial
        };
        let Some((anchor_direction, direction_squared, anchor_speed)) = (|| {
            Some((
                self.anchor_direction
                    .each_ref()
                    .map(|value| field.lift(value))
                    .into_iter()
                    .collect::<Option<Vec<_>>>()?
                    .try_into()
                    .ok()?,
                field.lift(&self.direction_squared)?,
                field.lift(&self.anchor_speed)?,
            ))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let [anchor_x, anchor_y]: [BezierRecursiveQuadraticValue2; 2] = anchor_direction;
        let [radial_x, radial_y] = radial;
        let Some((dot, oriented_cross)) = (|| {
            let dot = anchor_x
                .multiply(&radial_y)?
                .subtract(&anchor_y.multiply(&radial_x)?)?
                .scale(&self.signed_radius)?;
            let cross = anchor_x
                .multiply(&radial_x)?
                .add(&anchor_y.multiply(&radial_y)?)?
                .scale(&(-(&self.signed_radius * &self.turn)))?;
            Some((dot, cross))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let cross_sign = match oriented_cross.sign(&CurveContext::STRICT)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if cross_sign == RealSign::Negative {
            return Ok(Classification::Decided(None));
        }
        if cross_sign == RealSign::Zero {
            return Ok(match dot.sign(&CurveContext::STRICT)? {
                Classification::Decided(RealSign::Positive) => Classification::Decided(Some((
                    BezierAlgebraicCuspSemicircleContactLocation2::Start,
                    BezierRepresentedCircleChordAngularParameter2::Materialized(
                        BezierParameter2::Exact(Real::zero()),
                    ),
                ))),
                Classification::Decided(RealSign::Negative) => Classification::Decided(Some((
                    BezierAlgebraicCuspSemicircleContactLocation2::End,
                    BezierRepresentedCircleChordAngularParameter2::Materialized(
                        BezierParameter2::Exact(Real::one()),
                    ),
                ))),
                Classification::Decided(RealSign::Zero) => {
                    return Err(CurveError::Topology(
                        "a chord-normal nonzero contact had zero diameter coordinates".into(),
                    ));
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            });
        }
        let radius_squared = &self.signed_radius * &self.signed_radius;
        let Some(radial_complement) = anchor_speed
            .multiply(&direction_squared)
            .and_then(|value| value.scale(&radius_squared))
            .and_then(|value| value.add(&dot))
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        match radial_complement.sign(&CurveContext::STRICT)? {
            Classification::Decided(RealSign::Positive) => {}
            Classification::Decided(RealSign::Zero | RealSign::Negative) => {
                return Err(CurveError::Topology(
                    "an interior chord-normal contact had a nonpositive angular complement".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let Some(denominator) = radial_complement.add(&oriented_cross) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        Ok(Classification::Decided(Some((
            BezierAlgebraicCuspSemicircleContactLocation2::Interior,
            BezierRepresentedCircleChordAngularParameter2::Recursive(
                BezierRecursiveQuadraticProjectiveScalar2 {
                    numerator: oriented_cross,
                    denominator,
                },
            ),
        ))))
    }
}

struct BezierRepresentedParallelChordSupport2 {
    coordinates: [AlgebraicRootRepresentation; 5],
    distance: Real,
    translation_x: Real,
    translation_y: Real,
    direction: BezierAlgebraicChordUnitDisplacement2,
}

/// Evaluates a retained procedural parallel without first materializing both
/// displaced endpoints. With source direction `D`, positive speed `s`, and
/// selected source parameter `u`, the homogeneous point is
///
/// `s * (A + uD + translation) + distance * unit_numerator`.
///
/// Dividing once by `s` retains the authored positive normalization sheet and
/// avoids constructing two large endpoint fields only to interpolate them.
fn represented_parallel_chord_parameter_coordinates(
    chord: &BezierAlgebraicChord2,
    parameter: &BezierParameter2,
    parallel: BezierRepresentedParallelChordSupport2,
) -> Classification<[AlgebraicRootRepresentation; 2]> {
    let BezierRepresentedParallelChordSupport2 {
        coordinates,
        distance,
        translation_x,
        translation_y,
        direction,
    } = parallel;
    let parameter = bezier_parameter_root_representation(parameter);
    let represented = coordinates
        .into_iter()
        .chain(std::iter::once(parameter))
        .collect::<Vec<_>>();
    let Some((sources, coordinates)) = represented_affine_tensor_basis(&represented) else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let [origin_x, origin_y, dx, dy, speed, parameter]: [DenseTensorPolynomial; 6] = coordinates
        .try_into()
        .expect("a represented parallel retains its source frame and target parameter");
    let rank = sources.len() + 1;
    let constant = |value: &Real| {
        DenseTensorPolynomial::from_axis_polynomial(rank, 0, std::slice::from_ref(value))
    };
    let reversed = matches!(
        chord.start(),
        CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) if point.at_end
    );
    let Some((x_numerator, y_numerator)) = (|| {
        let selected_parameter = if reversed {
            constant(&Real::one())?.subtract(&parameter)?
        } else {
            parameter.clone()
        };
        let source_x = origin_x
            .add(&dx.multiply(&selected_parameter)?)?
            .add(&constant(&translation_x)?)?;
        let source_y = origin_y
            .add(&dy.multiply(&selected_parameter)?)?
            .add(&constant(&translation_y)?)?;
        let (unit_x, unit_y) = match direction {
            BezierAlgebraicChordUnitDisplacement2::LeftNormal => {
                (dy.scale(&Real::from(-1_i8))?, dx.clone())
            }
            BezierAlgebraicChordUnitDisplacement2::Tangent => (dx.clone(), dy.clone()),
        };
        Some((
            source_x.multiply(&speed)?.add(&unit_x.scale(&distance)?)?,
            source_y.multiply(&speed)?.add(&unit_y.scale(&distance)?)?,
        ))
    })() else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let materialize = |numerator: &DenseTensorPolynomial| {
        for (candidate, representation) in [
            (&origin_x, &represented[0]),
            (&origin_y, &represented[1]),
            (&dx, &represented[2]),
            (&dy, &represented[3]),
            (&speed, &represented[4]),
            (&parameter, &represented[5]),
        ] {
            let Some(residual) = speed
                .multiply(candidate)
                .and_then(|product| product.subtract(numerator))
            else {
                continue;
            };
            if polynomial_coefficients_are_identically_zero(
                residual.coefficients(),
                &CurveContext::STRICT,
            ) == Classification::Decided(true)
            {
                return Classification::Decided(representation.clone());
            }
        }
        represented_tensor_ratio(numerator, &speed, &sources)
    };
    let x = materialize(&x_numerator);
    let y = materialize(&y_numerator);
    match (x, y) {
        (Classification::Decided(x), Classification::Decided(y)) => {
            Classification::Decided([x, y].map(|coordinate| {
                hypersolve::compact_algebraic_root_low_degree_witness(&coordinate)
                    .unwrap_or(coordinate)
            }))
        }
        (Classification::Uncertain(UncertaintyReason::Unsupported), _)
        | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        _ => Classification::Uncertain(UncertaintyReason::Predicate),
    }
}

#[derive(Clone, Debug)]
struct BezierChordParallelSupportSource2 {
    source: BezierAlgebraicChord2,
    distance: Real,
    translation_x: Real,
    translation_y: Real,
    direction: BezierAlgebraicChordUnitDisplacement2,
}

fn chord_parallel_support_source(
    chord: &BezierAlgebraicChord2,
    policy: &CurveContext,
) -> CurveResult<Option<BezierChordParallelSupportSource2>> {
    chord.validate_policy(policy)?;
    // Finite Boolean splits and reversals replace endpoint evidence while
    // retaining the same affine support in `source`.  The procedural parallel
    // certificate belongs to that authoritative support, not necessarily to
    // the descendant's current endpoints.
    let chord = chord.retained_support();
    let (
        CurvePoint2(CurvePointData2::AlgebraicChordParallel(first)),
        CurvePoint2(CurvePointData2::AlgebraicChordParallel(second)),
    ) = (chord.start(), chord.end())
    else {
        return Ok(None);
    };
    if !Arc::ptr_eq(&first.data, &second.data)
        || first.at_end == second.at_end
        || first.data.source_point.is_some()
        || !first.accepts_policy(policy)
    {
        return Ok(None);
    }
    Ok(Some(BezierChordParallelSupportSource2 {
        source: first.data.source.clone(),
        distance: first.data.distance.clone(),
        translation_x: first.data.translation_x.clone(),
        translation_y: first.data.translation_y.clone(),
        direction: first.data.direction,
    }))
}

fn represented_parallel_chord_support(
    chord: &BezierAlgebraicChord2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierRepresentedParallelChordSupport2>>> {
    let Some(structural) = chord_parallel_support_source(chord, policy)? else {
        return Ok(Classification::Decided(None));
    };
    let origin = match represented_point_evidence_coordinates(structural.source.start(), policy)? {
        Classification::Decided(origin) => origin,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let ([dx, dy], speed) =
        match represented_chord_support_direction_speed(&structural.source, policy)? {
            Classification::Decided(direction_speed) => direction_speed,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    Ok(Classification::Decided(Some(
        BezierRepresentedParallelChordSupport2 {
            coordinates: [origin[0].clone(), origin[1].clone(), dx, dy, speed],
            distance: structural.distance,
            translation_x: structural.translation_x,
            translation_y: structural.translation_y,
            direction: structural.direction,
        },
    )))
}

fn represented_projective_line_intersection(
    first: [DenseTensorPolynomial; 3],
    second: [DenseTensorPolynomial; 3],
    sources: &[AlgebraicRootRepresentation],
) -> Classification<[AlgebraicRootRepresentation; 2]> {
    let [first_a, first_b, first_c] = first;
    let [second_a, second_b, second_c] = second;
    let Some((x_numerator, y_numerator, denominator)) = (|| {
        let denominator = first_a
            .multiply(&second_b)?
            .subtract(&second_a.multiply(&first_b)?)?;
        let x_numerator = first_b
            .multiply(&second_c)?
            .subtract(&second_b.multiply(&first_c)?)?;
        let y_numerator = first_c
            .multiply(&second_a)?
            .subtract(&second_c.multiply(&first_a)?)?;
        Some((x_numerator, y_numerator, denominator))
    })() else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let x = represented_tensor_ratio(&x_numerator, &denominator, sources);
    let y = represented_tensor_ratio(&y_numerator, &denominator, sources);
    match (x, y) {
        (Classification::Decided(x), Classification::Decided(y)) => {
            Classification::Decided([x, y].map(|coordinate| {
                hypersolve::compact_algebraic_root_low_degree_witness(&coordinate)
                    .unwrap_or(coordinate)
            }))
        }
        (Classification::Uncertain(UncertaintyReason::Unsupported), _)
        | (_, Classification::Uncertain(UncertaintyReason::Unsupported)) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        _ => Classification::Uncertain(UncertaintyReason::Predicate),
    }
}

fn algebraic_chord_strict_coordinate_between(
    first: &CurvePoint2,
    second: &CurvePoint2,
    parameter_axis: BezierAlgebraicChordParameterAxis2,
    policy: &CurveContext,
) -> CurveResult<Classification<Real>> {
    // Finding an interior scalar needs separated certified enclosures, not
    // independent coordinate roots. Try the local evidence before any cold
    // materialization, which can otherwise build a large tensor resultant.
    let (lower, upper) = if parameter_axis.coordinate_increases {
        (first, second)
    } else {
        (second, first)
    };
    let separated_midpoint = |lower_bounds: Aabb2,
                              upper_bounds: Aabb2,
                              refinement_steps: usize|
     -> CurveResult<Option<Real>> {
        let lower_bounds = lower_bounds
            .certified_rational_outer_envelope(refinement_steps)
            .unwrap_or(lower_bounds);
        let upper_bounds = upper_bounds
            .certified_rational_outer_envelope(refinement_steps)
            .unwrap_or(upper_bounds);
        let lower_upper = match parameter_axis.axis {
            Axis2::X => lower_bounds.max().x(),
            Axis2::Y => lower_bounds.max().y(),
        };
        let upper_lower = match parameter_axis.axis {
            Axis2::X => upper_bounds.min().x(),
            Axis2::Y => upper_bounds.min().y(),
        };
        Ok(
            (compare_reals(lower_upper, upper_lower, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less))
            .then(|| (lower_upper + upper_lower) / Real::from(2_i8))
            .transpose()?,
        )
    };
    for refinement_steps in [0, 2, 4] {
        let bounds = policy.strict_predicate_pass(|| {
            (
                algebraic_chord_endpoint_local_bounds_refined(lower, refinement_steps, policy),
                algebraic_chord_endpoint_local_bounds_refined(upper, refinement_steps, policy),
            )
        });
        if let (Classification::Decided(lower_bounds), Classification::Decided(upper_bounds)) =
            bounds
            && let Some(midpoint) =
                separated_midpoint(lower_bounds, upper_bounds, refinement_steps)?
        {
            return Ok(Classification::Decided(midpoint));
        }
    }
    let (Some(first_representation), Some(second_representation)) = (
        algebraic_chord_point_coordinate_representation(first, parameter_axis.axis, policy),
        algebraic_chord_point_coordinate_representation(second, parameter_axis.axis, policy),
    ) else {
        // Correlated multi-carrier endpoints intentionally have no independent
        // algebraic-root representation.  Their chord already certifies a
        // strict coordinate order, so disjoint conservative coordinate
        // intervals are an exact constructive witness of a scalar interior
        // coordinate.  A finite refinement budget may decline to construct a
        // witness, but it never turns unresolved equality into inequality.
        for refinement_steps in [8, 16, 32, 64, 128, 256, 512] {
            let (Classification::Decided(lower_bounds), Classification::Decided(upper_bounds)) = (
                algebraic_chord_endpoint_local_bounds_refined(lower, refinement_steps, policy),
                algebraic_chord_endpoint_local_bounds_refined(upper, refinement_steps, policy),
            ) else {
                continue;
            };
            if let Some(midpoint) =
                separated_midpoint(lower_bounds, upper_bounds, refinement_steps)?
            {
                return Ok(Classification::Decided(midpoint));
            }
        }
        // Local endpoint fields deliberately remain independent.  When their
        // boxes cannot expose the already-certified finite gap, join them only
        // in this cold witness-construction fallback and refine the correlated
        // recursive-projective points together.
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let (Classification::Decided(lower_bounds), Classification::Decided(upper_bounds)) = (
                algebraic_chord_endpoint_bounds_refined(lower, refinement_steps, policy),
                algebraic_chord_endpoint_bounds_refined(upper, refinement_steps, policy),
            ) else {
                continue;
            };
            if let Some(midpoint) =
                separated_midpoint(lower_bounds, upper_bounds, refinement_steps)?
            {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-representative",
                    "correlated-endpoint-separation",
                );
                return Ok(Classification::Decided(midpoint));
            }
        }
        return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
    };
    // Ordering need not separate the stored outer intervals: a point witness
    // can supersede them, and touching half-open isolators already prove order.
    // Reuse the constructive scalar query, which keeps those witnesses and
    // refines until it can certify an actual interior coordinate.
    policy.strict_predicate_pass(|| {
        for representation in [&first_representation, &second_representation] {
            if hypersolve::validate_algebraic_root_representation(
                representation,
                hypersolve::PredicatePolicy::STRICT,
            )
            .status
                != hypersolve::AlgebraicRootValidationStatus::Valid
            {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
        }
        let (lower, upper) = if parameter_axis.coordinate_increases {
            (&first_representation, &second_representation)
        } else {
            (&second_representation, &first_representation)
        };
        match (
            BezierParameter2::from_algebraic_root_representation_unbounded(lower, policy)?,
            BezierParameter2::from_algebraic_root_representation_unbounded(upper, policy)?,
        ) {
            (Classification::Decided(lower), Classification::Decided(upper)) => {
                lower.strict_scalar_between(&upper, policy)
            }
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                Ok(Classification::Uncertain(reason))
            }
        }
    })
}

fn recursive_projective_endpoint_bounds_refined(
    endpoint: &CurvePoint2,
    refinement_steps: usize,
    policy: &CurveContext,
) -> Option<Aabb2> {
    let Classification::Decided(Some(mut points)) =
        recursive_projective_evidence_points(&[endpoint], policy).ok()?
    else {
        return None;
    };
    let point = points.pop()?;
    if !points.is_empty() {
        return None;
    }
    match point.bounds_refined(refinement_steps) {
        Classification::Decided(bounds) => Some(bounds),
        Classification::Uncertain(_) => None,
    }
}

enum AlgebraicChordEndpointBoundsRefinement2<'a> {
    AnalyticParallelBezier {
        point: &'a BezierAnalyticParallelPoint2,
        parameter: BezierParameterRefinement2<'a>,
    },
    General {
        endpoint: &'a CurvePoint2,
        policy: &'a CurveContext,
    },
}

impl<'a> AlgebraicChordEndpointBoundsRefinement2<'a> {
    fn new(endpoint: &'a CurvePoint2, policy: &'a CurveContext) -> Self {
        if let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = endpoint
            && let BezierAnalyticParallelPointParameter2::Bezier(parameter) = &point.data.parameter
        {
            return Self::AnalyticParallelBezier {
                point,
                parameter: BezierParameterRefinement2::new(parameter, policy),
            };
        }
        Self::General { endpoint, policy }
    }

    fn refine_to(&mut self, refinement_steps: usize) -> Classification<Aabb2> {
        match self {
            Self::AnalyticParallelBezier { point, parameter } => {
                retained_analytic_parallel_point_bounds_at_bezier_parameter(
                    point,
                    parameter.refine_to(refinement_steps),
                )
            }
            Self::General { endpoint, policy } => {
                algebraic_chord_endpoint_bounds_refined(endpoint, refinement_steps, policy)
            }
        }
    }
}

pub(crate) fn algebraic_chord_endpoint_bounds_refined(
    endpoint: &CurvePoint2,
    refinement_steps: usize,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    algebraic_chord_endpoint_bounds_refined_impl(endpoint, refinement_steps, policy, false)
}

pub(crate) fn algebraic_chord_endpoint_local_bounds_refined(
    endpoint: &CurvePoint2,
    refinement_steps: usize,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    algebraic_chord_endpoint_bounds_refined_impl(endpoint, refinement_steps, policy, true)
}

fn algebraic_chord_endpoint_bounds_refined_impl(
    endpoint: &CurvePoint2,
    refinement_steps: usize,
    policy: &CurveContext,
    local_only: bool,
) -> Classification<Aabb2> {
    let composite = match endpoint {
        CurvePoint2(CurvePointData2::Endpoint(point)) => {
            Some(point.bounds(refinement_steps, policy))
        }
        CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) => {
            Some(point.conservative_bounds_refined(refinement_steps, policy))
        }
        CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) => {
            Some(point.conservative_bounds_refined(refinement_steps, policy))
        }
        CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) => Some(if local_only {
            point.conservative_local_bounds_refined(refinement_steps, policy)
        } else {
            point.conservative_bounds_refined(refinement_steps, policy)
        }),
        CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) => Some(if local_only {
            point.conservative_local_bounds_refined(refinement_steps, policy)
        } else {
            point.conservative_bounds_refined(refinement_steps, policy)
        }),
        CurvePoint2(CurvePointData2::AnalyticParallel(point)) => {
            return point.conservative_bounds_refined(refinement_steps, policy);
        }
        CurvePoint2(CurvePointData2::Similarity(point)) => Some(if local_only {
            point.conservative_local_bounds_refined(refinement_steps, policy)
        } else {
            point.conservative_bounds_refined(refinement_steps, policy)
        }),
        CurvePoint2(CurvePointData2::Exact(_)) | CurvePoint2(CurvePointData2::Algebraic(_)) => None,
    };
    if let Some(bounds) = composite {
        if bounds.is_decided() {
            return bounds;
        }
        if !local_only {
            // A coarse determinant enclosure can contain zero even when its
            // selected intersection is finite. Tighten the existing native
            // evidence before constructing a complete common point field.
            // A tighter conservative box also satisfies the original query.
            for steps in [2, 4, 8, 16] {
                if steps > refinement_steps {
                    let refined =
                        algebraic_chord_endpoint_local_bounds_refined(endpoint, steps, policy);
                    if refined.is_decided() {
                        return refined;
                    }
                }
            }
        }
        if !local_only
            && let Some(bounds) =
                recursive_projective_endpoint_bounds_refined(endpoint, refinement_steps, policy)
        {
            return Classification::Decided(bounds);
        }
        return bounds;
    }
    let CurvePoint2(CurvePointData2::Algebraic(image)) = endpoint else {
        let CurvePoint2(CurvePointData2::Exact(point)) = endpoint else {
            unreachable!();
        };
        return Classification::Decided(Aabb2::from_point(point.clone()));
    };
    if let Some(point) = image.exact_point(policy) {
        return Classification::Decided(Aabb2::from_point(point));
    }
    if let Some(bounds) = image.parametric_source_bounds_refined(refinement_steps, policy) {
        return bounds;
    }
    {
        if let Some((x, y, denominator)) = image.retained_coordinate_polynomials() {
            let parameter = match image.retained_parameter() {
                Some(parameter) => BezierParameter2::Algebraic(parameter.clone()),
                None => match BezierParameter2::from_algebraic_root_representation_unbounded(
                    image.parameter(),
                    policy,
                ) {
                    Ok(Classification::Decided(parameter)) => parameter,
                    Ok(Classification::Uncertain(_)) | Err(_) => {
                        return Classification::Uncertain(UncertaintyReason::Unsupported);
                    }
                },
            }
            .refined_isolating_interval(refinement_steps, policy);
            let parameter = real_interval_from_parameter(&parameter);
            let denominator = RealInterval::evaluate_power_basis(denominator, &parameter);
            let x = RealInterval::evaluate_power_basis(x, &parameter);
            let y = RealInterval::evaluate_power_basis(y, &parameter);
            if let (Some(denominator), Some(x), Some(y)) = (denominator, x, y)
                && let (Some(x), Some(y)) = (x.divide(&denominator), y.divide(&denominator))
            {
                return Classification::Decided(Aabb2::new_unchecked(
                    Point2::new(x.lower, y.lower),
                    Point2::new(x.upper, y.upper),
                ));
            }
        }
    }
    if local_only {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    }
    let Some(image) = image.resolved(policy) else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let (Some(x), Some(y)) = (
        image.x().and_then(|coordinate| coordinate.representation()),
        image.y().and_then(|coordinate| coordinate.representation()),
    ) else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    if !x.is_valid() || !y.is_valid() {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    }
    represented_point_bounds_refined(x, y, refinement_steps)
}

/// Decides whether the squared distance between two retained affine points is
/// at most a represented bound without adjoining their coordinate fields.
///
/// Each refinement encloses both exact points and performs only outward-safe
/// interval arithmetic.  STRICT therefore returns uncertainty when equality
/// cannot be proved, while APPROXIMATE_512 applies its one permitted terminal
/// equality decision only after the complete 512-step enclosure.
pub(crate) fn algebraic_point_distance_squared_at_most(
    first: &CurvePoint2,
    second: &CurvePoint2,
    maximum_squared: &Real,
    policy: &CurveContext,
) -> Classification<bool> {
    if let (Some(first), Some(second)) = (first.coordinates(), second.coordinates()) {
        return match compare_reals(&first.distance_squared(second), maximum_squared, policy) {
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal) => {
                Classification::Decided(true)
            }
            Some(std::cmp::Ordering::Greater) => Classification::Decided(false),
            None => Classification::Uncertain(UncertaintyReason::Ordering),
        };
    }

    let squared_difference_bounds =
        |first_min: &Real, first_max: &Real, second_min: &Real, second_max: &Real| {
            let lower = first_min - second_max;
            let upper = first_max - second_min;
            let zero = Real::zero();
            let lower_sign = compare_reals(&lower, &zero, &CurveContext::STRICT)?;
            let upper_sign = compare_reals(&upper, &zero, &CurveContext::STRICT)?;
            let lower_squared = &lower * &lower;
            let upper_squared = &upper * &upper;
            if upper_sign != std::cmp::Ordering::Greater {
                return Some((upper_squared, lower_squared));
            }
            if lower_sign != std::cmp::Ordering::Less {
                return Some((lower_squared, upper_squared));
            }
            let maximum =
                match compare_reals(&lower_squared, &upper_squared, &CurveContext::STRICT)? {
                    std::cmp::Ordering::Less | std::cmp::Ordering::Equal => upper_squared,
                    std::cmp::Ordering::Greater => lower_squared,
                };
            Some((zero, maximum))
        };

    let mut terminal_refined = false;
    for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let (Classification::Decided(first), Classification::Decided(second)) = (
            algebraic_chord_endpoint_bounds_refined(first, refinement_steps, policy),
            algebraic_chord_endpoint_bounds_refined(second, refinement_steps, policy),
        ) else {
            continue;
        };
        terminal_refined |= refinement_steps == 512;
        let Some((x_lower, x_upper)) = squared_difference_bounds(
            first.min().x(),
            first.max().x(),
            second.min().x(),
            second.max().x(),
        ) else {
            continue;
        };
        let Some((y_lower, y_upper)) = squared_difference_bounds(
            first.min().y(),
            first.max().y(),
            second.min().y(),
            second.max().y(),
        ) else {
            continue;
        };
        let lower = x_lower + y_lower;
        let upper = x_upper + y_upper;
        if matches!(
            compare_reals(&upper, maximum_squared, &CurveContext::STRICT),
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
        ) {
            return Classification::Decided(true);
        }
        if compare_reals(&lower, maximum_squared, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Greater)
        {
            return Classification::Decided(false);
        }
    }
    if terminal_refined && policy.permits_approximate_512() {
        policy.observe_approximate_512();
        Classification::Decided(true)
    } else {
        Classification::Uncertain(UncertaintyReason::Ordering)
    }
}

/// Signs one retained affine point's squared-distance residual against a
/// represented circle without flattening the point's selected field.
///
/// A native selected expression is evaluated in its own field. More deeply
/// retained points use short strict enclosures before shared-field radial
/// replay, which can prove exact incidence. The complete refinement fallback
/// permits only APPROXIMATE_512 to turn a terminal overlap into equality.
pub(crate) fn retained_point_circle_incidence_sign(
    point: &CurvePoint2,
    center: &Point2,
    radius_squared: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    match point {
        CurvePoint2(CurvePointData2::Exact(point)) => {
            return Ok(
                real_sign(&(point.distance_squared(center) - radius_squared), policy).map_or(
                    Classification::Uncertain(UncertaintyReason::RealSign),
                    Classification::Decided,
                ),
            );
        }
        CurvePoint2(CurvePointData2::Algebraic(point)) => {
            let point = match point.predicate_evaluator(policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let (x, y, denominator) = point.coordinate_polynomials();
            let dx = polynomial_subtract(x, &polynomial_scale(denominator, center.x()));
            let dy = polynomial_subtract(y, &polynomial_scale(denominator, center.y()));
            let residual = polynomial_subtract(
                &polynomial_add(
                    &polynomial_multiply(&dx, &dx),
                    &polynomial_multiply(&dy, &dy),
                ),
                &polynomial_scale(
                    &polynomial_multiply(denominator, denominator),
                    radius_squared,
                ),
            );
            return signed_coefficients_at_parameter(&residual, point.retained_parameter(), policy);
        }
        CurvePoint2(CurvePointData2::AnalyticParallel(point)) => {
            return point.circle_residual_sign_to_exact(center, radius_squared, policy);
        }
        CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
        | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
        | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {}
    }

    let radius = RealInterval {
        lower: radius_squared.clone(),
        upper: radius_squared.clone(),
    };
    let exact = |value: &Real| RealInterval {
        lower: value.clone(),
        upper: value.clone(),
    };
    let mut terminal_refined = false;
    for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        // Independent boxes cannot generally prove exact incidence. Reuse
        // selected-root and radical relations after the cheap separation
        // checks, without consuming an approximate terminal in this optional
        // path or projecting independent Cartesian coordinates.
        if refinement_steps == 8
            && let Classification::Decided(Some(sign)) = policy.strict_predicate_pass(|| {
                recursive_projective_point_evidence_circle_residual_sign(
                    point,
                    &CurvePoint2::from(center.clone()),
                    radius_squared,
                    policy,
                )
            })?
        {
            return Ok(Classification::Decided(sign));
        }
        let Classification::Decided(bounds) =
            algebraic_chord_endpoint_bounds_refined(point, refinement_steps, policy)
        else {
            continue;
        };
        terminal_refined |= refinement_steps == 512;
        let delta =
            |axis, coordinate| real_interval_from_axis(&bounds, axis).subtract(&exact(coordinate));
        let Some(residual) = delta(Axis2::X, center.x()).square().and_then(|x| {
            delta(Axis2::Y, center.y())
                .square()
                .map(|y| x.add(&y).subtract(&radius))
        }) else {
            continue;
        };
        let zero = Real::zero();
        if compare_reals(&residual.lower, &zero, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Greater)
        {
            return Ok(Classification::Decided(RealSign::Positive));
        }
        if compare_reals(&residual.upper, &zero, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Less)
        {
            return Ok(Classification::Decided(RealSign::Negative));
        }
        if compare_reals(&residual.lower, &zero, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Equal)
            && compare_reals(&residual.upper, &zero, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal)
        {
            return Ok(Classification::Decided(RealSign::Zero));
        }
    }
    if terminal_refined && policy.permits_approximate_512() {
        policy.observe_approximate_512();
        Ok(Classification::Decided(RealSign::Zero))
    } else {
        Ok(Classification::Uncertain(UncertaintyReason::Ordering))
    }
}

/// Signs one exact retained point projection relative to an algebraic query
/// point without forcing their independent scalar fields into a compositum.
///
/// Disjoint interval projections decide the strict result.  Only the explicit
/// `APPROXIMATE_512` policy may collapse an overlap remaining after the
/// 512-step terminal to equality.
pub(crate) fn retained_point_linear_difference_to_algebraic_sign(
    endpoint: &CurvePoint2,
    query: &RationalBezierAlgebraicPointPredicate2<'_>,
    coefficient_x: &Real,
    coefficient_y: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    if let CurvePoint2(CurvePointData2::Exact(point)) = endpoint {
        return query.homogeneous_linear_difference_sign(
            point.x(),
            point.y(),
            &Real::one(),
            coefficient_x,
            coefficient_y,
            RealSign::Positive,
            policy,
        );
    }

    let coefficient_x = RealInterval::from_values([coefficient_x.clone()])
        .expect("one exact coefficient defines an interval");
    let coefficient_y = RealInterval::from_values([coefficient_y.clone()])
        .expect("one exact coefficient defines an interval");
    let mut terminal_refined = false;
    for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let Classification::Decided(bounds) =
            algebraic_chord_endpoint_bounds_refined(endpoint, refinement_steps, policy)
        else {
            continue;
        };
        terminal_refined |= refinement_steps == 512;
        let Some(projection) = real_interval_from_axis(&bounds, Axis2::X)
            .multiply(&coefficient_x)
            .and_then(|x| {
                real_interval_from_axis(&bounds, Axis2::Y)
                    .multiply(&coefficient_y)
                    .map(|y| x.add(&y))
            })
        else {
            continue;
        };
        match query.linear_order_to_real(
            &coefficient_x.lower,
            &coefficient_y.lower,
            &projection.lower,
            &CurveContext::STRICT,
        )? {
            Classification::Decided(std::cmp::Ordering::Less) => {
                return Ok(Classification::Decided(RealSign::Positive));
            }
            Classification::Decided(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater) => {}
            Classification::Uncertain(_) => {}
        }
        match query.linear_order_to_real(
            &coefficient_x.lower,
            &coefficient_y.lower,
            &projection.upper,
            &CurveContext::STRICT,
        )? {
            Classification::Decided(std::cmp::Ordering::Greater) => {
                return Ok(Classification::Decided(RealSign::Negative));
            }
            Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Equal) => {}
            Classification::Uncertain(_) => {}
        }
        if projection.lower == projection.upper {
            match query.linear_order_to_real(
                &coefficient_x.lower,
                &coefficient_y.lower,
                &projection.lower,
                &CurveContext::STRICT,
            )? {
                Classification::Decided(std::cmp::Ordering::Equal) => {
                    return Ok(Classification::Decided(RealSign::Zero));
                }
                Classification::Decided(std::cmp::Ordering::Less | std::cmp::Ordering::Greater)
                | Classification::Uncertain(_) => {}
            }
        }
    }
    if terminal_refined && policy.permits_approximate_512() {
        policy.observe_approximate_512();
        Ok(Classification::Decided(RealSign::Zero))
    } else {
        Ok(Classification::Uncertain(UncertaintyReason::Predicate))
    }
}

/// Borrowed point-incidence equations shared by membership and winding.
/// A regular cell may supply its oriented primitive normal without requiring
/// endpoint geometry, traversal, or a winding-ray carrier.
struct BezierParallelPointQuery2<'a> {
    parallel: &'a BezierParallel2,
    range: &'a CurveParameterRange2,
    frame: Option<&'a BezierAnalyticParallelTangentField2>,
}

impl BezierParallelPointQuery2<'_> {
    /// The Cartesian incidence theorem also holds in a point's retained
    /// coefficient field. Homogeneous coordinates preserve their correlation;
    /// the common polynomial factor is the contact proof, and one oriented
    /// normal sign selects the authored offset without adjoining a speed root.
    fn visit_point_parameters(
        &self,
        point: &CurvePoint2,
        incident: Option<&BezierParallelIncidentDomain2>,
        domain: CurveParameterDomain2<'_>,
        policy: &CurveContext,
        visitor: &mut impl FnMut(Option<&CurveParameter2>) -> ControlFlow<()>,
    ) -> CurveResult<Classification<ControlFlow<()>>> {
        let strict = policy.strict_counterpart();
        let point = match recursive_projective_evidence_points(&[point], &strict)? {
            Classification::Decided(Some(mut points)) => points.remove(0),
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let field = point.denominator.field();
        let source = self.parallel.source_power_basis()?;
        let unit = [Real::one()];
        let weight = source.weight.unwrap_or(&unit);
        let finite = SelectedThirdAxisDomain2::Finite(domain.finite);
        let extension = incident.map(|incident| SelectedThirdAxisDomain2::IncidentRay {
            anchor: incident.anchor(),
            direction: incident.direction(),
            barrier: incident.barrier(),
        });
        for axis in std::iter::once(finite).chain(extension) {
            match axis.polynomial_is_nonzero(weight, &strict)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        let distance_sign = match real_sign(self.parallel.distance(), &strict) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let tangent = if distance_sign == RealSign::Zero {
            None
        } else {
            let differential = self.parallel.differential()?;
            Some(
                self.frame
                    .map(|frame| (&frame.x[..], &frame.y[..]))
                    .unwrap_or((&differential.tangent_x, &differential.tangent_y)),
            )
        };
        let Some((equations, normal)) = (|| {
            let real = |values: &[Real]| recursive_quadratic_real_polynomial(&field, values);
            let multiply = recursive_quadratic_polynomial_multiply;
            let scale = recursive_quadratic_polynomial_scale;
            let add = |a: &[_], b: &[_]| recursive_quadratic_polynomial_combine(a, b, false);
            let subtract = |a: &[_], b: &[_]| recursive_quadratic_polynomial_combine(a, b, true);
            let weight = real(weight)?;
            // delta = D*W*(point - source), for the retained point (X/D,Y/D).
            let delta_x = subtract(
                &scale(&weight, &point.x)?,
                &scale(&real(source.x_numerator)?, &point.denominator)?,
            )?;
            let delta_y = subtract(
                &scale(&weight, &point.y)?,
                &scale(&real(source.y_numerator)?, &point.denominator)?,
            )?;
            let Some((x, y)) = tangent else {
                return Some(([delta_x, delta_y], None));
            };
            let speed = polynomial_add(&polynomial_multiply(x, x), &polynomial_multiply(y, y));
            let x = real(x)?;
            let y = real(y)?;
            let orthogonality = add(&multiply(&delta_x, &x)?, &multiply(&delta_y, &y)?)?;
            let weighted_distance = recursive_quadratic_polynomial_scale_real(
                &scale(&weight, &point.denominator)?,
                self.parallel.distance(),
            )?;
            let distance = subtract(
                &add(
                    &multiply(&delta_x, &delta_x)?,
                    &multiply(&delta_y, &delta_y)?,
                )?,
                &multiply(&weighted_distance, &weighted_distance)?,
            )?;
            let orientation = multiply(
                &subtract(&multiply(&delta_y, &x)?, &multiply(&delta_x, &y)?)?,
                &weighted_distance,
            )?;
            Some(([orthogonality, distance], Some((speed, orientation))))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let common = match hypersolve::ordered_field_polynomial_gcd(
            &equations[0],
            &equations[1],
            &mut BezierRecursiveOrderedFieldContext2 {
                field: field.clone(),
                policy: strict,
            },
        ) {
            Ok(common) => common,
            Err(BezierRecursiveOrderedFieldError2::Curve(error)) => return Err(error),
            Err(BezierRecursiveOrderedFieldError2::Uncertain) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
        };
        if common.is_empty() {
            if let Some((_, orientation)) = &normal {
                // Both equations are polynomial identities. On this connected
                // source domain, nonzero weight, distance and speed make the
                // branch continuous and nonzero; one strict sign chooses the
                // whole authored sheet, including its incident continuation.
                for axis in std::iter::once(finite).chain(extension) {
                    if let Classification::Uncertain(reason) = self
                        .parallel
                        .certify_source_frame_in_domain(axis, self.frame, &strict)?
                    {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                match recursive_projective_polynomial_sign_at_parameter(
                    &field,
                    orientation,
                    domain.finite.start(),
                    &strict,
                )? {
                    Classification::Decided(RealSign::Positive) => {}
                    Classification::Decided(RealSign::Negative) => {
                        return Ok(Classification::Decided(ControlFlow::Continue(())));
                    }
                    Classification::Decided(RealSign::Zero) => {
                        return Err(CurveError::Topology(
                            "a regular collapsed parallel lost its normal branch".into(),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            return Ok(Classification::Decided(visitor(None)));
        }
        for axis in std::iter::once(finite).chain(extension) {
            let candidates = match recursive_projective_polynomial_parameters(
                &field,
                common.clone(),
                axis,
                &strict,
            )? {
                Classification::Decided(candidates) => candidates,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            for candidate in candidates {
                match CurveParameterDomain2::new(self.range, None)
                    .contains_finite_parameter(&candidate, &strict)?
                {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        let Some(incident) = incident else {
                            continue;
                        };
                        match incident.contains_extension_parameter(&candidate, &strict)? {
                            Classification::Decided(true) => {}
                            Classification::Decided(false) => continue,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                if let Some((speed, orientation)) = &normal {
                    match candidate.polynomial_sign(speed, &strict)? {
                        Classification::Decided(RealSign::Positive) => {}
                        Classification::Decided(RealSign::Zero) => {
                            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                        }
                        Classification::Decided(RealSign::Negative) => {
                            return Err(CurveError::Topology(
                                "a recursive point incidence had negative squared speed".into(),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                    match recursive_projective_polynomial_sign_at_parameter(
                        &field,
                        orientation,
                        &candidate,
                        &strict,
                    )? {
                        Classification::Decided(RealSign::Positive) => {}
                        Classification::Decided(RealSign::Negative) => continue,
                        Classification::Decided(RealSign::Zero) => {
                            return Err(CurveError::Topology(
                                "a regular point incidence lost its normal branch".into(),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                if let stop @ ControlFlow::Break(()) = visitor(Some(&candidate)) {
                    return Ok(Classification::Decided(stop));
                }
            }
        }
        Ok(Classification::Decided(ControlFlow::Continue(())))
    }

    fn system(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        factor_x: &Real,
        factor_y: &Real,
    ) -> CurveResult<BezierParallelAlgebraicIncidenceSystem2> {
        let source = self.parallel.source_power_basis()?;
        let differential = self.parallel.differential()?;
        let (tangent_x, tangent_y) = self
            .frame
            .map(|frame| (&frame.x[..], &frame.y[..]))
            .unwrap_or((&differential.tangent_x, &differential.tangent_y));
        let weight = source
            .weight
            .map_or_else(|| vec![Real::one()], <[Real]>::to_vec);
        let (query_x, query_y, query_weight) = point.coordinate_polynomials();
        let query_linear = polynomial_add(
            &polynomial_scale(query_x, factor_x),
            &polynomial_scale(query_y, factor_y),
        );
        let source_linear = polynomial_add(
            &polynomial_scale(source.x_numerator, factor_x),
            &polynomial_scale(source.y_numerator, factor_y),
        );
        // `source_term = qW*W*f·(P-query)`.
        let source_term = bivariate_subtract(
            &bivariate_outer_product(query_weight, &source_linear),
            &bivariate_outer_product(&query_linear, &weight),
        );
        if real_sign(self.parallel.distance(), &CurveContext::STRICT) == Some(RealSign::Zero) {
            return Ok(BezierParallelAlgebraicIncidenceSystem2 {
                incidence: source_term.clone(),
                expression: BezierAlgebraicCuspTwoTermExpression2 {
                    rational: source_term,
                    radical: BivariatePolynomial::new(vec![vec![Real::zero()]]),
                },
                speed_squared: BivariatePolynomial::new(vec![vec![Real::one()]]),
            });
        }
        let normal_projection = polynomial_add(
            &polynomial_scale(tangent_y, &(-factor_x.clone())),
            &polynomial_scale(tangent_x, factor_y),
        );
        // The common qW*W denominator gives
        // `source_term*sqrt(S) + normal_term`.
        let normal_term = bivariate_scale(
            bivariate_outer_product(
                query_weight,
                &polynomial_multiply(&weight, &normal_projection),
            ),
            self.parallel.distance(),
        );
        let speed_squared = bivariate_outer_product(
            &[Real::one()],
            &polynomial_add(
                &polynomial_multiply(tangent_x, tangent_x),
                &polynomial_multiply(tangent_y, tangent_y),
            ),
        );
        let incidence = bivariate_subtract(
            &bivariate_multiply(&normal_term, &normal_term),
            &bivariate_multiply(
                &bivariate_multiply(&source_term, &source_term),
                &speed_squared,
            ),
        );
        Ok(BezierParallelAlgebraicIncidenceSystem2 {
            incidence,
            expression: BezierAlgebraicCuspTwoTermExpression2 {
                rational: normal_term,
                radical: source_term,
            },
            speed_squared,
        })
    }

    fn projected_parameters(
        system: &BezierParallelAlgebraicIncidenceSystem2,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        domain: CurveParameterDomain2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<BezierAlgebraicFiberProjection2>> {
        policy.strict_predicate_pass(|| match point.retained_parameter() {
            BezierParameter2::Exact(parameter) => {
                let coefficients = bivariate_specialize_first(&system.incidence, parameter);
                let polynomial = match polynomial_from_coefficients(coefficients, policy)? {
                    Classification::Decided(Some(polynomial)) => polynomial,
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(
                            BezierAlgebraicFiberProjection2::IdenticallyZero,
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                Ok(
                    polynomial_roots_in_parameter_domain(&polynomial, domain, policy)?
                        .map(BezierAlgebraicFiberProjection2::Parameters),
                )
            }
            BezierParameter2::Algebraic(parameter) => {
                if domain.extension.is_some() {
                    algebraic_selected_fiber_parameters_with_incident_ray(
                        &system.incidence,
                        parameter,
                        domain,
                        MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE,
                        MAX_SELECTED_FIBER_QUOTIENT_DEGREE,
                        policy,
                    )
                } else {
                    selected_axis_parameters_in_domain(domain, policy, |axis_domain| {
                        let SelectedThirdAxisDomain2::Finite(range) = axis_domain else {
                            unreachable!("this query has no incident extension")
                        };
                        selected_fiber_parameters(
                            &system.incidence,
                            &BezierParameter2::Algebraic(parameter.clone()),
                            range,
                            policy,
                        )
                    })
                }
            }
        })
    }

    fn expression_sign_at_candidate(
        &self,
        authority: &BezierParallelAlgebraicIncidenceSystem2,
        expression: &BezierParallelAlgebraicIncidenceSystem2,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let expression_sign = match algebraic_cusp_selected_square_root_sum_sign(
            &authority.incidence,
            &expression.expression,
            &expression.speed_squared,
            point.retained_parameter(),
            parameter,
            policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let source = self.parallel.source_power_basis()?;
        let weight = source
            .weight
            .map_or_else(|| vec![Real::one()], <[Real]>::to_vec);
        let weight_sign = match signed_coefficients_at_parameter(&weight, parameter, policy)? {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(product_sign(
            expression_sign,
            product_sign(point.denominator_sign(), weight_sign),
        )))
    }
}

impl BezierParallelAlgebraicRay2 {
    pub(crate) fn try_new(
        parallel: BezierParallel2,
        range: CurveParameterRange2,
        reversed: bool,
        endpoints: [CurvePoint2; 2],
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if let Classification::Uncertain(reason) = parallel.certify_source_frame_in_domain(
            SelectedThirdAxisDomain2::Finite(&range),
            None,
            policy,
        )? {
            return Ok(Classification::Uncertain(reason));
        }
        let descending = match policy
            .strict_predicate_pass(|| range.start().cmp_by_refinement(range.end(), policy))?
        {
            Classification::Decided(order) => order.is_gt(),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let range = if descending {
            CurveParameterRange2::new_validated(range.end().clone(), range.start().clone())
        } else {
            range
        };
        let reversed = reversed != descending;
        Ok(Classification::Decided(Self {
            parallel,
            range,
            reversed,
            endpoints,
        }))
    }

    pub(crate) fn endpoint_side_signs(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        side_x: &Real,
        side_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[RealSign; 2]>> {
        let mut signs = [RealSign::Zero; 2];
        for (index, endpoint) in self.endpoints.iter().enumerate() {
            signs[index] = match retained_point_linear_difference_to_algebraic_sign(
                endpoint, point, side_x, side_y, policy,
            )? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        }
        Ok(Classification::Decided(signs))
    }

    fn parameter_orders(
        &self,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<(std::cmp::Ordering, std::cmp::Ordering)>> {
        let parameter = CurveParameter2::from(parameter.clone());
        let start = match parameter.cmp_by_refinement(self.range.start(), policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match parameter.cmp_by_refinement(self.range.end(), policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided((start, end)))
    }

    pub(crate) fn contains_point(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        incident: Option<&BezierParallelIncidentDomain2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let query = BezierParallelPointQuery2 {
            parallel: &self.parallel,
            range: &self.range,
            frame: None,
        };
        let expanded = match incident
            .map(|incident| incident.expanded_range(&self.range, policy))
            .transpose()?
        {
            Some(Classification::Decided(range)) => Some(range),
            Some(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            None => None,
        };
        let domain = CurveParameterDomain2::new(
            expanded.as_ref().unwrap_or(&self.range),
            incident.map(BezierParallelIncidentDomain2::parameter_ray),
        );
        let retained = CurvePoint2::from(point.point_image().clone());
        Ok(query
            .visit_point_parameters(&retained, incident, domain, policy, &mut |_| {
                ControlFlow::Break(())
            })?
            .map(|flow| flow.is_break()))
    }

    /// Omits every transverse contact at an algebraic boundary-side ray
    /// origin while retaining all other contacts on this finite parallel.
    pub(crate) fn forward_ray_winding_delta_skipping_incident_origin(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<i32>>> {
        match self.contains_point(point, None, policy)? {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        self.forward_ray_winding_delta_with_origin_mode(
            point,
            direction_x,
            direction_y,
            true,
            policy,
        )
        .map(|delta| delta.map(Some))
    }

    fn neighbor_sample(
        &self,
        parameters: &[BezierParameter2],
        index: usize,
        after: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Real>> {
        let root = CurveParameter2::from(parameters[index].clone());
        let mut boundary = if after {
            self.range.end().clone()
        } else {
            self.range.start().clone()
        };
        let neighbor = if after {
            parameters.get(index + 1)
        } else {
            index.checked_sub(1).and_then(|index| parameters.get(index))
        };
        if let Some(neighbor) = neighbor {
            let neighbor = CurveParameter2::from(neighbor.clone());
            let order = match neighbor.cmp_by_refinement(&boundary, policy)? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if after && order.is_lt() || !after && order.is_gt() {
                boundary = neighbor;
            }
        }
        if after {
            root.strict_scalar_between_ordered(&boundary, policy)
        } else {
            boundary.strict_scalar_between_ordered(&root, policy)
        }
    }

    fn exact_sample_sign(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        parameter: &Real,
        factor_x: &Real,
        factor_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RealSign>> {
        let sample = match self.parallel.point_at(parameter, policy)? {
            Classification::Decided(sample) => sample,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        point.homogeneous_linear_difference_sign(
            sample.x(),
            sample.y(),
            &Real::one(),
            factor_x,
            factor_y,
            RealSign::Positive,
            policy,
        )
    }

    pub(crate) fn forward_ray_winding_delta(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        self.forward_ray_winding_delta_with_origin_mode(
            point,
            direction_x,
            direction_y,
            false,
            policy,
        )
    }

    fn forward_ray_winding_delta_with_origin_mode(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        direction_x: &Real,
        direction_y: &Real,
        skip_incident_origin: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        let query = BezierParallelPointQuery2 {
            parallel: &self.parallel,
            range: &self.range,
            frame: None,
        };
        let side_x = -direction_y.clone();
        let side_y = direction_x.clone();
        let side = query.system(point, &side_x, &side_y)?;
        let ahead = query.system(point, direction_x, direction_y)?;
        let parameters = match BezierParallelPointQuery2::projected_parameters(
            &side,
            point,
            CurveParameterDomain2::new(&self.range, None),
            policy,
        )? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(parameters)) => {
                parameters
            }
            Classification::Decided(
                BezierAlgebraicFiberProjection2::IdenticallyZero
                | BezierAlgebraicFiberProjection2::Degenerate,
            ) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut winding = 0_i32;
        for (index, parameter) in parameters.iter().enumerate() {
            match query.expression_sign_at_candidate(&side, &side, point, parameter, policy)? {
                Classification::Decided(RealSign::Zero) => {}
                Classification::Decided(RealSign::Positive | RealSign::Negative) => continue,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
            let (start_order, end_order) = match self.parameter_orders(parameter, policy)? {
                Classification::Decided(orders) => orders,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if start_order.is_lt() || end_order.is_gt() {
                continue;
            }
            if start_order.is_eq() || end_order.is_eq() {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            let at_origin = match query
                .expression_sign_at_candidate(&side, &ahead, point, parameter, policy)?
            {
                Classification::Decided(RealSign::Positive) => false,
                Classification::Decided(RealSign::Negative) => continue,
                Classification::Decided(RealSign::Zero) => {
                    if !skip_incident_origin {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                    true
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let before = match self.neighbor_sample(&parameters, index, false, policy)? {
                Classification::Decided(sample) => {
                    match self.exact_sample_sign(point, &sample, &side_x, &side_y, policy)? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let after = match self.neighbor_sample(&parameters, index, true, policy)? {
                Classification::Decided(sample) => {
                    match self.exact_sample_sign(point, &sample, &side_x, &side_y, policy)? {
                        Classification::Decided(sign) => sign,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let delta = match (before, after) {
                (RealSign::Negative, RealSign::Positive) => 1_i32,
                (RealSign::Positive, RealSign::Negative) => -1_i32,
                (RealSign::Negative, RealSign::Negative)
                | (RealSign::Positive, RealSign::Positive) => 0_i32,
                (RealSign::Zero, _) | (_, RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
            };
            if at_origin {
                if delta == 0 {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                continue;
            }
            winding = winding
                .checked_add(if self.reversed { -delta } else { delta })
                .ok_or_else(|| {
                    CurveError::Topology("analytic-parallel ray winding overflow".into())
                })?;
        }
        Ok(Classification::Decided(winding))
    }
}

fn retained_point_evidence_equality_by_refinement(
    first: &CurvePoint2,
    second: &CurvePoint2,
    policy: &CurveContext,
) -> Classification<bool> {
    if first.shares_storage(second) {
        return Classification::Decided(true);
    }
    if policy.has_bounded_exact_predicate_budget() {
        // Even a zero-step general envelope can require constructing a
        // recursive composite point. Bounded callers already own their
        // representation-local boxes and may safely decline to the complete
        // equality authority instead of importing that coordinate tower.
        return Classification::Uncertain(UncertaintyReason::Predicate);
    }
    let bounds_equality = |refinement_steps| {
        let (Classification::Decided(first), Classification::Decided(second)) = (
            algebraic_chord_endpoint_bounds_refined(first, refinement_steps, policy),
            algebraic_chord_endpoint_bounds_refined(second, refinement_steps, policy),
        ) else {
            return None;
        };
        if first.overlaps(&second, policy) == Classification::Decided(false) {
            return Some(false);
        }
        let first_is_point = compare_reals(first.min().x(), first.max().x(), &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Equal)
            && compare_reals(first.min().y(), first.max().y(), &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal);
        let second_is_point =
            compare_reals(second.min().x(), second.max().x(), &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal)
                && compare_reals(second.min().y(), second.max().y(), &CurveContext::STRICT)
                    == Some(std::cmp::Ordering::Equal);
        if first_is_point
            && second_is_point
            && compare_reals(first.min().x(), second.min().x(), &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal)
            && compare_reals(first.min().y(), second.min().y(), &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal)
        {
            return Some(true);
        }
        None
    };
    for refinement_steps in [0, 2, 4, 8] {
        if let Some(equal) = bounds_equality(refinement_steps) {
            return Classification::Decided(equal);
        }
    }
    let source_and_point = match (&first.0, &second.0) {
        (CurvePointData2::AnalyticParallel(source), CurvePointData2::Exact(point))
        | (CurvePointData2::Exact(point), CurvePointData2::AnalyticParallel(source)) => {
            Some((source, point))
        }
        _ => None,
    };
    if let Some((source, point)) = source_and_point
        && let Ok(Some(equal)) =
            source.rational_circle_source_point_equality(point, &policy.strict_counterpart())
    {
        return Classification::Decided(equal);
    }
    // Equality boxes never separate. Before growing their rational endpoints
    // through hundreds of bisections, compare the exact represented
    // coordinates already owned by both carriers. Unsupported coordinate
    // forms retain the bounded interval fallback below.
    if let Some(equal) =
        policy.strict_predicate_pass(|| represented_point_evidence_equality(first, second, policy))
    {
        return Classification::Decided(equal);
    }
    let mut terminal_refined = false;
    for refinement_steps in [16, 32, 64, 128, 256, 512] {
        terminal_refined |= refinement_steps == 512;
        if let Some(equal) = bounds_equality(refinement_steps) {
            return Classification::Decided(equal);
        }
    }
    if terminal_refined && policy.permits_approximate_512() {
        policy.observe_approximate_512();
        Classification::Decided(true)
    } else {
        Classification::Uncertain(UncertaintyReason::Predicate)
    }
}

/// Compares two retained affine points through their final exact algebraic
/// coordinate representations.
///
/// Carrier-native incidence and interval separation remain the fast paths.
/// This cold fallback is the common authority for equality cases whose boxes
/// converge to the same point without ever becoming singleton `Real` boxes.
/// `represented_roots_equal` applies the requested terminal policy, including
/// APPROXIMATE_512 only after exact root comparison is exhausted.
fn represented_point_evidence_equality(
    first: &CurvePoint2,
    second: &CurvePoint2,
    policy: &CurveContext,
) -> Option<bool> {
    let (Ok(Classification::Decided(first)), Ok(Classification::Decided(second))) = (
        represented_point_evidence_coordinates(first, policy),
        represented_point_evidence_coordinates(second, policy),
    ) else {
        return None;
    };
    let x = crate::bezier_arrangement::represented_roots_equal(&first[0], &second[0], policy);
    if x == Some(false) {
        return Some(false);
    }
    let y = crate::bezier_arrangement::represented_roots_equal(&first[1], &second[1], policy);
    match (x, y) {
        (Some(true), Some(true)) => Some(true),
        (_, Some(false)) => Some(false),
        _ => None,
    }
}

pub(crate) fn algebraic_chord_point_linear_order_to_exact(
    point: &CurvePoint2,
    origin: &Point2,
    coefficient_x: &Real,
    coefficient_y: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<std::cmp::Ordering>> {
    if let CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) = point {
        let value = coefficient_x * origin.x() + coefficient_y * origin.y();
        return point.linear_order_to_real(coefficient_x, coefficient_y, &value, policy);
    }
    let cardinal_axis = if coefficient_y.zero_status() == ZeroKnowledge::Zero {
        Some((Axis2::X, origin.x(), coefficient_x))
    } else if coefficient_x.zero_status() == ZeroKnowledge::Zero {
        Some((Axis2::Y, origin.y(), coefficient_y))
    } else {
        None
    };
    if let Some((axis, coordinate, coefficient)) = cardinal_axis
        && let Some(sign) = real_sign(coefficient, &CurveContext::STRICT)
    {
        return match sign {
            RealSign::Zero => Ok(Classification::Decided(std::cmp::Ordering::Equal)),
            RealSign::Positive => {
                BezierAlgebraicChord2::point_axis_order_to_real(point, axis, coordinate, policy)
            }
            RealSign::Negative => Ok(BezierAlgebraicChord2::point_axis_order_to_real(
                point, axis, coordinate, policy,
            )?
            .map(std::cmp::Ordering::reverse)),
        };
    }
    let coefficient_x_interval = RealInterval::from_values([coefficient_x.clone()])
        .expect("one exact coefficient defines an interval");
    let coefficient_y_interval = RealInterval::from_values([coefficient_y.clone()])
        .expect("one exact coefficient defines an interval");
    let origin_x = RealInterval::from_values([origin.x().clone()])
        .expect("one exact coordinate defines an interval");
    let origin_y = RealInterval::from_values([origin.y().clone()])
        .expect("one exact coordinate defines an interval");
    let interval_order = |refinement_steps| {
        let Classification::Decided(bounds) =
            algebraic_chord_endpoint_bounds_refined(point, refinement_steps, policy)
        else {
            return None;
        };
        let delta_x = real_interval_from_axis(&bounds, Axis2::X).subtract(&origin_x);
        let delta_y = real_interval_from_axis(&bounds, Axis2::Y).subtract(&origin_y);
        let Some(value) = delta_x
            .multiply(&coefficient_x_interval)
            .and_then(|x| delta_y.multiply(&coefficient_y_interval).map(|y| x.add(&y)))
        else {
            return Some(None);
        };
        let zero = Real::zero();
        if compare_reals(&value.lower, &zero, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Greater)
        {
            return Some(Some(std::cmp::Ordering::Greater));
        }
        if compare_reals(&value.upper, &zero, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Less)
        {
            return Some(Some(std::cmp::Ordering::Less));
        }
        if compare_reals(&value.lower, &zero, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Equal)
            && compare_reals(&value.upper, &zero, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal)
        {
            return Some(Some(std::cmp::Ordering::Equal));
        }
        Some(None)
    };
    for refinement_steps in [0, 2, 4, 8, 16] {
        if let Some(Some(order)) = interval_order(refinement_steps) {
            return Ok(Classification::Decided(order));
        }
    }
    if point.same_point(&CurvePoint2::from(origin.clone()), policy) == Classification::Decided(true)
    {
        return Ok(Classification::Decided(std::cmp::Ordering::Equal));
    }
    // Preserve authored correlation before splitting a singly selected
    // rational point into independent Cartesian algebraic images.  An
    // oblique projection can cancel shared coordinate terms coefficientwise
    // even when each coordinate contains opaque Real coefficients that the
    // represented-root package cannot normalize independently.
    if let CurvePoint2(CurvePointData2::Algebraic(point)) = point {
        let value = coefficient_x * origin.x() + coefficient_y * origin.y();
        let order = policy.strict_predicate_pass(|| -> CurveResult<_> {
            let predicate = match point.predicate_evaluator(policy)? {
                Classification::Decided(predicate) => predicate,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            predicate.linear_order_to_real(coefficient_x, coefficient_y, &value, policy)
        })?;
        if order.is_decided() {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-point-linear-order",
                "retained-field-projection",
            );
            return Ok(order);
        }
    }
    // Independently refined Cartesian boxes cannot prove a correlated
    // oblique projection is exactly zero. Compare the retained final
    // algebraic image before applying the policy terminal; this is the same
    // exact cold authority used by point equality and axis ordering.
    if !policy.has_bounded_exact_predicate_budget()
        && let Classification::Decided([x, y]) = policy
            .strict_predicate_pass(|| represented_point_evidence_coordinates(point, policy))?
    {
        let offset = -(coefficient_x * origin.x() + coefficient_y * origin.y());
        if let Classification::Decided(projection) = Classification::from(
            represented_affine_coordinate(&[(&x, coefficient_x), (&y, coefficient_y)], &offset),
        ) && let Some(sign) = represented_strict_sign(&projection)
        {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-point-linear-order",
                "represented-cold-fallback",
            );
            return Ok(Classification::Decided(match sign {
                RealSign::Negative => std::cmp::Ordering::Less,
                RealSign::Zero => std::cmp::Ordering::Equal,
                RealSign::Positive => std::cmp::Ordering::Greater,
            }));
        }
    }
    let mut terminal_refined = false;
    for refinement_steps in [32, 64, 128, 256, 512] {
        match interval_order(refinement_steps) {
            Some(Some(order)) => return Ok(Classification::Decided(order)),
            Some(None) => terminal_refined |= refinement_steps == 512,
            None => {}
        }
    }
    if terminal_refined && policy.permits_approximate_512() {
        policy.observe_approximate_512();
        Ok(Classification::Decided(std::cmp::Ordering::Equal))
    } else {
        Ok(Classification::Uncertain(UncertaintyReason::Ordering))
    }
}

pub(crate) fn algebraic_chord_points_linear_order(
    first: &CurvePoint2,
    second: &CurvePoint2,
    coefficient_x: &Real,
    coefficient_y: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<std::cmp::Ordering>> {
    if let CurvePoint2(CurvePointData2::Exact(second)) = second {
        return algebraic_chord_point_linear_order_to_exact(
            first,
            second,
            coefficient_x,
            coefficient_y,
            policy,
        );
    }
    if let CurvePoint2(CurvePointData2::Exact(first)) = first {
        return Ok(algebraic_chord_point_linear_order_to_exact(
            second,
            first,
            coefficient_x,
            coefficient_y,
            policy,
        )?
        .map(std::cmp::Ordering::reverse));
    }
    if first.same_point(second, policy) == Classification::Decided(true) {
        return Ok(Classification::Decided(std::cmp::Ordering::Equal));
    }
    let coefficient_x_interval = RealInterval::from_values([coefficient_x.clone()])
        .expect("one exact coefficient defines an interval");
    let coefficient_y_interval = RealInterval::from_values([coefficient_y.clone()])
        .expect("one exact coefficient defines an interval");
    let interval_order = |refinement_steps| {
        let (Classification::Decided(first), Classification::Decided(second)) = (
            algebraic_chord_endpoint_bounds_refined(first, refinement_steps, policy),
            algebraic_chord_endpoint_bounds_refined(second, refinement_steps, policy),
        ) else {
            return None;
        };
        let delta_x = real_interval_from_axis(&first, Axis2::X)
            .subtract(&real_interval_from_axis(&second, Axis2::X));
        let delta_y = real_interval_from_axis(&first, Axis2::Y)
            .subtract(&real_interval_from_axis(&second, Axis2::Y));
        let Some(value) = delta_x
            .multiply(&coefficient_x_interval)
            .and_then(|x| delta_y.multiply(&coefficient_y_interval).map(|y| x.add(&y)))
        else {
            return Some(None);
        };
        let zero = Real::zero();
        if compare_reals(&value.lower, &zero, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Greater)
        {
            return Some(Some(std::cmp::Ordering::Greater));
        }
        if compare_reals(&value.upper, &zero, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Less)
        {
            return Some(Some(std::cmp::Ordering::Less));
        }
        if compare_reals(&value.lower, &zero, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Equal)
            && compare_reals(&value.upper, &zero, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal)
        {
            return Some(Some(std::cmp::Ordering::Equal));
        }
        Some(None)
    };
    for refinement_steps in [0, 2, 4, 8, 16] {
        if let Some(Some(order)) = interval_order(refinement_steps) {
            return Ok(Classification::Decided(order));
        }
    }

    // Preserve each endpoint's one-field coordinate correlation before
    // comparing independent selected fields. This handles exact oblique
    // equality without constructing a four-variable Cartesian tensor image.
    let retained_projections = match (first, second) {
        (
            CurvePoint2(CurvePointData2::Algebraic(first)),
            CurvePoint2(CurvePointData2::Algebraic(second)),
        ) => first
            .strict_linear_projection_representation(coefficient_x, coefficient_y)
            .zip(second.strict_linear_projection_representation(coefficient_x, coefficient_y)),
        _ => None,
    };
    if let Some((first, second)) = retained_projections.as_ref()
        && let Some(order) =
            compare_algebraic_representations_with_policy(first, second, &CurveContext::STRICT)
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-points-linear-order",
            "retained-field-projections",
        );
        return Ok(Classification::Decided(order));
    }

    let mut terminal_refined = false;
    for refinement_steps in [32, 64, 128, 256, 512] {
        match interval_order(refinement_steps) {
            Some(Some(order)) => return Ok(Classification::Decided(order)),
            Some(None) => terminal_refined |= refinement_steps == 512,
            None => {}
        }
    }
    if let Some((first, second)) = retained_projections.as_ref()
        && let Some(order) = compare_algebraic_representations_with_policy(first, second, policy)
    {
        return Ok(Classification::Decided(order));
    }
    if terminal_refined && policy.permits_approximate_512() {
        policy.observe_approximate_512();
        Ok(Classification::Decided(std::cmp::Ordering::Equal))
    } else {
        Ok(Classification::Uncertain(UncertaintyReason::Ordering))
    }
}

fn algebraic_chord_image_parameter(
    image: &RationalBezierAlgebraicPointImage2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicParameter2>> {
    if let Some(parameter) = image.retained_parameter() {
        return Ok(Classification::Decided(parameter.clone()));
    }
    Ok(
        match BezierParameter2::from_algebraic_root_representation_unbounded(
            image.parameter(),
            policy,
        )? {
            Classification::Decided(BezierParameter2::Algebraic(parameter)) => {
                Classification::Decided(parameter)
            }
            Classification::Decided(BezierParameter2::Exact(_)) => {
                Classification::Uncertain(UncertaintyReason::Unsupported)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

fn algebraic_chord_endpoint_images(
    start: &CurvePoint2,
    end: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<[RationalBezierAlgebraicPointImage2; 2]>> {
    if matches!(
        start,
        CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_))
    ) || matches!(
        end,
        CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_))
    ) {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let normalized = |point: &CurvePoint2| match point {
        CurvePoint2(CurvePointData2::Exact(point)) => (Some(point.clone()), None),
        CurvePoint2(CurvePointData2::Algebraic(image)) => image
            .exact_point(policy)
            .map_or((None, Some(image.clone())), |point| (Some(point), None)),
        CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
        | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
        | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
        | CurvePoint2(CurvePointData2::AnalyticParallel(_))
        | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
            unreachable!("correlated chord endpoints return before algebraic image normalization")
        }
    };
    let (start_exact, start_algebraic) = normalized(start);
    let (end_exact, end_algebraic) = normalized(end);
    match (start_exact, start_algebraic, end_exact, end_algebraic) {
        (None, Some(start), None, Some(end)) => Ok(Classification::Decided([start, end])),
        (Some(start), None, None, Some(end)) => {
            let parameter = match algebraic_chord_image_parameter(&end, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            Ok(Classification::Decided([
                algebraic_constant_point_image(&start, &parameter, policy),
                end,
            ]))
        }
        (None, Some(start), Some(end), None) => {
            let parameter = match algebraic_chord_image_parameter(&start, policy)? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            Ok(Classification::Decided([
                start,
                algebraic_constant_point_image(&end, &parameter, policy),
            ]))
        }
        (Some(_), None, Some(_), None) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        _ => Err(CurveError::Topology(
            "algebraic chord endpoint normalization was inconsistent".into(),
        )),
    }
}

impl BezierAlgebraicChordAlgebraicRay2 {
    fn exact_point_image(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierAlgebraicPointImage2>> {
        let parameter = match algebraic_chord_image_parameter(&self.start, policy)? {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(algebraic_constant_point_image(
            point, &parameter, policy,
        )))
    }

    fn endpoint_predicates<'a>(
        &'a self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[RationalBezierAlgebraicPointPredicate2<'a>; 2]>> {
        let start = match self.start.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end = match self.end.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided([start, end]))
    }

    pub(crate) fn endpoint_side_signs(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        side_x: &Real,
        side_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<[RealSign; 2]>> {
        let [start, end] = match self.endpoint_predicates(policy)? {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let start = match signed_algebraic_point_linear_difference(
            &start, point, side_x, side_y, policy,
        )? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end =
            match signed_algebraic_point_linear_difference(&end, point, side_x, side_y, policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        Ok(Classification::Decided([start, end]))
    }

    pub(crate) fn contains_exact_point(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let point = match self.exact_point_image(point, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let point = match point.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.contains_point(&point, policy)
    }

    pub(crate) fn contains_point(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let [start, end] = match self.endpoint_predicates(policy)? {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        match algebraic_point_oriented_line_side(&start, &end, point, policy)? {
            Classification::Decided(crate::classify::LineSide::On) => {}
            Classification::Decided(
                crate::classify::LineSide::Left | crate::classify::LineSide::Right,
            ) => return Ok(Classification::Decided(false)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        let one = Real::one();
        let zero = Real::zero();
        let axis =
            match signed_algebraic_point_linear_difference(&end, &start, &one, &zero, policy)? {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => (&one, &zero),
                Classification::Decided(RealSign::Zero) => (&zero, &one),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let start_order = match algebraic_point_linear_order(&start, point, axis.0, axis.1, policy)?
        {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let end_order = match algebraic_point_linear_order(&end, point, axis.0, axis.1, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(!matches!(
            (start_order, end_order),
            (std::cmp::Ordering::Less, std::cmp::Ordering::Less)
                | (std::cmp::Ordering::Greater, std::cmp::Ordering::Greater)
        )))
    }

    pub(crate) fn forward_ray_winding_delta_from_exact(
        &self,
        point: &Point2,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        let point = match self.exact_point_image(point, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let point = match point.predicate_evaluator(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.forward_ray_winding_delta(&point, direction_x, direction_y, policy)
    }

    pub(crate) fn forward_ray_winding_delta(
        &self,
        point: &RationalBezierAlgebraicPointPredicate2<'_>,
        direction_x: &Real,
        direction_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<i32>> {
        let [start, end] = match self.endpoint_predicates(policy)? {
            Classification::Decided(endpoints) => endpoints,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let side_x = -direction_y.clone();
        let side_y = direction_x.clone();
        let start_side =
            match algebraic_point_linear_order(&start, point, &side_x, &side_y, policy)? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let end_side = match algebraic_point_linear_order(&end, point, &side_x, &side_y, policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if matches!(
            (start_side, end_side),
            (std::cmp::Ordering::Less, std::cmp::Ordering::Less)
                | (std::cmp::Ordering::Greater, std::cmp::Ordering::Greater)
        ) {
            return Ok(Classification::Decided(0));
        }
        let side = match algebraic_point_oriented_line_side(&start, &end, point, policy)? {
            Classification::Decided(side) => side,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if side == crate::classify::LineSide::On {
            return match self.contains_point(point, policy)? {
                Classification::Decided(true) => {
                    Ok(Classification::Uncertain(UncertaintyReason::Boundary))
                }
                Classification::Decided(false) => {
                    Ok(Classification::Uncertain(UncertaintyReason::Predicate))
                }
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            };
        }
        Ok(Classification::Decided(
            if start_side != std::cmp::Ordering::Greater
                && end_side == std::cmp::Ordering::Greater
                && side == crate::classify::LineSide::Left
            {
                1
            } else if start_side == std::cmp::Ordering::Greater
                && end_side != std::cmp::Ordering::Greater
                && side == crate::classify::LineSide::Right
            {
                -1
            } else {
                0
            },
        ))
    }
}

impl BezierAlgebraicChordSupportPredicate2 {
    fn try_new(
        chord: &BezierAlgebraicChord2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        chord.validate_policy(policy)?;
        let support = chord.retained_support();
        if let Some(line) = support.exact_line() {
            return Ok(Classification::Decided(Self::Exact {
                chord: chord.clone(),
                line,
            }));
        }
        match support.algebraic_ray_evaluator(policy)? {
            Classification::Decided(ray) => Ok(Classification::Decided(Self::Algebraic {
                chord: chord.clone(),
                ray,
            })),
            Classification::Uncertain(UncertaintyReason::Unsupported)
                if chord.certified_unit_tangent().is_some() =>
            {
                Ok(Classification::Decided(Self::CertifiedTangent {
                    chord: chord.clone(),
                }))
            }
            Classification::Uncertain(UncertaintyReason::Unsupported) => {
                Ok(Classification::Decided(Self::RefinedEndpoint {
                    chord: chord.clone(),
                }))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    fn oriented_side(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<crate::classify::LineSide>> {
        let support_chord = match self {
            Self::Exact { chord, .. }
            | Self::Algebraic { chord, .. }
            | Self::CertifiedTangent { chord }
            | Self::RefinedEndpoint { chord } => chord,
        };
        if [support_chord.start(), support_chord.end()]
            .into_iter()
            .any(|endpoint| point.shares_storage(endpoint))
        {
            return Ok(Classification::Decided(crate::classify::LineSide::On));
        }
        if let Some(side) = support_chord.retained_procedural_point_side(point, policy)? {
            return Ok(Classification::Decided(side));
        }
        if let Some(direction) = support_chord.certified_axis_direction()
            && let Some(decided @ Classification::Decided(_)) =
                support_chord.axis_oriented_side(point, direction, policy)
        {
            // A retained cardinal support reduces oriented area to one exact
            // coordinate comparison. It is a stronger and smaller authority
            // than the general procedural offset expression, whose source
            // and normalization fields are irrelevant after the cardinal
            // direction has been certified.
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-side-kernel",
                "cardinal-coordinate-precedence",
            );
            return Ok(decided);
        }
        let retained_side = match point {
            CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) => {
                Some(point.oriented_side_to_chord(support_chord, policy))
            }
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) => {
                Some(point.oriented_side_to_chord(support_chord, policy))
            }
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) => {
                Some(point.oriented_side_to_chord(support_chord, policy))
            }
            CurvePoint2(CurvePointData2::Exact(_))
            | CurvePoint2(CurvePointData2::Algebraic(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => None,
        };
        let retained_side_was_uncertain = if let Some(side) = retained_side {
            match side? {
                decided @ Classification::Decided(_) => return Ok(decided),
                Classification::Uncertain(_) => true,
            }
        } else {
            false
        };
        // Structural retained-point incidence is both exact and normally
        // constant-time. Consume it before rebuilding the same point through
        // a represented unit-tangent predicate; selected circle/chord
        // contacts otherwise expand their compact local field merely to
        // rediscover the support that authored them.
        if support_chord.certified_unit_tangent().is_some() {
            let certified_tangent_side = support_chord.certified_tangent_side(point, policy);
            if let side @ Classification::Decided(_) = certified_tangent_side {
                return Ok(side);
            }
        }
        if retained_side_was_uncertain {
            return support_chord.oriented_side_by_refinement(point, policy);
        }
        let reverse = |side| match side {
            crate::classify::LineSide::Left => crate::classify::LineSide::Right,
            crate::classify::LineSide::Right => crate::classify::LineSide::Left,
            crate::classify::LineSide::On => crate::classify::LineSide::On,
        };
        let (chord, side) = match self {
            Self::Exact { chord, line } => match point {
                CurvePoint2(CurvePointData2::Exact(point)) => {
                    (chord, line.classify_point(point, policy))
                }
                CurvePoint2(CurvePointData2::Algebraic(point)) => {
                    let point = match point.predicate_evaluator(policy)? {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    (
                        chord,
                        point.oriented_line_side(line.start(), line.end(), policy)?,
                    )
                }
                CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) => {
                    return point.oriented_side_to_chord(chord, policy);
                }
                CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) => {
                    return point.oriented_side_to_chord(chord, policy);
                }
                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) => {
                    return point.oriented_side_to_chord(chord, policy);
                }
                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => {
                    return chord.oriented_side_by_refinement(point, policy);
                }
                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => {
                    return chord.oriented_side_by_refinement(point, policy);
                }
                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    return chord.oriented_side_by_refinement(point, policy);
                }
            },
            Self::Algebraic { chord, ray } => {
                if let CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) = point {
                    return point.oriented_side_to_chord(chord, policy);
                }
                if let CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) = point {
                    return point.oriented_side_to_chord(chord, policy);
                }
                if let CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) = point {
                    return point.oriented_side_to_chord(chord, policy);
                }
                if matches!(
                    point,
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                        | CurvePoint2(CurvePointData2::AnalyticParallel(_))
                        | CurvePoint2(
                            CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)
                        )
                ) {
                    return chord.oriented_side_by_refinement(point, policy);
                }
                let point = match point {
                    CurvePoint2(CurvePointData2::Exact(point)) => {
                        match ray.exact_point_image(point, policy)? {
                            Classification::Decided(point) => point,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    CurvePoint2(CurvePointData2::Algebraic(point)) => point.clone(),
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                    | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
                    | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
                    | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                    | CurvePoint2(CurvePointData2::AnalyticParallel(_))
                    | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                        unreachable!("correlated chord points return before image normalization")
                    }
                };
                let point = match point.predicate_evaluator(policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let [start, end] = match ray.endpoint_predicates(policy)? {
                    Classification::Decided(endpoints) => endpoints,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                (
                    chord,
                    algebraic_point_oriented_line_side(&start, &end, &point, policy)?,
                )
            }
            Self::CertifiedTangent { chord } => {
                return Ok(chord.certified_tangent_side(point, policy));
            }
            Self::RefinedEndpoint { chord } => {
                return chord.oriented_side_by_refinement(point, policy);
            }
        };
        Ok(if chord.retained_support_orientation_is_reversed() {
            side.map(reverse)
        } else {
            side
        })
    }
}

/// Encloses a retained Bezier parameter by its exact value or isolating interval.
fn real_interval_from_parameter(parameter: &BezierParameter2) -> RealInterval {
    match parameter {
        BezierParameter2::Exact(parameter) => RealInterval {
            lower: parameter.clone(),
            upper: parameter.clone(),
        },
        BezierParameter2::Algebraic(parameter) => RealInterval {
            lower: parameter.interval().start().clone(),
            upper: parameter.interval().end().clone(),
        },
    }
}

/// Encloses one coordinate of certified bounds.
fn real_interval_from_axis(bounds: &Aabb2, axis: Axis2) -> RealInterval {
    RealInterval {
        lower: match axis {
            Axis2::X => bounds.min().x(),
            Axis2::Y => bounds.min().y(),
        }
        .clone(),
        upper: match axis {
            Axis2::X => bounds.max().x(),
            Axis2::Y => bounds.max().y(),
        }
        .clone(),
    }
}

/// Encloses a trivariate power-basis polynomial over a parameter box.
fn trivariate_power_basis_interval(
    polynomial: &TrivariatePolynomial,
    first: &RealInterval,
    second: &RealInterval,
    third: &RealInterval,
) -> Option<RealInterval> {
    let mut value = RealInterval {
        lower: Real::zero(),
        upper: Real::zero(),
    };
    for rows in polynomial.coefficients.iter().rev() {
        value = value.multiply(first)?;
        let mut slice = RealInterval {
            lower: Real::zero(),
            upper: Real::zero(),
        };
        for row in rows.iter().rev() {
            slice = slice.multiply(second)?;
            slice = slice.add(&RealInterval::evaluate_power_basis(row, third)?);
        }
        value = value.add(&slice);
    }
    Some(value)
}

/// Encloses a quadrivariate power-basis polynomial over a parameter box.
fn quadrivariate_power_basis_interval(
    polynomial: &QuadrivariatePolynomial2,
    parameters: [&RealInterval; 4],
) -> Option<RealInterval> {
    let [first, second, third, fourth] = parameters;
    let [first_count, second_count, third_count, fourth_count] = polynomial.dimensions;
    let zero = || RealInterval {
        lower: Real::zero(),
        upper: Real::zero(),
    };
    let mut value = zero();
    for first_power in (0..first_count).rev() {
        value = value.multiply(first)?;
        let mut first_slice = zero();
        for second_power in (0..second_count).rev() {
            first_slice = first_slice.multiply(second)?;
            let mut second_slice = zero();
            for third_power in (0..third_count).rev() {
                second_slice = second_slice.multiply(third)?;
                let start = QuadrivariatePolynomial2::flat_index(
                    polynomial.dimensions,
                    [first_power, second_power, third_power, 0],
                );
                second_slice = second_slice.add(&RealInterval::evaluate_power_basis(
                    &polynomial.coefficients[start..start + fourth_count],
                    fourth,
                )?);
            }
            first_slice = first_slice.add(&second_slice);
        }
        value = value.add(&first_slice);
    }
    Some(value)
}

fn retained_bounds_axis_order_to_real(
    mut bounds_at: impl FnMut(usize) -> Classification<Aabb2>,
    axis: Axis2,
    value: &Real,
    policy: &CurveContext,
) -> Classification<std::cmp::Ordering> {
    let mut terminal_refined = false;
    for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        if policy.has_bounded_exact_predicate_budget() && refinement_steps > 8 {
            break;
        }
        let Classification::Decided(bounds) = bounds_at(refinement_steps) else {
            continue;
        };
        terminal_refined |= refinement_steps == 512;
        let coordinate = real_interval_from_axis(&bounds, axis);
        if compare_reals(&coordinate.upper, value, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Less)
        {
            return Classification::Decided(std::cmp::Ordering::Less);
        }
        if compare_reals(&coordinate.lower, value, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Greater)
        {
            return Classification::Decided(std::cmp::Ordering::Greater);
        }
        if compare_reals(&coordinate.lower, value, &CurveContext::STRICT)
            == Some(std::cmp::Ordering::Equal)
            && compare_reals(&coordinate.upper, value, &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Equal)
        {
            return Classification::Decided(std::cmp::Ordering::Equal);
        }
    }
    if terminal_refined && policy.permits_approximate_512() {
        policy.observe_approximate_512();
        Classification::Decided(std::cmp::Ordering::Equal)
    } else {
        Classification::Uncertain(UncertaintyReason::Ordering)
    }
}

impl BezierSimilarityPoint2 {
    pub(crate) fn new(
        mut source: CurvePoint2,
        mut transform: Similarity2,
        policy: &CurveContext,
    ) -> Self {
        while let CurvePoint2(CurvePointData2::Similarity(image)) = &source {
            // Keep incompatible construction evidence visible to predicates;
            // composing its matrix must not erase a retained policy barrier.
            if !policy.accepts_retained_policy(image.data.policy) {
                break;
            }
            transform = image.data.transform.then(&transform);
            source = image.data.source.clone();
        }
        Self {
            data: Arc::new(BezierSimilarityPointData2 {
                source,
                transform,
                policy: *policy,
            }),
        }
    }

    pub(crate) fn shares_storage(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.data.policy == other.data.policy
                && self.data.transform == other.data.transform
                && self.data.source.shares_storage(&other.data.source))
    }

    fn translated(
        &self,
        delta_x: &Real,
        delta_y: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "similarity point was translated under a different predicate policy".into(),
            ));
        }
        Ok(Self::new(
            self.data.source.clone(),
            self.data.transform.translated(delta_x, delta_y),
            policy,
        ))
    }

    /// Materializes only the one-field point forms understood by existing
    /// point-incidence kernels.  The stored similarity carrier remains the
    /// published construction evidence; this exact affine image is temporary
    /// predicate input and never becomes APPROXIMATE_512 construction data.
    fn predicate_point_evidence(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurvePoint2>>> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Err(CurveError::Topology(
                "similarity point entered a predicate under a different policy".into(),
            ));
        }
        let source = match &self.data.source {
            CurvePoint2(CurvePointData2::Endpoint(endpoint)) => match endpoint.resolve(policy)? {
                Classification::Decided(Some(point)) => point,
                Classification::Decided(None) => return Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            },
            source @ (CurvePoint2(CurvePointData2::Exact(_))
            | CurvePoint2(CurvePointData2::Algebraic(_))) => source.clone(),
            CurvePoint2(CurvePointData2::AnalyticParallel(point)) => {
                match point.predicate_point_evidence(policy)? {
                    Classification::Decided(Some(point)) => point,
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            CurvePoint2(CurvePointData2::Similarity(point)) => {
                match point.predicate_point_evidence(policy)? {
                    Classification::Decided(Some(point)) => point,
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => {
                return Ok(Classification::Decided(None));
            }
        };
        let (m00, m01, m10, m11, tx, ty) = self.data.transform.affine_components();
        Ok(
            match BezierAlgebraicChord2::affine_transformed_endpoint(
                &source, m00, m01, m10, m11, tx, ty, None, None, policy,
            )? {
                Classification::Decided(
                    point @ (CurvePoint2(CurvePointData2::Exact(_))
                    | CurvePoint2(CurvePointData2::Algebraic(_))),
                ) => Classification::Decided(Some(point)),
                Classification::Decided(_) => Classification::Decided(None),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    pub(crate) fn conservative_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        self.conservative_bounds_refined_impl(refinement_steps, policy, false)
    }

    fn conservative_local_bounds_refined(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
    ) -> Classification<Aabb2> {
        self.conservative_bounds_refined_impl(refinement_steps, policy, true)
    }

    fn conservative_bounds_refined_impl(
        &self,
        refinement_steps: usize,
        policy: &CurveContext,
        local_only: bool,
    ) -> Classification<Aabb2> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        let source = match if local_only {
            algebraic_chord_endpoint_local_bounds_refined(
                &self.data.source,
                refinement_steps,
                policy,
            )
        } else {
            algebraic_chord_endpoint_bounds_refined(&self.data.source, refinement_steps, policy)
        } {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                return Classification::Uncertain(reason);
            }
        };
        let corners = [
            Point2::new(source.min().x().clone(), source.min().y().clone()),
            Point2::new(source.min().x().clone(), source.max().y().clone()),
            Point2::new(source.max().x().clone(), source.min().y().clone()),
            Point2::new(source.max().x().clone(), source.max().y().clone()),
        ]
        .map(|point| self.data.transform.transform_point(&point));
        Aabb2::from_points(corners.iter())
    }

    pub(crate) fn same_point_evidence(
        &self,
        other: &CurvePoint2,
        policy: &CurveContext,
    ) -> Classification<bool> {
        if !policy.accepts_retained_policy(self.data.policy) {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        if let CurvePoint2(CurvePointData2::Similarity(other)) = other
            && policy.accepts_retained_policy(other.data.policy)
            && self.data.transform == other.data.transform
        {
            if self.shares_storage(other) {
                return Classification::Decided(true);
            }
            return self.data.source.same_point(&other.data.source, policy);
        }
        retained_point_evidence_equality_by_refinement(
            &CurvePoint2::from(self.clone()),
            other,
            policy,
        )
    }
}

/// Orders two caller-certified incident points in one selected semicircle's
/// retained recursive frame. The oriented cross product of their radial
/// vectors is monotone on the closed half-circle chart. If the caller also
/// certifies that the points differ, an exact nonnegative radial dot rules
/// out the remaining antipodal zero-cross case and makes nonzero-cross sign
/// refinement complete.
fn recursive_projective_incident_point_order(
    first: &CurvePoint2,
    second: &CurvePoint2,
    semicircle: &BezierAlgebraicCuspSemicircle2,
    certify_distinct: impl FnOnce() -> CurveResult<bool>,
    policy: &CurveContext,
) -> CurveResult<Option<Classification<std::cmp::Ordering>>> {
    let first = match recursive_projective_point_source(first, policy)? {
        Classification::Decided(Some(point)) => point,
        Classification::Decided(None) => return Ok(None),
        Classification::Uncertain(reason) => {
            return Ok(Some(Classification::Uncertain(reason)));
        }
    };
    let second = match recursive_projective_point_source(second, policy)? {
        Classification::Decided(Some(point)) => point,
        Classification::Decided(None) => return Ok(None),
        Classification::Uncertain(reason) => {
            return Ok(Some(Classification::Uncertain(reason)));
        }
    };
    let authority = match semicircle.recursive_circle_frame_authority(policy)? {
        Classification::Decided(Some(authority)) => authority,
        Classification::Decided(None) => return Ok(None),
        Classification::Uncertain(reason) => {
            return Ok(Some(Classification::Uncertain(reason)));
        }
    };
    let (authority, [first, second]) =
        match embed_recursive_projective_point_sources(authority, [first, second], policy)? {
            Classification::Decided(Some(embedded)) => embedded,
            Classification::Decided(None) => return Ok(None),
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
    let Some((first_x, first_y, _)) = first.difference_numerators(&authority.center) else {
        return Ok(Some(Classification::Uncertain(
            UncertaintyReason::Unsupported,
        )));
    };
    let Some((second_x, second_y, _)) = second.difference_numerators(&authority.center) else {
        return Ok(Some(Classification::Uncertain(
            UncertaintyReason::Unsupported,
        )));
    };
    let Some(cross) = first_x.multiply(&second_y).and_then(|value| {
        first_y
            .multiply(&second_x)
            .and_then(|other| value.subtract(&other))
    }) else {
        return Ok(Some(Classification::Uncertain(
            UncertaintyReason::Unsupported,
        )));
    };
    let radial_dot = || {
        first_x.multiply(&second_x).and_then(|value| {
            first_y
                .multiply(&second_y)
                .and_then(|other| value.add(&other))
        })
    };
    let cross = if let Some(sign) = cross.bounded_interval_sign(0..=512) {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-circle-incident-point-order",
            "recursive-projective-interval-separated",
        );
        sign
    } else if certify_distinct()? {
        let Some(dot) = radial_dot() else {
            return Ok(Some(Classification::Uncertain(
                UncertaintyReason::Unsupported,
            )));
        };
        // This sign participates in an exact nonzero certificate rather
        // than the final equality predicate. In APPROXIMATE_512, a
        // terminal approximate zero must therefore not be promoted into
        // construction evidence.
        if matches!(
            dot.bounded_interval_sign(0..=512),
            Some(RealSign::Positive | RealSign::Zero)
        ) {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-circle-incident-point-order",
                "recursive-projective-nonzero-certificate",
            );
            match cross.sign_with_nonzero_certificate()? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            }
        } else {
            match cross.sign(policy)? {
                Classification::Decided(sign) => sign,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            }
        }
    } else {
        match cross.sign(policy)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        }
    };
    if cross == RealSign::Zero {
        let Some(dot) = radial_dot() else {
            return Ok(Some(Classification::Uncertain(
                UncertaintyReason::Unsupported,
            )));
        };
        let order = match dot.sign(policy)? {
            Classification::Decided(RealSign::Positive) => {
                Classification::Decided(std::cmp::Ordering::Equal)
            }
            Classification::Decided(RealSign::Negative | RealSign::Zero) => {
                Classification::Uncertain(UncertaintyReason::Boundary)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        };
        return Ok(Some(order));
    }
    let follows_traversal = if semicircle.is_clockwise() {
        cross == RealSign::Negative
    } else {
        cross == RealSign::Positive
    };
    Ok(Some(Classification::Decided(if follows_traversal {
        std::cmp::Ordering::Less
    } else {
        std::cmp::Ordering::Greater
    })))
}

/// Signs `|point-center|^2-radius_squared` in the least shared retained
/// field. Squaring the common nonzero denominator preserves the affine sign;
/// selected-root and radical relations remain available to prove exact zero.
fn recursive_projective_point_evidence_circle_residual_sign(
    point: &CurvePoint2,
    center: &CurvePoint2,
    radius_squared: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<RealSign>>> {
    let points = match recursive_projective_evidence_points(&[point, center], policy)? {
        Classification::Decided(Some(points)) => points,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let [point, center]: [BezierRecursiveQuadraticProjectivePoint2; 2] = points
        .try_into()
        .expect("a circle residual retains its point and center");
    let Some(residual) = (|| {
        let (dx, dy, denominator) = point.difference_numerators(&center)?;
        dx.square()?
            .add(&dy.square()?)?
            .subtract(&denominator.square()?.scale(radius_squared)?)
    })() else {
        return Ok(Classification::Decided(None));
    };
    Ok(residual.sign(policy)?.map(Some))
}

/// Signs an oriented area directly in the least shared recursive quadratic
/// tower carried by three retained points. No affine chord parameter axis is
/// needed, so callers with an independent distinctness certificate can avoid
/// constructing one merely to evaluate a determinant.
fn recursive_projective_point_evidence_oriented_side(
    start: &CurvePoint2,
    end: &CurvePoint2,
    point: &CurvePoint2,
    certified_nonzero: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<crate::classify::LineSide>>> {
    let points = match recursive_projective_evidence_points(&[start, end, point], policy)? {
        Classification::Decided(Some(points)) => points,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let [start, end, point]: [BezierRecursiveQuadraticProjectivePoint2; 3] = points
        .try_into()
        .expect("a recursive oriented side retains three points");
    Ok(BezierRecursiveQuadraticProjectivePoint2::oriented_side(
        &start,
        &end,
        &point,
        certified_nonzero,
        policy,
    )?
    .map(Some))
}

/// Orders one coordinate of two retained points in their least shared
/// recursive quadratic tower. This is the scalar counterpart of the native
/// oriented-area authority and is especially important for collinear overlap
/// clipping, where independently refined Cartesian boxes cannot certify an
/// exact shared endpoint.
fn recursive_projective_point_evidence_axis_order(
    first: &CurvePoint2,
    second: &CurvePoint2,
    axis: Axis2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<std::cmp::Ordering>>> {
    let points = match recursive_projective_evidence_points(&[first, second], policy)? {
        Classification::Decided(Some(points)) => points,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let [first, second]: [BezierRecursiveQuadraticProjectivePoint2; 2] = points
        .try_into()
        .expect("a recursive coordinate order retains two points");
    Ok(match first.axis_order(&second, axis, policy)? {
        Some(Classification::Decided(order)) => Classification::Decided(Some(order)),
        Some(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
        None => Classification::Decided(None),
    })
}

/// Compares two retained affine points in their least shared recursive field.
///
/// A differing coordinate proves inequality immediately. Equality requires
/// both homogeneous coordinate differences to vanish under the active exact
/// predicate policy; no independently selected Cartesian roots are formed.
pub(crate) fn recursive_projective_point_evidence_equality(
    first: &CurvePoint2,
    second: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<bool>>> {
    // These carrier-local boxes retain the construction's point evidence.
    // Consume certified separation before importing a recursive field merely
    // to compare unrelated contacts and re-prove their denominator signs.
    let strict = policy.strict_counterpart();
    for steps in [0, 2, 4, 8] {
        if let (Classification::Decided(first), Classification::Decided(second)) = (
            algebraic_chord_endpoint_local_bounds_refined(first, steps, &strict),
            algebraic_chord_endpoint_local_bounds_refined(second, steps, &strict),
        ) && first.overlaps(&second, &strict) == Classification::Decided(false)
        {
            return Ok(Classification::Decided(Some(false)));
        }
    }
    let points = match recursive_projective_evidence_points(&[first, second], policy)? {
        Classification::Decided(Some(points)) => points,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let [first, second]: [BezierRecursiveQuadraticProjectivePoint2; 2] = points
        .try_into()
        .expect("a recursive point equality retains two points");
    for axis in [Axis2::X, Axis2::Y] {
        match first.axis_order(&second, axis, policy)? {
            Some(Classification::Decided(std::cmp::Ordering::Equal)) => {}
            Some(Classification::Decided(
                std::cmp::Ordering::Less | std::cmp::Ordering::Greater,
            )) => return Ok(Classification::Decided(Some(false))),
            Some(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            None => return Ok(Classification::Decided(None)),
        }
    }
    Ok(Classification::Decided(Some(true)))
}

/// Signs an oriented area directly in the shared represented root tensor of
/// three retained points. Like the recursive-projective authority above, this
/// avoids constructing an unrelated affine chord parameter merely to obtain
/// the determinant sign.
fn represented_point_evidence_oriented_side(
    start: &CurvePoint2,
    end: &CurvePoint2,
    point: &CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<crate::classify::LineSide>> {
    let mut represented = Vec::with_capacity(6);
    for point in [start, end, point] {
        match represented_point_evidence_coordinates(point, policy)? {
            Classification::Decided(coordinates) => represented.extend(coordinates),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let Some((sources, coordinates)) = represented_affine_tensor_basis(&represented) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let [start_x, start_y, end_x, end_y, point_x, point_y]: [DenseTensorPolynomial; 6] =
        coordinates
            .try_into()
            .expect("a represented oriented-side predicate retains six coordinates");
    let Some(cross) = (|| {
        let direction_x = end_x.subtract(&start_x)?;
        let direction_y = end_y.subtract(&start_y)?;
        let point_x = point_x.subtract(&start_x)?;
        let point_y = point_y.subtract(&start_y)?;
        direction_x
            .multiply(&point_y)?
            .subtract(&direction_y.multiply(&point_x)?)
    })() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let cross = dense_reduce_selected_root_relations(cross.clone(), &sources).unwrap_or(cross);
    let cross = match Classification::from(represented_dense_value_refined(&cross, &sources)) {
        Classification::Decided(cross) => cross,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(match represented_policy_sign(&cross, policy) {
        Classification::Decided(sign) => Classification::Decided(match sign {
            RealSign::Positive => crate::classify::LineSide::Left,
            RealSign::Negative => crate::classify::LineSide::Right,
            RealSign::Zero => crate::classify::LineSide::On,
        }),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

fn selected_circle_endpoint_chord_side(
    start: &CurvePoint2,
    end: &CurvePoint2,
    point: &CurvePoint2,
    certified_circle_incident: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<crate::classify::LineSide>> {
    if certified_circle_incident
        && let CurvePoint2(CurvePointData2::AlgebraicCuspChord(contact)) = point
    {
        let start_separated = contact.contact_support_separates_point(start, policy)?;
        let end_separated = contact.contact_support_separates_point(end, policy)?;
        if start_separated == Classification::Decided(true)
            && end_separated == Classification::Decided(true)
        {
            let recursive =
                recursive_projective_point_evidence_oriented_side(start, end, point, true, policy)?;
            match recursive {
                Classification::Decided(Some(side)) => {
                    return Ok(Classification::Decided(side));
                }
                Classification::Decided(None) | Classification::Uncertain(_) => {}
            }
            if matches!(
                (start, end),
                (
                    CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)),
                    CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
                )
            ) {
                let represented =
                    represented_point_evidence_oriented_side(start, end, point, policy)?;
                if let Classification::Decided(side) = represented {
                    return Ok(Classification::Decided(side));
                }
            }
        }
    }
    // Both call sites supply the endpoints of an already-validated positive
    // semicircle fragment. Its strictly ordered local parameter range and
    // injective half-circle chart certify that these points are distinct.
    // Re-running generic multi-field point equality here discards that proof
    // and can leave recursively derived pair endpoints unresolved.
    let chord = match BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
        start.clone(),
        end.clone(),
        policy,
    )? {
        Classification::Decided(chord) => chord,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let support = match BezierAlgebraicChordSupportPredicate2::try_new(&chord, policy)? {
        Classification::Decided(support) => support,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    support.oriented_side(point, policy)
}

fn reduce_bivariate_in_selected_parameter(
    polynomial: BivariatePolynomial,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<BivariatePolynomial>> {
    match parameter {
        BezierParameter2::Exact(_) => Ok(Classification::Decided(polynomial)),
        BezierParameter2::Algebraic(parameter) => {
            reduce_algebraic_cusp_bivariate(polynomial, parameter, policy)
        }
    }
}

fn reduce_radical_expression_in_selected_parameter(
    expression: BezierAlgebraicCuspTwoTermExpression2,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicCuspTwoTermExpression2>> {
    match parameter {
        BezierParameter2::Exact(_) => Ok(Classification::Decided(expression)),
        BezierParameter2::Algebraic(parameter) => {
            reduce_algebraic_cusp_radical_expression(expression, parameter, policy)
        }
    }
}

fn reduce_two_normal_expression_in_selected_parameter(
    expression: BezierParallelTwoNormalExpression2,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParallelTwoNormalExpression2>> {
    let reduce = |polynomial| reduce_bivariate_in_selected_parameter(polynomial, parameter, policy);
    let product = match reduce(expression.product)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let center = match reduce(expression.center)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let candidate = match reduce(expression.candidate)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let rational = match reduce(expression.rational)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(
        BezierParallelTwoNormalExpression2 {
            product,
            center,
            candidate,
            rational,
        },
    ))
}

/// Proves whether every target-parameter coefficient of
/// `rational + radical * sqrt(radicand)` vanishes in one selected algebraic
/// center field. The radicand is constant in the target parameter and is
/// positive at the selected center.
fn algebraic_selected_square_root_polynomial_is_identically_zero(
    rational: &BivariatePolynomial,
    radical: &BivariatePolynomial,
    radicand: &BivariatePolynomial,
    retained: &BezierAlgebraicParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    let speed = bivariate_specialize_second(radicand, &Real::zero());
    if *radicand != bivariate_outer_product(&speed, &[Real::one()]) {
        return Err(CurveError::Topology(
            "a selected-center speed unexpectedly depended on the target parameter".into(),
        ));
    }
    let retained = BezierParameter2::Algebraic(retained.clone());
    match signed_coefficients_at_parameter(&speed, &retained, policy)? {
        Classification::Decided(RealSign::Positive) => {}
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Decided(RealSign::Negative) => {
            return Err(CurveError::Topology(
                "a selected-center squared speed was negative".into(),
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let coefficient_count = rational
        .coefficients
        .iter()
        .chain(&radical.coefficients)
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    for power in 0..coefficient_count {
        let rational = bivariate_second_parameter_coefficient(rational, power);
        let radical = bivariate_second_parameter_coefficient(radical, power);
        let rational_sign = match signed_coefficients_at_parameter(&rational, &retained, policy)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let radical_sign = match signed_coefficients_at_parameter(&radical, &retained, policy)? {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let coefficient_sign = match (rational_sign, radical_sign) {
            (RealSign::Zero, sign) | (sign, RealSign::Zero) => sign,
            (first, second) if first == second => first,
            _ => {
                let magnitude = polynomial_subtract(
                    &polynomial_multiply(&rational, &rational),
                    &polynomial_multiply(&polynomial_multiply(&radical, &radical), &speed),
                );
                match signed_coefficients_at_parameter(&magnitude, &retained, policy)? {
                    Classification::Decided(RealSign::Positive) => rational_sign,
                    Classification::Decided(RealSign::Negative) => radical_sign,
                    Classification::Decided(RealSign::Zero) => RealSign::Zero,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        };
        if coefficient_sign != RealSign::Zero {
            return Ok(Classification::Decided(false));
        }
    }
    Ok(Classification::Decided(true))
}

fn algebraic_cusp_branched_expression(
    expression: &BezierAlgebraicCuspTwoTermExpression2,
    branch: i8,
) -> BezierAlgebraicCuspTwoTermExpression2 {
    debug_assert!((-1..=1).contains(&branch));
    BezierAlgebraicCuspTwoTermExpression2 {
        rational: expression.rational.clone(),
        radical: bivariate_scale(expression.radical.clone(), &Real::from(branch)),
    }
}

/// Signs `A + B sqrt(K)` at two independently selected algebraic cusp roots.
/// Callers supply `K > 0` whenever `B` is nonzero; tangent branches multiply
/// `B` by zero before entering this helper. Opposite signs are resolved by the
/// exact comparison `A^2 - B^2 K`, reduced in both local fields before replay.
fn algebraic_cusp_correlated_square_root_sum_sign(
    incidence: &BivariatePolynomial,
    expression: &BezierAlgebraicCuspTwoTermExpression2,
    radicand: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    algebraic_cusp_correlated_square_root_sum_sign_impl(
        incidence,
        expression,
        radicand,
        first_parameter,
        second_parameter,
        false,
        policy,
    )
}

/// Signs a square-root expression after the parameter pair has already been
/// certified on `incidence`. This is the construction-owning counterpart to
/// the generic correlated replay and avoids recounting the selected fiber when
/// the squared magnitude reduces back to the relation that produced the pair.
fn algebraic_cusp_selected_square_root_sum_sign(
    incidence: &BivariatePolynomial,
    expression: &BezierAlgebraicCuspTwoTermExpression2,
    radicand: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    algebraic_cusp_correlated_square_root_sum_sign_impl(
        incidence,
        expression,
        radicand,
        first_parameter,
        second_parameter,
        true,
        policy,
    )
}

#[allow(clippy::too_many_arguments)]
fn algebraic_cusp_correlated_square_root_sum_sign_impl(
    incidence: &BivariatePolynomial,
    expression: &BezierAlgebraicCuspTwoTermExpression2,
    radicand: &BivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    incidence_is_certified: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let sign = |polynomial: &BivariatePolynomial| {
        if incidence_is_certified && polynomial == incidence {
            return Ok(Classification::Decided(RealSign::Zero));
        }
        algebraic_selected_correlated_predicate_sign(
            incidence,
            polynomial,
            first_parameter,
            second_parameter,
            policy,
        )
    };
    let rational = match sign(&expression.rational)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let radical = match sign(&expression.radical)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    match (rational, radical) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
            return Ok(Classification::Decided(sign));
        }
        (first, second) if first == second => {
            return Ok(Classification::Decided(first));
        }
        _ => {}
    }

    let magnitude = bivariate_subtract(
        &bivariate_multiply(&expression.rational, &expression.rational),
        &bivariate_multiply(
            &bivariate_multiply(&expression.radical, &expression.radical),
            radicand,
        ),
    );
    let magnitude =
        if let (BezierParameter2::Algebraic(first), BezierParameter2::Algebraic(second)) =
            (first_parameter, second_parameter)
        {
            match bivariate_reduce_parameter_polynomials(
                &magnitude,
                first.polynomial(),
                second.polynomial(),
                policy,
            )? {
                Classification::Decided(magnitude) => magnitude,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        } else {
            magnitude
        };
    Ok(match sign(&magnitude)? {
        Classification::Decided(RealSign::Positive) => Classification::Decided(rational),
        Classification::Decided(RealSign::Negative) => Classification::Decided(radical),
        Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

/// Signs `A + B sqrt(K)` at three retained selected roots without forming a
/// primitive element. Opposite term signs reduce to the exact trivariate
/// predicate `A^2 - B^2 K`; the shared three-root predicate engine owns the
/// STRICT/APPROXIMATE_512 terminal decision.
fn algebraic_cusp_trivariate_square_root_sum_sign(
    expression: &BezierAlgebraicCuspTrivariateSquareRootExpression2,
    radicand: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    third_parameter: &BezierParameter2,
    branch: i8,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    algebraic_cusp_trivariate_square_root_components_sign(
        &expression.rational,
        &expression.radical,
        radicand,
        first_parameter,
        second_parameter,
        third_parameter,
        branch,
        policy,
    )
}

/// Signs one authored radical incidence whose squared projection produced the
/// supplied third-axis candidate.  A transverse Poincare--Miranda certificate
/// proves exact zero before the generic three-field signer attempts to
/// rediscover the resultant correlation by factorization.
fn algebraic_cusp_projected_trivariate_square_root_sum_sign(
    expression: &BezierAlgebraicCuspTrivariateSquareRootExpression2,
    radicand: &TrivariatePolynomial,
    projected_incidence: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    third_parameter: &BezierParameter2,
    branch: i8,
    projected_root_certified: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    algebraic_cusp_trivariate_square_root_components_sign_internal(
        &expression.rational,
        &expression.radical,
        radicand,
        Some(projected_incidence),
        projected_root_certified,
        first_parameter,
        second_parameter,
        third_parameter,
        branch,
        policy,
    )
}

fn algebraic_cusp_trivariate_square_root_components_sign(
    rational_term: &TrivariatePolynomial,
    radical_term: &TrivariatePolynomial,
    radicand: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    third_parameter: &BezierParameter2,
    branch: i8,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    algebraic_cusp_trivariate_square_root_components_sign_internal(
        rational_term,
        radical_term,
        radicand,
        None,
        false,
        first_parameter,
        second_parameter,
        third_parameter,
        branch,
        policy,
    )
}

#[allow(clippy::too_many_arguments)]
fn algebraic_cusp_trivariate_square_root_components_sign_internal(
    rational_term: &TrivariatePolynomial,
    radical_term: &TrivariatePolynomial,
    radicand: &TrivariatePolynomial,
    projected_incidence: Option<&TrivariatePolynomial>,
    projected_root_certified: bool,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    third_parameter: &BezierParameter2,
    branch: i8,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    debug_assert!((-1..=1).contains(&branch));
    // Most authored terms and their squared magnitude separate on the direct
    // product isolator.  Take that division-free exact path before invoking
    // the general three-field factor/sign engine; high-degree norm candidates
    // otherwise make even ordinary nonzero conjugate terms unnecessarily
    // expensive to classify.
    let bounded_rational = trivariate_parameter_triple_bounded_box_sign(
        rational_term,
        first_parameter,
        second_parameter,
        third_parameter,
        64,
    )?;
    if branch == 0 {
        if let Some(sign) = bounded_rational {
            return Ok(Classification::Decided(sign));
        }
    } else {
        let bounded_radical = trivariate_parameter_triple_bounded_box_sign(
            radical_term,
            first_parameter,
            second_parameter,
            third_parameter,
            64,
        )?
        .map(|sign| {
            if branch < 0 {
                match sign {
                    RealSign::Negative => RealSign::Positive,
                    RealSign::Zero => RealSign::Zero,
                    RealSign::Positive => RealSign::Negative,
                }
            } else {
                sign
            }
        });
        if let (Some(rational), Some(radical)) = (bounded_rational, bounded_radical) {
            // A certified root of the norm projection proves that either the
            // authored pair sheet or its conjugate vanishes. Once both term
            // signs are known, equal signs select the nonzero authored sheet
            // and opposite signs select its exact zero. Do not construct and
            // bound the already-certified squared magnitude merely to recover
            // the same branch decision.
            if projected_root_certified {
                debug_assert!(projected_incidence.is_some());
                return Ok(Classification::Decided(if rational == radical {
                    rational
                } else {
                    RealSign::Zero
                }));
            }
            if rational == radical {
                return Ok(Classification::Decided(rational));
            }
            let Some(radical_squared) = radical_term.multiply(radical_term) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let Some(magnitude) = TrivariatePolynomial::sum_products(&[
                (rational_term, rational_term, false),
                (&radical_squared, radicand, true),
            ]) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            if let Some(sign) = trivariate_parameter_triple_bounded_box_sign(
                &magnitude,
                first_parameter,
                second_parameter,
                third_parameter,
                64,
            )? {
                return Ok(Classification::Decided(match sign {
                    RealSign::Positive => rational,
                    RealSign::Negative => radical,
                    RealSign::Zero => unreachable!("a strict Bernstein box sign is nonzero"),
                }));
            }
        }
    }
    // A certified zero of A^2-B^2*K needs only the exact signs of A and
    // branch*B to select the authored square-root sheet.  Independent
    // isolator boxes usually separate those nonzero terms quickly, whereas
    // the general three-field signer may construct a local algebraic field
    // merely to rediscover their correlation with the projected root.
    if projected_root_certified && branch != 0 {
        let rational = trivariate_parameter_triple_bounded_box_sign(
            rational_term,
            first_parameter,
            second_parameter,
            third_parameter,
            64,
        )?;
        let radical = trivariate_parameter_triple_bounded_box_sign(
            radical_term,
            first_parameter,
            second_parameter,
            third_parameter,
            64,
        )?
        .map(|sign| {
            if branch < 0 {
                match sign {
                    RealSign::Negative => RealSign::Positive,
                    RealSign::Zero => RealSign::Zero,
                    RealSign::Positive => RealSign::Negative,
                }
            } else {
                sign
            }
        });
        if let (Some(rational), Some(radical)) = (rational, radical) {
            return Ok(Classification::Decided(if rational == radical {
                rational
            } else {
                RealSign::Zero
            }));
        }
        // At a certified norm root, either authored term vanishing forces
        // the complete radical sum to vanish: A=0 gives B^2*K=0, while B=0
        // gives A^2=0.  Detect this common geometric case (for example a
        // shared circle-pair endpoint) with the constrained selected-pair
        // authority instead of asking the generic signer to build a local
        // three-root field twice.
        for term in [rational_term, radical_term] {
            if projected_selected_trivariate_candidate_has_subresultant_root(
                term,
                first_parameter,
                second_parameter,
                third_parameter,
                policy,
            )? == Classification::Decided(true)
            {
                return Ok(Classification::Decided(RealSign::Zero));
            }
        }
    }
    let sign = |polynomial: &TrivariatePolynomial| {
        trivariate_parameter_triple_sign_by_refinement(
            polynomial,
            first_parameter,
            second_parameter,
            third_parameter,
            policy,
        )
    };
    let rational = match sign(rational_term)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let radical = if branch == 0 {
        RealSign::Zero
    } else {
        match sign(radical_term)? {
            Classification::Decided(sign) => {
                if branch < 0 {
                    match sign {
                        RealSign::Negative => RealSign::Positive,
                        RealSign::Zero => RealSign::Zero,
                        RealSign::Positive => RealSign::Negative,
                    }
                } else {
                    sign
                }
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    };
    match (rational, radical) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
            return Ok(Classification::Decided(sign));
        }
        (first, second) if first == second => {
            return Ok(Classification::Decided(first));
        }
        _ => {}
    }
    if projected_root_certified {
        debug_assert!(projected_incidence.is_some());
        return Ok(Classification::Decided(RealSign::Zero));
    }
    if let Some(projected_incidence) = projected_incidence
        && projected_selected_trivariate_candidate_has_box_root(
            projected_incidence,
            first_parameter,
            second_parameter,
            third_parameter,
            8,
        )?
    {
        return Ok(Classification::Decided(RealSign::Zero));
    }
    if let Some(projected_incidence) = projected_incidence
        && let Some(sign) = trivariate_parameter_triple_bounded_box_sign(
            projected_incidence,
            first_parameter,
            second_parameter,
            third_parameter,
            16,
        )?
    {
        return Ok(Classification::Decided(match sign {
            RealSign::Positive => rational,
            RealSign::Negative => radical,
            RealSign::Zero => unreachable!("a strict Bernstein box sign is nonzero"),
        }));
    }
    if let Some(projected_incidence) = projected_incidence {
        match projected_selected_trivariate_candidate_has_subresultant_root(
            projected_incidence,
            first_parameter,
            second_parameter,
            third_parameter,
            policy,
        )? {
            Classification::Decided(true) => {
                return Ok(Classification::Decided(RealSign::Zero));
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(_) => {
                // Preserve the historical complete transverse authority when
                // the symbolic backend exceeds a supported degree or
                // coefficient domain. Even roots normally avoid this path
                // because the subresultant proof decides them first.
                if projected_selected_trivariate_candidate_has_box_root(
                    projected_incidence,
                    first_parameter,
                    second_parameter,
                    third_parameter,
                    512,
                )? {
                    return Ok(Classification::Decided(RealSign::Zero));
                }
            }
        }
    }
    let Some(radical_squared) = radical_term.multiply(radical_term) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let Some(magnitude) = TrivariatePolynomial::sum_products(&[
        (rational_term, rational_term, false),
        (&radical_squared, radicand, true),
    ]) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let strict = policy.strict_counterpart();
    let strict_magnitude = trivariate_parameter_triple_sign_by_refinement(
        &magnitude,
        first_parameter,
        second_parameter,
        third_parameter,
        &strict,
    )?;
    Ok(match strict_magnitude {
        Classification::Decided(RealSign::Positive) => Classification::Decided(rational),
        Classification::Decided(RealSign::Negative) => Classification::Decided(radical),
        Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
        Classification::Uncertain(reason) => {
            if policy.permits_approximate_512() {
                match sign(&magnitude)? {
                    Classification::Decided(RealSign::Positive) => {
                        Classification::Decided(rational)
                    }
                    Classification::Decided(RealSign::Negative) => Classification::Decided(radical),
                    Classification::Decided(RealSign::Zero) => {
                        Classification::Decided(RealSign::Zero)
                    }
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                }
            } else {
                Classification::Uncertain(reason)
            }
        }
    })
}

#[allow(clippy::too_many_arguments)]
fn algebraic_cusp_projected_trivariate_two_square_root_sum_sign(
    expression: &BezierAlgebraicCuspTrivariateTwoSquareRootExpression2,
    candidate_norm: Option<&BezierAlgebraicCuspTrivariateSquareRootExpression2>,
    pair_discriminant: &TrivariatePolynomial,
    candidate_speed_squared: &TrivariatePolynomial,
    projected_incidence: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    third_parameter: &BezierParameter2,
    branch: i8,
    projected_root_certified: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    algebraic_cusp_trivariate_two_square_root_sum_sign_internal(
        expression,
        candidate_norm,
        pair_discriminant,
        candidate_speed_squared,
        Some(projected_incidence),
        projected_root_certified,
        first_parameter,
        second_parameter,
        third_parameter,
        branch,
        policy,
    )
}

#[allow(clippy::too_many_arguments)]
fn algebraic_cusp_trivariate_two_square_root_sum_sign(
    expression: &BezierAlgebraicCuspTrivariateTwoSquareRootExpression2,
    pair_discriminant: &TrivariatePolynomial,
    candidate_speed_squared: &TrivariatePolynomial,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    third_parameter: &BezierParameter2,
    branch: i8,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    algebraic_cusp_trivariate_two_square_root_sum_sign_internal(
        expression,
        None,
        pair_discriminant,
        candidate_speed_squared,
        None,
        false,
        first_parameter,
        second_parameter,
        third_parameter,
        branch,
        policy,
    )
}

#[allow(clippy::too_many_arguments)]
fn algebraic_cusp_trivariate_two_square_root_sum_sign_internal(
    expression: &BezierAlgebraicCuspTrivariateTwoSquareRootExpression2,
    candidate_norm: Option<&BezierAlgebraicCuspTrivariateSquareRootExpression2>,
    pair_discriminant: &TrivariatePolynomial,
    candidate_speed_squared: &TrivariatePolynomial,
    projected_incidence: Option<&TrivariatePolynomial>,
    projected_root_certified: bool,
    first_parameter: &BezierParameter2,
    second_parameter: &BezierParameter2,
    third_parameter: &BezierParameter2,
    branch: i8,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let pair_sign = |rational: &TrivariatePolynomial, radical: &TrivariatePolynomial| {
        algebraic_cusp_trivariate_square_root_components_sign(
            rational,
            radical,
            pair_discriminant,
            first_parameter,
            second_parameter,
            third_parameter,
            branch,
            policy,
        )
    };
    // At a root produced by the complete two-radical norm, inspect the
    // retained pair norm first. Conjugate target-speed or pair sheets are
    // thereby rejected after signing only the dominant group; signing both
    // groups up front can construct a large local field for an irrelevant
    // conjugate term of an even projection root.
    if let Some(projected_incidence) = projected_incidence {
        let owned_norm;
        let norm = if let Some(norm) = candidate_norm {
            norm
        } else {
            let Some(norm) = expression.candidate_norm(pair_discriminant, candidate_speed_squared)
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            owned_norm = norm;
            &owned_norm
        };
        let magnitude = algebraic_cusp_projected_trivariate_square_root_sum_sign(
            norm,
            pair_discriminant,
            projected_incidence,
            first_parameter,
            second_parameter,
            third_parameter,
            branch,
            projected_root_certified,
            policy,
        )?;
        match magnitude {
            Classification::Decided(RealSign::Positive) => {
                return pair_sign(&expression.rational, &expression.pair);
            }
            Classification::Decided(RealSign::Negative) => {
                return pair_sign(&expression.candidate, &expression.product);
            }
            Classification::Decided(RealSign::Zero) => {
                let retained = match pair_sign(&expression.rational, &expression.pair)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let candidate = match pair_sign(&expression.candidate, &expression.product)? {
                    Classification::Decided(sign) => sign,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                return Ok(Classification::Decided(match (retained, candidate) {
                    (RealSign::Zero, sign) | (sign, RealSign::Zero) => sign,
                    (first, second) if first == second => first,
                    _ => RealSign::Zero,
                }));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let retained = match pair_sign(&expression.rational, &expression.pair)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let candidate = match pair_sign(&expression.candidate, &expression.product)? {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    match (retained, candidate) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => {
            return Ok(Classification::Decided(sign));
        }
        (first, second) if first == second => {
            return Ok(Classification::Decided(first));
        }
        _ => {}
    }

    let Some(norm) = expression.candidate_norm(pair_discriminant, candidate_speed_squared) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let magnitude = if let Some(projected_incidence) = projected_incidence {
        algebraic_cusp_projected_trivariate_square_root_sum_sign(
            &norm,
            pair_discriminant,
            projected_incidence,
            first_parameter,
            second_parameter,
            third_parameter,
            branch,
            projected_root_certified,
            policy,
        )?
    } else {
        algebraic_cusp_trivariate_square_root_sum_sign(
            &norm,
            pair_discriminant,
            first_parameter,
            second_parameter,
            third_parameter,
            branch,
            policy,
        )?
    };
    Ok(match magnitude {
        Classification::Decided(RealSign::Positive) => Classification::Decided(retained),
        Classification::Decided(RealSign::Negative) => Classification::Decided(candidate),
        Classification::Decided(RealSign::Zero) => Classification::Decided(RealSign::Zero),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    })
}

fn reduce_algebraic_cusp_bivariate(
    polynomial: BivariatePolynomial,
    cusp: &BezierAlgebraicParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<BivariatePolynomial>> {
    bivariate_reduce_axis(
        &polynomial,
        cusp.polynomial(),
        CurveResultantParameter::First,
        policy,
    )
}

fn reduce_algebraic_cusp_radical_expression(
    expression: BezierAlgebraicCuspTwoTermExpression2,
    cusp: &BezierAlgebraicParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierAlgebraicCuspTwoTermExpression2>> {
    let rational = match reduce_algebraic_cusp_bivariate(expression.rational, cusp, policy)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let radical = match reduce_algebraic_cusp_bivariate(expression.radical, cusp, policy)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided(
        BezierAlgebraicCuspTwoTermExpression2 { rational, radical },
    ))
}

/// Complete exact parameter evidence for incidence on an analytic parallel.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum BezierParallelIncidence2 {
    /// Every defined parameter satisfies the incidence query.
    EntireCurve,
    /// The complete ordered set of represented or isolated algebraic parameters.
    Parameters(Vec<BezierParameter2>),
}

impl PartialEq for BezierParallel2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
            || (self.source() == other.source() && self.data.distance == other.data.distance)
    }
}

const MAX_PARALLEL_INTERSECTION_RESULTANT_DEGREE: usize = 128;
const MAX_FIXED_DISTANCE_RESULTANT_DEGREE: usize = 256;
const MAX_SELECTED_FIBER_QUOTIENT_DEGREE: usize = 8;
const MAX_FIXED_DISTANCE_QUOTIENT_DEGREE: usize = 10;
const MAX_SELECTED_FIBER_POLYNOMIAL_IMAGE_FIBER_DEGREE: usize = 16;
const MAX_SELECTED_FIBER_POLYNOMIAL_IMAGE_RETAINED_DEGREE: usize = 9;
const MAX_SELECTED_FIBER_POLYNOMIAL_IMAGE_DEGREE: usize = 32;
const MAX_SELECTED_FIBER_GLOBAL_IMAGE_SCHEDULE_DEGREE: usize = 64;
const MAX_DIRECT_SELECTED_NORM_ISOLATION_DEGREE: usize = 64;
const MAX_DIRECT_SELECTED_PAIR_NORM_ISOLATION_DEGREE: usize = 16;
const PARALLEL_INTERSECTION_RESULTANT_PRECISION: i32 = -128;

/// Certifies finiteness or regularity on the actual closed scalar range.
/// A strict hull is sufficient; otherwise the shared domain root authority
/// clips against the original endpoint evidence.
pub(crate) fn polynomial_is_nonzero_on_parameter_range(
    coefficients: &[Real],
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    // Constant weights need no endpoint envelope or root reconstruction.
    if coefficients
        .iter()
        .skip(1)
        .all(|coefficient| coefficient.zero_status() == hyperreal::ZeroKnowledge::Zero)
    {
        match coefficients
            .first()
            .map(|value| real_sign(value, &policy.strict_counterpart()))
        {
            None | Some(Some(RealSign::Zero)) => return Ok(Classification::Decided(false)),
            Some(Some(RealSign::Positive | RealSign::Negative)) => {
                return Ok(Classification::Decided(true));
            }
            Some(None) => {}
        }
    }
    if strict_polynomial_sign_on_curve_region_range(coefficients, range, policy)?.is_some() {
        return Ok(Classification::Decided(true));
    }
    policy.strict_predicate_pass(|| {
        let polynomial = match polynomial_from_coefficients(coefficients.to_vec(), policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => return Ok(Classification::Decided(false)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(CurveParameterDomain2::new(range, None)
            .finite_roots(&polynomial, policy)?
            .map(|roots| roots.is_empty()))
    })
}

/// Proves one polynomial has no zero on a retained finite parameter cell.
///
/// The cell endpoints may remain ordinary algebraic, selected-fiber, or
/// recursive-projective scalars. Their refined rational envelopes contain the
/// complete authored range. A Bernstein convex hull that lies strictly on one
/// side of zero therefore excludes every root without materializing either
/// endpoint or square-free factoring a whole-source polynomial.
fn strict_polynomial_sign_on_curve_region_range(
    coefficients: &[Real],
    range: &CurveParameterRange2,
    policy: &CurveContext,
) -> CurveResult<Option<RealSign>> {
    policy.strict_predicate_pass(|| {
        let strict = policy;
        let order = match range.start().cmp_by_refinement(range.end(), strict)? {
            Classification::Decided(std::cmp::Ordering::Less) => std::cmp::Ordering::Less,
            Classification::Decided(std::cmp::Ordering::Greater) => std::cmp::Ordering::Greater,
            Classification::Decided(std::cmp::Ordering::Equal) => {
                return Err(CurveError::InvalidBezierRange);
            }
            Classification::Uncertain(_) => return Ok(None),
        };
        let mut start_refinement = range
            .start()
            .as_bezier_parameter()
            .map(|parameter| BezierParameterRefinement2::new(parameter, strict));
        let mut end_refinement = range
            .end()
            .as_bezier_parameter()
            .map(|parameter| BezierParameterRefinement2::new(parameter, strict));
        // This is only a cheap accelerator for the complete polynomial-root
        // kernel below. Keep the proof budget bounded: a root on or extremely
        // near an endpoint can never produce a strict interval sign, and deep
        // speculative bisection makes the fallback strictly more expensive.
        for (steps, precision) in [(0, -32), (2, -64), (4, -96), (8, -128), (16, -192)] {
            let start = match start_refinement.as_mut() {
                Some(refinement) => CurveParameter2::from(refinement.refine_to(steps).clone()),
                None => match range.start().refined_for_finite_envelope(steps, strict)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(_) => continue,
                },
            };
            let end = match end_refinement.as_mut() {
                Some(refinement) => CurveParameter2::from(refinement.refine_to(steps).clone()),
                None => match range.end().refined_for_finite_envelope(steps, strict)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(_) => continue,
                },
            };
            let (Some(start), Some(end)) =
                (start.finite_envelope_bounds(), end.finite_envelope_bounds())
            else {
                return Ok(None);
            };
            let (lower, upper) = match order {
                std::cmp::Ordering::Less => (start.0, end.1),
                std::cmp::Ordering::Greater => (end.0, start.1),
                std::cmp::Ordering::Equal => unreachable!("the range order was validated"),
            };
            let Some([lower, upper]) = coefficients_value_interval_on_real_interval(
                coefficients,
                lower,
                upper,
                precision,
            )?
            else {
                continue;
            };
            let lower = Real::new(lower);
            let upper = Real::new(upper);
            if compare_reals(&lower, &Real::zero(), strict) == Some(std::cmp::Ordering::Greater) {
                return Ok(Some(RealSign::Positive));
            }
            if compare_reals(&upper, &Real::zero(), strict) == Some(std::cmp::Ordering::Less) {
                return Ok(Some(RealSign::Negative));
            }
        }
        Ok(None)
    })
}

fn polynomial_roots_in_parameter_domain(
    polynomial: &BezierParameterPolynomial,
    domain: CurveParameterDomain2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<BezierParameter2>>> {
    Ok(
        match selected_axis_parameters_in_domain(domain, policy, |axis| {
            Ok(axis
                .isolate(polynomial, policy)?
                .map(BezierAlgebraicFiberProjection2::Parameters))
        })? {
            Classification::Decided(BezierAlgebraicFiberProjection2::Parameters(parameters)) => {
                Classification::Decided(parameters)
            }
            Classification::Decided(_) => unreachable!("a nonzero polynomial has isolated roots"),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

fn common_polynomial_roots(
    first: Vec<Real>,
    second: Vec<Real>,
    domain: CurveParameterDomain2<'_>,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierParallelIncidence2>> {
    let first = match polynomial_from_coefficients(first, policy)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let second = match polynomial_from_coefficients(second, policy)? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
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
    Ok(
        match polynomial_roots_in_parameter_domain(&polynomial, domain, policy)? {
            Classification::Decided(parameters) => {
                Classification::Decided(BezierParallelIncidence2::Parameters(parameters))
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

fn first_incident_ray_polynomial_root(
    polynomials: &[&BezierParameterPolynomial],
    anchor: &Real,
    direction: BezierParameterRayDirection2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let mut first: Option<BezierParameter2> = None;
    for polynomial in polynomials {
        let roots = match polynomial.isolate_incident_ray_roots(anchor, direction, policy)? {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let Some(candidate) = roots.into_iter().next() else {
            continue;
        };
        let Some(retained) = first.as_ref() else {
            first = Some(candidate);
            continue;
        };
        let ordering = match candidate.cmp_by_refinement(retained, policy)? {
            Classification::Decided(ordering) => ordering,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let candidate_is_nearer = match direction {
            BezierParameterRayDirection2::Decreasing => ordering == std::cmp::Ordering::Greater,
            BezierParameterRayDirection2::Increasing => ordering == std::cmp::Ordering::Less,
        };
        if candidate_is_nearer {
            first = Some(candidate);
        }
    }
    Ok(Classification::Decided(first))
}

fn regular_barrier_polynomials(
    weight_coefficients: Option<&[Real]>,
    speed_squared: &[Real],
    policy: &CurveContext,
) -> CurveResult<Classification<(Option<BezierParameterPolynomial>, BezierParameterPolynomial)>> {
    let weight = match weight_coefficients
        .map(|weight| polynomial_from_coefficients(weight.to_vec(), policy))
        .transpose()?
    {
        Some(Classification::Decided(Some(weight))) => Some(weight),
        Some(Classification::Decided(None)) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Some(Classification::Uncertain(reason)) => {
            return Ok(Classification::Uncertain(reason));
        }
        None => None,
    };
    let speed = match polynomial_from_coefficients(speed_squared.to_vec(), policy)? {
        Classification::Decided(Some(speed)) => speed,
        Classification::Decided(None) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    Ok(Classification::Decided((weight, speed)))
}

struct BezierIncidentRayAnchor2 {
    adjacent_interval: Option<BezierParameterInterval>,
    represented_anchor: Real,
}

/// Certifies a regular enclosure beyond a retained scalar endpoint without
/// projecting its selected fiber or recursive field into a global scalar.
/// The directional enclosure boundary is a scheduling anchor; its complete
/// interval retains the bridge back to the exact authored endpoint.
fn retained_incident_ray_regular_anchor_from_polynomials(
    weight_coefficients: Option<&[Real]>,
    speed_squared: &[Real],
    endpoint: &CurveParameter2,
    direction: BezierParameterRayDirection2,
    policy: &CurveContext,
) -> CurveResult<Classification<BezierIncidentRayAnchor2>> {
    let unit_weight = [Real::one()];
    let weight = weight_coefficients.unwrap_or(&unit_weight);
    let retained_sign = |coefficients: &[Real]| {
        policy.strict_predicate_pass(|| endpoint.polynomial_sign(coefficients, policy))
    };
    match retained_sign(weight)? {
        Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    match retained_sign(speed_squared)? {
        Classification::Decided(RealSign::Positive) => {}
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Decided(RealSign::Negative) => {
            return Err(CurveError::Topology(
                "parallel source speed squared was certified negative".into(),
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }

    let strict_interval_sign = |coefficients: &[Real],
                                lower: &Real,
                                upper: &Real,
                                precision|
     -> CurveResult<Option<RealSign>> {
        let Some([lower, upper]) =
            coefficients_value_interval_on_real_interval(coefficients, lower, upper, precision)?
        else {
            return Ok(None);
        };
        let lower = Real::new(lower);
        let upper = Real::new(upper);
        Ok(
            if compare_reals(&lower, &Real::zero(), &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Greater)
            {
                Some(RealSign::Positive)
            } else if compare_reals(&upper, &Real::zero(), &CurveContext::STRICT)
                == Some(std::cmp::Ordering::Less)
            {
                Some(RealSign::Negative)
            } else {
                None
            },
        )
    };
    let mut retained_bridge = None;
    for (steps, precision) in [
        (0, -32),
        (2, -64),
        (4, -96),
        (8, -128),
        (16, -192),
        (32, -256),
        (64, -384),
        (128, -512),
        (256, -768),
        (512, -1024),
    ] {
        let refined = match policy
            .strict_predicate_pass(|| endpoint.refined_for_finite_envelope(steps, policy))?
        {
            Classification::Decided(refined) => refined,
            Classification::Uncertain(_) => continue,
        };
        let Some((lower, upper)) = refined.finite_envelope_bounds() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if strict_interval_sign(weight, lower, upper, precision)?.is_none()
            || strict_interval_sign(speed_squared, lower, upper, precision)?
                != Some(RealSign::Positive)
        {
            continue;
        }
        // The endpoint may belong to an exterior affine chart. Regularity
        // has already certified this entire bridge; only its ordering matters.
        let interval = match BezierParameterInterval::try_new(
            lower.clone(),
            upper.clone(),
            &CurveContext::STRICT,
        )? {
            Classification::Decided(interval) => interval,
            Classification::Uncertain(_) => continue,
        };
        let anchor = match direction {
            BezierParameterRayDirection2::Decreasing => lower.clone(),
            BezierParameterRayDirection2::Increasing => upper.clone(),
        };
        retained_bridge = Some((anchor, interval));
        break;
    }
    let Some((anchor, bridge)) = retained_bridge else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    Ok(Classification::Decided(BezierIncidentRayAnchor2 {
        adjacent_interval: Some(bridge),
        represented_anchor: anchor,
    }))
}

/// Finds a represented point in the same regular affine cell immediately
/// beyond one algebraic endpoint.
///
/// Exact nonvanishing at the selected endpoint is certified first. Its
/// isolator is then refined until neither the source weight nor tangent-speed
/// polynomial has a root anywhere in that isolator. The directional interval
/// boundary is consequently a safe represented chart anchor, while the whole
/// interval remains available to isolate the small selected-fiber piece
/// between the true endpoint and that chart.
fn algebraic_incident_ray_regular_anchor_from_polynomials(
    weight_coefficients: Option<&[Real]>,
    speed_squared: &[Real],
    endpoint: &BezierAlgebraicParameter2,
    direction: BezierParameterRayDirection2,
) -> CurveResult<Classification<BezierIncidentRayAnchor2>> {
    let strict = &CurveContext::STRICT;
    let (weight, speed) =
        match regular_barrier_polynomials(weight_coefficients, speed_squared, strict)? {
            Classification::Decided(polynomials) => polynomials,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    let endpoint_parameter = BezierParameter2::Algebraic(endpoint.clone());
    if let Some(weight) = weight.as_ref() {
        match signed_coefficients_at_parameter(weight.coefficients(), &endpoint_parameter, strict)?
        {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => {}
            Classification::Decided(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    match signed_coefficients_at_parameter(speed.coefficients(), &endpoint_parameter, strict)? {
        Classification::Decided(RealSign::Positive) => {}
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Decided(RealSign::Negative) => {
            return Err(CurveError::Topology(
                "Bezier tangent squared norm was certified negative".into(),
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }

    let mut refined = endpoint_parameter;
    loop {
        let interval = match &refined {
            BezierParameter2::Exact(anchor) => {
                return Ok(Classification::Decided(BezierIncidentRayAnchor2 {
                    adjacent_interval: None,
                    represented_anchor: anchor.clone(),
                }));
            }
            BezierParameter2::Algebraic(parameter) => parameter.interval().clone(),
        };
        let mut rootless = true;
        for polynomial in std::iter::once(&speed).chain(weight.as_ref()) {
            match polynomial.root_count_in_interval(&interval, strict)? {
                Classification::Decided(0) => {}
                Classification::Decided(_) | Classification::Uncertain(_) => {
                    rootless = false;
                    break;
                }
            }
        }
        if rootless {
            let represented_anchor = match direction {
                BezierParameterRayDirection2::Decreasing => interval.start().clone(),
                BezierParameterRayDirection2::Increasing => interval.end().clone(),
            };
            return Ok(Classification::Decided(BezierIncidentRayAnchor2 {
                adjacent_interval: Some(interval),
                represented_anchor,
            }));
        }
        let next = refined.clone().refined_isolating_interval(1, strict);
        if next == refined {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        }
        refined = next;
    }
}

fn regular_incident_ray_barrier_from_polynomials(
    weight_coefficients: Option<&[Real]>,
    speed_squared: &[Real],
    anchor: &Real,
    direction: BezierParameterRayDirection2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let (weight, speed) =
        match regular_barrier_polynomials(weight_coefficients, speed_squared, policy)? {
            Classification::Decided(polynomials) => polynomials,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
    if let Some(weight) = weight.as_ref() {
        match real_sign(&weight.evaluate(anchor), policy) {
            Some(RealSign::Positive | RealSign::Negative) => {}
            Some(RealSign::Zero) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
    match real_sign(&speed.evaluate(anchor), policy) {
        Some(RealSign::Positive) => {}
        Some(RealSign::Zero) => {
            // An authored stationary endpoint already is the first speed
            // barrier. Its extension is empty; contacts in the finite source
            // domain remain available with their own one-sided frames.
            return Ok(Classification::Decided(Some(BezierParameter2::Exact(
                anchor.clone(),
            ))));
        }
        Some(RealSign::Negative) => {
            return Err(CurveError::Topology(
                "Bezier tangent squared norm was certified negative".into(),
            ));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let mut polynomials = vec![&speed];
    if let Some(weight) = weight.as_ref() {
        polynomials.push(weight);
    }
    first_incident_ray_polynomial_root(&polynomials, anchor, direction, policy)
}

fn retain_parameters_before_incident_barrier(
    parameters: Vec<BezierParameter2>,
    barrier: Option<&BezierParameter2>,
    direction: BezierParameterRayDirection2,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<BezierParameter2>>> {
    let Some(barrier) = barrier else {
        return Ok(Classification::Decided(parameters));
    };
    let mut retained = Vec::with_capacity(parameters.len());
    for parameter in parameters {
        let ordering = match parameter.cmp_by_refinement(barrier, policy)? {
            Classification::Decided(ordering) => ordering,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let before = match direction {
            BezierParameterRayDirection2::Decreasing => ordering == std::cmp::Ordering::Greater,
            BezierParameterRayDirection2::Increasing => ordering == std::cmp::Ordering::Less,
        };
        if !before {
            break;
        }
        retained.push(parameter);
    }
    Ok(Classification::Decided(retained))
}

fn polynomial_incident_anchor_sign(
    polynomial: &BezierParameterPolynomial,
    anchor: &Real,
    direction: BezierParameterRayDirection2,
    policy: &CurveContext,
) -> Classification<RealSign> {
    let mut coefficients = polynomial.coefficients().to_vec();
    let direction_sign = match direction {
        BezierParameterRayDirection2::Decreasing => RealSign::Negative,
        BezierParameterRayDirection2::Increasing => RealSign::Positive,
    };
    let mut derivative_order = 0_usize;
    loop {
        match real_sign(&Real::eval_poly(&coefficients, anchor), policy) {
            Some(RealSign::Zero) => {}
            Some(sign @ (RealSign::Positive | RealSign::Negative)) => {
                return Classification::Decided(
                    if direction_sign == RealSign::Negative && !derivative_order.is_multiple_of(2) {
                        match sign {
                            RealSign::Positive => RealSign::Negative,
                            RealSign::Negative => RealSign::Positive,
                            RealSign::Zero => unreachable!("the incident germ sign is nonzero"),
                        }
                    } else {
                        sign
                    },
                );
            }
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
        if coefficients.len() == 1 {
            unreachable!("a normalized parameter polynomial is nonzero");
        }
        coefficients = polynomial_derivative(&coefficients);
        derivative_order += 1;
    }
}

const fn real_signs_are_opposite(first: RealSign, second: RealSign) -> bool {
    matches!(
        (first, second),
        (RealSign::Positive, RealSign::Negative) | (RealSign::Negative, RealSign::Positive)
    )
}

fn parallel_line_neighbor_sign(
    parallel: &BezierParallel2,
    line: &LineSeg2,
    certified_direction: Option<(&Real, &Real)>,
    roots: &[BezierParameter2],
    root_index: usize,
    after: bool,
    tangent_field: Option<&BezierAnalyticParallelTangentField2>,
    retained_range: Option<&CurveParameterRange2>,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let root = &roots[root_index];
    let default_boundary;
    let domain_boundary = match retained_range.and_then(CurveParameterRange2::as_bezier_parameters)
    {
        Some((start, end)) => {
            if after {
                end
            } else {
                start
            }
        }
        None => {
            default_boundary =
                BezierParameter2::Exact(if after { Real::one() } else { Real::zero() });
            &default_boundary
        }
    };
    let boundary_order = match root.cmp_by_refinement(domain_boundary, policy)? {
        Classification::Decided(order) => order,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let has_interior_side = if after {
        boundary_order == std::cmp::Ordering::Less
    } else {
        boundary_order == std::cmp::Ordering::Greater
    };
    let sample = if has_interior_side {
        let neighbor = if after {
            roots.get(root_index + 1).unwrap_or(domain_boundary)
        } else if root_index == 0 {
            domain_boundary
        } else {
            &roots[root_index - 1]
        };
        let sample = if after {
            root.strict_scalar_between_ordered(neighbor, policy)?
        } else {
            neighbor.strict_scalar_between_ordered(root, policy)?
        };
        match sample {
            Classification::Decided(sample) => sample,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    } else if boundary_order == std::cmp::Ordering::Equal && retained_range.is_none() {
        let mut step = (Real::one() / Real::from(2_u8))?;
        loop {
            let sample = if after {
                Real::one() + &step
            } else {
                -step.clone()
            };
            match signed_parallel_linear_projection_at_parameter(
                parallel,
                &BezierParameter2::Exact(sample),
                line,
                certified_direction,
                true,
                tangent_field,
                policy,
            )? {
                Classification::Decided(RealSign::Zero) => {
                    step = (step / Real::from(2_u8))?;
                }
                decided => return Ok(decided),
            }
        }
    } else {
        return Err(CurveError::Topology(
            "parallel supporting-line root lies outside the unit domain".into(),
        ));
    };

    match signed_parallel_linear_projection_at_parameter(
        parallel,
        &BezierParameter2::Exact(sample),
        line,
        certified_direction,
        true,
        tangent_field,
        policy,
    )? {
        Classification::Decided(RealSign::Zero) => {
            Ok(Classification::Uncertain(UncertaintyReason::Boundary))
        }
        decided => Ok(decided),
    }
}

fn signed_parallel_linear_projection_at_parameter(
    parallel: &BezierParallel2,
    parameter: &BezierParameter2,
    line: &LineSeg2,
    certified_direction: Option<(&Real, &Real)>,
    oriented_side: bool,
    tangent_field: Option<&BezierAnalyticParallelTangentField2>,
    policy: &CurveContext,
) -> CurveResult<Classification<RealSign>> {
    let source = parallel.source_power_basis()?;
    let differential = parallel.differential()?;
    let (tangent_x, tangent_y) = tangent_field
        .map(|field| (&field.x[..], &field.y[..]))
        .unwrap_or((&differential.tangent_x, &differential.tangent_y));
    let weight = source
        .weight
        .map_or_else(|| vec![Real::one()], ToOwned::to_owned);
    let weighted_origin_x = polynomial_scale(&weight, line.start().x());
    let weighted_origin_y = polynomial_scale(&weight, line.start().y());
    let delta_x = polynomial_subtract(source.x_numerator, &weighted_origin_x);
    let delta_y = polynomial_subtract(source.y_numerator, &weighted_origin_y);
    let (direction_x, direction_y) = certified_direction.map_or_else(
        || line.delta(),
        |(direction_x, direction_y)| (direction_x.clone(), direction_y.clone()),
    );
    let source_projection = if oriented_side {
        polynomial_subtract(
            &polynomial_scale(&delta_y, &direction_x),
            &polynomial_scale(&delta_x, &direction_y),
        )
    } else {
        polynomial_add(
            &polynomial_scale(&delta_x, &direction_x),
            &polynomial_scale(&delta_y, &direction_y),
        )
    };
    let weight_sign = match signed_coefficients_at_parameter(&weight, parameter, policy)? {
        Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => sign,
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let source_sign = match signed_coefficients_at_parameter(&source_projection, parameter, policy)?
    {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if real_sign(parallel.distance(), policy) == Some(RealSign::Zero) {
        return Ok(Classification::Decided(product_sign(
            source_sign,
            weight_sign,
        )));
    }

    let normal_projection = if oriented_side {
        polynomial_add(
            &polynomial_scale(tangent_x, &direction_x),
            &polynomial_scale(tangent_y, &direction_y),
        )
    } else {
        polynomial_subtract(
            &polynomial_scale(tangent_x, &direction_y),
            &polynomial_scale(tangent_y, &direction_x),
        )
    };
    let normal_projection = polynomial_multiply(
        &polynomial_scale(&normal_projection, parallel.distance()),
        &weight,
    );
    let normal_sign = match signed_coefficients_at_parameter(&normal_projection, parameter, policy)?
    {
        Classification::Decided(sign) => sign,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let speed_squared = polynomial_add(
        &polynomial_multiply(tangent_x, tangent_x),
        &polynomial_multiply(tangent_y, tangent_y),
    );
    match signed_coefficients_at_parameter(&speed_squared, parameter, policy)? {
        Classification::Decided(RealSign::Positive) => {}
        Classification::Decided(RealSign::Zero) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        Classification::Decided(RealSign::Negative) => {
            return Err(CurveError::Topology(
                "parallel tangent squared norm was certified negative".into(),
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }

    let radical_sum_sign = match (source_sign, normal_sign) {
        (RealSign::Zero, sign) | (sign, RealSign::Zero) => sign,
        (first, second) if first == second => first,
        (source_sign, normal_sign) => {
            let squared_difference = polynomial_subtract(
                &polynomial_multiply(
                    &polynomial_multiply(&source_projection, &source_projection),
                    &speed_squared,
                ),
                &polynomial_multiply(&normal_projection, &normal_projection),
            );
            match signed_coefficients_at_parameter(&squared_difference, parameter, policy)? {
                Classification::Decided(RealSign::Positive) => source_sign,
                Classification::Decided(RealSign::Negative) => normal_sign,
                Classification::Decided(RealSign::Zero) => RealSign::Zero,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    };
    Ok(Classification::Decided(product_sign(
        radical_sum_sign,
        weight_sign,
    )))
}

/// The derivative of a positive-leading circle/line quadratic has the root
/// branch sign.  The retained tangent-cross convention reverses that sign and
/// then applies the authored circle traversal.
const fn recursive_circle_contact_tangent_cross_sign(
    quadratic_branch: i8,
    turn_sign: RealSign,
) -> RealSign {
    match quadratic_branch {
        -1 => turn_sign,
        0 => RealSign::Zero,
        1 => match turn_sign {
            RealSign::Negative => RealSign::Positive,
            RealSign::Positive => RealSign::Negative,
            RealSign::Zero => unreachable!(),
        },
        _ => unreachable!(),
    }
}

/// Signs one affine predicate at both roots of a positive-leading recursive
/// quadratic without eagerly adjoining the root radical.  A geometric pair
/// of endpoint-side predicates is preferred for the product because it keeps
/// exact diameter tangencies structural; the coefficient identity remains
/// the complete fallback for arbitrary affine predicates.
fn recursive_quadratic_affine_predicate_root_signs(
    a: &BezierRecursiveQuadraticValue2,
    b: &BezierRecursiveQuadraticValue2,
    c: &BezierRecursiveQuadraticValue2,
    discriminant_sign: RealSign,
    constant: &BezierRecursiveQuadraticValue2,
    slope: &BezierRecursiveQuadraticValue2,
    geometric_product_factors: Option<&[BezierRecursiveQuadraticValue2; 2]>,
) -> CurveResult<Option<[RealSign; 2]>> {
    let decided_sign = |value: &BezierRecursiveQuadraticValue2| -> CurveResult<Option<RealSign>> {
        Ok(match value.sign(&CurveContext::STRICT)? {
            Classification::Decided(sign) => Some(sign),
            Classification::Uncertain(_) => None,
        })
    };
    let Some(sum) = a
        .multiply(constant)
        .and_then(|sum| sum.scale(&Real::from(2_i8)))
        .and_then(|sum| b.multiply(slope).and_then(|other| sum.subtract(&other)))
    else {
        return Ok(None);
    };
    if discriminant_sign == RealSign::Zero {
        let Some(sum_sign) = decided_sign(&sum)? else {
            return Ok(None);
        };
        return Ok(Some([sum_sign; 2]));
    }
    let geometric_product_sign = if let Some(factors) = geometric_product_factors
        && let Some(product) = factors[0].multiply(&factors[1])
    {
        decided_sign(&product)?
    } else {
        None
    };
    let product_sign = if let Some(sign) = geometric_product_sign {
        sign
    } else {
        let Some(product) = (|| {
            let product = a.multiply(&constant.square()?)?;
            let product = product.subtract(&b.multiply(&constant.multiply(slope)?)?)?;
            product.add(&c.multiply(&slope.square()?)?)
        })() else {
            return Ok(None);
        };
        let Some(sign) = decided_sign(&product)? else {
            return Ok(None);
        };
        sign
    };
    let nonzero_sign = |value: &BezierRecursiveQuadraticValue2| -> CurveResult<Option<RealSign>> {
        Ok(match value.sign_with_nonzero_certificate()? {
            Classification::Decided(sign) => Some(sign),
            Classification::Uncertain(_) => None,
        })
    };
    let (sum_sign, slope_sign) = match product_sign {
        // Equal nonzero root signs make their sum nonzero.
        RealSign::Positive => {
            let Some(sum_sign) = nonzero_sign(&sum)? else {
                return Ok(None);
            };
            (sum_sign, RealSign::Zero)
        }
        // Opposite nonzero root signs require a nonzero affine slope; its
        // sign orders the two quadratic branches.
        RealSign::Negative => {
            let Some(slope_sign) = nonzero_sign(slope)? else {
                return Ok(None);
            };
            (RealSign::Zero, slope_sign)
        }
        RealSign::Zero => {
            let (Some(sum_sign), Some(slope_sign)) = (decided_sign(&sum)?, decided_sign(slope)?)
            else {
                return Ok(None);
            };
            (sum_sign, slope_sign)
        }
    };
    Ok(quadratic_affine_root_signs(
        discriminant_sign,
        product_sign,
        sum_sign,
        slope_sign,
    ))
}

/// Signs an affine predicate at the `-sqrt(discriminant)` and
/// `+sqrt(discriminant)` roots of a positive-leading quadratic. `product` is
/// the predicate-value product, `sum` its value sum multiplied by the positive
/// leading coefficient, and `slope` the affine predicate slope.
const fn quadratic_affine_root_signs(
    discriminant: RealSign,
    product: RealSign,
    sum: RealSign,
    slope: RealSign,
) -> Option<[RealSign; 2]> {
    if matches!(discriminant, RealSign::Negative) {
        return None;
    }
    if matches!(discriminant, RealSign::Zero) {
        return Some([sum, sum]);
    }
    match product {
        RealSign::Positive => match sum {
            RealSign::Negative => Some([RealSign::Negative; 2]),
            RealSign::Positive => Some([RealSign::Positive; 2]),
            RealSign::Zero => None,
        },
        RealSign::Negative => match slope {
            RealSign::Negative => Some([RealSign::Positive, RealSign::Negative]),
            RealSign::Positive => Some([RealSign::Negative, RealSign::Positive]),
            RealSign::Zero => None,
        },
        RealSign::Zero => match (slope, sum) {
            (RealSign::Negative, RealSign::Negative) => Some([RealSign::Zero, RealSign::Negative]),
            (RealSign::Negative, RealSign::Positive) => Some([RealSign::Positive, RealSign::Zero]),
            (RealSign::Positive, RealSign::Negative) => Some([RealSign::Negative, RealSign::Zero]),
            (RealSign::Positive, RealSign::Positive) => Some([RealSign::Zero, RealSign::Positive]),
            (RealSign::Zero, RealSign::Zero) => Some([RealSign::Zero; 2]),
            _ => None,
        },
    }
}

#[cfg(test)]
pub(crate) use conversion_tests::recursively_line_contact_radial_half;

#[cfg(test)]
mod conversion_tests;

#[cfg(test)]
mod regression_tests;
