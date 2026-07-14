//! Native Bezier split materialization over exact and algebraic parameters.
//!
//! This module is the first consumer of [`BezierParameter2`]. It materializes
//! polynomial and rational Bezier subcurves when both range boundaries are
//! represented [`Real`](hyperreal::Real) values. For algebraic boundaries it
//! now consumes the boundary into exact endpoint point/tangent images when
//! that construction is certified, otherwise it carries the interval forward
//! as an unresolved fragment. That is intentional: the exactness model's exact
//! geometric-computation model requires exact objects to survive until the
//! kernel has a certified operation for them, rather than converting algebraic
//! roots to finite approximations.
//!
//! Exact materialization uses de Casteljau subdivision. The construction is
//! affine for polynomial Beziers and homogeneous for rational Beziers, matching
//! de Casteljau subdivision, and the rational Bezier treatment in the Bernstein and de Casteljau curve model. Algebraic parameters
//! whose defining equation is certified linear are first promoted to their
//! represented [`Real`] root, so the same exact subdivision path handles that
//! materializable algebraic subset without approximating nonlinear roots.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};

use crate::classify::{compare_reals, in_closed_unit_interval, is_zero};
use crate::{
    BezierAlgebraicEndpointImage2, BezierAlgebraicParameter2, BezierParameter2, Classification,
    CubicBezier2, CurveError, CurvePolicy, CurveResult, Point2, QuadraticBezier2, RationalBezier2,
    RationalQuadraticBezier2, UncertaintyReason,
};

/// A native Bezier subcurve produced by exact split materialization.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub enum BezierSubcurve2 {
    /// Polynomial quadratic Bezier subcurve.
    Quadratic(QuadraticBezier2),
    /// Polynomial cubic Bezier subcurve.
    Cubic(CubicBezier2),
    /// Rational quadratic Bezier/conic subcurve.
    RationalQuadratic(RationalQuadraticBezier2),
    /// General exact rational Bezier subcurve.
    Rational(RationalBezier2),
}

/// One fragment between adjacent split boundaries.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub enum BezierSplitFragment2 {
    /// Both boundaries were represented exactly and the native subcurve exists.
    Materialized {
        /// Start split boundary in the original parameter space.
        start: BezierParameter2,
        /// End split boundary in the original parameter space.
        end: BezierParameter2,
        /// Native subcurve over this range.
        curve: BezierSubcurve2,
    },
    /// At least one boundary is algebraic, and its exact endpoint images were
    /// constructed without making a native subcurve.
    AlgebraicEndpointImages {
        /// Whether traversal runs from the source end boundary to its start boundary.
        reversed: bool,
        /// Start split boundary in the original parameter space.
        start: BezierParameter2,
        /// End split boundary in the original parameter space.
        end: BezierParameter2,
        /// Source curve that generated this algebraic-boundary fragment.
        ///
        /// This is not a native subcurve over the algebraic parameter range.
        /// It is retained construction evidence for conservative exact
        /// measurements, such as source-curve envelopes, that can safely
        /// overbound the algebraic subrange without evaluating an algebraic
        /// split point as a floating coordinate.
        source_curve: Option<BezierSubcurve2>,
        /// Exact point/tangent image when the start boundary is algebraic.
        start_image: Option<BezierAlgebraicEndpointImage2>,
        /// Exact point/tangent image when the end boundary is algebraic.
        end_image: Option<BezierAlgebraicEndpointImage2>,
    },
    /// At least one boundary is algebraic and must be carried forward.
    Unresolved {
        /// Start split boundary in the original parameter space.
        start: BezierParameter2,
        /// End split boundary in the original parameter space.
        end: BezierParameter2,
    },
}

/// Ordered split result for one Bezier segment.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierSplitMaterialization2 {
    fragments: Vec<BezierSplitFragment2>,
}

impl BezierSplitMaterialization2 {
    /// Constructs a materialization result from ordered fragments.
    pub fn new(fragments: Vec<BezierSplitFragment2>) -> CurveResult<Self> {
        validate_bezier_split_fragments(&fragments)?;
        Ok(Self { fragments })
    }

    /// Returns fragments in increasing source-parameter order.
    pub fn fragments(&self) -> &[BezierSplitFragment2] {
        &self.fragments
    }

    /// Returns true when every fragment was materialized as a native curve.
    pub fn is_fully_materialized(&self) -> bool {
        self.fragments
            .iter()
            .all(|fragment| matches!(fragment, BezierSplitFragment2::Materialized { .. }))
    }

    /// Returns true when at least one algebraic-boundary fragment remains.
    pub fn has_unresolved_fragments(&self) -> bool {
        self.fragments
            .iter()
            .any(|fragment| matches!(fragment, BezierSplitFragment2::Unresolved { .. }))
    }

    /// Returns true when at least one algebraic-boundary fragment carries
    /// exact endpoint point/tangent images.
    pub fn has_algebraic_endpoint_images(&self) -> bool {
        self.fragments.iter().any(|fragment| {
            matches!(
                fragment,
                BezierSplitFragment2::AlgebraicEndpointImages { .. }
            )
        })
    }
}

impl BezierSubcurve2 {
    /// Returns the exact local-parameter start point.
    pub fn start(&self) -> &Point2 {
        match self {
            Self::Quadratic(curve) => curve.start(),
            Self::Cubic(curve) => curve.start(),
            Self::RationalQuadratic(curve) => curve.start(),
            Self::Rational(curve) => curve.start(),
        }
    }

    /// Returns the exact local-parameter end point.
    pub fn end(&self) -> &Point2 {
        match self {
            Self::Quadratic(curve) => curve.end(),
            Self::Cubic(curve) => curve.end(),
            Self::RationalQuadratic(curve) => curve.end(),
            Self::Rational(curve) => curve.end(),
        }
    }

    /// Evaluates this native subcurve at an exact local parameter.
    pub fn point_at(&self, parameter: &Real, policy: &CurvePolicy) -> Classification<Point2> {
        match self {
            Self::Quadratic(curve) => Classification::Decided(curve.point_at(parameter.clone())),
            Self::Cubic(curve) => Classification::Decided(curve.point_at(parameter.clone())),
            Self::RationalQuadratic(curve) => curve.point_at(parameter.clone(), policy),
            Self::Rational(curve) => curve.point_at_classified(parameter, policy),
        }
    }

    pub(crate) fn split_at_parameters(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        match self {
            Self::Quadratic(curve) => curve.split_at_parameters(parameters, policy),
            Self::Cubic(curve) => curve.split_at_parameters(parameters, policy),
            Self::RationalQuadratic(curve) => curve.split_at_parameters(parameters, policy),
            Self::Rational(curve) => curve.split_at_parameters(parameters, policy),
        }
    }

    pub(crate) fn split_at_parameters_refined(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        match self {
            Self::Quadratic(curve) => split_curve_at_parameters(
                parameters,
                policy,
                true,
                |_| true,
                |start, end| {
                    Ok(Self::Quadratic(
                        curve.subcurve_between_exact(start, end, policy)?,
                    ))
                },
                |parameter| BezierAlgebraicEndpointImage2::quadratic(curve, parameter, policy),
                self.clone(),
            ),
            Self::Cubic(curve) => split_curve_at_parameters(
                parameters,
                policy,
                true,
                |_| true,
                |start, end| {
                    Ok(Self::Cubic(
                        curve.subcurve_between_exact(start, end, policy)?,
                    ))
                },
                |parameter| BezierAlgebraicEndpointImage2::cubic(curve, parameter, policy),
                self.clone(),
            ),
            Self::RationalQuadratic(curve) => split_curve_at_parameters(
                parameters,
                policy,
                true,
                |parameter| {
                    matches!(
                        curve.point_at(parameter.clone(), policy),
                        Classification::Decided(_)
                    )
                },
                |start, end| {
                    Ok(Self::RationalQuadratic(
                        curve.subcurve_between_exact(start, end, policy)?,
                    ))
                },
                |parameter| {
                    BezierAlgebraicEndpointImage2::rational_quadratic(curve, parameter, policy)
                },
                self.clone(),
            ),
            Self::Rational(curve) => split_curve_at_parameters(
                parameters,
                policy,
                true,
                |parameter| {
                    matches!(
                        curve.point_at_classified(parameter, policy),
                        Classification::Decided(_)
                    )
                },
                |start, end| match curve.subcurve_between_exact(start, end, policy)? {
                    Classification::Decided(curve) => Ok(Self::Rational(curve)),
                    Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
                        "general rational Bezier exact split is uncertified: {reason:?}"
                    ))),
                },
                |parameter| BezierAlgebraicEndpointImage2::rational(curve, parameter, policy),
                self.clone(),
            ),
        }
    }

    pub(crate) fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        match self {
            Self::Quadratic(curve) => Ok(Classification::Decided(Self::Quadratic(
                curve.subcurve_between_exact(start, end, policy)?,
            ))),
            Self::Cubic(curve) => Ok(Classification::Decided(Self::Cubic(
                curve.subcurve_between_exact(start, end, policy)?,
            ))),
            Self::RationalQuadratic(curve) => Ok(Classification::Decided(Self::RationalQuadratic(
                curve.subcurve_between_exact(start, end, policy)?,
            ))),
            Self::Rational(curve) => curve
                .subcurve_between_exact(start, end, policy)
                .map(|result| result.map(Self::Rational)),
        }
    }

    /// Returns the same exact image with traversal direction reversed.
    pub fn reversed(&self) -> Self {
        match self {
            Self::Quadratic(curve) => Self::Quadratic(QuadraticBezier2::new(
                curve.end().clone(),
                curve.control().clone(),
                curve.start().clone(),
            )),
            Self::Cubic(curve) => Self::Cubic(CubicBezier2::new(
                curve.end().clone(),
                curve.control2().clone(),
                curve.control1().clone(),
                curve.start().clone(),
            )),
            Self::RationalQuadratic(curve) => Self::RationalQuadratic(
                RationalQuadraticBezier2::try_new_with_common_weight_sign(
                    curve.end().clone(),
                    curve.control().clone(),
                    curve.start().clone(),
                    curve.end_weight().clone(),
                    curve.control_weight().clone(),
                    curve.start_weight().clone(),
                    curve.common_nonzero_weight_sign(&CurvePolicy::certified()),
                )
                .expect("reversing a valid rational quadratic remains valid"),
            ),
            Self::Rational(curve) => Self::Rational(curve.reversed()),
        }
    }
}

impl BezierSplitFragment2 {
    /// Returns true when this fragment retains exact algebraic endpoint images.
    pub const fn is_algebraic_endpoint_images(&self) -> bool {
        matches!(self, Self::AlgebraicEndpointImages { .. })
    }

    /// Constructs an exact represented point certified inside this fragment.
    ///
    /// Algebraic boundaries use the rational gap between their disjoint
    /// isolating intervals. This samples neither root: interval ordering proves
    /// the represented parameter lies strictly between the exact boundaries.
    pub fn representative_point(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Point2>> {
        match self {
            Self::Materialized { curve, .. } => {
                let half = (Real::one() / Real::from(2_i8))?;
                Ok(curve.point_at(&half, policy))
            }
            Self::AlgebraicEndpointImages {
                start,
                end,
                source_curve: Some(source_curve),
                ..
            } => {
                let parameter = match start.strict_rational_between(end, policy)? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                Ok(source_curve.point_at(&parameter, policy))
            }
            Self::AlgebraicEndpointImages {
                source_curve: None, ..
            }
            | Self::Unresolved { .. } => {
                Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
            }
        }
    }

    /// Returns the retained fragment in reverse traversal direction.
    ///
    /// Materialized fragments reverse exactly. Algebraic endpoint-image
    /// carriers retain their source-oriented parameter range and exact images,
    /// while recording the opposite traversal direction. Consumers transform
    /// endpoint and derivative evidence when they traverse the carrier.
    pub fn reversed(&self) -> CurveResult<Self> {
        match self {
            Self::Materialized { start, end, curve } => Ok(Self::Materialized {
                start: start.clone(),
                end: end.clone(),
                curve: curve.reversed(),
            }),
            Self::AlgebraicEndpointImages {
                reversed,
                start,
                end,
                source_curve,
                start_image,
                end_image,
            } => Ok(Self::AlgebraicEndpointImages {
                reversed: !reversed,
                start: start.clone(),
                end: end.clone(),
                source_curve: source_curve.clone(),
                start_image: start_image.clone(),
                end_image: end_image.clone(),
            }),
            Self::Unresolved { .. } => Err(CurveError::Topology(
                "reversing an unresolved Bezier split fragment requires endpoint evidence"
                    .to_owned(),
            )),
        }
    }
}

fn validate_bezier_split_fragments(fragments: &[BezierSplitFragment2]) -> CurveResult<()> {
    if fragments.is_empty() {
        return Err(CurveError::Topology(
            "Bezier split materialization must carry at least one source fragment".into(),
        ));
    }

    let policy = CurvePolicy::certified();
    validate_bezier_split_coverage(fragments, &policy)?;
    for (left_index, left) in fragments.iter().enumerate() {
        validate_bezier_split_fragment(left, &policy)?;
        if let Some(right) = fragments.get(left_index + 1) {
            validate_adjacent_bezier_split_fragments(left, right)?;
        }
        if fragments[left_index + 1..]
            .iter()
            .any(|right| right == left)
        {
            return Err(CurveError::Topology(
                "Bezier split materialization must not contain duplicate fragments".into(),
            ));
        }
    }
    Ok(())
}

fn validate_bezier_split_coverage(
    fragments: &[BezierSplitFragment2],
    policy: &CurvePolicy,
) -> CurveResult<()> {
    let (first_start, _) = bezier_split_fragment_range(&fragments[0]);
    let (_, last_end) = bezier_split_fragment_range(&fragments[fragments.len() - 1]);
    validate_bezier_boundary_equals(first_start, &BezierParameter2::Exact(Real::zero()), policy)?;
    validate_bezier_boundary_equals(last_end, &BezierParameter2::Exact(Real::one()), policy)?;
    Ok(())
}

fn validate_bezier_boundary_equals(
    actual: &BezierParameter2,
    expected: &BezierParameter2,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    match actual.cmp_by_interval(expected, policy)? {
        Classification::Decided(Ordering::Equal) => Ok(()),
        Classification::Decided(_) => Err(CurveError::Topology(
            "Bezier split materialization must cover the full source parameter interval".into(),
        )),
        Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
            "Bezier split materialization source coverage is uncertain: {reason:?}"
        ))),
    }
}

fn validate_bezier_split_fragment(
    fragment: &BezierSplitFragment2,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    let (start, end) = bezier_split_fragment_range(fragment);
    validate_parameter(start, policy)?;
    validate_parameter(end, policy)?;
    validate_bezier_parameter_order(start, end, policy)?;

    match fragment {
        BezierSplitFragment2::Materialized { start, end, .. } => {
            if !start.is_exact() || !end.is_exact() {
                return Err(CurveError::Topology(
                    "materialized Bezier split fragment must have exact range boundaries".into(),
                ));
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
                return Err(CurveError::Topology(
                    "algebraic Bezier split endpoint images must retain source curve provenance"
                        .into(),
                ));
            };
            validate_algebraic_endpoint_image_boundary(
                "start",
                start,
                start_image.as_ref(),
                source_curve,
                policy,
            )?;
            validate_algebraic_endpoint_image_boundary(
                "end",
                end,
                end_image.as_ref(),
                source_curve,
                policy,
            )?;
        }
        BezierSplitFragment2::Unresolved { start, end } => {
            if start.is_exact() && end.is_exact() {
                return Err(CurveError::Topology(
                    "unresolved Bezier split fragment must have an algebraic range boundary".into(),
                ));
            }
        }
    }

    Ok(())
}

fn validate_adjacent_bezier_split_fragments(
    left: &BezierSplitFragment2,
    right: &BezierSplitFragment2,
) -> CurveResult<()> {
    let (_, left_end) = bezier_split_fragment_range(left);
    let (right_start, _) = bezier_split_fragment_range(right);
    if left_end != right_start {
        return Err(CurveError::Topology(
            "Bezier split materialization fragments must be contiguous and ordered".into(),
        ));
    }
    if let (
        BezierSplitFragment2::Materialized {
            curve: left_curve, ..
        },
        BezierSplitFragment2::Materialized {
            curve: right_curve, ..
        },
    ) = (left, right)
    {
        let left_endpoint = left_curve.end_point();
        let right_endpoint = right_curve.start_point();
        if !certified_split_points_equal(&left_endpoint, &right_endpoint, &CurvePolicy::certified())
        {
            return Err(CurveError::Topology(
                "adjacent materialized Bezier split fragments must be endpoint-connected".into(),
            ));
        }
    }
    Ok(())
}

fn certified_split_points_equal(left: &Point2, right: &Point2, policy: &CurvePolicy) -> bool {
    is_zero(&left.distance_squared(right), policy) == Some(true)
}

fn bezier_split_fragment_range(
    fragment: &BezierSplitFragment2,
) -> (&BezierParameter2, &BezierParameter2) {
    match fragment {
        BezierSplitFragment2::Materialized { start, end, .. }
        | BezierSplitFragment2::AlgebraicEndpointImages { start, end, .. }
        | BezierSplitFragment2::Unresolved { start, end } => (start, end),
    }
}

fn validate_bezier_parameter_order(
    start: &BezierParameter2,
    end: &BezierParameter2,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    match start.cmp_by_interval(end, policy)? {
        Classification::Decided(Ordering::Less) => Ok(()),
        Classification::Decided(Ordering::Equal | Ordering::Greater) => Err(CurveError::Topology(
            "Bezier split fragment range must be strictly increasing".into(),
        )),
        Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
            "Bezier split fragment range ordering is uncertain: {reason:?}"
        ))),
    }
}

fn validate_algebraic_endpoint_image_boundary(
    name: &str,
    boundary: &BezierParameter2,
    image: Option<&BezierAlgebraicEndpointImage2>,
    source_curve: &BezierSubcurve2,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    match (boundary, image) {
        (BezierParameter2::Exact(_), None) => Ok(()),
        (BezierParameter2::Exact(_), Some(_)) => Err(CurveError::Topology(format!(
            "exact {name} Bezier split boundary must not carry algebraic endpoint image evidence"
        ))),
        (BezierParameter2::Algebraic(parameter), Some(image)) => {
            if image.parameter() != parameter {
                return Err(CurveError::Topology(format!(
                    "algebraic {name} Bezier split endpoint image parameter does not match boundary"
                )));
            }
            if !image.is_transformed() {
                return Err(CurveError::Topology(format!(
                    "algebraic {name} Bezier split endpoint image must be exact transformed evidence"
                )));
            }
            let expected =
                BezierAlgebraicEndpointImage2::from_source_curve(source_curve, parameter, policy)?;
            if &expected != image {
                return Err(CurveError::Topology(format!(
                    "algebraic {name} Bezier split endpoint image does not match retained source curve"
                )));
            }
            Ok(())
        }
        (BezierParameter2::Algebraic(_), None) => Err(CurveError::Topology(format!(
            "algebraic {name} Bezier split boundary must carry endpoint image evidence"
        ))),
    }
}

impl QuadraticBezier2 {
    /// Splits this quadratic at exact/algebraic Bezier parameters.
    pub fn split_at_parameters(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(
            parameters,
            policy,
            false,
            |_| true,
            |start, end| {
                Ok(BezierSubcurve2::Quadratic(
                    self.subcurve_between_exact(start, end, policy)?,
                ))
            },
            |parameter| BezierAlgebraicEndpointImage2::quadratic(self, parameter, policy),
            BezierSubcurve2::Quadratic(self.clone()),
        )
    }

    /// Materializes the exact subcurve over `[start, end]`.
    pub fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurvePolicy,
    ) -> CurveResult<QuadraticBezier2> {
        validate_exact_range(start, end, policy)?;
        if compare_reals(start, end, policy) == Some(Ordering::Equal) {
            let point = self.point_at(start.clone());
            return Ok(QuadraticBezier2::new(point.clone(), point.clone(), point));
        }
        if compare_reals(start, &Real::zero(), policy) == Some(Ordering::Equal)
            && compare_reals(end, &Real::one(), policy) == Some(Ordering::Equal)
        {
            return Ok(self.clone());
        }
        if compare_reals(start, &Real::zero(), policy) == Some(Ordering::Equal) {
            let (left, _) = self.split_at_exact(end.clone());
            return Ok(left);
        }
        if compare_reals(end, &Real::one(), policy) == Some(Ordering::Equal) {
            let (_, right) = self.split_at_exact(start.clone());
            return Ok(right);
        }

        let (left, _) = self.split_at_exact(end.clone());
        let local_start = (start.clone() / end.clone())?;
        let (_, middle) = left.split_at_exact(local_start);
        Ok(middle)
    }

    /// Splits this quadratic at one represented parameter.
    pub fn split_at_exact(&self, t: Real) -> (QuadraticBezier2, QuadraticBezier2) {
        let p01 = self.start().lerp(self.control(), t.clone());
        let p12 = self.control().lerp(self.end(), t.clone());
        let p012 = p01.lerp(&p12, t);
        (
            QuadraticBezier2::new(self.start().clone(), p01, p012.clone()),
            QuadraticBezier2::new(p012, p12, self.end().clone()),
        )
    }
}

impl CubicBezier2 {
    /// Splits this cubic at exact/algebraic Bezier parameters.
    pub fn split_at_parameters(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(
            parameters,
            policy,
            false,
            |_| true,
            |start, end| {
                Ok(BezierSubcurve2::Cubic(
                    self.subcurve_between_exact(start, end, policy)?,
                ))
            },
            |parameter| BezierAlgebraicEndpointImage2::cubic(self, parameter, policy),
            BezierSubcurve2::Cubic(self.clone()),
        )
    }

    /// Materializes the exact subcurve over `[start, end]`.
    pub fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurvePolicy,
    ) -> CurveResult<CubicBezier2> {
        validate_exact_range(start, end, policy)?;
        if compare_reals(start, end, policy) == Some(Ordering::Equal) {
            let point = self.point_at(start.clone());
            return Ok(CubicBezier2::new(
                point.clone(),
                point.clone(),
                point.clone(),
                point,
            ));
        }
        if compare_reals(start, &Real::zero(), policy) == Some(Ordering::Equal)
            && compare_reals(end, &Real::one(), policy) == Some(Ordering::Equal)
        {
            return Ok(self.clone());
        }
        if compare_reals(start, &Real::zero(), policy) == Some(Ordering::Equal) {
            let (left, _) = self.split_at_exact(end.clone());
            return Ok(left);
        }
        if compare_reals(end, &Real::one(), policy) == Some(Ordering::Equal) {
            let (_, right) = self.split_at_exact(start.clone());
            return Ok(right);
        }

        let (left, _) = self.split_at_exact(end.clone());
        let local_start = (start.clone() / end.clone())?;
        let (_, middle) = left.split_at_exact(local_start);
        Ok(middle)
    }

    /// Splits this cubic at one represented parameter.
    pub fn split_at_exact(&self, t: Real) -> (CubicBezier2, CubicBezier2) {
        let p01 = self.start().lerp(self.control1(), t.clone());
        let p12 = self.control1().lerp(self.control2(), t.clone());
        let p23 = self.control2().lerp(self.end(), t.clone());
        let p012 = p01.lerp(&p12, t.clone());
        let p123 = p12.lerp(&p23, t.clone());
        let p0123 = p012.lerp(&p123, t);
        (
            CubicBezier2::new(self.start().clone(), p01, p012, p0123.clone()),
            CubicBezier2::new(p0123, p123, p23, self.end().clone()),
        )
    }
}

impl RationalQuadraticBezier2 {
    /// Splits this conic at exact/algebraic Bezier parameters.
    pub fn split_at_parameters(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(
            parameters,
            policy,
            false,
            |parameter| {
                matches!(
                    self.point_at(parameter.clone(), policy),
                    Classification::Decided(_)
                )
            },
            |start, end| {
                Ok(BezierSubcurve2::RationalQuadratic(
                    self.subcurve_between_exact(start, end, policy)?,
                ))
            },
            |parameter| BezierAlgebraicEndpointImage2::rational_quadratic(self, parameter, policy),
            BezierSubcurve2::RationalQuadratic(self.clone()),
        )
    }

    /// Materializes the exact conic subcurve over `[start, end]`.
    pub fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurvePolicy,
    ) -> CurveResult<RationalQuadraticBezier2> {
        validate_exact_range(start, end, policy)?;
        if compare_reals(start, end, policy) == Some(Ordering::Equal) {
            let point = match self.point_at(start.clone(), policy) {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Err(CurveError::Topology(format!(
                        "rational Bezier endpoint evaluation uncertain: {reason:?}"
                    )));
                }
            };
            return RationalQuadraticBezier2::try_new(
                point.clone(),
                point.clone(),
                point,
                Real::one(),
                Real::one(),
                Real::one(),
            );
        }
        if compare_reals(start, &Real::zero(), policy) == Some(Ordering::Equal)
            && compare_reals(end, &Real::one(), policy) == Some(Ordering::Equal)
        {
            return Ok(self.clone());
        }
        if compare_reals(start, &Real::zero(), policy) == Some(Ordering::Equal) {
            let (left, _) = self.split_at_exact(end.clone(), policy)?;
            return Ok(left);
        }
        if compare_reals(end, &Real::one(), policy) == Some(Ordering::Equal) {
            let (_, right) = self.split_at_exact(start.clone(), policy)?;
            return Ok(right);
        }

        let (left, _) = self.split_at_exact(end.clone(), policy)?;
        let local_start = (start.clone() / end.clone())?;
        let (_, middle) = left.split_at_exact(local_start, policy)?;
        Ok(middle)
    }

    /// Splits this rational quadratic at one represented parameter.
    pub fn split_at_exact(
        &self,
        t: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<(RationalQuadraticBezier2, RationalQuadraticBezier2)> {
        let retained_common_weight_sign = if in_closed_unit_interval(&t, policy) == Some(true) {
            self.common_nonzero_weight_sign(policy)
        } else {
            None
        };
        let controls = self.control_points();
        let weights = self.weights();
        let levels = homogeneous_de_casteljau_levels(&controls, &weights, t);
        let left = levels
            .iter()
            .map(|level| level[0].clone())
            .collect::<Vec<_>>();
        let right = levels
            .iter()
            .rev()
            .map(|level| level[level.len() - 1].clone())
            .collect::<Vec<_>>();
        Ok((
            rational_from_homogeneous(&left, policy, retained_common_weight_sign)?,
            rational_from_homogeneous(&right, policy, retained_common_weight_sign)?,
        ))
    }
}

impl RationalBezier2 {
    /// Splits this rational Bezier at exact/algebraic Bezier parameters.
    ///
    /// Represented parameters materialize exact homogeneous subcurves.
    /// Nonlinear algebraic boundaries retain exact point and tangent images;
    /// represented boundaries materialize native homogeneous subcurves.
    pub fn split_at_parameters(
        &self,
        parameters: &[BezierParameter2],
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierSplitMaterialization2>> {
        split_curve_at_parameters(
            parameters,
            policy,
            false,
            |parameter| {
                matches!(
                    self.point_at_classified(parameter, policy),
                    Classification::Decided(_)
                )
            },
            |start, end| match self.subcurve_between_exact(start, end, policy)? {
                Classification::Decided(curve) => Ok(BezierSubcurve2::Rational(curve)),
                Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
                    "general rational Bezier exact split is uncertified: {reason:?}"
                ))),
            },
            |parameter| BezierAlgebraicEndpointImage2::rational(self, parameter, policy),
            BezierSubcurve2::Rational(self.clone()),
        )
    }
}

fn split_curve_at_parameters<F, G, H>(
    parameters: &[BezierParameter2],
    policy: &CurvePolicy,
    refine_ordering: bool,
    mut exact_boundary_is_regular: H,
    mut materialize: F,
    mut endpoint_image: G,
    source_curve: BezierSubcurve2,
) -> CurveResult<Classification<BezierSplitMaterialization2>>
where
    F: FnMut(&Real, &Real) -> CurveResult<BezierSubcurve2>,
    G: FnMut(&BezierAlgebraicParameter2) -> CurveResult<BezierAlgebraicEndpointImage2>,
    H: FnMut(&Real) -> bool,
{
    let mut boundaries = vec![
        BezierParameter2::Exact(Real::zero()),
        BezierParameter2::Exact(Real::one()),
    ];
    for parameter in parameters {
        validate_parameter(parameter, policy)?;
        let promoted = match parameter
            .clone()
            .promote_represented_rational_root(policy)?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let parameter = match promoted.as_exact() {
            Some(exact) if !parameter.is_exact() && !exact_boundary_is_regular(exact) => {
                parameter.clone()
            }
            _ => promoted,
        };
        push_boundary(&mut boundaries, parameter, policy, refine_ordering)?;
    }
    match sort_boundaries(&mut boundaries, policy, refine_ordering)? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    let mut fragments = Vec::with_capacity(boundaries.len().saturating_sub(1));
    for pair in boundaries.windows(2) {
        let start = pair[0].clone();
        let end = pair[1].clone();
        match (start.as_exact(), end.as_exact()) {
            (Some(start_exact), Some(end_exact)) => {
                let curve = materialize(start_exact, end_exact)?;
                fragments.push(BezierSplitFragment2::Materialized { start, end, curve });
            }
            _ => {
                let start_image = endpoint_image_for(&start, &mut endpoint_image)?;
                let end_image = endpoint_image_for(&end, &mut endpoint_image)?;
                if start_image
                    .as_ref()
                    .is_none_or(BezierAlgebraicEndpointImage2::is_transformed)
                    && end_image
                        .as_ref()
                        .is_none_or(BezierAlgebraicEndpointImage2::is_transformed)
                {
                    fragments.push(BezierSplitFragment2::AlgebraicEndpointImages {
                        reversed: false,
                        start,
                        end,
                        source_curve: Some(source_curve.clone()),
                        start_image,
                        end_image,
                    });
                } else {
                    fragments.push(BezierSplitFragment2::Unresolved { start, end });
                }
            }
        }
    }

    Ok(Classification::Decided(BezierSplitMaterialization2::new(
        fragments,
    )?))
}

fn endpoint_image_for<G>(
    parameter: &BezierParameter2,
    endpoint_image: &mut G,
) -> CurveResult<Option<BezierAlgebraicEndpointImage2>>
where
    G: FnMut(&BezierAlgebraicParameter2) -> CurveResult<BezierAlgebraicEndpointImage2>,
{
    match parameter {
        BezierParameter2::Exact(_) => Ok(None),
        BezierParameter2::Algebraic(parameter) => Ok(Some(endpoint_image(parameter)?)),
    }
}

fn validate_parameter(parameter: &BezierParameter2, policy: &CurvePolicy) -> CurveResult<()> {
    match parameter.known_interval(policy)? {
        Classification::Decided(_) => Ok(()),
        Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
            "Bezier split parameter interval uncertain: {reason:?}"
        ))),
    }
}

fn push_boundary(
    boundaries: &mut Vec<BezierParameter2>,
    candidate: BezierParameter2,
    policy: &CurvePolicy,
    refine_ordering: bool,
) -> CurveResult<()> {
    for existing in boundaries.iter() {
        if let Classification::Decided(Ordering::Equal) =
            compare_boundary_parameters(&candidate, existing, policy, refine_ordering)?
        {
            return Ok(());
        }
    }
    boundaries.push(candidate);
    Ok(())
}

fn sort_boundaries(
    boundaries: &mut [BezierParameter2],
    policy: &CurvePolicy,
    refine_ordering: bool,
) -> CurveResult<Classification<()>> {
    for index in 1..boundaries.len() {
        let mut cursor = index;
        while cursor > 0 {
            match compare_boundary_parameters(
                &boundaries[cursor],
                &boundaries[cursor - 1],
                policy,
                refine_ordering,
            )? {
                Classification::Decided(Ordering::Less) => {
                    boundaries.swap(cursor, cursor - 1);
                    cursor -= 1;
                }
                Classification::Decided(Ordering::Equal | Ordering::Greater) => break,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
    }
    Ok(Classification::Decided(()))
}

fn compare_boundary_parameters(
    first: &BezierParameter2,
    second: &BezierParameter2,
    policy: &CurvePolicy,
    refine_ordering: bool,
) -> CurveResult<Classification<Ordering>> {
    if refine_ordering {
        first.cmp_by_refinement(second, policy)
    } else {
        first.cmp_by_interval(second, policy)
    }
}

fn validate_exact_range(start: &Real, end: &Real, policy: &CurvePolicy) -> CurveResult<()> {
    match (
        in_closed_unit_interval(start, policy),
        in_closed_unit_interval(end, policy),
    ) {
        (Some(true), Some(true)) => {}
        (Some(false), _) | (_, Some(false)) => return Err(CurveError::InvalidBezierParameter),
        _ => {
            return Err(CurveError::Topology(
                "Bezier exact split range endpoint ordering is uncertain".to_string(),
            ));
        }
    }
    match compare_reals(start, end, policy) {
        Some(Ordering::Greater) => Err(CurveError::InvalidBezierRange),
        Some(_) => Ok(()),
        None => Err(CurveError::Topology(
            "Bezier exact split range order is uncertain".to_string(),
        )),
    }
}

#[derive(Clone, Debug)]
struct HomogeneousControl {
    x: Real,
    y: Real,
    weight: Real,
}

fn homogeneous_de_casteljau_levels(
    controls: &[&Point2; 3],
    weights: &[&Real; 3],
    t: Real,
) -> Vec<Vec<HomogeneousControl>> {
    let mut levels = vec![
        controls
            .iter()
            .zip(weights.iter())
            .map(|(point, weight)| HomogeneousControl {
                x: point.x() * *weight,
                y: point.y() * *weight,
                weight: (*weight).clone(),
            })
            .collect::<Vec<_>>(),
    ];

    while levels.last().map(|level| level.len()).unwrap_or(0) > 1 {
        let previous = levels.last().expect("level exists");
        let next = previous
            .windows(2)
            .map(|pair| lerp_homogeneous(&pair[0], &pair[1], t.clone()))
            .collect::<Vec<_>>();
        levels.push(next);
    }

    levels
}

fn lerp_homogeneous(
    first: &HomogeneousControl,
    second: &HomogeneousControl,
    t: Real,
) -> HomogeneousControl {
    let one_minus_t = Real::one() - &t;
    HomogeneousControl {
        x: (&first.x * &one_minus_t) + (&second.x * &t),
        y: (&first.y * &one_minus_t) + (&second.y * &t),
        weight: (&first.weight * &one_minus_t) + (&second.weight * &t),
    }
}

fn rational_from_homogeneous(
    controls: &[HomogeneousControl],
    policy: &CurvePolicy,
    retained_common_weight_sign: Option<RealSign>,
) -> CurveResult<RationalQuadraticBezier2> {
    let mut points = Vec::with_capacity(controls.len());
    let mut weights = Vec::with_capacity(controls.len());
    for control in controls {
        match is_zero(&control.weight, policy) {
            Some(true) => return Err(CurveError::ZeroRationalBezierWeight),
            Some(false) => {}
            None => {
                return Err(CurveError::Real(
                    "rational split weight sign uncertain".into(),
                ));
            }
        }
        let x = (&control.x / &control.weight)?;
        let y = (&control.y / &control.weight)?;
        points.push(Point2::new(x, y));
        weights.push(control.weight.clone());
    }

    RationalQuadraticBezier2::try_new_with_common_weight_sign(
        points[0].clone(),
        points[1].clone(),
        points[2].clone(),
        weights[0].clone(),
        weights[1].clone(),
        weights[2].clone(),
        retained_common_weight_sign,
    )
}
