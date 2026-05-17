//! Exactness-aware topology helpers for polynomial Bezier segments.
//!
//! This module keeps Bezier topology predicates separate from the object
//! carriers in `bezier.rs`. The split follows Yap's exact geometric
//! computation model: preserve exact curve structure, then expose certified
//! predicates and explicit uncertainty at the branch boundary; see Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997).

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};

use crate::classify::{
    classify_oriented_line, compare_reals, in_closed_unit_interval, is_zero, orient2d_real_expr,
    real_sign,
};
use crate::{
    Aabb2, Classification, CubicBezier2, CurveError, CurvePolicy, LineLineIntersection, LineSeg2,
    LineSide, Point2, QuadraticBezier2, UncertaintyReason,
};

/// Coordinate axis used by Bezier monotonicity and bounds predicates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Axis2 {
    /// The x coordinate.
    X,
    /// The y coordinate.
    Y,
}

/// Closed parameter span on which a Bezier has no certified interior extremum.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierMonotoneSpan {
    start: Real,
    end: Real,
}

/// Pair of parameter spans that covers one possible curve/curve intersection region.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierCurveIntersectionRegion {
    first: BezierMonotoneSpan,
    second: BezierMonotoneSpan,
}

/// Certified geometric intersection point between two Bezier segments.
///
/// This point is emitted only after exact predicates prove that a candidate
/// from a lower-dimensional solve lies on both finite curve images. For the
/// current line-image dispatch that means a supporting-line root on the curved
/// Bezier plus exact containment on the certified endpoint line segment. This
/// follows Yap's exact geometric-computation boundary: algebraic candidates
/// are retained as exact objects and only promoted after certified predicates;
/// see Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997). The supporting-line Bezier root solve uses the Bezier
/// clipping/sign-variation idea of Sederberg and Nishita, "Curve intersection
/// using Bezier clipping" (1990).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierCurveIntersectionPoint {
    point: Point2,
}

impl BezierCurveIntersectionPoint {
    /// Constructs a certified Bezier curve/curve intersection point.
    pub const fn new(point: Point2) -> Self {
        Self { point }
    }

    /// Returns the exact point shared by both curve images.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }
}

impl BezierCurveIntersectionRegion {
    /// Constructs a paired curve/curve intersection region.
    pub const fn new(first: BezierMonotoneSpan, second: BezierMonotoneSpan) -> Self {
        Self { first, second }
    }

    /// Returns the parameter span on the first curve.
    pub const fn first(&self) -> &BezierMonotoneSpan {
        &self.first
    }

    /// Returns the parameter span on the second curve.
    pub const fn second(&self) -> &BezierMonotoneSpan {
        &self.second
    }
}

impl BezierMonotoneSpan {
    /// Constructs a closed monotone parameter span.
    pub const fn new(start: Real, end: Real) -> Self {
        Self { start, end }
    }

    /// Returns the start parameter.
    pub const fn start(&self) -> &Real {
        &self.start
    }

    /// Returns the end parameter.
    pub const fn end(&self) -> &Real {
        &self.end
    }
}

/// Certified relation between a Bezier segment and an infinite supporting line.
#[derive(Clone, Debug, PartialEq)]
pub enum BezierLineRelation {
    /// The Bezier control hull is certified to lie strictly on one side.
    ControlHullDisjoint {
        /// The side containing the control hull.
        side: LineSide,
    },
    /// Every Bezier control point is certified on the supporting line.
    OnSupportingLine,
    /// Certified parameter values where the Bezier intersects the line.
    Intersects {
        /// Sorted unique parameters in the closed unit interval.
        parameters: Vec<Real>,
    },
    /// Certified isolating spans for roots that are not represented as exact parameters.
    IsolatedIntersections {
        /// Sorted closed parameter spans, each retaining at least one line root.
        spans: Vec<BezierMonotoneSpan>,
    },
    /// The relation needs a higher-degree root or overlap solver.
    Unresolved,
}

/// Coarse certified relation between two polynomial Bezier segments.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum BezierCurveRelation {
    /// Certified tight polynomial boxes are disjoint.
    BoundingBoxesDisjoint,
    /// Exact predicates certified that the finite curve images do not meet.
    NoIntersection,
    /// The two curves have identical control polygons.
    SameControlPolygon,
    /// The two polynomial curves have the same image after degree normalization.
    ///
    /// This is narrower than arbitrary curve overlap: it currently certifies
    /// exact quadratic/cubic polynomial identity by elevating lower-degree
    /// Bernstein controls and comparing the resulting coordinate polynomials.
    /// The degree-elevation identity is the standard Bernstein basis relation
    /// in Farin, *Curves and Surfaces for CAGD* (5th ed., 2002); per Yap's
    /// exact geometric computation model, it is exposed only after exact
    /// coordinate comparisons decide equality.
    SameCurveImage,
    /// At least one endpoint is certified to be shared.
    SharedEndpoint,
    /// Both Bezier curves were certified as exact line-segment images.
    LineSegmentIntersection {
        /// Exact native line-line intersection result.
        intersection: LineLineIntersection,
    },
    /// Exact intersection points certified by a lower-dimensional dispatch.
    IntersectionPoints {
        /// Sorted unique certified geometric points.
        points: Vec<BezierCurveIntersectionPoint>,
    },
    /// Exact endpoint-on-curve intersections certified before generic subdivision.
    ///
    /// This relation certifies that one or more endpoints of either curve lie
    /// on the other finite curve image. It is intentionally narrower than a
    /// complete curve/curve solve: additional interior/interior intersections
    /// may still require the subdivision or algebraic solver.
    EndpointIntersections {
        /// Unique certified endpoint intersection points.
        points: Vec<BezierCurveIntersectionPoint>,
    },
    /// Certified parameter regions covering all remaining possible intersections.
    IntersectionRegions {
        /// Dyadic parameter boxes retained after exact subdivision pruning.
        regions: Vec<BezierCurveIntersectionRegion>,
    },
    /// The boxes overlap and a full curve/curve root solve is required.
    Unresolved,
}

/// Certified cusp status visible to this exact predicate slice.
#[derive(Clone, Debug, PartialEq)]
pub enum BezierCuspClassification {
    /// All control points are certified coincident.
    DegeneratePoint,
    /// No cusp is certified by the implemented exact derivative checks.
    None,
    /// A cusp is certified at the listed closed-interval parameters.
    Cusps {
        /// Sorted unique cusp parameters.
        parameters: Vec<Real>,
    },
    /// The derivative relation could not be fully decided.
    Unresolved,
}

/// Certified inflection status for a polynomial Bezier segment.
#[derive(Clone, Debug, PartialEq)]
pub enum BezierInflectionClassification {
    /// Quadratics have constant curvature sign and no proper inflection.
    NotApplicable,
    /// The cubic curvature polynomial is certified nonzero on `[0, 1]`.
    None,
    /// The cubic curvature polynomial is structurally zero.
    AllCurvatureZero,
    /// Certified inflection parameters in the closed unit interval.
    Inflections {
        /// Sorted unique inflection parameters.
        parameters: Vec<Real>,
    },
    /// The curvature relation could not be fully decided.
    Unresolved,
}

impl QuadraticBezier2 {
    /// Returns derivative-root parameters that split this curve into spans
    /// monotone along `axis`.
    ///
    /// For a degree-`n` Bezier, coordinate extrema can occur only where the
    /// corresponding derivative Bezier has a zero. This is the standard
    /// derivative-control-polygon fact used for Bezier bounds; see Farin,
    /// *Curves and Surfaces for Computer-Aided Geometric Design* (5th ed.,
    /// 2002). Roots are retained as exact [`Real`] parameters and filtered by
    /// certified closed-unit-interval comparisons.
    pub fn axis_monotone_parameters(
        &self,
        axis: Axis2,
        policy: &CurvePolicy,
    ) -> Classification<Vec<Real>> {
        derivative_roots_quadratic(axis_values3(self.control_points(), axis), policy)
    }

    /// Decomposes the curve at all certified x/y derivative roots.
    pub fn monotone_spans(&self, policy: &CurvePolicy) -> Classification<Vec<BezierMonotoneSpan>> {
        monotone_spans_from_parameters(
            [
                self.axis_monotone_parameters(Axis2::X, policy),
                self.axis_monotone_parameters(Axis2::Y, policy),
            ],
            policy,
        )
    }

    /// Returns a certified Bezier bounding box from endpoints and coordinate extrema.
    pub fn certified_bounds(&self, policy: &CurvePolicy) -> Classification<Aabb2> {
        certified_bounds(self, policy)
    }

    /// Classifies the relation between this quadratic and a supporting line.
    pub fn relation_to_line(
        &self,
        line: &LineSeg2,
        policy: &CurvePolicy,
    ) -> Classification<BezierLineRelation> {
        relation_to_line(self.control_points().as_slice(), line, policy)
    }

    /// Returns all certified parameters where `point` lies on this quadratic.
    ///
    /// This is the existential point-on-curve solver for polynomial
    /// quadratics. It solves the x/y Bernstein coordinate equations as exact
    /// low-degree scalar polynomials, then re-evaluates candidate parameters
    /// before exposing them. Keeping algebraic candidates exact until a
    /// certified predicate accepts them follows Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997). The
    /// Bernstein-to-power conversion is the standard Bezier identity described
    /// by Farin, *Curves and Surfaces for Computer-Aided Geometric Design*
    /// (5th ed., 2002).
    pub fn parameters_for_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<Vec<Real>> {
        quadratic_parameters_for_point(self.control_points(), point, policy)
    }

    /// Classifies whether `point` lies anywhere on this quadratic segment.
    ///
    /// The result is decided only when the exact parameter solver can certify
    /// the complete finite-curve query. Use [`Self::parameters_for_point`] when
    /// the caller needs the retained exact parameters for downstream topology.
    pub fn contains_point(&self, point: &Point2, policy: &CurvePolicy) -> Classification<bool> {
        self.parameters_for_point(point, policy)
            .map(|parameters| !parameters.is_empty())
    }

    /// Classifies the coarse relation between two quadratics.
    pub fn relation_to_quadratic(
        &self,
        other: &QuadraticBezier2,
        policy: &CurvePolicy,
    ) -> Classification<BezierCurveRelation> {
        relation_between_curves(self, other, policy)
    }

    /// Classifies the coarse relation between this quadratic and a cubic.
    ///
    /// This uses exact endpoint equality and certified Bezier bounds before
    /// returning [`BezierCurveRelation::Unresolved`] for overlapping boxes. It
    /// keeps mixed-family curve topology behind explicit predicates, following
    /// Yap's exact geometric computation boundary between structural filters
    /// and complete root solvers.
    pub fn relation_to_cubic(
        &self,
        other: &CubicBezier2,
        policy: &CurvePolicy,
    ) -> Classification<BezierCurveRelation> {
        relation_between_curves(self, other, policy)
    }

    /// Classifies cusps visible from the exact first-derivative equations.
    pub fn cusp_classification(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<BezierCuspClassification> {
        classify_quadratic_cusp(
            axis_values3(self.control_points(), Axis2::X),
            axis_values3(self.control_points(), Axis2::Y),
            policy,
        )
    }

    /// Returns the quadratic inflection classification.
    pub fn inflection_classification(&self) -> BezierInflectionClassification {
        BezierInflectionClassification::NotApplicable
    }
}

impl CubicBezier2 {
    /// Returns derivative-root parameters that split this curve into spans
    /// monotone along `axis`.
    pub fn axis_monotone_parameters(
        &self,
        axis: Axis2,
        policy: &CurvePolicy,
    ) -> Classification<Vec<Real>> {
        derivative_roots_cubic(axis_values4(self.control_points(), axis), policy)
    }

    /// Decomposes the curve at all certified x/y derivative roots.
    pub fn monotone_spans(&self, policy: &CurvePolicy) -> Classification<Vec<BezierMonotoneSpan>> {
        monotone_spans_from_parameters(
            [
                self.axis_monotone_parameters(Axis2::X, policy),
                self.axis_monotone_parameters(Axis2::Y, policy),
            ],
            policy,
        )
    }

    /// Returns a certified Bezier bounding box from endpoints and coordinate extrema.
    pub fn certified_bounds(&self, policy: &CurvePolicy) -> Classification<Aabb2> {
        certified_bounds(self, policy)
    }

    /// Classifies the relation between this cubic and a supporting line.
    pub fn relation_to_line(
        &self,
        line: &LineSeg2,
        policy: &CurvePolicy,
    ) -> Classification<BezierLineRelation> {
        relation_to_line(self.control_points().as_slice(), line, policy)
    }

    /// Returns certified dyadic subdivision parameters where `point` lies on this cubic.
    ///
    /// This is intentionally a finite candidate probe, not the complete
    /// existential point-on-cubic solver. It tests the dyadic parameters that
    /// the subdivision relation already materializes exactly and re-evaluates
    /// the cubic before returning a parameter. That keeps the branch boundary
    /// in Yap's exact-geometric-computation sense while avoiding a premature
    /// cubic resultant solver; see Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997). The exact de Casteljau
    /// evaluation and dyadic subdivision identities follow Farin, *Curves and
    /// Surfaces for Computer-Aided Geometric Design* (5th ed., 2002).
    pub fn dyadic_parameters_for_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<Vec<Real>> {
        cubic_dyadic_parameters_for_point(self, point, policy)
    }

    /// Classifies the coarse relation between two cubics.
    pub fn relation_to_cubic(
        &self,
        other: &CubicBezier2,
        policy: &CurvePolicy,
    ) -> Classification<BezierCurveRelation> {
        relation_between_curves(self, other, policy)
    }

    /// Classifies the coarse relation between this cubic and a quadratic.
    pub fn relation_to_quadratic(
        &self,
        other: &QuadraticBezier2,
        policy: &CurvePolicy,
    ) -> Classification<BezierCurveRelation> {
        relation_between_curves(self, other, policy)
    }

    /// Classifies cusps visible from endpoint and common derivative roots.
    pub fn cusp_classification(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<BezierCuspClassification> {
        classify_cubic_cusp(
            axis_values4(self.control_points(), Axis2::X),
            axis_values4(self.control_points(), Axis2::Y),
            policy,
        )
    }

    /// Classifies cubic inflection parameters through the exact curvature polynomial.
    pub fn inflection_classification(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<BezierInflectionClassification> {
        classify_cubic_inflections(self.control_points(), policy)
    }
}

trait BezierBounds {
    fn point_at(&self, t: Real) -> Point2;
    fn endpoints(&self) -> [&Point2; 2];
    fn monotone_spans(&self, policy: &CurvePolicy) -> Classification<Vec<BezierMonotoneSpan>>;
}

impl BezierBounds for QuadraticBezier2 {
    fn point_at(&self, t: Real) -> Point2 {
        Self::point_at(self, t)
    }

    fn endpoints(&self) -> [&Point2; 2] {
        [self.start(), self.end()]
    }

    fn monotone_spans(&self, policy: &CurvePolicy) -> Classification<Vec<BezierMonotoneSpan>> {
        Self::monotone_spans(self, policy)
    }
}

impl BezierBounds for CubicBezier2 {
    fn point_at(&self, t: Real) -> Point2 {
        Self::point_at(self, t)
    }

    fn endpoints(&self) -> [&Point2; 2] {
        [self.start(), self.end()]
    }

    fn monotone_spans(&self, policy: &CurvePolicy) -> Classification<Vec<BezierMonotoneSpan>> {
        Self::monotone_spans(self, policy)
    }
}

fn certified_bounds<C>(curve: &C, policy: &CurvePolicy) -> Classification<Aabb2>
where
    C: BezierBounds,
{
    let mut samples: Vec<Point2> = curve.endpoints().into_iter().cloned().collect();
    let spans = match curve.monotone_spans(policy) {
        Classification::Decided(spans) => spans,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    for span in spans {
        if !is_unit_endpoint(span.start(), policy) {
            samples.push(curve.point_at(span.start().clone()));
        }
        if !is_unit_endpoint(span.end(), policy) {
            samples.push(curve.point_at(span.end().clone()));
        }
    }
    Aabb2::from_points(samples.iter(), policy)
}

trait BezierCurveLike {
    fn control_points_vec(&self) -> Vec<&Point2>;
    fn certified_bounds(&self, policy: &CurvePolicy) -> Classification<Aabb2>;
    fn subdivision_node(&self) -> BezierSubdivisionNode;
    fn point_at(&self, t: Real) -> Point2;
    fn exact_parameters_for_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Option<Classification<Vec<Real>>>;
}

impl BezierCurveLike for QuadraticBezier2 {
    fn control_points_vec(&self) -> Vec<&Point2> {
        self.control_points().into_iter().collect()
    }

    fn certified_bounds(&self, policy: &CurvePolicy) -> Classification<Aabb2> {
        Self::certified_bounds(self, policy)
    }

    fn subdivision_node(&self) -> BezierSubdivisionNode {
        BezierSubdivisionNode::new(self.control_points().into_iter().cloned().collect())
    }

    fn point_at(&self, t: Real) -> Point2 {
        Self::point_at(self, t)
    }

    fn exact_parameters_for_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Option<Classification<Vec<Real>>> {
        Some(quadratic_parameters_for_point(
            self.control_points(),
            point,
            policy,
        ))
    }
}

impl BezierCurveLike for CubicBezier2 {
    fn control_points_vec(&self) -> Vec<&Point2> {
        self.control_points().into_iter().collect()
    }

    fn certified_bounds(&self, policy: &CurvePolicy) -> Classification<Aabb2> {
        Self::certified_bounds(self, policy)
    }

    fn subdivision_node(&self) -> BezierSubdivisionNode {
        BezierSubdivisionNode::new(self.control_points().into_iter().cloned().collect())
    }

    fn point_at(&self, t: Real) -> Point2 {
        Self::point_at(self, t)
    }

    fn exact_parameters_for_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Option<Classification<Vec<Real>>> {
        Some(self.dyadic_parameters_for_point(point, policy))
    }
}

fn same_polynomial_image_by_degree_elevation(
    first_controls: &[&Point2],
    second_controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<bool> {
    if !matches!(
        (first_controls.len(), second_controls.len()),
        (3, 4) | (4, 3)
    ) {
        return Classification::Decided(false);
    }

    // Quadratic-to-cubic degree elevation preserves the represented Bernstein
    // polynomial: Q0, (Q0 + 2Q1)/3, (2Q1 + Q2)/3, Q2. Comparing those exact
    // elevated coordinate controls certifies polynomial-image equality without
    // sampling or tolerance. This follows the Bernstein basis identities in
    // Farin, Curves and Surfaces for CAGD, 5th ed. (2002), and keeps the
    // branch decision at an exact predicate boundary in Yap's EGC sense.
    for axis in [Axis2::X, Axis2::Y] {
        let Some(first_values) = cubic_axis_values(first_controls, axis) else {
            return Classification::Decided(false);
        };
        let Some(second_values) = cubic_axis_values(second_controls, axis) else {
            return Classification::Decided(false);
        };
        for (first, second) in first_values.iter().zip(second_values.iter()) {
            match compare_reals(first, second, policy) {
                Some(Ordering::Equal) => {}
                Some(Ordering::Less | Ordering::Greater) => {
                    return Classification::Decided(false);
                }
                None => return Classification::Uncertain(UncertaintyReason::Ordering),
            }
        }
    }
    Classification::Decided(true)
}

fn relation_between_curves<A, B>(
    first: &A,
    second: &B,
    policy: &CurvePolicy,
) -> Classification<BezierCurveRelation>
where
    A: BezierCurveLike,
    B: BezierCurveLike,
{
    let first_controls = first.control_points_vec();
    let second_controls = second.control_points_vec();
    if first_controls.len() == second_controls.len()
        && first_controls
            .iter()
            .zip(second_controls.iter())
            .all(|(a, b)| point_equal(a, b, policy) == Some(true))
    {
        return Classification::Decided(BezierCurveRelation::SameControlPolygon);
    }

    match same_polynomial_image_by_degree_elevation(&first_controls, &second_controls, policy) {
        Classification::Decided(true) => {
            return Classification::Decided(BezierCurveRelation::SameCurveImage);
        }
        Classification::Decided(false) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    let first_hull = match Aabb2::from_points(first_controls.iter().copied(), policy) {
        Classification::Decided(bbox) => bbox,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let second_hull = match Aabb2::from_points(second_controls.iter().copied(), policy) {
        Classification::Decided(bbox) => bbox,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    match first_hull.overlaps(&second_hull, policy) {
        Classification::Decided(false) => {
            return Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint);
        }
        Classification::Decided(true) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    let first_line_image = match line_segment_image_from_controls(&first_controls, policy) {
        Classification::Decided(line) => line,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let second_line_image = match line_segment_image_from_controls(&second_controls, policy) {
        Classification::Decided(line) => line,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    match (&first_line_image, &second_line_image) {
        (Some(first_line), Some(second_line)) => {
            return match first_line.intersect_line(second_line, policy) {
                Ok(intersection) => {
                    Classification::Decided(BezierCurveRelation::LineSegmentIntersection {
                        intersection,
                    })
                }
                Err(CurveError::Real(_)) => Classification::Uncertain(UncertaintyReason::RealSign),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            };
        }
        (Some(line), None) => match line_image_curve_intersections(line, second, policy) {
            Classification::Decided(Some(points)) if points.is_empty() => {
                return Classification::Decided(BezierCurveRelation::NoIntersection);
            }
            Classification::Decided(Some(points)) => {
                return Classification::Decided(BezierCurveRelation::IntersectionPoints { points });
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        },
        (None, Some(line)) => match line_image_curve_intersections(line, first, policy) {
            Classification::Decided(Some(points)) if points.is_empty() => {
                return Classification::Decided(BezierCurveRelation::NoIntersection);
            }
            Classification::Decided(Some(points)) => {
                return Classification::Decided(BezierCurveRelation::IntersectionPoints { points });
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        },
        (None, None) => {}
    }

    let first_endpoints = [first_controls[0], first_controls[first_controls.len() - 1]];
    let second_endpoints = [
        second_controls[0],
        second_controls[second_controls.len() - 1],
    ];
    for a in first_endpoints {
        for b in second_endpoints {
            match point_coordinates_equal(a, b, policy) {
                Some(true) => return Classification::Decided(BezierCurveRelation::SharedEndpoint),
                Some(false) => {}
                None => return Classification::Uncertain(UncertaintyReason::Ordering),
            }
        }
    }

    if certifies_shared_axis_control_separation(&first_controls, &second_controls, policy) {
        return Classification::Decided(BezierCurveRelation::NoIntersection);
    }

    // Endpoint-on-curve facts are lower degree than the subdivision filters
    // below: for polynomial quadratics, a point query reduces to two exact
    // scalar quadratics and a certified re-evaluation. Running this before
    // derivative-refined bounds avoids making an endpoint certificate depend
    // on unrelated cubic extrema. This is the exact-object/decidable-predicate
    // separation advocated by Yap, "Towards Exact Geometric Computation,"
    // Computational Geometry 7.1-2 (1997).
    match endpoint_intersections(first, second, &first_endpoints, &second_endpoints, policy) {
        Classification::Decided(points) if !points.is_empty() => {
            return Classification::Decided(BezierCurveRelation::EndpointIntersections { points });
        }
        Classification::Decided(_) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    match same_parameter_dyadic_intersections(
        first,
        second,
        &first_controls,
        &second_controls,
        policy,
    ) {
        Classification::Decided(Some(points)) => {
            return Classification::Decided(BezierCurveRelation::IntersectionPoints { points });
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    match same_parameter_quadratic_relation(&first_controls, &second_controls, policy) {
        Classification::Decided(Some(relation)) => return Classification::Decided(relation),
        Classification::Decided(_) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    let first_box = match first.certified_bounds(policy) {
        Classification::Decided(bbox) => bbox,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let second_box = match second.certified_bounds(policy) {
        Classification::Decided(bbox) => bbox,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    match first_box.overlaps(&second_box, policy) {
        Classification::Decided(false) => {
            return Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint);
        }
        Classification::Decided(true) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    match isolate_curve_intersection_regions(
        first.subdivision_node(),
        second.subdivision_node(),
        policy,
    ) {
        Classification::Decided(regions) if !regions.is_empty() => {
            Classification::Decided(BezierCurveRelation::IntersectionRegions { regions })
        }
        Classification::Decided(_) => Classification::Decided(BezierCurveRelation::Unresolved),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

fn endpoint_intersections<A, B>(
    first: &A,
    second: &B,
    first_endpoints: &[&Point2; 2],
    second_endpoints: &[&Point2; 2],
    policy: &CurvePolicy,
) -> Classification<Vec<BezierCurveIntersectionPoint>>
where
    A: BezierCurveLike,
    B: BezierCurveLike,
{
    let mut points = Vec::new();
    for endpoint in first_endpoints {
        match second.exact_parameters_for_point(endpoint, policy) {
            Some(Classification::Decided(parameters)) if !parameters.is_empty() => {
                push_unique_intersection_point(&mut points, (*endpoint).clone(), policy);
            }
            Some(Classification::Decided(_)) | None => {}
            Some(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        }
    }
    for endpoint in second_endpoints {
        match first.exact_parameters_for_point(endpoint, policy) {
            Some(Classification::Decided(parameters)) if !parameters.is_empty() => {
                push_unique_intersection_point(&mut points, (*endpoint).clone(), policy);
            }
            Some(Classification::Decided(_)) | None => {}
            Some(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(points)
}

fn same_parameter_dyadic_intersections<A, B>(
    first: &A,
    second: &B,
    first_controls: &[&Point2],
    second_controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<Option<Vec<BezierCurveIntersectionPoint>>>
where
    A: BezierCurveLike,
    B: BezierCurveLike,
{
    if !matches!(
        (first_controls.len(), second_controls.len()),
        (3, 4) | (4, 3) | (4, 4)
    ) {
        return Classification::Decided(None);
    }

    // This is a finite exact-candidate slice of the same-parameter algebraic
    // curve/curve problem. Degree-normalize quadratic inputs to cubic
    // Bernstein controls, keep cubic inputs native, test dyadic parameters
    // that the subdivision solver already exposes exactly, and only then emit
    // certified shared points. This promotes useful algebraic candidates while
    // remaining explicit that it is not a complete resultant solve. The
    // Bernstein/de Casteljau identities are the standard ones in Farin, Curves
    // and Surfaces for CAGD, 5th ed. (2002), and the exact candidate boundary
    // follows Yap, "Towards Exact Geometric Computation," Computational
    // Geometry 7.1-2 (1997).
    let mut points = Vec::new();
    for parameter in dyadic_subdivision_candidate_parameters() {
        let x_equal = match cubic_difference_zero_at_parameter(
            first_controls,
            second_controls,
            Axis2::X,
            parameter.clone(),
            policy,
        ) {
            Some(equal) => equal,
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        };
        if !x_equal {
            continue;
        }
        let y_equal = match cubic_difference_zero_at_parameter(
            first_controls,
            second_controls,
            Axis2::Y,
            parameter.clone(),
            policy,
        ) {
            Some(equal) => equal,
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        };
        if !y_equal {
            continue;
        }

        let first_point = first.point_at(parameter.clone());
        let second_point = second.point_at(parameter);
        match point_equal(&first_point, &second_point, policy) {
            Some(true) => push_unique_intersection_point(&mut points, first_point, policy),
            Some(false) => {}
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }

    if points.is_empty() {
        Classification::Decided(None)
    } else {
        Classification::Decided(Some(points))
    }
}

fn cubic_dyadic_parameters_for_point(
    curve: &CubicBezier2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Classification<Vec<Real>> {
    let mut parameters = Vec::new();
    for parameter in dyadic_subdivision_candidate_parameters() {
        match point_coordinates_equal(&curve.point_at(parameter.clone()), point, policy) {
            Some(true) => push_unique_sorted(&mut parameters, parameter, policy),
            Some(false) => {}
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }
    Classification::Decided(parameters)
}

fn dyadic_subdivision_candidate_parameters() -> [Real; 3] {
    let quarter =
        (Real::one() / Real::from(4_i8)).expect("division by positive integer constant is defined");
    let half =
        (Real::one() / Real::from(2_i8)).expect("division by positive integer constant is defined");
    let three_quarters = (&quarter * &Real::from(3_i8)).clone();
    [quarter, half, three_quarters]
}

fn cubic_difference_zero_at_parameter(
    first_controls: &[&Point2],
    second_controls: &[&Point2],
    axis: Axis2,
    parameter: Real,
    policy: &CurvePolicy,
) -> Option<bool> {
    let first_values = cubic_axis_values(first_controls, axis)?;
    let second_values = cubic_axis_values(second_controls, axis)?;
    let difference = [
        &first_values[0] - &second_values[0],
        &first_values[1] - &second_values[1],
        &first_values[2] - &second_values[2],
        &first_values[3] - &second_values[3],
    ];
    is_zero(&scalar_cubic_at_parameter(&difference, parameter), policy)
}

fn same_parameter_quadratic_relation(
    first_controls: &[&Point2],
    second_controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<Option<BezierCurveRelation>> {
    if first_controls.len() != 3 || second_controls.len() != 3 {
        return Classification::Decided(None);
    }

    // This is a deliberately narrow algebraic curve/curve slice: when both
    // polynomial quadratics are evaluated at the same parameter, intersections
    // are roots of the vector-valued quadratic difference. We keep the
    // Bernstein difference as an exact object, solve its coordinate quadratics,
    // and re-evaluate both curves before emitting points. The result is a
    // certified shortcut, not a declaration that unrelated-parameter
    // intersections are absent. The one complete subcase below is when both
    // curves share an exact coordinate Bezier and that coordinate is strictly
    // monotone; then an image intersection must have the same parameter. This
    // follows Yap's exact-object predicate boundary and Farin's Bernstein
    // derivative identities; see Yap, "Towards Exact Geometric Computation,"
    // Computational Geometry 7.1-2 (1997), and Farin, Curves and Surfaces for
    // CAGD, 5th ed. (2002).
    if certifies_shared_axis_control_separation(first_controls, second_controls, policy) {
        return Classification::Decided(Some(BezierCurveRelation::NoIntersection));
    }

    let x_roots = match quadratic_axis_point_root_set(
        [
            first_controls[0].x() - second_controls[0].x(),
            first_controls[1].x() - second_controls[1].x(),
            first_controls[2].x() - second_controls[2].x(),
        ],
        policy,
    ) {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let y_roots = match quadratic_axis_point_root_set(
        [
            first_controls[0].y() - second_controls[0].y(),
            first_controls[1].y() - second_controls[1].y(),
            first_controls[2].y() - second_controls[2].y(),
        ],
        policy,
    ) {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };

    let candidates = match (&x_roots, &y_roots) {
        (RootSet::All, RootSet::All) => return Classification::Decided(None),
        (RootSet::All, RootSet::Roots(roots)) | (RootSet::Roots(roots), RootSet::All) => {
            roots.clone()
        }
        (RootSet::Roots(left), RootSet::Roots(right)) => {
            let Some(common) = common_parameters(left, right, policy) else {
                return Classification::Uncertain(UncertaintyReason::Ordering);
            };
            common
        }
    };

    let first = [first_controls[0], first_controls[1], first_controls[2]];
    let mut points = Vec::new();
    for candidate in candidates {
        let first_point = quadratic_point_at_controls(first, candidate);
        push_unique_intersection_point(&mut points, first_point, policy);
    }

    if !points.is_empty() {
        return Classification::Decided(Some(BezierCurveRelation::IntersectionPoints { points }));
    }

    match has_shared_strictly_monotone_axis(first_controls, second_controls, policy) {
        Classification::Decided(true) => {
            Classification::Decided(Some(BezierCurveRelation::NoIntersection))
        }
        Classification::Decided(false) => Classification::Decided(None),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

fn has_shared_strictly_monotone_axis(
    first_controls: &[&Point2],
    second_controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<bool> {
    for axis in [Axis2::X, Axis2::Y] {
        match shared_strictly_monotone_axis(first_controls, second_controls, axis, policy) {
            Classification::Decided(true) => return Classification::Decided(true),
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(false)
}

fn certifies_shared_axis_control_separation(
    first_controls: &[&Point2],
    second_controls: &[&Point2],
    policy: &CurvePolicy,
) -> bool {
    // Once a shared coordinate is proven injective, any geometric hit must
    // occur at a common parameter. A same-sign Bernstein control polygon for
    // the remaining coordinate difference then excludes zero by the convex-hull
    // property. This is the Bezier clipping sign-variation idea of Sederberg
    // and Nishita, "Curve intersection using Bezier clipping" (1990), guarded
    // by exact signs as required by Yap's EGC model.
    [Axis2::X, Axis2::Y].into_iter().any(|axis| {
        let Classification::Decided(true) =
            shared_strictly_monotone_axis(first_controls, second_controls, axis, policy)
        else {
            return false;
        };
        let other_axis = match axis {
            Axis2::X => Axis2::Y,
            Axis2::Y => Axis2::X,
        };
        control_differences_have_common_strict_sign(
            first_controls,
            second_controls,
            other_axis,
            policy,
        )
    })
}

fn control_differences_have_common_strict_sign(
    first_controls: &[&Point2],
    second_controls: &[&Point2],
    axis: Axis2,
    policy: &CurvePolicy,
) -> bool {
    let Some(first_values) = cubic_axis_values(first_controls, axis) else {
        return false;
    };
    let Some(second_values) = cubic_axis_values(second_controls, axis) else {
        return false;
    };
    let mut common_sign = None;
    for (first, second) in first_values.iter().zip(second_values.iter()) {
        let difference = first - second;
        let Some(sign) = real_sign(&difference, policy) else {
            return false;
        };
        match (common_sign, sign) {
            (_, RealSign::Zero) => return false,
            (None, RealSign::Positive | RealSign::Negative) => common_sign = Some(sign),
            (Some(previous), RealSign::Positive | RealSign::Negative) if previous == sign => {}
            (Some(_), RealSign::Positive | RealSign::Negative) => return false,
        }
    }
    common_sign.is_some()
}

fn shared_strictly_monotone_axis(
    first_controls: &[&Point2],
    second_controls: &[&Point2],
    axis: Axis2,
    policy: &CurvePolicy,
) -> Classification<bool> {
    let Some(first_values) = cubic_axis_values(first_controls, axis) else {
        return Classification::Decided(false);
    };
    let Some(second_values) = cubic_axis_values(second_controls, axis) else {
        return Classification::Decided(false);
    };

    for (first, second) in first_values.iter().zip(second_values.iter()) {
        match compare_reals(first, second, policy) {
            Some(Ordering::Equal) => {}
            Some(Ordering::Less | Ordering::Greater) => return Classification::Decided(false),
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }

    // Degree-normalizing quadratics to cubics lets mixed quadratic/cubic graph
    // proofs compare the same polynomial before checking monotonicity. For a
    // cubic Bezier coordinate b(t), b'(t) is the quadratic Bezier with controls
    // 3(P1-P0), 3(P2-P1), and 3(P3-P2). If all endpoint differences have the
    // same strict sign, every convex combination has that sign, so the shared
    // coordinate is injective on [0, 1]. This is the Bernstein derivative
    // criterion from Farin, Curves and Surfaces for CAGD, 5th ed. (2002),
    // used here only after exact sign predicates per Yap's EGC model.
    let mut common_sign = None;
    for pair in first_values.windows(2) {
        let difference = &pair[1] - &pair[0];
        let Some(sign) = real_sign(&difference, policy) else {
            return Classification::Uncertain(UncertaintyReason::RealSign);
        };
        match (common_sign, sign) {
            (_, RealSign::Zero) => return Classification::Decided(false),
            (None, RealSign::Positive | RealSign::Negative) => common_sign = Some(sign),
            (Some(previous), RealSign::Positive | RealSign::Negative) if previous == sign => {}
            (Some(_), RealSign::Positive | RealSign::Negative) => {
                return Classification::Decided(false);
            }
        }
    }
    Classification::Decided(common_sign.is_some())
}

fn cubic_axis_values(points: &[&Point2], axis: Axis2) -> Option<[Real; 4]> {
    match points.len() {
        3 => {
            let p0 = coordinate(points[0], axis);
            let p1 = coordinate(points[1], axis);
            let p2 = coordinate(points[2], axis);
            let three = Real::from(3_i8);
            Some([
                p0.clone(),
                ((p0 + &(&Real::from(2_i8) * p1)) / three.clone())
                    .expect("division by positive integer constant is defined"),
                (((&Real::from(2_i8) * p1) + p2) / three)
                    .expect("division by positive integer constant is defined"),
                p2.clone(),
            ])
        }
        4 => Some([
            coordinate(points[0], axis).clone(),
            coordinate(points[1], axis).clone(),
            coordinate(points[2], axis).clone(),
            coordinate(points[3], axis).clone(),
        ]),
        _ => None,
    }
}

fn line_image_curve_intersections<C>(
    line: &LineSeg2,
    curve: &C,
    policy: &CurvePolicy,
) -> Classification<Option<Vec<BezierCurveIntersectionPoint>>>
where
    C: BezierCurveLike,
{
    let controls = curve.control_points_vec();
    match relation_to_line(&controls, line, policy) {
        Classification::Decided(BezierLineRelation::ControlHullDisjoint { .. }) => {
            Classification::Decided(Some(Vec::new()))
        }
        Classification::Decided(BezierLineRelation::Intersects { parameters }) => {
            let mut points = Vec::new();
            for parameter in parameters {
                let point = curve.point_at(parameter);
                match line.contains_point(&point, policy) {
                    Classification::Decided(true) => {
                        push_unique_intersection_point(&mut points, point, policy);
                    }
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                }
            }
            Classification::Decided(Some(points))
        }
        Classification::Decided(
            BezierLineRelation::OnSupportingLine
            | BezierLineRelation::IsolatedIntersections { .. }
            | BezierLineRelation::Unresolved,
        ) => Classification::Decided(None),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

fn push_unique_intersection_point(
    points: &mut Vec<BezierCurveIntersectionPoint>,
    point: Point2,
    policy: &CurvePolicy,
) {
    if points
        .iter()
        .any(|existing| point_equal(existing.point(), &point, policy) == Some(true))
    {
        return;
    }
    points.push(BezierCurveIntersectionPoint::new(point));
}

#[derive(Clone, Debug)]
struct BezierSubdivisionNode {
    controls: Vec<Point2>,
    span: BezierMonotoneSpan,
}

impl BezierSubdivisionNode {
    fn new(controls: Vec<Point2>) -> Self {
        Self {
            controls,
            span: BezierMonotoneSpan::new(Real::zero(), Real::one()),
        }
    }

    fn with_span(controls: Vec<Point2>, start: Real, end: Real) -> Self {
        Self {
            controls,
            span: BezierMonotoneSpan::new(start, end),
        }
    }

    fn control_box(&self, policy: &CurvePolicy) -> Classification<Aabb2> {
        Aabb2::from_points(self.controls.iter(), policy)
    }

    fn split_half(&self) -> Result<(Self, Self), UncertaintyReason> {
        let (left_controls, right_controls) = subdivide_points_half(&self.controls);
        let mid = ((self.span.start() + self.span.end()) / Real::from(2_i8))
            .map_err(|_| UncertaintyReason::Unsupported)?;
        Ok((
            Self::with_span(left_controls, self.span.start().clone(), mid.clone()),
            Self::with_span(right_controls, mid, self.span.end().clone()),
        ))
    }
}

fn isolate_curve_intersection_regions(
    first: BezierSubdivisionNode,
    second: BezierSubdivisionNode,
    policy: &CurvePolicy,
) -> Classification<Vec<BezierCurveIntersectionRegion>> {
    let mut regions = Vec::new();
    if let Err(reason) =
        isolate_curve_intersection_regions_recursive(first, second, 0, policy, &mut regions)
    {
        return Classification::Uncertain(reason);
    }
    Classification::Decided(regions)
}

fn isolate_curve_intersection_regions_recursive(
    first: BezierSubdivisionNode,
    second: BezierSubdivisionNode,
    depth: usize,
    policy: &CurvePolicy,
    regions: &mut Vec<BezierCurveIntersectionRegion>,
) -> Result<(), UncertaintyReason> {
    // This is the subdivision half of Bezier clipping: exact control-hull boxes
    // that are disjoint certify absence of curve intersections in that
    // parameter product cell. Remaining cells are recursively bisected and kept
    // as dyadic parameter regions. Sederberg and Nishita, "Curve intersection
    // using Bezier clipping" (1990), use this convex-hull exclusion principle;
    // per Yap (1997), this implementation returns bounded regions rather than
    // choosing topology from floating tolerances.
    let first_box = match first.control_box(policy) {
        Classification::Decided(bbox) => bbox,
        Classification::Uncertain(reason) => return Err(reason),
    };
    let second_box = match second.control_box(policy) {
        Classification::Decided(bbox) => bbox,
        Classification::Uncertain(reason) => return Err(reason),
    };
    match first_box.overlaps(&second_box, policy) {
        Classification::Decided(false) => return Ok(()),
        Classification::Decided(true) => {}
        Classification::Uncertain(reason) => return Err(reason),
    }

    if depth >= 24 {
        push_unique_curve_region(
            regions,
            BezierCurveIntersectionRegion::new(first.span, second.span),
            policy,
        )?;
        return Ok(());
    }

    let first_width = span_width(first.span.start(), first.span.end(), policy)?;
    let second_width = span_width(second.span.start(), second.span.end(), policy)?;
    match compare_reals(&first_width, &second_width, policy) {
        Some(Ordering::Greater | Ordering::Equal) => {
            let (left, right) = first.split_half()?;
            isolate_curve_intersection_regions_recursive(
                left,
                second.clone(),
                depth + 1,
                policy,
                regions,
            )?;
            isolate_curve_intersection_regions_recursive(right, second, depth + 1, policy, regions)
        }
        Some(Ordering::Less) => {
            let (left, right) = second.split_half()?;
            isolate_curve_intersection_regions_recursive(
                first.clone(),
                left,
                depth + 1,
                policy,
                regions,
            )?;
            isolate_curve_intersection_regions_recursive(first, right, depth + 1, policy, regions)
        }
        None => Err(UncertaintyReason::Ordering),
    }
}

fn subdivide_points_half(points: &[Point2]) -> (Vec<Point2>, Vec<Point2>) {
    let mut levels = vec![points.to_vec()];
    while levels.last().map(|level| level.len()).unwrap_or(0) > 1 {
        let previous = levels.last().expect("level exists");
        let next = previous
            .windows(2)
            .map(|pair| midpoint_point(&pair[0], &pair[1]))
            .collect::<Vec<_>>();
        levels.push(next);
    }

    let left = levels
        .iter()
        .map(|level| level[0].clone())
        .collect::<Vec<_>>();
    let right = levels
        .iter()
        .rev()
        .map(|level| level[level.len() - 1].clone())
        .collect::<Vec<_>>();
    (left, right)
}

fn midpoint_point(first: &Point2, second: &Point2) -> Point2 {
    let half =
        (Real::one() / Real::from(2_i8)).expect("division by positive integer constant is defined");
    first.lerp(second, half)
}

fn span_width(start: &Real, end: &Real, policy: &CurvePolicy) -> Result<Real, UncertaintyReason> {
    match compare_reals(end, start, policy) {
        Some(Ordering::Greater | Ordering::Equal) => Ok(end - start),
        Some(Ordering::Less) => Ok(start - end),
        None => Err(UncertaintyReason::Ordering),
    }
}

fn push_unique_curve_region(
    regions: &mut Vec<BezierCurveIntersectionRegion>,
    region: BezierCurveIntersectionRegion,
    policy: &CurvePolicy,
) -> Result<(), UncertaintyReason> {
    let duplicate = regions.iter().any(|existing| {
        spans_equal(existing.first(), region.first(), policy)
            && spans_equal(existing.second(), region.second(), policy)
    });
    if !duplicate {
        regions.push(region);
    }
    Ok(())
}

fn spans_equal(
    first: &BezierMonotoneSpan,
    second: &BezierMonotoneSpan,
    policy: &CurvePolicy,
) -> bool {
    compare_reals(first.start(), second.start(), policy) == Some(Ordering::Equal)
        && compare_reals(first.end(), second.end(), policy) == Some(Ordering::Equal)
}

fn relation_to_line(
    controls: &[&Point2],
    line: &LineSeg2,
    policy: &CurvePolicy,
) -> Classification<BezierLineRelation> {
    let sides = controls
        .iter()
        .map(|point| classify_oriented_line(line.start(), line.end(), point, policy))
        .collect::<Vec<_>>();
    if let Some(reason) = sides.iter().find_map(|side| match side {
        Classification::Uncertain(reason) => Some(*reason),
        Classification::Decided(_) => None,
    }) {
        return Classification::Uncertain(reason);
    }

    let decided_sides = sides
        .into_iter()
        .map(|side| match side {
            Classification::Decided(side) => side,
            Classification::Uncertain(_) => unreachable!(),
        })
        .collect::<Vec<_>>();

    if decided_sides.iter().all(|side| *side == LineSide::On) {
        return Classification::Decided(BezierLineRelation::OnSupportingLine);
    }
    if decided_sides
        .iter()
        .all(|side| matches!(side, LineSide::Left))
    {
        return Classification::Decided(BezierLineRelation::ControlHullDisjoint {
            side: LineSide::Left,
        });
    }
    if decided_sides
        .iter()
        .all(|side| matches!(side, LineSide::Right))
    {
        return Classification::Decided(BezierLineRelation::ControlHullDisjoint {
            side: LineSide::Right,
        });
    }

    let distances = controls
        .iter()
        .map(|point| orient2d_real_expr(line.start(), line.end(), point))
        .collect::<Vec<_>>();
    let roots = match distances.as_slice() {
        [d0, d1, d2] => {
            let two = Real::from(2_i8);
            let c0 = d0.clone();
            let c1 = &two * &(d1 - d0);
            let c2 = d0 - &(two * d1) + d2;
            polynomial_roots_in_unit_interval(c0, c1, c2, policy)
        }
        [d0, d1, d2, d3] => {
            if [d0, d1, d2, d3]
                .iter()
                .all(|value| is_zero(value, policy) == Some(true))
            {
                return Classification::Decided(BezierLineRelation::OnSupportingLine);
            }
            return isolate_cubic_line_roots(
                [d0.clone(), d1.clone(), d2.clone(), d3.clone()],
                policy,
            );
        }
        _ => Classification::Uncertain(UncertaintyReason::Unsupported),
    };

    roots.map(|parameters| {
        if parameters.is_empty() {
            BezierLineRelation::ControlHullDisjoint {
                side: decided_sides
                    .into_iter()
                    .find(|side| *side != LineSide::On)
                    .unwrap_or(LineSide::On),
            }
        } else {
            BezierLineRelation::Intersects { parameters }
        }
    })
}

fn quadratic_parameters_for_point(
    controls: [&Point2; 3],
    point: &Point2,
    policy: &CurvePolicy,
) -> Classification<Vec<Real>> {
    // A point lies on a polynomial quadratic Bezier exactly when the x and y
    // coordinate Bernstein equations share a parameter in `[0, 1]`. Solving
    // those low-degree equations as exact `Real` roots and intersecting the
    // parameter sets follows Yap's EGC requirement to keep algebraic candidates
    // explicit until certified. The coordinate polynomial identities are the
    // standard Bernstein-to-power conversion described by Farin, *Curves and
    // Surfaces for Computer-Aided Geometric Design* (5th ed., 2002).
    let x_roots = match quadratic_axis_point_root_set(
        [
            controls[0].x() - point.x(),
            controls[1].x() - point.x(),
            controls[2].x() - point.x(),
        ],
        policy,
    ) {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let y_roots = match quadratic_axis_point_root_set(
        [
            controls[0].y() - point.y(),
            controls[1].y() - point.y(),
            controls[2].y() - point.y(),
        ],
        policy,
    ) {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    quadratic_point_parameters_from_root_sets(controls, point, x_roots, y_roots, policy)
}

fn quadratic_axis_point_root_set(
    values: [Real; 3],
    policy: &CurvePolicy,
) -> Classification<RootSet> {
    let [p0, p1, p2] = values;
    if is_zero(&p0, policy) == Some(true)
        && is_zero(&p1, policy) == Some(true)
        && is_zero(&p2, policy) == Some(true)
    {
        return Classification::Decided(RootSet::All);
    }
    let two = Real::from(2_i8);
    let c0 = p0.clone();
    let c1 = &two * &(&p1 - &p0);
    let c2 = &p0 - &(&two * &p1) + &p2;
    polynomial_roots_in_unit_interval(c0, c1, c2, policy).map(RootSet::Roots)
}

fn quadratic_point_parameters_from_root_sets(
    controls: [&Point2; 3],
    point: &Point2,
    x_roots: RootSet,
    y_roots: RootSet,
    policy: &CurvePolicy,
) -> Classification<Vec<Real>> {
    let candidates = match (&x_roots, &y_roots) {
        (RootSet::All, RootSet::All) => vec![Real::zero()],
        (RootSet::All, RootSet::Roots(roots)) | (RootSet::Roots(roots), RootSet::All) => {
            roots.clone()
        }
        (RootSet::Roots(left), RootSet::Roots(right)) => {
            let mut candidates = left.clone();
            candidates.extend(right.iter().cloned());
            candidates
        }
    };

    let mut parameters = Vec::new();
    for candidate in candidates {
        let curve_point = quadratic_point_at_controls(controls, candidate.clone());
        match point_equal(&curve_point, point, policy) {
            Some(true) => push_unique_sorted(&mut parameters, candidate, policy),
            Some(false) => {}
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }
    Classification::Decided(parameters)
}

fn quadratic_point_at_controls(controls: [&Point2; 3], t: Real) -> Point2 {
    let left = controls[0].lerp(controls[1], t.clone());
    let right = controls[1].lerp(controls[2], t.clone());
    left.lerp(&right, t)
}

fn line_segment_image_from_controls(
    controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<Option<LineSeg2>> {
    if controls.len() < 2 {
        return Classification::Decided(None);
    }
    let start = controls[0];
    let end = controls[controls.len() - 1];
    let line = match LineSeg2::try_new(start.clone(), end.clone()) {
        Ok(line) => line,
        Err(CurveError::ZeroLengthLine) => return Classification::Decided(None),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    let envelope = match Aabb2::from_points([start, end], policy) {
        Classification::Decided(envelope) => envelope,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    for point in controls
        .iter()
        .skip(1)
        .take(controls.len().saturating_sub(2))
    {
        match classify_oriented_line(start, end, point, policy) {
            Classification::Decided(LineSide::On) => {}
            Classification::Decided(_) => return Classification::Decided(None),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
        match envelope.contains_point(point, policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Classification::Decided(None),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(Some(line))
}

fn isolate_cubic_line_roots(
    distances: [Real; 4],
    policy: &CurvePolicy,
) -> Classification<BezierLineRelation> {
    // A cubic Bezier's signed distance to a supporting line is itself a scalar
    // cubic Bezier with control values equal to the control-point orientation
    // determinants. We isolate roots by exact Bernstein sign subdivision:
    // intervals whose control values have one strict sign are certified misses,
    // exact zero endpoints are retained as exact parameters, and remaining
    // mixed-sign cells are recursively bisected into certified dyadic brackets.
    // This is the Bezier clipping/sign-variation view used by Sederberg and
    // Nishita, "Curve intersection using Bezier clipping" (1990), with Yap's
    // exact-predicate boundary replacing floating tolerance decisions.
    let mut exact_parameters = Vec::new();
    let mut spans = Vec::new();
    if let Err(reason) = isolate_scalar_cubic_roots(
        distances,
        Real::zero(),
        Real::one(),
        0,
        policy,
        &mut exact_parameters,
        &mut spans,
    ) {
        return Classification::Uncertain(reason);
    }
    if !spans.is_empty() {
        for parameter in exact_parameters {
            push_unique_span(
                &mut spans,
                BezierMonotoneSpan::new(parameter.clone(), parameter),
                policy,
            );
        }
        return Classification::Decided(BezierLineRelation::IsolatedIntersections { spans });
    }
    if !exact_parameters.is_empty() {
        return Classification::Decided(BezierLineRelation::Intersects {
            parameters: exact_parameters,
        });
    }
    Classification::Decided(BezierLineRelation::Unresolved)
}

fn isolate_scalar_cubic_roots(
    controls: [Real; 4],
    start: Real,
    end: Real,
    depth: usize,
    policy: &CurvePolicy,
    exact_parameters: &mut Vec<Real>,
    spans: &mut Vec<BezierMonotoneSpan>,
) -> Result<(), UncertaintyReason> {
    let signs = controls
        .iter()
        .map(|value| real_sign(value, policy).ok_or(UncertaintyReason::RealSign))
        .collect::<Result<Vec<_>, _>>()?;

    if signs[0] == RealSign::Zero {
        push_unique_sorted(exact_parameters, start.clone(), policy);
    }
    if signs[3] == RealSign::Zero {
        push_unique_sorted(exact_parameters, end.clone(), policy);
    }

    let strict_signs = signs
        .iter()
        .copied()
        .filter(|sign| *sign != RealSign::Zero)
        .collect::<Vec<_>>();
    if strict_signs.is_empty() {
        push_unique_span(spans, BezierMonotoneSpan::new(start, end), policy);
        return Ok(());
    }
    if strict_signs.iter().all(|sign| *sign == strict_signs[0]) {
        return Ok(());
    }

    let mid = ((&start + &end) / Real::from(2_i8)).map_err(|_| UncertaintyReason::Unsupported)?;
    let mid_value = scalar_cubic_at_half(&controls);
    if is_zero(&mid_value, policy) == Some(true) {
        push_unique_sorted(exact_parameters, mid.clone(), policy);
        return Ok(());
    }

    if depth >= 32 {
        push_unique_span(spans, BezierMonotoneSpan::new(start, end), policy);
        return Ok(());
    }

    let (left, right) = subdivide_scalar_cubic_half(controls);
    isolate_scalar_cubic_roots(
        left,
        start,
        mid.clone(),
        depth + 1,
        policy,
        exact_parameters,
        spans,
    )?;
    isolate_scalar_cubic_roots(right, mid, end, depth + 1, policy, exact_parameters, spans)
}

fn scalar_cubic_at_half(controls: &[Real; 4]) -> Real {
    let eight = Real::from(8_i8);
    ((controls[0].clone()
        + (&Real::from(3_i8) * &controls[1])
        + (&Real::from(3_i8) * &controls[2])
        + controls[3].clone())
        / eight)
        .expect("division by positive integer constant is defined")
}

fn scalar_cubic_at_parameter(controls: &[Real; 4], t: Real) -> Real {
    let p01 = lerp_real(&controls[0], &controls[1], t.clone());
    let p12 = lerp_real(&controls[1], &controls[2], t.clone());
    let p23 = lerp_real(&controls[2], &controls[3], t.clone());
    let p012 = lerp_real(&p01, &p12, t.clone());
    let p123 = lerp_real(&p12, &p23, t.clone());
    lerp_real(&p012, &p123, t)
}

fn lerp_real(first: &Real, second: &Real, t: Real) -> Real {
    first + &(&t * &(second - first))
}

fn subdivide_scalar_cubic_half(controls: [Real; 4]) -> ([Real; 4], [Real; 4]) {
    let p01 = midpoint_real(&controls[0], &controls[1]);
    let p12 = midpoint_real(&controls[1], &controls[2]);
    let p23 = midpoint_real(&controls[2], &controls[3]);
    let p012 = midpoint_real(&p01, &p12);
    let p123 = midpoint_real(&p12, &p23);
    let p0123 = midpoint_real(&p012, &p123);
    (
        [
            controls[0].clone(),
            p01.clone(),
            p012.clone(),
            p0123.clone(),
        ],
        [p0123, p123, p23, controls[3].clone()],
    )
}

fn midpoint_real(left: &Real, right: &Real) -> Real {
    ((left + right) / Real::from(2_i8)).expect("division by positive integer constant is defined")
}

fn classify_quadratic_cusp(
    x: [Real; 3],
    y: [Real; 3],
    policy: &CurvePolicy,
) -> Classification<BezierCuspClassification> {
    if all_points_coincident3(&x, &y, policy) == Some(true) {
        return Classification::Decided(BezierCuspClassification::DegeneratePoint);
    }

    let x_roots = match derivative_root_set_quadratic(x, policy) {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let y_roots = match derivative_root_set_quadratic(y, policy) {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let common = common_root_set_parameters(&x_roots, &y_roots, policy);
    match common {
        Some(parameters) if parameters.is_empty() => {
            Classification::Decided(BezierCuspClassification::None)
        }
        Some(parameters) => Classification::Decided(BezierCuspClassification::Cusps { parameters }),
        None => Classification::Uncertain(UncertaintyReason::Ordering),
    }
}

fn classify_cubic_cusp(
    x: [Real; 4],
    y: [Real; 4],
    policy: &CurvePolicy,
) -> Classification<BezierCuspClassification> {
    if all_points_coincident4(&x, &y, policy) == Some(true) {
        return Classification::Decided(BezierCuspClassification::DegeneratePoint);
    }

    let x_roots = match derivative_root_set_cubic(x, policy) {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let y_roots = match derivative_root_set_cubic(y, policy) {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    match common_root_set_parameters(&x_roots, &y_roots, policy) {
        Some(parameters) if parameters.is_empty() => {
            Classification::Decided(BezierCuspClassification::None)
        }
        Some(parameters) => Classification::Decided(BezierCuspClassification::Cusps { parameters }),
        None => Classification::Uncertain(UncertaintyReason::Ordering),
    }
}

fn classify_cubic_inflections(
    controls: [&Point2; 4],
    policy: &CurvePolicy,
) -> Classification<BezierInflectionClassification> {
    // The curvature numerator is `cross(B'(t), B''(t))`. With cubic derivative
    // control edges `a = P1-P0`, `b = P2-P1`, `c = P3-P2`, the irrelevant
    // positive scalar factors can be dropped, leaving a quadratic in `t`.
    // This is the standard cubic Bezier inflection predicate; see Farin,
    // *Curves and Surfaces for Computer-Aided Geometric Design* (5th ed.,
    // 2002). Roots are retained exactly and only become branch parameters
    // after certified ordering against `[0, 1]`.
    let (ax, ay) = controls[1].delta_from(controls[0]);
    let (bx, by) = controls[2].delta_from(controls[1]);
    let (cx, cy) = controls[3].delta_from(controls[2]);

    let d0x = ax.clone();
    let d0y = ay.clone();
    let two = Real::from(2_i8);
    let d1x = &two * &(&bx - &ax);
    let d1y = &two * &(&by - &ay);
    let d2x = &ax - &(&two * &bx) + &cx;
    let d2y = &ay - &(&two * &by) + &cy;

    let e0x = &bx - &ax;
    let e0y = &by - &ay;
    let e1x = &cx - &(&two * &bx) + &ax;
    let e1y = &cy - &(&two * &by) + &ay;

    let c0 = cross(&d0x, &d0y, &e0x, &e0y);
    let c1 = cross(&d0x, &d0y, &e1x, &e1y) + cross(&d1x, &d1y, &e0x, &e0y);
    let c2 = cross(&d1x, &d1y, &e1x, &e1y) + cross(&d2x, &d2y, &e0x, &e0y);

    if [&c0, &c1, &c2]
        .iter()
        .all(|value| is_zero(value, policy) == Some(true))
    {
        return Classification::Decided(BezierInflectionClassification::AllCurvatureZero);
    }

    polynomial_roots_in_unit_interval(c0, c1, c2, policy).map(|parameters| {
        if parameters.is_empty() {
            BezierInflectionClassification::None
        } else {
            BezierInflectionClassification::Inflections { parameters }
        }
    })
}

fn derivative_roots_quadratic(
    values: [Real; 3],
    policy: &CurvePolicy,
) -> Classification<Vec<Real>> {
    let [p0, p1, p2] = values;
    let a = &p1 - &p0;
    let b = &p2 - &(Real::from(2_i8) * &p1) + &p0;
    linear_roots_in_unit_interval(a, b, policy)
}

fn derivative_roots_cubic(values: [Real; 4], policy: &CurvePolicy) -> Classification<Vec<Real>> {
    let [p0, p1, p2, p3] = values;
    let a = &p1 - &p0;
    let b = &p2 - &p1;
    let c = &p3 - &p2;
    let two = Real::from(2_i8);
    let c0 = a.clone();
    let c1 = &two * &(&b - &a);
    let c2 = &a - &(&two * &b) + &c;
    polynomial_roots_in_unit_interval(c0, c1, c2, policy)
}

#[derive(Clone, Debug, PartialEq)]
enum RootSet {
    All,
    Roots(Vec<Real>),
}

fn derivative_root_set_quadratic(
    values: [Real; 3],
    policy: &CurvePolicy,
) -> Classification<RootSet> {
    let [p0, p1, p2] = values;
    let a = &p1 - &p0;
    let b = &p2 - &(Real::from(2_i8) * &p1) + &p0;
    if is_zero(&a, policy) == Some(true) && is_zero(&b, policy) == Some(true) {
        return Classification::Decided(RootSet::All);
    }
    derivative_roots_quadratic([p0, p1, p2], policy).map(RootSet::Roots)
}

fn derivative_root_set_cubic(values: [Real; 4], policy: &CurvePolicy) -> Classification<RootSet> {
    let [p0, p1, p2, p3] = values;
    let a = &p1 - &p0;
    let b = &p2 - &p1;
    let c = &p3 - &p2;
    let two = Real::from(2_i8);
    let c0 = a.clone();
    let c1 = &two * &(&b - &a);
    let c2 = &a - &(&two * &b) + &c;
    if [&c0, &c1, &c2]
        .iter()
        .all(|value| is_zero(value, policy) == Some(true))
    {
        return Classification::Decided(RootSet::All);
    }
    derivative_roots_cubic([p0, p1, p2, p3], policy).map(RootSet::Roots)
}

pub(crate) fn polynomial_roots_in_unit_interval(
    c0: Real,
    c1: Real,
    c2: Real,
    policy: &CurvePolicy,
) -> Classification<Vec<Real>> {
    if is_zero(&c2, policy) == Some(true) {
        return linear_roots_in_unit_interval(c0, c1, policy);
    }
    if is_zero(&c2, policy).is_none() {
        return Classification::Uncertain(UncertaintyReason::RealSign);
    }

    let four = Real::from(4_i8);
    let two = Real::from(2_i8);
    let discriminant = (&c1 * &c1) - (&four * &c2 * &c0);
    match real_sign(&discriminant, policy) {
        Some(RealSign::Negative) => Classification::Decided(Vec::new()),
        Some(RealSign::Zero) => {
            let denominator = &two * &c2;
            match (Real::zero() - &c1) / denominator {
                Ok(root) => retain_unit_roots(vec![root], policy),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        Some(RealSign::Positive) => {
            let Ok(sqrt_discriminant) = discriminant.sqrt() else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            let denominator = &two * &c2;
            let Ok(root0) = (Real::zero() - &c1 - &sqrt_discriminant) / &denominator else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            let Ok(root1) = (Real::zero() - &c1 + sqrt_discriminant) / denominator else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            retain_unit_roots(vec![root0, root1], policy)
        }
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn linear_roots_in_unit_interval(
    c0: Real,
    c1: Real,
    policy: &CurvePolicy,
) -> Classification<Vec<Real>> {
    match is_zero(&c1, policy) {
        Some(true) => Classification::Decided(Vec::new()),
        Some(false) => match (Real::zero() - &c0) / c1 {
            Ok(root) => retain_unit_roots(vec![root], policy),
            Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
        },
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn retain_unit_roots(roots: Vec<Real>, policy: &CurvePolicy) -> Classification<Vec<Real>> {
    let mut retained = Vec::new();
    for root in roots {
        match in_closed_unit_interval(&root, policy) {
            Some(true) => push_unique_sorted(&mut retained, root, policy),
            Some(false) => {}
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }
    Classification::Decided(retained)
}

fn push_unique_sorted(values: &mut Vec<Real>, value: Real, policy: &CurvePolicy) {
    if values
        .iter()
        .any(|existing| compare_reals(existing, &value, policy) == Some(Ordering::Equal))
    {
        return;
    }
    values.push(value);
    values.sort_by(|a, b| compare_reals(a, b, policy).unwrap_or(Ordering::Equal));
}

fn push_unique_span(
    spans: &mut Vec<BezierMonotoneSpan>,
    span: BezierMonotoneSpan,
    policy: &CurvePolicy,
) {
    if spans.iter().any(|existing| {
        compare_reals(existing.start(), span.start(), policy) == Some(Ordering::Equal)
            && compare_reals(existing.end(), span.end(), policy) == Some(Ordering::Equal)
    }) {
        return;
    }
    spans.push(span);
    spans.sort_by(|a, b| compare_reals(a.start(), b.start(), policy).unwrap_or(Ordering::Equal));
}

pub(crate) fn monotone_spans_from_parameters(
    parameters: [Classification<Vec<Real>>; 2],
    policy: &CurvePolicy,
) -> Classification<Vec<BezierMonotoneSpan>> {
    let mut split_parameters = vec![Real::zero(), Real::one()];
    for roots in parameters {
        let roots = match roots {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        for root in roots {
            push_unique_sorted(&mut split_parameters, root, policy);
        }
    }

    let spans = split_parameters
        .windows(2)
        .map(|pair| BezierMonotoneSpan::new(pair[0].clone(), pair[1].clone()))
        .collect();
    Classification::Decided(spans)
}

fn axis_values3(points: [&Point2; 3], axis: Axis2) -> [Real; 3] {
    [
        coordinate(points[0], axis).clone(),
        coordinate(points[1], axis).clone(),
        coordinate(points[2], axis).clone(),
    ]
}

fn axis_values4(points: [&Point2; 4], axis: Axis2) -> [Real; 4] {
    [
        coordinate(points[0], axis).clone(),
        coordinate(points[1], axis).clone(),
        coordinate(points[2], axis).clone(),
        coordinate(points[3], axis).clone(),
    ]
}

fn coordinate(point: &Point2, axis: Axis2) -> &Real {
    match axis {
        Axis2::X => point.x(),
        Axis2::Y => point.y(),
    }
}

fn point_equal(a: &Point2, b: &Point2, policy: &CurvePolicy) -> Option<bool> {
    is_zero(&a.distance_squared(b), policy)
}

fn point_coordinates_equal(a: &Point2, b: &Point2, policy: &CurvePolicy) -> Option<bool> {
    match (
        is_zero(&(a.x() - b.x()), policy),
        is_zero(&(a.y() - b.y()), policy),
    ) {
        (Some(true), Some(true)) => Some(true),
        (Some(false), _) | (_, Some(false)) => Some(false),
        _ => None,
    }
}

fn common_parameters(left: &[Real], right: &[Real], policy: &CurvePolicy) -> Option<Vec<Real>> {
    let mut common = Vec::new();
    for a in left {
        for b in right {
            match compare_reals(a, b, policy)? {
                Ordering::Equal => push_unique_sorted(&mut common, a.clone(), policy),
                Ordering::Less | Ordering::Greater => {}
            }
        }
    }
    Some(common)
}

fn common_root_set_parameters(
    left: &RootSet,
    right: &RootSet,
    policy: &CurvePolicy,
) -> Option<Vec<Real>> {
    match (left, right) {
        (RootSet::All, RootSet::All) => Some(vec![Real::zero()]),
        (RootSet::All, RootSet::Roots(roots)) | (RootSet::Roots(roots), RootSet::All) => {
            Some(roots.clone())
        }
        (RootSet::Roots(left), RootSet::Roots(right)) => common_parameters(left, right, policy),
    }
}

fn is_unit_endpoint(value: &Real, policy: &CurvePolicy) -> bool {
    compare_reals(value, &Real::zero(), policy) == Some(Ordering::Equal)
        || compare_reals(value, &Real::one(), policy) == Some(Ordering::Equal)
}

fn all_points_coincident3(x: &[Real; 3], y: &[Real; 3], policy: &CurvePolicy) -> Option<bool> {
    Some(all_same(&[&x[0], &x[1], &x[2]], policy)? && all_same(&[&y[0], &y[1], &y[2]], policy)?)
}

fn all_points_coincident4(x: &[Real; 4], y: &[Real; 4], policy: &CurvePolicy) -> Option<bool> {
    Some(
        all_same(&[&x[0], &x[1], &x[2], &x[3]], policy)?
            && all_same(&[&y[0], &y[1], &y[2], &y[3]], policy)?,
    )
}

fn all_same(values: &[&Real], policy: &CurvePolicy) -> Option<bool> {
    for value in &values[1..] {
        if compare_reals(values[0], value, policy)? != Ordering::Equal {
            return Some(false);
        }
    }
    Some(true)
}

fn cross(ax: &Real, ay: &Real, bx: &Real, by: &Real) -> Real {
    (ax * by) - (ay * bx)
}
