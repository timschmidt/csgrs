//! Rational quadratic Bezier and conic primitives.
//!
//! Rational quadratics are the lowest-degree Bezier representation that can
//! carry non-parabolic conics exactly. The homogeneous evaluation below keeps
//! the polynomial numerator and weight denominator visible instead of
//! flattening to sampled chords, matching Yap's exact geometric computation
//! advice to preserve object structure until a certified predicate boundary;
//! see Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7.1-2 (1997). The conic weight classifier follows the rational quadratic
//! treatment in Farin, *Curves and Surfaces for Computer-Aided Geometric
//! Design* (5th ed., 2002).

use hyperreal::{Real, RealSign, ZeroKnowledge as ZeroStatus};

use crate::bezier_topology::polynomial_roots_in_unit_interval;
use crate::classify::{classify_oriented_line, is_zero, orient2d_real_expr, real_sign};
use crate::{
    Aabb2, Axis2, BezierCurveIntersectionPoint, BezierCurveIntersectionRegion, BezierCurveRelation,
    BezierLineRelation, BezierMonotoneSpan, Classification, CubicBezier2, CurveError, CurvePolicy,
    LineSeg2, LineSide, Point2, QuadraticBezier2, UncertaintyReason,
};

/// Coarse conic family represented by a rational quadratic Bezier segment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RationalQuadraticConicKind {
    /// Ellipse-like conic arc, including circular arcs.
    EllipseLike,
    /// Parabolic conic arc.
    Parabola,
    /// Hyperbola-like conic arc.
    HyperbolaLike,
}

/// A rational quadratic Bezier segment with exact control points and weights.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalQuadraticBezier2 {
    start: Point2,
    control: Point2,
    end: Point2,
    start_weight: Real,
    control_weight: Real,
    end_weight: Real,
}

impl RationalQuadraticBezier2 {
    /// Constructs a rational quadratic segment after rejecting provably zero weights.
    pub fn try_new(
        start: Point2,
        control: Point2,
        end: Point2,
        start_weight: Real,
        control_weight: Real,
        end_weight: Real,
    ) -> Result<Self, CurveError> {
        if [
            start_weight.zero_status(),
            control_weight.zero_status(),
            end_weight.zero_status(),
        ]
        .contains(&ZeroStatus::Zero)
        {
            return Err(CurveError::ZeroRationalBezierWeight);
        }
        Ok(Self {
            start,
            control,
            end,
            start_weight,
            control_weight,
            end_weight,
        })
    }

    /// Constructs the common conic form with endpoint weights equal to one.
    pub fn try_unit_end_weights(
        start: Point2,
        control: Point2,
        end: Point2,
        control_weight: Real,
    ) -> Result<Self, CurveError> {
        Self::try_new(
            start,
            control,
            end,
            Real::one(),
            control_weight,
            Real::one(),
        )
    }

    /// Returns the start point.
    pub const fn start(&self) -> &Point2 {
        &self.start
    }

    /// Returns the interior control point.
    pub const fn control(&self) -> &Point2 {
        &self.control
    }

    /// Returns the end point.
    pub const fn end(&self) -> &Point2 {
        &self.end
    }

    /// Returns the start weight.
    pub const fn start_weight(&self) -> &Real {
        &self.start_weight
    }

    /// Returns the interior control weight.
    pub const fn control_weight(&self) -> &Real {
        &self.control_weight
    }

    /// Returns the end weight.
    pub const fn end_weight(&self) -> &Real {
        &self.end_weight
    }

    /// Returns the control points in polynomial order.
    pub fn control_points(&self) -> [&Point2; 3] {
        [&self.start, &self.control, &self.end]
    }

    /// Returns the weights in polynomial order.
    pub fn weights(&self) -> [&Real; 3] {
        [&self.start_weight, &self.control_weight, &self.end_weight]
    }

    /// Evaluates the rational segment at affine parameter `t`.
    ///
    /// The numerator and denominator are evaluated in homogeneous Bernstein
    /// form: `(sum B_i(t) w_i P_i) / (sum B_i(t) w_i)`. A zero denominator is
    /// a projective boundary, so this API returns explicit uncertainty instead
    /// of inventing an affine point.
    pub fn point_at(&self, t: Real, policy: &CurvePolicy) -> Classification<Point2> {
        let one_minus_t = Real::one() - &t;
        let two = Real::from(2_i8);
        let b0 = &one_minus_t * &one_minus_t * &self.start_weight;
        let b1 = &two * &one_minus_t * &t * &self.control_weight;
        let b2 = &t * &t * &self.end_weight;
        let denominator = &b0 + &b1 + &b2;
        match is_zero(&denominator, policy) {
            Some(true) => return Classification::Uncertain(UncertaintyReason::Boundary),
            Some(false) => {}
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }

        let numerator_x = (&b0 * self.start.x()) + (&b1 * self.control.x()) + (&b2 * self.end.x());
        let numerator_y = (&b0 * self.start.y()) + (&b1 * self.control.y()) + (&b2 * self.end.y());
        let Ok(x) = numerator_x / &denominator else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };
        let Ok(y) = numerator_y / denominator else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };
        Classification::Decided(Point2::new(x, y))
    }

    /// Classifies whether `point` equals this conic at parameter `t`.
    ///
    /// This is a parameterized predicate rather than an existential conic
    /// solve. It first evaluates the homogeneous rational point, then certifies
    /// affine equality through the active curve policy. Returning uncertainty
    /// at denominator boundaries keeps projective singularities explicit in
    /// the style advocated by Yap's exact geometric computation model.
    pub fn contains_point_at_parameter(
        &self,
        point: &Point2,
        t: Real,
        policy: &CurvePolicy,
    ) -> Classification<bool> {
        let curve_point = match self.point_at(t, policy) {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        is_zero(&curve_point.distance_squared(point), policy)
            .map(Classification::Decided)
            .unwrap_or(Classification::Uncertain(UncertaintyReason::RealSign))
    }

    /// Returns all certified affine parameters where `point` lies on this conic.
    ///
    /// This is the existential point-on-conic solver for rational quadratics.
    /// Each coordinate equation is kept in homogeneous form as
    /// `N_axis(t) - point_axis * D(t) = 0`, so the rational structure is
    /// preserved until exact candidate parameters are certified by
    /// re-evaluating the conic. That follows Yap's requirement to keep exact
    /// geometric objects explicit until a predicate boundary; see Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997). The weighted Bernstein numerator/denominator identities follow
    /// the rational Bezier treatment in Farin, *Curves and Surfaces for
    /// Computer-Aided Geometric Design* (5th ed., 2002).
    pub fn parameters_for_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<Vec<Real>> {
        rational_parameters_for_point(self, point, policy)
    }

    /// Classifies whether `point` lies anywhere on this finite conic segment.
    ///
    /// Denominator boundaries are reported as uncertainty instead of being
    /// projected into affine space. Use [`Self::parameters_for_point`] when the
    /// certified parameters themselves are needed by downstream topology.
    pub fn contains_point(&self, point: &Point2, policy: &CurvePolicy) -> Classification<bool> {
        self.parameters_for_point(point, policy)
            .map(|parameters| !parameters.is_empty())
    }

    /// Classifies this rational quadratic against a supporting line.
    ///
    /// The signed distance numerator of a rational Bezier to a line is a
    /// polynomial Bezier with control values `w_i * orient(line, P_i)`.
    /// Solving that numerator preserves the homogeneous conic structure instead
    /// of sampling or flattening the curve. This is the rational Bezier form of
    /// the line-incidence predicates described by Farin (2002), with branch
    /// decisions routed through exact scalar signs per Yap (1997).
    pub fn relation_to_line(
        &self,
        line: &LineSeg2,
        policy: &CurvePolicy,
    ) -> Classification<BezierLineRelation> {
        let controls = self.control_points();
        let weights = self.weights();
        let weighted_distances = controls
            .iter()
            .zip(weights)
            .map(|(point, weight)| orient2d_real_expr(line.start(), line.end(), point) * weight)
            .collect::<Vec<_>>();

        if weighted_distances
            .iter()
            .all(|value| is_zero(value, policy) == Some(true))
        {
            return Classification::Decided(BezierLineRelation::OnSupportingLine);
        }

        let two = Real::from(2_i8);
        let c0 = weighted_distances[0].clone();
        let c1 = &two * &(&weighted_distances[1] - &weighted_distances[0]);
        let c2 = &weighted_distances[0] - &(&two * &weighted_distances[1]) + &weighted_distances[2];
        let roots = match polynomial_roots_in_unit_interval(c0, c1, c2, policy) {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let mut retained_roots = Vec::new();
        for root in roots {
            match is_zero(&self.denominator_at(root.clone()), policy) {
                Some(true) => return Classification::Uncertain(UncertaintyReason::Boundary),
                Some(false) => retained_roots.push(root),
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        if !retained_roots.is_empty() {
            return Classification::Decided(BezierLineRelation::Intersects {
                parameters: retained_roots,
            });
        }

        if self.weights_known_positive(policy) == Some(true) {
            let sides = controls
                .iter()
                .map(|point| classify_oriented_line(line.start(), line.end(), point, policy))
                .collect::<Vec<_>>();
            if sides
                .iter()
                .all(|side| matches!(side, Classification::Decided(LineSide::Left)))
            {
                return Classification::Decided(BezierLineRelation::ControlHullDisjoint {
                    side: LineSide::Left,
                });
            }
            if sides
                .iter()
                .all(|side| matches!(side, Classification::Decided(LineSide::Right)))
            {
                return Classification::Decided(BezierLineRelation::ControlHullDisjoint {
                    side: LineSide::Right,
                });
            }
        }

        Classification::Decided(BezierLineRelation::Unresolved)
    }

    /// Returns quotient-derivative roots that split this conic into monotone spans.
    ///
    /// For a rational coordinate `N(t) / D(t)`, extrema occur where
    /// `N'(t)D(t) - N(t)D'(t) = 0` and `D(t) != 0`. The cubic terms cancel for
    /// rational quadratics, leaving an exact quadratic root problem. This keeps
    /// the homogeneous numerator/denominator visible as recommended by Yap
    /// (1997), and follows the rational Bezier derivative identity in Farin
    /// (2002).
    pub fn axis_monotone_parameters(
        &self,
        axis: Axis2,
        policy: &CurvePolicy,
    ) -> Classification<Vec<Real>> {
        let (n0, n1, n2) = self.weighted_coordinate_power_basis(axis);
        let (d0, d1, d2) = self.weight_power_basis();
        let two = Real::from(2_i8);
        let c0 = (&n1 * &d0) - (&n0 * &d1);
        let c1 = &two * &((&n2 * &d0) - (&n0 * &d2));
        let c2 = (&n2 * &d1) - (&n1 * &d2);
        let roots = match polynomial_roots_in_unit_interval(c0, c1, c2, policy) {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };

        let mut retained_roots = Vec::new();
        for root in roots {
            match is_zero(&self.denominator_at(root.clone()), policy) {
                Some(true) => return Classification::Uncertain(UncertaintyReason::Boundary),
                Some(false) => retained_roots.push(root),
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        Classification::Decided(retained_roots)
    }

    /// Decomposes the conic at all certified x/y quotient-derivative roots.
    pub fn monotone_spans(&self, policy: &CurvePolicy) -> Classification<Vec<BezierMonotoneSpan>> {
        crate::bezier_topology::monotone_spans_from_parameters(
            [
                self.axis_monotone_parameters(Axis2::X, policy),
                self.axis_monotone_parameters(Axis2::Y, policy),
            ],
            policy,
        )
    }

    /// Returns a certified rational-conic bounding box from endpoints and extrema.
    ///
    /// Non-equal positive weights also include the Euclidean control point as a
    /// conservative hull witness. That preserves the rational Bezier
    /// convex-hull guarantee when algebraic extrema are present but still
    /// avoids making a topology decision from an approximate sample.
    pub fn certified_bounds(&self, policy: &CurvePolicy) -> Classification<Aabb2> {
        let mut samples = vec![self.start.clone(), self.end.clone()];
        if self.weights_known_positive(policy) == Some(true)
            && self.weights_equal(policy) == Some(false)
        {
            samples.push(self.control.clone());
        }
        let spans = match self.monotone_spans(policy) {
            Classification::Decided(spans) => spans,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        for span in spans {
            if !is_unit_endpoint(span.start(), policy) {
                match self.point_at(span.start().clone(), policy) {
                    Classification::Decided(point) => samples.push(point),
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
            if !is_unit_endpoint(span.end(), policy) {
                match self.point_at(span.end().clone(), policy) {
                    Classification::Decided(point) => samples.push(point),
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
        }
        Aabb2::from_points(samples.iter(), policy)
    }

    /// Classifies a coarse relation between two rational quadratic conics.
    ///
    /// This is deliberately not a full conic/conic solver. It certifies exact
    /// homogeneous-control identity, positive-weight convex-hull disjointness,
    /// certified endpoint line-segment images, and shared endpoints, then
    /// leaves overlapping boxes unresolved for a later resultant or subdivision
    /// predicate. Keeping that boundary explicit follows Yap's
    /// exact-computation separation between cheap structural facts and complete
    /// topology solvers.
    pub fn relation_to_rational_quadratic(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Classification<BezierCurveRelation> {
        match self.same_homogeneous_controls(other, policy) {
            Some(true) => return Classification::Decided(BezierCurveRelation::SameControlPolygon),
            Some(false) => {}
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
        match self.same_projective_homogeneous_controls(other, policy) {
            Some(true) => return Classification::Decided(BezierCurveRelation::SameCurveImage),
            Some(false) => {}
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }

        match polynomial_relation_for_equal_weight_rationals(self, other, policy) {
            Classification::Decided(Some(relation)) => return Classification::Decided(relation),
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }

        if self.weights_known_positive(policy) == Some(true)
            && other.weights_known_positive(policy) == Some(true)
        {
            let first_box = match Aabb2::from_points(self.control_points(), policy) {
                Classification::Decided(bbox) => bbox,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
            let second_box = match Aabb2::from_points(other.control_points(), policy) {
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
        }

        match (
            rational_line_segment_image(self, policy),
            rational_line_segment_image(other, policy),
        ) {
            (Classification::Decided(Some(first)), Classification::Decided(Some(second))) => {
                return line_segment_intersection_relation(&first, &second, policy);
            }
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            _ => {}
        }

        for a in [self.start(), self.end()] {
            for b in [other.start(), other.end()] {
                match point_equal(a, b, policy) {
                    Some(true) => {
                        return Classification::Decided(BezierCurveRelation::SharedEndpoint);
                    }
                    Some(false) => {}
                    None => return Classification::Uncertain(UncertaintyReason::RealSign),
                }
            }
        }

        match rational_rational_endpoint_intersections(self, other, policy) {
            Classification::Decided(points) if !points.is_empty() => {
                return Classification::Decided(BezierCurveRelation::EndpointIntersections {
                    points,
                });
            }
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }

        match same_parameter_matching_weight_rational_relation(self, other, policy) {
            Classification::Decided(Some(relation)) => return Classification::Decided(relation),
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }

        if self.weights_known_positive(policy) == Some(true)
            && other.weights_known_positive(policy) == Some(true)
        {
            return isolate_curve_regions(
                RationalSubdivisionNode::from_rational(self),
                RationalSubdivisionNode::from_rational(other),
                policy,
            );
        }

        Classification::Decided(BezierCurveRelation::Unresolved)
    }

    /// Classifies a coarse relation between this conic and a polynomial quadratic.
    ///
    /// A positive-weight rational Bezier segment lies in the convex hull of its
    /// Euclidean control points, so a disjoint hull box is a certified miss.
    /// This is the convex-hull predicate for rational Beziers described by
    /// Farin (2002), used only when exact signs prove the weights positive in
    /// the EGC sense of Yap (1997). Equal weights collapse the homogeneous
    /// conic to the polynomial quadratic with the same control polygon.
    pub fn relation_to_quadratic(
        &self,
        other: &QuadraticBezier2,
        policy: &CurvePolicy,
    ) -> Classification<BezierCurveRelation> {
        relation_to_polynomial_bezier(self, other.control_points().as_slice(), policy)
    }

    /// Classifies a coarse relation between this conic and a polynomial cubic.
    ///
    /// Degree-mismatched overlaps are intentionally not solved here; the
    /// predicate certifies hull disjointness and shared endpoints, then returns
    /// [`BezierCurveRelation::Unresolved`] for cases needing a complete
    /// curve/curve root solver.
    pub fn relation_to_cubic(
        &self,
        other: &CubicBezier2,
        policy: &CurvePolicy,
    ) -> Classification<BezierCurveRelation> {
        relation_to_polynomial_bezier(self, other.control_points().as_slice(), policy)
    }

    /// Classifies the represented conic family from the homogeneous weights.
    pub fn conic_kind(&self, policy: &CurvePolicy) -> Classification<RationalQuadraticConicKind> {
        let discriminant =
            (&self.control_weight * &self.control_weight) - (&self.start_weight * &self.end_weight);
        match real_sign(&discriminant, policy) {
            Some(RealSign::Negative) => {
                Classification::Decided(RationalQuadraticConicKind::EllipseLike)
            }
            Some(RealSign::Zero) => Classification::Decided(RationalQuadraticConicKind::Parabola),
            Some(RealSign::Positive) => {
                Classification::Decided(RationalQuadraticConicKind::HyperbolaLike)
            }
            None => Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }

    /// Returns conservative structural facts for exact predicate scheduling.
    pub fn structural_facts(&self) -> crate::RationalQuadraticBezier2Facts {
        crate::facts::rational_quadratic_bezier_facts(self)
    }

    fn denominator_at(&self, t: Real) -> Real {
        let one_minus_t = Real::one() - &t;
        let two = Real::from(2_i8);
        let b0 = &one_minus_t * &one_minus_t * &self.start_weight;
        let b1 = &two * &one_minus_t * &t * &self.control_weight;
        let b2 = &t * &t * &self.end_weight;
        &b0 + &b1 + &b2
    }

    fn weighted_coordinate_power_basis(&self, axis: Axis2) -> (Real, Real, Real) {
        quadratic_bernstein_to_power([
            coordinate(self.start(), axis) * &self.start_weight,
            coordinate(self.control(), axis) * &self.control_weight,
            coordinate(self.end(), axis) * &self.end_weight,
        ])
    }

    fn weight_power_basis(&self) -> (Real, Real, Real) {
        quadratic_bernstein_to_power([
            self.start_weight.clone(),
            self.control_weight.clone(),
            self.end_weight.clone(),
        ])
    }

    fn weights_known_positive(&self, policy: &CurvePolicy) -> Option<bool> {
        self.weights()
            .iter()
            .map(|weight| real_sign(weight, policy).map(|sign| sign == RealSign::Positive))
            .try_fold(true, |all_positive, positive| {
                positive.map(|positive| all_positive && positive)
            })
    }

    fn weights_equal(&self, policy: &CurvePolicy) -> Option<bool> {
        [
            &self.start_weight - &self.control_weight,
            &self.control_weight - &self.end_weight,
        ]
        .into_iter()
        .map(|difference| is_zero(&difference, policy))
        .try_fold(true, |same, item| item.map(|item| same && item))
    }

    fn same_homogeneous_controls(&self, other: &Self, policy: &CurvePolicy) -> Option<bool> {
        let same_points = self
            .control_points()
            .iter()
            .zip(other.control_points().iter())
            .map(|(a, b)| point_equal(a, b, policy))
            .try_fold(true, |same, item| item.map(|item| same && item))?;
        let same_weights = self
            .weights()
            .iter()
            .zip(other.weights().iter())
            .map(|(a, b)| is_zero(&(*a - *b), policy))
            .try_fold(true, |same, item| item.map(|item| same && item))?;
        Some(same_points && same_weights)
    }

    fn same_projective_homogeneous_controls(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Option<bool> {
        let same_points = self
            .control_points()
            .iter()
            .zip(other.control_points().iter())
            .map(|(a, b)| point_equal(a, b, policy))
            .try_fold(true, |same, item| item.map(|item| same && item))?;
        if !same_points {
            return Some(false);
        }

        // Homogeneous rational Bezier controls are projective: multiplying all
        // weights by one nonzero scalar multiplies both numerator and
        // denominator by that scalar and leaves the represented conic segment
        // unchanged. We test proportionality by exact cross-products instead
        // of division, preserving Yap's exact predicate boundary; the
        // homogeneous rational Bezier model is the one described by Farin,
        // Curves and Surfaces for CAGD, 5th ed. (2002).
        let first = self.weights();
        let second = other.weights();
        [
            first[0] * second[1] - &(first[1] * second[0]),
            first[1] * second[2] - &(first[2] * second[1]),
        ]
        .into_iter()
        .map(|difference| is_zero(&difference, policy))
        .try_fold(true, |same, item| item.map(|item| same && item))
    }

    fn same_polynomial_quadratic_controls(
        &self,
        polynomial_controls: &[&Point2],
        policy: &CurvePolicy,
    ) -> Option<bool> {
        let same_points = self
            .control_points()
            .iter()
            .zip(polynomial_controls.iter())
            .map(|(a, b)| point_equal(a, b, policy))
            .try_fold(true, |same, item| item.map(|item| same && item))?;
        let same_weights = self.weights_equal(policy)?;
        Some(same_points && same_weights)
    }
}

fn coordinate(point: &Point2, axis: Axis2) -> &Real {
    match axis {
        Axis2::X => point.x(),
        Axis2::Y => point.y(),
    }
}

fn quadratic_bernstein_to_power(values: [Real; 3]) -> (Real, Real, Real) {
    let two = Real::from(2_i8);
    let c0 = values[0].clone();
    let c1 = &two * &(&values[1] - &values[0]);
    let c2 = &values[0] - &(&two * &values[1]) + &values[2];
    (c0, c1, c2)
}

#[derive(Clone, Debug, PartialEq)]
enum RationalPointRootSet {
    All,
    Roots(Vec<Real>),
}

fn rational_parameters_for_point(
    curve: &RationalQuadraticBezier2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Classification<Vec<Real>> {
    let x_roots = match rational_axis_point_root_set(curve, point, Axis2::X, policy) {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let y_roots = match rational_axis_point_root_set(curve, point, Axis2::Y, policy) {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    rational_point_parameters_from_root_sets(curve, point, x_roots, y_roots, policy)
}

fn rational_axis_point_root_set(
    curve: &RationalQuadraticBezier2,
    point: &Point2,
    axis: Axis2,
    policy: &CurvePolicy,
) -> Classification<RationalPointRootSet> {
    let target = coordinate(point, axis);
    let controls = curve.control_points();
    let weights = curve.weights();
    let values = [
        weights[0] * &(coordinate(controls[0], axis) - target),
        weights[1] * &(coordinate(controls[1], axis) - target),
        weights[2] * &(coordinate(controls[2], axis) - target),
    ];
    if values
        .iter()
        .all(|value| is_zero(value, policy) == Some(true))
    {
        return Classification::Decided(RationalPointRootSet::All);
    }
    let (c0, c1, c2) = quadratic_bernstein_to_power(values);
    polynomial_roots_in_unit_interval(c0, c1, c2, policy).map(RationalPointRootSet::Roots)
}

fn rational_point_parameters_from_root_sets(
    curve: &RationalQuadraticBezier2,
    point: &Point2,
    x_roots: RationalPointRootSet,
    y_roots: RationalPointRootSet,
    policy: &CurvePolicy,
) -> Classification<Vec<Real>> {
    let candidates = match (&x_roots, &y_roots) {
        (RationalPointRootSet::All, RationalPointRootSet::All) => vec![Real::zero()],
        (RationalPointRootSet::All, RationalPointRootSet::Roots(roots))
        | (RationalPointRootSet::Roots(roots), RationalPointRootSet::All) => roots.clone(),
        (RationalPointRootSet::Roots(left), RationalPointRootSet::Roots(right)) => {
            let mut candidates = left.clone();
            candidates.extend(right.iter().cloned());
            candidates
        }
    };

    let mut parameters = Vec::new();
    for candidate in candidates {
        match curve.point_at(candidate.clone(), policy) {
            Classification::Decided(curve_point) => {
                match point_equal(&curve_point, point, policy) {
                    Some(true) => parameters.push(candidate),
                    Some(false) => {}
                    None => return Classification::Uncertain(UncertaintyReason::RealSign),
                }
            }
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(parameters)
}

fn is_unit_endpoint(value: &Real, policy: &CurvePolicy) -> bool {
    is_zero(value, policy) == Some(true) || is_zero(&(value - &Real::one()), policy) == Some(true)
}

impl QuadraticBezier2 {
    /// Classifies a coarse relation between this polynomial quadratic and a rational conic.
    pub fn relation_to_rational_quadratic(
        &self,
        other: &RationalQuadraticBezier2,
        policy: &CurvePolicy,
    ) -> Classification<BezierCurveRelation> {
        other.relation_to_quadratic(self, policy)
    }
}

impl CubicBezier2 {
    /// Classifies a coarse relation between this polynomial cubic and a rational conic.
    pub fn relation_to_rational_quadratic(
        &self,
        other: &RationalQuadraticBezier2,
        policy: &CurvePolicy,
    ) -> Classification<BezierCurveRelation> {
        other.relation_to_cubic(self, policy)
    }
}

fn relation_to_polynomial_bezier(
    rational: &RationalQuadraticBezier2,
    polynomial_controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<BezierCurveRelation> {
    if polynomial_controls.len() == 3 {
        match rational.same_polynomial_quadratic_controls(polynomial_controls, policy) {
            Some(true) => return Classification::Decided(BezierCurveRelation::SameControlPolygon),
            Some(false) => {}
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }

    match polynomial_relation_for_equal_weight_rational(rational, polynomial_controls, policy) {
        Classification::Decided(Some(relation)) => return Classification::Decided(relation),
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    let mut deferred_uncertainty = None;
    if rational.weights_known_positive(policy) == Some(true) {
        let boxes = match (
            Aabb2::from_points(rational.control_points(), policy),
            Aabb2::from_points(polynomial_controls.iter().copied(), policy),
        ) {
            (Classification::Decided(rational_box), Classification::Decided(polynomial_box)) => {
                Some((rational_box, polynomial_box))
            }
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                deferred_uncertainty = Some(reason);
                None
            }
        };
        if let Some((rational_box, polynomial_box)) = boxes {
            match rational_box.overlaps(&polynomial_box, policy) {
                Classification::Decided(false) => {
                    return Classification::Decided(BezierCurveRelation::BoundingBoxesDisjoint);
                }
                Classification::Decided(true) => {}
                Classification::Uncertain(reason) => deferred_uncertainty = Some(reason),
            }
        }
    }

    let rational_line_image = match rational_line_segment_image(rational, policy) {
        Classification::Decided(line) => line,
        Classification::Uncertain(reason) => {
            deferred_uncertainty.get_or_insert(reason);
            None
        }
    };
    let polynomial_line_image = match line_segment_image_from_controls(polynomial_controls, policy)
    {
        Classification::Decided(line) => line,
        Classification::Uncertain(reason) => {
            deferred_uncertainty.get_or_insert(reason);
            None
        }
    };
    match (&rational_line_image, &polynomial_line_image) {
        (Some(first), Some(second)) => {
            return line_segment_intersection_relation(&first, &second, policy);
        }
        (None, Some(line)) => match line_image_rational_intersections(line, rational, policy) {
            Classification::Decided(Some(points)) if points.is_empty() => {
                return Classification::Decided(BezierCurveRelation::NoIntersection);
            }
            Classification::Decided(Some(points)) => {
                return Classification::Decided(BezierCurveRelation::IntersectionPoints { points });
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        },
        (Some(line), None) => {
            match line_image_polynomial_intersections(line, polynomial_controls, policy) {
                Classification::Decided(Some(points)) if points.is_empty() => {
                    return Classification::Decided(BezierCurveRelation::NoIntersection);
                }
                Classification::Decided(Some(points)) => {
                    return Classification::Decided(BezierCurveRelation::IntersectionPoints {
                        points,
                    });
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
        (None, None) => {}
    }

    for a in [rational.start(), rational.end()] {
        for b in [
            polynomial_controls[0],
            polynomial_controls[polynomial_controls.len() - 1],
        ] {
            match point_equal(a, b, policy) {
                Some(true) => return Classification::Decided(BezierCurveRelation::SharedEndpoint),
                Some(false) => {}
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
    }

    match rational_polynomial_endpoint_intersections(rational, polynomial_controls, policy) {
        Classification::Decided(points) if !points.is_empty() => {
            return Classification::Decided(BezierCurveRelation::EndpointIntersections { points });
        }
        Classification::Decided(_) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    match same_parameter_dyadic_rational_polynomial_relation(rational, polynomial_controls, policy)
    {
        Classification::Decided(Some(relation)) => return Classification::Decided(relation),
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    if let Some(reason) = deferred_uncertainty {
        return Classification::Uncertain(reason);
    }

    if rational.weights_known_positive(policy) == Some(true) {
        return isolate_curve_regions(
            RationalSubdivisionNode::from_rational(rational),
            RationalSubdivisionNode::from_polynomial(polynomial_controls),
            policy,
        );
    }

    Classification::Decided(BezierCurveRelation::Unresolved)
}

const RATIONAL_POLYNOMIAL_DYADIC_CANDIDATE_DENOMINATOR: i16 = 512;

fn same_parameter_dyadic_rational_polynomial_relation(
    rational: &RationalQuadraticBezier2,
    polynomial_controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<Option<BezierCurveRelation>> {
    let denominator = Real::from(RATIONAL_POLYNOMIAL_DYADIC_CANDIDATE_DENOMINATOR);
    let mut points = Vec::new();
    for numerator in 1..RATIONAL_POLYNOMIAL_DYADIC_CANDIDATE_DENOMINATOR {
        let parameter = (Real::from(numerator) / &denominator)
            .expect("division by positive dyadic-grid denominator is defined");
        let rational_point = match rational.point_at(parameter.clone(), policy) {
            Classification::Decided(point) => point,
            // Denominator-boundary candidates are not promoted. The caller keeps
            // the conservative subdivision/uncertainty path for topology.
            Classification::Uncertain(_) => continue,
        };
        let polynomial_point = polynomial_point_at(polynomial_controls, parameter);
        match point_coordinates_equal(&rational_point, &polynomial_point, policy) {
            Some(true) => push_unique_intersection_point(&mut points, rational_point, policy),
            Some(false) => {}
            // The finite grid is an opportunistic promotion pass, not a
            // no-intersection proof. If a non-hit candidate cannot be signed
            // cheaply, keep scanning and let the caller's conservative fallback
            // cover the unresolved topology.
            None => {}
        }
    }

    if points.is_empty() {
        Classification::Decided(None)
    } else {
        // This finite same-parameter grid is the conic/polynomial analogue of
        // the low-degree dyadic promotions used for polynomial Beziers. It
        // follows Yap's exact-geometric-computation boundary by promoting only
        // points that survive exact homogeneous rational evaluation and exact
        // de Casteljau polynomial evaluation; see Yap, "Towards Exact Geometric
        // Computation," Computational Geometry 7.1-2 (1997). The homogeneous
        // rational evaluation is Farin's rational Bezier form, *Curves and
        // Surfaces for CAGD* (5th ed., 2002), while the subdivision/evaluation
        // model is de Casteljau's algorithm (1959).
        Classification::Decided(Some(BezierCurveRelation::IntersectionPoints { points }))
    }
}

fn polynomial_relation_for_equal_weight_rationals(
    first: &RationalQuadraticBezier2,
    second: &RationalQuadraticBezier2,
    policy: &CurvePolicy,
) -> Classification<Option<BezierCurveRelation>> {
    match (first.weights_equal(policy), second.weights_equal(policy)) {
        (Some(true), Some(true)) => {
            let first_polynomial = polynomial_quadratic_from_rational_controls(first);
            let second_polynomial = polynomial_quadratic_from_rational_controls(second);
            // Equal Bernstein weights cancel from `sum B_i(t) w P_i /
            // sum B_i(t) w`, so the rational segment is exactly the
            // polynomial Bezier with the same Euclidean controls. This is the
            // homogeneous-to-affine reduction described by Farin, *Curves and
            // Surfaces for CAGD* (5th ed., 2002). Per Yap's EGC model, we
            // preserve that object identity and delegate to the polynomial
            // predicate surface instead of pushing an already-polynomial curve
            // through conservative conic subdivision; see Yap, "Towards Exact
            // Geometric Computation," Computational Geometry 7.1-2 (1997).
            first_polynomial
                .relation_to_quadratic(&second_polynomial, policy)
                .map(Some)
        }
        (Some(false), _) | (_, Some(false)) => Classification::Decided(None),
        _ => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn polynomial_relation_for_equal_weight_rational(
    rational: &RationalQuadraticBezier2,
    polynomial_controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<Option<BezierCurveRelation>> {
    match rational.weights_equal(policy) {
        Some(true) => {
            let polynomial = polynomial_quadratic_from_rational_controls(rational);
            match polynomial_controls {
                [start, control, end] => {
                    let other =
                        QuadraticBezier2::new((*start).clone(), (*control).clone(), (*end).clone());
                    polynomial.relation_to_quadratic(&other, policy).map(Some)
                }
                [start, control1, control2, end] => {
                    let other = CubicBezier2::new(
                        (*start).clone(),
                        (*control1).clone(),
                        (*control2).clone(),
                        (*end).clone(),
                    );
                    polynomial.relation_to_cubic(&other, policy).map(Some)
                }
                _ => Classification::Uncertain(UncertaintyReason::Unsupported),
            }
        }
        Some(false) => Classification::Decided(None),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn polynomial_quadratic_from_rational_controls(
    curve: &RationalQuadraticBezier2,
) -> QuadraticBezier2 {
    QuadraticBezier2::new(
        curve.start().clone(),
        curve.control().clone(),
        curve.end().clone(),
    )
}

fn rational_line_segment_image(
    curve: &RationalQuadraticBezier2,
    policy: &CurvePolicy,
) -> Classification<Option<LineSeg2>> {
    if curve.weights_known_positive(policy) != Some(true) {
        return Classification::Decided(None);
    }
    line_segment_image_from_controls(&curve.control_points(), policy)
}

fn line_segment_image_from_controls(
    controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<Option<LineSeg2>> {
    // Positive-weight rational Beziers preserve the Euclidean convex-hull
    // property, while collinear Bernstein controls keep the image on the
    // supporting line; see Farin, Curves and Surfaces for CAGD (2002). When
    // every interior control is also inside the endpoint box, the segment image
    // is exactly the endpoint line segment, so Yap's EGC boundary lets us
    // dispatch to the native exact line-line predicate instead of subdividing
    // an already linear object; see Yap, "Towards Exact Geometric
    // Computation" (1997).
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

fn line_segment_intersection_relation(
    first: &LineSeg2,
    second: &LineSeg2,
    policy: &CurvePolicy,
) -> Classification<BezierCurveRelation> {
    match first.intersect_line(second, policy) {
        Ok(intersection) => {
            Classification::Decided(BezierCurveRelation::LineSegmentIntersection { intersection })
        }
        Err(CurveError::Real(_)) => Classification::Uncertain(UncertaintyReason::RealSign),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

fn rational_rational_endpoint_intersections(
    first: &RationalQuadraticBezier2,
    second: &RationalQuadraticBezier2,
    policy: &CurvePolicy,
) -> Classification<Vec<BezierCurveIntersectionPoint>> {
    let mut points = Vec::new();
    for endpoint in [first.start(), first.end()] {
        match second.parameters_for_point(endpoint, policy) {
            Classification::Decided(parameters) if !parameters.is_empty() => {
                push_unique_intersection_point(&mut points, endpoint.clone(), policy);
            }
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    for endpoint in [second.start(), second.end()] {
        match first.parameters_for_point(endpoint, policy) {
            Classification::Decided(parameters) if !parameters.is_empty() => {
                push_unique_intersection_point(&mut points, endpoint.clone(), policy);
            }
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(points)
}

fn same_parameter_matching_weight_rational_relation(
    first: &RationalQuadraticBezier2,
    second: &RationalQuadraticBezier2,
    policy: &CurvePolicy,
) -> Classification<Option<BezierCurveRelation>> {
    match matching_rational_weights(first, second, policy) {
        Some(true) => {}
        Some(false) | None => return Classification::Decided(None),
    }

    let x_roots = match matching_weight_axis_difference_root_set(first, second, Axis2::X, policy) {
        Classification::Decided(Some(roots)) => roots,
        Classification::Decided(None) => return Classification::Decided(None),
        Classification::Uncertain(_) => return Classification::Decided(None),
    };
    let y_roots = match matching_weight_axis_difference_root_set(first, second, Axis2::Y, policy) {
        Classification::Decided(Some(roots)) => roots,
        Classification::Decided(None) => return Classification::Decided(None),
        Classification::Uncertain(_) => return Classification::Decided(None),
    };

    let candidates = match same_parameter_candidates_from_root_sets(x_roots, y_roots, policy) {
        Classification::Decided(candidates) => candidates,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let mut points = Vec::new();
    for parameter in candidates {
        let first_point = match first.point_at(parameter.clone(), policy) {
            Classification::Decided(point) => point,
            Classification::Uncertain(_) => return Classification::Decided(None),
        };
        let second_point = match second.point_at(parameter, policy) {
            Classification::Decided(point) => point,
            Classification::Uncertain(_) => return Classification::Decided(None),
        };
        match point_equal(&first_point, &second_point, policy) {
            Some(true) => push_unique_intersection_point(&mut points, first_point, policy),
            Some(false) => {}
            None => return Classification::Decided(None),
        }
    }

    if points.is_empty() {
        Classification::Decided(None)
    } else {
        Classification::Decided(Some(BezierCurveRelation::IntersectionPoints { points }))
    }
}

fn matching_rational_weights(
    first: &RationalQuadraticBezier2,
    second: &RationalQuadraticBezier2,
    policy: &CurvePolicy,
) -> Option<bool> {
    first
        .weights()
        .iter()
        .zip(second.weights().iter())
        .map(|(a, b)| is_zero(&(*a - *b), policy))
        .try_fold(true, |same, item| item.map(|item| same && item))
}

fn matching_weight_axis_difference_root_set(
    first: &RationalQuadraticBezier2,
    second: &RationalQuadraticBezier2,
    axis: Axis2,
    policy: &CurvePolicy,
) -> Classification<Option<RationalPointRootSet>> {
    let first_controls = first.control_points();
    let second_controls = second.control_points();
    let weights = first.weights();
    let values = [
        weights[0] * &(coordinate(first_controls[0], axis) - coordinate(second_controls[0], axis)),
        weights[1] * &(coordinate(first_controls[1], axis) - coordinate(second_controls[1], axis)),
        weights[2] * &(coordinate(first_controls[2], axis) - coordinate(second_controls[2], axis)),
    ];
    if values
        .iter()
        .all(|value| is_zero(value, policy) == Some(true))
    {
        return Classification::Decided(Some(RationalPointRootSet::All));
    }

    // Matching rational weights give both curves the same Bernstein denominator,
    // so same-parameter equality reduces to zeros of the weighted homogeneous
    // numerator difference. This is Farin's rational Bezier model kept at Yap's
    // certified predicate boundary: exact scalar roots are promoted, while roots
    // outside the current scalar proof surface fall back to conservative
    // subdivision regions; see Farin, *Curves and Surfaces for CAGD* (2002), and
    // Yap, "Towards Exact Geometric Computation," Computational Geometry 7.1-2
    // (1997).
    let (c0, c1, c2) = quadratic_bernstein_to_power(values);
    match polynomial_roots_in_unit_interval(c0, c1, c2, policy) {
        Classification::Decided(roots) => {
            Classification::Decided(Some(RationalPointRootSet::Roots(roots)))
        }
        Classification::Uncertain(_) => Classification::Decided(None),
    }
}

fn same_parameter_candidates_from_root_sets(
    x_roots: RationalPointRootSet,
    y_roots: RationalPointRootSet,
    policy: &CurvePolicy,
) -> Classification<Vec<Real>> {
    let mut candidates = Vec::new();
    match (&x_roots, &y_roots) {
        (RationalPointRootSet::All, RationalPointRootSet::All) => {}
        (RationalPointRootSet::All, RationalPointRootSet::Roots(roots))
        | (RationalPointRootSet::Roots(roots), RationalPointRootSet::All) => {
            candidates.extend(roots.iter().cloned());
        }
        (RationalPointRootSet::Roots(left), RationalPointRootSet::Roots(right)) => {
            for left_root in left {
                if right
                    .iter()
                    .any(|right_root| is_zero(&(left_root - right_root), policy) == Some(true))
                {
                    match push_unique_real(&mut candidates, left_root.clone(), policy) {
                        Classification::Decided(()) => {}
                        Classification::Uncertain(reason) => {
                            return Classification::Uncertain(reason);
                        }
                    }
                }
            }
        }
    }
    Classification::Decided(candidates)
}

fn rational_polynomial_endpoint_intersections(
    rational: &RationalQuadraticBezier2,
    polynomial_controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<Vec<BezierCurveIntersectionPoint>> {
    let mut points = Vec::new();
    for endpoint in [
        polynomial_controls[0],
        polynomial_controls[polynomial_controls.len() - 1],
    ] {
        match rational.parameters_for_point(endpoint, policy) {
            Classification::Decided(parameters) if !parameters.is_empty() => {
                push_unique_intersection_point(&mut points, endpoint.clone(), policy);
            }
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }

    if let [start, control, end] = polynomial_controls {
        let polynomial =
            QuadraticBezier2::new((*start).clone(), (*control).clone(), (*end).clone());
        for endpoint in [rational.start(), rational.end()] {
            match polynomial.parameters_for_point(endpoint, policy) {
                Classification::Decided(parameters) if !parameters.is_empty() => {
                    push_unique_intersection_point(&mut points, endpoint.clone(), policy);
                }
                Classification::Decided(_) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
    }

    Classification::Decided(points)
}

fn line_image_rational_intersections(
    line: &LineSeg2,
    rational: &RationalQuadraticBezier2,
    policy: &CurvePolicy,
) -> Classification<Option<Vec<BezierCurveIntersectionPoint>>> {
    // A line-image curve turns one side of the curve/curve problem into a
    // finite segment containment predicate. The rational side still uses its
    // homogeneous supporting-line numerator, following Farin's rational Bezier
    // incidence formulas, and the candidate promotion remains at Yap's exact
    // predicate boundary instead of using sampled distances.
    match rational.relation_to_line(line, policy) {
        Classification::Decided(BezierLineRelation::ControlHullDisjoint { .. }) => {
            Classification::Decided(Some(Vec::new()))
        }
        Classification::Decided(BezierLineRelation::Intersects { parameters }) => {
            let mut points = Vec::new();
            for parameter in parameters {
                let point = match rational.point_at(parameter, policy) {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                };
                match line.contains_point(&point, policy) {
                    Classification::Decided(true) => {
                        push_unique_intersection_point(&mut points, point, policy);
                    }
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
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

fn line_image_polynomial_intersections(
    line: &LineSeg2,
    controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<Option<Vec<BezierCurveIntersectionPoint>>> {
    // For polynomial quadratics, the supporting-line distance is an exact
    // quadratic Bernstein polynomial. Solving it before falling back to
    // subdivision is the low-degree slice of the Sederberg-Nishita Bezier
    // clipping strategy, kept exact by Yap-style certified containment checks.
    let distances = controls
        .iter()
        .map(|point| orient2d_real_expr(line.start(), line.end(), point))
        .collect::<Vec<_>>();
    if distances
        .iter()
        .all(|value| is_zero(value, policy) == Some(true))
    {
        return Classification::Decided(None);
    }

    let roots = match distances.as_slice() {
        [d0, d1, d2] => {
            let two = Real::from(2_i8);
            let c0 = d0.clone();
            let c1 = &two * &(d1 - d0);
            let c2 = d0 - &(two * d1) + d2;
            polynomial_roots_in_unit_interval(c0, c1, c2, policy)
        }
        // Cubic polynomial line roots can include irreducible algebraic
        // parameters. Keep those behind the existing subdivision-region
        // fallback until the complete algebraic isolator is available.
        [_, _, _, _] => return Classification::Decided(None),
        _ => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    let roots = match roots {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let mut points = Vec::new();
    for root in roots {
        let point = polynomial_point_at(controls, root);
        match line.contains_point(&point, policy) {
            Classification::Decided(true) => {
                push_unique_intersection_point(&mut points, point, policy)
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(Some(points))
}

fn polynomial_point_at(controls: &[&Point2], t: Real) -> Point2 {
    let mut level = controls
        .iter()
        .map(|point| (*point).clone())
        .collect::<Vec<_>>();
    while level.len() > 1 {
        level = level
            .windows(2)
            .map(|pair| pair[0].lerp(&pair[1], t.clone()))
            .collect();
    }
    level
        .pop()
        .expect("polynomial Bezier evaluation requires at least one control")
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

fn push_unique_real(
    values: &mut Vec<Real>,
    value: Real,
    policy: &CurvePolicy,
) -> Classification<()> {
    if values
        .iter()
        .any(|existing| is_zero(&(existing - &value), policy) == Some(true))
    {
        return Classification::Decided(());
    }
    values.push(value);
    Classification::Decided(())
}

fn point_coordinates_equal(a: &Point2, b: &Point2, policy: &CurvePolicy) -> Option<bool> {
    let same_x = is_zero(&(a.x() - b.x()), policy)?;
    let same_y = is_zero(&(a.y() - b.y()), policy)?;
    Some(same_x && same_y)
}

#[derive(Clone, Debug)]
struct RationalSubdivisionNode {
    controls: Vec<Point2>,
    weights: Option<Vec<Real>>,
    span: BezierMonotoneSpan,
}

impl RationalSubdivisionNode {
    fn from_rational(curve: &RationalQuadraticBezier2) -> Self {
        Self {
            controls: curve.control_points().into_iter().cloned().collect(),
            weights: Some(curve.weights().into_iter().cloned().collect()),
            span: BezierMonotoneSpan::new(Real::zero(), Real::one()),
        }
    }

    fn from_polynomial(controls: &[&Point2]) -> Self {
        Self {
            controls: controls.iter().map(|point| (*point).clone()).collect(),
            weights: None,
            span: BezierMonotoneSpan::new(Real::zero(), Real::one()),
        }
    }

    fn with_span(
        controls: Vec<Point2>,
        weights: Option<Vec<Real>>,
        start: Real,
        end: Real,
    ) -> Self {
        Self {
            controls,
            weights,
            span: BezierMonotoneSpan::new(start, end),
        }
    }

    fn control_box(&self, policy: &CurvePolicy) -> Classification<Aabb2> {
        Aabb2::from_points(self.controls.iter(), policy)
    }

    fn split_half(&self, policy: &CurvePolicy) -> Result<(Self, Self), UncertaintyReason> {
        let (left_controls, left_weights, right_controls, right_weights) =
            match self.weights.as_ref() {
                Some(weights) => split_rational_controls_half(&self.controls, weights, policy)?,
                None => {
                    let (left, right) = split_polynomial_controls_half(&self.controls);
                    (left, None, right, None)
                }
            };
        let mid = ((self.span.start() + self.span.end()) / Real::from(2_i8))
            .map_err(|_| UncertaintyReason::Unsupported)?;
        Ok((
            Self::with_span(
                left_controls,
                left_weights,
                self.span.start().clone(),
                mid.clone(),
            ),
            Self::with_span(right_controls, right_weights, mid, self.span.end().clone()),
        ))
    }
}

fn isolate_curve_regions(
    first: RationalSubdivisionNode,
    second: RationalSubdivisionNode,
    policy: &CurvePolicy,
) -> Classification<BezierCurveRelation> {
    let mut regions = Vec::new();
    if let Err(reason) = isolate_curve_regions_recursive(first, second, 0, policy, &mut regions) {
        return Classification::Uncertain(reason);
    }
    if regions.is_empty() {
        Classification::Decided(BezierCurveRelation::Unresolved)
    } else {
        Classification::Decided(BezierCurveRelation::IntersectionRegions { regions })
    }
}

fn isolate_curve_regions_recursive(
    first: RationalSubdivisionNode,
    second: RationalSubdivisionNode,
    depth: usize,
    policy: &CurvePolicy,
    regions: &mut Vec<BezierCurveIntersectionRegion>,
) -> Result<(), UncertaintyReason> {
    // Positive-weight rational Beziers preserve the convex-hull property in
    // Euclidean control space. Subdividing in homogeneous coordinates preserves
    // that rational structure while allowing the same certified hull pruning
    // used by Bezier clipping; see Sederberg and Nishita (1990). Yap's EGC
    // boundary is kept by returning parameter regions instead of toleranced
    // intersection points.
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

    if depth >= 20 {
        regions.push(BezierCurveIntersectionRegion::new(first.span, second.span));
        return Ok(());
    }

    if depth.is_multiple_of(2) {
        let (left, right) = first.split_half(policy)?;
        isolate_curve_regions_recursive(left, second.clone(), depth + 1, policy, regions)?;
        isolate_curve_regions_recursive(right, second, depth + 1, policy, regions)
    } else {
        let (left, right) = second.split_half(policy)?;
        isolate_curve_regions_recursive(first.clone(), left, depth + 1, policy, regions)?;
        isolate_curve_regions_recursive(first, right, depth + 1, policy, regions)
    }
}

type RationalSplit = (
    Vec<Point2>,
    Option<Vec<Real>>,
    Vec<Point2>,
    Option<Vec<Real>>,
);

fn split_rational_controls_half(
    controls: &[Point2],
    weights: &[Real],
    policy: &CurvePolicy,
) -> Result<RationalSplit, UncertaintyReason> {
    let mut levels = vec![homogeneous_controls(controls, weights)];
    while levels.last().map(|level| level.len()).unwrap_or(0) > 1 {
        let previous = levels.last().expect("level exists");
        let next = previous
            .windows(2)
            .map(|pair| midpoint_homogeneous(&pair[0], &pair[1]))
            .collect::<Vec<_>>();
        levels.push(next);
    }

    let left_h = levels
        .iter()
        .map(|level| level[0].clone())
        .collect::<Vec<_>>();
    let right_h = levels
        .iter()
        .rev()
        .map(|level| level[level.len() - 1].clone())
        .collect::<Vec<_>>();
    let (left_controls, left_weights) = project_homogeneous_controls(&left_h, policy)?;
    let (right_controls, right_weights) = project_homogeneous_controls(&right_h, policy)?;
    Ok((
        left_controls,
        Some(left_weights),
        right_controls,
        Some(right_weights),
    ))
}

fn split_polynomial_controls_half(points: &[Point2]) -> (Vec<Point2>, Vec<Point2>) {
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

#[derive(Clone, Debug)]
struct HomogeneousControl {
    x: Real,
    y: Real,
    weight: Real,
}

fn homogeneous_controls(controls: &[Point2], weights: &[Real]) -> Vec<HomogeneousControl> {
    controls
        .iter()
        .zip(weights.iter())
        .map(|(point, weight)| HomogeneousControl {
            x: point.x() * weight,
            y: point.y() * weight,
            weight: weight.clone(),
        })
        .collect()
}

fn midpoint_homogeneous(
    first: &HomogeneousControl,
    second: &HomogeneousControl,
) -> HomogeneousControl {
    HomogeneousControl {
        x: midpoint_real(&first.x, &second.x),
        y: midpoint_real(&first.y, &second.y),
        weight: midpoint_real(&first.weight, &second.weight),
    }
}

fn project_homogeneous_controls(
    controls: &[HomogeneousControl],
    policy: &CurvePolicy,
) -> Result<(Vec<Point2>, Vec<Real>), UncertaintyReason> {
    let mut points = Vec::with_capacity(controls.len());
    let mut weights = Vec::with_capacity(controls.len());
    for control in controls {
        match is_zero(&control.weight, policy) {
            Some(true) => return Err(UncertaintyReason::Boundary),
            Some(false) => {}
            None => return Err(UncertaintyReason::RealSign),
        }
        let x = (&control.x / &control.weight).map_err(|_| UncertaintyReason::Boundary)?;
        let y = (&control.y / &control.weight).map_err(|_| UncertaintyReason::Boundary)?;
        points.push(Point2::new(x, y));
        weights.push(control.weight.clone());
    }
    Ok((points, weights))
}

fn midpoint_point(first: &Point2, second: &Point2) -> Point2 {
    first.lerp(second, half())
}

fn midpoint_real(first: &Real, second: &Real) -> Real {
    ((first + second) / Real::from(2_i8)).expect("division by positive integer constant is defined")
}

fn half() -> Real {
    (Real::one() / Real::from(2_i8)).expect("division by positive integer constant is defined")
}

fn point_equal(a: &Point2, b: &Point2, policy: &CurvePolicy) -> Option<bool> {
    is_zero(&a.distance_squared(b), policy)
}
