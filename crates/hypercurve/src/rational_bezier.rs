//! Rational quadratic Bezier and conic primitives.
//!
//! Rational quadratics are the lowest-degree Bezier representation that can
//! carry non-parabolic conics exactly. The homogeneous evaluation below keeps
//! the polynomial numerator and weight denominator visible instead of
//! flattening to sampled chords, matching the exactness model's exact geometric computation
//! advice to preserve object structure until a certified predicate boundary;
//! The conic weight classifier follows the rational quadratic
//! treatment in the Bernstein and de Casteljau curve model.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign, ZeroKnowledge as ZeroStatus};

use crate::bezier_topology::exact_line_contact_relation_from_bernstein_distances;
use crate::bezier_topology::polynomial_roots_in_unit_interval;
use crate::classify::{
    classify_oriented_line, compare_reals, is_zero, orient2d_real_expr, real_sign,
};
use crate::{
    Aabb2, Axis2, BezierCurveIntersectionPoint, BezierCurveIntersectionRegion, BezierCurveRelation,
    BezierLineContactRelation, BezierLineRelation, BezierMonotoneGraphOrder, BezierMonotoneSpan,
    Classification, CubicBezier2, CurveError, CurvePolicy, LineSeg2, LineSide, Point2,
    QuadraticBezier2, UncertaintyReason,
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
#[derive(Clone, Debug)]
pub struct RationalQuadraticBezier2 {
    start: Point2,
    control: Point2,
    end: Point2,
    start_weight: Real,
    control_weight: Real,
    end_weight: Real,
    common_weight_sign: Option<RealSign>,
}

impl PartialEq for RationalQuadraticBezier2 {
    fn eq(&self, other: &Self) -> bool {
        self.start == other.start
            && self.control == other.control
            && self.end == other.end
            && self.start_weight == other.start_weight
            && self.control_weight == other.control_weight
            && self.end_weight == other.end_weight
    }
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
        Self::try_new_with_common_weight_sign(
            start,
            control,
            end,
            start_weight,
            control_weight,
            end_weight,
            None,
        )
    }

    pub(crate) fn try_new_with_common_weight_sign(
        start: Point2,
        control: Point2,
        end: Point2,
        start_weight: Real,
        control_weight: Real,
        end_weight: Real,
        retained_common_weight_sign: Option<RealSign>,
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
        let classified_common_weight_sign = common_weight_sign_for_values(
            [&start_weight, &control_weight, &end_weight],
            &CurvePolicy::certified(),
        );
        debug_assert!(
            classified_common_weight_sign.is_none()
                || retained_common_weight_sign.is_none()
                || classified_common_weight_sign == retained_common_weight_sign,
            "retained rational weight sign contradicts classified weights"
        );
        Ok(Self {
            start,
            control,
            end,
            start_weight,
            control_weight,
            end_weight,
            common_weight_sign: classified_common_weight_sign.or(retained_common_weight_sign),
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

    pub(crate) fn common_nonzero_weight_sign(&self, policy: &CurvePolicy) -> Option<RealSign> {
        self.common_weight_sign
            .or_else(|| common_weight_sign_for_values(self.weights(), policy))
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
        if self.start_weight == Real::one()
            && self.end_weight == Real::one()
            && let Some(point) =
                self.point_at_unit_end_weights_rationalized(&t, &one_minus_t, &two, policy)
        {
            return point;
        }
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

    fn point_at_unit_end_weights_rationalized(
        &self,
        t: &Real,
        one_minus_t: &Real,
        two: &Real,
        policy: &CurvePolicy,
    ) -> Option<Classification<Point2>> {
        let u_squared = one_minus_t * one_minus_t;
        let t_squared = t * t;
        let unweighted = &u_squared + &t_squared;
        let middle_basis = two * one_minus_t * t;
        let weight_squared = &self.control_weight * &self.control_weight;
        let conjugate_denominator =
            (&unweighted * &unweighted) - (&middle_basis * &middle_basis * &weight_squared);
        if is_zero(&conjugate_denominator, policy) != Some(false) {
            return None;
        }

        let coordinate = |start: &Real, control: &Real, end: &Real| {
            let unweighted_numerator = (&u_squared * start) + (&t_squared * end);
            let weighted_control = &middle_basis * control;
            let rational_part = (&unweighted_numerator * &unweighted)
                - (&weighted_control * &middle_basis * &weight_squared);
            let radical_part = ((&weighted_control * &unweighted)
                - (&unweighted_numerator * &middle_basis))
                * &self.control_weight;
            (rational_part + radical_part) / &conjugate_denominator
        };
        let Ok(x) = coordinate(self.start.x(), self.control.x(), self.end.x()) else {
            return Some(Classification::Uncertain(UncertaintyReason::Boundary));
        };
        let Ok(y) = coordinate(self.start.y(), self.control.y(), self.end.y()) else {
            return Some(Classification::Uncertain(UncertaintyReason::Boundary));
        };
        Some(Classification::Decided(Point2::new(x, y)))
    }

    /// Classifies whether `point` equals this conic at parameter `t`.
    ///
    /// This is a parameterized predicate rather than an existential conic
    /// solve. It first evaluates the homogeneous rational point, then certifies
    /// affine equality through the active curve policy. Returning uncertainty
    /// at denominator boundaries keeps projective singularities explicit in
    /// the style advocated by the exactness model's exact geometric computation model.
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
    /// re-evaluating the conic. That follows the exactness model's requirement to keep exact
    /// geometric objects explicit until a predicate boundary. The weighted Bernstein numerator/denominator identities follow
    /// the rational Bezier treatment in the Bernstein and de Casteljau curve model.
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
    /// the line-incidence predicates described by the Bernstein curve model, with branch
    /// decisions routed through exact scalar signs per the exactness model.
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
            Classification::Uncertain(reason) => {
                if self.weights_known_same_nonzero_sign(policy) == Some(true) {
                    return isolate_rational_quadratic_line_roots(
                        [
                            weighted_distances[0].clone(),
                            weighted_distances[1].clone(),
                            weighted_distances[2].clone(),
                        ],
                        policy,
                    );
                }
                return Classification::Uncertain(reason);
            }
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

        if self.weights_known_same_nonzero_sign(policy) == Some(true) {
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

    /// Classifies exact conic/supporting-line contacts as crossings or tangencies.
    ///
    /// The signed affine line predicate for a rational quadratic is the
    /// weighted Bernstein numerator `sum B_i(t) w_i orient(line, P_i)`. A root
    /// becomes a finite conic contact only when the homogeneous denominator is
    /// certified nonzero at the same parameter; denominator zeros remain
    /// explicit projective-boundary uncertainty. Contacts retain represented or
    /// algebraically isolated parameters and are labelled from exact root
    /// multiplicity parity. This follows
    /// the exactness model's exact geometric computation boundary. The
    /// rational Bezier numerator/denominator identities are from the Bernstein and de Casteljau curve model.
    pub fn relation_to_line_with_contacts(
        &self,
        line: &LineSeg2,
        policy: &CurvePolicy,
    ) -> Classification<BezierLineContactRelation> {
        match self.weights_known_same_nonzero_sign(policy) {
            Some(true) => {}
            Some(false) => return Classification::Uncertain(UncertaintyReason::Boundary),
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
        let weighted_distances = self.weighted_line_distances(line);
        if weighted_distances
            .iter()
            .all(|value| is_zero(value, policy) == Some(true))
        {
            return Classification::Decided(BezierLineContactRelation::OnSupportingLine);
        }
        for side in [LineSide::Left, LineSide::Right] {
            if self.control_points().iter().all(|point| {
                matches!(
                    classify_oriented_line(line.start(), line.end(), point, policy),
                    Classification::Decided(candidate) if candidate == side
                )
            }) {
                return Classification::Decided(BezierLineContactRelation::ControlHullDisjoint {
                    side,
                });
            }
        }
        exact_line_contact_relation_from_bernstein_distances(weighted_distances.to_vec(), policy)
    }

    /// Returns quotient-derivative roots that split this conic into monotone spans.
    ///
    /// For a rational coordinate `N(t) / D(t)`, extrema occur where
    /// `N'(t)D(t) - N(t)D'(t) = 0` and `D(t) != 0`. The cubic terms cancel for
    /// rational quadratics, leaving an exact quadratic root problem. This keeps
    /// the homogeneous numerator/denominator visible as recommended by the exactness model
    ///, and follows the rational Bezier derivative identity in the Bernstein curve model
    ///.
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
    /// Non-equal same-sign weights also include the Euclidean control point as
    /// a conservative hull witness. A uniformly negative homogeneous lift is
    /// sign-normalized to the positive case, preserving the rational Bezier
    /// convex-hull guarantee when algebraic extrema are present while still
    /// avoiding topology decisions from approximate samples.
    pub fn certified_bounds(&self, policy: &CurvePolicy) -> Classification<Aabb2> {
        let mut samples = vec![self.start.clone(), self.end.clone()];
        if self.weights_known_same_nonzero_sign(policy) == Some(true)
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
    /// homogeneous-control identity, same-sign convex-hull disjointness,
    /// certified point and endpoint line-segment images, and shared endpoints, then
    /// leaves overlapping boxes unresolved for a later resultant or subdivision
    /// predicate. Keeping that boundary explicit follows the exactness model's
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
        match self.same_reversed_projective_homogeneous_controls(other, policy) {
            Some(true) => return Classification::Decided(BezierCurveRelation::SameCurveImage),
            Some(false) => {}
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }

        match polynomial_relation_for_equal_weight_rationals(self, other, policy) {
            Classification::Decided(Some(relation)) => return Classification::Decided(relation),
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }

        if self.weights_known_same_nonzero_sign(policy) == Some(true)
            && other.weights_known_same_nonzero_sign(policy) == Some(true)
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
            rational_point_image(self, policy),
            rational_point_image(other, policy),
        ) {
            (Classification::Decided(Some(first)), Classification::Decided(Some(second))) => {
                match point_equal(&first, &second, policy) {
                    Some(true) => {
                        return Classification::Decided(BezierCurveRelation::IntersectionPoints {
                            points: vec![BezierCurveIntersectionPoint::new(first)],
                        });
                    }
                    Some(false) => {
                        return Classification::Decided(BezierCurveRelation::NoIntersection);
                    }
                    None => return Classification::Uncertain(UncertaintyReason::RealSign),
                }
            }
            (Classification::Decided(Some(point)), Classification::Decided(None)) => {
                match point_image_rational_intersections(&point, other, policy) {
                    Classification::Decided(points) if points.is_empty() => {
                        return Classification::Decided(BezierCurveRelation::NoIntersection);
                    }
                    Classification::Decided(points) => {
                        return Classification::Decided(BezierCurveRelation::IntersectionPoints {
                            points,
                        });
                    }
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
            (Classification::Decided(None), Classification::Decided(Some(point))) => {
                match point_image_rational_intersections(&point, self, policy) {
                    Classification::Decided(points) if points.is_empty() => {
                        return Classification::Decided(BezierCurveRelation::NoIntersection);
                    }
                    Classification::Decided(points) => {
                        return Classification::Decided(BezierCurveRelation::IntersectionPoints {
                            points,
                        });
                    }
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
            (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            _ => {}
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

        let mut shared_endpoint_points = Vec::new();
        for a in [self.start(), self.end()] {
            for b in [other.start(), other.end()] {
                match point_equal(a, b, policy) {
                    Some(true) => push_unique_intersection_point(
                        &mut shared_endpoint_points,
                        a.clone(),
                        policy,
                    ),
                    Some(false) => {}
                    None => return Classification::Uncertain(UncertaintyReason::RealSign),
                }
            }
        }

        let endpoint_points = match rational_rational_endpoint_intersections(self, other, policy) {
            Classification::Decided(points) => points,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };

        match same_parameter_matching_weight_rational_relation(self, other, policy) {
            Classification::Decided(Some(relation)) => {
                return Classification::Decided(merge_endpoint_points_into_relation(
                    relation,
                    &endpoint_points,
                    policy,
                ));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }

        if !endpoint_points.is_empty() {
            return if endpoint_points_are_shared_only(
                &endpoint_points,
                &shared_endpoint_points,
                policy,
            ) {
                Classification::Decided(BezierCurveRelation::SharedEndpoint)
            } else {
                Classification::Decided(BezierCurveRelation::EndpointIntersections {
                    points: endpoint_points,
                })
            };
        }

        if self.weights_known_same_nonzero_sign(policy) == Some(true)
            && other.weights_known_same_nonzero_sign(policy) == Some(true)
        {
            let first = match RationalSubdivisionNode::from_rational(self) {
                Ok(node) => node,
                Err(reason) => return Classification::Uncertain(reason),
            };
            let second = match RationalSubdivisionNode::from_rational(other) {
                Ok(node) => node,
                Err(reason) => return Classification::Uncertain(reason),
            };
            return isolate_curve_regions(first, second, policy);
        }

        Classification::Decided(BezierCurveRelation::Unresolved)
    }

    /// Classifies graph order against another matching-weight conic over one shared axis.
    ///
    /// This is the rational quadratic analogue of polynomial Bezier graph
    /// ordering. It first certifies that both conics share the same
    /// homogeneous weights, that those weights have one nonzero sign, that the
    /// requested Euclidean coordinate is identical, and that this coordinate
    /// is strictly monotone. The remaining coordinate then reduces to one
    /// weighted Bernstein numerator over the common denominator. Strict order
    /// is sign-normalized by the denominator sign, so uniformly negative
    /// homogeneous weights report the same Euclidean order as their positive
    /// projective image. This follows the exactness model's exact geometric computation
    /// boundary. The rational homogeneous Bezier
    /// model is the Bernstein and de Casteljau curve model, and
    /// retained crossing brackets use Bezier clipping.
    pub fn graph_order_to_rational_quadratic_over_axis(
        &self,
        other: &RationalQuadraticBezier2,
        shared_axis: Axis2,
        policy: &CurvePolicy,
    ) -> Classification<BezierMonotoneGraphOrder> {
        matching_weight_rational_graph_order(self, other, shared_axis, policy)
    }

    /// Classifies graph order against a polynomial quadratic.
    ///
    /// Equal homogeneous weights collapse a rational quadratic Bezier to the
    /// polynomial quadratic with the same Euclidean controls. This method
    /// exposes that exact collapse as the mixed conic/polynomial graph-order
    /// bridge, then delegates to the polynomial predicate. For non-equal
    /// same-sign weights, it also certifies strict graph order when one
    /// coordinate is exactly shared and the degree-4 homogeneous Bernstein
    /// numerator for the remaining coordinate has one strict sign. Non-strict
    /// mixed quartic roots are retained as represented parameters or isolating
    /// spans instead of being sampled. This preserves the exactness model's
    /// exact geometric computation boundary. The
    /// rational Bezier identities are from the Bernstein and de Casteljau curve model, and the Bernstein sign argument is the
    /// Bezier clipping criterion.
    pub fn graph_order_to_quadratic_over_axis(
        &self,
        other: &QuadraticBezier2,
        shared_axis: Axis2,
        policy: &CurvePolicy,
    ) -> Classification<BezierMonotoneGraphOrder> {
        match self.equal_weight_polynomial_quadratic_image(policy) {
            Classification::Decided(Some(curve)) => {
                collapsed_graph_order_to_quadratic(&curve, other, shared_axis, policy)
            }
            Classification::Decided(None) => {
                self.rational_quadratic_graph_order_to_quadratic(other, shared_axis, policy)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Classifies graph order against a polynomial cubic.
    ///
    /// Equal same-sign nonzero weights certify that the rational conic is
    /// exactly the polynomial quadratic with the same Euclidean control
    /// points, so the predicate delegates to the degree-normalized polynomial
    /// quadratic/cubic graph order. For non-equal same-sign weights, this also
    /// certifies strict order when one coordinate is exactly shared and the
    /// degree-5 homogeneous Bernstein numerator for the remaining coordinate
    /// has one strict sign. Non-strict mixed quintic roots are retained as
    /// represented parameters or isolating spans. The branch boundary follows
    /// exact-computation discipline;
    /// the rational identities follow the Bernstein and de Casteljau curve model, and strict Bernstein sign exclusion follows
    /// Bezier clipping.
    pub fn graph_order_to_cubic_over_axis(
        &self,
        other: &CubicBezier2,
        shared_axis: Axis2,
        policy: &CurvePolicy,
    ) -> Classification<BezierMonotoneGraphOrder> {
        match self.equal_weight_polynomial_quadratic_image(policy) {
            Classification::Decided(Some(curve)) => {
                curve.graph_order_to_cubic_over_axis(other, shared_axis, policy)
            }
            Classification::Decided(None) => {
                self.rational_quadratic_graph_order_to_cubic(other, shared_axis, policy)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Classifies a coarse relation between this conic and a polynomial quadratic.
    ///
    /// A same-sign rational Bezier segment sign-normalizes to the positive
    /// case and lies in the convex hull of its Euclidean control points, so a
    /// disjoint hull box is a certified miss. This is the convex-hull predicate
    /// for rational Beziers described by the Bernstein curve model, used only when exact
    /// signs prove the weights share one nonzero sign in the EGC sense of the exactness model
    ///. Equal weights collapse the homogeneous conic to the polynomial
    /// quadratic with the same control polygon.
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

    fn weighted_line_distances(&self, line: &LineSeg2) -> [Real; 3] {
        let controls = self.control_points();
        let weights = self.weights();
        [
            orient2d_real_expr(line.start(), line.end(), controls[0]) * weights[0],
            orient2d_real_expr(line.start(), line.end(), controls[1]) * weights[1],
            orient2d_real_expr(line.start(), line.end(), controls[2]) * weights[2],
        ]
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

    fn weights_known_same_nonzero_sign(&self, policy: &CurvePolicy) -> Option<bool> {
        if self.common_weight_sign.is_some() {
            return Some(true);
        }
        let mut expected = None;
        for weight in self.weights() {
            let sign = real_sign(weight, policy)?;
            match sign {
                RealSign::Positive | RealSign::Negative => {
                    if let Some(expected) = expected {
                        if sign != expected {
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
        // of division, preserving the exactness model's exact predicate boundary; the
        // homogeneous rational Bezier model is the one described by the Bernstein and de Casteljau curve model.
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

    fn same_reversed_projective_homogeneous_controls(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Option<bool> {
        let same_reversed_points = self
            .control_points()
            .iter()
            .zip(other.control_points().iter().rev())
            .map(|(a, b)| point_equal(a, b, policy))
            .try_fold(true, |same, item| item.map(|item| same && item))?;
        if !same_reversed_points {
            return Some(false);
        }

        // Reversing Bernstein controls represents the same rational image with
        // parameter `1 - t`; multiplying every homogeneous weight by one common
        // nonzero scalar is projectively invisible. Exact cross-products test
        // that reversed proportionality without division, preserving the exactness model's
        // certified predicate boundary. The rational Bernstein and reversal
        // identities are the standard homogeneous Bezier identities in the Bernstein and de Casteljau curve model.
        let first = self.weights();
        let second = other.weights();
        [
            first[0] * second[1] - &(first[1] * second[2]),
            first[1] * second[0] - &(first[2] * second[1]),
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

    fn equal_weight_polynomial_quadratic_image(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<Option<QuadraticBezier2>> {
        equal_weight_polynomial_quadratic_image(self, policy)
    }

    fn rational_quadratic_graph_order_to_quadratic(
        &self,
        other: &QuadraticBezier2,
        shared_axis: Axis2,
        policy: &CurvePolicy,
    ) -> Classification<BezierMonotoneGraphOrder> {
        rational_quadratic_graph_order_to_quadratic(self, other, shared_axis, policy)
    }

    fn rational_quadratic_graph_order_to_cubic(
        &self,
        other: &CubicBezier2,
        shared_axis: Axis2,
        policy: &CurvePolicy,
    ) -> Classification<BezierMonotoneGraphOrder> {
        rational_quadratic_graph_order_to_cubic(self, other, shared_axis, policy)
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

#[derive(Clone, Debug, PartialEq)]
enum RationalQuadraticRootCover {
    All,
    Isolated {
        exact: Vec<Real>,
        spans: Vec<BezierMonotoneSpan>,
    },
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

fn equal_weight_polynomial_quadratic_image(
    rational: &RationalQuadraticBezier2,
    policy: &CurvePolicy,
) -> Classification<Option<QuadraticBezier2>> {
    match rational.weights_equal(policy) {
        Some(true) => {}
        Some(false) => return Classification::Decided(None),
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    }
    match rational.weights_known_same_nonzero_sign(policy) {
        Some(true) => {}
        Some(false) => return Classification::Decided(None),
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    }

    let controls = rational.control_points();
    Classification::Decided(Some(QuadraticBezier2::new(
        controls[0].clone(),
        controls[1].clone(),
        controls[2].clone(),
    )))
}

fn collapsed_graph_order_to_quadratic(
    collapsed: &QuadraticBezier2,
    other: &QuadraticBezier2,
    shared_axis: Axis2,
    policy: &CurvePolicy,
) -> Classification<BezierMonotoneGraphOrder> {
    collapsed.graph_order_to_quadratic_over_axis(other, shared_axis, policy)
}

fn rational_quadratic_graph_order_to_quadratic(
    rational: &RationalQuadraticBezier2,
    polynomial: &QuadraticBezier2,
    shared_axis: Axis2,
    policy: &CurvePolicy,
) -> Classification<BezierMonotoneGraphOrder> {
    // For non-equal weights, the supported mixed graph shortcut is deliberately
    // narrower than a full rational/polynomial intersection solver. If one
    // coordinate is certified identical, ordering of the other coordinate is
    // the sign of `(N_rational - P_polynomial * D_rational) / D_rational`.
    // The numerator is a degree-4 Bernstein polynomial. A common strict sign
    // certifies order; otherwise exact Bernstein sign subdivision retains the
    // graph roots as represented parameters or isolating spans. This is the exactness model's
    // EGC boundary applied to the Bernstein curve model's homogeneous rational Bezier model, using
    // Bernstein convex-hull signs as in Bezier clipping.
    match rational.weights_known_same_nonzero_sign(policy) {
        Some(true) => {}
        Some(false) => {
            return Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone);
        }
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    }
    let Some(weight_sign) = common_rational_weight_sign(rational, policy) else {
        return Classification::Uncertain(UncertaintyReason::RealSign);
    };

    let shared_difference = match rational_polynomial_axis_difference_degree4_controls(
        rational,
        polynomial,
        shared_axis,
    ) {
        Ok(difference) => difference,
        Err(reason) => return Classification::Uncertain(reason),
    };
    match degree4_controls_all_zero(&shared_difference, policy) {
        Classification::Decided(true) => {}
        Classification::Decided(false) => {
            return Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone);
        }
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }
    match rational_axis_strictly_monotone(rational, shared_axis, policy) {
        Classification::Decided(true) => {}
        Classification::Decided(false) => {
            return Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone);
        }
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    let order_axis = other_axis(shared_axis);
    let order_difference = match rational_polynomial_axis_difference_degree4_controls(
        rational, polynomial, order_axis,
    ) {
        Ok(difference) => difference,
        Err(reason) => return Classification::Uncertain(reason),
    };
    match degree4_controls_all_zero(&order_difference, policy) {
        Classification::Decided(true) => {
            return Classification::Decided(BezierMonotoneGraphOrder::Coincident);
        }
        Classification::Decided(false) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }
    if let Some(order) = strict_rational_graph_order_from_degree4_weighted_signs(
        &order_difference,
        weight_sign,
        policy,
    ) {
        return Classification::Decided(order);
    }

    match degree4_root_cover(order_difference.clone(), policy) {
        Ok(RationalPolynomialRootCover::All) => {
            Classification::Decided(BezierMonotoneGraphOrder::Coincident)
        }
        Ok(RationalPolynomialRootCover::Isolated { exact, spans }) => {
            if exact.is_empty() && spans.is_empty() {
                match real_sign(&order_difference[0], policy) {
                    Some(RealSign::Positive | RealSign::Negative) => {
                        strict_rational_graph_order_from_degree4_weighted_signs(
                            &order_difference,
                            weight_sign,
                            policy,
                        )
                        .map_or(
                            Classification::Uncertain(UncertaintyReason::RealSign),
                            Classification::Decided,
                        )
                    }
                    Some(RealSign::Zero) | None => {
                        Classification::Uncertain(UncertaintyReason::RealSign)
                    }
                }
            } else {
                Classification::Decided(BezierMonotoneGraphOrder::IntersectsOrTouches {
                    parameters: exact,
                    spans,
                })
            }
        }
        Err(reason) => Classification::Uncertain(reason),
    }
}

fn rational_quadratic_graph_order_to_cubic(
    rational: &RationalQuadraticBezier2,
    polynomial: &CubicBezier2,
    shared_axis: Axis2,
    policy: &CurvePolicy,
) -> Classification<BezierMonotoneGraphOrder> {
    // This is the cubic partner of the degree-4 mixed graph shortcut above.
    // When one coordinate is shared, strict order is the sign of the degree-5
    // Bernstein numerator `(N_rational - P_cubic * D_rational)` divided by the
    // same-sign rational denominator. A common nonzero Bernstein sign is a
    // certified no-root certificate by the convex-hull property; crossings and
    // tangencies are retained by exact quintic sign subdivision rather than
    // collapsed into sampled topology.
    match rational.weights_known_same_nonzero_sign(policy) {
        Some(true) => {}
        Some(false) => {
            return Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone);
        }
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    }
    let Some(weight_sign) = common_rational_weight_sign(rational, policy) else {
        return Classification::Uncertain(UncertaintyReason::RealSign);
    };

    let shared_difference =
        match rational_cubic_axis_difference_degree5_controls(rational, polynomial, shared_axis) {
            Ok(difference) => difference,
            Err(reason) => return Classification::Uncertain(reason),
        };
    match degree5_controls_all_zero(&shared_difference, policy) {
        Classification::Decided(true) => {}
        Classification::Decided(false) => {
            return Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone);
        }
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }
    match rational_axis_strictly_monotone(rational, shared_axis, policy) {
        Classification::Decided(true) => {}
        Classification::Decided(false) => {
            return Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone);
        }
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    let order_axis = other_axis(shared_axis);
    let order_difference =
        match rational_cubic_axis_difference_degree5_controls(rational, polynomial, order_axis) {
            Ok(difference) => difference,
            Err(reason) => return Classification::Uncertain(reason),
        };
    match degree5_controls_all_zero(&order_difference, policy) {
        Classification::Decided(true) => {
            return Classification::Decided(BezierMonotoneGraphOrder::Coincident);
        }
        Classification::Decided(false) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }
    if let Some(order) = strict_rational_graph_order_from_degree5_weighted_signs(
        &order_difference,
        weight_sign,
        policy,
    ) {
        return Classification::Decided(order);
    }

    match degree5_root_cover(order_difference.clone(), policy) {
        Ok(RationalPolynomialRootCover::All) => {
            Classification::Decided(BezierMonotoneGraphOrder::Coincident)
        }
        Ok(RationalPolynomialRootCover::Isolated { exact, spans }) => {
            if exact.is_empty() && spans.is_empty() {
                match real_sign(&order_difference[0], policy) {
                    Some(RealSign::Positive | RealSign::Negative) => {
                        strict_rational_graph_order_from_degree5_weighted_signs(
                            &order_difference,
                            weight_sign,
                            policy,
                        )
                        .map_or(
                            Classification::Uncertain(UncertaintyReason::RealSign),
                            Classification::Decided,
                        )
                    }
                    Some(RealSign::Zero) | None => {
                        Classification::Uncertain(UncertaintyReason::RealSign)
                    }
                }
            } else {
                Classification::Decided(BezierMonotoneGraphOrder::IntersectsOrTouches {
                    parameters: exact,
                    spans,
                })
            }
        }
        Err(reason) => Classification::Uncertain(reason),
    }
}

fn rational_polynomial_axis_difference_degree4_controls(
    rational: &RationalQuadraticBezier2,
    polynomial: &QuadraticBezier2,
    axis: Axis2,
) -> Result<[Real; 5], UncertaintyReason> {
    let rational_controls = rational.control_points();
    let weights = rational.weights();
    let rational_weighted_axis = [
        weights[0] * coordinate(rational_controls[0], axis),
        weights[1] * coordinate(rational_controls[1], axis),
        weights[2] * coordinate(rational_controls[2], axis),
    ];
    let numerator = elevate_quadratic_bernstein_to_quartic(rational_weighted_axis)?;
    let polynomial_controls = polynomial.control_points();
    let polynomial_axis = [
        coordinate(polynomial_controls[0], axis).clone(),
        coordinate(polynomial_controls[1], axis).clone(),
        coordinate(polynomial_controls[2], axis).clone(),
    ];
    let denominator = [weights[0].clone(), weights[1].clone(), weights[2].clone()];
    let product = multiply_quadratic_bernstein_to_quartic(polynomial_axis, denominator)?;
    Ok([
        &numerator[0] - &product[0],
        &numerator[1] - &product[1],
        &numerator[2] - &product[2],
        &numerator[3] - &product[3],
        &numerator[4] - &product[4],
    ])
}

fn rational_cubic_axis_difference_degree5_controls(
    rational: &RationalQuadraticBezier2,
    polynomial: &CubicBezier2,
    axis: Axis2,
) -> Result<[Real; 6], UncertaintyReason> {
    let rational_controls = rational.control_points();
    let weights = rational.weights();
    let rational_weighted_axis = [
        weights[0] * coordinate(rational_controls[0], axis),
        weights[1] * coordinate(rational_controls[1], axis),
        weights[2] * coordinate(rational_controls[2], axis),
    ];
    let numerator = elevate_quadratic_bernstein_to_quintic(rational_weighted_axis)?;
    let polynomial_controls = polynomial.control_points();
    let polynomial_axis = [
        coordinate(polynomial_controls[0], axis).clone(),
        coordinate(polynomial_controls[1], axis).clone(),
        coordinate(polynomial_controls[2], axis).clone(),
        coordinate(polynomial_controls[3], axis).clone(),
    ];
    let denominator = [weights[0].clone(), weights[1].clone(), weights[2].clone()];
    let product = multiply_cubic_quadratic_bernstein_to_quintic(polynomial_axis, denominator)?;
    Ok([
        &numerator[0] - &product[0],
        &numerator[1] - &product[1],
        &numerator[2] - &product[2],
        &numerator[3] - &product[3],
        &numerator[4] - &product[4],
        &numerator[5] - &product[5],
    ])
}

fn elevate_quadratic_bernstein_to_quartic(
    values: [Real; 3],
) -> Result<[Real; 5], UncertaintyReason> {
    Ok([
        values[0].clone(),
        divide_by_positive_integer(&values[0] + &values[1], 2)?,
        divide_by_positive_integer(
            &values[0] + (&Real::from(4_i8) * &values[1]) + &values[2],
            6,
        )?,
        divide_by_positive_integer(&values[1] + &values[2], 2)?,
        values[2].clone(),
    ])
}

fn elevate_quadratic_bernstein_to_quintic(
    values: [Real; 3],
) -> Result<[Real; 6], UncertaintyReason> {
    Ok([
        values[0].clone(),
        divide_by_positive_integer(
            (Real::from(3_i8) * &values[0]) + (Real::from(2_i8) * &values[1]),
            5,
        )?,
        divide_by_positive_integer(
            (Real::from(3_i8) * &values[0]) + (Real::from(6_i8) * &values[1]) + &values[2],
            10,
        )?,
        divide_by_positive_integer(
            &values[0] + (Real::from(6_i8) * &values[1]) + (Real::from(3_i8) * &values[2]),
            10,
        )?,
        divide_by_positive_integer(
            (Real::from(2_i8) * &values[1]) + (Real::from(3_i8) * &values[2]),
            5,
        )?,
        values[2].clone(),
    ])
}

fn multiply_quadratic_bernstein_to_quartic(
    first: [Real; 3],
    second: [Real; 3],
) -> Result<[Real; 5], UncertaintyReason> {
    Ok([
        &first[0] * &second[0],
        divide_by_positive_integer((&first[0] * &second[1]) + (&first[1] * &second[0]), 2)?,
        divide_by_positive_integer(
            (&first[0] * &second[2])
                + (Real::from(4_i8) * &first[1] * &second[1])
                + (&first[2] * &second[0]),
            6,
        )?,
        divide_by_positive_integer((&first[1] * &second[2]) + (&first[2] * &second[1]), 2)?,
        &first[2] * &second[2],
    ])
}

fn multiply_cubic_quadratic_bernstein_to_quintic(
    first: [Real; 4],
    second: [Real; 3],
) -> Result<[Real; 6], UncertaintyReason> {
    Ok([
        &first[0] * &second[0],
        divide_by_positive_integer(
            (Real::from(2_i8) * &first[0] * &second[1])
                + (Real::from(3_i8) * &first[1] * &second[0]),
            5,
        )?,
        divide_by_positive_integer(
            (&first[0] * &second[2])
                + (Real::from(6_i8) * &first[1] * &second[1])
                + (Real::from(3_i8) * &first[2] * &second[0]),
            10,
        )?,
        divide_by_positive_integer(
            (Real::from(3_i8) * &first[1] * &second[2])
                + (Real::from(6_i8) * &first[2] * &second[1])
                + (&first[3] * &second[0]),
            10,
        )?,
        divide_by_positive_integer(
            (Real::from(3_i8) * &first[2] * &second[2])
                + (Real::from(2_i8) * &first[3] * &second[1]),
            5,
        )?,
        &first[3] * &second[2],
    ])
}

fn divide_by_positive_integer(numerator: Real, denominator: i8) -> Result<Real, UncertaintyReason> {
    (numerator / Real::from(denominator)).map_err(|_| UncertaintyReason::Unsupported)
}

fn degree4_controls_all_zero(controls: &[Real; 5], policy: &CurvePolicy) -> Classification<bool> {
    let mut all_zero = true;
    for control in controls {
        match is_zero(control, policy) {
            Some(true) => {}
            Some(false) => all_zero = false,
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }
    Classification::Decided(all_zero)
}

fn degree5_controls_all_zero(controls: &[Real; 6], policy: &CurvePolicy) -> Classification<bool> {
    let mut all_zero = true;
    for control in controls {
        match is_zero(control, policy) {
            Some(true) => {}
            Some(false) => all_zero = false,
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }
    Classification::Decided(all_zero)
}

#[derive(Clone, Debug, PartialEq)]
enum RationalPolynomialRootCover {
    All,
    Isolated {
        exact: Vec<Real>,
        spans: Vec<BezierMonotoneSpan>,
    },
}

fn degree4_root_cover(
    controls: [Real; 5],
    policy: &CurvePolicy,
) -> Result<RationalPolynomialRootCover, UncertaintyReason> {
    match degree4_controls_all_zero(&controls, policy) {
        Classification::Decided(true) => return Ok(RationalPolynomialRootCover::All),
        Classification::Decided(false) => {}
        Classification::Uncertain(reason) => return Err(reason),
    }

    let mut exact = Vec::new();
    let mut spans = Vec::new();
    isolate_scalar_bernstein_roots(
        controls.to_vec(),
        Real::zero(),
        Real::one(),
        0,
        policy,
        &mut exact,
        &mut spans,
    )?;
    Ok(RationalPolynomialRootCover::Isolated { exact, spans })
}

fn degree5_root_cover(
    controls: [Real; 6],
    policy: &CurvePolicy,
) -> Result<RationalPolynomialRootCover, UncertaintyReason> {
    match degree5_controls_all_zero(&controls, policy) {
        Classification::Decided(true) => return Ok(RationalPolynomialRootCover::All),
        Classification::Decided(false) => {}
        Classification::Uncertain(reason) => return Err(reason),
    }

    let mut exact = Vec::new();
    let mut spans = Vec::new();
    isolate_scalar_bernstein_roots(
        controls.to_vec(),
        Real::zero(),
        Real::one(),
        0,
        policy,
        &mut exact,
        &mut spans,
    )?;
    Ok(RationalPolynomialRootCover::Isolated { exact, spans })
}

pub(crate) fn isolate_scalar_bernstein_roots(
    controls: Vec<Real>,
    start: Real,
    end: Real,
    depth: usize,
    policy: &CurvePolicy,
    exact_parameters: &mut Vec<Real>,
    spans: &mut Vec<BezierMonotoneSpan>,
) -> Result<(), UncertaintyReason> {
    // This is the scalar sign-subdivision kernel used for mixed quartic and
    // quintic graph numerators. The control-polygon sign exclusion is the
    // Bezier clipping certificate of Bezier clipping, the
    // midpoint split is de Casteljau/the Bernstein curve model Bernstein subdivision, and the API
    // follows the exactness model by returning exact parameters or explicit brackets.
    debug_assert!(controls.len() >= 2);
    let signs = controls
        .iter()
        .map(|value| real_sign(value, policy).ok_or(UncertaintyReason::RealSign))
        .collect::<Result<Vec<_>, _>>()?;

    if signs[0] == RealSign::Zero {
        push_unique_graph_parameter(exact_parameters, start.clone(), policy)?;
    }
    if signs[signs.len() - 1] == RealSign::Zero {
        push_unique_graph_parameter(exact_parameters, end.clone(), policy)?;
    }

    let strict_signs = signs
        .iter()
        .copied()
        .filter(|sign| *sign != RealSign::Zero)
        .collect::<Vec<_>>();
    if strict_signs.is_empty() {
        push_unique_graph_region_span(
            spans,
            BezierMonotoneSpan::new(start, end).map_err(|_| UncertaintyReason::Ordering)?,
            policy,
        )?;
        return Ok(());
    }
    if strict_signs.iter().all(|sign| *sign == strict_signs[0]) {
        return Ok(());
    }

    let mid = ((&start + &end) / Real::from(2_i8)).map_err(|_| UncertaintyReason::Unsupported)?;
    let (left, right) = subdivide_scalar_bernstein_half(&controls)?;
    if is_zero(&left[left.len() - 1], policy) == Some(true) {
        push_unique_graph_parameter(exact_parameters, mid.clone(), policy)?;
    }

    if depth >= 32 {
        push_unique_graph_region_span(
            spans,
            BezierMonotoneSpan::new(start, end).map_err(|_| UncertaintyReason::Ordering)?,
            policy,
        )?;
        return Ok(());
    }

    isolate_scalar_bernstein_roots(
        left,
        start,
        mid.clone(),
        depth + 1,
        policy,
        exact_parameters,
        spans,
    )?;
    isolate_scalar_bernstein_roots(right, mid, end, depth + 1, policy, exact_parameters, spans)
}

fn subdivide_scalar_bernstein_half(
    controls: &[Real],
) -> Result<(Vec<Real>, Vec<Real>), UncertaintyReason> {
    if controls.is_empty() {
        return Err(UncertaintyReason::Unsupported);
    }

    let degree = controls.len() - 1;
    let mut work = controls.to_vec();
    let mut left = Vec::with_capacity(controls.len());
    let mut right = Vec::with_capacity(controls.len());
    left.push(work[0].clone());
    right.push(work[degree].clone());
    for level in 1..=degree {
        for index in 0..=degree - level {
            work[index] = midpoint_real(&work[index], &work[index + 1])?;
        }
        left.push(work[0].clone());
        right.push(work[degree - level].clone());
    }
    right.reverse();
    Ok((left, right))
}

fn strict_rational_graph_order_from_degree4_weighted_signs(
    values: &[Real; 5],
    weight_sign: RealSign,
    policy: &CurvePolicy,
) -> Option<BezierMonotoneGraphOrder> {
    let mut common = None;
    for value in values {
        let sign = real_sign(value, policy)?;
        match (common, sign) {
            (_, RealSign::Zero) => return None,
            (None, RealSign::Positive | RealSign::Negative) => common = Some(sign),
            (Some(previous), RealSign::Positive | RealSign::Negative) if previous == sign => {}
            (Some(_), RealSign::Positive | RealSign::Negative) => return None,
        }
    }
    rational_graph_order_from_sign(common?, weight_sign)
}

fn strict_rational_graph_order_from_degree5_weighted_signs(
    values: &[Real; 6],
    weight_sign: RealSign,
    policy: &CurvePolicy,
) -> Option<BezierMonotoneGraphOrder> {
    let mut common = None;
    for value in values {
        let sign = real_sign(value, policy)?;
        match (common, sign) {
            (_, RealSign::Zero) => return None,
            (None, RealSign::Positive | RealSign::Negative) => common = Some(sign),
            (Some(previous), RealSign::Positive | RealSign::Negative) if previous == sign => {}
            (Some(_), RealSign::Positive | RealSign::Negative) => return None,
        }
    }
    rational_graph_order_from_sign(common?, weight_sign)
}

fn invert_graph_order(order: BezierMonotoneGraphOrder) -> BezierMonotoneGraphOrder {
    match order {
        BezierMonotoneGraphOrder::FirstLess => BezierMonotoneGraphOrder::FirstGreater,
        BezierMonotoneGraphOrder::FirstGreater => BezierMonotoneGraphOrder::FirstLess,
        BezierMonotoneGraphOrder::Coincident => BezierMonotoneGraphOrder::Coincident,
        BezierMonotoneGraphOrder::IntersectsOrTouches { parameters, spans } => {
            BezierMonotoneGraphOrder::IntersectsOrTouches { parameters, spans }
        }
        BezierMonotoneGraphOrder::NotSharedStrictlyMonotone => {
            BezierMonotoneGraphOrder::NotSharedStrictlyMonotone
        }
    }
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

    /// Classifies graph order against a rational quadratic conic.
    ///
    /// This is the polynomial-side companion to
    /// [`RationalQuadraticBezier2::graph_order_to_quadratic_over_axis`]. It
    /// includes the equal-weight polynomial collapse and the non-equal
    /// homogeneous numerator shortcut, then inverts the order so callers see
    /// the polynomial curve as `self`. Non-strict mixed quartic roots are
    /// retained as represented parameters or isolating spans following the exactness model's
    /// exact-predicate boundary.
    pub fn graph_order_to_rational_quadratic_over_axis(
        &self,
        other: &RationalQuadraticBezier2,
        shared_axis: Axis2,
        policy: &CurvePolicy,
    ) -> Classification<BezierMonotoneGraphOrder> {
        match other.equal_weight_polynomial_quadratic_image(policy) {
            Classification::Decided(Some(curve)) => {
                self.graph_order_to_quadratic_over_axis(&curve, shared_axis, policy)
            }
            Classification::Decided(None) => other
                .rational_quadratic_graph_order_to_quadratic(self, shared_axis, policy)
                .map(invert_graph_order),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
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

    /// Classifies graph order against a rational quadratic conic.
    ///
    /// Equal homogeneous conic weights collapse exactly to the polynomial
    /// quadratic image. Non-equal same-sign weights use the degree-5
    /// homogeneous Bernstein root/sign shortcut when one coordinate is exactly
    /// shared, then invert the result so `self` is the first curve. Non-strict
    /// mixed quintic roots are retained as represented parameters or isolating
    /// spans rather than sampled topology, preserving the exactness model's EGC boundary; see
    /// exact-computation discipline, the Bernstein and de Casteljau curve model, and Bezier clipping.
    pub fn graph_order_to_rational_quadratic_over_axis(
        &self,
        other: &RationalQuadraticBezier2,
        shared_axis: Axis2,
        policy: &CurvePolicy,
    ) -> Classification<BezierMonotoneGraphOrder> {
        match other.equal_weight_polynomial_quadratic_image(policy) {
            Classification::Decided(Some(curve)) => {
                self.graph_order_to_quadratic_over_axis(&curve, shared_axis, policy)
            }
            Classification::Decided(None) => other
                .rational_quadratic_graph_order_to_cubic(self, shared_axis, policy)
                .map(invert_graph_order),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }
}

fn relation_to_polynomial_bezier(
    rational: &RationalQuadraticBezier2,
    polynomial_controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<BezierCurveRelation> {
    if polynomial_controls.is_empty() {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    }

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
    match polynomial_relation_for_strict_mixed_graph_order(rational, polynomial_controls, policy) {
        Classification::Decided(Some(relation)) => return Classification::Decided(relation),
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    let mut deferred_uncertainty = None;
    if rational.weights_known_same_nonzero_sign(policy) == Some(true) {
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

    let rational_point_image = match rational_point_image(rational, policy) {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            deferred_uncertainty.get_or_insert(reason);
            None
        }
    };
    let polynomial_point_image = match point_image_from_controls(polynomial_controls, policy) {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            deferred_uncertainty.get_or_insert(reason);
            None
        }
    };
    match (&rational_point_image, &polynomial_point_image) {
        (Some(first), Some(second)) => match point_equal(first, second, policy) {
            Some(true) => {
                return Classification::Decided(BezierCurveRelation::IntersectionPoints {
                    points: vec![BezierCurveIntersectionPoint::new(first.clone())],
                });
            }
            Some(false) => return Classification::Decided(BezierCurveRelation::NoIntersection),
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        },
        (Some(point), None) => {
            match point_image_polynomial_intersections(point, polynomial_controls, policy) {
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
        (None, Some(point)) => match point_image_rational_intersections(point, rational, policy) {
            Classification::Decided(points) if points.is_empty() => {
                return Classification::Decided(BezierCurveRelation::NoIntersection);
            }
            Classification::Decided(points) => {
                return Classification::Decided(BezierCurveRelation::IntersectionPoints { points });
            }
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        },
        (None, None) => {}
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
            return line_segment_intersection_relation(first, second, policy);
        }
        (None, Some(line)) => match line_image_rational_relation(line, rational, false, policy) {
            Classification::Decided(Some(relation)) => {
                return Classification::Decided(relation);
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        },
        (Some(line), None) => {
            match line_image_polynomial_relation(line, polynomial_controls, true, policy) {
                Classification::Decided(Some(relation)) => {
                    return Classification::Decided(relation);
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
        (None, None) => {}
    }

    let mut shared_endpoint_points = Vec::new();
    for a in [rational.start(), rational.end()] {
        for b in [
            polynomial_controls[0],
            polynomial_controls[polynomial_controls.len() - 1],
        ] {
            match point_equal(a, b, policy) {
                Some(true) => {
                    push_unique_intersection_point(&mut shared_endpoint_points, a.clone(), policy)
                }
                Some(false) => {}
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
    }

    let endpoint_points =
        match rational_polynomial_endpoint_intersections(rational, polynomial_controls, policy) {
            Classification::Decided(points) => points,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };

    match same_parameter_dyadic_rational_polynomial_relation(rational, polynomial_controls, policy)
    {
        Classification::Decided(Some(relation)) => {
            return Classification::Decided(merge_endpoint_points_into_relation(
                relation,
                &endpoint_points,
                policy,
            ));
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    if !endpoint_points.is_empty() {
        return if endpoint_points_are_shared_only(&endpoint_points, &shared_endpoint_points, policy)
        {
            Classification::Decided(BezierCurveRelation::SharedEndpoint)
        } else {
            Classification::Decided(BezierCurveRelation::EndpointIntersections {
                points: endpoint_points,
            })
        };
    }

    if let Some(reason) = deferred_uncertainty {
        return Classification::Uncertain(reason);
    }

    if rational.weights_known_same_nonzero_sign(policy) == Some(true) {
        let rational_node = match RationalSubdivisionNode::from_rational(rational) {
            Ok(node) => node,
            Err(reason) => return Classification::Uncertain(reason),
        };
        let polynomial_node = match RationalSubdivisionNode::from_polynomial(polynomial_controls) {
            Ok(node) => node,
            Err(reason) => return Classification::Uncertain(reason),
        };
        return isolate_curve_regions(rational_node, polynomial_node, policy);
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
        let parameter = match Real::from(numerator) / &denominator {
            Ok(parameter) => parameter,
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
        let rational_point = match rational.point_at(parameter.clone(), policy) {
            Classification::Decided(point) => point,
            // Denominator-boundary candidates are not promoted. The caller keeps
            // the conservative subdivision/uncertainty path for topology.
            Classification::Uncertain(_) => continue,
        };
        let Some(polynomial_point) = polynomial_point_at(polynomial_controls, parameter) else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
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
        // follows the exact-geometric-computation boundary by promoting only
        // points that survive exact homogeneous rational evaluation and exact
        // de Casteljau polynomial evaluation.
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
            // homogeneous-to-affine reduction described by the Bernstein and de Casteljau curve model. Per the exactness model's EGC model, we
            // preserve that object identity and delegate to the polynomial
            // predicate surface instead of pushing an already-polynomial curve
            // through conservative conic subdivision.
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

fn polynomial_relation_for_strict_mixed_graph_order(
    rational: &RationalQuadraticBezier2,
    polynomial_controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<Option<BezierCurveRelation>> {
    // Once one coordinate is certified identical and injective, the remaining
    // homogeneous numerator is a complete same-parameter graph predicate:
    // strict signs prove no intersection, represented roots promote exact
    // points, and non-represented roots stay as retained parameter regions.
    // This is the Bezier clipping exclusion/retention idea
    // used under the exactness model's exact branch boundary; the Bernstein curve model gives the homogeneous
    // rational Bernstein identities that produce the degree-4/5 numerators.
    for axis in [Axis2::X, Axis2::Y] {
        let order = match polynomial_controls {
            [start, control, end] => {
                let polynomial =
                    QuadraticBezier2::new((*start).clone(), (*control).clone(), (*end).clone());
                rational.graph_order_to_quadratic_over_axis(&polynomial, axis, policy)
            }
            [start, control1, control2, end] => {
                let polynomial = CubicBezier2::new(
                    (*start).clone(),
                    (*control1).clone(),
                    (*control2).clone(),
                    (*end).clone(),
                );
                rational.graph_order_to_cubic_over_axis(&polynomial, axis, policy)
            }
            _ => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
        match order {
            Classification::Decided(order) => {
                match relation_from_mixed_graph_order(rational, order, policy) {
                    Ok(Some(relation)) => return Classification::Decided(Some(relation)),
                    Ok(None) => {}
                    Err(reason) => return Classification::Uncertain(reason),
                }
            }
            Classification::Uncertain(UncertaintyReason::Unsupported) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }

    Classification::Decided(None)
}

fn relation_from_mixed_graph_order(
    rational: &RationalQuadraticBezier2,
    order: BezierMonotoneGraphOrder,
    policy: &CurvePolicy,
) -> Result<Option<BezierCurveRelation>, UncertaintyReason> {
    match order {
        BezierMonotoneGraphOrder::FirstLess | BezierMonotoneGraphOrder::FirstGreater => {
            Ok(Some(BezierCurveRelation::NoIntersection))
        }
        BezierMonotoneGraphOrder::Coincident => Ok(Some(BezierCurveRelation::SameCurveImage)),
        BezierMonotoneGraphOrder::NotSharedStrictlyMonotone => Ok(None),
        BezierMonotoneGraphOrder::IntersectsOrTouches {
            parameters,
            mut spans,
        } => {
            if spans.is_empty() {
                let mut points = Vec::new();
                for parameter in parameters {
                    let point = match rational.point_at(parameter, policy) {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => return Err(reason),
                    };
                    push_unique_intersection_point(&mut points, point, policy);
                }
                return Ok(Some(BezierCurveRelation::IntersectionPoints { points }));
            }

            for parameter in parameters {
                push_unique_graph_region_span(&mut spans, zero_width_span(parameter)?, policy)?;
            }
            let regions = spans
                .into_iter()
                .map(|span| BezierCurveIntersectionRegion::new(span.clone(), span))
                .collect();
            Ok(Some(BezierCurveRelation::IntersectionRegions { regions }))
        }
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
    if curve.weights_known_same_nonzero_sign(policy) != Some(true) {
        return Classification::Decided(None);
    }
    line_segment_image_from_controls(&curve.control_points(), policy)
}

fn rational_point_image(
    curve: &RationalQuadraticBezier2,
    policy: &CurvePolicy,
) -> Classification<Option<Point2>> {
    if curve.weights_known_same_nonzero_sign(policy) != Some(true) {
        return Classification::Decided(None);
    }
    point_image_from_controls(&curve.control_points(), policy)
}

fn point_image_from_controls(
    controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<Option<Point2>> {
    let Some(point) = controls.first().copied() else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    for control in controls.iter().skip(1) {
        match point_equal(point, control, policy) {
            Some(true) => {}
            Some(false) => return Classification::Decided(None),
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }
    Classification::Decided(Some(point.clone()))
}

fn line_segment_image_from_controls(
    controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<Option<LineSeg2>> {
    // Same-sign rational Bezier weights can be homogeneous-sign-normalized to
    // the positive case, preserving the Euclidean convex-hull property, while
    // collinear Bernstein controls keep the image on the supporting line; see
    // the Bernstein and de Casteljau curve model. When every interior control
    // is also inside the endpoint box, the segment image is exactly the
    // endpoint line segment, so the exactness model's EGC boundary lets us dispatch to the
    // native exact line-line predicate instead of subdividing an already
    // linear object.
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

fn merge_endpoint_points_into_relation(
    relation: BezierCurveRelation,
    endpoint_points: &[BezierCurveIntersectionPoint],
    policy: &CurvePolicy,
) -> BezierCurveRelation {
    if endpoint_points.is_empty() {
        return relation;
    }

    match relation {
        BezierCurveRelation::IntersectionPoints { mut points } => {
            // Rational/conic endpoints are exact lower-degree point predicates.
            // When a later same-parameter conic solve finds additional roots,
            // keep both evidence sets instead of letting an endpoint shortcut
            // mask interior topology. This follows the exact-geometric-
            // computation boundary, and the rational homogeneous model is the
            // standard Bernstein curve model representation retained by this module.
            for endpoint in endpoint_points {
                push_unique_intersection_point(&mut points, endpoint.point().clone(), policy);
            }
            BezierCurveRelation::IntersectionPoints { points }
        }
        BezierCurveRelation::NoIntersection
        | BezierCurveRelation::BoundingBoxesDisjoint
        | BezierCurveRelation::Unresolved => BezierCurveRelation::EndpointIntersections {
            points: endpoint_points.to_vec(),
        },
        relation => relation,
    }
}

fn endpoint_points_are_shared_only(
    endpoint_points: &[BezierCurveIntersectionPoint],
    shared_endpoint_points: &[BezierCurveIntersectionPoint],
    policy: &CurvePolicy,
) -> bool {
    !endpoint_points.is_empty()
        && endpoint_points.iter().all(|endpoint| {
            shared_endpoint_points
                .iter()
                .any(|shared| point_equal(endpoint.point(), shared.point(), policy) == Some(true))
        })
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

    match matching_weight_rational_graph_relation(first, second, policy) {
        Classification::Decided(Some(relation)) => return Classification::Decided(Some(relation)),
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
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
        match matching_weight_rational_same_parameter_is_complete(first, second, policy) {
            Classification::Decided(true) => {
                Classification::Decided(Some(BezierCurveRelation::NoIntersection))
            }
            Classification::Decided(false) => Classification::Decided(None),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    } else {
        Classification::Decided(Some(BezierCurveRelation::IntersectionPoints { points }))
    }
}

fn matching_weight_rational_graph_relation(
    first: &RationalQuadraticBezier2,
    second: &RationalQuadraticBezier2,
    policy: &CurvePolicy,
) -> Classification<Option<BezierCurveRelation>> {
    if first.weights_known_same_nonzero_sign(policy) != Some(true) {
        return Classification::Decided(None);
    }

    for shared_axis in [Axis2::X, Axis2::Y] {
        if !rational_matching_axis_equal(first, second, shared_axis, policy) {
            continue;
        }
        match rational_axis_strictly_monotone(first, shared_axis, policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => continue,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }

        let solve_axis = other_axis(shared_axis);
        let cover =
            match matching_weight_axis_difference_root_cover(first, second, solve_axis, policy) {
                Classification::Decided(Some(cover)) => cover,
                Classification::Decided(None) => return Classification::Decided(None),
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };

        let relation = match relation_from_matching_weight_graph_root_cover(first, cover, policy) {
            Ok(relation) => relation,
            Err(reason) => return Classification::Uncertain(reason),
        };
        return Classification::Decided(Some(relation));
    }

    Classification::Decided(None)
}

fn matching_weight_rational_graph_order(
    first: &RationalQuadraticBezier2,
    second: &RationalQuadraticBezier2,
    shared_axis: Axis2,
    policy: &CurvePolicy,
) -> Classification<BezierMonotoneGraphOrder> {
    match matching_rational_weights(first, second, policy) {
        Some(true) => {}
        Some(false) | None => {
            return Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone);
        }
    }
    let Some(weight_sign) = common_rational_weight_sign(first, policy) else {
        return Classification::Uncertain(UncertaintyReason::RealSign);
    };

    if first.weights_known_same_nonzero_sign(policy) != Some(true) {
        return Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone);
    }
    if !rational_matching_axis_equal(first, second, shared_axis, policy) {
        return Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone);
    }
    match rational_axis_strictly_monotone(first, shared_axis, policy) {
        Classification::Decided(true) => {}
        Classification::Decided(false) => {
            return Classification::Decided(BezierMonotoneGraphOrder::NotSharedStrictlyMonotone);
        }
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    let solve_axis = other_axis(shared_axis);
    let values = matching_weight_axis_difference_values(first, second, solve_axis);
    if values
        .iter()
        .all(|value| is_zero(value, policy) == Some(true))
    {
        return Classification::Decided(BezierMonotoneGraphOrder::Coincident);
    }
    if let Some(order) =
        strict_rational_graph_order_from_weighted_signs(&values, weight_sign, policy)
    {
        return Classification::Decided(order);
    }

    let cover = match matching_weight_axis_difference_root_cover(first, second, solve_axis, policy)
    {
        Classification::Decided(Some(cover)) => cover,
        Classification::Decided(None) => {
            return Classification::Uncertain(UncertaintyReason::RealSign);
        }
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    match cover {
        RationalQuadraticRootCover::All => {
            Classification::Decided(BezierMonotoneGraphOrder::Coincident)
        }
        RationalQuadraticRootCover::Isolated { exact, spans } => {
            if exact.is_empty() && spans.is_empty() {
                match real_sign(&values[0], policy) {
                    Some(sign) => match rational_graph_order_from_sign(sign, weight_sign) {
                        Some(order) => Classification::Decided(order),
                        None => Classification::Uncertain(UncertaintyReason::RealSign),
                    },
                    None => Classification::Uncertain(UncertaintyReason::RealSign),
                }
            } else {
                Classification::Decided(BezierMonotoneGraphOrder::IntersectsOrTouches {
                    parameters: exact,
                    spans,
                })
            }
        }
    }
}

fn matching_weight_rational_same_parameter_is_complete(
    first: &RationalQuadraticBezier2,
    second: &RationalQuadraticBezier2,
    policy: &CurvePolicy,
) -> Classification<bool> {
    // Matching weights give both rational quadratics one common homogeneous
    // denominator. If one Euclidean coordinate is also certified identical and
    // strictly monotone, that coordinate is injective, so every geometric
    // intersection has the same parameter on both curves. The remaining
    // coordinate is then a single exact quadratic Bernstein root problem. This
    // is the rational analogue of the polynomial graph shortcut above: the Bernstein curve model's
    // homogeneous rational Bezier model supplies the shared denominator, and
    // the exactness model's EGC model keeps the no-hit conclusion behind exact sign and root
    // certificates; see the Bernstein and de Casteljau curve model,
    // and exact-computation discipline.
    if first.weights_known_same_nonzero_sign(policy) != Some(true) {
        return Classification::Decided(false);
    }

    for axis in [Axis2::X, Axis2::Y] {
        if !rational_matching_axis_equal(first, second, axis, policy) {
            continue;
        }
        match rational_axis_strictly_monotone(first, axis, policy) {
            Classification::Decided(true) => return Classification::Decided(true),
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(false)
}

fn other_axis(axis: Axis2) -> Axis2 {
    match axis {
        Axis2::X => Axis2::Y,
        Axis2::Y => Axis2::X,
    }
}

fn rational_matching_axis_equal(
    first: &RationalQuadraticBezier2,
    second: &RationalQuadraticBezier2,
    axis: Axis2,
    policy: &CurvePolicy,
) -> bool {
    first
        .control_points()
        .iter()
        .zip(second.control_points().iter())
        .all(|(a, b)| is_zero(&(coordinate(a, axis) - coordinate(b, axis)), policy) == Some(true))
}

fn rational_axis_strictly_monotone(
    curve: &RationalQuadraticBezier2,
    axis: Axis2,
    policy: &CurvePolicy,
) -> Classification<bool> {
    let roots = match curve.axis_monotone_parameters(axis, policy) {
        Classification::Decided(roots) => roots,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    if !roots.is_empty() {
        return Classification::Decided(false);
    }
    let endpoint_delta = coordinate(curve.end(), axis) - coordinate(curve.start(), axis);
    Classification::Decided(!matches!(
        real_sign(&endpoint_delta, policy),
        Some(RealSign::Zero) | None
    ))
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

fn common_rational_weight_sign(
    curve: &RationalQuadraticBezier2,
    policy: &CurvePolicy,
) -> Option<RealSign> {
    curve.common_nonzero_weight_sign(policy)
}

fn common_weight_sign_for_values(weights: [&Real; 3], policy: &CurvePolicy) -> Option<RealSign> {
    let mut common = None;
    for weight in weights {
        let sign = real_sign(weight, policy)?;
        match (common, sign) {
            (_, RealSign::Zero) => return None,
            (None, RealSign::Positive | RealSign::Negative) => common = Some(sign),
            (Some(previous), RealSign::Positive | RealSign::Negative) if previous == sign => {}
            (Some(_), RealSign::Positive | RealSign::Negative) => return None,
        }
    }
    common
}

fn strict_rational_graph_order_from_weighted_signs(
    values: &[Real; 3],
    weight_sign: RealSign,
    policy: &CurvePolicy,
) -> Option<BezierMonotoneGraphOrder> {
    let mut common = None;
    for value in values {
        let sign = real_sign(value, policy)?;
        match (common, sign) {
            (_, RealSign::Zero) => return None,
            (None, RealSign::Positive | RealSign::Negative) => common = Some(sign),
            (Some(previous), RealSign::Positive | RealSign::Negative) if previous == sign => {}
            (Some(_), RealSign::Positive | RealSign::Negative) => return None,
        }
    }
    rational_graph_order_from_sign(common?, weight_sign)
}

fn rational_graph_order_from_sign(
    weighted_numerator_sign: RealSign,
    weight_sign: RealSign,
) -> Option<BezierMonotoneGraphOrder> {
    match (weighted_numerator_sign, weight_sign) {
        (RealSign::Positive, RealSign::Positive) | (RealSign::Negative, RealSign::Negative) => {
            Some(BezierMonotoneGraphOrder::FirstGreater)
        }
        (RealSign::Negative, RealSign::Positive) | (RealSign::Positive, RealSign::Negative) => {
            Some(BezierMonotoneGraphOrder::FirstLess)
        }
        _ => None,
    }
}

fn matching_weight_axis_difference_root_set(
    first: &RationalQuadraticBezier2,
    second: &RationalQuadraticBezier2,
    axis: Axis2,
    policy: &CurvePolicy,
) -> Classification<Option<RationalPointRootSet>> {
    let values = matching_weight_axis_difference_values(first, second, axis);
    if values
        .iter()
        .all(|value| is_zero(value, policy) == Some(true))
    {
        return Classification::Decided(Some(RationalPointRootSet::All));
    }

    // Matching rational weights give both curves the same Bernstein denominator,
    // so same-parameter equality reduces to zeros of the weighted homogeneous
    // numerator difference. This is the Bernstein curve model's rational Bezier model kept at the exactness model's
    // certified predicate boundary: exact scalar roots are promoted, while roots
    // outside the current scalar proof surface fall back to conservative
    // subdivision regions; see the Bernstein and de Casteljau curve model, and
    // exact-computation discipline.
    let (c0, c1, c2) = quadratic_bernstein_to_power(values);
    match polynomial_roots_in_unit_interval(c0, c1, c2, policy) {
        Classification::Decided(roots) => {
            Classification::Decided(Some(RationalPointRootSet::Roots(roots)))
        }
        Classification::Uncertain(_) => Classification::Decided(None),
    }
}

fn matching_weight_axis_difference_root_cover(
    first: &RationalQuadraticBezier2,
    second: &RationalQuadraticBezier2,
    axis: Axis2,
    policy: &CurvePolicy,
) -> Classification<Option<RationalQuadraticRootCover>> {
    let values = matching_weight_axis_difference_values(first, second, axis);
    if values
        .iter()
        .all(|value| is_zero(value, policy) == Some(true))
    {
        return Classification::Decided(Some(RationalQuadraticRootCover::All));
    }

    let (c0, c1, c2) = quadratic_bernstein_to_power(values.clone());
    match polynomial_roots_in_unit_interval(c0, c1, c2, policy) {
        Classification::Decided(exact) => {
            return Classification::Decided(Some(RationalQuadraticRootCover::Isolated {
                exact,
                spans: Vec::new(),
            }));
        }
        Classification::Uncertain(_) => {}
    }

    // Complete graph cases still have a useful certificate when an exact
    // quadratic root cannot be represented by the scalar API: the Bernstein
    // convex-hull sign test either discards a subspan or keeps it as a bounded
    // parameter region. This mirrors the Bezier clipping exclusion of
    // Bezier clipping,
    // while preserving the exactness model's exact-predicate boundary instead of converting the
    // undecidable root into an approximate point.
    let mut exact = Vec::new();
    let mut spans = Vec::new();
    if let Err(reason) = isolate_scalar_quadratic_roots(
        values,
        Real::zero(),
        Real::one(),
        0,
        policy,
        &mut exact,
        &mut spans,
    ) {
        return Classification::Uncertain(reason);
    }
    Classification::Decided(Some(RationalQuadraticRootCover::Isolated { exact, spans }))
}

fn matching_weight_axis_difference_values(
    first: &RationalQuadraticBezier2,
    second: &RationalQuadraticBezier2,
    axis: Axis2,
) -> [Real; 3] {
    let first_controls = first.control_points();
    let second_controls = second.control_points();
    let weights = first.weights();
    [
        weights[0] * &(coordinate(first_controls[0], axis) - coordinate(second_controls[0], axis)),
        weights[1] * &(coordinate(first_controls[1], axis) - coordinate(second_controls[1], axis)),
        weights[2] * &(coordinate(first_controls[2], axis) - coordinate(second_controls[2], axis)),
    ]
}

fn isolate_rational_quadratic_line_roots(
    weighted_distances: [Real; 3],
    policy: &CurvePolicy,
) -> Classification<BezierLineRelation> {
    // The signed distance numerator of a same-sign rational quadratic Bezier
    // has a denominator that cannot vanish on the affine segment, so its line
    // intersections are exactly the zeros of this scalar quadratic Bernstein
    // numerator. When the closed-form scalar solver cannot represent a root,
    // retain a certified dyadic Bernstein bracket instead of collapsing the
    // topology decision into an approximate sample. This is the same
    // convex-hull sign isolation used by Bezier clipping; see Bezier clipping, with the
    // rational homogeneous numerator/denominator model from the Bernstein and de Casteljau curve model, and the exactness model's exact predicate boundary.
    let mut exact_parameters = Vec::new();
    let mut spans = Vec::new();
    if let Err(reason) = isolate_scalar_quadratic_roots(
        weighted_distances,
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
            let span = match zero_width_span(parameter) {
                Ok(span) => span,
                Err(reason) => return Classification::Uncertain(reason),
            };
            if let Err(reason) = push_unique_graph_region_span(&mut spans, span, policy) {
                return Classification::Uncertain(reason);
            }
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

fn relation_from_matching_weight_graph_root_cover(
    curve: &RationalQuadraticBezier2,
    cover: RationalQuadraticRootCover,
    policy: &CurvePolicy,
) -> Result<BezierCurveRelation, UncertaintyReason> {
    match cover {
        RationalQuadraticRootCover::All => Ok(BezierCurveRelation::SameCurveImage),
        RationalQuadraticRootCover::Isolated { exact, spans } => {
            if spans.is_empty() {
                if exact.is_empty() {
                    return Ok(BezierCurveRelation::NoIntersection);
                }
                let mut points = Vec::new();
                for parameter in exact {
                    let point = match curve.point_at(parameter, policy) {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => return Err(reason),
                    };
                    push_unique_intersection_point(&mut points, point, policy);
                }
                return Ok(BezierCurveRelation::IntersectionPoints { points });
            }

            let mut regions = spans;
            for parameter in exact {
                push_unique_graph_region_span(&mut regions, zero_width_span(parameter)?, policy)?;
            }
            let regions = regions
                .into_iter()
                .map(|span| BezierCurveIntersectionRegion::new(span.clone(), span))
                .collect();
            Ok(BezierCurveRelation::IntersectionRegions { regions })
        }
    }
}

fn isolate_scalar_quadratic_roots(
    controls: [Real; 3],
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
        push_unique_graph_parameter(exact_parameters, start.clone(), policy)?;
    }
    if signs[2] == RealSign::Zero {
        push_unique_graph_parameter(exact_parameters, end.clone(), policy)?;
    }

    let strict_signs = signs
        .iter()
        .copied()
        .filter(|sign| *sign != RealSign::Zero)
        .collect::<Vec<_>>();
    if strict_signs.is_empty() {
        push_unique_graph_region_span(
            spans,
            BezierMonotoneSpan::new(start, end).map_err(|_| UncertaintyReason::Ordering)?,
            policy,
        )?;
        return Ok(());
    }
    if strict_signs.iter().all(|sign| *sign == strict_signs[0]) {
        return Ok(());
    }

    let mid = ((&start + &end) / Real::from(2_i8)).map_err(|_| UncertaintyReason::Unsupported)?;
    let mid_value = scalar_quadratic_at_half(&controls)?;
    if is_zero(&mid_value, policy) == Some(true) {
        push_unique_graph_parameter(exact_parameters, mid.clone(), policy)?;
    }

    if depth >= 32 {
        push_unique_graph_region_span(
            spans,
            BezierMonotoneSpan::new(start, end).map_err(|_| UncertaintyReason::Ordering)?,
            policy,
        )?;
        return Ok(());
    }

    let (left, right) = subdivide_scalar_quadratic_half(controls)?;
    isolate_scalar_quadratic_roots(
        left,
        start,
        mid.clone(),
        depth + 1,
        policy,
        exact_parameters,
        spans,
    )?;
    isolate_scalar_quadratic_roots(right, mid, end, depth + 1, policy, exact_parameters, spans)
}

fn scalar_quadratic_at_half(controls: &[Real; 3]) -> Result<Real, UncertaintyReason> {
    divide_by_positive_integer(
        controls[0].clone() + (&Real::from(2_i8) * &controls[1]) + controls[2].clone(),
        4,
    )
}

fn subdivide_scalar_quadratic_half(
    controls: [Real; 3],
) -> Result<([Real; 3], [Real; 3]), UncertaintyReason> {
    let p01 = midpoint_real(&controls[0], &controls[1])?;
    let p12 = midpoint_real(&controls[1], &controls[2])?;
    let p012 = midpoint_real(&p01, &p12)?;
    Ok((
        [controls[0].clone(), p01.clone(), p012.clone()],
        [p012, p12, controls[2].clone()],
    ))
}

pub(crate) fn push_unique_graph_parameter(
    values: &mut Vec<Real>,
    value: Real,
    policy: &CurvePolicy,
) -> Result<(), UncertaintyReason> {
    let mut insert_at = values.len();
    for (index, existing) in values.iter().enumerate() {
        match compare_reals(existing, &value, policy) {
            Some(Ordering::Equal) => return Ok(()),
            Some(Ordering::Greater) => {
                insert_at = index;
                break;
            }
            Some(Ordering::Less) => {}
            None => return Err(UncertaintyReason::Ordering),
        }
    }
    values.insert(insert_at, value);
    Ok(())
}

pub(crate) fn push_unique_graph_region_span(
    spans: &mut Vec<BezierMonotoneSpan>,
    span: BezierMonotoneSpan,
    policy: &CurvePolicy,
) -> Result<(), UncertaintyReason> {
    let mut insert_at = spans.len();
    for (index, existing) in spans.iter().enumerate() {
        match compare_reals(existing.start(), span.start(), policy) {
            Some(Ordering::Less) => {}
            Some(Ordering::Greater) => {
                insert_at = index;
                break;
            }
            Some(Ordering::Equal) => match compare_reals(existing.end(), span.end(), policy) {
                Some(Ordering::Equal) => return Ok(()),
                Some(Ordering::Greater) => {
                    insert_at = index;
                    break;
                }
                Some(Ordering::Less) => {}
                None => return Err(UncertaintyReason::Ordering),
            },
            None => return Err(UncertaintyReason::Ordering),
        }
    }
    spans.insert(insert_at, span);
    Ok(())
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

fn point_image_rational_intersections(
    point: &Point2,
    rational: &RationalQuadraticBezier2,
    policy: &CurvePolicy,
) -> Classification<Vec<BezierCurveIntersectionPoint>> {
    // Once a rational conic is certified to collapse to one affine point, the
    // curve/curve predicate reduces to the other conic's homogeneous
    // point-parameter equations. This keeps the exactness model's exact branch boundary: no
    // sampled epsilon is introduced, and projective denominator uncertainty is
    // still surfaced by the rational point solver.
    match rational.parameters_for_point(point, policy) {
        Classification::Decided(parameters) if parameters.is_empty() => {
            Classification::Decided(Vec::new())
        }
        Classification::Decided(_) => {
            Classification::Decided(vec![BezierCurveIntersectionPoint::new(point.clone())])
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

fn point_image_polynomial_intersections(
    point: &Point2,
    controls: &[&Point2],
    policy: &CurvePolicy,
) -> Classification<Option<Vec<BezierCurveIntersectionPoint>>> {
    // Polynomial quadratic point queries are complete low-degree Bernstein
    // solves; cubic point queries currently remain a finite dyadic promotion
    // pass, so a miss for cubic controls is not a no-intersection proof. This
    // mirrors the polynomial Bezier dispatcher and follows the exactness model's distinction
    // between certified decisions and conservative unresolved cases.
    match controls {
        [start, control, end] => {
            let curve = QuadraticBezier2::new((*start).clone(), (*control).clone(), (*end).clone());
            match curve.parameters_for_point(point, policy) {
                Classification::Decided(parameters) if parameters.is_empty() => {
                    Classification::Decided(Some(Vec::new()))
                }
                Classification::Decided(_) => {
                    Classification::Decided(Some(vec![BezierCurveIntersectionPoint::new(
                        point.clone(),
                    )]))
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            }
        }
        [start, control1, control2, end] => {
            let curve = CubicBezier2::new(
                (*start).clone(),
                (*control1).clone(),
                (*control2).clone(),
                (*end).clone(),
            );
            match curve.dyadic_parameters_for_point(point, policy) {
                Classification::Decided(parameters) if parameters.is_empty() => {
                    Classification::Decided(None)
                }
                Classification::Decided(_) => {
                    Classification::Decided(Some(vec![BezierCurveIntersectionPoint::new(
                        point.clone(),
                    )]))
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            }
        }
        _ => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

fn line_image_rational_relation(
    line: &LineSeg2,
    rational: &RationalQuadraticBezier2,
    line_is_first: bool,
    policy: &CurvePolicy,
) -> Classification<Option<BezierCurveRelation>> {
    // A line-image curve turns one side of the curve/curve problem into a
    // finite segment containment predicate. The rational side still uses its
    // homogeneous supporting-line numerator, following the Bernstein curve model's rational Bezier
    // incidence formulas. Isolated line roots become curve/curve regions with a
    // full span on the line-image side, preserving the Bezier clipping
    // Bezier-clipping bracket at the exactness model's exact predicate boundary instead
    // of using sampled distances.
    match rational.relation_to_line(line, policy) {
        Classification::Decided(BezierLineRelation::ControlHullDisjoint { .. }) => {
            Classification::Decided(Some(BezierCurveRelation::NoIntersection))
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
            if points.is_empty() {
                Classification::Decided(Some(BezierCurveRelation::NoIntersection))
            } else {
                Classification::Decided(Some(BezierCurveRelation::IntersectionPoints { points }))
            }
        }
        Classification::Decided(BezierLineRelation::IsolatedIntersections { spans }) => {
            line_image_regions_from_curve_spans(spans, line_is_first).map(Some)
        }
        Classification::Decided(
            BezierLineRelation::OnSupportingLine | BezierLineRelation::Unresolved,
        ) => Classification::Decided(None),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

fn line_image_polynomial_relation(
    line: &LineSeg2,
    controls: &[&Point2],
    line_is_first: bool,
    policy: &CurvePolicy,
) -> Classification<Option<BezierCurveRelation>> {
    // For polynomial quadratics, the supporting-line distance is an exact
    // quadratic Bernstein polynomial. Solving it before falling back to
    // subdivision is the low-degree slice of the Bezier
    // clipping strategy, kept exact by exactness- certified containment checks.
    let relation = match polynomial_relation_to_line(controls, line, policy) {
        Classification::Decided(relation) => relation,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    match relation {
        BezierLineRelation::ControlHullDisjoint { .. } => {
            Classification::Decided(Some(BezierCurveRelation::NoIntersection))
        }
        BezierLineRelation::Intersects { parameters } => {
            let mut points = Vec::new();
            for parameter in parameters {
                let Some(point) = polynomial_point_at(controls, parameter) else {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                };
                match line.contains_point(&point, policy) {
                    Classification::Decided(true) => {
                        push_unique_intersection_point(&mut points, point, policy)
                    }
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
            }
            if points.is_empty() {
                Classification::Decided(Some(BezierCurveRelation::NoIntersection))
            } else {
                Classification::Decided(Some(BezierCurveRelation::IntersectionPoints { points }))
            }
        }
        BezierLineRelation::IsolatedIntersections { spans } => {
            line_image_regions_from_curve_spans(spans, line_is_first).map(Some)
        }
        BezierLineRelation::OnSupportingLine | BezierLineRelation::Unresolved => {
            Classification::Decided(None)
        }
    }
}

fn line_image_regions_from_curve_spans(
    spans: Vec<BezierMonotoneSpan>,
    line_is_first: bool,
) -> Classification<BezierCurveRelation> {
    let line_span = match BezierMonotoneSpan::new(Real::zero(), Real::one()) {
        Ok(span) => span,
        Err(_) => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    let regions = spans
        .into_iter()
        .map(|curve_span| {
            if line_is_first {
                BezierCurveIntersectionRegion::new(line_span.clone(), curve_span)
            } else {
                BezierCurveIntersectionRegion::new(curve_span, line_span.clone())
            }
        })
        .collect();
    Classification::Decided(BezierCurveRelation::IntersectionRegions { regions })
}

fn zero_width_span(parameter: Real) -> Result<BezierMonotoneSpan, UncertaintyReason> {
    BezierMonotoneSpan::new(parameter.clone(), parameter).map_err(|_| UncertaintyReason::Ordering)
}

fn polynomial_relation_to_line(
    controls: &[&Point2],
    line: &LineSeg2,
    policy: &CurvePolicy,
) -> Classification<BezierLineRelation> {
    match controls {
        [start, control, end] => {
            QuadraticBezier2::new((*start).clone(), (*control).clone(), (*end).clone())
                .relation_to_line(line, policy)
        }
        [start, control1, control2, end] => CubicBezier2::new(
            (*start).clone(),
            (*control1).clone(),
            (*control2).clone(),
            (*end).clone(),
        )
        .relation_to_line(line, policy),
        _ => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

fn polynomial_point_at(controls: &[&Point2], t: Real) -> Option<Point2> {
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
    level.pop()
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
    fn from_rational(curve: &RationalQuadraticBezier2) -> Result<Self, UncertaintyReason> {
        Ok(Self {
            controls: curve.control_points().into_iter().cloned().collect(),
            weights: Some(curve.weights().into_iter().cloned().collect()),
            span: subdivision_span(Real::zero(), Real::one())?,
        })
    }

    fn from_polynomial(controls: &[&Point2]) -> Result<Self, UncertaintyReason> {
        Ok(Self {
            controls: controls.iter().map(|point| (*point).clone()).collect(),
            weights: None,
            span: subdivision_span(Real::zero(), Real::one())?,
        })
    }

    fn with_span(
        controls: Vec<Point2>,
        weights: Option<Vec<Real>>,
        start: Real,
        end: Real,
    ) -> Result<Self, UncertaintyReason> {
        Ok(Self {
            controls,
            weights,
            span: subdivision_span(start, end)?,
        })
    }

    fn control_box(&self, policy: &CurvePolicy) -> Classification<Aabb2> {
        Aabb2::from_points(self.controls.iter(), policy)
    }

    fn split_half(&self, policy: &CurvePolicy) -> Result<(Self, Self), UncertaintyReason> {
        let (left_controls, left_weights, right_controls, right_weights) =
            match self.weights.as_ref() {
                Some(weights) => split_rational_controls_half(&self.controls, weights, policy)?,
                None => {
                    let (left, right) = split_polynomial_controls_half(&self.controls)?;
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
            )?,
            Self::with_span(right_controls, right_weights, mid, self.span.end().clone())?,
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
    // used by Bezier clipping; see Bezier clipping. The exactness model's EGC
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
    if controls.is_empty() {
        return Err(UncertaintyReason::Unsupported);
    }
    let mut levels = vec![homogeneous_controls(controls, weights)];
    while levels.last().map(|level| level.len()).unwrap_or(0) > 1 {
        let Some(previous) = levels.last() else {
            return Err(UncertaintyReason::Unsupported);
        };
        let next = previous
            .windows(2)
            .map(|pair| midpoint_homogeneous(&pair[0], &pair[1]))
            .collect::<Result<Vec<_>, _>>()?;
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

fn split_polynomial_controls_half(
    points: &[Point2],
) -> Result<(Vec<Point2>, Vec<Point2>), UncertaintyReason> {
    if points.is_empty() {
        return Err(UncertaintyReason::Unsupported);
    }
    let mut levels = vec![points.to_vec()];
    while levels.last().map(|level| level.len()).unwrap_or(0) > 1 {
        let Some(previous) = levels.last() else {
            return Err(UncertaintyReason::Unsupported);
        };
        let next = previous
            .windows(2)
            .map(|pair| midpoint_point(&pair[0], &pair[1]))
            .collect::<Result<Vec<_>, _>>()?;
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
    Ok((left, right))
}

fn subdivision_span(start: Real, end: Real) -> Result<BezierMonotoneSpan, UncertaintyReason> {
    BezierMonotoneSpan::new(start, end).map_err(|_| UncertaintyReason::Ordering)
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
) -> Result<HomogeneousControl, UncertaintyReason> {
    Ok(HomogeneousControl {
        x: midpoint_real(&first.x, &second.x)?,
        y: midpoint_real(&first.y, &second.y)?,
        weight: midpoint_real(&first.weight, &second.weight)?,
    })
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

fn midpoint_point(first: &Point2, second: &Point2) -> Result<Point2, UncertaintyReason> {
    Ok(Point2::new(
        midpoint_real(first.x(), second.x())?,
        midpoint_real(first.y(), second.y())?,
    ))
}

fn midpoint_real(first: &Real, second: &Real) -> Result<Real, UncertaintyReason> {
    divide_by_positive_integer(first + second, 2)
}

fn point_equal(a: &Point2, b: &Point2, policy: &CurvePolicy) -> Option<bool> {
    is_zero(&a.distance_squared(b), policy)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: i32, y: i32) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn empty_polynomial_control_relation_reports_unsupported() {
        let rational = RationalQuadraticBezier2::try_new(
            point(0, 0),
            point(1, 1),
            point(2, 0),
            Real::one(),
            Real::one(),
            Real::one(),
        )
        .unwrap();

        assert_eq!(
            relation_to_polynomial_bezier(&rational, &[], &CurvePolicy::certified()),
            Classification::Uncertain(UncertaintyReason::Unsupported)
        );
        assert_eq!(polynomial_point_at(&[], Real::zero()), None);
    }

    #[test]
    fn rational_polynomial_bernstein_helpers_keep_exact_weights() {
        assert_eq!(
            elevate_quadratic_bernstein_to_quartic([Real::zero(), Real::zero(), Real::one()])
                .unwrap(),
            [
                Real::zero(),
                Real::zero(),
                (Real::one() / Real::from(6_i8)).unwrap(),
                (Real::one() / Real::from(2_i8)).unwrap(),
                Real::one()
            ]
        );

        assert_eq!(
            multiply_quadratic_bernstein_to_quartic(
                [Real::one(), Real::one(), Real::one()],
                [Real::one(), Real::one(), Real::one()],
            )
            .unwrap(),
            [
                Real::one(),
                Real::one(),
                Real::one(),
                Real::one(),
                Real::one()
            ]
        );
    }

    #[test]
    fn rational_midpoint_subdivision_reports_empty_controls() {
        assert_eq!(
            subdivide_scalar_bernstein_half(&[]),
            Err(UncertaintyReason::Unsupported)
        );
    }

    #[test]
    fn rational_midpoint_subdivision_retains_both_de_casteljau_diagonals() {
        let controls = [0_i8, 2, 4, 6, 8].map(Real::from);
        let (left, right) = subdivide_scalar_bernstein_half(&controls).unwrap();

        assert_eq!(left, [0_i8, 1, 2, 3, 4].map(Real::from));
        assert_eq!(right, [4_i8, 5, 6, 7, 8].map(Real::from));
    }

    #[test]
    fn rational_midpoint_subdivision_keeps_exact_de_casteljau_values() {
        let (left, right) =
            subdivide_scalar_quadratic_half([Real::zero(), Real::from(2_i8), Real::from(4_i8)])
                .unwrap();

        assert_eq!(left, [Real::zero(), Real::one(), Real::from(2_i8)]);
        assert_eq!(
            right,
            [Real::from(2_i8), Real::from(3_i8), Real::from(4_i8)]
        );
        assert_eq!(
            scalar_quadratic_at_half(&[Real::zero(), Real::from(2_i8), Real::from(4_i8)]).unwrap(),
            Real::from(2_i8)
        );
    }

    #[test]
    fn rational_polynomial_dyadic_promotion_uses_checked_parameters() {
        let rational = RationalQuadraticBezier2::try_new(
            point(0, 0),
            point(1, 2),
            point(2, 0),
            Real::one(),
            Real::from(2_i8),
            Real::one(),
        )
        .unwrap();
        let y_mid = (Real::from(8_i8) / Real::from(3_i8)).unwrap();
        let p0 = point(0, 0);
        let p1 = Point2::new(Real::one(), y_mid);
        let p2 = point(2, 0);
        let controls = vec![&p0, &p1, &p2];

        let relation = same_parameter_dyadic_rational_polynomial_relation(
            &rational,
            &controls,
            &CurvePolicy::certified(),
        );

        match relation {
            Classification::Decided(Some(BezierCurveRelation::IntersectionPoints { points })) => {
                assert_eq!(points.len(), 1);
                assert_eq!(
                    points[0].point(),
                    &Point2::new(Real::one(), (Real::from(4_i8) / Real::from(3_i8)).unwrap())
                );
            }
            other => panic!("expected exact dyadic promotion, got {other:?}"),
        }
    }
}
