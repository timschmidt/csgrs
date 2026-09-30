//! Rational quadratic Bezier and conic primitives.
//!
//! Rational quadratics are the lowest-degree Bezier representation that can
//! carry non-parabolic conics exactly. The homogeneous evaluation below keeps
//! the polynomial numerator and weight denominator visible instead of
//! flattening to sampled chords, matching the exactness model's exact geometric computation
//! advice to preserve object structure until a certified predicate boundary;
//! The conic weight classifier follows the rational quadratic
//! treatment in the Bernstein and de Casteljau curve model.

use std::sync::Arc;

use hyperreal::{Real, RealSign, ZeroKnowledge};

use crate::bezier_parameter::quadratic_bernstein_to_power;
use crate::bezier_topology::exact_line_contact_relation_from_bernstein_distances;
use crate::bezier_topology::polynomial_roots_in_unit_interval_with_endpoints;
use crate::classify::{classify_oriented_line, is_zero, orient2_real_expr, real_sign};
use crate::{
    Aabb2, Axis2, BezierLineContactRelation, Classification, CubicBezier2, CurveContext,
    CurveError, LineSeg2, LineSide, Point2, QuadraticBezier2, RationalBezier2, UncertaintyReason,
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
    implicit_quadratic_conic: Option<Arc<[Real; 6]>>,
    circular_conic: Option<Arc<RationalQuadraticCircle2>>,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct RationalQuadraticCircle2 {
    pub(crate) center: Point2,
    pub(crate) radius_squared: Real,
    pub(crate) tangent_contacts: Option<Arc<[RationalQuadraticCircleTangentContact2]>>,
}

/// Construction evidence for a tangent contact between a circular join and
/// an analytic parallel, initially at one endpoint of that join. The
/// corresponding circle-incidence eliminant has the represented parameter
/// as a root of multiplicity at least two. The certificate belongs to the
/// supporting circle and survives restriction and extension. Each current
/// conic chart must independently decide whether it contains the contact.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct RationalQuadraticParallelCircleContact2 {
    pub(crate) parallel: crate::BezierParallel2,
    pub(crate) parameter: Real,
    pub(crate) point: Point2,
    pub(crate) eliminant_root_multiplicity: u8,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum RationalQuadraticCircleTangentContact2 {
    Parallel(RationalQuadraticParallelCircleContact2),
    Line { line: LineSeg2, point: Point2 },
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
        Self::try_new_with_common_weight_sign_and_implicit_conic(
            start,
            control,
            end,
            start_weight,
            control_weight,
            end_weight,
            None,
            None,
            None,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn try_new_with_common_weight_sign_and_implicit_conic(
        start: Point2,
        control: Point2,
        end: Point2,
        start_weight: Real,
        control_weight: Real,
        end_weight: Real,
        retained_common_weight_sign: Option<RealSign>,
        implicit_quadratic_conic: Option<Arc<[Real; 6]>>,
        circular_conic: Option<Arc<RationalQuadraticCircle2>>,
    ) -> Result<Self, CurveError> {
        if [
            start_weight.zero_status(),
            control_weight.zero_status(),
            end_weight.zero_status(),
        ]
        .contains(&ZeroKnowledge::Zero)
        {
            return Err(CurveError::ZeroRationalBezierWeight);
        }
        let classified_common_weight_sign = common_weight_sign_for_values(
            [&start_weight, &control_weight, &end_weight],
            &CurveContext::STRICT,
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
            implicit_quadratic_conic,
            circular_conic,
        })
    }

    pub(crate) fn retained_implicit_quadratic_conic(&self) -> Option<&Arc<[Real; 6]>> {
        self.implicit_quadratic_conic.as_ref()
    }

    pub(crate) fn retained_circular_conic(&self) -> Option<&Arc<RationalQuadraticCircle2>> {
        self.circular_conic.as_ref()
    }

    pub(crate) fn with_retained_conic_provenance(
        mut self,
        implicit_quadratic_conic: Option<Arc<[Real; 6]>>,
        circular_conic: Option<Arc<RationalQuadraticCircle2>>,
    ) -> Self {
        self.implicit_quadratic_conic = implicit_quadratic_conic;
        self.circular_conic = circular_conic;
        self
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

    pub(crate) fn common_nonzero_weight_sign(&self, policy: &CurveContext) -> Option<RealSign> {
        self.common_weight_sign
            .or_else(|| common_weight_sign_for_values(self.weights(), policy))
    }

    /// Evaluates the rational segment at affine parameter `t`.
    ///
    /// The homogeneous numerator and denominator use the same power basis for
    /// rational and arbitrary exact parameters. A zero denominator is a
    /// projective boundary, so this API returns explicit uncertainty instead
    /// of inventing an affine point.
    pub fn point_at(&self, t: Real, policy: &CurveContext) -> Classification<Point2> {
        let endpoint = match t.exact_rational_ref() {
            Some(t) if t.is_zero() => Some((&self.start, &self.start_weight)),
            Some(t) if t.is_one() => Some((&self.end, &self.end_weight)),
            _ => None,
        };
        if let Some((point, weight)) = endpoint {
            return match is_zero(weight, policy) {
                Some(false) => Classification::Decided(point.clone()),
                Some(true) => Classification::Uncertain(UncertaintyReason::Boundary),
                None => Classification::Uncertain(UncertaintyReason::RealSign),
            };
        }
        // In Q(w), with w^2 rational, conjugation keeps a rational parameter's
        // coordinates linear in w. Outside this field use the shared power
        // basis: unrestricted conjugation expands equivalent exact expressions
        // that their retained coordinate images cannot necessarily identify.
        if t.exact_rational_ref().is_some()
            && self.start_weight == Real::one()
            && self.end_weight == Real::one()
            && self.control_weight.exact_rational_ref().is_none()
            && self.control_points().iter().all(|point| {
                point.x().exact_rational_ref().is_some() && point.y().exact_rational_ref().is_some()
            })
        {
            let weight_squared = &self.control_weight * &self.control_weight;
            if weight_squared.exact_rational_ref().is_some()
                && let Some(point) = self.point_at_quadratic_weight(&t, &weight_squared)
            {
                return Classification::Decided(point);
            }
        }
        let denominator = self.denominator_at(&t);
        crate::rational_bezier_general::project_homogeneous(
            &denominator,
            || {
                [
                    evaluate_quadratic_power_basis(
                        self.weighted_coordinate_power_basis(Axis2::X),
                        &t,
                    ),
                    evaluate_quadratic_power_basis(
                        self.weighted_coordinate_power_basis(Axis2::Y),
                        &t,
                    ),
                ]
            },
            policy,
        )
    }

    fn point_at_quadratic_weight(&self, t: &Real, weight_squared: &Real) -> Option<Point2> {
        let one_minus_t = Real::one() - t;
        let u_squared = &one_minus_t * &one_minus_t;
        let t_squared = t * t;
        let unweighted = &u_squared + &t_squared;
        let middle_basis = Real::from(2_i8) * &one_minus_t * t;
        let conjugate_denominator =
            (&unweighted * &unweighted) - (&middle_basis * &middle_basis * weight_squared);
        if conjugate_denominator.zero_status() != ZeroKnowledge::NonZero {
            return None;
        }

        let coordinate = |start: &Real, control: &Real, end: &Real| {
            let unweighted_numerator = (&u_squared * start) + (&t_squared * end);
            let weighted_control = &middle_basis * control;
            let rational_part = (&unweighted_numerator * &unweighted)
                - (&weighted_control * &middle_basis * weight_squared);
            let radical_part = ((&weighted_control * &unweighted)
                - (&unweighted_numerator * &middle_basis))
                * &self.control_weight;
            ((rational_part + radical_part) / &conjugate_denominator).ok()
        };
        Some(Point2::new(
            coordinate(self.start.x(), self.control.x(), self.end.x())?,
            coordinate(self.start.y(), self.control.y(), self.end.y())?,
        ))
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
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> Classification<Vec<Real>> {
        rational_parameters_for_point(self, point, policy)
    }

    /// Classifies whether `point` lies anywhere on this finite conic segment.
    ///
    /// Denominator boundaries are reported as uncertainty instead of being
    /// projected into affine space. Use [`Self::parameters_for_point`] when the
    /// certified parameters themselves are needed by downstream topology.
    pub fn contains_point(&self, point: &Point2, policy: &CurveContext) -> Classification<bool> {
        self.parameters_for_point(point, policy)
            .map(|parameters| !parameters.is_empty())
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
    pub(crate) fn relation_to_line_with_contacts(
        &self,
        line: &LineSeg2,
        policy: &CurveContext,
    ) -> Classification<BezierLineContactRelation> {
        if let (Some(implicit_conic), Some(circular_conic)) = (
            self.retained_implicit_quadratic_conic(),
            self.retained_circular_conic(),
        ) {
            let promoted = RationalBezier2::try_new(
                self.control_points().into_iter().cloned().collect(),
                self.weights().into_iter().cloned().collect(),
            )
            .map(|curve| {
                curve.with_implicit_quadratic_conic(
                    implicit_conic.clone(),
                    Some(circular_conic.clone()),
                )
            });
            return match promoted {
                Ok(promoted) => promoted.relation_to_line_with_contacts(line, policy),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            };
        }
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
        policy: &CurveContext,
    ) -> Classification<Vec<Real>> {
        let weight_products = self.derivative_weight_products();
        let denominator = self.weight_power_basis();
        self.axis_monotone_parameters_with_basis(axis, &weight_products, &denominator, policy)
    }

    fn axis_monotone_parameters_with_basis(
        &self,
        axis: Axis2,
        weight_products: &(Real, Real, Real),
        denominator: &[Real; 3],
        policy: &CurveContext,
    ) -> Classification<Vec<Real>> {
        // After removing the irrelevant common factor two, the quotient
        // derivative has these quadratic Bernstein controls:
        //
        //   w0 w1 (p1 - p0), w0 w2 (p2 - p0), w1 w2 (p2 - p1).
        //
        // Forming them directly avoids separately expanding N, D, N', and D'
        // and then cancelling their cubic terms.
        let a = &weight_products.0
            * &(coordinate(self.control(), axis) - coordinate(self.start(), axis));
        let b =
            &weight_products.1 * &(coordinate(self.end(), axis) - coordinate(self.start(), axis));
        let c =
            &weight_products.2 * &(coordinate(self.end(), axis) - coordinate(self.control(), axis));
        let c0 = a.clone();
        let c1 = &b - &(Real::from(2_i8) * &a);
        let c2 = &a - &b + &c;
        let roots =
            match polynomial_roots_in_unit_interval_with_endpoints(c0, c1, c2, &a, &c, policy) {
                Classification::Decided(roots) => roots,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };

        let mut retained_roots = Vec::new();
        for root in roots {
            let denominator_at_root =
                ((&denominator[2] * &root) + &denominator[1]) * &root + &denominator[0];
            match is_zero(&denominator_at_root, policy) {
                Some(true) => return Classification::Uncertain(UncertaintyReason::Boundary),
                Some(false) => retained_roots.push(root),
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        Classification::Decided(retained_roots)
    }

    /// Returns a conservative exact bound for this rational conic.
    ///
    /// Equal nonzero weights use polynomial extrema. Other common-sign weights
    /// use the affine control hull; mixed weights share the general homogeneous
    /// subdivision bound after certifying that the denominator never vanishes.
    pub fn certified_bounds(&self) -> Classification<Aabb2> {
        if self.weights_known_same_nonzero_sign(&CurveContext::STRICT) == Some(true) {
            if self.weights_equal(&CurveContext::STRICT) == Some(true) {
                return QuadraticBezier2::new(
                    self.start.clone(),
                    self.control.clone(),
                    self.end.clone(),
                )
                .certified_bounds();
            }
            return Aabb2::from_points(self.control_points());
        }
        RationalBezier2::from(self.clone()).certified_bounds_classified()
    }

    /// Classifies the represented conic family from the homogeneous weights.
    pub fn conic_kind(&self, policy: &CurveContext) -> Classification<RationalQuadraticConicKind> {
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

    pub(crate) fn denominator_at(&self, t: &Real) -> Real {
        evaluate_quadratic_power_basis(self.weight_power_basis(), t)
    }

    fn weighted_line_distances(&self, line: &LineSeg2) -> [Real; 3] {
        let controls = self.control_points();
        let weights = self.weights();
        [
            orient2_real_expr(line.start(), line.end(), controls[0]) * weights[0],
            orient2_real_expr(line.start(), line.end(), controls[1]) * weights[1],
            orient2_real_expr(line.start(), line.end(), controls[2]) * weights[2],
        ]
    }

    fn weighted_coordinate_power_basis(&self, axis: Axis2) -> [Real; 3] {
        quadratic_bernstein_to_power(
            [
                coordinate(self.start(), axis) * &self.start_weight,
                coordinate(self.control(), axis) * &self.control_weight,
                coordinate(self.end(), axis) * &self.end_weight,
            ]
            .each_ref(),
        )
    }

    fn derivative_weight_products(&self) -> (Real, Real, Real) {
        (
            &self.start_weight * &self.control_weight,
            &self.start_weight * &self.end_weight,
            &self.control_weight * &self.end_weight,
        )
    }

    fn weight_power_basis(&self) -> [Real; 3] {
        quadratic_bernstein_to_power(self.weights())
    }

    fn weights_known_same_nonzero_sign(&self, policy: &CurveContext) -> Option<bool> {
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

    fn weights_equal(&self, policy: &CurveContext) -> Option<bool> {
        [
            &self.start_weight - &self.control_weight,
            &self.control_weight - &self.end_weight,
        ]
        .into_iter()
        .map(|difference| is_zero(&difference, policy))
        .try_fold(true, |same, item| item.map(|item| same && item))
    }
}

fn coordinate(point: &Point2, axis: Axis2) -> &Real {
    match axis {
        Axis2::X => point.x(),
        Axis2::Y => point.y(),
    }
}

fn evaluate_quadratic_power_basis(
    [constant, linear, quadratic]: [Real; 3],
    parameter: &Real,
) -> Real {
    if constant.exact_rational_ref().is_some()
        && linear.exact_rational_ref().is_some()
        && quadratic.exact_rational_ref().is_some()
        && parameter.exact_rational_ref().is_some()
    {
        let one = Real::one();
        return Real::exact_rational_signed_product_sum_known_exact(
            [true; 3],
            [
                [&quadratic, parameter, parameter],
                [&linear, parameter, &one],
                [&constant, &one, &one],
            ],
        );
    }
    ((&quadratic * parameter) + linear) * parameter + constant
}

#[derive(Clone, Debug, PartialEq)]
enum RationalPointRootSet {
    All,
    Roots(Vec<Real>),
}

fn rational_parameters_for_point(
    curve: &RationalQuadraticBezier2,
    point: &Point2,
    policy: &CurveContext,
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
    policy: &CurveContext,
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
    let [c0, c1, c2] = quadratic_bernstein_to_power(values.each_ref());
    polynomial_roots_in_unit_interval_with_endpoints(c0, c1, c2, &values[0], &values[2], policy)
        .map(RationalPointRootSet::Roots)
}

fn rational_point_parameters_from_root_sets(
    curve: &RationalQuadraticBezier2,
    point: &Point2,
    x_roots: RationalPointRootSet,
    y_roots: RationalPointRootSet,
    policy: &CurveContext,
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

impl QuadraticBezier2 {}

impl CubicBezier2 {}

/// Bernstein independence proves a finite rational image is constant exactly
/// when every coefficient of X-p.x*W and Y-p.y*W vanishes. Zero homogeneous
/// weights require no affine reconstruction. One nonzero residual disproves
/// constancy even when a different coefficient remains unresolved.
pub(crate) fn point_image_from_residuals(
    point: &Point2,
    residuals: impl IntoIterator<Item = [Real; 2]>,
    policy: &CurveContext,
) -> Classification<Option<Point2>> {
    let mut uncertain = false;
    for residual in residuals.into_iter().flatten() {
        match is_zero(&residual, policy) {
            Some(true) => {}
            Some(false) => return Classification::Decided(None),
            None => uncertain = true,
        }
    }
    if uncertain {
        Classification::Uncertain(UncertaintyReason::RealSign)
    } else {
        Classification::Decided(Some(point.clone()))
    }
}

fn common_weight_sign_for_values(weights: [&Real; 3], policy: &CurveContext) -> Option<RealSign> {
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

/// Returns endpoint/control signs when a quadratic Bernstein denominator is
/// certified nonzero throughout `[0, 1]`.
///
/// Equal-sign endpoints and an opposite-sign middle coefficient are regular
/// exactly when `w0*w2-w1^2 > 0`. Keeping this certificate here gives circle
/// recognition and general rational predicates one projective-pole authority.
/// A zero middle coefficient is also pole-free; only its control is infinite.
pub(crate) fn pole_free_quadratic_weight_signs(
    weights: [&Real; 3],
    policy: &CurveContext,
) -> Classification<Option<(RealSign, RealSign)>> {
    let endpoint_sign = match real_sign(weights[0], policy) {
        Some(sign @ (RealSign::Negative | RealSign::Positive)) => sign,
        Some(RealSign::Zero) => return Classification::Decided(None),
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    };
    match real_sign(weights[2], policy) {
        Some(sign) if sign == endpoint_sign => {}
        Some(_) => return Classification::Decided(None),
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    }
    let control_sign = match real_sign(weights[1], policy) {
        Some(sign) => sign,
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    };
    if control_sign == endpoint_sign || control_sign == RealSign::Zero {
        return Classification::Decided(Some((endpoint_sign, control_sign)));
    }
    match real_sign(&(weights[0] * weights[2] - weights[1] * weights[1]), policy) {
        Some(RealSign::Positive) => Classification::Decided(Some((endpoint_sign, control_sign))),
        Some(RealSign::Zero | RealSign::Negative) => Classification::Decided(None),
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn point_equal(a: &Point2, b: &Point2, policy: &CurveContext) -> Option<bool> {
    is_zero(&a.distance_squared(b), policy)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: i32, y: i32) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn conic_bounds_certify_mixed_weight_charts_and_reject_poles() {
        let ratio = |numerator: i8, denominator: i8| {
            (Real::from(numerator) / Real::from(denominator)).unwrap()
        };
        // W(t)=1-3t+3t^2 >= 1/4. These independent points also lie outside
        // the original affine control hull, so a mixed-weight hull is invalid.
        let witnesses = [
            point(0, 0),
            Point2::new(ratio(-1, 7), ratio(-3, 7)),
            point(1, -1),
            Point2::new(ratio(15, 7), ratio(-3, 7)),
            point(2, 0),
        ];
        for scale in [Real::one(), -Real::one()] {
            let finite = RationalQuadraticBezier2::try_new(
                point(0, 0),
                point(1, 1),
                point(2, 0),
                scale.clone(),
                &scale * ratio(-1, 2),
                scale.clone(),
            )
            .unwrap();
            // Replacing the middle weight with -1 gives W(t)=(1-2t)^2,
            // with a genuine pole at 1/2 for this numerator.
            let singular = RationalQuadraticBezier2::try_new(
                point(0, 0),
                point(1, 1),
                point(2, 0),
                scale.clone(),
                -&scale,
                scale,
            )
            .unwrap();
            for policy in [CurveContext::APPROXIMATE_512, CurveContext::STRICT] {
                let outcome = crate::policy::resolve_certified_value(&policy, |_| {
                    (finite.certified_bounds(), singular.certified_bounds())
                });
                assert_eq!(outcome.certainty, crate::CurveCertainty::Certified);
                let (
                    Classification::Decided(bounds),
                    Classification::Uncertain(UncertaintyReason::Boundary),
                ) = outcome.value
                else {
                    panic!("only the pole-free chart has a finite bound");
                };
                for witness in &witnesses {
                    assert_eq!(
                        bounds.contains_point(witness, &CurveContext::STRICT),
                        Classification::Decided(true)
                    );
                }
            }
        }
    }

    #[test]
    fn retained_mixed_weight_circle_decides_disjoint_line_relations() {
        let half_sqrt_two = (Real::one() / Real::from(2_i8)).unwrap().sqrt().unwrap();
        let conic = RationalQuadraticBezier2::try_new(
            point(-1, 0),
            point(0, 0),
            point(0, 1),
            Real::one(),
            -half_sqrt_two,
            Real::one(),
        )
        .unwrap();
        let Classification::Decided(Some(support)) =
            crate::arc_bezier::rational_quadratic_circular_arc(&conic, &CurveContext::STRICT)
                .unwrap()
        else {
            panic!("the pole-free mixed-weight conic must be recognized as a major circle");
        };
        assert_eq!(support.center(), &point(-1, 1));
        assert_eq!(support.radius_squared_ref(), &Real::one());
        let (implicit, circular) = crate::arc_bezier::circular_conic_provenance(&support);
        let conic = conic.with_retained_conic_provenance(Some(implicit), Some(circular));
        let promoted = RationalBezier2::from(conic.clone());
        let disjoint = LineSeg2::try_new(point(-2, 3), point(2, 3)).unwrap();

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            assert_eq!(
                conic.relation_to_line_with_contacts(&disjoint, &policy),
                Classification::Decided(BezierLineContactRelation::NoContact),
            );
            assert_eq!(
                promoted.relation_to_line_with_contacts(&disjoint, &policy),
                Classification::Decided(BezierLineContactRelation::NoContact),
            );
        }
    }

    #[test]
    fn quadratic_power_evaluation_fuses_exact_terms_and_preserves_symbolic_horner_form() {
        let parameter = (Real::one() / Real::from(3_i8)).unwrap();
        let coefficients = [Real::from(5_i8), Real::from(-7_i8), Real::from(11_i8)];
        let expected = ((&coefficients[2] * &parameter) + coefficients[1].clone()) * &parameter
            + coefficients[0].clone();
        assert_eq!(
            evaluate_quadratic_power_basis(coefficients, &parameter),
            expected
        );

        let coefficients = [
            Real::from(2_i8).sqrt().unwrap(),
            Real::from(3_i8).sqrt().unwrap(),
            Real::from(5_i8).sqrt().unwrap(),
        ];
        let expected = ((&coefficients[2] * &parameter) + coefficients[1].clone()) * &parameter
            + coefficients[0].clone();
        assert_eq!(
            evaluate_quadratic_power_basis(coefficients, &parameter),
            expected
        );
    }
}
