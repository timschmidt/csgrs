//! Exactness-aware topology helpers for polynomial Bezier segments.
//!
//! This module keeps Bezier topology predicates separate from the object
//! carriers in `bezier.rs`. The split follows the exactness model's exact geometric
//! computation model: preserve exact curve structure, then expose certified
//! predicates and explicit uncertainty at the branch boundary.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};

use crate::bezier_parameter::bernstein_to_power_coefficients;
use crate::classify::{
    classify_oriented_line, compare_reals, in_closed_unit_interval, is_zero, orient2_real_expr,
    real_sign,
};
use crate::{
    Aabb2, BezierParameter2, BezierParameterPolynomial, Classification, CubicBezier2, CurveContext,
    CurveError, CurveResult, LineSeg2, LineSide, Point2, QuadraticBezier2, UncertaintyReason,
};
use hypersolve::exact_factor::divide_by_linear_root;

/// Current finite dyadic frontier for exact same-parameter Bezier candidates.
///
/// This is deliberately a named implementation boundary rather than a hidden
/// tolerance. It marks the bisection parameters that the polynomial
/// curve/curve shortcuts prove exactly before handing remaining cases to
/// conservative subdivision.
const DYADIC_CANDIDATE_DENOMINATOR: i32 = 512;

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
pub(crate) struct BezierMonotoneSpan {
    start: Real,
    end: Real,
}

impl BezierMonotoneSpan {
    /// Constructs a closed monotone parameter span.
    pub fn new(start: Real, end: Real) -> CurveResult<Self> {
        match compare_reals(&start, &end, &CurveContext::STRICT) {
            Some(Ordering::Less | Ordering::Equal) => Ok(Self { start, end }),
            Some(Ordering::Greater) | None => Err(CurveError::Topology(
                "Bezier monotone span endpoints must be certified in nondecreasing order".into(),
            )),
        }
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

/// Certified contact kind for an exact Bezier/supporting-line root.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BezierLineContactKind {
    /// The signed line-distance polynomial has odd root multiplicity.
    Crossing,
    /// The signed line-distance polynomial has even root multiplicity.
    Tangent,
}

/// Exact signed-distance transition at a supporting-line crossing.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BezierLineCrossingDirection {
    /// The oriented-line predicate changes from negative to positive as the
    /// curve parameter increases.
    NegativeToPositive,
    /// The oriented-line predicate changes from positive to negative as the
    /// curve parameter increases.
    PositiveToNegative,
}

/// Certified exact root of a Bezier/supporting-line predicate.
///
/// The parameter remains represented or algebraically isolated. Contact kind
/// is decided from exact root-multiplicity parity, which correctly handles
/// higher-order crossings as well as ordinary tangencies. This keeps
/// contact classification in the algebraic predicate layer, following
/// the exactness model's exact geometric computation boundary. The
/// signed-distance Bernstein polynomial identities are the standard Bezier
/// formulas in the Bernstein and de Casteljau curve model, while
/// Sturm isolators retain irrational roots without sampling.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BezierLineContact {
    parameter: BezierParameter2,
    kind: BezierLineContactKind,
    crossing_direction: Option<BezierLineCrossingDirection>,
    tangent_side: Option<LineSide>,
    supporting_line_parameter: Option<Real>,
}

/// Complete supporting-line relation with exact root contact classification.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum BezierLineContactRelation {
    /// The Bezier control hull is certified to lie strictly on one side.
    ControlHullDisjoint {
        /// The side containing the control hull.
        side: LineSide,
    },
    /// Every Bezier control point is certified on the supporting line.
    OnSupportingLine,
    /// The finite curve does not meet the supporting line.
    NoContact,
    /// Certified exact line roots with crossing/tangent classification.
    Contacts {
        /// Sorted unique exact contacts in the queried parameter domain.
        contacts: Vec<BezierLineContact>,
    },
}

impl BezierLineContact {
    /// Constructs contact evidence at an exact affine parameter.
    /// The incidence solver owns admission to its queried parameter domain.
    pub fn new(parameter: BezierParameter2, kind: BezierLineContactKind) -> Self {
        Self {
            parameter,
            kind,
            crossing_direction: None,
            tangent_side: None,
            supporting_line_parameter: None,
        }
    }

    /// Returns the exact Bezier parameter of the contact.
    pub const fn parameter(&self) -> &BezierParameter2 {
        &self.parameter
    }

    /// Returns the certified contact kind.
    pub const fn kind(&self) -> BezierLineContactKind {
        self.kind
    }

    /// Returns the certified signed-distance transition for crossing contacts.
    ///
    /// Contacts constructed directly with [`Self::new`] do not claim this
    /// additional evidence. Contacts emitted by the exact supporting-line
    /// solver carry it for every crossing.
    pub const fn crossing_direction(&self) -> Option<BezierLineCrossingDirection> {
        self.crossing_direction
    }

    /// Returns the strict oriented-line side occupied on both sides of a tangency.
    pub const fn tangent_side(&self) -> Option<LineSide> {
        self.tangent_side
    }

    pub(crate) const fn supporting_line_parameter(&self) -> Option<&Real> {
        self.supporting_line_parameter.as_ref()
    }

    pub(crate) fn with_crossing_direction(
        parameter: BezierParameter2,
        kind: BezierLineContactKind,
        crossing_direction: Option<BezierLineCrossingDirection>,
    ) -> CurveResult<Self> {
        if (kind == BezierLineContactKind::Crossing) != crossing_direction.is_some() {
            return Err(CurveError::Topology(
                "Bezier line crossing direction must match crossing contact kind".into(),
            ));
        }
        let mut contact = Self::new(parameter, kind);
        contact.crossing_direction = crossing_direction;
        Ok(contact)
    }

    pub(crate) fn with_tangent_side(
        parameter: BezierParameter2,
        side: LineSide,
    ) -> CurveResult<Self> {
        if side == LineSide::On {
            return Err(CurveError::Topology(
                "Bezier line tangency requires a strict neighboring side".into(),
            ));
        }
        let mut contact = Self::new(parameter, BezierLineContactKind::Tangent);
        contact.tangent_side = Some(side);
        Ok(contact)
    }

    pub(crate) fn with_crossing_direction_and_line_parameter(
        parameter: BezierParameter2,
        kind: BezierLineContactKind,
        crossing_direction: Option<BezierLineCrossingDirection>,
        supporting_line_parameter: Real,
    ) -> CurveResult<Self> {
        let mut contact = Self::with_crossing_direction(parameter, kind, crossing_direction)?;
        contact.supporting_line_parameter = Some(supporting_line_parameter);
        Ok(contact)
    }
}

impl QuadraticBezier2 {
    /// Returns derivative-root parameters that split this curve into spans
    /// monotone along `axis`.
    ///
    /// For a degree-`n` Bezier, coordinate extrema can occur only where the
    /// corresponding derivative Bezier has a zero. This is the standard
    /// derivative-control-polygon fact used for Bezier bounds; see the Bernstein and de Casteljau curve model. Roots are retained as exact [`Real`] parameters and filtered by
    /// certified closed-unit-interval comparisons.
    pub fn axis_monotone_parameters(&self, axis: Axis2) -> crate::ExactCurveResult<Vec<Real>> {
        crate::ExactCurveError::decided_for(
            crate::CurveOperation2::Classification,
            crate::CurveFamily2::QuadraticBezier,
            self.axis_monotone_parameters_with_policy(axis, &crate::policy::principal_context()),
        )
    }

    /// [`Self::axis_monotone_parameters`] under an explicit predicate policy.
    pub(crate) fn axis_monotone_parameters_with_policy(
        &self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> Classification<Vec<Real>> {
        derivative_roots_quadratic(axis_values3(self.control_points(), axis), policy)
    }

    /// Returns a certified Bezier bounding box from endpoints and coordinate extrema.
    /// Extrema are certified independently on each axis, without topology policy.
    pub fn certified_bounds(&self) -> Classification<Aabb2> {
        bounds_from_axis_extrema(
            [self.start(), self.end()],
            [
                self.axis_monotone_parameters_with_policy(Axis2::X, &CurveContext::STRICT),
                self.axis_monotone_parameters_with_policy(Axis2::Y, &CurveContext::STRICT),
            ],
            |parameter| self.point_at(parameter),
        )
    }

    /// Classifies represented supporting-line roots as crossings or tangencies.
    ///
    /// This is the contact-detail companion to [`Self::relation_to_line`].
    /// It preserves bracket-only roots as isolating spans and labels only
    /// roots whose exact parameter is represented by the current scalar API.
    pub(crate) fn relation_to_line_with_contacts(
        &self,
        line: &LineSeg2,
        policy: &CurveContext,
    ) -> Classification<BezierLineContactRelation> {
        if let Some(image) = self.retained_exact_line_image() {
            let (image_dx, image_dy) = image.delta();
            let (line_dx, line_dy) = line.delta();
            let denominator = Real::signed_product_sum(
                [true, false],
                [[&image_dx, &line_dy], [&image_dy, &line_dx]],
            );
            match real_sign(&denominator, policy) {
                Some(RealSign::Zero) => {
                    return match classify_oriented_line(
                        line.start(),
                        line.end(),
                        image.start(),
                        policy,
                    ) {
                        Classification::Decided(LineSide::On) => {
                            Classification::Decided(BezierLineContactRelation::OnSupportingLine)
                        }
                        Classification::Decided(side) => Classification::Decided(
                            BezierLineContactRelation::ControlHullDisjoint { side },
                        ),
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    };
                }
                Some(RealSign::Positive | RealSign::Negative) => {}
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
            let Ok(denominator_reciprocal) = denominator.inverse_ref_assuming_nonzero() else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            let (from_image_x, from_image_y) = line.start().delta_from(image.start());
            let source_numerator = Real::signed_product_sum(
                [true, false],
                [[&from_image_x, &line_dy], [&from_image_y, &line_dx]],
            );
            let source_parameter = source_numerator * &denominator_reciprocal;
            match in_closed_unit_interval(&source_parameter, policy) {
                Some(false) => {
                    return Classification::Decided(BezierLineContactRelation::NoContact);
                }
                Some(true) => {}
                None => return Classification::Uncertain(UncertaintyReason::Ordering),
            }
            let line_numerator = Real::signed_product_sum(
                [true, false],
                [[&from_image_x, &image_dy], [&from_image_y, &image_dx]],
            );
            let line_parameter = line_numerator * denominator_reciprocal;
            let crossing_direction = match real_sign(
                &Real::signed_product_sum(
                    [true, false],
                    [[&line_dx, &image_dy], [&line_dy, &image_dx]],
                ),
                policy,
            ) {
                Some(RealSign::Positive) => Some(BezierLineCrossingDirection::NegativeToPositive),
                Some(RealSign::Negative) => Some(BezierLineCrossingDirection::PositiveToNegative),
                Some(RealSign::Zero) | None => {
                    return Classification::Uncertain(UncertaintyReason::RealSign);
                }
            };
            let contact = match BezierLineContact::with_crossing_direction_and_line_parameter(
                BezierParameter2::Exact(source_parameter),
                BezierLineContactKind::Crossing,
                crossing_direction,
                line_parameter,
            ) {
                Ok(contact) => contact,
                Err(_) => return Classification::Uncertain(UncertaintyReason::Ordering),
            };
            return Classification::Decided(BezierLineContactRelation::Contacts {
                contacts: vec![contact],
            });
        }
        relation_to_line_with_contacts(self.control_points().as_slice(), line, policy)
    }

    /// Returns all certified parameters where `point` lies on this quadratic.
    ///
    /// This is the existential point-on-curve solver for polynomial
    /// quadratics. It solves the x/y Bernstein coordinate equations as exact
    /// low-degree scalar polynomials, then re-evaluates candidate parameters
    /// before exposing them. Keeping algebraic candidates exact until a
    /// certified predicate accepts them follows exact-computation discipline. The
    /// Bernstein-to-power conversion is the standard Bezier identity described
    /// by the Bernstein and de Casteljau curve model.
    pub fn parameters_for_point(&self, point: &Point2) -> crate::ExactCurveResult<Vec<Real>> {
        crate::ExactCurveError::decided_for(
            crate::CurveOperation2::Classification,
            crate::CurveFamily2::QuadraticBezier,
            self.parameters_for_point_with_policy(point, &crate::policy::principal_context()),
        )
    }

    /// [`Self::parameters_for_point`] under an explicit predicate policy.
    pub(crate) fn parameters_for_point_with_policy(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> Classification<Vec<Real>> {
        quadratic_parameters_for_point(self.control_points(), point, policy)
    }

    /// Classifies whether `point` lies anywhere on this quadratic segment.
    ///
    /// The result is decided only when the exact parameter solver can certify
    /// the complete finite-curve query. Use [`Self::parameters_for_point`] when
    /// the caller needs the retained exact parameters for downstream topology.
    pub fn contains_point(&self, point: &Point2) -> crate::ExactCurveResult<bool> {
        crate::ExactCurveError::decided_for(
            crate::CurveOperation2::Classification,
            crate::CurveFamily2::QuadraticBezier,
            self.contains_point_with_policy(point, &crate::policy::principal_context()),
        )
    }

    /// [`Self::contains_point`] under an explicit predicate policy.
    pub(crate) fn contains_point_with_policy(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> Classification<bool> {
        self.parameters_for_point_with_policy(point, policy)
            .map(|parameters| !parameters.is_empty())
    }
}

impl CubicBezier2 {
    /// Returns derivative-root parameters that split this curve into spans
    /// monotone along `axis`.
    pub fn axis_monotone_parameters(&self, axis: Axis2) -> crate::ExactCurveResult<Vec<Real>> {
        crate::ExactCurveError::decided_for(
            crate::CurveOperation2::Classification,
            crate::CurveFamily2::CubicBezier,
            self.axis_monotone_parameters_with_policy(axis, &crate::policy::principal_context()),
        )
    }

    /// [`Self::axis_monotone_parameters`] under an explicit predicate policy.
    pub(crate) fn axis_monotone_parameters_with_policy(
        &self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> Classification<Vec<Real>> {
        derivative_roots_cubic(axis_values4(self.control_points(), axis), policy)
    }

    /// Returns a certified Bezier bounding box from endpoints and coordinate extrema.
    /// Extrema are certified independently on each axis, without topology policy.
    pub fn certified_bounds(&self) -> Classification<Aabb2> {
        bounds_from_axis_extrema(
            [self.start(), self.end()],
            [
                self.axis_monotone_parameters_with_policy(Axis2::X, &CurveContext::STRICT),
                self.axis_monotone_parameters_with_policy(Axis2::Y, &CurveContext::STRICT),
            ],
            |parameter| self.point_at(parameter),
        )
    }

    /// Classifies represented supporting-line roots as crossings or tangencies.
    ///
    /// See [`QuadraticBezier2::relation_to_line_with_contacts`] for the
    /// exactness contract.
    pub(crate) fn relation_to_line_with_contacts(
        &self,
        line: &LineSeg2,
        policy: &CurveContext,
    ) -> Classification<BezierLineContactRelation> {
        relation_to_line_with_contacts(self.control_points().as_slice(), line, policy)
    }

    /// Returns certified dyadic subdivision parameters where `point` lies on this cubic.
    ///
    /// This is intentionally a finite candidate probe, not the complete
    /// existential point-on-cubic solver. It tests the dyadic parameters that
    /// the subdivision relation already materializes exactly and re-evaluates
    /// the cubic before returning a parameter. The current candidate set is
    /// the non-endpoint dyadic bisection parameters through
    /// five-hundred-twelfths, so it remains a certified finite shortcut
    /// rather than a premature cubic resultant solver. That
    /// keeps the branch boundary in the exact-geometric-computation sense;
    /// The exact de Casteljau evaluation and dyadic
    /// subdivision identities follow the Bernstein and de Casteljau curve model.
    pub fn dyadic_parameters_for_point(
        &self,
        point: &Point2,
    ) -> crate::ExactCurveResult<Vec<Real>> {
        crate::ExactCurveError::decided_for(
            crate::CurveOperation2::Classification,
            crate::CurveFamily2::CubicBezier,
            self.dyadic_parameters_for_point_with_policy(
                point,
                &crate::policy::principal_context(),
            ),
        )
    }

    /// [`Self::dyadic_parameters_for_point`] under an explicit predicate policy.
    pub(crate) fn dyadic_parameters_for_point_with_policy(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> Classification<Vec<Real>> {
        cubic_dyadic_parameters_for_point(self, point, policy)
    }
}

fn bounds_from_axis_extrema(
    endpoints: [&Point2; 2],
    axis_roots: [Classification<Vec<Real>>; 2],
    point_at: impl Fn(Real) -> Point2,
) -> Classification<Aabb2> {
    let mut samples: Vec<Point2> = endpoints.into_iter().cloned().collect();
    // A box needs each coordinate's extrema, not their joint parameter order.
    // In particular, distinct near-endpoint roots must never be merged through
    // a topology tolerance. Each axis root is evaluated just once.
    for roots in axis_roots {
        match roots {
            Classification::Decided(roots) => samples.extend(roots.into_iter().map(&point_at)),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Aabb2::from_points(samples.iter())
}

fn cubic_dyadic_parameters_for_point(
    curve: &CubicBezier2,
    point: &Point2,
    policy: &CurveContext,
) -> Classification<Vec<Real>> {
    let mut parameters = Vec::new();
    let candidates = match dyadic_subdivision_candidate_parameters() {
        Ok(candidates) => candidates,
        Err(reason) => return Classification::Uncertain(reason),
    };
    for parameter in candidates {
        match point_coordinates_equal(&curve.point_at(parameter.clone()), point, policy) {
            Some(true) => {
                if let Err(reason) = push_unique_sorted(&mut parameters, parameter, policy) {
                    return Classification::Uncertain(reason);
                }
            }
            Some(false) => {}
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }
    Classification::Decided(parameters)
}

fn dyadic_subdivision_candidate_parameters() -> Result<Vec<Real>, UncertaintyReason> {
    (1_i32..DYADIC_CANDIDATE_DENOMINATOR)
        .map(dyadic_subdivision_candidate_parameter)
        .collect()
}

fn dyadic_subdivision_candidate_parameter(numerator: i32) -> Result<Real, UncertaintyReason> {
    let frontier_unit = divide_by_positive_integer(Real::one(), DYADIC_CANDIDATE_DENOMINATOR)?;
    Ok(&frontier_unit * &Real::from(numerator))
}

fn control_sides_against_line(
    controls: &[&Point2],
    line: &LineSeg2,
    policy: &CurveContext,
) -> Classification<([LineSide; 4], usize)> {
    if controls.len() > 4 {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    }
    let mut sides = [LineSide::On; 4];
    for (index, point) in controls.iter().enumerate() {
        match classify_oriented_line(line.start(), line.end(), point, policy) {
            Classification::Decided(side) => sides[index] = side,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided((sides, controls.len()))
}

fn relation_to_line_with_contacts(
    controls: &[&Point2],
    line: &LineSeg2,
    policy: &CurveContext,
) -> Classification<BezierLineContactRelation> {
    let (decided_sides, side_count) = match control_sides_against_line(controls, line, policy) {
        Classification::Decided(sides) => sides,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let decided_sides = &decided_sides[..side_count];

    if decided_sides.iter().all(|side| *side == LineSide::On) {
        return Classification::Decided(BezierLineContactRelation::OnSupportingLine);
    }
    if decided_sides
        .iter()
        .all(|side| matches!(side, LineSide::Left))
    {
        return Classification::Decided(BezierLineContactRelation::ControlHullDisjoint {
            side: LineSide::Left,
        });
    }
    if decided_sides
        .iter()
        .all(|side| matches!(side, LineSide::Right))
    {
        return Classification::Decided(BezierLineContactRelation::ControlHullDisjoint {
            side: LineSide::Right,
        });
    }

    let distances = controls
        .iter()
        .map(|point| orient2_real_expr(line.start(), line.end(), point))
        .collect::<Vec<_>>();
    exact_line_contact_relation_from_bernstein_distances(distances, policy)
}

pub(crate) fn exact_line_contact_relation_from_bernstein_distances(
    distances: Vec<Real>,
    policy: &CurveContext,
) -> Classification<BezierLineContactRelation> {
    if distances
        .iter()
        .any(|distance| distance.exact_rational_ref().is_none())
        && let [d0, d1, d2] = distances.as_slice()
        && let Classification::Decided(relation) =
            exact_quadratic_line_contact_relation([d0, d1, d2], policy)
    {
        return Classification::Decided(relation);
    }
    let polynomial =
        match BezierParameterPolynomial::try_new_bernstein_basis_with_policy(distances, policy) {
            Ok(Classification::Decided(polynomial)) => polynomial,
            Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
    exact_line_contact_relation_from_polynomial(
        polynomial,
        &crate::CurveParameterRange2::unit(),
        policy,
    )
}

/// Solves a quadratic line-incidence polynomial after removing one
/// construction-certified simple crossing.
///
/// A quadratic has at most one remaining root. Its crossing direction is the
/// opposite of the certified root because the derivatives at two distinct
/// simple roots of the same quadratic have opposite signs. The certificate
/// therefore avoids inverse point-to-parameter replay and root sorting without
/// weakening completeness.
pub(crate) fn exact_quadratic_line_contact_relation_with_certified_crossing(
    distances: [Real; 3],
    parameter: &Real,
    crossing_direction: BezierLineCrossingDirection,
    policy: &CurveContext,
) -> Classification<BezierLineContactRelation> {
    let coefficients = match bernstein_to_power_coefficients(distances.to_vec()) {
        Ok(coefficients) => coefficients,
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    let quotient = divide_by_linear_root(&coefficients, parameter);
    let known = match BezierLineContact::with_crossing_direction(
        BezierParameter2::Exact(parameter.clone()),
        BezierLineContactKind::Crossing,
        Some(crossing_direction),
    ) {
        Ok(contact) => contact,
        Err(_) => return Classification::Uncertain(UncertaintyReason::Ordering),
    };
    let Some(linear) = quotient.get(1) else {
        return Classification::Decided(BezierLineContactRelation::Contacts {
            contacts: vec![known],
        });
    };
    let Some(linear_sign) = real_sign(linear, policy) else {
        return Classification::Uncertain(UncertaintyReason::RealSign);
    };
    if linear_sign == RealSign::Zero {
        return match quotient
            .first()
            .and_then(|constant| real_sign(constant, policy))
        {
            Some(RealSign::Positive | RealSign::Negative) => {
                Classification::Decided(BezierLineContactRelation::Contacts {
                    contacts: vec![known],
                })
            }
            Some(RealSign::Zero) => Classification::Uncertain(UncertaintyReason::Boundary),
            None => Classification::Uncertain(UncertaintyReason::RealSign),
        };
    }
    let Some(constant) = quotient.first() else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let Ok(other_parameter) = (-constant.clone()) / linear else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    match in_closed_unit_interval(&other_parameter, policy) {
        Some(false) => Classification::Decided(BezierLineContactRelation::Contacts {
            contacts: vec![known],
        }),
        None => Classification::Uncertain(UncertaintyReason::Ordering),
        Some(true) => {
            let other_direction = match crossing_direction {
                BezierLineCrossingDirection::NegativeToPositive => {
                    BezierLineCrossingDirection::PositiveToNegative
                }
                BezierLineCrossingDirection::PositiveToNegative => {
                    BezierLineCrossingDirection::NegativeToPositive
                }
            };
            let other = match BezierLineContact::with_crossing_direction(
                BezierParameter2::Exact(other_parameter),
                BezierLineContactKind::Crossing,
                Some(other_direction),
            ) {
                Ok(contact) => contact,
                Err(_) => return Classification::Uncertain(UncertaintyReason::Ordering),
            };
            let contacts = match other
                .parameter()
                .cmp_by_interval_with_policy(known.parameter(), policy)
            {
                Ok(Classification::Decided(Ordering::Less)) => vec![other, known],
                Ok(Classification::Decided(Ordering::Greater)) => vec![known, other],
                Ok(Classification::Decided(Ordering::Equal)) => {
                    return Classification::Uncertain(UncertaintyReason::Boundary);
                }
                Ok(Classification::Uncertain(_)) | Err(_) => {
                    return Classification::Uncertain(UncertaintyReason::Ordering);
                }
            };
            Classification::Decided(BezierLineContactRelation::Contacts { contacts })
        }
    }
}

pub(crate) fn exact_polynomial_line_contact_relation_from_direction(
    controls: &[&Point2],
    origin: &Point2,
    direction_x: &Real,
    direction_y: &Real,
    policy: &CurveContext,
) -> Classification<BezierLineContactRelation> {
    let origin_projection = oriented_projection(direction_x, direction_y, origin);
    let projections = controls
        .iter()
        .map(|point| oriented_projection(direction_x, direction_y, point))
        .collect::<Vec<_>>();
    if let [p0, p1, p2] = projections.as_slice() {
        let distances = [
            p0 - &origin_projection,
            p1 - &origin_projection,
            p2 - &origin_projection,
        ];
        if let Classification::Decided(relation) = exact_quadratic_line_contact_relation(
            [&distances[0], &distances[1], &distances[2]],
            policy,
        ) {
            return Classification::Decided(relation);
        }
    }
    let mut coefficients = match bernstein_to_power_coefficients(projections) {
        Ok(coefficients) => coefficients,
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    coefficients[0] = &coefficients[0] - &origin_projection;
    exact_line_contact_relation_from_power_coefficients(coefficients, policy)
}

fn oriented_projection(direction_x: &Real, direction_y: &Real, point: &Point2) -> Real {
    let negative_x = -point.x().clone();
    Real::dot2_refs([direction_x, direction_y], [point.y(), &negative_x])
}

fn exact_line_contact_relation_from_power_coefficients(
    coefficients: Vec<Real>,
    policy: &CurveContext,
) -> Classification<BezierLineContactRelation> {
    let polynomial =
        match BezierParameterPolynomial::try_new_power_basis_with_policy(coefficients, policy) {
            Ok(Classification::Decided(polynomial)) => polynomial,
            Ok(Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
    if polynomial.degree() <= 2 {
        return exact_low_degree_power_line_contact_relation(polynomial.coefficients(), policy);
    }
    exact_line_contact_relation_from_polynomial(
        polynomial,
        &crate::CurveParameterRange2::unit(),
        policy,
    )
}

pub(crate) fn exact_line_contact_relation_from_polynomial(
    polynomial: BezierParameterPolynomial,
    range: &crate::CurveParameterRange2,
    policy: &CurveContext,
) -> Classification<BezierLineContactRelation> {
    let parameters = match crate::bezier_split::CurveParameterDomain2::new(range, None)
        .finite_roots(&polynomial, policy)
    {
        Ok(Classification::Decided(parameters)) => parameters,
        Ok(Classification::Uncertain(reason)) => {
            return Classification::Uncertain(reason);
        }
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    if parameters.is_empty() {
        return Classification::Decided(BezierLineContactRelation::NoContact);
    }
    let mut contacts = Vec::with_capacity(parameters.len());
    for parameter in parameters {
        let sign_after = match polynomial.sign_after_crossing_root(&parameter, policy) {
            Ok(Classification::Decided(sign_after)) => sign_after,
            Ok(Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
        let kind = if sign_after.is_some() {
            BezierLineContactKind::Crossing
        } else {
            BezierLineContactKind::Tangent
        };
        let crossing_direction = sign_after.map(|sign| match sign {
            RealSign::Positive => BezierLineCrossingDirection::NegativeToPositive,
            RealSign::Negative => BezierLineCrossingDirection::PositiveToNegative,
            RealSign::Zero => unreachable!("crossing residual sign is nonzero"),
        });
        let contact =
            match BezierLineContact::with_crossing_direction(parameter, kind, crossing_direction) {
                Ok(contact) => contact,
                Err(_) => return Classification::Uncertain(UncertaintyReason::Ordering),
            };
        contacts.push(contact);
    }
    Classification::Decided(BezierLineContactRelation::Contacts { contacts })
}

fn exact_low_degree_power_line_contact_relation(
    coefficients: &[Real],
    policy: &CurveContext,
) -> Classification<BezierLineContactRelation> {
    if coefficients.len() == 1 {
        return Classification::Decided(BezierLineContactRelation::NoContact);
    }
    let c0 = coefficients[0].clone();
    let c1 = coefficients[1].clone();
    let c2 = coefficients.get(2).cloned().unwrap_or_else(Real::zero);
    let parameters = match polynomial_roots_in_unit_interval(c0, c1.clone(), c2.clone(), policy) {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    if parameters.is_empty() {
        return Classification::Decided(BezierLineContactRelation::NoContact);
    }
    let two = Real::from(2_i8);
    let mut contacts = Vec::with_capacity(parameters.len());
    for parameter in parameters {
        let derivative = &c1 + &two * &c2 * &parameter;
        let (kind, crossing_direction) = match real_sign(&derivative, policy) {
            Some(RealSign::Negative) => (
                BezierLineContactKind::Crossing,
                Some(BezierLineCrossingDirection::PositiveToNegative),
            ),
            Some(RealSign::Positive) => (
                BezierLineContactKind::Crossing,
                Some(BezierLineCrossingDirection::NegativeToPositive),
            ),
            Some(RealSign::Zero) => (BezierLineContactKind::Tangent, None),
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        };
        let contact = match BezierLineContact::with_crossing_direction(
            BezierParameter2::Exact(parameter),
            kind,
            crossing_direction,
        ) {
            Ok(contact) => contact,
            Err(_) => return Classification::Uncertain(UncertaintyReason::Ordering),
        };
        contacts.push(contact);
    }
    Classification::Decided(BezierLineContactRelation::Contacts { contacts })
}

fn exact_quadratic_line_contact_relation(
    distances: [&Real; 3],
    policy: &CurveContext,
) -> Classification<BezierLineContactRelation> {
    let two = Real::from(2_i8);
    let c0 = distances[0].clone();
    let c1 = &two * &(distances[1] - distances[0]);
    let c2 = distances[0] - &(&two * distances[1]) + distances[2];
    let parameters = match polynomial_roots_in_unit_interval(c0, c1.clone(), c2.clone(), policy) {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    if parameters.is_empty() {
        return Classification::Decided(BezierLineContactRelation::NoContact);
    }

    let mut contacts = Vec::with_capacity(parameters.len());
    for parameter in parameters {
        let derivative = &c1 + &two * &c2 * &parameter;
        let (kind, crossing_direction) = match real_sign(&derivative, policy) {
            Some(RealSign::Negative) => (
                BezierLineContactKind::Crossing,
                Some(BezierLineCrossingDirection::PositiveToNegative),
            ),
            Some(RealSign::Positive) => (
                BezierLineContactKind::Crossing,
                Some(BezierLineCrossingDirection::NegativeToPositive),
            ),
            Some(RealSign::Zero) => (BezierLineContactKind::Tangent, None),
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        };
        let contact = match BezierLineContact::with_crossing_direction(
            BezierParameter2::Exact(parameter),
            kind,
            crossing_direction,
        ) {
            Ok(contact) => contact,
            Err(_) => return Classification::Uncertain(UncertaintyReason::Ordering),
        };
        contacts.push(contact);
    }
    Classification::Decided(BezierLineContactRelation::Contacts { contacts })
}

fn quadratic_parameters_for_point(
    controls: [&Point2; 3],
    point: &Point2,
    policy: &CurveContext,
) -> Classification<Vec<Real>> {
    // A point lies on a polynomial quadratic Bezier exactly when the x and y
    // coordinate Bernstein equations share a parameter in `[0, 1]`. Solving
    // those low-degree equations as exact `Real` roots and intersecting the
    // parameter sets follows the exactness model's EGC requirement to keep algebraic candidates
    // explicit until certified. The coordinate polynomial identities are the
    // standard Bernstein-to-power conversion described by the Bernstein and de Casteljau curve model.
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
    policy: &CurveContext,
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
    policy: &CurveContext,
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
            Some(true) => {
                if let Err(reason) = push_unique_sorted(&mut parameters, candidate, policy) {
                    return Classification::Uncertain(reason);
                }
            }
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

fn divide_by_positive_integer(
    numerator: Real,
    denominator: i32,
) -> Result<Real, UncertaintyReason> {
    (numerator / Real::from(denominator)).map_err(|_| UncertaintyReason::Unsupported)
}

fn derivative_roots_quadratic(
    values: [Real; 3],
    policy: &CurveContext,
) -> Classification<Vec<Real>> {
    let [p0, p1, p2] = values;
    let a = &p1 - &p0;
    let b = &p2 - &(Real::from(2_i8) * &p1) + &p0;
    linear_roots_in_unit_interval(a, b, policy)
}

fn derivative_roots_cubic(values: [Real; 4], policy: &CurveContext) -> Classification<Vec<Real>> {
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

pub(crate) fn polynomial_roots_in_unit_interval(
    c0: Real,
    c1: Real,
    c2: Real,
    policy: &CurveContext,
) -> Classification<Vec<Real>> {
    let start_is_root = c0.definitely_zero();
    let end_value = &(&c0 + &c1) + &c2;
    let end_is_root = end_value.definitely_zero();
    polynomial_roots_in_unit_interval_with_endpoint_flags(
        c0,
        c1,
        c2,
        start_is_root,
        end_is_root,
        policy,
    )
}

pub(crate) fn polynomial_roots_in_unit_interval_with_endpoints(
    c0: Real,
    c1: Real,
    c2: Real,
    start_value: &Real,
    end_value: &Real,
    policy: &CurveContext,
) -> Classification<Vec<Real>> {
    polynomial_roots_in_unit_interval_with_endpoint_flags(
        c0,
        c1,
        c2,
        start_value.definitely_zero(),
        end_value.definitely_zero(),
        policy,
    )
}

fn polynomial_roots_in_unit_interval_with_endpoint_flags(
    c0: Real,
    c1: Real,
    c2: Real,
    start_is_root: bool,
    end_is_root: bool,
    policy: &CurveContext,
) -> Classification<Vec<Real>> {
    match is_zero(&c2, policy) {
        Some(true) => return linear_roots_in_unit_interval(c0, c1, policy),
        Some(false) => {}
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    }

    // Extract exact endpoint roots before the quadratic formula. A root at
    // either boundary is represented directly by zero or one; constructing it
    // as `(-c1 +/- sqrt(discriminant)) / (2*c2)` can instead create a deeply
    // cancellative computable expression even when the power-basis endpoint
    // evaluation already reduces structurally to zero.
    match (start_is_root, end_is_root) {
        (true, true) => {
            return Classification::Decided(vec![Real::zero(), Real::one()]);
        }
        (true, false) => {
            let Ok(other_root) = (Real::zero() - &c1) / &c2 else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            return retain_unit_roots(vec![Real::zero(), other_root], policy);
        }
        (false, true) => {
            let Ok(other_root) = &c0 / &c2 else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            return retain_unit_roots(vec![Real::one(), other_root], policy);
        }
        (false, false) => {}
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
    policy: &CurveContext,
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

fn retain_unit_roots(roots: Vec<Real>, policy: &CurveContext) -> Classification<Vec<Real>> {
    let mut retained = Vec::new();
    for root in roots {
        match in_closed_unit_interval(&root, policy) {
            Some(true) => {
                if let Err(reason) = push_unique_sorted(&mut retained, root, policy) {
                    return Classification::Uncertain(reason);
                }
            }
            Some(false) => {}
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }
    Classification::Decided(retained)
}

fn push_unique_sorted(
    values: &mut Vec<Real>,
    value: Real,
    policy: &CurveContext,
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

fn point_equal(a: &Point2, b: &Point2, policy: &CurveContext) -> Option<bool> {
    is_zero(&a.distance_squared(b), policy)
}

fn point_coordinates_equal(a: &Point2, b: &Point2, policy: &CurveContext) -> Option<bool> {
    match (
        is_zero(&(a.x() - b.x()), policy),
        is_zero(&(a.y() - b.y()), policy),
    ) {
        (Some(true), Some(true)) => Some(true),
        (Some(false), _) | (_, Some(false)) => Some(false),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bounds_keep_close_extrema(family: usize) {
        use crate::{CurveCertainty, RationalBezier2, RationalQuadraticBezier2};

        let mut case = family as i64 * 64;
        for policy in [CurveContext::APPROXIMATE_512, CurveContext::STRICT] {
            for rational_gap in [false, true] {
                for axis in [Axis2::X, Axis2::Y] {
                    for near_end in [false, true] {
                        for reverse in [false, true] {
                            for maximum in [false, true] {
                                case += 1;
                                let gap = if rational_gap {
                                    Real::from(2_i8).powi_i64(-600).unwrap()
                                } else {
                                    Real::one()
                                        - Real::from(2_i8).powi_i64(-301 - case).unwrap().cos()
                                };
                                let parameter = if near_end { Real::one() - gap } else { gap };
                                let square = &parameter * &parameter;
                                let point = |x: Real, y: Real| {
                                    let y = if maximum { -y } else { y };
                                    match axis {
                                        Axis2::X => Point2::new(y, x),
                                        Axis2::Y => Point2::new(x, y),
                                    }
                                };
                                // Q(t)=(t,+/-(t-a)^2), with the axes optionally
                                // exchanged. Its exact extremum is (a,0).
                                let mut controls = [
                                    point(Real::zero(), square.clone()),
                                    point(
                                        (Real::one() / Real::from(2_i8)).unwrap(),
                                        &square - &parameter,
                                    ),
                                    point(
                                        Real::one(),
                                        Real::one() - Real::from(2_i8) * &parameter + &square,
                                    ),
                                ];
                                let witnesses = [
                                    controls[0].clone(),
                                    point(parameter, Real::zero()),
                                    controls[2].clone(),
                                ];
                                if reverse {
                                    controls.reverse();
                                }
                                let outcome = crate::policy::resolve_certified_value(
                                    &policy,
                                    |_| match family {
                                        0 => QuadraticBezier2::new(
                                            controls[0].clone(),
                                            controls[1].clone(),
                                            controls[2].clone(),
                                        )
                                        .certified_bounds(),
                                        1 => {
                                            let blend = |a: &Point2, b: &Point2| {
                                                Point2::new(
                                                    ((a.x() + Real::from(2_i8) * b.x())
                                                        / Real::from(3_i8))
                                                    .unwrap(),
                                                    ((a.y() + Real::from(2_i8) * b.y())
                                                        / Real::from(3_i8))
                                                    .unwrap(),
                                                )
                                            };
                                            CubicBezier2::new(
                                                controls[0].clone(),
                                                blend(&controls[0], &controls[1]),
                                                blend(&controls[2], &controls[1]),
                                                controls[2].clone(),
                                            )
                                            .certified_bounds()
                                        }
                                        2 | 3 => {
                                            let weight =
                                                if reverse { -Real::pi() } else { Real::pi() };
                                            let conic = RationalQuadraticBezier2::try_new(
                                                controls[0].clone(),
                                                controls[1].clone(),
                                                controls[2].clone(),
                                                weight.clone(),
                                                weight.clone(),
                                                weight,
                                            )
                                            .unwrap();
                                            if family == 2 {
                                                conic.certified_bounds()
                                            } else {
                                                RationalBezier2::from(conic)
                                                    .certified_bounds_classified()
                                            }
                                        }
                                        _ => unreachable!(),
                                    },
                                );
                                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                                let bounds = match outcome.value {
                                    Classification::Decided(bounds) => bounds,
                                    Classification::Uncertain(_) if !rational_gap => continue,
                                    Classification::Uncertain(reason) => {
                                        panic!("rational extrema must be bounded: {reason:?}");
                                    }
                                };
                                for witness in witnesses {
                                    for (lower, upper, value) in [
                                        (bounds.min_x(), bounds.max_x(), witness.x()),
                                        (bounds.min_y(), bounds.max_y(), witness.y()),
                                    ] {
                                        assert!(matches!(
                                            lower.certified_cmp_until(value, -3600).ordering(),
                                            Some(Ordering::Less | Ordering::Equal),
                                        ));
                                        assert!(matches!(
                                            upper.certified_cmp_until(value, -3600).ordering(),
                                            Some(Ordering::Greater | Ordering::Equal),
                                        ));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn quadratic_bounds_keep_close_extrema() {
        bounds_keep_close_extrema(0);
    }

    #[test]
    fn cubic_bounds_keep_close_extrema() {
        bounds_keep_close_extrema(1);
    }

    #[test]
    fn conic_bounds_keep_close_extrema() {
        bounds_keep_close_extrema(2);
    }

    #[test]
    fn homogeneous_bounds_keep_close_extrema() {
        bounds_keep_close_extrema(3);
    }

    #[test]
    fn certified_quadratic_crossing_deflation_retains_sorted_opposite_crossings() {
        let sixteenth = (Real::one() / Real::from(16_u8)).unwrap();
        let distances = [
            Real::from(3_u8) * &sixteenth,
            -Real::from(5_u8) * &sixteenth,
            Real::from(3_u8) * sixteenth,
        ];
        let known = (Real::from(3_u8) / Real::from(4_u8)).unwrap();
        let relation = exact_quadratic_line_contact_relation_with_certified_crossing(
            distances,
            &known,
            BezierLineCrossingDirection::NegativeToPositive,
            &CurveContext::STRICT,
        );
        let Classification::Decided(BezierLineContactRelation::Contacts { contacts }) = relation
        else {
            panic!("the certified quadratic crossings must be decided");
        };
        assert_eq!(contacts.len(), 2);
        let other = (Real::one() / Real::from(4_u8)).unwrap();
        assert_eq!(contacts[0].parameter().scalar(), Some(&other));
        assert_eq!(
            contacts[0].crossing_direction(),
            Some(BezierLineCrossingDirection::PositiveToNegative)
        );
        assert_eq!(contacts[1].parameter().scalar(), Some(&known));
        assert_eq!(
            contacts[1].crossing_direction(),
            Some(BezierLineCrossingDirection::NegativeToPositive)
        );
    }
}
