//! Exact tangent-vector predicates for path continuity.
//!
//! Path smoothing, fillet construction, and PCB length tuning all eventually
//! ask whether two retained path objects meet with compatible tangents. This
//! module keeps that decision in the exact predicate layer: vectors are
//! compared by cross and dot products through `hyperreal` and `hyperlimit`,
//! rather than by normalized floating angles. That follows Yap, "Towards Exact
//! Geometric Computation," and matches the object/predicate split used by CGAL
//! curve arrangement kernels. Farouki's PH-curve work motivates retaining
//! hodographs as exact objects for later feed-rate and arc-length stages.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy, point2_equal_with_policy};
use hyperreal::Real;
use hypersolve::{
    CandidateCertificationReport, Constraint, ConstraintKind, Expr, PreparedProblem, Problem,
    SymbolId, certify_candidate, context_from_problem,
};

use crate::arc::{CircularArc, ExplicitCircularArc};
use crate::bezier::{
    BezierParameter, CubicBezier, QuadraticBezier, RationalQuadraticBezier,
    RationalQuadraticBezierError,
};
use crate::segment::LinePathSegment;

/// Exact tangent-vector alignment classification.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TangentAlignment {
    /// The two nonzero vectors are parallel and point the same way.
    SameDirection,
    /// The two nonzero vectors are parallel and point opposite ways.
    OppositeDirection,
    /// The vectors are certified not parallel.
    NotParallel,
    /// At least one vector is exactly zero.
    Degenerate,
    /// The current exact comparison policy could not decide the products.
    Unknown,
}

/// Exact endpoint-and-tangent continuity classification for a path join.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TangentJoinClass {
    /// Endpoints coincide and tangent vectors point the same way.
    G1Continuous,
    /// Endpoints coincide but tangent vectors point opposite ways.
    ReversedTangent,
    /// Endpoints coincide but tangent vectors are certified nonparallel.
    Corner,
    /// Endpoints coincide but at least one tangent vector is zero.
    DegenerateTangent,
    /// Endpoints are certified different.
    EndpointMismatch,
    /// Endpoint equality or tangent alignment could not be decided.
    Unknown,
}

/// Exact report for one path join continuity check.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TangentJoinReport {
    /// Final join classification.
    pub class: TangentJoinClass,
    /// Whether endpoint equality was certified.
    pub endpoints_equal: Option<bool>,
    /// Tangent alignment if endpoint equality was certified.
    pub alignment: Option<TangentAlignment>,
}

/// Retained endpoint/tangent data for one path span.
#[derive(Clone, Debug, PartialEq)]
pub struct TangentSpan {
    /// Exact start endpoint.
    pub start: Point2,
    /// Exact start tangent vector.
    pub start_tangent: Point2,
    /// Exact end endpoint.
    pub end: Point2,
    /// Exact end tangent vector.
    pub end_tangent: Point2,
}

impl TangentSpan {
    /// Retain endpoint/tangent data from an exact line segment.
    ///
    /// The span stores the segment endpoints and the directed displacement
    /// vector exposed by [`LinePathSegment::start_tangent`] and
    /// [`LinePathSegment::end_tangent`]. This follows Yap's exact geometric
    /// computation boundary: downstream continuity and solver replay consume
    /// the constructed exact object, not a sampled or normalized direction.
    pub fn from_line_segment(segment: &LinePathSegment) -> Self {
        Self {
            start: segment.start().clone(),
            start_tangent: segment.start_tangent(),
            end: segment.end().clone(),
            end_tangent: segment.end_tangent(),
        }
    }

    /// Retain endpoint/tangent data from a cardinal circular arc.
    ///
    /// Arc tangents are radial vectors rotated exactly by traversal direction,
    /// matching the circular-arc arrangement-kernel style used by CGAL and the
    /// predicate-first discipline described by Yap. The vectors are not
    /// unit-normalized, so no square-root approximation is introduced.
    pub fn from_cardinal_arc(arc: &CircularArc) -> Self {
        Self {
            start: arc.start(),
            start_tangent: arc.start_tangent(),
            end: arc.end(),
            end_tangent: arc.end_tangent(),
        }
    }

    /// Retain endpoint/tangent data from an explicit circular arc.
    ///
    /// This preserves the exact retained endpoints plus the radial-rotation
    /// tangents used by [`ExplicitCircularArc`]. It is the preferred handoff
    /// from arc construction to chain-level G1 predicates and `hypersolve`
    /// residual replay.
    pub fn from_explicit_arc(arc: &ExplicitCircularArc) -> Self {
        Self {
            start: arc.start().clone(),
            start_tangent: arc.start_tangent(),
            end: arc.end().clone(),
            end_tangent: arc.end_tangent(),
        }
    }

    /// Retain endpoint/tangent data from a quadratic Bezier curve.
    ///
    /// The endpoint tangents are exact evaluations of the polynomial
    /// hodograph at `t = 0` and `t = 1`. Farouki's Pythagorean-hodograph work
    /// motivates carrying the hodograph as the feed-rate and continuity object
    /// instead of replacing the curve by sampled secants.
    pub fn from_quadratic_bezier(curve: &QuadraticBezier) -> Self {
        Self {
            start: curve.start().clone(),
            start_tangent: curve.derivative(parameter_zero()),
            end: curve.end().clone(),
            end_tangent: curve.derivative(parameter_one()),
        }
    }

    /// Retain endpoint/tangent data from a cubic Bezier curve.
    ///
    /// The tangent vectors come from the exact quadratic hodograph at the
    /// endpoints. Keeping that polynomial derivative structure gives later
    /// smoothing and `hypersolve` stages a precise residual input, consistent
    /// with Yap's object/predicate split and Farouki-style PH analysis.
    pub fn from_cubic_bezier(curve: &CubicBezier) -> Self {
        Self {
            start: curve.start().clone(),
            start_tangent: curve.derivative(parameter_zero()),
            end: curve.end().clone(),
            end_tangent: curve.derivative(parameter_one()),
        }
    }

    /// Retain endpoint/tangent data from a rational quadratic Bezier/conic.
    ///
    /// Rational conics use the homogeneous quotient-rule derivative from
    /// [`RationalQuadraticBezier::derivative`]. Division failure remains part
    /// of the result instead of silently degrading to a sampled secant; this is
    /// the exact-construction behavior required before G1 predicates or
    /// `hypersolve` replay accept a conic span.
    pub fn from_rational_quadratic_bezier(
        curve: &RationalQuadraticBezier,
    ) -> Result<Self, RationalQuadraticBezierError> {
        Ok(Self {
            start: curve.start().clone(),
            start_tangent: curve.derivative(parameter_zero())?,
            end: curve.end().clone(),
            end_tangent: curve.derivative(parameter_one())?,
        })
    }
}

/// Exact G1 continuity report for a chain of path spans.
#[derive(Clone, Debug, PartialEq)]
pub struct TangentChainReport {
    /// Join reports in adjacent-span order.
    pub joins: Vec<TangentJoinReport>,
}

/// Exact `hypersolve` replay report for a chain of G1 joins.
#[derive(Clone, Debug, PartialEq)]
pub struct G1ChainCertificationReport {
    /// Per-adjacent-join certification reports.
    pub joins: Vec<CandidateCertificationReport>,
}

impl TangentChainReport {
    /// Return whether every adjacent join is certified G1-continuous.
    pub fn all_g1_continuous(&self) -> bool {
        self.joins
            .iter()
            .all(|join| join.class == TangentJoinClass::G1Continuous)
    }

    /// Return the first non-G1 join index, if any.
    pub fn first_non_g1_join(&self) -> Option<usize> {
        self.joins
            .iter()
            .position(|join| join.class != TangentJoinClass::G1Continuous)
    }
}

impl G1ChainCertificationReport {
    /// Return whether every adjacent join was certified by exact replay.
    pub fn all_certified(&self) -> bool {
        self.joins
            .iter()
            .all(CandidateCertificationReport::all_satisfied)
    }

    /// Return the first join whose exact replay was not fully certified.
    pub fn first_uncertified_join(&self) -> Option<usize> {
        self.joins.iter().position(|join| !join.all_satisfied())
    }
}

/// Tiny `hypersolve` model for exact tangent cross-product certification.
#[derive(Clone, Debug)]
pub struct TangentAlignmentProblem {
    /// Solver problem containing candidate tangent components.
    pub problem: Problem,
    /// Symbol for the candidate tangent X component.
    pub candidate_x_symbol: SymbolId,
    /// Symbol for the candidate tangent Y component.
    pub candidate_y_symbol: SymbolId,
    /// Retained target tangent vector.
    pub target: Point2,
}

/// Tiny `hypersolve` model for exact G1 join certification.
#[derive(Clone, Debug)]
pub struct G1JoinProblem {
    /// Solver problem containing endpoint and tangent candidate components.
    pub problem: Problem,
    /// Symbol for candidate endpoint X.
    pub endpoint_x_symbol: SymbolId,
    /// Symbol for candidate endpoint Y.
    pub endpoint_y_symbol: SymbolId,
    /// Symbol for candidate tangent X.
    pub tangent_x_symbol: SymbolId,
    /// Symbol for candidate tangent Y.
    pub tangent_y_symbol: SymbolId,
    /// Retained target endpoint.
    pub target_endpoint: Point2,
    /// Retained target tangent vector.
    pub target_tangent: Point2,
}

/// Classify two exact tangent vectors by cross and dot products.
///
/// A zero cross product certifies parallelism; the dot product then separates
/// same-direction from opposite-direction tangency. No normalization is
/// performed, so this predicate works with radius-scaled arc tangents,
/// polynomial Bezier hodographs, rational conic derivatives, and line
/// direction vectors without introducing square roots.
pub fn classify_tangent_alignment(
    first: &Point2,
    second: &Point2,
    policy: PredicatePolicy,
) -> TangentAlignment {
    if is_zero_vector(first, policy) == Some(true) || is_zero_vector(second, policy) == Some(true) {
        return TangentAlignment::Degenerate;
    }
    if is_zero_vector(first, policy).is_none() || is_zero_vector(second, policy).is_none() {
        return TangentAlignment::Unknown;
    }

    let cross = tangent_cross(first, second);
    match compare_reals_with_policy(&cross, &Real::zero(), policy).value() {
        Some(Ordering::Less | Ordering::Greater) => TangentAlignment::NotParallel,
        Some(Ordering::Equal) => {
            let dot = tangent_dot(first, second);
            match compare_reals_with_policy(&dot, &Real::zero(), policy).value() {
                Some(Ordering::Greater) => TangentAlignment::SameDirection,
                Some(Ordering::Less) => TangentAlignment::OppositeDirection,
                Some(Ordering::Equal) => TangentAlignment::Degenerate,
                None => TangentAlignment::Unknown,
            }
        }
        None => TangentAlignment::Unknown,
    }
}

/// Classify a path join from exact endpoint and tangent data.
///
/// The endpoint decision uses `hyperlimit` point equality, then tangent
/// continuity uses [`classify_tangent_alignment`]. This is intentionally a
/// small predicate report, not a repair operation: tangent fillet construction
/// or `hypersolve` may propose adjusted geometry, but under Yap's exact
/// geometric computation model this function only certifies the retained join.
pub fn classify_tangent_join(
    first_endpoint: &Point2,
    first_tangent: &Point2,
    second_endpoint: &Point2,
    second_tangent: &Point2,
    policy: PredicatePolicy,
) -> TangentJoinReport {
    match point2_equal_with_policy(first_endpoint, second_endpoint, policy).value() {
        Some(false) => TangentJoinReport {
            class: TangentJoinClass::EndpointMismatch,
            endpoints_equal: Some(false),
            alignment: None,
        },
        Some(true) => {
            let alignment = classify_tangent_alignment(first_tangent, second_tangent, policy);
            let class = match alignment {
                TangentAlignment::SameDirection => TangentJoinClass::G1Continuous,
                TangentAlignment::OppositeDirection => TangentJoinClass::ReversedTangent,
                TangentAlignment::NotParallel => TangentJoinClass::Corner,
                TangentAlignment::Degenerate => TangentJoinClass::DegenerateTangent,
                TangentAlignment::Unknown => TangentJoinClass::Unknown,
            };
            TangentJoinReport {
                class,
                endpoints_equal: Some(true),
                alignment: Some(alignment),
            }
        }
        None => TangentJoinReport {
            class: TangentJoinClass::Unknown,
            endpoints_equal: None,
            alignment: None,
        },
    }
}

/// Classify every adjacent tangent join in a retained path chain.
///
/// This is the chain-level counterpart to [`classify_tangent_join`]. It does
/// not infer tangents from sampled geometry; callers pass the exact span
/// endpoint/tangent objects produced by lines, arcs, Beziers, or conics. This
/// mirrors CGAL-style arrangement validation and Yap's exact object/predicate
/// split: a path generator may propose a full chain, but each local join is
/// certified before the chain is accepted for smoothing, toolpathing, or
/// routing.
pub fn classify_tangent_chain(
    spans: &[TangentSpan],
    policy: PredicatePolicy,
) -> TangentChainReport {
    let joins = spans
        .windows(2)
        .map(|pair| {
            classify_tangent_join(
                &pair[0].end,
                &pair[0].end_tangent,
                &pair[1].start,
                &pair[1].start_tangent,
                policy,
            )
        })
        .collect();
    TangentChainReport { joins }
}

/// Certify every adjacent G1 join in a retained path chain through `hypersolve`.
///
/// Each adjacent pair lowers to [`build_g1_join_problem`] and is replayed by
/// [`certify_g1_join_candidate`]. This is intentionally redundant with the
/// direct predicate classifier: it gives solver-generated smooth chains the
/// same exact residual replay boundary as individual tangent and length
/// candidates before downstream CAM or routing accepts the chain.
pub fn certify_g1_chain(spans: &[TangentSpan]) -> G1ChainCertificationReport {
    let joins = spans
        .windows(2)
        .map(|pair| {
            let model = build_g1_join_problem(
                pair[0].end.clone(),
                pair[0].end_tangent.clone(),
                pair[1].start.clone(),
                pair[1].start_tangent.clone(),
            );
            certify_g1_join_candidate(&model)
        })
        .collect();
    G1ChainCertificationReport { joins }
}

/// Build a two-variable exact tangent-alignment residual.
///
/// The residual is `candidate_x * target_y - candidate_y * target_x = 0`.
/// A nonlinear solver may move the candidate tangent components, but accepted
/// results must be replayed by [`certify_tangent_alignment_candidate`]. This is
/// the tangent analogue of the length replay boundary used in routing and
/// follows Yap's rule that numerical proposals are not trusted until exact
/// predicates certify them.
pub fn build_tangent_alignment_problem(
    candidate: Point2,
    target: Point2,
) -> TangentAlignmentProblem {
    build_tangent_alignment_problem_with_orientation(candidate, target, false)
}

/// Build a two-variable oriented tangent-alignment model.
///
/// This includes the cross-product equality from
/// [`build_tangent_alignment_problem`] and adds `candidate · target >= 0`.
/// The inequality rejects opposite-direction tangents during exact replay while
/// still avoiding vector normalization. That makes it the solver-side
/// counterpart to [`TangentAlignment::SameDirection`].
pub fn build_oriented_tangent_alignment_problem(
    candidate: Point2,
    target: Point2,
) -> TangentAlignmentProblem {
    build_tangent_alignment_problem_with_orientation(candidate, target, true)
}

/// Build an exact endpoint-plus-oriented-tangent G1 join model.
///
/// The model contains endpoint equality residuals
/// `candidate_endpoint - target_endpoint = 0`, the tangent cross-product
/// equality, and the same-direction dot-product inequality. It is intentionally
/// a certification target for solver proposals: a fillet or smoothing solver
/// may move endpoint and tangent variables, but this model replays the exact
/// retained candidate before the geometry is accepted. That is the path-level
/// version of Yap's proposed-object/certified-decision boundary.
pub fn build_g1_join_problem(
    candidate_endpoint: Point2,
    candidate_tangent: Point2,
    target_endpoint: Point2,
    target_tangent: Point2,
) -> G1JoinProblem {
    let mut problem = Problem::default();
    let endpoint_x = problem.add_variable("candidate_endpoint_x", candidate_endpoint.x);
    let endpoint_y = problem.add_variable("candidate_endpoint_y", candidate_endpoint.y);
    let tangent_x = problem.add_variable("candidate_tangent_x", candidate_tangent.x);
    let tangent_y = problem.add_variable("candidate_tangent_y", candidate_tangent.y);

    let endpoint_x_symbol = SymbolId(endpoint_x.0);
    let endpoint_y_symbol = SymbolId(endpoint_y.0);
    let tangent_x_symbol = SymbolId(tangent_x.0);
    let tangent_y_symbol = SymbolId(tangent_y.0);
    let endpoint_x_expr = Expr::symbol(endpoint_x_symbol, "candidate_endpoint_x");
    let endpoint_y_expr = Expr::symbol(endpoint_y_symbol, "candidate_endpoint_y");
    let tangent_x_expr = Expr::symbol(tangent_x_symbol, "candidate_tangent_x");
    let tangent_y_expr = Expr::symbol(tangent_y_symbol, "candidate_tangent_y");

    problem.add_constraint(Constraint::equality(
        "join endpoint x",
        endpoint_x_expr - Expr::real(target_endpoint.x.clone()),
    ));
    problem.add_constraint(Constraint::equality(
        "join endpoint y",
        endpoint_y_expr - Expr::real(target_endpoint.y.clone()),
    ));
    problem.add_constraint(Constraint::equality(
        "join tangent cross product",
        tangent_x_expr.clone() * Expr::real(target_tangent.y.clone())
            - tangent_y_expr.clone() * Expr::real(target_tangent.x.clone()),
    ));
    problem.add_constraint(Constraint {
        name: "join tangent same direction dot product".to_string(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: tangent_x_expr * Expr::real(target_tangent.x.clone())
            + tangent_y_expr * Expr::real(target_tangent.y.clone()),
        weight: Real::one(),
        active: true,
    });

    G1JoinProblem {
        problem,
        endpoint_x_symbol,
        endpoint_y_symbol,
        tangent_x_symbol,
        tangent_y_symbol,
        target_endpoint,
        target_tangent,
    }
}

fn build_tangent_alignment_problem_with_orientation(
    candidate: Point2,
    target: Point2,
    require_same_direction: bool,
) -> TangentAlignmentProblem {
    let mut problem = Problem::default();
    let candidate_x = problem.add_variable("candidate_tangent_x", candidate.x);
    let candidate_y = problem.add_variable("candidate_tangent_y", candidate.y);
    let candidate_x_symbol = SymbolId(candidate_x.0);
    let candidate_y_symbol = SymbolId(candidate_y.0);
    let candidate_x_expr = Expr::symbol(candidate_x_symbol, "candidate_tangent_x");
    let candidate_y_expr = Expr::symbol(candidate_y_symbol, "candidate_tangent_y");
    let residual = candidate_x_expr.clone() * Expr::real(target.y.clone())
        - candidate_y_expr.clone() * Expr::real(target.x.clone());
    problem.add_constraint(Constraint::equality("tangent cross product", residual));
    if require_same_direction {
        problem.add_constraint(Constraint {
            name: "tangent same direction dot product".to_string(),
            kind: ConstraintKind::GreaterOrEqual,
            residual: candidate_x_expr * Expr::real(target.x.clone())
                + candidate_y_expr * Expr::real(target.y.clone()),
            weight: Real::one(),
            active: true,
        });
    }
    TangentAlignmentProblem {
        problem,
        candidate_x_symbol,
        candidate_y_symbol,
        target,
    }
}

/// Certify the current candidate tangent by exact residual replay.
pub fn certify_tangent_alignment_candidate(
    model: &TangentAlignmentProblem,
) -> CandidateCertificationReport {
    let prepared = PreparedProblem::new(&model.problem);
    let context = context_from_problem(&model.problem);
    certify_candidate(&prepared, &context)
}

/// Certify the current G1 join candidate by exact residual replay.
pub fn certify_g1_join_candidate(model: &G1JoinProblem) -> CandidateCertificationReport {
    let prepared = PreparedProblem::new(&model.problem);
    let context = context_from_problem(&model.problem);
    certify_candidate(&prepared, &context)
}

/// Return the exact 2D cross product of two tangent vectors.
pub fn tangent_cross(first: &Point2, second: &Point2) -> Real {
    first.x.clone() * second.y.clone() - first.y.clone() * second.x.clone()
}

/// Return the exact dot product of two tangent vectors.
pub fn tangent_dot(first: &Point2, second: &Point2) -> Real {
    first.x.clone() * second.x.clone() + first.y.clone() * second.y.clone()
}

/// Return the exact squared norm of a tangent vector.
pub fn tangent_norm_squared(vector: &Point2) -> Real {
    Real::signed_product_sum(
        [true, true],
        [[&vector.x, &vector.x], [&vector.y, &vector.y]],
    )
}

fn is_zero_vector(vector: &Point2, policy: PredicatePolicy) -> Option<bool> {
    match compare_reals_with_policy(&tangent_norm_squared(vector), &Real::zero(), policy).value()? {
        Ordering::Equal => Some(true),
        Ordering::Greater => Some(false),
        Ordering::Less => None,
    }
}

fn parameter_zero() -> BezierParameter {
    BezierParameter::new(0, 1).expect("zero is a valid closed Bezier parameter")
}

fn parameter_one() -> BezierParameter {
    BezierParameter::new(1, 1).expect("one is a valid closed Bezier parameter")
}
