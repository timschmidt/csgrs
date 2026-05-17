//! Orientation predicates.

use crate::RealSymbolicDependencyMask;
use crate::classify::LineSide;
pub use crate::geometry::{Point2, Point3};
use crate::predicate::{
    Certainty, DeterminantScheduleHint, Escalation, ExactPredicateKernel, PredicateCertificate,
    PredicateOutcome, PredicatePolicy, PredicateReport, RefinementNeed, Sign,
};
use crate::real::{add_ref, mul_ref, sub_ref};
use crate::resolve::{map_outcome, resolve_real_sign, signed_term_filter};
use hyperreal::{Real, RealExactSetFacts, ZeroKnowledge};

pub use crate::batch::{
    Incircle2dCase, Insphere3dCase, Orient2dCase, Orient3dCase, classify_point_line_batch,
    classify_point_line_batch_with_policy, incircle2d_batch, incircle2d_batch_with_policy,
    insphere3d_batch, insphere3d_batch_with_policy, orient2d_batch, orient2d_batch_with_policy,
    orient3d_batch, orient3d_batch_with_policy,
};
#[cfg(feature = "parallel")]
pub use crate::batch::{
    classify_point_line_batch_parallel, classify_point_line_batch_parallel_with_policy,
    incircle2d_batch_parallel, incircle2d_batch_parallel_with_policy, insphere3d_batch_parallel,
    insphere3d_batch_parallel_with_policy, orient2d_batch_parallel,
    orient2d_batch_parallel_with_policy, orient3d_batch_parallel,
    orient3d_batch_parallel_with_policy,
};

/// Orientation of three 2D points.
pub fn orient2d(a: &Point2, b: &Point2, c: &Point2) -> PredicateOutcome<Sign> {
    orient2d_with_policy(a, b, c, PredicatePolicy::default())
}

/// Orientation of three 2D points with an explicit escalation policy.
pub fn orient2d_with_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    orient2d_report_with_policy(a, b, c, policy).outcome
}

/// Orientation of three 2D points with a provenance certificate.
pub fn orient2d_report(a: &Point2, b: &Point2, c: &Point2) -> PredicateReport<Sign> {
    orient2d_report_with_policy(a, b, c, PredicatePolicy::default())
}

/// Orientation of three 2D points with an explicit escalation policy and
/// provenance certificate.
pub fn orient2d_report_with_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    policy: PredicatePolicy,
) -> PredicateReport<Sign> {
    // Structural-dispatch note: when callers carry integer-grid scale,
    // affine-transform conditioning, or dyadic denominator facts, this
    // predicate can choose a faster exact determinant expansion before building
    // the generic Real expression tree.
    if let Some(report) = exact_report(policy, ExactPredicateKernel::Orient2dRationalDet2, || {
        super::exact::orient2d_shared_scale(a, b, c).or_else(|| super::exact::orient2d(a, b, c))
    }) {
        return report;
    }
    PredicateReport::from_outcome(orient2d_real_expr(a, b, c, policy))
}

fn orient2d_real_expr(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    crate::trace_dispatch!("hyperlimit", "orient2d", "real-determinant");
    let abx = sub(&b.x, &a.x);
    let aby = sub(&b.y, &a.y);
    let acx = sub(&c.x, &a.x);
    let acy = sub(&c.y, &a.y);
    let left = mul(&abx, &acy);
    let right = mul(&aby, &acx);
    let det = sub(&left, &right);

    resolve_real_sign(
        &det,
        policy,
        || {
            let _ = (&abx, &aby, &acx, &acy);
            signed_term_filter(&[(&left, Sign::Positive), (&right, Sign::Negative)])
        },
        || super::exact::orient2d_shared_scale(a, b, c).or_else(|| super::exact::orient2d(a, b, c)),
        RefinementNeed::RealRefinement,
    )
}

/// Orientation of four 3D points. Positive means `d` is on the positive side
/// of the oriented plane through `a`, `b`, and `c`.
pub fn orient3d(a: &Point3, b: &Point3, c: &Point3, d: &Point3) -> PredicateOutcome<Sign> {
    orient3d_with_policy(a, b, c, d, PredicatePolicy::default())
}

/// Orientation of four 3D points with an explicit escalation policy.
pub fn orient3d_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    orient3d_report_with_policy(a, b, c, d, policy).outcome
}

/// Orientation of four 3D points with a provenance certificate.
pub fn orient3d_report(a: &Point3, b: &Point3, c: &Point3, d: &Point3) -> PredicateReport<Sign> {
    orient3d_report_with_policy(a, b, c, d, PredicatePolicy::default())
}

/// Orientation of four 3D points with an explicit escalation policy and
/// provenance certificate.
pub fn orient3d_report_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    policy: PredicatePolicy,
) -> PredicateReport<Sign> {
    if let Some(report) = exact_report(policy, ExactPredicateKernel::Orient3dRationalDet3, || {
        super::exact::orient3d_shared_scale(a, b, c, d)
            .or_else(|| super::exact::orient3d(a, b, c, d))
    }) {
        return report;
    }

    crate::trace_dispatch!("hyperlimit", "orient3d", "real-determinant");
    let adx = sub(&a.x, &d.x);
    let ady = sub(&a.y, &d.y);
    let adz = sub(&a.z, &d.z);
    let bdx = sub(&b.x, &d.x);
    let bdy = sub(&b.y, &d.y);
    let bdz = sub(&b.z, &d.z);
    let cdx = sub(&c.x, &d.x);
    let cdy = sub(&c.y, &d.y);
    let cdz = sub(&c.z, &d.z);

    // Keep the translated determinant as a six-term product-sum until the
    // `Real` layer has a chance to route exact rationals through one
    // shared-denominator reducer. This mirrors the exact-predicate schedule of
    // Shewchuk, "Adaptive Precision Floating-Point Arithmetic and Fast Robust
    // Geometric Predicates," *Discrete & Computational Geometry* 18.3 (1997),
    // but remains exact-real and policy-visible in Yap's EGC sense.
    let det = Real::signed_product_sum(
        [true, false, true, false, true, false],
        [
            [&adx, &bdy, &cdz],
            [&adx, &bdz, &cdy],
            [&ady, &bdz, &cdx],
            [&ady, &bdx, &cdz],
            [&adz, &bdx, &cdy],
            [&adz, &bdy, &cdx],
        ],
    );

    PredicateReport::from_outcome(resolve_real_sign(
        &det,
        policy,
        || None,
        || {
            super::exact::orient3d_shared_scale(a, b, c, d)
                .or_else(|| super::exact::orient3d(a, b, c, d))
        },
        RefinementNeed::RealRefinement,
    ))
}

/// Classify `point` relative to the oriented line from `from` to `to`.
pub fn classify_point_line(
    from: &Point2,
    to: &Point2,
    point: &Point2,
) -> PredicateOutcome<LineSide> {
    classify_point_line_with_policy(from, to, point, PredicatePolicy::default())
}

/// Classify `point` relative to the oriented line from `from` to `to` with an
/// explicit escalation policy.
pub fn classify_point_line_with_policy(
    from: &Point2,
    to: &Point2,
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<LineSide> {
    map_outcome(
        orient2d_with_policy(from, to, point, policy),
        LineSide::from,
    )
}

/// Cheap facts cached by prepared orientation and lifted-circle handles.
///
/// These facts are intentionally about the fixed part of a prepared predicate,
/// not about the query point. They reserve the object-level dispatch boundary
/// requested by Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997): repeated predicates can select exact rational,
/// dyadic, or future shared-scale schedules before building scalar expression
/// trees for every query.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PreparedPredicateFacts {
    /// Every fixed coordinate is represented as an exact rational `Real`.
    pub fixed_coordinates_exact_rational: bool,
    /// Every fixed coordinate is represented as an exact dyadic rational.
    pub fixed_coordinates_dyadic: bool,
    /// All fixed exact-rational coordinates have the same reduced denominator.
    ///
    /// This is a borrowed common-scale fact in the sense of Yap, "Towards
    /// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997):
    /// the prepared object records that a shared-denominator schedule is
    /// eligible without owning a new coordinate representation yet.
    pub fixed_coordinates_shared_denominator: bool,
    /// Bit mask of fixed points whose own coordinates share one reduced
    /// denominator.
    ///
    /// This is deliberately weaker than
    /// [`PreparedPredicateFacts::fixed_coordinates_shared_denominator`]: a
    /// prepared predicate may have point-local homogeneous/common-scale
    /// structure even when different fixed points use different grids. Carrying
    /// that object-local fact preserves the information needed for future
    /// homogeneous determinant schedules without exposing rational storage.
    /// This is the object-structure preservation principle from Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997).
    pub fixed_point_shared_scale_mask: u128,
    /// Bit mask of fixed points structurally known to be the coordinate origin.
    ///
    /// This point-level sparse fact is cached on the prepared predicate rather
    /// than rediscovered in each query. It follows Yap's object-structure-first
    /// exact computation model: prepare reusable geometric objects with cheap
    /// facts, then select arithmetic packages from those facts.
    pub fixed_point_origin_mask: u128,
    /// Bit mask of fixed points structurally known to have exactly one nonzero
    /// coordinate and all remaining coordinates zero.
    ///
    /// These points are not necessarily signed unit axes; the mask only records
    /// one-hot coordinate support. It is a scheduling hint for future sparse
    /// determinant kernels, not an incidence or orientation decision.
    pub fixed_point_one_hot_mask: u128,
    /// Bit mask of fixed points with at least one coordinate whose zero status
    /// is structurally unknown.
    ///
    /// Keeping unknown-zero provenance lets prepared predicates avoid selecting
    /// sparse exact kernels from incomplete facts while still carrying the
    /// uncertainty explicitly.
    pub fixed_point_unknown_zero_mask: u128,
    /// Union of scalar symbolic dependency families across all fixed points.
    ///
    /// This is a prepared-object scheduling fact, not an exact predicate
    /// certificate. It lets repeated line, circle, plane, and sphere queries
    /// retain the same symbolic-family summary as their fixed point objects
    /// without exposing `Real` internals. The boundary follows Yap's
    /// object-package guidance: carry reusable expression structure to the
    /// arithmetic package selection point, then certify signs separately; see
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997).
    pub fixed_symbolic_dependencies: RealSymbolicDependencyMask,
    /// Exact kernel that can be attempted when the query coordinates match.
    pub exact_kernel_hint: Option<ExactPredicateKernel>,
}

impl PreparedPredicateFacts {
    /// Counts fixed points whose own coordinates share one reduced denominator.
    pub fn fixed_point_shared_scale_count(self) -> u32 {
        self.fixed_point_shared_scale_mask.count_ones()
    }

    /// Counts fixed points structurally known to be the coordinate origin.
    pub fn fixed_point_origin_count(self) -> u32 {
        self.fixed_point_origin_mask.count_ones()
    }

    /// Counts fixed points structurally known to have exactly one nonzero
    /// coordinate.
    pub fn fixed_point_one_hot_count(self) -> u32 {
        self.fixed_point_one_hot_mask.count_ones()
    }

    /// Counts fixed points with at least one coordinate whose zero status is
    /// structurally unknown.
    pub fn fixed_point_unknown_zero_count(self) -> u32 {
        self.fixed_point_unknown_zero_mask.count_ones()
    }

    /// Returns whether any fixed point carries coordinate zero uncertainty.
    ///
    /// This keeps future sparse determinant dispatch honest: unknown-zero facts
    /// must block sparse schedules that require certified support. Carrying the
    /// uncertainty at the prepared-object layer follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    pub fn has_fixed_point_unknown_zero(self) -> bool {
        self.fixed_point_unknown_zero_mask != 0
    }

    /// Returns fixed points eligible for sparse-coordinate schedules.
    ///
    /// Origins and one-hot points are both sparse support patterns. This helper
    /// only exposes candidate arithmetic structure; predicate signs and
    /// incidence still come from exact predicate evaluation.
    pub fn fixed_point_sparse_support_mask(self) -> u128 {
        self.fixed_point_origin_mask | self.fixed_point_one_hot_mask
    }

    /// Select an advisory determinant schedule from retained object facts.
    ///
    /// The returned value is deliberately a hint. It is useful for choosing
    /// prepared arithmetic packages, trace labels, and higher-level cache
    /// payoff estimates, but it is not a predicate certificate. Exact predicate
    /// reports still certify topology. This preserves Yap's exact-geometric-
    /// computation split: prepare geometric objects with reusable structure,
    /// then let certified arithmetic decide signs; see Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    pub fn determinant_schedule_hint(self) -> DeterminantScheduleHint {
        let Some(kernel) = self.exact_kernel_hint else {
            return DeterminantScheduleHint::GenericRealFallback;
        };

        let sparse_points = self.fixed_point_sparse_support_mask().count_ones();
        if sparse_points > 0 && !self.has_fixed_point_unknown_zero() {
            return DeterminantScheduleHint::SparseSupportCandidate {
                kernel,
                fixed_sparse_points: sparse_points,
            };
        }
        if self.fixed_coordinates_shared_denominator {
            return DeterminantScheduleHint::SharedDenominatorCandidate { kernel };
        }
        if self.fixed_coordinates_dyadic {
            return DeterminantScheduleHint::DyadicCandidate { kernel };
        }
        DeterminantScheduleHint::ExactRationalKernel { kernel }
    }

    fn line2(from: &Point2, to: &Point2) -> Self {
        fixed_point_facts_2([from, to], ExactPredicateKernel::Orient2dRationalDet2)
    }

    fn incircle2(a: &Point2, b: &Point2, c: &Point2) -> Self {
        fixed_point_facts_2(
            [a, b, c],
            ExactPredicateKernel::Incircle2dRationalLiftedDet3,
        )
    }

    fn insphere3(a: &Point3, b: &Point3, c: &Point3, d: &Point3) -> Self {
        fixed_point_facts_3(
            [a, b, c, d],
            ExactPredicateKernel::Insphere3dRationalLiftedDet4,
        )
    }
}

/// Structural facts for a prepared lifted-circle or lifted-sphere polynomial.
///
/// A prepared in-circle or in-sphere query evaluates a fixed polynomial in the
/// query point's coordinates. This fact package summarizes those fixed
/// coefficients so downstream caches can retain exact-set, dyadic,
/// shared-scale, and sparse-support opportunities without exposing internal
/// coefficient storage. The split follows Yap's exact-geometric-computation
/// model: preserve geometric object structure first, then choose certified
/// arithmetic near the predicate call. See Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PreparedLiftedPolynomialFacts {
    /// Exact-rational representation facts for the fixed polynomial coefficients.
    pub coefficient_exact: RealExactSetFacts,
    /// Bit mask of coefficients known to be exactly zero.
    pub coefficient_zero_mask: u128,
    /// Bit mask of coefficients known to be nonzero.
    pub coefficient_nonzero_mask: u128,
    /// Bit mask of coefficients whose zero status is unknown.
    pub coefficient_unknown_zero_mask: u128,
}

impl PreparedLiftedPolynomialFacts {
    /// Counts coefficients known to be exactly zero.
    pub fn coefficient_zero_count(self) -> u32 {
        self.coefficient_zero_mask.count_ones()
    }

    /// Counts coefficients known to be nonzero.
    pub fn coefficient_nonzero_count(self) -> u32 {
        self.coefficient_nonzero_mask.count_ones()
    }

    /// Counts coefficients with unknown zero status.
    pub fn coefficient_unknown_zero_count(self) -> u32 {
        self.coefficient_unknown_zero_mask.count_ones()
    }

    /// Returns whether all coefficients share one exact denominator.
    pub fn has_shared_denominator_schedule(self) -> bool {
        self.coefficient_exact.has_shared_denominator_schedule()
    }

    /// Returns whether all coefficients are exact dyadics.
    pub fn has_dyadic_schedule(self) -> bool {
        self.coefficient_exact.has_dyadic_schedule()
    }

    /// Returns whether sparse polynomial evaluation may be profitable.
    ///
    /// This is only a schedule hint: unknown-zero coefficients prevent a
    /// certified sparse path, and predicate signs still come from exact
    /// evaluation. Sparse schedule selection belongs here rather than in
    /// topology crates for the same retained-structure reason discussed by
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997).
    pub fn has_sparse_coefficient_support(self) -> bool {
        self.coefficient_zero_count() > 0 && self.coefficient_unknown_zero_mask == 0
    }
}

/// Borrowed coefficient view for a prepared lifted 2D circle polynomial.
///
/// Query evaluation uses `x_coeff*x + y_coeff*y + lift_coeff*(x^2+y^2) +
/// constant`, where the sign is interpreted by the in-circle convention.
#[derive(Clone, Copy, Debug)]
pub struct PreparedCircle2Polynomial<'a> {
    /// Coefficient multiplied by query `x`.
    pub x_coeff: &'a Real,
    /// Coefficient multiplied by query `y`.
    pub y_coeff: &'a Real,
    /// Coefficient multiplied by query `x^2 + y^2`.
    pub lift_coeff: &'a Real,
    /// Constant coefficient.
    pub constant: &'a Real,
}

/// Borrowed coefficient view for a prepared lifted 3D sphere polynomial.
///
/// Query evaluation uses `x_coeff*x + y_coeff*y + z_coeff*z +
/// lift_coeff*(x^2+y^2+z^2) + constant`, with sign interpreted by the
/// in-sphere convention.
#[derive(Clone, Copy, Debug)]
pub struct PreparedSphere3Polynomial<'a> {
    /// Coefficient multiplied by query `x`.
    pub x_coeff: &'a Real,
    /// Coefficient multiplied by query `y`.
    pub y_coeff: &'a Real,
    /// Coefficient multiplied by query `z`.
    pub z_coeff: &'a Real,
    /// Coefficient multiplied by query `x^2 + y^2 + z^2`.
    pub lift_coeff: &'a Real,
    /// Constant coefficient.
    pub constant: &'a Real,
}

/// Reusable point-line classifier for a fixed oriented line segment.
#[derive(Clone, Copy, Debug)]
pub struct PreparedLine2<'a> {
    from: &'a Point2,
    to: &'a Point2,
    facts: PreparedPredicateFacts,
}

impl<'a> PreparedLine2<'a> {
    /// Prepare the oriented line from `from` to `to`.
    pub fn new(from: &'a Point2, to: &'a Point2) -> Self {
        crate::trace_dispatch!("hyperlimit", "prepared_line2", "new");
        Self {
            from,
            to,
            facts: PreparedPredicateFacts::line2(from, to),
        }
    }

    /// Prepare the oriented line from already-collected fixed-coordinate facts.
    ///
    /// Higher-level geometry objects often collect structural facts while
    /// preparing their own topology caches. Reusing those facts preserves
    /// Yap's object-level EGC boundary: the owning curve layer can remember
    /// common-scale or dyadic eligibility, while `hyperlimit` remains the
    /// predicate layer that decides sidedness. See Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    pub const fn from_facts(
        from: &'a Point2,
        to: &'a Point2,
        facts: PreparedPredicateFacts,
    ) -> Self {
        Self { from, to, facts }
    }

    /// Return cheap fixed-coordinate facts collected at preparation time.
    pub const fn facts(&self) -> PreparedPredicateFacts {
        self.facts
    }

    /// Classify a point using the default predicate policy.
    pub fn classify_point(&self, point: &Point2) -> PredicateOutcome<LineSide> {
        self.classify_point_with_policy(point, PredicatePolicy::default())
    }

    /// Classify a point using an explicit predicate policy.
    pub fn classify_point_with_policy(
        &self,
        point: &Point2,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<LineSide> {
        if let Some(report) =
            exact_report(policy, ExactPredicateKernel::Orient2dRationalDet2, || {
                super::exact::orient2d_shared_scale(self.from, self.to, point)
                    .or_else(|| super::exact::orient2d(self.from, self.to, point))
            })
        {
            return map_outcome(report.outcome, LineSide::from);
        }
        map_outcome(
            orient2d_real_expr(self.from, self.to, point, policy),
            LineSide::from,
        )
    }
}

/// In-circle predicate for four 2D points.
///
/// Positive means `d` lies inside the oriented circumcircle through `a`, `b`,
/// and `c` when those three points are counter-clockwise. Reversing the
/// orientation of `a`, `b`, and `c` reverses the sign.
pub fn incircle2d(a: &Point2, b: &Point2, c: &Point2, d: &Point2) -> PredicateOutcome<Sign> {
    incircle2d_with_policy(a, b, c, d, PredicatePolicy::default())
}

/// In-circle predicate for four 2D points with an explicit escalation policy.
pub fn incircle2d_with_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    incircle2d_report_with_policy(a, b, c, d, policy).outcome
}

/// In-circle predicate with a provenance certificate.
pub fn incircle2d_report(a: &Point2, b: &Point2, c: &Point2, d: &Point2) -> PredicateReport<Sign> {
    incircle2d_report_with_policy(a, b, c, d, PredicatePolicy::default())
}

/// In-circle predicate with an explicit escalation policy and certificate.
pub fn incircle2d_report_with_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    policy: PredicatePolicy,
) -> PredicateReport<Sign> {
    if let Some(report) = exact_report(
        policy,
        ExactPredicateKernel::Incircle2dRationalLiftedDet3,
        || {
            super::exact::incircle2d_shared_scale(a, b, c, d)
                .or_else(|| super::exact::incircle2d(a, b, c, d))
        },
    ) {
        return report;
    }
    PredicateReport::from_outcome(incircle2d_real_expr(a, b, c, d, policy))
}

fn incircle2d_real_expr(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    crate::trace_dispatch!("hyperlimit", "incircle2d", "real-determinant");
    let adx = sub(&a.x, &d.x);
    let ady = sub(&a.y, &d.y);
    let bdx = sub(&b.x, &d.x);
    let bdy = sub(&b.y, &d.y);
    let cdx = sub(&c.x, &d.x);
    let cdy = sub(&c.y, &d.y);

    let adx2 = mul(&adx, &adx);
    let ady2 = mul(&ady, &ady);
    let bdx2 = mul(&bdx, &bdx);
    let bdy2 = mul(&bdy, &bdy);
    let cdx2 = mul(&cdx, &cdx);
    let cdy2 = mul(&cdy, &cdy);
    let alift = add(&adx2, &ady2);
    let blift = add(&bdx2, &bdy2);
    let clift = add(&cdx2, &cdy2);

    // The lifted determinant is a six-term polynomial. Passing it as a whole
    // product-sum preserves Yap's object-shape boundary for symbolic fallback
    // and gives exact rational inputs one fraction-delayed reducer.
    let det = Real::signed_product_sum(
        [true, false, true, false, true, false],
        [
            [&alift, &bdx, &cdy],
            [&alift, &cdx, &bdy],
            [&blift, &cdx, &ady],
            [&blift, &adx, &cdy],
            [&clift, &adx, &bdy],
            [&clift, &bdx, &ady],
        ],
    );

    resolve_real_sign(
        &det,
        policy,
        || None,
        || {
            super::exact::incircle2d_shared_scale(a, b, c, d)
                .or_else(|| super::exact::incircle2d(a, b, c, d))
        },
        RefinementNeed::RealRefinement,
    )
}

/// Reusable in-circle predicate for a fixed oriented circle through three 2D points.
#[derive(Clone, Debug)]
pub struct PreparedIncircle2<'a> {
    a: &'a Point2,
    b: &'a Point2,
    c: &'a Point2,
    facts: PreparedPredicateFacts,
    coefficient_facts: PreparedLiftedPolynomialFacts,
    x_coeff: Real,
    y_coeff: Real,
    lift_coeff: Real,
    constant: Real,
}

impl<'a> PreparedIncircle2<'a> {
    /// Prepare the oriented circumcircle through `a`, `b`, and `c`.
    pub fn new(a: &'a Point2, b: &'a Point2, c: &'a Point2) -> Self {
        crate::trace_dispatch!("hyperlimit", "prepared_incircle2", "new");
        let a_lift = point2_lift(a);
        let b_lift = point2_lift(b);
        let c_lift = point2_lift(c);

        let y_lift_one = det3_with_unit_col2(&a.y, &a_lift, &b.y, &b_lift, &c.y, &c_lift);
        let x_lift_one = det3_with_unit_col2(&a.x, &a_lift, &b.x, &b_lift, &c.x, &c_lift);
        let x_y_one = det3_with_unit_col2(&a.x, &a.y, &b.x, &b.y, &c.x, &c.y);
        let x_y_lift = det3_refs(
            [&a.x, &a.y, &a_lift],
            [&b.x, &b.y, &b_lift],
            [&c.x, &c.y, &c_lift],
        );
        let x_coeff = neg(&y_lift_one);
        let y_coeff = x_lift_one;
        let lift_coeff = neg(&x_y_one);
        let constant = x_y_lift;
        let coefficient_facts =
            lifted_polynomial_facts([&x_coeff, &y_coeff, &lift_coeff, &constant]);

        Self {
            a,
            b,
            c,
            facts: PreparedPredicateFacts::incircle2(a, b, c),
            coefficient_facts,
            x_coeff,
            y_coeff,
            lift_coeff,
            constant,
        }
    }

    /// Test a point using the default predicate policy.
    pub fn test_point(&self, point: &Point2) -> PredicateOutcome<Sign> {
        self.test_point_with_policy(point, PredicatePolicy::default())
    }

    /// Return cheap fixed-coordinate facts collected at preparation time.
    pub const fn facts(&self) -> PreparedPredicateFacts {
        self.facts
    }

    /// Return structural facts for the cached lifted-circle polynomial.
    pub const fn coefficient_facts(&self) -> PreparedLiftedPolynomialFacts {
        self.coefficient_facts
    }

    /// Return borrowed cached coefficients for the lifted-circle polynomial.
    pub const fn polynomial(&self) -> PreparedCircle2Polynomial<'_> {
        PreparedCircle2Polynomial {
            x_coeff: &self.x_coeff,
            y_coeff: &self.y_coeff,
            lift_coeff: &self.lift_coeff,
            constant: &self.constant,
        }
    }

    /// Test a point using an explicit predicate policy.
    pub fn test_point_with_policy(
        &self,
        point: &Point2,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<Sign> {
        if let Some(report) = exact_report(
            policy,
            ExactPredicateKernel::Incircle2dRationalLiftedDet3,
            || {
                super::exact::incircle2d_shared_scale(self.a, self.b, self.c, point)
                    .or_else(|| super::exact::incircle2d(self.a, self.b, self.c, point))
            },
        ) {
            return report.outcome;
        }

        crate::trace_dispatch!("hyperlimit", "prepared_incircle2", "circle-polynomial");
        let x_term = mul(&self.x_coeff, &point.x);
        let y_term = mul(&self.y_coeff, &point.y);
        let lift = point2_lift(point);
        let lift_term = mul(&self.lift_coeff, &lift);
        let xy = add(&x_term, &y_term);
        let xyl = add(&xy, &lift_term);
        let det = add(&xyl, &self.constant);

        resolve_real_sign(
            &det,
            policy,
            || None,
            || {
                super::exact::incircle2d_shared_scale(self.a, self.b, self.c, point)
                    .or_else(|| super::exact::incircle2d(self.a, self.b, self.c, point))
            },
            RefinementNeed::RealRefinement,
        )
    }
}

/// In-sphere predicate for five 3D points.
///
/// Positive means `e` lies inside the oriented circumsphere through `a`, `b`,
/// `c`, and `d` when the tetrahedron orientation matches the exact kernel's
/// convention. Reversing that orientation reverses the sign.
pub fn insphere3d(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    e: &Point3,
) -> PredicateOutcome<Sign> {
    insphere3d_with_policy(a, b, c, d, e, PredicatePolicy::default())
}

/// In-sphere predicate for five 3D points with an explicit escalation policy.
pub fn insphere3d_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    e: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    insphere3d_report_with_policy(a, b, c, d, e, policy).outcome
}

/// In-sphere predicate with a provenance certificate.
pub fn insphere3d_report(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    e: &Point3,
) -> PredicateReport<Sign> {
    insphere3d_report_with_policy(a, b, c, d, e, PredicatePolicy::default())
}

/// In-sphere predicate with an explicit escalation policy and certificate.
pub fn insphere3d_report_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    e: &Point3,
    policy: PredicatePolicy,
) -> PredicateReport<Sign> {
    if let Some(report) = exact_report(
        policy,
        ExactPredicateKernel::Insphere3dRationalLiftedDet4,
        || {
            super::exact::insphere3d_shared_scale(a, b, c, d, e)
                .or_else(|| super::exact::insphere3d(a, b, c, d, e))
        },
    ) {
        return report;
    }
    PredicateReport::from_outcome(insphere3d_real_expr(a, b, c, d, e, policy))
}

fn insphere3d_real_expr(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    e: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    crate::trace_dispatch!("hyperlimit", "insphere3d", "real-determinant");
    let aex = sub(&a.x, &e.x);
    let bex = sub(&b.x, &e.x);
    let cex = sub(&c.x, &e.x);
    let dex = sub(&d.x, &e.x);
    let aey = sub(&a.y, &e.y);
    let bey = sub(&b.y, &e.y);
    let cey = sub(&c.y, &e.y);
    let dey = sub(&d.y, &e.y);
    let aez = sub(&a.z, &e.z);
    let bez = sub(&b.z, &e.z);
    let cez = sub(&c.z, &e.z);
    let dez = sub(&d.z, &e.z);

    let aex_bey = mul(&aex, &bey);
    let bex_aey = mul(&bex, &aey);
    let ab = sub(&aex_bey, &bex_aey);

    let bex_cey = mul(&bex, &cey);
    let cex_bey = mul(&cex, &bey);
    let bc = sub(&bex_cey, &cex_bey);

    let cex_dey = mul(&cex, &dey);
    let dex_cey = mul(&dex, &cey);
    let cd = sub(&cex_dey, &dex_cey);

    let dex_aey = mul(&dex, &aey);
    let aex_dey = mul(&aex, &dey);
    let da = sub(&dex_aey, &aex_dey);

    let aex_cey = mul(&aex, &cey);
    let cex_aey = mul(&cex, &aey);
    let ac = sub(&aex_cey, &cex_aey);

    let bex_dey = mul(&bex, &dey);
    let dex_bey = mul(&dex, &bey);
    let bd = sub(&bex_dey, &dex_bey);

    let aez_bc = mul(&aez, &bc);
    let bez_ac = mul(&bez, &ac);
    let cez_ab = mul(&cez, &ab);
    let abc_minus = sub(&aez_bc, &bez_ac);
    let abc = add(&abc_minus, &cez_ab);

    let bez_cd = mul(&bez, &cd);
    let cez_bd = mul(&cez, &bd);
    let dez_bc = mul(&dez, &bc);
    let bcd_minus = sub(&bez_cd, &cez_bd);
    let bcd = add(&bcd_minus, &dez_bc);

    let cez_da = mul(&cez, &da);
    let dez_ac = mul(&dez, &ac);
    let aez_cd = mul(&aez, &cd);
    let cda_partial = add(&cez_da, &dez_ac);
    let cda = add(&cda_partial, &aez_cd);

    let dez_ab = mul(&dez, &ab);
    let aez_bd = mul(&aez, &bd);
    let bez_da = mul(&bez, &da);
    let dab_partial = add(&dez_ab, &aez_bd);
    let dab = add(&dab_partial, &bez_da);

    let aex2 = mul(&aex, &aex);
    let aey2 = mul(&aey, &aey);
    let aez2 = mul(&aez, &aez);
    let alift_xy = add(&aex2, &aey2);
    let alift = add(&alift_xy, &aez2);

    let bex2 = mul(&bex, &bex);
    let bey2 = mul(&bey, &bey);
    let bez2 = mul(&bez, &bez);
    let blift_xy = add(&bex2, &bey2);
    let blift = add(&blift_xy, &bez2);

    let cex2 = mul(&cex, &cex);
    let cey2 = mul(&cey, &cey);
    let cez2 = mul(&cez, &cez);
    let clift_xy = add(&cex2, &cey2);
    let clift = add(&clift_xy, &cez2);

    let dex2 = mul(&dex, &dex);
    let dey2 = mul(&dey, &dey);
    let dez2 = mul(&dez, &dez);
    let dlift_xy = add(&dex2, &dey2);
    let dlift = add(&dlift_xy, &dez2);

    let dlift_abc = mul(&dlift, &abc);
    let blift_cda = mul(&blift, &cda);
    let left = add(&dlift_abc, &blift_cda);

    let clift_dab = mul(&clift, &dab);
    let alift_bcd = mul(&alift, &bcd);
    let right = add(&clift_dab, &alift_bcd);
    let det = sub(&left, &right);

    resolve_real_sign(
        &det,
        policy,
        || signed_term_filter(&[(&left, Sign::Positive), (&right, Sign::Negative)]),
        || {
            super::exact::insphere3d_shared_scale(a, b, c, d, e)
                .or_else(|| super::exact::insphere3d(a, b, c, d, e))
        },
        RefinementNeed::RealRefinement,
    )
}

/// Reusable in-sphere predicate for a fixed oriented sphere through four 3D points.
#[derive(Clone, Debug)]
pub struct PreparedInsphere3<'a> {
    a: &'a Point3,
    b: &'a Point3,
    c: &'a Point3,
    d: &'a Point3,
    facts: PreparedPredicateFacts,
    coefficient_facts: PreparedLiftedPolynomialFacts,
    x_coeff: Real,
    y_coeff: Real,
    z_coeff: Real,
    lift_coeff: Real,
    constant: Real,
}

impl<'a> PreparedInsphere3<'a> {
    /// Prepare the oriented circumsphere through `a`, `b`, `c`, and `d`.
    pub fn new(a: &'a Point3, b: &'a Point3, c: &'a Point3, d: &'a Point3) -> Self {
        crate::trace_dispatch!("hyperlimit", "prepared_insphere3", "new");
        let a_lift = point3_lift(a);
        let b_lift = point3_lift(b);
        let c_lift = point3_lift(c);
        let d_lift = point3_lift(d);

        let y_z_lift_one = det4_with_unit_col3(
            [&a.y, &a.z, &a_lift],
            [&b.y, &b.z, &b_lift],
            [&c.y, &c.z, &c_lift],
            [&d.y, &d.z, &d_lift],
        );
        let x_z_lift_one = det4_with_unit_col3(
            [&a.x, &a.z, &a_lift],
            [&b.x, &b.z, &b_lift],
            [&c.x, &c.z, &c_lift],
            [&d.x, &d.z, &d_lift],
        );
        let x_y_lift_one = det4_with_unit_col3(
            [&a.x, &a.y, &a_lift],
            [&b.x, &b.y, &b_lift],
            [&c.x, &c.y, &c_lift],
            [&d.x, &d.y, &d_lift],
        );
        let x_y_z_one = det4_with_unit_col3(
            [&a.x, &a.y, &a.z],
            [&b.x, &b.y, &b.z],
            [&c.x, &c.y, &c.z],
            [&d.x, &d.y, &d.z],
        );
        let x_y_z_lift = det4_refs(
            [&a.x, &a.y, &a.z, &a_lift],
            [&b.x, &b.y, &b.z, &b_lift],
            [&c.x, &c.y, &c.z, &c_lift],
            [&d.x, &d.y, &d.z, &d_lift],
        );
        let x_coeff = y_z_lift_one;
        let y_coeff = neg(&x_z_lift_one);
        let z_coeff = x_y_lift_one;
        let lift_coeff = neg(&x_y_z_one);
        let constant = x_y_z_lift;
        let coefficient_facts =
            lifted_polynomial_facts([&x_coeff, &y_coeff, &z_coeff, &lift_coeff, &constant]);

        Self {
            a,
            b,
            c,
            d,
            facts: PreparedPredicateFacts::insphere3(a, b, c, d),
            coefficient_facts,
            x_coeff,
            y_coeff,
            z_coeff,
            lift_coeff,
            constant,
        }
    }

    /// Test a point using the default predicate policy.
    pub fn test_point(&self, point: &Point3) -> PredicateOutcome<Sign> {
        self.test_point_with_policy(point, PredicatePolicy::default())
    }

    /// Return cheap fixed-coordinate facts collected at preparation time.
    pub const fn facts(&self) -> PreparedPredicateFacts {
        self.facts
    }

    /// Return structural facts for the cached lifted-sphere polynomial.
    pub const fn coefficient_facts(&self) -> PreparedLiftedPolynomialFacts {
        self.coefficient_facts
    }

    /// Return borrowed cached coefficients for the lifted-sphere polynomial.
    pub const fn polynomial(&self) -> PreparedSphere3Polynomial<'_> {
        PreparedSphere3Polynomial {
            x_coeff: &self.x_coeff,
            y_coeff: &self.y_coeff,
            z_coeff: &self.z_coeff,
            lift_coeff: &self.lift_coeff,
            constant: &self.constant,
        }
    }

    /// Test a point using an explicit predicate policy.
    pub fn test_point_with_policy(
        &self,
        point: &Point3,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<Sign> {
        if let Some(report) = exact_report(
            policy,
            ExactPredicateKernel::Insphere3dRationalLiftedDet4,
            || {
                super::exact::insphere3d_shared_scale(self.a, self.b, self.c, self.d, point)
                    .or_else(|| super::exact::insphere3d(self.a, self.b, self.c, self.d, point))
            },
        ) {
            return report.outcome;
        }

        crate::trace_dispatch!("hyperlimit", "prepared_insphere3", "sphere-polynomial");
        let x_term = mul(&self.x_coeff, &point.x);
        let y_term = mul(&self.y_coeff, &point.y);
        let z_term = mul(&self.z_coeff, &point.z);
        let lift = point3_lift(point);
        let lift_term = mul(&self.lift_coeff, &lift);
        let xy = add(&x_term, &y_term);
        let xyz = add(&xy, &z_term);
        let xyzl = add(&xyz, &lift_term);
        let det = add(&xyzl, &self.constant);

        resolve_real_sign(
            &det,
            policy,
            || None,
            || {
                super::exact::insphere3d_shared_scale(self.a, self.b, self.c, self.d, point)
                    .or_else(|| super::exact::insphere3d(self.a, self.b, self.c, self.d, point))
            },
            RefinementNeed::RealRefinement,
        )
    }
}

fn add(left: &Real, right: &Real) -> Real {
    add_ref(left, right)
}

fn sub(left: &Real, right: &Real) -> Real {
    sub_ref(left, right)
}

fn neg(value: &Real) -> Real {
    sub(&sub(value, value), value)
}

fn point2_lift(point: &Point2) -> Real {
    add(&mul(&point.x, &point.x), &mul(&point.y, &point.y))
}

fn point3_lift(point: &Point3) -> Real {
    add(
        &add(&mul(&point.x, &point.x), &mul(&point.y, &point.y)),
        &mul(&point.z, &point.z),
    )
}

fn det3_refs(a: [&Real; 3], b: [&Real; 3], c: [&Real; 3]) -> Real {
    // Prepared circle/sphere coefficients repeatedly build 3x3 determinants.
    // Preserve the determinant as a fixed six-term product-sum so exact
    // rational coordinates use one fraction-delayed reduction, following
    // Bareiss, "Sylvester's Identity and Multistep Integer-Preserving Gaussian
    // Elimination," *Mathematics of Computation* 22.103 (1968), and Yap's
    // instruction to delay scalar expansion until after object-shape dispatch.
    Real::signed_product_sum(
        [true, false, false, true, true, false],
        [
            [a[0], b[1], c[2]],
            [a[0], b[2], c[1]],
            [a[1], b[0], c[2]],
            [a[1], b[2], c[0]],
            [a[2], b[0], c[1]],
            [a[2], b[1], c[0]],
        ],
    )
}

fn det3_with_unit_col2(a0: &Real, a1: &Real, b0: &Real, b1: &Real, c0: &Real, c1: &Real) -> Real {
    Real::signed_product_sum(
        [true, false, false, true, true, false],
        [[a0, b1], [a0, c1], [a1, b0], [a1, c0], [b0, c1], [b1, c0]],
    )
}

fn det4_refs(a: [&Real; 4], b: [&Real; 4], c: [&Real; 4], d: [&Real; 4]) -> Real {
    let minor0 = det3_refs([b[1], b[2], b[3]], [c[1], c[2], c[3]], [d[1], d[2], d[3]]);
    let minor1 = det3_refs([b[0], b[2], b[3]], [c[0], c[2], c[3]], [d[0], d[2], d[3]]);
    let minor2 = det3_refs([b[0], b[1], b[3]], [c[0], c[1], c[3]], [d[0], d[1], d[3]]);
    let minor3 = det3_refs([b[0], b[1], b[2]], [c[0], c[1], c[2]], [d[0], d[1], d[2]]);

    // The Laplace expansion is still only a fallback for prepared symbolic
    // coefficients, but keep the cofactor combination as a fixed product-sum
    // instead of immediately materializing four products. This carries Yap's
    // determinant object shape one layer deeper and gives exact-rational
    // prepared coefficients the same Bareiss-style delayed normalization route
    // cited in `det3_refs`.
    Real::signed_product_sum(
        [true, false, true, false],
        [
            [a[0], &minor0],
            [a[1], &minor1],
            [a[2], &minor2],
            [a[3], &minor3],
        ],
    )
}

fn det4_with_unit_col3(a: [&Real; 3], b: [&Real; 3], c: [&Real; 3], d: [&Real; 3]) -> Real {
    let ad0 = sub(a[0], d[0]);
    let ad1 = sub(a[1], d[1]);
    let ad2 = sub(a[2], d[2]);
    let bd0 = sub(b[0], d[0]);
    let bd1 = sub(b[1], d[1]);
    let bd2 = sub(b[2], d[2]);
    let cd0 = sub(c[0], d[0]);
    let cd1 = sub(c[1], d[1]);
    let cd2 = sub(c[2], d[2]);
    det3_refs([&ad0, &ad1, &ad2], [&bd0, &bd1, &bd2], [&cd0, &cd1, &cd2])
}

fn mul(left: &Real, right: &Real) -> Real {
    mul_ref(left, right)
}

fn exact_report(
    policy: PredicatePolicy,
    kernel: ExactPredicateKernel,
    exact: impl FnOnce() -> Option<Sign>,
) -> Option<PredicateReport<Sign>> {
    if !policy.allow_exact {
        return None;
    }

    exact().map(|sign| {
        PredicateReport::new(
            PredicateOutcome::decided(sign, Certainty::Exact, Escalation::Exact),
            PredicateCertificate::ExactRationalKernel { kernel },
        )
    })
}

fn fixed_point_facts_2<const N: usize>(
    points: [&Point2; N],
    kernel: ExactPredicateKernel,
) -> PreparedPredicateFacts {
    // Delegate scalar representation classification to `hyperreal` and retain
    // only the predicate-level summary here. This keeps denominator identity
    // opaque while still carrying the common-scale fact Yap asks geometric
    // objects to preserve before exact kernel selection.
    let facts = Real::exact_set_facts(points.iter().flat_map(|point| [&point.x, &point.y]));
    let point_masks = fixed_point_structure_masks_2(points);

    PreparedPredicateFacts {
        fixed_coordinates_exact_rational: facts.all_exact_rational,
        fixed_coordinates_dyadic: facts.all_dyadic,
        fixed_coordinates_shared_denominator: facts.shared_denominator,
        fixed_point_shared_scale_mask: point_masks.shared_scale,
        fixed_point_origin_mask: point_masks.origin,
        fixed_point_one_hot_mask: point_masks.one_hot,
        fixed_point_unknown_zero_mask: point_masks.unknown_zero,
        fixed_symbolic_dependencies: point_masks.symbolic_dependencies,
        exact_kernel_hint: facts.all_exact_rational.then_some(kernel),
    }
}

fn lifted_polynomial_facts<const N: usize>(
    coefficients: [&Real; N],
) -> PreparedLiftedPolynomialFacts {
    debug_assert!(N <= u128::BITS as usize);
    // Coefficient facts are kept at the prepared-object boundary rather than
    // recomputed by triangulation or CSG code. This is a direct application of
    // Yap's EGC discipline: preserve useful numerical structure on the
    // geometric object, then use it to select faster exact arithmetic packages.
    let coefficient_exact = Real::exact_set_facts(coefficients);
    let (coefficient_zero_mask, coefficient_nonzero_mask, coefficient_unknown_zero_mask) =
        real_zero_masks(coefficients);

    PreparedLiftedPolynomialFacts {
        coefficient_exact,
        coefficient_zero_mask,
        coefficient_nonzero_mask,
        coefficient_unknown_zero_mask,
    }
}

fn real_zero_masks<const N: usize>(coordinates: [&Real; N]) -> (u128, u128, u128) {
    debug_assert!(N <= u128::BITS as usize);
    let mut known_zero_mask = 0_u128;
    let mut known_nonzero_mask = 0_u128;
    let mut unknown_zero_mask = 0_u128;
    for (index, coordinate) in coordinates.into_iter().enumerate() {
        let bit = 1_u128 << index;
        match coordinate.structural_facts().zero {
            ZeroKnowledge::Zero => known_zero_mask |= bit,
            ZeroKnowledge::NonZero => known_nonzero_mask |= bit,
            ZeroKnowledge::Unknown => unknown_zero_mask |= bit,
        }
    }
    (known_zero_mask, known_nonzero_mask, unknown_zero_mask)
}

fn fixed_point_facts_3<const N: usize>(
    points: [&Point3; N],
    kernel: ExactPredicateKernel,
) -> PreparedPredicateFacts {
    let facts = Real::exact_set_facts(
        points
            .iter()
            .flat_map(|point| [&point.x, &point.y, &point.z]),
    );
    let point_masks = fixed_point_structure_masks_3(points);

    PreparedPredicateFacts {
        fixed_coordinates_exact_rational: facts.all_exact_rational,
        fixed_coordinates_dyadic: facts.all_dyadic,
        fixed_coordinates_shared_denominator: facts.shared_denominator,
        fixed_point_shared_scale_mask: point_masks.shared_scale,
        fixed_point_origin_mask: point_masks.origin,
        fixed_point_one_hot_mask: point_masks.one_hot,
        fixed_point_unknown_zero_mask: point_masks.unknown_zero,
        fixed_symbolic_dependencies: point_masks.symbolic_dependencies,
        exact_kernel_hint: facts.all_exact_rational.then_some(kernel),
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct FixedPointStructureMasks {
    shared_scale: u128,
    origin: u128,
    one_hot: u128,
    unknown_zero: u128,
    symbolic_dependencies: RealSymbolicDependencyMask,
}

#[inline]
fn fixed_point_structure_masks_2<const N: usize>(points: [&Point2; N]) -> FixedPointStructureMasks {
    debug_assert!(N <= u128::BITS as usize);
    let mut masks = FixedPointStructureMasks::default();
    for (index, point) in points.into_iter().enumerate() {
        let bit = 1_u128 << index;
        let facts = point.structural_facts();
        if facts.exact.has_shared_denominator_schedule() {
            masks.shared_scale |= bit;
        }
        if facts.known_zero {
            masks.origin |= bit;
        }
        if facts.is_one_hot() {
            masks.one_hot |= bit;
        }
        if facts.has_unknown_zero() {
            masks.unknown_zero |= bit;
        }
        masks.symbolic_dependencies = masks
            .symbolic_dependencies
            .union(facts.symbolic_dependencies);
    }
    masks
}

#[inline]
fn fixed_point_structure_masks_3<const N: usize>(points: [&Point3; N]) -> FixedPointStructureMasks {
    debug_assert!(N <= u128::BITS as usize);
    let mut masks = FixedPointStructureMasks::default();
    for (index, point) in points.into_iter().enumerate() {
        let bit = 1_u128 << index;
        let facts = point.structural_facts();
        if facts.exact.has_shared_denominator_schedule() {
            masks.shared_scale |= bit;
        }
        if facts.known_zero {
            masks.origin |= bit;
        }
        if facts.is_one_hot() {
            masks.one_hot |= bit;
        }
        if facts.has_unknown_zero() {
            masks.unknown_zero |= bit;
        }
        masks.symbolic_dependencies = masks
            .symbolic_dependencies
            .union(facts.symbolic_dependencies);
    }
    masks
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::predicate::{Certainty, Escalation};
    use hyperreal::Rational;
    use proptest::prelude::*;

    #[cfg(feature = "dispatch-trace")]
    fn dispatch_trace_test_lock() -> &'static std::sync::Mutex<()> {
        static LOCK: std::sync::OnceLock<std::sync::Mutex<()>> = std::sync::OnceLock::new();
        LOCK.get_or_init(|| std::sync::Mutex::new(()))
    }

    fn real(value: f64) -> Real {
        Real::try_from(value).expect("finite test Real")
    }

    fn p2(x: f64, y: f64) -> Point2 {
        Point2::new(real(x), real(y))
    }

    fn p3(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(real(x), real(y), real(z))
    }

    fn rational(value: i32) -> Real {
        Real::from(value)
    }

    fn rp2(x: i32, y: i32) -> Point2 {
        Point2::new(rational(x), rational(y))
    }

    fn rp3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(rational(x), rational(y), rational(z))
    }

    #[test]
    fn orient2d_classifies_simple_triangle() {
        let a = p2(0.0, 0.0);
        let b = p2(1.0, 0.0);
        let c = p2(0.0, 1.0);
        assert_eq!(orient2d(&a, &b, &c).value(), Some(Sign::Positive));
    }

    #[test]
    fn orient2d_decides_strict_degenerate_reals_exactly() {
        let a = p2(0.0, 0.0);
        let b = p2(1.0, 1.0);
        let c = p2(2.0, 2.0);
        assert_eq!(
            orient2d_with_policy(&a, &b, &c, PredicatePolicy::STRICT),
            PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::Exact)
        );
    }

    #[test]
    fn exact_rational_predicates_do_not_need_refinement_budget() {
        let policy = PredicatePolicy {
            allow_refinement: false,
            max_refinement_precision: Some(0),
            ..PredicatePolicy::STRICT
        };

        let a = Point2::new(Real::from(0), Real::from(0));
        let b = Point2::new(Real::from(2), Real::from(0));
        let c = Point2::new(Real::from(0), Real::from(2));
        let d = Point2::new(Real::from(1), Real::from(1));

        assert_eq!(
            orient2d_with_policy(&a, &b, &c, policy),
            PredicateOutcome::decided(Sign::Positive, Certainty::Exact, Escalation::Exact)
        );
        assert_eq!(
            incircle2d_with_policy(&a, &b, &c, &d, policy),
            PredicateOutcome::decided(Sign::Positive, Certainty::Exact, Escalation::Exact)
        );

        let p = Point3::new(Real::from(0), Real::from(0), Real::from(0));
        let q = Point3::new(Real::from(1), Real::from(0), Real::from(0));
        let r = Point3::new(Real::from(0), Real::from(1), Real::from(0));
        let s = Point3::new(Real::from(0), Real::from(0), Real::from(1));
        let t = Point3::new(
            Real::from(Rational::fraction(1, 4).unwrap()),
            Real::from(Rational::fraction(1, 4).unwrap()),
            Real::from(Rational::fraction(1, 4).unwrap()),
        );

        assert_eq!(
            orient3d_with_policy(&p, &q, &r, &s, policy),
            PredicateOutcome::decided(Sign::Negative, Certainty::Exact, Escalation::Exact)
        );
        assert_eq!(
            insphere3d_with_policy(&p, &q, &r, &s, &t, policy),
            PredicateOutcome::decided(Sign::Negative, Certainty::Exact, Escalation::Exact)
        );
    }

    #[test]
    fn orient3d_classifies_simple_tetrahedron() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(1.0, 0.0, 0.0);
        let c = p3(0.0, 1.0, 0.0);
        let d = p3(0.0, 0.0, 1.0);
        assert_eq!(orient3d(&a, &b, &c, &d).value(), Some(Sign::Negative));
    }

    #[test]
    fn exact_rational_reports_identify_selected_kernel() {
        let policy = PredicatePolicy {
            allow_refinement: false,
            ..PredicatePolicy::STRICT
        };

        let a = Point2::new(Real::from(0), Real::from(0));
        let b = Point2::new(Real::from(2), Real::from(0));
        let c = Point2::new(Real::from(0), Real::from(2));
        let d = Point2::new(Real::from(1), Real::from(1));

        assert_eq!(
            orient2d_report_with_policy(&a, &b, &c, policy).certificate,
            PredicateCertificate::ExactRationalKernel {
                kernel: ExactPredicateKernel::Orient2dRationalDet2
            }
        );
        assert_eq!(
            incircle2d_report_with_policy(&a, &b, &c, &d, policy).certificate,
            PredicateCertificate::ExactRationalKernel {
                kernel: ExactPredicateKernel::Incircle2dRationalLiftedDet3
            }
        );

        let p = Point3::new(Real::from(0), Real::from(0), Real::from(0));
        let q = Point3::new(Real::from(1), Real::from(0), Real::from(0));
        let r = Point3::new(Real::from(0), Real::from(1), Real::from(0));
        let s = Point3::new(Real::from(0), Real::from(0), Real::from(1));
        let t = Point3::new(
            Real::from(Rational::fraction(1, 4).unwrap()),
            Real::from(Rational::fraction(1, 4).unwrap()),
            Real::from(Rational::fraction(1, 4).unwrap()),
        );

        assert_eq!(
            orient3d_report_with_policy(&p, &q, &r, &s, policy).certificate,
            PredicateCertificate::ExactRationalKernel {
                kernel: ExactPredicateKernel::Orient3dRationalDet3
            }
        );
        assert_eq!(
            insphere3d_report_with_policy(&p, &q, &r, &s, &t, policy).certificate,
            PredicateCertificate::ExactRationalKernel {
                kernel: ExactPredicateKernel::Insphere3dRationalLiftedDet4
            }
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn orient2d_consumes_global_common_denominator_before_generic_shared_scale() {
        let _trace_lock = dispatch_trace_test_lock()
            .lock()
            .expect("dispatch trace test lock poisoned");
        let point = |x, y| {
            Point2::new(
                Real::from(Rational::fraction(x, 5).unwrap()),
                Real::from(Rational::fraction(y, 5).unwrap()),
            )
        };
        let a = point(1, 1);
        let b = point(4, 1);
        let c = point(1, 3);

        hyperreal::dispatch_trace::reset();
        let report = hyperreal::dispatch_trace::with_recording(|| {
            orient2d_report_with_policy(&a, &b, &c, PredicatePolicy::STRICT)
        });

        assert_eq!(report.outcome.value(), Some(Sign::Positive));
        assert_eq!(
            report.certificate,
            PredicateCertificate::ExactRationalKernel {
                kernel: ExactPredicateKernel::Orient2dRationalDet2
            }
        );

        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient2d", "common-denominator-det2"),
            1
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient2d", "shared-scale-view-det2"),
            0
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient2d", "rational-det2"),
            0
        );
        assert_eq!(
            trace.path_count("real", "product_sum", "exact-rational-known-common-scale"),
            1
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn orient2d_keeps_point_local_shared_scale_when_global_denominator_differs() {
        let _trace_lock = dispatch_trace_test_lock()
            .lock()
            .expect("dispatch trace test lock poisoned");
        let point = |x, y, denominator| {
            Point2::new(
                Real::from(Rational::fraction(x, denominator).unwrap()),
                Real::from(Rational::fraction(y, denominator).unwrap()),
            )
        };
        let a = point(1, 1, 5);
        let b = point(4, 1, 7);
        let c = point(1, 3, 11);

        hyperreal::dispatch_trace::reset();
        let report = hyperreal::dispatch_trace::with_recording(|| {
            orient2d_report_with_policy(&a, &b, &c, PredicatePolicy::STRICT)
        });

        assert_eq!(report.outcome.value(), Some(Sign::Positive));
        assert_eq!(
            report.certificate,
            PredicateCertificate::ExactRationalKernel {
                kernel: ExactPredicateKernel::Orient2dRationalDet2
            }
        );

        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient2d", "common-denominator-det2"),
            0
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient2d", "shared-scale-view-det2"),
            1
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient2d", "rational-det2"),
            0
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn orient3d_consumes_global_common_denominator_before_generic_shared_scale() {
        let _trace_lock = dispatch_trace_test_lock()
            .lock()
            .expect("dispatch trace test lock poisoned");
        let point = |x, y, z| {
            Point3::new(
                Real::from(Rational::fraction(x, 7).unwrap()),
                Real::from(Rational::fraction(y, 7).unwrap()),
                Real::from(Rational::fraction(z, 7).unwrap()),
            )
        };
        let a = point(1, 1, 1);
        let b = point(4, 1, 1);
        let c = point(1, 4, 1);
        let d = point(1, 1, 3);

        hyperreal::dispatch_trace::reset();
        let report = hyperreal::dispatch_trace::with_recording(|| {
            orient3d_report_with_policy(&a, &b, &c, &d, PredicatePolicy::STRICT)
        });

        assert_eq!(report.outcome.value(), Some(Sign::Negative));
        assert_eq!(
            report.certificate,
            PredicateCertificate::ExactRationalKernel {
                kernel: ExactPredicateKernel::Orient3dRationalDet3
            }
        );

        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient3d", "common-denominator-det4"),
            1
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient3d", "shared-scale-view-det3"),
            0
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient3d", "rational-det3"),
            0
        );
        assert_eq!(
            trace.path_count("real", "product_sum", "exact-rational-known-common-scale"),
            1
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn orient3d_keeps_point_local_shared_scale_when_global_denominator_differs() {
        let _trace_lock = dispatch_trace_test_lock()
            .lock()
            .expect("dispatch trace test lock poisoned");
        let point = |x, y, z, denominator| {
            Point3::new(
                Real::from(Rational::fraction(x, denominator).unwrap()),
                Real::from(Rational::fraction(y, denominator).unwrap()),
                Real::from(Rational::fraction(z, denominator).unwrap()),
            )
        };
        let a = point(1, 1, 1, 5);
        let b = point(4, 1, 1, 7);
        let c = point(1, 4, 1, 11);
        let d = point(1, 1, 3, 13);

        hyperreal::dispatch_trace::reset();
        let report = hyperreal::dispatch_trace::with_recording(|| {
            orient3d_report_with_policy(&a, &b, &c, &d, PredicatePolicy::STRICT)
        });

        assert_eq!(report.outcome.value(), Some(Sign::Positive));
        assert_eq!(
            report.certificate,
            PredicateCertificate::ExactRationalKernel {
                kernel: ExactPredicateKernel::Orient3dRationalDet3
            }
        );

        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient3d", "common-denominator-det4"),
            0
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient3d", "shared-scale-view-det3"),
            1
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient3d", "rational-det3"),
            0
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn incircle2d_consumes_borrowed_shared_scale_views_before_rational_fallback() {
        let _trace_lock = dispatch_trace_test_lock()
            .lock()
            .expect("dispatch trace test lock poisoned");
        let point = |x, y| {
            Point2::new(
                Real::from(Rational::fraction(x, 7).unwrap()),
                Real::from(Rational::fraction(y, 7).unwrap()),
            )
        };
        let a = point(1, 1);
        let b = point(4, 1);
        let c = point(1, 4);
        let d = point(2, 2);

        hyperreal::dispatch_trace::reset();
        let report = hyperreal::dispatch_trace::with_recording(|| {
            incircle2d_report_with_policy(&a, &b, &c, &d, PredicatePolicy::STRICT)
        });

        assert_eq!(report.outcome.value(), Some(Sign::Positive));
        assert_eq!(
            report.certificate,
            PredicateCertificate::ExactRationalKernel {
                kernel: ExactPredicateKernel::Incircle2dRationalLiftedDet3
            }
        );

        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count(
                "hyperlimit",
                "exact_incircle2d",
                "shared-scale-view-lifted-det3"
            ),
            1
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_incircle2d", "rational-det3-lifted"),
            0
        );
        assert_eq!(
            trace.path_count("real", "product_sum", "exact-rational-known-shared-denom"),
            4
        );

        let prepared = PreparedIncircle2::new(&a, &b, &c);
        hyperreal::dispatch_trace::reset();
        let prepared_outcome = hyperreal::dispatch_trace::with_recording(|| {
            prepared.test_point_with_policy(&d, PredicatePolicy::STRICT)
        });
        assert_eq!(prepared_outcome.value(), Some(Sign::Positive));
        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count(
                "hyperlimit",
                "exact_incircle2d",
                "shared-scale-view-lifted-det3"
            ),
            1
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_incircle2d", "rational-det3-lifted"),
            0
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn insphere3d_consumes_borrowed_shared_scale_views_before_rational_fallback() {
        let _trace_lock = dispatch_trace_test_lock()
            .lock()
            .expect("dispatch trace test lock poisoned");
        let point = |x, y, z| {
            Point3::new(
                Real::from(Rational::fraction(x, 7).unwrap()),
                Real::from(Rational::fraction(y, 7).unwrap()),
                Real::from(Rational::fraction(z, 7).unwrap()),
            )
        };
        let a = point(1, 1, 1);
        let b = point(4, 1, 1);
        let c = point(1, 4, 1);
        let d = point(1, 1, 4);
        let e = point(2, 2, 2);

        hyperreal::dispatch_trace::reset();
        let report = hyperreal::dispatch_trace::with_recording(|| {
            insphere3d_report_with_policy(&a, &b, &c, &d, &e, PredicatePolicy::STRICT)
        });

        assert_eq!(report.outcome.value(), Some(Sign::Negative));
        assert_eq!(
            report.certificate,
            PredicateCertificate::ExactRationalKernel {
                kernel: ExactPredicateKernel::Insphere3dRationalLiftedDet4
            }
        );

        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count(
                "hyperlimit",
                "exact_insphere3d",
                "shared-scale-view-lifted-det4"
            ),
            1
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_insphere3d", "rational-det4-lifted"),
            0
        );

        let prepared = PreparedInsphere3::new(&a, &b, &c, &d);
        hyperreal::dispatch_trace::reset();
        let prepared_outcome = hyperreal::dispatch_trace::with_recording(|| {
            prepared.test_point_with_policy(&e, PredicatePolicy::STRICT)
        });
        assert_eq!(prepared_outcome.value(), Some(Sign::Negative));
        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count(
                "hyperlimit",
                "exact_insphere3d",
                "shared-scale-view-lifted-det4"
            ),
            1
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_insphere3d", "rational-det4-lifted"),
            0
        );
    }

    #[test]
    fn prepared_line_matches_orient2d_side() {
        let a = p2(-1.0, -1.0);
        let b = p2(1.0, 1.0);
        let prepared = PreparedLine2::new(&a, &b);
        assert_eq!(
            prepared.facts().exact_kernel_hint,
            Some(ExactPredicateKernel::Orient2dRationalDet2)
        );
        assert!(prepared.facts().fixed_coordinates_shared_denominator);
        for point in [p2(-0.75, -0.5), p2(0.5, 0.25), p2(0.125, 0.125)] {
            assert_eq!(
                prepared.classify_point(&point).value(),
                classify_point_line(&a, &b, &point).value()
            );
        }
    }

    #[test]
    fn prepared_facts_distinguish_mixed_denominators() {
        let a = Point2::new(
            Real::from(Rational::fraction(1, 3).unwrap()),
            Real::from(Rational::fraction(2, 3).unwrap()),
        );
        let b = Point2::new(
            Real::from(Rational::fraction(1, 5).unwrap()),
            Real::from(Rational::fraction(2, 5).unwrap()),
        );
        let prepared = PreparedLine2::new(&a, &b);

        assert!(prepared.facts().fixed_coordinates_exact_rational);
        assert!(!prepared.facts().fixed_coordinates_dyadic);
        assert!(!prepared.facts().fixed_coordinates_shared_denominator);
        assert_eq!(prepared.facts().fixed_point_shared_scale_mask, 0b11);
    }

    #[test]
    fn prepared_facts_detect_common_reduced_denominator() {
        let p2r = |x, y| {
            Point2::new(
                Real::from(Rational::fraction(x, 7).unwrap()),
                Real::from(Rational::fraction(y, 7).unwrap()),
            )
        };
        let a = p2r(1, 2);
        let b = p2r(3, 4);
        let c = p2r(5, 6);

        assert!(
            PreparedLine2::new(&a, &b)
                .facts()
                .fixed_coordinates_shared_denominator
        );
        assert_eq!(
            PreparedLine2::new(&a, &b)
                .facts()
                .fixed_point_shared_scale_mask,
            0b11
        );
        assert!(
            PreparedIncircle2::new(&a, &b, &c)
                .facts()
                .fixed_coordinates_shared_denominator
        );
        assert_eq!(
            PreparedIncircle2::new(&a, &b, &c)
                .facts()
                .fixed_point_shared_scale_mask,
            0b111
        );

        let p3r = |x, y, z| {
            Point3::new(
                Real::from(Rational::fraction(x, 11).unwrap()),
                Real::from(Rational::fraction(y, 11).unwrap()),
                Real::from(Rational::fraction(z, 11).unwrap()),
            )
        };
        let p = p3r(1, 2, 3);
        let q = p3r(4, 5, 6);
        let r = p3r(7, 8, 9);
        let s = p3r(10, 12, 13);

        assert!(
            PreparedInsphere3::new(&p, &q, &r, &s)
                .facts()
                .fixed_coordinates_shared_denominator
        );
        assert_eq!(
            PreparedInsphere3::new(&p, &q, &r, &s)
                .facts()
                .fixed_point_shared_scale_mask,
            0b1111
        );
    }

    #[test]
    fn prepared_lifted_polynomial_facts_match_cached_coefficients() {
        let a = rp2(1, 0);
        let b = rp2(0, 1);
        let c = rp2(-1, 0);
        let prepared = PreparedIncircle2::new(&a, &b, &c);
        let poly = prepared.polynomial();

        assert_eq!(
            prepared.coefficient_facts(),
            lifted_polynomial_facts([poly.x_coeff, poly.y_coeff, poly.lift_coeff, poly.constant])
        );
        assert!(
            prepared
                .coefficient_facts()
                .coefficient_exact
                .all_exact_rational
        );
        assert!(prepared.coefficient_facts().has_dyadic_schedule());
        assert_eq!(
            prepared
                .coefficient_facts()
                .coefficient_unknown_zero_count(),
            0
        );

        let query = rp2(0, 0);
        let lift = point2_lift(&query);
        let det = add(
            &add(
                &add(&mul(poly.x_coeff, &query.x), &mul(poly.y_coeff, &query.y)),
                &mul(poly.lift_coeff, &lift),
            ),
            poly.constant,
        );
        assert_eq!(
            resolve_real_sign(
                &det,
                PredicatePolicy::STRICT,
                || None,
                || None,
                RefinementNeed::RealRefinement,
            )
            .value(),
            prepared.test_point(&query).value()
        );

        let p = rp3(0, 0, 0);
        let q = rp3(1, 0, 0);
        let r = rp3(0, 1, 0);
        let s = rp3(0, 0, 1);
        let sphere = PreparedInsphere3::new(&p, &q, &r, &s);
        let sphere_poly = sphere.polynomial();

        assert_eq!(
            sphere.coefficient_facts(),
            lifted_polynomial_facts([
                sphere_poly.x_coeff,
                sphere_poly.y_coeff,
                sphere_poly.z_coeff,
                sphere_poly.lift_coeff,
                sphere_poly.constant
            ])
        );
        assert!(
            sphere
                .coefficient_facts()
                .coefficient_exact
                .all_exact_rational
        );
        assert!(sphere.coefficient_facts().has_dyadic_schedule());
        assert_eq!(
            sphere.coefficient_facts().coefficient_unknown_zero_count(),
            0
        );

        let sphere_query = Point3::new(
            Real::from(Rational::fraction(1, 4).unwrap()),
            Real::from(Rational::fraction(1, 4).unwrap()),
            Real::from(Rational::fraction(1, 4).unwrap()),
        );
        let sphere_lift = point3_lift(&sphere_query);
        let sphere_det = add(
            &add(
                &add(
                    &add(
                        &mul(sphere_poly.x_coeff, &sphere_query.x),
                        &mul(sphere_poly.y_coeff, &sphere_query.y),
                    ),
                    &mul(sphere_poly.z_coeff, &sphere_query.z),
                ),
                &mul(sphere_poly.lift_coeff, &sphere_lift),
            ),
            sphere_poly.constant,
        );
        assert_eq!(
            resolve_real_sign(
                &sphere_det,
                PredicatePolicy::STRICT,
                || None,
                || None,
                RefinementNeed::RealRefinement,
            )
            .value(),
            sphere.test_point(&sphere_query).value()
        );
    }

    #[test]
    fn prepared_incircle_matches_incircle2d_sign() {
        let a = p2(0.82, 0.0);
        let b = p2(0.0, 0.82);
        let c = p2(-0.82, 0.0);
        let prepared = PreparedIncircle2::new(&a, &b, &c);
        assert_eq!(
            prepared.facts().exact_kernel_hint,
            Some(ExactPredicateKernel::Incircle2dRationalLiftedDet3)
        );
        for point in [p2(0.2, 0.1), p2(0.95, 0.0), p2(0.82, 0.0)] {
            assert_eq!(
                prepared.test_point(&point).value(),
                incircle2d(&a, &b, &c, &point).value()
            );
        }
    }

    #[test]
    fn prepared_insphere_matches_insphere3d_sign() {
        let a = p3(0.82, 0.0, 0.0);
        let b = p3(-0.82, 0.0, 0.0);
        let c = p3(0.0, 0.82, 0.0);
        let d = p3(0.0, 0.0, 0.82);
        let prepared = PreparedInsphere3::new(&a, &b, &c, &d);
        assert_eq!(
            prepared.facts().exact_kernel_hint,
            Some(ExactPredicateKernel::Insphere3dRationalLiftedDet4)
        );
        for point in [p3(0.1, 0.1, 0.1), p3(1.1, 0.0, 0.0), p3(0.82, 0.0, 0.0)] {
            assert_eq!(
                prepared.test_point(&point).value(),
                insphere3d(&a, &b, &c, &d, &point).value()
            );
        }
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(256))]

        #[test]
        fn exact_orient2d_is_translation_invariant(
            ax in -64_i32..64, ay in -64_i32..64,
            bx in -64_i32..64, by in -64_i32..64,
            cx in -64_i32..64, cy in -64_i32..64,
            tx in -64_i32..64, ty in -64_i32..64,
        ) {
            let policy = PredicatePolicy {
                allow_refinement: false,
                ..PredicatePolicy::STRICT
            };
            let a = rp2(ax, ay);
            let b = rp2(bx, by);
            let c = rp2(cx, cy);
            let moved_a = rp2(ax + tx, ay + ty);
            let moved_b = rp2(bx + tx, by + ty);
            let moved_c = rp2(cx + tx, cy + ty);

            prop_assert_eq!(
                orient2d_with_policy(&a, &b, &c, policy),
                orient2d_with_policy(&moved_a, &moved_b, &moved_c, policy)
            );
        }

        #[test]
        fn exact_orient3d_is_translation_invariant(
            ax in -16_i32..16, ay in -16_i32..16, az in -16_i32..16,
            bx in -16_i32..16, by in -16_i32..16, bz in -16_i32..16,
            cx in -16_i32..16, cy in -16_i32..16, cz in -16_i32..16,
            dx in -16_i32..16, dy in -16_i32..16, dz in -16_i32..16,
            tx in -16_i32..16, ty in -16_i32..16, tz in -16_i32..16,
        ) {
            let policy = PredicatePolicy {
                allow_refinement: false,
                ..PredicatePolicy::STRICT
            };
            let a = rp3(ax, ay, az);
            let b = rp3(bx, by, bz);
            let c = rp3(cx, cy, cz);
            let d = rp3(dx, dy, dz);
            let moved_a = rp3(ax + tx, ay + ty, az + tz);
            let moved_b = rp3(bx + tx, by + ty, bz + tz);
            let moved_c = rp3(cx + tx, cy + ty, cz + tz);
            let moved_d = rp3(dx + tx, dy + ty, dz + tz);

            prop_assert_eq!(
                orient3d_with_policy(&a, &b, &c, &d, policy),
                orient3d_with_policy(&moved_a, &moved_b, &moved_c, &moved_d, policy)
            );
        }

        #[test]
        fn exact_incircle_reports_boundary_for_input_site(
            ax in -16_i32..16, ay in -16_i32..16,
            bx in -16_i32..16, by in -16_i32..16,
            cx in -16_i32..16, cy in -16_i32..16,
        ) {
            let policy = PredicatePolicy {
                allow_refinement: false,
                ..PredicatePolicy::STRICT
            };
            let a = rp2(ax, ay);
            let b = rp2(bx, by);
            let c = rp2(cx, cy);

            prop_assert_eq!(
                incircle2d_with_policy(&a, &b, &c, &a, policy),
                PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::Exact)
            );
        }

        #[test]
        fn exact_insphere_reports_boundary_for_input_site(
            ax in -8_i32..8, ay in -8_i32..8, az in -8_i32..8,
            bx in -8_i32..8, by in -8_i32..8, bz in -8_i32..8,
            cx in -8_i32..8, cy in -8_i32..8, cz in -8_i32..8,
            dx in -8_i32..8, dy in -8_i32..8, dz in -8_i32..8,
        ) {
            let policy = PredicatePolicy {
                allow_refinement: false,
                ..PredicatePolicy::STRICT
            };
            let a = rp3(ax, ay, az);
            let b = rp3(bx, by, bz);
            let c = rp3(cx, cy, cz);
            let d = rp3(dx, dy, dz);

            prop_assert_eq!(
                insphere3d_with_policy(&a, &b, &c, &d, &a, policy),
                PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::Exact)
            );
        }
    }
}
