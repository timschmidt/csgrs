//! Oriented determinant signs and reusable line/circle/sphere query evidence.
//!
//! Scalar filters and rational kernels precede exact Real expressions and the
//! policy resolver. A declined filter is inconclusive; a terminal-permitting
//! policy can return an Approximate sign whose receipt must remain attached.
//! A determinant zero alone does not validate a nondegenerate supporting object.
//!
//! Retained evidence stores arithmetic data, not a checked association with later
//! source arguments. Reuse requires the same ordered source coordinates and,
//! for retained queries, the same query coordinates. Rebuild after a changed
//! source or transform. Public fact snapshots are scheduling hints, not proofs
//! of incidence, source identity or validity. These schedules add no total-work,
//! retained-memory or cancellation budget beyond their underlying scalar calls.

use crate::SymbolicDependencyMask;
use crate::classify::LineSide;
/// Shared point carriers used by orientation predicates; no alternate geometry type.
pub use crate::geometry::{Point2, Point3};
use crate::predicate::PredicatePolicy;
use crate::predicate::{
    Certainty, DeterminantScheduleHint, Escalation, ExactPredicateKernel, PredicateOutcome,
    RefinementNeed, Sign,
};
use crate::predicates::exact::sign_from_ordering;
use crate::real::{add_ref, mul_ref, sub_ref};
use crate::resolve::{map_outcome, resolve_real_sign, signed_term_filter};
use hyperreal::{
    AffineDet2ExactWordFilter, AffineDet2ExactWordQuery, AffineDet2Filter, Incircle2Filter,
    Insphere3Filter, Rational, RationalLinearForm4Filter, RationalLinearForm4Query, Real,
    RealExactSetFacts, ZeroKnowledge,
};

/// Orientation of three 2D points with an explicit escalation policy.
///
/// Returns the sign of `(b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x)`:
/// counterclockwise is positive. Collinear or repeated points may give Zero;
/// no distinctness check is performed. Keep the outcome's certainty, including
/// any approximate terminal qualification.
pub fn orient2d_with_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    orient2d_coordinates_with_policy([&a.x, &a.y], [&b.x, &b.y], [&c.x, &c.y], policy)
}

/// Runs the 2D orientation cascade over borrowed coordinates in x/y order.
///
/// Tries certified dyadic, checked word, certified rational and full rational
/// paths in that order, then the Real-expression resolver. These successful
/// shortcuts use Exact/Exact receipts; decline does not classify the geometry.
pub(crate) fn orient2d_coordinates_with_policy(
    a: [&Real; 2],
    b: [&Real; 2],
    c: [&Real; 2],
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    if let Some(sign) = orient2d_certified_real_filter(a, b, c) {
        // The primitive operations are only a conservative proof shortcut over
        // exact dyadic Real values. Preserve the existing public exact-rational
        // semantics and kernel certificate; dispatch tracing still identifies
        // the faster internal route for profiling.
        return PredicateOutcome::decided(sign, Certainty::Exact, Escalation::Exact);
    }

    if let Some(sign) = orient2d_exact_word_filter(a, b, c) {
        return PredicateOutcome::decided(sign, Certainty::Exact, Escalation::Exact);
    }

    if let Some(sign) = orient2d_certified_rational_filter(a, b, c) {
        return PredicateOutcome::decided(sign, Certainty::Exact, Escalation::Exact);
    }

    // Structural-dispatch note: when callers carry integer-grid scale,
    // affine-transform conditioning, or dyadic denominator facts, this
    // predicate can choose a faster exact determinant expansion before building
    // the generic Real expression tree.
    if let Some(outcome) = exact_outcome(policy, ExactPredicateKernel::Orient2RationalDet2, || {
        super::exact::orient2d_coordinates(a, b, c)
    }) {
        return outcome;
    }
    orient2d_real_coordinates(a, b, c, policy)
}

/// Adapts points to the Real-expression orientation path without front filters.
fn orient2d_real_expr(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    orient2d_real_coordinates([&a.x, &a.y], [&b.x, &b.y], [&c.x, &c.y], policy)
}

/// Resolves the exact difference-of-products orientation expression.
///
/// Retains factor identity for scalar cancellation. The resolver can use a
/// signed-term filter or rational callback before refinement/terminal policy;
/// constructing the expression alone does not certify its sign.
pub(crate) fn orient2d_real_coordinates(
    a: [&Real; 2],
    b: [&Real; 2],
    c: [&Real; 2],
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    crate::trace_dispatch!("hyperlimit", "orient2d", "real-determinant");
    let abx = sub(b[0], a[0]);
    let aby = sub(b[1], a[1]);
    let acx = sub(c[0], a[0]);
    let acy = sub(c[1], a[1]);
    // Keep factor identity available to the scalar cancellation certificate.
    let det = Real::diff_of_products(&abx, &acy, &aby, &acx);

    resolve_real_sign(
        &det,
        policy,
        || {
            let left = mul(&abx, &acy);
            let right = mul(&aby, &acx);
            signed_term_filter(&[(&left, Sign::Positive), (&right, Sign::Negative)])
        },
        || super::exact::orient2d_coordinates(a, b, c),
        RefinementNeed::RealRefinement,
    )
}

/// Tries the scalar's certified dyadic affine determinant and maps its sign.
/// `None` leaves the exact fallback available; no approximate sign is returned.
#[inline]
pub(crate) fn orient2d_certified_real_filter(
    a: [&Real; 2],
    b: [&Real; 2],
    c: [&Real; 2],
) -> Option<Sign> {
    let sign = Real::certified_affine_det2_sign(a, b, c)?;
    crate::trace_dispatch!("hyperlimit", "orient2d", "certified-real-det2-filter");
    Some(crate::real::map_real_sign(sign))
}

/// Tries checked rational homogeneous word arithmetic; unsupported sizes decline.
#[inline]
pub(crate) fn orient2d_exact_word_filter(
    a: [&Real; 2],
    b: [&Real; 2],
    c: [&Real; 2],
) -> Option<Sign> {
    let sign = Real::exact_rational_affine_det2_word_sign(a, b, c)?;
    crate::trace_dispatch!("hyperlimit", "orient2d", "exact-word-rational-det2-filter");
    Some(crate::real::map_real_sign(sign))
}

/// Checks all six coordinates for rational storage before the rational filter.
/// Eligibility queries retain a-x/a-y/b-x/b-y/c-x/c-y order; no symbolic
/// rational normalization is attempted by this guard.
#[inline]
pub(crate) fn orient2d_certified_rational_filter(
    a: [&Real; 2],
    b: [&Real; 2],
    c: [&Real; 2],
) -> Option<Sign> {
    // Keep the eligibility guard in coordinate order so symbolic and mixed
    // calls decline before constructing a reusable rational filter. Fully
    // exact inputs then reuse each `Real`'s retained conversion schedule.
    let _ = (
        a[0].exact_rational_ref()?,
        a[1].exact_rational_ref()?,
        b[0].exact_rational_ref()?,
        b[1].exact_rational_ref()?,
        c[0].exact_rational_ref()?,
        c[1].exact_rational_ref()?,
    );
    orient2d_certified_rational_filter_exact(a, b, c)
}

/// Calls the certified rational line filter after the caller's representation guard.
/// An inconclusive enclosure, including possible exact contact, returns `None`.
#[inline(never)]
fn orient2d_certified_rational_filter_exact(
    a: [&Real; 2],
    b: [&Real; 2],
    c: [&Real; 2],
) -> Option<Sign> {
    let sign = Real::certified_rational_line2_sign(a, b, c)?;
    crate::trace_dispatch!("hyperlimit", "orient2d", "certified-rational-det2-filter");
    Some(crate::real::map_real_sign(sign))
}

/// Orientation of four 3D points with an explicit escalation policy.
///
/// Uses the determinant with rows `a-d`, `b-d`, `c-d`. The origin followed
/// by the ordered standard basis has negative sign, opposite to
/// [`crate::orient_d`]'s first-point-anchored convention in three dimensions.
/// Tries certified dyadic, checked word and rational paths before a six-term
/// Real determinant. Degenerate inputs are admitted; Zero does not prove a
/// valid tetrahedron, and an approximate result retains its qualification.
pub fn orient3d_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    if let Some(sign) = Real::certified_affine_det3_sign(
        [&a.x, &a.y, &a.z],
        [&b.x, &b.y, &b.z],
        [&c.x, &c.y, &c.z],
        [&d.x, &d.y, &d.z],
    ) {
        crate::trace_dispatch!("hyperlimit", "orient3d", "certified-real-det3-filter");
        return PredicateOutcome::decided(
            crate::real::map_real_sign(sign),
            Certainty::Exact,
            Escalation::Exact,
        );
    }

    if let Some(sign) = Real::exact_rational_affine_det3_word_sign(
        [&a.x, &a.y, &a.z],
        [&b.x, &b.y, &b.z],
        [&c.x, &c.y, &c.z],
        [&d.x, &d.y, &d.z],
    ) {
        crate::trace_dispatch!("hyperlimit", "orient3d", "exact-word-rational-det3-filter");
        return PredicateOutcome::decided(
            crate::real::map_real_sign(sign),
            Certainty::Exact,
            Escalation::Exact,
        );
    }

    if let Some(outcome) = exact_outcome(policy, ExactPredicateKernel::Orient3RationalDet3, || {
        super::exact::orient3d(a, b, c, d)
    }) {
        return outcome;
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

    // Keep the translated determinant as a six-term product-sum so the `Real`
    // layer can route exact rationals through one shared-denominator reducer.
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

    resolve_real_sign(
        &det,
        policy,
        || None,
        || super::exact::orient3d(a, b, c, d),
        RefinementNeed::RealRefinement,
    )
}

/// Classify `point` relative to the oriented line from `from` to `to` with an
/// explicit escalation policy.
///
/// Maps negative/zero/positive orientation to Right/On/Left without dropping
/// the receipt. Distinct endpoints are required to interpret this as a line:
/// coincident endpoints give a zero determinant for every query point.
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

/// Cheap facts retained by orientation and lifted-predicate evidence.
///
/// These facts are intentionally about the fixed part of retained evidence,
/// not about the query point. Repeated predicates can use them to select exact
/// rational, dyadic, or future shared-scale schedules before building scalar
/// expression trees for every query.
///
/// Fields are public snapshots with no source binding or consistency checks.
/// Bit positions follow source-point order. Constructors supply at most four
/// points here; the mask representation has capacity for 128. These summaries
/// do not establish nondegeneracy or force a particular dispatch path.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PredicateFacts {
    /// Every fixed coordinate is represented as an exact rational `Real`.
    pub fixed_coordinates_exact_rational: bool,
    /// Every fixed coordinate is represented as an exact dyadic rational.
    pub fixed_coordinates_dyadic: bool,
    /// All fixed exact-rational coordinates have the same reduced denominator.
    ///
    /// The evidence records that a shared-denominator schedule is
    /// eligible without owning a new coordinate representation.
    pub fixed_coordinates_shared_denominator: bool,
    /// Bit mask of fixed points whose own coordinates share one reduced
    /// denominator.
    ///
    /// This is deliberately weaker than
    /// [`PredicateFacts::fixed_coordinates_shared_denominator`]: predicate
    /// evidence may have point-local homogeneous/common-scale
    /// structure even when different fixed points use different grids. Carrying
    /// that object-local fact preserves the information needed for future
    /// homogeneous determinant schedules without exposing rational storage.
    pub fixed_point_shared_scale_mask: u128,
    /// Bit mask of fixed points structurally known to be the coordinate origin.
    ///
    /// This point-level sparse fact is retained with the predicate evidence rather
    /// than rediscovered in each query, allowing arithmetic schedules to be
    /// selected from reusable object facts.
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
    /// Keeping unknown-zero provenance avoids selecting
    /// sparse exact kernels from incomplete facts while still carrying the
    /// uncertainty explicitly.
    pub fixed_point_unknown_zero_mask: u128,
    /// Union of scalar symbolic dependency families across all fixed points.
    ///
    /// This is an evidence scheduling fact, not an exact predicate
    /// certificate. It lets repeated line, circle, plane, and sphere queries
    /// retain the same symbolic-family summary as their fixed point objects
    /// without exposing `Real` internals. Reusable expression structure reaches
    /// arithmetic selection, while predicate signs remain separately certified.
    pub fixed_symbolic_dependencies: SymbolicDependencyMask,
    /// Exact kernel that can be attempted when the query coordinates match.
    pub exact_kernel_hint: Option<ExactPredicateKernel>,
}

impl PredicateFacts {
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
    /// Unknown-zero facts must block sparse schedules that require certified
    /// support, so the uncertainty stays in the evidence layer.
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
    /// retained arithmetic packages, trace labels, and higher-level cache
    /// payoff estimates, but it is not a predicate certificate. Exact predicate
    /// reports still certify topology after reusable object structure selects a
    /// candidate arithmetic schedule.
    /// Priority is no kernel → generic, then sparse support with no unknown
    /// zero facts, shared denominator, dyadic, and finally the rational kernel.
    /// Publicly supplied inconsistent facts are not validated by this selection.
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

    /// Collects source-point facts in from/to order with the rational 2D kernel hint.
    fn line2(from: &Point2, to: &Point2) -> Self {
        fixed_point_facts_2([from, to], ExactPredicateKernel::Orient2RationalDet2)
    }

    /// Collects a/b/c facts for the lifted rational circle kernel, without a circle test.
    fn incircle2(a: &Point2, b: &Point2, c: &Point2) -> Self {
        fixed_point_facts_2([a, b, c], ExactPredicateKernel::Incircle2RationalLiftedDet3)
    }

    /// Collects a/b/c/d facts for the lifted rational sphere kernel, without a rank test.
    fn insphere3(a: &Point3, b: &Point3, c: &Point3, d: &Point3) -> Self {
        fixed_point_facts_3(
            [a, b, c, d],
            ExactPredicateKernel::Insphere3RationalLiftedDet4,
        )
    }
}

/// Structural facts for a retained lifted-circle or lifted-sphere polynomial.
///
/// An evidence-aware in-circle or in-sphere query evaluates a fixed polynomial in the
/// query point's coordinates. This fact package summarizes those fixed
/// coefficients so downstream caches can retain exact-set, dyadic,
/// shared-scale, and sparse-support opportunities without exposing internal
/// coefficient storage. Current query paths inspect scalar values and prepared
/// filters directly rather than dispatching from this snapshot.
///
/// Public fields are snapshots, not validated certificates bound to a
/// polynomial. Bits use coefficient order x, y, optional z, lift, constant.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LiftedPolynomialFacts {
    /// Exact-rational representation facts for the fixed polynomial coefficients.
    pub coefficient_exact: RealExactSetFacts,
    /// Bit mask of coefficients known to be exactly zero.
    pub coefficient_zero_mask: u128,
    /// Bit mask of coefficients known to be nonzero.
    pub coefficient_nonzero_mask: u128,
    /// Bit mask of coefficients whose zero status is unknown.
    pub coefficient_unknown_zero_mask: u128,
}

impl LiftedPolynomialFacts {
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
    /// topology crates because this type owns the retained coefficient facts.
    /// The test is any known zero with no unknown zeros; an all-zero polynomial
    /// also qualifies, so true does not establish a nondegenerate circle/sphere.
    pub fn has_sparse_coefficient_support(self) -> bool {
        self.coefficient_zero_count() > 0 && self.coefficient_unknown_zero_mask == 0
    }
}

/// Borrowed coefficient view for a retained lifted 2D circle polynomial.
///
/// Query evaluation uses `x_coeff*x + y_coeff*y + lift_coeff*(x^2+y^2) +
/// constant`, where the sign is interpreted by the in-circle convention.
/// No normalization, radius, center or nondegeneracy is established by this
/// view; its public references can also describe an arbitrary lifted quadratic.
#[derive(Clone, Copy, Debug)]
pub struct Circle2Polynomial<'a> {
    /// Coefficient multiplied by query `x`.
    pub x_coeff: &'a Real,
    /// Coefficient multiplied by query `y`.
    pub y_coeff: &'a Real,
    /// Coefficient multiplied by query `x^2 + y^2`.
    pub lift_coeff: &'a Real,
    /// Constant coefficient.
    pub constant: &'a Real,
}

/// Borrowed coefficient view for a retained lifted 3D sphere polynomial.
///
/// Query evaluation uses `x_coeff*x + y_coeff*y + z_coeff*z +
/// lift_coeff*(x^2+y^2+z^2) + constant`, with sign interpreted by the
/// in-sphere convention.
/// The view supplies no center, radius or nondegeneracy validation; public
/// references need not have come from sphere evidence.
#[derive(Clone, Copy, Debug)]
pub struct Sphere3Polynomial<'a> {
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

/// Derived exact-predicate evidence for one fixed oriented 2D line.
///
/// This value owns certified dyadic and exact-word filters plus fixed-input
/// scheduling facts, but not the line endpoints. Retaining it alongside the
/// endpoints also lets wide rational queries use their scalar-local conversion
/// schedules before the arbitrary-precision determinant fallback.
/// Reuse with changed or reordered endpoints is not checked. A filter may
/// decide without reading the supplied endpoint values, so source association
/// must be maintained by the caller. This is not a serialized proof receipt.
#[derive(Clone, Copy, Debug)]
pub struct Line2Orientation {
    /// Fixed-input scheduling summary; does not itself decide an orientation.
    facts: PredicateFacts,
    /// Optional certified dyadic filter compiled from the ordered endpoints.
    filter: Option<AffineDet2Filter>,
    /// Optional checked homogeneous i128 line coefficients.
    exact_word_filter: Option<AffineDet2ExactWordFilter>,
}

/// Reusable exact-word representation of one 2D line-orientation query.
///
/// The carrier only schedules a checked exact filter. Unsupported points keep
/// an empty schedule and enter the same arbitrary-precision/refinement cascade
/// when queried.
/// It does not retain or validate the source point; a stale retained query can
/// disagree with the current point on paths that consume its word coordinates.
#[derive(Clone, Copy, Debug)]
pub struct Line2OrientationQuery {
    /// Checked homogeneous query coordinates, absent when conversion declines.
    exact_word: Option<AffineDet2ExactWordQuery>,
}

impl Line2Orientation {
    /// Return fixed-coordinate scheduling facts for this line.
    pub const fn facts(&self) -> PredicateFacts {
        self.facts
    }
}

/// Derive reusable exact-predicate evidence for oriented line `from -> to`.
/// Includes coincident endpoints; constructing evidence does not validate a line.
pub fn line2_orientation(from: &Point2, to: &Point2) -> Line2Orientation {
    line2_orientation_with_facts(from, to, PredicateFacts::line2(from, to))
}

/// Derive reusable exact-word query evidence for one point.
/// Retain it only with that point's unchanged exact coordinates.
pub fn line2_orientation_query(point: &Point2) -> Line2OrientationQuery {
    Line2OrientationQuery {
        exact_word: AffineDet2ExactWordQuery::from_reals([&point.x, &point.y]),
    }
}

/// Derive line-orientation evidence from already-collected fixed-input facts.
///
/// Stores `facts` as supplied, without verifying them against the endpoints.
/// Both filters are independently derived from `from` and `to`. The caller
/// must keep the fact snapshot and future endpoint arguments consistent.
pub fn line2_orientation_with_facts(
    from: &Point2,
    to: &Point2,
    facts: PredicateFacts,
) -> Line2Orientation {
    let filter = AffineDet2Filter::from_reals([&from.x, &from.y], [&to.x, &to.y]);
    let exact_word_filter =
        AffineDet2ExactWordFilter::from_reals([&from.x, &from.y], [&to.x, &to.y]);
    Line2Orientation {
        facts,
        filter,
        exact_word_filter,
    }
}

/// Classify a point using retained line evidence and an explicit policy.
///
/// `orientation` must have been derived from the same ordered endpoints with
/// [`line2_orientation`] or [`line2_orientation_with_facts`].
/// This precondition is not checked; retained filters can decide before the
/// endpoint arguments are read. Maps the sign while preserving its receipt.
pub fn classify_point_line_with_orientation_and_policy(
    from: &Point2,
    to: &Point2,
    point: &Point2,
    orientation: &Line2Orientation,
    policy: PredicatePolicy,
) -> PredicateOutcome<LineSide> {
    map_outcome(
        orient2d_with_orientation_and_policy(from, to, point, orientation, policy),
        LineSide::from,
    )
}

/// Classify a point using retained evidence for both the fixed line and query.
///
/// `orientation` and `query` must have been derived from the same ordered line
/// endpoints and query point passed here. An unavailable or inconclusive query
/// filter enters the same rational/exact-expression resolver as
/// [`classify_point_line_with_orientation_and_policy`].
/// No endpoint or query association is validated. An absent retained word
/// query skips that word path rather than reconstructing it from `point`.
pub fn classify_point_line_with_orientation_and_query_and_policy(
    from: &Point2,
    to: &Point2,
    point: &Point2,
    orientation: &Line2Orientation,
    query: &Line2OrientationQuery,
    policy: PredicatePolicy,
) -> PredicateOutcome<LineSide> {
    map_outcome(
        orient2d_with_orientation_and_query_and_policy(from, to, point, orientation, query, policy),
        LineSide::from,
    )
}

/// Runs retained line orientation without a precompiled query; source matching
/// is a caller precondition, not checked here.
pub(crate) fn orient2d_with_orientation_and_policy(
    from: &Point2,
    to: &Point2,
    point: &Point2,
    orientation: &Line2Orientation,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    orient2d_with_retained_evidence(from, to, point, orientation, None, policy)
}

/// Runs retained line orientation with a matching precompiled query; neither
/// the line nor query association is revalidated.
pub(crate) fn orient2d_with_orientation_and_query_and_policy(
    from: &Point2,
    to: &Point2,
    point: &Point2,
    orientation: &Line2Orientation,
    query: &Line2OrientationQuery,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    orient2d_with_retained_evidence(from, to, point, orientation, Some(query), policy)
}

/// Tries retained dyadic and word filters, then current-source rational paths.
///
/// A supplied query's empty word schedule declines without a fresh conversion.
/// After the rational line filter and full rational determinant decline, uses
/// the Real orientation resolver. Stored facts are not consumed by this path;
/// stale filters/query data are not detected before an answer is published.
fn orient2d_with_retained_evidence(
    from: &Point2,
    to: &Point2,
    point: &Point2,
    orientation: &Line2Orientation,
    query: Option<&Line2OrientationQuery>,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    if let Some(sign) = orientation
        .filter
        .and_then(|filter| filter.sign([&point.x, &point.y]))
    {
        crate::trace_dispatch!(
            "hyperlimit",
            "line2_orientation",
            "certified-real-det2-filter"
        );
        return PredicateOutcome::decided(
            crate::real::map_real_sign(sign),
            Certainty::Exact,
            Escalation::Exact,
        );
    }
    if let Some(filter) = orientation.exact_word_filter
        && let Some(sign) = match query {
            Some(query) => query
                .exact_word
                .as_ref()
                .and_then(|query| filter.sign_query(query)),
            None => filter.sign([&point.x, &point.y]),
        }
    {
        crate::trace_dispatch!(
            "hyperlimit",
            "line2_orientation",
            "exact-word-homogeneous-det2"
        );
        return PredicateOutcome::decided(
            crate::real::map_real_sign(sign),
            Certainty::Exact,
            Escalation::Exact,
        );
    }
    if let Some(sign) = Real::certified_rational_line2_sign(
        [&from.x, &from.y],
        [&to.x, &to.y],
        [&point.x, &point.y],
    ) {
        crate::trace_dispatch!(
            "hyperlimit",
            "line2_orientation",
            "certified-rational-det2-filter"
        );
        return PredicateOutcome::decided(
            crate::real::map_real_sign(sign),
            Certainty::Exact,
            Escalation::Exact,
        );
    }
    if let Some(outcome) = exact_outcome(policy, ExactPredicateKernel::Orient2RationalDet2, || {
        super::exact::orient2d(from, to, point)
    }) {
        return outcome;
    }
    orient2d_real_expr(from, to, point, policy)
}

/// In-circle predicate for four 2D points with an explicit escalation policy.
///
/// Positive means `d` lies inside the oriented circumcircle through `a`, `b`,
/// and `c` when those three points are counter-clockwise. Reversing the
/// orientation of `a`, `b`, and `c` reverses the sign.
/// The raw determinant has rows a,b,c,d and columns `[x,y,x²+y²,1]`.
/// Noncollinearity is needed for the circle interpretation but is not checked;
/// degenerate inputs still evaluate the polynomial. Dyadic and rational paths
/// precede the Real resolver, which retains any approximate qualification.
pub fn incircle2d_with_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    if let Some(sign) =
        Real::certified_incircle2_sign([&a.x, &a.y], [&b.x, &b.y], [&c.x, &c.y], [&d.x, &d.y])
    {
        crate::trace_dispatch!(
            "hyperlimit",
            "incircle2d",
            "certified-real-incircle2d-filter"
        );
        return PredicateOutcome::decided(
            crate::real::map_real_sign(sign),
            Certainty::Exact,
            Escalation::Exact,
        );
    }

    if let Some(outcome) = exact_outcome(
        policy,
        ExactPredicateKernel::Incircle2RationalLiftedDet3,
        || super::exact::incircle2d(a, b, c, d),
    ) {
        return outcome;
    }
    incircle2d_real_expr(a, b, c, d, policy)
}

/// Resolves the six-term lifted determinant after translating by query `d`.
/// Uses an exact rational callback and no signed-term filter; the expression
/// retains the full polynomial even for degenerate defining points.
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

    // Pass the six-term lifted determinant as one product-sum so symbolic
    // fallback preserves its shape and exact rationals use one delayed reducer.
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
        || super::exact::incircle2d(a, b, c, d),
        RefinementNeed::RealRefinement,
    )
}

/// Reusable exact-predicate evidence for an oriented circle through three points.
///
/// Owns lifted polynomial coefficients and optional filters, not the source
/// points or a validated circle. Collinear/repeated sources are accepted.
/// Later queries must supply the same ordered sources; their association with
/// the retained polynomial/filter is not checked.
#[derive(Clone, Debug)]
pub struct Incircle2Evidence {
    /// Source-coordinate scheduling snapshot, separate from coefficient facts.
    facts: PredicateFacts,
    /// Optional certified dyadic filter for the three fixed source points.
    filter: Option<Incircle2Filter>,
    /// Optional rational linear-form filter for x, y, squared norm and one.
    rational_filter: Option<RationalLinearForm4Filter>,
    /// Retained coefficient representation and structural-zero summary.
    coefficient_facts: LiftedPolynomialFacts,
    /// Cofactor multiplying query x in the full lifted determinant.
    x_coeff: Real,
    /// Cofactor multiplying query y.
    y_coeff: Real,
    /// Cofactor multiplying the query's squared Euclidean norm.
    lift_coeff: Real,
    /// Cofactor of the query row's final one.
    constant: Real,
}

impl Incircle2Evidence {
    /// Return fixed-coordinate scheduling facts for the source points.
    pub const fn facts(&self) -> PredicateFacts {
        self.facts
    }

    /// Return structural facts for the retained lifted-circle polynomial.
    pub const fn coefficient_facts(&self) -> LiftedPolynomialFacts {
        self.coefficient_facts
    }

    /// Return borrowed retained coefficients for the lifted-circle polynomial.
    pub const fn polynomial(&self) -> Circle2Polynomial<'_> {
        Circle2Polynomial {
            x_coeff: &self.x_coeff,
            y_coeff: &self.y_coeff,
            lift_coeff: &self.lift_coeff,
            constant: &self.constant,
        }
    }
}

/// Derive reusable evidence for the oriented circumcircle through `a`, `b`, and `c`.
///
/// Expands the query row of the lifted determinant into four coefficients and
/// derives optional filters. No division, center/radius construction or
/// noncollinearity decision is required; existence of a circle is not certified.
pub fn incircle2_evidence(a: &Point2, b: &Point2, c: &Point2) -> Incircle2Evidence {
    crate::trace_dispatch!("hyperlimit", "incircle2_evidence", "derive");
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
    let coefficient_facts = lifted_polynomial_facts([&x_coeff, &y_coeff, &lift_coeff, &constant]);
    let filter = Incircle2Filter::from_reals([&a.x, &a.y], [&b.x, &b.y], [&c.x, &c.y]);
    let rational_filter =
        RationalLinearForm4Filter::from_reals([&x_coeff, &y_coeff, &lift_coeff, &constant]);

    Incircle2Evidence {
        facts: PredicateFacts::incircle2(a, b, c),
        filter,
        rational_filter,
        coefficient_facts,
        x_coeff,
        y_coeff,
        lift_coeff,
        constant,
    }
}

/// Test a point using retained circle evidence and an explicit policy.
///
/// `evidence` must have been derived from the same ordered source points with
/// [`incircle2_evidence`].
/// The association is not rechecked: retained filter/polynomial paths can
/// answer without consulting a,b,c. Tries the dyadic filter, rational retained
/// polynomial, source rational determinant, then the retained Real polynomial
/// resolver. All paths retain their local certainty, but none recovers source
/// provenance or validates a unique circle.
pub fn incircle2d_with_evidence_and_policy(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    evidence: &Incircle2Evidence,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    if let Some(sign) = evidence
        .filter
        .and_then(|filter| filter.sign([&point.x, &point.y]))
    {
        crate::trace_dispatch!(
            "hyperlimit",
            "incircle2_evidence",
            "certified-real-incircle2d-filter"
        );
        return PredicateOutcome::decided(
            crate::real::map_real_sign(sign),
            Certainty::Exact,
            Escalation::Exact,
        );
    }

    if let Some(outcome) = exact_rational_circle_polynomial_sign(point, evidence) {
        return outcome;
    }

    if let Some(outcome) = exact_outcome(
        policy,
        ExactPredicateKernel::Incircle2RationalLiftedDet3,
        || super::exact::incircle2d(a, b, c, point),
    ) {
        return outcome;
    }

    crate::trace_dispatch!("hyperlimit", "incircle2_evidence", "circle-polynomial");
    let x_term = mul(&evidence.x_coeff, &point.x);
    let y_term = mul(&evidence.y_coeff, &point.y);
    let lift = point2_lift(point);
    let lift_term = mul(&evidence.lift_coeff, &lift);
    let xy = add(&x_term, &y_term);
    let xyl = add(&xy, &lift_term);
    let det = add(&xyl, &evidence.constant);

    resolve_real_sign(
        &det,
        policy,
        || None,
        || super::exact::incircle2d(a, b, c, point),
        RefinementNeed::RealRefinement,
    )
}

/// In-sphere predicate for five 3D points with an explicit escalation policy.
///
/// Returns the raw determinant with rows a,b,c,d,e and columns
/// `[x,y,z,x²+y²+z²,1]`. For a nondegenerate defining tetrahedron, an interior
/// query has the same sign as [`orient3d_with_policy`], an exterior query the
/// opposite sign, and a sphere point gives Zero. In particular, the ordered
/// standard tetrahedron gives a negative interior sign. Reversing orientation
/// reverses the sign. Degeneracy is not rejected, so zero alone does not prove
/// a unique sphere; policy-qualified terminal results retain their certainty.
pub fn insphere3d_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    e: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    if let Some(sign) = Real::certified_insphere3_sign(
        [&a.x, &a.y, &a.z],
        [&b.x, &b.y, &b.z],
        [&c.x, &c.y, &c.z],
        [&d.x, &d.y, &d.z],
        [&e.x, &e.y, &e.z],
    ) {
        crate::trace_dispatch!(
            "hyperlimit",
            "insphere3d",
            "certified-real-insphere3d-filter"
        );
        return PredicateOutcome::decided(
            crate::real::map_real_sign(sign),
            Certainty::Exact,
            Escalation::Exact,
        );
    }

    if let Some(outcome) = exact_outcome(
        policy,
        ExactPredicateKernel::Insphere3RationalLiftedDet4,
        || super::exact::insphere3d(a, b, c, d, e),
    ) {
        return outcome;
    }
    insphere3d_real_expr(a, b, c, d, e, policy)
}

/// Resolves the query-translated lifted 4x4 determinant as exact Real arithmetic.
///
/// Forms xy minors, z cofactors and squared lifts, then a difference of two
/// sums. Supplies their signed-term filter and a rational kernel callback to
/// the resolver; no sphere nondegeneracy check is performed.
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
        || super::exact::insphere3d(a, b, c, d, e),
        RefinementNeed::RealRefinement,
    )
}

/// Reusable exact-predicate evidence for an oriented sphere through four points.
///
/// Stores lifted coefficients and an optional filter without owning or binding
/// the four source points. Degenerate tuples are accepted; this carrier does
/// not certify a sphere's existence. Rebuild for changed or reordered sources.
#[derive(Clone, Debug)]
pub struct Insphere3Evidence {
    /// Optional certified dyadic filter compiled from the ordered source tuple.
    filter: Option<Insphere3Filter>,
    /// Source-coordinate scheduling summary.
    facts: PredicateFacts,
    /// Coefficient representation and structural-zero snapshot.
    coefficient_facts: LiftedPolynomialFacts,
    /// Cofactor multiplying query x in the full lifted determinant.
    x_coeff: Real,
    /// Cofactor multiplying query y.
    y_coeff: Real,
    /// Cofactor multiplying query z.
    z_coeff: Real,
    /// Cofactor multiplying the query's squared Euclidean norm.
    lift_coeff: Real,
    /// Cofactor of the query row's final one.
    constant: Real,
}

impl Insphere3Evidence {
    /// Return fixed-coordinate scheduling facts for the source points.
    pub const fn facts(&self) -> PredicateFacts {
        self.facts
    }

    /// Return structural facts for the retained lifted-sphere polynomial.
    pub const fn coefficient_facts(&self) -> LiftedPolynomialFacts {
        self.coefficient_facts
    }

    /// Return borrowed retained coefficients for the lifted-sphere polynomial.
    pub const fn polynomial(&self) -> Sphere3Polynomial<'_> {
        Sphere3Polynomial {
            x_coeff: &self.x_coeff,
            y_coeff: &self.y_coeff,
            z_coeff: &self.z_coeff,
            lift_coeff: &self.lift_coeff,
            constant: &self.constant,
        }
    }
}

/// Derive reusable evidence for the oriented sphere through `a`, `b`, `c`, and `d`.
///
/// Expands the final query row of the 5x5 lifted determinant into five exact
/// coefficients. Does not divide, construct a center/radius or test affine
/// independence; degenerate sources retain their determinant polynomial.
pub fn insphere3_evidence(a: &Point3, b: &Point3, c: &Point3, d: &Point3) -> Insphere3Evidence {
    crate::trace_dispatch!("hyperlimit", "insphere3_evidence", "derive");
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
    let filter = Insphere3Filter::from_reals(
        [&a.x, &a.y, &a.z],
        [&b.x, &b.y, &b.z],
        [&c.x, &c.y, &c.z],
        [&d.x, &d.y, &d.z],
    );

    Insphere3Evidence {
        filter,
        facts: PredicateFacts::insphere3(a, b, c, d),
        coefficient_facts,
        x_coeff,
        y_coeff,
        z_coeff,
        lift_coeff,
        constant,
    }
}

/// Test a point using retained sphere evidence and an explicit policy.
///
/// `evidence` must have been derived from the same ordered source points with
/// [`insphere3_evidence`].
/// The association is not validated. Tries retained dyadic filtering and
/// rational polynomial evaluation before the source rational determinant and
/// retained Real polynomial resolver. Earlier paths can answer without reading
/// a,b,c,d; a stale evidence object therefore violates the input contract.
pub fn insphere3d_with_evidence_and_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    d: &Point3,
    point: &Point3,
    evidence: &Insphere3Evidence,
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    if let Some(sign) = evidence
        .filter
        .and_then(|filter| filter.sign([&point.x, &point.y, &point.z]))
    {
        crate::trace_dispatch!(
            "hyperlimit",
            "insphere3_evidence",
            "certified-real-insphere3d-filter"
        );
        return PredicateOutcome::decided(
            crate::real::map_real_sign(sign),
            Certainty::Exact,
            Escalation::Exact,
        );
    }

    if let Some(outcome) = exact_rational_sphere_polynomial_sign(point, evidence) {
        return outcome;
    }

    if let Some(outcome) = exact_outcome(
        policy,
        ExactPredicateKernel::Insphere3RationalLiftedDet4,
        || super::exact::insphere3d(a, b, c, d, point),
    ) {
        return outcome;
    }

    crate::trace_dispatch!("hyperlimit", "insphere3_evidence", "sphere-polynomial");
    let x_term = mul(&evidence.x_coeff, &point.x);
    let y_term = mul(&evidence.y_coeff, &point.y);
    let z_term = mul(&evidence.z_coeff, &point.z);
    let lift = point3_lift(point);
    let lift_term = mul(&evidence.lift_coeff, &lift);
    let xy = add(&x_term, &y_term);
    let xyz = add(&xy, &z_term);
    let xyzl = add(&xyz, &lift_term);
    let det = add(&xyzl, &evidence.constant);

    resolve_real_sign(
        &det,
        policy,
        || None,
        || super::exact::insphere3d(a, b, c, d, point),
        RefinementNeed::RealRefinement,
    )
}

/// Evaluates a retained circle polynomial when query and coefficients are rational.
///
/// First tries the retained four-term linear-form filter with Exact/Filter
/// receipt, then compares the exact signed product sum with Exact/Exact receipt.
/// `None` means a coordinate/coefficient lacks rational storage; no source
/// point association is checked and no final rational determinant is required.
#[inline]
fn exact_rational_circle_polynomial_sign(
    point: &Point2,
    evidence: &Incircle2Evidence,
) -> Option<PredicateOutcome<Sign>> {
    let x = point.x.exact_rational_ref()?;
    let y = point.y.exact_rational_ref()?;
    let x_coeff = evidence.x_coeff.exact_rational_ref()?;
    let y_coeff = evidence.y_coeff.exact_rational_ref()?;
    let lift_coeff = evidence.lift_coeff.exact_rational_ref()?;
    let constant = evidence.constant.exact_rational_ref()?;
    let lift = Rational::signed_product_sum([true; 2], [[x, x], [y, y]]);
    let one = Rational::one();

    if let Some(sign) = evidence.rational_filter.and_then(|filter| {
        RationalLinearForm4Query::from_rationals([x, y, &lift, &one])
            .and_then(|query| filter.sign(&query))
    }) {
        crate::trace_dispatch!(
            "hyperlimit",
            "incircle2_evidence",
            "rational-lifted-polynomial-filter"
        );
        return Some(PredicateOutcome::decided(
            crate::real::map_real_sign(sign),
            Certainty::Exact,
            Escalation::Filter,
        ));
    }

    let ordering = Rational::signed_product_sum_ordering(
        [true; 4],
        [
            [x_coeff, x],
            [y_coeff, y],
            [lift_coeff, &lift],
            [constant, &one],
        ],
    );
    crate::trace_dispatch!(
        "hyperlimit",
        "incircle2_evidence",
        "exact-rational-lifted-polynomial"
    );
    Some(PredicateOutcome::decided(
        sign_from_ordering(ordering),
        Certainty::Exact,
        Escalation::Exact,
    ))
}

/// Evaluates a rational retained sphere polynomial by a five-term product sum.
/// Returns Exact/Exact on success and declines nonrational storage. It consumes
/// only the query and retained coefficients, without checking source identity.
#[inline]
fn exact_rational_sphere_polynomial_sign(
    point: &Point3,
    evidence: &Insphere3Evidence,
) -> Option<PredicateOutcome<Sign>> {
    let x = point.x.exact_rational_ref()?;
    let y = point.y.exact_rational_ref()?;
    let z = point.z.exact_rational_ref()?;
    let x_coeff = evidence.x_coeff.exact_rational_ref()?;
    let y_coeff = evidence.y_coeff.exact_rational_ref()?;
    let z_coeff = evidence.z_coeff.exact_rational_ref()?;
    let lift_coeff = evidence.lift_coeff.exact_rational_ref()?;
    let constant = evidence.constant.exact_rational_ref()?;
    let lift = Rational::signed_product_sum([true; 3], [[x, x], [y, y], [z, z]]);
    let one = Rational::one();
    let ordering = Rational::signed_product_sum_ordering(
        [true; 5],
        [
            [x_coeff, x],
            [y_coeff, y],
            [z_coeff, z],
            [lift_coeff, &lift],
            [constant, &one],
        ],
    );
    crate::trace_dispatch!(
        "hyperlimit",
        "insphere3_evidence",
        "exact-rational-lifted-polynomial"
    );
    Some(PredicateOutcome::decided(
        sign_from_ordering(ordering),
        Certainty::Exact,
        Escalation::Exact,
    ))
}

/// Delegates borrowed exact addition to the predicate scalar adapter.
fn add(left: &Real, right: &Real) -> Real {
    add_ref(left, right)
}

/// Delegates borrowed exact subtraction to the predicate scalar adapter.
fn sub(left: &Real, right: &Real) -> Real {
    sub_ref(left, right)
}

/// Forms the additive inverse as `(value-value)-value` using the existing adapters.
fn neg(value: &Real) -> Real {
    sub(&sub(value, value), value)
}

/// Constructs the exact squared norm x²+y² for a planar lifted row.
fn point2_lift(point: &Point2) -> Real {
    add(&mul(&point.x, &point.x), &mul(&point.y, &point.y))
}

/// Constructs the exact squared norm `(x²+y²)+z²` for a spatial lifted row.
fn point3_lift(point: &Point3) -> Real {
    add(
        &add(&mul(&point.x, &point.x), &mul(&point.y, &point.y)),
        &mul(&point.z, &point.z),
    )
}

/// Constructs the determinant of rows a,b,c as one six-term signed product sum.
/// Retains scalar reduction opportunities; this constructs a value, not a sign receipt.
fn det3_refs(a: [&Real; 3], b: [&Real; 3], c: [&Real; 3]) -> Real {
    // Preserve this frequently rebuilt 3x3 determinant as a fixed six-term
    // product-sum so exact rationals use one delayed reduction.
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

/// Expands rows `[a0,a1,1]`, `[b0,b1,1]`, `[c0,c1,1]` without multiplying by ones.
fn det3_with_unit_col2(a0: &Real, a1: &Real, b0: &Real, b1: &Real, c0: &Real, c1: &Real) -> Real {
    Real::signed_product_sum(
        [true, false, false, true, true, false],
        [[a0, b1], [a0, c1], [a1, b0], [a1, c0], [b0, c1], [b1, c0]],
    )
}

/// Expands a 4x4 determinant along row a with alternating 3x3 cofactors.
/// Combines them in one product sum without scalar division or pivot decisions.
fn det4_refs(a: [&Real; 4], b: [&Real; 4], c: [&Real; 4], d: [&Real; 4]) -> Real {
    let minor0 = det3_refs([b[1], b[2], b[3]], [c[1], c[2], c[3]], [d[1], d[2], d[3]]);
    let minor1 = det3_refs([b[0], b[2], b[3]], [c[0], c[2], c[3]], [d[0], d[2], d[3]]);
    let minor2 = det3_refs([b[0], b[1], b[3]], [c[0], c[1], c[3]], [d[0], d[1], d[3]]);
    let minor3 = det3_refs([b[0], b[1], b[2]], [c[0], c[1], c[2]], [d[0], d[1], d[2]]);

    // Keep the fallback Laplace cofactor combination as one fixed product-sum
    // instead of materializing four products. Exact-rational retained
    // coefficients then use the same delayed normalization as `det3_refs`.
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

/// Evaluates the 4x4 determinant whose rows append one to a,b,c,d.
/// Subtracting d from the first three rows reduces it to a 3x3 determinant
/// with the same sign and value, without constructing the final unit column.
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

/// Delegates borrowed exact multiplication to the predicate scalar adapter.
fn mul(left: &Real, right: &Real) -> Real {
    mul_ref(left, right)
}

/// Calls the supplied rational kernel once and wraps success as Exact/Exact.
/// The policy and kernel-tag arguments are currently unused; this wrapper
/// neither validates the callback's proof nor records its kernel identity.
fn exact_outcome(
    _policy: PredicatePolicy,
    _kernel: ExactPredicateKernel,
    exact: impl FnOnce() -> Option<Sign>,
) -> Option<PredicateOutcome<Sign>> {
    exact().map(|sign| PredicateOutcome::decided(sign, Certainty::Exact, Escalation::Exact))
}

/// Summarizes planar source representation facts and point masks in input order.
/// Uses scalar/point fact owners, setting the kernel hint only when all
/// coordinates have rational storage. Requires N <= 128 for the masks.
fn fixed_point_facts_2<const N: usize>(
    points: [&Point2; N],
    kernel: ExactPredicateKernel,
) -> PredicateFacts {
    // Delegate scalar representation classification to `hyperreal` and retain
    // only the predicate-level summary, keeping denominator identity opaque
    // while carrying common-scale eligibility to exact-kernel selection.
    let facts = Real::exact_set_facts(points.iter().flat_map(|point| [&point.x, &point.y]));
    let point_masks = fixed_point_structure_masks_2(points);

    PredicateFacts {
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

/// Collects scalar-set and zero-status facts in retained coefficient order.
/// Requires N <= 128 (debug-asserted); this snapshot does not validate geometry.
fn lifted_polynomial_facts<const N: usize>(coefficients: [&Real; N]) -> LiftedPolynomialFacts {
    debug_assert!(N <= u128::BITS as usize);
    // Keep coefficient facts at the evidence boundary rather than
    // recomputing them in triangulation or CSG code; they select faster exact
    // arithmetic schedules without becoming topology certificates.
    let coefficient_exact = Real::exact_set_facts(coefficients);
    let (coefficient_zero_mask, coefficient_nonzero_mask, coefficient_unknown_zero_mask) =
        real_zero_masks(coefficients);

    LiftedPolynomialFacts {
        coefficient_exact,
        coefficient_zero_mask,
        coefficient_nonzero_mask,
        coefficient_unknown_zero_mask,
    }
}

/// Partitions coordinate bits into known-zero, known-nonzero and unknown masks.
/// Uses structural facts without policy decisions; requires N <= 128
/// (debug-asserted). Unknown is never treated as known nonzero.
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

/// Summarizes spatial source representations and ordered point masks.
/// The kernel hint requires rational storage for every coordinate; mask
/// construction requires N <= 128 and performs no affine-independence test.
fn fixed_point_facts_3<const N: usize>(
    points: [&Point3; N],
    kernel: ExactPredicateKernel,
) -> PredicateFacts {
    let facts = Real::exact_set_facts(
        points
            .iter()
            .flat_map(|point| [&point.x, &point.y, &point.z]),
    );
    let point_masks = fixed_point_structure_masks_3(points);

    PredicateFacts {
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

/// Temporary point-level summaries used to populate retained predicate facts.
/// Default has empty masks/dependencies; no geometric claim follows from it.
#[derive(Clone, Copy, Debug, Default)]
struct FixedPointStructureMasks {
    /// Points whose coordinates have a common reduced rational denominator.
    shared_scale: u128,
    /// Points structurally known to have all-zero coordinates.
    origin: u128,
    /// Points with one known-nonzero coordinate and all others known zero.
    one_hot: u128,
    /// Points with at least one coordinate of unknown zero status.
    unknown_zero: u128,
    /// Union of scalar dependency families, without individual source identities.
    symbolic_dependencies: SymbolicDependencyMask,
}

/// Collects planar point masks and dependency families with bit i for point i.
/// Requires N <= 128 (debug-asserted); point structural facts own each claim.
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

/// Collects spatial point masks and dependency families with bit i for point i.
/// Requires N <= 128 (debug-asserted); no predicate refinement is requested.
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

/// Sign conventions, retained-path comparisons, exact dispatch and finite generators.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::predicate::{Certainty, Escalation};
    use crate::predicates::exact;
    use core::cmp::Ordering;
    use hyperreal::Rational;
    use proptest::prelude::*;

    /// Allows a terminal while the rational fixtures still require exact results.
    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    /// Serializes this module's tests that reset and inspect shared dispatch traces.
    #[cfg(feature = "dispatch-trace")]
    fn dispatch_trace_test_lock() -> &'static std::sync::Mutex<()> {
        /// Lazily initialized process-local lock for this test module's trace fixtures.
        static LOCK: std::sync::OnceLock<std::sync::Mutex<()>> = std::sync::OnceLock::new();
        LOCK.get_or_init(|| std::sync::Mutex::new(()))
    }

    /// Imports a finite binary64 fixture exactly, not as its displayed decimal ratio.
    fn real(value: f64) -> Real {
        Real::try_from(value).expect("finite test Real")
    }

    /// Constructs a planar fixture from the exact binary values of finite inputs.
    fn p2(x: f64, y: f64) -> Point2 {
        Point2::new(real(x), real(y))
    }

    /// Constructs a spatial fixture from the exact binary values of finite inputs.
    fn p3(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(real(x), real(y), real(z))
    }

    /// Imports an integer fixture without floating-point conversion.
    fn rational(value: i32) -> Real {
        Real::from(value)
    }

    /// Constructs an exact integer planar fixture.
    fn rp2(x: i32, y: i32) -> Point2 {
        Point2::new(rational(x), rational(y))
    }

    /// Constructs an exact integer spatial fixture.
    fn rp3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(rational(x), rational(y), rational(z))
    }

    /// Multiplies integer coordinates by an exact scale for wide-rational tests.
    fn wide_rp3(scale: &Rational, x: i32, y: i32, z: i32) -> Point3 {
        let coordinate = |value| Real::from(scale * &Rational::new(i64::from(value)));
        Point3::new(coordinate(x), coordinate(y), coordinate(z))
    }

    #[test]
    /// Checks positive orientation of a counterclockwise binary-exact unit triangle.
    fn orient2d_classifies_simple_triangle() {
        let a = p2(0.0, 0.0);
        let b = p2(1.0, 0.0);
        let c = p2(0.0, 1.0);
        assert_eq!(
            orient2d_with_policy(&a, &b, &c, APPROX).value(),
            Some(Sign::Positive)
        );
    }

    #[test]
    /// Requires an Exact/Exact zero receipt for a strict collinear integer fixture.
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
    /// Checks Exact/Exact receipts for four rational orientation/lifted predicates.
    /// This asserts result metadata, not a measured total-work or refinement count.
    fn exact_rational_predicates_do_not_need_refinement_budget() {
        let policy = PredicatePolicy::STRICT;

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
    /// Checks the negative orientation convention for the ordered standard tetrahedron.
    fn orient3d_classifies_simple_tetrahedron() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(1.0, 0.0, 0.0);
        let c = p3(0.0, 1.0, 0.0);
        let d = p3(0.0, 0.0, 1.0);
        assert_eq!(
            orient3d_with_policy(&a, &b, &c, &d, APPROX).value(),
            Some(Sign::Negative)
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    /// Requires one word-filter hit and zero full rational-kernel hits for denominator five.
    fn orient2d_uses_exact_word_filter_for_common_denominators() {
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
        let outcome = hyperreal::dispatch_trace::with_recording(|| {
            orient2d_with_policy(&a, &b, &c, PredicatePolicy::STRICT)
        });

        assert_eq!(outcome.value(), Some(Sign::Positive));

        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count("hyperlimit", "orient2d", "exact-word-rational-det2-filter"),
            1
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient2d", "rational-det2"),
            0
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    /// Requires one word-filter hit and zero full rational-kernel hits for mixed point scales.
    fn orient2d_uses_exact_word_filter_for_mixed_denominators() {
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
        let outcome = hyperreal::dispatch_trace::with_recording(|| {
            orient2d_with_policy(&a, &b, &c, PredicatePolicy::STRICT)
        });

        assert_eq!(outcome.value(), Some(Sign::Positive));

        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count("hyperlimit", "orient2d", "exact-word-rational-det2-filter"),
            1
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient2d", "rational-det2"),
            0
        );
    }

    /// Builds the exact positive integer 8*(i64::MAX)², beyond signed i128 range.
    fn rational_beyond_i128() -> Rational {
        let word = Rational::new(i64::MAX);
        let square = &word * &word;
        &square * &Rational::new(8)
    }

    #[test]
    /// Checks word decline, rational-filter success and exact receipts under both policies.
    fn orient2d_certified_rational_filter_handles_values_beyond_word_kernel() {
        let zero = Real::zero();
        let wide = Real::from(rational_beyond_i128());
        let a = [&zero, &zero];
        let b = [&wide, &zero];
        let c = [&zero, &wide];

        assert_eq!(orient2d_exact_word_filter(a, b, c), None);
        assert_eq!(
            orient2d_certified_rational_filter(a, b, c),
            Some(Sign::Positive)
        );

        let a = Point2::new(zero.clone(), zero.clone());
        let b = Point2::new(wide.clone(), zero.clone());
        let c = Point2::new(zero, wide);
        for policy in [PredicatePolicy::STRICT, APPROX] {
            assert_eq!(
                orient2d_with_policy(&a, &b, &c, policy),
                PredicateOutcome::decided(Sign::Positive, Certainty::Exact, Escalation::Exact)
            );
        }
    }

    #[test]
    /// Checks both fast filters decline on a wide collinear case, retaining exact zero.
    fn orient2d_certified_rational_filter_falls_through_on_exact_boundary() {
        let zero = Real::zero();
        let wide = rational_beyond_i128();
        let two_wide = &wide * &Rational::new(2);
        let four_wide = &wide * &Rational::new(4);
        let wide = Real::from(wide);
        let two_wide = Real::from(two_wide);
        let four_wide = Real::from(four_wide);
        let a = [&zero, &zero];
        let b = [&wide, &two_wide];
        let c = [&two_wide, &four_wide];

        assert_eq!(orient2d_exact_word_filter(a, b, c), None);
        assert_eq!(orient2d_certified_rational_filter(a, b, c), None);

        let a = Point2::new(zero.clone(), zero);
        let b = Point2::new(wide, two_wide.clone());
        let c = Point2::new(two_wide, four_wide);
        for policy in [PredicatePolicy::STRICT, APPROX] {
            assert_eq!(
                orient2d_with_policy(&a, &b, &c, policy),
                PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::Exact)
            );
        }
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    /// Requires the negative spatial sign, one word hit and zero full rational hits.
    fn orient3d_uses_exact_word_filter_for_common_denominators() {
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
        let outcome = hyperreal::dispatch_trace::with_recording(|| {
            orient3d_with_policy(&a, &b, &c, &d, PredicatePolicy::STRICT)
        });

        assert_eq!(outcome.value(), Some(Sign::Negative));

        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count("hyperlimit", "orient3d", "exact-word-rational-det3-filter"),
            1
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient3d", "rational-det3"),
            0
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    /// Requires the mixed-scale spatial sign, one word hit and zero full rational hits.
    fn orient3d_uses_exact_word_filter_for_mixed_denominators() {
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
        let outcome = hyperreal::dispatch_trace::with_recording(|| {
            orient3d_with_policy(&a, &b, &c, &d, PredicatePolicy::STRICT)
        });

        assert_eq!(outcome.value(), Some(Sign::Positive));

        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count("hyperlimit", "orient3d", "exact-word-rational-det3-filter"),
            1
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient3d", "rational-det3"),
            0
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    /// Checks positive orientation with zero word hits and one full rational-kernel hit.
    fn orient3d_word_overflow_uses_compact_rational_fallback() {
        let _trace_lock = dispatch_trace_test_lock()
            .lock()
            .expect("dispatch trace test lock poisoned");
        let zero = Real::zero();
        let large = Real::from(i64::MAX);
        let a = Point3::new(large.clone(), zero.clone(), zero.clone());
        let b = Point3::new(zero.clone(), large.clone(), zero.clone());
        let c = Point3::new(zero.clone(), zero.clone(), large);
        let d = Point3::new(zero.clone(), zero.clone(), zero);

        hyperreal::dispatch_trace::reset();
        let outcome = hyperreal::dispatch_trace::with_recording(|| {
            orient3d_with_policy(&a, &b, &c, &d, PredicatePolicy::STRICT)
        });

        assert_eq!(outcome.value(), Some(Sign::Positive));

        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count("hyperlimit", "orient3d", "exact-word-rational-det3-filter"),
            0
        );
        assert_eq!(
            trace.path_count("hyperlimit", "exact_orient3d", "rational-det3"),
            1
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    /// Checks positive circle sign, one direct rational hit and one retained polynomial-filter hit.
    fn incircle2d_uses_compact_rational_kernel_for_exact_inputs() {
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
        let outcome = hyperreal::dispatch_trace::with_recording(|| {
            incircle2d_with_policy(&a, &b, &c, &d, PredicatePolicy::STRICT)
        });

        assert_eq!(outcome.value(), Some(Sign::Positive));

        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count("hyperlimit", "exact_incircle2d", "rational-det3-lifted"),
            1
        );

        let evidence = incircle2_evidence(&a, &b, &c);
        hyperreal::dispatch_trace::reset();
        let evidence_outcome = hyperreal::dispatch_trace::with_recording(|| {
            incircle2d_with_evidence_and_policy(&a, &b, &c, &d, &evidence, PredicatePolicy::STRICT)
        });
        assert_eq!(evidence_outcome.value(), Some(Sign::Positive));
        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count(
                "hyperlimit",
                "incircle2_evidence",
                "rational-lifted-polynomial-filter"
            ),
            1
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    /// Checks negative interior sign and one hit each for direct and retained rational paths.
    fn insphere3d_uses_compact_rational_kernel_for_exact_inputs() {
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
        let outcome = hyperreal::dispatch_trace::with_recording(|| {
            insphere3d_with_policy(&a, &b, &c, &d, &e, PredicatePolicy::STRICT)
        });

        assert_eq!(outcome.value(), Some(Sign::Negative));

        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count("hyperlimit", "exact_insphere3d", "rational-det4-lifted"),
            1
        );

        let evidence = insphere3_evidence(&a, &b, &c, &d);
        hyperreal::dispatch_trace::reset();
        let evidence_outcome = hyperreal::dispatch_trace::with_recording(|| {
            insphere3d_with_evidence_and_policy(
                &a,
                &b,
                &c,
                &d,
                &e,
                &evidence,
                PredicatePolicy::STRICT,
            )
        });
        assert_eq!(evidence_outcome.value(), Some(Sign::Negative));
        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count(
                "hyperlimit",
                "insphere3_evidence",
                "exact-rational-lifted-polynomial"
            ),
            1
        );
    }

    #[test]
    /// Compares retained/direct side values and selected facts on three dyadic queries.
    /// This shares predicate machinery rather than supplying an independent geometry oracle.
    fn retained_line_orientation_matches_orient2d_side() {
        let a = p2(-1.0, -1.0);
        let b = p2(1.0, 1.0);
        let orientation = line2_orientation(&a, &b);
        assert_eq!(
            orientation.facts().exact_kernel_hint,
            Some(ExactPredicateKernel::Orient2RationalDet2)
        );
        assert!(orientation.facts().fixed_coordinates_shared_denominator);
        for point in [p2(-0.75, -0.5), p2(0.5, 0.25), p2(0.125, 0.125)] {
            assert_eq!(
                crate::classify_point_line_with_orientation(&a, &b, &point, &orientation, APPROX)
                    .value(),
                crate::classify_point_line(&a, &b, &point, APPROX).value()
            );
        }
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    /// Checks a wide-rational Left result, one retained rational-filter hit and both policies.
    fn retained_line_orientation_certifies_wide_rational_queries() {
        let _trace_lock = dispatch_trace_test_lock()
            .lock()
            .expect("dispatch trace test lock poisoned");
        let scale: Rational =
            "100000000000000000000000000000000000000000000000000000000000000000000000000000001"
                .parse()
                .expect("wide exact integer");
        let scaled = |factor| Real::from(&scale * Rational::new(factor));
        let a = Point2::new(scaled(1), scaled(2));
        let b = Point2::new(scaled(4), scaled(3));
        let point = Point2::new(scaled(2), scaled(5));
        let orientation = line2_orientation(&a, &b);

        hyperreal::dispatch_trace::reset();
        let strict = hyperreal::dispatch_trace::with_recording(|| {
            crate::classify_point_line_with_orientation(
                &a,
                &b,
                &point,
                &orientation,
                PredicatePolicy::STRICT,
            )
        });
        assert_eq!(strict.value(), Some(LineSide::Left));
        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count(
                "hyperlimit",
                "line2_orientation",
                "certified-rational-det2-filter"
            ),
            1
        );
        assert_eq!(
            crate::classify_point_line_with_orientation(&a, &b, &point, &orientation, APPROX)
                .value(),
            Some(LineSide::Left)
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    /// Requires one hit each for retained dyadic, word and rational filters on three Left queries.
    fn retained_line_orientation_cascades_by_query_representation() {
        let _trace_lock = dispatch_trace_test_lock()
            .lock()
            .expect("dispatch trace test lock poisoned");
        let a = rp2(0, 0);
        let b = rp2(4, 1);
        let dyadic = rp2(1, 2);
        let word = Point2::new(
            Real::from(Rational::fraction(1, 3).unwrap()),
            Real::from(Rational::fraction(2, 7).unwrap()),
        );
        let scale: Rational =
            "100000000000000000000000000000000000000000000000000000000000000000000000000000001"
                .parse()
                .expect("wide exact integer");
        let wide = Point2::new(
            Real::from(scale.clone()),
            Real::from(&scale * Rational::new(5)),
        );
        let orientation = line2_orientation(&a, &b);

        hyperreal::dispatch_trace::reset();
        let sides = hyperreal::dispatch_trace::with_recording(|| {
            [&dyadic, &word, &wide].map(|point| {
                crate::classify_point_line_with_orientation(
                    &a,
                    &b,
                    point,
                    &orientation,
                    PredicatePolicy::STRICT,
                )
                .value()
            })
        });
        assert_eq!(sides, [Some(LineSide::Left); 3]);
        let trace = hyperreal::dispatch_trace::take_trace();
        for path in [
            "certified-real-det2-filter",
            "exact-word-homogeneous-det2",
            "certified-rational-det2-filter",
        ] {
            assert_eq!(trace.path_count("hyperlimit", "line2_orientation", path), 1);
        }
    }

    #[test]
    /// Checks point-local shared scales without a shared denominator across both points.
    fn line_orientation_facts_distinguish_mixed_denominators() {
        let a = Point2::new(
            Real::from(Rational::fraction(1, 3).unwrap()),
            Real::from(Rational::fraction(2, 3).unwrap()),
        );
        let b = Point2::new(
            Real::from(Rational::fraction(1, 5).unwrap()),
            Real::from(Rational::fraction(2, 5).unwrap()),
        );
        let orientation = line2_orientation(&a, &b);

        assert!(orientation.facts().fixed_coordinates_exact_rational);
        assert!(!orientation.facts().fixed_coordinates_dyadic);
        assert!(!orientation.facts().fixed_coordinates_shared_denominator);
        assert_eq!(orientation.facts().fixed_point_shared_scale_mask, 0b11);
    }

    #[test]
    /// Checks global and per-point denominator summaries for line, circle and sphere sources.
    fn line_orientation_facts_detect_common_reduced_denominator() {
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
            line2_orientation(&a, &b)
                .facts()
                .fixed_coordinates_shared_denominator
        );
        assert_eq!(
            line2_orientation(&a, &b)
                .facts()
                .fixed_point_shared_scale_mask,
            0b11
        );
        assert!(
            incircle2_evidence(&a, &b, &c)
                .facts()
                .fixed_coordinates_shared_denominator
        );
        assert_eq!(
            incircle2_evidence(&a, &b, &c)
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
            insphere3_evidence(&p, &q, &r, &s)
                .facts()
                .fixed_coordinates_shared_denominator
        );
        assert_eq!(
            insphere3_evidence(&p, &q, &r, &s)
                .facts()
                .fixed_point_shared_scale_mask,
            0b1111
        );
    }

    #[test]
    /// Recomputes coefficient summaries and compares polynomial/evidence signs on two fixtures.
    /// The comparisons share scalar and fact helpers and do not prove general equivalence.
    fn lifted_polynomial_facts_match_retained_coefficients() {
        let a = rp2(1, 0);
        let b = rp2(0, 1);
        let c = rp2(-1, 0);
        let evidence = incircle2_evidence(&a, &b, &c);
        let poly = evidence.polynomial();

        assert_eq!(
            evidence.coefficient_facts(),
            lifted_polynomial_facts([poly.x_coeff, poly.y_coeff, poly.lift_coeff, poly.constant])
        );
        assert!(
            evidence
                .coefficient_facts()
                .coefficient_exact
                .all_exact_rational
        );
        assert!(evidence.coefficient_facts().has_dyadic_schedule());
        assert_eq!(
            evidence
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
            incircle2d_with_evidence_and_policy(&a, &b, &c, &query, &evidence, APPROX,).value()
        );

        let p = rp3(0, 0, 0);
        let q = rp3(1, 0, 0);
        let r = rp3(0, 1, 0);
        let s = rp3(0, 0, 1);
        let sphere = insphere3_evidence(&p, &q, &r, &s);
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
            insphere3d_with_evidence_and_policy(&p, &q, &r, &s, &sphere_query, &sphere, APPROX,)
                .value()
        );
    }

    #[test]
    /// Compares retained/direct circle signs for interior, exterior and source-point queries.
    /// Decimal literals enter as their exact binary64 values; receipts are not compared.
    fn incircle_evidence_matches_incircle2d_sign() {
        let a = p2(0.82, 0.0);
        let b = p2(0.0, 0.82);
        let c = p2(-0.82, 0.0);
        let evidence = incircle2_evidence(&a, &b, &c);
        assert_eq!(
            evidence.facts().exact_kernel_hint,
            Some(ExactPredicateKernel::Incircle2RationalLiftedDet3)
        );
        for point in [p2(0.2, 0.1), p2(0.95, 0.0), p2(0.82, 0.0)] {
            assert_eq!(
                incircle2d_with_evidence_and_policy(&a, &b, &c, &point, &evidence, APPROX,).value(),
                incircle2d_with_policy(&a, &b, &c, &point, APPROX,).value()
            );
        }
    }

    #[test]
    /// Compares retained/direct sphere signs for interior, exterior and source-point queries.
    /// The binary64 fixture imports are exact; only returned sign values are compared.
    fn insphere_evidence_matches_insphere3d_sign() {
        let a = p3(0.82, 0.0, 0.0);
        let b = p3(-0.82, 0.0, 0.0);
        let c = p3(0.0, 0.82, 0.0);
        let d = p3(0.0, 0.0, 0.82);
        let evidence = insphere3_evidence(&a, &b, &c, &d);
        assert_eq!(
            evidence.facts().exact_kernel_hint,
            Some(ExactPredicateKernel::Insphere3RationalLiftedDet4)
        );
        for point in [p3(0.1, 0.1, 0.1), p3(1.1, 0.0, 0.0), p3(0.82, 0.0, 0.0)] {
            assert_eq!(
                insphere3d_with_evidence_and_policy(&a, &b, &c, &d, &point, &evidence, APPROX,)
                    .value(),
                insphere3d_with_policy(&a, &b, &c, &d, &point, APPROX,).value()
            );
        }
    }

    #[test]
    /// Compares forced Real-expression and ordinary paths on finite rational sign controls.
    fn generic_real_lifted_determinants_match_exact_rational_kernels() {
        let a = rp2(0, 0);
        let b = rp2(2, 0);
        let c = rp2(0, 2);
        for query in [rp2(1, 1), rp2(2, 2), rp2(3, 3)] {
            assert_eq!(
                incircle2d_real_expr(&a, &b, &c, &query, PredicatePolicy::STRICT).value(),
                incircle2d_with_policy(&a, &b, &c, &query, PredicatePolicy::STRICT).value()
            );
        }

        let p = rp3(0, 0, 0);
        let q = rp3(1, 0, 0);
        let r = rp3(0, 1, 0);
        let s = rp3(0, 0, 1);
        for query in [
            Point3::new(
                Real::from(Rational::fraction(1, 4).unwrap()),
                Real::from(Rational::fraction(1, 4).unwrap()),
                Real::from(Rational::fraction(1, 4).unwrap()),
            ),
            rp3(0, 0, 0),
            rp3(2, 2, 2),
        ] {
            assert_eq!(
                insphere3d_real_expr(&p, &q, &r, &s, &query, PredicatePolicy::STRICT).value(),
                insphere3d_with_policy(&p, &q, &r, &s, &query, PredicatePolicy::STRICT).value()
            );
        }

        assert_eq!(
            orient2d_real_expr(&a, &b, &c, PredicatePolicy::STRICT).value(),
            orient2d_with_policy(&a, &b, &c, PredicatePolicy::STRICT).value()
        );
    }

    #[test]
    /// Checks retained query results, symbolic fallback signs and coefficient-mask partitions.
    /// Disabling filters selects internal paths; it does not validate arbitrary evidence mutation.
    fn retained_query_and_polynomial_fallbacks_match_scalar_predicates() {
        let from = rp2(0, 0);
        let to = rp2(2, 0);
        let point = rp2(1, 1);
        let orientation = line2_orientation(&from, &to);
        let query = line2_orientation_query(&point);
        assert_eq!(
            classify_point_line_with_orientation_and_query_and_policy(
                &from,
                &to,
                &point,
                &orientation,
                &query,
                PredicatePolicy::STRICT,
            ),
            classify_point_line_with_policy(&from, &to, &point, PredicatePolicy::STRICT)
        );

        let symbolic_point = Point2::new(Real::pi(), Real::from(1));
        assert_eq!(
            crate::classify_point_line_with_orientation(
                &from,
                &to,
                &symbolic_point,
                &orientation,
                APPROX,
            ),
            classify_point_line_with_policy(&from, &to, &symbolic_point, APPROX)
        );

        let a = rp2(1, 0);
        let b = rp2(0, 1);
        let c = rp2(-1, 0);
        let symbolic_query = Point2::new(Real::pi(), Real::from(0));
        let mut circle = incircle2_evidence(&a, &b, &c);
        circle.filter = None;
        circle.rational_filter = None;
        assert_eq!(
            incircle2d_with_evidence_and_policy(&a, &b, &c, &symbolic_query, &circle, APPROX,)
                .value(),
            incircle2d_with_policy(&a, &b, &c, &symbolic_query, APPROX).value()
        );

        let p = rp3(0, 0, 0);
        let q = rp3(1, 0, 0);
        let r = rp3(0, 1, 0);
        let s = rp3(0, 0, 1);
        let symbolic_query = Point3::new(Real::pi(), Real::from(0), Real::from(0));
        let mut sphere = insphere3_evidence(&p, &q, &r, &s);
        sphere.filter = None;
        assert_eq!(
            insphere3d_with_evidence_and_policy(&p, &q, &r, &s, &symbolic_query, &sphere, APPROX,)
                .value(),
            insphere3d_with_policy(&p, &q, &r, &s, &symbolic_query, APPROX).value()
        );

        let circle_facts = circle.coefficient_facts();
        assert_eq!(
            circle_facts.coefficient_zero_count()
                + circle_facts.coefficient_nonzero_count()
                + circle_facts.coefficient_unknown_zero_count(),
            4
        );
        assert!(circle_facts.has_shared_denominator_schedule());
        assert_eq!(
            circle_facts.has_sparse_coefficient_support(),
            circle_facts.coefficient_zero_count() > 0
        );

        let sphere_facts = sphere.coefficient_facts();
        assert_eq!(
            sphere_facts.coefficient_zero_count()
                + sphere_facts.coefficient_nonzero_count()
                + sphere_facts.coefficient_unknown_zero_count(),
            5
        );
        assert!(sphere_facts.has_shared_denominator_schedule());
        assert_eq!(
            sphere_facts.has_sparse_coefficient_support(),
            sphere_facts.coefficient_zero_count() > 0
        );
    }

    #[test]
    /// Disables filters and makes selected coefficients nonrational to reach source callbacks.
    /// Exact source-boundary receipts test branch reachability, not stale-evidence safety.
    fn retained_exact_callbacks_cover_filter_decline_boundaries() {
        let from = rp2(0, 0);
        let to = rp2(2, 0);
        let boundary = rp2(1, 0);
        let mut orientation = line2_orientation(&from, &to);
        orientation.filter = None;
        orientation.exact_word_filter = None;
        assert_eq!(
            orient2d_with_retained_evidence(
                &from,
                &to,
                &boundary,
                &orientation,
                None,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::Exact)
        );

        let a = rp2(1, 0);
        let b = rp2(0, 1);
        let c = rp2(-1, 0);
        let mut circle = incircle2_evidence(&a, &b, &c);
        circle.filter = None;
        circle.rational_filter = None;
        circle.x_coeff = Real::pi();
        assert_eq!(
            incircle2d_with_evidence_and_policy(&a, &b, &c, &a, &circle, PredicatePolicy::STRICT,),
            PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::Exact)
        );

        let p = rp3(0, 0, 0);
        let q = rp3(1, 0, 0);
        let r = rp3(0, 1, 0);
        let s = rp3(0, 0, 1);
        let mut sphere = insphere3_evidence(&p, &q, &r, &s);
        sphere.filter = None;
        sphere.x_coeff = Real::pi();
        assert_eq!(
            insphere3d_with_evidence_and_policy(
                &p,
                &q,
                &r,
                &s,
                &p,
                &sphere,
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::Exact)
        );

        assert_eq!(sign_from_ordering(Ordering::Greater), Sign::Positive);
    }

    // Each bounded generator below requests 256 cases; this finite corpus is
    // evidence for its assertions, not a universal determinant-closure proof.
    proptest! {
        #![proptest_config(ProptestConfig::with_cases(256))]

        #[test]
        /// Compares complete strict outcomes before/after bounded integer planar translations.
        fn exact_orient2d_is_translation_invariant(
            ax in -64_i32..64, ay in -64_i32..64,
            bx in -64_i32..64, by in -64_i32..64,
            cx in -64_i32..64, cy in -64_i32..64,
            tx in -64_i32..64, ty in -64_i32..64,
        ) {
            let policy = PredicatePolicy::STRICT;
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
        /// Compares complete strict outcomes before/after bounded integer spatial translations.
        fn exact_orient3d_is_translation_invariant(
            ax in -16_i32..16, ay in -16_i32..16, az in -16_i32..16,
            bx in -16_i32..16, by in -16_i32..16, bz in -16_i32..16,
            cx in -16_i32..16, cy in -16_i32..16, cz in -16_i32..16,
            dx in -16_i32..16, dy in -16_i32..16, dz in -16_i32..16,
            tx in -16_i32..16, ty in -16_i32..16, tz in -16_i32..16,
        ) {
            let policy = PredicatePolicy::STRICT;
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
        /// Uses an independent i128 determinant sign before positive wide scaling of all coordinates.
        fn sign_only_exact_orient3d_matches_wide_integer_determinants(
            ax in -16_i32..16, ay in -16_i32..16, az in -16_i32..16,
            bx in -16_i32..16, by in -16_i32..16, bz in -16_i32..16,
            cx in -16_i32..16, cy in -16_i32..16, cz in -16_i32..16,
            dx in -16_i32..16, dy in -16_i32..16, dz in -16_i32..16,
        ) {
            let adx = i128::from(ax - dx);
            let ady = i128::from(ay - dy);
            let adz = i128::from(az - dz);
            let bdx = i128::from(bx - dx);
            let bdy = i128::from(by - dy);
            let bdz = i128::from(bz - dz);
            let cdx = i128::from(cx - dx);
            let cdy = i128::from(cy - dy);
            let cdz = i128::from(cz - dz);
            let determinant = adx * (bdy * cdz - bdz * cdy)
                - ady * (bdx * cdz - bdz * cdx)
                + adz * (bdx * cdy - bdy * cdx);
            let expected = match determinant.cmp(&0) {
                Ordering::Less => Sign::Negative,
                Ordering::Equal => Sign::Zero,
                Ordering::Greater => Sign::Positive,
            };
            let scale = rational_beyond_i128();
            let a = wide_rp3(&scale, ax, ay, az);
            let b = wide_rp3(&scale, bx, by, bz);
            let c = wide_rp3(&scale, cx, cy, cz);
            let d = wide_rp3(&scale, dx, dy, dz);

            prop_assert_eq!(exact::orient3d(&a, &b, &c, &d), Some(expected));
        }

        #[test]
        /// Requires exact zero when the query repeats a source row, including degenerate triples.
        /// The determinant identity alone does not assert a valid circle.
        fn exact_incircle_reports_boundary_for_input_site(
            ax in -16_i32..16, ay in -16_i32..16,
            bx in -16_i32..16, by in -16_i32..16,
            cx in -16_i32..16, cy in -16_i32..16,
        ) {
            let policy = PredicatePolicy::STRICT;
            let a = rp2(ax, ay);
            let b = rp2(bx, by);
            let c = rp2(cx, cy);

            prop_assert_eq!(
                incircle2d_with_policy(&a, &b, &c, &a, policy),
                PredicateOutcome::decided(Sign::Zero, Certainty::Exact, Escalation::Exact)
            );
        }

        #[test]
        /// Requires exact zero for a repeated query/source row, including degenerate tetrahedra.
        /// The finite generator does not establish existence of a sphere.
        fn exact_insphere_reports_boundary_for_input_site(
            ax in -8_i32..8, ay in -8_i32..8, az in -8_i32..8,
            bx in -8_i32..8, by in -8_i32..8, bz in -8_i32..8,
            cx in -8_i32..8, cy in -8_i32..8, cz in -8_i32..8,
            dx in -8_i32..8, dy in -8_i32..8, dz in -8_i32..8,
        ) {
            let policy = PredicatePolicy::STRICT;
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
