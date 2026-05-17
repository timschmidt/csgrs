//! Plane classification helpers.

use hyperreal::{Real, RealExactSetFacts, ZeroKnowledge};

use crate::RealSymbolicDependencyMask;
use crate::classify::PlaneSide;
use crate::geometry::Point3;
use crate::predicate::{PredicateOutcome, PredicatePolicy, RefinementNeed, Sign};
use crate::predicates::orient3d_with_policy;
use crate::real::{add_ref, mul_ref, sub_ref};
use crate::resolve::{map_outcome, resolve_real_sign, signed_term_filter};

pub use crate::batch::{
    Orient3dCase, PointPlaneCase, classify_point_oriented_plane_batch,
    classify_point_oriented_plane_batch_with_policy, classify_point_plane_batch,
    classify_point_plane_batch_with_policy,
};
#[cfg(feature = "parallel")]
pub use crate::batch::{
    classify_point_oriented_plane_batch_parallel,
    classify_point_oriented_plane_batch_parallel_with_policy, classify_point_plane_batch_parallel,
    classify_point_plane_batch_parallel_with_policy,
};

/// Plane represented by `normal . point + offset = 0`.
#[derive(Clone, Debug, PartialEq)]
pub struct Plane3 {
    /// Plane normal vector.
    pub normal: Point3,
    /// Constant offset in `normal . point + offset = 0`.
    pub offset: Real,
}

/// Cheap structural facts for a [`Plane3`].
///
/// The facts are conservative scheduling metadata for repeated point-plane
/// classification. They record exact-set and sparse-support signals for the
/// coefficients of `normal . point + offset = 0`, but they do not decide which
/// side a query point lies on. That boundary follows Yap's exact geometric
/// computation model: preserve object structure at the geometric-object layer,
/// then use certified predicates for topology. See Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997). Sparse
/// coefficient support is the same retained-structure idea used by classical
/// sparse linear algebra schedules such as Gustavson, "Two Fast Algorithms for
/// Sparse Matrices: Multiplication and Permuted Transposition," *ACM
/// Transactions on Mathematical Software* 4.3 (1978).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Plane3Facts {
    /// Exact-rational representation facts for normal coordinates plus offset.
    pub coefficient_exact: RealExactSetFacts,
    /// Union of scalar symbolic dependency families for normal coordinates plus offset.
    ///
    /// Prepared plane queries can carry this scheduling fact next to exact-set
    /// and sparse coefficient facts without inspecting `Real` internals. It is
    /// not a side classification certificate; point-plane sidedness still comes
    /// from exact sign resolution. This follows Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997).
    pub coefficient_symbolic_dependencies: RealSymbolicDependencyMask,
    /// Structural facts for the normal vector.
    pub normal: crate::geometry::Point3Facts,
    /// Bit mask of coefficients known to be exactly zero.
    ///
    /// Bits 0, 1, and 2 correspond to `normal.x`, `normal.y`, and `normal.z`;
    /// bit 3 corresponds to `offset`.
    pub coefficient_zero_mask: u8,
    /// Bit mask of coefficients known to be nonzero.
    pub coefficient_nonzero_mask: u8,
    /// Bit mask of coefficients whose zero status is unknown.
    pub coefficient_unknown_zero_mask: u8,
}

impl Plane3Facts {
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

    /// Returns whether the plane normal is structurally known to be zero.
    ///
    /// A zero normal is usually invalid domain geometry for oriented topology,
    /// but this fact remains advisory: callers must decide domain policy above
    /// the predicate layer.
    pub fn normal_known_zero(self) -> bool {
        self.normal.known_zero
    }

    /// Returns whether the nonzero normal support is certified sparse.
    pub fn normal_has_sparse_support(self) -> bool {
        self.normal.has_sparse_support()
    }

    /// Returns whether all coefficients share one exact denominator.
    pub fn has_shared_denominator_schedule(self) -> bool {
        self.coefficient_exact.has_shared_denominator_schedule()
    }

    /// Returns whether all coefficients are exact dyadics.
    pub fn has_dyadic_schedule(self) -> bool {
        self.coefficient_exact.has_dyadic_schedule()
    }
}

impl Plane3 {
    /// Construct a plane from a normal vector and offset.
    pub const fn new(normal: Point3, offset: Real) -> Self {
        Self { normal, offset }
    }

    /// Return structural facts for this plane's coefficients.
    pub fn structural_facts(&self) -> Plane3Facts {
        plane3_facts(self)
    }

    /// Prepare this plane for repeated point classification.
    pub fn prepare(&self) -> PreparedPlane3<'_> {
        PreparedPlane3::new(self)
    }
}

/// Reusable point-plane classifier for a fixed plane.
#[derive(Clone, Copy, Debug)]
pub struct PreparedPlane3<'a> {
    plane: &'a Plane3,
    facts: Plane3Facts,
}

impl<'a> PreparedPlane3<'a> {
    /// Prepare a plane for repeated point classification.
    pub fn new(plane: &'a Plane3) -> Self {
        crate::trace_dispatch!("hyperlimit", "prepared_plane3", "new");
        Self {
            plane,
            facts: plane.structural_facts(),
        }
    }

    /// Return the borrowed plane.
    pub fn plane(&self) -> &'a Plane3 {
        self.plane
    }

    /// Return cached structural facts for this prepared plane.
    pub fn facts(&self) -> Plane3Facts {
        self.facts
    }

    /// Classify a point using the default predicate policy.
    pub fn classify_point(&self, point: &Point3) -> PredicateOutcome<PlaneSide> {
        self.classify_point_with_policy(point, PredicatePolicy::default())
    }

    /// Classify a point using an explicit predicate policy.
    pub fn classify_point_with_policy(
        &self,
        point: &Point3,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<PlaneSide> {
        classify_point_plane_prepared(point, self.plane, self.facts, policy)
    }
}

/// Reusable oriented-plane classifier for a fixed triangle plane.
///
/// This reduces the oriented plane through `a`, `b`, and `c` once into an exact
/// explicit plane. Repeated point queries can then share the same prepared
/// point-plane path instead of rebuilding the `orient3d` determinant.
#[derive(Clone, Debug)]
pub struct PreparedOrientedPlane3 {
    plane: Plane3,
    facts: Plane3Facts,
}

impl PreparedOrientedPlane3 {
    /// Prepare the oriented plane through `a`, `b`, and `c`.
    pub fn new(a: &Point3, b: &Point3, c: &Point3) -> Self {
        crate::trace_dispatch!("hyperlimit", "prepared_oriented_plane3", "new");
        let abx = sub(&b.x, &a.x);
        let aby = sub(&b.y, &a.y);
        let abz = sub(&b.z, &a.z);
        let acx = sub(&c.x, &a.x);
        let acy = sub(&c.y, &a.y);
        let acz = sub(&c.z, &a.z);

        let cross_x = sub(&mul(&aby, &acz), &mul(&abz, &acy));
        let cross_y = sub(&mul(&abz, &acx), &mul(&abx, &acz));
        let cross_z = sub(&mul(&abx, &acy), &mul(&aby, &acx));

        let nx_ax = mul(&cross_x, &a.x);
        let ny_ay = mul(&cross_y, &a.y);
        let nz_az = mul(&cross_z, &a.z);
        let nxy_a = add(&nx_ax, &ny_ay);
        let dot_a = add(&nxy_a, &nz_az);
        let zero = sub(&a.x, &a.x);
        let nx = sub(&zero, &cross_x);
        let ny = sub(&zero, &cross_y);
        let nz = sub(&zero, &cross_z);

        let plane = Plane3::new(Point3::new(nx, ny, nz), dot_a);
        let facts = plane.structural_facts();
        Self { plane, facts }
    }

    /// Return the explicit plane built from the oriented point triple.
    pub fn plane(&self) -> &Plane3 {
        &self.plane
    }

    /// Return cached structural facts for the explicit plane coefficients.
    pub fn facts(&self) -> Plane3Facts {
        self.facts
    }

    /// Classify a point using the default predicate policy.
    pub fn classify_point(&self, point: &Point3) -> PredicateOutcome<PlaneSide> {
        self.classify_point_with_policy(point, PredicatePolicy::default())
    }

    /// Classify a point using an explicit predicate policy.
    pub fn classify_point_with_policy(
        &self,
        point: &Point3,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<PlaneSide> {
        classify_point_plane_prepared(point, &self.plane, self.facts, policy)
    }
}

#[inline(always)]
fn classify_point_plane_prepared(
    point: &Point3,
    plane: &Plane3,
    facts: Plane3Facts,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    classify_point_plane_real(point, plane, Some(facts), policy)
}

/// Classify a point relative to a plane.
pub fn classify_point_plane(point: &Point3, plane: &Plane3) -> PredicateOutcome<PlaneSide> {
    classify_point_plane_with_policy(point, plane, PredicatePolicy::default())
}

/// Classify a point relative to a plane with an explicit escalation policy.
pub fn classify_point_plane_with_policy(
    point: &Point3,
    plane: &Plane3,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    classify_point_plane_real(point, plane, None, policy)
}

fn classify_point_plane_real(
    point: &Point3,
    plane: &Plane3,
    plane_facts: Option<Plane3Facts>,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    crate::trace_dispatch!("hyperlimit", "classify_point_plane", "real-dot");
    let value = point_plane_expression(point, plane, plane_facts);

    map_outcome(
        resolve_real_sign(
            &value,
            policy,
            || {
                let x_term = mul(&plane.normal.x, &point.x);
                let y_term = mul(&plane.normal.y, &point.y);
                let z_term = mul(&plane.normal.z, &point.z);
                signed_term_filter(&[
                    (&x_term, Sign::Positive),
                    (&y_term, Sign::Positive),
                    (&z_term, Sign::Positive),
                    (&plane.offset, Sign::Positive),
                ])
            },
            || None,
            RefinementNeed::RealRefinement,
        ),
        PlaneSide::from,
    )
}

/// Build `normal . point + offset` as one fixed product-sum when the object
/// facts make that route valid.
///
/// Prepared planes carry coefficient exactness and sparse-support facts beside
/// the plane rather than forcing every query to rediscover them. This helper
/// consumes those facts at the predicate-object boundary and passes the whole
/// point-plane polynomial to `hyperreal` before scalar expansion. That is the
/// representation separation advocated by Yap's exact geometric computation
/// model; see Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997). The exact-rational path uses the same
/// delayed-normalization idea as Bareiss, "Sylvester's Identity and Multistep
/// Integer-Preserving Gaussian Elimination," *Mathematics of Computation*
/// 22.103 (1968).
fn point_plane_expression(
    point: &Point3,
    plane: &Plane3,
    plane_facts: Option<Plane3Facts>,
) -> Real {
    let one = Real::one();
    let terms = [
        [&plane.normal.x, &point.x],
        [&plane.normal.y, &point.y],
        [&plane.normal.z, &point.z],
        [&plane.offset, &one],
    ];

    if let Some(plane_facts) = plane_facts {
        let point_exact = point.structural_facts().exact;
        if plane_facts.coefficient_exact.all_exact_rational && point_exact.all_exact_rational {
            crate::trace_dispatch!(
                "hyperlimit",
                "classify_point_plane",
                "prepared-exact-product-sum"
            );
            return Real::exact_rational_signed_product_sum_known_exact([true; 4], terms);
        }
    }

    crate::trace_dispatch!(
        "hyperlimit",
        "classify_point_plane",
        "fixed-real-product-sum"
    );
    Real::signed_product_sum([true; 4], terms)
}

fn plane3_facts(plane: &Plane3) -> Plane3Facts {
    let coefficients = [
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ];
    let coefficient_exact = Real::exact_set_facts(coefficients);
    let (coefficient_zero_mask, coefficient_nonzero_mask, coefficient_unknown_zero_mask) =
        plane_coefficient_zero_masks(coefficients);

    Plane3Facts {
        coefficient_exact,
        coefficient_symbolic_dependencies: plane_coefficient_symbolic_dependencies(coefficients),
        normal: plane.normal.structural_facts(),
        coefficient_zero_mask: coefficient_zero_mask as u8,
        coefficient_nonzero_mask: coefficient_nonzero_mask as u8,
        coefficient_unknown_zero_mask: coefficient_unknown_zero_mask as u8,
    }
}

fn plane_coefficient_symbolic_dependencies(coordinates: [&Real; 4]) -> RealSymbolicDependencyMask {
    coordinates
        .into_iter()
        .fold(RealSymbolicDependencyMask::NONE, |mask, coordinate| {
            mask.union(coordinate.detailed_facts().symbolic.dependencies)
        })
}

fn plane_coefficient_zero_masks(coordinates: [&Real; 4]) -> (u8, u8, u8) {
    let mut known_zero_mask = 0_u8;
    let mut known_nonzero_mask = 0_u8;
    let mut unknown_zero_mask = 0_u8;
    for (index, coordinate) in coordinates.into_iter().enumerate() {
        let bit = 1_u8 << index;
        match coordinate.structural_facts().zero {
            ZeroKnowledge::Zero => known_zero_mask |= bit,
            ZeroKnowledge::NonZero => known_nonzero_mask |= bit,
            ZeroKnowledge::Unknown => unknown_zero_mask |= bit,
        }
    }
    (known_zero_mask, known_nonzero_mask, unknown_zero_mask)
}

/// Classify a point relative to the oriented plane through `a`, `b`, and `c`.
pub fn classify_point_oriented_plane(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    point: &Point3,
) -> PredicateOutcome<PlaneSide> {
    classify_point_oriented_plane_with_policy(a, b, c, point, PredicatePolicy::default())
}

/// Classify a point relative to the oriented plane through `a`, `b`, and `c`
/// with an explicit escalation policy.
pub fn classify_point_oriented_plane_with_policy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<PlaneSide> {
    crate::trace_dispatch!("hyperlimit", "classify_point_oriented_plane", "orient3d");
    map_outcome(
        orient3d_with_policy(a, b, c, point, policy),
        PlaneSide::from,
    )
}

fn add(left: &Real, right: &Real) -> Real {
    add_ref(left, right)
}

fn mul(left: &Real, right: &Real) -> Real {
    mul_ref(left, right)
}

fn sub(left: &Real, right: &Real) -> Real {
    sub_ref(left, right)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "dispatch-trace")]
    use hyperreal::Rational;

    #[cfg(feature = "dispatch-trace")]
    fn dispatch_trace_test_lock() -> &'static std::sync::Mutex<()> {
        static LOCK: std::sync::OnceLock<std::sync::Mutex<()>> = std::sync::OnceLock::new();
        LOCK.get_or_init(|| std::sync::Mutex::new(()))
    }

    fn real(value: f64) -> Real {
        Real::try_from(value).expect("finite test Real")
    }

    fn p3(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(real(x), real(y), real(z))
    }

    #[test]
    fn classifies_point_plane() {
        let plane = Plane3::new(p3(0.0, 0.0, 1.0), real(-2.0));

        assert_eq!(
            classify_point_plane(&p3(0.0, 0.0, 3.0), &plane).value(),
            Some(PlaneSide::Above)
        );
        assert_eq!(
            classify_point_plane(&p3(0.0, 0.0, 1.0), &plane).value(),
            Some(PlaneSide::Below)
        );
    }

    #[test]
    fn classifies_point_oriented_plane_from_points() {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(1.0, 0.0, 0.0);
        let c = p3(0.0, 1.0, 0.0);

        assert_eq!(
            classify_point_oriented_plane(&a, &b, &c, &p3(0.0, 0.0, 1.0)).value(),
            Some(PlaneSide::Below)
        );
    }

    #[test]
    fn plane_facts_preserve_coefficient_structure_for_prepared_queries() {
        let plane = Plane3::new(
            Point3::new(Real::from(0), Real::from(3), Real::from(0)),
            Real::from(-6),
        );
        let facts = plane.structural_facts();

        assert_eq!(facts.coefficient_zero_mask, 0b0101);
        assert_eq!(facts.coefficient_nonzero_mask, 0b1010);
        assert_eq!(facts.coefficient_unknown_zero_mask, 0);
        assert_eq!(facts.coefficient_zero_count(), 2);
        assert_eq!(facts.coefficient_nonzero_count(), 2);
        assert_eq!(facts.coefficient_unknown_zero_count(), 0);
        assert!(facts.normal_has_sparse_support());
        assert!(!facts.normal_known_zero());
        assert!(facts.has_dyadic_schedule());
        assert!(facts.has_shared_denominator_schedule());
        assert!(facts.coefficient_symbolic_dependencies.is_empty());

        let prepared = plane.prepare();
        assert_eq!(prepared.plane(), &plane);
        assert_eq!(prepared.facts(), facts);
        assert_eq!(
            prepared.classify_point(&Point3::new(0.into(), 3.into(), 0.into())),
            classify_point_plane(&Point3::new(0.into(), 3.into(), 0.into()), &plane)
        );
    }

    #[test]
    fn prepared_oriented_plane_matches_orient3d_side() {
        let a = p3(-0.85, -0.7, -0.25);
        let b = p3(0.9, -0.35, 0.35);
        let c = p3(-0.35, 0.85, 0.05);
        let prepared = PreparedOrientedPlane3::new(&a, &b, &c);
        assert_eq!(prepared.facts(), prepared.plane().structural_facts());

        for point in [
            p3(0.2, -0.1, 0.8),
            p3(-0.4, 0.3, -0.2),
            p3(0.1, 0.2, 0.38 * 0.1 + 0.24 * 0.2),
        ] {
            assert_eq!(
                prepared.classify_point(&point).value(),
                classify_point_oriented_plane(&a, &b, &c, &point).value()
            );
        }
    }

    #[test]
    fn classifies_point_plane_with_hyperreal_structural_facts() {
        use crate::predicate::{Certainty, Escalation};

        let plane = Plane3::new(Point3::new(Real::from(1), 0.into(), 0.into()), (-4).into());
        let point = Point3::new(Real::pi(), 0.into(), 0.into());

        assert_eq!(
            classify_point_plane(&point, &plane),
            PredicateOutcome::decided(PlaneSide::Below, Certainty::Exact, Escalation::Structural)
        );
    }

    #[test]
    fn plane_facts_summarize_symbolic_dependencies_for_prepared_queries() {
        let trig = (Real::from(hyperreal::Rational::fraction(1, 5).unwrap()) * Real::pi()).sin();
        let plane = Plane3::new(Point3::new(Real::pi(), trig, 0.into()), Real::e());
        let facts = plane.structural_facts();

        assert!(
            facts
                .coefficient_symbolic_dependencies
                .contains(RealSymbolicDependencyMask::PI)
        );
        assert!(
            facts
                .coefficient_symbolic_dependencies
                .contains(RealSymbolicDependencyMask::TRIG)
        );
        assert!(
            facts
                .coefficient_symbolic_dependencies
                .contains(RealSymbolicDependencyMask::EXP)
        );

        let prepared = plane.prepare();
        assert_eq!(
            prepared.facts().coefficient_symbolic_dependencies,
            facts.coefficient_symbolic_dependencies
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn prepared_point_plane_reuses_coefficients_for_one_exact_product_sum() {
        let _trace_lock = dispatch_trace_test_lock()
            .lock()
            .expect("dispatch trace test lock poisoned");
        let fifth = |value| Real::from(Rational::fraction(value, 5).unwrap());
        let plane = Plane3::new(Point3::new(fifth(2), fifth(-3), fifth(4)), fifth(-6));
        let point = Point3::new(fifth(5), fifth(5), fifth(5));
        let prepared = plane.prepare();

        hyperreal::dispatch_trace::reset();
        let outcome = hyperreal::dispatch_trace::with_recording(|| {
            prepared.classify_point_with_policy(&point, PredicatePolicy::STRICT)
        });

        assert_eq!(outcome.value(), Some(PlaneSide::Below));
        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count(
                "hyperlimit",
                "classify_point_plane",
                "prepared-exact-product-sum"
            ),
            1
        );
        assert_eq!(
            trace.path_count(
                "hyperlimit",
                "classify_point_plane",
                "fixed-real-product-sum"
            ),
            0
        );
        assert_eq!(
            trace.path_count("real", "product_sum", "exact-rational-known-shared-denom"),
            1
        );
    }
}
