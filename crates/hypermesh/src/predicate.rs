//! Certified scalar and point-plane predicate dispatch.

use std::cmp::Ordering;

use hyperlattice::{HomogeneousPoint3, Point3, Rational, Real, homogeneous_point_plane_expression};
use hyperlimit::{Sign, classify_real_sign};
use hyperreal::{RationalLinearForm4Filter, RationalLinearForm4Query, RealSign};

use crate::context::{DecisionContext, MeshContext, MeshOutcome};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::Plane;

const RATIONAL_LINEAR_FORM4_FILTER_CACHE_CAPACITY: usize = 8_192;
const RATIONAL_LINEAR_FORM4_FILTER_SLOT_CAPACITY: usize =
    RATIONAL_LINEAR_FORM4_FILTER_CACHE_CAPACITY * 2;
const INITIAL_RATIONAL_LINEAR_FORM4_FILTER_SLOT_CAPACITY: usize = 16;
const EMPTY_RATIONAL_LINEAR_FORM4_FILTER_SLOT: u16 = u16::MAX;

struct CachedRationalLinearForm4Filter {
    fingerprint: u64,
    owners: [Rational; 4],
    filter: Option<RationalLinearForm4Filter>,
}

pub(crate) struct RationalLinearForm4FilterCache {
    /// Open-addressed slots contain compact indices into `entries`. Keeping
    /// storage owners only in the dense entry array avoids duplicating four
    /// pointers in every hash-table entry.
    slots: Vec<u16>,
    entries: Vec<CachedRationalLinearForm4Filter>,
}

impl Default for RationalLinearForm4FilterCache {
    fn default() -> Self {
        Self::new()
    }
}

impl RationalLinearForm4FilterCache {
    pub(crate) const fn new() -> Self {
        Self {
            slots: Vec::new(),
            entries: Vec::new(),
        }
    }

    #[inline]
    fn fingerprint(key: [usize; 4]) -> u64 {
        let mut mixed = 4_u64.wrapping_mul(0x517c_c1b7_2722_0a95);
        for word in key {
            mixed = mixed.rotate_left(19).wrapping_add(word as u64);
        }
        mixed = (mixed ^ (mixed >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        mixed = (mixed ^ (mixed >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        mixed ^ (mixed >> 31)
    }

    #[inline]
    fn entry_matches(entry: &CachedRationalLinearForm4Filter, key: [usize; 4]) -> bool {
        entry
            .owners
            .iter()
            .zip(key)
            .all(|(owner, identity)| owner.storage_identity() == identity)
    }

    #[inline]
    fn find_index(&self, key: [usize; 4]) -> Option<usize> {
        if self.slots.is_empty() {
            return None;
        }
        let fingerprint = Self::fingerprint(key);
        let mut slot = fingerprint as usize & (self.slots.len() - 1);
        loop {
            let entry_index = self.slots[slot];
            if entry_index == EMPTY_RATIONAL_LINEAR_FORM4_FILTER_SLOT {
                return None;
            }
            let entry_index = usize::from(entry_index);
            let entry = &self.entries[entry_index];
            if entry.fingerprint == fingerprint && Self::entry_matches(entry, key) {
                return Some(entry_index);
            }
            slot = (slot + 1) & (self.slots.len() - 1);
        }
    }

    #[inline]
    #[cfg(test)]
    fn find(&self, key: [usize; 4]) -> Option<Option<RationalLinearForm4Filter>> {
        self.find_index(key).map(|index| self.entries[index].filter)
    }

    fn grow_slots(&mut self) {
        let new_capacity = (self.slots.len() * 2).min(RATIONAL_LINEAR_FORM4_FILTER_SLOT_CAPACITY);
        debug_assert!(new_capacity > self.slots.len());
        self.slots = vec![EMPTY_RATIONAL_LINEAR_FORM4_FILTER_SLOT; new_capacity];
        for (entry_index, entry) in self.entries.iter().enumerate() {
            let mut slot = entry.fingerprint as usize & (new_capacity - 1);
            while self.slots[slot] != EMPTY_RATIONAL_LINEAR_FORM4_FILTER_SLOT {
                slot = (slot + 1) & (new_capacity - 1);
            }
            self.slots[slot] = entry_index as u16;
        }
    }

    #[inline]
    fn insert(&mut self, entry: CachedRationalLinearForm4Filter) -> usize {
        if self.slots.is_empty() {
            self.slots = vec![
                EMPTY_RATIONAL_LINEAR_FORM4_FILTER_SLOT;
                INITIAL_RATIONAL_LINEAR_FORM4_FILTER_SLOT_CAPACITY
            ];
        }
        if self.entries.len() >= RATIONAL_LINEAR_FORM4_FILTER_CACHE_CAPACITY {
            crate::trace_dispatch!("rational-linear-form4-filter-cache", "clear-at-capacity");
            self.entries.clear();
            self.slots.fill(EMPTY_RATIONAL_LINEAR_FORM4_FILTER_SLOT);
        }
        if self.entries.len() >= self.slots.len() / 2 {
            self.grow_slots();
        }
        let mut slot = entry.fingerprint as usize & (self.slots.len() - 1);
        while self.slots[slot] != EMPTY_RATIONAL_LINEAR_FORM4_FILTER_SLOT {
            slot = (slot + 1) & (self.slots.len() - 1);
        }
        let entry_index = self.entries.len();
        debug_assert!(entry_index < usize::from(EMPTY_RATIONAL_LINEAR_FORM4_FILTER_SLOT));
        self.entries.push(entry);
        self.slots[slot] = entry_index as u16;
        entry_index
    }
}

fn rational_linear_form4_filter(
    decisions: &DecisionContext,
    plane: &Plane,
    coefficients: [&Rational; 4],
) -> Option<RationalLinearForm4Filter> {
    let key = coefficients.map(Rational::storage_identity);
    let mut cache = decisions.rational_linear_form4_filters.borrow_mut();
    if let Some(index) = cache.find_index(key) {
        crate::trace_dispatch!("rational-linear-form4-filter-cache", "hit");
        return cache.entries[index].filter;
    }
    crate::trace_dispatch!("rational-linear-form4-filter-cache", "miss");
    let filter = RationalLinearForm4Filter::from_reals([
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ]);
    cache.insert(CachedRationalLinearForm4Filter {
        fingerprint: RationalLinearForm4FilterCache::fingerprint(key),
        owners: coefficients.map(Clone::clone),
        filter,
    });
    filter
}

/// Certified point-vs-plane classification.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Classification {
    /// Point is on the negative side of the plane.
    Negative,
    /// Point lies exactly on the plane.
    On,
    /// Point is on the positive side of the plane.
    Positive,
}

impl Classification {
    /// Returns true when the classification is positive.
    pub const fn is_positive(self) -> bool {
        matches!(self, Self::Positive)
    }

    /// Returns true when the classification is negative.
    pub const fn is_negative(self) -> bool {
        matches!(self, Self::Negative)
    }

    /// Returns true when the point is on the negative side or on the plane.
    pub const fn is_non_positive(self) -> bool {
        !self.is_positive()
    }

    /// Returns true when the point is on the positive side or on the plane.
    pub const fn is_non_negative(self) -> bool {
        !self.is_negative()
    }
}

/// Classifies an affine point against a plane.
pub fn classify_point(
    context: &MeshContext,
    point: &Point3,
    plane: &Plane,
) -> HypermeshResult<MeshOutcome<Classification>> {
    let decisions = DecisionContext::new(context);
    let classification = classify_point_decision(&decisions, point, plane)?;
    Ok(decisions.finish(classification))
}

pub(crate) fn classify_point_decision(
    decisions: &DecisionContext,
    point: &Point3,
    plane: &Plane,
) -> HypermeshResult<Classification> {
    Point3PredicateQuery::new(point).classify(decisions, point, plane)
}

/// Classifies a point with certified query facts retained by its source owner.
///
/// The query is only a floating filter input. An inconclusive filter reaches
/// the same exact-rational fallback as [`classify_point_decision`].
pub(crate) fn classify_point_with_rational_query_decision(
    decisions: &DecisionContext,
    point: &Point3,
    plane: &Plane,
    rational_query: &RationalLinearForm4Query,
) -> HypermeshResult<Classification> {
    classify_point_with_optional_rational_query_decision(
        decisions,
        point,
        plane,
        Some(rational_query),
    )
}

/// Compact scalar-owned query facts cached for repeated plane predicates.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Point3PredicateQuery {
    rational_filter_query: Option<RationalLinearForm4Query>,
}

impl Point3PredicateQuery {
    /// Retains the certified floating query without cloning exact coordinates.
    pub(crate) fn new(point: &Point3) -> Self {
        let exact_coordinates = match (
            point.x.exact_rational_ref(),
            point.y.exact_rational_ref(),
            point.z.exact_rational_ref(),
        ) {
            (Some(x), Some(y), Some(z)) => Some([x, y, z]),
            _ => None,
        };
        crate::trace_dispatch!(
            "point3-evidence",
            if exact_coordinates.is_some() {
                "exact-rational"
            } else {
                "general-real"
            }
        );
        let rational_filter_query =
            exact_coordinates.and_then(RationalLinearForm4Query::from_affine_point3);
        if rational_filter_query.is_some() {
            crate::trace_dispatch!("point3-evidence", "rational-filter-query");
        }
        Self {
            rational_filter_query,
        }
    }

    /// Returns the compact certified filter query, when this representation
    /// supplies one.
    pub(crate) const fn rational_filter_query(self) -> Option<RationalLinearForm4Query> {
        self.rational_filter_query
    }

    /// Classifies the immutable point that supplied these retained query facts.
    pub(crate) fn classify(
        &self,
        decisions: &DecisionContext,
        point: &Point3,
        plane: &Plane,
    ) -> HypermeshResult<Classification> {
        classify_point_with_optional_rational_query_decision(
            decisions,
            point,
            plane,
            self.rational_filter_query.as_ref(),
        )
    }
}

fn classify_point_with_optional_rational_query_decision(
    decisions: &DecisionContext,
    point: &Point3,
    plane: &Plane,
    rational_filter_query: Option<&RationalLinearForm4Query>,
) -> HypermeshResult<Classification> {
    if let [Some(x), Some(y), Some(z)] =
        [&point.x, &point.y, &point.z].map(Real::exact_rational_ref)
        && let Some(classification) = classify_exact_rational_coordinates(
            decisions,
            plane,
            [x, y, z],
            Rational::one_ref(),
            rational_filter_query,
        )
    {
        crate::trace_dispatch!("classify-point", "affine-exact-rational");
        return Ok(classification);
    }

    crate::trace_dispatch!("classify-point", "affine-real-fallback");
    classify_real(decisions, &plane.expression_at_point(point))
}

/// Classifies a homogeneous point against a plane.
pub fn classify_projective_point(
    context: &MeshContext,
    point: &HomogeneousPoint3,
    plane: &Plane,
) -> HypermeshResult<MeshOutcome<Classification>> {
    let decisions = DecisionContext::new(context);
    let classification = classify_projective_point_decision(&decisions, point, plane)?;
    Ok(decisions.finish(classification))
}

pub(crate) fn classify_projective_point_decision(
    decisions: &DecisionContext,
    point: &HomogeneousPoint3,
    plane: &Plane,
) -> HypermeshResult<Classification> {
    if let Some(weight) = point.w.exact_rational_ref()
        && let Some(classification) =
            classify_exact_rational_terms(decisions, plane, [&point.x, &point.y, &point.z], weight)
    {
        crate::trace_dispatch!("classify-point", "projective-exact-rational");
        return Ok(classification);
    }
    crate::trace_dispatch!("classify-point", "projective-real-fallback");
    classify_real(decisions, &homogeneous_point_plane_expression(point, plane))
}

fn classify_exact_rational_terms(
    decisions: &DecisionContext,
    plane: &Plane,
    coordinates: [&Real; 3],
    homogeneous_weight: &Rational,
) -> Option<Classification> {
    let [Some(x), Some(y), Some(z)] = coordinates.map(Real::exact_rational_ref) else {
        return None;
    };
    classify_exact_rational_coordinates(decisions, plane, [x, y, z], homogeneous_weight, None)
}

fn classify_exact_rational_coordinates(
    decisions: &DecisionContext,
    plane: &Plane,
    [x, y, z]: [&Rational; 3],
    homogeneous_weight: &Rational,
    rational_query: Option<&RationalLinearForm4Query>,
) -> Option<Classification> {
    let [Some(a), Some(b), Some(c), Some(d)] = [
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ]
    .map(Real::exact_rational_ref) else {
        return None;
    };
    Some(classify_exact_rational_coordinates_with_filter(
        [a, b, c, d],
        [x, y, z],
        homogeneous_weight,
        rational_query,
        rational_linear_form4_filter(decisions, plane, [a, b, c, d]),
    ))
}

fn classify_exact_rational_coordinates_with_filter(
    [a, b, c, d]: [&Rational; 4],
    [x, y, z]: [&Rational; 3],
    homogeneous_weight: &Rational,
    rational_query: Option<&RationalLinearForm4Query>,
    filter: Option<RationalLinearForm4Filter>,
) -> Classification {
    let filtered_sign = filter.and_then(|filter| match rational_query {
        Some(query) => filter.sign(query),
        None => filter.sign_rationals([x, y, z, homogeneous_weight]),
    });
    if let Some(sign) = filtered_sign {
        crate::trace_dispatch!(
            "classify-point",
            if homogeneous_weight.is_one() {
                "affine-rational-floating-filter"
            } else {
                "projective-rational-floating-filter"
            }
        );
        return match sign {
            RealSign::Negative => Classification::Negative,
            RealSign::Zero => Classification::On,
            RealSign::Positive => Classification::Positive,
        };
    }

    match Rational::signed_product_sum_ordering(
        [true; 4],
        [[a, x], [b, y], [c, z], [d, homogeneous_weight]],
    ) {
        Ordering::Less => Classification::Negative,
        Ordering::Equal => Classification::On,
        Ordering::Greater => Classification::Positive,
    }
}

/// Returns an ordering resolved by Hyperlimit's active predicate policy.
pub fn compare_real(
    context: &MeshContext,
    left: &Real,
    right: &Real,
) -> HypermeshResult<MeshOutcome<Ordering>> {
    let decisions = DecisionContext::new(context);
    let ordering = compare_real_decision(&decisions, left, right)?;
    Ok(decisions.finish(ordering))
}

pub(crate) fn compare_real_decision(
    decisions: &DecisionContext,
    left: &Real,
    right: &Real,
) -> HypermeshResult<Ordering> {
    if let (Some(left), Some(right)) = (left.exact_rational_ref(), right.exact_rational_ref()) {
        crate::trace_dispatch!("compare-real", "exact-rational");
        return Ok(left
            .partial_cmp(right)
            .expect("exact rationals are totally ordered"));
    }
    crate::trace_dispatch!("compare-real", "hyperlimit");
    decisions.decide(
        hyperlimit::compare_reals(left, right, decisions.policy()),
        "Real ordering",
    )
}

/// Decides equality of three exact coordinates.
///
/// Exact rationals and identical retained expressions are sound early
/// certificates. Every remaining coordinate is decided by Hyperlimit's active
/// policy, and exhausted certainty is preserved as `UnknownClassification`.
#[inline]
pub(crate) fn coordinates3_equal(
    decisions: &DecisionContext,
    left: [&Real; 3],
    right: [&Real; 3],
) -> HypermeshResult<bool> {
    let mut saw_unknown = false;
    for (left, right) in left.into_iter().zip(right) {
        if let (Some(left), Some(right)) = (left.exact_rational_ref(), right.exact_rational_ref()) {
            if left != right {
                return Ok(false);
            }
            continue;
        }
        if left == right {
            continue;
        }
        match decisions.probe(hyperlimit::compare_reals(left, right, decisions.policy())) {
            Some(Ordering::Equal) => {}
            Some(_) => return Ok(false),
            None => saw_unknown = true,
        }
    }
    if saw_unknown {
        Err(HypermeshError::PredicateUndecided {
            predicate: "3D coordinate equality",
        })
    } else {
        Ok(true)
    }
}

/// Returns true when retained exact-rational coordinates already disprove
/// affine-point equality. Unknown coordinate representations never contradict.
#[inline]
pub(crate) fn exact_rational_points_contradict(left: &Point3, right: &Point3) -> bool {
    let left = [&left.x, &left.y, &left.z].map(Real::exact_rational_ref);
    let right = [&right.x, &right.y, &right.z].map(Real::exact_rational_ref);
    left.into_iter()
        .zip(right)
        .any(|(left, right)| matches!((left, right), (Some(left), Some(right)) if left != right))
}

/// Decides exact affine-point equality through the shared coordinate cascade.
#[inline]
pub(crate) fn points_equal(
    decisions: &DecisionContext,
    left: &Point3,
    right: &Point3,
) -> HypermeshResult<bool> {
    coordinates3_equal(
        decisions,
        [&left.x, &left.y, &left.z],
        [&right.x, &right.y, &right.z],
    )
}

pub(crate) fn classify_real(
    decisions: &DecisionContext,
    value: &Real,
) -> HypermeshResult<Classification> {
    if let Some(value) = value.exact_rational_ref() {
        crate::trace_dispatch!("classify-real", "exact-rational");
        return Ok(
            match value
                .partial_cmp(&Rational::zero())
                .expect("exact rationals are totally ordered")
            {
                Ordering::Less => Classification::Negative,
                Ordering::Equal => Classification::On,
                Ordering::Greater => Classification::Positive,
            },
        );
    }
    crate::trace_dispatch!("classify-real", "hyperlimit");
    decisions
        .decide(
            classify_real_sign(value, decisions.policy()),
            "Real sign classification",
        )
        .map(|sign| match sign {
            Sign::Negative => Classification::Negative,
            Sign::Zero => Classification::On,
            Sign::Positive => Classification::Positive,
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: Real, y: Real, z: Real) -> Point3 {
        Point3::new(x, y, z)
    }

    #[test]
    fn linear_form_filter_cache_retains_owners_and_probes_collisions() {
        let mut cache = RationalLinearForm4FilterCache::default();
        let mut first_by_slot = std::collections::HashMap::new();
        let (first, second) = (1_i64..20_000)
            .find_map(|value| {
                let owners = std::array::from_fn(|offset| Rational::new(value + offset as i64));
                let key = owners.each_ref().map(|owner| owner.storage_identity());
                let slot = RationalLinearForm4FilterCache::fingerprint(key) as usize
                    & (RATIONAL_LINEAR_FORM4_FILTER_SLOT_CAPACITY - 1);
                if let Some(first) = first_by_slot.remove(&slot) {
                    Some((first, (key, owners)))
                } else {
                    first_by_slot.insert(slot, (key, owners));
                    None
                }
            })
            .expect("more candidates than slots must produce a collision");

        let (first_key, first_owners) = first;
        let (second_key, second_owners) = second;
        cache.insert(CachedRationalLinearForm4Filter {
            fingerprint: RationalLinearForm4FilterCache::fingerprint(first_key),
            owners: first_owners,
            filter: None,
        });
        cache.insert(CachedRationalLinearForm4Filter {
            fingerprint: RationalLinearForm4FilterCache::fingerprint(second_key),
            owners: second_owners,
            filter: None,
        });

        assert!(matches!(cache.find(first_key), Some(None)));
        assert!(matches!(cache.find(second_key), Some(None)));
        assert_eq!(cache.entries.len(), 2);

        let mut grown_keys = Vec::new();
        for value in 30_000_i64..30_512 {
            let owners = std::array::from_fn(|offset| Rational::new(value + offset as i64));
            let key = owners.each_ref().map(|owner| owner.storage_identity());
            cache.insert(CachedRationalLinearForm4Filter {
                fingerprint: RationalLinearForm4FilterCache::fingerprint(key),
                owners,
                filter: None,
            });
            grown_keys.push(key);
        }
        assert!(cache.slots.len() > INITIAL_RATIONAL_LINEAR_FORM4_FILTER_SLOT_CAPACITY);
        assert!(
            grown_keys
                .into_iter()
                .all(|key| matches!(cache.find(key), Some(None)))
        );
    }

    #[test]
    fn linear_form_filter_cache_clears_at_bounded_capacity() {
        let mut cache = RationalLinearForm4FilterCache::default();
        let mut first_key = None;
        for value in 0..=RATIONAL_LINEAR_FORM4_FILTER_CACHE_CAPACITY {
            let base = i64::try_from(value).unwrap() * 4;
            let owners = std::array::from_fn(|offset| Rational::new(base + offset as i64));
            let key = owners.each_ref().map(|owner| owner.storage_identity());
            first_key.get_or_insert(key);
            cache.insert(CachedRationalLinearForm4Filter {
                fingerprint: RationalLinearForm4FilterCache::fingerprint(key),
                owners,
                filter: None,
            });
        }

        assert_eq!(cache.entries.len(), 1);
        let retained_key = cache.entries[0]
            .owners
            .each_ref()
            .map(|owner| owner.storage_identity());
        assert!(cache.find(first_key.unwrap()).is_none());
        assert!(matches!(cache.find(retained_key), Some(None)));
        assert_eq!(
            cache.slots.len(),
            RATIONAL_LINEAR_FORM4_FILTER_SLOT_CAPACITY
        );
    }

    #[test]
    fn point_query_matches_direct_exact_classification() {
        let point = point(Real::from(2), Real::from(3), Real::from(5));
        let query = Point3PredicateQuery::new(&point);
        let planes = [
            Plane::from_coefficients(Real::one(), Real::zero(), Real::zero(), Real::from(-2)),
            Plane::from_coefficients(Real::zero(), Real::one(), Real::zero(), Real::from(-4)),
        ];

        assert_eq!(
            query
                .classify(
                    &crate::test_support::approximate_decisions(),
                    &point,
                    &planes[0],
                )
                .unwrap(),
            Classification::On
        );
        assert_eq!(
            query
                .classify(
                    &crate::test_support::approximate_decisions(),
                    &point,
                    &planes[1],
                )
                .unwrap(),
            Classification::Negative
        );
        for plane in &planes {
            assert_eq!(
                query.classify(&crate::test_support::approximate_decisions(), &point, plane,),
                classify_point_decision(
                    &crate::test_support::approximate_decisions(),
                    &point,
                    plane,
                )
            );
        }
    }

    #[test]
    fn wide_dyadic_point_plane_sign_uses_complete_exact_path_under_both_policies() {
        let denominator = Rational::new(2)
            .powi(2048_i64.into())
            .expect("fixture exponent is positive");
        let scale = (&denominator + Rational::one()) / &denominator;
        let scale_squared = &scale * &scale;
        let coefficient_scale = &scale_squared * &scale_squared;
        let plane = Plane::from_coefficients(
            Real::from(coefficient_scale.clone()),
            Real::from(-coefficient_scale.clone()),
            Real::zero(),
            Real::zero(),
        );
        let on_plane = point(
            Real::from(scale.clone()),
            Real::from(scale.clone()),
            Real::zero(),
        );
        let positive = point(
            Real::from(Rational::new(2) * &scale),
            Real::from(scale),
            Real::zero(),
        );

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            assert_eq!(
                Point3PredicateQuery::new(&on_plane)
                    .classify(&decisions, &on_plane, &plane)
                    .unwrap(),
                Classification::On
            );
            assert_eq!(
                Point3PredicateQuery::new(&positive)
                    .classify(&decisions, &positive, &plane)
                    .unwrap(),
                Classification::Positive
            );
            assert_eq!(
                decisions.certainty(),
                crate::context::MeshCertainty::Certified
            );
        }
    }

    #[test]
    fn symbolic_coefficients_preserve_general_exact_fallback() {
        let point = point(Real::one(), Real::zero(), Real::zero());
        let plane =
            Plane::from_coefficients(Real::pi(), Real::zero(), Real::zero(), Real::from(-3));

        assert_eq!(
            classify_point_decision(
                &crate::test_support::approximate_decisions(),
                &point,
                &plane,
            )
            .unwrap(),
            Classification::Positive
        );
        assert_eq!(
            Point3PredicateQuery::new(&point)
                .classify(
                    &crate::test_support::approximate_decisions(),
                    &point,
                    &plane,
                )
                .unwrap(),
            Classification::Positive
        );
    }

    #[test]
    fn projective_exact_dispatch_respects_homogeneous_weight() {
        let plane =
            Plane::from_coefficients(Real::one(), Real::zero(), Real::zero(), Real::from(-2));
        let point =
            HomogeneousPoint3::new(Real::from(6), Real::zero(), Real::zero(), Real::from(3));
        assert_eq!(
            classify_projective_point_decision(
                &crate::test_support::approximate_decisions(),
                &point,
                &plane,
            )
            .unwrap(),
            Classification::On
        );
    }

    #[test]
    fn exact_real_comparison_matches_rational_ordering() {
        assert_eq!(
            compare_real_decision(
                &crate::test_support::approximate_decisions(),
                &Real::from(-3),
                &Real::from(2),
            )
            .unwrap(),
            Ordering::Less,
        );
        assert_eq!(
            compare_real_decision(
                &crate::test_support::approximate_decisions(),
                &Real::from(5),
                &Real::from(5),
            )
            .unwrap(),
            Ordering::Equal,
        );
    }

    #[cfg(feature = "fuzz-bounded-campaign")]
    #[test]
    fn fuzz_campaign_uses_central_symbolic_comparison_and_zero_classification() {
        assert_eq!(
            compare_real_decision(
                &crate::test_support::approximate_decisions(),
                &Real::e(),
                &Real::pi(),
            )
            .unwrap(),
            Ordering::Less,
        );
        assert_eq!(
            compare_real_decision(
                &crate::test_support::approximate_decisions(),
                &Real::pi(),
                &Real::pi(),
            )
            .unwrap(),
            Ordering::Equal,
        );
        assert_eq!(
            classify_real(&crate::test_support::approximate_decisions(), &Real::pi()).unwrap(),
            Classification::Positive,
        );
        assert_eq!(
            classify_real(&crate::test_support::approximate_decisions(), &-Real::pi()).unwrap(),
            Classification::Negative,
        );
        assert_eq!(
            classify_real(&crate::test_support::approximate_decisions(), &Real::zero()).unwrap(),
            Classification::On,
        );
    }
}
