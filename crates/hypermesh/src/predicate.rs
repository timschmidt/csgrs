//! Certified scalar and point-plane predicate dispatch.

use std::cell::RefCell;
use std::cmp::Ordering;

use hyperlattice::{HomogeneousPoint3, Point3, Rational, Real, homogeneous_point_plane_expression};
use hyperlimit::{PredicateOutcome, Sign, classify_real_sign};
use hyperreal::{PreparedRationalLinearForm4Filter, PreparedRationalLinearForm4Query, RealSign};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::Plane;

const LINEAR_FORM_FILTER_CACHE_CAPACITY: usize = 8_192;
const LINEAR_FORM_FILTER_SLOT_CAPACITY: usize = LINEAR_FORM_FILTER_CACHE_CAPACITY * 2;
const INITIAL_LINEAR_FORM_FILTER_SLOT_CAPACITY: usize = 16;
const EMPTY_LINEAR_FORM_FILTER_SLOT: u16 = u16::MAX;

struct CachedLinearForm3Filter {
    fingerprint: u64,
    owners: [Rational; 4],
    filter: Option<PreparedRationalLinearForm4Filter>,
}

struct LinearForm3FilterCache {
    /// Open-addressed slots contain compact indices into `entries`. Keeping
    /// storage owners only in the dense entry array avoids duplicating four
    /// pointers in every hash-table entry.
    slots: Vec<u16>,
    entries: Vec<CachedLinearForm3Filter>,
}

impl Default for LinearForm3FilterCache {
    fn default() -> Self {
        Self {
            slots: vec![EMPTY_LINEAR_FORM_FILTER_SLOT; INITIAL_LINEAR_FORM_FILTER_SLOT_CAPACITY],
            entries: Vec::new(),
        }
    }
}

impl LinearForm3FilterCache {
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
    fn entry_matches(entry: &CachedLinearForm3Filter, key: [usize; 4]) -> bool {
        entry
            .owners
            .iter()
            .zip(key)
            .all(|(owner, identity)| owner.storage_identity() == identity)
    }

    #[inline]
    fn find(&self, key: [usize; 4]) -> Option<Option<PreparedRationalLinearForm4Filter>> {
        let fingerprint = Self::fingerprint(key);
        let mut slot = fingerprint as usize & (self.slots.len() - 1);
        loop {
            let entry_index = self.slots[slot];
            if entry_index == EMPTY_LINEAR_FORM_FILTER_SLOT {
                return None;
            }
            let entry = &self.entries[usize::from(entry_index)];
            if entry.fingerprint == fingerprint && Self::entry_matches(entry, key) {
                return Some(entry.filter);
            }
            slot = (slot + 1) & (self.slots.len() - 1);
        }
    }

    fn grow_slots(&mut self) {
        let new_capacity = (self.slots.len() * 2).min(LINEAR_FORM_FILTER_SLOT_CAPACITY);
        debug_assert!(new_capacity > self.slots.len());
        self.slots = vec![EMPTY_LINEAR_FORM_FILTER_SLOT; new_capacity];
        for (entry_index, entry) in self.entries.iter().enumerate() {
            let mut slot = entry.fingerprint as usize & (new_capacity - 1);
            while self.slots[slot] != EMPTY_LINEAR_FORM_FILTER_SLOT {
                slot = (slot + 1) & (new_capacity - 1);
            }
            self.slots[slot] = entry_index as u16;
        }
    }

    #[inline]
    fn insert(&mut self, entry: CachedLinearForm3Filter) {
        if self.entries.len() >= LINEAR_FORM_FILTER_CACHE_CAPACITY {
            self.entries.clear();
            self.slots.fill(EMPTY_LINEAR_FORM_FILTER_SLOT);
        }
        if self.entries.len() >= self.slots.len() / 2 {
            self.grow_slots();
        }
        let mut slot = entry.fingerprint as usize & (self.slots.len() - 1);
        while self.slots[slot] != EMPTY_LINEAR_FORM_FILTER_SLOT {
            slot = (slot + 1) & (self.slots.len() - 1);
        }
        let entry_index = self.entries.len();
        debug_assert!(entry_index < usize::from(EMPTY_LINEAR_FORM_FILTER_SLOT));
        self.entries.push(entry);
        self.slots[slot] = entry_index as u16;
    }
}

thread_local! {
    static LINEAR_FORM_FILTERS: RefCell<LinearForm3FilterCache> =
        RefCell::new(LinearForm3FilterCache::default());
}

fn prepared_linear_form3_filter(
    plane: &Plane,
    coefficients: [&Rational; 4],
) -> Option<PreparedRationalLinearForm4Filter> {
    let key = coefficients.map(Rational::storage_identity);
    LINEAR_FORM_FILTERS.with_borrow_mut(|cache| {
        if let Some(filter) = cache.find(key) {
            return filter;
        }
        let filter = Real::prepare_rational_linear_form4_filter([
            &plane.normal.x,
            &plane.normal.y,
            &plane.normal.z,
            &plane.offset,
        ]);
        cache.insert(CachedLinearForm3Filter {
            fingerprint: LinearForm3FilterCache::fingerprint(key),
            owners: coefficients.map(Clone::clone),
            filter,
        });
        filter
    })
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
pub fn classify_point(point: &Point3, plane: &Plane) -> HypermeshResult<Classification> {
    PreparedPoint3::new(point).classify(plane)
}

/// Borrowed point coordinates cached for repeated plane predicates.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PreparedPoint3<'a> {
    point: &'a Point3,
    exact_coordinates: Option<[&'a Rational; 3]>,
    rational_filter_query: Option<PreparedRationalLinearForm4Query>,
}

impl<'a> PreparedPoint3<'a> {
    /// Retains exact-rational coordinate facts without cloning scalar storage.
    pub(crate) fn new(point: &'a Point3) -> Self {
        let exact_coordinates = match (
            point.x.exact_rational_ref(),
            point.y.exact_rational_ref(),
            point.z.exact_rational_ref(),
        ) {
            (Some(x), Some(y), Some(z)) => Some([x, y, z]),
            _ => None,
        };
        crate::trace_dispatch!(
            "prepared-point3",
            if exact_coordinates.is_some() {
                "exact-rational"
            } else {
                "general-real"
            }
        );
        let rational_filter_query =
            exact_coordinates.and_then(Real::prepare_rational_affine_point3_query);
        if rational_filter_query.is_some() {
            crate::trace_dispatch!("prepared-point3", "rational-filter-query");
        }
        Self {
            point,
            exact_coordinates,
            rational_filter_query,
        }
    }

    /// Classifies the retained point against one plane.
    pub(crate) fn classify(&self, plane: &Plane) -> HypermeshResult<Classification> {
        if let Some(coordinates) = self.exact_coordinates
            && let Some(classification) = classify_exact_rational_coordinates(
                plane,
                coordinates,
                Rational::one_ref(),
                self.rational_filter_query.as_ref(),
            )
        {
            crate::trace_dispatch!("classify-point", "affine-exact-rational");
            return Ok(classification);
        }

        crate::trace_dispatch!("classify-point", "affine-real-fallback");
        classify_real(&plane.expression_at_point(self.point))
    }
}

pub(crate) fn classify_point_with_prepared_query(
    point: &Point3,
    plane: &Plane,
    rational_filter_query: Option<&PreparedRationalLinearForm4Query>,
) -> HypermeshResult<Classification> {
    let exact_coordinates = match (
        point.x.exact_rational_ref(),
        point.y.exact_rational_ref(),
        point.z.exact_rational_ref(),
    ) {
        (Some(x), Some(y), Some(z)) => Some([x, y, z]),
        _ => None,
    };
    PreparedPoint3 {
        point,
        exact_coordinates,
        rational_filter_query: rational_filter_query.copied(),
    }
    .classify(plane)
}

/// Classifies a homogeneous point against a plane.
pub fn classify_projective_point(
    point: &HomogeneousPoint3,
    plane: &Plane,
) -> HypermeshResult<Classification> {
    if let Some(weight) = point.w.exact_rational_ref()
        && let Some(classification) =
            classify_exact_rational_terms(plane, [&point.x, &point.y, &point.z], weight)
    {
        crate::trace_dispatch!("classify-point", "projective-exact-rational");
        return Ok(classification);
    }
    crate::trace_dispatch!("classify-point", "projective-real-fallback");
    classify_real(&homogeneous_point_plane_expression(point, plane))
}

/// Borrowed homogeneous coordinates cached for repeated plane predicates.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PreparedProjectivePoint3<'a> {
    point: &'a HomogeneousPoint3,
    exact_coordinates: Option<[&'a Rational; 4]>,
    rational_filter_query: Option<PreparedRationalLinearForm4Query>,
}

#[derive(Clone, Copy)]
pub(crate) struct PreparedRationalPlane4<'a> {
    coefficients: [&'a Rational; 4],
    filter: Option<PreparedRationalLinearForm4Filter>,
}

impl<'a> PreparedRationalPlane4<'a> {
    pub(crate) fn new(plane: &'a Plane) -> Option<Self> {
        let [Some(a), Some(b), Some(c), Some(d)] = [
            &plane.normal.x,
            &plane.normal.y,
            &plane.normal.z,
            &plane.offset,
        ]
        .map(Real::exact_rational_ref) else {
            return None;
        };
        let coefficients = [a, b, c, d];
        Some(Self {
            coefficients,
            filter: prepared_linear_form3_filter(plane, coefficients),
        })
    }
}

impl<'a> PreparedProjectivePoint3<'a> {
    pub(crate) fn new(point: &'a HomogeneousPoint3) -> Self {
        let mut prepared = Self::with_rational_filter_query(point, None);
        prepared.rational_filter_query = prepared
            .exact_coordinates
            .and_then(Real::prepare_rational_linear_form4_query);
        prepared
    }

    pub(crate) fn with_rational_filter_query(
        point: &'a HomogeneousPoint3,
        rational_filter_query: Option<PreparedRationalLinearForm4Query>,
    ) -> Self {
        let exact_coordinates = match (
            point.x.exact_rational_ref(),
            point.y.exact_rational_ref(),
            point.z.exact_rational_ref(),
            point.w.exact_rational_ref(),
        ) {
            (Some(x), Some(y), Some(z), Some(w)) => Some([x, y, z, w]),
            _ => None,
        };
        Self {
            point,
            exact_coordinates,
            rational_filter_query,
        }
    }

    pub(crate) fn rational_filter_query(&self) -> Option<PreparedRationalLinearForm4Query> {
        self.rational_filter_query
    }

    pub(crate) fn classify(&self, plane: &Plane) -> HypermeshResult<Classification> {
        if let Some([x, y, z, weight]) = self.exact_coordinates
            && let Some(classification) = classify_exact_rational_coordinates(
                plane,
                [x, y, z],
                weight,
                self.rational_filter_query.as_ref(),
            )
        {
            crate::trace_dispatch!("classify-point", "projective-exact-rational");
            return Ok(classification);
        }
        crate::trace_dispatch!("classify-point", "projective-real-fallback");
        classify_real(&homogeneous_point_plane_expression(self.point, plane))
    }

    #[inline]
    pub(crate) fn classify_rational_plane_filter(
        &self,
        plane: &PreparedRationalPlane4<'_>,
    ) -> Option<Classification> {
        let [x, y, z, weight] = self.exact_coordinates?;
        let sign = plane
            .filter
            .and_then(|filter| match &self.rational_filter_query {
                Some(query) => filter.sign_prepared(query),
                None => filter.sign_rational([x, y, z, weight]),
            })?;
        crate::trace_dispatch!("classify-point", "projective-rational-floating-filter");
        crate::trace_dispatch!("classify-point", "projective-exact-rational");
        Some(match sign {
            RealSign::Negative => Classification::Negative,
            RealSign::Zero => Classification::On,
            RealSign::Positive => Classification::Positive,
        })
    }

    pub(crate) fn classify_rational_plane_exact(
        &self,
        plane: &PreparedRationalPlane4<'_>,
    ) -> Option<Classification> {
        let [x, y, z, weight] = self.exact_coordinates?;
        let [a, b, c, d] = plane.coefficients;
        let ordering =
            Rational::signed_product_sum_ordering([true; 4], [[a, x], [b, y], [c, z], [d, weight]]);
        crate::trace_dispatch!(
            "classify-point",
            match ordering {
                Ordering::Less => "projective-rational-exact-fallback-negative",
                Ordering::Equal => "projective-rational-exact-fallback-on",
                Ordering::Greater => "projective-rational-exact-fallback-positive",
            }
        );
        crate::trace_dispatch!("classify-point", "projective-exact-rational");
        Some(match ordering {
            Ordering::Less => Classification::Negative,
            Ordering::Equal => Classification::On,
            Ordering::Greater => Classification::Positive,
        })
    }
}

fn classify_exact_rational_terms(
    plane: &Plane,
    coordinates: [&Real; 3],
    homogeneous_weight: &Rational,
) -> Option<Classification> {
    let [Some(x), Some(y), Some(z)] = coordinates.map(Real::exact_rational_ref) else {
        return None;
    };
    classify_exact_rational_coordinates(plane, [x, y, z], homogeneous_weight, None)
}

fn classify_exact_rational_coordinates(
    plane: &Plane,
    [x, y, z]: [&Rational; 3],
    homogeneous_weight: &Rational,
    prepared_query: Option<&PreparedRationalLinearForm4Query>,
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
        prepared_query,
        prepared_linear_form3_filter(plane, [a, b, c, d]),
    ))
}

fn classify_exact_rational_coordinates_with_filter(
    [a, b, c, d]: [&Rational; 4],
    [x, y, z]: [&Rational; 3],
    homogeneous_weight: &Rational,
    prepared_query: Option<&PreparedRationalLinearForm4Query>,
    filter: Option<PreparedRationalLinearForm4Filter>,
) -> Classification {
    let filtered_sign = filter.and_then(|filter| match prepared_query {
        Some(query) => filter.sign_prepared(query),
        None => filter.sign_rational([x, y, z, homogeneous_weight]),
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

/// Returns a certified ordering for two exact reals.
pub fn compare_real(left: &Real, right: &Real) -> HypermeshResult<Ordering> {
    if let (Some(left), Some(right)) = (left.exact_rational_ref(), right.exact_rational_ref()) {
        crate::trace_dispatch!("compare-real", "exact-rational");
        return Ok(left
            .partial_cmp(right)
            .expect("exact rationals are totally ordered"));
    }
    crate::trace_dispatch!("compare-real", "hyperlimit");
    match hyperlimit::compare_reals(left, right) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { .. } => Err(HypermeshError::UnknownClassification),
    }
}

pub(crate) fn classify_real(value: &Real) -> HypermeshResult<Classification> {
    crate::trace_dispatch!("classify-real", "hyperlimit");
    match classify_real_sign(value) {
        PredicateOutcome::Decided {
            value: Sign::Negative,
            ..
        } => Ok(Classification::Negative),
        PredicateOutcome::Decided {
            value: Sign::Zero, ..
        } => Ok(Classification::On),
        PredicateOutcome::Decided {
            value: Sign::Positive,
            ..
        } => Ok(Classification::Positive),
        PredicateOutcome::Unknown { .. } => Err(HypermeshError::UnknownClassification),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: Real, y: Real, z: Real) -> Point3 {
        Point3::new(x, y, z)
    }

    #[test]
    fn linear_form_filter_cache_retains_owners_and_probes_collisions() {
        let mut cache = LinearForm3FilterCache::default();
        let mut first_by_slot = std::collections::HashMap::new();
        let (first, second) = (1_i64..20_000)
            .find_map(|value| {
                let owners = std::array::from_fn(|offset| Rational::new(value + offset as i64));
                let key = owners.each_ref().map(|owner| owner.storage_identity());
                let slot = LinearForm3FilterCache::fingerprint(key) as usize
                    & (LINEAR_FORM_FILTER_SLOT_CAPACITY - 1);
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
        cache.insert(CachedLinearForm3Filter {
            fingerprint: LinearForm3FilterCache::fingerprint(first_key),
            owners: first_owners,
            filter: None,
        });
        cache.insert(CachedLinearForm3Filter {
            fingerprint: LinearForm3FilterCache::fingerprint(second_key),
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
            cache.insert(CachedLinearForm3Filter {
                fingerprint: LinearForm3FilterCache::fingerprint(key),
                owners,
                filter: None,
            });
            grown_keys.push(key);
        }
        assert!(cache.slots.len() > INITIAL_LINEAR_FORM_FILTER_SLOT_CAPACITY);
        assert!(
            grown_keys
                .into_iter()
                .all(|key| matches!(cache.find(key), Some(None)))
        );
    }

    #[test]
    fn prepared_point_matches_direct_exact_classification() {
        let point = point(Real::from(2), Real::from(3), Real::from(5));
        let prepared = PreparedPoint3::new(&point);
        let planes = [
            Plane::from_coefficients(Real::one(), Real::zero(), Real::zero(), Real::from(-2)),
            Plane::from_coefficients(Real::zero(), Real::one(), Real::zero(), Real::from(-4)),
        ];

        assert_eq!(prepared.classify(&planes[0]).unwrap(), Classification::On);
        assert_eq!(
            prepared.classify(&planes[1]).unwrap(),
            Classification::Negative
        );
        for plane in &planes {
            assert_eq!(prepared.classify(plane), classify_point(&point, plane));
        }
    }

    #[test]
    fn symbolic_coefficients_preserve_general_exact_fallback() {
        let point = point(Real::one(), Real::zero(), Real::zero());
        let plane =
            Plane::from_coefficients(Real::pi(), Real::zero(), Real::zero(), Real::from(-3));

        assert_eq!(
            classify_point(&point, &plane).unwrap(),
            Classification::Positive
        );
        assert_eq!(
            PreparedPoint3::new(&point).classify(&plane).unwrap(),
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
            classify_projective_point(&point, &plane).unwrap(),
            Classification::On
        );
    }

    #[test]
    fn prepared_projective_point_matches_repeated_plane_classification() {
        let point =
            HomogeneousPoint3::new(Real::from(6), Real::from(3), Real::zero(), Real::from(3));
        let prepared = PreparedProjectivePoint3::new(&point);
        let planes = [
            Plane::from_coefficients(Real::one(), Real::zero(), Real::zero(), Real::from(-2)),
            Plane::from_coefficients(Real::zero(), Real::one(), Real::zero(), Real::from(-2)),
            Plane::from_coefficients(Real::pi(), Real::zero(), Real::zero(), Real::from(-3)),
        ];

        assert_eq!(prepared.classify(&planes[0]).unwrap(), Classification::On);
        assert_eq!(
            prepared.classify(&planes[1]).unwrap(),
            Classification::Negative
        );
        assert_eq!(
            prepared.classify(&planes[2]).unwrap(),
            Classification::Positive
        );
        for plane in &planes {
            assert_eq!(
                prepared.classify(plane),
                classify_projective_point(&point, plane)
            );
        }
        for plane in &planes[..2] {
            let prepared_plane =
                PreparedRationalPlane4::new(plane).expect("integer plane is exactly rational");
            assert_eq!(
                prepared
                    .classify_rational_plane_filter(&prepared_plane)
                    .or_else(|| prepared.classify_rational_plane_exact(&prepared_plane)),
                Some(prepared.classify(plane).unwrap()),
            );
        }
        assert!(PreparedRationalPlane4::new(&planes[2]).is_none());
    }

    #[test]
    fn exact_real_comparison_matches_rational_ordering() {
        assert_eq!(
            compare_real(&Real::from(-3), &Real::from(2)).unwrap(),
            Ordering::Less,
        );
        assert_eq!(
            compare_real(&Real::from(5), &Real::from(5)).unwrap(),
            Ordering::Equal,
        );
    }
}
