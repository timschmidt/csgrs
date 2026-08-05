use std::collections::HashMap;
use std::collections::TryReserveError;
use std::hash::{Hash, Hasher};

use hyperlattice::{Point3, Rational, Real};

use crate::context::DecisionContext;
use crate::error::{HypermeshError, HypermeshResult};
use crate::storage_hash::{StorageHashMap, StorageIdentityHasher};

const BROAD_PHASE_PRECISION: i32 = -20;
const CELLS_PER_UNIT: i64 = 256;
const MAX_CELLS_PER_POINT: u64 = 64;
const OPERATION: &str = "point interning";

type CertifiedPointInterval = [[Rational; 2]; 3];
type CertifiedPointCells = [[i64; 2]; 3];

pub(crate) trait PointCoordinates {
    fn coordinates(&self) -> [&Real; 3];

    fn has_exact_rational_coordinates(&self) -> bool {
        self.coordinates()
            .into_iter()
            .all(|coordinate| coordinate.exact_rational_ref().is_some())
    }
}

impl PointCoordinates for Point3 {
    fn coordinates(&self) -> [&Real; 3] {
        [&self.x, &self.y, &self.z]
    }
}

impl<T> PointCoordinates for &T
where
    T: PointCoordinates + ?Sized,
{
    fn coordinates(&self) -> [&Real; 3] {
        (*self).coordinates()
    }
}

struct ExactPointProfile {
    storage: [usize; 3],
    fingerprint: u64,
}

pub(crate) struct PointInterner<I> {
    exact_only: bool,
    exact_storage: StorageHashMap<[usize; 3], usize>,
    exact_heads: StorageHashMap<u64, usize>,
    next_exact: Vec<Option<usize>>,
    intervals: Vec<Option<CertifiedPointInterval>>,
    cells: StorageHashMap<[i64; 3], Vec<usize>>,
    unbucketed: Vec<usize>,
    identities: Option<HashMap<I, usize>>,
    point_identities: Option<Vec<Option<I>>>,
    candidates: Vec<usize>,
    candidate_marks: Vec<u32>,
    candidate_epoch: u32,
}

impl<I> PointInterner<I>
where
    I: Clone + Eq + Hash,
{
    pub(crate) fn new_exact_unreserved() -> Self {
        Self::new_unreserved(true, false)
    }

    pub(crate) fn try_with_capacity(
        capacity: usize,
        exact_only: bool,
        retain_identities: bool,
    ) -> HypermeshResult<Self> {
        let mut interner = Self::new_unreserved(exact_only, retain_identities);
        interner.reserve_base(capacity)?;
        Ok(interner)
    }

    fn new_unreserved(exact_only: bool, retain_identities: bool) -> Self {
        Self {
            exact_only,
            exact_storage: StorageHashMap::default(),
            exact_heads: StorageHashMap::default(),
            next_exact: Vec::new(),
            intervals: Vec::new(),
            cells: StorageHashMap::default(),
            unbucketed: Vec::new(),
            identities: retain_identities.then(HashMap::new),
            point_identities: retain_identities.then(Vec::new),
            candidates: Vec::new(),
            candidate_marks: Vec::new(),
            candidate_epoch: 0,
        }
    }

    #[inline]
    pub(crate) fn intern_cloned<P>(
        &mut self,
        decisions: &DecisionContext,
        points: &mut Vec<P>,
        point: &P,
        identity: Option<I>,
    ) -> HypermeshResult<usize>
    where
        P: Clone + PointCoordinates,
    {
        self.intern_with(decisions, points, point, identity, || point.clone())
    }

    #[inline]
    pub(crate) fn intern_owned<P>(
        &mut self,
        decisions: &DecisionContext,
        points: &mut Vec<P>,
        point: P,
        identity: Option<I>,
    ) -> HypermeshResult<usize>
    where
        P: PointCoordinates,
    {
        if self.exact_only && identity.is_none() && point.has_exact_rational_coordinates() {
            return Ok(match self.find_exact(points, &point) {
                Some(index) => index,
                None => self.insert_exact(points, point)?,
            });
        }
        if self.exact_only && !point.has_exact_rational_coordinates() {
            self.promote_to_general(points)?;
        }
        if let Some(index) = self.find(decisions, points, &point, identity.as_ref())? {
            self.record_identity(identity, index)?;
            return Ok(index);
        }
        self.insert(points, point, identity)
    }

    /// Interns one exact-rational point pair as a single transaction.
    ///
    /// If either point is not exact rational, both are appended without a new
    /// equality decision. All fallible reservations precede mutation, so an
    /// allocation failure cannot leave a single endpoint behind.
    pub(crate) fn intern_exact_pair_or_append<P>(
        &mut self,
        points: &mut Vec<P>,
        pair: [P; 2],
    ) -> HypermeshResult<[usize; 2]>
    where
        P: PointCoordinates,
    {
        debug_assert!(self.exact_only);
        debug_assert!(self.identities.is_none());
        let [first, second] = pair;
        if first.has_exact_rational_coordinates() && second.has_exact_rational_coordinates() {
            reserve(points.try_reserve(2))?;
            reserve(self.next_exact.try_reserve(2))?;
            reserve(self.exact_storage.try_reserve(2))?;
            reserve(self.exact_heads.try_reserve(2))?;
            let first = self.intern_exact_prepared(points, first);
            let second = self.intern_exact_prepared(points, second);
            return Ok([first, second]);
        }

        reserve(points.try_reserve(2))?;
        reserve(self.next_exact.try_reserve(2))?;
        let first_index = points.len();
        points.push(first);
        self.next_exact.push(None);
        let second_index = points.len();
        points.push(second);
        self.next_exact.push(None);
        Ok([first_index, second_index])
    }

    /// Interns one exact-rational point or appends one general `Real` point.
    ///
    /// Exact-only intersection arenas do not introduce a policy-sensitive
    /// equality decision for symbolic points. This mirrors the pair operation
    /// above while avoiding a duplicate symbolic point solely to represent an
    /// isolated intersection event.
    pub(crate) fn intern_exact_or_append<P>(
        &mut self,
        points: &mut Vec<P>,
        point: P,
    ) -> HypermeshResult<usize>
    where
        P: PointCoordinates,
    {
        debug_assert!(self.exact_only);
        debug_assert!(self.identities.is_none());
        if point.has_exact_rational_coordinates() {
            reserve(points.try_reserve(1))?;
            reserve(self.next_exact.try_reserve(1))?;
            reserve(self.exact_storage.try_reserve(1))?;
            reserve(self.exact_heads.try_reserve(1))?;
            return Ok(self.intern_exact_prepared(points, point));
        }

        reserve(points.try_reserve(1))?;
        reserve(self.next_exact.try_reserve(1))?;
        let index = points.len();
        points.push(point);
        self.next_exact.push(None);
        Ok(index)
    }

    #[inline]
    pub(crate) fn intern_with<P, Q>(
        &mut self,
        decisions: &DecisionContext,
        points: &mut Vec<P>,
        point: &Q,
        identity: Option<I>,
        make_point: impl FnOnce() -> P,
    ) -> HypermeshResult<usize>
    where
        P: PointCoordinates,
        Q: PointCoordinates + ?Sized,
    {
        if self.exact_only && identity.is_none() && point.has_exact_rational_coordinates() {
            return Ok(match self.find_exact(points, point) {
                Some(index) => index,
                None => self.insert_exact(points, make_point())?,
            });
        }
        if self.exact_only && !point.has_exact_rational_coordinates() {
            self.promote_to_general(points)?;
        }
        if let Some(index) = self.find(decisions, points, point, identity.as_ref())? {
            self.record_identity(identity, index)?;
            return Ok(index);
        }
        self.insert(points, make_point(), identity)
    }

    #[inline]
    fn find_exact<P, Q>(&self, points: &[P], point: &Q) -> Option<usize>
    where
        P: PointCoordinates,
        Q: PointCoordinates + ?Sized,
    {
        let exact = exact_rational_coordinates(point)
            .expect("the exact-only interner path requires rational coordinates");
        let storage = exact.map(Rational::storage_identity);
        if let Some(&index) = self.exact_storage.get(&storage) {
            return Some(index);
        }
        let mut candidate = self.exact_heads.get(&exact_fingerprint(exact)).copied();
        while let Some(index) = candidate {
            if exact_coordinates_equal(&points[index], point) {
                return Some(index);
            }
            candidate = self.next_exact[index];
        }
        None
    }

    #[inline]
    fn insert_exact<P>(&mut self, points: &mut Vec<P>, point: P) -> HypermeshResult<usize>
    where
        P: PointCoordinates,
    {
        self.reserve_exact_point(points)?;
        Ok(self.insert_exact_prepared(points, point))
    }

    fn intern_exact_prepared<P>(&mut self, points: &mut Vec<P>, point: P) -> usize
    where
        P: PointCoordinates,
    {
        match self.find_exact(points, &point) {
            Some(index) => index,
            None => self.insert_exact_prepared(points, point),
        }
    }

    fn insert_exact_prepared<P>(&mut self, points: &mut Vec<P>, point: P) -> usize
    where
        P: PointCoordinates,
    {
        let index = points.len();
        points.push(point);
        let exact = exact_point_profile(&points[index])
            .expect("the exact-only interner path requires rational coordinates");
        self.exact_storage.entry(exact.storage).or_insert(index);
        self.next_exact
            .push(self.exact_heads.insert(exact.fingerprint, index));
        index
    }

    fn find<P, Q>(
        &mut self,
        decisions: &DecisionContext,
        points: &[P],
        point: &Q,
        identity: Option<&I>,
    ) -> HypermeshResult<Option<usize>>
    where
        P: PointCoordinates,
        Q: PointCoordinates + ?Sized,
    {
        if let (Some(identity), Some(identities)) = (identity, self.identities.as_ref())
            && let Some(&index) = identities.get(identity)
        {
            return Ok(Some(index));
        }

        let exact = exact_rational_coordinates(point);
        if let Some(exact) = exact {
            let storage = exact.map(Rational::storage_identity);
            if let Some(&index) = self.exact_storage.get(&storage)
                && self.identity_is_compatible(index, identity)
            {
                return Ok(Some(index));
            }

            let fingerprint = exact_fingerprint(exact);
            let mut candidate = self.exact_heads.get(&fingerprint).copied();
            while let Some(index) = candidate {
                if self.identity_is_compatible(index, identity)
                    && exact_coordinates_equal(&points[index], point)
                {
                    return Ok(Some(index));
                }
                candidate = self.next_exact[index];
            }
            if self.exact_only {
                return Ok(None);
            }
        }

        let interval = certified_point_interval(point);
        let cell_range = certified_point_cells(interval.as_ref());
        self.candidates.clear();
        if let Some(cells) = cell_range {
            reserve(self.candidates.try_reserve(points.len()))?;
            self.candidate_epoch = self.candidate_epoch.wrapping_add(1);
            if self.candidate_epoch == 0 {
                self.candidate_marks.fill(0);
                self.candidate_epoch = 1;
            }
            for cell in point_cells(cells) {
                if let Some(indices) = self.cells.get(&cell) {
                    for &index in indices {
                        if self.candidate_marks[index] != self.candidate_epoch {
                            self.candidate_marks[index] = self.candidate_epoch;
                            self.candidates.push(index);
                        }
                    }
                }
            }
            for &index in &self.unbucketed {
                if self.candidate_marks[index] != self.candidate_epoch {
                    self.candidate_marks[index] = self.candidate_epoch;
                    self.candidates.push(index);
                }
            }
            self.candidates.sort_unstable();
        } else {
            reserve(self.candidates.try_reserve(points.len()))?;
            self.candidates.extend(0..points.len());
        }

        let mut undecided = None;
        for &index in &self.candidates {
            if !self.identity_is_compatible(index, identity)
                || certified_point_intervals_are_disjoint(
                    self.intervals[index].as_ref(),
                    interval.as_ref(),
                )
            {
                continue;
            }
            match crate::predicate::coordinates3_equal(
                decisions,
                points[index].coordinates(),
                point.coordinates(),
            ) {
                Ok(true) => return Ok(Some(index)),
                Ok(false) => {}
                Err(error @ HypermeshError::PredicateUndecided { .. }) => {
                    undecided.get_or_insert(error);
                }
                Err(error) => return Err(error),
            }
        }
        match undecided {
            Some(error) => Err(error),
            None => Ok(None),
        }
    }

    fn insert<P>(
        &mut self,
        points: &mut Vec<P>,
        point: P,
        identity: Option<I>,
    ) -> HypermeshResult<usize>
    where
        P: PointCoordinates,
    {
        let index = points.len();
        self.reserve_point(points, identity.is_some())?;
        points.push(point);
        self.register_point(index, &points[index], identity)?;
        Ok(index)
    }

    fn register_point<P>(
        &mut self,
        index: usize,
        point: &P,
        identity: Option<I>,
    ) -> HypermeshResult<()>
    where
        P: PointCoordinates + ?Sized,
    {
        debug_assert_eq!(self.next_exact.len(), index);
        if let Some(exact) = exact_point_profile(point) {
            self.exact_storage.entry(exact.storage).or_insert(index);
            self.next_exact
                .push(self.exact_heads.insert(exact.fingerprint, index));
        } else {
            self.next_exact.push(None);
        }

        if !self.exact_only {
            let interval = certified_point_interval(point);
            let cells = certified_point_cells(interval.as_ref());
            self.intervals.push(interval);
            self.candidate_marks.push(0);
            self.register_cells(index, cells)?;
        }

        if let Some(point_identities) = self.point_identities.as_mut() {
            point_identities.push(identity.clone());
        } else {
            debug_assert!(identity.is_none());
        }
        self.record_identity(identity, index)
    }

    fn register_cells(
        &mut self,
        index: usize,
        cells: Option<CertifiedPointCells>,
    ) -> HypermeshResult<()> {
        if let Some(cells) = cells {
            let cell_count = point_cell_count(cells);
            reserve(self.cells.try_reserve(cell_count))?;
            for cell in point_cells(cells) {
                let candidates = self.cells.entry(cell).or_default();
                reserve(candidates.try_reserve(1))?;
                candidates.push(index);
            }
        } else {
            reserve(self.unbucketed.try_reserve(1))?;
            self.unbucketed.push(index);
        }
        Ok(())
    }

    fn promote_to_general<P>(&mut self, points: &[P]) -> HypermeshResult<()>
    where
        P: PointCoordinates,
    {
        debug_assert!(self.exact_only);
        self.exact_only = false;
        reserve(self.candidates.try_reserve(points.len()))?;
        reserve(self.intervals.try_reserve(points.len()))?;
        reserve(self.candidate_marks.try_reserve(points.len()))?;
        reserve(self.cells.try_reserve(points.len()))?;
        reserve(self.unbucketed.try_reserve(points.len()))?;
        for (index, point) in points.iter().enumerate() {
            let interval = certified_point_interval(point);
            let cells = certified_point_cells(interval.as_ref());
            self.intervals.push(interval);
            self.candidate_marks.push(0);
            self.register_cells(index, cells)?;
        }
        Ok(())
    }

    fn identity_is_compatible(&self, index: usize, identity: Option<&I>) -> bool {
        !matches!(
            (
                self.point_identities
                    .as_ref()
                    .and_then(|identities| identities[index].as_ref()),
                identity,
            ),
            (Some(existing), Some(incoming)) if existing != incoming
        )
    }

    fn record_identity(&mut self, identity: Option<I>, index: usize) -> HypermeshResult<()> {
        let Some(identity) = identity else {
            return Ok(());
        };
        let Some(identities) = self.identities.as_mut() else {
            debug_assert!(false, "identity supplied to identity-free point interner");
            return Ok(());
        };
        reserve(identities.try_reserve(1))?;
        identities.insert(identity, index);
        Ok(())
    }

    fn reserve_base(&mut self, capacity: usize) -> HypermeshResult<()> {
        reserve(self.exact_storage.try_reserve(capacity))?;
        reserve(self.exact_heads.try_reserve(capacity))?;
        reserve(self.next_exact.try_reserve(capacity))?;
        if !self.exact_only {
            reserve(self.candidates.try_reserve(capacity))?;
            reserve(self.intervals.try_reserve(capacity))?;
            reserve(self.candidate_marks.try_reserve(capacity))?;
            reserve(self.cells.try_reserve(capacity))?;
            reserve(self.unbucketed.try_reserve(capacity))?;
        }
        if let Some(identities) = self.identities.as_mut() {
            reserve(identities.try_reserve(capacity))?;
        }
        if let Some(point_identities) = self.point_identities.as_mut() {
            reserve(point_identities.try_reserve(capacity))?;
        }
        Ok(())
    }

    fn reserve_point<P>(&mut self, points: &mut Vec<P>, has_identity: bool) -> HypermeshResult<()> {
        self.reserve_exact_point(points)?;
        if !self.exact_only {
            reserve(self.intervals.try_reserve(1))?;
            reserve(self.candidate_marks.try_reserve(1))?;
        }
        if let Some(point_identities) = self.point_identities.as_mut() {
            reserve(point_identities.try_reserve(1))?;
        }
        if has_identity && let Some(identities) = self.identities.as_mut() {
            reserve(identities.try_reserve(1))?;
        }
        Ok(())
    }

    fn reserve_exact_point<P>(&mut self, points: &mut Vec<P>) -> HypermeshResult<()> {
        reserve(points.try_reserve(1))?;
        reserve(self.next_exact.try_reserve(1))?;
        reserve(self.exact_storage.try_reserve(1))?;
        reserve(self.exact_heads.try_reserve(1))?;
        Ok(())
    }
}

fn reserve(result: Result<(), TryReserveError>) -> HypermeshResult<()> {
    result.map_err(|_| HypermeshError::CapacityOverflow {
        operation: OPERATION,
    })
}

fn exact_point_profile<P>(point: &P) -> Option<ExactPointProfile>
where
    P: PointCoordinates + ?Sized,
{
    let exact = exact_rational_coordinates(point)?;
    Some(ExactPointProfile {
        storage: exact.map(Rational::storage_identity),
        fingerprint: exact_fingerprint(exact),
    })
}

fn exact_rational_coordinates<P>(point: &P) -> Option<[&Rational; 3]>
where
    P: PointCoordinates + ?Sized,
{
    let exact = point.coordinates().map(Real::exact_rational_ref);
    Some([exact[0]?, exact[1]?, exact[2]?])
}

fn exact_fingerprint(exact: [&Rational; 3]) -> u64 {
    let mut hasher = StorageIdentityHasher::default();
    exact.hash(&mut hasher);
    hasher.finish()
}

fn exact_coordinates_equal<P, Q>(left: &P, right: &Q) -> bool
where
    P: PointCoordinates + ?Sized,
    Q: PointCoordinates + ?Sized,
{
    left.coordinates()
        .into_iter()
        .zip(right.coordinates())
        .all(|(left, right)| left.exact_rational_ref() == right.exact_rational_ref())
}

fn certified_point_interval<P>(point: &P) -> Option<CertifiedPointInterval>
where
    P: PointCoordinates + ?Sized,
{
    let [x, y, z] = point.coordinates();
    Some([
        x.certified_dyadic_interval(BROAD_PHASE_PRECISION)?,
        y.certified_dyadic_interval(BROAD_PHASE_PRECISION)?,
        z.certified_dyadic_interval(BROAD_PHASE_PRECISION)?,
    ])
}

fn certified_point_intervals_are_disjoint(
    left: Option<&CertifiedPointInterval>,
    right: Option<&CertifiedPointInterval>,
) -> bool {
    let (Some(left), Some(right)) = (left, right) else {
        return false;
    };
    left.iter()
        .zip(right)
        .any(|(left, right)| left[1] < right[0] || right[1] < left[0])
}

fn rational_floor_i64(value: &Rational) -> Option<i64> {
    let truncated = value.trunc();
    let mut integer = i64::try_from(truncated.clone()).ok()?;
    if value.is_negative() && truncated != *value {
        integer = integer.checked_sub(1)?;
    }
    Some(integer)
}

fn certified_point_cells(interval: Option<&CertifiedPointInterval>) -> Option<CertifiedPointCells> {
    let interval = interval?;
    let scale = Rational::from(CELLS_PER_UNIT);
    let mut cells = [[0; 2]; 3];
    let mut cell_count = 1_u64;
    for axis in 0..3 {
        cells[axis] = [
            rational_floor_i64(&(&interval[axis][0] * &scale))?,
            rational_floor_i64(&(&interval[axis][1] * &scale))?,
        ];
        let axis_count = cells[axis][1].checked_sub(cells[axis][0])?.checked_add(1)?;
        cell_count = cell_count.checked_mul(u64::try_from(axis_count).ok()?)?;
        if cell_count > MAX_CELLS_PER_POINT {
            return None;
        }
    }
    Some(cells)
}

fn point_cells(cells: CertifiedPointCells) -> impl Iterator<Item = [i64; 3]> {
    (cells[0][0]..=cells[0][1]).flat_map(move |x| {
        (cells[1][0]..=cells[1][1])
            .flat_map(move |y| (cells[2][0]..=cells[2][1]).map(move |z| [x, y, z]))
    })
}

fn point_cell_count(cells: CertifiedPointCells) -> usize {
    cells
        .map(|range| {
            usize::try_from(range[1] - range[0] + 1)
                .expect("certified point cell ranges are positive and bounded")
        })
        .into_iter()
        .product()
}

#[cfg(test)]
mod tests {
    use hyperlattice::{Point3, Rational, Real};
    use hyperlimit::PredicatePolicy;

    use super::PointInterner;
    use crate::context::{DecisionContext, MeshCertainty, MeshContext};
    use crate::error::HypermeshError;

    fn point(x: Real) -> Point3 {
        Point3::new(x, Real::zero(), Real::zero())
    }

    fn terminal_zero() -> Real {
        (Real::pi() + Real::e()) - (Real::e() + Real::pi())
    }

    fn interner_from_unique(points: &[Point3]) -> PointInterner<()> {
        let mut interner = PointInterner::try_with_capacity(points.len(), false, false).unwrap();
        for (index, point) in points.iter().enumerate() {
            interner.register_point(index, point, None).unwrap();
        }
        interner
    }

    #[test]
    fn exact_fingerprint_separates_values_with_identical_f64_keys() {
        let tiny = Rational::fraction(1, 1_u64 << 60).unwrap();
        let one = point(Real::one());
        let rounded_to_one = point(Real::from(Rational::one() + tiny));
        assert_eq!(
            one.x.to_f64_lossy().map(f64::to_bits),
            rounded_to_one.x.to_f64_lossy().map(f64::to_bits)
        );

        let mut points = Vec::new();
        let mut interner = PointInterner::<()>::try_with_capacity(2, true, false).unwrap();
        let first = interner
            .intern_cloned(
                &crate::test_support::approximate_decisions(),
                &mut points,
                &one,
                None,
            )
            .unwrap();
        let second = interner
            .intern_cloned(
                &crate::test_support::approximate_decisions(),
                &mut points,
                &rounded_to_one,
                None,
            )
            .unwrap();

        assert_eq!(first, 0);
        assert_eq!(second, 1);
        assert_eq!(points.len(), 2);
    }

    #[test]
    fn exact_fingerprint_merges_equal_values_with_distinct_storage() {
        let first = point(Real::from(Rational::fraction(1, 2).unwrap()));
        let second = point(Real::from(
            Rational::fraction(3, 6).unwrap() + Rational::zero(),
        ));
        assert_ne!(
            first.x.exact_rational_ref().unwrap().storage_identity(),
            second.x.exact_rational_ref().unwrap().storage_identity()
        );

        let mut points = Vec::new();
        let mut interner = PointInterner::<()>::try_with_capacity(2, true, false).unwrap();
        let first = interner
            .intern_cloned(
                &crate::test_support::approximate_decisions(),
                &mut points,
                &first,
                None,
            )
            .unwrap();
        let second = interner
            .intern_cloned(
                &crate::test_support::approximate_decisions(),
                &mut points,
                &second,
                None,
            )
            .unwrap();

        assert_eq!(first, second);
        assert_eq!(points.len(), 1);
    }

    #[test]
    fn general_interner_promotes_without_losing_exact_points() {
        let exact = point(Real::from(2));
        let symbolic = point(Real::from(2).sqrt().unwrap());

        let mut points = Vec::new();
        let mut interner = PointInterner::<()>::try_with_capacity(2, true, false).unwrap();
        assert_eq!(interner.candidates.capacity(), 0);
        assert_eq!(interner.intervals.capacity(), 0);
        assert!(interner.cells.is_empty());
        interner
            .intern_cloned(
                &crate::test_support::approximate_decisions(),
                &mut points,
                &exact,
                None,
            )
            .unwrap();
        interner
            .intern_cloned(
                &crate::test_support::approximate_decisions(),
                &mut points,
                &symbolic,
                None,
            )
            .unwrap();

        assert_eq!(points.len(), 2);
        assert!(!interner.exact_only);
        assert!(interner.candidates.capacity() >= 1);
        assert_eq!(interner.intervals.len(), 2);
        assert_eq!(interner.candidate_marks.len(), 2);
    }

    #[test]
    fn certified_intervals_only_reject_provably_distinct_points() {
        let left = point(Real::pi() + Real::e());
        let equivalent = point(Real::e() + Real::pi());
        let distinct = point(Real::pi());
        let left_interval = super::certified_point_interval(&left);
        let equivalent_interval = super::certified_point_interval(&equivalent);
        let distinct_interval = super::certified_point_interval(&distinct);

        assert!(!super::certified_point_intervals_are_disjoint(
            left_interval.as_ref(),
            equivalent_interval.as_ref(),
        ));
        assert!(super::certified_point_intervals_are_disjoint(
            left_interval.as_ref(),
            distinct_interval.as_ref(),
        ));
    }

    #[test]
    fn undecided_candidate_does_not_hide_a_later_structural_match() {
        let zero = point(Real::zero());
        let retained = point(terminal_zero());
        let mut points = vec![zero, retained.clone()];
        let mut interner = interner_from_unique(&points);
        let context = MeshContext::new(PredicatePolicy::STRICT);
        let decisions = DecisionContext::new(&context);

        let index = interner
            .intern_cloned(&decisions, &mut points, &retained, None)
            .unwrap();

        assert_eq!(index, 1);
        assert_eq!(points.len(), 2);
        assert_eq!(decisions.certainty(), MeshCertainty::Certified);
    }

    #[test]
    fn construction_identity_precedes_policy_aware_numeric_equality() {
        let zero = point(Real::zero());
        let terminal = point(terminal_zero());
        let context = MeshContext::new(PredicatePolicy::STRICT);
        let decisions = DecisionContext::new(&context);
        let mut points = Vec::new();
        let mut interner = PointInterner::<u8>::try_with_capacity(2, false, true).unwrap();

        assert_eq!(
            interner
                .intern_cloned(&decisions, &mut points, &zero, Some(1))
                .unwrap(),
            0
        );
        assert_eq!(
            interner
                .intern_cloned(&decisions, &mut points, &terminal, Some(1))
                .unwrap(),
            0
        );
        assert_eq!(
            interner
                .intern_cloned(&decisions, &mut points, &zero, Some(2))
                .unwrap(),
            1
        );
        assert_eq!(points.len(), 2);
        assert_eq!(decisions.certainty(), MeshCertainty::Certified);
    }

    #[test]
    fn candidate_epoch_wrap_preserves_general_matches() {
        let retained = point(terminal_zero());
        let mut points = vec![retained.clone()];
        let mut interner = interner_from_unique(&points);
        interner.candidate_epoch = u32::MAX;
        interner.candidate_marks.fill(u32::MAX);

        let context = MeshContext::new(PredicatePolicy::STRICT);
        let decisions = DecisionContext::new(&context);
        assert_eq!(
            interner
                .intern_cloned(&decisions, &mut points, &retained, None)
                .unwrap(),
            0
        );
        assert_eq!(interner.candidate_epoch, 1);
        assert_eq!(points.len(), 1);
    }

    #[test]
    fn terminal_equality_obeys_policy_and_updates_aggregate_certainty() {
        let zero = point(Real::zero());
        let terminal = point(terminal_zero());

        let strict_context = MeshContext::new(PredicatePolicy::STRICT);
        let strict = DecisionContext::new(&strict_context);
        let mut strict_points = vec![zero.clone()];
        let mut strict_interner = interner_from_unique(&strict_points);
        assert!(matches!(
            strict_interner.intern_cloned(&strict, &mut strict_points, &terminal, None),
            Err(HypermeshError::PredicateUndecided { .. })
        ));
        assert_eq!(strict_points, vec![zero.clone()]);
        assert_eq!(strict.certainty(), MeshCertainty::Certified);

        let approximate_context = MeshContext::new(PredicatePolicy::APPROXIMATE_512);
        let approximate = DecisionContext::new(&approximate_context);
        let mut approximate_points = vec![zero];
        let mut approximate_interner = interner_from_unique(&approximate_points);
        assert_eq!(
            approximate_interner
                .intern_cloned(&approximate, &mut approximate_points, &terminal, None,)
                .unwrap(),
            0
        );
        assert_eq!(approximate_points.len(), 1);
        assert_eq!(
            approximate.certainty(),
            MeshCertainty::Approximate512Consumed
        );
    }

    #[test]
    fn exact_prefix_promotion_preserves_terminal_policy() {
        let zero = point(Real::zero());
        let terminal = point(terminal_zero());

        for (policy, expected) in [
            (PredicatePolicy::STRICT, None),
            (PredicatePolicy::APPROXIMATE_512, Some(0)),
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let mut points = Vec::new();
            let mut interner = PointInterner::<()>::try_with_capacity(2, true, false).unwrap();
            assert_eq!(
                interner
                    .intern_cloned(&decisions, &mut points, &zero, None)
                    .unwrap(),
                0
            );

            let result = interner.intern_cloned(&decisions, &mut points, &terminal, None);
            match expected {
                None => assert!(matches!(
                    result,
                    Err(HypermeshError::PredicateUndecided { .. })
                )),
                Some(index) => assert_eq!(result.unwrap(), index),
            }
            assert!(!interner.exact_only);
            assert_eq!(points.len(), 1);
            assert_eq!(
                decisions.certainty(),
                if policy == PredicatePolicy::STRICT {
                    MeshCertainty::Certified
                } else {
                    MeshCertainty::Approximate512Consumed
                }
            );
        }
    }
}
