//! Exact 3D bounds for broad-phase scheduling.
//!
//! AABB facts are acceleration facts, not topology certificates. An exact box
//! can prove that two objects are disjoint; otherwise the pair must continue to
//! a `hyperlimit` narrow-phase predicate before topology changes. Cheap bounds
//! may schedule work, but certified predicates decide combinatorics.

use std::cmp::Ordering;

use hyperlimit::{
    Aabb3Intersection, Point3, PredicateOutcome, classify_aabb3_intersection, compare_reals,
};
use hyperreal::Real;

/// Exact broad-phase relation between two 3D boxes.
pub type AabbIntersectionKind = Aabb3Intersection;

/// Structural inconsistency in retained exact bounds.
///
/// Bounds are object-level acceleration facts, not topology certificates.
/// They must replay before they can reject or retain topological work.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoundsValidationError {
    /// An axis minimum is certified greater than its maximum.
    InvertedAxis,
    /// An axis minimum/maximum relation could not be certified.
    UnknownAxisOrder,
    /// Mesh-level bounds are missing for a nonempty vertex set.
    MissingMeshBounds,
    /// Mesh-level bounds exist for an empty vertex set.
    UnexpectedMeshBounds,
    /// The retained face-bound vector length does not match the face count.
    FaceBoundsCountMismatch,
    /// Recomputing bounds from the supplied source vertices and triangles did
    /// not reproduce the retained bounds object.
    SourceReplayMismatch,
}

/// Exact 3D axis-aligned bounding box.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactAabb3 {
    /// Minimum corner.
    pub min: Point3,
    /// Maximum corner.
    pub max: Point3,
}

impl ExactAabb3 {
    /// Build an exact box around one point.
    pub fn point(point: &Point3) -> Self {
        Self {
            min: point.clone(),
            max: point.clone(),
        }
    }

    /// Build an exact box around a nonempty point slice.
    pub fn from_points(points: &[Point3]) -> Option<Self> {
        let first = points.first()?;
        let mut bounds = Self::point(first);
        for point in &points[1..] {
            bounds.include(point);
        }
        Some(bounds)
    }

    /// Build an exact box around one triangle.
    pub fn from_triangle(points: [&Point3; 3]) -> Self {
        let mut bounds = Self::point(points[0]);
        bounds.include(points[1]);
        bounds.include(points[2]);
        bounds
    }

    /// Expand the box to include one point.
    pub fn include(&mut self, point: &Point3) {
        include_axis(&mut self.min.x, &mut self.max.x, &point.x);
        include_axis(&mut self.min.y, &mut self.max.y, &point.y);
        include_axis(&mut self.min.z, &mut self.max.z, &point.z);
    }

    /// Classify this box against another exact box.
    ///
    /// `Disjoint` is a certified broad-phase rejection. `Touching`,
    /// `Overlapping`, and [`PredicateOutcome::Unknown`] must be treated as
    /// candidates for exact narrow-phase predicates before topology changes.
    pub fn classify_intersection(&self, other: &Self) -> PredicateOutcome<AabbIntersectionKind> {
        classify_aabb3_intersection(&self.min, &self.max, &other.min, &other.max)
    }

    /// Validate that each retained axis interval is ordered.
    ///
    /// Unknown comparisons are rejected here because a bounds object with an
    /// uncertified min/max ordering cannot safely serve as an exact broad-phase
    /// fact for later predicate scheduling.
    pub fn validate(&self) -> Result<(), BoundsValidationError> {
        for (min, max) in [
            (&self.min.x, &self.max.x),
            (&self.min.y, &self.max.y),
            (&self.min.z, &self.max.z),
        ] {
            match compare(min, max) {
                Some(Ordering::Less | Ordering::Equal) => {}
                Some(Ordering::Greater) => return Err(BoundsValidationError::InvertedAxis),
                None => return Err(BoundsValidationError::UnknownAxisOrder),
            }
        }
        Ok(())
    }

    /// Validate this box against the source points it summarizes.
    ///
    /// Local validation proves only that each interval is ordered. Source
    /// replay rebuilds the box from the exact points and requires equality
    /// before the box may act as broad-phase evidence. This is the bounds-level
    /// object summary can schedule predicate work only while it still replays
    /// from the exact object it summarizes.
    pub fn validate_against_points(&self, points: &[Point3]) -> Result<(), BoundsValidationError> {
        self.validate()?;
        let replay = Self::from_points(points).ok_or(BoundsValidationError::MissingMeshBounds)?;
        if self == &replay {
            Ok(())
        } else {
            Err(BoundsValidationError::SourceReplayMismatch)
        }
    }

    /// Validate this box against one source triangle.
    ///
    /// This is the per-face counterpart to [`Self::validate_against_points`].
    /// It lets callers audit retained face AABBs directly before broad-phase
    /// face-pair scheduling consumes them.
    pub fn validate_against_triangle(
        &self,
        points: [&Point3; 3],
    ) -> Result<(), BoundsValidationError> {
        self.validate()?;
        let replay = Self::from_triangle(points);
        if self == &replay {
            Ok(())
        } else {
            Err(BoundsValidationError::SourceReplayMismatch)
        }
    }
}

/// Retained mesh and face bounds.
#[derive(Clone, Debug, PartialEq)]
pub struct MeshBounds {
    /// Whole-mesh bounds, or `None` for an empty mesh.
    pub mesh: Option<ExactAabb3>,
    /// Per-face bounds in face order.
    pub faces: Vec<ExactAabb3>,
}

/// Exact broad-phase face ordering prepared for repeated pair queries.
///
/// This borrows retained source bounds and caches axis interval views and sort
/// orders. It is an acceleration fact, not topology evidence: disjoint AABBs
/// may reject work, while retained pairs still require exact narrow-phase
/// predicates.
#[derive(Clone, Debug)]
pub(crate) struct PreparedMeshBounds<'a> {
    bounds: &'a MeshBounds,
    axis_intervals: [Vec<FaceAxisInterval<'a>>; 3],
    min_axis_orders: [Option<Vec<usize>>; 3],
    max_axis_orders: [Option<Vec<usize>>; 3],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    const ALL: [Self; 3] = [Self::X, Self::Y, Self::Z];

    const fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
            Self::Z => 2,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SweepDirection {
    LeftDriven,
    RightDriven,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct SweepPlan {
    axis: Axis,
    direction: SweepDirection,
    interval_pairs: usize,
    cost: usize,
}

#[derive(Clone, Copy, Debug)]
struct FaceAxisInterval<'a> {
    min: &'a Real,
    max: &'a Real,
}

/// Prepared broad-phase plan for one retained bounds pair.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct CandidateFacePairPlan {
    mesh_bounds_overlap: bool,
    sweep: Option<SweepPlan>,
    capacity_hint: usize,
}

impl CandidateFacePairPlan {
    const fn empty() -> Self {
        Self {
            mesh_bounds_overlap: false,
            sweep: None,
            capacity_hint: 0,
        }
    }

    pub(crate) const fn capacity_hint(self) -> usize {
        self.capacity_hint
    }
}

impl MeshBounds {
    /// Build retained bounds from predicate points and triangle indices.
    pub fn from_triangles(points: &[Point3], triangles: &[[usize; 3]]) -> Self {
        let mesh = ExactAabb3::from_points(points);
        let faces = triangles
            .iter()
            .map(|tri| {
                ExactAabb3::from_triangle([&points[tri[0]], &points[tri[1]], &points[tri[2]]])
            })
            .collect();
        Self { mesh, faces }
    }

    /// Return face-pair candidates whose exact boxes are not disjoint.
    #[cfg(test)]
    pub(crate) fn candidate_face_pairs(&self, other: &Self) -> Vec<[usize; 2]> {
        self.prepare().candidate_face_pairs(&other.prepare())
    }

    /// Prepare exact axis intervals and face orders for repeated broad-phase queries.
    ///
    /// An axis order is retained only when all exact comparisons needed for
    /// sorting were decided. Querying two prepared bounds falls back to the
    /// exact quadratic scheduler when no common sweep axis is usable.
    pub(crate) fn prepare(&self) -> PreparedMeshBounds<'_> {
        let axis_intervals = [
            face_axis_intervals(&self.faces, Axis::X),
            face_axis_intervals(&self.faces, Axis::Y),
            face_axis_intervals(&self.faces, Axis::Z),
        ];
        let min_axis_orders = [
            sorted_face_indices_by_axis_bound(&axis_intervals[Axis::X.index()], AxisBound::Min),
            sorted_face_indices_by_axis_bound(&axis_intervals[Axis::Y.index()], AxisBound::Min),
            sorted_face_indices_by_axis_bound(&axis_intervals[Axis::Z.index()], AxisBound::Min),
        ];
        let max_axis_orders = [
            sorted_face_indices_by_axis_bound(&axis_intervals[Axis::X.index()], AxisBound::Max),
            sorted_face_indices_by_axis_bound(&axis_intervals[Axis::Y.index()], AxisBound::Max),
            sorted_face_indices_by_axis_bound(&axis_intervals[Axis::Z.index()], AxisBound::Max),
        ];
        PreparedMeshBounds {
            bounds: self,
            axis_intervals,
            min_axis_orders,
            max_axis_orders,
        }
    }
}

impl<'a> PreparedMeshBounds<'a> {
    /// Return the retained bounds object this prepared scheduler borrows.
    /// Return face-pair candidates whose exact boxes are not disjoint.
    #[cfg(test)]
    pub(crate) fn candidate_face_pairs(&self, other: &PreparedMeshBounds<'_>) -> Vec<[usize; 2]> {
        let mut pairs = Vec::with_capacity(self.candidate_face_pair_capacity_hint(other));
        let result = self.try_visit_candidate_face_pairs(other, |pair| {
            pairs.push(pair);
            Ok::<(), ()>(())
        });
        debug_assert!(result.is_ok());
        pairs.sort_unstable();
        pairs
    }

    /// Return an upper bound for broad-phase candidate face pairs.
    ///
    /// When a certified sweep plan is available, this is the selected
    /// one-dimensional interval overlap count. Otherwise it falls back to the
    /// quadratic face-pair product. The final full-AABB filter may emit fewer
    /// pairs.
    pub(crate) fn candidate_face_pair_capacity_hint(
        &self,
        other: &PreparedMeshBounds<'_>,
    ) -> usize {
        self.candidate_face_pair_plan(other).capacity_hint()
    }

    pub(crate) fn candidate_face_pair_plan(
        &self,
        other: &PreparedMeshBounds<'_>,
    ) -> CandidateFacePairPlan {
        if !self.mesh_bounds_may_overlap(other) {
            return CandidateFacePairPlan::empty();
        }
        if let Some(sweep) = self.best_sweep_plan(other) {
            return CandidateFacePairPlan {
                mesh_bounds_overlap: true,
                sweep: Some(sweep),
                capacity_hint: sweep.interval_pairs,
            };
        }
        CandidateFacePairPlan {
            mesh_bounds_overlap: true,
            sweep: None,
            capacity_hint: self
                .bounds
                .faces
                .len()
                .saturating_mul(other.bounds.faces.len()),
        }
    }

    pub(crate) fn try_visit_candidate_face_pairs<E>(
        &self,
        other: &PreparedMeshBounds<'_>,
        mut visit: impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<(), E> {
        let plan = self.candidate_face_pair_plan(other);
        self.try_visit_candidate_face_pairs_with_plan(other, plan, &mut visit)
    }

    pub(crate) fn try_visit_candidate_face_pairs_with_plan<E>(
        &self,
        other: &PreparedMeshBounds<'_>,
        plan: CandidateFacePairPlan,
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<(), E> {
        if !plan.mesh_bounds_overlap {
            return Ok(());
        }
        let Some(sweep) = plan.sweep else {
            return self.try_visit_candidate_face_pairs_quadratic(other, visit);
        };
        let used_sweep = match sweep.direction {
            SweepDirection::LeftDriven => {
                self.try_visit_candidate_face_pairs_sweep_axis(other, sweep.axis, visit)?
            }
            SweepDirection::RightDriven => {
                other.try_visit_candidate_face_pairs_sweep_axis(self, sweep.axis, &mut |pair| {
                    visit([pair[1], pair[0]])
                })?
            }
        };
        if !used_sweep {
            return self.try_visit_candidate_face_pairs_quadratic(other, visit);
        }
        Ok(())
    }

    fn mesh_bounds_may_overlap(&self, other: &PreparedMeshBounds<'_>) -> bool {
        match (&self.bounds.mesh, &other.bounds.mesh) {
            (Some(left), Some(right)) => must_keep_candidate(left.classify_intersection(right)),
            _ => false,
        }
    }

    #[cfg(test)]
    fn candidate_face_pairs_quadratic(&self, other: &PreparedMeshBounds<'_>) -> Vec<[usize; 2]> {
        let mut pairs = Vec::new();
        let result = self.try_visit_candidate_face_pairs_quadratic(other, &mut |pair| {
            pairs.push(pair);
            Ok::<(), ()>(())
        });
        debug_assert!(result.is_ok());
        pairs
    }

    fn try_visit_candidate_face_pairs_quadratic<E>(
        &self,
        other: &PreparedMeshBounds<'_>,
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<(), E> {
        for (left, left_box) in self.bounds.faces.iter().enumerate() {
            for (right, right_box) in other.bounds.faces.iter().enumerate() {
                if must_keep_candidate(left_box.classify_intersection(right_box)) {
                    visit([left, right])?;
                }
            }
        }
        Ok(())
    }

    fn best_sweep_plan(&self, other: &PreparedMeshBounds<'_>) -> Option<SweepPlan> {
        let mut best_plan = None;
        for axis in [Axis::X, Axis::Y, Axis::Z] {
            best_plan = choose_better_sweep_plan(
                best_plan,
                self.sweep_plan_for_axis(other, axis, SweepDirection::LeftDriven),
            );
            best_plan = choose_better_sweep_plan(
                best_plan,
                other.sweep_plan_for_axis(self, axis, SweepDirection::RightDriven),
            );
        }
        best_plan
    }

    fn sweep_plan_for_axis(
        &self,
        other: &PreparedMeshBounds<'_>,
        axis: Axis,
        direction: SweepDirection,
    ) -> Option<SweepPlan> {
        let interval_pairs = self.interval_candidate_pair_count_sweep_axis(other, axis)?;
        let cost = interval_pairs.checked_add(self.bounds.faces.len())?;
        Some(SweepPlan {
            axis,
            direction,
            interval_pairs,
            cost,
        })
    }

    fn interval_candidate_pair_count_sweep_axis(
        &self,
        other: &PreparedMeshBounds<'_>,
        axis: Axis,
    ) -> Option<usize> {
        let left_min_order = self.min_axis_order(axis)?;
        let left_max_order = self.max_axis_order(axis)?;
        let right_min_order = other.min_axis_order(axis)?;
        let right_max_order = other.max_axis_order(axis)?;
        let starts_before_left_ends = count_ordered_axis_bounds(
            left_max_order,
            self.axis_intervals(axis),
            AxisBound::Max,
            right_min_order,
            other.axis_intervals(axis),
            AxisBound::Min,
            AxisBoundCount::LessOrEqual,
        )?;
        let ends_before_left_starts = count_ordered_axis_bounds(
            left_min_order,
            self.axis_intervals(axis),
            AxisBound::Min,
            right_max_order,
            other.axis_intervals(axis),
            AxisBound::Max,
            AxisBoundCount::Less,
        )?;
        starts_before_left_ends.checked_sub(ends_before_left_starts)
    }

    fn try_visit_candidate_face_pairs_sweep_axis<E>(
        &self,
        other: &PreparedMeshBounds<'_>,
        axis: Axis,
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<bool, E> {
        let Some(left_order) = self.min_axis_order(axis) else {
            return Ok(false);
        };
        let Some(right_order) = other.min_axis_order(axis) else {
            return Ok(false);
        };
        let Some(right_max_order) = other.max_axis_order(axis) else {
            return Ok(false);
        };
        let mut active_right = Vec::<usize>::new();
        let mut right_active = vec![false; other.bounds.faces.len()];
        let mut next_right = 0usize;
        let mut next_expiring_right = 0usize;
        let mut inactive_rights = 0usize;

        for &left in left_order {
            let left_interval = self.axis_interval(axis, left);
            while let Some(&right) = right_max_order.get(next_expiring_right) {
                let Some(ordering) =
                    compare(other.axis_interval(axis, right).max, left_interval.min)
                else {
                    return Ok(false);
                };
                if ordering != Ordering::Less {
                    break;
                }
                if right_active[right] {
                    right_active[right] = false;
                    inactive_rights += 1;
                }
                next_expiring_right += 1;
            }

            if inactive_rights > active_right.len() / 2 {
                active_right.retain(|&right| right_active[right]);
                inactive_rights = 0;
            }

            while let Some(&right) = right_order.get(next_right) {
                let right_interval = other.axis_interval(axis, right);
                let Some(ordering) = compare(right_interval.min, left_interval.max) else {
                    return Ok(false);
                };
                if ordering == Ordering::Greater {
                    break;
                }
                let Some(ordering) = compare(right_interval.max, left_interval.min) else {
                    return Ok(false);
                };
                if ordering != Ordering::Less {
                    active_right.push(right);
                    right_active[right] = true;
                }
                next_right += 1;
            }

            for &right in &active_right {
                let right_interval = other.axis_interval(axis, right);
                let Some(ordering) = compare(right_interval.min, left_interval.max) else {
                    return Ok(false);
                };
                if ordering == Ordering::Greater {
                    break;
                }
                if !right_active[right] {
                    continue;
                }
                let pair = [left, right];
                if self.full_aabb_may_overlap_on_remaining_axes(other, pair, axis) {
                    visit(pair)?;
                }
            }
        }

        Ok(true)
    }

    fn full_aabb_may_overlap_on_remaining_axes(
        &self,
        other: &PreparedMeshBounds<'_>,
        pair: [usize; 2],
        sweep_axis: Axis,
    ) -> bool {
        let [left, right] = pair;
        Axis::ALL
            .into_iter()
            .filter(|&axis| axis != sweep_axis)
            .all(|axis| {
                axis_intervals_may_overlap(
                    self.axis_interval(axis, left),
                    other.axis_interval(axis, right),
                )
            })
    }

    fn min_axis_order(&self, axis: Axis) -> Option<&[usize]> {
        self.min_axis_orders[axis.index()].as_deref()
    }

    fn max_axis_order(&self, axis: Axis) -> Option<&[usize]> {
        self.max_axis_orders[axis.index()].as_deref()
    }

    fn axis_intervals(&self, axis: Axis) -> &[FaceAxisInterval<'a>] {
        &self.axis_intervals[axis.index()]
    }

    fn axis_interval(&self, axis: Axis, face: usize) -> FaceAxisInterval<'a> {
        self.axis_intervals(axis)[face]
    }
}

impl MeshBounds {
    /// Validate retained mesh and face bounds against expected topology sizes.
    ///
    /// This validates only the bounds object shape and interval ordering. It
    /// does not recompute bounds from vertices; construction code owns that
    /// stronger check when it builds [`MeshBounds`] from exact points.
    pub fn validate(
        &self,
        vertex_count: usize,
        face_count: usize,
    ) -> Result<(), BoundsValidationError> {
        match (&self.mesh, vertex_count) {
            (Some(bounds), 1..) => bounds.validate()?,
            (None, 0) => {}
            (None, _) => return Err(BoundsValidationError::MissingMeshBounds),
            (Some(_), 0) => return Err(BoundsValidationError::UnexpectedMeshBounds),
        }
        if self.faces.len() != face_count {
            return Err(BoundsValidationError::FaceBoundsCountMismatch);
        }
        for face in &self.faces {
            face.validate()?;
        }
        Ok(())
    }

    /// Validate retained bounds against the source points and triangle rows.
    ///
    /// Local validation proves only interval ordering and table shape. Source
    /// replay rebuilds the mesh and per-face AABBs from exact source geometry
    /// broad-phase facts may schedule or reject work only while they still
    /// replay from the exact objects they summarize.
    pub fn validate_against_sources(
        &self,
        points: &[Point3],
        triangles: &[[usize; 3]],
    ) -> Result<(), BoundsValidationError> {
        self.validate_against_triangle_rows(points, triangles.len(), triangles.iter().copied())
    }

    pub(crate) fn validate_against_triangle_rows(
        &self,
        points: &[Point3],
        triangle_count: usize,
        triangles: impl IntoIterator<Item = [usize; 3]>,
    ) -> Result<(), BoundsValidationError> {
        self.validate(points.len(), triangle_count)?;
        let mut replay = Self {
            mesh: ExactAabb3::from_points(points),
            faces: Vec::with_capacity(triangle_count),
        };
        for triangle in triangles {
            if triangle.iter().any(|&vertex| vertex >= points.len()) {
                return Err(BoundsValidationError::SourceReplayMismatch);
            }
            replay.faces.push(ExactAabb3::from_triangle([
                &points[triangle[0]],
                &points[triangle[1]],
                &points[triangle[2]],
            ]));
        }
        if self == &replay {
            Ok(())
        } else {
            Err(BoundsValidationError::SourceReplayMismatch)
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum AxisBound {
    Min,
    Max,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum AxisBoundCount {
    Less,
    LessOrEqual,
}

fn face_axis_intervals(faces: &[ExactAabb3], axis: Axis) -> Vec<FaceAxisInterval<'_>> {
    faces
        .iter()
        .map(|bounds| FaceAxisInterval {
            min: axis_min(bounds, axis),
            max: axis_max(bounds, axis),
        })
        .collect()
}

fn sorted_face_indices_by_axis_bound(
    intervals: &[FaceAxisInterval<'_>],
    bound: AxisBound,
) -> Option<Vec<usize>> {
    let mut decided = true;
    let mut indices = (0..intervals.len()).collect::<Vec<_>>();
    indices.sort_by(|&left, &right| {
        match compare(
            axis_bound(intervals[left], bound),
            axis_bound(intervals[right], bound),
        ) {
            Some(ordering) => ordering,
            None => {
                decided = false;
                Ordering::Equal
            }
        }
    });
    decided.then_some(indices)
}

fn choose_better_sweep_plan(
    current: Option<SweepPlan>,
    candidate: Option<SweepPlan>,
) -> Option<SweepPlan> {
    match (current, candidate) {
        (None, candidate) => candidate,
        (Some(current), None) => Some(current),
        (Some(current), Some(candidate)) => {
            if candidate.cost < current.cost {
                Some(candidate)
            } else {
                Some(current)
            }
        }
    }
}

fn count_ordered_axis_bounds(
    query_order: &[usize],
    query_intervals: &[FaceAxisInterval<'_>],
    query_bound: AxisBound,
    value_order: &[usize],
    value_intervals: &[FaceAxisInterval<'_>],
    value_bound: AxisBound,
    count: AxisBoundCount,
) -> Option<usize> {
    let mut total = 0usize;
    let mut values_before_query = 0usize;
    for &query in query_order {
        let query_value = axis_bound(query_intervals[query], query_bound);
        while let Some(&value) = value_order.get(values_before_query) {
            let ordering = compare(axis_bound(value_intervals[value], value_bound), query_value)?;
            let retain_value = match count {
                AxisBoundCount::Less => ordering == Ordering::Less,
                AxisBoundCount::LessOrEqual => ordering != Ordering::Greater,
            };
            if !retain_value {
                break;
            }
            values_before_query += 1;
        }
        total = total.checked_add(values_before_query)?;
    }
    Some(total)
}

fn axis_bound(interval: FaceAxisInterval<'_>, bound: AxisBound) -> &Real {
    match bound {
        AxisBound::Min => interval.min,
        AxisBound::Max => interval.max,
    }
}

fn axis_intervals_may_overlap(left: FaceAxisInterval<'_>, right: FaceAxisInterval<'_>) -> bool {
    !matches!(compare(left.max, right.min), Some(Ordering::Less))
        && !matches!(compare(right.max, left.min), Some(Ordering::Less))
}

fn axis_min(bounds: &ExactAabb3, axis: Axis) -> &Real {
    match axis {
        Axis::X => &bounds.min.x,
        Axis::Y => &bounds.min.y,
        Axis::Z => &bounds.min.z,
    }
}

fn axis_max(bounds: &ExactAabb3, axis: Axis) -> &Real {
    match axis {
        Axis::X => &bounds.max.x,
        Axis::Y => &bounds.max.y,
        Axis::Z => &bounds.max.z,
    }
}

fn include_axis(min: &mut Real, max: &mut Real, value: &Real) {
    if matches!(compare(value, min), Some(Ordering::Less)) {
        *min = value.clone();
    }
    if matches!(compare(value, max), Some(Ordering::Greater)) {
        *max = value.clone();
    }
}

fn compare(left: &Real, right: &Real) -> Option<Ordering> {
    compare_reals(left, right).value()
}

fn must_keep_candidate(outcome: PredicateOutcome<AabbIntersectionKind>) -> bool {
    match outcome {
        PredicateOutcome::Decided { value, .. } => value.needs_narrow_phase(),
        PredicateOutcome::Unknown { .. } => true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn candidate_face_pairs_prune_certified_disjoint_bounds() {
        let left_points = vec![
            p(0, 0, 0),
            p(2, 0, 0),
            p(0, 2, 0),
            p(10, 0, 0),
            p(12, 0, 0),
            p(10, 2, 0),
        ];
        let right_points = vec![
            p(1, 0, 0),
            p(3, 0, 0),
            p(1, 2, 0),
            p(20, 0, 0),
            p(22, 0, 0),
            p(20, 2, 0),
        ];
        let triangles = [[0, 1, 2], [3, 4, 5]];
        let left = MeshBounds::from_triangles(&left_points, &triangles);
        let right = MeshBounds::from_triangles(&right_points, &triangles);

        assert_eq!(left.candidate_face_pairs(&right), vec![[0, 0]]);
    }

    #[test]
    fn candidate_face_pairs_keep_exact_touching_bounds() {
        let left_points = vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0)];
        let right_points = vec![p(2, 0, 0), p(4, 0, 0), p(2, 2, 0)];
        let triangles = [[0, 1, 2]];
        let left = MeshBounds::from_triangles(&left_points, &triangles);
        let right = MeshBounds::from_triangles(&right_points, &triangles);

        assert_eq!(left.candidate_face_pairs(&right), vec![[0, 0]]);
    }

    #[test]
    fn candidate_face_pairs_can_prune_on_non_x_axis() {
        let left_points = vec![
            p(0, 0, 0),
            p(10, 0, 0),
            p(0, 1, 0),
            p(0, 10, 0),
            p(10, 10, 0),
            p(0, 11, 0),
        ];
        let right_points = vec![
            p(0, 10, 0),
            p(10, 10, 0),
            p(0, 11, 0),
            p(0, 20, 0),
            p(10, 20, 0),
            p(0, 21, 0),
        ];
        let triangles = [[0, 1, 2], [3, 4, 5]];
        let left = MeshBounds::from_triangles(&left_points, &triangles);
        let right = MeshBounds::from_triangles(&right_points, &triangles);

        assert_eq!(left.candidate_face_pairs(&right), vec![[1, 0]]);
    }

    #[test]
    fn prepared_sweep_matches_quadratic_candidates() {
        let left_points = vec![
            p(0, 0, 0),
            p(5, 0, 0),
            p(0, 5, 0),
            p(10, 10, 0),
            p(15, 10, 0),
            p(10, 15, 0),
            p(20, 0, 0),
            p(25, 0, 0),
            p(20, 5, 0),
        ];
        let right_points = vec![
            p(4, 4, 0),
            p(9, 4, 0),
            p(4, 9, 0),
            p(12, 12, 0),
            p(17, 12, 0),
            p(12, 17, 0),
            p(30, 0, 0),
            p(35, 0, 0),
            p(30, 5, 0),
        ];
        let triangles = [[0, 1, 2], [3, 4, 5], [6, 7, 8]];
        let left = MeshBounds::from_triangles(&left_points, &triangles);
        let right = MeshBounds::from_triangles(&right_points, &triangles);
        let prepared_left = left.prepare();
        let prepared_right = right.prepare();

        assert_eq!(
            prepared_left.candidate_face_pairs(&prepared_right),
            prepared_left.candidate_face_pairs_quadratic(&prepared_right)
        );
        assert!(
            prepared_left.candidate_face_pair_capacity_hint(&prepared_right)
                >= prepared_left.candidate_face_pairs(&prepared_right).len()
        );
    }

    #[test]
    fn prepared_sweep_expires_active_intervals_by_max_order() {
        let left_points = vec![
            p(0, 0, 0),
            p(100, 0, 0),
            p(0, 1, 0),
            p(50, 0, 0),
            p(51, 0, 0),
            p(50, 1, 0),
            p(80, 0, 0),
            p(81, 0, 0),
            p(80, 1, 0),
        ];
        let right_points = vec![
            p(1, 0, 0),
            p(2, 0, 0),
            p(1, 1, 0),
            p(3, 0, 0),
            p(4, 0, 0),
            p(3, 1, 0),
            p(50, 0, 0),
            p(51, 0, 0),
            p(50, 1, 0),
            p(90, 0, 0),
            p(91, 0, 0),
            p(90, 1, 0),
        ];
        let left_triangles = [[0, 1, 2], [3, 4, 5], [6, 7, 8]];
        let right_triangles = [[0, 1, 2], [3, 4, 5], [6, 7, 8], [9, 10, 11]];
        let left = MeshBounds::from_triangles(&left_points, &left_triangles);
        let right = MeshBounds::from_triangles(&right_points, &right_triangles);
        let prepared_left = left.prepare();
        let prepared_right = right.prepare();

        assert_eq!(
            prepared_left.candidate_face_pairs(&prepared_right),
            prepared_left.candidate_face_pairs_quadratic(&prepared_right)
        );
    }

    #[test]
    fn prepared_sweep_can_drive_from_smaller_right_side() {
        let left_points = vec![
            p(0, 0, 0),
            p(10, 0, 0),
            p(0, 10, 0),
            p(0, 0, 1),
            p(10, 0, 1),
            p(0, 10, 1),
            p(0, 0, 2),
            p(10, 0, 2),
            p(0, 10, 2),
            p(0, 0, 3),
            p(10, 0, 3),
            p(0, 10, 3),
        ];
        let right_points = vec![p(0, 0, 0), p(10, 0, 0), p(0, 10, 0)];
        let left_triangles = [[0, 1, 2], [3, 4, 5], [6, 7, 8], [9, 10, 11]];
        let right_triangles = [[0, 1, 2]];
        let left = MeshBounds::from_triangles(&left_points, &left_triangles);
        let right = MeshBounds::from_triangles(&right_points, &right_triangles);
        let prepared_left = left.prepare();
        let prepared_right = right.prepare();

        assert_eq!(
            prepared_left
                .best_sweep_plan(&prepared_right)
                .unwrap()
                .direction,
            SweepDirection::RightDriven
        );
        assert_eq!(
            prepared_left.candidate_face_pairs(&prepared_right),
            prepared_left.candidate_face_pairs_quadratic(&prepared_right)
        );
        assert!(
            prepared_left.candidate_face_pair_capacity_hint(&prepared_right)
                >= prepared_left.candidate_face_pairs(&prepared_right).len()
        );
    }
}
