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
pub(crate) type AabbIntersectionKind = Aabb3Intersection;

/// Structural inconsistency in retained exact bounds.
///
/// Bounds are object-level acceleration facts, not topology certificates.
/// They must replay before they can reject or retain topological work.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BoundsValidationError {
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
pub(crate) struct ExactAabb3 {
    /// Minimum corner.
    pub min: Point3,
    /// Maximum corner.
    pub max: Point3,
}

impl ExactAabb3 {
    /// Build an exact box around one point.
    pub(crate) fn point(point: &Point3) -> Self {
        Self {
            min: point.clone(),
            max: point.clone(),
        }
    }

    /// Build an exact box around a nonempty point slice.
    pub(crate) fn from_points(points: &[Point3]) -> Option<Self> {
        let first = points.first()?;
        let mut bounds = Self::point(first);
        for point in &points[1..] {
            bounds.include(point);
        }
        Some(bounds)
    }

    /// Build an exact box around one triangle.
    pub(crate) fn from_triangle(points: [&Point3; 3]) -> Self {
        let mut bounds = Self::point(points[0]);
        bounds.include(points[1]);
        bounds.include(points[2]);
        bounds
    }

    /// Expand the box to include one point.
    pub(crate) fn include(&mut self, point: &Point3) {
        include_axis(&mut self.min.x, &mut self.max.x, &point.x);
        include_axis(&mut self.min.y, &mut self.max.y, &point.y);
        include_axis(&mut self.min.z, &mut self.max.z, &point.z);
    }

    /// Classify this box against another exact box.
    ///
    /// `Disjoint` is a certified broad-phase rejection. `Touching`,
    /// `Overlapping`, and [`PredicateOutcome::Unknown`] must be treated as
    /// candidates for exact narrow-phase predicates before topology changes.
    pub(crate) fn classify_intersection(
        &self,
        other: &Self,
    ) -> PredicateOutcome<AabbIntersectionKind> {
        classify_aabb3_intersection(&self.min, &self.max, &other.min, &other.max)
    }

    /// Validate that each retained axis interval is ordered.
    ///
    /// Unknown comparisons are rejected here because a bounds object with an
    /// uncertified min/max ordering cannot safely serve as an exact broad-phase
    /// fact for later predicate scheduling.
    pub(crate) fn validate(&self) -> Result<(), BoundsValidationError> {
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
}

/// Retained mesh and face bounds.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct MeshBounds {
    /// Whole-mesh bounds, or `None` for an empty mesh.
    mesh: Option<ExactAabb3>,
    /// Per-face bounds in face order.
    faces: Vec<ExactAabb3>,
}

/// Exact broad-phase face ordering prepared for repeated pair queries.
///
/// This borrows retained source bounds and caches exact sort orders. It is an
/// acceleration fact, not topology evidence: disjoint AABBs may reject work,
/// while retained pairs still require exact narrow-phase predicates.
#[derive(Debug)]
pub(crate) struct PreparedMeshBounds<'a> {
    bounds: &'a MeshBounds,
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
pub(crate) struct SweepPlan {
    axis: Axis,
    direction: SweepDirection,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct SweepPlanEstimate {
    plan: SweepPlan,
    axis_pair_count: usize,
    driver_face_count: usize,
    active_face_capacity_hint: usize,
}

impl SweepPlanEstimate {
    const fn is_better_than(self, other: Self) -> bool {
        self.axis_pair_count < other.axis_pair_count
            || (self.axis_pair_count == other.axis_pair_count
                && self.driver_face_count < other.driver_face_count)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct AxisOverlapEstimate {
    pair_count: usize,
    max_target_active: usize,
}

#[derive(Clone, Copy, Debug)]
struct FaceAxisInterval<'a> {
    min: &'a Real,
    max: &'a Real,
}

/// Prepared broad-phase plan for one retained bounds pair.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum CandidateFacePairPlan {
    Empty,
    Sweep {
        plan: SweepPlan,
        active_face_capacity_hint: usize,
    },
    Quadratic,
}

impl CandidateFacePairPlan {
    const fn empty() -> Self {
        Self::Empty
    }
}

impl MeshBounds {
    /// Build retained bounds from predicate points and triangle indices.
    #[cfg(test)]
    pub(crate) fn from_triangles(points: &[Point3], triangles: &[[usize; 3]]) -> Self {
        Self::from_triangle_rows(points, triangles.len(), triangles.iter().copied())
    }

    pub(crate) fn from_triangle_rows(
        points: &[Point3],
        triangle_count: usize,
        triangles: impl IntoIterator<Item = [usize; 3]>,
    ) -> Self {
        let mesh = ExactAabb3::from_points(points);
        let mut faces = Vec::with_capacity(triangle_count);
        for tri in triangles {
            faces.push(ExactAabb3::from_triangle([
                &points[tri[0]],
                &points[tri[1]],
                &points[tri[2]],
            ]));
        }
        Self { mesh, faces }
    }

    /// Return retained whole-mesh bounds, or `None` for an empty mesh.
    pub(crate) fn mesh(&self) -> Option<&ExactAabb3> {
        self.mesh.as_ref()
    }

    /// Prepare exact face orders for repeated broad-phase queries.
    ///
    /// An axis order is retained only when all exact comparisons needed for
    /// sorting were decided. Querying two prepared bounds falls back to the
    /// exact quadratic scheduler when no common sweep axis is usable.
    pub(crate) fn prepare(&self) -> PreparedMeshBounds<'_> {
        let min_axis_orders = [
            sorted_face_indices_by_axis_bound(&self.faces, Axis::X, AxisBound::Min),
            sorted_face_indices_by_axis_bound(&self.faces, Axis::Y, AxisBound::Min),
            sorted_face_indices_by_axis_bound(&self.faces, Axis::Z, AxisBound::Min),
        ];
        let max_axis_orders = [
            sorted_face_indices_by_axis_bound(&self.faces, Axis::X, AxisBound::Max),
            sorted_face_indices_by_axis_bound(&self.faces, Axis::Y, AxisBound::Max),
            sorted_face_indices_by_axis_bound(&self.faces, Axis::Z, AxisBound::Max),
        ];
        PreparedMeshBounds {
            bounds: self,
            min_axis_orders,
            max_axis_orders,
        }
    }
}

impl<'a> PreparedMeshBounds<'a> {
    pub(crate) fn candidate_face_pair_plan(
        &self,
        other: &PreparedMeshBounds<'_>,
    ) -> CandidateFacePairPlan {
        if !self.mesh_bounds_may_overlap(other) {
            return CandidateFacePairPlan::empty();
        }
        if let Some(sweep) = self.sweep_plan(other) {
            return CandidateFacePairPlan::Sweep {
                plan: sweep.plan,
                active_face_capacity_hint: sweep.active_face_capacity_hint,
            };
        }
        CandidateFacePairPlan::Quadratic
    }

    pub(crate) fn try_visit_candidate_face_pairs_with_plan<E>(
        &self,
        other: &PreparedMeshBounds<'_>,
        plan: CandidateFacePairPlan,
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<(), E> {
        let (sweep_plan, active_face_capacity_hint) = match plan {
            CandidateFacePairPlan::Empty => return Ok(()),
            CandidateFacePairPlan::Quadratic => {
                return self.try_visit_candidate_face_pairs_quadratic(other, visit);
            }
            CandidateFacePairPlan::Sweep {
                plan,
                active_face_capacity_hint,
            } => (plan, active_face_capacity_hint),
        };
        let used_sweep = match sweep_plan.direction {
            SweepDirection::LeftDriven => self.try_visit_candidate_face_pairs_sweep_axis(
                other,
                sweep_plan.axis,
                active_face_capacity_hint,
                visit,
            )?,
            SweepDirection::RightDriven => other.try_visit_candidate_face_pairs_sweep_axis(
                self,
                sweep_plan.axis,
                active_face_capacity_hint,
                &mut |pair| visit([pair[1], pair[0]]),
            )?,
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

    fn sweep_plan(&self, other: &PreparedMeshBounds<'_>) -> Option<SweepPlanEstimate> {
        let directions = if self.bounds.faces.len() <= other.bounds.faces.len() {
            [SweepDirection::LeftDriven, SweepDirection::RightDriven]
        } else {
            [SweepDirection::RightDriven, SweepDirection::LeftDriven]
        };
        let mut best = None::<SweepPlanEstimate>;
        for direction in directions {
            for axis in Axis::ALL {
                let Some(estimate) = self.estimate_sweep_plan(other, axis, direction) else {
                    continue;
                };
                if best.is_none_or(|best| estimate.is_better_than(best)) {
                    best = Some(estimate);
                }
            }
        }
        best
    }

    fn estimate_sweep_plan(
        &self,
        other: &PreparedMeshBounds<'_>,
        axis: Axis,
        direction: SweepDirection,
    ) -> Option<SweepPlanEstimate> {
        if !self.sweep_axis_is_usable(other, axis, direction) {
            return None;
        }
        let (driver, target) = match direction {
            SweepDirection::LeftDriven => (self, other),
            SweepDirection::RightDriven => (other, self),
        };
        let estimate = driver.axis_interval_overlap_estimate(target, axis)?;
        Some(SweepPlanEstimate {
            plan: SweepPlan { axis, direction },
            axis_pair_count: estimate.pair_count,
            driver_face_count: driver.bounds.faces.len(),
            active_face_capacity_hint: estimate.max_target_active,
        })
    }

    fn sweep_axis_is_usable(
        &self,
        other: &PreparedMeshBounds<'_>,
        axis: Axis,
        direction: SweepDirection,
    ) -> bool {
        match direction {
            SweepDirection::LeftDriven => {
                self.min_axis_order(axis).is_some()
                    && other.min_axis_order(axis).is_some()
                    && other.max_axis_order(axis).is_some()
            }
            SweepDirection::RightDriven => {
                other.min_axis_order(axis).is_some()
                    && self.min_axis_order(axis).is_some()
                    && self.max_axis_order(axis).is_some()
            }
        }
    }

    fn axis_interval_overlap_estimate(
        &self,
        other: &PreparedMeshBounds<'_>,
        axis: Axis,
    ) -> Option<AxisOverlapEstimate> {
        let other_min_order = other.min_axis_order(axis)?;
        let other_max_order = other.max_axis_order(axis)?;
        let mut pair_count = 0usize;
        let mut max_target_active = 0usize;

        for driver_interval in self.face_axis_intervals(axis) {
            let started = upper_bound_axis_bound(
                other_min_order,
                other.bounds.faces.as_slice(),
                axis,
                AxisBound::Min,
                driver_interval.max,
            )?;
            let ended = lower_bound_axis_bound(
                other_max_order,
                other.bounds.faces.as_slice(),
                axis,
                AxisBound::Max,
                driver_interval.min,
            )?;
            let active = started.saturating_sub(ended);
            pair_count = pair_count.saturating_add(active);
            max_target_active = max_target_active.max(active);
        }

        Some(AxisOverlapEstimate {
            pair_count,
            max_target_active,
        })
    }

    fn try_visit_candidate_face_pairs_sweep_axis<E>(
        &self,
        other: &PreparedMeshBounds<'_>,
        axis: Axis,
        active_face_capacity_hint: usize,
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<bool, E> {
        let Some(left_order) = self.min_axis_order(axis) else {
            return Ok(false);
        };
        let Some(right_order) = other.min_axis_order(axis) else {
            return Ok(false);
        };
        if should_use_sparse_sweep(active_face_capacity_hint, other.bounds.faces.len()) {
            return self.try_visit_candidate_face_pairs_sparse_sweep_axis(
                other,
                axis,
                active_face_capacity_hint,
                left_order,
                right_order,
                visit,
            );
        }
        self.try_visit_candidate_face_pairs_marked_sweep_axis(
            other,
            axis,
            active_face_capacity_hint,
            left_order,
            right_order,
            visit,
        )
    }

    fn try_visit_candidate_face_pairs_marked_sweep_axis<E>(
        &self,
        other: &PreparedMeshBounds<'_>,
        axis: Axis,
        active_face_capacity_hint: usize,
        left_order: &[usize],
        right_order: &[usize],
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<bool, E> {
        let Some(right_max_order) = other.max_axis_order(axis) else {
            return Ok(false);
        };
        let active_right_capacity = active_face_capacity_hint.min(other.bounds.faces.len());
        let mut active_right = Vec::<usize>::with_capacity(active_right_capacity);
        let mut right_active = vec![0u8; other.bounds.faces.len()];
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
                if right_active[right] != 0 {
                    right_active[right] = 0;
                    inactive_rights += 1;
                }
                next_expiring_right += 1;
            }

            if inactive_rights > active_right.len() / 2 {
                active_right.retain(|&right| right_active[right] != 0);
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
                    right_active[right] = 1;
                }
                next_right += 1;
            }

            for &right in &active_right {
                if right_active[right] == 0 {
                    continue;
                }
                let right_interval = other.axis_interval(axis, right);
                let Some(ordering) = compare(right_interval.min, left_interval.max) else {
                    return Ok(false);
                };
                if ordering == Ordering::Greater {
                    break;
                }
                let pair = [left, right];
                if self.full_aabb_may_overlap_on_remaining_axes(other, pair, axis) {
                    visit(pair)?;
                }
            }
        }

        Ok(true)
    }

    fn try_visit_candidate_face_pairs_sparse_sweep_axis<E>(
        &self,
        other: &PreparedMeshBounds<'_>,
        axis: Axis,
        active_face_capacity_hint: usize,
        left_order: &[usize],
        right_order: &[usize],
        visit: &mut impl FnMut([usize; 2]) -> Result<(), E>,
    ) -> Result<bool, E> {
        let active_right_capacity = active_face_capacity_hint.min(other.bounds.faces.len());
        let mut active_right = Vec::<usize>::with_capacity(active_right_capacity);
        let mut next_right = 0usize;

        for &left in left_order {
            let left_interval = self.axis_interval(axis, left);
            let mut retained = 0usize;
            for read in 0..active_right.len() {
                let right = active_right[read];
                let Some(ordering) =
                    compare(other.axis_interval(axis, right).max, left_interval.min)
                else {
                    return Ok(false);
                };
                if ordering != Ordering::Less {
                    active_right[retained] = right;
                    retained += 1;
                }
            }
            active_right.truncate(retained);

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

    fn face_axis_intervals(&self, axis: Axis) -> impl Iterator<Item = FaceAxisInterval<'a>> + '_ {
        self.bounds
            .faces
            .iter()
            .map(move |bounds| face_axis_interval(bounds, axis))
    }

    fn axis_interval(&self, axis: Axis, face: usize) -> FaceAxisInterval<'a> {
        face_axis_interval(&self.bounds.faces[face], axis)
    }
}

impl MeshBounds {
    /// Validate retained mesh and face bounds against expected topology sizes.
    ///
    /// This validates only the bounds object shape and interval ordering. It
    /// does not recompute bounds from vertices; construction code owns that
    /// stronger check when it builds [`MeshBounds`] from exact points.
    pub(crate) fn validate(
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

fn face_axis_interval(bounds: &ExactAabb3, axis: Axis) -> FaceAxisInterval<'_> {
    FaceAxisInterval {
        min: axis_min(bounds, axis),
        max: axis_max(bounds, axis),
    }
}

fn sorted_face_indices_by_axis_bound(
    faces: &[ExactAabb3],
    axis: Axis,
    bound: AxisBound,
) -> Option<Vec<usize>> {
    let mut decided = true;
    let mut indices = (0..faces.len()).collect::<Vec<_>>();
    indices.sort_by(|&left, &right| {
        match compare(
            axis_bound(&faces[left], axis, bound),
            axis_bound(&faces[right], axis, bound),
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

fn axis_bound(bounds: &ExactAabb3, axis: Axis, bound: AxisBound) -> &Real {
    match bound {
        AxisBound::Min => axis_min(bounds, axis),
        AxisBound::Max => axis_max(bounds, axis),
    }
}

fn lower_bound_axis_bound(
    order: &[usize],
    faces: &[ExactAabb3],
    axis: Axis,
    bound: AxisBound,
    value: &Real,
) -> Option<usize> {
    let mut start = 0usize;
    let mut end = order.len();
    while start < end {
        let mid = start + (end - start) / 2;
        let ordering = compare(axis_bound(&faces[order[mid]], axis, bound), value)?;
        if ordering == Ordering::Less {
            start = mid + 1;
        } else {
            end = mid;
        }
    }
    Some(start)
}

fn upper_bound_axis_bound(
    order: &[usize],
    faces: &[ExactAabb3],
    axis: Axis,
    bound: AxisBound,
    value: &Real,
) -> Option<usize> {
    let mut start = 0usize;
    let mut end = order.len();
    while start < end {
        let mid = start + (end - start) / 2;
        let ordering = compare(axis_bound(&faces[order[mid]], axis, bound), value)?;
        if ordering == Ordering::Greater {
            end = mid;
        } else {
            start = mid + 1;
        }
    }
    Some(start)
}

const fn should_use_sparse_sweep(
    active_face_capacity_hint: usize,
    target_face_count: usize,
) -> bool {
    active_face_capacity_hint.saturating_mul(4) < target_face_count
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

    fn sorted_pairs(mut pairs: Vec<[usize; 2]>) -> Vec<[usize; 2]> {
        pairs.sort_unstable();
        pairs
    }

    fn candidate_face_pairs(left: &MeshBounds, right: &MeshBounds) -> Vec<[usize; 2]> {
        prepared_candidate_face_pairs(&left.prepare(), &right.prepare())
    }

    fn prepared_candidate_face_pairs(
        left: &PreparedMeshBounds<'_>,
        right: &PreparedMeshBounds<'_>,
    ) -> Vec<[usize; 2]> {
        let mut pairs = Vec::new();
        let plan = left.candidate_face_pair_plan(right);
        let result = left.try_visit_candidate_face_pairs_with_plan(right, plan, &mut |pair| {
            pairs.push(pair);
            Ok::<(), ()>(())
        });
        debug_assert!(result.is_ok());
        pairs
    }

    fn quadratic_candidate_face_pairs(
        left: &PreparedMeshBounds<'_>,
        right: &PreparedMeshBounds<'_>,
    ) -> Vec<[usize; 2]> {
        let mut pairs = Vec::new();
        let result = left.try_visit_candidate_face_pairs_quadratic(right, &mut |pair| {
            pairs.push(pair);
            Ok::<(), ()>(())
        });
        debug_assert!(result.is_ok());
        pairs
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

        assert_eq!(candidate_face_pairs(&left, &right), vec![[0, 0]]);
    }

    #[test]
    fn candidate_face_pairs_keep_exact_touching_bounds() {
        let left_points = vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0)];
        let right_points = vec![p(2, 0, 0), p(4, 0, 0), p(2, 2, 0)];
        let triangles = [[0, 1, 2]];
        let left = MeshBounds::from_triangles(&left_points, &triangles);
        let right = MeshBounds::from_triangles(&right_points, &triangles);

        assert_eq!(candidate_face_pairs(&left, &right), vec![[0, 0]]);
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

        let prepared_left = left.prepare();
        let prepared_right = right.prepare();
        let plan = prepared_left.candidate_face_pair_plan(&prepared_right);
        let CandidateFacePairPlan::Sweep {
            plan: sweep_plan, ..
        } = plan
        else {
            panic!("expected sweep plan");
        };
        assert_eq!(sweep_plan.axis, Axis::Y);
        assert_eq!(candidate_face_pairs(&left, &right), vec![[1, 0]]);
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
            sorted_pairs(prepared_candidate_face_pairs(
                &prepared_left,
                &prepared_right
            )),
            sorted_pairs(quadratic_candidate_face_pairs(
                &prepared_left,
                &prepared_right
            ))
        );
    }

    #[test]
    fn prepared_sweep_counts_nonmonotonic_driver_maxima() {
        let left_points = vec![
            p(0, 0, 0),
            p(100, 0, 0),
            p(0, 1, 0),
            p(50, 0, 0),
            p(51, 0, 0),
            p(50, 1, 0),
        ];
        let right_points = vec![p(60, 0, 0), p(61, 0, 0), p(60, 1, 0)];
        let left_triangles = [[0, 1, 2], [3, 4, 5]];
        let right_triangles = [[0, 1, 2]];
        let left = MeshBounds::from_triangles(&left_points, &left_triangles);
        let right = MeshBounds::from_triangles(&right_points, &right_triangles);
        let prepared_left = left.prepare();
        let prepared_right = right.prepare();

        assert_eq!(
            prepared_left.axis_interval_overlap_estimate(&prepared_right, Axis::X),
            Some(AxisOverlapEstimate {
                pair_count: 1,
                max_target_active: 1,
            })
        );
        assert_eq!(
            sorted_pairs(prepared_candidate_face_pairs(
                &prepared_left,
                &prepared_right
            )),
            vec![[0, 0]]
        );
    }

    #[test]
    fn prepared_sweep_capacity_hint_tracks_peak_active_targets() {
        let mut left_points = Vec::new();
        let mut left_triangles = Vec::new();
        for face in 0..5 {
            let base = left_points.len();
            let x = face as i64 * 10;
            left_points.extend([p(x, 0, 0), p(x + 2, 0, 0), p(x, 1, 0)]);
            left_triangles.push([base, base + 1, base + 2]);
        }

        let mut right_points = Vec::new();
        let mut right_triangles = Vec::new();
        for face in 0..5 {
            let base = right_points.len();
            let x = face as i64 * 10 + 1;
            right_points.extend([p(x, 0, 0), p(x + 1, 0, 0), p(x, 1, 0)]);
            right_triangles.push([base, base + 1, base + 2]);
        }

        let left = MeshBounds::from_triangles(&left_points, &left_triangles);
        let right = MeshBounds::from_triangles(&right_points, &right_triangles);
        let prepared_left = left.prepare();
        let prepared_right = right.prepare();
        let estimate = prepared_left
            .axis_interval_overlap_estimate(&prepared_right, Axis::X)
            .unwrap();

        assert_eq!(estimate.pair_count, 5);
        assert_eq!(estimate.max_target_active, 1);
        assert!(should_use_sparse_sweep(
            estimate.max_target_active,
            prepared_right.bounds.faces.len()
        ));

        let CandidateFacePairPlan::Sweep {
            active_face_capacity_hint,
            ..
        } = prepared_left.candidate_face_pair_plan(&prepared_right)
        else {
            panic!("expected sweep plan");
        };
        assert_eq!(active_face_capacity_hint, 1);
        assert_eq!(
            sorted_pairs(prepared_candidate_face_pairs(
                &prepared_left,
                &prepared_right
            )),
            sorted_pairs(quadratic_candidate_face_pairs(
                &prepared_left,
                &prepared_right
            ))
        );
    }

    #[test]
    fn prepared_sweep_keeps_marker_path_for_dense_active_targets() {
        assert!(!should_use_sparse_sweep(3, 12));
        assert!(!should_use_sparse_sweep(8, 12));
        assert!(should_use_sparse_sweep(2, 12));
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
            sorted_pairs(prepared_candidate_face_pairs(
                &prepared_left,
                &prepared_right
            )),
            sorted_pairs(quadratic_candidate_face_pairs(
                &prepared_left,
                &prepared_right
            ))
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
                .sweep_plan(&prepared_right)
                .unwrap()
                .plan
                .direction,
            SweepDirection::RightDriven
        );
        assert_eq!(
            sorted_pairs(prepared_candidate_face_pairs(
                &prepared_left,
                &prepared_right
            )),
            sorted_pairs(quadratic_candidate_face_pairs(
                &prepared_left,
                &prepared_right
            ))
        );
    }
}
