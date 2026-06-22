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
/// validated before they can reject or retain topological work.
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
/// This borrows retained source bounds and caches only axis sort orders. It is
/// an acceleration fact, not topology evidence: disjoint AABBs may reject work,
/// while retained pairs still require exact narrow-phase predicates.
#[derive(Clone, Debug)]
pub struct PreparedMeshBounds<'a> {
    bounds: &'a MeshBounds,
    min_axis_orders: [Option<Vec<usize>>; 3],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    const fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
            Self::Z => 2,
        }
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
    pub fn candidate_face_pairs(&self, other: &Self) -> Vec<[usize; 2]> {
        self.prepare().candidate_face_pairs(&other.prepare())
    }

    /// Prepare exact min-axis face orders for repeated broad-phase queries.
    ///
    /// An axis order is retained only when all exact comparisons needed for
    /// sorting were decided. Querying two prepared bounds falls back to the
    /// exact quadratic scheduler when no common sweep axis is usable.
    pub fn prepare(&self) -> PreparedMeshBounds<'_> {
        PreparedMeshBounds {
            bounds: self,
            min_axis_orders: [
                sorted_face_indices_by_min_axis(&self.faces, Axis::X),
                sorted_face_indices_by_min_axis(&self.faces, Axis::Y),
                sorted_face_indices_by_min_axis(&self.faces, Axis::Z),
            ],
        }
    }
}

impl<'a> PreparedMeshBounds<'a> {
    /// Return the retained bounds object this prepared scheduler borrows.
    pub const fn bounds(&self) -> &'a MeshBounds {
        self.bounds
    }

    /// Return face-pair candidates whose exact boxes are not disjoint.
    pub fn candidate_face_pairs(&self, other: &PreparedMeshBounds<'_>) -> Vec<[usize; 2]> {
        if !self.mesh_bounds_may_overlap(other) {
            return Vec::new();
        }
        self.candidate_face_pairs_sweep(other)
            .unwrap_or_else(|| self.candidate_face_pairs_quadratic(other))
    }

    fn mesh_bounds_may_overlap(&self, other: &PreparedMeshBounds<'_>) -> bool {
        match (&self.bounds.mesh, &other.bounds.mesh) {
            (Some(left), Some(right)) => must_keep_candidate(left.classify_intersection(right)),
            _ => false,
        }
    }

    fn candidate_face_pairs_quadratic(&self, other: &PreparedMeshBounds<'_>) -> Vec<[usize; 2]> {
        let mut pairs = Vec::new();
        for (left, left_box) in self.bounds.faces.iter().enumerate() {
            for (right, right_box) in other.bounds.faces.iter().enumerate() {
                if must_keep_candidate(left_box.classify_intersection(right_box)) {
                    pairs.push([left, right]);
                }
            }
        }
        pairs
    }

    fn candidate_face_pairs_sweep(
        &self,
        other: &PreparedMeshBounds<'_>,
    ) -> Option<Vec<[usize; 2]>> {
        let mut best_axis = None;
        for axis in [Axis::X, Axis::Y, Axis::Z] {
            if let Some(pair_count) = self.interval_candidate_pair_count_sweep_axis(other, axis)
                && best_axis
                    .as_ref()
                    .is_none_or(|&(_, best_count)| pair_count < best_count)
            {
                best_axis = Some((axis, pair_count));
            }
        }
        let (axis, pair_count) = best_axis?;
        let pairs = self.interval_candidate_pairs_sweep_axis(other, axis, pair_count)?;
        Some(self.filter_full_aabb_candidates(other, pairs))
    }

    fn interval_candidate_pair_count_sweep_axis(
        &self,
        other: &PreparedMeshBounds<'_>,
        axis: Axis,
    ) -> Option<usize> {
        let left_order = self.min_axis_order(axis)?;
        let right_order = other.min_axis_order(axis)?;
        let mut active_right = Vec::<usize>::new();
        let mut next_right = 0usize;
        let mut pair_count = 0usize;

        for &left in left_order {
            let left_box = &self.bounds.faces[left];
            while let Some(&right) = right_order.get(next_right) {
                if compare(
                    axis_min(&other.bounds.faces[right], axis),
                    axis_max(left_box, axis),
                )? == Ordering::Greater
                {
                    break;
                }
                active_right.push(right);
                next_right += 1;
            }

            retain_active_right_axis(
                &mut active_right,
                &other.bounds.faces,
                axis,
                axis_min(left_box, axis),
            )?;
            for &right in &active_right {
                let right_box = &other.bounds.faces[right];
                if compare(axis_min(right_box, axis), axis_max(left_box, axis))?
                    != Ordering::Greater
                {
                    pair_count += 1;
                }
            }
        }

        Some(pair_count)
    }

    fn interval_candidate_pairs_sweep_axis(
        &self,
        other: &PreparedMeshBounds<'_>,
        axis: Axis,
        pair_capacity: usize,
    ) -> Option<Vec<[usize; 2]>> {
        let left_order = self.min_axis_order(axis)?;
        let right_order = other.min_axis_order(axis)?;
        let mut active_right = Vec::<usize>::new();
        let mut next_right = 0usize;
        let mut pairs = Vec::with_capacity(pair_capacity);

        for &left in left_order {
            let left_box = &self.bounds.faces[left];
            while let Some(&right) = right_order.get(next_right) {
                if compare(
                    axis_min(&other.bounds.faces[right], axis),
                    axis_max(left_box, axis),
                )? == Ordering::Greater
                {
                    break;
                }
                active_right.push(right);
                next_right += 1;
            }

            retain_active_right_axis(
                &mut active_right,
                &other.bounds.faces,
                axis,
                axis_min(left_box, axis),
            )?;

            for &right in &active_right {
                let right_box = &other.bounds.faces[right];
                if compare(axis_min(right_box, axis), axis_max(left_box, axis))?
                    == Ordering::Greater
                {
                    continue;
                }
                pairs.push([left, right]);
            }
        }

        pairs.sort_unstable();
        Some(pairs)
    }

    fn filter_full_aabb_candidates(
        &self,
        other: &PreparedMeshBounds<'_>,
        pairs: Vec<[usize; 2]>,
    ) -> Vec<[usize; 2]> {
        pairs
            .into_iter()
            .filter(|[left, right]| {
                let left_box = &self.bounds.faces[*left];
                let right_box = &other.bounds.faces[*right];
                must_keep_candidate(left_box.classify_intersection(right_box))
            })
            .collect()
    }

    fn min_axis_order(&self, axis: Axis) -> Option<&[usize]> {
        self.min_axis_orders[axis.index()].as_deref()
    }
}

fn retain_active_right_axis(
    active_right: &mut Vec<usize>,
    right_faces: &[ExactAabb3],
    axis: Axis,
    left_min: &Real,
) -> Option<()> {
    let mut retained = 0usize;
    for index in 0..active_right.len() {
        let right = active_right[index];
        if compare(axis_max(&right_faces[right], axis), left_min)? != Ordering::Less {
            active_right[retained] = right;
            retained += 1;
        }
    }
    active_right.truncate(retained);
    Some(())
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

fn sorted_face_indices_by_min_axis(faces: &[ExactAabb3], axis: Axis) -> Option<Vec<usize>> {
    exact_merge_sort_face_indices((0..faces.len()).collect(), faces, axis)
}

fn exact_merge_sort_face_indices(
    mut indices: Vec<usize>,
    faces: &[ExactAabb3],
    axis: Axis,
) -> Option<Vec<usize>> {
    if indices.len() <= 1 {
        return Some(indices);
    }
    let right = indices.split_off(indices.len() / 2);
    let left = exact_merge_sort_face_indices(indices, faces, axis)?;
    let right = exact_merge_sort_face_indices(right, faces, axis)?;
    merge_face_indices_by_min_axis(left, right, faces, axis)
}

fn merge_face_indices_by_min_axis(
    left: Vec<usize>,
    right: Vec<usize>,
    faces: &[ExactAabb3],
    axis: Axis,
) -> Option<Vec<usize>> {
    let mut merged = Vec::with_capacity(left.len() + right.len());
    let mut left_iter = left.into_iter().peekable();
    let mut right_iter = right.into_iter().peekable();
    while let (Some(&left), Some(&right)) = (left_iter.peek(), right_iter.peek()) {
        match compare(axis_min(&faces[left], axis), axis_min(&faces[right], axis))? {
            Ordering::Less | Ordering::Equal => merged.push(left_iter.next()?),
            Ordering::Greater => merged.push(right_iter.next()?),
        }
    }
    merged.extend(left_iter);
    merged.extend(right_iter);
    Some(merged)
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
    }
}
