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
        self.candidate_face_pairs_sweep(other)
            .unwrap_or_else(|| self.candidate_face_pairs_quadratic(other))
    }

    fn candidate_face_pairs_quadratic(&self, other: &Self) -> Vec<[usize; 2]> {
        let mut pairs = Vec::new();
        for (left, left_box) in self.faces.iter().enumerate() {
            for (right, right_box) in other.faces.iter().enumerate() {
                if must_keep_candidate(left_box.classify_intersection(right_box)) {
                    pairs.push([left, right]);
                }
            }
        }
        pairs
    }

    fn candidate_face_pairs_sweep(&self, other: &Self) -> Option<Vec<[usize; 2]>> {
        let mut left_order = sorted_face_indices_by_min_x(&self.faces)?;
        let right_order = sorted_face_indices_by_min_x(&other.faces)?;
        let mut active_right = Vec::<usize>::new();
        let mut next_right = 0usize;
        let mut pairs = Vec::new();

        for left in left_order.drain(..) {
            let left_box = &self.faces[left];
            while let Some(&right) = right_order.get(next_right) {
                if compare(&other.faces[right].min.x, &left_box.max.x)? == Ordering::Greater {
                    break;
                }
                active_right.push(right);
                next_right += 1;
            }

            let mut retained_active = Vec::with_capacity(active_right.len());
            for right in active_right {
                if compare(&other.faces[right].max.x, &left_box.min.x)? != Ordering::Less {
                    retained_active.push(right);
                }
            }
            active_right = retained_active;

            for &right in &active_right {
                let right_box = &other.faces[right];
                if compare(&right_box.min.x, &left_box.max.x)? == Ordering::Greater {
                    continue;
                }
                if must_keep_candidate(left_box.classify_intersection(right_box)) {
                    pairs.push([left, right]);
                }
            }
        }

        pairs.sort_unstable();
        Some(pairs)
    }

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
        self.validate(points.len(), triangles.len())?;
        if triangles
            .iter()
            .flatten()
            .any(|&vertex| vertex >= points.len())
        {
            return Err(BoundsValidationError::SourceReplayMismatch);
        }
        let replay = Self::from_triangles(points, triangles);
        if self == &replay {
            Ok(())
        } else {
            Err(BoundsValidationError::SourceReplayMismatch)
        }
    }
}

fn sorted_face_indices_by_min_x(faces: &[ExactAabb3]) -> Option<Vec<usize>> {
    let mut indices = (0..faces.len()).collect::<Vec<_>>();
    for index in 1..indices.len() {
        let mut cursor = index;
        while cursor > 0 {
            match compare(
                &faces[indices[cursor]].min.x,
                &faces[indices[cursor - 1]].min.x,
            )? {
                Ordering::Less => indices.swap(cursor, cursor - 1),
                Ordering::Equal | Ordering::Greater => break,
            }
            cursor -= 1;
        }
    }
    Some(indices)
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
}
