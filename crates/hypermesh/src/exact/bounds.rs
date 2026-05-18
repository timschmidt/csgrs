//! Exact 3D bounds for broad-phase scheduling.
//!
//! AABB facts are acceleration facts, not topology certificates. An exact box
//! can prove that two objects are disjoint; otherwise the pair must continue to
//! a `hyperlimit` narrow-phase predicate before topology changes. This is the
//! package split advocated by Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): cheap geometric-object facts may
//! schedule work, but certified predicates decide combinatorics.

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
/// Still, Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997), requires object facts consumed by predicate scheduling to be
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
