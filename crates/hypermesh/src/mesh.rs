//! Canonical owned triangle geometry and kernel polygon-soup preparation.

use std::collections::{BTreeSet, HashMap};
use std::num::NonZeroU32;
use std::sync::{
    Arc, OnceLock,
    atomic::{AtomicU8, Ordering},
};

use hyperlattice::{Aabb as ExactAabb, HomogeneousPoint3, Matrix4, Point3, Real, Vector3, Vector4};

use crate::context::{CertaintyFact, DecisionContext, MeshCertainty, MeshContext, MeshOutcome};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{
    Aabb, Classification, Plane, affine_projective_point_decision, axis_ref, compare_real_decision,
};
use crate::point_interner::{PointCoordinates, PointInterner};
use crate::polygon::{
    ConvexPolygon, convex_triangle_decision, points_require_wide_dyadic_plane_normalization,
};
use crate::predicate::classify_point_decision;
use crate::storage_hash::StorageHashMap;

/// Triangle: three indices into a [`TriangleMesh`]'s position buffer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Triangle {
    /// First vertex index.
    pub v0: usize,
    /// Second vertex index.
    pub v1: usize,
    /// Third vertex index.
    pub v2: usize,
}

impl Triangle {
    /// Constructs an input triangle.
    pub const fn new(v0: usize, v1: usize, v2: usize) -> Self {
        Self { v0, v1, v2 }
    }

    /// Returns the indices as an array.
    pub const fn indices(self) -> [usize; 3] {
        [self.v0, self.v1, self.v2]
    }
}

#[derive(Debug, Default)]
pub(crate) struct TriangleMeshFacts {
    exact_bounds: OnceLock<Option<Arc<ExactAabb>>>,
    valid_pwn: CertaintyFact,
    axis_aligned_box: OnceLock<Option<Arc<ExactAabb>>>,
    wide_dyadic_plane_schedule: AtomicU8,
}

impl TriangleMeshFacts {
    fn normalizes_wide_dyadic_planes(&self, positions: &[Point3]) -> bool {
        match self.wide_dyadic_plane_schedule.load(Ordering::Relaxed) {
            1 => false,
            2 => true,
            _ => {
                let normalize = points_require_wide_dyadic_plane_normalization(positions);
                // The fact is a deterministic property of immutable positions.
                // Concurrent first consumers may repeat the scan harmlessly.
                self.wide_dyadic_plane_schedule
                    .store(u8::from(normalize) + 1, Ordering::Relaxed);
                normalize
            }
        }
    }
}

/// Canonical owned triangle mesh.
///
/// This is the reusable geometry carrier accepted by Hypermesh operations and
/// produced from [`crate::BooleanMeshBatch`] results. Reusable exact construction
/// planes may be retained privately alongside geometry so iterated operations
/// do not need to expand them again from derived coordinates.
#[derive(Clone, Debug)]
pub struct TriangleMesh {
    /// Vertex positions.
    pub positions: Arc<[Point3]>,
    /// Triangle indices.
    pub triangles: Arc<[Triangle]>,
    pub(crate) facts: Arc<TriangleMeshFacts>,
}

impl PartialEq for TriangleMesh {
    fn eq(&self, other: &Self) -> bool {
        self.positions == other.positions && self.triangles == other.triangles
    }
}

impl TriangleMesh {
    /// Creates an owned triangle mesh.
    pub fn new(positions: Vec<Point3>, triangles: Vec<Triangle>) -> Self {
        Self {
            positions: positions.into(),
            triangles: triangles.into(),
            facts: Arc::new(TriangleMeshFacts::default()),
        }
    }

    pub(crate) fn from_shared_positions(
        positions: Arc<[Point3]>,
        triangles: Vec<Triangle>,
    ) -> Self {
        Self {
            positions,
            triangles: triangles.into(),
            facts: Arc::new(TriangleMeshFacts::default()),
        }
    }

    pub(crate) fn with_valid_pwn_certainty(self, certainty: MeshCertainty) -> Self {
        self.facts.valid_pwn.record(certainty);
        self
    }

    /// Returns a borrowed mesh view.
    pub fn as_ref(&self) -> TriangleMeshRef<'_> {
        TriangleMeshRef {
            positions: &self.positions,
            triangles: &self.triangles,
            native: Some(self),
        }
    }

    /// Builds caller-owned native vertex adjacency from triangle index rows.
    ///
    /// The result is deliberately not retained by the mesh carrier. Algorithms
    /// that reuse adjacency keep this value for the duration of their own
    /// operation, so one-shot queries do not permanently increase mesh memory.
    pub fn adjacency(&self) -> HypermeshResult<Vec<Vec<usize>>> {
        self.validate_triangle_indices()?;
        Ok(self.build_adjacency())
    }

    /// Returns `(position rows, directed adjacency entries)` counts without
    /// retaining or materializing the complete adjacency table.
    pub fn connectivity_counts(&self) -> HypermeshResult<(usize, usize)> {
        self.validate_triangle_indices()?;
        let mut edges = BTreeSet::new();
        for triangle in self.triangles.iter() {
            let [a, b, c] = triangle.indices();
            for [left, right] in [[a, b], [b, c], [c, a]] {
                if left != right {
                    edges.insert(if left < right {
                        [left, right]
                    } else {
                        [right, left]
                    });
                }
            }
        }
        let adjacency_entries =
            edges
                .len()
                .checked_mul(2)
                .ok_or(HypermeshError::CapacityOverflow {
                    operation: "connectivity counting",
                })?;
        Ok((self.positions.len(), adjacency_entries))
    }

    fn build_adjacency(&self) -> Vec<Vec<usize>> {
        let mut adjacency = vec![BTreeSet::new(); self.positions.len()];
        for triangle in self.triangles.iter() {
            let [a, b, c] = triangle.indices();
            for [left, right] in [[a, b], [b, c], [c, a]] {
                if left != right {
                    adjacency[left].insert(right);
                    adjacency[right].insert(left);
                }
            }
        }
        adjacency
            .into_iter()
            .map(|neighbors| neighbors.into_iter().collect())
            .collect()
    }

    fn validate_triangle_indices(&self) -> HypermeshResult<()> {
        for triangle in self.triangles.iter() {
            if let Some(index) = triangle
                .indices()
                .into_iter()
                .find(|index| *index >= self.positions.len())
            {
                return Err(HypermeshError::VertexIndexOutOfBounds {
                    index,
                    vertex_count: self.positions.len(),
                });
            }
        }
        Ok(())
    }

    /// Checks indexed edge pairing for a closed, consistently oriented
    /// two-manifold.
    pub fn is_closed_manifold(&self) -> bool {
        let mut edges = HashMap::<[usize; 2], [usize; 2]>::new();
        for triangle in self.triangles.iter() {
            let [a, b, c] = triangle.indices();
            if [a, b, c]
                .into_iter()
                .any(|index| index >= self.positions.len())
            {
                return false;
            }
            for [start, end] in [[a, b], [b, c], [c, a]] {
                if start == end {
                    return false;
                }
                let key = if start < end {
                    [start, end]
                } else {
                    [end, start]
                };
                edges.entry(key).or_default()[usize::from(start > end)] += 1;
            }
        }
        !self.triangles.is_empty() && edges.values().all(|uses| uses[0] == 1 && uses[1] == 1)
    }

    /// Returns true when every triangle has valid, nondegenerate exact
    /// geometry and no two triangles cover the same three exact points.
    ///
    /// Position rows are canonicalized by exact coordinate equality before
    /// triangle keys are compared, so independently indexed duplicate faces
    /// are rejected as well.
    pub fn has_unique_nondegenerate_triangles(
        &self,
        context: &MeshContext,
    ) -> HypermeshResult<MeshOutcome<bool>> {
        let decisions = DecisionContext::new(context);
        let unique = self.has_unique_nondegenerate_triangles_decision(&decisions)?;
        Ok(decisions.finish(unique))
    }

    pub(crate) fn has_unique_nondegenerate_triangles_decision(
        &self,
        decisions: &DecisionContext,
    ) -> HypermeshResult<bool> {
        let canonical_indices = canonical_position_indices(decisions, &self.positions)?;
        let mut seen = BTreeSet::new();
        for triangle in self.triangles.iter() {
            let indices = triangle.indices();
            let [Some(a), Some(b), Some(c)] = indices.map(|index| self.positions.get(index)) else {
                return Ok(false);
            };
            let mut key = indices.map(|index| canonical_indices[index]);
            if key[0] == key[1]
                || key[1] == key[2]
                || key[0] == key[2]
                || !Plane::decide_points_are_nondegenerate(decisions, a, b, c)?
            {
                return Ok(false);
            }
            key.sort_unstable();
            if !seen.insert(key) {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Checks exact geometric edge pairing for a closed, consistently
    /// oriented two-manifold.
    ///
    /// Unlike [`Self::is_closed_manifold`], this canonicalizes independently
    /// indexed position rows by exact coordinate equality. Duplicate faces,
    /// degenerate faces, and geometrically non-manifold edge valence are
    /// rejected.
    pub fn is_closed_manifold_geometry(
        &self,
        context: &MeshContext,
    ) -> HypermeshResult<MeshOutcome<bool>> {
        let decisions = DecisionContext::new(context);
        let closed = self.is_closed_manifold_geometry_decision(&decisions)?;
        Ok(decisions.finish(closed))
    }

    pub(crate) fn is_closed_manifold_geometry_decision(
        &self,
        decisions: &DecisionContext,
    ) -> HypermeshResult<bool> {
        if self.triangles.is_empty()
            || !self.has_unique_nondegenerate_triangles_decision(decisions)?
        {
            return Ok(false);
        }
        let canonical_indices = canonical_position_indices(decisions, &self.positions)?;
        let mut edges = HashMap::<[usize; 2], [usize; 2]>::new();
        for triangle in self.triangles.iter() {
            let [a, b, c] = triangle.indices().map(|index| canonical_indices[index]);
            for [start, end] in [[a, b], [b, c], [c, a]] {
                let (key, direction) = if start < end {
                    ([start, end], 0)
                } else {
                    ([end, start], 1)
                };
                edges.entry(key).or_default()[direction] += 1;
            }
        }
        Ok(edges.values().all(|uses| *uses == [1, 1]))
    }

    /// Returns policy-certified exact bounds, or `None` for empty geometry.
    pub fn exact_bounds(
        &self,
        context: &MeshContext,
    ) -> HypermeshResult<MeshOutcome<Option<ExactAabb>>> {
        let decisions = DecisionContext::new(context);
        let bounds = self.exact_bounds_decision(&decisions)?;
        Ok(decisions.finish(bounds))
    }

    pub(crate) fn exact_bounds_decision(
        &self,
        decisions: &DecisionContext,
    ) -> HypermeshResult<Option<ExactAabb>> {
        if let Some(bounds) = self.facts.exact_bounds.get() {
            return Ok(bounds.as_deref().cloned());
        }

        let local = decisions.isolated();
        let bounds = self.compute_exact_bounds_decision(&local);
        decisions.absorb(local.certainty());
        let bounds = bounds?;
        if local.certainty() == crate::MeshCertainty::Certified {
            let _ = self.facts.exact_bounds.set(bounds.clone().map(Arc::new));
            return Ok(self
                .facts
                .exact_bounds
                .get()
                .and_then(Option::as_deref)
                .cloned());
        }
        Ok(bounds)
    }

    fn compute_exact_bounds_decision(
        &self,
        decisions: &DecisionContext,
    ) -> HypermeshResult<Option<ExactAabb>> {
        let Some(first) = self.positions.first().cloned() else {
            return Ok(None);
        };
        let mut bounds = ExactAabb::new(first.clone(), first);
        for point in &self.positions[1..] {
            for axis in 0..3 {
                let value = axis_ref(point, axis);
                let minimum = axis_ref(&bounds.mins, axis);
                if compare_real_decision(decisions, value, minimum)?.is_lt() {
                    *crate::geometry::axis_mut(&mut bounds.mins, axis) = value.clone();
                }
                let maximum = axis_ref(&bounds.maxs, axis);
                if compare_real_decision(decisions, value, maximum)?.is_gt() {
                    *crate::geometry::axis_mut(&mut bounds.maxs, axis) = value.clone();
                }
            }
        }
        Ok(Some(bounds))
    }

    /// Returns a caller-owned finite projection of every exact position.
    ///
    /// This is an explicit approximation boundary for renderers, exporters,
    /// diagnostics, and benchmarks. Native geometry remains exact.
    pub fn finite_positions(&self) -> Option<Vec<[f64; 3]>> {
        self.positions
            .iter()
            .map(|point| {
                Some([
                    point.x.to_f64_lossy()?,
                    point.y.to_f64_lossy()?,
                    point.z.to_f64_lossy()?,
                ])
            })
            .collect()
    }

    /// Returns geometry whose coordinates are exact promotions of
    /// this mesh's finite binary64 projection.
    ///
    /// `None` means at least one native coordinate has no finite projection.
    pub fn materialize_finite(&self) -> Option<Self> {
        let positions = self
            .finite_positions()?
            .into_iter()
            .map(|position| {
                Some(Point3::new(
                    Real::try_from(position[0]).ok()?,
                    Real::try_from(position[1]).ok()?,
                    Real::try_from(position[2]).ok()?,
                ))
            })
            .collect::<Option<Vec<_>>>()?;
        Some(Self::new(positions, self.triangles.to_vec()))
    }

    /// Returns the retained exact convex hull of this mesh's native positions.
    pub fn convex_hull(&self, context: &MeshContext) -> HypermeshResult<MeshOutcome<Self>> {
        crate::convex_hull(context, &self.positions)
    }

    /// Returns policy-certified bounds when the native rows form one complete
    /// axis-aligned box surface.
    pub fn axis_aligned_box_bounds(
        &self,
        context: &MeshContext,
    ) -> HypermeshResult<MeshOutcome<Option<ExactAabb>>> {
        let decisions = DecisionContext::new(context);
        let bounds = self.axis_aligned_box_bounds_decision(&decisions)?;
        Ok(decisions.finish(bounds))
    }

    pub(crate) fn axis_aligned_box_bounds_decision(
        &self,
        decisions: &DecisionContext,
    ) -> HypermeshResult<Option<ExactAabb>> {
        if let Some(bounds) = self.facts.axis_aligned_box.get() {
            return Ok(bounds.as_deref().cloned());
        }

        let local = decisions.isolated();
        let bounds = self.compute_axis_aligned_box_bounds_decision(&local);
        decisions.absorb(local.certainty());
        let bounds = bounds?;
        if bounds.is_some() {
            self.facts.valid_pwn.record(local.certainty());
        }
        if local.certainty() == crate::MeshCertainty::Certified {
            let cached = bounds.as_ref().map(|bounds| {
                self.facts
                    .exact_bounds
                    .get()
                    .and_then(Option::as_ref)
                    .cloned()
                    .unwrap_or_else(|| Arc::new(bounds.clone()))
            });
            let _ = self.facts.axis_aligned_box.set(cached);
            return Ok(self
                .facts
                .axis_aligned_box
                .get()
                .and_then(Option::as_deref)
                .cloned());
        }
        Ok(bounds)
    }

    fn compute_axis_aligned_box_bounds_decision(
        &self,
        decisions: &DecisionContext,
    ) -> HypermeshResult<Option<ExactAabb>> {
        if self.positions.len() != 8 || self.triangles.len() != 12 {
            return Ok(None);
        }
        let Some(bounds) = self.exact_bounds_decision(decisions)? else {
            return Ok(None);
        };
        for axis in 0..3 {
            if compare_real_decision(
                decisions,
                axis_ref(&bounds.mins, axis),
                axis_ref(&bounds.maxs, axis),
            )?
            .is_eq()
            {
                return Ok(None);
            }
        }

        let mut corners = [false; 8];
        let mut position_corners = [0_u8; 8];
        for (position_index, point) in self.positions.iter().enumerate() {
            let mut corner = 0;
            for axis in 0..3 {
                let value = axis_ref(point, axis);
                if compare_real_decision(decisions, value, axis_ref(&bounds.maxs, axis))?.is_eq() {
                    corner |= 1 << axis;
                } else if !compare_real_decision(decisions, value, axis_ref(&bounds.mins, axis))?
                    .is_eq()
                {
                    return Ok(None);
                }
            }
            if std::mem::replace(&mut corners[corner], true) {
                return Ok(None);
            }
            position_corners[position_index] = corner as u8;
        }
        if corners.iter().any(|present| !present) {
            return Ok(None);
        }

        let mut face_triangles = [0_u8; 6];
        let mut face_corner_masks = [0_u8; 6];
        for triangle in self.triangles.iter() {
            let [a, b, c] = triangle.indices();
            let Some(corners) = Option::zip(
                Option::zip(position_corners.get(a), position_corners.get(b)),
                position_corners.get(c),
            )
            .map(|((a, b), c)| [*a, *b, *c]) else {
                return Ok(None);
            };
            let corner_mask = corners
                .into_iter()
                .fold(0_u8, |mask, corner| mask | (1_u8 << corner));
            if corner_mask.count_ones() != 3 {
                return Ok(None);
            }
            let mut face = None;
            for axis in 0..3 {
                let side = (corners[0] >> axis) & 1;
                if corners.iter().all(|corner| ((corner >> axis) & 1) == side) {
                    face = Some((axis, side));
                    break;
                }
            }
            let Some((axis, side)) = face else {
                return Ok(None);
            };
            let first_axis = (axis + 1) % 3;
            let second_axis = (axis + 2) % 3;
            let coordinate = |corner: u8, axis| i8::from(((corner >> axis) & 1) != 0);
            let orientation = (coordinate(corners[1], first_axis)
                - coordinate(corners[0], first_axis))
                * (coordinate(corners[2], second_axis) - coordinate(corners[0], second_axis))
                - (coordinate(corners[1], second_axis) - coordinate(corners[0], second_axis))
                    * (coordinate(corners[2], first_axis) - coordinate(corners[0], first_axis));
            if orientation != if side == 0 { -1 } else { 1 } {
                return Ok(None);
            }
            let face = axis * 2 + usize::from(side);
            face_triangles[face] = face_triangles[face].saturating_add(1);
            face_corner_masks[face] |= corner_mask;
        }
        Ok(
            (face_triangles == [2; 6] && face_corner_masks == [0x55, 0xaa, 0x33, 0xcc, 0x0f, 0xf0])
                .then_some(bounds),
        )
    }

    /// Applies an exact homogeneous transform under the selected predicate
    /// policy.
    pub fn try_transformed(
        &self,
        context: &MeshContext,
        matrix: &Matrix4,
    ) -> HypermeshResult<MeshOutcome<Self>> {
        let decisions = DecisionContext::new(context);
        let transformed = self.try_transformed_decision(&decisions, matrix)?;
        Ok(decisions.finish(transformed))
    }

    fn try_transformed_decision(
        &self,
        decisions: &DecisionContext,
        matrix: &Matrix4,
    ) -> HypermeshResult<Self> {
        let matrix_facts = matrix.structural_facts();
        let positions = if matrix_facts.is_affine {
            matrix
                .transform_point3_batch(&self.positions)
                .map_err(|_| HypermeshError::UnknownClassification)?
        } else {
            self.positions
                .iter()
                .map(|point| {
                    let transformed = matrix.transform_vec4_point(&Vector4::new([
                        point.x.clone(),
                        point.y.clone(),
                        point.z.clone(),
                        Real::one(),
                    ]));
                    let [x, y, z, w] = transformed.0;
                    affine_projective_point_decision(decisions, &HomogeneousPoint3::new(x, y, z, w))
                })
                .collect::<HypermeshResult<Vec<_>>>()?
        };
        let transformed = Self::new(positions, self.triangles.to_vec());
        let preserves_valid_pwn = matches!(
            matrix_facts.transform_kind,
            hyperlattice::Matrix4TransformKind::Identity
                | hyperlattice::Matrix4TransformKind::AffineTranslation
        ) || matrix_facts.is_affine
            && matrix_facts.transform_kind == hyperlattice::Matrix4TransformKind::SignedPermutation;
        if preserves_valid_pwn && let Some(certainty) = self.facts.valid_pwn.certainty() {
            transformed.facts.valid_pwn.record(certainty);
        }
        Ok(transformed)
    }

    /// Reverses every triangle while sharing the immutable position buffer.
    ///
    /// The returned mesh owns its derived index buffer; callers that need it
    /// repeatedly retain the returned value explicitly.
    pub fn reversed_winding(&self) -> Self {
        let facts = Arc::new(TriangleMeshFacts::default());
        if let Some(certainty) = self.facts.valid_pwn.certainty() {
            facts.valid_pwn.record(certainty);
        }
        Self {
            positions: Arc::clone(&self.positions),
            triangles: self
                .triangles
                .iter()
                .map(|triangle| Triangle::new(triangle.v2, triangle.v1, triangle.v0))
                .collect::<Vec<_>>()
                .into(),
            facts,
        }
    }

    /// Applies an exact `Rz * Ry * Rx` Euler rotation in degrees.
    pub fn try_rotated_xyz_degrees(
        &self,
        context: &MeshContext,
        x: Real,
        y: Real,
        z: Real,
    ) -> HypermeshResult<MeshOutcome<Self>> {
        let decisions = DecisionContext::new(context);
        let transformed = self.try_rotated_xyz_degrees_decision(&decisions, x, y, z)?;
        Ok(decisions.finish(transformed))
    }

    fn try_rotated_xyz_degrees_decision(
        &self,
        decisions: &DecisionContext,
        x: Real,
        y: Real,
        z: Real,
    ) -> HypermeshResult<Self> {
        let degrees = [x, y, z];
        let x = degrees[0].clone().to_radians();
        let y = degrees[1].clone().to_radians();
        let z = degrees[2].clone().to_radians();
        let (sin_x, cos_x) = (x.clone().sin(), x.cos());
        let (sin_y, cos_y) = (y.clone().sin(), y.cos());
        let (sin_z, cos_z) = (z.clone().sin(), z.cos());
        let cos_z_sin_y = cos_z.clone() * sin_y.clone();
        let sin_z_sin_y = sin_z.clone() * sin_y.clone();
        let zero = Real::zero();
        let one = Real::one();
        let matrix = Matrix4::from_row_major([
            cos_z.clone() * cos_y.clone(),
            cos_z_sin_y.clone() * sin_x.clone() - sin_z.clone() * cos_x.clone(),
            cos_z_sin_y * cos_x.clone() + sin_z.clone() * sin_x.clone(),
            zero.clone(),
            sin_z.clone() * cos_y.clone(),
            sin_z_sin_y.clone() * sin_x.clone() + cos_z.clone() * cos_x.clone(),
            sin_z_sin_y * cos_x.clone() - cos_z * sin_x.clone(),
            zero.clone(),
            -sin_y,
            cos_y.clone() * sin_x,
            cos_y * cos_x,
            zero.clone(),
            zero.clone(),
            zero.clone(),
            zero,
            one,
        ]);
        let transformed = self.try_transformed_decision(decisions, &matrix)?;
        if let Some(certainty) = self.facts.valid_pwn.certainty() {
            transformed.facts.valid_pwn.record(certainty);
        }
        Ok(transformed)
    }

    /// Uniformly subdivides every triangle while sharing edge midpoints.
    ///
    /// Each level replaces one triangle with four consistently oriented
    /// triangles. Midpoints are indexed once per undirected edge, so adjacent
    /// input triangles remain adjacent in the result.
    pub fn subdivide_triangles(&self, levels: NonZeroU32) -> HypermeshResult<Self> {
        let mut positions = self.positions.to_vec();
        let mut triangles = self.triangles.to_vec();
        let two = Real::from(2_u8);
        for _ in 0..levels.get() {
            let mut midpoints = HashMap::<(usize, usize), usize>::new();
            let refined_capacity =
                triangles
                    .len()
                    .checked_mul(4)
                    .ok_or(HypermeshError::CapacityOverflow {
                        operation: "triangle subdivision",
                    })?;
            let mut refined = Vec::with_capacity(refined_capacity);
            for triangle in triangles {
                let [a, b, c] = triangle.indices();
                if let Some(index) = [a, b, c]
                    .into_iter()
                    .find(|index| *index >= positions.len())
                {
                    return Err(HypermeshError::VertexIndexOutOfBounds {
                        index,
                        vertex_count: positions.len(),
                    });
                }
                let mut midpoint = |left: usize, right: usize| -> HypermeshResult<usize> {
                    let edge = if left < right {
                        (left, right)
                    } else {
                        (right, left)
                    };
                    if let Some(index) = midpoints.get(&edge) {
                        return Ok(*index);
                    }
                    let left = &positions[edge.0];
                    let right = &positions[edge.1];
                    let point = Point3::new(
                        ((&left.x + &right.x) / &two)
                            .map_err(|_| HypermeshError::UnknownClassification)?,
                        ((&left.y + &right.y) / &two)
                            .map_err(|_| HypermeshError::UnknownClassification)?,
                        ((&left.z + &right.z) / &two)
                            .map_err(|_| HypermeshError::UnknownClassification)?,
                    );
                    let index = positions.len();
                    positions.push(point);
                    midpoints.insert(edge, index);
                    Ok(index)
                };
                let ab = midpoint(a, b)?;
                let bc = midpoint(b, c)?;
                let ca = midpoint(c, a)?;
                refined.extend([
                    Triangle::new(a, ab, ca),
                    Triangle::new(ab, b, bc),
                    Triangle::new(ca, bc, c),
                    Triangle::new(ab, bc, ca),
                ]);
            }
            triangles = refined;
        }
        let mesh = Self::new(positions, triangles);
        if let Some(certainty) = self.facts.valid_pwn.certainty() {
            mesh.facts.valid_pwn.record(certainty);
        }
        Ok(mesh)
    }

    /// Returns exact ray/triangle intersections sorted by ray parameter.
    ///
    /// Coplanar ray/triangle overlap has no unique point and is therefore not
    /// emitted. Exact edge and vertex contacts are deduplicated by point
    /// identity after all triangle reports have been collected.
    pub fn ray_intersections(
        &self,
        context: &MeshContext,
        origin: &Point3,
        direction: &Vector3,
    ) -> HypermeshResult<MeshOutcome<Vec<(Point3, Real)>>> {
        let decisions = DecisionContext::new(context);
        let hits = self.ray_intersections_decision(&decisions, origin, direction)?;
        Ok(decisions.finish(hits))
    }

    pub(crate) fn ray_intersections_decision(
        &self,
        decisions: &DecisionContext,
        origin: &Point3,
        direction: &Vector3,
    ) -> HypermeshResult<Vec<(Point3, Real)>> {
        let origin_limit =
            hyperlimit::Point3::new(origin.x.clone(), origin.y.clone(), origin.z.clone());
        let direction_limit = hyperlimit::Point3::new(
            direction.0[0].clone(),
            direction.0[1].clone(),
            direction.0[2].clone(),
        );
        let mut hits = Vec::new();
        for triangle in self.triangles.iter() {
            let [a, b, c] = triangle.indices();
            let vertex = |index| {
                self.positions
                    .get(index)
                    .ok_or(HypermeshError::VertexIndexOutOfBounds {
                        index,
                        vertex_count: self.positions.len(),
                    })
            };
            let (a, b, c) = (vertex(a)?, vertex(b)?, vertex(c)?);
            let vertices = [a, b, c].map(|point| {
                hyperlimit::Point3::new(point.x.clone(), point.y.clone(), point.z.clone())
            });
            let report = decisions.decide(
                hyperlimit::classify_ray_triangle3_intersection_report(
                    &origin_limit,
                    &direction_limit,
                    &vertices[0],
                    &vertices[1],
                    &vertices[2],
                    decisions.policy(),
                ),
                "ray-triangle intersection",
            )?;
            if !matches!(
                report.relation,
                hyperlimit::RayTriangleIntersection::Proper
                    | hyperlimit::RayTriangleIntersection::BoundaryTouch
            ) {
                continue;
            }
            if let (Some(point), Some(parameter)) = (&report.point, &report.parameter) {
                hits.push((
                    Point3::new(point.x.clone(), point.y.clone(), point.z.clone()),
                    parameter.clone(),
                ));
            }
        }
        for index in 1..hits.len() {
            let mut current = index;
            while current > 0
                && compare_real_decision(decisions, &hits[current - 1].1, &hits[current].1)?.is_gt()
            {
                hits.swap(current - 1, current);
                current -= 1;
            }
        }
        let mut unique_hits: Vec<(Point3, Real)> = Vec::with_capacity(hits.len());
        for hit in hits {
            let duplicate = if let Some((last, _)) = unique_hits.last() {
                let left = hyperlimit::Point3::new(last.x.clone(), last.y.clone(), last.z.clone());
                let right =
                    hyperlimit::Point3::new(hit.0.x.clone(), hit.0.y.clone(), hit.0.z.clone());
                decisions.decide(
                    hyperlimit::point3_equal(&left, &right, decisions.policy()),
                    "ray-hit point equality",
                )?
            } else {
                false
            };
            if !duplicate {
                unique_hits.push(hit);
            }
        }
        Ok(unique_hits)
    }

    /// Tests strict solid containment using exact boundary predicates and ray
    /// parity. Points on a triangle, edge, or vertex are not inside.
    pub fn contains_point(
        &self,
        context: &MeshContext,
        point: &Point3,
    ) -> HypermeshResult<MeshOutcome<bool>> {
        let decisions = DecisionContext::new(context);
        let contains = self.contains_point_decision(&decisions, point)?;
        Ok(decisions.finish(contains))
    }

    pub(crate) fn contains_point_decision(
        &self,
        decisions: &DecisionContext,
        point: &Point3,
    ) -> HypermeshResult<bool> {
        if self.triangles.is_empty() {
            return Ok(false);
        }
        let bounds = match self.exact_bounds_decision(decisions) {
            Ok(bounds) => bounds,
            Err(HypermeshError::PredicateUndecided { .. }) => None,
            Err(error) => return Err(error),
        };
        if let Some(bounds) = bounds {
            for (coordinate, minimum, maximum) in [
                (&point.x, &bounds.mins.x, &bounds.maxs.x),
                (&point.y, &bounds.mins.y, &bounds.maxs.y),
                (&point.z, &bounds.mins.z, &bounds.maxs.z),
            ] {
                if compare_real_decision(decisions, coordinate, minimum)?.is_lt()
                    || compare_real_decision(decisions, coordinate, maximum)?.is_gt()
                {
                    return Ok(false);
                }
            }
        }
        let query = hyperlimit::Point3::new(point.x.clone(), point.y.clone(), point.z.clone());
        for triangle in self.triangles.iter() {
            let [a, b, c] = triangle.indices();
            let vertex = |index| {
                self.positions
                    .get(index)
                    .ok_or(HypermeshError::VertexIndexOutOfBounds {
                        index,
                        vertex_count: self.positions.len(),
                    })
            };
            let (a, b, c) = (vertex(a)?, vertex(b)?, vertex(c)?);
            let vertices = [a, b, c].map(|vertex| {
                hyperlimit::Point3::new(vertex.x.clone(), vertex.y.clone(), vertex.z.clone())
            });
            let location = decisions.decide(
                hyperlimit::classify_point_triangle3(
                    &vertices[0],
                    &vertices[1],
                    &vertices[2],
                    &query,
                    decisions.policy(),
                ),
                "point-triangle classification",
            )?;
            if matches!(
                location,
                hyperlimit::Triangle3Location::Inside
                    | hyperlimit::Triangle3Location::OnEdge
                    | hyperlimit::Triangle3Location::OnVertex
            ) {
                return Ok(false);
            }
        }
        let direction = Vector3::from_xyz(
            Real::one(),
            (Real::from(3_u8) / Real::from(8_u8)).expect("eight is nonzero"),
            (Real::from(5_u8) / Real::from(16_u8)).expect("sixteen is nonzero"),
        );
        Ok(self
            .ray_intersections_decision(decisions, point, &direction)?
            .len()
            % 2
            == 1)
    }

    /// Intersects consecutive polyline segments with native triangles and
    /// returns exact points in polyline order.
    pub fn polyline_intersections(
        &self,
        context: &MeshContext,
        polyline: &[Point3],
    ) -> HypermeshResult<MeshOutcome<Vec<Point3>>> {
        let decisions = DecisionContext::new(context);
        let intersections = self.polyline_intersections_decision(&decisions, polyline)?;
        Ok(decisions.finish(intersections))
    }

    pub(crate) fn polyline_intersections_decision(
        &self,
        decisions: &DecisionContext,
        polyline: &[Point3],
    ) -> HypermeshResult<Vec<Point3>> {
        let mut output: Vec<Point3> = Vec::new();
        for segment in polyline.windows(2) {
            let direction = &segment[1] - &segment[0];
            for (point, parameter) in
                self.ray_intersections_decision(decisions, &segment[0], &direction)?
            {
                if compare_real_decision(decisions, &parameter, &Real::zero())?.is_lt()
                    || compare_real_decision(decisions, &parameter, &Real::one())?.is_gt()
                {
                    continue;
                }
                let duplicate = if let Some(last) = output.last() {
                    let left =
                        hyperlimit::Point3::new(last.x.clone(), last.y.clone(), last.z.clone());
                    let right =
                        hyperlimit::Point3::new(point.x.clone(), point.y.clone(), point.z.clone());
                    decisions.decide(
                        hyperlimit::point3_equal(&left, &right, decisions.policy()),
                        "polyline-hit point equality",
                    )?
                } else {
                    false
                };
                if !duplicate {
                    output.push(point);
                }
            }
        }
        Ok(output)
    }

    /// Returns the angle in radians between two indexed triangle normals.
    pub fn dihedral_angle(
        &self,
        context: &MeshContext,
        first: Triangle,
        second: Triangle,
    ) -> HypermeshResult<MeshOutcome<Real>> {
        let decisions = DecisionContext::new(context);
        let normal = |triangle: Triangle| -> HypermeshResult<Vector3> {
            let [a, b, c] = triangle.indices();
            let point = |index| {
                self.positions
                    .get(index)
                    .ok_or(HypermeshError::VertexIndexOutOfBounds {
                        index,
                        vertex_count: self.positions.len(),
                    })
            };
            let normal = (point(b)? - point(a)?).cross(&(point(c)? - point(a)?));
            match crate::predicate::classify_real(&decisions, &normal.dot(&normal))? {
                Classification::Positive => normal
                    .normalize_checked()
                    .map_err(|_| HypermeshError::UnknownClassification),
                Classification::On => Err(HypermeshError::DegeneratePointSet),
                Classification::Negative => Err(HypermeshError::UnknownClassification),
            }
        };
        let angle = normal(first)?
            .angle_to(&normal(second)?)
            .map_err(|_| HypermeshError::UnknownClassification)?;
        Ok(decisions.finish(angle))
    }

    /// Applies exact Laplacian smoothing to native positions.
    ///
    /// Triangle indexing is retained. Boundary preservation is intentionally
    /// not inferred here; callers requiring constrained smoothing should
    /// supply an explicit constraint set in a higher-level algorithm.
    pub fn laplacian_smooth(&self, lambda: &Real, iterations: usize) -> HypermeshResult<Self> {
        let adjacency = self.adjacency()?;
        let mut positions = self.positions.to_vec();
        let mut scratch = Vec::with_capacity(positions.len());
        for _ in 0..iterations {
            smooth_positions_once(&positions, &mut scratch, &adjacency, lambda);
            std::mem::swap(&mut positions, &mut scratch);
        }
        Ok(Self::new(positions, self.triangles.to_vec()))
    }

    /// Applies alternating Laplacian shrink and inflation passes.
    ///
    /// Each iteration applies `lambda` followed by `mu`, retaining the native
    /// triangle indexing while reducing the volume loss of one-sided
    /// Laplacian smoothing.
    pub fn taubin_smooth(
        &self,
        lambda: &Real,
        mu: &Real,
        iterations: usize,
    ) -> HypermeshResult<Self> {
        let adjacency = self.adjacency()?;
        let mut positions = self.positions.to_vec();
        let mut scratch = Vec::with_capacity(positions.len());
        for _ in 0..iterations {
            smooth_positions_once(&positions, &mut scratch, &adjacency, lambda);
            std::mem::swap(&mut positions, &mut scratch);
            smooth_positions_once(&positions, &mut scratch, &adjacency, mu);
            std::mem::swap(&mut positions, &mut scratch);
        }
        Ok(Self::new(positions, self.triangles.to_vec()))
    }
}

fn smooth_positions_once(
    positions: &[Point3],
    output: &mut Vec<Point3>,
    adjacency: &[Vec<usize>],
    factor: &Real,
) {
    output.clear();
    for (index, neighbors) in adjacency.iter().enumerate() {
        if neighbors.is_empty() {
            output.push(positions[index].clone());
            continue;
        }
        let mut sum = hyperlattice::Vector3::zero();
        for &neighbor in neighbors {
            sum = sum + positions[neighbor].to_vector();
        }
        let count = Real::from(neighbors.len() as u64);
        let average = (sum / count).expect("a nonempty adjacency row has a nonzero divisor");
        let current = positions[index].to_vector();
        output.push(Point3::origin() + current.clone() + (average - current) * factor.clone());
    }
}

/// Borrowed triangle mesh view.
#[derive(Clone, Copy, Debug)]
pub struct TriangleMeshRef<'a> {
    /// Borrowed positions.
    pub positions: &'a [Point3],
    /// Borrowed triangles.
    pub triangles: &'a [Triangle],
    pub(crate) native: Option<&'a TriangleMesh>,
}

impl<'a> TriangleMeshRef<'a> {
    /// Borrows position and triangle slices without retained native facts.
    pub const fn new(positions: &'a [Point3], triangles: &'a [Triangle]) -> Self {
        Self {
            positions,
            triangles,
            native: None,
        }
    }
}

/// Working polygon soup.
#[derive(Clone, Debug, PartialEq)]
pub struct PolygonSoup {
    /// Polygons produced from input triangles.
    pub polygons: Vec<ConvexPolygon>,
    /// Exact bounds across all source positions.
    pub bounds: Aabb,
    /// Number of input meshes.
    pub num_meshes: usize,
}

impl PolygonSoup {
    /// Recomputes exact bounds from polygon vertices.
    pub fn compute_bounds_from_vertices(
        &mut self,
        context: &MeshContext,
    ) -> HypermeshResult<MeshOutcome<()>> {
        let decisions = DecisionContext::new(context);
        self.compute_bounds_from_vertices_decision(&decisions)?;
        Ok(decisions.finish(()))
    }

    pub(crate) fn compute_bounds_from_vertices_decision(
        &mut self,
        decisions: &DecisionContext,
    ) -> HypermeshResult<()> {
        let mut vertices = Vec::new();
        for polygon in &self.polygons {
            vertices.extend(polygon.vertices_decision(decisions)?);
        }
        self.bounds = bounds_for_positions(decisions, vertices.iter())?;
        Ok(())
    }
}

/// Validates borrowed mesh views and returns their combined polygon soup.
pub fn polygon_soup(
    context: &MeshContext,
    meshes: &[TriangleMeshRef<'_>],
) -> HypermeshResult<MeshOutcome<PolygonSoup>> {
    let decisions = DecisionContext::new(context);
    let soup = build_polygon_soup_internal(&decisions, meshes)?;
    Ok(decisions.finish(soup))
}

/// Validates a closed PWN mesh and certifies that every vertex lies in every
/// outward-oriented face half-space.
///
/// Native mesh owners retain only the policy-qualified closed-PWN validation
/// fact discovered while preparing the source faces. Convexity itself is not
/// a trusted shortcut or reusable Boolean-engine selector.
pub fn certify_convex_mesh(
    context: &MeshContext,
    mesh: TriangleMeshRef<'_>,
) -> HypermeshResult<MeshOutcome<()>> {
    let decisions = DecisionContext::new(context);
    certify_convex_mesh_decision(&decisions, mesh)?;
    Ok(decisions.finish(()))
}

fn certify_convex_mesh_decision(
    decisions: &DecisionContext,
    mesh: TriangleMeshRef<'_>,
) -> HypermeshResult<()> {
    let soup = build_polygon_soup_internal(decisions, &[mesh])?;
    for polygon in &soup.polygons {
        for point in mesh.positions {
            if classify_point_decision(decisions, point, &polygon.support)?
                == Classification::Positive
            {
                return Err(HypermeshError::NonConvexInput);
            }
        }
    }
    Ok(())
}

pub(crate) fn build_polygon_soup_internal(
    decisions: &DecisionContext,
    meshes: &[TriangleMeshRef<'_>],
) -> HypermeshResult<PolygonSoup> {
    crate::trace_dispatch!("build-polygon-soup", "start");
    validate_non_empty_mesh_views(meshes)?;

    let bounds = bounds_for_positions(
        decisions,
        meshes.iter().flat_map(|mesh| mesh.positions.iter()),
    )?;
    crate::trace_dispatch!("build-polygon-soup", "bounds-computed");

    let polygon_capacity = meshes
        .iter()
        .try_fold(0usize, |total, mesh| {
            total.checked_add(mesh.triangles.len())
        })
        .ok_or(HypermeshError::UnknownClassification)?;
    let mut polygons: Vec<ConvexPolygon> = Vec::with_capacity(polygon_capacity);
    let mut polygon_index = 0isize;
    for (mesh_index, mesh) in meshes.iter().enumerate() {
        let edge_balance_is_certified = mesh
            .native
            .is_some_and(|native| native.facts.valid_pwn.consume(decisions));
        let normalize_wide_dyadic = mesh.native.map_or_else(
            || points_require_wide_dyadic_plane_normalization(mesh.positions),
            |native| native.facts.normalizes_wide_dyadic_planes(mesh.positions),
        );
        for (triangle_index, triangle) in mesh.triangles.iter().enumerate() {
            let [i0, i1, i2] = triangle.indices();
            let p0 = mesh
                .positions
                .get(i0)
                .ok_or(HypermeshError::VertexIndexOutOfBounds {
                    index: i0,
                    vertex_count: mesh.positions.len(),
                })?;
            let p1 = mesh
                .positions
                .get(i1)
                .ok_or(HypermeshError::VertexIndexOutOfBounds {
                    index: i1,
                    vertex_count: mesh.positions.len(),
                })?;
            let p2 = mesh
                .positions
                .get(i2)
                .ok_or(HypermeshError::VertexIndexOutOfBounds {
                    index: i2,
                    vertex_count: mesh.positions.len(),
                })?;
            let mut polygon = convex_triangle_decision(
                decisions,
                p0,
                p1,
                p2,
                mesh_index as isize,
                polygon_index,
                normalize_wide_dyadic,
            )?;
            polygon.set_source_triangle_edge_identities(mesh_index, [i0, i1, i2])?;
            if !polygon.support.decide_is_valid(decisions)? {
                return Err(HypermeshError::DegenerateTriangle {
                    mesh_index,
                    triangle_index,
                });
            }
            polygon.delta_w = vec![0; meshes.len()];
            polygon.delta_w[mesh_index] = 1;
            let stored_polygon = polygons.len();
            polygons.reserve(1);
            // SAFETY: `reserve(1)` guarantees that `stored_polygon` addresses
            // one spare slot. `write` initializes it before the length grows.
            unsafe {
                polygons.as_mut_ptr().add(stored_polygon).write(polygon);
                polygons.set_len(stored_polygon + 1);
            }
            polygon_index += 1;
        }
        if !edge_balance_is_certified {
            let local = decisions.isolated();
            let edge_balance = classify_indexed_edge_balance(&local, mesh);
            decisions.absorb(local.certainty());
            let edge_balance = edge_balance?;
            if edge_balance.boundary_edges != 0 {
                return Err(HypermeshError::OpenInput {
                    mesh_index,
                    boundary_edges: edge_balance.boundary_edges,
                });
            }
            if edge_balance.unbalanced_edges != 0 {
                return Err(HypermeshError::NonPwnInput {
                    mesh_index,
                    unbalanced_edges: edge_balance.unbalanced_edges,
                });
            }
            if let Some(native) = mesh.native {
                native.facts.valid_pwn.record(local.certainty());
            }
        }
    }

    crate::trace_dispatch!("build-polygon-soup", "complete");
    Ok(PolygonSoup {
        polygons,
        bounds,
        num_meshes: meshes.len(),
    })
}

fn canonical_position_indices(
    decisions: &DecisionContext,
    positions: &[Point3],
) -> HypermeshResult<Vec<usize>> {
    let exact_only = positions
        .iter()
        .all(PointCoordinates::has_exact_rational_coordinates);
    let mut interner = PointInterner::<()>::try_with_capacity(positions.len(), exact_only, false)?;
    let mut canonical_positions: Vec<&Point3> = Vec::with_capacity(positions.len());
    let mut canonical_indices = Vec::with_capacity(positions.len());
    for position in positions {
        canonical_indices.push(interner.intern_cloned(
            decisions,
            &mut canonical_positions,
            &position,
            None,
        )?);
    }
    Ok(canonical_indices)
}

fn classify_indexed_edge_balance(
    decisions: &DecisionContext,
    mesh: &TriangleMeshRef<'_>,
) -> HypermeshResult<EdgeBalance> {
    let canonical_indices = canonical_position_indices(decisions, mesh.positions)?;
    let mut edge_uses: StorageHashMap<[usize; 2], [usize; 2]> = StorageHashMap::default();
    for triangle in mesh.triangles {
        let [a, b, c] = triangle.indices().map(|index| canonical_indices[index]);
        for [start, end] in [[a, b], [b, c], [c, a]] {
            let (key, direction) = if start < end {
                ([start, end], 0)
            } else {
                ([end, start], 1)
            };
            edge_uses.entry(key).or_default()[direction] += 1;
        }
    }

    Ok(edge_uses
        .values()
        .fold(EdgeBalance::default(), |mut balance, uses| {
            if uses[0] + uses[1] == 1 {
                balance.boundary_edges += 1;
            }
            if uses[0] != uses[1] {
                balance.unbalanced_edges += 1;
            }
            balance
        }))
}

fn validate_non_empty_mesh_views(meshes: &[TriangleMeshRef<'_>]) -> HypermeshResult<()> {
    if meshes.is_empty() {
        return Err(HypermeshError::EmptyInput);
    }
    for (mesh_index, mesh) in meshes.iter().enumerate() {
        if mesh.positions.is_empty() || mesh.triangles.is_empty() {
            return Err(HypermeshError::EmptyMesh { mesh_index });
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct EdgeBalance {
    pub(crate) boundary_edges: usize,
    pub(crate) unbalanced_edges: usize,
}

fn bounds_for_positions<'a>(
    decisions: &DecisionContext,
    positions: impl IntoIterator<Item = &'a Point3>,
) -> HypermeshResult<Aabb> {
    let mut positions = positions.into_iter();
    let first = positions.next().ok_or(HypermeshError::EmptyInput)?;
    let mut min = first.clone();
    let mut max = first.clone();
    let first_coordinates = [&first.x, &first.y, &first.z];
    let mut exact_dyadic_bounds = match first_coordinates.map(Real::to_f64_exact_dyadic) {
        [Some(x), Some(y), Some(z)] => Some(([x, y, z], [x, y, z])),
        _ => None,
    };

    for position in positions {
        if let Some((min_f64, max_f64)) = &mut exact_dyadic_bounds {
            let coordinates = [&position.x, &position.y, &position.z];
            if let [Some(x), Some(y), Some(z)] = coordinates.map(Real::to_f64_exact_dyadic) {
                for (axis, value) in [x, y, z].into_iter().enumerate() {
                    if value < min_f64[axis] {
                        min_f64[axis] = value;
                        *crate::geometry::axis_mut(&mut min, axis) =
                            axis_ref(position, axis).clone();
                    }
                    if value > max_f64[axis] {
                        max_f64[axis] = value;
                        *crate::geometry::axis_mut(&mut max, axis) =
                            axis_ref(position, axis).clone();
                    }
                }
                continue;
            }
            exact_dyadic_bounds = None;
        }
        for axis in 0..3 {
            if compare_real_decision(decisions, axis_ref(position, axis), axis_ref(&min, axis))?
                .is_lt()
            {
                *crate::geometry::axis_mut(&mut min, axis) = axis_ref(position, axis).clone();
            }
            if compare_real_decision(decisions, axis_ref(position, axis), axis_ref(&max, axis))?
                .is_gt()
            {
                *crate::geometry::axis_mut(&mut max, axis) = axis_ref(position, axis).clone();
            }
        }
    }

    Ok(Aabb::new(min, max))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn approximate_pwn_facts_obey_later_operation_policy() {
        let mesh = tetrahedron_with_independent_face_indices()
            .with_valid_pwn_certainty(MeshCertainty::Approximate512Consumed);

        let strict_context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let strict = DecisionContext::new(&strict_context);
        assert!(!mesh.facts.valid_pwn.consume(&strict));
        assert_eq!(strict.certainty(), MeshCertainty::Certified);

        let approximate_context = MeshContext::new(hyperlimit::PredicatePolicy::APPROXIMATE_512);
        let approximate = DecisionContext::new(&approximate_context);
        assert!(mesh.facts.valid_pwn.consume(&approximate));
        assert_eq!(
            approximate.certainty(),
            MeshCertainty::Approximate512Consumed
        );

        mesh.facts.valid_pwn.record(MeshCertainty::Certified);
        let upgraded_strict = DecisionContext::new(&strict_context);
        assert!(mesh.facts.valid_pwn.consume(&upgraded_strict));
        assert_eq!(upgraded_strict.certainty(), MeshCertainty::Certified);
    }

    #[test]
    fn strict_soup_validation_upgrades_an_approximate_cached_pwn_fact() {
        let mesh = tetrahedron_with_independent_face_indices();
        mesh.facts
            .valid_pwn
            .record(MeshCertainty::Approximate512Consumed);

        let strict_context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let strict = DecisionContext::new(&strict_context);
        build_polygon_soup_internal(&strict, &[mesh.as_ref()]).unwrap();

        assert_eq!(
            mesh.facts.valid_pwn.certainty(),
            Some(MeshCertainty::Certified)
        );
        assert_eq!(strict.certainty(), MeshCertainty::Certified);
    }

    fn tetrahedron_with_independent_face_indices() -> TriangleMesh {
        let points = [
            Point3::new(Real::zero(), Real::zero(), Real::zero()),
            Point3::new(Real::one(), Real::zero(), Real::zero()),
            Point3::new(Real::zero(), Real::one(), Real::zero()),
            Point3::new(Real::zero(), Real::zero(), Real::one()),
        ];
        let faces = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
        let mut positions = Vec::new();
        let mut triangles = Vec::new();
        for face in faces {
            let base = positions.len();
            positions.extend(face.map(|index| points[index].clone()));
            triangles.push(Triangle::new(base, base + 1, base + 2));
        }
        TriangleMesh::new(positions, triangles)
    }

    #[test]
    fn exact_triangle_quality_canonicalizes_positions_and_rejects_duplicate_faces() {
        let mesh = tetrahedron_with_independent_face_indices();
        assert!(
            mesh.has_unique_nondegenerate_triangles_decision(
                &crate::test_support::approximate_decisions()
            )
            .unwrap()
        );
        assert!(
            mesh.is_closed_manifold_geometry_decision(
                &crate::test_support::approximate_decisions()
            )
            .unwrap()
        );
        assert!(!mesh.is_closed_manifold());

        let mut triangles = mesh.triangles.to_vec();
        triangles.push(triangles[0]);
        let duplicate = TriangleMesh::new(mesh.positions.to_vec(), triangles);
        assert!(
            !duplicate
                .has_unique_nondegenerate_triangles_decision(
                    &crate::test_support::approximate_decisions()
                )
                .unwrap()
        );
        assert!(
            !duplicate
                .is_closed_manifold_geometry_decision(&crate::test_support::approximate_decisions())
                .unwrap()
        );
    }

    #[test]
    fn geometric_closure_uses_numeric_point_equality() {
        let mesh = tetrahedron_with_independent_face_indices();
        let left = Real::pi() + Real::e();
        let equivalent_left = Real::e() + Real::pi();
        assert_ne!(left, equivalent_left);
        let positions = mesh
            .positions
            .iter()
            .enumerate()
            .map(|(index, point)| {
                let offset = if index % 2 == 0 {
                    left.clone()
                } else {
                    equivalent_left.clone()
                };
                Point3::new(&point.x + offset, point.y.clone(), point.z.clone())
            })
            .collect();
        let mesh = TriangleMesh::new(positions, mesh.triangles.to_vec());

        assert!(
            mesh.has_unique_nondegenerate_triangles_decision(
                &crate::test_support::approximate_decisions()
            )
            .unwrap()
        );
        assert!(
            mesh.is_closed_manifold_geometry_decision(
                &crate::test_support::approximate_decisions()
            )
            .unwrap()
        );
    }

    #[test]
    fn reversed_winding_shares_positions_without_retaining_derived_indices() {
        let mesh = TriangleMesh::new(
            vec![
                Point3::new(Real::zero(), Real::zero(), Real::zero()),
                Point3::new(Real::one(), Real::zero(), Real::zero()),
                Point3::new(Real::zero(), Real::one(), Real::zero()),
            ],
            vec![Triangle::new(0, 1, 2)],
        );

        let reversed = mesh.reversed_winding();
        let retained = mesh.reversed_winding();
        assert!(Arc::ptr_eq(&mesh.positions, &reversed.positions));
        assert!(!Arc::ptr_eq(&reversed.triangles, &retained.triangles));
        assert_eq!(reversed, retained);
        assert_eq!(mesh.triangles[0], Triangle::new(0, 1, 2));
        assert_eq!(reversed.triangles[0], Triangle::new(2, 1, 0));
    }

    #[test]
    fn mesh_fact_header_stays_compact() {
        assert!(
            std::mem::size_of::<TriangleMeshFacts>() <= 80,
            "cold mesh facts occupy {} bytes",
            std::mem::size_of::<TriangleMeshFacts>(),
        );
    }
    use hyperlattice::Rational;

    #[test]
    fn bounds_exact_dyadic_scan_falls_back_for_later_general_rational() {
        let one_third = Real::new(Rational::fraction(1, 3).unwrap());
        let points = [
            Point3::new(Real::from(4), Real::from(-2), Real::from(8)),
            Point3::new(Real::from(-3), Real::from(5), Real::from(1)),
            Point3::new(one_third.clone(), Real::from(-7), Real::from(9)),
        ];

        assert_eq!(
            bounds_for_positions(&crate::test_support::approximate_decisions(), &points).unwrap(),
            Aabb::new(
                Point3::new(Real::from(-3), Real::from(-7), Real::from(1)),
                Point3::new(Real::from(4), Real::from(5), Real::from(9)),
            ),
        );
    }

    #[test]
    fn indexed_edge_balance_canonicalizes_coincident_input_vertices() {
        let geometric = [
            Point3::new(Real::zero(), Real::zero(), Real::zero()),
            Point3::new(Real::one(), Real::zero(), Real::zero()),
            Point3::new(Real::zero(), Real::one(), Real::zero()),
            Point3::new(Real::zero(), Real::zero(), Real::one()),
        ];
        let faces = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]];
        let mut positions = Vec::new();
        let mut triangles = Vec::new();
        for face in faces {
            let start = positions.len();
            positions.extend(face.map(|index| geometric[index].clone()));
            triangles.push(Triangle::new(start, start + 1, start + 2));
        }
        let mesh = TriangleMesh::new(positions, triangles);

        assert_eq!(
            classify_indexed_edge_balance(
                &crate::test_support::approximate_decisions(),
                &mesh.as_ref()
            ),
            Ok(EdgeBalance::default())
        );
        crate::test_support::approximate_polygon_soup(&[mesh.as_ref()])
            .expect("closed coincident-index tetrahedron");
    }

    #[test]
    fn exact_native_queries_reject_invalid_triangle_indices() {
        let mesh = TriangleMesh::new(
            vec![
                Point3::new(Real::zero(), Real::zero(), Real::zero()),
                Point3::new(Real::one(), Real::zero(), Real::zero()),
                Point3::new(Real::zero(), Real::one(), Real::zero()),
            ],
            vec![Triangle::new(0, 1, 3)],
        );
        let expected = HypermeshError::VertexIndexOutOfBounds {
            index: 3,
            vertex_count: 3,
        };

        assert_eq!(
            mesh.ray_intersections_decision(
                &crate::test_support::approximate_decisions(),
                &Point3::new(Real::zero(), Real::zero(), Real::one()),
                &Vector3::from_xyz(Real::zero(), Real::zero(), -Real::one()),
            ),
            Err(expected.clone()),
        );
        assert_eq!(
            mesh.contains_point_decision(
                &crate::test_support::approximate_decisions(),
                &Point3::new(
                    Real::from(Rational::fraction(1, 4).unwrap()),
                    Real::from(Rational::fraction(1, 4).unwrap()),
                    Real::zero(),
                )
            ),
            Err(expected),
        );
    }
}
