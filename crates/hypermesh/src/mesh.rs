//! Canonical owned triangle geometry and kernel polygon-soup preparation.

use std::collections::{BTreeSet, HashMap};
use std::num::NonZeroU32;
use std::sync::{Arc, Mutex, OnceLock};

use hyperlattice::{Aabb as ExactAabb, Matrix4, Point3, Rational, Real, RealSign, Vector3};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Aabb, Classification, Plane, axis_ref, classify_point, compare_real};
use crate::output::TriangleSource;
use crate::polygon::{
    ConvexPolygon, InputTrianglePlanes, convex_triangle, edge_plane,
    exact_axis_aligned_triangle_support, make_indexed_triangle_with_deferred_edges,
    make_indexed_triangle_with_deferred_edges_and_input_planes, make_triangle_with_input_planes,
};
use crate::storage_hash::StorageHashMap;
use crate::winding::BooleanOp;

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
    input_planes: OnceLock<Option<Arc<[InputTrianglePlanes]>>>,
    input_plane_sources: Option<Arc<[TriangleSource]>>,
    input_polygons: OnceLock<Option<Arc<[ConvexPolygon]>>>,
    exact_bounds: OnceLock<Option<ExactAabb>>,
    adjacency: OnceLock<Vec<Vec<usize>>>,
    connectivity_counts: OnceLock<(usize, usize)>,
    closed_manifold: OnceLock<bool>,
    finite_positions: OnceLock<Option<Vec<[f64; 3]>>>,
    finite_materialization: OnceLock<Option<TriangleMesh>>,
    pub(crate) exact_gpu:
        OnceLock<Result<crate::gpu::ExactGpuMeshBuffers, crate::gpu::GpuMeshError>>,
    pub(crate) gpu_f32: OnceLock<Result<crate::gpu::GpuMeshBuffersF32, crate::gpu::GpuMeshError>>,
    pub(crate) gpu_f64: OnceLock<Result<crate::gpu::GpuMeshBuffersF64, crate::gpu::GpuMeshError>>,
    certified_convex: OnceLock<bool>,
    axis_aligned_box: OnceLock<Option<ExactAabb>>,
    convex_hull: OnceLock<Result<TriangleMesh, HypermeshError>>,
    transforms: Mutex<Vec<(Matrix4, TriangleMesh)>>,
    rotations: Mutex<Vec<([Real; 3], TriangleMesh)>>,
    reversed_winding: OnceLock<TriangleMesh>,
    subdivisions: Mutex<Vec<(NonZeroU32, TriangleMesh)>>,
    boolean_results: Mutex<Vec<(Arc<[Point3]>, Arc<[Triangle]>, BooleanOp, TriangleMesh)>>,
    ray_queries: Mutex<Vec<(Point3, Vector3, Arc<[(Point3, Real)]>)>>,
    containment_queries: Mutex<Vec<(Point3, bool)>>,
    laplacian_smoothings: Mutex<Vec<(Real, usize, TriangleMesh)>>,
    unique_nondegenerate_triangles: OnceLock<bool>,
}

/// Canonical owned triangle mesh.
///
/// This is the reusable geometry carrier accepted by Hypermesh operations and
/// produced from [`crate::BooleanMesh`] results. Reusable exact construction
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

    pub(crate) fn with_boolean_provenance(
        mut self,
        sources: Vec<TriangleSource>,
        polygons: Vec<ConvexPolygon>,
    ) -> Self {
        debug_assert_eq!(sources.len(), self.triangles.len());
        if sources.len() == self.triangles.len() {
            let facts = TriangleMeshFacts {
                input_plane_sources: Some(sources.into()),
                ..TriangleMeshFacts::default()
            };
            let _ = facts.input_polygons.set(Some(polygons.into()));
            self.facts = Arc::new(facts);
        }
        self
    }

    pub(crate) fn retained_input_planes(&self) -> Option<&[InputTrianglePlanes]> {
        self.facts
            .input_planes
            .get_or_init(|| self.build_retained_input_planes().map(Arc::from))
            .as_deref()
    }

    pub(crate) fn retained_input_polygons(&self) -> Option<&[ConvexPolygon]> {
        self.facts.input_polygons.get()?.as_deref()
    }

    fn build_retained_input_planes(&self) -> Option<Vec<InputTrianglePlanes>> {
        let sources = self.facts.input_plane_sources.as_deref()?;
        let polygons = self.retained_input_polygons()?;
        if sources.len() != self.triangles.len() {
            return None;
        }
        let polygon_by_source = polygons
            .iter()
            .map(|polygon| (polygon.polygon_index, polygon))
            .collect::<HashMap<_, _>>();
        self.triangles
            .iter()
            .zip(sources)
            .map(|(triangle, source)| {
                let [a, b, c] = triangle.indices();
                let [p0, p1, p2] = [
                    self.positions.get(a)?,
                    self.positions.get(b)?,
                    self.positions.get(c)?,
                ];
                let source_polygon = polygon_by_source.get(&source.triangle).copied();
                let support = source_polygon
                    .map(|polygon| polygon.support.clone())
                    .unwrap_or_else(|| Plane::from_points(p0, p1, p2));
                let source_edges = source_polygon
                    .map(|polygon| polygon.edges.as_slice())
                    .unwrap_or(&[]);
                let retained_edge = |a: &Point3, b: &Point3, opposite: &Point3| {
                    source_edges
                        .iter()
                        .find_map(|plane| oriented_retained_edge_plane(a, b, opposite, plane))
                        .unwrap_or_else(|| {
                            edge_plane(a, b, opposite, &support).normalized_projective_scale()
                        })
                };
                Some(InputTrianglePlanes {
                    edges: [
                        retained_edge(p0, p1, p2),
                        retained_edge(p1, p2, p0),
                        retained_edge(p2, p0, p1),
                    ],
                    support,
                })
            })
            .collect()
    }

    /// Returns a borrowed mesh view.
    pub fn as_ref(&self) -> TriangleMeshRef<'_> {
        TriangleMeshRef {
            positions: &self.positions,
            triangles: &self.triangles,
        }
    }

    /// Builds native vertex adjacency from triangle index rows.
    pub fn adjacency(&self) -> &[Vec<usize>] {
        self.facts.adjacency.get_or_init(|| {
            let mut adjacency = vec![BTreeSet::new(); self.positions.len()];
            for triangle in self.triangles.iter() {
                let [a, b, c] = triangle.indices();
                if [a, b, c].into_iter().any(|index| index >= adjacency.len()) {
                    continue;
                }
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
        })
    }

    /// Returns retained `(position rows, directed adjacency entries)` counts.
    ///
    /// This is the compact topology fact for callers that need connectivity
    /// diagnostics without walking every retained adjacency row again.
    pub fn connectivity_counts(&self) -> (usize, usize) {
        *self.facts.connectivity_counts.get_or_init(|| {
            (
                self.positions.len(),
                self.adjacency().iter().map(Vec::len).sum(),
            )
        })
    }

    /// Checks indexed edge pairing for a closed, consistently oriented
    /// two-manifold.
    pub fn is_closed_manifold(&self) -> bool {
        *self.facts.closed_manifold.get_or_init(|| {
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
        })
    }

    /// Returns true when every triangle has valid, nondegenerate exact
    /// geometry and no two triangles cover the same three exact points.
    ///
    /// Position rows are canonicalized by exact coordinate equality before
    /// triangle keys are compared, so independently indexed duplicate faces
    /// are rejected as well.
    pub fn has_unique_nondegenerate_triangles(&self) -> bool {
        *self.facts.unique_nondegenerate_triangles.get_or_init(|| {
            let Ok(canonical_indices) = canonical_position_indices(&self.positions) else {
                return false;
            };
            let mut seen = BTreeSet::new();
            for triangle in self.triangles.iter() {
                let indices = triangle.indices();
                let [Some(a), Some(b), Some(c)] = indices.map(|index| self.positions.get(index))
                else {
                    return false;
                };
                let mut key = indices.map(|index| canonical_indices[index]);
                if key[0] == key[1]
                    || key[1] == key[2]
                    || key[0] == key[2]
                    || !Plane::points_are_nondegenerate(a, b, c)
                {
                    return false;
                }
                key.sort_unstable();
                if !seen.insert(key) {
                    return false;
                }
            }
            true
        })
    }

    /// Checks exact geometric edge pairing for a closed, consistently
    /// oriented two-manifold.
    ///
    /// Unlike [`Self::is_closed_manifold`], this canonicalizes independently
    /// indexed position rows by exact coordinate equality. Duplicate faces,
    /// degenerate faces, and geometrically non-manifold edge valence are
    /// rejected.
    pub fn is_closed_manifold_geometry(&self) -> bool {
        if self.triangles.is_empty() || !self.has_unique_nondegenerate_triangles() {
            return false;
        }
        let Ok(canonical_indices) = canonical_position_indices(&self.positions) else {
            return false;
        };
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
        edges.values().all(|uses| *uses == [1, 1])
    }

    /// Returns retained exact bounds, or `None` for empty geometry.
    pub fn exact_bounds(&self) -> Option<&ExactAabb> {
        self.facts
            .exact_bounds
            .get_or_init(|| {
                let first = self.positions.first()?.clone();
                let mut bounds = ExactAabb::new(first.clone(), first);
                for point in &self.positions[1..] {
                    bounds.mins.x = bounds.mins.x.min(&point.x).clone();
                    bounds.mins.y = bounds.mins.y.min(&point.y).clone();
                    bounds.mins.z = bounds.mins.z.min(&point.z).clone();
                    bounds.maxs.x = bounds.maxs.x.max(&point.x).clone();
                    bounds.maxs.y = bounds.maxs.y.max(&point.y).clone();
                    bounds.maxs.z = bounds.maxs.z.max(&point.z).clone();
                }
                Some(bounds)
            })
            .as_ref()
    }

    /// Returns a retained finite projection of every exact position.
    ///
    /// This is an explicit approximation boundary for renderers, exporters,
    /// diagnostics, and benchmarks. Native geometry remains exact.
    pub fn finite_positions(&self) -> Option<&[[f64; 3]]> {
        self.facts
            .finite_positions
            .get_or_init(|| {
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
            })
            .as_deref()
    }

    /// Returns retained geometry whose coordinates are exact promotions of
    /// this mesh's finite binary64 projection.
    ///
    /// `None` means at least one native coordinate has no finite projection.
    pub fn materialize_finite(&self) -> Option<Self> {
        self.facts
            .finite_materialization
            .get_or_init(|| {
                let positions = self
                    .finite_positions()?
                    .iter()
                    .map(|position| {
                        Some(Point3::new(
                            Real::try_from(position[0]).ok()?,
                            Real::try_from(position[1]).ok()?,
                            Real::try_from(position[2]).ok()?,
                        ))
                    })
                    .collect::<Option<Vec<_>>>()?;
                Some(Self::new(positions, self.triangles.to_vec()))
            })
            .clone()
    }

    /// Records that the constructor certifies this mesh as convex.
    ///
    /// Callers only provide the constructor-owned convexity fact; Hypermesh
    /// remains responsible for all per-operation support geometry.
    #[doc(hidden)]
    pub fn with_certified_convexity(self) -> Self {
        let _ = self.facts.certified_convex.set(true);
        self
    }

    /// Certifies this mesh as a closed, outward-oriented convex PWN and
    /// retains that fact for subsequent native Boolean operations.
    pub fn try_certify_convex(self) -> HypermeshResult<Self> {
        certify_convex_mesh(self.as_ref())?;
        Ok(self.with_certified_convexity())
    }

    /// Returns the retained exact convex hull of this mesh's native positions.
    pub fn convex_hull(&self) -> HypermeshResult<Self> {
        self.facts
            .convex_hull
            .get_or_init(|| crate::convex_hull(&self.positions))
            .clone()
    }

    pub(crate) fn has_certified_convex_fact(&self) -> bool {
        self.facts.certified_convex.get().copied().unwrap_or(false)
    }

    /// Returns a retained Boolean result for the same immutable operand pair.
    #[doc(hidden)]
    pub fn retained_boolean_result(&self, other: &Self, operation: BooleanOp) -> Option<Self> {
        self.facts
            .boolean_results
            .lock()
            .ok()?
            .iter()
            .find(|(positions, triangles, cached_operation, _)| {
                Arc::ptr_eq(positions, &other.positions)
                    && Arc::ptr_eq(triangles, &other.triangles)
                    && *cached_operation == operation
            })
            .map(|(_, _, _, result)| result.clone())
    }

    /// Retains a reusable Boolean result for this immutable operand pair.
    #[doc(hidden)]
    pub fn retain_boolean_result(&self, other: &Self, operation: BooleanOp, result: &Self) {
        if Arc::ptr_eq(&self.facts, &result.facts) || Arc::ptr_eq(&other.facts, &result.facts) {
            return;
        }
        if let Ok(mut results) = self.facts.boolean_results.lock() {
            if let Some((_, _, _, cached)) =
                results
                    .iter_mut()
                    .find(|(positions, triangles, cached_operation, _)| {
                        Arc::ptr_eq(positions, &other.positions)
                            && Arc::ptr_eq(triangles, &other.triangles)
                            && *cached_operation == operation
                    })
            {
                *cached = result.clone();
                return;
            }
            const CAPACITY: usize = 8;
            if results.len() == CAPACITY {
                results.remove(0);
            }
            results.push((
                Arc::clone(&other.positions),
                Arc::clone(&other.triangles),
                operation,
                result.clone(),
            ));
        }
    }

    /// Returns retained exact bounds when the native rows form a complete
    /// axis-aligned box surface.
    pub fn axis_aligned_box_bounds(&self) -> Option<&ExactAabb> {
        self.facts
            .axis_aligned_box
            .get_or_init(|| {
                if self.positions.len() != 8 || self.triangles.len() != 12 {
                    return None;
                }
                let bounds = self.exact_bounds()?.clone();
                if bounds.mins.x == bounds.maxs.x
                    || bounds.mins.y == bounds.maxs.y
                    || bounds.mins.z == bounds.maxs.z
                {
                    return None;
                }
                let mut corners = [false; 8];
                for point in self.positions.iter() {
                    let mut corner = 0;
                    for (axis, (value, minimum, maximum)) in [
                        (&point.x, &bounds.mins.x, &bounds.maxs.x),
                        (&point.y, &bounds.mins.y, &bounds.maxs.y),
                        (&point.z, &bounds.mins.z, &bounds.maxs.z),
                    ]
                    .into_iter()
                    .enumerate()
                    {
                        if value == maximum {
                            corner |= 1 << axis;
                        } else if value != minimum {
                            return None;
                        }
                    }
                    if std::mem::replace(&mut corners[corner], true) {
                        return None;
                    }
                }
                if corners.iter().any(|present| !present) {
                    return None;
                }
                let mut face_triangles = [0_u8; 6];
                for triangle in self.triangles.iter() {
                    let [a, b, c] = triangle.indices();
                    let points = [
                        self.positions.get(a)?,
                        self.positions.get(b)?,
                        self.positions.get(c)?,
                    ];
                    let face = [
                        (&bounds.mins.x, &bounds.maxs.x),
                        (&bounds.mins.y, &bounds.maxs.y),
                        (&bounds.mins.z, &bounds.maxs.z),
                    ]
                    .into_iter()
                    .enumerate()
                    .find_map(|(axis, (minimum, maximum))| {
                        let coordinate_matches = |point: &&Point3, value: &Real| match axis {
                            0 => &point.x == value,
                            1 => &point.y == value,
                            _ => &point.z == value,
                        };
                        if points
                            .iter()
                            .all(|point| coordinate_matches(point, minimum))
                        {
                            Some(axis * 2)
                        } else if points
                            .iter()
                            .all(|point| coordinate_matches(point, maximum))
                        {
                            Some(axis * 2 + 1)
                        } else {
                            None
                        }
                    })?;
                    face_triangles[face] = face_triangles[face].saturating_add(1);
                }
                (face_triangles == [2; 6]).then_some(bounds)
            })
            .as_ref()
    }

    /// Applies and retains an exact homogeneous transform.
    ///
    /// Repeated transforms of the same immutable native geometry share the
    /// transformed position and triangle buffers.
    pub fn try_transformed(&self, matrix: &Matrix4) -> Option<Self> {
        if let Ok(transforms) = self.facts.transforms.lock()
            && let Some((_, transformed)) = transforms.iter().find(|(cached, _)| cached == matrix)
        {
            return Some(transformed.clone());
        }
        let positions = matrix.transform_point3_batch(&self.positions).ok()?;
        let mut transformed = Self::new(positions, self.triangles.to_vec());
        if self.has_certified_convex_fact() {
            transformed = transformed.with_certified_convexity();
        }
        if let Ok(mut transforms) = self.facts.transforms.lock() {
            const CAPACITY: usize = 32;
            if transforms.len() == CAPACITY {
                transforms.remove(0);
            }
            transforms.push((matrix.clone(), transformed.clone()));
        }
        Some(transformed)
    }

    /// Reverses every triangle while sharing the immutable position buffer.
    ///
    /// The derived orientation is retained, so repeated requests on the same
    /// native mesh share both indexed buffers.
    pub fn reversed_winding(&self) -> Self {
        self.facts
            .reversed_winding
            .get_or_init(|| Self {
                positions: Arc::clone(&self.positions),
                triangles: self
                    .triangles
                    .iter()
                    .map(|triangle| Triangle::new(triangle.v2, triangle.v1, triangle.v0))
                    .collect::<Vec<_>>()
                    .into(),
                facts: Arc::new(TriangleMeshFacts::default()),
            })
            .clone()
    }

    /// Applies and retains an exact `Rz * Ry * Rx` Euler rotation in degrees.
    ///
    /// Returns `None` when a transformed coordinate cannot be produced rather
    /// than substituting the unchanged input mesh.
    pub fn try_rotated_xyz_degrees(&self, x: Real, y: Real, z: Real) -> Option<Self> {
        let degrees = [x, y, z];
        if let Ok(rotations) = self.facts.rotations.lock()
            && let Some((_, transformed)) = rotations.iter().find(|(cached, _)| cached == &degrees)
        {
            return Some(transformed.clone());
        }
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
        let transformed = self.try_transformed(&matrix)?;
        if let Ok(mut rotations) = self.facts.rotations.lock() {
            const CAPACITY: usize = 8;
            if rotations.len() == CAPACITY {
                rotations.remove(0);
            }
            rotations.push((degrees, transformed.clone()));
        }
        Some(transformed)
    }

    /// Applies an exact Euler rotation, retaining the compatibility behavior
    /// of returning the unchanged mesh if coordinate transformation fails.
    pub fn rotated_xyz_degrees(&self, x: Real, y: Real, z: Real) -> Self {
        self.try_rotated_xyz_degrees(x, y, z)
            .unwrap_or_else(|| self.clone())
    }

    /// Uniformly subdivides every triangle while sharing edge midpoints.
    ///
    /// Each level replaces one triangle with four consistently oriented
    /// triangles. Midpoints are indexed once per undirected edge, so adjacent
    /// input triangles remain adjacent in the result.
    pub fn subdivide_triangles(&self, levels: NonZeroU32) -> Self {
        if let Ok(subdivisions) = self.facts.subdivisions.lock()
            && let Some((_, mesh)) = subdivisions
                .iter()
                .find(|(cached_levels, _)| *cached_levels == levels)
        {
            return mesh.clone();
        }

        let mut positions = self.positions.to_vec();
        let mut triangles = self.triangles.to_vec();
        let two = Real::from(2_u8);
        for _ in 0..levels.get() {
            let mut midpoints = HashMap::<(usize, usize), usize>::new();
            let mut refined = Vec::with_capacity(triangles.len().saturating_mul(4));
            for triangle in triangles {
                let [a, b, c] = triangle.indices();
                if a >= positions.len() || b >= positions.len() || c >= positions.len() {
                    continue;
                }
                let mut midpoint = |left: usize, right: usize| {
                    let edge = if left < right {
                        (left, right)
                    } else {
                        (right, left)
                    };
                    *midpoints.entry(edge).or_insert_with(|| {
                        let left = &positions[edge.0];
                        let right = &positions[edge.1];
                        let point = Point3::new(
                            ((&left.x + &right.x) / &two)
                                .expect("two is a nonzero midpoint divisor"),
                            ((&left.y + &right.y) / &two)
                                .expect("two is a nonzero midpoint divisor"),
                            ((&left.z + &right.z) / &two)
                                .expect("two is a nonzero midpoint divisor"),
                        );
                        let index = positions.len();
                        positions.push(point);
                        index
                    })
                };
                let ab = midpoint(a, b);
                let bc = midpoint(b, c);
                let ca = midpoint(c, a);
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
        if self.is_closed_manifold() {
            let _ = mesh.facts.closed_manifold.set(true);
        }
        if self.has_certified_convex_fact() {
            let _ = mesh.facts.certified_convex.set(true);
        }
        if let Ok(mut subdivisions) = self.facts.subdivisions.lock() {
            const CAPACITY: usize = 4;
            if subdivisions.len() == CAPACITY {
                subdivisions.remove(0);
            }
            subdivisions.push((levels, mesh.clone()));
        }
        mesh
    }

    /// Returns exact ray/triangle intersections sorted by ray parameter.
    ///
    /// Coplanar ray/triangle overlap has no unique point and is therefore not
    /// emitted. Exact edge and vertex contacts are deduplicated by point
    /// identity after all triangle reports have been collected.
    pub fn ray_intersections(
        &self,
        origin: &Point3,
        direction: &Vector3,
    ) -> HypermeshResult<Vec<(Point3, Real)>> {
        if let Ok(queries) = self.facts.ray_queries.lock()
            && let Some((_, _, hits)) =
                queries.iter().find(|(cached_origin, cached_direction, _)| {
                    cached_origin == origin && cached_direction == direction
                })
        {
            return Ok(hits.to_vec());
        }
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
            let report = hyperlimit::classify_ray_triangle3_intersection_report(
                &origin_limit,
                &direction_limit,
                &vertices[0],
                &vertices[1],
                &vertices[2],
            )
            .value()
            .ok_or(HypermeshError::UnknownClassification)?;
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
            while current > 0 && compare_real(&hits[current - 1].1, &hits[current].1)?.is_gt() {
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
                hyperlimit::point3_equal(&left, &right)
                    .value()
                    .ok_or(HypermeshError::UnknownClassification)?
            } else {
                false
            };
            if !duplicate {
                unique_hits.push(hit);
            }
        }
        if let Ok(mut queries) = self.facts.ray_queries.lock() {
            const CAPACITY: usize = 8;
            if queries.len() == CAPACITY {
                queries.remove(0);
            }
            queries.push((
                origin.clone(),
                direction.clone(),
                Arc::from(unique_hits.as_slice()),
            ));
        }
        Ok(unique_hits)
    }

    /// Tests strict solid containment using exact boundary predicates and ray
    /// parity. Points on a triangle, edge, or vertex are not inside.
    pub fn contains_point(&self, point: &Point3) -> HypermeshResult<bool> {
        if let Ok(queries) = self.facts.containment_queries.lock()
            && let Some((_, contains)) = queries
                .iter()
                .find(|(cached_point, _)| cached_point == point)
        {
            return Ok(*contains);
        }
        let contains = self.contains_point_uncached(point)?;
        if let Ok(mut queries) = self.facts.containment_queries.lock() {
            const CAPACITY: usize = 8;
            if queries.len() == CAPACITY {
                queries.remove(0);
            }
            queries.push((point.clone(), contains));
        }
        Ok(contains)
    }

    fn contains_point_uncached(&self, point: &Point3) -> HypermeshResult<bool> {
        if self.triangles.is_empty() {
            return Ok(false);
        }
        if let Some(bounds) = self.exact_bounds() {
            for (coordinate, minimum, maximum) in [
                (&point.x, &bounds.mins.x, &bounds.maxs.x),
                (&point.y, &bounds.mins.y, &bounds.maxs.y),
                (&point.z, &bounds.mins.z, &bounds.maxs.z),
            ] {
                if compare_real(coordinate, minimum)?.is_lt()
                    || compare_real(coordinate, maximum)?.is_gt()
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
            let location = hyperlimit::classify_point_triangle3(
                &vertices[0],
                &vertices[1],
                &vertices[2],
                &query,
            )
            .value()
            .ok_or(HypermeshError::UnknownClassification)?;
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
        Ok(self.ray_intersections(point, &direction)?.len() % 2 == 1)
    }

    /// Intersects consecutive polyline segments with native triangles and
    /// returns exact points in polyline order.
    pub fn polyline_intersections(&self, polyline: &[Point3]) -> HypermeshResult<Vec<Point3>> {
        let mut output: Vec<Point3> = Vec::new();
        for segment in polyline.windows(2) {
            let direction = &segment[1] - &segment[0];
            for (point, parameter) in self.ray_intersections(&segment[0], &direction)? {
                if compare_real(&parameter, &Real::zero())?.is_lt()
                    || compare_real(&parameter, &Real::one())?.is_gt()
                {
                    continue;
                }
                let duplicate = if let Some(last) = output.last() {
                    let left =
                        hyperlimit::Point3::new(last.x.clone(), last.y.clone(), last.z.clone());
                    let right =
                        hyperlimit::Point3::new(point.x.clone(), point.y.clone(), point.z.clone());
                    hyperlimit::point3_equal(&left, &right)
                        .value()
                        .ok_or(HypermeshError::UnknownClassification)?
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
    pub fn dihedral_angle(&self, first: Triangle, second: Triangle) -> Option<Real> {
        let normal = |triangle: Triangle| {
            let [a, b, c] = triangle.indices();
            let a = self.positions.get(a)?;
            let b = self.positions.get(b)?;
            let c = self.positions.get(c)?;
            (b - a).cross(&(c - a)).normalize_checked().ok()
        };
        normal(first)?.angle_to(&normal(second)?).ok()
    }

    /// Applies exact Laplacian smoothing to native positions.
    ///
    /// Triangle indexing is retained. Boundary preservation is intentionally
    /// not inferred here; callers requiring constrained smoothing should
    /// supply an explicit constraint set in a higher-level algorithm.
    pub fn laplacian_smooth(&self, lambda: &Real, iterations: usize) -> Self {
        if let Ok(smoothings) = self.facts.laplacian_smoothings.lock()
            && let Some((_, _, mesh)) =
                smoothings
                    .iter()
                    .find(|(cached_lambda, cached_iterations, _)| {
                        cached_lambda == lambda && *cached_iterations == iterations
                    })
        {
            return mesh.clone();
        }
        let adjacency = self.adjacency();
        let mut positions = self.positions.to_vec();
        for _ in 0..iterations {
            let previous = positions.clone();
            for (index, neighbors) in adjacency.iter().enumerate() {
                if neighbors.is_empty() {
                    continue;
                }
                let mut sum = hyperlattice::Vector3::zero();
                for &neighbor in neighbors {
                    sum = sum + previous[neighbor].to_vector();
                }
                let count = Real::from(neighbors.len() as u64);
                let average =
                    (sum / count).expect("a nonempty adjacency row has a nonzero divisor");
                let current = previous[index].to_vector();
                positions[index] =
                    Point3::origin() + current.clone() + (average - current) * lambda.clone();
            }
        }
        let mesh = Self::new(positions, self.triangles.to_vec());
        if let Ok(mut smoothings) = self.facts.laplacian_smoothings.lock() {
            const CAPACITY: usize = 8;
            if smoothings.len() == CAPACITY {
                smoothings.remove(0);
            }
            smoothings.push((lambda.clone(), iterations, mesh.clone()));
        }
        mesh
    }

    /// Applies alternating Laplacian shrink and inflation passes.
    ///
    /// Each iteration applies `lambda` followed by `mu`, retaining the native
    /// triangle indexing while reducing the volume loss of one-sided
    /// Laplacian smoothing.
    pub fn taubin_smooth(&self, lambda: &Real, mu: &Real, iterations: usize) -> Self {
        let mut mesh = self.clone();
        for _ in 0..iterations {
            mesh = mesh.laplacian_smooth(lambda, 1);
            mesh = mesh.laplacian_smooth(mu, 1);
        }
        mesh
    }
}

fn oriented_retained_edge_plane(
    a: &Point3,
    b: &Point3,
    opposite: &Point3,
    plane: &Plane,
) -> Option<Plane> {
    if classify_point(a, plane).ok()? != Classification::On
        || classify_point(b, plane).ok()? != Classification::On
    {
        return None;
    }
    match classify_point(opposite, plane).ok()? {
        Classification::Negative => Some(plane.clone()),
        Classification::Positive => Some(plane.inverted()),
        Classification::On => None,
    }
}

/// Borrowed triangle mesh view.
#[derive(Clone, Copy, Debug)]
pub struct TriangleMeshRef<'a> {
    /// Borrowed positions.
    pub positions: &'a [Point3],
    /// Borrowed triangles.
    pub triangles: &'a [Triangle],
}

/// Exact vertex emitted while materializing a Boolean result.
#[derive(Clone, Debug, PartialEq)]
pub struct OutputVertex {
    /// X coordinate.
    pub x: Real,
    /// Y coordinate.
    pub y: Real,
    /// Z coordinate.
    pub z: Real,
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
    pub fn compute_bounds_from_vertices(&mut self) -> HypermeshResult<()> {
        let mut vertices = Vec::new();
        for polygon in &self.polygons {
            vertices.extend(polygon.vertices()?);
        }
        self.bounds = bounds_for_positions(vertices.iter())?;
        Ok(())
    }
}

struct AdjacentSupportEdges {
    heads: Vec<usize>,
    entries: Vec<[usize; 5]>,
}

impl AdjacentSupportEdges {
    const NONE: usize = usize::MAX;

    fn new(vertex_count: usize, edge_capacity: usize) -> Self {
        Self {
            heads: vec![Self::NONE; vertex_count],
            entries: Vec::with_capacity(edge_capacity),
        }
    }

    #[inline]
    fn get(&self, start: usize, end: usize) -> Option<(usize, usize, usize)> {
        let (head, other) = if start < end {
            (start, end)
        } else {
            (end, start)
        };
        let mut entry_index = self.heads[head];
        while entry_index != Self::NONE {
            let [stored_other, stored_start, stored_end, polygon, next] = self.entries[entry_index];
            if stored_other == other {
                return Some((stored_start, stored_end, polygon));
            }
            entry_index = next;
        }
        None
    }

    #[inline]
    fn insert_if_absent(&mut self, start: usize, end: usize, polygon: usize) {
        if self.get(start, end).is_some() {
            return;
        }
        let (head, other) = if start < end {
            (start, end)
        } else {
            (end, start)
        };
        let next = self.heads[head];
        self.heads[head] = self.entries.len();
        self.entries.push([other, start, end, polygon, next]);
    }
}

/// Validates borrowed mesh views and returns their combined polygon soup.
pub fn polygon_soup(meshes: &[TriangleMeshRef<'_>]) -> HypermeshResult<PolygonSoup> {
    build_polygon_soup_with_edge_mode(meshes, None, None, false)
}

/// Validates a closed PWN mesh and certifies that every vertex lies in every
/// outward-oriented face half-space.
///
/// A successful result may be retained by mesh owners as a reusable convexity
/// fact for subsequent Boolean operations.
pub fn certify_convex_mesh(mesh: TriangleMeshRef<'_>) -> HypermeshResult<()> {
    let soup = polygon_soup(&[mesh])?;
    for polygon in &soup.polygons {
        for point in mesh.positions {
            if crate::predicate::classify_point(point, &polygon.support)?
                == crate::geometry::Classification::Positive
            {
                return Err(HypermeshError::NonConvexInput);
            }
        }
    }
    Ok(())
}

pub(crate) fn build_polygon_soup_with_certified_convex_inputs(
    meshes: &[TriangleMeshRef<'_>],
    certified_convex_inputs: &[bool],
    input_planes: Option<&[&[InputTrianglePlanes]]>,
) -> HypermeshResult<PolygonSoup> {
    build_polygon_soup_with_edge_mode(meshes, Some(certified_convex_inputs), input_planes, false)
}

pub(crate) fn build_polygon_soup_with_deferred_edges(
    meshes: &[TriangleMeshRef<'_>],
    certified_convex_inputs: &[bool],
    input_planes: Option<&[&[InputTrianglePlanes]]>,
) -> HypermeshResult<PolygonSoup> {
    build_polygon_soup_with_edge_mode(meshes, Some(certified_convex_inputs), input_planes, true)
}

fn build_polygon_soup_with_edge_mode(
    meshes: &[TriangleMeshRef<'_>],
    certified_convex_inputs: Option<&[bool]>,
    input_planes: Option<&[&[InputTrianglePlanes]]>,
    defer_edges: bool,
) -> HypermeshResult<PolygonSoup> {
    crate::trace_dispatch!("build-polygon-soup", "start");
    if certified_convex_inputs.is_some_and(|certified| certified.len() != meshes.len()) {
        return Err(HypermeshError::UnknownClassification);
    }
    if input_planes.is_some_and(|planes| {
        planes.len() != meshes.len()
            || planes
                .iter()
                .zip(meshes)
                .any(|(planes, mesh)| planes.len() != mesh.triangles.len())
    }) {
        return Err(HypermeshError::UnknownClassification);
    }
    validate_non_empty_mesh_views(meshes)?;

    let bounds = bounds_for_positions(meshes.iter().flat_map(|mesh| mesh.positions.iter()))?;
    crate::trace_dispatch!("build-polygon-soup", "bounds-computed");

    let polygon_capacity = meshes
        .iter()
        .try_fold(0usize, |total, mesh| {
            total.checked_add(mesh.triangles.len())
        })
        .ok_or(HypermeshError::UnknownClassification)?;
    let mut polygons = Vec::with_capacity(polygon_capacity);
    let mut polygon_index = 0isize;
    // Deferred source triangles have no materialized boundary planes. Keep
    // that immutable empty cycle once for the whole input instead of
    // allocating an identical Arc<Vec<_>> for every triangle.
    let deferred_edges = defer_edges.then(|| Arc::new(Vec::<Plane>::new()));
    for (mesh_index, mesh) in meshes.iter().enumerate() {
        let input_is_certified_convex =
            certified_convex_inputs.is_some_and(|certified| certified[mesh_index]);
        let retained_positions = (defer_edges && input_is_certified_convex)
            .then(|| Arc::<[Point3]>::from(mesh.positions));
        // Bound the admission scan before retaining an approximate position
        // cache. A missed axis face only skips the fast path, and every hint
        // is revalidated exactly when its support plane is constructed.
        let sample_count = mesh.triangles.len().min(64);
        let predominantly_axis_aligned = retained_positions.is_some()
            && (0..sample_count).all(|sample| {
                let triangle_index = sample * mesh.triangles.len() / sample_count;
                approximate_triangle_axis(mesh.positions, mesh.triangles[triangle_index].indices())
                    .is_some()
            });
        let (approximate_positions, approximate_positions_are_exact_dyadic) =
            if predominantly_axis_aligned {
                let exact_dyadic = mesh
                    .positions
                    .iter()
                    .map(|point| {
                        let coordinates = [&point.x, &point.y, &point.z];
                        let [Some(x), Some(y), Some(z)] =
                            coordinates.map(Real::to_f64_exact_dyadic)
                        else {
                            return None;
                        };
                        Some([x, y, z])
                    })
                    .collect::<Option<Vec<_>>>();
                match exact_dyadic {
                    Some(positions) => (Some(positions), true),
                    None => (
                        mesh.positions
                            .iter()
                            .map(|point| {
                                Some([
                                    point.x.to_f64_lossy()?,
                                    point.y.to_f64_lossy()?,
                                    point.z.to_f64_lossy()?,
                                ])
                            })
                            .collect::<Option<Vec<_>>>(),
                        false,
                    ),
                }
            } else {
                (None, false)
            };
        let mut axis_support_planes: Vec<((usize, u64, bool), Plane)> = Vec::with_capacity(6);
        let mut adjacent_support_planes =
            (!predominantly_axis_aligned && retained_positions.is_some() && input_planes.is_none())
                .then(|| {
                    AdjacentSupportEdges::new(
                        mesh.positions.len(),
                        mesh.triangles.len().saturating_mul(3).div_ceil(2),
                    )
                });
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
            let supplied_planes = input_planes
                .and_then(|planes| planes.get(mesh_index))
                .and_then(|planes| planes.get(triangle_index))
                .cloned();
            let mut polygon = match (retained_positions.as_ref(), supplied_planes) {
                (Some(positions), Some(planes)) => {
                    make_indexed_triangle_with_deferred_edges_and_input_planes(
                        Arc::clone(positions),
                        [i0, i1, i2],
                        planes,
                        mesh_index as isize,
                        polygon_index,
                    )
                }
                (Some(positions), None) => {
                    let axis_hint = approximate_positions.as_ref().and_then(|points| {
                        let [p0, p1, p2] = [points[i0], points[i1], points[i2]];
                        let axis =
                            (0..3).find(|&axis| p0[axis] == p1[axis] && p0[axis] == p2[axis])?;
                        let orientation = if approximate_positions_are_exact_dyadic {
                            let u = (axis + 1) % 3;
                            let v = (axis + 2) % 3;
                            Real::certified_affine_det2_sign_exact_dyadic_f64(
                                [p0[u], p0[v]],
                                [p1[u], p1[v]],
                                [p2[u], p2[v]],
                            )
                        } else {
                            None
                        };
                        let exact_coordinate =
                            approximate_positions_are_exact_dyadic.then(|| p0[axis].to_bits());
                        Some((axis, orientation, exact_coordinate))
                    });
                    let axis_support_hint =
                        axis_hint.and_then(|(axis, orientation, exact_coordinate)| {
                            let orientation_positive = match orientation {
                                Some(RealSign::Negative) => false,
                                Some(RealSign::Positive) => true,
                                Some(RealSign::Zero) | None => {
                                    return exact_axis_aligned_triangle_support(
                                        p0,
                                        p1,
                                        p2,
                                        axis,
                                        orientation,
                                    );
                                }
                            };
                            let key = (axis, exact_coordinate?, orientation_positive);
                            if let Some((_, support)) = axis_support_planes
                                .iter()
                                .find(|(candidate, _)| *candidate == key)
                            {
                                return Some(support.clone());
                            }
                            let support =
                                exact_axis_aligned_triangle_support(p0, p1, p2, axis, orientation)?;
                            axis_support_planes.push((key, support.clone()));
                            Some(support)
                        });
                    let support_hint = axis_support_hint.or_else(|| {
                        adjacent_support_planes.as_ref().and_then(|adjacent| {
                            adjacent_coplanar_support_hint(
                                mesh.positions,
                                [i0, i1, i2],
                                &polygons,
                                adjacent,
                            )
                        })
                    });
                    make_indexed_triangle_with_deferred_edges(
                        Arc::clone(positions),
                        [i0, i1, i2],
                        support_hint,
                        Arc::clone(
                            deferred_edges
                                .as_ref()
                                .expect("retained positions imply deferred edges"),
                        ),
                        mesh_index as isize,
                        polygon_index,
                    )
                }
                (None, Some(planes)) => make_triangle_with_input_planes(
                    p0,
                    p1,
                    p2,
                    planes,
                    mesh_index as isize,
                    polygon_index,
                ),
                (None, None) => convex_triangle(p0, p1, p2, mesh_index as isize, polygon_index),
            }
            .with_source_triangle_edge_identities(mesh_index, [i0, i1, i2]);
            if !polygon.support.decide_is_valid()? {
                return Err(HypermeshError::DegenerateTriangle {
                    mesh_index,
                    triangle_index,
                });
            }
            if !defer_edges {
                polygon.delta_w = vec![0; meshes.len()];
                polygon.delta_w[mesh_index] = 1;
            }
            let stored_polygon = polygons.len();
            polygons.push(polygon);
            if let Some(adjacent) = adjacent_support_planes.as_mut() {
                for [start, end] in [[i0, i1], [i1, i2], [i2, i0]] {
                    adjacent.insert_if_absent(start, end, stored_polygon);
                }
            }
            polygon_index += 1;
        }
        if !input_is_certified_convex {
            let edge_balance = classify_indexed_edge_balance(mesh)?;
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
        }
    }

    crate::trace_dispatch!("build-polygon-soup", "complete");
    Ok(PolygonSoup {
        polygons,
        bounds,
        num_meshes: meshes.len(),
    })
}

fn adjacent_coplanar_support_hint(
    positions: &[Point3],
    triangle: [usize; 3],
    polygons: &[ConvexPolygon],
    adjacent: &AdjacentSupportEdges,
) -> Option<Plane> {
    let [Some(p0), Some(p1), Some(p2)] = triangle.map(|index| positions.get(index)) else {
        return None;
    };
    let points = [p0, p1, p2];
    for edge in 0..3 {
        let start = triangle[edge];
        let end = triangle[(edge + 1) % 3];
        let Some((stored_start, stored_end, polygon_index)) = adjacent.get(start, end) else {
            continue;
        };
        let Some(candidate) = polygons.get(polygon_index).map(|polygon| &polygon.support) else {
            continue;
        };
        if !matches!(
            classify_point(points[(edge + 2) % 3], candidate),
            Ok(Classification::On)
        ) {
            continue;
        }
        if stored_start == end && stored_end == start {
            return Some(candidate.clone());
        }
        if stored_start == start && stored_end == end {
            return Some(candidate.inverted());
        }
    }
    None
}

fn approximate_triangle_axis(positions: &[Point3], indices: [usize; 3]) -> Option<usize> {
    let points = indices.map(|index| positions.get(index));
    let [Some(p0), Some(p1), Some(p2)] = points else {
        return None;
    };
    let points = [p0, p1, p2].map(|point| {
        Some([
            point.x.to_f64_lossy()?,
            point.y.to_f64_lossy()?,
            point.z.to_f64_lossy()?,
        ])
    });
    let [Some(p0), Some(p1), Some(p2)] = points else {
        return None;
    };
    (0..3).find(|&axis| p0[axis] == p1[axis] && p0[axis] == p2[axis])
}

#[derive(Clone, Copy, Eq, Hash, PartialEq)]
struct ExactRationalPositionBucket([Option<u64>; 3]);

type CertifiedPositionInterval = [[Rational; 2]; 3];

fn certified_position_interval(position: &Point3) -> Option<CertifiedPositionInterval> {
    const BROAD_PHASE_PRECISION: i32 = -20;

    Some([
        position
            .x
            .certified_dyadic_interval(BROAD_PHASE_PRECISION)?,
        position
            .y
            .certified_dyadic_interval(BROAD_PHASE_PRECISION)?,
        position
            .z
            .certified_dyadic_interval(BROAD_PHASE_PRECISION)?,
    ])
}

fn certified_position_intervals_are_disjoint(
    left: Option<&CertifiedPositionInterval>,
    right: Option<&CertifiedPositionInterval>,
) -> bool {
    let (Some(left), Some(right)) = (left, right) else {
        return false;
    };
    left.iter()
        .zip(right)
        .any(|(left, right)| left[1] < right[0] || right[1] < left[0])
}

type CertifiedPositionCells = [[i64; 2]; 3];

fn rational_floor_i64(value: &Rational) -> Option<i64> {
    let truncated = value.trunc();
    let mut integer = i64::try_from(truncated.clone()).ok()?;
    if value.is_negative() && truncated != *value {
        integer = integer.checked_sub(1)?;
    }
    Some(integer)
}

fn certified_position_cells(
    interval: Option<&CertifiedPositionInterval>,
) -> Option<CertifiedPositionCells> {
    const CELLS_PER_UNIT: i64 = 256;
    const MAX_CELLS_PER_POSITION: u64 = 64;

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
        if cell_count > MAX_CELLS_PER_POSITION {
            return None;
        }
    }
    Some(cells)
}

fn position_cells(cells: CertifiedPositionCells) -> impl Iterator<Item = [i64; 3]> {
    (cells[0][0]..=cells[0][1]).flat_map(move |x| {
        (cells[1][0]..=cells[1][1])
            .flat_map(move |y| (cells[2][0]..=cells[2][1]).map(move |z| [x, y, z]))
    })
}

fn exact_rational_position_bucket(position: &Point3) -> Option<ExactRationalPositionBucket> {
    [&position.x, &position.y, &position.z]
        .iter()
        .all(|coordinate| coordinate.exact_rational_ref().is_some())
        .then(|| {
            ExactRationalPositionBucket([
                position.x.to_f64_lossy().map(f64::to_bits),
                position.y.to_f64_lossy().map(f64::to_bits),
                position.z.to_f64_lossy().map(f64::to_bits),
            ])
        })
}

fn canonical_exact_rational_position_indices(positions: &[Point3]) -> Option<Vec<usize>> {
    let mut canonical_positions: Vec<&Point3> = Vec::with_capacity(positions.len());
    let mut buckets = HashMap::<ExactRationalPositionBucket, Vec<usize>>::new();
    let mut canonical_indices = Vec::with_capacity(positions.len());
    for position in positions {
        let key = exact_rational_position_bucket(position)?;
        let candidates = buckets.entry(key).or_default();
        let canonical = candidates
            .iter()
            .copied()
            .find(|&index| {
                let candidate = canonical_positions[index];
                candidate.x.exact_rational_ref() == position.x.exact_rational_ref()
                    && candidate.y.exact_rational_ref() == position.y.exact_rational_ref()
                    && candidate.z.exact_rational_ref() == position.z.exact_rational_ref()
            })
            .unwrap_or_else(|| {
                let index = canonical_positions.len();
                canonical_positions.push(position);
                candidates.push(index);
                index
            });
        canonical_indices.push(canonical);
    }
    Some(canonical_indices)
}

fn canonical_position_indices(positions: &[Point3]) -> HypermeshResult<Vec<usize>> {
    if let Some(indices) = canonical_exact_rational_position_indices(positions) {
        return Ok(indices);
    }

    let mut canonical_positions: Vec<&Point3> = Vec::with_capacity(positions.len());
    let mut canonical_position_intervals: Vec<Option<CertifiedPositionInterval>> =
        Vec::with_capacity(positions.len());
    let mut exact_rational_buckets = HashMap::<ExactRationalPositionBucket, Vec<usize>>::new();
    let mut certified_position_buckets = HashMap::<[i64; 3], Vec<usize>>::new();
    let mut unbucketed_positions = Vec::<usize>::new();
    let mut canonical_indices = Vec::with_capacity(positions.len());
    for position in positions {
        let position_interval = certified_position_interval(position);
        let position_cell_range = certified_position_cells(position_interval.as_ref());
        let exact_coordinates = [
            position.x.exact_rational_ref(),
            position.y.exact_rational_ref(),
            position.z.exact_rational_ref(),
        ];
        let exact_rational_bucket = exact_rational_position_bucket(position);
        let mut canonical = exact_rational_bucket
            .as_ref()
            .and_then(|key| exact_rational_buckets.get(key))
            .into_iter()
            .flatten()
            .copied()
            .find(|&index| {
                let candidate = canonical_positions[index];
                candidate.x.exact_rational_ref() == exact_coordinates[0]
                    && candidate.y.exact_rational_ref() == exact_coordinates[1]
                    && candidate.z.exact_rational_ref() == exact_coordinates[2]
            });
        if canonical.is_none() {
            let mut candidates = BTreeSet::new();
            if let Some(cells) = position_cell_range {
                for cell in position_cells(cells) {
                    candidates.extend(
                        certified_position_buckets
                            .get(&cell)
                            .into_iter()
                            .flatten()
                            .copied(),
                    );
                }
                candidates.extend(unbucketed_positions.iter().copied());
            } else {
                candidates.extend(0..canonical_positions.len());
            }
            for index in candidates {
                if certified_position_intervals_are_disjoint(
                    canonical_position_intervals[index].as_ref(),
                    position_interval.as_ref(),
                ) {
                    continue;
                }
                if crate::predicate::points_equal(canonical_positions[index], position)? {
                    canonical = Some(index);
                    break;
                }
            }
        }
        let is_new = canonical.is_none();
        let canonical = canonical.unwrap_or_else(|| {
            let index = canonical_positions.len();
            canonical_positions.push(position);
            canonical_position_intervals.push(position_interval);
            if let Some(cells) = position_cell_range {
                for cell in position_cells(cells) {
                    certified_position_buckets
                        .entry(cell)
                        .or_default()
                        .push(index);
                }
            } else {
                unbucketed_positions.push(index);
            }
            index
        });
        if is_new && let Some(key) = exact_rational_bucket {
            exact_rational_buckets
                .entry(key)
                .or_default()
                .push(canonical);
        }
        canonical_indices.push(canonical);
    }
    Ok(canonical_indices)
}

fn classify_indexed_edge_balance(mesh: &TriangleMeshRef<'_>) -> HypermeshResult<EdgeBalance> {
    let canonical_indices = canonical_position_indices(mesh.positions)?;
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

pub(crate) fn classify_edge_balance(edges: &[[Point3; 2]]) -> HypermeshResult<EdgeBalance> {
    let mut balance = EdgeBalance::default();
    let mut visited = vec![false; edges.len()];
    for (index, edge) in edges.iter().enumerate() {
        if visited[index] {
            continue;
        }

        let mut forward_uses = 0usize;
        let mut reverse_uses = 0usize;
        for (other_index, other) in edges.iter().enumerate() {
            match edge_match_direction(edge, other)? {
                Some(false) => {
                    visited[other_index] = true;
                    forward_uses += 1;
                }
                Some(true) => {
                    visited[other_index] = true;
                    reverse_uses += 1;
                }
                None => {}
            }
        }

        if forward_uses + reverse_uses == 1 {
            balance.boundary_edges += 1;
        }
        if forward_uses != reverse_uses {
            balance.unbalanced_edges += 1;
        }
    }
    Ok(balance)
}

fn ordered_edge_matches(
    left_start: &Point3,
    left_end: &Point3,
    right_start: &Point3,
    right_end: &Point3,
) -> HypermeshResult<bool> {
    let start = crate::predicate::points_equal(left_start, right_start);
    let end = crate::predicate::points_equal(left_end, right_end);
    match (start, end) {
        (Ok(false), _) | (_, Ok(false)) => Ok(false),
        (Ok(true), Ok(true)) => Ok(true),
        (Err(HypermeshError::UnknownClassification), _)
        | (_, Err(HypermeshError::UnknownClassification)) => {
            Err(HypermeshError::UnknownClassification)
        }
        (Err(error), _) | (_, Err(error)) => Err(error),
    }
}

/// Returns `Some(false)` for the same direction, `Some(true)` for the reverse
/// direction, or `None` for distinct undirected edges.
fn edge_match_direction(left: &[Point3; 2], right: &[Point3; 2]) -> HypermeshResult<Option<bool>> {
    let forward = ordered_edge_matches(&left[0], &left[1], &right[0], &right[1]);
    let reverse = ordered_edge_matches(&left[0], &left[1], &right[1], &right[0]);
    match (forward, reverse) {
        (Ok(true), _) => Ok(Some(false)),
        (_, Ok(true)) => Ok(Some(true)),
        (Ok(false), Ok(false)) => Ok(None),
        (Err(HypermeshError::UnknownClassification), _)
        | (_, Err(HypermeshError::UnknownClassification)) => {
            Err(HypermeshError::UnknownClassification)
        }
        (Err(error), _) | (_, Err(error)) => Err(error),
    }
}

fn bounds_for_positions<'a>(
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
            if compare_real(axis_ref(position, axis), axis_ref(&min, axis))?.is_lt() {
                *crate::geometry::axis_mut(&mut min, axis) = axis_ref(position, axis).clone();
            }
            if compare_real(axis_ref(position, axis), axis_ref(&max, axis))?.is_gt() {
                *crate::geometry::axis_mut(&mut max, axis) = axis_ref(position, axis).clone();
            }
        }
    }

    Ok(Aabb::new(min, max))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::polygon::RetainedVertexCycle;

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
        assert!(mesh.has_unique_nondegenerate_triangles());
        assert!(mesh.is_closed_manifold_geometry());
        assert!(!mesh.is_closed_manifold());

        let mut triangles = mesh.triangles.to_vec();
        triangles.push(triangles[0]);
        let duplicate = TriangleMesh::new(mesh.positions.to_vec(), triangles);
        assert!(!duplicate.has_unique_nondegenerate_triangles());
        assert!(!duplicate.is_closed_manifold_geometry());
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

        assert!(mesh.has_unique_nondegenerate_triangles());
        assert!(mesh.is_closed_manifold_geometry());
    }

    #[test]
    fn certified_position_intervals_only_reject_provably_distinct_points() {
        let left = Point3::new(Real::pi() + Real::e(), Real::zero(), Real::zero());
        let equivalent = Point3::new(Real::e() + Real::pi(), Real::zero(), Real::zero());
        let distinct = Point3::new(Real::pi(), Real::zero(), Real::zero());
        let left_interval = certified_position_interval(&left);
        let equivalent_interval = certified_position_interval(&equivalent);
        let distinct_interval = certified_position_interval(&distinct);

        assert!(!certified_position_intervals_are_disjoint(
            left_interval.as_ref(),
            equivalent_interval.as_ref(),
        ));
        assert!(certified_position_intervals_are_disjoint(
            left_interval.as_ref(),
            distinct_interval.as_ref(),
        ));
    }

    #[test]
    fn geometric_edge_balance_uses_numeric_point_equality() {
        let left = Real::pi() + Real::e();
        let equivalent_left = Real::e() + Real::pi();
        assert_ne!(left, equivalent_left);
        let edges = [
            [
                Point3::new(left, Real::zero(), Real::zero()),
                Point3::new(Real::from(2_u8), Real::one(), Real::zero()),
            ],
            [
                Point3::new(Real::from(2_u8), Real::one(), Real::zero()),
                Point3::new(equivalent_left, Real::zero(), Real::zero()),
            ],
        ];

        assert_eq!(classify_edge_balance(&edges), Ok(EdgeBalance::default()));
    }

    #[test]
    fn reversed_winding_shares_native_positions_and_retains_the_result() {
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
        assert!(Arc::ptr_eq(&reversed.triangles, &retained.triangles));
        assert_eq!(mesh.triangles[0], Triangle::new(0, 1, 2));
        assert_eq!(reversed.triangles[0], Triangle::new(2, 1, 0));
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
            bounds_for_positions(&points).unwrap(),
            Aabb::new(
                Point3::new(Real::from(-3), Real::from(-7), Real::from(1)),
                Point3::new(Real::from(4), Real::from(5), Real::from(9)),
            ),
        );
    }

    #[test]
    fn deferred_certified_triangles_share_one_indexed_position_pool() {
        let positions = vec![
            Point3::new(Real::zero(), Real::zero(), Real::zero()),
            Point3::new(Real::one(), Real::zero(), Real::zero()),
            Point3::new(Real::zero(), Real::one(), Real::zero()),
            Point3::new(Real::zero(), Real::zero(), Real::one()),
        ];
        let mesh = TriangleMesh::new(
            positions.clone(),
            vec![Triangle::new(0, 1, 2), Triangle::new(0, 3, 1)],
        );
        let soup = build_polygon_soup_with_deferred_edges(&[mesh.as_ref()], &[true], None).unwrap();

        let (
            Some(RetainedVertexCycle::IndexedTriangle {
                positions: first,
                indices: first_indices,
            }),
            Some(RetainedVertexCycle::IndexedTriangle {
                positions: second,
                indices: second_indices,
            }),
        ) = (
            &soup.polygons[0].known_vertices,
            &soup.polygons[1].known_vertices,
        )
        else {
            panic!("certified deferred triangles must retain indexed vertices");
        };

        assert!(Arc::ptr_eq(first, second));
        assert!(Arc::ptr_eq(
            &soup.polygons[0].edges,
            &soup.polygons[1].edges
        ));
        assert_eq!(*first_indices, [0, 1, 2]);
        assert_eq!(*second_indices, [0, 3, 1]);
        assert_eq!(soup.polygons[0].vertices().unwrap(), positions[..3]);
    }

    #[test]
    fn deferred_certified_triangles_reuse_adjacent_coplanar_support() {
        let positions = vec![
            Point3::new(Real::from(0), Real::from(0), Real::from(0)),
            Point3::new(Real::from(4), Real::from(0), Real::from(4)),
            Point3::new(Real::from(0), Real::from(4), Real::from(8)),
            Point3::new(Real::from(0), Real::from(-2), Real::from(-4)),
        ];
        let mesh = TriangleMesh::new(
            positions,
            vec![Triangle::new(0, 1, 2), Triangle::new(1, 0, 3)],
        );

        let soup = build_polygon_soup_with_deferred_edges(&[mesh.as_ref()], &[true], None).unwrap();

        assert_eq!(soup.polygons[0].support, soup.polygons[1].support);
    }

    #[test]
    fn adjacent_support_edges_retain_first_directed_use() {
        let mut adjacent = AdjacentSupportEdges::new(5, 3);
        adjacent.insert_if_absent(3, 1, 7);
        adjacent.insert_if_absent(1, 3, 9);
        adjacent.insert_if_absent(1, 4, 11);

        assert_eq!(adjacent.get(1, 3), Some((3, 1, 7)));
        assert_eq!(adjacent.get(3, 1), Some((3, 1, 7)));
        assert_eq!(adjacent.get(4, 1), Some((1, 4, 11)));
        assert_eq!(adjacent.get(0, 2), None);
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
            classify_indexed_edge_balance(&mesh.as_ref()),
            Ok(EdgeBalance::default())
        );
        polygon_soup(&[mesh.as_ref()]).expect("closed coincident-index tetrahedron");
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
            mesh.ray_intersections(
                &Point3::new(Real::zero(), Real::zero(), Real::one()),
                &Vector3::from_xyz(Real::zero(), Real::zero(), -Real::one()),
            ),
            Err(expected.clone()),
        );
        assert_eq!(
            mesh.contains_point(&Point3::new(
                Real::from(Rational::fraction(1, 4).unwrap()),
                Real::from(Rational::fraction(1, 4).unwrap()),
                Real::zero(),
            )),
            Err(expected),
        );
    }
}
