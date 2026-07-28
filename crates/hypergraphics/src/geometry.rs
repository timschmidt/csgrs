//! Hyperreal-owned mesh geometry and explicit render exports.

use std::fmt;
use std::sync::{
    Arc, OnceLock,
    atomic::{AtomicUsize, Ordering},
};

use hyperlattice::Point3;
use hyperlimit::{
    Certainty, OrientedPlane3Evidence, PlaneSide, PredicateOutcome, Sign,
    classify_point_oriented_plane_with_evidence, orient3, oriented_plane3_evidence,
};

use crate::error::{Error, Result};
pub use crate::render::{Color3, Primitive, RenderVertex64};

/// One exact scene vertex.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactVertex {
    /// Hyperreal-owned position.
    pub position: Point3,
    /// Render color.
    pub color: Color3,
}

impl ExactVertex {
    /// Construct one exact vertex.
    pub const fn new(position: Point3, color: Color3) -> Self {
        Self { position, color }
    }
}

/// Orientation of one triangle against a query point.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleOrientation {
    /// The query point is on the negative side of the oriented triangle plane.
    Negative(Certainty),
    /// The query point is exactly coplanar with the oriented triangle plane.
    Coplanar(Certainty),
    /// The query point is on the positive side of the oriented triangle plane.
    Positive(Certainty),
    /// The exact predicate could not decide under the default policy.
    Unknown,
}

impl TriangleOrientation {
    fn from_outcome(outcome: PredicateOutcome<Sign>) -> Self {
        match outcome {
            PredicateOutcome::Decided {
                value: Sign::Negative,
                certainty,
                ..
            } => Self::Negative(certainty),
            PredicateOutcome::Decided {
                value: Sign::Zero,
                certainty,
                ..
            } => Self::Coplanar(certainty),
            PredicateOutcome::Decided {
                value: Sign::Positive,
                certainty,
                ..
            } => Self::Positive(certainty),
            PredicateOutcome::Unknown { .. } => Self::Unknown,
        }
    }

    fn from_plane_outcome(outcome: PredicateOutcome<PlaneSide>) -> Self {
        match outcome {
            PredicateOutcome::Decided {
                value: PlaneSide::Below,
                certainty,
                ..
            } => Self::Negative(certainty),
            PredicateOutcome::Decided {
                value: PlaneSide::On,
                certainty,
                ..
            } => Self::Coplanar(certainty),
            PredicateOutcome::Decided {
                value: PlaneSide::Above,
                certainty,
                ..
            } => Self::Positive(certainty),
            PredicateOutcome::Unknown { .. } => Self::Unknown,
        }
    }
}

const TRIANGLE_ORIENTATION_CACHE_SLOTS: usize = 4;
const NO_TRIANGLE_INDEX: usize = usize::MAX;

#[derive(Debug)]
struct CachedTriangleOrientation {
    triangle_index: usize,
    evidence: OrientedPlane3Evidence,
}

impl CachedTriangleOrientation {
    fn new(triangle_index: usize, a: &Point3, b: &Point3, c: &Point3) -> Self {
        Self {
            triangle_index,
            evidence: oriented_plane3_evidence(a, b, c),
        }
    }
}

#[derive(Debug)]
struct TriangleOrientationCacheSlot {
    last_query: AtomicUsize,
    orientation: OnceLock<Box<CachedTriangleOrientation>>,
}

impl TriangleOrientationCacheSlot {
    fn new() -> Self {
        Self {
            last_query: AtomicUsize::new(NO_TRIANGLE_INDEX),
            orientation: OnceLock::new(),
        }
    }
}

#[derive(Debug)]
struct TriangleOrientationCache {
    slots: [TriangleOrientationCacheSlot; TRIANGLE_ORIENTATION_CACHE_SLOTS],
}

impl TriangleOrientationCache {
    fn new() -> Self {
        Self {
            slots: std::array::from_fn(|_| TriangleOrientationCacheSlot::new()),
        }
    }

    fn classify(
        &self,
        triangle_index: usize,
        triangle: [&Point3; 3],
        point: &Point3,
    ) -> TriangleOrientation {
        let slot = &self.slots[triangle_index % TRIANGLE_ORIENTATION_CACHE_SLOTS];
        if let Some(orientation) = self.classify_cached(triangle_index, point) {
            return orientation;
        }
        if slot.orientation.get().is_some() {
            return TriangleOrientation::from_outcome(orient3(
                triangle[0],
                triangle[1],
                triangle[2],
                point,
            ));
        }
        let previous = slot.last_query.swap(triangle_index, Ordering::Relaxed);
        if previous != triangle_index {
            return TriangleOrientation::from_outcome(orient3(
                triangle[0],
                triangle[1],
                triangle[2],
                point,
            ));
        }
        let cached = slot.orientation.get_or_init(|| {
            Box::new(CachedTriangleOrientation::new(
                triangle_index,
                triangle[0],
                triangle[1],
                triangle[2],
            ))
        });
        TriangleOrientation::from_plane_outcome(classify_point_oriented_plane_with_evidence(
            point,
            &cached.evidence,
        ))
    }

    fn classify_cached(
        &self,
        triangle_index: usize,
        point: &Point3,
    ) -> Option<TriangleOrientation> {
        let cached = self.slots[triangle_index % TRIANGLE_ORIENTATION_CACHE_SLOTS]
            .orientation
            .get()?;
        (cached.triangle_index == triangle_index).then(|| {
            TriangleOrientation::from_plane_outcome(classify_point_oriented_plane_with_evidence(
                point,
                &cached.evidence,
            ))
        })
    }
}

/// Hyperreal-owned colored mesh.
pub struct ExactMesh {
    primitive: Primitive,
    vertices: Vec<ExactVertex>,
    triangle_orientation_cache: Arc<TriangleOrientationCache>,
}

impl Clone for ExactMesh {
    fn clone(&self) -> Self {
        Self {
            primitive: self.primitive,
            vertices: self.vertices.clone(),
            triangle_orientation_cache: Arc::clone(&self.triangle_orientation_cache),
        }
    }
}

impl fmt::Debug for ExactMesh {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("ExactMesh")
            .field("primitive", &self.primitive)
            .field("vertices", &self.vertices)
            .finish()
    }
}

impl PartialEq for ExactMesh {
    fn eq(&self, other: &Self) -> bool {
        self.primitive == other.primitive && self.vertices == other.vertices
    }
}

impl ExactMesh {
    fn invalidate_triangle_orientation_cache(&mut self) {
        if let Some(cache) = Arc::get_mut(&mut self.triangle_orientation_cache) {
            *cache = TriangleOrientationCache::new();
        } else {
            self.triangle_orientation_cache = Arc::new(TriangleOrientationCache::new());
        }
    }

    /// Construct a mesh from exact vertices.
    pub fn new(primitive: Primitive, vertices: Vec<ExactVertex>) -> Self {
        Self {
            primitive,
            vertices,
            triangle_orientation_cache: Arc::new(TriangleOrientationCache::new()),
        }
    }

    /// Construct an empty mesh.
    pub fn empty(primitive: Primitive) -> Self {
        Self::new(primitive, Vec::new())
    }

    /// Return the render primitive.
    pub const fn primitive(&self) -> Primitive {
        self.primitive
    }

    /// Borrow exact vertices.
    pub fn vertices(&self) -> &[ExactVertex] {
        &self.vertices
    }

    /// Mutably borrow exact vertices.
    pub fn vertices_mut(&mut self) -> &mut Vec<ExactVertex> {
        self.invalidate_triangle_orientation_cache();
        &mut self.vertices
    }

    /// Append one exact vertex.
    pub fn push(&mut self, vertex: ExactVertex) {
        if Arc::strong_count(&self.triangle_orientation_cache) > 1 {
            self.triangle_orientation_cache = Arc::new(TriangleOrientationCache::new());
        }
        self.vertices.push(vertex);
    }

    /// Return the number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Return the number of full triangles in the mesh.
    pub fn triangle_count(&self) -> usize {
        if self.primitive == Primitive::Triangles {
            self.vertices.len() / 3
        } else {
            0
        }
    }

    /// Lossily export exact vertices to finite f64 render vertices.
    pub fn to_render_vertices64(&self) -> Result<Vec<RenderVertex64>> {
        self.vertices
            .iter()
            .map(|vertex| {
                let position =
                    vertex
                        .position
                        .to_f64_array_lossy()
                        .ok_or(Error::NonFiniteProjection {
                            value: "vertex position",
                        })?;
                Ok(RenderVertex64 {
                    position,
                    color: vertex.color.to_array(),
                })
            })
            .collect()
    }

    /// Lossily export a flat `xyz rgb` f64 buffer.
    pub fn to_xyz_rgb_f64(&self) -> Result<Vec<f64>> {
        let mut out = Vec::with_capacity(self.vertices.len() * 6);
        for vertex in &self.vertices {
            let position =
                vertex
                    .position
                    .to_f64_array_lossy()
                    .ok_or(Error::NonFiniteProjection {
                        value: "vertex position",
                    })?;
            out.extend_from_slice(&position);
            out.extend(vertex.color.to_array().map(f64::from));
        }
        Ok(out)
    }

    /// Run an exact orientation predicate for one triangle against `point`.
    pub fn triangle_orientation_against(
        &self,
        triangle_index: usize,
        point: &Point3,
    ) -> Result<TriangleOrientation> {
        if let Some(orientation) = self
            .triangle_orientation_cache
            .classify_cached(triangle_index, point)
        {
            return Ok(orientation);
        }
        if self.primitive != Primitive::Triangles {
            return Err(Error::RequiresTriangles);
        }
        let triangle_count = self.triangle_count();
        if triangle_index >= triangle_count {
            return Err(Error::TriangleIndexOutOfBounds {
                index: triangle_index,
                triangle_count,
            });
        }
        let base = triangle_index * 3;
        let a = &self.vertices[base].position;
        let b = &self.vertices[base + 1].position;
        let c = &self.vertices[base + 2].position;
        let triangle = [a, b, c];
        Ok(self
            .triangle_orientation_cache
            .classify(triangle_index, triangle, point))
    }
}

#[cfg(test)]
mod tests {
    use hyperlattice::Point3;
    use hyperreal::Real;

    use super::*;

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn exports_vertices_to_f64_at_boundary() {
        let red = Color3::new(1.0, 0.0, 0.0).unwrap();
        let mesh = ExactMesh::new(Primitive::Lines, vec![ExactVertex::new(p(1, 2, 3), red)]);

        assert_eq!(
            mesh.to_xyz_rgb_f64().unwrap(),
            vec![1.0, 2.0, 3.0, 1.0, 0.0, 0.0]
        );
    }

    #[test]
    fn triangle_orientation_uses_hyperlimit() {
        let color = Color3::new(0.0, 1.0, 0.0).unwrap();
        let mesh = ExactMesh::new(
            Primitive::Triangles,
            vec![
                ExactVertex::new(p(0, 0, 0), color),
                ExactVertex::new(p(1, 0, 0), color),
                ExactVertex::new(p(0, 1, 0), color),
            ],
        );

        assert!(matches!(
            mesh.triangle_orientation_against(0, &p(0, 0, 1)).unwrap(),
            TriangleOrientation::Positive(_) | TriangleOrientation::Negative(_)
        ));

        for query in [p(0, 0, 1), p(0, 0, -1), p(0, 0, 0)] {
            assert_eq!(
                mesh.triangle_orientation_against(0, &query).unwrap(),
                TriangleOrientation::from_outcome(orient3(
                    &mesh.vertices()[0].position,
                    &mesh.vertices()[1].position,
                    &mesh.vertices()[2].position,
                    &query,
                ))
            );
        }
    }

    #[test]
    fn mutable_vertex_access_invalidates_orientation_reuse() {
        let color = Color3::new(0.0, 1.0, 0.0).unwrap();
        let mut mesh = ExactMesh::new(
            Primitive::Triangles,
            vec![
                ExactVertex::new(p(0, 0, 0), color),
                ExactVertex::new(p(1, 0, 0), color),
                ExactVertex::new(p(0, 1, 0), color),
            ],
        );
        let query = p(0, 0, 1);

        let before = mesh.triangle_orientation_against(0, &query).unwrap();
        assert_eq!(
            mesh.triangle_orientation_against(0, &query).unwrap(),
            before
        );
        let unchanged_clone = mesh.clone();
        mesh.vertices_mut()[2].position = p(0, -1, 0);
        let after = mesh.triangle_orientation_against(0, &query).unwrap();

        assert_ne!(after, before);
        assert_eq!(
            unchanged_clone
                .triangle_orientation_against(0, &query)
                .unwrap(),
            before
        );
    }
}
