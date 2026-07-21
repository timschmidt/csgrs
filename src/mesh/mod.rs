//! `Mesh` struct and implementations of the `CSGOps` trait for `Mesh`

use crate::errors::ValidationError;
use crate::mesh::plane::Plane;
use crate::triangulated::IndexedTriangleMesh3D;

use crate::vertex::{
    Vertex, cache_position_f64, cache_position_f64_range, cache_shared_position_f64_range,
    fresh_position_id, reserve_position_ids,
};

#[cfg(feature = "sketch")]
use crate::sketch::Profile;

use crate::csg::{CSG, finite_axis_aligned_reflection, finite_reflection};
#[cfg(feature = "sketch")]
use hypercurve::{Classification, FiniteProjectionOptions};
use hyperlattice::{Aabb, Matrix4, Point3, Real, Vector3};
use hyperphysics::{
    ClosedTriangleMesh3 as HyperClosedTriangleMesh3,
    MassPropertyReport3 as HyperMassPropertyReport3, Triangle3 as HyperTriangle3,
    Vector3 as HyperVector3,
};
use hyperreal::RealSign;
use std::{
    cell::RefCell,
    cmp::PartialEq,
    collections::HashMap,
    fmt::Debug,
    num::NonZeroU32,
    ops::{Deref, DerefMut},
    sync::{Arc, OnceLock},
};

pub mod convex_hull;
#[cfg(feature = "sketch")]
pub mod flatten_slice;

pub mod connectivity;
pub mod hypermesh;
pub mod manifold;
#[cfg(feature = "metaballs")]
pub mod metaballs;
pub mod plane;
pub(crate) mod polygon;
pub use polygon::Polygon as Triangle;
pub(crate) use polygon::Polygon;
use polygon::{
    CertifiedF64Bounds, LazySubdivisionVertexPool, PreparedExactXAxisTriangle,
    PreparedTriangleQuery, finite_normalized_exact_rational, fresh_plane_id,
    reserve_plane_ids,
};

#[derive(Clone, Debug)]
struct CachedRigidTransform {
    source_geometry_identity: u64,
    matrix: Matrix4,
    polygons: Vec<Polygon<()>>,
}

#[derive(Clone, Debug)]
struct CachedGeneralTransform {
    source_geometry_identity: u64,
    matrix: Matrix4,
    polygons: Vec<Polygon<()>>,
    bounding_box: Option<Aabb>,
}

#[derive(Clone, Debug)]
struct CachedRotation {
    source_geometry_identity: u64,
    degrees: [Real; 3],
    polygons: Vec<Polygon<()>>,
}

#[derive(Clone, Debug)]
struct CachedTranslation {
    source_geometry_identity: u64,
    vector: Vector3,
    polygons: Vec<Polygon<()>>,
    bounding_box: Option<Aabb>,
    axis_aligned_box: bool,
    centering_offset: Option<Vector3>,
    cuboid_vertex_pool: Option<Arc<LazySubdivisionVertexPool>>,
}

#[derive(Clone, Debug)]
struct CachedScale {
    source_geometry_identity: u64,
    scales: [Real; 3],
    polygons: Vec<Polygon<()>>,
}

#[derive(Clone, Debug)]
struct CachedInverse {
    source_geometry_identity: u64,
    polygons: Vec<Polygon<()>>,
}

#[derive(Clone, Debug)]
struct PendingTransform {
    source_geometry_identity: u64,
    matrix: Matrix4,
}

#[derive(Clone, Debug)]
struct PendingRotation {
    source_geometry_identity: u64,
    degrees: [Real; 3],
}

#[derive(Clone, Debug)]
struct PendingTranslation {
    source_geometry_identity: u64,
    vector: Vector3,
}

#[derive(Clone, Debug)]
struct PendingScale {
    source_geometry_identity: u64,
    scales: [Real; 3],
}

#[derive(Clone, Debug)]
struct CachedSubdivision {
    source_storage_identity: u64,
    levels: NonZeroU32,
    polygons: Vec<Polygon<()>>,
    metadata_sources: Vec<usize>,
}

#[derive(Clone, Debug)]
struct CachedMirror {
    source_storage_identity: u64,
    plane: Plane,
    polygons: Vec<Polygon<()>>,
}

#[derive(Clone, Debug, PartialEq)]
struct ArcDistributionKey {
    source_storage_identity: u64,
    count: usize,
    radius: Real,
    start_angle_deg: Real,
    end_angle_deg: Real,
}

#[derive(Clone, Debug)]
struct CachedArcDistribution {
    key: ArcDistributionKey,
    polygons: Vec<Polygon<()>>,
}

#[derive(Clone, Debug, PartialEq)]
struct LinearDistributionKey {
    source_storage_identity: u64,
    count: usize,
    direction: Vector3,
    spacing: Real,
}

#[derive(Clone, Debug)]
struct CachedLinearDistribution {
    key: LinearDistributionKey,
    polygons: Vec<Polygon<()>>,
}

#[derive(Clone, Debug, PartialEq)]
struct GridDistributionKey {
    source_storage_identity: u64,
    rows: usize,
    cols: usize,
    dx: Real,
    dy: Real,
}

#[derive(Clone, Debug)]
struct CachedGridDistribution {
    key: GridDistributionKey,
    polygons: Vec<Polygon<()>>,
}

thread_local! {
    static LAST_RIGID_TRANSFORM: RefCell<Option<CachedRigidTransform>> = const { RefCell::new(None) };
    static LAST_GENERAL_TRANSFORM: RefCell<Vec<CachedGeneralTransform>> = const { RefCell::new(Vec::new()) };
    static LAST_ROTATION: RefCell<Option<CachedRotation>> = const { RefCell::new(None) };
    static LAST_TRANSLATION: RefCell<Vec<CachedTranslation>> = const { RefCell::new(Vec::new()) };
    static LAST_SCALE: RefCell<Option<CachedScale>> = const { RefCell::new(None) };
    static LAST_INVERSE: RefCell<Option<CachedInverse>> = const { RefCell::new(None) };
    static PENDING_RIGID_TRANSFORM: RefCell<Option<PendingTransform>> = const { RefCell::new(None) };
    static PENDING_GENERAL_TRANSFORM: RefCell<Vec<PendingTransform>> = const { RefCell::new(Vec::new()) };
    static PENDING_ROTATION: RefCell<Option<PendingRotation>> = const { RefCell::new(None) };
    static PENDING_TRANSLATION: RefCell<Vec<PendingTranslation>> = const { RefCell::new(Vec::new()) };
    static PENDING_SCALE: RefCell<Option<PendingScale>> = const { RefCell::new(None) };
    static PENDING_INVERSE: RefCell<Option<u64>> = const { RefCell::new(None) };
    static LAST_SUBDIVISION: RefCell<Option<CachedSubdivision>> = const { RefCell::new(None) };
    static PENDING_SUBDIVISION: RefCell<Option<(u64, NonZeroU32)>> = const { RefCell::new(None) };
    static LAST_MIRROR: RefCell<Option<CachedMirror>> = const { RefCell::new(None) };
    static PENDING_MIRROR: RefCell<Option<(u64, Plane)>> = const { RefCell::new(None) };
    static LAST_DIHEDRAL_ANGLE: RefCell<Option<(u64, u64, Real)>> = const { RefCell::new(None) };
    static LAST_ARC_DISTRIBUTION: RefCell<Option<CachedArcDistribution>> = const { RefCell::new(None) };
    static PENDING_ARC_DISTRIBUTION: RefCell<Option<ArcDistributionKey>> = const { RefCell::new(None) };
    static LAST_LINEAR_DISTRIBUTION: RefCell<Option<CachedLinearDistribution>> = const { RefCell::new(None) };
    static PENDING_LINEAR_DISTRIBUTION: RefCell<Option<LinearDistributionKey>> = const { RefCell::new(None) };
    static LAST_GRID_DISTRIBUTION: RefCell<Option<CachedGridDistribution>> = const { RefCell::new(None) };
    static PENDING_GRID_DISTRIBUTION: RefCell<Option<GridDistributionKey>> = const { RefCell::new(None) };
}

const TRANSFORM_CACHE_CAPACITY: usize = 64;
#[cfg(feature = "sdf")]
pub mod sdf;
pub mod shapes;
pub mod smoothing;
#[cfg(feature = "sdf")]
pub mod tpms;
pub mod triangulated;

/// Stored as `(position: [Real; 3], normal: [Real; 3])`.
pub type GraphicsMeshVertex = ::hypermesh::ExactGpuVertex;

#[derive(Debug, Clone)]
pub struct SharedVec<T>(Arc<Vec<T>>);

impl<T> From<Vec<T>> for SharedVec<T> {
    fn from(values: Vec<T>) -> Self {
        Self(Arc::new(values))
    }
}

impl<T> Deref for SharedVec<T> {
    type Target = Vec<T>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T: Clone> DerefMut for SharedVec<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        Arc::make_mut(&mut self.0)
    }
}

impl<T: PartialEq> PartialEq<Vec<T>> for SharedVec<T> {
    fn eq(&self, other: &Vec<T>) -> bool {
        self.0.as_ref() == other
    }
}

/// Mesh data laid out for renderers that want vertex buffers plus u32 indices.
#[derive(Debug, Clone)]
pub struct GraphicsMesh {
    pub vertices: SharedVec<GraphicsMeshVertex>,
    pub indices: SharedVec<u32>,
}

static NEXT_MESH_STORAGE_ID: std::sync::atomic::AtomicU64 =
    std::sync::atomic::AtomicU64::new(1);

fn fresh_mesh_storage_id() -> u64 {
    NEXT_MESH_STORAGE_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
}

const CUBOID_CORNER_POSITION_SLOTS: [usize; 24] = [
    0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 3, 7, 6, 2, 0, 4, 7, 3, 1, 2, 6, 5,
];
const CUBOID_POINT_COORDINATE_SLOTS: [[usize; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [1, 1, 0],
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [1, 1, 1],
    [0, 1, 1],
];
const CUBOID_POSITION_REPRESENTATIVES: [[usize; 2]; 8] = [
    [0, 0],
    [0, 3],
    [0, 2],
    [0, 1],
    [1, 0],
    [1, 1],
    [1, 2],
    [1, 3],
];
const OCTAHEDRON_FACES: [[usize; 3]; 8] = [
    [0, 2, 4],
    [2, 1, 4],
    [1, 3, 4],
    [3, 0, 4],
    [5, 2, 0],
    [5, 1, 2],
    [5, 3, 1],
    [5, 0, 3],
];

#[derive(Clone, Debug)]
struct TransformLayout {
    corner_position_slots: Arc<Vec<usize>>,
    corner_coordinate_slots: [Option<Vec<usize>>; 3],
    polygon_plane_slots: Option<Vec<usize>>,
    position_representatives: Arc<Vec<[usize; 2]>>,
    coordinate_counts: [usize; 3],
    plane_count: usize,
    normals_match_positions: bool,
    indexed_triangle_pool: Option<Arc<LazySubdivisionVertexPool>>,
    indexed_polygon_corner_counts: Option<Vec<usize>>,
    position_f64: Option<Arc<Vec<[f64; 3]>>>,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct IndexedPositionKey([Option<u64>; 3]);

impl IndexedPositionKey {
    fn new(position: &Point3) -> Self {
        fn coordinate_key(value: &Real) -> Option<u64> {
            value.to_f64_lossy().map(|value| {
                if value == 0.0 {
                    0.0_f64.to_bits()
                } else {
                    value.to_bits()
                }
            })
        }

        Self([
            coordinate_key(&position.x),
            coordinate_key(&position.y),
            coordinate_key(&position.z),
        ])
    }
}

impl TransformLayout {
    fn from_polygons<M: Clone>(polygons: &[Polygon<M>]) -> Self {
        let corner_count = polygons.iter().map(|polygon| polygon.vertices.len()).sum();
        let mut position_slots = hashbrown::HashMap::with_capacity(corner_count);
        let mut coordinate_slots: [hashbrown::HashMap<u64, usize>; 3] =
            std::array::from_fn(|_| hashbrown::HashMap::with_capacity(corner_count));
        let mut plane_slots = hashbrown::HashMap::with_capacity(polygons.len());
        let mut corner_position_slots = Vec::with_capacity(corner_count);
        let mut corner_coordinate_slots: [Vec<usize>; 3] =
            std::array::from_fn(|_| Vec::with_capacity(corner_count));
        let mut polygon_plane_slots = Vec::with_capacity(polygons.len());
        let mut position_representatives = Vec::new();

        for (polygon_index, polygon) in polygons.iter().enumerate() {
            let next_plane_slot = plane_slots.len();
            polygon_plane_slots
                .push(*plane_slots.entry(polygon.plane_id).or_insert(next_plane_slot));
            for (vertex_index, vertex) in polygon.vertices.iter().enumerate() {
                let next_position_slot = position_slots.len();
                let position_slot =
                    *position_slots.entry(vertex.position_id).or_insert_with(|| {
                        position_representatives.push([polygon_index, vertex_index]);
                        next_position_slot
                    });
                corner_position_slots.push(position_slot);
                for axis in 0..3 {
                    let next_coordinate_slot = coordinate_slots[axis].len();
                    corner_coordinate_slots[axis].push(
                        *coordinate_slots[axis]
                            .entry(vertex.coordinate_ids[axis])
                            .or_insert(next_coordinate_slot),
                    );
                }
            }
        }

        let coordinate_counts = std::array::from_fn(|axis| coordinate_slots[axis].len());
        let corner_coordinate_slots = std::array::from_fn(|axis| {
            (corner_coordinate_slots[axis] != corner_position_slots)
                .then(|| std::mem::take(&mut corner_coordinate_slots[axis]))
        });
        let polygon_plane_slots = (polygon_plane_slots.iter().copied().ne(0..polygons.len()))
            .then_some(polygon_plane_slots);
        Self {
            corner_position_slots: Arc::new(corner_position_slots),
            corner_coordinate_slots,
            polygon_plane_slots,
            position_representatives: Arc::new(position_representatives),
            coordinate_counts,
            plane_count: plane_slots.len(),
            normals_match_positions: false,
            indexed_triangle_pool: None,
            indexed_polygon_corner_counts: None,
            position_f64: None,
        }
    }

    fn shared_position_identity(
        corner_position_slots: Vec<usize>,
        position_count: usize,
        polygon_count: usize,
        position_f64: Option<Arc<Vec<[f64; 3]>>>,
        indexed_triangle_pool: Option<Arc<LazySubdivisionVertexPool>>,
        indexed_polygon_corner_counts: Option<Vec<usize>>,
        normals_match_positions: bool,
    ) -> Self {
        debug_assert!(
            position_f64
                .as_ref()
                .is_none_or(|positions| positions.len() == position_count)
        );
        let mut position_representatives = vec![[usize::MAX; 2]; position_count];
        if let Some(corner_counts) = &indexed_polygon_corner_counts {
            debug_assert_eq!(corner_counts.len(), polygon_count);
            debug_assert_eq!(
                corner_counts.iter().sum::<usize>(),
                corner_position_slots.len()
            );
            let mut corner_index = 0;
            for (polygon_index, &corner_count) in corner_counts.iter().enumerate() {
                for vertex_index in 0..corner_count {
                    let slot = corner_position_slots[corner_index];
                    if position_representatives[slot][0] == usize::MAX {
                        position_representatives[slot] = [polygon_index, vertex_index];
                    }
                    corner_index += 1;
                }
            }
        } else {
            for (corner_index, &slot) in corner_position_slots.iter().enumerate() {
                if position_representatives[slot][0] == usize::MAX {
                    position_representatives[slot] = [corner_index / 3, corner_index % 3];
                }
            }
        }
        debug_assert!(
            position_representatives
                .iter()
                .all(|representative| representative[0] != usize::MAX)
        );
        Self::shared_position_identity_with_representatives(
            Arc::new(corner_position_slots),
            Arc::new(position_representatives),
            position_count,
            polygon_count,
            position_f64,
            indexed_triangle_pool,
            indexed_polygon_corner_counts,
            normals_match_positions,
        )
    }

    fn shared_position_identity_with_representatives(
        corner_position_slots: Arc<Vec<usize>>,
        position_representatives: Arc<Vec<[usize; 2]>>,
        position_count: usize,
        polygon_count: usize,
        position_f64: Option<Arc<Vec<[f64; 3]>>>,
        indexed_triangle_pool: Option<Arc<LazySubdivisionVertexPool>>,
        indexed_polygon_corner_counts: Option<Vec<usize>>,
        normals_match_positions: bool,
    ) -> Self {
        debug_assert_eq!(position_representatives.len(), position_count);
        debug_assert!(
            corner_position_slots
                .iter()
                .all(|slot| *slot < position_count)
        );
        Self {
            corner_position_slots,
            corner_coordinate_slots: std::array::from_fn(|_| None),
            polygon_plane_slots: None,
            position_representatives,
            coordinate_counts: [position_count; 3],
            plane_count: polygon_count,
            normals_match_positions,
            indexed_triangle_pool,
            indexed_polygon_corner_counts,
            position_f64,
        }
    }

    fn position_representative<'a, M: Clone>(
        &'a self,
        polygons: &'a MeshPolygons<M>,
        position_slot: usize,
    ) -> &'a Vertex {
        if let Some(pool) = &self.indexed_triangle_pool {
            return pool.vertex(position_slot);
        }
        if let Some(pool) = polygons.cuboid_vertex_pool() {
            return pool.cuboid_position_vertex(position_slot);
        }
        let [polygon_index, vertex_index] = self.position_representatives[position_slot];
        &polygons[polygon_index].vertices[vertex_index]
    }

    fn coordinate_slot(&self, axis: usize, corner_index: usize) -> usize {
        self.corner_coordinate_slots[axis]
            .as_ref()
            .map_or(self.corner_position_slots[corner_index], |slots| {
                slots[corner_index]
            })
    }

    fn plane_slot(&self, polygon_index: usize) -> usize {
        self.polygon_plane_slots
            .as_ref()
            .map_or(polygon_index, |slots| slots[polygon_index])
    }
}

/// Copy-on-write polygon storage for a [`Mesh`].
///
/// This dereferences to `Vec<Polygon<M>>`, so existing read and mutation
/// operations keep their usual collection syntax. Cloning an unchanged mesh is
/// constant-time; the polygon vector detaches on the first mutable access.
#[derive(Clone, Debug)]
struct MeshPolygonStorage<M: Clone> {
    identity: u64,
    // Preserved only across metadata mapping and refreshed before mutable access.
    geometry_lineage: u64,
    polygons: Vec<Polygon<M>>,
    topology: OnceLock<(usize, usize, bool)>,
    connectivity: OnceLock<connectivity::Connectivity>,
    connectivity_counts: OnceLock<(usize, usize)>,
    disjoint_partner: OnceLock<u64>,
    cuboid_vertex_pool: OnceLock<Arc<LazySubdivisionVertexPool>>,
    transform_layout: OnceLock<Arc<TransformLayout>>,
    manifold: OnceLock<bool>,
    nondegenerate_triangles: OnceLock<bool>,
    convex_pwn: OnceLock<()>,
    axis_aligned_box: OnceLock<Aabb>,
    centering_offset: OnceLock<Vector3>,
    graphics_mesh: OnceLock<GraphicsMesh>,
    #[cfg(feature = "stl-io")]
    stl_binary: OnceLock<Arc<Vec<u8>>>,
    renormalized: OnceLock<MeshPolygons<M>>,
    materialized_finite: OnceLock<MeshPolygons<M>>,
}

impl<M: Clone> MeshPolygonStorage<M> {
    fn new(polygons: Vec<Polygon<M>>) -> Self {
        let topology = polygon_topology(&polygons);
        Self::new_with_topology(polygons, topology)
    }

    fn new_with_topology(polygons: Vec<Polygon<M>>, topology: (usize, usize, bool)) -> Self {
        let identity = fresh_mesh_storage_id();
        Self::new_with_topology_and_geometry_lineage(polygons, topology, identity, identity)
    }

    fn new_with_topology_and_geometry_lineage(
        polygons: Vec<Polygon<M>>,
        topology: (usize, usize, bool),
        identity: u64,
        geometry_lineage: u64,
    ) -> Self {
        Self {
            identity,
            geometry_lineage,
            polygons,
            topology: OnceLock::from(topology),
            connectivity: OnceLock::new(),
            connectivity_counts: OnceLock::new(),
            disjoint_partner: OnceLock::new(),
            cuboid_vertex_pool: OnceLock::new(),
            transform_layout: OnceLock::new(),
            manifold: OnceLock::new(),
            nondegenerate_triangles: OnceLock::new(),
            convex_pwn: OnceLock::new(),
            axis_aligned_box: OnceLock::new(),
            centering_offset: OnceLock::new(),
            graphics_mesh: OnceLock::new(),
            #[cfg(feature = "stl-io")]
            stl_binary: OnceLock::new(),
            renormalized: OnceLock::new(),
            materialized_finite: OnceLock::new(),
        }
    }

    #[cfg(feature = "stl-io")]
    fn reset_stl_binary(&mut self) {
        self.stl_binary = OnceLock::new();
    }

    #[cfg(not(feature = "stl-io"))]
    fn reset_stl_binary(&mut self) {}
}

#[derive(Clone, Debug)]
pub(crate) struct MeshPolygons<M: Clone>(Arc<MeshPolygonStorage<M>>);

impl<M: Clone> MeshPolygons<M> {
    pub fn new(polygons: Vec<Polygon<M>>) -> Self {
        Self(Arc::new(MeshPolygonStorage::new(polygons)))
    }

    pub(crate) fn new_with_topology(
        polygons: Vec<Polygon<M>>,
        topology: (usize, usize, bool),
    ) -> Self {
        Self(Arc::new(MeshPolygonStorage::new_with_topology(
            polygons, topology,
        )))
    }

    fn new_with_geometry_lineage(
        polygons: Vec<Polygon<M>>,
        topology: (usize, usize, bool),
        geometry_lineage: u64,
    ) -> Self {
        Self(Arc::new(
            MeshPolygonStorage::new_with_topology_and_geometry_lineage(
                polygons,
                topology,
                fresh_mesh_storage_id(),
                geometry_lineage,
            ),
        ))
    }

    pub fn into_vec(self) -> Vec<Polygon<M>> {
        Arc::try_unwrap(self.0)
            .map(|storage| storage.polygons)
            .unwrap_or_else(|storage| storage.polygons.clone())
    }

    fn topology(&self) -> (usize, usize, bool) {
        *self
            .0
            .topology
            .get_or_init(|| polygon_topology(&self.0.polygons))
    }

    pub(crate) fn connectivity(&self) -> Option<&connectivity::Connectivity> {
        self.0.connectivity.get()
    }

    pub(crate) fn retain_connectivity(&self, connectivity: connectivity::Connectivity) {
        let _ = self.0.connectivity.set(connectivity);
    }

    pub(crate) fn connectivity_counts(&self) -> Option<(usize, usize)> {
        self.0.connectivity_counts.get().copied()
    }

    pub(crate) fn retain_connectivity_counts(&self, counts: (usize, usize)) {
        let _ = self.0.connectivity_counts.set(counts);
    }

    pub(crate) fn storage_identity(&self) -> u64 {
        self.0.identity
    }

    pub(crate) fn geometry_lineage_identity(&self) -> u64 {
        self.0.geometry_lineage
    }

    #[inline]
    fn shares_storage_with(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }

    pub(crate) fn is_retained_disjoint_with(&self, other_storage_identity: u64) -> bool {
        self.0.disjoint_partner.get() == Some(&other_storage_identity)
    }

    pub(crate) fn retain_disjoint_partner(&self, other_storage_identity: u64) {
        let _ = self.0.disjoint_partner.set(other_storage_identity);
    }

    fn transform_layout(&self) -> &Arc<TransformLayout> {
        self.0.transform_layout.get_or_init(|| {
            if self.0.cuboid_vertex_pool.get().is_some() {
                shapes::cuboid_transform_layout()
            } else {
                Arc::new(TransformLayout::from_polygons(&self.0.polygons))
            }
        })
    }

    fn retained_transform_layout(&self) -> Option<&Arc<TransformLayout>> {
        if self.0.cuboid_vertex_pool.get().is_some() {
            Some(self.transform_layout())
        } else {
            self.0.transform_layout.get()
        }
    }

    fn retain_transform_layout(&self, layout: Arc<TransformLayout>) {
        let _ = self.0.transform_layout.set(layout);
    }

    pub(crate) fn cuboid_vertex_pool(&self) -> Option<&Arc<LazySubdivisionVertexPool>> {
        self.0.cuboid_vertex_pool.get()
    }

    pub(crate) fn retain_cuboid_vertex_pool(&self, pool: Arc<LazySubdivisionVertexPool>) {
        let _ = self.0.cuboid_vertex_pool.set(pool);
    }

    pub(crate) fn manifold_fact(&self) -> Option<bool> {
        self.0.manifold.get().copied()
    }

    pub(crate) fn retain_manifold_fact(&self, is_manifold: bool) {
        let _ = self.0.manifold.set(is_manifold);
    }

    pub(crate) fn nondegenerate_triangles_fact(&self) -> Option<bool> {
        self.0.nondegenerate_triangles.get().copied()
    }

    pub(crate) fn retain_nondegenerate_triangles_fact(&self, nondegenerate: bool) {
        let _ = self.0.nondegenerate_triangles.set(nondegenerate);
    }

    pub(crate) fn has_convex_pwn_fact(&self) -> bool {
        self.0.convex_pwn.get().is_some()
    }

    pub(crate) fn retain_convex_pwn_fact(&self) {
        let _ = self.0.convex_pwn.set(());
    }

    pub(crate) fn axis_aligned_box_fact(&self) -> Option<&Aabb> {
        self.0.axis_aligned_box.get()
    }

    pub(crate) fn retain_axis_aligned_box_fact(&self, bounds: Aabb) {
        let _ = self.0.axis_aligned_box.set(bounds);
    }

    fn centering_offset_fact(&self) -> Option<&Vector3> {
        self.0.centering_offset.get()
    }

    fn retain_centering_offset(&self, offset: Vector3) {
        let _ = self.0.centering_offset.set(offset);
    }

    fn graphics_mesh(&self) -> Option<&GraphicsMesh> {
        self.0.graphics_mesh.get()
    }

    pub(crate) fn retain_graphics_mesh(&self, graphics: GraphicsMesh) {
        let _ = self.0.graphics_mesh.set(graphics);
    }

    #[cfg(feature = "stl-io")]
    pub(crate) fn stl_binary(&self) -> Option<&Arc<Vec<u8>>> {
        self.0.stl_binary.get()
    }

    #[cfg(feature = "stl-io")]
    pub(crate) fn retain_stl_binary(&self, bytes: Arc<Vec<u8>>) {
        let _ = self.0.stl_binary.set(bytes);
    }

    fn renormalized(&self) -> Option<&Self> {
        self.0.renormalized.get()
    }

    pub(crate) fn retain_renormalized(&self, polygons: Self) {
        let _ = self.0.renormalized.set(polygons);
    }

    fn materialized_finite(&self) -> Option<&Self> {
        self.0.materialized_finite.get()
    }

    pub(crate) fn retain_materialized_finite(&self, polygons: Self) {
        let _ = self.0.materialized_finite.set(polygons);
    }
}

impl<M: Clone> Deref for MeshPolygons<M> {
    type Target = Vec<Polygon<M>>;

    fn deref(&self) -> &Self::Target {
        &self.0.polygons
    }
}

impl<M: Clone> DerefMut for MeshPolygons<M> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        let storage = Arc::make_mut(&mut self.0);
        storage.identity = fresh_mesh_storage_id();
        storage.geometry_lineage = storage.identity;
        storage.topology = OnceLock::new();
        storage.connectivity = OnceLock::new();
        storage.connectivity_counts = OnceLock::new();
        storage.disjoint_partner = OnceLock::new();
        storage.cuboid_vertex_pool = OnceLock::new();
        storage.transform_layout = OnceLock::new();
        storage.manifold = OnceLock::new();
        storage.convex_pwn = OnceLock::new();
        storage.axis_aligned_box = OnceLock::new();
        storage.centering_offset = OnceLock::new();
        storage.graphics_mesh = OnceLock::new();
        storage.reset_stl_binary();
        storage.renormalized = OnceLock::new();
        storage.materialized_finite = OnceLock::new();
        &mut storage.polygons
    }
}

impl<M: Clone + PartialEq> PartialEq for MeshPolygons<M> {
    fn eq(&self, other: &Self) -> bool {
        self.0.polygons == other.0.polygons
    }
}

impl<M: Clone> Default for MeshPolygons<M> {
    fn default() -> Self {
        Self::new(Vec::new())
    }
}

impl<M: Clone> From<Vec<Polygon<M>>> for MeshPolygons<M> {
    fn from(polygons: Vec<Polygon<M>>) -> Self {
        Self::new(polygons)
    }
}

impl<M: Clone> FromIterator<Polygon<M>> for MeshPolygons<M> {
    fn from_iter<T: IntoIterator<Item = Polygon<M>>>(iter: T) -> Self {
        Self::new(iter.into_iter().collect())
    }
}

impl<M: Clone> IntoIterator for MeshPolygons<M> {
    type Item = Polygon<M>;
    type IntoIter = std::vec::IntoIter<Polygon<M>>;

    fn into_iter(self) -> Self::IntoIter {
        self.into_vec().into_iter()
    }
}

impl<'a, M: Clone> IntoIterator for &'a MeshPolygons<M> {
    type Item = &'a Polygon<M>;
    type IntoIter = std::slice::Iter<'a, Polygon<M>>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl<'a, M: Clone> IntoIterator for &'a mut MeshPolygons<M> {
    type Item = &'a mut Polygon<M>;
    type IntoIter = std::slice::IterMut<'a, Polygon<M>>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter_mut()
    }
}

fn polygon_topology<M: Clone>(polygons: &[Polygon<M>]) -> (usize, usize, bool) {
    polygons
        .iter()
        .fold((0, 0, true), |(facets, vertices, all_triangles), polygon| {
            let polygon_vertices = polygon.vertices.len();
            (
                facets + polygon_vertices.saturating_sub(2),
                vertices + polygon_vertices,
                all_triangles && polygon_vertices == 3,
            )
        })
}

#[derive(Clone, Debug)]
pub struct Mesh<M: Clone + Send + Sync + Debug> {
    /// Triangle-only volumetric surface storage.
    pub(crate) polygons: MeshPolygons<M>,

    /// Lazily calculated AABB that spans `polygons`.
    pub bounding_box: OnceLock<Aabb>,
}

#[derive(Clone, Debug)]
struct CachedMassProperties {
    geometry_identity: Vec<u64>,
    density: Real,
    report: HyperMassPropertyReport3,
}

#[derive(Clone, Debug)]
struct CachedBasicMassProperties {
    geometry_identity: Vec<u64>,
    density: Real,
    mass: Real,
    center: Point3,
}

#[derive(Clone, Debug)]
struct CachedRayIntersections {
    spatial_identity: Vec<u64>,
    origin: Point3,
    direction: Vector3,
    hits: Vec<(Point3, Real)>,
}

#[derive(Clone, Debug)]
struct CachedPolylineIntersections {
    spatial_identity: Vec<u64>,
    polyline: Vec<Point3>,
    hits: Vec<Point3>,
}

thread_local! {
    static LAST_MASS_PROPERTIES: RefCell<Option<CachedMassProperties>> = const { RefCell::new(None) };
    static LAST_BASIC_MASS_PROPERTIES: RefCell<Option<CachedBasicMassProperties>> = const { RefCell::new(None) };
    static LAST_RAY_INTERSECTIONS: RefCell<Option<CachedRayIntersections>> = const { RefCell::new(None) };
    static LAST_POLYLINE_INTERSECTIONS: RefCell<Option<CachedPolylineIntersections>> = const { RefCell::new(None) };
}

fn real_cmp(lhs: &Real, rhs: &Real) -> std::cmp::Ordering {
    if let (Some(lhs), Some(rhs)) = (lhs.exact_rational_ref(), rhs.exact_rational_ref()) {
        return lhs
            .partial_cmp(rhs)
            .expect("exact rationals are totally ordered");
    }
    hyperlimit::compare_reals(lhs, rhs)
        .value()
        .unwrap_or_else(|| match (lhs.clone() - rhs.clone()).refine_sign_until(-128) {
            Some(RealSign::Positive) => std::cmp::Ordering::Greater,
            Some(RealSign::Negative) => std::cmp::Ordering::Less,
            Some(RealSign::Zero) | None => std::cmp::Ordering::Equal,
        })
}

fn real_add_exact_fast(lhs: &Real, rhs: &Real) -> Real {
    if let (Some(lhs), Some(rhs)) = (lhs.exact_rational_ref(), rhs.exact_rational_ref()) {
        Real::from(lhs + rhs)
    } else {
        lhs + rhs
    }
}

fn real_mul_exact_fast(lhs: &Real, rhs: &Real) -> Real {
    if let (Some(lhs), Some(rhs)) = (lhs.exact_rational_ref(), rhs.exact_rational_ref()) {
        Real::from(lhs * rhs)
    } else {
        lhs * rhs
    }
}

fn real_gt(lhs: &Real, rhs: &Real) -> bool {
    matches!(real_cmp(lhs, rhs), std::cmp::Ordering::Greater)
}

fn real_lt(lhs: &Real, rhs: &Real) -> bool {
    matches!(real_cmp(lhs, rhs), std::cmp::Ordering::Less)
}

fn real_max(lhs: &Real, rhs: &Real) -> Real {
    if real_lt(lhs, rhs) {
        rhs.clone()
    } else {
        lhs.clone()
    }
}

fn real_min(lhs: &Real, rhs: &Real) -> Real {
    if real_gt(lhs, rhs) {
        rhs.clone()
    } else {
        lhs.clone()
    }
}

fn axis_aligned_box_intersection(left: &Aabb, right: &Aabb) -> Option<Aabb> {
    let mins = Point3::new(
        real_max(&left.mins.x, &right.mins.x),
        real_max(&left.mins.y, &right.mins.y),
        real_max(&left.mins.z, &right.mins.z),
    );
    let maxs = Point3::new(
        real_min(&left.maxs.x, &right.maxs.x),
        real_min(&left.maxs.y, &right.maxs.y),
        real_min(&left.maxs.z, &right.maxs.z),
    );
    (real_lt(&mins.x, &maxs.x) && real_lt(&mins.y, &maxs.y) && real_lt(&mins.z, &maxs.z))
        .then(|| Aabb::new(mins, maxs))
}

fn axis_aligned_box_rectangular_union(
    left: &Aabb,
    right: &Aabb,
) -> Option<(Aabb, usize, Real)> {
    let equal_x = left.mins.x == right.mins.x && left.maxs.x == right.maxs.x;
    let equal_y = left.mins.y == right.mins.y && left.maxs.y == right.maxs.y;
    let equal_z = left.mins.z == right.mins.z && left.maxs.z == right.maxs.z;
    let joins_x =
        equal_y && equal_z && (left.maxs.x == right.mins.x || right.maxs.x == left.mins.x);
    let joins_y =
        equal_x && equal_z && (left.maxs.y == right.mins.y || right.maxs.y == left.mins.y);
    let joins_z =
        equal_x && equal_y && (left.maxs.z == right.mins.z || right.maxs.z == left.mins.z);
    let (axis, seam) = if joins_x {
        (0, real_max(&left.mins.x, &right.mins.x))
    } else if joins_y {
        (1, real_max(&left.mins.y, &right.mins.y))
    } else if joins_z {
        (2, real_max(&left.mins.z, &right.mins.z))
    } else {
        return None;
    };
    Some((
        Aabb::new(
            Point3::new(
                real_min(&left.mins.x, &right.mins.x),
                real_min(&left.mins.y, &right.mins.y),
                real_min(&left.mins.z, &right.mins.z),
            ),
            Point3::new(
                real_max(&left.maxs.x, &right.maxs.x),
                real_max(&left.maxs.y, &right.maxs.y),
                real_max(&left.maxs.z, &right.maxs.z),
            ),
        ),
        axis,
        seam,
    ))
}

fn axis_aligned_box_min_corner_difference<M: Clone + Send + Sync + Debug>(
    left: &Aabb,
    right: &Aabb,
    left_metadata: &M,
    right_metadata: &M,
) -> Option<Mesh<M>> {
    if left.mins != right.mins
        || !real_lt(&right.maxs.x, &left.maxs.x)
        || !real_lt(&right.maxs.y, &left.maxs.y)
        || !real_lt(&right.maxs.z, &left.maxs.z)
    {
        return None;
    }

    let point = |axis: usize, coordinate: Real, u: Real, v: Real| match axis {
        0 => Point3::new(coordinate, u, v),
        1 => Point3::new(v, coordinate, u),
        _ => Point3::new(u, v, coordinate),
    };
    let mut faces = Vec::<(Vec<Point3>, Vector3, usize, M)>::with_capacity(9);
    let mut push_face = |axis: usize,
                         coordinate: Real,
                         positive: bool,
                         boundary: Vec<[Real; 2]>,
                         plane_slot: usize,
                         metadata: &M| {
        let mut points = boundary
            .into_iter()
            .map(|[u, v]| point(axis, coordinate.clone(), u, v))
            .collect::<Vec<_>>();
        if !positive {
            points.reverse();
        }
        let mut normal_components = [Real::zero(), Real::zero(), Real::zero()];
        normal_components[axis] = if positive { Real::one() } else { -Real::one() };
        faces.push((
            points,
            Vector3::new(normal_components),
            plane_slot,
            metadata.clone(),
        ));
    };

    push_face(
        0,
        left.maxs.x.clone(),
        true,
        vec![
            [left.mins.y.clone(), left.mins.z.clone()],
            [left.maxs.y.clone(), left.mins.z.clone()],
            [left.maxs.y.clone(), left.maxs.z.clone()],
            [left.mins.y.clone(), left.maxs.z.clone()],
        ],
        1,
        left_metadata,
    );
    push_face(
        1,
        left.maxs.y.clone(),
        true,
        vec![
            [left.mins.z.clone(), left.mins.x.clone()],
            [left.maxs.z.clone(), left.mins.x.clone()],
            [left.maxs.z.clone(), left.maxs.x.clone()],
            [left.mins.z.clone(), left.maxs.x.clone()],
        ],
        3,
        left_metadata,
    );
    push_face(
        2,
        left.maxs.z.clone(),
        true,
        vec![
            [left.mins.x.clone(), left.mins.y.clone()],
            [left.maxs.x.clone(), left.mins.y.clone()],
            [left.maxs.x.clone(), left.maxs.y.clone()],
            [left.mins.x.clone(), left.maxs.y.clone()],
        ],
        5,
        left_metadata,
    );

    push_face(
        0,
        left.mins.x.clone(),
        false,
        vec![
            [right.maxs.y.clone(), left.mins.z.clone()],
            [left.maxs.y.clone(), left.mins.z.clone()],
            [left.maxs.y.clone(), left.maxs.z.clone()],
            [left.mins.y.clone(), left.maxs.z.clone()],
            [left.mins.y.clone(), right.maxs.z.clone()],
            [right.maxs.y.clone(), right.maxs.z.clone()],
        ],
        0,
        left_metadata,
    );
    push_face(
        1,
        left.mins.y.clone(),
        false,
        vec![
            [right.maxs.z.clone(), left.mins.x.clone()],
            [left.maxs.z.clone(), left.mins.x.clone()],
            [left.maxs.z.clone(), left.maxs.x.clone()],
            [left.mins.z.clone(), left.maxs.x.clone()],
            [left.mins.z.clone(), right.maxs.x.clone()],
            [right.maxs.z.clone(), right.maxs.x.clone()],
        ],
        2,
        left_metadata,
    );
    push_face(
        2,
        left.mins.z.clone(),
        false,
        vec![
            [right.maxs.x.clone(), left.mins.y.clone()],
            [left.maxs.x.clone(), left.mins.y.clone()],
            [left.maxs.x.clone(), left.maxs.y.clone()],
            [left.mins.x.clone(), left.maxs.y.clone()],
            [left.mins.x.clone(), right.maxs.y.clone()],
            [right.maxs.x.clone(), right.maxs.y.clone()],
        ],
        4,
        left_metadata,
    );

    push_face(
        0,
        right.maxs.x.clone(),
        false,
        vec![
            [right.mins.y.clone(), right.mins.z.clone()],
            [right.maxs.y.clone(), right.mins.z.clone()],
            [right.maxs.y.clone(), right.maxs.z.clone()],
            [right.mins.y.clone(), right.maxs.z.clone()],
        ],
        6,
        right_metadata,
    );
    push_face(
        1,
        right.maxs.y.clone(),
        false,
        vec![
            [right.mins.z.clone(), right.mins.x.clone()],
            [right.maxs.z.clone(), right.mins.x.clone()],
            [right.maxs.z.clone(), right.maxs.x.clone()],
            [right.mins.z.clone(), right.maxs.x.clone()],
        ],
        7,
        right_metadata,
    );
    push_face(
        2,
        right.maxs.z.clone(),
        false,
        vec![
            [right.mins.x.clone(), right.mins.y.clone()],
            [right.maxs.x.clone(), right.mins.y.clone()],
            [right.maxs.x.clone(), right.maxs.y.clone()],
            [right.mins.x.clone(), right.maxs.y.clone()],
        ],
        8,
        right_metadata,
    );

    let first_position_id = reserve_position_ids(14 * 4);
    let first_plane_id = reserve_plane_ids(9);
    let mut unique_positions = Vec::<Point3>::with_capacity(14);
    let mut vertices = Vec::with_capacity(42);
    let mut polygons = Vec::with_capacity(9);
    for (face, normal, plane_slot, metadata) in faces {
        let start = vertices.len();
        for position in face {
            let slot = unique_positions
                .iter()
                .position(|candidate| *candidate == position)
                .unwrap_or_else(|| {
                    unique_positions.push(position.clone());
                    unique_positions.len() - 1
                });
            vertices.push(Vertex::new_with_reserved_identity(
                position,
                normal.clone(),
                first_position_id,
                slot,
            ));
        }
        polygons.push((
            start..vertices.len(),
            first_plane_id + u64::try_from(plane_slot).ok()?,
            metadata,
        ));
    }
    debug_assert_eq!(unique_positions.len(), 14);
    let vertices = Arc::new(vertices);
    let polygons = polygons
        .into_iter()
        .map(|(range, plane_id, metadata)| {
            Polygon::from_shared_vertices(Arc::clone(&vertices), range, metadata, plane_id)
        })
        .collect();
    let mut mesh = Mesh::from_polygons(polygons);
    mesh.bounding_box = OnceLock::from(left.clone());
    mesh.cache_manifold_fact(true);
    Some(mesh)
}

fn aabbs_f64_decided_disjoint(left: &Aabb, right: &Aabb) -> bool {
    let finite = [
        left.mins.x.to_f64_exact_dyadic(),
        left.mins.y.to_f64_exact_dyadic(),
        left.mins.z.to_f64_exact_dyadic(),
        left.maxs.x.to_f64_exact_dyadic(),
        left.maxs.y.to_f64_exact_dyadic(),
        left.maxs.z.to_f64_exact_dyadic(),
        right.mins.x.to_f64_exact_dyadic(),
        right.mins.y.to_f64_exact_dyadic(),
        right.mins.z.to_f64_exact_dyadic(),
        right.maxs.x.to_f64_exact_dyadic(),
        right.maxs.y.to_f64_exact_dyadic(),
        right.maxs.z.to_f64_exact_dyadic(),
    ];
    let [
        Some(lmin_x),
        Some(lmin_y),
        Some(lmin_z),
        Some(lmax_x),
        Some(lmax_y),
        Some(lmax_z),
        Some(rmin_x),
        Some(rmin_y),
        Some(rmin_z),
        Some(rmax_x),
        Some(rmax_y),
        Some(rmax_z),
    ] = finite
    else {
        return false;
    };
    lmax_x < rmin_x
        || rmax_x < lmin_x
        || lmax_y < rmin_y
        || rmax_y < lmin_y
        || lmax_z < rmin_z
        || rmax_z < lmin_z
}

fn real_zero(value: &Real) -> bool {
    if let Some(value) = value.exact_rational_ref() {
        return value.is_zero();
    }
    matches!(value.refine_sign_until(-128), Some(RealSign::Zero))
}

fn real_sign(value: &Real) -> Option<RealSign> {
    if let Some(value) = value.exact_rational_ref() {
        return Some(if value.is_zero() {
            RealSign::Zero
        } else if value.is_positive() {
            RealSign::Positive
        } else {
            RealSign::Negative
        });
    }
    value.refine_sign_until(-128)
}

fn include_point3_bounds(bounds: &mut Option<(Point3, Point3)>, point: &Point3) {
    let Some((mins, maxs)) = bounds else {
        *bounds = Some((point.clone(), point.clone()));
        return;
    };
    if real_lt(&point.x, &mins.x) {
        mins.x = point.x.clone();
    }
    if real_lt(&point.y, &mins.y) {
        mins.y = point.y.clone();
    }
    if real_lt(&point.z, &mins.z) {
        mins.z = point.z.clone();
    }
    if real_gt(&point.x, &maxs.x) {
        maxs.x = point.x.clone();
    }
    if real_gt(&point.y, &maxs.y) {
        maxs.y = point.y.clone();
    }
    if real_gt(&point.z, &maxs.z) {
        maxs.z = point.z.clone();
    }
}

fn aabbs_decided_disjoint(left: &Aabb, right: &Aabb) -> bool {
    [
        (&left.maxs.x, &right.mins.x),
        (&right.maxs.x, &left.mins.x),
        (&left.maxs.y, &right.mins.y),
        (&right.maxs.y, &left.mins.y),
        (&left.maxs.z, &right.mins.z),
        (&right.maxs.z, &left.mins.z),
    ]
    .into_iter()
    .any(|(maximum, minimum)| {
        matches!(
            hyperlimit::compare_reals(maximum, minimum).value(),
            Some(std::cmp::Ordering::Less)
        )
    })
}

fn point_decided_outside_aabb(point: &Point3, bounds: &Aabb) -> bool {
    [
        (&point.x, &bounds.mins.x, &bounds.maxs.x),
        (&point.y, &bounds.mins.y, &bounds.maxs.y),
        (&point.z, &bounds.mins.z, &bounds.maxs.z),
    ]
    .into_iter()
    .any(|(coordinate, minimum, maximum)| {
        matches!(
            hyperlimit::compare_reals(coordinate, minimum).value(),
            Some(std::cmp::Ordering::Less)
        ) || matches!(
            hyperlimit::compare_reals(coordinate, maximum).value(),
            Some(std::cmp::Ordering::Greater)
        )
    })
}

fn exact_f64(value: &Real) -> Option<f64> {
    if !value.is_exact_dyadic_rational() {
        return None;
    }
    let approximate = value.to_f64_lossy()?;
    if !approximate.is_finite() {
        return None;
    }
    let promoted = Real::try_from(approximate).ok()?;
    (promoted.exact_rational_ref() == value.exact_rational_ref()).then_some(approximate)
}

fn exact_integer_i64(value: &Real) -> Option<i64> {
    i64::try_from(value.exact_rational_ref()?.to_big_integer()?).ok()
}

fn canonical_sin_cos_degrees(
    angle_degrees: i64,
    half: &Real,
    sqrt_three_halves: &Real,
) -> Option<(Real, Real)> {
    let zero = Real::zero();
    let one = Real::one();
    Some(match angle_degrees.rem_euclid(360) {
        0 => (zero, one),
        30 => (half.clone(), sqrt_three_halves.clone()),
        60 => (sqrt_three_halves.clone(), half.clone()),
        90 => (one, zero),
        120 => (sqrt_three_halves.clone(), -half.clone()),
        150 => (half.clone(), -sqrt_three_halves.clone()),
        180 => (zero, -one),
        210 => (-half.clone(), -sqrt_three_halves.clone()),
        240 => (-sqrt_three_halves.clone(), -half.clone()),
        270 => (-one, zero),
        300 => (-sqrt_three_halves.clone(), half.clone()),
        330 => (-half.clone(), sqrt_three_halves.clone()),
        _ => return None,
    })
}

fn point_exact_f64(point: &Point3) -> Option<[f64; 3]> {
    Some([
        exact_f64(&point.x)?,
        exact_f64(&point.y)?,
        exact_f64(&point.z)?,
    ])
}

fn vector_exact_f64(vector: &Vector3) -> Option<[f64; 3]> {
    Some([
        exact_f64(&vector.0[0])?,
        exact_f64(&vector.0[1])?,
        exact_f64(&vector.0[2])?,
    ])
}

fn matrix_f64_lossy(matrix: &Matrix4) -> Option<[[f64; 4]; 4]> {
    if (0..4)
        .all(|row| (0..4).all(|column| matrix[row][column].exact_rational_ref().is_some()))
    {
        return None;
    }
    let mut finite = [[0.0; 4]; 4];
    for row in 0..4 {
        for column in 0..4 {
            finite[row][column] = matrix[row][column].to_f64_lossy()?;
        }
    }
    Some(finite)
}

/// Materialize a finite matrix specifically for an already-lossy position
/// boundary.
///
/// Unlike [`matrix_f64_lossy`], exact-rational matrices are accepted here:
/// callers retain a separate lazy exact transform recipe and use this matrix
/// only to map an existing finite position cache. Exact geometry and
/// predicates continue to materialize through that recipe.
fn matrix_f64_position_boundary(matrix: &Matrix4) -> Option<[[f64; 4]; 4]> {
    let mut finite = [[0.0; 4]; 4];
    for row in 0..4 {
        for column in 0..4 {
            finite[row][column] = matrix[row][column].to_f64_lossy()?;
        }
    }
    Some(finite)
}

fn transform_point_f64_lossy(matrix: &[[f64; 4]; 4], point: [f64; 3]) -> Option<[f64; 3]> {
    let homogeneous = [point[0], point[1], point[2], 1.0];
    let mut transformed = [0.0; 4];
    for row in 0..4 {
        transformed[row] = matrix[row]
            .iter()
            .zip(homogeneous)
            .map(|(coefficient, coordinate)| coefficient * coordinate)
            .sum();
    }
    let w = transformed[3];
    if !w.is_finite() || w == 0.0 {
        return None;
    }
    let point = [transformed[0] / w, transformed[1] / w, transformed[2] / w];
    point
        .iter()
        .all(|coordinate| coordinate.is_finite())
        .then_some(point)
}

fn rotation_xyz_f64_lossy(degrees: &[Real; 3]) -> Option<[[f64; 4]; 4]> {
    let [x, y, z] = [
        degrees[0].to_f64_lossy()?.to_radians(),
        degrees[1].to_f64_lossy()?.to_radians(),
        degrees[2].to_f64_lossy()?.to_radians(),
    ];
    let (sin_x, cos_x) = x.sin_cos();
    let (sin_y, cos_y) = y.sin_cos();
    let (sin_z, cos_z) = z.sin_cos();
    Some([
        [
            cos_z * cos_y,
            cos_z * sin_y * sin_x - sin_z * cos_x,
            cos_z * sin_y * cos_x + sin_z * sin_x,
            0.0,
        ],
        [
            sin_z * cos_y,
            sin_z * sin_y * sin_x + cos_z * cos_x,
            sin_z * sin_y * cos_x - cos_z * sin_x,
            0.0,
        ],
        [-sin_y, cos_y * sin_x, cos_y * cos_x, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
}

fn point_decided_outside_certified_f64_bounds(
    point: [f64; 3],
    bounds: &CertifiedF64Bounds,
) -> bool {
    (0..3).any(|axis| point[axis] < bounds.min[axis] || point[axis] > bounds.max[axis])
}

fn ray_decided_miss_certified_f64_bounds(
    origin: [f64; 3],
    direction: [f64; 3],
    bounds: &CertifiedF64Bounds,
    parameter_maximum: Option<f64>,
) -> bool {
    let mut active_axis = None;
    for (axis, ray_direction) in direction.iter().copied().enumerate() {
        if ray_direction != 0.0 {
            if active_axis.is_some() {
                active_axis = None;
                break;
            }
            active_axis = Some(axis);
        }
    }
    if let Some(axis) = active_axis {
        for (fixed_axis, ray_origin) in origin.iter().copied().enumerate() {
            if fixed_axis != axis
                && (ray_origin < bounds.min[fixed_axis] || ray_origin > bounds.max[fixed_axis])
            {
                return true;
            }
        }
        if let Some(maximum) = parameter_maximum {
            let end = origin[axis] + direction[axis] * maximum;
            let segment_minimum = origin[axis].min(end).next_down();
            let segment_maximum = origin[axis].max(end).next_up();
            return bounds.max[axis] < segment_minimum || bounds.min[axis] > segment_maximum;
        }
        return if direction[axis].is_sign_positive() {
            bounds.max[axis] < origin[axis]
        } else {
            bounds.min[axis] > origin[axis]
        };
    }

    let mut interval_minimum = 0.0_f64;
    let mut interval_maximum = parameter_maximum.unwrap_or(f64::INFINITY);

    for (axis, ray_direction) in direction.iter().copied().enumerate() {
        let ray_origin = origin[axis];
        if ray_direction == 0.0 {
            if ray_origin < bounds.min[axis] || ray_origin > bounds.max[axis] {
                return true;
            }
            continue;
        }

        let numerator_minimum = (bounds.min[axis] - ray_origin).next_down();
        let numerator_maximum = (bounds.max[axis] - ray_origin).next_up();
        let (near, far) = if ray_direction.is_sign_positive() {
            (
                (numerator_minimum / ray_direction).next_down(),
                (numerator_maximum / ray_direction).next_up(),
            )
        } else {
            (
                (numerator_maximum / ray_direction).next_down(),
                (numerator_minimum / ray_direction).next_up(),
            )
        };
        if near.is_nan() || far.is_nan() {
            return false;
        }
        interval_minimum = interval_minimum.max(near);
        interval_maximum = interval_maximum.min(far);
        if interval_maximum < interval_minimum {
            return true;
        }
    }
    false
}

fn ray_decided_miss_aabb(
    origin: &Point3,
    direction: &Vector3,
    bounds: &Aabb,
    parameter_maximum: Option<&Real>,
) -> bool {
    let zero = Real::zero();
    let axes = [
        (&origin.x, &direction.0[0], &bounds.mins.x, &bounds.maxs.x),
        (&origin.y, &direction.0[1], &bounds.mins.y, &bounds.maxs.y),
        (&origin.z, &direction.0[2], &bounds.mins.z, &bounds.maxs.z),
    ];

    // Zero-direction slabs are cheap and commonly reject axis-aligned queries
    // before any exact division is needed.
    for (ray_origin, ray_direction, minimum, maximum) in axes {
        if matches!(
            hyperlimit::compare_reals(ray_direction, &zero).value(),
            Some(std::cmp::Ordering::Equal)
        ) && (matches!(
            hyperlimit::compare_reals(ray_origin, minimum).value(),
            Some(std::cmp::Ordering::Less)
        ) || matches!(
            hyperlimit::compare_reals(ray_origin, maximum).value(),
            Some(std::cmp::Ordering::Greater)
        )) {
            return true;
        }
    }

    let mut interval_minimum = zero.clone();
    let mut interval_maximum = parameter_maximum.cloned();
    for (ray_origin, ray_direction, minimum, maximum) in axes {
        let Some(direction_ordering) = hyperlimit::compare_reals(ray_direction, &zero).value()
        else {
            // An uncertified broad-phase fact may not reject an exact candidate.
            return false;
        };
        if direction_ordering == std::cmp::Ordering::Equal {
            continue;
        }

        let Ok(first) = (minimum.clone() - ray_origin.clone()) / ray_direction.clone() else {
            return false;
        };
        let Ok(second) = (maximum.clone() - ray_origin.clone()) / ray_direction.clone() else {
            return false;
        };
        let (near, far) = if direction_ordering == std::cmp::Ordering::Greater {
            (first, second)
        } else {
            (second, first)
        };
        if matches!(
            hyperlimit::compare_reals(&far, &zero).value(),
            Some(std::cmp::Ordering::Less)
        ) {
            return true;
        }
        if matches!(
            hyperlimit::compare_reals(&near, &interval_minimum).value(),
            Some(std::cmp::Ordering::Greater)
        ) {
            interval_minimum = near;
        }
        if let Some(current_maximum) = &interval_maximum
            && matches!(
                hyperlimit::compare_reals(&far, current_maximum).value(),
                Some(std::cmp::Ordering::Less)
            )
        {
            interval_maximum = Some(far);
        } else if interval_maximum.is_none() {
            interval_maximum = Some(far);
        }
        if interval_maximum.as_ref().is_some_and(|maximum| {
            matches!(
                hyperlimit::compare_reals(maximum, &interval_minimum).value(),
                Some(std::cmp::Ordering::Less)
            )
        }) {
            return true;
        }
    }
    false
}

fn concatenate_disjoint_meshes<M: Clone + Send + Sync + Debug>(
    left: &Mesh<M>,
    right: &Mesh<M>,
) -> Mesh<M> {
    let (left_facets, left_vertices, left_triangles) = left.polygons.topology();
    let (right_facets, right_vertices, right_triangles) = right.polygons.topology();
    let mut polygons = Vec::with_capacity(left.polygons.len() + right.polygons.len());
    polygons.extend(left.polygons.iter().cloned());
    polygons.extend(right.polygons.iter().cloned());
    Mesh::from_polygons_with_topology(
        polygons,
        (
            left_facets + right_facets,
            left_vertices + right_vertices,
            left_triangles && right_triangles,
        ),
    )
}

fn hyperphysics_vector3_from_point3(point: &Point3) -> Result<HyperVector3, ValidationError> {
    Ok(HyperVector3::new([
        point.x.clone(),
        point.y.clone(),
        point.z.clone(),
    ]))
}

#[cfg(test)]
fn ray_triangle_intersection(
    origin: &Point3,
    direction: &Vector3,
    tri: &[Vertex; 3],
) -> Option<(Point3, Real)> {
    ray_triangle_positions_hyperlimit(
        origin,
        direction,
        [&tri[0].position, &tri[1].position, &tri[2].position],
    )
}

fn ray_triangle_positions(
    origin: &Point3,
    direction: &Vector3,
    positions: [&Point3; 3],
) -> Option<(Point3, Real)> {
    ray_triangle_positions_prepared(origin, direction, positions, None)
}

fn ray_triangle_positions_prepared(
    origin: &Point3,
    direction: &Vector3,
    positions: [&Point3; 3],
    prepared: Option<&PreparedTriangleQuery>,
) -> Option<(Point3, Real)> {
    match ray_triangle_positions_moller_trumbore(origin, direction, positions, prepared) {
        Some(result) => result,
        None => ray_triangle_positions_hyperlimit(origin, direction, positions),
    }
}

fn ray_triangle_positions_moller_trumbore(
    origin: &Point3,
    direction: &Vector3,
    positions: [&Point3; 3],
    prepared: Option<&PreparedTriangleQuery>,
) -> Option<Option<(Point3, Real)>> {
    if let Some(result) = ray_triangle_positions_x_axis(origin, direction, positions, prepared)
    {
        return Some(result);
    }

    let computed_edges;
    let (edge1, edge2) = if let Some(prepared) = prepared {
        (&prepared.edge1, &prepared.edge2)
    } else {
        computed_edges = (
            positions[1].clone() - positions[0].clone(),
            positions[2].clone() - positions[0].clone(),
        );
        (&computed_edges.0, &computed_edges.1)
    };
    let perpendicular = direction.cross(edge2);
    let determinant = edge1.dot(&perpendicular);
    let determinant_sign = real_sign(&determinant)?;
    if determinant_sign == RealSign::Zero {
        return Some(None);
    }

    let from_first = origin.clone() - positions[0].clone();
    let u_numerator = from_first.dot(&perpendicular);
    let u_sign = real_sign(&u_numerator)?;
    if (determinant_sign == RealSign::Positive && u_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && u_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let cross = from_first.cross(edge1);
    let v_numerator = direction.dot(&cross);
    let v_sign = real_sign(&v_numerator)?;
    if (determinant_sign == RealSign::Positive && v_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && v_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let barycentric_remainder =
        determinant.clone() - u_numerator.clone() - v_numerator.clone();
    let remainder_sign = real_sign(&barycentric_remainder)?;
    if (determinant_sign == RealSign::Positive && remainder_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && remainder_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let t_numerator = edge2.dot(&cross);
    let t_sign = real_sign(&t_numerator)?;
    if (determinant_sign == RealSign::Positive && t_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && t_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let parameter = (t_numerator / determinant).ok()?;
    let point = origin.clone() + direction.clone() * parameter.clone();
    Some(Some((point, parameter)))
}

fn ray_triangle_positions_x_axis(
    origin: &Point3,
    direction: &Vector3,
    positions: [&Point3; 3],
    prepared: Option<&PreparedTriangleQuery>,
) -> Option<Option<(Point3, Real)>> {
    if let Some(result) =
        ray_triangle_positions_x_axis_exact_rational(origin, direction, positions, prepared)
    {
        return Some(result);
    }
    if real_sign(&direction.0[1])? != RealSign::Zero
        || real_sign(&direction.0[2])? != RealSign::Zero
        || real_sign(&direction.0[0])? == RealSign::Zero
    {
        return None;
    }

    let computed_edges;
    let (edge1, edge2) = if let Some(prepared) = prepared {
        (&prepared.edge1, &prepared.edge2)
    } else {
        computed_edges = (
            positions[1].clone() - positions[0].clone(),
            positions[2].clone() - positions[0].clone(),
        );
        (&computed_edges.0, &computed_edges.1)
    };
    let scale = &direction.0[0];
    let determinant = scale.clone()
        * Real::signed_product_sum(
            [true, false],
            [[&edge1.0[2], &edge2.0[1]], [&edge1.0[1], &edge2.0[2]]],
        );
    let determinant_sign = real_sign(&determinant)?;
    if determinant_sign == RealSign::Zero {
        return Some(None);
    }

    let from_first = origin.clone() - positions[0].clone();
    let u_numerator = scale.clone()
        * Real::signed_product_sum(
            [true, false],
            [
                [&from_first.0[2], &edge2.0[1]],
                [&from_first.0[1], &edge2.0[2]],
            ],
        );
    let u_sign = real_sign(&u_numerator)?;
    if (determinant_sign == RealSign::Positive && u_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && u_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let v_numerator = scale.clone()
        * Real::signed_product_sum(
            [true, false],
            [
                [&from_first.0[1], &edge1.0[2]],
                [&from_first.0[2], &edge1.0[1]],
            ],
        );
    let v_sign = real_sign(&v_numerator)?;
    if (determinant_sign == RealSign::Positive && v_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && v_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let barycentric_remainder =
        determinant.clone() - u_numerator.clone() - v_numerator.clone();
    let remainder_sign = real_sign(&barycentric_remainder)?;
    if (determinant_sign == RealSign::Positive && remainder_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && remainder_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let cross = from_first.cross(edge1);
    let t_numerator = edge2.dot(&cross);
    let t_sign = real_sign(&t_numerator)?;
    if (determinant_sign == RealSign::Positive && t_sign == RealSign::Negative)
        || (determinant_sign == RealSign::Negative && t_sign == RealSign::Positive)
    {
        return Some(None);
    }

    let parameter = (t_numerator / determinant).ok()?;
    let point = origin.clone() + direction.clone() * parameter.clone();
    Some(Some((point, parameter)))
}

fn ray_triangle_positions_x_axis_exact_rational(
    origin: &Point3,
    direction: &Vector3,
    positions: [&Point3; 3],
    prepared: Option<&PreparedTriangleQuery>,
) -> Option<Option<(Point3, Real)>> {
    let direction_x = direction.0[0].exact_rational_ref()?;
    if direction_x.is_zero()
        || !direction.0[1].exact_rational_ref()?.is_zero()
        || !direction.0[2].exact_rational_ref()?.is_zero()
    {
        return None;
    }
    let exact = prepared_exact_x_axis_query(prepared?, positions)?;
    let [_edge1_x, edge1_y, edge1_z] = exact.edge1.each_ref();
    let [_edge2_x, edge2_y, edge2_z] = exact.edge2.each_ref();
    let origin_x = origin.x.exact_rational_ref()?;
    let origin_y = origin.y.exact_rational_ref()?;
    let origin_z = origin.z.exact_rational_ref()?;
    let from_y = origin_y - &exact.first[1];
    let from_z = origin_z - &exact.first[2];

    let determinant = direction_x * &exact.determinant_unit;
    if determinant.is_zero() {
        return Some(None);
    }
    let conflicts = |value: &hyperreal::Rational| {
        (determinant.is_positive() && value.is_negative())
            || (determinant.is_negative() && value.is_positive())
    };
    let u_numerator = direction_x * &(from_z.clone() * edge2_y - from_y.clone() * edge2_z);
    if conflicts(&u_numerator) {
        return Some(None);
    }
    let v_numerator = direction_x * &(from_y.clone() * edge1_z - from_z.clone() * edge1_y);
    if conflicts(&v_numerator) {
        return Some(None);
    }
    let remainder = determinant.clone() - &u_numerator - &v_numerator;
    if conflicts(&remainder) {
        return Some(None);
    }

    let t_numerator = origin_x * &exact.normal[0]
        + origin_y * &exact.normal[1]
        + origin_z * &exact.normal[2]
        - &exact.first_dot_normal;
    if conflicts(&t_numerator) {
        return Some(None);
    }
    let parameter = &t_numerator / &determinant;
    let point_x = origin_x + direction_x * &parameter;
    Some(Some((
        Point3::new(
            Real::new(point_x),
            Real::new(origin_y.clone()),
            Real::new(origin_z.clone()),
        ),
        Real::new(parameter),
    )))
}

fn prepared_exact_x_axis_query<'a>(
    prepared: &'a PreparedTriangleQuery,
    positions: [&Point3; 3],
) -> Option<&'a PreparedExactXAxisTriangle> {
    prepared
        .exact_x_axis
        .get_or_init(|| {
            let first = [
                positions[0].x.exact_rational_ref()?.clone(),
                positions[0].y.exact_rational_ref()?.clone(),
                positions[0].z.exact_rational_ref()?.clone(),
            ];
            let edge1 = [
                prepared.edge1.0[0].exact_rational_ref()?.clone(),
                prepared.edge1.0[1].exact_rational_ref()?.clone(),
                prepared.edge1.0[2].exact_rational_ref()?.clone(),
            ];
            let edge2 = [
                prepared.edge2.0[0].exact_rational_ref()?.clone(),
                prepared.edge2.0[1].exact_rational_ref()?.clone(),
                prepared.edge2.0[2].exact_rational_ref()?.clone(),
            ];
            let normal = [
                &edge1[1] * &edge2[2] - &edge1[2] * &edge2[1],
                &edge1[2] * &edge2[0] - &edge1[0] * &edge2[2],
                &edge1[0] * &edge2[1] - &edge1[1] * &edge2[0],
            ];
            let determinant_unit = -normal[0].clone();
            let first_dot_normal =
                &first[0] * &normal[0] + &first[1] * &normal[1] + &first[2] * &normal[2];
            Some(PreparedExactXAxisTriangle {
                first,
                edge1,
                edge2,
                determinant_unit,
                normal,
                first_dot_normal,
            })
        })
        .as_ref()
}

fn ray_triangle_parallel_decided(
    direction: &Vector3,
    positions: [&Point3; 3],
    prepared: Option<&PreparedTriangleQuery>,
) -> Option<bool> {
    let computed_edges;
    let (edge1, edge2) = if let Some(prepared) = prepared {
        (&prepared.edge1, &prepared.edge2)
    } else {
        computed_edges = (
            positions[1].clone() - positions[0].clone(),
            positions[2].clone() - positions[0].clone(),
        );
        (&computed_edges.0, &computed_edges.1)
    };
    let determinant = edge1.dot(&direction.cross(edge2));
    real_sign(&determinant).map(|sign| sign == RealSign::Zero)
}

fn ray_triangle_positions_hyperlimit(
    origin: &Point3,
    direction: &Vector3,
    positions: [&Point3; 3],
) -> Option<(Point3, Real)> {
    let exact_origin = hyperlimit_point3(origin);
    let exact_direction = hyperlimit::Point3::new(
        direction.0[0].clone(),
        direction.0[1].clone(),
        direction.0[2].clone(),
    );
    let a = hyperlimit_point3(positions[0]);
    let b = hyperlimit_point3(positions[1]);
    let c = hyperlimit_point3(positions[2]);
    match hyperlimit::classify_ray_triangle3_intersection_report(
        &exact_origin,
        &exact_direction,
        &a,
        &b,
        &c,
    ) {
        hyperlimit::PredicateOutcome::Decided { value: report, .. } => {
            if matches!(
                report.relation,
                hyperlimit::RayTriangleIntersection::Disjoint
                    | hyperlimit::RayTriangleIntersection::Coplanar
            ) {
                return None;
            }
            let point = report.point?;
            let parameter = report.parameter?;
            Some((Point3::new(point.x, point.y, point.z), parameter))
        },
        hyperlimit::PredicateOutcome::Unknown { .. } => None,
    }
}

fn hyperlimit_point3(point: &Point3) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(point.x.clone(), point.y.clone(), point.z.clone())
}

fn canonicalize_ray_hits(hits: &mut Vec<(Point3, Real)>) {
    hits.sort_by(|a, b| real_cmp(&a.1, &b.1));
    hits.dedup_by(|a, b| {
        if let (Some(a_parameter), Some(b_parameter)) =
            (a.1.exact_rational_ref(), b.1.exact_rational_ref())
        {
            return a_parameter == b_parameter;
        }
        if real_zero(&(a.1.clone() - b.1.clone())) {
            return true;
        }
        let a_point = hyperlimit::Point3::new(a.0.x.clone(), a.0.y.clone(), a.0.z.clone());
        let b_point = hyperlimit::Point3::new(b.0.x.clone(), b.0.y.clone(), b.0.z.clone());
        matches!(
            hyperlimit::point3_equal(&a_point, &b_point).value(),
            Some(true)
        )
    });
}

impl<M: Clone + Send + Sync + Debug + PartialEq> Mesh<M> {
    /// Example: retain only polygons whose metadata matches `needle`
    #[inline]
    pub fn filter_polygons_by_metadata(&self, needle: &M) -> Mesh<M> {
        let polys = self
            .polygons
            .iter()
            .filter(|&p| &p.metadata == needle)
            .cloned()
            .collect();

        Mesh {
            polygons: polys,
            bounding_box: std::sync::OnceLock::new(),
        }
    }
}

fn triangulate_mesh_faces<M: Clone + Send + Sync>(
    polygons: Vec<Polygon<M>>,
) -> Vec<Polygon<M>> {
    polygons
        .into_iter()
        .flat_map(|polygon| {
            if polygon.vertices().len() == 3 {
                return vec![polygon];
            }
            let metadata = polygon.metadata().clone();
            let plane_id = polygon.plane_id;
            polygon
                .triangulate()
                .into_iter()
                .map(|triangle| {
                    Polygon::from_planar_vertices(triangle.to_vec(), metadata.clone())
                        .with_plane_id(plane_id)
                })
                .collect()
        })
        .collect()
}

impl<M: Clone + Send + Sync + Debug> Mesh<M> {
    /// Return a new empty mesh.
    pub fn empty() -> Self {
        Mesh {
            polygons: MeshPolygons::new(Vec::new()),
            bounding_box: OnceLock::new(),
        }
    }

    /// Build a triangle mesh from planar faces.
    ///
    /// Faces with more than three vertices are triangulated at this explicit
    /// compatibility boundary. New code that needs to retain face boundaries
    /// should construct `PolygonMesh` and call `PolygonMesh::triangulate`.
    pub(crate) fn from_polygons(polygons: Vec<Polygon<M>>) -> Self {
        let polygons = triangulate_mesh_faces(polygons);
        Mesh {
            polygons: MeshPolygons::new(polygons),
            bounding_box: OnceLock::new(),
        }
    }

    /// Build a mesh from already-triangulated faces.
    ///
    /// Use [`crate::PolygonMesh::triangulate`] when starting from planar faces
    /// with more than three vertices.
    pub fn from_triangles(triangles: Vec<Triangle<M>>) -> Result<Self, ValidationError> {
        if triangles
            .iter()
            .any(|triangle| triangle.vertices().len() != 3)
        {
            return Err(ValidationError::InvalidArguments);
        }
        Ok(Mesh {
            polygons: MeshPolygons::new(triangles),
            bounding_box: OnceLock::new(),
        })
    }

    /// Borrow the triangle faces backing this mesh.
    pub fn triangles(&self) -> &[Triangle<M>] {
        &self.polygons
    }

    /// Consume this mesh and return its triangle faces.
    pub fn into_triangles(self) -> Vec<Triangle<M>> {
        self.polygons.into_vec()
    }

    /// Build a mesh from exact indexed triangle rows.
    ///
    /// Equal position rows are canonicalized exactly, while distinct authored
    /// normals remain separate render vertices. The indexed construction facts
    /// are retained for transforms, bounds, connectivity, and hypermesh
    /// adapters, avoiding a later triangle-corner topology reconstruction.
    pub fn from_indexed_triangles(
        indexed: IndexedTriangleMesh3D,
        metadata: M,
    ) -> Result<Self, ValidationError> {
        if indexed.faces.is_empty() {
            return Ok(Self::empty());
        }
        for face in &indexed.faces {
            for &(position, normal) in face {
                if position >= indexed.positions.len() {
                    return Err(ValidationError::IndexOutOfRangeWithLen {
                        index: position,
                        len: indexed.positions.len(),
                    });
                }
                if normal >= indexed.normals.len() {
                    return Err(ValidationError::IndexOutOfRangeWithLen {
                        index: normal,
                        len: indexed.normals.len(),
                    });
                }
            }
        }

        let mut source_position_slots = vec![None; indexed.positions.len()];
        let mut position_buckets = hashbrown::HashMap::<IndexedPositionKey, Vec<usize>>::new();
        let mut positions = Vec::<Point3>::new();
        let mut faces = indexed.faces;
        for face in &mut faces {
            for (source_position, _) in face {
                let slot = if let Some(slot) = source_position_slots[*source_position] {
                    slot
                } else {
                    let position = &indexed.positions[*source_position];
                    let key = IndexedPositionKey::new(position);
                    let slot = position_buckets.get(&key).and_then(|candidates| {
                        candidates
                            .iter()
                            .copied()
                            .find(|&slot| positions[slot] == *position)
                    });
                    let slot = slot.unwrap_or_else(|| {
                        let slot = positions.len();
                        positions.push(position.clone());
                        position_buckets.entry(key).or_default().push(slot);
                        slot
                    });
                    source_position_slots[*source_position] = Some(slot);
                    slot
                };
                *source_position = slot;
            }
        }
        let nondegenerate = faces.iter().all(|face| {
            let [a, b, c] = face.map(|(position, _)| position);
            ::hypermesh::Plane::points_are_nondegenerate(
                &positions[a],
                &positions[b],
                &positions[c],
            )
        });

        let mut position_normals = vec![None; positions.len()];
        let mut normals_match_positions = true;
        for face in &faces {
            for &(position, normal) in face {
                match position_normals[position] {
                    Some(existing) if existing != normal => normals_match_positions = false,
                    Some(_) => {},
                    None => position_normals[position] = Some(normal),
                }
            }
        }

        let base_vertices = positions
            .iter()
            .cloned()
            .map(|position| Vertex::new(position, Vector3::z()))
            .collect::<Vec<_>>();
        let mut pool_vertices = Vec::new();
        let mut pool_faces = Vec::with_capacity(faces.len());
        if normals_match_positions {
            pool_vertices.reserve(positions.len());
            for (position, normal) in base_vertices.iter().zip(position_normals) {
                let mut vertex = position.clone();
                vertex.normal = indexed.normals
                    [normal.expect("used indexed position has an authored normal")]
                .clone();
                pool_vertices.push(vertex);
            }
            pool_faces.extend(faces.iter().map(|face| face.map(|(position, _)| position)));
        } else {
            let mut vertex_slots = hashbrown::HashMap::<(usize, usize), usize>::new();
            for face in &faces {
                pool_faces.push(face.map(|(position, normal)| {
                    *vertex_slots.entry((position, normal)).or_insert_with(|| {
                        let slot = pool_vertices.len();
                        let mut vertex = base_vertices[position].clone();
                        vertex.normal = indexed.normals[normal].clone();
                        pool_vertices.push(vertex);
                        slot
                    })
                }));
            }
        }

        let triangle_count = faces.len();
        let vertex_pool = Arc::new(LazySubdivisionVertexPool::new(
            pool_vertices,
            Vec::new(),
            triangle_count,
            0,
        ));
        let first_plane_id = reserve_plane_ids(triangle_count);
        let polygons = pool_faces
            .into_iter()
            .enumerate()
            .map(|(triangle_slot, indices)| {
                Polygon::from_lazy_subdivision_triangle(
                    Arc::clone(&vertex_pool),
                    triangle_slot,
                    indices,
                    metadata.clone(),
                    first_plane_id
                        + u64::try_from(triangle_slot).expect("triangle plane slot fits u64"),
                )
            })
            .collect();
        let mesh = Self::from_polygons_with_topology(
            polygons,
            (triangle_count, triangle_count.saturating_mul(3), true),
        );
        let corner_position_slots = faces
            .iter()
            .flat_map(|face| face.map(|(position, _)| position))
            .collect::<Vec<_>>();
        let position_f64 = positions
            .iter()
            .map(|position| {
                Some([
                    position.x.to_f64_lossy()?,
                    position.y.to_f64_lossy()?,
                    position.z.to_f64_lossy()?,
                ])
            })
            .collect::<Option<Vec<_>>>()
            .map(Arc::new);
        mesh.retain_shared_position_transform_layout(
            corner_position_slots,
            positions.len(),
            position_f64,
            normals_match_positions.then(|| Arc::clone(&vertex_pool)),
            None,
            normals_match_positions,
        );

        let mut adjacent = vec![false; positions.len()];
        let mut edge_counts = hashbrown::HashMap::<(usize, usize), [u8; 2]>::with_capacity(
            faces.len().saturating_mul(3) / 2,
        );
        let mut directed_edges_valid = true;
        for face in &faces {
            let positions = face.map(|(position, _)| position);
            for [left, right] in [
                [positions[0], positions[1]],
                [positions[1], positions[2]],
                [positions[2], positions[0]],
            ] {
                if left != right {
                    adjacent[left] = true;
                    adjacent[right] = true;
                    let (edge, direction) = if left < right {
                        ((left, right), 0)
                    } else {
                        ((right, left), 1)
                    };
                    let counts = edge_counts.entry(edge).or_insert([0; 2]);
                    counts[direction] = counts[direction].saturating_add(1);
                    directed_edges_valid &= counts[direction] <= 1;
                } else {
                    directed_edges_valid = false;
                }
            }
        }
        mesh.polygons.retain_connectivity_counts((
            positions.len(),
            adjacent.into_iter().filter(|adjacent| *adjacent).count(),
        ));
        mesh.polygons
            .retain_nondegenerate_triangles_fact(nondegenerate);
        mesh.polygons.retain_manifold_fact(
            nondegenerate
                && directed_edges_valid
                && !edge_counts.is_empty()
                && edge_counts.values().all(|counts| *counts == [1, 1]),
        );
        let mut bounds = None;
        for position in &positions {
            include_point3_bounds(&mut bounds, position);
        }
        if let Some((mins, maxs)) = bounds {
            let bounds = Aabb::new(mins, maxs);
            let _ = mesh.bounding_box.set(bounds.clone());
            mesh.polygons.retain_axis_aligned_box_fact(bounds);
        }
        Ok(mesh)
    }

    pub(crate) fn from_polygons_with_topology(
        polygons: Vec<Polygon<M>>,
        topology: (usize, usize, bool),
    ) -> Self {
        if !topology.2 || polygons.iter().any(|polygon| polygon.vertices().len() != 3) {
            return Self::from_polygons(polygons);
        }
        Mesh {
            polygons: MeshPolygons::new_with_topology(polygons, topology),
            bounding_box: OnceLock::new(),
        }
    }

    pub(crate) fn retain_shared_position_transform_layout(
        &self,
        corner_position_slots: Vec<usize>,
        position_count: usize,
        position_f64: Option<Arc<Vec<[f64; 3]>>>,
        indexed_triangle_pool: Option<Arc<LazySubdivisionVertexPool>>,
        indexed_polygon_corner_counts: Option<Vec<usize>>,
        normals_match_positions: bool,
    ) {
        self.polygons.retain_transform_layout(Arc::new(
            TransformLayout::shared_position_identity(
                corner_position_slots,
                position_count,
                self.polygons.len(),
                position_f64,
                indexed_triangle_pool,
                indexed_polygon_corner_counts,
                normals_match_positions,
            ),
        ));
    }

    fn retained_indexed_triangle_output(
        &self,
        transform_layout: &TransformLayout,
        vertices: Vec<Vertex>,
        position_f64: Option<Arc<Vec<[f64; 3]>>>,
        plane_ids: u64,
    ) -> Option<Self> {
        let vertex_pool = Arc::new(LazySubdivisionVertexPool::new(
            vertices,
            Vec::new(),
            self.polygons.len(),
            0,
        ));
        self.retained_indexed_triangle_output_from_pool(
            transform_layout,
            vertex_pool,
            position_f64,
            plane_ids,
        )
    }

    fn retained_indexed_triangle_output_from_pool(
        &self,
        transform_layout: &TransformLayout,
        vertex_pool: Arc<LazySubdivisionVertexPool>,
        position_f64: Option<Arc<Vec<[f64; 3]>>>,
        plane_ids: u64,
    ) -> Option<Self> {
        self.retained_indexed_triangle_output_from_pool_oriented(
            transform_layout,
            vertex_pool,
            position_f64,
            plane_ids,
            false,
        )
    }

    fn retained_indexed_triangle_output_from_pool_oriented(
        &self,
        transform_layout: &TransformLayout,
        vertex_pool: Arc<LazySubdivisionVertexPool>,
        position_f64: Option<Arc<Vec<[f64; 3]>>>,
        plane_ids: u64,
        reverse_orientation: bool,
    ) -> Option<Self> {
        if !transform_layout.normals_match_positions
            || transform_layout
                .corner_coordinate_slots
                .iter()
                .any(Option::is_some)
            || !self.polygons.topology().2
            || vertex_pool.len() != transform_layout.position_representatives.len()
        {
            return None;
        }
        let mut reversed_corner_position_slots = reverse_orientation
            .then(|| Vec::with_capacity(transform_layout.corner_position_slots.len()));
        let polygons = transform_layout
            .corner_position_slots
            .chunks_exact(3)
            .enumerate()
            .map(|(polygon_index, slots)| {
                let indices = if reverse_orientation {
                    [slots[2], slots[1], slots[0]]
                } else {
                    [slots[0], slots[1], slots[2]]
                };
                if let Some(corners) = reversed_corner_position_slots.as_mut() {
                    corners.extend(indices);
                }
                Polygon::from_lazy_subdivision_triangle(
                    Arc::clone(&vertex_pool),
                    polygon_index,
                    indices,
                    self.polygons[polygon_index].metadata.clone(),
                    plane_ids
                        + u64::try_from(transform_layout.plane_slot(polygon_index))
                            .expect("plane slot fits u64"),
                )
            })
            .collect();
        let mesh = Mesh::from_polygons_with_topology(polygons, self.polygons.topology());
        let corner_position_slots = reversed_corner_position_slots
            .map(Arc::new)
            .unwrap_or_else(|| Arc::clone(&transform_layout.corner_position_slots));
        mesh.polygons.retain_transform_layout(Arc::new(
            TransformLayout::shared_position_identity_with_representatives(
                corner_position_slots,
                Arc::clone(&transform_layout.position_representatives),
                transform_layout.position_representatives.len(),
                self.polygons.len(),
                position_f64,
                Some(vertex_pool),
                None,
                true,
            ),
        ));
        Some(mesh)
    }

    fn finish_indexed_translation(
        &self,
        mut translated: Self,
        translated_bounds: Option<Aabb>,
        vector: Vector3,
        source_geometry_identity: u64,
        cache_on_completion: bool,
        convex_pwn: bool,
    ) -> Self {
        translated.bounding_box = translated_bounds.map_or_else(OnceLock::new, OnceLock::from);
        if let Some(bounds) = translated.bounding_box.get().cloned()
            && self.polygons.axis_aligned_box_fact().is_some()
        {
            translated
                .polygons
                .retain_axis_aligned_box_fact(bounds.clone());
            let centering_offset = if self.polygons.centering_offset_fact() == Some(&vector) {
                Vector3::zero()
            } else {
                let half =
                    (Real::one() / Real::from(2_u8)).expect("nonzero exact denominator");
                Vector3::new([
                    -(bounds.mins.x + bounds.maxs.x) * half.clone(),
                    -(bounds.mins.y + bounds.maxs.y) * half.clone(),
                    -(bounds.mins.z + bounds.maxs.z) * half,
                ])
            };
            translated.polygons.retain_centering_offset(centering_offset);
        }
        if cache_on_completion {
            LAST_TRANSLATION.with_borrow_mut(|cached| {
                if let Some(index) = cached.iter().position(|cached| {
                    cached.source_geometry_identity == source_geometry_identity
                        && cached.vector == vector
                }) {
                    cached.remove(index);
                }
                if cached.len() == TRANSFORM_CACHE_CAPACITY {
                    cached.remove(0);
                }
                cached.push(CachedTranslation {
                    source_geometry_identity,
                    vector,
                    polygons: translated
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                    bounding_box: translated.bounding_box.get().cloned(),
                    axis_aligned_box: translated.polygons.axis_aligned_box_fact().is_some(),
                    centering_offset: translated.polygons.centering_offset_fact().cloned(),
                    cuboid_vertex_pool: translated.polygons.cuboid_vertex_pool().cloned(),
                });
            });
        }
        if convex_pwn {
            translated.cache_convex_pwn_fact();
        }
        translated
    }

    fn retained_indexed_translation(&self, vector: Vector3) -> Option<Self> {
        let transform_layout = self
            .polygons
            .retained_transform_layout()
            .filter(|layout| {
                layout.normals_match_positions
                    && layout.corner_coordinate_slots.iter().all(Option::is_none)
                    && self.polygons.topology().2
            })?
            .clone();
        let source_pool = transform_layout.indexed_triangle_pool.as_ref()?;
        let source_position_f64 = transform_layout.position_f64.as_ref()?;
        let translation = vector.0[0]
            .to_f64_lossy()
            .zip(vector.0[1].to_f64_lossy())
            .zip(vector.0[2].to_f64_lossy())
            .map(|((x, y), z)| [x, y, z])?;
        let source_geometry_identity = self.polygons.storage_identity();
        let cache_on_completion = PENDING_TRANSLATION.with_borrow_mut(|pending| {
            let repeated = pending.iter().any(|pending| {
                pending.vector == vector
                    && pending.source_geometry_identity == source_geometry_identity
            });
            if !repeated {
                if pending.len() == TRANSFORM_CACHE_CAPACITY {
                    pending.remove(0);
                }
                pending.push(PendingTranslation {
                    source_geometry_identity,
                    vector: vector.clone(),
                });
            }
            repeated
        });
        let translated_bounds = self.polygons.axis_aligned_box_fact().map(|bounds| {
            Aabb::new(
                bounds.mins.clone() + vector.clone(),
                bounds.maxs.clone() + vector.clone(),
            )
        });
        let indexed_position_f64 = Arc::new(
            source_position_f64
                .iter()
                .map(|source| {
                    [
                        source[0] + translation[0],
                        source[1] + translation[1],
                        source[2] + translation[2],
                    ]
                })
                .collect::<Vec<_>>(),
        );
        let position_ids = reserve_position_ids(indexed_position_f64.len());
        cache_shared_position_f64_range(position_ids, Arc::clone(&indexed_position_f64));
        let coordinate_ids = transform_layout.coordinate_counts.map(reserve_position_ids);
        let plane_ids = reserve_plane_ids(transform_layout.plane_count);
        let vertex_pool = Arc::new(LazySubdivisionVertexPool::new_translated(
            Arc::clone(source_pool),
            vector.clone(),
            Some(Arc::clone(&indexed_position_f64)),
            self.polygons.len(),
            position_ids,
            coordinate_ids,
        ));
        let translated = self.retained_indexed_triangle_output_from_pool(
            &transform_layout,
            vertex_pool,
            Some(indexed_position_f64),
            plane_ids,
        )?;
        Some(self.finish_indexed_translation(
            translated,
            translated_bounds,
            vector,
            source_geometry_identity,
            cache_on_completion,
            self.has_convex_pwn_fact(),
        ))
    }

    fn retained_cuboid_translation(&self, vector: Vector3) -> Option<Self> {
        self.polygons.cuboid_vertex_pool()?;
        if self.polygons.len() != 6 {
            return None;
        }
        let translated_bounds = self.polygons.axis_aligned_box_fact().map(|bounds| {
            Aabb::new(
                bounds.mins.clone() + vector.clone(),
                bounds.maxs.clone() + vector.clone(),
            )
        });
        let bounds = translated_bounds.as_ref()?;
        let source_geometry_identity = self.polygons.storage_identity();
        let cache_on_completion = PENDING_TRANSLATION.with_borrow_mut(|pending| {
            let repeated = pending.iter().any(|pending| {
                pending.vector == vector
                    && pending.source_geometry_identity == source_geometry_identity
            });
            if !repeated {
                if pending.len() == TRANSFORM_CACHE_CAPACITY {
                    pending.remove(0);
                }
                pending.push(PendingTranslation {
                    source_geometry_identity,
                    vector: vector.clone(),
                });
            }
            repeated
        });
        let first_position_identity = reserve_position_ids(8);
        let first_coordinate_identity = reserve_position_ids(6);
        let vertex_pool = Arc::new(LazySubdivisionVertexPool::new_cuboid_bounds(
            [
                [bounds.mins.x.clone(), bounds.maxs.x.clone()],
                [bounds.mins.y.clone(), bounds.maxs.y.clone()],
                [bounds.mins.z.clone(), bounds.maxs.z.clone()],
            ],
            first_position_identity,
            first_coordinate_identity,
        ));
        let first_plane_id = reserve_plane_ids(6);
        let polygons = (0..6)
            .map(|index| {
                let first_corner = index * 4;
                Polygon::from_lazy_indexed_quad(
                    Arc::clone(&vertex_pool),
                    index,
                    [
                        first_corner,
                        first_corner + 1,
                        first_corner + 2,
                        first_corner + 3,
                    ],
                    self.polygons[index].metadata.clone(),
                    first_plane_id
                        + u64::try_from(index).expect("cuboid polygon index fits u64"),
                )
            })
            .collect();
        let translated = Mesh::from_polygons_with_topology(polygons, self.polygons.topology());
        translated
            .polygons
            .retain_cuboid_vertex_pool(Arc::clone(&vertex_pool));
        let translated = self.finish_indexed_translation(
            translated,
            translated_bounds,
            vector,
            source_geometry_identity,
            cache_on_completion,
            self.has_convex_pwn_fact(),
        );
        Some(translated)
    }

    fn finish_indexed_scale(
        &self,
        mut scaled: Self,
        scales: [Real; 3],
        source_geometry_identity: u64,
        cache_on_completion: bool,
        convex_pwn: bool,
    ) -> Self {
        if let Some(bounds) = self.bounding_box.get() {
            let scaled_mins = [
                real_mul_exact_fast(&bounds.mins.x, &scales[0]),
                real_mul_exact_fast(&bounds.mins.y, &scales[1]),
                real_mul_exact_fast(&bounds.mins.z, &scales[2]),
            ];
            let scaled_maxs = [
                real_mul_exact_fast(&bounds.maxs.x, &scales[0]),
                real_mul_exact_fast(&bounds.maxs.y, &scales[1]),
                real_mul_exact_fast(&bounds.maxs.z, &scales[2]),
            ];
            scaled.bounding_box = OnceLock::from(Aabb::new(
                Point3::new(
                    real_min(&scaled_mins[0], &scaled_maxs[0]),
                    real_min(&scaled_mins[1], &scaled_maxs[1]),
                    real_min(&scaled_mins[2], &scaled_maxs[2]),
                ),
                Point3::new(
                    real_max(&scaled_mins[0], &scaled_maxs[0]),
                    real_max(&scaled_mins[1], &scaled_maxs[1]),
                    real_max(&scaled_mins[2], &scaled_maxs[2]),
                ),
            ));
        }
        if cache_on_completion {
            LAST_SCALE.with_borrow_mut(|cached| {
                *cached = Some(CachedScale {
                    source_geometry_identity,
                    scales,
                    polygons: scaled
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                });
            });
        }
        if convex_pwn {
            scaled.cache_convex_pwn_fact();
        }
        scaled
    }

    fn finish_indexed_general_transform(
        &self,
        mut mesh: Self,
        transformed_bounds: Option<Aabb>,
        matrix: &Matrix4,
        source_geometry_identity: u64,
        cache_on_completion: bool,
    ) -> Self {
        mesh.bounding_box = transformed_bounds
            .clone()
            .map_or_else(OnceLock::new, OnceLock::from);
        if self.polygons.manifold_fact() == Some(true) {
            mesh.polygons.retain_manifold_fact(true);
        }
        if self.has_convex_pwn_fact() {
            mesh.cache_convex_pwn_fact();
        }
        if cache_on_completion {
            LAST_GENERAL_TRANSFORM.with_borrow_mut(|cached| {
                if let Some(index) = cached.iter().position(|cached| {
                    cached.source_geometry_identity == source_geometry_identity
                        && cached.matrix == *matrix
                }) {
                    cached.remove(index);
                }
                if cached.len() == TRANSFORM_CACHE_CAPACITY {
                    cached.remove(0);
                }
                cached.push(CachedGeneralTransform {
                    source_geometry_identity,
                    matrix: matrix.clone(),
                    polygons: mesh
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                    bounding_box: transformed_bounds,
                });
            });
        }
        mesh
    }

    fn with_cached_geometry_without_facts(&self, cached: &[Polygon<()>]) -> Self {
        debug_assert_eq!(self.polygons.len(), cached.len());
        Mesh::from_polygons(
            self.polygons
                .iter()
                .zip(cached)
                .map(|(source, cached)| cached.clone().with_metadata(source.metadata.clone()))
                .collect(),
        )
    }

    fn with_cached_geometry(&self, cached: &[Polygon<()>]) -> Self {
        let mesh = self.with_cached_geometry_without_facts(cached);
        if self.has_convex_pwn_fact() {
            mesh.cache_convex_pwn_fact();
        }
        mesh
    }

    fn rigid_transform_owned(self, matrix: &Matrix4) -> Self {
        let finite_matrix = matrix_f64_lossy(matrix);
        self.rigid_transform_owned_with_finite(matrix, finite_matrix)
    }

    fn rigid_transform_owned_with_finite(
        mut self,
        matrix: &Matrix4,
        finite_matrix: Option<[[f64; 4]; 4]>,
    ) -> Self {
        let convex_pwn = self.has_convex_pwn_fact();
        let source_geometry_identity = self.polygons.storage_identity();
        if let Some(cached_polygons) = LAST_RIGID_TRANSFORM.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.matrix == *matrix
                        && cached.source_geometry_identity == source_geometry_identity
                })
                .map(|cached| cached.polygons.clone())
        }) {
            for (polygon, cached) in self.polygons.iter_mut().zip(cached_polygons) {
                let metadata = polygon.metadata.clone();
                *polygon = cached.with_metadata(metadata);
            }
            self.bounding_box = OnceLock::new();
            if convex_pwn {
                self.cache_convex_pwn_fact();
            }
            return self;
        }
        let cache_on_completion = PENDING_RIGID_TRANSFORM.with_borrow_mut(|pending| {
            let repeated = pending.as_ref().is_some_and(|pending| {
                pending.matrix == *matrix
                    && pending.source_geometry_identity == source_geometry_identity
            });
            *pending = Some(PendingTransform {
                source_geometry_identity,
                matrix: matrix.clone(),
            });
            repeated
        });

        let prepared_matrix = matrix.prepare();
        if let Some(transform_layout) = self
            .polygons
            .retained_transform_layout()
            .filter(|layout| layout.normals_match_positions)
            .cloned()
            && let Some(source_pool) = transform_layout.indexed_triangle_pool.as_ref()
            && let Some(source_position_f64) = transform_layout.position_f64.as_ref()
            && let Some(finite_matrix) = finite_matrix.as_ref()
        {
            let indexed_position_f64 = source_position_f64
                .iter()
                .map(|&source| transform_point_f64_lossy(finite_matrix, source))
                .collect::<Option<Vec<_>>>()
                .map(Arc::new);
            if let Some(indexed_position_f64) = indexed_position_f64 {
                let position_ids = reserve_position_ids(indexed_position_f64.len());
                cache_shared_position_f64_range(
                    position_ids,
                    Arc::clone(&indexed_position_f64),
                );
                let coordinate_ids =
                    transform_layout.coordinate_counts.map(reserve_position_ids);
                let plane_ids = reserve_plane_ids(transform_layout.plane_count);
                let vertex_pool = Arc::new(LazySubdivisionVertexPool::new_affine_transform(
                    Arc::clone(source_pool),
                    matrix.clone(),
                    matrix.clone(),
                    false,
                    false,
                    Some(Arc::clone(&indexed_position_f64)),
                    self.polygons.len(),
                    position_ids,
                    coordinate_ids,
                ));
                if let Some(transformed) = self.retained_indexed_triangle_output_from_pool(
                    &transform_layout,
                    vertex_pool,
                    Some(indexed_position_f64),
                    plane_ids,
                ) {
                    if cache_on_completion {
                        LAST_RIGID_TRANSFORM.with_borrow_mut(|cached| {
                            *cached = Some(CachedRigidTransform {
                                source_geometry_identity,
                                matrix: matrix.clone(),
                                polygons: transformed
                                    .polygons
                                    .iter()
                                    .cloned()
                                    .map(|polygon| polygon.with_metadata(()))
                                    .collect(),
                            });
                        });
                    }
                    if convex_pwn {
                        transformed.cache_convex_pwn_fact();
                    }
                    return transformed;
                }
            }
        }
        if let Some(transform_layout) = self
            .polygons
            .retained_transform_layout()
            .filter(|layout| layout.normals_match_positions)
            .cloned()
        {
            let transformed_positions = transform_layout
                .position_representatives
                .iter()
                .enumerate()
                .map(|(slot, _)| {
                    let source =
                        transform_layout.position_representative(&self.polygons, slot);
                    let position = prepared_matrix
                        .transform_point3(&source.position)
                        .expect("rigid transforms preserve affine points");
                    let normal = prepared_matrix.transform_direction3(&source.normal);
                    let finite = finite_matrix.as_ref().and_then(|matrix| {
                        let source_finite = transform_layout
                            .position_f64
                            .as_ref()
                            .map(|positions| positions[slot])
                            .or_else(|| source.position_f64_lossy())?;
                        transform_point_f64_lossy(matrix, source_finite)
                    });
                    (position, normal, finite)
                })
                .collect::<Vec<_>>();
            let position_ids = reserve_position_ids(transformed_positions.len());
            cache_position_f64_range(
                position_ids,
                transformed_positions.len(),
                transformed_positions.iter().map(|(_, _, finite)| *finite),
            );
            let coordinate_ids = transform_layout.coordinate_counts.map(reserve_position_ids);
            let plane_ids = reserve_plane_ids(transform_layout.plane_count);
            let indexed_vertices = transformed_positions
                .iter()
                .enumerate()
                .map(|(position_slot, (position, normal, _))| {
                    let source = transform_layout
                        .position_representative(&self.polygons, position_slot);
                    Vertex {
                        position: position.clone(),
                        normal: normal.clone(),
                        position_id: position_ids
                            + u64::try_from(position_slot).expect("position slot fits u64"),
                        coordinate_ids: std::array::from_fn(|axis| {
                            coordinate_ids[axis]
                                + u64::try_from(position_slot)
                                    .expect("coordinate slot fits u64")
                        }),
                        ruled_line: source.ruled_line,
                        hull_candidate: source.hull_candidate,
                    }
                })
                .collect();
            let indexed_position_f64 = transformed_positions
                .iter()
                .map(|(_, _, finite)| *finite)
                .collect::<Option<Vec<_>>>()
                .map(Arc::new);
            if let Some(transformed) = self.retained_indexed_triangle_output(
                &transform_layout,
                indexed_vertices,
                indexed_position_f64,
                plane_ids,
            ) {
                if cache_on_completion {
                    LAST_RIGID_TRANSFORM.with_borrow_mut(|cached| {
                        *cached = Some(CachedRigidTransform {
                            source_geometry_identity,
                            matrix: matrix.clone(),
                            polygons: transformed
                                .polygons
                                .iter()
                                .cloned()
                                .map(|polygon| polygon.with_metadata(()))
                                .collect(),
                        });
                    });
                }
                if convex_pwn {
                    transformed.cache_convex_pwn_fact();
                }
                return transformed;
            }
            let mut vertices = Vec::with_capacity(self.vertex_count());
            let mut push_corner = |corner_index: usize, vertex: &Vertex| {
                let position_slot = transform_layout.corner_position_slots[corner_index];
                vertices.push(Vertex {
                    position: transformed_positions[position_slot].0.clone(),
                    normal: transformed_positions[position_slot].1.clone(),
                    position_id: position_ids
                        + u64::try_from(position_slot).expect("position slot fits u64"),
                    coordinate_ids: std::array::from_fn(|axis| {
                        coordinate_ids[axis]
                            + u64::try_from(
                                transform_layout.coordinate_slot(axis, corner_index),
                            )
                            .expect("coordinate slot fits u64")
                    }),
                    ruled_line: vertex.ruled_line,
                    hull_candidate: vertex.hull_candidate,
                });
            };
            if let Some(pool) = &transform_layout.indexed_triangle_pool {
                for (corner_index, &position_slot) in
                    transform_layout.corner_position_slots.iter().enumerate()
                {
                    push_corner(corner_index, pool.vertex(position_slot));
                }
            } else {
                let mut corner_index = 0;
                for polygon in &self.polygons {
                    for vertex in &polygon.vertices {
                        push_corner(corner_index, vertex);
                        corner_index += 1;
                    }
                }
            }
            let vertices = Arc::new(vertices);
            let mut polygons = Vec::with_capacity(self.polygons.len());
            let mut start = 0;
            for (polygon_index, polygon) in self.polygons.iter().enumerate() {
                let end = start
                    + if transform_layout.indexed_triangle_pool.is_some() {
                        transform_layout
                            .indexed_polygon_corner_counts
                            .as_ref()
                            .map_or(3, |counts| counts[polygon_index])
                    } else {
                        polygon.vertices.len()
                    };
                polygons.push(Polygon::from_shared_vertices(
                    Arc::clone(&vertices),
                    start..end,
                    polygon.metadata.clone(),
                    plane_ids
                        + u64::try_from(transform_layout.plane_slot(polygon_index))
                            .expect("plane slot fits u64"),
                ));
                start = end;
            }
            let topology = self.polygons.topology();
            let transformed = Self::from_polygons_with_topology(polygons, topology);
            let mut transformed_layout = (*transform_layout).clone();
            transformed_layout.indexed_triangle_pool = None;
            transformed_layout.indexed_polygon_corner_counts = None;
            transformed_layout.position_f64 = transformed_positions
                .iter()
                .map(|(_, _, finite)| *finite)
                .collect::<Option<Vec<_>>>()
                .map(Arc::new);
            transformed
                .polygons
                .retain_transform_layout(Arc::new(transformed_layout));
            if cache_on_completion {
                LAST_RIGID_TRANSFORM.with_borrow_mut(|cached| {
                    *cached = Some(CachedRigidTransform {
                        source_geometry_identity,
                        matrix: matrix.clone(),
                        polygons: transformed
                            .polygons
                            .iter()
                            .cloned()
                            .map(|polygon| polygon.with_metadata(()))
                            .collect(),
                    });
                });
            }
            if convex_pwn {
                transformed.cache_convex_pwn_fact();
            }
            return transformed;
        }
        let mut transformed_positions = HashMap::<u64, (Point3, u64, Option<[f64; 3]>)>::new();
        let mut transformed_normals = HashMap::<u64, Vec<(Vector3, Vector3)>>::new();
        let matrix_facts = prepared_matrix.structural_facts();
        let mut transformed_coordinates: [HashMap<[Option<u64>; 3], u64>; 3] =
            std::array::from_fn(|_| HashMap::new());
        let mut transformed_planes = HashMap::new();

        for polygon in &mut self.polygons {
            polygon.plane_id = *transformed_planes
                .entry(polygon.plane_id)
                .or_insert_with(fresh_plane_id);
            for vertex in &mut polygon.vertices {
                let source_position_id = vertex.position_id;
                let source_coordinate_ids = vertex.coordinate_ids;
                for (row, ids) in transformed_coordinates.iter_mut().enumerate() {
                    let key = std::array::from_fn(|column| {
                        (matrix_facts.entry_known_zero(row, column) != Some(true))
                            .then_some(source_coordinate_ids[column])
                    });
                    vertex.coordinate_ids[row] =
                        *ids.entry(key).or_insert_with(fresh_position_id);
                }
                if let Some((position, position_id, position_f64)) =
                    transformed_positions.get(&source_position_id)
                {
                    vertex.position = position.clone();
                    vertex.position_id = *position_id;
                    cache_position_f64(*position_id, *position_f64);
                } else {
                    let position_f64 = finite_matrix.as_ref().and_then(|matrix| {
                        transform_point_f64_lossy(matrix, vertex.position_f64_lossy()?)
                    });
                    let position = prepared_matrix
                        .transform_point3(&vertex.position)
                        .expect("rigid transforms preserve affine points");
                    let position_id = fresh_position_id();
                    transformed_positions.insert(
                        source_position_id,
                        (position.clone(), position_id, position_f64),
                    );
                    vertex.position = position;
                    vertex.position_id = position_id;
                    cache_position_f64(position_id, position_f64);
                }
                let cached_normal =
                    transformed_normals
                        .get(&source_position_id)
                        .and_then(|entries| {
                            entries
                                .iter()
                                .find(|(source, _)| source == &vertex.normal)
                                .map(|(_, transformed)| transformed.clone())
                        });
                if let Some(normal) = cached_normal {
                    vertex.normal = normal;
                } else {
                    let source_normal = vertex.normal.clone();
                    let transformed_normal =
                        prepared_matrix.transform_direction3(&source_normal);
                    transformed_normals
                        .entry(source_position_id)
                        .or_default()
                        .push((source_normal, transformed_normal.clone()));
                    vertex.normal = transformed_normal;
                }
            }
            if polygon.vertices.len() == 3 {
                polygon.defer_plane_from_vertices();
            } else {
                assert!(
                    polygon.plane.transform_affine_in_place(matrix),
                    "rigid transforms preserve affine plane points"
                );
            }
            polygon.invalidate_bounding_box();
        }
        self.bounding_box = OnceLock::new();
        if cache_on_completion {
            LAST_RIGID_TRANSFORM.with_borrow_mut(|cached| {
                *cached = Some(CachedRigidTransform {
                    source_geometry_identity,
                    matrix: matrix.clone(),
                    polygons: self
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                });
            });
        }
        if convex_pwn {
            self.cache_convex_pwn_fact();
        }
        self
    }

    fn translate_vector_owned(mut self, vector: Vector3) -> Self {
        let convex_pwn = self.has_convex_pwn_fact();
        let source_axis_aligned_box = self.polygons.axis_aligned_box_fact().cloned();
        let translated_bounds = self.bounding_box.get().map(|bounds| {
            Aabb::new(
                bounds.mins.clone() + vector.clone(),
                bounds.maxs.clone() + vector.clone(),
            )
        });
        let source_geometry_identity = self.polygons.storage_identity();
        if let Some((cached_polygons, cached_bounds, cached_box, cached_centering_offset)) =
            LAST_TRANSLATION.with_borrow(|cached| {
                cached
                    .iter()
                    .rev()
                    .find(|cached| {
                        cached.vector == vector
                            && cached.source_geometry_identity == source_geometry_identity
                    })
                    .map(|cached| {
                        (
                            cached.polygons.clone(),
                            cached.bounding_box.clone(),
                            cached.axis_aligned_box,
                            cached.centering_offset.clone(),
                        )
                    })
            })
        {
            for (polygon, cached) in self.polygons.iter_mut().zip(cached_polygons) {
                let metadata = polygon.metadata.clone();
                *polygon = cached.with_metadata(metadata);
            }
            self.bounding_box = cached_bounds.map_or_else(OnceLock::new, OnceLock::from);
            if cached_box && let Some(bounds) = self.bounding_box.get().cloned() {
                self.polygons.retain_axis_aligned_box_fact(bounds);
            }
            if let Some(offset) = cached_centering_offset {
                self.polygons.retain_centering_offset(offset);
            }
            if convex_pwn {
                self.cache_convex_pwn_fact();
            }
            return self;
        }
        let cache_on_completion = PENDING_TRANSLATION.with_borrow_mut(|pending| {
            let repeated = pending.iter().any(|pending| {
                pending.vector == vector
                    && pending.source_geometry_identity == source_geometry_identity
            });
            if !repeated {
                if pending.len() == TRANSFORM_CACHE_CAPACITY {
                    pending.remove(0);
                }
                pending.push(PendingTranslation {
                    source_geometry_identity,
                    vector: vector.clone(),
                });
            }
            repeated
        });

        let transform_layout = Arc::clone(self.polygons.transform_layout());
        let vector_f64 = vector.0[0]
            .to_f64_lossy()
            .zip(vector.0[1].to_f64_lossy())
            .zip(vector.0[2].to_f64_lossy())
            .map(|((x, y), z)| [x, y, z]);
        if let Some(source_pool) = transform_layout.indexed_triangle_pool.as_ref()
            && let Some(source_position_f64) = transform_layout.position_f64.as_ref()
            && let Some(translation) = vector_f64
        {
            let indexed_position_f64 = Arc::new(
                source_position_f64
                    .iter()
                    .map(|source| {
                        [
                            source[0] + translation[0],
                            source[1] + translation[1],
                            source[2] + translation[2],
                        ]
                    })
                    .collect::<Vec<_>>(),
            );
            let position_ids = reserve_position_ids(indexed_position_f64.len());
            cache_shared_position_f64_range(position_ids, Arc::clone(&indexed_position_f64));
            let coordinate_ids = transform_layout.coordinate_counts.map(reserve_position_ids);
            let plane_ids = reserve_plane_ids(transform_layout.plane_count);
            let vertex_pool = Arc::new(LazySubdivisionVertexPool::new_translated(
                Arc::clone(source_pool),
                vector.clone(),
                Some(Arc::clone(&indexed_position_f64)),
                self.polygons.len(),
                position_ids,
                coordinate_ids,
            ));
            if let Some(translated) = self.retained_indexed_triangle_output_from_pool(
                &transform_layout,
                vertex_pool,
                Some(indexed_position_f64),
                plane_ids,
            ) {
                return self.finish_indexed_translation(
                    translated,
                    translated_bounds,
                    vector,
                    source_geometry_identity,
                    cache_on_completion,
                    convex_pwn,
                );
            }
        }
        let transformed_positions = transform_layout
            .position_representatives
            .iter()
            .enumerate()
            .map(|(position_slot, _)| {
                let source =
                    transform_layout.position_representative(&self.polygons, position_slot);
                let position = source_axis_aligned_box
                    .as_ref()
                    .zip(translated_bounds.as_ref())
                    .map(|(source_bounds, translated_bounds)| {
                        let translated_coordinate = |axis: usize, coordinate: &Real| {
                            let (source_min, source_max, translated_min, translated_max) =
                                match axis {
                                    0 => (
                                        &source_bounds.mins.x,
                                        &source_bounds.maxs.x,
                                        &translated_bounds.mins.x,
                                        &translated_bounds.maxs.x,
                                    ),
                                    1 => (
                                        &source_bounds.mins.y,
                                        &source_bounds.maxs.y,
                                        &translated_bounds.mins.y,
                                        &translated_bounds.maxs.y,
                                    ),
                                    _ => (
                                        &source_bounds.mins.z,
                                        &source_bounds.maxs.z,
                                        &translated_bounds.mins.z,
                                        &translated_bounds.maxs.z,
                                    ),
                                };
                            if coordinate == source_min {
                                translated_min.clone()
                            } else if coordinate == source_max {
                                translated_max.clone()
                            } else {
                                real_add_exact_fast(coordinate, &vector.0[axis])
                            }
                        };
                        Point3::new(
                            translated_coordinate(0, &source.position.x),
                            translated_coordinate(1, &source.position.y),
                            translated_coordinate(2, &source.position.z),
                        )
                    })
                    .unwrap_or_else(|| {
                        Point3::new(
                            real_add_exact_fast(&source.position.x, &vector.0[0]),
                            real_add_exact_fast(&source.position.y, &vector.0[1]),
                            real_add_exact_fast(&source.position.z, &vector.0[2]),
                        )
                    });
                let finite = vector_f64.and_then(|translation| {
                    let source = transform_layout
                        .position_f64
                        .as_ref()
                        .map(|positions| positions[position_slot])
                        .or_else(|| source.position_f64_lossy())?;
                    Some([
                        source[0] + translation[0],
                        source[1] + translation[1],
                        source[2] + translation[2],
                    ])
                });
                (position, finite)
            })
            .collect::<Vec<_>>();
        let position_ids = reserve_position_ids(transformed_positions.len());
        let indexed_position_f64 = transformed_positions
            .iter()
            .map(|(_, finite)| *finite)
            .collect::<Option<Vec<_>>>()
            .map(Arc::new);
        if let Some(position_f64) = indexed_position_f64.as_ref() {
            cache_shared_position_f64_range(position_ids, Arc::clone(position_f64));
        } else {
            cache_position_f64_range(
                position_ids,
                transformed_positions.len(),
                transformed_positions.iter().map(|(_, finite)| *finite),
            );
        }
        let coordinate_ids = transform_layout.coordinate_counts.map(reserve_position_ids);
        let plane_ids = reserve_plane_ids(transform_layout.plane_count);
        if transform_layout.normals_match_positions
            && transform_layout
                .corner_coordinate_slots
                .iter()
                .all(Option::is_none)
            && self.polygons.topology().2
        {
            let indexed_vertices = transformed_positions
                .iter()
                .enumerate()
                .map(|(position_slot, (position, _))| {
                    let source = transform_layout
                        .position_representative(&self.polygons, position_slot);
                    Vertex {
                        position: position.clone(),
                        normal: source.normal.clone(),
                        position_id: position_ids
                            + u64::try_from(position_slot).expect("position slot fits u64"),
                        coordinate_ids: std::array::from_fn(|axis| {
                            coordinate_ids[axis]
                                + u64::try_from(position_slot)
                                    .expect("coordinate slot fits u64")
                        }),
                        ruled_line: source.ruled_line,
                        hull_candidate: source.hull_candidate,
                    }
                })
                .collect();
            if let Some(translated) = self.retained_indexed_triangle_output(
                &transform_layout,
                indexed_vertices,
                indexed_position_f64.clone(),
                plane_ids,
            ) {
                return self.finish_indexed_translation(
                    translated,
                    translated_bounds,
                    vector,
                    source_geometry_identity,
                    cache_on_completion,
                    convex_pwn,
                );
            }
        }
        let mut vertices = Vec::with_capacity(self.vertex_count());
        let mut push_corner = |corner_index: usize, vertex: &Vertex| {
            let position_slot = transform_layout.corner_position_slots[corner_index];
            vertices.push(Vertex {
                position: transformed_positions[position_slot].0.clone(),
                normal: vertex.normal.clone(),
                position_id: position_ids
                    + u64::try_from(position_slot).expect("position slot fits u64"),
                coordinate_ids: std::array::from_fn(|axis| {
                    coordinate_ids[axis]
                        + u64::try_from(transform_layout.coordinate_slot(axis, corner_index))
                            .expect("coordinate slot fits u64")
                }),
                ruled_line: vertex.ruled_line,
                hull_candidate: vertex.hull_candidate,
            });
        };
        if let Some(pool) = &transform_layout.indexed_triangle_pool {
            for (corner_index, &position_slot) in
                transform_layout.corner_position_slots.iter().enumerate()
            {
                push_corner(corner_index, pool.vertex(position_slot));
            }
        } else {
            let mut corner_index = 0;
            for polygon in &self.polygons {
                for vertex in &polygon.vertices {
                    push_corner(corner_index, vertex);
                    corner_index += 1;
                }
            }
        }
        let shared_vertices = Arc::new(vertices);
        let mut polygons = Vec::with_capacity(self.polygons.len());
        let mut start = 0;
        for (polygon_index, polygon) in self.polygons.iter().enumerate() {
            let end = start
                + if transform_layout.indexed_triangle_pool.is_some() {
                    transform_layout
                        .indexed_polygon_corner_counts
                        .as_ref()
                        .map_or(3, |counts| counts[polygon_index])
                } else {
                    polygon.vertices.len()
                };
            let plane_id = plane_ids
                + u64::try_from(transform_layout.plane_slot(polygon_index))
                    .expect("plane slot fits u64");
            polygons.push(Polygon::from_shared_vertices(
                Arc::clone(&shared_vertices),
                start..end,
                polygon.metadata.clone(),
                plane_id,
            ));
            start = end;
        }
        let mut translated = Mesh::from_polygons(polygons);
        let mut translated_layout = (*transform_layout).clone();
        translated_layout.indexed_triangle_pool = None;
        translated_layout.position_f64 = indexed_position_f64;
        translated
            .polygons
            .retain_transform_layout(Arc::new(translated_layout));
        translated.bounding_box = translated_bounds.map_or_else(OnceLock::new, OnceLock::from);
        if let Some(bounds) = translated.bounding_box.get().cloned()
            && self.polygons.axis_aligned_box_fact().is_some()
        {
            translated
                .polygons
                .retain_axis_aligned_box_fact(bounds.clone());
            let centering_offset = if self.polygons.centering_offset_fact() == Some(&vector) {
                Vector3::zero()
            } else {
                let half =
                    (Real::one() / Real::from(2_u8)).expect("nonzero exact denominator");
                Vector3::new([
                    -(bounds.mins.x + bounds.maxs.x) * half.clone(),
                    -(bounds.mins.y + bounds.maxs.y) * half.clone(),
                    -(bounds.mins.z + bounds.maxs.z) * half,
                ])
            };
            translated.polygons.retain_centering_offset(centering_offset);
        }
        if cache_on_completion {
            LAST_TRANSLATION.with_borrow_mut(|cached| {
                if let Some(index) = cached.iter().position(|cached| {
                    cached.source_geometry_identity == source_geometry_identity
                        && cached.vector == vector
                }) {
                    cached.remove(index);
                }
                if cached.len() == TRANSFORM_CACHE_CAPACITY {
                    cached.remove(0);
                }
                cached.push(CachedTranslation {
                    source_geometry_identity,
                    vector,
                    polygons: translated
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                    bounding_box: translated.bounding_box.get().cloned(),
                    axis_aligned_box: translated.polygons.axis_aligned_box_fact().is_some(),
                    centering_offset: translated.polygons.centering_offset_fact().cloned(),
                    cuboid_vertex_pool: translated.polygons.cuboid_vertex_pool().cloned(),
                });
            });
        }
        if convex_pwn {
            translated.cache_convex_pwn_fact();
        }
        translated
    }

    fn disjoint_translated_copies(
        &self,
        offsets: &[Vector3],
        disjoint_certified: bool,
    ) -> Option<Self> {
        let source_bounds = self.polygons.axis_aligned_box_fact()?;
        if !disjoint_certified {
            let bounds = offsets
                .iter()
                .map(|offset| {
                    Aabb::new(
                        source_bounds.mins.clone() + offset.clone(),
                        source_bounds.maxs.clone() + offset.clone(),
                    )
                })
                .collect::<Vec<_>>();
            if !bounds.iter().enumerate().all(|(left_index, left)| {
                bounds
                    .iter()
                    .skip(left_index + 1)
                    .all(|right| aabbs_decided_disjoint(left, right))
            }) {
                return None;
            }
        }

        let transform_layout = Arc::clone(self.polygons.transform_layout());
        let copies = offsets.len();
        let positions_per_copy = transform_layout.position_representatives.len();
        let first_position_id = reserve_position_ids(positions_per_copy.checked_mul(copies)?);
        let first_coordinate_ids = transform_layout.coordinate_counts.map(|count| {
            reserve_position_ids(
                count
                    .checked_mul(copies)
                    .expect("distribution coordinate identity count does not overflow"),
            )
        });
        let first_plane_id =
            reserve_plane_ids(transform_layout.plane_count.checked_mul(copies)?);
        let total_polygons = self.polygons.len().checked_mul(copies)?;
        let mut source_corners = Vec::with_capacity(self.vertex_count());
        let polygon_corner_counts = self
            .polygons
            .iter()
            .map(|polygon| {
                source_corners.extend(polygon.vertices.iter().cloned());
                polygon.vertices.len()
            })
            .collect::<Vec<_>>();
        let corner_coordinate_slots = std::array::from_fn(|axis| {
            (0..source_corners.len())
                .map(|corner| transform_layout.coordinate_slot(axis, corner))
                .collect()
        });
        let corners_per_copy = source_corners.len();
        let vertex_pool = Arc::new(LazySubdivisionVertexPool::new_translated_copies(
            source_corners,
            offsets.to_vec(),
            transform_layout.corner_position_slots.to_vec(),
            corner_coordinate_slots,
            positions_per_copy,
            transform_layout.coordinate_counts,
            total_polygons,
            first_position_id,
            first_coordinate_ids,
        ));
        let mut polygons = Vec::with_capacity(total_polygons);
        for copy_index in 0..copies {
            let mut corner_start = copy_index * corners_per_copy;
            for (polygon_index, &corner_count) in polygon_corner_counts.iter().enumerate() {
                let polygon_slot = polygons.len();
                let plane_id = first_plane_id
                    + u64::try_from(
                        copy_index * transform_layout.plane_count
                            + transform_layout.plane_slot(polygon_index),
                    )
                    .ok()?;
                let metadata = self.polygons[polygon_index].metadata.clone();
                polygons.push(match corner_count {
                    3 => Polygon::from_lazy_subdivision_triangle(
                        Arc::clone(&vertex_pool),
                        polygon_slot,
                        [corner_start, corner_start + 1, corner_start + 2],
                        metadata,
                        plane_id,
                    ),
                    4 => Polygon::from_lazy_indexed_quad(
                        Arc::clone(&vertex_pool),
                        polygon_slot,
                        [
                            corner_start,
                            corner_start + 1,
                            corner_start + 2,
                            corner_start + 3,
                        ],
                        metadata,
                        plane_id,
                    ),
                    _ => Polygon::from_lazy_indexed_polygon(
                        Arc::clone(&vertex_pool),
                        polygon_slot,
                        (corner_start..corner_start + corner_count).collect(),
                        metadata,
                        plane_id,
                    ),
                });
                corner_start += corner_count;
            }
        }
        let (source_facets, source_corners, source_triangles) = self.polygons.topology();
        let mesh = Self::from_polygons_with_topology(
            polygons,
            (
                source_facets.checked_mul(copies)?,
                source_corners.checked_mul(copies)?,
                source_triangles,
            ),
        );
        if self.polygons.manifold_fact() == Some(true) {
            mesh.polygons.retain_manifold_fact(true);
        }
        Some(mesh)
    }

    fn disjoint_arc_copies(
        &self,
        samples: &[(Real, Real)],
        radius: &Real,
        certified_minimum_center_distance_squared: &Real,
    ) -> Option<Self> {
        let transform_layout = Arc::clone(self.polygons.transform_layout());
        let positions_per_copy = transform_layout.position_representatives.len();
        let total_positions = positions_per_copy.checked_mul(samples.len())?;
        let mut source_radius_squared = Real::zero();
        for position_slot in 0..positions_per_copy {
            let position = &transform_layout
                .position_representative(&self.polygons, position_slot)
                .position;
            let radius_squared = position.to_vector().dot(&position.to_vector());
            if real_gt(&radius_squared, &source_radius_squared) {
                source_radius_squared = radius_squared;
            }
        }
        if !real_gt(
            certified_minimum_center_distance_squared,
            &(source_radius_squared * Real::from(4_u8)),
        ) {
            return None;
        }

        let first_vertex_identity = reserve_position_ids(total_positions.checked_mul(4)?);
        let first_plane_id =
            reserve_plane_ids(transform_layout.plane_count.checked_mul(samples.len())?);
        let total_polygons = self.polygons.len().checked_mul(samples.len())?;
        let mut source_corners = Vec::with_capacity(self.vertex_count());
        let polygon_corner_counts = self
            .polygons
            .iter()
            .map(|polygon| {
                source_corners.extend(polygon.vertices.iter().cloned());
                polygon.vertices.len()
            })
            .collect::<Vec<_>>();
        let corners_per_copy = source_corners.len();
        let vertex_pool = Arc::new(LazySubdivisionVertexPool::new_arc_copies(
            source_corners,
            samples.to_vec(),
            radius.clone(),
            transform_layout.corner_position_slots.to_vec(),
            positions_per_copy,
            total_polygons,
            first_vertex_identity,
        ));
        let mut polygons = Vec::with_capacity(total_polygons);
        for copy_index in 0..samples.len() {
            let mut corner_start = copy_index * corners_per_copy;
            for (polygon_index, &corner_count) in polygon_corner_counts.iter().enumerate() {
                let polygon_slot = polygons.len();
                let plane_id = first_plane_id
                    + u64::try_from(
                        copy_index * transform_layout.plane_count
                            + transform_layout.plane_slot(polygon_index),
                    )
                    .ok()?;
                let metadata = self.polygons[polygon_index].metadata.clone();
                polygons.push(match corner_count {
                    3 => Polygon::from_lazy_subdivision_triangle(
                        Arc::clone(&vertex_pool),
                        polygon_slot,
                        [corner_start, corner_start + 1, corner_start + 2],
                        metadata,
                        plane_id,
                    ),
                    4 => Polygon::from_lazy_indexed_quad(
                        Arc::clone(&vertex_pool),
                        polygon_slot,
                        [
                            corner_start,
                            corner_start + 1,
                            corner_start + 2,
                            corner_start + 3,
                        ],
                        metadata,
                        plane_id,
                    ),
                    _ => Polygon::from_lazy_indexed_polygon(
                        Arc::clone(&vertex_pool),
                        polygon_slot,
                        (corner_start..corner_start + corner_count).collect(),
                        metadata,
                        plane_id,
                    ),
                });
                corner_start += corner_count;
            }
        }
        let (source_facets, source_corners, source_triangles) = self.polygons.topology();
        let mesh = Self::from_polygons_with_topology(
            polygons,
            (
                source_facets.checked_mul(samples.len())?,
                source_corners.checked_mul(samples.len())?,
                source_triangles,
            ),
        );
        if self.polygons.manifold_fact() == Some(true) {
            mesh.polygons.retain_manifold_fact(true);
        }
        Some(mesh)
    }

    fn disjoint_rigid_copies(
        &self,
        matrices: &[Matrix4],
        certified_minimum_center_distance_squared: Option<Real>,
    ) -> Option<Self> {
        let transform_layout = Arc::clone(self.polygons.transform_layout());
        let positions_per_copy = transform_layout.position_representatives.len();
        let total_positions = positions_per_copy.checked_mul(matrices.len())?;
        let first_vertex_identity = reserve_position_ids(total_positions.checked_mul(4)?);
        let first_plane_id =
            reserve_plane_ids(transform_layout.plane_count.checked_mul(matrices.len())?);
        let mut source_radius_squared = Real::zero();
        for position_slot in 0..positions_per_copy {
            let position = &transform_layout
                .position_representative(&self.polygons, position_slot)
                .position;
            let radius_squared = position.to_vector().dot(&position.to_vector());
            if real_gt(&radius_squared, &source_radius_squared) {
                source_radius_squared = radius_squared;
            }
        }
        let disjoint_threshold = source_radius_squared * Real::from(4_u8);
        if let Some(minimum_distance_squared) = certified_minimum_center_distance_squared {
            if !real_gt(&minimum_distance_squared, &disjoint_threshold) {
                return None;
            }
        } else {
            let mut centers = Vec::with_capacity(matrices.len());
            for matrix in matrices {
                centers.push(matrix.transform_point3(&Point3::origin()).ok()?);
            }
            if !centers.iter().enumerate().all(|(left_index, left)| {
                centers.iter().skip(left_index + 1).all(|right| {
                    let difference = right - left;
                    real_gt(&difference.dot(&difference), &disjoint_threshold)
                })
            }) {
                return None;
            }
        }

        let total_corners = self.vertex_count().checked_mul(matrices.len())?;
        let total_polygons = self.polygons.len().checked_mul(matrices.len())?;
        let mut source_corners = Vec::with_capacity(self.vertex_count());
        let polygon_corner_counts = self
            .polygons
            .iter()
            .map(|polygon| {
                source_corners.extend(polygon.vertices.iter().cloned());
                polygon.vertices.len()
            })
            .collect::<Vec<_>>();
        debug_assert_eq!(source_corners.len(), total_corners / matrices.len());
        let corners_per_copy = source_corners.len();
        let vertex_pool = Arc::new(LazySubdivisionVertexPool::new_rigid_copies(
            source_corners,
            matrices.to_vec(),
            transform_layout.corner_position_slots.to_vec(),
            positions_per_copy,
            total_polygons,
            first_vertex_identity,
        ));
        let mut polygons = Vec::with_capacity(total_polygons);
        for copy_index in 0..matrices.len() {
            let mut corner_start = copy_index * corners_per_copy;
            for (polygon_index, &corner_count) in polygon_corner_counts.iter().enumerate() {
                let polygon_slot = polygons.len();
                let plane_id = first_plane_id
                    + u64::try_from(
                        copy_index * transform_layout.plane_count
                            + transform_layout.plane_slot(polygon_index),
                    )
                    .ok()?;
                let metadata = self.polygons[polygon_index].metadata.clone();
                polygons.push(match corner_count {
                    3 => Polygon::from_lazy_subdivision_triangle(
                        Arc::clone(&vertex_pool),
                        polygon_slot,
                        [corner_start, corner_start + 1, corner_start + 2],
                        metadata,
                        plane_id,
                    ),
                    4 => Polygon::from_lazy_indexed_quad(
                        Arc::clone(&vertex_pool),
                        polygon_slot,
                        [
                            corner_start,
                            corner_start + 1,
                            corner_start + 2,
                            corner_start + 3,
                        ],
                        metadata,
                        plane_id,
                    ),
                    _ => Polygon::from_lazy_indexed_polygon(
                        Arc::clone(&vertex_pool),
                        polygon_slot,
                        (corner_start..corner_start + corner_count).collect(),
                        metadata,
                        plane_id,
                    ),
                });
                corner_start += corner_count;
            }
        }
        let (source_facets, source_corners, source_triangles) = self.polygons.topology();
        let mesh = Self::from_polygons_with_topology(
            polygons,
            (
                source_facets.checked_mul(matrices.len())?,
                source_corners.checked_mul(matrices.len())?,
                source_triangles,
            ),
        );
        if self.polygons.manifold_fact() == Some(true) {
            mesh.polygons.retain_manifold_fact(true);
        }
        Some(mesh)
    }

    pub(crate) fn nonuniform_scale_owned(mut self, sx: Real, sy: Real, sz: Real) -> Self {
        let convex_pwn = self.has_convex_pwn_fact();
        let scales = [sx, sy, sz];
        let source_geometry_identity = self.polygons.storage_identity();
        if let Some(cached_polygons) = LAST_SCALE.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.scales == scales
                        && cached.source_geometry_identity == source_geometry_identity
                })
                .map(|cached| cached.polygons.clone())
        }) {
            for (polygon, cached) in self.polygons.iter_mut().zip(cached_polygons) {
                let metadata = polygon.metadata.clone();
                *polygon = cached.with_metadata(metadata);
            }
            self.bounding_box = OnceLock::new();
            if convex_pwn {
                self.cache_convex_pwn_fact();
            }
            return self;
        }
        let cache_on_completion = PENDING_SCALE.with_borrow_mut(|pending| {
            let repeated = pending.as_ref().is_some_and(|pending| {
                pending.scales == scales
                    && pending.source_geometry_identity == source_geometry_identity
            });
            *pending = Some(PendingScale {
                source_geometry_identity,
                scales: scales.clone(),
            });
            repeated
        });
        let inverse_scales = [
            scales[0]
                .clone()
                .inverse()
                .expect("shape scale is certified nonzero"),
            scales[1]
                .clone()
                .inverse()
                .expect("shape scale is certified nonzero"),
            scales[2]
                .clone()
                .inverse()
                .expect("shape scale is certified nonzero"),
        ];
        if let Some(transform_layout) = self.polygons.retained_transform_layout().cloned() {
            let scales_f64 = [
                scales[0].to_f64_lossy(),
                scales[1].to_f64_lossy(),
                scales[2].to_f64_lossy(),
            ];
            if let Some(source_pool) = transform_layout.indexed_triangle_pool.as_ref()
                && let Some(source_position_f64) = transform_layout.position_f64.as_ref()
                && let [Some(sx), Some(sy), Some(sz)] = scales_f64
            {
                let indexed_position_f64 = Arc::new(
                    source_position_f64
                        .iter()
                        .map(|source| [source[0] * sx, source[1] * sy, source[2] * sz])
                        .collect::<Vec<_>>(),
                );
                let position_ids = reserve_position_ids(indexed_position_f64.len());
                cache_shared_position_f64_range(
                    position_ids,
                    Arc::clone(&indexed_position_f64),
                );
                let coordinate_ids =
                    transform_layout.coordinate_counts.map(reserve_position_ids);
                let plane_ids = reserve_plane_ids(transform_layout.plane_count);
                let vertex_pool = Arc::new(LazySubdivisionVertexPool::new_scale_transform(
                    Arc::clone(source_pool),
                    scales.clone(),
                    inverse_scales.clone(),
                    Some(Arc::clone(&indexed_position_f64)),
                    self.polygons.len(),
                    position_ids,
                    coordinate_ids,
                ));
                if let Some(scaled) = self.retained_indexed_triangle_output_from_pool(
                    &transform_layout,
                    vertex_pool,
                    Some(indexed_position_f64),
                    plane_ids,
                ) {
                    return self.finish_indexed_scale(
                        scaled,
                        scales,
                        source_geometry_identity,
                        cache_on_completion,
                        convex_pwn,
                    );
                }
            }
            let transformed_positions = transform_layout
                .position_representatives
                .iter()
                .enumerate()
                .map(|(slot, _)| {
                    let source =
                        transform_layout.position_representative(&self.polygons, slot);
                    let finite =
                        transform_layout.position_f64.as_ref().and_then(|positions| {
                            let scale_f64 = [scales_f64[0]?, scales_f64[1]?, scales_f64[2]?];
                            Some([
                                positions[slot][0] * scale_f64[0],
                                positions[slot][1] * scale_f64[1],
                                positions[slot][2] * scale_f64[2],
                            ])
                        });
                    (
                        Point3::new(
                            real_mul_exact_fast(&source.position.x, &scales[0]),
                            real_mul_exact_fast(&source.position.y, &scales[1]),
                            real_mul_exact_fast(&source.position.z, &scales[2]),
                        ),
                        Vector3::new([
                            real_mul_exact_fast(&source.normal.0[0], &inverse_scales[0]),
                            real_mul_exact_fast(&source.normal.0[1], &inverse_scales[1]),
                            real_mul_exact_fast(&source.normal.0[2], &inverse_scales[2]),
                        ]),
                        finite,
                    )
                })
                .collect::<Vec<_>>();
            let position_ids = reserve_position_ids(transformed_positions.len());
            cache_position_f64_range(
                position_ids,
                transformed_positions.len(),
                transformed_positions.iter().map(|(_, _, finite)| *finite),
            );
            let coordinate_ids = transform_layout.coordinate_counts.map(reserve_position_ids);
            let plane_ids = reserve_plane_ids(transform_layout.plane_count);
            let indexed_vertices = transformed_positions
                .iter()
                .enumerate()
                .map(|(position_slot, (position, normal, _))| {
                    let source = transform_layout
                        .position_representative(&self.polygons, position_slot);
                    Vertex {
                        position: position.clone(),
                        normal: normal.clone(),
                        position_id: position_ids
                            + u64::try_from(position_slot).expect("position slot fits u64"),
                        coordinate_ids: std::array::from_fn(|axis| {
                            coordinate_ids[axis]
                                + u64::try_from(position_slot)
                                    .expect("coordinate slot fits u64")
                        }),
                        ruled_line: source.ruled_line,
                        hull_candidate: source.hull_candidate,
                    }
                })
                .collect();
            let indexed_position_f64 = transformed_positions
                .iter()
                .map(|(_, _, finite)| *finite)
                .collect::<Option<Vec<_>>>()
                .map(Arc::new);
            if let Some(scaled) = self.retained_indexed_triangle_output(
                &transform_layout,
                indexed_vertices,
                indexed_position_f64,
                plane_ids,
            ) {
                return self.finish_indexed_scale(
                    scaled,
                    scales,
                    source_geometry_identity,
                    cache_on_completion,
                    convex_pwn,
                );
            }
            let mut vertices = Vec::with_capacity(self.vertex_count());
            let mut push_corner = |corner_index: usize, vertex: &Vertex| {
                let position_slot = transform_layout.corner_position_slots[corner_index];
                vertices.push(Vertex {
                    position: transformed_positions[position_slot].0.clone(),
                    normal: transformed_positions[position_slot].1.clone(),
                    position_id: position_ids
                        + u64::try_from(position_slot).expect("position slot fits u64"),
                    coordinate_ids: std::array::from_fn(|axis| {
                        coordinate_ids[axis]
                            + u64::try_from(
                                transform_layout.coordinate_slot(axis, corner_index),
                            )
                            .expect("coordinate slot fits u64")
                    }),
                    ruled_line: vertex.ruled_line,
                    hull_candidate: vertex.hull_candidate,
                });
            };
            if let Some(pool) = &transform_layout.indexed_triangle_pool {
                for (corner_index, &position_slot) in
                    transform_layout.corner_position_slots.iter().enumerate()
                {
                    push_corner(corner_index, pool.vertex(position_slot));
                }
            } else {
                let mut corner_index = 0;
                for polygon in &self.polygons {
                    for vertex in &polygon.vertices {
                        push_corner(corner_index, vertex);
                        corner_index += 1;
                    }
                }
            }
            let vertices = Arc::new(vertices);
            let mut polygons = Vec::with_capacity(self.polygons.len());
            let mut start = 0;
            for (polygon_index, polygon) in self.polygons.iter().enumerate() {
                let end = start
                    + if transform_layout.indexed_triangle_pool.is_some() {
                        transform_layout
                            .indexed_polygon_corner_counts
                            .as_ref()
                            .map_or(3, |counts| counts[polygon_index])
                    } else {
                        polygon.vertices.len()
                    };
                polygons.push(Polygon::from_shared_vertices(
                    Arc::clone(&vertices),
                    start..end,
                    polygon.metadata.clone(),
                    plane_ids
                        + u64::try_from(transform_layout.plane_slot(polygon_index))
                            .expect("plane slot fits u64"),
                ));
                start = end;
            }
            let mut scaled = Mesh::from_polygons(polygons);
            if let Some(bounds) = self.bounding_box.get() {
                let scaled_mins = [
                    bounds.mins.x.clone() * scales[0].clone(),
                    bounds.mins.y.clone() * scales[1].clone(),
                    bounds.mins.z.clone() * scales[2].clone(),
                ];
                let scaled_maxs = [
                    bounds.maxs.x.clone() * scales[0].clone(),
                    bounds.maxs.y.clone() * scales[1].clone(),
                    bounds.maxs.z.clone() * scales[2].clone(),
                ];
                scaled.bounding_box = OnceLock::from(Aabb::new(
                    Point3::new(
                        real_min(&scaled_mins[0], &scaled_maxs[0]),
                        real_min(&scaled_mins[1], &scaled_maxs[1]),
                        real_min(&scaled_mins[2], &scaled_maxs[2]),
                    ),
                    Point3::new(
                        real_max(&scaled_mins[0], &scaled_maxs[0]),
                        real_max(&scaled_mins[1], &scaled_maxs[1]),
                        real_max(&scaled_mins[2], &scaled_maxs[2]),
                    ),
                ));
            }
            if cache_on_completion {
                LAST_SCALE.with_borrow_mut(|cached| {
                    *cached = Some(CachedScale {
                        source_geometry_identity,
                        scales,
                        polygons: scaled
                            .polygons
                            .iter()
                            .cloned()
                            .map(|polygon| polygon.with_metadata(()))
                            .collect(),
                    });
                });
            }
            if convex_pwn {
                scaled.cache_convex_pwn_fact();
            }
            return scaled;
        }
        let matrix = Matrix4::affine_nonuniform_scale(scales.clone());
        let mut transformed_positions = HashMap::<u64, (Point3, u64)>::new();
        let mut transformed_normals = HashMap::<u64, Vec<(Vector3, Vector3)>>::new();
        let mut transformed_coordinates: [HashMap<u64, u64>; 3] =
            std::array::from_fn(|_| HashMap::new());
        let mut transformed_planes = HashMap::new();

        for polygon in &mut self.polygons {
            polygon.plane_id = *transformed_planes
                .entry(polygon.plane_id)
                .or_insert_with(fresh_plane_id);
            for vertex in &mut polygon.vertices {
                for (axis, ids) in transformed_coordinates.iter_mut().enumerate() {
                    vertex.coordinate_ids[axis] = *ids
                        .entry(vertex.coordinate_ids[axis])
                        .or_insert_with(fresh_position_id);
                }
                let source_position_id = vertex.position_id;
                if let Some((position, position_id)) =
                    transformed_positions.get(&source_position_id)
                {
                    vertex.position = position.clone();
                    vertex.position_id = *position_id;
                } else {
                    let position = Point3::new(
                        vertex.position.x.clone() * scales[0].clone(),
                        vertex.position.y.clone() * scales[1].clone(),
                        vertex.position.z.clone() * scales[2].clone(),
                    );
                    let position_id = fresh_position_id();
                    transformed_positions
                        .insert(source_position_id, (position.clone(), position_id));
                    vertex.position = position;
                    vertex.position_id = position_id;
                }

                let cached_normal =
                    transformed_normals
                        .get(&source_position_id)
                        .and_then(|entries| {
                            entries
                                .iter()
                                .find(|(source, _)| source == &vertex.normal)
                                .map(|(_, transformed)| transformed.clone())
                        });
                if let Some(normal) = cached_normal {
                    vertex.normal = normal;
                } else {
                    let source_normal = vertex.normal.clone();
                    let scaled = Vector3::new([
                        source_normal.0[0].clone() * inverse_scales[0].clone(),
                        source_normal.0[1].clone() * inverse_scales[1].clone(),
                        source_normal.0[2].clone() * inverse_scales[2].clone(),
                    ]);
                    transformed_normals
                        .entry(source_position_id)
                        .or_default()
                        .push((source_normal, scaled.clone()));
                    vertex.normal = scaled;
                }
            }
            if polygon.vertices.len() == 3 {
                polygon.defer_plane_from_vertices();
            } else {
                assert!(
                    polygon.plane.transform_affine_in_place(&matrix),
                    "nonzero diagonal shape scales preserve affine plane points"
                );
            }
            polygon.invalidate_bounding_box();
        }
        self.bounding_box = OnceLock::new();
        if cache_on_completion {
            LAST_SCALE.with_borrow_mut(|cached| {
                *cached = Some(CachedScale {
                    source_geometry_identity,
                    scales,
                    polygons: self
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                });
            });
        }
        if convex_pwn {
            self.cache_convex_pwn_fact();
        }
        self
    }

    /// Consume and translate this mesh while reusing its polygon storage.
    pub fn into_translated(self, x: Real, y: Real, z: Real) -> Self {
        self.translate_vector_owned(Vector3::new([x, y, z]))
    }

    /// Consume and rigidly rotate this mesh while reusing its polygon storage.
    pub fn into_rotated(mut self, x_deg: Real, y_deg: Real, z_deg: Real) -> Self {
        let convex_pwn = self.has_convex_pwn_fact();
        let source_geometry_identity = self.polygons.storage_identity();
        let degrees = [x_deg, y_deg, z_deg];
        if let Some(cached_polygons) = LAST_ROTATION.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.degrees == degrees
                        && cached.source_geometry_identity == source_geometry_identity
                })
                .map(|cached| cached.polygons.clone())
        }) {
            for (polygon, cached) in self.polygons.iter_mut().zip(cached_polygons) {
                let metadata = polygon.metadata.clone();
                *polygon = cached.with_metadata(metadata);
            }
            self.bounding_box = OnceLock::new();
            if convex_pwn {
                self.cache_convex_pwn_fact();
            }
            return self;
        }
        let cache_on_completion = PENDING_ROTATION.with_borrow_mut(|pending| {
            let repeated = pending.as_ref().is_some_and(|pending| {
                pending.degrees == degrees
                    && pending.source_geometry_identity == source_geometry_identity
            });
            *pending = Some(PendingRotation {
                source_geometry_identity,
                degrees: degrees.clone(),
            });
            repeated
        });

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
        // Rz * Ry * Rx, expanded once. Avoiding two generic 4x4 matrix
        // products keeps the six retained trigonometric objects shallow and
        // removes zero/identity arithmetic from first-use rotations.
        let rotation = Matrix4::from_row_major([
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
        let finite_rotation = rotation_xyz_f64_lossy(&degrees);
        let rotated = self.rigid_transform_owned_with_finite(&rotation, finite_rotation);
        if cache_on_completion {
            LAST_ROTATION.with_borrow_mut(|cached| {
                *cached = Some(CachedRotation {
                    source_geometry_identity,
                    degrees,
                    polygons: rotated
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                });
            });
        }
        rotated
    }

    /// Return this mesh with replacement metadata on the mesh and every polygon.
    pub fn with_metadata<NewM: Clone + Send + Sync + Debug>(
        self,
        metadata: NewM,
    ) -> Mesh<NewM> {
        let topology = self.polygons.topology();
        let geometry_lineage = self.polygons.geometry_lineage_identity();
        let polygons = self
            .polygons
            .into_iter()
            .map(|polygon| polygon.with_metadata(metadata.clone()))
            .collect::<Vec<_>>();

        Mesh {
            polygons: MeshPolygons::new_with_geometry_lineage(
                polygons,
                topology,
                geometry_lineage,
            ),
            bounding_box: OnceLock::new(),
        }
    }

    /// Map metadata on the mesh and every polygon while preserving geometry.
    pub fn map_metadata<NewM: Clone + Send + Sync + Debug, F>(self, mut f: F) -> Mesh<NewM>
    where
        F: FnMut(M) -> NewM,
    {
        let topology = self.polygons.topology();
        let geometry_lineage = self.polygons.geometry_lineage_identity();
        let polygons = self
            .polygons
            .into_iter()
            .map(|polygon| polygon.map_metadata(&mut f))
            .collect::<Vec<_>>();
        Mesh {
            polygons: MeshPolygons::new_with_geometry_lineage(
                polygons,
                topology,
                geometry_lineage,
            ),
            bounding_box: OnceLock::new(),
        }
    }

    /// Helper to collect all vertices from the CSG.
    pub fn vertices(&self) -> Vec<Vertex> {
        self.vertex_iter().cloned().collect()
    }

    /// Borrow every polygon corner without allocating or cloning exact values.
    pub fn vertex_iter(&self) -> impl Iterator<Item = &Vertex> {
        self.polygons.iter().flat_map(Polygon::vertex_iter)
    }

    /// Count polygon corners without materializing or inspecting vertex values.
    pub fn vertex_count(&self) -> usize {
        self.topology_counts().1
    }

    /// Count fan-triangulated facets and polygon corners in one topology pass.
    pub fn topology_counts(&self) -> (usize, usize) {
        let (facets, vertices, _) = self.polygons.topology();
        (facets, vertices)
    }

    /// Triangulate each polygon in the Mesh returning a Mesh containing triangles
    pub fn triangulate(&self) -> Mesh<M> {
        if self.polygons.topology().2 {
            return self.clone();
        }
        let triangles = self
            .polygons
            .iter()
            .flat_map(|poly| {
                poly.triangulate().into_iter().map(move |triangle| {
                    Polygon::from_planar_vertices(triangle.to_vec(), poly.metadata.clone())
                        .with_plane_id(poly.plane_id)
                })
            })
            .collect::<Vec<_>>();

        Mesh::from_polygons(triangles)
    }

    /// Subdivide all polygons in this Mesh 'levels' times, returning a new Mesh.
    /// This results in a triangular mesh with more detail.
    pub fn subdivide_triangles(&self, levels: NonZeroU32) -> Mesh<M> {
        let source_storage_identity = self.polygons.storage_identity();
        if let Some((polygons, metadata_sources)) = LAST_SUBDIVISION.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.source_storage_identity == source_storage_identity
                        && cached.levels == levels
                })
                .map(|cached| (cached.polygons.clone(), cached.metadata_sources.clone()))
        }) {
            return Mesh::from_polygons(
                polygons
                    .into_iter()
                    .zip(metadata_sources)
                    .map(|(polygon, source_index)| {
                        polygon.with_metadata(self.polygons[source_index].metadata.clone())
                    })
                    .collect(),
            );
        }
        let cache_on_completion = PENDING_SUBDIVISION.with_borrow_mut(|pending| {
            let key = (source_storage_identity, levels);
            let repeated = pending.as_ref() == Some(&key);
            *pending = Some(key);
            repeated
        });
        if levels.get() == 1
            && self.polygons.topology().2
            && self
                .polygons
                .retained_transform_layout()
                .is_some_and(|layout| layout.normals_match_positions)
        {
            let transform_layout = self
                .polygons
                .retained_transform_layout()
                .expect("subdivision fast path requires a retained layout");
            let maximum_midpoints = self.polygons.len() * 3;
            let first_midpoint_identity = reserve_position_ids(
                maximum_midpoints
                    .checked_mul(4)
                    .expect("subdivision identity reservation does not overflow"),
            );
            let mut edge_midpoint_slots =
                HashMap::<(usize, usize), usize>::with_capacity(self.polygons.len() * 3 / 2);
            let source_position_count = transform_layout.position_representatives.len();
            let source_vertices = transform_layout
                .position_representatives
                .iter()
                .map(|[polygon_index, vertex_index]| {
                    self.polygons[*polygon_index].vertices[*vertex_index].clone()
                })
                .collect();
            let mut midpoint_edges = Vec::with_capacity(self.polygons.len() * 3 / 2);
            let mut triangles_and_sources = Vec::with_capacity(self.polygons.len() * 4);
            for (source_index, polygon) in self.polygons.iter().enumerate() {
                let source_corner = source_index * 3;
                let [a_slot, b_slot, c_slot] = [
                    transform_layout.corner_position_slots[source_corner],
                    transform_layout.corner_position_slots[source_corner + 1],
                    transform_layout.corner_position_slots[source_corner + 2],
                ];
                let mut midpoint = |left_slot: usize, right_slot: usize| {
                    let key = if left_slot < right_slot {
                        (left_slot, right_slot)
                    } else {
                        (right_slot, left_slot)
                    };
                    match edge_midpoint_slots.entry(key) {
                        std::collections::hash_map::Entry::Occupied(entry) => *entry.get(),
                        std::collections::hash_map::Entry::Vacant(entry) => {
                            let slot = source_position_count + midpoint_edges.len();
                            midpoint_edges.push([key.0, key.1]);
                            entry.insert(slot);
                            slot
                        },
                    }
                };
                let ab = midpoint(a_slot, b_slot);
                let bc = midpoint(b_slot, c_slot);
                let ca = midpoint(c_slot, a_slot);
                for indices in [
                    [a_slot, ab, ca],
                    [ab, b_slot, bc],
                    [ca, bc, c_slot],
                    [ab, bc, ca],
                ] {
                    triangles_and_sources.push((indices, source_index, polygon.plane_id));
                }
            }
            let output_position_count = source_position_count + midpoint_edges.len();
            let vertices = Arc::new(LazySubdivisionVertexPool::new(
                source_vertices,
                midpoint_edges,
                triangles_and_sources.len(),
                first_midpoint_identity,
            ));
            let mut metadata_sources = Vec::with_capacity(triangles_and_sources.len());
            let mut corner_position_slots =
                Vec::with_capacity(triangles_and_sources.len() * 3);
            let polygons = triangles_and_sources
                .into_iter()
                .enumerate()
                .map(|(triangle_slot, (indices, source_index, plane_id))| {
                    metadata_sources.push(source_index);
                    corner_position_slots.extend(indices);
                    Polygon::from_lazy_subdivision_triangle(
                        Arc::clone(&vertices),
                        triangle_slot,
                        indices,
                        self.polygons[source_index].metadata.clone(),
                        plane_id,
                    )
                })
                .collect();
            let mesh = Self::from_polygons_with_topology(
                polygons,
                (self.polygons.len() * 4, self.polygons.len() * 12, true),
            );
            mesh.retain_shared_position_transform_layout(
                corner_position_slots,
                output_position_count,
                None,
                Some(vertices),
                None,
                true,
            );
            if self.polygons.manifold_fact() == Some(true) {
                mesh.polygons.retain_manifold_fact(true);
            }
            if self.has_convex_pwn_fact() {
                mesh.cache_convex_pwn_fact();
            }
            if cache_on_completion {
                LAST_SUBDIVISION.with_borrow_mut(|cached| {
                    *cached = Some(CachedSubdivision {
                        source_storage_identity,
                        levels,
                        polygons: mesh
                            .polygons
                            .iter()
                            .cloned()
                            .map(|polygon| polygon.with_metadata(()))
                            .collect(),
                        metadata_sources,
                    });
                });
            }
            return mesh;
        }
        let mut metadata_sources = Vec::new();
        let new_polygons: Vec<Polygon<M>> = self
            .polygons
            .iter()
            .enumerate()
            .flat_map(|(source_index, poly)| {
                let sub_tris = poly.subdivide_triangles(levels);
                metadata_sources.extend(std::iter::repeat_n(source_index, sub_tris.len()));
                sub_tris.into_iter().map(move |tri| {
                    Polygon::from_planar_vertices(
                        vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                        poly.metadata.clone(),
                    )
                    .with_plane_id(poly.plane_id)
                })
            })
            .collect();

        let mesh = Mesh::from_polygons(new_polygons);
        if cache_on_completion {
            LAST_SUBDIVISION.with_borrow_mut(|cached| {
                *cached = Some(CachedSubdivision {
                    source_storage_identity,
                    levels,
                    polygons: mesh
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                    metadata_sources,
                });
            });
        }
        mesh
    }

    /// Subdivide all polygons in this Mesh 'levels' times, in place.
    /// This results in a triangular mesh with more detail.
    ///
    /// ## Example
    /// ```ignore
    /// use csgrs::mesh::Mesh;
    /// use core::num::NonZeroU32;
    /// let mut cube: Mesh<()> = Mesh::cube(2.0, ());
    /// // subdivide_triangles(1) => each polygon (quad) is triangulated => 2 triangles => each tri subdivides => 4
    /// // So each face with 4 vertices => 2 triangles => each becomes 4 => total 8 per face => 6 faces => 48
    /// cube.subdivide_triangles_mut(1.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 48);
    ///
    /// let mut cube: Mesh<()> = Mesh::cube(2.0, ());
    /// cube.subdivide_triangles_mut(2.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 192);
    /// ```
    pub fn subdivide_triangles_mut(&mut self, levels: NonZeroU32) {
        self.polygons = self
            .polygons
            .iter()
            .flat_map(|poly| {
                let polytri = poly.subdivide_triangles(levels);
                polytri.into_iter().map(move |tri| {
                    Polygon::from_planar_vertices(tri.to_vec(), poly.metadata.clone())
                        .with_plane_id(poly.plane_id)
                })
            })
            .collect();
    }

    /// Renormalize all polygons in this Mesh by re-computing each polygon’s plane
    /// and assigning that plane’s normal to all vertices.
    pub fn renormalize(&mut self) {
        if let Some(polygons) = self.polygons.renormalized().cloned() {
            self.polygons = polygons;
            return;
        }
        let source = self.polygons.clone();
        for poly in &mut self.polygons {
            poly.set_new_normal();
        }
        source.retain_renormalized(self.polygons.clone());
    }

    /// Sample every coordinate at an explicit finite application/output boundary.
    ///
    /// This is intended for callers whose source language or file format has
    /// finite-number semantics and which want to retry an operation that could
    /// not certify a symbolic predicate. Native mesh operations do not call it
    /// implicitly.
    pub fn materialize_finite_output(&self) -> Option<Self> {
        if let Some(polygons) = self.polygons.materialized_finite().cloned() {
            return Some(Mesh {
                polygons,
                bounding_box: OnceLock::new(),
            });
        }
        let mesh = if let Some(transform_layout) =
            self.polygons.retained_transform_layout().cloned()
        {
            let converted_positions = transform_layout
                .position_representatives
                .iter()
                .enumerate()
                .map(|(position_slot, _)| {
                    let source = transform_layout
                        .position_representative(&self.polygons, position_slot);
                    let finite = [
                        source.position.x.to_f64_lossy()?,
                        source.position.y.to_f64_lossy()?,
                        source.position.z.to_f64_lossy()?,
                    ];
                    Some((
                        Point3::new(
                            Real::try_from(finite[0]).ok()?,
                            Real::try_from(finite[1]).ok()?,
                            Real::try_from(finite[2]).ok()?,
                        ),
                        finite,
                    ))
                })
                .collect::<Option<Vec<_>>>()?;
            let position_ids = reserve_position_ids(converted_positions.len());
            cache_position_f64_range(
                position_ids,
                converted_positions.len(),
                converted_positions.iter().map(|(_, finite)| Some(*finite)),
            );
            let coordinate_ids = transform_layout.coordinate_counts.map(reserve_position_ids);
            let plane_ids = reserve_plane_ids(transform_layout.plane_count);
            let mut converted_normals =
                vec![Vec::<(Vector3, Vector3)>::new(); converted_positions.len()];
            let mut vertices = Vec::with_capacity(self.vertex_count());
            let mut ranges = Vec::with_capacity(self.polygons.len());
            let mut corner_index = 0;
            for polygon in &self.polygons {
                let start = vertices.len();
                for source in &polygon.vertices {
                    let position_slot = transform_layout.corner_position_slots[corner_index];
                    let normal = if let Some((_, converted)) = converted_normals[position_slot]
                        .iter()
                        .find(|(candidate, _)| candidate == &source.normal)
                    {
                        converted.clone()
                    } else {
                        let converted = Vector3::new([
                            Real::try_from(source.normal.0[0].to_f64_lossy()?).ok()?,
                            Real::try_from(source.normal.0[1].to_f64_lossy()?).ok()?,
                            Real::try_from(source.normal.0[2].to_f64_lossy()?).ok()?,
                        ]);
                        converted_normals[position_slot]
                            .push((source.normal.clone(), converted.clone()));
                        converted
                    };
                    vertices.push(Vertex {
                        position: converted_positions[position_slot].0.clone(),
                        normal,
                        position_id: position_ids + u64::try_from(position_slot).ok()?,
                        coordinate_ids: std::array::from_fn(|axis| {
                            coordinate_ids[axis]
                                + u64::try_from(
                                    transform_layout.coordinate_slot(axis, corner_index),
                                )
                                .expect("coordinate slot fits u64")
                        }),
                        ruled_line: None,
                        hull_candidate: true,
                    });
                    corner_index += 1;
                }
                ranges.push(start..vertices.len());
            }
            let vertices = Arc::new(vertices);
            let polygons = ranges
                .into_iter()
                .enumerate()
                .map(|(polygon_index, range)| {
                    Polygon::from_shared_vertices(
                        Arc::clone(&vertices),
                        range,
                        self.polygons[polygon_index].metadata.clone(),
                        plane_ids
                            + u64::try_from(transform_layout.plane_slot(polygon_index))
                                .expect("plane slot fits u64"),
                    )
                })
                .collect();
            let mesh = Self::from_polygons(polygons);
            let mut output_layout = (*transform_layout).clone();
            output_layout.indexed_triangle_pool = None;
            output_layout.indexed_polygon_corner_counts = None;
            output_layout.position_f64 = Some(Arc::new(
                converted_positions
                    .iter()
                    .map(|(_, finite)| *finite)
                    .collect(),
            ));
            mesh.polygons.retain_transform_layout(Arc::new(output_layout));
            mesh
        } else {
            let polygons = self
                .polygons
                .iter()
                .map(|polygon| {
                    let vertices = polygon
                        .vertices
                        .iter()
                        .map(|vertex| {
                            Some(Vertex::new(
                                Point3::new(
                                    Real::try_from(vertex.position.x.to_f64_lossy()?).ok()?,
                                    Real::try_from(vertex.position.y.to_f64_lossy()?).ok()?,
                                    Real::try_from(vertex.position.z.to_f64_lossy()?).ok()?,
                                ),
                                Vector3::new([
                                    Real::try_from(vertex.normal.0[0].to_f64_lossy()?).ok()?,
                                    Real::try_from(vertex.normal.0[1].to_f64_lossy()?).ok()?,
                                    Real::try_from(vertex.normal.0[2].to_f64_lossy()?).ok()?,
                                ]),
                            ))
                        })
                        .collect::<Option<Vec<_>>>()?;
                    Some(Polygon::from_planar_vertices(
                        vertices,
                        polygon.metadata.clone(),
                    ))
                })
                .collect::<Option<Vec<_>>>()?;
            Self::from_polygons(polygons)
        };
        self.polygons
            .retain_materialized_finite(mesh.polygons.clone());
        Some(mesh)
    }

    /// **Mathematical Foundation: Dihedral Angle Calculation**
    ///
    /// Computes the dihedral angle between two triangles sharing an edge.
    /// The angle is computed as the angle between their normal vectors.
    /// Normalization, dot product, clamping, and arccos are delegated to
    /// `hyperlattice`/`hyperreal`, keeping the mesh query on the same exact-
    /// geometric-computation boundary as other normal-angle predicates. See
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    ///
    /// Returns the angle in radians.
    pub fn dihedral_angle(p1: &Triangle<M>, p2: &Triangle<M>) -> Real {
        if let Some(angle) = LAST_DIHEDRAL_ANGLE.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|(left, right, _)| {
                    (*left == p1.plane_id && *right == p2.plane_id)
                        || (*left == p2.plane_id && *right == p1.plane_id)
                })
                .map(|(_, _, angle)| angle.clone())
        }) {
            return angle;
        }
        let n1 = p1.plane.normal();
        let n2 = p2.plane.normal();
        let angle = n1.angle_to(&n2).unwrap_or_else(|_| Real::zero());
        LAST_DIHEDRAL_ANGLE.with_borrow_mut(|cached| {
            *cached = Some((p1.plane_id, p2.plane_id, angle.clone()));
        });
        angle
    }

    /// Converts this mesh into exact vertex/index buffers suitable for rendering adapters.
    pub fn build_graphics_mesh(&self) -> GraphicsMesh {
        self.try_build_graphics_mesh()
            .unwrap_or_else(|error| panic!("graphics mesh construction failed: {error}"))
    }

    fn try_build_graphics_mesh(&self) -> Result<GraphicsMesh, ::hypermesh::GpuMeshError> {
        if let Some(graphics) = self.polygons.graphics_mesh() {
            return Ok(graphics.clone());
        }
        if let Some(layout) = self.polygons.retained_transform_layout()
            && layout.normals_match_positions
            && layout.indexed_triangle_pool.is_some()
            && self.polygons.topology().2
        {
            let vertex_count = layout.corner_position_slots.len();
            let mut vertices = Vec::with_capacity(vertex_count);
            for &position_slot in layout.corner_position_slots.iter() {
                let vertex = layout.position_representative(&self.polygons, position_slot);
                vertices.push((
                    [
                        vertex.position.x.clone(),
                        vertex.position.y.clone(),
                        vertex.position.z.clone(),
                    ],
                    [
                        vertex.normal.0[0].clone(),
                        vertex.normal.0[1].clone(),
                        vertex.normal.0[2].clone(),
                    ],
                ));
            }
            let indices = (0..vertex_count)
                .map(|index| {
                    u32::try_from(index)
                        .map_err(|_| ::hypermesh::GpuMeshError::VertexCountExceededU32)
                })
                .collect::<Result<Vec<_>, _>>()?;
            let graphics = GraphicsMesh {
                vertices: vertices.into(),
                indices: indices.into(),
            };
            self.polygons.retain_graphics_mesh(graphics.clone());
            return Ok(graphics);
        }
        let triangle_capacity = self
            .polygons
            .iter()
            .map(|polygon| polygon.vertices.len().saturating_sub(2))
            .sum::<usize>();
        let triangles = self.polygons.iter().flat_map(|polygon| {
            polygon.triangulate_indices().into_iter().map(|triangle| {
                triangle.map(|vertex_index| {
                    let vertex = &polygon.vertices[vertex_index];
                    let position = [
                        vertex.position.x.clone(),
                        vertex.position.y.clone(),
                        vertex.position.z.clone(),
                    ];
                    let normal = [
                        vertex.normal.0[0].clone(),
                        vertex.normal.0[1].clone(),
                        vertex.normal.0[2].clone(),
                    ];
                    (position, normal)
                })
            })
        });
        let exact = ::hypermesh::ExactGpuMeshBuffers::from_triangles_with_capacity(
            triangle_capacity,
            triangles,
        )?;

        let graphics = GraphicsMesh {
            vertices: exact.vertices.into(),
            indices: exact.indices.into(),
        };
        self.polygons.retain_graphics_mesh(graphics.clone());
        Ok(graphics)
    }

    /// Try to extract hyperreal vertices and triangle indices.
    ///
    /// This is the native mesh-buffer view. It does not cross a primitive-float
    /// boundary; coordinates stay in [`Real`] and only the
    /// topological index carrier is lowered to `u32`.
    pub fn try_get_vertices_and_indices(
        &self,
    ) -> Result<(Vec<Point3>, Vec<[u32; 3]>), ValidationError> {
        let triangles = self
            .polygons
            .iter()
            .flat_map(|polygon| polygon.triangulate())
            .collect::<Vec<_>>();
        let mut vertices = Vec::with_capacity(triangles.len() * 3);
        let mut indices = Vec::with_capacity(triangles.len());

        for triangle in triangles {
            let base = u32::try_from(vertices.len()).map_err(|_| {
                ValidationError::MeshBufferError("mesh vertex count exceeded u32".into())
            })?;
            vertices.push(triangle[0].position.clone());
            vertices.push(triangle[1].position.clone());
            vertices.push(triangle[2].position.clone());
            indices.push([base, base + 1, base + 2]);
        }

        Ok((vertices, indices))
    }

    /// Approximates this mesh as strict finite-`f32` GPU buffers.
    pub fn try_to_gpu_mesh_f32(
        &self,
    ) -> Result<::hypermesh::GpuMeshBuffersF32, ::hypermesh::GpuMeshError> {
        let graphics = self.try_build_graphics_mesh()?;
        ::hypermesh::approximate_gpu_mesh_f32(&graphics.vertices, &graphics.indices)
    }

    /// Approximates this mesh as GPU buffers, substituting zero for an
    /// unrepresentable position row or normal component.
    pub fn to_gpu_mesh_f32_or_zero(
        &self,
    ) -> Result<::hypermesh::GpuMeshBuffersF32, ::hypermesh::GpuMeshError> {
        let graphics = self.try_build_graphics_mesh()?;
        ::hypermesh::approximate_gpu_mesh_f32_or_zero(&graphics.vertices, &graphics.indices)
    }

    /// Approximates this mesh as strict finite-`f64` GPU buffers.
    pub fn try_to_gpu_mesh_f64(
        &self,
    ) -> Result<::hypermesh::GpuMeshBuffersF64, ::hypermesh::GpuMeshError> {
        let graphics = self.try_build_graphics_mesh()?;
        ::hypermesh::approximate_gpu_mesh_f64(&graphics.vertices, &graphics.indices)
    }

    /// Approximates this mesh as binary64 GPU buffers, substituting zero for an
    /// unrepresentable position row or normal component.
    pub fn to_gpu_mesh_f64_or_zero(
        &self,
    ) -> Result<::hypermesh::GpuMeshBuffersF64, ::hypermesh::GpuMeshError> {
        let graphics = self.try_build_graphics_mesh()?;
        ::hypermesh::approximate_gpu_mesh_f64_or_zero(&graphics.vertices, &graphics.indices)
    }

    /// Casts a ray defined by `origin` + t * `direction` against all triangles
    /// of this Mesh and returns a list of (intersection_point, distance),
    /// sorted by ascending distance.
    ///
    /// # Parameters
    /// - `origin`: The ray’s start point.
    /// - `direction`: The ray’s direction vector.
    ///
    /// # Returns
    /// A `Vec` of `(Point3, Real)` where:
    /// - `Point3` is the intersection coordinate in 3D,
    /// - `Real` is the distance (the ray parameter t) from `origin`.
    ///
    /// Triangle intersections are evaluated with hyperreal vector operations,
    /// keeping hit ordering and deduplication in the same scalar domain as the
    /// mesh coordinates.
    pub fn ray_intersections(
        &self,
        origin: &Point3,
        direction: &Vector3,
    ) -> Vec<(Point3, Real)> {
        if let Some(hits) = LAST_RAY_INTERSECTIONS.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.origin == *origin
                        && cached.direction == *direction
                        && self.spatial_identity_matches(&cached.spatial_identity)
                })
                .map(|cached| cached.hits.clone())
        }) {
            return hits;
        }

        let certified_query = point_exact_f64(origin).zip(vector_exact_f64(direction));
        let mut candidates = Vec::new();
        for (index, polygon) in self.polygons.iter().enumerate() {
            let misses_bounds = match (certified_query, polygon.certified_f64_bounds_ref()) {
                (Some((origin, direction)), Some(bounds)) => {
                    ray_decided_miss_certified_f64_bounds(origin, direction, bounds, None)
                },
                _ => {
                    ray_decided_miss_aabb(origin, direction, polygon.bounding_box_ref(), None)
                },
            };
            if !misses_bounds {
                candidates.push(index);
            }
        }
        let mut hits = Vec::new();
        for &candidate in &candidates {
            let polygon = &self.polygons[candidate];
            if polygon.vertices.len() == 3 {
                let prepared = polygon.prepared_triangle_query_ref();
                let positions = [
                    &polygon.vertices[0].position,
                    &polygon.vertices[1].position,
                    &polygon.vertices[2].position,
                ];
                if let Some(hit) =
                    ray_triangle_positions_prepared(origin, direction, positions, prepared)
                {
                    hits.push(hit);
                }
                continue;
            }
            for [a, b, c] in polygon.triangulate_indices() {
                if let Some(hit) = ray_triangle_positions(
                    origin,
                    direction,
                    [
                        &polygon.vertices[a].position,
                        &polygon.vertices[b].position,
                        &polygon.vertices[c].position,
                    ],
                ) {
                    hits.push(hit);
                }
            }
        }
        // Sort and deduplicate only when Hyper proves exact identity.
        canonicalize_ray_hits(&mut hits);

        LAST_RAY_INTERSECTIONS.with_borrow_mut(|cached| {
            *cached = Some(CachedRayIntersections {
                spatial_identity: self.spatial_identity(),
                origin: origin.clone(),
                direction: direction.clone(),
                hits: hits.clone(),
            });
        });
        hits
    }

    /// Find all intersection points between a polyline and this mesh's
    /// triangulated surface.
    ///
    /// Each consecutive pair of points defines one segment. Hits are deduplicated
    /// locally and returned in polyline order. Deduplication requires exact
    /// hit-point equality through `hyperlattice::Vector3` and `Real`,
    /// keeping this topology-affecting equality decision out of local f64
    /// tolerance arithmetic. This follows Yap's
    /// exact-geometric-computation boundary discipline
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn intersect_polyline(&self, polyline: &[Point3]) -> Vec<Point3> {
        if polyline.len() < 2 {
            return Vec::new();
        }
        if let Some(hits) = LAST_POLYLINE_INTERSECTIONS.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.polyline == polyline
                        && self.spatial_identity_matches(&cached.spatial_identity)
                })
                .map(|cached| cached.hits.clone())
        }) {
            return hits;
        }

        let mut hits: Vec<Point3> = Vec::new();

        for seg in polyline.windows(2) {
            let seg_start = seg[0].clone();
            let seg_end = seg[1].clone();
            let seg_dir = &seg_end - &seg_start;
            if real_zero(&seg_dir.dot(&seg_dir)) {
                continue;
            }

            if let Some(mut seg_hits) = LAST_RAY_INTERSECTIONS.with_borrow(|cached| {
                let cached = cached.as_ref()?;
                if cached.origin != seg_start
                    || !self.spatial_identity_matches(&cached.spatial_identity)
                {
                    return None;
                }
                let axis = cached
                    .direction
                    .0
                    .iter()
                    .position(|component| !real_zero(component))?;
                let scale =
                    (seg_dir.0[axis].clone() / cached.direction.0[axis].clone()).ok()?;
                if !matches!(real_sign(&scale), Some(RealSign::Positive))
                    || (0..3).any(|component| {
                        !real_zero(
                            &(seg_dir.0[component].clone()
                                - cached.direction.0[component].clone() * scale.clone()),
                        )
                    })
                {
                    return None;
                }
                Some(
                    cached
                        .hits
                        .iter()
                        .filter_map(|(point, parameter)| {
                            let parameter = (parameter.clone() / scale.clone()).ok()?;
                            (!real_lt(&parameter, &Real::zero())
                                && !real_gt(&parameter, &Real::one()))
                            .then(|| (point.clone(), parameter))
                        })
                        .collect::<Vec<_>>(),
                )
            }) {
                seg_hits.sort_by(|a, b| real_cmp(&a.1, &b.1));
                for (point, _) in seg_hits {
                    if let Some(last) = hits.last() {
                        let point_h = hyperlimit_point3(&point);
                        let last_h = hyperlimit_point3(last);
                        if matches!(
                            hyperlimit::point3_equal(&point_h, &last_h).value(),
                            Some(true)
                        ) {
                            continue;
                        }
                    }
                    hits.push(point);
                }
                continue;
            }
            let certified_query = point_exact_f64(&seg_start).zip(vector_exact_f64(&seg_dir));

            let mut seg_hits: Vec<(Point3, Real)> = Vec::new();

            for polygon in &self.polygons {
                let misses_bounds = match (certified_query, polygon.certified_f64_bounds_ref())
                {
                    (Some((origin, direction)), Some(bounds)) => {
                        ray_decided_miss_certified_f64_bounds(
                            origin,
                            direction,
                            bounds,
                            Some(1.0),
                        )
                    },
                    _ => ray_decided_miss_aabb(
                        &seg_start,
                        &seg_dir,
                        polygon.bounding_box_ref(),
                        Some(&Real::one()),
                    ),
                };
                if misses_bounds {
                    continue;
                }
                for [a, b, c] in polygon.triangulate_indices() {
                    let prepared = polygon.prepared_triangle_query_ref();
                    let positions = [
                        &polygon.vertices[a].position,
                        &polygon.vertices[b].position,
                        &polygon.vertices[c].position,
                    ];
                    if let Some((point, t)) = ray_triangle_positions_prepared(
                        &seg_start, &seg_dir, positions, prepared,
                    ) && !real_lt(&t, &Real::zero())
                        && !real_gt(&t, &Real::one())
                    {
                        seg_hits.push((point, t));
                    }
                }
            }

            seg_hits.sort_by(|a, b| real_cmp(&a.1, &b.1));

            for (point, _) in seg_hits {
                if let Some(last) = hits.last() {
                    let point_h = hyperlimit::Point3::new(
                        point.x.clone(),
                        point.y.clone(),
                        point.z.clone(),
                    );
                    let last_h = hyperlimit::Point3::new(
                        last.x.clone(),
                        last.y.clone(),
                        last.z.clone(),
                    );
                    if matches!(
                        hyperlimit::point3_equal(&point_h, &last_h).value(),
                        Some(true)
                    ) {
                        continue;
                    }
                }
                hits.push(point);
            }
        }

        LAST_POLYLINE_INTERSECTIONS.with_borrow_mut(|cached| {
            *cached = Some(CachedPolylineIntersections {
                spatial_identity: self.spatial_identity(),
                polyline: polyline.to_vec(),
                hits: hits.clone(),
            });
        });
        hits
    }

    /// Uses hyperreal triangle ray intersections to check if a point is inside a `Mesh`.
    /// Note: this only use the 3d geometry of `CSG`
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices
    ///
    /// ## Example
    /// ```ignore
    /// # use csgrs::mesh::Mesh;
    /// # use hyperlattice::Point3;
    /// # use hyperlattice::Vector3;
    /// let csg_cube = Mesh::<()>::cube(6.0, ());
    ///
    /// assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 3.0)));
    /// assert!(csg_cube.contains_vertex(&Point3::new(1.0, 2.0, 5.9)));
    ///
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0)));
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, -6.0)));
    /// ```
    pub fn contains_vertex(&self, point: &Point3) -> bool {
        if self.polygons.is_empty() {
            return false;
        }

        let bounds = self.bounding_box();
        if real_lt(&point.x, &bounds.mins.x)
            || real_gt(&point.x, &bounds.maxs.x)
            || real_lt(&point.y, &bounds.mins.y)
            || real_gt(&point.y, &bounds.maxs.y)
            || real_lt(&point.z, &bounds.mins.z)
            || real_gt(&point.z, &bounds.maxs.z)
        {
            return false;
        }

        let one = Real::one();
        let eight = Real::from(8_u8);
        let sixteen = Real::from(16_u8);
        let direction = Vector3::from_xyz(
            one.clone(),
            (Real::from(3_u8) / eight).unwrap_or_else(|_| one.clone()),
            (Real::from(5_u8) / sixteen).unwrap_or(one),
        );

        // A nonparallel surface contact is retained by the ray query at exact
        // parameter zero. Only parallel triangles need a separate containment
        // replay because a coplanar ray intentionally has no unique hit point.
        let query = hyperlimit_point3(point);
        let certified_point = point_exact_f64(point);
        for polygon in &self.polygons {
            let certified_bounds = polygon.certified_f64_bounds_ref();
            if let (Some(point), Some(bounds)) = (certified_point, certified_bounds)
                && point[1] >= bounds.min[1]
                && point[1] <= bounds.max[1]
                && point[2] >= bounds.min[2]
                && point[2] <= bounds.max[2]
                && polygon.vertices.len() == 3
                && let Some(prepared) = polygon.prepared_triangle_query_ref()
            {
                let _ = prepared_exact_x_axis_query(
                    prepared,
                    [
                        &polygon.vertices[0].position,
                        &polygon.vertices[1].position,
                        &polygon.vertices[2].position,
                    ],
                );
            }
            let outside_bounds = match (certified_point, certified_bounds) {
                (Some(point), Some(bounds)) => {
                    point_decided_outside_certified_f64_bounds(point, bounds)
                },
                _ => point_decided_outside_aabb(point, polygon.bounding_box_ref()),
            };
            if outside_bounds {
                continue;
            }
            for [a, b, c] in polygon.triangulate_indices() {
                let positions = [
                    &polygon.vertices[a].position,
                    &polygon.vertices[b].position,
                    &polygon.vertices[c].position,
                ];
                if matches!(
                    ray_triangle_parallel_decided(
                        &direction,
                        positions,
                        polygon.prepared_triangle_query_ref(),
                    ),
                    Some(false)
                ) {
                    continue;
                }
                let [a, b, c] = positions.map(hyperlimit_point3);
                if matches!(
                    hyperlimit::classify_point_triangle3(&a, &b, &c, &query).value(),
                    Some(
                        hyperlimit::Triangle3Location::Inside
                            | hyperlimit::Triangle3Location::OnEdge
                            | hyperlimit::Triangle3Location::OnVertex
                    )
                ) {
                    return false;
                }
            }
        }

        let hits = self.ray_intersections(point, &direction);
        if hits.iter().any(|(_, parameter)| real_zero(parameter)) {
            return false;
        }
        hits.len() % 2 == 1
    }

    /// Mass properties through the Hyper physics stack.
    pub fn mass_properties(
        &self,
        density: Real,
    ) -> Result<(Real, Point3, Matrix4), ValidationError> {
        let geometry_identity = self.geometry_identity();
        if let Some((mass, center)) = LAST_BASIC_MASS_PROPERTIES.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.density == density && cached.geometry_identity == geometry_identity
                })
                .map(|cached| (cached.mass.clone(), cached.center.clone()))
        }) {
            return Ok((mass, center, Matrix4::identity()));
        }
        if !matches!(real_sign(&density), Some(RealSign::Positive)) {
            return Err(ValidationError::Other(
                "mass density must be certified positive".to_owned(),
                None,
            ));
        }
        if let Some((mass, center)) = self.finite_basic_mass_properties(&density) {
            LAST_BASIC_MASS_PROPERTIES.with_borrow_mut(|cached| {
                *cached = Some(CachedBasicMassProperties {
                    geometry_identity,
                    density,
                    mass: mass.clone(),
                    center: center.clone(),
                });
            });
            return Ok((mass, center, Matrix4::identity()));
        }

        let mut signed_volume_numerator = Real::zero();
        let mut first_moment_numerator = [Real::zero(), Real::zero(), Real::zero()];
        for polygon in &self.polygons {
            for [a, b, c] in polygon.triangulate_indices() {
                let a = &polygon.vertices[a].position;
                let b = &polygon.vertices[b].position;
                let c = &polygon.vertices[c].position;
                let determinant = Real::signed_product_sum(
                    [true, false, false, true, true, false],
                    [
                        [&a.x, &b.y, &c.z],
                        [&a.x, &b.z, &c.y],
                        [&a.y, &b.x, &c.z],
                        [&a.y, &b.z, &c.x],
                        [&a.z, &b.x, &c.y],
                        [&a.z, &b.y, &c.x],
                    ],
                );
                signed_volume_numerator = Real::signed_product_sum(
                    [true, true],
                    [[&signed_volume_numerator], [&determinant]],
                );
                for (axis, accumulator) in first_moment_numerator.iter_mut().enumerate() {
                    let coordinates =
                        [[&a.x, &b.x, &c.x], [&a.y, &b.y, &c.y], [&a.z, &b.z, &c.z]][axis];
                    let vertex_sum =
                        Real::signed_product_sum([true, true, true], coordinates.map(|v| [v]));
                    *accumulator = Real::signed_product_sum(
                        [true, true],
                        [[accumulator, &Real::one()], [&vertex_sum, &determinant]],
                    );
                }
            }
        }
        let sign = real_sign(&signed_volume_numerator);
        if !matches!(sign, Some(RealSign::Positive | RealSign::Negative)) {
            return Err(ValidationError::Other(
                "mesh volume is zero or could not be certified".to_owned(),
                None,
            ));
        }
        let signed_volume = (signed_volume_numerator.clone() / Real::from(6_u8))
            .map_err(|error| ValidationError::Other(format!("{error:?}"), None))?;
        let volume = if matches!(sign, Some(RealSign::Negative)) {
            -signed_volume
        } else {
            signed_volume
        };
        let center_denominator = signed_volume_numerator * Real::from(4_u8);
        let center = Point3::new(
            (first_moment_numerator[0].clone() / center_denominator.clone())
                .map_err(|error| ValidationError::Other(format!("{error:?}"), None))?,
            (first_moment_numerator[1].clone() / center_denominator.clone())
                .map_err(|error| ValidationError::Other(format!("{error:?}"), None))?,
            (first_moment_numerator[2].clone() / center_denominator)
                .map_err(|error| ValidationError::Other(format!("{error:?}"), None))?,
        );
        let mass = density.clone() * volume;
        LAST_BASIC_MASS_PROPERTIES.with_borrow_mut(|cached| {
            *cached = Some(CachedBasicMassProperties {
                geometry_identity,
                density,
                mass: mass.clone(),
                center: center.clone(),
            });
        });
        Ok((mass, center, Matrix4::identity()))
    }

    fn finite_basic_mass_properties(&self, density: &Real) -> Option<(Real, Point3)> {
        let density = density.to_f64_lossy()?;
        if !density.is_finite() || density <= 0.0 {
            return None;
        }
        let mut sums = [0.0_f64; 4];
        let mut compensations = [0.0_f64; 4];
        let mut add = |slot: usize, value: f64| {
            let corrected = value - compensations[slot];
            let next = sums[slot] + corrected;
            compensations[slot] = (next - sums[slot]) - corrected;
            sums[slot] = next;
        };
        for polygon in &self.polygons {
            let mut accumulate_triangle = |[ia, ib, ic]: [usize; 3]| -> Option<()> {
                let points = [ia, ib, ic].map(|index| {
                    let point = &polygon.vertices[index].position;
                    Some([
                        point.x.to_f64_lossy()?,
                        point.y.to_f64_lossy()?,
                        point.z.to_f64_lossy()?,
                    ])
                });
                let [Some(a), Some(b), Some(c)] = points else {
                    return None;
                };
                let determinant = a[0] * (b[1] * c[2] - b[2] * c[1])
                    - a[1] * (b[0] * c[2] - b[2] * c[0])
                    + a[2] * (b[0] * c[1] - b[1] * c[0]);
                if !determinant.is_finite() {
                    return None;
                }
                add(0, determinant);
                add(1, (a[0] + b[0] + c[0]) * determinant);
                add(2, (a[1] + b[1] + c[1]) * determinant);
                add(3, (a[2] + b[2] + c[2]) * determinant);
                Some(())
            };
            if polygon.vertices.len() == 3 {
                accumulate_triangle([0, 1, 2])?;
            } else {
                for triangle in polygon.triangulate_indices() {
                    accumulate_triangle(triangle)?;
                }
            }
        }
        if !sums.iter().all(|value| value.is_finite()) || sums[0] == 0.0 {
            return None;
        }
        let volume = (sums[0] / 6.0).abs();
        let denominator = 4.0 * sums[0];
        let mass = Real::try_from(density * volume).ok()?;
        let center = Point3::new(
            Real::try_from(sums[1] / denominator).ok()?,
            Real::try_from(sums[2] / denominator).ok()?,
            Real::try_from(sums[3] / denominator).ok()?,
        );
        Some((mass, center))
    }

    /// Exact uniform-density mass properties through `hyperphysics`.
    ///
    /// This is the report-bearing counterpart to [`Mesh::mass_properties`]. Mesh
    /// coordinates are finite `csgrs` adapter scalars at this boundary; each
    /// coordinate and the density are lifted into [`hyperphysics::Real`] before
    /// volume, center of mass, and inertia are accumulated. The report-bearing
    /// path follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>) by keeping the lossy
    /// exact object layer.
    pub fn exact_mass_properties(
        &self,
        density: Real,
    ) -> Result<HyperMassPropertyReport3, ValidationError> {
        let geometry_identity = self.geometry_identity();
        if let Some(report) = LAST_MASS_PROPERTIES.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.density == density && cached.geometry_identity == geometry_identity
                })
                .map(|cached| cached.report.clone())
        }) {
            return Ok(report);
        }

        let mesh = self.to_hyperphysics_closed_triangle_mesh()?;
        let report = mesh
            .uniform_density_mass_properties(density.clone())
            .map_err(|err| ValidationError::Other(format!("{err:?}"), None))?;
        LAST_MASS_PROPERTIES.with_borrow_mut(|cached| {
            *cached = Some(CachedMassProperties {
                geometry_identity,
                density,
                report: report.clone(),
            });
        });
        Ok(report)
    }

    fn geometry_identity(&self) -> Vec<u64> {
        let capacity = self
            .polygons
            .iter()
            .map(|polygon| polygon.vertices.len() + 1)
            .sum();
        let mut identity = Vec::with_capacity(capacity);
        for polygon in &self.polygons {
            identity.push(u64::try_from(polygon.vertices.len()).unwrap_or(u64::MAX));
            identity.extend(polygon.vertices.iter().map(|vertex| vertex.position_id));
        }
        identity
    }

    fn geometry_identity_matches(&self, identity: &[u64]) -> bool {
        let mut expected = identity.iter().copied();
        for polygon in &self.polygons {
            if expected.next()
                != Some(u64::try_from(polygon.vertices.len()).unwrap_or(u64::MAX))
            {
                return false;
            }
            for vertex in &polygon.vertices {
                if expected.next() != Some(vertex.position_id) {
                    return false;
                }
            }
        }
        expected.next().is_none()
    }

    fn spatial_identity(&self) -> Vec<u64> {
        self.polygons.iter().map(|polygon| polygon.plane_id).collect()
    }

    fn spatial_identity_matches(&self, identity: &[u64]) -> bool {
        self.polygons.len() == identity.len()
            && self
                .polygons
                .iter()
                .zip(identity)
                .all(|(polygon, plane_id)| polygon.plane_id == *plane_id)
    }

    /// Converts triangulated mesh polygons into a `hyperphysics` closed mesh carrier.
    pub fn to_hyperphysics_closed_triangle_mesh(
        &self,
    ) -> Result<HyperClosedTriangleMesh3, ValidationError> {
        let tri_mesh = self.triangulate();
        let triangles = tri_mesh
            .polygons
            .iter()
            .map(|polygon| {
                if polygon.vertices.len() != 3 {
                    return Err(ValidationError::TooFewPoints(Point3::origin()));
                }
                Ok(HyperTriangle3::new([
                    hyperphysics_vector3_from_point3(&polygon.vertices[0].position)?,
                    hyperphysics_vector3_from_point3(&polygon.vertices[1].position)?,
                    hyperphysics_vector3_from_point3(&polygon.vertices[2].position)?,
                ]))
            })
            .collect::<Result<Vec<_>, _>>()?;
        HyperClosedTriangleMesh3::new(triangles)
            .map_err(|err| ValidationError::Other(format!("{err:?}"), None))
    }

    /// Convert a Mesh into a Bevy `Mesh`.
    #[cfg(feature = "bevymesh")]
    pub fn to_bevy_mesh(&self) -> bevy_mesh::Mesh {
        use bevy_asset::RenderAssetUsages;
        use bevy_mesh::{Indices, Mesh, PrimitiveTopology};

        let gpu = self
            .to_gpu_mesh_f32_or_zero()
            .unwrap_or_else(|error| panic!("GPU mesh approximation failed: {error}"));

        let mut mesh =
            Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, gpu.positions);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, gpu.normals);
        mesh.insert_indices(Indices::U32(gpu.indices));

        mesh
    }

    pub(super) fn exact_axis_aligned_box_union(&self, other: &Self) -> Option<Self> {
        let (left, right) = (
            self.polygons.axis_aligned_box_fact()?,
            other.polygons.axis_aligned_box_fact()?,
        );
        let (bounds, axis, seam) = axis_aligned_box_rectangular_union(left, right)?;
        let polygons = self
            .polygons
            .iter()
            .chain(other.polygons.iter())
            .filter(|polygon| {
                !polygon.vertices.iter().all(|vertex| {
                    let coordinate = match axis {
                        0 => &vertex.position.x,
                        1 => &vertex.position.y,
                        _ => &vertex.position.z,
                    };
                    coordinate == &seam
                })
            })
            .cloned()
            .collect();
        let mut mesh = Mesh::from_polygons(polygons);
        mesh.bounding_box = OnceLock::from(bounds.clone());
        mesh.polygons.retain_axis_aligned_box_fact(bounds);
        mesh.cache_manifold_fact(true);
        mesh.cache_convex_pwn_fact();
        Some(mesh)
    }

    pub(super) fn exact_axis_aligned_box_difference(&self, other: &Self) -> Option<Self> {
        let (left, right, left_polygon, right_polygon) = (
            self.polygons.axis_aligned_box_fact()?,
            other.polygons.axis_aligned_box_fact()?,
            self.polygons.first()?,
            other.polygons.first()?,
        );
        axis_aligned_box_min_corner_difference(
            left,
            right,
            &left_polygon.metadata,
            &right_polygon.metadata,
        )
    }

    pub(super) fn exact_axis_aligned_box_intersection(&self, other: &Self) -> Option<Self> {
        let left_bounds = self.polygons.axis_aligned_box_fact()?;
        let right_bounds = other.polygons.axis_aligned_box_fact()?;
        let bounds = axis_aligned_box_intersection(left_bounds, right_bounds)?;
        let left_metadata = &self.polygons.first()?.metadata;
        let right_metadata = &other.polygons.first()?.metadata;
        let mut mesh = Self::cuboid(
            bounds.maxs.x.clone() - bounds.mins.x.clone(),
            bounds.maxs.y.clone() - bounds.mins.y.clone(),
            bounds.maxs.z.clone() - bounds.mins.z.clone(),
            left_metadata.clone(),
        )
        .into_translated(
            bounds.mins.x.clone(),
            bounds.mins.y.clone(),
            bounds.mins.z.clone(),
        );
        for polygon in &mut mesh.polygons {
            let belongs_exclusively_to_right = (polygon
                .vertices
                .iter()
                .all(|vertex| vertex.position.x == right_bounds.mins.x)
                && right_bounds.mins.x != left_bounds.mins.x)
                || (polygon
                    .vertices
                    .iter()
                    .all(|vertex| vertex.position.x == right_bounds.maxs.x)
                    && right_bounds.maxs.x != left_bounds.maxs.x)
                || (polygon
                    .vertices
                    .iter()
                    .all(|vertex| vertex.position.y == right_bounds.mins.y)
                    && right_bounds.mins.y != left_bounds.mins.y)
                || (polygon
                    .vertices
                    .iter()
                    .all(|vertex| vertex.position.y == right_bounds.maxs.y)
                    && right_bounds.maxs.y != left_bounds.maxs.y)
                || (polygon
                    .vertices
                    .iter()
                    .all(|vertex| vertex.position.z == right_bounds.mins.z)
                    && right_bounds.mins.z != left_bounds.mins.z)
                || (polygon
                    .vertices
                    .iter()
                    .all(|vertex| vertex.position.z == right_bounds.maxs.z)
                    && right_bounds.maxs.z != left_bounds.maxs.z);
            if belongs_exclusively_to_right {
                polygon.metadata = right_metadata.clone();
            }
        }
        mesh.bounding_box = OnceLock::from(bounds.clone());
        mesh.polygons.retain_axis_aligned_box_fact(bounds);
        mesh.cache_manifold_fact(true);
        mesh.cache_convex_pwn_fact();
        Some(mesh)
    }

    /// Return a new mesh representing the exact hypermesh union, or the reason
    /// hypermesh could not import, validate, or materialize the result.
    pub fn try_union(&self, other: &Self) -> Result<Self, hypermesh::HypermeshError> {
        if self.polygons.storage_identity() == other.polygons.storage_identity() {
            return Ok(Self {
                polygons: self.polygons.clone(),
                bounding_box: OnceLock::new(),
            });
        }
        if let Some(mesh) = self.exact_axis_aligned_box_union(other) {
            return Ok(mesh);
        }
        self.try_prepare_boolean_operation(other, hypermesh::HypermeshBooleanOp::Union)?
            .try_union()
    }

    /// Return a new mesh representing the exact hypermesh difference, or the
    /// typed reason hypermesh could not produce it.
    pub fn try_difference(&self, other: &Self) -> Result<Self, hypermesh::HypermeshError> {
        if self.polygons.storage_identity() == other.polygons.storage_identity() {
            return Ok(Self::empty());
        }
        if let Some(mesh) = self.exact_axis_aligned_box_difference(other) {
            return Ok(mesh);
        }
        self.try_prepare_boolean_operation(other, hypermesh::HypermeshBooleanOp::Difference)?
            .try_difference()
    }

    /// Return a new mesh representing the exact hypermesh intersection, or the
    /// typed reason hypermesh could not produce it.
    pub fn try_intersection(&self, other: &Self) -> Result<Self, hypermesh::HypermeshError> {
        if self.polygons.shares_storage_with(&other.polygons) {
            return Ok(Self {
                polygons: self.polygons.clone(),
                bounding_box: OnceLock::new(),
            });
        }
        if let Some(mesh) = self.exact_axis_aligned_box_intersection(other) {
            return Ok(mesh);
        }
        self.try_prepare_boolean_operation(other, hypermesh::HypermeshBooleanOp::Intersection)?
            .try_intersection()
    }

    /// Return a new mesh representing the exact hypermesh symmetric
    /// difference, or the typed reason hypermesh could not produce it.
    pub fn try_xor(&self, other: &Self) -> Result<Self, hypermesh::HypermeshError> {
        if self.polygons.storage_identity() == other.polygons.storage_identity() {
            return Ok(Self::empty());
        }
        self.try_prepare_boolean_operation(other, hypermesh::HypermeshBooleanOp::Xor)?
            .try_xor()
    }
}

impl Mesh<()> {
    /// Return a new empty mesh.
    pub fn new() -> Self {
        Self::empty()
    }
}

impl<M: Clone + Send + Sync + Debug> CSG for Mesh<M> {
    fn union_distributed(copies: Vec<Self>) -> Self {
        let retained_bounds = copies
            .iter()
            .map(|mesh| mesh.polygons.axis_aligned_box_fact())
            .collect::<Option<Vec<_>>>();
        let pairwise_disjoint = if let Some(bounds) = retained_bounds {
            bounds.iter().enumerate().all(|(left_index, left)| {
                bounds
                    .iter()
                    .skip(left_index + 1)
                    .all(|right| aabbs_decided_disjoint(left, right))
            })
        } else {
            let bounds = copies.iter().map(CSG::bounding_box).collect::<Vec<_>>();
            bounds.iter().enumerate().all(|(left_index, left)| {
                bounds.iter().skip(left_index + 1).all(|right| {
                    aabbs_f64_decided_disjoint(left, right)
                        || aabbs_decided_disjoint(left, right)
                })
            })
        };

        if pairwise_disjoint {
            let polygon_count = copies.iter().map(|mesh| mesh.polygons.len()).sum();
            let mut polygons = Vec::with_capacity(polygon_count);
            for mesh in copies {
                polygons.extend(mesh.polygons);
            }
            return Mesh::from_polygons(polygons);
        }

        copies
            .into_iter()
            .reduce(|acc, mesh| acc.union(&mesh))
            .expect("distribution always produces at least one copy")
    }

    fn distribute_arc(
        &self,
        count: usize,
        radius: Real,
        start_angle_deg: Real,
        end_angle_deg: Real,
    ) -> Self {
        if count < 1 {
            return self.clone();
        }
        let key = ArcDistributionKey {
            source_storage_identity: self.polygons.storage_identity(),
            count,
            radius: radius.clone(),
            start_angle_deg: start_angle_deg.clone(),
            end_angle_deg: end_angle_deg.clone(),
        };
        if !self.polygons.is_empty()
            && let Some(cached) = LAST_ARC_DISTRIBUTION.with_borrow(|cached| {
                cached.as_ref().filter(|cached| cached.key == key).cloned()
            })
        {
            let source_polygon_count = self.polygons.len();
            return Mesh::from_polygons(
                cached
                    .polygons
                    .into_iter()
                    .enumerate()
                    .map(|(index, polygon)| {
                        polygon.with_metadata(
                            self.polygons[index % source_polygon_count].metadata.clone(),
                        )
                    })
                    .collect(),
            );
        }
        let cache_on_completion = PENDING_ARC_DISTRIBUTION.with_borrow_mut(|pending| {
            let repeated = pending.as_ref() == Some(&key);
            *pending = Some(key.clone());
            repeated
        });

        let canonical_step = exact_integer_i64(&start_angle_deg)
            .zip(exact_integer_i64(&end_angle_deg))
            .and_then(|(start, end)| {
                if count == 1 {
                    return Some((start, 0));
                }
                let denominator = i64::try_from(count - 1).ok()?;
                let sweep = end.checked_sub(start)?;
                (sweep % denominator == 0).then_some((start, sweep / denominator))
            })
            .filter(|(_, step)| step % 30 == 0);
        let copies = if let Some((start, step)) = canonical_step {
            let half = (Real::one() / Real::from(2_u8)).expect("nonzero exact denominator");
            let sqrt_three_halves = (Real::from(3_u8)
                .sqrt()
                .expect("positive integer has an exact computable square root")
                / Real::from(2_u8))
            .expect("nonzero exact denominator");
            let samples = (0..count)
                .map(|index| {
                    let angle =
                        start + step * i64::try_from(index).expect("arc index fits i64");
                    let (sin, cos) =
                        canonical_sin_cos_degrees(angle, &half, &sqrt_three_halves)
                            .expect("30-degree arc step has a canonical rotation");
                    (sin, cos)
                })
                .collect::<Vec<_>>();
            let total_sweep = step
                .checked_mul(i64::try_from(count - 1).expect("arc interval count fits i64"))
                .expect("canonical arc sweep does not overflow");
            let certified_minimum_center_distance_squared = (count > 1
                && step != 0
                && step.unsigned_abs() <= 180
                && total_sweep.unsigned_abs() <= 360)
                .then(|| {
                    let minimum_gap =
                        step.unsigned_abs().min(360 - total_sweep.unsigned_abs());
                    let factor = match minimum_gap {
                        30 => (Real::one() / Real::from(4_u8)).expect("four is nonzero"),
                        60 => Real::one(),
                        90 => Real::from(2_u8),
                        120 => Real::from(3_u8),
                        150 => {
                            (Real::from(15_u8) / Real::from(4_u8)).expect("four is nonzero")
                        },
                        180 => Real::from(4_u8),
                        _ => Real::zero(),
                    };
                    factor * radius.clone() * radius.clone()
                });
            if let Some(minimum_distance_squared) = &certified_minimum_center_distance_squared
                && let Some(mesh) =
                    self.disjoint_arc_copies(&samples, &radius, minimum_distance_squared)
            {
                if cache_on_completion {
                    LAST_ARC_DISTRIBUTION.with_borrow_mut(|cached| {
                        *cached = Some(CachedArcDistribution {
                            key,
                            polygons: mesh
                                .polygons
                                .iter()
                                .cloned()
                                .map(|polygon| polygon.with_metadata(()))
                                .collect(),
                        });
                    });
                }
                return mesh;
            }
            let matrices = samples
                .into_iter()
                .map(|(sin, cos)| {
                    Matrix4::from_row_major([
                        cos.clone(),
                        -sin.clone(),
                        Real::zero(),
                        cos.clone() * radius.clone(),
                        sin.clone(),
                        cos,
                        Real::zero(),
                        sin * radius.clone(),
                        Real::zero(),
                        Real::zero(),
                        Real::one(),
                        Real::zero(),
                        Real::zero(),
                        Real::zero(),
                        Real::zero(),
                        Real::one(),
                    ])
                })
                .collect::<Vec<_>>();
            if let Some(mesh) = self
                .disjoint_rigid_copies(&matrices, certified_minimum_center_distance_squared)
            {
                if cache_on_completion {
                    LAST_ARC_DISTRIBUTION.with_borrow_mut(|cached| {
                        *cached = Some(CachedArcDistribution {
                            key,
                            polygons: mesh
                                .polygons
                                .iter()
                                .cloned()
                                .map(|polygon| polygon.with_metadata(()))
                                .collect(),
                        });
                    });
                }
                return mesh;
            }
            matrices
                .iter()
                .map(|matrix| self.transform(matrix))
                .collect::<Vec<_>>()
        } else {
            let start_rad = start_angle_deg.to_radians();
            let sweep = end_angle_deg.to_radians() - start_rad.clone();
            let translation =
                Matrix4::affine_translation([radius, Real::zero(), Real::zero()]);
            (0..count)
                .map(|index| {
                    let t = if count == 1 {
                        (Real::one() / Real::from(2_u8)).expect("nonzero exact denominator")
                    } else {
                        let numerator =
                            Real::from(u64::try_from(index).expect("arc index fits u64"));
                        let denominator =
                            Real::from(u64::try_from(count - 1).expect("arc count fits u64"));
                        (numerator / denominator).expect("nonzero arc denominator")
                    };
                    let angle = Real::affine(&start_rad, &t, &sweep);
                    self.transform(&(Matrix4::rotation_z(angle) * translation.clone()))
                })
                .collect::<Vec<_>>()
        };
        let bounds = copies.iter().map(CSG::bounding_box).collect::<Vec<_>>();
        let pairwise_disjoint = bounds.iter().enumerate().all(|(left_index, left)| {
            bounds.iter().skip(left_index + 1).all(|right| {
                aabbs_f64_decided_disjoint(left, right) || aabbs_decided_disjoint(left, right)
            })
        });
        if !pairwise_disjoint {
            return copies
                .into_iter()
                .reduce(|acc, mesh| acc.union(&mesh))
                .expect("arc distribution has at least one copy");
        }

        let polygon_count = copies.iter().map(|mesh| mesh.polygons.len()).sum();
        let mut polygons = Vec::with_capacity(polygon_count);
        for mesh in copies {
            polygons.extend(mesh.polygons);
        }
        let mesh = Mesh::from_polygons(polygons);
        if cache_on_completion {
            LAST_ARC_DISTRIBUTION.with_borrow_mut(|cached| {
                *cached = Some(CachedArcDistribution {
                    key,
                    polygons: mesh
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                });
            });
        }
        mesh
    }

    fn distribute_linear(&self, count: usize, dir: Vector3, spacing: Real) -> Self {
        if count < 1 {
            return self.clone();
        }
        let key = LinearDistributionKey {
            source_storage_identity: self.polygons.storage_identity(),
            count,
            direction: dir.clone(),
            spacing: spacing.clone(),
        };
        if let Some(cached) = LAST_LINEAR_DISTRIBUTION
            .with_borrow(|cached| cached.as_ref().filter(|cached| cached.key == key).cloned())
        {
            let source_polygon_count = self.polygons.len();
            return Mesh::from_polygons(
                cached
                    .polygons
                    .into_iter()
                    .enumerate()
                    .map(|(index, polygon)| {
                        polygon.with_metadata(
                            self.polygons[index % source_polygon_count].metadata.clone(),
                        )
                    })
                    .collect(),
            );
        }
        let cache_on_completion = PENDING_LINEAR_DISTRIBUTION.with_borrow_mut(|pending| {
            let repeated = pending.as_ref() == Some(&key);
            *pending = Some(key.clone());
            repeated
        });
        let Ok(direction) = dir.normalize_checked() else {
            return self.clone();
        };
        let step = direction * spacing;
        let offsets = (0..count)
            .map(|index| {
                step.clone()
                    * Real::from(
                        u64::try_from(index).expect("linear distribution index fits u64"),
                    )
            })
            .collect::<Vec<_>>();
        let disjoint_certified = self.polygons.axis_aligned_box_fact().is_some_and(|bounds| {
            (0..3).any(|axis| {
                let component = match real_sign(&step.0[axis]) {
                    Some(RealSign::Negative) => -step.0[axis].clone(),
                    Some(RealSign::Positive) => step.0[axis].clone(),
                    _ => return false,
                };
                let extent = match axis {
                    0 => bounds.maxs.x.clone() - bounds.mins.x.clone(),
                    1 => bounds.maxs.y.clone() - bounds.mins.y.clone(),
                    _ => bounds.maxs.z.clone() - bounds.mins.z.clone(),
                };
                real_gt(&component, &extent)
            })
        });
        let mesh = self
            .disjoint_translated_copies(&offsets, disjoint_certified)
            .unwrap_or_else(|| {
                Self::union_distributed(
                    offsets
                        .into_iter()
                        .map(|offset| self.translate_vector(offset))
                        .collect(),
                )
            });
        if cache_on_completion {
            LAST_LINEAR_DISTRIBUTION.with_borrow_mut(|cached| {
                *cached = Some(CachedLinearDistribution {
                    key,
                    polygons: mesh
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                });
            });
        }
        mesh
    }

    fn distribute_grid(&self, rows: usize, cols: usize, dx: Real, dy: Real) -> Self {
        if rows < 1 || cols < 1 {
            return self.clone();
        }
        let key = GridDistributionKey {
            source_storage_identity: self.polygons.storage_identity(),
            rows,
            cols,
            dx: dx.clone(),
            dy: dy.clone(),
        };
        if let Some(cached) = LAST_GRID_DISTRIBUTION
            .with_borrow(|cached| cached.as_ref().filter(|cached| cached.key == key).cloned())
        {
            let source_polygon_count = self.polygons.len();
            return Mesh::from_polygons(
                cached
                    .polygons
                    .into_iter()
                    .enumerate()
                    .map(|(index, polygon)| {
                        polygon.with_metadata(
                            self.polygons[index % source_polygon_count].metadata.clone(),
                        )
                    })
                    .collect(),
            );
        }
        let cache_on_completion = PENDING_GRID_DISTRIBUTION.with_borrow_mut(|pending| {
            let repeated = pending.as_ref() == Some(&key);
            *pending = Some(key.clone());
            repeated
        });
        let offsets = (0..rows)
            .flat_map(|row| {
                let dy = dy.clone();
                let dx = dx.clone();
                (0..cols).map(move |column| {
                    Vector3::new([
                        dx.clone()
                            * Real::from(u64::try_from(column).expect("grid column fits u64")),
                        dy.clone()
                            * Real::from(u64::try_from(row).expect("grid row fits u64")),
                        Real::zero(),
                    ])
                })
            })
            .collect::<Vec<_>>();
        let disjoint_certified = self.polygons.axis_aligned_box_fact().is_some_and(|bounds| {
            let magnitude = |value: &Real| match real_sign(value) {
                Some(RealSign::Negative) => Some(-value.clone()),
                Some(RealSign::Positive) => Some(value.clone()),
                _ => None,
            };
            magnitude(&dx).zip(magnitude(&dy)).is_some_and(|(dx, dy)| {
                real_gt(&dx, &(bounds.maxs.x.clone() - bounds.mins.x.clone()))
                    && real_gt(&dy, &(bounds.maxs.y.clone() - bounds.mins.y.clone()))
            })
        });
        let mesh = self
            .disjoint_translated_copies(&offsets, disjoint_certified)
            .unwrap_or_else(|| {
                Self::union_distributed(
                    offsets
                        .into_iter()
                        .map(|offset| self.translate_vector(offset))
                        .collect(),
                )
            });
        if cache_on_completion {
            LAST_GRID_DISTRIBUTION.with_borrow_mut(|cached| {
                *cached = Some(CachedGridDistribution {
                    key,
                    polygons: mesh
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                });
            });
        }
        mesh
    }

    fn center(&self) -> Self {
        if let Some(offset) = self.polygons.centering_offset_fact() {
            return self.translate_vector(offset.clone());
        }
        let half = (Real::one() / Real::from(2_u8)).expect("nonzero exact denominator");
        let offset = |bounds: &Aabb| {
            Vector3::new([
                -(bounds.mins.x.clone() + bounds.maxs.x.clone()) * half.clone(),
                -(bounds.mins.y.clone() + bounds.maxs.y.clone()) * half.clone(),
                -(bounds.mins.z.clone() + bounds.maxs.z.clone()) * half.clone(),
            ])
        };
        if let Some(bounds) = self.polygons.axis_aligned_box_fact() {
            self.translate_vector(offset(bounds))
        } else {
            let bounds = self.bounding_box();
            self.translate_vector(offset(&bounds))
        }
    }

    /// Return a new Mesh representing union of the two Meshes.
    ///
    /// ```text
    /// let c = a.union(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |       +----+
    ///     +----+--+    |       +----+       |
    ///          |   b   |            |   c   |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    fn union(&self, other: &Mesh<M>) -> Mesh<M> {
        if self.polygons.is_empty() {
            return other.clone();
        }
        if other.polygons.is_empty() {
            return self.clone();
        }
        self.try_union(other)
            .unwrap_or_else(|error| panic!("hypermesh union failed: {error}"))
    }

    /// Return a new Mesh representing diffarence of the two Meshes.
    ///
    /// ```text
    /// let c = a.difference(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |    +--+
    ///     +----+--+    |       +----+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    fn difference(&self, other: &Mesh<M>) -> Mesh<M> {
        if self.polygons.is_empty() {
            return Mesh::empty();
        }
        if other.polygons.is_empty() {
            return self.clone();
        }
        self.try_difference(other)
            .unwrap_or_else(|error| panic!("hypermesh difference failed: {error}"))
    }

    /// Return a new CSG representing intersection of the two CSG's.
    ///
    /// ```text
    /// let c = a.intersect(b);
    ///     +-------+
    ///     |       |
    ///     |   a   |
    ///     |    +--+----+   =   +--+
    ///     +----+--+    |       +--+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    fn intersection(&self, other: &Mesh<M>) -> Mesh<M> {
        if self.polygons.is_empty() || other.polygons.is_empty() {
            return Mesh::empty();
        }
        self.try_intersection(other)
            .unwrap_or_else(|error| panic!("hypermesh intersection failed: {error}"))
    }

    /// Return a new CSG representing space in this CSG excluding the space in the
    /// other CSG plus the space in the other CSG excluding the space in this CSG.
    ///
    /// ```text
    /// let c = a.xor(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   a   |
    ///     |    +--+----+   =   |    +--+----+
    ///     +----+--+    |       +----+--+    |
    ///          |   b   |            |       |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    fn xor(&self, other: &Mesh<M>) -> Mesh<M> {
        self.try_xor(other)
            .unwrap_or_else(|error| panic!("hypermesh xor failed: {error}"))
    }

    fn translate_vector(&self, vector: Vector3) -> Self {
        if let Some(cached) = LAST_TRANSLATION.with_borrow(|cached| {
            cached.iter().rev().find_map(|cached| {
                (cached.vector == vector
                    && self.polygons.storage_identity() == cached.source_geometry_identity)
                    .then(|| {
                        let mut mesh = self.with_cached_geometry(&cached.polygons);
                        mesh.bounding_box = cached
                            .bounding_box
                            .clone()
                            .map_or_else(OnceLock::new, OnceLock::from);
                        if cached.axis_aligned_box
                            && let Some(bounds) = mesh.bounding_box.get().cloned()
                        {
                            mesh.polygons.retain_axis_aligned_box_fact(bounds);
                        }
                        if let Some(offset) = cached.centering_offset.clone() {
                            mesh.polygons.retain_centering_offset(offset);
                        }
                        if let Some(pool) = cached.cuboid_vertex_pool.clone() {
                            mesh.polygons.retain_cuboid_vertex_pool(pool);
                        }
                        mesh
                    })
            })
        }) {
            return cached;
        }
        if let Some(translated) = self.retained_cuboid_translation(vector.clone()) {
            return translated;
        }
        if let Some(translated) = self.retained_indexed_translation(vector.clone()) {
            return translated;
        }
        self.clone().translate_vector_owned(vector)
    }

    fn rotate(&self, x_deg: Real, y_deg: Real, z_deg: Real) -> Self {
        let degrees = [x_deg, y_deg, z_deg];
        if let Some(cached) = LAST_ROTATION.with_borrow(|cached| {
            cached.as_ref().and_then(|cached| {
                (cached.degrees == degrees
                    && self.polygons.storage_identity() == cached.source_geometry_identity)
                    .then(|| self.with_cached_geometry(&cached.polygons))
            })
        }) {
            return cached;
        }
        let [x_deg, y_deg, z_deg] = degrees;
        self.clone().into_rotated(x_deg, y_deg, z_deg)
    }

    fn scale(&self, sx: Real, sy: Real, sz: Real) -> Self {
        let scales = [sx, sy, sz];
        let nonzero = scales
            .iter()
            .all(|scale| !matches!(real_sign(scale), Some(RealSign::Zero) | None));
        if nonzero {
            if let Some(cached) = LAST_SCALE.with_borrow(|cached| {
                cached.as_ref().and_then(|cached| {
                    (cached.scales == scales
                        && self.polygons.storage_identity() == cached.source_geometry_identity)
                        .then(|| self.with_cached_geometry(&cached.polygons))
                })
            }) {
                return cached;
            }
            let [sx, sy, sz] = scales;
            return self.clone().nonuniform_scale_owned(sx, sy, sz);
        }
        let [sx, sy, sz] = scales;
        self.transform(&Matrix4::affine_nonuniform_scale([sx, sy, sz]))
    }

    fn mirror(&self, plane: Plane) -> Self {
        let source_storage_identity = self.polygons.storage_identity();
        if let Some(polygons) = LAST_MIRROR.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.source_storage_identity == source_storage_identity
                        && cached.plane == plane
                })
                .map(|cached| cached.polygons.clone())
        }) {
            return self.with_cached_geometry(&polygons);
        }
        let cache_on_completion = PENDING_MIRROR.with_borrow_mut(|pending| {
            let repeated = pending.as_ref().is_some_and(|(identity, cached_plane)| {
                *identity == source_storage_identity && cached_plane == &plane
            });
            *pending = Some((source_storage_identity, plane.clone()));
            repeated
        });
        let axis_reflection = finite_axis_aligned_reflection(&plane);
        let fallback_matrix = axis_reflection
            .is_none()
            .then(|| finite_reflection(&plane))
            .flatten();
        if axis_reflection.is_none() && fallback_matrix.is_none() {
            return self.clone();
        }
        let mirrored = (|| {
            let transform_layout = self
                .polygons
                .retained_transform_layout()
                .filter(|layout| layout.normals_match_positions)?
                .clone();
            let source_pool = transform_layout.indexed_triangle_pool.as_ref()?;
            let source_position_f64 = transform_layout.position_f64.as_ref()?;
            let indexed_position_f64 = if let Some((axis, value)) = axis_reflection.as_ref() {
                let value = value.to_f64_lossy()?;
                Arc::new(
                    source_position_f64
                        .iter()
                        .map(|&source| {
                            let mut reflected = source;
                            reflected[*axis] = 2.0 * value - source[*axis];
                            reflected
                        })
                        .collect(),
                )
            } else {
                let matrix = fallback_matrix.as_ref()?;
                let finite_matrix = matrix_f64_lossy(matrix)?;
                Arc::new(
                    source_position_f64
                        .iter()
                        .map(|&source| transform_point_f64_lossy(&finite_matrix, source))
                        .collect::<Option<Vec<_>>>()?,
                )
            };
            let position_ids = reserve_position_ids(indexed_position_f64.len());
            cache_shared_position_f64_range(position_ids, Arc::clone(&indexed_position_f64));
            let coordinate_ids = transform_layout.coordinate_counts.map(reserve_position_ids);
            let plane_ids = reserve_plane_ids(transform_layout.plane_count);
            let vertex_pool =
                Arc::new(if let Some((axis, value)) = axis_reflection.as_ref() {
                    LazySubdivisionVertexPool::new_axis_reflected(
                        Arc::clone(source_pool),
                        *axis,
                        value.clone(),
                        Some(Arc::clone(&indexed_position_f64)),
                        self.polygons.len(),
                        position_ids,
                        coordinate_ids,
                    )
                } else {
                    let matrix = fallback_matrix.as_ref()?;
                    LazySubdivisionVertexPool::new_affine_transform(
                        Arc::clone(source_pool),
                        matrix.clone(),
                        matrix.clone(),
                        false,
                        true,
                        Some(Arc::clone(&indexed_position_f64)),
                        self.polygons.len(),
                        position_ids,
                        coordinate_ids,
                    )
                });
            let mirrored = self.retained_indexed_triangle_output_from_pool_oriented(
                &transform_layout,
                vertex_pool,
                Some(indexed_position_f64),
                plane_ids,
                true,
            )?;
            if self.has_convex_pwn_fact() {
                mirrored.cache_convex_pwn_fact();
            }
            Some(mirrored)
        })()
        .unwrap_or_else(|| {
            let matrix = fallback_matrix.unwrap_or_else(|| {
                finite_reflection(&plane)
                    .expect("validated reflection plane has a finite matrix")
            });
            self.clone().rigid_transform_owned(&matrix).inverse()
        });
        if cache_on_completion {
            LAST_MIRROR.with_borrow_mut(|cached| {
                *cached = Some(CachedMirror {
                    source_storage_identity,
                    plane,
                    polygons: mirrored
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                });
            });
        }
        mirrored
    }

    /// **Mathematical Foundation: General 3D Transformations**
    ///
    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to Mesh.
    /// This implements the complete theory of affine transformations in homogeneous coordinates.
    ///
    /// ## **Transformation Mathematics**
    ///
    /// ### **Homogeneous Coordinates**
    /// Points and vectors are represented in 4D homogeneous coordinates:
    /// - **Point**: (x, y, z, 1)ᵀ → transforms as p' = Mp
    /// - **Vector**: (x, y, z, 0)ᵀ → transforms as v' = Mv
    /// - **Normal**: n'ᵀ = nᵀM⁻¹ (inverse transpose rule)
    ///
    /// ### **Normal Vector Transformation**
    /// Normals require special handling to remain perpendicular to surfaces:
    /// ```text
    /// If: T(p)·n = 0 (tangent perpendicular to normal)
    /// Then: T(p)·T(n) ≠ 0 in general
    /// But: T(p)·(M⁻¹)ᵀn = 0 ✓
    /// ```
    /// **Proof**: (Mp)ᵀ(M⁻¹)ᵀn = pᵀMᵀ(M⁻¹)ᵀn = pᵀ(M⁻¹M)ᵀn = pᵀn = 0
    ///
    /// ### **Numerical Stability**
    /// - **Degeneracy Detection**: Check determinant before inversion
    /// - **Homogeneous Division**: Validate w-coordinate after transformation
    /// - **Precision**: Maintain accuracy through matrix decomposition
    ///
    /// ## **Algorithm Complexity**
    /// - **Vertices**: O(n) matrix-vector multiplications
    /// - **Matrix Inversion**: O(1) for 4×4 matrices
    /// - **Plane Updates**: O(n) plane reconstructions from transformed vertices
    ///
    /// The polygon z-coordinates and normal vectors are fully transformed in 3D
    ///
    /// Exact-rational transformed normals are normalized into finite dyadic
    /// shading attributes; symbolic normals retain checked `Real`
    /// normalization. Vertex normals are never consumed by topology or
    /// predicates. This keeps the inverse-transpose normal path on the same
    /// exact-aware boundary discipline as other finite mesh attributes; see Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    fn transform(&self, mat: &Matrix4) -> Mesh<M> {
        let source_geometry_identity = self.polygons.storage_identity();
        if let Some((cached_polygons, cached_bounds)) =
            LAST_GENERAL_TRANSFORM.with_borrow(|cached| {
                cached
                    .iter()
                    .rev()
                    .find(|cached| {
                        cached.matrix == *mat
                            && cached.source_geometry_identity == source_geometry_identity
                    })
                    .map(|cached| (cached.polygons.clone(), cached.bounding_box.clone()))
            })
        {
            let mut mesh = self.with_cached_geometry(&cached_polygons);
            mesh.bounding_box = cached_bounds.map_or_else(OnceLock::new, OnceLock::from);
            return mesh;
        }
        let cache_on_completion = PENDING_GENERAL_TRANSFORM.with_borrow_mut(|pending| {
            let repeated = pending.iter().any(|pending| {
                pending.matrix == *mat
                    && pending.source_geometry_identity == source_geometry_identity
            });
            if !repeated {
                if pending.len() == TRANSFORM_CACHE_CAPACITY {
                    pending.remove(0);
                }
                pending.push(PendingTransform {
                    source_geometry_identity,
                    matrix: mat.clone(),
                });
            }
            repeated
        });

        let mut prepared_matrix = mat.prepare();
        let transformed_bounds = self.polygons.axis_aligned_box_fact().and_then(|bounds| {
            let xs = [&bounds.mins.x, &bounds.maxs.x];
            let ys = [&bounds.mins.y, &bounds.maxs.y];
            let zs = [&bounds.mins.z, &bounds.maxs.z];
            let mut transformed = None;
            for x in xs {
                for y in ys {
                    for z in zs {
                        let point = Point3::new(x.clone(), y.clone(), z.clone());
                        let point = prepared_matrix.transform_point3(&point).ok()?;
                        include_point3_bounds(&mut transformed, &point);
                    }
                }
            }
            transformed.map(|(mins, maxs)| Aabb::new(mins, maxs))
        });
        // Compute inverse transpose for normal transformation
        let mat_inv_transpose = match prepared_matrix.inverse() {
            Ok(inv) => inv.transpose(),
            Err(_) => {
                eprintln!(
                    "Warning: Transformation matrix is not invertible, using identity for normals"
                );
                Matrix4::identity()
            },
        };
        let prepared_inverse_transpose = mat_inv_transpose.prepare();

        let finite_matrix = matrix_f64_lossy(mat);
        if let Some(transform_layout) = self
            .polygons
            .retained_transform_layout()
            .filter(|layout| layout.normals_match_positions)
            .cloned()
            && let Some(source_pool) = transform_layout.indexed_triangle_pool.as_ref()
            && let Some(source_position_f64) = transform_layout.position_f64.as_ref()
            && let Some(position_matrix) = matrix_f64_position_boundary(mat)
        {
            let indexed_position_f64 = source_position_f64
                .iter()
                .map(|&source| transform_point_f64_lossy(&position_matrix, source))
                .collect::<Option<Vec<_>>>()
                .map(Arc::new);
            if let Some(indexed_position_f64) = indexed_position_f64 {
                let position_ids = reserve_position_ids(indexed_position_f64.len());
                cache_shared_position_f64_range(
                    position_ids,
                    Arc::clone(&indexed_position_f64),
                );
                let coordinate_ids =
                    transform_layout.coordinate_counts.map(reserve_position_ids);
                let plane_ids = reserve_plane_ids(transform_layout.plane_count);
                let vertex_pool = Arc::new(LazySubdivisionVertexPool::new_affine_transform(
                    Arc::clone(source_pool),
                    mat.clone(),
                    mat_inv_transpose.clone(),
                    true,
                    false,
                    Some(Arc::clone(&indexed_position_f64)),
                    self.polygons.len(),
                    position_ids,
                    coordinate_ids,
                ));
                if let Some(mesh) = self.retained_indexed_triangle_output_from_pool(
                    &transform_layout,
                    vertex_pool,
                    Some(indexed_position_f64),
                    plane_ids,
                ) {
                    return self.finish_indexed_general_transform(
                        mesh,
                        transformed_bounds,
                        mat,
                        source_geometry_identity,
                        cache_on_completion,
                    );
                }
            }
        }
        if let Some(transform_layout) = self
            .polygons
            .retained_transform_layout()
            .filter(|layout| layout.normals_match_positions)
            .cloned()
        {
            let transformed_positions = transform_layout
                .position_representatives
                .iter()
                .enumerate()
                .map(|(position_slot, _)| {
                    let source = transform_layout
                        .position_representative(&self.polygons, position_slot);
                    let position = prepared_matrix.transform_point3(&source.position).ok()?;
                    let transformed_normal =
                        prepared_inverse_transpose.transform_direction3(&source.normal);
                    let normal = finite_normalized_exact_rational(&transformed_normal)
                        .or_else(|| transformed_normal.normalize_checked().ok())?;
                    let finite = finite_matrix.as_ref().and_then(|matrix| {
                        let source_finite = transform_layout
                            .position_f64
                            .as_ref()
                            .map(|positions| positions[position_slot])
                            .or_else(|| source.position_f64_lossy())?;
                        transform_point_f64_lossy(matrix, source_finite)
                    });
                    Some((position, normal, finite))
                })
                .collect::<Option<Vec<_>>>();
            if let Some(transformed_positions) = transformed_positions {
                let position_ids = reserve_position_ids(transformed_positions.len());
                cache_position_f64_range(
                    position_ids,
                    transformed_positions.len(),
                    transformed_positions.iter().map(|(_, _, finite)| *finite),
                );
                let coordinate_ids =
                    transform_layout.coordinate_counts.map(reserve_position_ids);
                let plane_ids = reserve_plane_ids(transform_layout.plane_count);
                let indexed_vertices = transformed_positions
                    .iter()
                    .enumerate()
                    .map(|(position_slot, (position, normal, _))| {
                        let source = transform_layout
                            .position_representative(&self.polygons, position_slot);
                        Vertex {
                            position: position.clone(),
                            normal: normal.clone(),
                            position_id: position_ids
                                + u64::try_from(position_slot)
                                    .expect("position slot fits u64"),
                            coordinate_ids: std::array::from_fn(|axis| {
                                coordinate_ids[axis]
                                    + u64::try_from(position_slot)
                                        .expect("coordinate slot fits u64")
                            }),
                            ruled_line: source.ruled_line,
                            hull_candidate: source.hull_candidate,
                        }
                    })
                    .collect();
                let indexed_position_f64 = transformed_positions
                    .iter()
                    .map(|(_, _, finite)| *finite)
                    .collect::<Option<Vec<_>>>()
                    .map(Arc::new);
                if let Some(mesh) = self.retained_indexed_triangle_output(
                    &transform_layout,
                    indexed_vertices,
                    indexed_position_f64,
                    plane_ids,
                ) {
                    return self.finish_indexed_general_transform(
                        mesh,
                        transformed_bounds,
                        mat,
                        source_geometry_identity,
                        cache_on_completion,
                    );
                }
            }
        }
        let mut mesh = self.clone();
        let mut transformed_positions = HashMap::<u64, (Point3, u64, Option<[f64; 3]>)>::new();
        let mut transformed_normals = HashMap::<u64, Vec<(Vector3, Vector3)>>::new();
        let matrix_facts = prepared_matrix.structural_facts();
        let mut transformed_coordinates: [HashMap<[Option<u64>; 3], u64>; 3] =
            std::array::from_fn(|_| HashMap::new());
        let mut transformed_planes = HashMap::new();

        for poly in &mut mesh.polygons {
            let plane_id = *transformed_planes
                .entry(poly.plane_id)
                .or_insert_with(fresh_plane_id);
            let mut vertices = poly.vertices_mut_with_managed_identity();
            for vert in vertices.iter_mut() {
                let source_position_id = vert.position_id;
                let source_coordinate_ids = vert.coordinate_ids;
                for (row, ids) in transformed_coordinates.iter_mut().enumerate() {
                    let key = std::array::from_fn(|column| {
                        (matrix_facts.entry_known_zero(row, column) != Some(true))
                            .then_some(source_coordinate_ids[column])
                    });
                    vert.coordinate_ids[row] =
                        *ids.entry(key).or_insert_with(fresh_position_id);
                }
                if let Some((position, position_id, position_f64)) =
                    transformed_positions.get(&source_position_id)
                {
                    vert.position = position.clone();
                    vert.position_id = *position_id;
                    cache_position_f64(*position_id, *position_f64);
                } else {
                    let position_f64 = finite_matrix.as_ref().and_then(|matrix| {
                        transform_point_f64_lossy(matrix, vert.position_f64_lossy()?)
                    });
                    match prepared_matrix.transform_point3(&vert.position) {
                        Ok(position) => {
                            let position_id = fresh_position_id();
                            transformed_positions.insert(
                                source_position_id,
                                (position.clone(), position_id, position_f64),
                            );
                            vert.position = position;
                            vert.position_id = position_id;
                            cache_position_f64(position_id, position_f64);
                        },
                        Err(_) => {
                            eprintln!(
                                "Warning: Invalid homogeneous coordinates after transformation, skipping vertex"
                            );
                            continue;
                        },
                    }
                }

                // Transform normal using inverse transpose rule
                let cached_normal =
                    transformed_normals
                        .get(&source_position_id)
                        .and_then(|entries| {
                            entries
                                .iter()
                                .find(|(source, _)| source == &vert.normal)
                                .map(|(_, transformed)| transformed.clone())
                        });
                if let Some(normal) = cached_normal {
                    vert.normal = normal;
                } else {
                    let source_normal = vert.normal.clone();
                    let transformed_normal =
                        prepared_inverse_transpose.transform_direction3(&source_normal);
                    if let Some(normal) = finite_normalized_exact_rational(&transformed_normal)
                        .or_else(|| transformed_normal.normalize_checked().ok())
                    {
                        transformed_normals
                            .entry(source_position_id)
                            .or_default()
                            .push((source_normal, normal.clone()));
                        vert.normal = normal;
                    }
                }
            }
            drop(vertices);
            poly.plane_id = plane_id;
        }

        // invalidate the old cached bounding box
        mesh.bounding_box = transformed_bounds
            .clone()
            .map_or_else(OnceLock::new, OnceLock::from);

        if cache_on_completion {
            LAST_GENERAL_TRANSFORM.with_borrow_mut(|cached| {
                if let Some(index) = cached.iter().position(|cached| {
                    cached.source_geometry_identity == source_geometry_identity
                        && cached.matrix == *mat
                }) {
                    cached.remove(index);
                }
                if cached.len() == TRANSFORM_CACHE_CAPACITY {
                    cached.remove(0);
                }
                cached.push(CachedGeneralTransform {
                    source_geometry_identity,
                    matrix: mat.clone(),
                    polygons: mesh
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                    bounding_box: transformed_bounds,
                });
            });
        }

        mesh
    }

    /// Returns an axis-aligned bounding box indicating the 3D bounds of all
    /// `polygons`.
    fn bounding_box(&self) -> Aabb {
        self.bounding_box
            .get_or_init(|| {
                let mut bounds = None;
                for polygon in &self.polygons {
                    if let Some(polygon_bounds) = polygon.cached_bounding_box_ref() {
                        include_point3_bounds(&mut bounds, &polygon_bounds.mins);
                        include_point3_bounds(&mut bounds, &polygon_bounds.maxs);
                    } else {
                        for vertex in &polygon.vertices {
                            include_point3_bounds(&mut bounds, &vertex.position);
                        }
                    }
                }
                let Some((mins, maxs)) = bounds else {
                    return Aabb::origin();
                };
                Aabb::new(mins, maxs)
            })
            .clone()
    }

    /// Invalidates object's cached bounding box.
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }

    /// Invert this Mesh (flip inside vs. outside)
    fn inverse(&self) -> Mesh<M> {
        if let Some(cached) = LAST_INVERSE.with_borrow(|cached| {
            cached.as_ref().and_then(|cached| {
                (self.polygons.storage_identity() == cached.source_geometry_identity)
                    .then(|| self.with_cached_geometry_without_facts(&cached.polygons))
            })
        }) {
            return cached;
        }

        let source_geometry_identity = self.polygons.storage_identity();
        let cache_on_completion = PENDING_INVERSE.with_borrow_mut(|pending| {
            let repeated = pending.as_ref() == Some(&source_geometry_identity);
            *pending = Some(source_geometry_identity);
            repeated
        });
        let mesh = self
            .polygons
            .retained_transform_layout()
            .filter(|layout| {
                layout.normals_match_positions
                    && layout.corner_coordinate_slots.iter().all(Option::is_none)
                    && self.polygons.topology().2
            })
            .map(|layout| {
                let vertex_pool = layout
                    .indexed_triangle_pool
                    .as_ref()
                    .map(|source_pool| {
                        Arc::new(LazySubdivisionVertexPool::new_inverted(
                            Arc::clone(source_pool),
                            layout.position_f64.clone(),
                            self.polygons.len(),
                        ))
                    })
                    .unwrap_or_else(|| {
                        let vertices = layout
                            .position_representatives
                            .iter()
                            .enumerate()
                            .map(|(position_slot, _)| {
                                let mut vertex = layout
                                    .position_representative(&self.polygons, position_slot)
                                    .clone();
                                vertex.flip();
                                vertex
                            })
                            .collect();
                        Arc::new(LazySubdivisionVertexPool::new(
                            vertices,
                            Vec::new(),
                            self.polygons.len(),
                            0,
                        ))
                    });
                let mut corner_position_slots =
                    Vec::with_capacity(layout.corner_position_slots.len());
                let polygons = layout
                    .corner_position_slots
                    .chunks_exact(3)
                    .enumerate()
                    .map(|(polygon_index, slots)| {
                        let reversed = [slots[2], slots[1], slots[0]];
                        corner_position_slots.extend(reversed);
                        Polygon::from_lazy_subdivision_triangle(
                            Arc::clone(&vertex_pool),
                            polygon_index,
                            reversed,
                            self.polygons[polygon_index].metadata.clone(),
                            self.polygons[polygon_index].plane_id,
                        )
                    })
                    .collect();
                let mut mesh =
                    Mesh::from_polygons_with_topology(polygons, self.polygons.topology());
                mesh.bounding_box = self.bounding_box.clone();
                mesh.retain_shared_position_transform_layout(
                    corner_position_slots,
                    layout.position_representatives.len(),
                    layout.position_f64.clone(),
                    Some(vertex_pool),
                    None,
                    true,
                );
                if let Some(is_manifold) = self.polygons.manifold_fact() {
                    mesh.polygons.retain_manifold_fact(is_manifold);
                }
                if let Some(bounds) = self.polygons.axis_aligned_box_fact().cloned() {
                    mesh.polygons.retain_axis_aligned_box_fact(bounds);
                }
                if let Some(offset) = self.polygons.centering_offset_fact().cloned() {
                    mesh.polygons.retain_centering_offset(offset);
                }
                if self.has_convex_pwn_fact() {
                    mesh.cache_convex_pwn_fact();
                }
                mesh
            })
            .unwrap_or_else(|| {
                let mut mesh = self.clone();
                for polygon in &mut mesh.polygons {
                    polygon.flip();
                }
                mesh
            });
        if cache_on_completion {
            LAST_INVERSE.with_borrow_mut(|cached| {
                *cached = Some(CachedInverse {
                    source_geometry_identity,
                    polygons: mesh
                        .polygons
                        .iter()
                        .cloned()
                        .map(|polygon| polygon.with_metadata(()))
                        .collect(),
                });
            });
        }
        mesh
    }
}

#[cfg(feature = "sketch")]
impl<M: Clone + Send + Sync + Debug> Mesh<M> {
    /// Convert a Profile into a Mesh.
    ///
    /// Closed area sketches are consumed only from hypercurve-owned finite
    /// [`hypercurve::FiniteRegionProfile2`] projections. `CurveRegion2` and
    /// `CurveString2` are the CAD source of truth. The grouping follows the
    /// point-in-polygon ownership structure surveyed by Hormann and Agathos,
    /// "The point in polygon problem for arbitrary polygons," *Computational
    /// Geometry* 20(3), 2001
    /// (<https://doi.org/10.1016/S0925-7721(01)00012-8>). This keeps the
    /// conversion aligned with the exact-geometric-computation boundary
    /// discipline from Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>): topology lives in
    /// hyper geometry, while mesh vertices are the finite API-boundary
    /// realization.
    pub fn from_profile(sketch: Profile, metadata: M) -> Self {
        fn ring_to_vertices(ring: &[[f64; 2]]) -> Vec<Vertex> {
            let mut vertices: Vec<_> = ring
                .iter()
                .map(|p| {
                    Vertex::new(
                        Point3::new(
                            Real::try_from(p[0]).unwrap_or_else(|_| Real::zero()),
                            Real::try_from(p[1]).unwrap_or_else(|_| Real::zero()),
                            Real::zero(),
                        ),
                        Vector3::z(),
                    )
                })
                .collect();
            if vertices.first() == vertices.last() {
                vertices.pop();
            }
            vertices
        }

        fn ring_to_polygon<M: Clone + Send + Sync>(
            ring: &[[f64; 2]],
            metadata: &M,
        ) -> Option<Polygon<M>> {
            let vertices = ring_to_vertices(ring);
            (vertices.len() >= 3)
                .then(|| Polygon::from_planar_vertices(vertices, metadata.clone()))
        }

        let projection_options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance is valid");
        let region_profiles = match sketch.project_region_profiles(&projection_options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
        };
        if !region_profiles.is_empty() {
            let final_polygons = region_profiles
                .iter()
                .flat_map(|profile| {
                    std::iter::once(profile.material().points())
                        .chain(profile.holes().iter().map(|hole| hole.points()))
                })
                .filter_map(|ring| ring_to_polygon(ring, &metadata))
                .collect();

            return Mesh {
                polygons: final_polygons,
                bounding_box: OnceLock::new(),
            };
        }

        Mesh {
            polygons: MeshPolygons::new(Vec::new()),
            bounding_box: OnceLock::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hyper_math::{Real, hreal_from_f64, tolerance};
    use crate::triangulated::IndexedTriangulated3D;

    fn r(value: f64) -> Real {
        hreal_from_f64(value).expect("test values must be finite")
    }

    fn p3(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    #[test]
    fn indexed_triangle_constructor_retains_exact_shared_position_facts() {
        let mesh = Mesh::from_indexed_triangles(
            IndexedTriangleMesh3D {
                positions: vec![
                    p3(0.0, 0.0, 0.0),
                    p3(2.0, 0.0, 0.0),
                    p3(0.0, 3.0, 0.0),
                    p3(0.0, 0.0, 0.0),
                ],
                normals: vec![Vector3::z()],
                faces: vec![[(0, 0), (1, 0), (2, 0)], [(3, 0), (2, 0), (1, 0)]],
            },
            (),
        )
        .unwrap();

        assert_eq!(mesh.topology_counts(), (2, 6));
        assert_eq!(mesh.connectivity_counts(), (3, 3));
        assert_eq!(mesh.to_hypermesh_buffers().positions.len(), 9);
        assert_eq!(
            mesh.bounding_box(),
            Aabb::new(p3(0.0, 0.0, 0.0), p3(2.0, 3.0, 0.0))
        );
        assert_eq!(mesh.build_graphics_mesh().vertices.len(), 6);
    }

    #[test]
    fn indexed_triangle_export_retains_authored_position_normal_rows() {
        let indexed = IndexedTriangleMesh3D {
            positions: vec![p3(0.0, 0.0, 0.0), p3(2.0, 0.0, 0.0), p3(0.0, 3.0, 0.0)],
            normals: vec![Vector3::x(), Vector3::y(), Vector3::z()],
            faces: vec![[(0, 0), (1, 1), (2, 2)]],
        };
        let mesh = Mesh::from_indexed_triangles(indexed.clone(), ()).unwrap();

        assert_eq!(mesh.indexed_triangles(), indexed);
    }

    #[test]
    fn indexed_triangle_constructor_rejects_missing_rows() {
        let error = Mesh::from_indexed_triangles(
            IndexedTriangleMesh3D {
                positions: vec![p3(0.0, 0.0, 0.0)],
                normals: vec![Vector3::z()],
                faces: vec![[(0, 0), (1, 0), (0, 0)]],
            },
            (),
        )
        .unwrap_err();

        assert_eq!(
            error,
            ValidationError::IndexOutOfRangeWithLen { index: 1, len: 1 }
        );
    }

    #[test]
    fn decided_disjoint_mesh_booleans_use_exact_set_identities() {
        let left = Mesh::cube(r(2.0), ());
        let right = Mesh::cube(r(2.0), ()).translate(r(10.0), Real::zero(), Real::zero());

        assert!(aabbs_decided_disjoint(
            &left.bounding_box(),
            &right.bounding_box()
        ));
        let union = left.try_union(&right).unwrap();
        assert_eq!(union.polygons.len(), 24);
        assert_eq!(union.topology_counts(), (24, 72));
        assert_eq!(left.try_difference(&right).unwrap().polygons.len(), 12);
        assert!(left.try_intersection(&right).unwrap().polygons.is_empty());
        assert_eq!(left.try_xor(&right).unwrap().polygons.len(), 24);

        let touching = Mesh::cube(r(2.0), ()).translate(r(2.0), Real::zero(), Real::zero());
        assert!(!aabbs_decided_disjoint(
            &left.bounding_box(),
            &touching.bounding_box()
        ));
    }

    #[test]
    fn min_corner_box_difference_uses_exact_closed_boundary() {
        let left = Mesh::cube(r(4.0), "left");
        let right = Mesh::cube(r(2.0), "right");
        let result = left.try_difference(&right).unwrap();

        assert_eq!(result.polygons.len(), 24);
        assert_eq!(result.topology_counts().0, 24);
        assert_eq!(result.bounding_box(), left.bounding_box());
        assert!(
            result
                .polygons
                .iter()
                .any(|polygon| polygon.metadata == "left")
        );
        assert!(
            result
                .polygons
                .iter()
                .any(|polygon| polygon.metadata == "right")
        );
        result
            .to_hypermesh_exact()
            .expect("corner box subtraction remains a closed exact mesh");
    }

    #[test]
    fn streaming_ray_intersections_match_materialized_triangle_enumeration() {
        let mesh = Mesh::sphere(r(10.0), 16, 8, ());
        let origin = p3(-20.0, 0.0, 0.0);
        let direction = Vector3::x();
        let mut materialized = mesh
            .polygons
            .iter()
            .flat_map(Polygon::triangulate)
            .filter_map(|triangle| ray_triangle_intersection(&origin, &direction, &triangle))
            .collect::<Vec<_>>();
        canonicalize_ray_hits(&mut materialized);

        assert_eq!(mesh.ray_intersections(&origin, &direction), materialized);
    }

    #[test]
    fn sphere_query_bounds_are_certified_and_selective() {
        let mesh = Mesh::sphere(r(10.0), 32, 16, ());
        let origin = p3(-20.0, 0.0, 0.0);
        let direction = Vector3::x();
        let certified_query = point_exact_f64(&origin)
            .zip(vector_exact_f64(&direction))
            .unwrap();
        let certified = mesh
            .polygons
            .iter()
            .filter_map(Polygon::certified_f64_bounds_ref)
            .count();
        let candidates = mesh
            .polygons
            .iter()
            .filter(|polygon| {
                let bounds = polygon.certified_f64_bounds_ref().unwrap();
                !ray_decided_miss_certified_f64_bounds(
                    certified_query.0,
                    certified_query.1,
                    bounds,
                    None,
                )
            })
            .count();

        assert_eq!(certified, mesh.polygons.len());
        assert!(candidates < mesh.polygons.len() / 8, "{candidates}");
    }

    #[test]
    fn query_memoization_tracks_public_geometry_edits() {
        let mut mesh = Mesh::cube(r(2.0), ());
        let origin = p3(-10.0, 0.0, 0.0);
        let direction = Vector3::x();
        let polyline = [origin.clone(), p3(10.0, 0.0, 0.0)];
        let first_ray = mesh.ray_intersections(&origin, &direction);
        let first_polyline = mesh.intersect_polyline(&polyline);
        assert_eq!(mesh.ray_intersections(&origin, &direction), first_ray);
        assert_eq!(mesh.intersect_polyline(&polyline), first_polyline);

        for polygon in &mut mesh.polygons {
            for vertex in polygon.vertices_mut().iter_mut() {
                vertex.position.x += Real::from(5_u8);
            }
        }

        let edited_ray = mesh.ray_intersections(&origin, &direction);
        let edited_polyline = mesh.intersect_polyline(&polyline);
        assert_ne!(edited_ray, first_ray);
        assert_ne!(edited_polyline, first_polyline);
    }

    #[test]
    fn rigid_transform_cache_is_invalidated_by_public_vertex_edits() {
        let mut mesh = Mesh::cube(r(2.0), ());
        let before = mesh
            .rotate(Real::zero(), Real::zero(), Real::zero())
            .bounding_box();

        for polygon in &mut mesh.polygons {
            for vertex in polygon.vertices_mut().iter_mut() {
                vertex.position.x += Real::one();
            }
        }

        let after = mesh
            .rotate(Real::zero(), Real::zero(), Real::zero())
            .bounding_box();
        assert_eq!(after.mins.x, before.mins.x + Real::one());
        assert_eq!(after.maxs.x, before.maxs.x + Real::one());
    }

    #[test]
    fn rigid_transform_cache_preserves_exact_geometry() {
        let mesh = Mesh::sphere(r(2.0), 8, 4, 17_u8);
        let first = mesh.rotate(Real::from(17_u8), Real::from(29_u8), Real::from(43_u8));
        let second = mesh.rotate(Real::from(17_u8), Real::from(29_u8), Real::from(43_u8));

        assert_eq!(first.polygons.len(), second.polygons.len());
        for (left, right) in first.polygons.iter().zip(&second.polygons) {
            assert_eq!(left.metadata, right.metadata);
            assert_eq!(left.plane, right.plane);
            assert_eq!(left.vertices, right.vertices);
        }
    }

    #[test]
    fn cloned_mesh_polygons_detach_on_mutation() {
        let mesh = Mesh::sphere(r(2.0), 8, 4, ());
        let mut cloned = mesh.clone();
        assert!(Arc::ptr_eq(&mesh.polygons.0, &cloned.polygons.0));

        cloned.polygons.remove(0);

        assert!(!Arc::ptr_eq(&mesh.polygons.0, &cloned.polygons.0));
        assert_eq!(mesh.polygons.len(), cloned.polygons.len() + 1);
    }

    #[test]
    fn triangulating_an_already_triangular_mesh_shares_polygons() {
        let mesh = Mesh::sphere(r(2.0), 8, 4, ());
        assert!(
            mesh.polygons
                .iter()
                .all(|polygon| polygon.vertices.len() == 3)
        );

        let triangulated = mesh.triangulate();

        assert!(Arc::ptr_eq(&mesh.polygons.0, &triangulated.polygons.0));
    }

    #[test]
    fn mesh_constructor_converts_planar_faces_to_triangle_storage() {
        let face = Polygon::from_planar_vertices(
            vec![
                Vertex::new(p3(0.0, 0.0, 0.0), Vector3::z()),
                Vertex::new(p3(2.0, 0.0, 0.0), Vector3::z()),
                Vertex::new(p3(2.0, 2.0, 0.0), Vector3::z()),
                Vertex::new(p3(0.0, 2.0, 0.0), Vector3::z()),
            ],
            "face",
        );

        let mesh = Mesh::from_polygons(vec![face]);

        assert_eq!(mesh.polygons.len(), 2);
        assert!(mesh.polygons.iter().all(|triangle| {
            triangle.vertices().len() == 3 && triangle.metadata() == &"face"
        }));
    }

    #[test]
    fn public_triangle_constructor_rejects_nontriangular_faces() {
        let face = Polygon::from_planar_vertices(
            vec![
                Vertex::new(p3(0.0, 0.0, 0.0), Vector3::z()),
                Vertex::new(p3(2.0, 0.0, 0.0), Vector3::z()),
                Vertex::new(p3(2.0, 2.0, 0.0), Vector3::z()),
                Vertex::new(p3(0.0, 2.0, 0.0), Vector3::z()),
            ],
            (),
        );

        assert!(matches!(
            Mesh::from_triangles(vec![face]),
            Err(ValidationError::InvalidArguments)
        ));
    }

    #[test]
    fn borrowed_vertex_iteration_matches_owned_materialization() {
        let mesh = Mesh::sphere(r(2.0), 8, 4, ());
        let borrowed = mesh.vertex_iter().cloned().collect::<Vec<_>>();

        assert_eq!(borrowed, mesh.vertices());
        assert_eq!(mesh.vertex_count(), borrowed.len());
    }

    #[test]
    fn translation_and_scale_caches_preserve_exact_geometry_and_current_metadata() {
        fn assert_same_geometry(left: &Mesh<u8>, right: &Mesh<u8>, expected_metadata: u8) {
            assert_eq!(left.polygons.len(), right.polygons.len());
            for (left, right) in left.polygons.iter().zip(&right.polygons) {
                assert_eq!(right.metadata, expected_metadata);
                assert_eq!(left.plane, right.plane);
                assert_eq!(left.vertices, right.vertices);
            }
        }

        let source = Mesh::sphere(r(2.0), 8, 4, 17_u8);
        let first_translation =
            source.translate(Real::from(3_u8), Real::from(-2_i8), Real::from(5_u8));
        let _populate_translation =
            source.translate(Real::from(3_u8), Real::from(-2_i8), Real::from(5_u8));
        let cached_translation = source.clone().map_metadata(|_| 29_u8).translate(
            Real::from(3_u8),
            Real::from(-2_i8),
            Real::from(5_u8),
        );
        assert_same_geometry(&first_translation, &cached_translation, 29);

        let half = (Real::one() / Real::from(2_u8)).unwrap();
        let first_scale = source.scale(Real::from(2_u8), half.clone(), Real::from(3_u8));
        let _populate_scale = source.scale(Real::from(2_u8), half.clone(), Real::from(3_u8));
        let cached_scale =
            source
                .map_metadata(|_| 31_u8)
                .scale(Real::from(2_u8), half, Real::from(3_u8));
        assert_same_geometry(&first_scale, &cached_scale, 31);
    }

    #[test]
    fn inverse_cache_preserves_exact_geometry_and_current_metadata() {
        let source = Mesh::sphere(r(2.0), 8, 4, 17_u8);
        let first = source.inverse();
        let _populate = source.inverse();
        let cached = source.map_metadata(|_| 29_u8).inverse();

        assert_eq!(first.polygons.len(), cached.polygons.len());
        for (left, right) in first.polygons.iter().zip(&cached.polygons) {
            assert_eq!(right.metadata, 29);
            assert_eq!(left.plane, right.plane);
            assert_eq!(left.vertices, right.vertices);
        }
    }

    #[test]
    fn general_transform_cache_preserves_exact_geometry_and_current_metadata() {
        let quarter = (Real::one() / Real::from(4_u8)).unwrap();
        let matrix = Matrix4::from_row_major([
            Real::one(),
            quarter,
            Real::zero(),
            Real::from(2_u8),
            Real::zero(),
            Real::one(),
            Real::zero(),
            Real::from(-3_i8),
            Real::zero(),
            Real::zero(),
            Real::one(),
            Real::from(4_u8),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
        ]);
        let mesh = Mesh::sphere(r(2.0), 8, 4, 17_u8);
        let first = mesh.transform(&matrix);
        let remapped = mesh.map_metadata(|_| 29_u8).transform(&matrix);

        assert_eq!(first.polygons.len(), remapped.polygons.len());
        for (left, right) in first.polygons.iter().zip(&remapped.polygons) {
            assert_eq!(left.plane, right.plane);
            assert_eq!(left.vertices, right.vertices);
            assert_eq!(left.metadata, 17);
            assert_eq!(right.metadata, 29);
        }
    }

    #[test]
    fn mesh_bounding_box_helpers_do_not_widen_by_tolerance() {
        let container = Aabb::new(Point3::origin(), p3(1.0, 1.0, 1.0));
        let exact_touch = Aabb::new(p3(1.0, 0.25, 0.25), p3(2.0, 0.75, 0.75));
        let just_outside = Aabb::new(
            Point3::new(r(1.0) + tolerance() * r(0.25), r(0.25), r(0.25)),
            p3(2.0, 0.75, 0.75),
        );
        let overhanging = Aabb::new(
            Point3::new(tolerance() * r(-0.25), r(0.25), r(0.25)),
            p3(0.75, 0.75, 0.75),
        );

        let intersecting = |lhs: &Aabb, rhs: &Aabb| {
            let lhs_min = hyperlimit::Point3::new(
                lhs.mins.x.clone(),
                lhs.mins.y.clone(),
                lhs.mins.z.clone(),
            );
            let lhs_max = hyperlimit::Point3::new(
                lhs.maxs.x.clone(),
                lhs.maxs.y.clone(),
                lhs.maxs.z.clone(),
            );
            let rhs_min = hyperlimit::Point3::new(
                rhs.mins.x.clone(),
                rhs.mins.y.clone(),
                rhs.mins.z.clone(),
            );
            let rhs_max = hyperlimit::Point3::new(
                rhs.maxs.x.clone(),
                rhs.maxs.y.clone(),
                rhs.maxs.z.clone(),
            );

            matches!(
                hyperlimit::aabb3s_intersect(&lhs_min, &lhs_max, &rhs_min, &rhs_max).value(),
                Some(true)
            )
        };
        let contains = |container: &Aabb, contained: &Aabb| {
            let container_min = hyperlimit::Point3::new(
                container.mins.x.clone(),
                container.mins.y.clone(),
                container.mins.z.clone(),
            );
            let container_max = hyperlimit::Point3::new(
                container.maxs.x.clone(),
                container.maxs.y.clone(),
                container.maxs.z.clone(),
            );
            let contained_min = hyperlimit::Point3::new(
                contained.mins.x.clone(),
                contained.mins.y.clone(),
                contained.mins.z.clone(),
            );
            let contained_max = hyperlimit::Point3::new(
                contained.maxs.x.clone(),
                contained.maxs.y.clone(),
                contained.maxs.z.clone(),
            );

            let container = hyperlimit::PreparedAabb3::new(&container_min, &container_max);
            matches!(container.contains_point(&contained_min).value(), Some(true))
                && matches!(container.contains_point(&contained_max).value(), Some(true))
        };

        assert!(intersecting(&container, &exact_touch));
        assert!(!intersecting(&container, &just_outside));
        assert!(!contains(&container, &overhanging));
    }

    #[test]
    fn mesh_triangulate_does_not_repair_cross_polygon_t_junctions() {
        let normal = Vector3::z();
        let polygons = vec![
            Polygon::from_planar_vertices(
                vec![
                    Vertex::new(p3(0.0, 0.0, 0.0), normal.clone()),
                    Vertex::new(p3(2.0, 0.0, 0.0), normal.clone()),
                    Vertex::new(p3(2.0, 1.0, 0.0), normal.clone()),
                    Vertex::new(p3(0.0, 1.0, 0.0), normal.clone()),
                ],
                (),
            ),
            Polygon::from_planar_vertices(
                vec![
                    Vertex::new(p3(1.0, 0.0, 0.0), normal.clone()),
                    Vertex::new(p3(1.5, -0.5, 0.0), normal.clone()),
                    Vertex::new(p3(0.5, -0.5, 0.0), normal),
                ],
                (),
            ),
        ];

        let triangulated = Mesh::from_polygons(polygons).triangulate();

        assert_eq!(triangulated.polygons.len(), 3);
    }

    #[test]
    fn graphics_mesh_streams_the_same_exact_triangle_rows_as_mesh_triangulation() {
        let mesh = Mesh::cube(r(2.0), ());
        let expected = mesh
            .triangulate()
            .polygons
            .into_iter()
            .flat_map(|polygon| polygon.vertices)
            .map(|vertex| {
                (
                    [vertex.position.x, vertex.position.y, vertex.position.z],
                    [
                        vertex.normal.0[0].clone(),
                        vertex.normal.0[1].clone(),
                        vertex.normal.0[2].clone(),
                    ],
                )
            })
            .collect::<Vec<_>>();

        let graphics = mesh.build_graphics_mesh();

        assert_eq!(graphics.vertices, expected);
        assert_eq!(
            graphics.indices,
            (0..u32::try_from(expected.len()).unwrap()).collect::<Vec<_>>()
        );
    }

    #[test]
    fn gpu_approximation_shims_match_hypermesh_adapters() {
        let mesh = Mesh::cube(r(2.0), ());
        let graphics = mesh.build_graphics_mesh();
        let expected_f32 =
            ::hypermesh::approximate_gpu_mesh_f32(&graphics.vertices, &graphics.indices)
                .unwrap();
        let expected_f64 =
            ::hypermesh::approximate_gpu_mesh_f64(&graphics.vertices, &graphics.indices)
                .unwrap();

        assert_eq!(mesh.try_to_gpu_mesh_f32().unwrap(), expected_f32);
        assert_eq!(mesh.try_to_gpu_mesh_f64().unwrap(), expected_f64);
        assert_eq!(expected_f32.positions.len(), 36);
        assert_eq!(expected_f64.positions.len(), 36);
        assert_eq!(expected_f32.normals.len(), 36);
        assert_eq!(expected_f64.normals.len(), 36);
        assert_eq!(expected_f32.indices, (0..36).collect::<Vec<_>>());
        assert_eq!(expected_f64.indices, expected_f32.indices);
    }

    #[cfg(feature = "bevymesh")]
    #[test]
    fn bevy_mesh_is_a_thin_wrapper_over_hypermesh_gpu_buffers() {
        use bevy_mesh::{Indices, Mesh as BevyMesh, VertexAttributeValues};

        let source = Mesh::cube(r(2.0), ());
        let expected = source.to_gpu_mesh_f32_or_zero().unwrap();
        let bevy = source.to_bevy_mesh();

        assert_eq!(
            bevy.attribute(BevyMesh::ATTRIBUTE_POSITION),
            Some(&VertexAttributeValues::Float32x3(expected.positions))
        );
        assert_eq!(
            bevy.attribute(BevyMesh::ATTRIBUTE_NORMAL),
            Some(&VertexAttributeValues::Float32x3(expected.normals))
        );
        assert_eq!(bevy.indices(), Some(&Indices::U32(expected.indices)));
    }
}
