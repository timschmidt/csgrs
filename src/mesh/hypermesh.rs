//! Adapters from the transitional `csgrs::Mesh` carrier into `hypermesh`.
//!
//! This module keeps the exact adapter boundary explicit: polygons are
//! tessellated, vertices are deduplicated by canonical coordinate keys, and the
//! flat stream is passed to `hypermesh::InputMesh` for boolean operations. This
//! follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997),
//! <https://doi.org/10.1016/0925-7721(95)00040-2>: approximate input channels
//! may propose exact objects only through audited conversion boundaries.

use std::cell::RefCell;
use std::fmt::Debug;
use std::sync::Arc;

use hashbrown::HashMap;
use hyperlattice::{Point3, Real, Vector3};
use hypermesh::{
    BooleanArrangement, BooleanOp, EmberConfig, InputMesh, Triangle, TriangleSoup,
};

use crate::{
    csg::CSG,
    mesh::Polygon,
    triangulated::{IndexedTriangleMesh3D, IndexedTriangulated3D, Triangulated3D},
    vertex::Vertex,
};

use super::{Mesh, aabbs_decided_disjoint, concatenate_disjoint_meshes};

/// Error returned while importing or materializing a transitional `csgrs::Mesh`
/// through hypermesh.
#[derive(Debug, thiserror::Error)]
pub enum HypermeshError {
    /// The mesh could not be prepared by hypermesh.
    #[error("hypermesh exact import failed: {0}")]
    Mesh(::hypermesh::HypermeshError),
    /// The exact boolean operation failed.
    #[error("hypermesh boolean operation failed: {0}")]
    Boolean(::hypermesh::HypermeshError),
    /// The boolean output could not be triangulated for csgrs.
    #[error("hypermesh materialization failed: {0}")]
    Materialize(::hypermesh::HypermeshError),
    /// Hypermesh returned a triangle without a valid source polygon.
    #[error("hypermesh returned invalid source triangle {triangle} for mesh {mesh}")]
    InvalidSource { mesh: isize, triangle: isize },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum HypermeshBooleanOp {
    Union,
    Difference,
    Intersection,
    Xor,
}

/// Finite exact buffers ready for `hypermesh::InputMesh` import.
///
/// Rational coordinates remain exact. Non-rational computable coordinates are
/// sampled at the finite `csgrs` compatibility boundary and promoted to exact
/// binary rationals before they enter `hypermesh`.
#[derive(Clone, Debug, PartialEq)]
pub struct HypermeshBuffers {
    /// Flat `x, y, z` coordinate rows.
    pub positions: Vec<Real>,
    /// Flat triangle index rows.
    pub indices: Vec<usize>,
}

pub(super) struct HypermeshAdapterInput {
    pub(super) buffers: HypermeshBuffers,
    source_polygons: Vec<usize>,
    position_ids: HashMap<u64, usize>,
}

/// Borrowed, certified preparation for extracting several Boolean results from
/// the same two meshes.
///
/// Input adaptation, intersection subdivision, BSP construction, and winding
/// classification are performed once. Each extraction still closure-certifies
/// the selected exact result and materializes fresh CSGRS polygons with their
/// source metadata.
#[derive(Debug)]
pub struct PreparedMeshBoolean<'a, M: Clone + Send + Sync + Debug> {
    left: &'a Mesh<M>,
    right: &'a Mesh<M>,
    state: PreparedMeshBooleanState,
}

#[derive(Debug)]
enum PreparedMeshBooleanState {
    Shortcut(PreparedMeshBooleanShortcut),
    Arrangement(Box<PreparedMeshBooleanArrangement>),
}

#[derive(Debug)]
struct PreparedMeshBooleanArrangement {
    arrangement: Arc<BooleanArrangement>,
    left_source_polygons: Arc<[usize]>,
    right_source_polygons: Arc<[usize]>,
}

#[derive(Clone, Debug)]
struct CachedPreparedMeshBooleanArrangement {
    left_geometry_identity: Vec<u64>,
    right_geometry_identity: Vec<u64>,
    arrangement: Arc<BooleanArrangement>,
    left_source_polygons: Arc<[usize]>,
    right_source_polygons: Arc<[usize]>,
}

thread_local! {
    static LAST_PREPARED_BOOLEAN: RefCell<Option<CachedPreparedMeshBooleanArrangement>> = const { RefCell::new(None) };
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PreparedMeshBooleanShortcut {
    LeftEmpty,
    RightEmpty,
    Disjoint,
    Identical,
}

impl<'a, M: Clone + Send + Sync + Debug> PreparedMeshBoolean<'a, M> {
    /// Returns the number of certified arrangement fragments retained for
    /// repeated extraction.
    pub fn fragment_count(&self) -> usize {
        match &self.state {
            PreparedMeshBooleanState::Shortcut(_) => 0,
            PreparedMeshBooleanState::Arrangement(prepared) => {
                prepared.arrangement.fragment_count()
            },
        }
    }

    /// Extracts the exact union from the retained arrangement.
    pub fn try_union(&self) -> Result<Mesh<M>, HypermeshError> {
        self.extract(HypermeshBooleanOp::Union)
    }

    /// Extracts the exact left-minus-right difference from the retained
    /// arrangement.
    pub fn try_difference(&self) -> Result<Mesh<M>, HypermeshError> {
        self.extract(HypermeshBooleanOp::Difference)
    }

    /// Extracts the exact intersection from the retained arrangement.
    pub fn try_intersection(&self) -> Result<Mesh<M>, HypermeshError> {
        self.extract(HypermeshBooleanOp::Intersection)
    }

    /// Extracts the exact symmetric difference from the retained arrangement.
    pub fn try_xor(&self) -> Result<Mesh<M>, HypermeshError> {
        self.extract(HypermeshBooleanOp::Xor)
    }

    fn extract(&self, operation: HypermeshBooleanOp) -> Result<Mesh<M>, HypermeshError> {
        match &self.state {
            PreparedMeshBooleanState::Shortcut(shortcut) => {
                Ok(self.extract_shortcut(*shortcut, operation))
            },
            PreparedMeshBooleanState::Arrangement(prepared) => {
                let exact_box_result = match operation {
                    HypermeshBooleanOp::Union => {
                        self.left.exact_axis_aligned_box_union(self.right)
                    },
                    HypermeshBooleanOp::Difference => {
                        self.left.exact_axis_aligned_box_difference(self.right)
                    },
                    HypermeshBooleanOp::Intersection => {
                        self.left.exact_axis_aligned_box_intersection(self.right)
                    },
                    HypermeshBooleanOp::Xor => None,
                };
                if let Some(mesh) = exact_box_result {
                    return Ok(mesh);
                }
                let soup = prepared
                    .arrangement
                    .extract_triangle_soup(operation.as_hypermesh())
                    .map_err(HypermeshError::Boolean)?;
                self.left.materialize_hypermesh_soup(
                    self.right,
                    &soup,
                    prepared.left_source_polygons.as_ref(),
                    prepared.right_source_polygons.as_ref(),
                )
            },
        }
    }

    fn extract_shortcut(
        &self,
        shortcut: PreparedMeshBooleanShortcut,
        operation: HypermeshBooleanOp,
    ) -> Mesh<M> {
        match (shortcut, operation) {
            (PreparedMeshBooleanShortcut::LeftEmpty, HypermeshBooleanOp::Union)
            | (PreparedMeshBooleanShortcut::LeftEmpty, HypermeshBooleanOp::Xor) => {
                self.right.clone()
            },
            (PreparedMeshBooleanShortcut::LeftEmpty, _)
            | (PreparedMeshBooleanShortcut::Disjoint, HypermeshBooleanOp::Intersection)
            | (PreparedMeshBooleanShortcut::Identical, HypermeshBooleanOp::Difference)
            | (PreparedMeshBooleanShortcut::Identical, HypermeshBooleanOp::Xor) => {
                Mesh::empty()
            },
            (PreparedMeshBooleanShortcut::RightEmpty, HypermeshBooleanOp::Intersection) => {
                Mesh::empty()
            },
            (PreparedMeshBooleanShortcut::RightEmpty, _)
            | (PreparedMeshBooleanShortcut::Disjoint, HypermeshBooleanOp::Difference)
            | (PreparedMeshBooleanShortcut::Identical, HypermeshBooleanOp::Union)
            | (PreparedMeshBooleanShortcut::Identical, HypermeshBooleanOp::Intersection) => {
                self.left.clone()
            },
            (PreparedMeshBooleanShortcut::Disjoint, HypermeshBooleanOp::Union)
            | (PreparedMeshBooleanShortcut::Disjoint, HypermeshBooleanOp::Xor) => {
                concatenate_disjoint_meshes(self.left, self.right)
            },
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct PositionKey([Option<u64>; 3]);

impl PositionKey {
    fn new(position: &Point3) -> Self {
        Self([
            position.x.to_f64_lossy().map(f64::to_bits),
            position.y.to_f64_lossy().map(f64::to_bits),
            position.z.to_f64_lossy().map(f64::to_bits),
        ])
    }
}

enum PositionIndices {
    One(usize),
    Collisions(Vec<usize>),
}

impl PositionIndices {
    fn find(&self, positions: &[Point3], position: &Point3) -> Option<usize> {
        match self {
            Self::One(index) => (positions[*index] == *position).then_some(*index),
            Self::Collisions(indices) => indices
                .iter()
                .copied()
                .find(|index| positions[*index] == *position),
        }
    }

    fn insert_collision(&mut self, index: usize) {
        match self {
            Self::One(first) => {
                let first = *first;
                *self = Self::Collisions(vec![first, index]);
            },
            Self::Collisions(indices) => indices.push(index),
        }
    }
}

impl<M: Clone + Send + Sync + Debug> Mesh<M> {
    /// Prepares two meshes once for repeated exact Boolean extraction.
    ///
    /// This is intended for callers that need more than one operation for the
    /// same pair. Empty, exactly disjoint, and identical inputs retain the same
    /// exact set-identity shortcuts and source polygonization as direct calls.
    pub fn try_prepare_boolean<'a>(
        &'a self,
        other: &'a Self,
    ) -> Result<PreparedMeshBoolean<'a, M>, HypermeshError> {
        self.try_prepare_boolean_operations(
            other,
            &[
                HypermeshBooleanOp::Union,
                HypermeshBooleanOp::Difference,
                HypermeshBooleanOp::Intersection,
                HypermeshBooleanOp::Xor,
            ],
        )
    }

    pub(super) fn try_prepare_boolean_operation<'a>(
        &'a self,
        other: &'a Self,
        operation: HypermeshBooleanOp,
    ) -> Result<PreparedMeshBoolean<'a, M>, HypermeshError> {
        self.try_prepare_boolean_operations(other, &[operation])
    }

    fn try_prepare_boolean_operations<'a>(
        &'a self,
        other: &'a Self,
        operations: &[HypermeshBooleanOp],
    ) -> Result<PreparedMeshBoolean<'a, M>, HypermeshError> {
        let left_storage_identity = self.polygons.storage_identity();
        let right_storage_identity = other.polygons.storage_identity();
        if left_storage_identity == right_storage_identity {
            return Ok(PreparedMeshBoolean {
                left: self,
                right: other,
                state: PreparedMeshBooleanState::Shortcut(
                    PreparedMeshBooleanShortcut::Identical,
                ),
            });
        }
        if self
            .polygons
            .is_retained_disjoint_with(right_storage_identity)
        {
            return Ok(PreparedMeshBoolean {
                left: self,
                right: other,
                state: PreparedMeshBooleanState::Shortcut(
                    PreparedMeshBooleanShortcut::Disjoint,
                ),
            });
        }

        let shortcut = if self.polygons.is_empty() {
            Some(PreparedMeshBooleanShortcut::LeftEmpty)
        } else if other.polygons.is_empty() {
            Some(PreparedMeshBooleanShortcut::RightEmpty)
        } else if aabbs_decided_disjoint(&self.bounding_box(), &other.bounding_box()) {
            Some(PreparedMeshBooleanShortcut::Disjoint)
        } else {
            None
        };
        if let Some(shortcut) = shortcut {
            if shortcut == PreparedMeshBooleanShortcut::Disjoint {
                self.polygons.retain_disjoint_partner(right_storage_identity);
                other.polygons.retain_disjoint_partner(left_storage_identity);
            }
            return Ok(PreparedMeshBoolean {
                left: self,
                right: other,
                state: PreparedMeshBooleanState::Shortcut(shortcut),
            });
        }

        let left_geometry_identity = self.geometry_identity();
        let right_geometry_identity = other.geometry_identity();
        if let Some(prepared) = LAST_PREPARED_BOOLEAN.with_borrow(|cached| {
            cached
                .as_ref()
                .filter(|cached| {
                    cached.left_geometry_identity == left_geometry_identity
                        && cached.right_geometry_identity == right_geometry_identity
                        && operations.iter().all(|operation| {
                            cached.arrangement.supports(operation.as_hypermesh())
                        })
                })
                .cloned()
        }) {
            return Ok(PreparedMeshBoolean {
                left: self,
                right: other,
                state: PreparedMeshBooleanState::Arrangement(Box::new(
                    PreparedMeshBooleanArrangement {
                        arrangement: prepared.arrangement,
                        left_source_polygons: prepared.left_source_polygons,
                        right_source_polygons: prepared.right_source_polygons,
                    },
                )),
            });
        }

        let left_input = self.build_hypermesh_input(true);
        let right_input = other.build_hypermesh_input(true);
        let shortcut = if left_input.buffers.indices.is_empty() {
            Some(PreparedMeshBooleanShortcut::LeftEmpty)
        } else if right_input.buffers.indices.is_empty() {
            Some(PreparedMeshBooleanShortcut::RightEmpty)
        } else if hypermesh_buffers_have_same_indexed_geometry(
            &left_input.buffers,
            &right_input.buffers,
        ) {
            Some(PreparedMeshBooleanShortcut::Identical)
        } else {
            None
        };
        if let Some(shortcut) = shortcut {
            return Ok(PreparedMeshBoolean {
                left: self,
                right: other,
                state: PreparedMeshBooleanState::Shortcut(shortcut),
            });
        }

        let left = input_mesh_from_buffers(&left_input.buffers);
        let right = input_mesh_from_buffers(&right_input.buffers);
        let hypermesh_operations = operations
            .iter()
            .copied()
            .map(HypermeshBooleanOp::as_hypermesh)
            .collect::<Vec<_>>();
        let certified_convex_inputs =
            [self.has_convex_pwn_fact(), other.has_convex_pwn_fact()];
        let arrangement = Arc::new(
            ::hypermesh::prepare_boolean_operations_with_certified_convex_inputs(
                &[left.as_ref(), right.as_ref()],
                &hypermesh_operations,
                &certified_convex_inputs,
                EmberConfig::default(),
            )
            .map_err(HypermeshError::Boolean)?,
        );
        let left_source_polygons = Arc::<[usize]>::from(left_input.source_polygons);
        let right_source_polygons = Arc::<[usize]>::from(right_input.source_polygons);

        LAST_PREPARED_BOOLEAN.with_borrow_mut(|cached| {
            *cached = Some(CachedPreparedMeshBooleanArrangement {
                left_geometry_identity,
                right_geometry_identity,
                arrangement: Arc::clone(&arrangement),
                left_source_polygons: Arc::clone(&left_source_polygons),
                right_source_polygons: Arc::clone(&right_source_polygons),
            });
        });

        Ok(PreparedMeshBoolean {
            left: self,
            right: other,
            state: PreparedMeshBooleanState::Arrangement(Box::new(
                PreparedMeshBooleanArrangement {
                    arrangement,
                    left_source_polygons,
                    right_source_polygons,
                },
            )),
        })
    }

    /// Build deduplicated flat triangle buffers for `hypermesh`.
    ///
    /// This differs from renderer/export buffers, which may intentionally keep
    /// per-face duplicate vertices for normal seams. Exact mesh validation needs
    /// topology-bearing vertex identity, so equal finite coordinates are
    /// canonicalized to shared indices before import into `hypermesh`.
    pub fn to_hypermesh_buffers(&self) -> HypermeshBuffers {
        self.build_hypermesh_input(false).buffers
    }

    pub(super) fn build_hypermesh_input(&self, retain_sources: bool) -> HypermeshAdapterInput {
        if let Some(input) = self.build_hypermesh_input_from_retained_layout(retain_sources) {
            return input;
        }
        let triangle_capacity = self
            .polygons
            .iter()
            .map(|polygon| polygon.vertices.len().saturating_sub(2))
            .sum::<usize>();
        let mut canonical_positions = Vec::<Point3>::with_capacity(triangle_capacity * 3);
        let mut indices = Vec::with_capacity(triangle_capacity * 3);
        let mut vertex_indices = HashMap::<PositionKey, PositionIndices>::new();
        let mut position_ids = HashMap::<u64, usize>::new();
        let mut source_polygons =
            Vec::with_capacity(if retain_sources { triangle_capacity } else { 0 });

        let mut push_triangle = |vertices: &[Vertex]| {
            for vertex in vertices {
                if let Some(&index) = position_ids.get(&vertex.position_id) {
                    indices.push(index);
                    continue;
                }
                let position = canonical_position(&vertex.position);
                let key = PositionKey::new(&position);
                let index = match vertex_indices.entry(key) {
                    hashbrown::hash_map::Entry::Vacant(entry) => {
                        let index = canonical_positions.len();
                        canonical_positions.push(position);
                        entry.insert(PositionIndices::One(index));
                        index
                    },
                    hashbrown::hash_map::Entry::Occupied(mut entry) => {
                        if let Some(index) = entry.get().find(&canonical_positions, &position)
                        {
                            index
                        } else {
                            let index = canonical_positions.len();
                            canonical_positions.push(position);
                            entry.get_mut().insert_collision(index);
                            index
                        }
                    },
                };
                position_ids.insert(vertex.position_id, index);
                indices.push(index);
            }
        };

        for (polygon_index, polygon) in self.polygons.iter().enumerate() {
            if polygon.vertices.len() == 3 {
                push_triangle(&polygon.vertices);
                if retain_sources {
                    source_polygons.push(polygon_index);
                }
                continue;
            }
            for triangle in polygon.triangulate() {
                push_triangle(&triangle);
                if retain_sources {
                    source_polygons.push(polygon_index);
                }
            }
        }

        let positions = canonical_positions
            .into_iter()
            .flat_map(|point| [point.x, point.y, point.z])
            .collect();

        HypermeshAdapterInput {
            buffers: HypermeshBuffers { positions, indices },
            source_polygons,
            position_ids,
        }
    }

    fn build_hypermesh_input_from_retained_layout(
        &self,
        retain_sources: bool,
    ) -> Option<HypermeshAdapterInput> {
        let layout = self.polygons.retained_transform_layout()?;
        let all_triangles = self.polygons.topology().2;
        if layout.indexed_polygon_corner_counts.is_none() && !all_triangles {
            return None;
        }

        let mut positions = Vec::with_capacity(layout.position_representatives.len() * 3);
        for position_slot in 0..layout.position_representatives.len() {
            let position = canonical_position(
                &layout
                    .position_representative(&self.polygons, position_slot)
                    .position,
            );
            positions.extend([position.x, position.y, position.z]);
        }

        let triangle_capacity = self
            .polygons
            .iter()
            .map(|polygon| polygon.vertices.len().saturating_sub(2))
            .sum::<usize>();
        let mut indices = Vec::with_capacity(triangle_capacity * 3);
        let mut source_polygons =
            Vec::with_capacity(if retain_sources { triangle_capacity } else { 0 });
        let mut corner_offset = 0usize;
        for polygon_index in 0..self.polygons.len() {
            let corner_count = layout
                .indexed_polygon_corner_counts
                .as_ref()
                .map_or(3, |counts| counts[polygon_index]);
            let polygon_slots = layout
                .corner_position_slots
                .get(corner_offset..corner_offset + corner_count)?;
            for index in 1..corner_count.saturating_sub(1) {
                indices.extend([
                    polygon_slots[0],
                    polygon_slots[index],
                    polygon_slots[index + 1],
                ]);
                if retain_sources {
                    source_polygons.push(polygon_index);
                }
            }
            corner_offset += corner_count;
        }
        if corner_offset != layout.corner_position_slots.len() {
            return None;
        }

        let mut position_ids = HashMap::with_capacity(layout.corner_position_slots.len());
        let mut corner_index = 0usize;
        for polygon in self.polygons.iter() {
            for vertex in polygon.vertices.iter() {
                position_ids
                    .insert(vertex.position_id, layout.corner_position_slots[corner_index]);
                corner_index += 1;
            }
        }
        Some(HypermeshAdapterInput {
            buffers: HypermeshBuffers { positions, indices },
            source_polygons,
            position_ids,
        })
    }

    /// Convert this `csgrs` mesh into an exact `hypermesh` input mesh.
    ///
    /// The returned mesh is validated against hypermesh's closed-PWN input
    /// contract. Use [`Mesh::to_hypermesh_triangle_mesh`] when a caller only
    /// needs the indexed triangle carrier and does not intend to run a boolean.
    pub fn to_hypermesh_exact(&self) -> ::hypermesh::HypermeshResult<InputMesh> {
        let buffers = self.to_hypermesh_buffers();
        let mesh = input_mesh_from_buffers(&buffers);
        ::hypermesh::prepare_input(&[mesh.as_ref()])?;
        Ok(mesh)
    }

    /// Convert this mesh into a validated indexed `hypermesh` triangle carrier.
    ///
    /// This validates indices and source-triangle degeneracy but deliberately
    /// does not claim that open surfaces satisfy hypermesh's Boolean input
    /// contract.
    pub fn to_hypermesh_triangle_mesh(&self) -> ::hypermesh::HypermeshResult<InputMesh> {
        let buffers = self.to_hypermesh_buffers();
        let mesh = input_mesh_from_buffers(&buffers);
        if mesh.positions.is_empty() {
            return Ok(mesh);
        }
        validate_surface_input(&mesh)?;
        Ok(mesh)
    }

    pub(super) fn to_hypermesh_connectivity_mesh(
        &self,
    ) -> ::hypermesh::HypermeshResult<(InputMesh, HashMap<u64, usize>)> {
        let input = self.build_hypermesh_input(false);
        let mesh = input_mesh_from_buffers(&input.buffers);
        Ok((mesh, input.position_ids))
    }

    fn materialize_hypermesh_soup(
        &self,
        other: &Self,
        soup: &TriangleSoup,
        left_source_polygons: &[usize],
        right_source_polygons: &[usize],
    ) -> Result<Self, HypermeshError> {
        let mut polygons = Vec::with_capacity(soup.triangles.len());

        for (triangle, source) in soup.triangles.iter().zip(&soup.sources) {
            let source_index = usize::try_from(source.triangle).map_err(|_| {
                HypermeshError::InvalidSource {
                    mesh: source.mesh,
                    triangle: source.triangle,
                }
            })?;
            let polygon = match source.mesh {
                0 => left_source_polygons
                    .get(source_index)
                    .and_then(|index| self.polygons.get(*index)),
                1 => source_index
                    .checked_sub(left_source_polygons.len())
                    .and_then(|index| right_source_polygons.get(index))
                    .and_then(|index| other.polygons.get(*index)),
                _ => None,
            }
            .ok_or(HypermeshError::InvalidSource {
                mesh: source.mesh,
                triangle: source.triangle,
            })?;
            let Some(a) = soup_vertex_position(soup, triangle[0]) else {
                continue;
            };
            let Some(b) = soup_vertex_position(soup, triangle[1]) else {
                continue;
            };
            let Some(c) = soup_vertex_position(soup, triangle[2]) else {
                continue;
            };
            let normal = match source.orientation {
                1 => polygon.calculate_new_normal(),
                -1 => -polygon.calculate_new_normal(),
                _ => hyper_triangle_unit_normal(&a, &b, &c).unwrap_or_else(Vector3::z),
            };
            polygons.push(Polygon::new(
                vec![
                    Vertex::new(a, normal.clone()),
                    Vertex::new(b, normal.clone()),
                    Vertex::new(c, normal),
                ],
                polygon.metadata.clone(),
            ));
        }

        Ok(Self::from_polygons(polygons))
    }
}

impl HypermeshBooleanOp {
    const fn as_hypermesh(self) -> BooleanOp {
        match self {
            Self::Union => BooleanOp::Union,
            Self::Difference => BooleanOp::Difference,
            Self::Intersection => BooleanOp::Intersection,
            Self::Xor => BooleanOp::SymmetricDifference,
        }
    }
}

impl Triangulated3D for InputMesh {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3]),
    {
        for triangle in &self.triangles {
            let [ia, ib, ic] = triangle.indices();
            let Some(a) = self.positions.get(ia).cloned() else {
                continue;
            };
            let Some(b) = self.positions.get(ib).cloned() else {
                continue;
            };
            let Some(c) = self.positions.get(ic).cloned() else {
                continue;
            };
            let normal = hyper_triangle_unit_normal(&a, &b, &c).unwrap_or_else(Vector3::z);
            f([
                Vertex::new(a, normal.clone()),
                Vertex::new(b, normal.clone()),
                Vertex::new(c, normal),
            ]);
        }
    }
}

impl IndexedTriangulated3D for InputMesh {
    fn indexed_triangles(&self) -> IndexedTriangleMesh3D {
        let positions = self.positions.clone();
        let mut normals = Vec::with_capacity(self.triangles.len());
        let mut faces = Vec::with_capacity(self.triangles.len());

        for (normal_index, triangle) in self.triangles.iter().enumerate() {
            let triangle = triangle.indices();
            if triangle.iter().any(|&index| index >= positions.len()) {
                continue;
            }
            let a = positions[triangle[0]].clone();
            let b = positions[triangle[1]].clone();
            let c = positions[triangle[2]].clone();
            let normal = hyper_triangle_unit_normal(&a, &b, &c).unwrap_or_else(Vector3::z);
            normals.push(normal);
            faces.push([
                (triangle[0], normal_index),
                (triangle[1], normal_index),
                (triangle[2], normal_index),
            ]);
        }

        IndexedTriangleMesh3D {
            positions,
            normals,
            faces,
        }
    }
}

impl Triangulated3D for TriangleSoup {
    fn visit_triangles<F>(&self, mut f: F)
    where
        F: FnMut([Vertex; 3]),
    {
        for triangle in &self.triangles {
            let Some(a) = soup_vertex_position(self, triangle[0]) else {
                continue;
            };
            let Some(b) = soup_vertex_position(self, triangle[1]) else {
                continue;
            };
            let Some(c) = soup_vertex_position(self, triangle[2]) else {
                continue;
            };
            let normal = hyper_triangle_unit_normal(&a, &b, &c).unwrap_or_else(Vector3::z);
            f([
                Vertex::new(a, normal.clone()),
                Vertex::new(b, normal.clone()),
                Vertex::new(c, normal),
            ]);
        }
    }
}

impl IndexedTriangulated3D for TriangleSoup {
    fn indexed_triangles(&self) -> IndexedTriangleMesh3D {
        let positions = self
            .vertices
            .iter()
            .map(|point| Point3::new(point.x.clone(), point.y.clone(), point.z.clone()))
            .collect::<Vec<_>>();
        let mut normals = Vec::with_capacity(self.triangles.len());
        let mut faces = Vec::with_capacity(self.triangles.len());

        for (normal_index, triangle) in self.triangles.iter().enumerate() {
            if triangle.iter().any(|&index| index >= positions.len()) {
                continue;
            }
            let a = positions[triangle[0]].clone();
            let b = positions[triangle[1]].clone();
            let c = positions[triangle[2]].clone();
            let normal = hyper_triangle_unit_normal(&a, &b, &c).unwrap_or_else(Vector3::z);
            normals.push(normal);
            faces.push([
                (triangle[0], normal_index),
                (triangle[1], normal_index),
                (triangle[2], normal_index),
            ]);
        }

        IndexedTriangleMesh3D {
            positions,
            normals,
            faces,
        }
    }
}

fn canonical_coordinate(value: &Real) -> Real {
    let is_zero = value
        .exact_rational_ref()
        .is_some_and(|rational| rational.is_zero())
        || (value.exact_rational_ref().is_none()
            && matches!(value.refine_sign_until(-128), Some(hyperreal::RealSign::Zero)));
    if is_zero { Real::zero() } else { value.clone() }
}

fn canonical_position(position: &Point3) -> Point3 {
    Point3::new(
        canonical_coordinate(&position.x),
        canonical_coordinate(&position.y),
        canonical_coordinate(&position.z),
    )
}

fn input_mesh_from_buffers(buffers: &HypermeshBuffers) -> InputMesh {
    let positions = buffers
        .positions
        .chunks_exact(3)
        .map(|chunk| Point3::new(chunk[0].clone(), chunk[1].clone(), chunk[2].clone()))
        .collect::<Vec<_>>();
    let triangles = buffers
        .indices
        .chunks_exact(3)
        .map(|chunk| Triangle::new(chunk[0], chunk[1], chunk[2]))
        .collect::<Vec<_>>();
    InputMesh::new(positions, triangles)
}

fn hypermesh_buffers_have_same_indexed_geometry(
    left: &HypermeshBuffers,
    right: &HypermeshBuffers,
) -> bool {
    if left.positions.len() != right.positions.len()
        || left.indices.len() != right.indices.len()
    {
        return false;
    }
    left.indices
        .iter()
        .zip(&right.indices)
        .all(|(&left_index, &right_index)| {
            let Some(left_start) = left_index.checked_mul(3) else {
                return false;
            };
            let Some(right_start) = right_index.checked_mul(3) else {
                return false;
            };
            left_start
                .checked_add(3)
                .and_then(|end| left.positions.get(left_start..end))
                .zip(
                    right_start
                        .checked_add(3)
                        .and_then(|end| right.positions.get(right_start..end)),
                )
                .is_some_and(|(left, right)| left == right)
        })
}

fn validate_surface_input(mesh: &InputMesh) -> ::hypermesh::HypermeshResult<()> {
    for (triangle_index, triangle) in mesh.triangles.iter().enumerate() {
        let [a, b, c] = triangle.indices();
        for index in [a, b, c] {
            if index >= mesh.positions.len() {
                return Err(::hypermesh::HypermeshError::VertexIndexOutOfBounds {
                    index,
                    vertex_count: mesh.positions.len(),
                });
            }
        }
        if !::hypermesh::Plane::from_points(
            &mesh.positions[a],
            &mesh.positions[b],
            &mesh.positions[c],
        )
        .is_valid()
        {
            return Err(::hypermesh::HypermeshError::DegenerateTriangle {
                mesh_index: 0,
                triangle_index,
            });
        }
    }
    Ok(())
}

pub(crate) fn hypermesh_is_closed_manifold(mesh: &InputMesh) -> bool {
    let mut edge_counts =
        HashMap::<(usize, usize), [u8; 2]>::with_capacity(mesh.triangles.len() * 3 / 2);
    for triangle in &mesh.triangles {
        let [a, b, c] = triangle.indices();
        for [a, b] in [[a, b], [b, c], [c, a]] {
            if a >= mesh.positions.len() || b >= mesh.positions.len() || a == b {
                return false;
            }
            let (key, direction) = if a < b { ((a, b), 0) } else { ((b, a), 1) };
            let counts = edge_counts.entry(key).or_insert([0; 2]);
            counts[direction] = counts[direction].saturating_add(1);
            if counts[direction] > 1 {
                return false;
            }
        }
    }
    !edge_counts.is_empty() && edge_counts.values().all(|counts| *counts == [1, 1])
}

fn soup_vertex_position(soup: &TriangleSoup, vertex: usize) -> Option<Point3> {
    soup.vertices
        .get(vertex)
        .map(|point| Point3::new(point.x.clone(), point.y.clone(), point.z.clone()))
}

fn hyper_triangle_unit_normal(a: &Point3, b: &Point3, c: &Point3) -> Option<Vector3> {
    (b - a).unit_cross_checked(&(c - a)).ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hyper_math::hreal_from_f64;
    use crate::vertex::Vertex;

    #[test]
    fn sphere_materializes_as_closed_hypermesh_input() {
        let radius = hreal_from_f64(1.0).expect("finite test radius");
        let sphere = Mesh::sphere(radius, 16, 8, ());
        let surface = sphere
            .to_hypermesh_triangle_mesh()
            .expect("sphere triangles should form a valid indexed carrier");
        let pole_count = surface
            .positions
            .iter()
            .filter(|point| {
                point.x.refine_sign_until(-128) == Some(hyperreal::RealSign::Zero)
                    && point.z.refine_sign_until(-128) == Some(hyperreal::RealSign::Zero)
            })
            .count();
        assert_eq!(pole_count, 2);
        assert_eq!(surface.positions.len(), 114);
        assert_eq!(surface.triangles.len(), 224);

        let input = sphere
            .to_hypermesh_exact()
            .expect("sphere should prepare as a closed hypermesh input");

        assert_eq!(input.triangles.len(), 224);
        assert!(hypermesh_is_closed_manifold(&input));
    }

    #[test]
    fn hypermesh_buffers_do_not_weld_distinct_exact_nearby_positions() {
        let zero = Real::zero();
        let one = Real::one();
        let epsilon = (one.clone() / Real::from(1_000_000_000_000_000_u64)).unwrap();
        let normal = Vector3::z();
        let first = Polygon::new(
            vec![
                Vertex::new(
                    Point3::new(zero.clone(), zero.clone(), zero.clone()),
                    normal.clone(),
                ),
                Vertex::new(
                    Point3::new(one.clone(), zero.clone(), zero.clone()),
                    normal.clone(),
                ),
                Vertex::new(
                    Point3::new(zero.clone(), one.clone(), zero.clone()),
                    normal.clone(),
                ),
            ],
            (),
        );
        let second = Polygon::new(
            vec![
                Vertex::new(
                    Point3::new(epsilon.clone(), zero.clone(), zero.clone()),
                    normal.clone(),
                ),
                Vertex::new(
                    Point3::new(one.clone(), zero.clone(), zero.clone()),
                    normal.clone(),
                ),
                Vertex::new(Point3::new(zero.clone(), one.clone(), zero), normal),
            ],
            (),
        );
        let buffers = Mesh::from_polygons(vec![first, second]).to_hypermesh_buffers();
        let positions = buffers
            .positions
            .chunks_exact(3)
            .map(|row| Point3::new(row[0].clone(), row[1].clone(), row[2].clone()))
            .collect::<Vec<_>>();

        assert_eq!(positions.len(), 4);
        assert!(positions.iter().any(|point| point.x == epsilon));
    }

    #[test]
    fn translated_cuboid_retained_adapter_matches_generic_triangulation() {
        let translated =
            Mesh::cuboid(Real::from(2_u8), Real::from(4_u8), Real::from(6_u8), ()).translate(
                Real::from(3_u8),
                Real::from(-2_i8),
                Real::from(5_u8),
            );
        let generic = Mesh::from_polygons(translated.polygons.iter().cloned().collect());

        let retained_input = translated.build_hypermesh_input(true);
        let generic_input = generic.build_hypermesh_input(true);
        let triangle_rows = |input: &HypermeshAdapterInput| {
            let positions = input
                .buffers
                .positions
                .chunks_exact(3)
                .map(|row| Point3::new(row[0].clone(), row[1].clone(), row[2].clone()))
                .collect::<Vec<_>>();
            input
                .buffers
                .indices
                .chunks_exact(3)
                .map(|triangle| {
                    [
                        positions[triangle[0]].clone(),
                        positions[triangle[1]].clone(),
                        positions[triangle[2]].clone(),
                    ]
                })
                .collect::<Vec<_>>()
        };
        let position_for_id = |input: &HypermeshAdapterInput, position_id: u64| {
            let position_index = input.position_ids[&position_id];
            let start = position_index * 3;
            Point3::new(
                input.buffers.positions[start].clone(),
                input.buffers.positions[start + 1].clone(),
                input.buffers.positions[start + 2].clone(),
            )
        };

        assert_eq!(triangle_rows(&retained_input), triangle_rows(&generic_input));
        assert_eq!(retained_input.source_polygons, generic_input.source_polygons);
        for position_id in retained_input.position_ids.keys() {
            assert_eq!(
                position_for_id(&retained_input, *position_id),
                position_for_id(&generic_input, *position_id)
            );
        }
    }

    #[test]
    fn prepared_mesh_booleans_match_direct_exact_outputs_and_metadata() {
        let left = Mesh::cube(Real::from(2_u8), 11_u8);
        let right = Mesh::cube(Real::from(2_u8), 29_u8).translate(
            Real::one(),
            (Real::from(1_u8) / Real::from(2_u8)).unwrap(),
            Real::zero(),
        );
        let prepared = left
            .try_prepare_boolean(&right)
            .expect("overlapping cubes should produce a certified arrangement");
        assert!(prepared.fragment_count() > 0);

        let cases = [
            (prepared.try_union().unwrap(), left.try_union(&right).unwrap()),
            (
                prepared.try_difference().unwrap(),
                left.try_difference(&right).unwrap(),
            ),
            (
                prepared.try_intersection().unwrap(),
                left.try_intersection(&right).unwrap(),
            ),
            (prepared.try_xor().unwrap(), left.try_xor(&right).unwrap()),
        ];

        for (prepared_result, direct_result) in cases {
            assert_eq!(
                prepared_result.to_hypermesh_buffers(),
                direct_result.to_hypermesh_buffers()
            );
            assert_eq!(
                prepared_result
                    .polygons
                    .iter()
                    .map(|polygon| polygon.metadata)
                    .collect::<Vec<_>>(),
                direct_result
                    .polygons
                    .iter()
                    .map(|polygon| polygon.metadata)
                    .collect::<Vec<_>>()
            );
        }
    }

    #[test]
    fn repeated_direct_boolean_reuses_exact_arrangement_without_reusing_metadata() {
        let left = Mesh::cube(Real::from(2_u8), 11_u8);
        let right = Mesh::cube(Real::from(2_u8), 29_u8).translate(
            Real::one(),
            (Real::one() / Real::from(2_u8)).unwrap(),
            Real::zero(),
        );
        let first = left.try_union(&right).unwrap();
        let second = left.try_union(&right).unwrap();
        assert_eq!(polygon_rows(&first), polygon_rows(&second));

        let remapped_left = left.map_metadata(|_| 31_u8);
        let remapped_right = right.map_metadata(|_| 47_u8);
        let remapped = remapped_left.try_union(&remapped_right).unwrap();
        let metadata = remapped
            .polygons
            .iter()
            .map(|polygon| polygon.metadata)
            .collect::<Vec<_>>();

        assert!(metadata.contains(&31));
        assert!(metadata.contains(&47));
        assert!(metadata.iter().all(|value| matches!(value, 31 | 47)));
        assert_eq!(first.to_hypermesh_buffers(), remapped.to_hypermesh_buffers());
    }

    fn polygon_rows(mesh: &Mesh<u8>) -> Vec<(Vec<Point3>, u8)> {
        mesh.polygons
            .iter()
            .map(|polygon| {
                (
                    polygon
                        .vertices()
                        .iter()
                        .map(|vertex| vertex.position.clone())
                        .collect(),
                    polygon.metadata,
                )
            })
            .collect()
    }

    fn assert_prepared_matches_direct_rows(left: &Mesh<u8>, right: &Mesh<u8>, label: &str) {
        let prepared = left.try_prepare_boolean(right).unwrap();
        let cases = [
            (prepared.try_union().unwrap(), left.try_union(right).unwrap()),
            (
                prepared.try_difference().unwrap(),
                left.try_difference(right).unwrap(),
            ),
            (
                prepared.try_intersection().unwrap(),
                left.try_intersection(right).unwrap(),
            ),
            (prepared.try_xor().unwrap(), left.try_xor(right).unwrap()),
        ];
        for (prepared_result, direct_result) in cases {
            assert_eq!(
                polygon_rows(&prepared_result),
                polygon_rows(&direct_result),
                "{label}"
            );
        }
    }

    #[test]
    fn prepared_shortcuts_preserve_direct_polygonization_and_metadata() {
        let left = Mesh::cube(Real::from(2_u8), 11_u8);
        let disjoint = Mesh::cube(Real::from(2_u8), 29_u8).translate(
            Real::from(5_u8),
            Real::zero(),
            Real::zero(),
        );
        let identical = Mesh::cube(Real::from(2_u8), 29_u8);
        let empty = Mesh::empty();

        for (right, label) in [
            (&disjoint, "disjoint"),
            (&identical, "identical"),
            (&empty, "right-empty"),
        ] {
            let prepared = left.try_prepare_boolean(right).unwrap();
            assert_eq!(prepared.fragment_count(), 0, "{label}");
            let cases = [
                (prepared.try_union().unwrap(), left.try_union(right).unwrap()),
                (
                    prepared.try_difference().unwrap(),
                    left.try_difference(right).unwrap(),
                ),
                (
                    prepared.try_intersection().unwrap(),
                    left.try_intersection(right).unwrap(),
                ),
                (prepared.try_xor().unwrap(), left.try_xor(right).unwrap()),
            ];
            for (prepared_result, direct_result) in cases {
                assert_eq!(
                    polygon_rows(&prepared_result),
                    polygon_rows(&direct_result),
                    "{label}"
                );
            }
        }

        let prepared_disjoint = left.try_prepare_boolean(&disjoint).unwrap();
        let mut expected_union = polygon_rows(&left);
        expected_union.extend(polygon_rows(&disjoint));
        assert_eq!(
            polygon_rows(&prepared_disjoint.try_union().unwrap()),
            expected_union
        );

        let prepared_identical = left.try_prepare_boolean(&identical).unwrap();
        assert_eq!(
            polygon_rows(&prepared_identical.try_union().unwrap()),
            polygon_rows(&left)
        );
        assert!(
            prepared_identical
                .try_difference()
                .unwrap()
                .polygons
                .is_empty()
        );

        let prepared_left_empty = empty.try_prepare_boolean(&left).unwrap();
        assert_eq!(prepared_left_empty.fragment_count(), 0);
        assert_eq!(
            polygon_rows(&prepared_left_empty.try_union().unwrap()),
            polygon_rows(&left)
        );
    }

    #[test]
    fn prepared_and_scoped_operations_match_all_topology_classes_exactly() {
        let left = Mesh::cube(Real::from(4_u8), 11_u8);
        let epsilon = (Real::one() / Real::from(1_000_000_u32)).unwrap();
        let fixtures = [
            (
                "general-overlap",
                Mesh::cube(Real::from(4_u8), 29_u8).translate(
                    Real::one(),
                    Real::one(),
                    Real::one(),
                ),
            ),
            ("contained", Mesh::cube(Real::from(2_u8), 29_u8)),
            (
                "face-touching",
                Mesh::cube(Real::from(4_u8), 29_u8).translate(
                    Real::from(4_u8),
                    Real::zero(),
                    Real::zero(),
                ),
            ),
            (
                "disjoint",
                Mesh::cube(Real::from(4_u8), 29_u8).translate(
                    Real::from(10_u8),
                    Real::zero(),
                    Real::zero(),
                ),
            ),
            ("identical", Mesh::cube(Real::from(4_u8), 29_u8)),
            (
                "tiny-exact-overlap",
                Mesh::cube(Real::from(4_u8), 29_u8).translate(
                    Real::from(4_u8) - epsilon,
                    Real::zero(),
                    Real::zero(),
                ),
            ),
        ];

        for (label, right) in fixtures {
            assert_prepared_matches_direct_rows(&left, &right, label);
        }
    }
}
