//! Host-scalar facade over native triangle meshes and rendering buffers.
//!
//! Modeling stays in [`TriangleMesh`]. Primitive inputs enter through
//! [`ScalarAdapter`]; scalar output conversion is an explicit boundary that may
//! be lossy or fail. Context-bearing operations preserve local predicate receipts,
//! independently of the mesh's inherited construction qualifications.

use super::Aabb3;
use super::scalar::{
    AdapterError, AdapterResult, F32, F64, I128, RawReal, ScalarAdapter, real3_to_scalar,
    scalar3_to_real,
};
use crate::solid::{self, SolidExt};
use crate::{GeometryContext, GeometryOutcome};
use hyperlattice::{Matrix4, Point3, Vector3};
use hypermesh::{Triangle, TriangleMesh};
use std::{
    any::{Any, TypeId},
    cell::RefCell,
    marker::PhantomData,
    sync::Arc,
};

/// One graphics row: XYZ position followed by its XYZ flat-shading normal.
pub type MeshVertex<S> = ([S; 3], [S; 3]);
/// Exported positions and triangle index triples, without normal or certainty data.
pub type IndexedMeshBuffers<S> = (Vec<[S; 3]>, Vec<[u32; 3]>);

/// Shared flat-shaded rendering buffers in the selected host scalar format.
///
/// Export may round positions and normals. These buffers carry no source-fidelity
/// certificate and must not replace the native geometry in exact decisions.
#[derive(Clone, Debug, PartialEq)]
pub struct GraphicsMesh<S> {
    /// Position/normal rows; triangle corners are emitted separately.
    pub vertices: Arc<[MeshVertex<S>]>,
    /// Flat triangle index stream into `vertices`.
    pub indices: Arc<[u32]>,
}

/// One successful graphics export retained with the source allocations it keys.
struct CachedGraphicsMesh {
    /// Owned source position identity; prevents dangling pointer keys.
    positions: Arc<[Point3]>,
    /// Owned source triangle identity, paired with the positions.
    triangles: Arc<[Triangle]>,
    /// Conversion strategy identity, not merely the host scalar type.
    adapter: TypeId,
    /// Type-erased `GraphicsMesh<A::Scalar>` recovered after matching the key.
    graphics: Box<dyn Any>,
}

thread_local! {
    /// Per-thread FIFO of up to eight successful exports, keyed by both source
    /// `Arc` identities and adapter type. Hits do not refresh insertion order.
    /// Entries retain sources and output until eviction or thread exit; there is
    /// no byte cap or public reset. Failed conversions are not cached. Custom
    /// adapters must keep conversion semantics stable for an identity to be reused.
    static GRAPHICS_MESH_CACHE: RefCell<Vec<CachedGraphicsMesh>> =
        const { RefCell::new(Vec::new()) };
}

/// Mesh facade using native-real scalar inputs and outputs.
pub type RawTriangleMesh = ScalarMesh<RawReal>;
/// Mesh facade with exact binary32 ingress and lossy scalar egress.
pub type TriangleMeshF32 = ScalarMesh<F32>;
/// Mesh facade with exact binary64 ingress and lossy scalar egress.
pub type TriangleMeshF64 = ScalarMesh<F64>;
/// Mesh facade with exact integer ingress and checked integer egress.
pub type TriangleMeshI128 = ScalarMesh<I128>;

/// Primitive-scalar boundary over native [`TriangleMesh`] geometry.
///
/// Boolean failures retain their native cause in [`AdapterError::Construction`]
/// through [`crate::errors::ValidationError::Mesh`]. Inspect the validation
/// error's [`kind`](crate::errors::ValidationError::kind) or follow
/// [`std::error::Error::source`] to distinguish invalid input from an undecided
/// predicate. This replaces text-only [`AdapterError::Validation`] failures.
/// Polyhedron and graphics construction also retain native causes; graphics
/// errors use [`crate::errors::ValidationError::Gpu`]. Point transforms retain
/// scalar arithmetic failures through [`AdapterError::Hyperreal`].
#[derive(Clone, Debug)]
pub struct ScalarMesh<A>
where
    A: ScalarAdapter,
{
    /// Authoritative native geometry and its inherited construction evidence.
    inner: TriangleMesh,
    /// Selects boundary conversions without storing an adapter instance.
    _adapter: PhantomData<fn() -> A>,
}

impl<A> ScalarMesh<A>
where
    A: ScalarAdapter,
{
    /// Wrap native geometry without scalar conversion or additional validation.
    pub fn from_native(inner: TriangleMesh) -> Self {
        Self {
            inner,
            _adapter: PhantomData,
        }
    }

    /// Consume the facade and return the native mesh with its retained evidence.
    pub fn into_native(self) -> TriangleMesh {
        self.inner
    }

    /// Borrow the native mesh without changing coordinates or evidence.
    pub const fn native(&self) -> &TriangleMesh {
        &self.inner
    }

    /// Construct canonical empty triangle geometry.
    pub fn empty() -> Self {
        Self::from_native(solid::empty())
    }

    /// Construct an axis-aligned cube with minimum corner at the origin.
    ///
    /// Imports width through `A` and validates nonnegativity with strict predicates.
    /// Zero width produces canonical empty geometry; negative or undecided width fails.
    pub fn cube(width: A::Scalar) -> AdapterResult<Self> {
        Ok(Self::from_native(solid::cube(A::into_real(width)?)?))
    }

    /// Construct an origin-anchored cuboid with nonnegative XYZ dimensions.
    ///
    /// All dimensions are validated under strict predicates; a zero dimension
    /// produces canonical empty geometry. See [`solid::cuboid`].
    pub fn cuboid(
        width: A::Scalar,
        length: A::Scalar,
        height: A::Scalar,
    ) -> AdapterResult<Self> {
        Ok(Self::from_native(solid::cuboid(
            A::into_real(width)?,
            A::into_real(length)?,
            A::into_real(height)?,
        )?))
    }

    /// Construct a tessellated sphere centered at the origin.
    ///
    /// Requires nonnegative radius, at least three longitude segments and two
    /// latitude stacks. The result is sampled triangle geometry with construction
    /// qualifications, not an analytic sphere. See [`solid::sphere`].
    pub fn sphere(radius: A::Scalar, segments: usize, stacks: usize) -> AdapterResult<Self> {
        Ok(Self::from_native(solid::sphere(
            A::into_real(radius)?,
            segments,
            stacks,
        )?))
    }

    /// Construct a tessellated cylinder along +Z from imported radius and height.
    ///
    /// Delegates dimension and segment validation to [`solid::cylinder`]. The
    /// result is a polygonal approximation with native construction qualifications.
    pub fn cylinder(
        radius: A::Scalar,
        height: A::Scalar,
        segments: usize,
    ) -> AdapterResult<Self> {
        Ok(Self::from_native(solid::cylinder(
            A::into_real(radius)?,
            A::into_real(height)?,
            segments,
        )?))
    }

    /// Import XYZ positions and triangulate simple planar polygon index rows.
    ///
    /// Preserves authored winding under strict predicates. Faces may form an open
    /// surface; success does not validate the entire collection as a closed solid.
    /// Invalid indices or malformed faces fail. See [`solid::polyhedron`].
    pub fn polyhedron(points: &[[A::Scalar; 3]], faces: &[&[usize]]) -> AdapterResult<Self> {
        let points = points
            .iter()
            .cloned()
            .map(scalar3_to_real::<A>)
            .collect::<AdapterResult<Vec<_>>>()?;
        solid::polyhedron(&points, faces)
            .map(Self::from_native)
            .map_err(AdapterError::from)
    }

    /// Compute the strict regularized union, preserving native failure causes.
    ///
    /// Returns the mesh without a separate local receipt; use
    /// [`Self::union_with_context`] for explicit policy and certainty.
    pub fn union(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .try_union(&other.inner)
            .map(Self::from_native)
            .map_err(AdapterError::from)
    }

    /// Compute the strict regularized part of this mesh outside `other`.
    ///
    /// Native validation and predicate failures remain typed; no separate local
    /// receipt is returned.
    pub fn difference(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .try_difference(&other.inner)
            .map(Self::from_native)
            .map_err(AdapterError::from)
    }

    /// Compute the strict regularized common material of the meshes.
    ///
    /// Native validation and predicate failures remain typed; no separate local
    /// receipt is returned.
    pub fn intersection(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .try_intersection(&other.inner)
            .map(Self::from_native)
            .map_err(AdapterError::from)
    }

    /// Compute the strict regularized material belonging to exactly one mesh.
    ///
    /// Native validation and predicate failures remain typed; no separate local
    /// receipt is returned.
    pub fn xor(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .try_xor(&other.inner)
            .map(Self::from_native)
            .map_err(AdapterError::from)
    }

    /// Regularized union with the selected policy.
    ///
    /// Retains the operation-local predicate receipt; inherited construction
    /// qualifications remain on the native mesh.
    pub fn union_with_context(
        &self,
        other: &Self,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        self.inner
            .try_union_with_context(&other.inner, context)
            .map(|outcome| outcome.map(Self::from_native))
            .map_err(AdapterError::from)
    }

    /// Regularized subtraction of `other` from this mesh with the selected policy.
    ///
    /// Retains the operation-local predicate receipt; inherited construction
    /// qualifications remain on the native mesh.
    pub fn difference_with_context(
        &self,
        other: &Self,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        self.inner
            .try_difference_with_context(&other.inner, context)
            .map(|outcome| outcome.map(Self::from_native))
            .map_err(AdapterError::from)
    }

    /// Regularized intersection with the selected policy.
    ///
    /// Retains the operation-local predicate receipt; inherited construction
    /// qualifications remain on the native mesh.
    pub fn intersection_with_context(
        &self,
        other: &Self,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        self.inner
            .try_intersection_with_context(&other.inner, context)
            .map(|outcome| outcome.map(Self::from_native))
            .map_err(AdapterError::from)
    }

    /// Regularized symmetric difference with the selected policy.
    ///
    /// Retains the operation-local predicate receipt; inherited construction
    /// qualifications remain on the native mesh.
    pub fn xor_with_context(
        &self,
        other: &Self,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        self.inner
            .try_xor_with_context(&other.inner, context)
            .map(|outcome| outcome.map(Self::from_native))
            .map_err(AdapterError::from)
    }

    /// Apply a native homogeneous transform under strict predicates.
    ///
    /// Preserves orientation through the grammar's checked transform path and
    /// returns no separate receipt. See [`solid::transform_with_context`].
    pub fn transform(&self, matrix: &Matrix4) -> AdapterResult<Self> {
        solid::transform(&self.inner, matrix)
            .map(Self::from_native)
            .map_err(AdapterError::from)
    }

    /// Import an XYZ displacement and apply the native mesh translation.
    ///
    /// Uses [`TriangleMesh::translated`] directly, without a separate local receipt.
    pub fn translate(&self, x: A::Scalar, y: A::Scalar, z: A::Scalar) -> AdapterResult<Self> {
        Ok(Self::from_native(self.inner.translated(
            A::into_real(x)?,
            A::into_real(y)?,
            A::into_real(z)?,
        )?))
    }

    /// Scale each axis under strict predicates, correcting winding for reflections.
    ///
    /// Zero axes are invalid; unresolved signs fail. Scalar imports occur before
    /// [`solid::scale`] validates the transform.
    pub fn scale(&self, sx: A::Scalar, sy: A::Scalar, sz: A::Scalar) -> AdapterResult<Self> {
        solid::scale(
            &self.inner,
            A::into_real(sx)?,
            A::into_real(sy)?,
            A::into_real(sz)?,
        )
        .map(Self::from_native)
        .map_err(AdapterError::from)
    }

    /// Rotate about X, then Y, then Z with imported angles in degrees.
    ///
    /// Uses [`solid::rotate`] under strict predicates and returns no separate receipt.
    pub fn rotate(
        &self,
        x_degrees: A::Scalar,
        y_degrees: A::Scalar,
        z_degrees: A::Scalar,
    ) -> AdapterResult<Self> {
        solid::rotate(
            &self.inner,
            A::into_real(x_degrees)?,
            A::into_real(y_degrees)?,
            A::into_real(z_degrees)?,
        )
        .map(Self::from_native)
        .map_err(AdapterError::from)
    }

    /// Reverse every triangle's winding without changing its positions.
    ///
    /// This is winding reversal, not construction of an unbounded set complement.
    pub fn inverse(&self) -> Self {
        Self::from_native(self.inner.reversed_winding())
    }

    /// Translate the center of native bounds to the origin under strict predicates.
    ///
    /// A mesh without bounds is returned unchanged. No scalar export is performed.
    pub fn center(&self) -> AdapterResult<Self> {
        solid::center(&self.inner)
            .map(Self::from_native)
            .map_err(AdapterError::from)
    }

    /// Translate the minimum native Z bound to zero under strict predicates.
    ///
    /// A mesh without bounds is returned unchanged. This is geometric placement,
    /// not conversion to a floating-point scalar.
    pub fn float(&self) -> AdapterResult<Self> {
        solid::float(&self.inner)
            .map(Self::from_native)
            .map_err(AdapterError::from)
    }

    /// Compute strict native bounds and export their extrema through `A`.
    ///
    /// Returns `None` when there are no positions. Export may fail or round;
    /// [`Aabb3`] does not promise an outward enclosure for lossy adapters.
    pub fn bounding_box(&self) -> AdapterResult<Option<Aabb3<A::Scalar>>> {
        solid::bounding_box(&self.inner)?
            .map(super::convert_bounds::<A>)
            .transpose()
    }

    /// Orientation-preserving checked transformation with the selected policy.
    ///
    /// Retains the operation-local predicate receipt; inherited construction
    /// qualifications remain on the native mesh.
    pub fn transform_with_context(
        &self,
        matrix: &Matrix4,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        solid::transform_with_context(&self.inner, matrix, context)
            .map(|outcome| outcome.map(Self::from_native))
            .map_err(AdapterError::from)
    }

    /// Translation by imported XYZ displacements with the selected policy.
    ///
    /// Retains the operation-local predicate receipt; inherited construction
    /// qualifications remain on the native mesh.
    pub fn translate_with_context(
        &self,
        x: A::Scalar,
        y: A::Scalar,
        z: A::Scalar,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        solid::transform_with_context(
            &self.inner,
            &Matrix4::affine_translation([
                A::into_real(x)?,
                A::into_real(y)?,
                A::into_real(z)?,
            ]),
            context,
        )
        .map(|outcome| outcome.map(Self::from_native))
        .map_err(AdapterError::from)
    }

    /// Axis scaling with zero-axis rejection and reflection winding correction with the selected policy.
    ///
    /// Retains the operation-local predicate receipt; inherited construction
    /// qualifications remain on the native mesh.
    pub fn scale_with_context(
        &self,
        x: A::Scalar,
        y: A::Scalar,
        z: A::Scalar,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        solid::scale_with_context(
            &self.inner,
            A::into_real(x)?,
            A::into_real(y)?,
            A::into_real(z)?,
            context,
        )
        .map(|outcome| outcome.map(Self::from_native))
        .map_err(AdapterError::from)
    }

    /// X/Y/Z rotation by imported degree angles with the selected policy.
    ///
    /// Retains the operation-local predicate receipt; inherited construction
    /// qualifications remain on the native mesh.
    pub fn rotate_with_context(
        &self,
        x: A::Scalar,
        y: A::Scalar,
        z: A::Scalar,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        solid::rotate_with_context(
            &self.inner,
            A::into_real(x)?,
            A::into_real(y)?,
            A::into_real(z)?,
            context,
        )
        .map(|outcome| outcome.map(Self::from_native))
        .map_err(AdapterError::from)
    }

    /// Translation of the native bounds center to the origin with the selected policy.
    ///
    /// Retains the operation-local predicate receipt; inherited construction
    /// qualifications remain on the native mesh.
    pub fn center_with_context(
        &self,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        solid::center_with_context(&self.inner, context)
            .map(|outcome| outcome.map(Self::from_native))
            .map_err(AdapterError::from)
    }

    /// Translation of the minimum native Z bound to zero with the selected policy.
    ///
    /// Retains the operation-local predicate receipt; inherited construction
    /// qualifications remain on the native mesh.
    pub fn float_with_context(
        &self,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        solid::float_with_context(&self.inner, context)
            .map(|outcome| outcome.map(Self::from_native))
            .map_err(AdapterError::from)
    }

    /// Compute native bounds with the selected policy before scalar export.
    ///
    /// The receipt covers native decisions, not scalar conversion fidelity.
    /// Rounded extrema need not enclose the original exact geometry.
    pub fn bounding_box_with_context(
        &self,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Option<Aabb3<A::Scalar>>>> {
        solid::bounding_box_with_context(&self.inner, context)?
            .try_map(|bounds| bounds.map(super::convert_bounds::<A>).transpose())
    }

    /// Triangulate faces with the selected policy after importing scalar coordinates.
    ///
    /// Retains the local receipt; like [`Self::polyhedron`], this does not prove
    /// that the collection of faces forms a closed solid.
    pub fn polyhedron_with_context(
        points: &[[A::Scalar; 3]],
        faces: &[&[usize]],
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        let points = points
            .iter()
            .cloned()
            .map(scalar3_to_real::<A>)
            .collect::<AdapterResult<Vec<_>>>()?;
        solid::polyhedron_with_context(&points, faces, context)
            .map(|outcome| outcome.map(Self::from_native))
            .map_err(AdapterError::from)
    }

    /// Export flat-shaded position/normal rows and `u32` indices for rendering.
    ///
    /// Native buffer construction checks source indices and derives exact unit
    /// face normals before scalar export. Degenerate or undecidable normals and
    /// failed conversions remain errors; no partial buffer is returned. Rounded
    /// positions and normals carry no exact-geometry or unit-length guarantee.
    ///
    /// Successful results share buffers through an eight-entry thread-local FIFO
    /// keyed by both source allocation identities and adapter type. Hits do not
    /// refresh order; entries retain input and output until eviction or thread exit.
    /// The entry limit is not a byte bound and there is no public cache reset.
    pub fn graphics_mesh(&self) -> AdapterResult<GraphicsMesh<A::Scalar>> {
        if let Some(graphics) = GRAPHICS_MESH_CACHE.with_borrow(|entries| {
            entries
                .iter()
                .rev()
                .find(|entry| {
                    Arc::ptr_eq(&entry.positions, &self.inner.positions)
                        && Arc::ptr_eq(&entry.triangles, &self.inner.triangles)
                        && entry.adapter == TypeId::of::<A>()
                })
                .and_then(|entry| {
                    entry
                        .graphics
                        .downcast_ref::<GraphicsMesh<A::Scalar>>()
                        .cloned()
                })
        }) {
            return Ok(graphics);
        }
        let mesh = self
            .inner
            .to_exact_gpu_mesh_buffers()
            .map_err(AdapterError::from)?;
        let vertices = mesh
            .vertices
            .iter()
            .map(|(position, normal)| {
                Ok((real3_to_scalar::<A>(position)?, real3_to_scalar::<A>(normal)?))
            })
            .collect::<AdapterResult<Vec<_>>>()?;

        let graphics = GraphicsMesh {
            vertices: vertices.into(),
            indices: Arc::from(mesh.indices.as_slice()),
        };
        GRAPHICS_MESH_CACHE.with_borrow_mut(|entries| {
            /// Maximum retained exports per thread, not a byte limit.
            const CAPACITY: usize = 8;
            if entries.len() == CAPACITY {
                entries.remove(0);
            }
            entries.push(CachedGraphicsMesh {
                positions: Arc::clone(&self.inner.positions),
                triangles: Arc::clone(&self.inner.triangles),
                adapter: TypeId::of::<A>(),
                graphics: Box::new(graphics.clone()),
            });
        });
        Ok(graphics)
    }

    /// Export native position rows and triangle indices without duplicating corners.
    ///
    /// Scalar conversion follows `A`; indices that do not fit `u32` return a
    /// textual validation error. This function does not check index bounds or
    /// closed topology and does not export normals or construction evidence.
    pub fn vertices_and_indices(&self) -> AdapterResult<IndexedMeshBuffers<A::Scalar>> {
        let vertices = self
            .inner
            .positions
            .iter()
            .map(point3_to_scalar::<A>)
            .collect::<AdapterResult<Vec<_>>>()?;
        let indices = self
            .inner
            .triangles
            .iter()
            .map(|triangle| {
                let [a, b, c] = triangle.indices();
                Ok([
                    u32::try_from(a).map_err(index_error)?,
                    u32::try_from(b).map_err(index_error)?,
                    u32::try_from(c).map_err(index_error)?,
                ])
            })
            .collect::<AdapterResult<Vec<_>>>()?;
        Ok((vertices, indices))
    }

    /// Import points, apply homogeneous point transforms, and export through `A`.
    ///
    /// Stops at the first conversion or native arithmetic failure. This independent
    /// point operation performs no mesh topology validation and returns no receipt.
    pub fn transform_adapter_points(
        points: &[[A::Scalar; 3]],
        matrix: &Matrix4,
    ) -> AdapterResult<Vec<[A::Scalar; 3]>> {
        points
            .iter()
            .cloned()
            .map(scalar3_to_real::<A>)
            .map(|point| {
                let [x, y, z] = point?;
                let transformed = matrix
                    .transform_point3(&Point3::new(x, y, z))
                    .map_err(AdapterError::from)?;
                point3_to_scalar::<A>(&transformed)
            })
            .collect()
    }

    /// Import XYZ components as a native vector without normalization.
    pub fn vector_from_adapter(values: [A::Scalar; 3]) -> AdapterResult<Vector3> {
        let [x, y, z] = scalar3_to_real::<A>(values)?;
        Ok(Vector3::from_xyz(x, y, z))
    }
}

/// Describe a failed `usize`-to-`u32` index conversion as adapter validation.
fn index_error(error: std::num::TryFromIntError) -> AdapterError {
    AdapterError::Validation(format!("native mesh index exceeds u32: {error}"))
}

/// Export XYZ coordinates in order, returning the first scalar conversion error.
fn point3_to_scalar<A: ScalarAdapter>(point: &Point3) -> AdapterResult<[A::Scalar; 3]> {
    Ok([
        A::from_real(&point.x)?,
        A::from_real(&point.y)?,
        A::from_real(&point.z)?,
    ])
}
