use super::Aabb3;
use super::scalar::{
    AdapterError, AdapterResult, F32, F64, I128, RawReal, ScalarAdapter, real3_to_scalar,
    scalar3_to_real,
};
use crate::solid::{self, SolidExt};
use hyperlattice::{Matrix4, Point3, Vector3};
use hypermesh::{Triangle, TriangleMesh};
use std::{
    any::{Any, TypeId},
    cell::RefCell,
    marker::PhantomData,
    sync::Arc,
};

pub type MeshVertex<S> = ([S; 3], [S; 3]);
pub type IndexedMeshBuffers<S> = (Vec<[S; 3]>, Vec<[u32; 3]>);

#[derive(Clone, Debug, PartialEq)]
pub struct GraphicsMesh<S> {
    pub vertices: Arc<[MeshVertex<S>]>,
    pub indices: Arc<[u32]>,
}

struct CachedGraphicsMesh {
    positions: Arc<[Point3]>,
    triangles: Arc<[Triangle]>,
    adapter: TypeId,
    graphics: Box<dyn Any>,
}

thread_local! {
    static GRAPHICS_MESH_CACHE: RefCell<Vec<CachedGraphicsMesh>> =
        const { RefCell::new(Vec::new()) };
}

pub type RawTriangleMesh = ScalarMesh<RawReal>;
pub type TriangleMeshF32 = ScalarMesh<F32>;
pub type TriangleMeshF64 = ScalarMesh<F64>;
pub type TriangleMeshI128 = ScalarMesh<I128>;

/// Primitive-scalar boundary over native [`TriangleMesh`] geometry.
#[derive(Clone, Debug)]
pub struct ScalarMesh<A>
where
    A: ScalarAdapter,
{
    inner: TriangleMesh,
    _adapter: PhantomData<fn() -> A>,
}

impl<A> ScalarMesh<A>
where
    A: ScalarAdapter,
{
    pub fn from_native(inner: TriangleMesh) -> Self {
        Self {
            inner,
            _adapter: PhantomData,
        }
    }

    pub fn into_native(self) -> TriangleMesh {
        self.inner
    }

    pub const fn native(&self) -> &TriangleMesh {
        &self.inner
    }

    pub fn empty() -> Self {
        Self::from_native(solid::empty())
    }

    pub fn cube(width: A::Scalar) -> AdapterResult<Self> {
        Ok(Self::from_native(solid::cube(A::into_real(width)?)))
    }

    pub fn cuboid(
        width: A::Scalar,
        length: A::Scalar,
        height: A::Scalar,
    ) -> AdapterResult<Self> {
        Ok(Self::from_native(solid::cuboid(
            A::into_real(width)?,
            A::into_real(length)?,
            A::into_real(height)?,
        )))
    }

    pub fn sphere(radius: A::Scalar, segments: usize, stacks: usize) -> AdapterResult<Self> {
        Ok(Self::from_native(solid::sphere(
            A::into_real(radius)?,
            segments,
            stacks,
        )))
    }

    pub fn cylinder(
        radius: A::Scalar,
        height: A::Scalar,
        segments: usize,
    ) -> AdapterResult<Self> {
        Ok(Self::from_native(solid::cylinder(
            A::into_real(radius)?,
            A::into_real(height)?,
            segments,
        )))
    }

    pub fn polyhedron(points: &[[A::Scalar; 3]], faces: &[&[usize]]) -> AdapterResult<Self> {
        let points = points
            .iter()
            .cloned()
            .map(scalar3_to_real::<A>)
            .collect::<AdapterResult<Vec<_>>>()?;
        solid::polyhedron(&points, faces)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn union(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .try_union(&other.inner)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn difference(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .try_difference(&other.inner)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn intersection(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .try_intersection(&other.inner)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn xor(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .try_xor(&other.inner)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn transform(&self, matrix: &Matrix4) -> AdapterResult<Self> {
        solid::try_transform(&self.inner, matrix)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn translate(&self, x: A::Scalar, y: A::Scalar, z: A::Scalar) -> AdapterResult<Self> {
        Ok(Self::from_native(self.inner.translated(
            A::into_real(x)?,
            A::into_real(y)?,
            A::into_real(z)?,
        )))
    }

    pub fn scale(&self, sx: A::Scalar, sy: A::Scalar, sz: A::Scalar) -> AdapterResult<Self> {
        solid::try_scale(
            &self.inner,
            A::into_real(sx)?,
            A::into_real(sy)?,
            A::into_real(sz)?,
        )
        .map(Self::from_native)
        .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn rotate(
        &self,
        x_degrees: A::Scalar,
        y_degrees: A::Scalar,
        z_degrees: A::Scalar,
    ) -> AdapterResult<Self> {
        solid::try_rotate(
            &self.inner,
            A::into_real(x_degrees)?,
            A::into_real(y_degrees)?,
            A::into_real(z_degrees)?,
        )
        .map(Self::from_native)
        .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn inverse(&self) -> Self {
        let triangles = self
            .inner
            .triangles
            .iter()
            .map(|triangle| Triangle::new(triangle.v2, triangle.v1, triangle.v0))
            .collect();
        Self::from_native(TriangleMesh::new(self.inner.positions.to_vec(), triangles))
    }

    pub fn center(&self) -> AdapterResult<Self> {
        solid::try_center(&self.inner)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn float(&self) -> AdapterResult<Self> {
        solid::try_float(&self.inner)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn bounding_box(&self) -> AdapterResult<Aabb3<A::Scalar>> {
        let bounds = solid::try_bounding_box(&self.inner)
            .map_err(|error| AdapterError::Validation(error.to_string()))?;
        Ok(Aabb3 {
            mins: [
                A::from_real(&bounds.mins.x)?,
                A::from_real(&bounds.mins.y)?,
                A::from_real(&bounds.mins.z)?,
            ],
            maxs: [
                A::from_real(&bounds.maxs.x)?,
                A::from_real(&bounds.maxs.y)?,
                A::from_real(&bounds.maxs.z)?,
            ],
        })
    }

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
            .map_err(|error| AdapterError::Validation(error.to_string()))?;
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
                    .map_err(|error| AdapterError::Validation(error.to_string()))?;
                point3_to_scalar::<A>(&transformed)
            })
            .collect()
    }

    pub fn vector_from_adapter(values: [A::Scalar; 3]) -> AdapterResult<Vector3> {
        let [x, y, z] = scalar3_to_real::<A>(values)?;
        Ok(Vector3::from_xyz(x, y, z))
    }
}

fn index_error(error: std::num::TryFromIntError) -> AdapterError {
    AdapterError::Validation(format!("native mesh index exceeds u32: {error}"))
}

fn point3_to_scalar<A: ScalarAdapter>(point: &Point3) -> AdapterResult<[A::Scalar; 3]> {
    Ok([
        A::from_real(&point.x)?,
        A::from_real(&point.y)?,
        A::from_real(&point.z)?,
    ])
}
