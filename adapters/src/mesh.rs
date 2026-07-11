use crate::Aabb3;
use crate::scalar::{
    AdapterError, AdapterResult, F32, F64, I128, RawReal, ScalarAdapter, real3_to_scalar,
    scalar3_to_real,
};
use csgrs::csg::CSG;
use csgrs::mesh::Mesh as CoreMesh;
use hyperlattice::{Matrix4, Point3, Vector3};
use std::fmt::Debug;
use std::marker::PhantomData;

pub type MeshVertex<S> = ([S; 3], [S; 3]);

#[derive(Clone, Debug, PartialEq)]
pub struct GraphicsMesh<S> {
    pub vertices: Vec<MeshVertex<S>>,
    pub indices: Vec<u32>,
}

pub type RawMesh<M> = Mesh<RawReal, M>;
pub type MeshF32<M> = Mesh<F32, M>;
pub type MeshF64<M> = Mesh<F64, M>;
pub type MeshI128<M> = Mesh<I128, M>;

#[derive(Debug)]
pub struct Mesh<A, M>
where
    A: ScalarAdapter,
    M: Clone + Send + Sync + Debug,
{
    inner: CoreMesh<M>,
    _adapter: PhantomData<fn() -> A>,
}

impl<A, M> Clone for Mesh<A, M>
where
    A: ScalarAdapter,
    M: Clone + Send + Sync + Debug,
{
    fn clone(&self) -> Self {
        Self::from_raw(self.inner.clone())
    }
}

impl<A, M> Mesh<A, M>
where
    A: ScalarAdapter,
    M: Clone + Send + Sync + Debug,
{
    pub fn from_raw(inner: CoreMesh<M>) -> Self {
        Self {
            inner,
            _adapter: PhantomData,
        }
    }

    pub fn into_raw(self) -> CoreMesh<M> {
        self.inner
    }

    pub fn raw(&self) -> &CoreMesh<M> {
        &self.inner
    }

    pub fn raw_mut(&mut self) -> &mut CoreMesh<M> {
        &mut self.inner
    }

    pub fn empty() -> Self {
        Self::from_raw(CoreMesh::empty())
    }

    pub fn cube(width: A::Scalar, metadata: M) -> AdapterResult<Self> {
        Ok(Self::from_raw(CoreMesh::cube(A::into_real(width)?, metadata)))
    }

    pub fn cuboid(
        width: A::Scalar,
        length: A::Scalar,
        height: A::Scalar,
        metadata: M,
    ) -> AdapterResult<Self> {
        Ok(Self::from_raw(CoreMesh::cuboid(
            A::into_real(width)?,
            A::into_real(length)?,
            A::into_real(height)?,
            metadata,
        )))
    }

    pub fn sphere(
        radius: A::Scalar,
        segments: usize,
        stacks: usize,
        metadata: M,
    ) -> AdapterResult<Self> {
        Ok(Self::from_raw(CoreMesh::sphere(
            A::into_real(radius)?,
            segments,
            stacks,
            metadata,
        )))
    }

    pub fn cylinder(
        radius: A::Scalar,
        height: A::Scalar,
        segments: usize,
        metadata: M,
    ) -> AdapterResult<Self> {
        Ok(Self::from_raw(CoreMesh::cylinder(
            A::into_real(radius)?,
            A::into_real(height)?,
            segments,
            metadata,
        )))
    }

    pub fn polyhedron(
        points: &[[A::Scalar; 3]],
        faces: &[&[usize]],
        metadata: M,
    ) -> AdapterResult<Self> {
        let points = points
            .iter()
            .cloned()
            .map(scalar3_to_real::<A>)
            .collect::<AdapterResult<Vec<_>>>()?;
        CoreMesh::polyhedron(&points, faces, metadata)
            .map(Self::from_raw)
            .map_err(|err| AdapterError::Validation(err.to_string()))
    }

    pub fn union(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .try_union(&other.inner)
            .map(Self::from_raw)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn difference(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .try_difference(&other.inner)
            .map(Self::from_raw)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn intersection(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .try_intersection(&other.inner)
            .map(Self::from_raw)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn xor(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .try_xor(&other.inner)
            .map(Self::from_raw)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn transform(&self, matrix: &Matrix4) -> Self {
        Self::from_raw(self.inner.transform(matrix))
    }

    pub fn translate(&self, x: A::Scalar, y: A::Scalar, z: A::Scalar) -> AdapterResult<Self> {
        Ok(Self::from_raw(self.inner.translate(
            A::into_real(x)?,
            A::into_real(y)?,
            A::into_real(z)?,
        )))
    }

    pub fn scale(&self, sx: A::Scalar, sy: A::Scalar, sz: A::Scalar) -> AdapterResult<Self> {
        Ok(Self::from_raw(self.inner.scale(
            A::into_real(sx)?,
            A::into_real(sy)?,
            A::into_real(sz)?,
        )))
    }

    pub fn rotate(
        &self,
        x_degrees: A::Scalar,
        y_degrees: A::Scalar,
        z_degrees: A::Scalar,
    ) -> AdapterResult<Self> {
        Ok(Self::from_raw(self.inner.rotate(
            A::into_real(x_degrees)?,
            A::into_real(y_degrees)?,
            A::into_real(z_degrees)?,
        )))
    }

    pub fn inverse(&self) -> Self {
        Self::from_raw(self.inner.inverse())
    }

    pub fn center(&self) -> Self {
        Self::from_raw(self.inner.center())
    }

    pub fn float(&self) -> Self {
        Self::from_raw(self.inner.float())
    }

    pub fn bounding_box(&self) -> AdapterResult<Aabb3<A::Scalar>> {
        let bounds = self.inner.bounding_box();
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
        let mesh = self.inner.build_graphics_mesh();
        let vertices = mesh
            .vertices
            .iter()
            .map(|(position, normal)| {
                Ok((real3_to_scalar::<A>(position)?, real3_to_scalar::<A>(normal)?))
            })
            .collect::<AdapterResult<Vec<_>>>()?;

        Ok(GraphicsMesh {
            vertices,
            indices: mesh.indices,
        })
    }

    pub fn vertices_and_indices(&self) -> AdapterResult<(Vec<[A::Scalar; 3]>, Vec<[u32; 3]>)> {
        let (vertices, indices) =
            self.inner.try_get_vertices_and_indices().map_err(|err| {
                AdapterError::Validation(format!("could not extract mesh buffers: {err}"))
            })?;
        let vertices = vertices
            .iter()
            .map(point3_to_scalar::<A>)
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
                    .map_err(|err| AdapterError::Validation(err.to_string()))?;
                point3_to_scalar::<A>(&transformed)
            })
            .collect()
    }

    pub fn vector_from_adapter(values: [A::Scalar; 3]) -> AdapterResult<Vector3> {
        let [x, y, z] = scalar3_to_real::<A>(values)?;
        Ok(Vector3::from_xyz(x, y, z))
    }
}

fn point3_to_scalar<A: ScalarAdapter>(point: &Point3) -> AdapterResult<[A::Scalar; 3]> {
    Ok([
        A::from_real(&point.x)?,
        A::from_real(&point.y)?,
        A::from_real(&point.z)?,
    ])
}
