use crate::Aabb3;
use crate::mesh::Mesh;
use crate::scalar::{
    AdapterError, AdapterResult, F32, F64, I128, RawReal, ScalarAdapter, real2_to_scalar,
    scalar2_to_real, scalar3_to_real,
};
use csgrs::csg::CSG;
use csgrs::profile::Profile as CoreProfile;
use hyperlattice::{Matrix4, Vector3};
use hyperreal::Real;
use std::fmt::Debug;
use std::marker::PhantomData;

#[derive(Debug)]
pub struct Profile<A, M>
where
    A: ScalarAdapter,
    M: Clone + Send + Sync + Debug,
{
    inner: CoreProfile<M>,
    _adapter: PhantomData<fn() -> A>,
}

pub type RawProfile<M> = Profile<RawReal, M>;
pub type ProfileF32<M> = Profile<F32, M>;
pub type ProfileF64<M> = Profile<F64, M>;
pub type ProfileI128<M> = Profile<I128, M>;

impl<A, M> Clone for Profile<A, M>
where
    A: ScalarAdapter,
    M: Clone + Send + Sync + Debug,
{
    fn clone(&self) -> Self {
        Self::from_raw(self.inner.clone())
    }
}

impl<A, M> Profile<A, M>
where
    A: ScalarAdapter,
    M: Clone + Send + Sync + Debug,
{
    pub fn from_raw(inner: CoreProfile<M>) -> Self {
        Self {
            inner,
            _adapter: PhantomData,
        }
    }

    pub fn into_raw(self) -> CoreProfile<M> {
        self.inner
    }

    pub fn raw(&self) -> &CoreProfile<M> {
        &self.inner
    }

    pub fn raw_mut(&mut self) -> &mut CoreProfile<M> {
        &mut self.inner
    }

    pub fn empty(metadata: M) -> Self {
        Self::from_raw(CoreProfile::empty(metadata))
    }

    pub fn square(width: A::Scalar, metadata: M) -> AdapterResult<Self> {
        Ok(Self::from_raw(CoreProfile::square(
            A::into_real(width)?,
            metadata,
        )))
    }

    pub fn rectangle(width: A::Scalar, length: A::Scalar, metadata: M) -> AdapterResult<Self> {
        Ok(Self::from_raw(CoreProfile::rectangle(
            A::into_real(width)?,
            A::into_real(length)?,
            metadata,
        )))
    }

    pub fn circle(radius: A::Scalar, segments: usize, metadata: M) -> AdapterResult<Self> {
        Ok(Self::from_raw(CoreProfile::circle(
            A::into_real(radius)?,
            segments,
            metadata,
        )))
    }

    pub fn polygon(points: &[[A::Scalar; 2]], metadata: M) -> AdapterResult<Self> {
        let points = points
            .iter()
            .cloned()
            .map(scalar2_to_real::<A>)
            .collect::<AdapterResult<Vec<_>>>()?;
        Ok(Self::from_raw(CoreProfile::polygon(&points, metadata)))
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

    pub fn extrude(&self, height: A::Scalar) -> AdapterResult<Mesh<A, M>> {
        Ok(Mesh::from_raw(self.inner.extrude(A::into_real(height)?)))
    }

    pub fn extrude_vector(&self, direction: [A::Scalar; 3]) -> AdapterResult<Mesh<A, M>> {
        let [x, y, z] = scalar3_to_real::<A>(direction)?;
        Ok(Mesh::from_raw(
            self.inner.extrude_vector(Vector3::from_xyz(x, y, z)),
        ))
    }

    pub fn revolve(
        &self,
        angle_degrees: A::Scalar,
        segments: usize,
    ) -> AdapterResult<Mesh<A, M>> {
        self.inner
            .revolve(A::into_real(angle_degrees)?, segments)
            .map(Mesh::from_raw)
            .map_err(|err| AdapterError::Validation(err.to_string()))
    }

    pub fn region_profiles(&self) -> AdapterResult<Vec<RegionProfile<A::Scalar>>> {
        self.inner
            .region_profiles()
            .iter()
            .map(|profile| {
                let exterior = profile
                    .material()
                    .points()
                    .iter()
                    .copied()
                    .map(finite2_to_scalar::<A>)
                    .collect::<AdapterResult<Vec<_>>>()?;
                let holes = profile
                    .holes()
                    .iter()
                    .map(|hole| {
                        hole.points()
                            .iter()
                            .copied()
                            .map(finite2_to_scalar::<A>)
                            .collect::<AdapterResult<Vec<_>>>()
                    })
                    .collect::<AdapterResult<Vec<_>>>()?;
                Ok(RegionProfile { exterior, holes })
            })
            .collect()
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct RegionProfile<S> {
    pub exterior: Vec<[S; 2]>,
    pub holes: Vec<Vec<[S; 2]>>,
}

fn finite2_to_scalar<A: ScalarAdapter>(point: [f64; 2]) -> AdapterResult<[A::Scalar; 2]> {
    let [x, y] = point;
    real2_to_scalar::<A>(&[
        Real::try_from(x).map_err(AdapterError::from)?,
        Real::try_from(y).map_err(AdapterError::from)?,
    ])
}
