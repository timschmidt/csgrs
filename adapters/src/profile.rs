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
pub struct Profile<A>
where
    A: ScalarAdapter,
{
    inner: CoreProfile,
    _adapter: PhantomData<fn() -> A>,
}

pub type RawProfile = Profile<RawReal>;
pub type ProfileF32 = Profile<F32>;
pub type ProfileF64 = Profile<F64>;
pub type ProfileI128 = Profile<I128>;

impl<A> Clone for Profile<A>
where
    A: ScalarAdapter,
{
    fn clone(&self) -> Self {
        Self::from_raw(self.inner.clone())
    }
}

impl<A> Profile<A>
where
    A: ScalarAdapter,
{
    pub fn from_raw(inner: CoreProfile) -> Self {
        Self {
            inner,
            _adapter: PhantomData,
        }
    }

    pub fn into_raw(self) -> CoreProfile {
        self.inner
    }

    pub const fn raw(&self) -> &CoreProfile {
        &self.inner
    }

    pub const fn raw_mut(&mut self) -> &mut CoreProfile {
        &mut self.inner
    }

    pub fn empty() -> Self {
        Self::from_raw(CoreProfile::empty())
    }

    pub fn square(width: A::Scalar) -> AdapterResult<Self> {
        Ok(Self::from_raw(CoreProfile::square(A::into_real(width)?)))
    }

    pub fn rectangle(width: A::Scalar, length: A::Scalar) -> AdapterResult<Self> {
        Ok(Self::from_raw(CoreProfile::rectangle(
            A::into_real(width)?,
            A::into_real(length)?,
        )))
    }

    pub fn circle(radius: A::Scalar, segments: usize) -> AdapterResult<Self> {
        Ok(Self::from_raw(CoreProfile::circle(
            A::into_real(radius)?,
            segments,
        )))
    }

    pub fn polygon(points: &[[A::Scalar; 2]]) -> AdapterResult<Self> {
        let points = points
            .iter()
            .cloned()
            .map(scalar2_to_real::<A>)
            .collect::<AdapterResult<Vec<_>>>()?;
        Ok(Self::from_raw(CoreProfile::polygon(&points)))
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

    pub fn extrude<M>(&self, height: A::Scalar, metadata: M) -> AdapterResult<Mesh<A, M>>
    where
        M: Clone + Send + Sync + Debug,
    {
        Ok(Mesh::from_raw(
            self.inner.extrude(A::into_real(height)?, metadata),
        ))
    }

    pub fn extrude_vector<M>(
        &self,
        direction: [A::Scalar; 3],
        metadata: M,
    ) -> AdapterResult<Mesh<A, M>>
    where
        M: Clone + Send + Sync + Debug,
    {
        let [x, y, z] = scalar3_to_real::<A>(direction)?;
        Ok(Mesh::from_raw(
            self.inner
                .extrude_vector(Vector3::from_xyz(x, y, z), metadata),
        ))
    }

    pub fn revolve<M>(
        &self,
        angle_degrees: A::Scalar,
        segments: usize,
        metadata: M,
    ) -> AdapterResult<Mesh<A, M>>
    where
        M: Clone + Send + Sync + Debug,
    {
        self.inner
            .revolve(A::into_real(angle_degrees)?, segments, metadata)
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
