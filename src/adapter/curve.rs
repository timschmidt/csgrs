use super::Aabb3;
use super::mesh::ScalarMesh;
use super::scalar::{
    AdapterError, AdapterResult, F32, F64, I128, RawReal, ScalarAdapter, real2_to_scalar,
    scalar2_to_real, scalar3_to_real,
};
use crate::curve::{self, CurveRegionExt};
use hypercurve::CurveRegion2;
use hyperlattice::{Matrix4, Vector3};
use hyperreal::Real;
use std::marker::PhantomData;

/// Primitive-scalar boundary over a native filled Hypercurve region.
#[derive(Clone, Debug)]
pub struct ScalarCurve<A>
where
    A: ScalarAdapter,
{
    inner: CurveRegion2,
    _adapter: PhantomData<fn() -> A>,
}

pub type RawCurveRegion = ScalarCurve<RawReal>;
pub type CurveRegionF32 = ScalarCurve<F32>;
pub type CurveRegionF64 = ScalarCurve<F64>;
pub type CurveRegionI128 = ScalarCurve<I128>;

impl<A> ScalarCurve<A>
where
    A: ScalarAdapter,
{
    pub fn from_native(inner: CurveRegion2) -> Self {
        Self {
            inner,
            _adapter: PhantomData,
        }
    }

    pub fn into_native(self) -> CurveRegion2 {
        self.inner
    }

    pub const fn native(&self) -> &CurveRegion2 {
        &self.inner
    }

    pub fn empty() -> Self {
        Self::from_native(curve::empty())
    }

    pub fn square(width: A::Scalar) -> AdapterResult<Self> {
        Ok(Self::from_native(curve::square(A::into_real(width)?)))
    }

    pub fn rectangle(width: A::Scalar, length: A::Scalar) -> AdapterResult<Self> {
        Ok(Self::from_native(curve::rectangle(
            A::into_real(width)?,
            A::into_real(length)?,
        )))
    }

    pub fn circle(radius: A::Scalar, segments: usize) -> AdapterResult<Self> {
        Ok(Self::from_native(curve::circle(
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
        Ok(Self::from_native(curve::polygon(&points)))
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
        curve::try_transformed(&self.inner, matrix)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn translate(&self, x: A::Scalar, y: A::Scalar, _z: A::Scalar) -> AdapterResult<Self> {
        curve::try_translated(&self.inner, A::into_real(x)?, A::into_real(y)?)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn scale(&self, sx: A::Scalar, sy: A::Scalar, _sz: A::Scalar) -> AdapterResult<Self> {
        curve::try_scaled(&self.inner, A::into_real(sx)?, A::into_real(sy)?)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn rotate(
        &self,
        x_degrees: A::Scalar,
        y_degrees: A::Scalar,
        z_degrees: A::Scalar,
    ) -> AdapterResult<Self> {
        let _ = (A::into_real(x_degrees)?, A::into_real(y_degrees)?);
        curve::try_rotated(&self.inner, A::into_real(z_degrees)?)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn bounding_box(&self) -> AdapterResult<Aabb3<A::Scalar>> {
        let bounds = curve::try_bounding_box(&self.inner)
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

    pub fn extrude(&self, height: A::Scalar) -> AdapterResult<ScalarMesh<A>> {
        curve::try_extrude(&self.inner, A::into_real(height)?)
            .map(ScalarMesh::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn extrude_vector(&self, direction: [A::Scalar; 3]) -> AdapterResult<ScalarMesh<A>> {
        let [x, y, z] = scalar3_to_real::<A>(direction)?;
        curve::try_extrude_vector(&self.inner, Vector3::from_xyz(x, y, z))
            .map(ScalarMesh::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn revolve(
        &self,
        angle_degrees: A::Scalar,
        segments: usize,
    ) -> AdapterResult<ScalarMesh<A>> {
        curve::revolve(&self.inner, A::into_real(angle_degrees)?, segments)
            .map(ScalarMesh::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn region_profiles(&self) -> AdapterResult<Vec<RegionProfile<A::Scalar>>> {
        curve::try_finite_profiles(&self.inner)
            .map_err(|error| AdapterError::Validation(error.to_string()))?
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
