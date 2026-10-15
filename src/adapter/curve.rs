use super::Aabb3;
use super::mesh::ScalarMesh;
use super::scalar::{
    AdapterError, AdapterResult, F32, F64, I128, RawReal, ScalarAdapter, real2_to_scalar,
    scalar2_to_real, scalar3_to_real,
};
use crate::curve;
use crate::{GeometryContext, GeometryOutcome};
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
        Ok(Self::from_native(curve::square(A::into_real(width)?)?))
    }

    pub fn rectangle(width: A::Scalar, length: A::Scalar) -> AdapterResult<Self> {
        Ok(Self::from_native(curve::rectangle(
            A::into_real(width)?,
            A::into_real(length)?,
        )?))
    }

    pub fn circle(radius: A::Scalar, segments: usize) -> AdapterResult<Self> {
        Ok(Self::from_native(curve::circle(
            A::into_real(radius)?,
            segments,
        )?))
    }

    pub fn polygon(points: &[[A::Scalar; 2]]) -> AdapterResult<Self> {
        let points = points
            .iter()
            .cloned()
            .map(scalar2_to_real::<A>)
            .collect::<AdapterResult<Vec<_>>>()?;
        Ok(Self::from_native(curve::polygon(&points)?))
    }

    pub fn union(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .boolean_region(&other.inner, hypercurve::BooleanOp::Union)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn difference(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .boolean_region(&other.inner, hypercurve::BooleanOp::Difference)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn intersection(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .boolean_region(&other.inner, hypercurve::BooleanOp::Intersection)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn xor(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .boolean_region(&other.inner, hypercurve::BooleanOp::Xor)
            .map(Self::from_native)
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    /// Applies a checked transform whose image remains in the XY plane.
    pub fn transform(&self, matrix: &Matrix4) -> AdapterResult<Self> {
        self.transform_with_context(matrix, &GeometryContext::STRICT)
            .map(GeometryOutcome::into_value)
    }

    /// Translates the region; nonzero Z translation cannot remain planar.
    pub fn translate(&self, x: A::Scalar, y: A::Scalar, z: A::Scalar) -> AdapterResult<Self> {
        self.translate_with_context(x, y, z, &GeometryContext::STRICT)
            .map(GeometryOutcome::into_value)
    }

    /// Scales the XY region. Z scale is validated at ingress and has no planar effect.
    pub fn scale(&self, sx: A::Scalar, sy: A::Scalar, sz: A::Scalar) -> AdapterResult<Self> {
        self.scale_with_context(sx, sy, sz, &GeometryContext::STRICT)
            .map(GeometryOutcome::into_value)
    }

    /// Applies X, then Y, then Z rotations when their image remains in the XY plane.
    pub fn rotate(
        &self,
        x_degrees: A::Scalar,
        y_degrees: A::Scalar,
        z_degrees: A::Scalar,
    ) -> AdapterResult<Self> {
        self.rotate_with_context(x_degrees, y_degrees, z_degrees, &GeometryContext::STRICT)
            .map(GeometryOutcome::into_value)
    }

    pub fn bounding_box(&self) -> AdapterResult<Option<Aabb3<A::Scalar>>> {
        curve::try_bounding_box(&self.inner)?
            .map(super::convert_bounds::<A>)
            .transpose()
    }

    /// Applies the same planar matrix admission with the selected policy and certainty.
    pub fn transform_with_context(
        &self,
        matrix: &Matrix4,
        context: &GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        curve::try_transformed_with_context(&self.inner, matrix, context)
            .map(|outcome| outcome.map(Self::from_native))
            .map_err(AdapterError::from)
    }

    /// Translates with the selected policy, rejecting a nonzero Z image.
    pub fn translate_with_context(
        &self,
        x: A::Scalar,
        y: A::Scalar,
        z: A::Scalar,
        context: &GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        self.transform_with_context(
            &Matrix4::affine_translation([
                A::into_real(x)?,
                A::into_real(y)?,
                A::into_real(z)?,
            ]),
            context,
        )
    }

    /// Scales with the selected policy after validating all three scalar inputs.
    pub fn scale_with_context(
        &self,
        x: A::Scalar,
        y: A::Scalar,
        z: A::Scalar,
        context: &GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        self.transform_with_context(
            &Matrix4::affine_nonuniform_scale([
                A::into_real(x)?,
                A::into_real(y)?,
                A::into_real(z)?,
            ]),
            context,
        )
    }

    /// Applies X/Y/Z rotations through shared planar matrix admission.
    pub fn rotate_with_context(
        &self,
        x: A::Scalar,
        y: A::Scalar,
        z: A::Scalar,
        context: &GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        let x = Matrix4::rotation_x(A::into_real(x)?.to_radians());
        let y = Matrix4::rotation_y(A::into_real(y)?.to_radians());
        let z = Matrix4::rotation_z(A::into_real(z)?.to_radians());
        self.transform_with_context(&(z * y * x), context)
    }

    /// Computes bounds before scalar conversion, retaining predicate certainty.
    pub fn bounding_box_with_context(
        &self,
        context: &GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Option<Aabb3<A::Scalar>>>> {
        curve::try_bounding_box_with_context(&self.inner, context)?
            .try_map(|bounds| bounds.map(super::convert_bounds::<A>).transpose())
    }

    /// Publishes a ring with the selected policy after promoting scalar coordinates.
    pub fn polygon_with_context(
        points: &[[A::Scalar; 2]],
        context: &GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        let points = points
            .iter()
            .cloned()
            .map(scalar2_to_real::<A>)
            .collect::<AdapterResult<Vec<_>>>()?;
        curve::polygon_with_context(&points, context)
            .map(|outcome| outcome.map(Self::from_native))
            .map_err(AdapterError::from)
    }

    pub fn extrude(
        &self,
        height: A::Scalar,
        context: &GeometryContext,
    ) -> AdapterResult<GeometryOutcome<ScalarMesh<A>>> {
        curve::try_extrude(&self.inner, A::into_real(height)?, context)
            .map(|outcome| outcome.map(ScalarMesh::from_native))
            .map_err(AdapterError::from)
    }

    pub fn extrude_vector(
        &self,
        direction: [A::Scalar; 3],
        context: &GeometryContext,
    ) -> AdapterResult<GeometryOutcome<ScalarMesh<A>>> {
        let [x, y, z] = scalar3_to_real::<A>(direction)?;
        curve::try_extrude_vector(&self.inner, Vector3::from_xyz(x, y, z), context)
            .map(|outcome| outcome.map(ScalarMesh::from_native))
            .map_err(AdapterError::from)
    }

    pub fn revolve(
        &self,
        angle_degrees: A::Scalar,
        segments: usize,
        context: &GeometryContext,
    ) -> AdapterResult<GeometryOutcome<ScalarMesh<A>>> {
        curve::revolve(&self.inner, A::into_real(angle_degrees)?, segments, context)
            .map(|outcome| outcome.map(ScalarMesh::from_native))
            .map_err(|error| AdapterError::Validation(error.to_string()))
    }

    pub fn region_profiles(
        &self,
        context: &GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Vec<RegionProfile<A::Scalar>>>> {
        curve::try_finite_profiles(&self.inner, context)
            .map_err(|error| AdapterError::Validation(error.to_string()))?
            .try_map(|profiles| {
                profiles
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
            })
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
