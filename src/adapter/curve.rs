//! Host-scalar facade over filled XY regions and their mesh conversions.
//!
//! Geometry stays in [`CurveRegion2`]; scalar conversion happens at ingress and
//! explicit output boundaries. Context-bearing operations retain local predicate
//! receipts. [`ScalarCurve::region_profiles`] is a finite display/interchange
//! projection even when the chosen adapter is [`RawReal`].

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
///
/// Wrapping does not validate or approximate the region. Constructors delegate
/// to the CSG grammar; Booleans delegate directly to Hypercurve. Methods returning
/// a bare region do not expose a separate predicate receipt. Retain outcomes from
/// context-bearing methods when a chain needs to combine local certainty.
/// Scalar conversion failures and native operation failures remain typed.
#[derive(Clone, Debug)]
pub struct ScalarCurve<A>
where
    A: ScalarAdapter,
{
    /// Authoritative region, including its selected exact boundary definitions.
    inner: CurveRegion2,
    /// Selects scalar conversions without storing an adapter instance.
    _adapter: PhantomData<fn() -> A>,
}

/// Region facade with native-real scalar inputs and outputs.
pub type RawCurveRegion = ScalarCurve<RawReal>;
/// Region facade with exact binary32 ingress and lossy scalar egress.
pub type CurveRegionF32 = ScalarCurve<F32>;
/// Region facade with exact binary64 ingress and lossy scalar egress.
pub type CurveRegionF64 = ScalarCurve<F64>;
/// Region facade with exact integer ingress and checked integer egress.
pub type CurveRegionI128 = ScalarCurve<I128>;

impl<A> ScalarCurve<A>
where
    A: ScalarAdapter,
{
    /// Wrap a native region without conversion or additional topology validation.
    pub fn from_native(inner: CurveRegion2) -> Self {
        Self {
            inner,
            _adapter: PhantomData,
        }
    }

    /// Consume the facade and return its unchanged native region.
    pub fn into_native(self) -> CurveRegion2 {
        self.inner
    }

    /// Borrow the authoritative region without scalar conversion.
    pub const fn native(&self) -> &CurveRegion2 {
        &self.inner
    }

    /// Construct the canonical empty filled region.
    pub fn empty() -> Self {
        Self::from_native(curve::empty())
    }

    /// Construct an origin-anchored square under strict predicates.
    ///
    /// The width must be nonnegative; zero gives the canonical empty region.
    /// Scalar import, negative dimensions, and undecided predicates can fail.
    pub fn square(width: A::Scalar) -> AdapterResult<Self> {
        Ok(Self::from_native(curve::square(A::into_real(width)?)?))
    }

    /// Construct an axis-aligned XY rectangle with minimum corner at the origin.
    ///
    /// Both dimensions are validated as nonnegative under strict predicates. A
    /// zero dimension gives the canonical empty region after validation.
    pub fn rectangle(width: A::Scalar, length: A::Scalar) -> AdapterResult<Self> {
        Ok(Self::from_native(curve::rectangle(
            A::into_real(width)?,
            A::into_real(length)?,
        )?))
    }

    /// Construct a regular sampled polygon centered at the origin.
    ///
    /// Requires nonnegative radius and at least three segments, including when
    /// the radius is zero. This is the polygonal region from [`curve::circle`],
    /// not an analytic circular boundary or a certified circle approximation.
    pub fn circle(radius: A::Scalar, segments: usize) -> AdapterResult<Self> {
        Ok(Self::from_native(curve::circle(
            A::into_real(radius)?,
            segments,
        )?))
    }

    /// Import XY rows and construct a filled polygon under strict predicates.
    ///
    /// Requires a valid ring of at least three vertices. Empty or malformed rings
    /// fail; use [`Self::empty`] for absence. See [`curve::polygon`].
    pub fn polygon(points: &[[A::Scalar; 2]]) -> AdapterResult<Self> {
        let points = points
            .iter()
            .cloned()
            .map(scalar2_to_real::<A>)
            .collect::<AdapterResult<Vec<_>>>()?;
        Ok(Self::from_native(curve::polygon(&points)?))
    }

    /// Compute the regularized union through Hypercurve.
    ///
    /// Uses Hypercurve's active evaluation scope (strict outside a provisional or
    /// preview scope) and returns no separate predicate receipt.
    pub fn union(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .boolean_region(&other.inner, hypercurve::BooleanOp::Union)
            .map(Self::from_native)
            .map_err(AdapterError::from)
    }

    /// Compute the regularized part of this region outside `other`.
    ///
    /// Uses the same active Hypercurve policy and receipt contract as [`Self::union`].
    pub fn difference(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .boolean_region(&other.inner, hypercurve::BooleanOp::Difference)
            .map(Self::from_native)
            .map_err(AdapterError::from)
    }

    /// Compute the regularized common material of both regions.
    ///
    /// Uses the same active Hypercurve policy and receipt contract as [`Self::union`].
    pub fn intersection(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .boolean_region(&other.inner, hypercurve::BooleanOp::Intersection)
            .map(Self::from_native)
            .map_err(AdapterError::from)
    }

    /// Compute the regularized material belonging to exactly one operand.
    ///
    /// Uses the same active Hypercurve policy and receipt contract as [`Self::union`].
    pub fn xor(&self, other: &Self) -> AdapterResult<Self> {
        self.inner
            .boolean_region(&other.inner, hypercurve::BooleanOp::Xor)
            .map(Self::from_native)
            .map_err(AdapterError::from)
    }

    /// Apply a strict homogeneous transform whose XY restriction is affine.
    ///
    /// Requires a constant nonzero homogeneous weight and a nonsingular planar
    /// image. Perspective and out-of-plane maps are unsupported; zero weight
    /// can be unbounded or invalid. See [`curve::transformed`]. The separate
    /// operation receipt is discarded.
    pub fn transform(&self, matrix: &Matrix4) -> AdapterResult<Self> {
        self.transform_with_context(matrix, GeometryContext::STRICT)
            .map(GeometryOutcome::into_value)
    }

    /// Translates the region; nonzero Z translation cannot remain planar.
    pub fn translate(&self, x: A::Scalar, y: A::Scalar, z: A::Scalar) -> AdapterResult<Self> {
        self.translate_with_context(x, y, z, GeometryContext::STRICT)
            .map(GeometryOutcome::into_value)
    }

    /// Scales the XY region. Z scale is validated at ingress and has no planar effect.
    pub fn scale(&self, sx: A::Scalar, sy: A::Scalar, sz: A::Scalar) -> AdapterResult<Self> {
        self.scale_with_context(sx, sy, sz, GeometryContext::STRICT)
            .map(GeometryOutcome::into_value)
    }

    /// Applies X, then Y, then Z rotations when their image remains in the XY plane.
    pub fn rotate(
        &self,
        x_degrees: A::Scalar,
        y_degrees: A::Scalar,
        z_degrees: A::Scalar,
    ) -> AdapterResult<Self> {
        self.rotate_with_context(x_degrees, y_degrees, z_degrees, GeometryContext::STRICT)
            .map(GeometryOutcome::into_value)
    }

    /// Compute native bounds under strict predicates, then export their coordinates.
    ///
    /// Algebraically trimmed fragments may use a looser certified source envelope;
    /// see [`curve::bounding_box`]. Returns `None` for an empty region.
    /// Scalar export may fail or be lossy;
    /// [`Aabb3`] is not an outward-enclosure certificate for rounded coordinates.
    pub fn bounding_box(&self) -> AdapterResult<Option<Aabb3<A::Scalar>>> {
        curve::bounding_box(&self.inner)?
            .map(super::convert_bounds::<A>)
            .transpose()
    }

    /// Applies the same planar matrix admission with the selected policy and certainty.
    pub fn transform_with_context(
        &self,
        matrix: &Matrix4,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        curve::transformed_with_context(&self.inner, matrix, context)
            .map(|outcome| outcome.map(Self::from_native))
            .map_err(AdapterError::from)
    }

    /// Translates with the selected policy, rejecting a nonzero Z image.
    pub fn translate_with_context(
        &self,
        x: A::Scalar,
        y: A::Scalar,
        z: A::Scalar,
        context: GeometryContext,
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
        context: GeometryContext,
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

    /// Apply X, then Y, then Z rotations in degrees through planar matrix admission.
    pub fn rotate_with_context(
        &self,
        x: A::Scalar,
        y: A::Scalar,
        z: A::Scalar,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Self>> {
        let x = Matrix4::rotation_x(A::into_real(x)?.to_radians());
        let y = Matrix4::rotation_y(A::into_real(y)?.to_radians());
        let z = Matrix4::rotation_z(A::into_real(z)?.to_radians());
        self.transform_with_context(&(z * y * x), context)
    }

    /// Compute native bounds with the selected policy, then convert their scalars.
    ///
    /// Native bounds follow [`curve::bounding_box`], including potentially loose
    /// envelopes for algebraic trims. The receipt covers native decisions, not
    /// conversion fidelity. `None`
    /// denotes an empty region; rounded bounds need not enclose the source.
    pub fn bounding_box_with_context(
        &self,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Option<Aabb3<A::Scalar>>>> {
        curve::bounding_box_with_context(&self.inner, context)?
            .try_map(|bounds| bounds.map(super::convert_bounds::<A>).transpose())
    }

    /// Publishes a ring with the selected policy after promoting scalar coordinates.
    pub fn polygon_with_context(
        points: &[[A::Scalar; 2]],
        context: GeometryContext,
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

    /// Extrude along Z using an imported, nonzero signed height.
    ///
    /// Returns the local predicate receipt with native mesh geometry. Exact line
    /// pieces and native endpoints are retained; nonlinear interiors may be sampled
    /// with construction qualifications. See [`curve::try_extrude`] for fidelity.
    pub fn extrude(
        &self,
        height: A::Scalar,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<ScalarMesh<A>>> {
        curve::try_extrude(&self.inner, A::into_real(height)?, context)
            .map(|outcome| outcome.map(ScalarMesh::from_native))
            .map_err(AdapterError::from)
    }

    /// Extrude along an imported vector whose Z component must be nonzero.
    ///
    /// Uses the selected predicate policy and the same profile fidelity contract
    /// as [`Self::extrude`]. Unresolved direction or topology decisions fail typed.
    pub fn extrude_vector(
        &self,
        direction: [A::Scalar; 3],
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<ScalarMesh<A>>> {
        let [x, y, z] = scalar3_to_real::<A>(direction)?;
        curve::try_extrude_vector(&self.inner, Vector3::from_xyz(x, y, z), context)
            .map(|outcome| outcome.map(ScalarMesh::from_native))
            .map_err(AdapterError::from)
    }

    /// Build a segmented swept boundary about Y, with profile X as signed radius.
    ///
    /// The angle is in degrees. Requires at least two segments, or three for a
    /// full turn. Zero and oversized noninteger-turn angles are invalid. Partial
    /// sweeps reject axis-crossing loops and may retain overlapping winding.
    /// See [`curve::revolve`] for signed-radius, full-turn and sampling rules.
    /// The mesh is polyhedral, not an analytic surface of revolution.
    pub fn revolve(
        &self,
        angle_degrees: A::Scalar,
        segments: usize,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<ScalarMesh<A>>> {
        curve::revolve(&self.inner, A::into_real(angle_degrees)?, segments, context)
            .map(|outcome| outcome.map(ScalarMesh::from_native))
            .map_err(AdapterError::from)
    }

    /// Build the regularized union of the sampled revolution pieces about Y.
    ///
    /// Uses [`Self::revolve`]'s angular lattice and angle domain, but admits
    /// axis-crossing partial profiles and counts overlapping material once.
    /// Point/edge contacts can remain nonmanifold. See [`curve::revolve_solid`];
    /// its predicate receipt does not certify analytic-surface fidelity.
    pub fn revolve_solid(
        &self,
        angle_degrees: A::Scalar,
        segments: usize,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<ScalarMesh<A>>> {
        curve::revolve_solid(&self.inner, A::into_real(angle_degrees)?, segments, context)
            .map(|outcome| outcome.map(ScalarMesh::from_native))
            .map_err(AdapterError::from)
    }

    /// Export sampled material and hole rings for display or interchange.
    ///
    /// First projects every boundary to `f64` through [`curve::try_finite_profiles`]
    /// using [`curve::SAMPLED_PROFILE_CHORD_ERROR`], then imports those binary values
    /// to `Real` and exports them through `A`. Even `RawReal` therefore describes
    /// the projected points, not the original exact curves. Integer export can fail.
    /// The local predicate receipt does not certify fidelity after scalar export.
    pub fn region_profiles(
        &self,
        context: GeometryContext,
    ) -> AdapterResult<GeometryOutcome<Vec<RegionProfile<A::Scalar>>>> {
        curve::try_finite_profiles(&self.inner, context)
            .map_err(AdapterError::from)?
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

/// One projected material ring with its projected hole rings.
///
/// Produced by [`ScalarCurve::region_profiles`]; these host-scalar coordinates
/// are a finite view, not retained exact curve supports or a new topology proof.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionProfile<S> {
    /// XY vertices of the projected material boundary.
    pub exterior: Vec<[S; 2]>,
    /// XY vertex rings of the projected holes associated with that material.
    pub holes: Vec<Vec<[S; 2]>>,
}

/// Reimport finite-projection coordinates exactly as binary rationals, then export
/// through `A`. This cannot recover the source geometry lost during projection.
fn finite2_to_scalar<A: ScalarAdapter>(point: [f64; 2]) -> AdapterResult<[A::Scalar; 2]> {
    let [x, y] = point;
    real2_to_scalar::<A>(&[
        Real::try_from(x).map_err(AdapterError::from)?,
        Real::try_from(y).map_err(AdapterError::from)?,
    ])
}
