//! A modeling session that selects a policy and accumulates successful operation
//! receipts without wrapping the native geometry carriers.
//!
//! Context-bearing free functions return each operation's receipt in a
//! [`GeometryOutcome`]. A chain that calls [`GeometryOutcome::into_value`], or
//! the policy-free convenience functions, discards those receipts. [`Csg`]
//! makes keeping them the default:
//!
//! ```
//! use csgrs::{Csg, GeometryCertainty};
//!
//! # fn main() -> csgrs::Result<()> {
//! let m = Csg::strict();
//! let body = m.cube(12)?;
//! # #[cfg(feature = "curve")] {
//! let hole = m.translate(&m.extrude(&m.square(4)?, 16)?, 4, 4, -2)?;
//! let part = m.difference(&body, &hole)?;
//! assert!(!part.triangles.is_empty());
//! # }
//! assert_eq!(m.certainty(), GeometryCertainty::Certified);
//! # Ok(())
//! # }
//! ```
//!
//! # What the session certainty covers
//!
//! [`Csg::certainty`] combines the operation-local receipts supplied to
//! [`Csg::record`], including successful results of [`Csg::run`]. Once a recorded
//! receipt consumed Hyperlimit's 512-bit terminal, the session stays
//! [`GeometryCertainty::Approximate512Consumed`]. A later certified receipt does
//! not certify an earlier approximate one. An operation returning an error supplies
//! no receipt to `run`; earlier recorded outcomes are not rolled back.
//!
//! Most modeling methods forward the session policy and record the returned
//! receipt. The solid constructors [`Csg::cube`], [`Csg::cuboid`], [`Csg::sphere`]
//! and [`Csg::cylinder`] instead call the strict grammar functions directly. They
//! do not use the selected approximate policy or add a receipt to the accumulator.
//! A certified session receipt does not certify sampling fidelity or topology
//! independently of the operation that produced the geometry.
//!
//! A mesh created elsewhere keeps its inherited qualifications in
//! [`TriangleMesh::construction_evidence`]. Native modeling operations propagate
//! that evidence separately from the session's local receipts. Read both claims
//! when judging a chain. Generic `record` and `run` do not inspect the value or
//! attach history: callers supplying arbitrary outcomes must preserve it themselves.
//! For values without inherited history, retain the relevant receipts explicitly;
//! a new session cannot reconstruct decisions made outside it.
//!
//! A session is not `Sync`; use one session per thread and combine their
//! certainties with [`GeometryCertainty::combine`].

use std::cell::Cell;

use hyperlattice::Matrix4;
use hypermesh::{BooleanOp, Plane};

use crate::context::{GeometryCertainty, GeometryContext, GeometryOutcome};
use crate::errors::ValidationError;
use crate::{Real, TriangleMesh, solid};

/// The single public error type of CSGRS modeling operations.
///
/// Typed kernel failures remain in the error's variant payloads; source-chain
/// exposure follows [`ValidationError`]'s variant-specific contract.
/// [`ValidationError::kind`] supplies a coarser category for callers and bindings.
pub type Error = ValidationError;

/// Result of a CSGRS modeling operation.
pub type Result<T> = std::result::Result<T, Error>;

/// Exact conversion of an ordinary number into a [`Real`].
///
/// Fixed-width integers from `i8`/`u8` through `i128`/`u128`, [`Real`],
/// `&Real` and [`hyperreal::Rational`] have exact built-in conversions.
/// `f32` and `f64` convert to the exact value of their binary representation,
/// and NaN or an infinity is an [`Error::InvalidNumber`]. No value is rounded.
///
/// `0.1_f64` is the binary value nearest 1/10, not 1/10 itself. For decimal
/// intent use [`decimal`] (`decimal("0.1")`) or [`ratio`] (`ratio(1, 10)`).
pub trait IntoReal {
    /// Convert without changing the represented numeric value.
    ///
    /// Implementations must reject inputs without a finite exact real value;
    /// custom implementations are responsible for honoring this contract.
    fn into_real(self) -> Result<Real>;
}

impl IntoReal for Real {
    /// Move an existing exact scalar without conversion or failure.
    fn into_real(self) -> Result<Real> {
        Ok(self)
    }
}

impl IntoReal for &Real {
    /// Clone an existing exact scalar without numerical approximation.
    fn into_real(self) -> Result<Real> {
        Ok(self.clone())
    }
}

impl IntoReal for hyperreal::Rational {
    /// Preserve the rational exactly in the native scalar representation.
    fn into_real(self) -> Result<Real> {
        Ok(Real::from(self))
    }
}

/// Generate infallible exact conversions for the listed fixed-width integer types.
/// Pointer-sized integers are not included in this invocation's public domain.
macro_rules! exact_integer_into_real {
    ($($integer:ty),*) => {
        $(impl IntoReal for $integer {
            /// Preserve this fixed-width integer exactly as a native real.
            fn into_real(self) -> Result<Real> {
                Ok(Real::from(self))
            }
        })*
    };
}

exact_integer_into_real!(i8, i16, i32, i64, i128, u8, u16, u32, u64, u128);

impl IntoReal for f64 {
    /// Import the exact finite binary64 value; map NaN or infinity to `InvalidNumber`.
    fn into_real(self) -> Result<Real> {
        Real::try_from(self).map_err(Error::InvalidNumber)
    }
}

impl IntoReal for f32 {
    /// Import the exact finite binary32 value; map NaN or infinity to `InvalidNumber`.
    fn into_real(self) -> Result<Real> {
        Real::try_from(self).map_err(Error::InvalidNumber)
    }
}

/// Construct the exact ratio `numerator / denominator` from signed 64-bit inputs.
///
/// A zero denominator returns [`Error::InvalidNumber`]; negative denominators
/// are allowed. The division does not pass through floating point.
pub fn ratio(numerator: i64, denominator: i64) -> Result<Real> {
    (Real::from(numerator) / Real::from(denominator)).map_err(Error::InvalidNumber)
}

/// Parses an exact decimal, fraction or scientific literal, such as `"0.1"`,
/// `"-3/7"` or `"2.5e-3"`, without passing through binary floating point.
///
/// Delegates to `Real` parsing and wraps every parsing failure as
/// [`Error::InvalidNumber`]. Large scientific exponents may be retained lazily;
/// successful parsing does not promise cheap downstream evaluation.
pub fn decimal(text: &str) -> Result<Real> {
    text.parse::<Real>().map_err(Error::InvalidNumber)
}

/// A selected modeling policy and a monotone accumulator of supplied receipts.
///
/// The accumulator covers successful recorded outcomes, not arbitrary work on
/// values later passed to the session. Strict solid constructors bypass it.
///
/// See the [module documentation](self) for what the certainty covers.
#[derive(Debug)]
pub struct Csg {
    /// Immutable policy forwarded to context-bearing operations.
    context: GeometryContext,
    /// Weakest recorded receipt; interior mutation allows operations through `&self`.
    certainty: Cell<GeometryCertainty>,
}

impl Csg {
    /// Start a session under the selected policy with no approximate receipts.
    ///
    /// The initial `Certified` state describes an empty receipt history; it
    /// does not validate any model or recover prior construction evidence.
    pub const fn new(context: GeometryContext) -> Self {
        Self {
            context,
            certainty: Cell::new(GeometryCertainty::Certified),
        }
    }

    /// A session under [`GeometryContext::STRICT`].
    pub const fn strict() -> Self {
        Self::new(GeometryContext::STRICT)
    }

    /// A session under [`GeometryContext::APPROXIMATE_512`]. Selecting this
    /// policy does not by itself make a result approximate; the session
    /// reports approximate only after recording a receipt that consumed the terminal.
    pub const fn approximate_512() -> Self {
        Self::new(GeometryContext::APPROXIMATE_512)
    }

    /// Return the policy forwarded by this session's context-bearing operations.
    pub const fn context(&self) -> GeometryContext {
        self.context
    }

    /// Return the weakest receipt recorded so far, without inspecting geometry.
    ///
    /// Failed calls supply no receipt; strict primitive constructors do not
    /// update this value. See the [module documentation](self).
    pub const fn certainty(&self) -> GeometryCertainty {
        self.certainty.get()
    }

    /// Records a receipt and returns its value.
    ///
    /// Use this for an outcome produced outside the session, such as by a
    /// `_with_context` function. It is approximate if either the session or
    /// the outcome already is. This trusts the supplied receipt, does not
    /// verify that it was produced under this session's policy, and does not
    /// inspect or modify the value's construction history.
    pub fn record<T>(&self, outcome: GeometryOutcome<T>) -> T {
        self.certainty
            .set(self.certainty.get().combine(outcome.certainty));
        outcome.value
    }

    /// Pass the session policy to a callback and record its successful receipt.
    ///
    /// The callback is responsible for using that policy and preserving evidence
    /// in its value. An error converts through `Error: From<E>` and supplies no
    /// receipt; any earlier or explicitly nested records remain in the session.
    ///
    /// ```
    /// # use csgrs::{Csg, solid};
    /// # fn main() -> csgrs::Result<()> {
    /// let m = Csg::strict();
    /// let cube = m.cube(2)?;
    /// let centered = m.run(|context| solid::center_with_context(&cube, context))?;
    /// # assert!(!centered.triangles.is_empty());
    /// # Ok(())
    /// # }
    /// ```
    pub fn run<T, E>(
        &self,
        operation: impl FnOnce(GeometryContext) -> std::result::Result<GeometryOutcome<T>, E>,
    ) -> Result<T>
    where
        Error: From<E>,
    {
        Ok(self.record(operation(self.context)?))
    }

    /// Construct an origin-anchored cube with nonnegative width.
    ///
    /// Uses strict [`solid::cube`] regardless of session policy and does not
    /// update the receipt accumulator. Zero width yields canonical empty geometry;
    /// negative or undecided width and invalid number imports fail.
    pub fn cube(&self, width: impl IntoReal) -> Result<TriangleMesh> {
        solid::cube(width.into_real()?)
    }

    /// Construct an origin-anchored box with nonnegative XYZ dimensions.
    ///
    /// Uses strict [`solid::cuboid`] regardless of session policy and does not
    /// update the accumulator. All dimensions are validated; a zero dimension
    /// yields canonical empty geometry after validation.
    pub fn cuboid(
        &self,
        width: impl IntoReal,
        length: impl IntoReal,
        height: impl IntoReal,
    ) -> Result<TriangleMesh> {
        solid::cuboid(width.into_real()?, length.into_real()?, height.into_real()?)
    }

    /// Construct a tessellated sphere centered at the origin.
    ///
    /// Requires nonnegative radius, at least three segments and two stacks.
    /// Uses strict [`solid::sphere`] without updating the session accumulator.
    /// Sampling qualifications remain on the mesh; it is not an analytic sphere.
    pub fn sphere(
        &self,
        radius: impl IntoReal,
        segments: usize,
        stacks: usize,
    ) -> Result<TriangleMesh> {
        solid::sphere(radius.into_real()?, segments, stacks)
    }

    /// Construct a tessellated cylinder from the origin to Z = `height`.
    ///
    /// Requires nonnegative radius and at least three segments; height may be
    /// signed. Uses strict [`solid::cylinder`] without updating the session
    /// accumulator. Radius or axis collapse produces empty geometry after
    /// validation; the mesh retains sampling qualifications.
    pub fn cylinder(
        &self,
        radius: impl IntoReal,
        height: impl IntoReal,
        segments: usize,
    ) -> Result<TriangleMesh> {
        solid::cylinder(radius.into_real()?, height.into_real()?, segments)
    }

    /// Compute one regularized mesh Boolean under the session policy.
    ///
    /// Records the successful local receipt and preserves inherited mesh
    /// qualifications through [`solid::boolean_with_context`]. Empty operands
    /// remain distinct from malformed inputs; native failures remain typed.
    pub fn boolean(
        &self,
        left: &TriangleMesh,
        right: &TriangleMesh,
        operation: BooleanOp,
    ) -> Result<TriangleMesh> {
        self.run(|context| solid::boolean_with_context(left, right, operation, context))
    }

    /// The regularized union of two meshes.
    pub fn union(&self, left: &TriangleMesh, right: &TriangleMesh) -> Result<TriangleMesh> {
        self.boolean(left, right, BooleanOp::Union)
    }

    /// The regularized difference `left - right`.
    pub fn difference(
        &self,
        left: &TriangleMesh,
        right: &TriangleMesh,
    ) -> Result<TriangleMesh> {
        self.boolean(left, right, BooleanOp::Difference)
    }

    /// The regularized intersection of two meshes.
    pub fn intersection(
        &self,
        left: &TriangleMesh,
        right: &TriangleMesh,
    ) -> Result<TriangleMesh> {
        self.boolean(left, right, BooleanOp::Intersection)
    }

    /// The regularized symmetric difference of two meshes.
    pub fn xor(&self, left: &TriangleMesh, right: &TriangleMesh) -> Result<TriangleMesh> {
        self.boolean(left, right, BooleanOp::SymmetricDifference)
    }

    /// Apply a checked homogeneous transform and record its successful receipt.
    ///
    /// Uses [`solid::transform_with_context`] with the session policy, preserving
    /// inherited qualifications and orientation. Singular transforms fail;
    /// projective failures remain typed rather than publishing an invalid mesh.
    pub fn transform(&self, mesh: &TriangleMesh, matrix: &Matrix4) -> Result<TriangleMesh> {
        self.run(|context| solid::transform_with_context(mesh, matrix, context))
    }

    /// Translates by an exact vector.
    pub fn translate(
        &self,
        mesh: &TriangleMesh,
        x: impl IntoReal,
        y: impl IntoReal,
        z: impl IntoReal,
    ) -> Result<TriangleMesh> {
        let matrix =
            Matrix4::affine_translation([x.into_real()?, y.into_real()?, z.into_real()?]);
        self.transform(mesh, &matrix)
    }

    /// Rotates by exact angles in degrees about X, then Y, then Z.
    pub fn rotate(
        &self,
        mesh: &TriangleMesh,
        x_degrees: impl IntoReal,
        y_degrees: impl IntoReal,
        z_degrees: impl IntoReal,
    ) -> Result<TriangleMesh> {
        let (x, y, z) = (
            x_degrees.into_real()?,
            y_degrees.into_real()?,
            z_degrees.into_real()?,
        );
        self.run(|context| solid::rotate_with_context(mesh, x, y, z, context))
    }

    /// Scale by imported per-axis factors and record the successful receipt.
    ///
    /// Zero axes are invalid; unresolved signs fail under the session policy.
    /// Reflections correct winding and preserve inherited mesh qualifications.
    pub fn scale(
        &self,
        mesh: &TriangleMesh,
        x: impl IntoReal,
        y: impl IntoReal,
        z: impl IntoReal,
    ) -> Result<TriangleMesh> {
        let (x, y, z) = (x.into_real()?, y.into_real()?, z.into_real()?);
        self.run(|context| solid::scale_with_context(mesh, x, y, z, context))
    }

    /// Reflect across a plane using the session policy and record the receipt.
    ///
    /// Invalid planes or undecided construction fail through
    /// [`solid::mirror_with_context`]. Reflections preserve oriented material.
    pub fn mirror(&self, mesh: &TriangleMesh, plane: &Plane) -> Result<TriangleMesh> {
        self.run(|context| solid::mirror_with_context(mesh, plane, context))
    }

    /// Evaluate a lazy expression and record its successful receipt.
    ///
    /// Nonempty lowered operands share one Boolean program and arrangement;
    /// an all-empty expression needs no arrangement. Subtree reuse depends on
    /// both identity and enclosing transforms. Failures retain an expression
    /// path through [`crate::SolidExpr::evaluate_with_context`].
    pub fn evaluate(&self, expression: &crate::SolidExpr) -> Result<TriangleMesh> {
        self.run(|context| expression.evaluate_with_context(context))
    }
}

#[cfg(feature = "curve")]
impl Csg {
    /// Construct an origin-anchored XY rectangle under the session policy.
    ///
    /// Dimensions must be nonnegative; validated zero dimensions produce empty
    /// geometry. Records the receipt from `curve::rectangle_with_context`.
    pub fn rectangle(
        &self,
        width: impl IntoReal,
        length: impl IntoReal,
    ) -> Result<hypercurve::CurveRegion2> {
        let (width, length) = (width.into_real()?, length.into_real()?);
        self.run(|context| crate::curve::rectangle_with_context(width, length, context))
    }

    /// Construct an origin-anchored XY square under the session policy.
    ///
    /// Width must be nonnegative; zero produces empty geometry. Records the
    /// successful constructor receipt.
    pub fn square(&self, width: impl IntoReal) -> Result<hypercurve::CurveRegion2> {
        let width = width.into_real()?;
        self.run(|context| crate::curve::square_with_context(width, context))
    }

    /// Construct a regular sampled XY polygon centered at the origin.
    ///
    /// Requires nonnegative radius and at least three segments, including at
    /// zero radius. Records the local predicate receipt under the session policy;
    /// that receipt does not certify fidelity to an analytic circle.
    pub fn circle(
        &self,
        radius: impl IntoReal,
        segments: usize,
    ) -> Result<hypercurve::CurveRegion2> {
        let radius = radius.into_real()?;
        self.run(|context| crate::curve::circle_with_context(radius, segments, context))
    }

    /// Extrude along Z by a nonzero signed height under the session policy.
    ///
    /// Negative heights extrude toward -Z; zero or undecided direction fails.
    /// Exact line pieces and native endpoints remain authoritative, while
    /// nonlinear interiors may be sampled with mesh construction qualifications.
    /// The recorded local receipt does not certify analytic-source fidelity.
    pub fn extrude(
        &self,
        region: &hypercurve::CurveRegion2,
        height: impl IntoReal,
    ) -> Result<TriangleMesh> {
        let height = height.into_real()?;
        self.run(|context| crate::curve::try_extrude(region, height, context))
    }
}
