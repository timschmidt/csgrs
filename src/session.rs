//! A modeling session that chooses the predicate policy once and keeps the
//! weakest certainty of every operation it runs.
//!
//! The free functions return each operation's receipt in a
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
//! [`Csg::certainty`] is the weakest operation-local receipt of every
//! operation run through the session, including those recorded with
//! [`Csg::record`] or [`Csg::run`]. Once an operation consumes Hyperlimit's
//! 512-bit terminal, the session stays [`GeometryCertainty::Approximate512Consumed`].
//! A later certified step does not certify an earlier approximate one (P9).
//!
//! A mesh created elsewhere and passed in keeps its own inherited
//! qualification in [`TriangleMesh::construction_evidence`]. Every mesh that
//! the session produces carries that inherited evidence forward. Read both
//! claims to judge a whole model: the session for the decisions made in this
//! chain, and the final mesh's construction evidence for its ancestry. Curve
//! regions and query results carry no inherited qualification, so for them the
//! session certainty is the chain's record.
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
/// Kernel failures are kept as the error's [`std::error::Error::source`], and
/// [`ValidationError::kind`] classifies every failure.
pub type Error = ValidationError;

/// Result of a CSGRS modeling operation.
pub type Result<T> = std::result::Result<T, Error>;

/// Exact conversion of an ordinary number into a [`Real`].
///
/// Integers, [`Real`], `&Real` and [`hyperreal::Rational`] convert exactly.
/// `f32` and `f64` convert to the exact value of their binary representation,
/// and NaN or an infinity is an [`Error::InvalidNumber`]. No value is rounded.
///
/// `0.1_f64` is the binary value nearest 1/10, not 1/10 itself. For decimal
/// intent use [`decimal`] (`decimal("0.1")`) or [`ratio`] (`ratio(1, 10)`).
pub trait IntoReal {
    /// Converts the value exactly.
    fn into_real(self) -> Result<Real>;
}

impl IntoReal for Real {
    fn into_real(self) -> Result<Real> {
        Ok(self)
    }
}

impl IntoReal for &Real {
    fn into_real(self) -> Result<Real> {
        Ok(self.clone())
    }
}

impl IntoReal for hyperreal::Rational {
    fn into_real(self) -> Result<Real> {
        Ok(Real::from(self))
    }
}

macro_rules! exact_integer_into_real {
    ($($integer:ty),*) => {
        $(impl IntoReal for $integer {
            fn into_real(self) -> Result<Real> {
                Ok(Real::from(self))
            }
        })*
    };
}

exact_integer_into_real!(i8, i16, i32, i64, i128, u8, u16, u32, u64, u128);

impl IntoReal for f64 {
    fn into_real(self) -> Result<Real> {
        Real::try_from(self).map_err(Error::InvalidNumber)
    }
}

impl IntoReal for f32 {
    fn into_real(self) -> Result<Real> {
        Real::try_from(self).map_err(Error::InvalidNumber)
    }
}

/// The exact ratio `numerator / denominator`; a zero denominator is an error.
pub fn ratio(numerator: i64, denominator: i64) -> Result<Real> {
    (Real::from(numerator) / Real::from(denominator)).map_err(Error::InvalidNumber)
}

/// Parses an exact decimal, fraction or scientific literal, such as `"0.1"`,
/// `"-3/7"` or `"2.5e-3"`, without passing through binary floating point.
pub fn decimal(text: &str) -> Result<Real> {
    text.parse::<Real>().map_err(Error::InvalidNumber)
}

/// A modeling session: one explicit predicate policy and the weakest
/// certainty of every operation run through it.
///
/// See the [module documentation](self) for what the certainty covers.
#[derive(Debug)]
pub struct Csg {
    context: GeometryContext,
    certainty: Cell<GeometryCertainty>,
}

impl Csg {
    /// A session under the selected policy, starting certified.
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
    /// reports approximate only after an operation consumes the terminal.
    pub const fn approximate_512() -> Self {
        Self::new(GeometryContext::APPROXIMATE_512)
    }

    /// The session's predicate policy.
    pub const fn context(&self) -> GeometryContext {
        self.context
    }

    /// The weakest operation-local certainty of every operation so far.
    pub const fn certainty(&self) -> GeometryCertainty {
        self.certainty.get()
    }

    /// Records a receipt and returns its value.
    ///
    /// Use this for an outcome produced outside the session, such as by a
    /// `_with_context` function. It is approximate if either the session or
    /// the outcome already is.
    pub fn record<T>(&self, outcome: GeometryOutcome<T>) -> T {
        self.certainty
            .set(self.certainty.get().combine(outcome.certainty));
        outcome.value
    }

    /// Runs any policy-taking operation under the session's policy and
    /// records its receipt.
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
        operation: impl FnOnce(&GeometryContext) -> std::result::Result<GeometryOutcome<T>, E>,
    ) -> Result<T>
    where
        Error: From<E>,
    {
        Ok(self.record(operation(&self.context)?))
    }

    /// An axis-aligned cube with one corner at the origin.
    pub fn cube(&self, width: impl IntoReal) -> Result<TriangleMesh> {
        solid::cube(width.into_real()?)
    }

    /// An axis-aligned box with one corner at the origin.
    pub fn cuboid(
        &self,
        width: impl IntoReal,
        length: impl IntoReal,
        height: impl IntoReal,
    ) -> Result<TriangleMesh> {
        solid::cuboid(width.into_real()?, length.into_real()?, height.into_real()?)
    }

    /// A sampled sphere centered at the origin.
    pub fn sphere(
        &self,
        radius: impl IntoReal,
        segments: usize,
        stacks: usize,
    ) -> Result<TriangleMesh> {
        solid::sphere(radius.into_real()?, segments, stacks)
    }

    /// A sampled cylinder along +Z with its base at the origin.
    pub fn cylinder(
        &self,
        radius: impl IntoReal,
        height: impl IntoReal,
        segments: usize,
    ) -> Result<TriangleMesh> {
        solid::cylinder(radius.into_real()?, height.into_real()?, segments)
    }

    /// A regularized Boolean of two meshes.
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

    /// Applies a homogeneous transform; a singular transform is an error.
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

    /// Scales by exact per-axis factors.
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

    /// Mirrors across a plane.
    pub fn mirror(&self, mesh: &TriangleMesh, plane: &Plane) -> Result<TriangleMesh> {
        self.run(|context| solid::mirror_with_context(mesh, plane, context))
    }

    /// Evaluates a lazy CSG expression as one Boolean program and records
    /// its receipt. Every operand is arranged once, however many operations
    /// the expression contains.
    pub fn evaluate(&self, expression: &crate::SolidExpr) -> Result<TriangleMesh> {
        self.run(|context| expression.evaluate_with_context(context))
    }
}

#[cfg(feature = "curve")]
impl Csg {
    /// An axis-aligned rectangle with one corner at the origin.
    pub fn rectangle(
        &self,
        width: impl IntoReal,
        length: impl IntoReal,
    ) -> Result<hypercurve::CurveRegion2> {
        let (width, length) = (width.into_real()?, length.into_real()?);
        self.run(|context| crate::curve::rectangle_with_context(width, length, context))
    }

    /// An axis-aligned square with one corner at the origin.
    pub fn square(&self, width: impl IntoReal) -> Result<hypercurve::CurveRegion2> {
        let width = width.into_real()?;
        self.run(|context| crate::curve::square_with_context(width, context))
    }

    /// A sampled circle centered at the origin.
    pub fn circle(
        &self,
        radius: impl IntoReal,
        segments: usize,
    ) -> Result<hypercurve::CurveRegion2> {
        let radius = radius.into_real()?;
        self.run(|context| crate::curve::circle_with_context(radius, segments, context))
    }

    /// Extrudes a region along +Z.
    pub fn extrude(
        &self,
        region: &hypercurve::CurveRegion2,
        height: impl IntoReal,
    ) -> Result<TriangleMesh> {
        let height = height.into_real()?;
        self.run(|context| crate::curve::try_extrude(region, height, context))
    }
}
