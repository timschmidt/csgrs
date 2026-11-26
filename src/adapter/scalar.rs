//! Scalar conversion contracts for the native-geometry adapters.
//!
//! Built-in adapters import finite primitives exactly. Float exports are lossy
//! views for display or interchange, not evidence for geometric decisions.
//! [`RawReal`] preserves the native scalar; [`I128`] requires an exactly
//! represented rational integer on export.

use hyperreal::{Problem, Rational, Real};
use std::fmt;

/// Result of scalar conversion or a modeling operation through an adapter.
pub type AdapterResult<T> = Result<T, AdapterError>;

/// Failure at a scalar, geometry, or graphics-buffer adapter boundary.
///
/// Native scalar and modeling failures retain their typed sources. A textual
/// validation failure has no structured source.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum AdapterError {
    /// Native scalar conversion failure, including invalid floats or integer range errors.
    Hyperreal(
        /// Original scalar diagnostic.
        Problem,
    ),
    /// Float export could not provide a finite lossy approximation.
    NotFiniteApproximation,
    /// Integer export could not extract a structurally represented rational.
    /// This is not proof that the mathematical value is nonintegral.
    /// Known fractions instead report `Hyperreal(Problem::NotAnInteger)`.
    NotExactInteger,
    /// Adapter validation reported a message without a typed underlying source.
    Validation(
        /// Description of the rejected adapter input or output.
        String,
    ),
    /// Structured failure from a native modeling operation.
    Construction(
        /// Original modeling diagnostic, including its native kernel source.
        crate::errors::ValidationError,
    ),
}

impl fmt::Display for AdapterError {
    /// Render the retained cause or the adapter-specific failure message.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Hyperreal(problem) => write!(f, "{problem}"),
            Self::NotFiniteApproximation => {
                f.write_str("hyperreal value has no finite primitive approximation")
            },
            Self::NotExactInteger => {
                f.write_str("hyperreal value is not an exact i128 integer")
            },
            Self::Validation(message) => f.write_str(message),
            Self::Construction(error) => error.fmt(f),
        }
    }
}

impl std::error::Error for AdapterError {
    /// Expose native scalar or construction causes; other variants have no source.
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Hyperreal(error) => Some(error),
            Self::Construction(error) => Some(error),
            _ => None,
        }
    }
}

impl From<crate::errors::ValidationError> for AdapterError {
    /// Retain a modeling failure as a structured construction error.
    fn from(value: crate::errors::ValidationError) -> Self {
        Self::Construction(value)
    }
}

impl From<hypermesh::HypermeshError> for AdapterError {
    /// Wrap a mesh failure through the modeling error type, preserving its cause.
    fn from(value: hypermesh::HypermeshError) -> Self {
        Self::from(crate::errors::ValidationError::from(value))
    }
}

impl From<hypermesh::GpuMeshError> for AdapterError {
    /// Wrap a graphics-buffer failure through the modeling error type.
    fn from(value: hypermesh::GpuMeshError) -> Self {
        Self::from(crate::errors::ValidationError::from(value))
    }
}

#[cfg(feature = "curve")]
impl From<hypercurve::ExactCurveError> for AdapterError {
    /// Wrap an exact curve failure through the modeling error type.
    fn from(value: hypercurve::ExactCurveError) -> Self {
        Self::from(crate::errors::ValidationError::from(value))
    }
}

impl From<Problem> for AdapterError {
    /// Retain the native scalar diagnostic without converting it to text.
    fn from(value: Problem) -> Self {
        Self::Hyperreal(value)
    }
}

/// Conversion strategy used by mesh and curve facades at scalar boundaries.
///
/// Implementations should import the exact represented input value and document
/// any export loss or rejection. The built-in float strategies reject NaN and
/// infinity on import, but export finite approximations that can round or
/// underflow. This interface carries neither a rounding-error bound nor a
/// predicate certainty receipt; exported approximations are not exact predicates.
/// Custom implementations are responsible for their own conversion contracts.
pub trait ScalarAdapter: 'static {
    /// Host-side scalar accepted and returned by the facade.
    type Scalar: Clone + fmt::Debug + PartialEq + 'static;

    /// Human-readable identifier for the scalar representation.
    const NAME: &'static str;

    /// Import a host value, rejecting inputs outside this strategy's domain.
    fn into_real(value: Self::Scalar) -> AdapterResult<Real>;
    /// Export a native scalar according to this strategy's loss and error contract.
    fn from_real(value: &Real) -> AdapterResult<Self::Scalar>;
}

/// Identity strategy: import a native real by value and export by cloning.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RawReal;

/// Binary32 strategy: exact finite input and lossy finite output.
///
/// NaN and infinity fail with a native scalar diagnostic. Export may round or
/// underflow to zero; overflow fails with [`AdapterError::NotFiniteApproximation`].
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct F32;

/// Binary64 strategy: exact finite input and lossy finite output.
///
/// `0.1_f64` imports its exact binary value, not the rational 1/10. Use a
/// native [`Real`] ratio with [`RawReal`] when that distinction matters. NaN
/// and infinity are rejected; export may round or underflow and rejects overflow.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct F64;

/// Signed 128-bit integer strategy with exact import and checked export.
///
/// Export first requires [`Real::exact_rational`] to recognize the representation.
/// A known fraction returns `Hyperreal(Problem::NotAnInteger)` and an integer
/// outside the `i128` range returns `Hyperreal(Problem::OutOfRange)`. Unrecognized
/// representations return [`AdapterError::NotExactInteger`] without numerical
/// refinement or an integrality proof.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct I128;

impl ScalarAdapter for RawReal {
    /// Scalar representation exposed by this strategy.
    type Scalar = Real;

    /// Name of the host scalar representation.
    const NAME: &'static str = "real";

    /// Move the native scalar unchanged; this conversion is infallible.
    fn into_real(value: Self::Scalar) -> AdapterResult<Real> {
        Ok(value)
    }

    /// Clone the native scalar without approximation.
    fn from_real(value: &Real) -> AdapterResult<Self::Scalar> {
        Ok(value.clone())
    }
}

impl ScalarAdapter for F32 {
    /// Scalar representation exposed by this strategy.
    type Scalar = f32;

    /// Name of the host scalar representation.
    const NAME: &'static str = "f32";

    /// Import the exact finite binary32 value; reject NaN and infinity.
    fn into_real(value: Self::Scalar) -> AdapterResult<Real> {
        Real::try_from(value).map_err(AdapterError::from)
    }

    /// Return a lossy binary32 view, rejecting a missing finite approximation.
    fn from_real(value: &Real) -> AdapterResult<Self::Scalar> {
        value
            .to_f32_lossy()
            .ok_or(AdapterError::NotFiniteApproximation)
    }
}

impl ScalarAdapter for F64 {
    /// Scalar representation exposed by this strategy.
    type Scalar = f64;

    /// Name of the host scalar representation.
    const NAME: &'static str = "f64";

    /// Import the exact finite binary64 value; reject NaN and infinity.
    fn into_real(value: Self::Scalar) -> AdapterResult<Real> {
        Real::try_from(value).map_err(AdapterError::from)
    }

    /// Return a lossy binary64 view and reject missing or nonfinite output.
    fn from_real(value: &Real) -> AdapterResult<Self::Scalar> {
        value
            .to_f64_lossy()
            .filter(|value| value.is_finite())
            .ok_or(AdapterError::NotFiniteApproximation)
    }
}

impl ScalarAdapter for I128 {
    /// Scalar representation exposed by this strategy.
    type Scalar = i128;

    /// Name of the host scalar representation.
    const NAME: &'static str = "i128";

    /// Import the integer exactly; every `i128` value is admitted.
    fn into_real(value: Self::Scalar) -> AdapterResult<Real> {
        Ok(Real::from(value))
    }

    /// Extract a represented rational and require an in-range integer.
    ///
    /// Does not refine an unrecognized representation to decide integrality.
    fn from_real(value: &Real) -> AdapterResult<Self::Scalar> {
        let rational: Rational =
            value.exact_rational().ok_or(AdapterError::NotExactInteger)?;
        rational.try_into().map_err(AdapterError::from)
    }
}

/// Import XY components in order, stopping at the first conversion error.
#[cfg(feature = "curve")]
pub(crate) fn scalar2_to_real<A: ScalarAdapter>(
    point: [A::Scalar; 2],
) -> AdapterResult<[Real; 2]> {
    let [x, y] = point;
    Ok([A::into_real(x)?, A::into_real(y)?])
}

/// Import XYZ components in order, stopping at the first conversion error.
pub(crate) fn scalar3_to_real<A: ScalarAdapter>(
    point: [A::Scalar; 3],
) -> AdapterResult<[Real; 3]> {
    let [x, y, z] = point;
    Ok([A::into_real(x)?, A::into_real(y)?, A::into_real(z)?])
}

/// Export XY components using the selected strategy's loss and error contract.
#[cfg(feature = "curve")]
pub(crate) fn real2_to_scalar<A: ScalarAdapter>(
    point: &[Real; 2],
) -> AdapterResult<[A::Scalar; 2]> {
    Ok([A::from_real(&point[0])?, A::from_real(&point[1])?])
}

/// Export XYZ components using the selected strategy's loss and error contract.
pub(crate) fn real3_to_scalar<A: ScalarAdapter>(
    point: &[Real; 3],
) -> AdapterResult<[A::Scalar; 3]> {
    Ok([
        A::from_real(&point[0])?,
        A::from_real(&point[1])?,
        A::from_real(&point[2])?,
    ])
}
