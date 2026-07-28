use hyperreal::{Problem, Rational, Real};
use std::fmt;

pub type AdapterResult<T> = Result<T, AdapterError>;

#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum AdapterError {
    Hyperreal(Problem),
    NotFiniteApproximation,
    NotExactInteger,
    Validation(String),
}

impl fmt::Display for AdapterError {
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
        }
    }
}

impl std::error::Error for AdapterError {}

impl From<Problem> for AdapterError {
    fn from(value: Problem) -> Self {
        Self::Hyperreal(value)
    }
}

pub trait ScalarAdapter {
    type Scalar: Clone + fmt::Debug + PartialEq;

    const NAME: &'static str;

    fn into_real(value: Self::Scalar) -> AdapterResult<Real>;
    fn from_real(value: &Real) -> AdapterResult<Self::Scalar>;
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RawReal;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct F32;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct F64;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct I128;

impl ScalarAdapter for RawReal {
    type Scalar = Real;

    const NAME: &'static str = "real";

    fn into_real(value: Self::Scalar) -> AdapterResult<Real> {
        Ok(value)
    }

    fn from_real(value: &Real) -> AdapterResult<Self::Scalar> {
        Ok(value.clone())
    }
}

impl ScalarAdapter for F32 {
    type Scalar = f32;

    const NAME: &'static str = "f32";

    fn into_real(value: Self::Scalar) -> AdapterResult<Real> {
        Real::try_from(value).map_err(AdapterError::from)
    }

    fn from_real(value: &Real) -> AdapterResult<Self::Scalar> {
        value
            .to_f32_lossy()
            .ok_or(AdapterError::NotFiniteApproximation)
    }
}

impl ScalarAdapter for F64 {
    type Scalar = f64;

    const NAME: &'static str = "f64";

    fn into_real(value: Self::Scalar) -> AdapterResult<Real> {
        Real::try_from(value).map_err(AdapterError::from)
    }

    fn from_real(value: &Real) -> AdapterResult<Self::Scalar> {
        value
            .to_f64_lossy()
            .filter(|value| value.is_finite())
            .ok_or(AdapterError::NotFiniteApproximation)
    }
}

impl ScalarAdapter for I128 {
    type Scalar = i128;

    const NAME: &'static str = "i128";

    fn into_real(value: Self::Scalar) -> AdapterResult<Real> {
        Ok(Real::from(value))
    }

    fn from_real(value: &Real) -> AdapterResult<Self::Scalar> {
        let rational: Rational =
            value.exact_rational().ok_or(AdapterError::NotExactInteger)?;
        rational.try_into().map_err(AdapterError::from)
    }
}

#[cfg(feature = "sketch")]
pub(crate) fn scalar2_to_real<A: ScalarAdapter>(
    point: [A::Scalar; 2],
) -> AdapterResult<[Real; 2]> {
    let [x, y] = point;
    Ok([A::into_real(x)?, A::into_real(y)?])
}

pub(crate) fn scalar3_to_real<A: ScalarAdapter>(
    point: [A::Scalar; 3],
) -> AdapterResult<[Real; 3]> {
    let [x, y, z] = point;
    Ok([A::into_real(x)?, A::into_real(y)?, A::into_real(z)?])
}

#[cfg(feature = "sketch")]
pub(crate) fn real2_to_scalar<A: ScalarAdapter>(
    point: &[Real; 2],
) -> AdapterResult<[A::Scalar; 2]> {
    Ok([A::from_real(&point[0])?, A::from_real(&point[1])?])
}

pub(crate) fn real3_to_scalar<A: ScalarAdapter>(
    point: &[Real; 3],
) -> AdapterResult<[A::Scalar; 3]> {
    Ok([
        A::from_real(&point[0])?,
        A::from_real(&point[1])?,
        A::from_real(&point[2])?,
    ])
}
