//! [`Arbitrary`](arbitrary::Arbitrary) implementations for lattice types.
//!
//! The coordinate type is [`Real`](crate::Real). Because `Real` is owned by the
//! `hyperreal` crate, this module generates real components with a local helper
//! and implements `Arbitrary` only for the local vector, matrix, and complex
//! types.

use arbitrary::{Arbitrary, Error, Unstructured};

use crate::{Complex, Matrix3, Matrix4, Real, Vector2, Vector3, Vector4};

/// Maps any `u64` bit pattern to a finite `f64`.
///
/// Non-finite patterns map to `0.0`, so the full finite IEEE 754 range
/// including subnormals and negative zero is reachable while NaN and infinities
/// remain outside the exact-real API boundary.
fn finite_f64(u: &mut Unstructured<'_>) -> arbitrary::Result<f64> {
    let bits = u64::arbitrary(u)?;
    let f = f64::from_bits(bits);
    if f.is_finite() { Ok(f) } else { Ok(0.0) }
}

fn arbitrary_real(u: &mut Unstructured<'_>) -> arbitrary::Result<Real> {
    if u.ratio(1, 2)? {
        Real::try_from(finite_f64(u)?).map_err(|_| Error::IncorrectFormat)
    } else {
        Ok(Real::from(i128::arbitrary(u)?))
    }
}

impl<'a> Arbitrary<'a> for Complex {
    fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self::new(arbitrary_real(u)?, arbitrary_real(u)?))
    }
}

impl<'a> Arbitrary<'a> for Vector2 {
    fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self::new([arbitrary_real(u)?, arbitrary_real(u)?]))
    }
}

impl<'a> Arbitrary<'a> for Vector3 {
    fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self::new([
            arbitrary_real(u)?,
            arbitrary_real(u)?,
            arbitrary_real(u)?,
        ]))
    }
}

impl<'a> Arbitrary<'a> for Vector4 {
    fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self::new([
            arbitrary_real(u)?,
            arbitrary_real(u)?,
            arbitrary_real(u)?,
            arbitrary_real(u)?,
        ]))
    }
}

impl<'a> Arbitrary<'a> for Matrix3 {
    fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self::new([
            [arbitrary_real(u)?, arbitrary_real(u)?, arbitrary_real(u)?],
            [arbitrary_real(u)?, arbitrary_real(u)?, arbitrary_real(u)?],
            [arbitrary_real(u)?, arbitrary_real(u)?, arbitrary_real(u)?],
        ]))
    }
}

impl<'a> Arbitrary<'a> for Matrix4 {
    fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self::new([
            [
                arbitrary_real(u)?,
                arbitrary_real(u)?,
                arbitrary_real(u)?,
                arbitrary_real(u)?,
            ],
            [
                arbitrary_real(u)?,
                arbitrary_real(u)?,
                arbitrary_real(u)?,
                arbitrary_real(u)?,
            ],
            [
                arbitrary_real(u)?,
                arbitrary_real(u)?,
                arbitrary_real(u)?,
                arbitrary_real(u)?,
            ],
            [
                arbitrary_real(u)?,
                arbitrary_real(u)?,
                arbitrary_real(u)?,
                arbitrary_real(u)?,
            ],
        ]))
    }
}
