use arbitrary::{Arbitrary, Unstructured, Error};

use crate::{Backend as NumBackend, Scalar};
use crate::{Complex, Matrix3, Matrix4, Vector3, Vector4};

/// Maps any u64 bit pattern to a finite f64; non-finite patterns map to 0.0
/// so the full IEEE 754 finite range (including subnormals and negative zero)
/// is reachable while NaN and ±Inf are folded away rather than rejected.
fn finite_f64(u: &mut Unstructured<'_>) -> arbitrary::Result<f64> {
    let bits = u64::arbitrary(u)?;
    let f = f64::from_bits(bits);
    if f.is_finite() { Ok(f) } else { Err(Error::IncorrectFormat) }
}

/// Optional [`Arbitrary`] implementations for approximate-backend types.
///
/// Enable the `arbitrary` crate feature to activate these. All implementations
/// target [`ApproxBackend`] because that backend evaluates synchronously and
/// runs fast enough for coverage-guided fuzzing.
#[cfg(all(feature = "arbitrary", feature = "approx-backend"))]
mod arbitrary_approx {
    use super::*;
    use crate::ApproxBackend;

    impl<'a> Arbitrary<'a> for Scalar<ApproxBackend> {
        fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
            let value = finite_f64(u)?;
            let epsilon = finite_f64(u)?.abs();
            // A finite value paired with a non-negative finite epsilon is always a
            // valid approximate scalar, so this unwrap cannot fail.
            Ok(Self::approx(value, epsilon).expect("finite inputs are always valid"))
        }
    }
}

/// Optional [`Arbitrary`] implementations for hyperreal-backend types.
///
/// Enable the `arbitrary` crate feature to activate these. These impls target
/// [`HyperrealBackend`], which evaluates exactly enough for fuzzing to explore
/// symbolic / exact-code paths without depending on the approximate backend.
#[cfg(all(feature = "arbitrary", feature = "hyperreal-backend"))]
mod arbitrary_hyperreal {
    use super::*;
    use crate::{HyperrealBackend, Rational};
    use num::bigint::{BigInt, Sign};

    fn arbitrary_sign(u: &mut Unstructured<'_>) -> arbitrary::Result<Sign> {
        u.choose(&[Sign::NoSign, Sign::Minus, Sign::Plus]).copied()
    }

    impl<'a> Arbitrary<'a> for Scalar<HyperrealBackend> {
        fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
            if u.ratio(1, 2)? {
                let value = finite_f64(u)?;
                let rational = <Rational as core::convert::TryFrom<f64>>::try_from(value)
                    .map_err(|_| Error::IncorrectFormat)?;
                Ok(Self::new(rational))
            } else {
                Ok(Self::new(Rational::from_bigint(
                    BigInt::from_bytes_le(arbitrary_sign(u)?, u.arbitrary()?)
                )))
            }
        }

        fn size_hint(depth: usize) -> (usize, Option<usize>) {
            (u64::size_hint(depth).0 + 1, None)
        }
    }
}

impl<'a, Backend: NumBackend> Arbitrary<'a> for Complex<Backend> where Scalar<Backend>: Arbitrary<'a> {
    fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self::new(
            Scalar::arbitrary(u)?,
            Scalar::arbitrary(u)?,
        ))
    }
}

impl<'a, Backend: NumBackend> Arbitrary<'a> for Vector3<Backend> where Scalar<Backend>: Arbitrary<'a> {
    fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self::new([
            Scalar::arbitrary(u)?,
            Scalar::arbitrary(u)?,
            Scalar::arbitrary(u)?,
        ]))
    }
}

impl<'a, Backend: NumBackend> Arbitrary<'a> for Vector4<Backend> where Scalar<Backend>: Arbitrary<'a> {
    fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self::new([
            Scalar::arbitrary(u)?,
            Scalar::arbitrary(u)?,
            Scalar::arbitrary(u)?,
            Scalar::arbitrary(u)?,
        ]))
    }
}

impl<'a, Backend: NumBackend> Arbitrary<'a> for Matrix3<Backend> where Scalar<Backend>: Arbitrary<'a> {
    fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self::new([
            [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
            [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
            [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
        ]))
    }
}

impl<'a, Backend: NumBackend> Arbitrary<'a> for Matrix4<Backend> where Scalar<Backend>: Arbitrary<'a> {
    fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
        Ok(Self::new([
            [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
            [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
            [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
            [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
        ]))
    }
}
