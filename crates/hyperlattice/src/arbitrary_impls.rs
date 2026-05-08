/// Optional [`Arbitrary`] implementations for approximate-backend types.
///
/// Enable the `arbitrary` crate feature to activate these. All implementations
/// target [`ApproxBackend`] because that backend evaluates synchronously and
/// runs fast enough for coverage-guided fuzzing.
#[cfg(all(feature = "arbitrary", feature = "approx-backend"))]
mod arbitrary_approx {
    use arbitrary::{Arbitrary, Unstructured, Error};

    use crate::{ApproxBackend, Complex, Matrix3, Matrix4, Scalar, Vector3, Vector4};

    // Maps any u64 bit pattern to a finite f64; non-finite patterns map to 0.0
    // so the full IEEE 754 finite range (including subnormals and negative zero)
    // is reachable while NaN and ±Inf are folded away rather than rejected.
    fn finite_f64(u: &mut Unstructured<'_>) -> arbitrary::Result<f64> {
        let bits = u64::arbitrary(u)?;
        let f = f64::from_bits(bits);
        if f.is_finite() { Ok(f) } else { Err(Error::IncorrectFormat) }
    }

    // Maps any u64 bit pattern to a non-negative finite f64 (suitable for use as
    // an error bound). The `.abs()` call covers the negative-finite case, and
    // non-finite results fold to 0.0.
    fn nonneg_finite_f64(u: &mut Unstructured<'_>) -> arbitrary::Result<f64> {
        let bits = u64::arbitrary(u)?;
        let f = f64::from_bits(bits).abs();
        if f.is_finite() { Ok(f) } else { Err(Error::IncorrectFormat) }
    }

    impl<'a> Arbitrary<'a> for Scalar<ApproxBackend> {
        fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
            let value = finite_f64(u)?;
            let epsilon = nonneg_finite_f64(u)?;
            // A finite value paired with a non-negative finite epsilon is always a
            // valid approximate scalar, so this unwrap cannot fail.
            Ok(Self::approx(value, epsilon).expect("finite inputs are always valid"))
        }
    }

    impl<'a> Arbitrary<'a> for Complex<ApproxBackend> {
        fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
            Ok(Self::new(Scalar::arbitrary(u)?, Scalar::arbitrary(u)?))
        }
    }

    impl<'a> Arbitrary<'a> for Vector3<ApproxBackend> {
        fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
            Ok(Self::new([
                Scalar::arbitrary(u)?,
                Scalar::arbitrary(u)?,
                Scalar::arbitrary(u)?,
            ]))
        }
    }

    impl<'a> Arbitrary<'a> for Vector4<ApproxBackend> {
        fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
            Ok(Self::new([
                Scalar::arbitrary(u)?,
                Scalar::arbitrary(u)?,
                Scalar::arbitrary(u)?,
                Scalar::arbitrary(u)?,
            ]))
        }
    }

    impl<'a> Arbitrary<'a> for Matrix3<ApproxBackend> {
        fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
            Ok(Self::new([
                [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
                [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
                [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
            ]))
        }
    }

    impl<'a> Arbitrary<'a> for Matrix4<ApproxBackend> {
        fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
            Ok(Self::new([
                [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
                [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
                [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
                [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
            ]))
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
    use arbitrary::{Arbitrary, Error, Unstructured};

    use crate::{
        Complex, HyperrealBackend, Matrix3, Matrix4, Rational, Scalar, Vector3, Vector4,
    };

    // Maps any u64 bit pattern to a finite f64. Non-finite patterns are rejected
    // so the fuzzer can still explore the full finite IEEE-754 space, including
    // subnormals and negative zero.
    fn finite_f64(u: &mut Unstructured<'_>) -> arbitrary::Result<f64> {
        let bits = u64::arbitrary(u)?;
        let f = f64::from_bits(bits);
        if f.is_finite() {
            Ok(f)
        } else {
            Err(Error::IncorrectFormat)
        }
    }

    impl<'a> Arbitrary<'a> for Scalar<HyperrealBackend> {
        fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
            let value = finite_f64(u)?;
            let rational = <Rational as core::convert::TryFrom<f64>>::try_from(value)
                .map_err(|_| Error::IncorrectFormat)?;
            Ok(Self::new(rational))
        }
    }

    impl<'a> Arbitrary<'a> for Complex<HyperrealBackend> {
        fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
            Ok(Self::new(
                Scalar::arbitrary(u)?,
                Scalar::arbitrary(u)?,
            ))
        }
    }

    impl<'a> Arbitrary<'a> for Vector3<HyperrealBackend> {
        fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
            Ok(Self::new([
                Scalar::arbitrary(u)?,
                Scalar::arbitrary(u)?,
                Scalar::arbitrary(u)?,
            ]))
        }
    }

    impl<'a> Arbitrary<'a> for Vector4<HyperrealBackend> {
        fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
            Ok(Self::new([
                Scalar::arbitrary(u)?,
                Scalar::arbitrary(u)?,
                Scalar::arbitrary(u)?,
                Scalar::arbitrary(u)?,
            ]))
        }
    }

    impl<'a> Arbitrary<'a> for Matrix3<HyperrealBackend> {
        fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
            Ok(Self::new([
                [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
                [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
                [Scalar::arbitrary(u)?, Scalar::arbitrary(u)?, Scalar::arbitrary(u)?],
            ]))
        }
    }

    impl<'a> Arbitrary<'a> for Matrix4<HyperrealBackend> {
        fn arbitrary(u: &mut Unstructured<'a>) -> arbitrary::Result<Self> {
            Ok(Self::new([
                [
                    Scalar::arbitrary(u)?,
                    Scalar::arbitrary(u)?,
                    Scalar::arbitrary(u)?,
                    Scalar::arbitrary(u)?,
                ],
                [
                    Scalar::arbitrary(u)?,
                    Scalar::arbitrary(u)?,
                    Scalar::arbitrary(u)?,
                    Scalar::arbitrary(u)?,
                ],
                [
                    Scalar::arbitrary(u)?,
                    Scalar::arbitrary(u)?,
                    Scalar::arbitrary(u)?,
                    Scalar::arbitrary(u)?,
                ],
                [
                    Scalar::arbitrary(u)?,
                    Scalar::arbitrary(u)?,
                    Scalar::arbitrary(u)?,
                    Scalar::arbitrary(u)?,
                ],
            ]))
        }
    }
}
