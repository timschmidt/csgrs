//! Rectangular Jacobi continuation and epsilon over shared exact scalar expressions.
//!
//! Hyperreal owns real amplitude/elliptic evaluation and scalar relations. This
//! module assembles complex values using complementary or reciprocal parameters;
//! it keeps a common denominator so removable intermediate singularities do not
//! become poles. The parameter convention is m=k², not the modulus k used in DLMF.

use super::Complex;
use crate::{BlasResult, Problem, Real, RealKernelExt, RealSign};

/// Complementary real data and a shared inverse for rectangular continuation.
///
/// For original m!=0, m<1, these fields represent (S,C,D) at y|1-m. They are
/// not the complex triplet at i*y. Negative m requires reciprocal transformation
/// because the real evaluator admits parameters below one. The common denominator
/// depends also on the original real-axis triplet; avoiding a separate 1/C keeps
/// finite quarter-period points available to both complex operations.
struct ImaginaryJacobi {
    /// Complementary S=sn(y|1-m), rescaled from sn(sqrt(1-m)*y|1/(1-m)) for m<0.
    sn: Real,
    /// Complementary C=cn(y|1-m), obtained from the reciprocal triplet's dn for m<0.
    cn: Real,
    /// Complementary D=dn(y|1-m), obtained from the reciprocal triplet's cn for m<0; may vanish.
    dn: Real,
    /// Shared exact 1/Q, with Q=C²+m*sn(x|m)²*S², constructed as a nonnegative sum in either branch.
    inverse: Real,
    /// Some(y-epsilon(y|1-m)) exactly when requested; the reciprocal branch forms the same value as r*(r*y-epsilon(r*y|1/(1-m))).
    epsilon_vertical: Option<Real>,
}

impl ImaginaryJacobi {
    /// Build complementary data after the caller validates m<1 and certifies its nonzero sign.
    ///
    /// real_sn and real_cn must belong to the same x|m. This helper trusts those
    /// relations and the supplied sign; it does not recertify them. Positive m uses
    /// argument y and parameter h=1-m. Negative m uses r=sqrt(h), argument r*y,
    /// parameter 1/h, then maps (s,c,d) to (s/r,d,c), following the
    /// [reciprocal-modulus identities](https://dlmf.nist.gov/22.17).
    /// Its denominator c²+(-m/h)*real_cn²*s² equals Q without subtractive cancellation.
    ///
    /// with_epsilon selects the shared four-output real evaluator and stores the
    /// vertical correction; false avoids constructing epsilon. Scalar square-root,
    /// division, evaluation and inverse failures propagate. A successful inverse
    /// retains its native domain obligations; no separate proof receipt is created.
    fn new(
        y: Real,
        parameter: &Real,
        real_sn: &Real,
        real_cn: &Real,
        sign: RealSign,
        with_epsilon: bool,
    ) -> BlasResult<Self> {
        let complement = Real::one() - parameter;
        let (argument, transformed_parameter, scale) = if sign == RealSign::Positive {
            (y, complement.clone(), Real::one())
        } else {
            let scale = complement.clone().sqrt()?;
            (y * &scale, complement.clone().inverse()?, scale)
        };
        let (s, c, d, epsilon_vertical) = if with_epsilon {
            let (s, c, d, epsilon) = argument
                .clone()
                .jacobi_sncndn_epsilon(&transformed_parameter)?;
            (s, c, d, Some(&scale * (&argument - epsilon)))
        } else {
            let (s, c, d) = argument.jacobi_sncndn(&transformed_parameter)?;
            (s, c, d, None)
        };
        let inverse_scale = if sign == RealSign::Positive {
            Real::one()
        } else {
            scale.clone().inverse()?
        };
        let denominator = if sign == RealSign::Positive {
            &c * &c + parameter * (real_sn * real_sn) * (&s * &s)
        } else {
            // This nonnegative sum equals complementary cn²+m*sn(x)²*sn(y)².
            &c * &c + (-parameter / complement)? * (real_cn * real_cn) * (&s * &s)
        };
        let inverse = denominator.inverse()?;
        let (sn, cn, dn) = if sign == RealSign::Positive {
            (s, c, d)
        } else {
            (s * inverse_scale, d, c)
        };
        Ok(Self {
            sn,
            cn,
            dn,
            inverse,
            epsilon_vertical,
        })
    }
}

impl Complex {
    /// Jacobi's `(sn(z|m), cn(z|m), dn(z|m))` for a real parameter `m < 1`.
    ///
    /// The parameter is the squared modulus. All components remain native
    /// [`Real`] expressions. The real axis reuses [`Real::jacobi_sncndn`];
    /// elsewhere, rectangular addition and imaginary transformations share
    /// one denominator. Negative parameters are supported. Complex `dn` can
    /// be zero and does not have the real-axis positivity restriction.
    ///
    /// Real evaluation at x validates the parameter before the real-axis
    /// shortcut. If the imaginary component is definitely zero, its triplet is
    /// returned directly. Otherwise the sign of m is certified at
    /// [`Real::PARTIAL_CMP_MIN_PRECISION`]; m=0 uses sin/cos continued with
    /// sinh/cosh and dn=1. Nonzero m uses the
    /// [addition formulas](https://dlmf.nist.gov/22.8) with complementary real data,
    /// sharing one inverse across all six output components. Scalar identities
    /// remain in those expressions; this layer does not add a complex proof node.
    ///
    /// Proved invalid parameters return [`Problem::NotANumber`] before any zero
    /// shortcut. Unresolved parameter branches return [`Problem::Exhausted`].
    /// Proved poles return [`Problem::DivideByZero`]; an unresolved denominator
    /// returns [`Problem::UnknownZero`]. Scalar precision and cancellation
    /// contracts apply; this operation adds no total-work or heap-byte bound.
    pub fn jacobi_sncndn(self, parameter: &Real) -> BlasResult<(Self, Self, Self)> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "jacobi-sncndn");
        let (sn, cn, dn) = self.re.jacobi_sncndn(parameter)?;
        if self.im.definitely_zero() {
            return Ok((Self::from(sn), Self::from(cn), Self::from(dn)));
        }
        let sign = parameter
            .certified_sign_until(Real::PARTIAL_CMP_MIN_PRECISION)
            .sign()
            .ok_or(Problem::Exhausted)?;
        if sign == RealSign::Zero {
            let sinh = self.im.clone().sinh()?;
            let cosh = self.im.cosh()?;
            return Ok((
                Self::new(&sn * &cosh, &cn * &sinh),
                Self::new(&cn * &cosh, -(&sn * &sinh)),
                Self::one(),
            ));
        }
        let imaginary = ImaginaryJacobi::new(self.im, parameter, &sn, &cn, sign, false)?;
        let components = [
            (
                &sn * &imaginary.dn,
                &cn * &dn * &imaginary.sn * &imaginary.cn,
            ),
            (
                &cn * &imaginary.cn,
                -(&sn * &dn * &imaginary.sn * &imaginary.dn),
            ),
            (
                &dn * &imaginary.cn * &imaginary.dn,
                -(parameter * &sn * &cn * &imaginary.sn),
            ),
        ];
        let [sn, cn, dn] = components.map(|(re, im)| {
            Self::new(
                re.mul_cached(&imaginary.inverse),
                im.mul_cached(&imaginary.inverse),
            )
        });
        Ok((sn, cn, dn))
    }

    /// Jacobi's epsilon, the meromorphic primitive of `dn(z|m)²` with
    /// `epsilon(0|m)=0`, for an admitted real squared modulus `m < 1`.
    ///
    /// The real axis uses [`Real::jacobi_epsilon`]. Complex continuation
    /// preserves its additive quasi-periods and finite quarter-period values.
    /// The argument is a Jacobi argument, not a Legendre amplitude. Negative
    /// parameters are supported; `epsilon(z|0)=z`.
    ///
    /// If imaginary zero is not cheaply known, ln(1-m) validates m<1 before sign
    /// certification and the zero-parameter identity. The nonzero-parameter branch
    /// requests the real triplet and epsilon together at x, then shares
    /// complementary data and one inverse.
    /// With (s,c,d) at x|m, (S,C,D) at y|1-m and Q=C²+m*s²*S², its components are
    /// `epsilon(x|m)+m*s*c*d*S²/Q` and `y-epsilon(y|1-m)+d²*S*C*D/Q`.
    /// The [epsilon addition law](https://dlmf.nist.gov/22.16.E27) permits this
    /// form without a separate divisor C. The private vertical term is present
    /// by construction because this path requests epsilon; its expect checks that
    /// internal invariant, not a user-domain condition.
    ///
    /// Parameter, pole, precision and cancellation errors are those of
    /// [`Self::jacobi_sncndn`]. Values remain exact native expressions; the
    /// operation adds no total-work, retained-heap or wall-time bound.
    pub fn jacobi_epsilon(self, parameter: &Real) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "jacobi-epsilon");
        // Validate the parameter even when z=0. Retain the unwrapped real
        // composition and its defining inverse relation on the real axis.
        if self.im.definitely_zero() {
            return Ok(Self::from(self.re.jacobi_epsilon(parameter)?));
        }
        // Preserve parameter validation before the zero-parameter identity.
        (Real::one() - parameter).ln()?;
        let sign = parameter
            .certified_sign_until(Real::PARTIAL_CMP_MIN_PRECISION)
            .sign()
            .ok_or(Problem::Exhausted)?;
        if sign == RealSign::Zero {
            return Ok(self);
        }
        let (sn, cn, dn, real_epsilon) = self.re.jacobi_sncndn_epsilon(parameter)?;
        let imaginary = ImaginaryJacobi::new(self.im, parameter, &sn, &cn, sign, true)?;
        let re = parameter * &sn * &cn * &dn * (&imaginary.sn * &imaginary.sn);
        let im = (&dn * &dn) * &imaginary.sn * &imaginary.cn * &imaginary.dn;
        Ok(Self::new(
            real_epsilon + re.mul_cached(&imaginary.inverse),
            imaginary.epsilon_vertical.expect("epsilon was requested")
                + im.mul_cached(&imaginary.inverse),
        ))
    }
}
