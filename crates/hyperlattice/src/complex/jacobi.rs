use super::Complex;
use crate::{BlasResult, Problem, Real, RealKernelExt, RealSign};

impl Complex {
    /// Jacobi's `(sn(z|m), cn(z|m), dn(z|m))` for a real parameter `m < 1`.
    ///
    /// The parameter is the squared modulus. All components remain native
    /// [`Real`] expressions. The real axis reuses [`Real::jacobi_sncndn`];
    /// elsewhere, rectangular addition and imaginary transformations share
    /// one denominator. Negative parameters are supported. Complex `dn` can
    /// be zero and does not have the real-axis positivity restriction.
    ///
    /// Invalid parameters return [`Problem::NotANumber`] before any zero
    /// shortcut. Unresolved parameter branches return [`Problem::Exhausted`].
    /// Poles return [`Problem::DivideByZero`]; an unresolved denominator
    /// returns [`Problem::UnknownZero`]. Scalar precision and cancellation
    /// contracts apply; this operation adds no total-work or heap-byte bound.
    pub fn jacobi_sncndn(self, parameter: &Real) -> BlasResult<(Self, Self, Self)> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "jacobi-sncndn");
        // This validates m even at z=0, and preserves the native amplitude
        // identities on the real axis.
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

        let complement = Real::one() - parameter;
        let (s, c, d, inverse_scale) = if sign == RealSign::Positive {
            let (s, c, d) = self.im.jacobi_sncndn(&complement)?;
            (s, c, d, Real::one())
        } else {
            // Reciprocal modulus for the complementary parameter h=1-m>1:
            // sn(y|h)=sn(sqrt(h)y|1/h)/sqrt(h), cn(y|h)=dn(...),
            // dn(y|h)=cn(...). Do not form singular intermediate quotients.
            let scale = complement.clone().sqrt()?;
            let reciprocal_parameter = complement.clone().inverse()?;
            let (s, c, d) = (self.im * &scale).jacobi_sncndn(&reciprocal_parameter)?;
            (s, c, d, scale.inverse()?)
        };
        let denominator = if sign == RealSign::Positive {
            &c * &c + parameter * (&sn * &sn) * (&s * &s)
        } else {
            // Equivalent to d²+(m/h)sn²s², but a nonnegative sum avoids
            // cancellation. Its zeros are c=0 and cn=0.
            &c * &c + (-parameter / complement)? * (&cn * &cn) * (&s * &s)
        };
        let inverse = denominator.inverse()?;
        let imaginary_sine = &s * inverse_scale;
        let (complementary_cn, complementary_dn) = if sign == RealSign::Positive {
            (c, d)
        } else {
            (d, c)
        };
        let components = [
            (
                &sn * &complementary_dn,
                &cn * &dn * &imaginary_sine * &complementary_cn,
            ),
            (
                &cn * &complementary_cn,
                -(&sn * &dn * &imaginary_sine * &complementary_dn),
            ),
            (
                &dn * &complementary_cn * &complementary_dn,
                -(parameter * &sn * &cn * &imaginary_sine),
            ),
        ];
        let [sn, cn, dn] =
            components.map(|(re, im)| Self::new(re.mul_cached(&inverse), im.mul_cached(&inverse)));
        Ok((sn, cn, dn))
    }
}
