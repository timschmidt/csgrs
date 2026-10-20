use super::Complex;
use crate::{BlasResult, Problem, Real, RealKernelExt, RealSign};

// Shared imaginary transformation for the triplet and epsilon. The
// reciprocal branch keeps every real evaluation inside its admitted m<1
// domain, and the one denominator includes removable quarter-period points.
struct ImaginaryJacobi {
    sn: Real,
    cn: Real,
    dn: Real,
    inverse: Real,
    epsilon_vertical: Option<Real>,
}

impl ImaginaryJacobi {
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
    /// Invalid parameters return [`Problem::NotANumber`] before any zero
    /// shortcut. Unresolved parameter branches return [`Problem::Exhausted`].
    /// Poles return [`Problem::DivideByZero`]; an unresolved denominator
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
