use super::Complex;
use crate::{BlasResult, Problem, Real, RealSign};

impl Complex {
    /// Principal inverse hyperbolic tangent off the real cuts `(-∞,-1]`
    /// and `[1,∞)`. Components remain exact native [`Real`] expressions.
    ///
    /// The real interval `(-1,1)` reuses [`Real::atanh`]. The endpoints
    /// return [`Problem::Infinity`]; other points on a cut return
    /// [`Problem::NotANumber`]. This API does not choose a bank of a cut
    /// from a signed-zero convention. Undecided cut membership returns
    /// [`Problem::Exhausted`]. Native precision and cancellation contracts
    /// apply; no total-work or retained-heap bound is added.
    pub fn atanh(self) -> BlasResult<Self> {
        crate::trace_dispatch!("hyperlattice_complex", "method", "atanh");
        if self.im.definitely_zero() {
            return Ok(Self::from(self.re.atanh()?));
        }
        if self.re.definitely_zero() {
            return Ok(Self::new(Real::zero(), self.im.atan()?));
        }
        let imaginary_sign = self
            .im
            .certified_sign_until(Real::PARTIAL_CMP_MIN_PRECISION)
            .sign();
        match imaginary_sign {
            Some(RealSign::Zero) => return Ok(Self::from(self.re.atanh()?)),
            Some(RealSign::Positive | RealSign::Negative) => {}
            None => {
                let margin = Real::one() - &self.re * &self.re;
                if margin
                    .certified_sign_until(Real::PARTIAL_CMP_MIN_PRECISION)
                    .sign()
                    != Some(RealSign::Positive)
                {
                    return Err(Problem::Exhausted);
                }
            }
        }
        let minus = Real::one() - &self.re;
        let plus = Real::one() + &self.re;
        let imaginary_square = &self.im * &self.im;
        let lower = &minus * &minus + &imaginary_square;
        let upper = &plus * &plus + &imaginary_square;
        // Select a nonnegative log1p argument only from an existing exact
        // sign. Unknown sign keeps the exact logarithmic quotient fallback.
        let logarithm = match self.re.immediate_sign() {
            Some(RealSign::Positive | RealSign::Zero) => {
                ((Real::from(4) * &self.re / lower)?).ln_1p()?
            }
            Some(RealSign::Negative) => -((Real::from(-4) * &self.re / upper)?).ln_1p()?,
            None => (upper / lower)?.ln()?,
        };
        let argument_x = Real::diff_of_products(&minus, &plus, &self.im, &self.im);
        let angle = (Real::from(2) * self.im).try_atan2(argument_x)?;
        Ok(Self::new(
            (logarithm / Real::from(4))?,
            (angle / Real::from(2))?,
        ))
    }
}
