//! Principal complex elementary continuation over exact native Real expressions.
//!
//! The atanh path separates axis/domain decisions from logarithmic magnitude and
//! quadrant-aware angle construction. It delegates scalar decisions and failures
//! to Hyperreal; it does not fit, sample, or round authoritative complex values.

use super::Complex;
use crate::{BlasResult, Problem, Real, RealSign};

impl Complex {
    /// Principal inverse hyperbolic tangent off the real cuts `(-∞,-1]` and `[1,∞)`.
    ///
    /// Components remain exact native [`Real`] expressions. For z=x+iy, put
    /// L=(1-x)^2+y^2 and U=(1+x)^2+y^2. The general result is
    /// `ln(U/L)/4 + i*atan2(2y, 1-x^2-y^2)/2`, derived from the
    /// [principal logarithmic definition](https://dlmf.nist.gov/4.37.E24).
    /// An immediately known nonnegative x uses ln_1p(4x/L); negative x uses
    /// -ln_1p(-4x/U). Unknown x keeps ln(U/L), without refining merely to select
    /// the stable form. The angle uses the native difference-of-products constructor.
    ///
    /// Axis checks precede sign refinement: definitely-zero imaginary part delegates
    /// to real atanh, then definitely-zero real part gives i*atan(y). Otherwise query
    /// the imaginary sign at [`Real::PARTIAL_CMP_MIN_PRECISION`]. A certified zero
    /// delegates to the real path. If the sign is unresolved, a proved positive
    /// 1-x^2 still admits the input inside the cut-free real strip; otherwise return
    /// [`Problem::Exhausted`]. Subsequent scalar log/division/atan2 failures propagate.
    ///
    /// The real interval `(-1,1)` uses [`Real::atanh`]. Certified endpoints return
    /// [`Problem::Infinity`]; certified exterior real cut points return
    /// [`Problem::NotANumber`]. No signed-zero convention chooses a bank of a cut.
    /// Undecided branch decisions remain typed failures, not guessed zero signs.
    /// The scalar precision/cancellation contract applies; no total-work, retained-heap,
    /// wall-time bound, predicate-policy argument, or separate certainty receipt is added.
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
