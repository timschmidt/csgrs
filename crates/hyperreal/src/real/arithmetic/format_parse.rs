use core::fmt;

impl Real {
    /// Format this Real as a decimal rather than rational.
    /// Scientific notation will be used if the value is very large or small.
    pub fn decimal(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let folded = self.fold_ref();
        match folded.iter_msd_stop(-20) {
            Some(-19..60) => fmt::Display::fmt(&folded, f),
            _ => fmt::LowerExp::fmt(&folded, f),
        }
    }
}

impl fmt::UpperExp for Real {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let folded = self.fold_ref();
        folded.fmt(f)
    }
}

impl fmt::LowerExp for Real {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let folded = self.fold_ref();
        folded.fmt(f)
    }
}

impl fmt::Display for Real {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if f.alternate() {
            self.decimal(f)
        } else {
            self.rational.fmt(f)?;
            match &self.class {
                One => Ok(()),
                Pi => f.write_str(" Pi"),
                PiPow(n) => write!(f, " x Pi**({})", n),
                PiInv => f.write_str(" / Pi"),
                PiExp(n) => write!(f, " x Pi x e**({})", n),
                PiInvExp(n) => write!(f, " x e**({}) / Pi", n),
                PiSqrt(n) => write!(f, " x Pi x √({})", n),
                ConstProduct(product) => write!(
                    f,
                    " x Pi**({}) x e**({})",
                    product.pi_power, product.exp_power
                ),
                ConstOffset(offset) => write!(
                    f,
                    " x (Pi**({}) x e**({}) + {})",
                    offset.pi_power, offset.exp_power, offset.offset
                ),
                ConstProductSqrt(product) => write!(
                    f,
                    " x Pi**({}) x e**({}) x √({})",
                    product.pi_power, product.exp_power, product.radicand
                ),
                Exp(n) => write!(f, " x e**({})", n),
                Ln(n) => write!(f, " x ln({})", n),
                LnAffine(term) => write!(f, " x ({} + ln({}))", term.offset, term.base),
                LnProduct(product) => {
                    write!(f, " x ln({}) x ln({})", product.left, product.right)
                }
                Log10(n) => write!(f, " x log10({})", n),
                Log2(n) => write!(f, " x log2({})", n),
                Sqrt(n) => write!(f, " √({})", n),
                SinPi(n) => write!(f, " x sin({} x Pi)", n),
                TanPi(n) => write!(f, " x tan({} x Pi)", n),
                _ => write!(f, " x {:?}", self.class),
            }
        }
    }
}

impl std::str::FromStr for Real {
    type Err = Problem;

    fn from_str(s: &str) -> Result<Self, Problem> {
        let rational: Rational = s.parse()?;
        Ok(Self {
            rational,
            class: One,
            computable: None,
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        })
    }
}
