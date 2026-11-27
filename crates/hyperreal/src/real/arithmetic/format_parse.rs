use core::fmt;

/// Borrowed and formatter-driven presentation adapters over the exact scalar.
impl Real {
    /// Borrow a decimal display view with a fixed number of fractional digits.
    ///
    /// The formatter's precision sets the number of decimal places. Without an
    /// explicit precision, up to 32 fractional digits are shown. This is an
    /// approximate presentation boundary; a displayed zero is not an exact
    /// zero certificate. Unlike [`Real::decimal`], this view never switches to
    /// scientific notation or searches indefinitely for a nonzero leading bit.
    /// Refinement uses the absolute output precision, including for unresolved
    /// cancellation. Work and output size still depend on the requested
    /// precision, value magnitude, and underlying computable expression.
    ///
    /// ```
    /// use hyperreal::Real;
    /// let value: Real = "1/8".parse().unwrap();
    /// assert_eq!(format!("{:.3}", value.fixed_decimal()), "0.125");
    /// ```
    pub fn fixed_decimal(&self) -> impl fmt::Display + '_ {
        /// Borrow the authoritative scalar while selecting Computable's fixed decimal formatter.
        struct FixedDecimal<'a>(
            /// Source including its rational scale; folded only when formatting is requested.
            &'a Real,
        );

        /// Format the folded full value without an adaptive scientific-notation decision.
        impl fmt::Display for FixedDecimal<'_> {
            /// Forward precision and signs to the underlying approximate display implementation.
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                fmt::Display::fmt(&self.0.fold_ref(), f)
            }
        }

        FixedDecimal(self)
    }

    /// Format an approximate decimal, choosing notation from a leading-bit estimate.
    ///
    /// Fold the rational scale into the computable value, query its leading bit
    /// with a -20 precision stop, and use fixed notation only for an estimate
    /// in -19..60. All other outcomes select lowercase scientific notation,
    /// whose leading-bit search may require much more refinement. Cache state
    /// can affect this estimate; this is not an exact magnitude-window test.
    /// The formatter precision is fractional places in the selected notation.
    ///
    /// This is the alternate Display path. It returns only formatting errors,
    /// not typed numerical uncertainty or abort receipts. Use Self::try_decimal
    /// for checked rounding with an exact notation window, or Self::fixed_decimal
    /// to avoid scientific leading-bit discovery for unresolved cancellation.
    pub fn decimal(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let folded = self.fold_ref();
        match folded.iter_msd_stop(-20) {
            Some(-19..60) => fmt::Display::fmt(&folded, f),
            _ => fmt::LowerExp::fmt(&folded, f),
        }
    }
}

/// Approximate uppercase scientific display of the folded value.
/// Delegates leading-bit search, digits and formatter flags to Computable; it
/// does not provide the checked rounding/failure contract of Real::try_scientific.
impl fmt::UpperExp for Real {
    /// Include the rational scale and delegate to the matching computable formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let folded = self.fold_ref();
        folded.fmt(f)
    }
}

/// Approximate lowercase scientific display of the folded value.
/// Delegates leading-bit search, digits and formatter flags to Computable; it
/// does not provide the checked rounding/failure contract of Real::try_scientific.
impl fmt::LowerExp for Real {
    /// Include the rational scale and delegate to the matching computable formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let folded = self.fold_ref();
        folded.fmt(f)
    }
}

/// Human-readable retained-form display, or approximate decimal when alternate is set.
/// Ordinary output prints the exact rational scale followed by a named basis
/// expression (or class Debug fallback). It omits the opaque graph and is not
/// a round-trip serialization grammar; even rational mixed fractions need not
/// parse back through FromStr. Alternate output delegates to Real::decimal.
impl fmt::Display for Real {
    /// Select decimal display for `#`; otherwise emit the scale and class-specific suffix.
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
                Pow10(n) => write!(f, " x 10**({})", n),
                Pow2(n) => write!(f, " x 2**({})", n),
                Sqrt(n) => write!(f, " √({})", n),
                SinPi(n) => write!(f, " x sin({} x Pi)", n),
                TanPi(n) => write!(f, " x tan({} x Pi)", n),
                _ => write!(f, " x {:?}", self.class),
            }
        }
    }
}

/// Parse exact rational numeric text, with a lazy fallback for very large scientific exponents.
/// Integers, fractions, decimals and e/E scientific literals enter through Rational;
/// no binary floating value is introduced. Symbolic Display expressions such as
/// pi/root/log forms are not accepted by this parser. Malformed input and zero
/// denominators retain the rational parser's typed errors.
///
/// Only Exhausted with an e/E marker selects the fallback: parse the significand
/// as Rational and the exponent as BigInt, then retain significand * 10^exponent
/// through powi. This avoids mandatory expanded numerator/denominator storage;
/// it does not guarantee cheap future evaluation or a total resource bound.
impl std::str::FromStr for Real {
    /// Preserve parse, domain and construction/resource failures from the owning scalar paths.
    type Err = Problem;

    /// Prefer a payload-free exact rational; retry only exhausted scientific input as an exact power.
    fn from_str(s: &str) -> Result<Self, Problem> {
        match s.parse::<Rational>() {
            Ok(rational) => Ok(Self {
                rational,
                class: One,
                computable: None,
                primitive_approx_cache: AtomicPrimitiveApproxCache::new(
                    PrimitiveApproxCache::Empty,
                ),
            }),
            Err(Problem::Exhausted) if s.contains(['e', 'E']) => {
                // Rational parsing deliberately bounds exponent-only storage
                // amplification. Real can retain the same exact mathematical
                // value as a lazy integer power without materializing that
                // numerator or denominator.
                let (significand, exponent) =
                    s.split_once(['e', 'E']).ok_or(Problem::BadDecimal)?;
                let significand = significand.parse::<Rational>()?;
                let exponent = exponent
                    .parse::<BigInt>()
                    .map_err(|_| Problem::BadDecimal)?;
                let scale = Self::from(10_u8).powi(exponent)?;
                crate::trace_dispatch!("real", "parse", "lazy-scientific-scale");
                Ok(Self::new(significand) * scale)
            }
            Err(problem) => Err(problem),
        }
    }
}
