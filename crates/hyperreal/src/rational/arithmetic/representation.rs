/// Ratio of two integers
///
/// This type is a [`Sign`]ed ratio between two [`BigUint`]
/// (the numerator and denominator). The numerator and denominator are finite.
///
/// The "ordinary" floating point numbers are rationals, but when converted
/// the exact rational may not be what you intuitively expected. It's obvious
/// that one third isn't represented exactly as an f64, but not everybody
/// will realize that 0.3 isn't either.
///
/// # Examples
///
/// Parsing a rational from a simple fraction
/// ```
/// use hyperreal::Rational;
/// let half: Rational = "9/18".parse().unwrap();
/// ```
///
/// Parsing a decimal fraction
/// ```
/// use hyperreal::Rational;
/// let point_two_five: Rational = "0.25".parse().unwrap();
/// ```
///
/// Converting a 64-bit floating point number
/// ```
/// use hyperreal::Rational;
/// let r: Rational = 0.3_f64.try_into().unwrap();
/// assert!(r != Rational::fraction(3, 10).unwrap());
/// ```
///
/// Simple arithmetic
/// ```
/// use hyperreal::Rational;
/// let quarter = Rational::fraction(1, 4).unwrap();
/// let eighteen = Rational::new(18);
/// let two = Rational::one() + Rational::one();
/// let sixteen = eighteen - two;
/// let four = quarter * sixteen;
/// assert_eq!(four, Rational::new(4));
/// ```

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct Rational {
    sign: Sign,
    numerator: BigUint,
    denominator: BigUint,
}

#[cfg(feature = "serde")]
impl<'de> Deserialize<'de> for Rational {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct RationalWire {
            sign: Sign,
            numerator: BigUint,
            denominator: BigUint,
        }

        let wire = RationalWire::deserialize(deserializer)?;
        if wire.denominator.is_zero() {
            return Err(serde::de::Error::custom(
                "Rational denominator must be nonzero",
            ));
        }
        Ok(Self::from_fraction_parts(wire.sign, wire.numerator, wire.denominator).reduce())
    }
}

static ONE: LazyLock<BigUint> = LazyLock::new(BigUint::one);
// Small positive constants use their narrow primitive source type; this keeps
// construction direct and avoids an intermediate `ToBigUint` conversion.
static TWO: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(2_u8));
static FIVE: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(5_u8));
static TEN: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(10_u8));
static RATIONAL_ONE: LazyLock<Rational> = LazyLock::new(Rational::one);

macro_rules! trace_rational_temporary {
    () => {{
        #[cfg(feature = "dispatch-trace")]
        crate::dispatch_trace::record_rational_temporary();
    }};
}

macro_rules! trace_rational_reduction {
    ($numerator:expr, $denominator:expr) => {{
        #[cfg(feature = "dispatch-trace")]
        crate::dispatch_trace::record_rational_reduction($numerator, $denominator);
    }};
}

macro_rules! trace_rational_gcd {
    ($left:expr, $right:expr, $divisor:expr) => {{
        #[cfg(feature = "dispatch-trace")]
        crate::dispatch_trace::record_rational_gcd($left, $right, $divisor);
    }};
}

macro_rules! trace_rational_power_of_two_common_factor {
    ($shift:expr) => {{
        #[cfg(feature = "dispatch-trace")]
        crate::dispatch_trace::record_rational_power_of_two_common_factor($shift);
    }};
}
