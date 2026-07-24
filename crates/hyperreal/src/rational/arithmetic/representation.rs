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
pub struct Rational(Arc<RationalData>);

// The null pointer is the initialization state, avoiding `OnceLock`'s
// separate state word for a value that is already heap allocated.
struct CompactOnceBox<T>(
    std::sync::atomic::AtomicPtr<T>,
    std::marker::PhantomData<Option<Box<T>>>,
);

impl<T> CompactOnceBox<T> {
    const fn new() -> Self {
        Self(
            std::sync::atomic::AtomicPtr::new(std::ptr::null_mut()),
            std::marker::PhantomData,
        )
    }

    #[inline(always)]
    fn get(&self) -> Option<&T> {
        let pointer = self.0.load(std::sync::atomic::Ordering::Acquire);
        if pointer.is_null() {
            None
        } else {
            // SAFETY: a successful `set` owns this allocation until `self` is
            // exclusively cleared or dropped, so a shared borrow keeps it live.
            Some(unsafe { &*pointer })
        }
    }

    fn set(&self, value: Box<T>) -> Result<(), Box<T>> {
        let pointer = Box::into_raw(value);
        if self
            .0
            .compare_exchange(
                std::ptr::null_mut(),
                pointer,
                std::sync::atomic::Ordering::Release,
                std::sync::atomic::Ordering::Acquire,
            )
            .is_ok()
        {
            Ok(())
        } else {
            // SAFETY: the failed exchange never transferred ownership.
            Err(unsafe { Box::from_raw(pointer) })
        }
    }

    fn clear(&mut self) {
        let pointer = std::mem::replace(self.0.get_mut(), std::ptr::null_mut());
        if !pointer.is_null() {
            // SAFETY: exclusive access prevents readers and the cell owns the
            // allocation installed by the successful exchange.
            drop(unsafe { Box::from_raw(pointer) });
        }
    }
}

impl<T> Drop for CompactOnceBox<T> {
    fn drop(&mut self) {
        self.clear();
    }
}

// SAFETY: publication uses release/acquire ordering; shared references require
// `T: Sync`, and the owning allocation may be dropped on another thread only
// when `T: Send`.
unsafe impl<T: Send + Sync> Sync for CompactOnceBox<T> {}

struct CachedRationalProduct {
    // `None` reserves the primary cache for the canonical value of an
    // internally unreduced rational.
    other: Option<std::sync::Weak<RationalData>>,
    result: Rational,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CachedRationalLinearKind {
    Product,
    Sum,
    OwnerMinusOther,
    OtherMinusOwner,
    StrongInversePlaceholder,
    WeakInversePlaceholder,
    StrongNegationPlaceholder,
    WeakNegationPlaceholder,
    SquareReductionPlaceholder,
}

impl CachedRationalLinearKind {
    #[inline]
    fn is_inverse_placeholder(self) -> bool {
        matches!(
            self,
            Self::StrongInversePlaceholder | Self::WeakInversePlaceholder
        )
    }

    #[inline]
    fn is_negation_placeholder(self) -> bool {
        matches!(
            self,
            Self::StrongNegationPlaceholder | Self::WeakNegationPlaceholder
        )
    }

    #[inline]
    fn is_unary_placeholder(self) -> bool {
        self.is_inverse_placeholder() || self.is_negation_placeholder()
    }

    #[inline]
    fn is_primary_placeholder(self) -> bool {
        self.is_unary_placeholder() || matches!(self, Self::SquareReductionPlaceholder)
    }
}

struct CachedRationalLinearEntry {
    other: std::sync::Weak<RationalData>,
    kind: CachedRationalLinearKind,
    result: Rational,
}

enum CachedRationalUnary {
    Strong(Rational),
    Weak(std::sync::Weak<RationalData>),
}

struct CachedRationalSquareReduction {
    square: Rational,
    rest: Rational,
}

struct CachedRationalArithmetic {
    primary: CachedRationalLinearEntry,
    secondary: OnceLock<CachedRationalLinearEntry>,
    tertiary: OnceLock<CachedRationalLinearEntry>,
    quaternary: OnceLock<CachedRationalLinearEntry>,
    quinary: OnceLock<CachedRationalLinearEntry>,
    square_reduction: OnceLock<CachedRationalSquareReduction>,
}

const RETAINED_LINEAR_REUSE_SEEN: u8 = 1 << 0;
const RETAINED_POWER_REUSE_SEEN: u8 = 1 << 1;
const RETAINED_SQUARE_REUSE_SEEN: u8 = 1 << 2;
const RETAINED_EXACT_F64_VIEW: u8 = 1 << 3;
const RETAINED_DYADIC_KNOWN: u8 = 1 << 4;
const RETAINED_DYADIC_VALUE: u8 = 1 << 5;
const RETAINED_SELF_DOT_CONFLICT_ATTEMPTED: u8 = 1 << 6;
const RETAINED_UNREDUCED_INTERNAL: u8 = 1 << 7;
const RETAINED_REUSE_MASK: u8 = RETAINED_LINEAR_REUSE_SEEN
    | RETAINED_POWER_REUSE_SEEN
    | RETAINED_SQUARE_REUSE_SEEN
    | RETAINED_SELF_DOT_CONFLICT_ATTEMPTED;

#[doc(hidden)]
pub struct RationalData {
    sign: Sign,
    numerator: BigUint,
    denominator: BigUint,
    product_cache: OnceLock<CachedRationalProduct>,
    linear_cache: CompactOnceBox<CachedRationalArithmetic>,
    /// Monotonic representation and reuse evidence retained by this immutable
    /// rational. Packing these facts keeps the node layout bounded while
    /// leaving room for additional benchmark-proven dispatch certificates.
    retained_facts: std::sync::atomic::AtomicU8,
}

impl std::fmt::Debug for Rational {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = self.canonicalized_ref();
        formatter
            .debug_struct("Rational")
            .field("sign", &value.sign)
            .field("numerator", &value.numerator)
            .field("denominator", &value.denominator)
            .finish()
    }
}

impl Clone for Rational {
    #[inline]
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl Deref for Rational {
    type Target = RationalData;

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[cfg(feature = "serde")]
impl Serialize for Rational {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;

        let value = self.canonicalized_ref();
        let mut state = serializer.serialize_struct("Rational", 3)?;
        state.serialize_field("sign", &value.sign)?;
        state.serialize_field("numerator", &value.numerator)?;
        state.serialize_field("denominator", &value.denominator)?;
        state.end()
    }
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
static RATIONAL_ZERO: LazyLock<Rational> = LazyLock::new(|| {
    Rational::from_parts_raw(NoSign, BigUint::ZERO, BigUint::one())
});
static RATIONAL_ONE: LazyLock<Rational> =
    LazyLock::new(|| Rational::from_parts_raw(Plus, BigUint::one(), BigUint::one()));
static RATIONAL_MINUS_ONE: LazyLock<Rational> =
    LazyLock::new(|| Rational::from_parts_raw(Minus, BigUint::one(), BigUint::one()));
static SMALL_POSITIVE_RATIONALS: [OnceLock<Rational>; 63] =
    [const { OnceLock::new() }; 63];
static SMALL_NEGATIVE_RATIONALS: [OnceLock<Rational>; 63] =
    [const { OnceLock::new() }; 63];
const SMALL_DYADIC_ODD_MAGNITUDES: usize = 32;
const SMALL_DYADIC_MAX_SHIFT: usize = 63;
const SMALL_DYADIC_CACHE_LEN: usize =
    SMALL_DYADIC_ODD_MAGNITUDES * SMALL_DYADIC_MAX_SHIFT;
static SMALL_POSITIVE_DYADICS: [OnceLock<Rational>; SMALL_DYADIC_CACHE_LEN] =
    [const { OnceLock::new() }; SMALL_DYADIC_CACHE_LEN];
static SMALL_NEGATIVE_DYADICS: [OnceLock<Rational>; SMALL_DYADIC_CACHE_LEN] =
    [const { OnceLock::new() }; SMALL_DYADIC_CACHE_LEN];
// Geometry repeatedly produces reduced word-sized fractions with small
// non-dyadic parts. Keep the same inclusive 63 boundary as the scalar and
// dyadic caches; lazy slots retain only values that a process actually uses.
const SMALL_GENERAL_MAX_MAGNITUDE: usize = 63;
const SMALL_GENERAL_MAX_DENOMINATOR: usize = 63;
const SMALL_GENERAL_CACHE_LEN: usize =
    SMALL_GENERAL_MAX_MAGNITUDE * SMALL_GENERAL_MAX_DENOMINATOR;
static SMALL_POSITIVE_GENERAL_RATIONALS: [OnceLock<Rational>; SMALL_GENERAL_CACHE_LEN] =
    [const { OnceLock::new() }; SMALL_GENERAL_CACHE_LEN];
static SMALL_NEGATIVE_GENERAL_RATIONALS: [OnceLock<Rational>; SMALL_GENERAL_CACHE_LEN] =
    [const { OnceLock::new() }; SMALL_GENERAL_CACHE_LEN];

impl Rational {
    #[inline]
    fn retained_fact(&self, fact: u8) -> bool {
        self.retained_facts
            .load(std::sync::atomic::Ordering::Relaxed)
            & fact
            != 0
    }

    #[inline]
    fn retain_fact(&self, fact: u8) {
        self.retained_facts
            .fetch_or(fact, std::sync::atomic::Ordering::Relaxed);
    }

    /// Mark a monotonic observation and return whether it had already been
    /// retained. Racing observers may both take the cold path; subsequent
    /// calls see the evidence without locks or node growth.
    #[inline]
    fn observe_retained_fact(&self, fact: u8) -> bool {
        self.retained_facts
            .fetch_or(fact, std::sync::atomic::Ordering::Relaxed)
            & fact
            != 0
    }

}

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
        {
            crate::dispatch_trace::record_rational_gcd($left, $right, $divisor);
        }
    }};
}

macro_rules! trace_rational_division_algorithm {
    ($operation:expr, $dividend:expr, $divisor:expr) => {{
        #[cfg(feature = "dispatch-trace")]
        Rational::trace_backend_division($operation, $dividend, $divisor);
    }};
}

macro_rules! trace_rational_radix_output_algorithm {
    ($value:expr) => {{
        #[cfg(feature = "dispatch-trace")]
        Rational::trace_backend_radix_output($value);
    }};
}

macro_rules! trace_rational_power_of_two_common_factor {
    ($shift:expr) => {{
        #[cfg(feature = "dispatch-trace")]
        crate::dispatch_trace::record_rational_power_of_two_common_factor($shift);
    }};
}
