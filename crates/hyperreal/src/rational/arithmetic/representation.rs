/// Exact signed ratio of arbitrary-width integers with shared arithmetic evidence.
///
/// The value is a sign times a nonnegative numerator divided by a positive
/// denominator. Canonical zero has `NoSign`, numerator zero and denominator one.
/// Most construction paths reduce the fraction; some internal paths retain
/// unreduced parts with an explicit marker and canonicalize on demand. Equality
/// compares values, independently of storage identity or retained common factors.
///
/// Cloning shares storage, learned facts and cached exact results. Shared values
/// preserve their mathematical value; a consuming operation may reuse uniquely
/// owned storage after invalidating affected evidence. These caches accelerate
/// arithmetic but do not provide a total work, memory or cancellation budget.
/// Fixed slot counts do not bound the size of retained integer values or graphs.
///
/// Finite float imports preserve the exact binary value: `0.3_f64` is not `3/10`.
/// Use a fraction or decimal string for that rational value; NaN and infinities
/// are rejected by the fallible float conversions. With `serde`, serialization
/// emits canonical signed parts, while deserialization rejects a zero denominator
/// and normalizes the sign/zero representation and common factors. It does not
/// restore allocation identity, caches or learned facts.
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
pub struct Rational(
    /// Shared exact payload and its object-local caches; cloning this handle does
    /// not copy the integer limbs. Pointer identity is only a reuse shortcut.
    Arc<RationalData>
);

/// Owning, write-once heap slot with a null pointer as its uninitialized state.
///
/// Release publication and acquire reads permit shared access after construction.
/// A failed publisher receives its box back. Exclusive `clear` or destruction
/// drops the installed value, and `clear` permits later reuse. No reader may
/// outlive the slot. This compact state avoids a separate initialization word;
/// it is not a byte budget or an eviction policy for the pointed-to object.
struct CompactOnceBox<T>(
    /// Null when empty; otherwise the owned `Box<T>` pointer installed by `set`.
    std::sync::atomic::AtomicPtr<T>,
    /// Model ownership/drop and auto-trait constraints of the optional boxed value.
    /// The marker has no runtime payload.
    std::marker::PhantomData<Option<Box<T>>>,
);

/// Publish or exclusively clear one owned allocation while borrowing reads.
impl<T> CompactOnceBox<T> {
    /// Create an empty slot without allocating or initializing a `T`.
    const fn new() -> Self {
        Self(
            std::sync::atomic::AtomicPtr::new(std::ptr::null_mut()),
            std::marker::PhantomData,
        )
    }

    /// Acquire the installed pointer and borrow its value, or return `None`.
    ///
    /// The shared borrow prevents an exclusive clear while the returned reference
    /// is live. Acquire ordering observes initialization preceding publication.
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

    /// Publish a box only if the slot is empty, returning ownership on conflict.
    ///
    /// Release ordering exposes the fully initialized `T` to acquiring readers.
    /// A failed compare-exchange reconstructs exactly the submitted box; it neither
    /// replaces nor drops the winning allocation. Concurrent publishers may prepare
    /// duplicate boxes, so write-once retention does not bound temporary work.
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

    /// Reset the exclusively borrowed slot and drop any installed allocation.
    ///
    /// The pointer is made null before dropping the value; empty slots are a no-op.
    /// An arbitrary `T` destructor may panic. Exclusive access rules out live readers
    /// or concurrent publishers during this operation.
    fn clear(&mut self) {
        let pointer = std::mem::replace(self.0.get_mut(), std::ptr::null_mut());
        if !pointer.is_null() {
            // SAFETY: exclusive access prevents readers and the cell owns the
            // allocation installed by the successful exchange.
            drop(unsafe { Box::from_raw(pointer) });
        }
    }
}

/// Release the installed box when the slot itself is destroyed.
impl<T> Drop for CompactOnceBox<T> {
    /// Use the exclusive clear path, dropping an installed value exactly once.
    fn drop(&mut self) {
        self.clear();
    }
}

// SAFETY: publication uses release/acquire ordering; shared references require
// `T: Sync`, and the owning allocation may be dropped on another thread only
// when `T: Send`.
/// Allow concurrent shared slot access only for thread-safe owned values.
///
/// `T: Sync` permits acquired shared references and `T: Send` permits eventual
/// destruction on another thread. Publication uses release/acquire ordering and
/// removal requires exclusive access; these are the safety obligations of the
/// raw-pointer implementation. `Send` is governed by the fields' auto traits.
unsafe impl<T: Send + Sync> Sync for CompactOnceBox<T> {}

/// One retained exact product, or the canonical form of an unreduced owner.
///
/// The owner's immutable unreduced marker determines which meaning is allowed.
/// Binary lookup compares weak allocation identities while borrowing the live
/// other operand; the weak handle keeps the allocation identity from being reused
/// without retaining that operand's value. Results are held strongly until the
/// owner cache is cleared/dropped. No replacement or byte-size cap is provided.
struct CachedRationalProduct {
    /// Weak other-operand identity for a product; `None` reserves this slot
    /// for the owning unreduced fraction's canonical value.
    other: Option<std::sync::Weak<RationalData>>,
    /// Exact product or canonical fraction, interpreted according to `other`.
    result: Rational,
}

/// Interpretation tag for a retained binary entry or a unary/layout placeholder.
///
/// Placeholder entries share the record layout but are not binary arithmetic
/// results. Derived Clone/Copy duplicate the tag, Debug names it and Eq/PartialEq
/// compare tags only; none compares rational values or proves a construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CachedRationalLinearKind {
    /// Owner times the weakly identified other operand.
    Product,
    /// Owner plus the weakly identified other operand.
    Sum,
    /// Owner minus the weakly identified other operand.
    OwnerMinusOther,
    /// Weakly identified other operand minus the owner.
    OtherMinusOwner,
    /// Reciprocal held strongly in `result`; `other` is unused.
    StrongInversePlaceholder,
    /// Reciprocal reached by upgrading `other`; `result` is an inert zero.
    WeakInversePlaceholder,
    /// Additive inverse held strongly in `result`; `other` is unused.
    StrongNegationPlaceholder,
    /// Additive inverse reached by upgrading `other`; `result` is an inert zero.
    WeakNegationPlaceholder,
    /// Primary slot reserved when square reduction creates the cache first;
    /// its result is inert and the factors live in the separate square-reduction slot.
    SquareReductionPlaceholder,
}

/// Classify layout tags for the binary and unary cache-slot schedules.
impl CachedRationalLinearKind {
    /// Recognize either strong or weak reciprocal retention, without dereferencing it.
    #[inline]
    fn is_inverse_placeholder(self) -> bool {
        matches!(
            self,
            Self::StrongInversePlaceholder | Self::WeakInversePlaceholder
        )
    }

    /// Recognize either strong or weak additive-inverse retention.
    #[inline]
    fn is_negation_placeholder(self) -> bool {
        matches!(
            self,
            Self::StrongNegationPlaceholder | Self::WeakNegationPlaceholder
        )
    }

    /// Recognize reciprocal or negation tags; square reduction is excluded.
    #[inline]
    fn is_unary_placeholder(self) -> bool {
        self.is_inverse_placeholder() || self.is_negation_placeholder()
    }

    /// Identify a primary slot unavailable for binary results.
    ///
    /// Includes unary and square-reduction placeholders; consumers then reserve
    /// secondary and tertiary for their two binary entries.
    #[inline]
    fn is_primary_placeholder(self) -> bool {
        self.is_unary_placeholder() || matches!(self, Self::SquareReductionPlaceholder)
    }
}

/// Tagged exact cache record interpreted relative to the containing owner.
///
/// Binary records pair a weak other-operand key with a strong result. Unary tags
/// instead select a strong result or a weak target; placeholder zeros are never
/// arithmetic answers. Consumers must check the tag before interpreting fields.
struct CachedRationalLinearEntry {
    /// Binary operand identity or weak unary target; empty for strong unary
    /// and square-reduction placeholders. Expired weak targets are cache misses.
    other: std::sync::Weak<RationalData>,
    /// Operation direction or placeholder meaning controlling the two payloads.
    kind: CachedRationalLinearKind,
    /// Strong exact result for binary/strong-unary entries; inert canonical zero
    /// for weak-unary and square-reduction placeholders.
    result: Rational,
}

/// Ownership choice used when installing reciprocal or negation pairs.
///
/// The forward link can own its result while the reverse link is weak, avoiding
/// a two-way strong-reference cycle. This enum is an installation input; retained
/// entries encode the same distinction in their kind and shared payload fields.
enum CachedRationalUnary {
    /// Keep the exact unary result alive with the owner.
    Strong(
        /// Reciprocal or additive inverse selected by the installation routine.
        Rational
    ),
    /// Refer back to the source without keeping its value alive.
    Weak(
        /// Upgradeable source identity; expiration means no retained result.
        std::sync::Weak<RationalData>
    ),
}

/// Retained factors satisfying `owner = square^2 * rest`.
///
/// The scale is nonnegative and the residual is signed with an integer
/// denominator-one representation after rationalization. The extraction routine
/// owns the factorization domain and any limits; storing this pair adds no proof
/// that every possible square factor has been found.
struct CachedRationalSquareReduction {
    /// Nonnegative scale pulled outside the symbolic square root.
    square: Rational,
    /// Signed residual radicand; with the scale, reconstructs the original value.
    rest: Rational,
}

/// Lazy fixed-slot storage for binary, unary and square-reduction reuse.
///
/// A binary-first layout uses primary/secondary for two binary entries and later
/// slots for unary links. A placeholder-first layout shifts the binary pair to
/// secondary/tertiary and uses quaternary/quinary for later unary links. Consumers
/// choose and search these schedules; fields alone do not enforce them. Each slot
/// is write-once, with no live eviction or byte budget. Clearing the owner's box
/// or dropping it releases retained results; their reachable graphs can be large.
struct CachedRationalArithmetic {
    /// First installed binary entry or the placeholder that selects the layout.
    primary: CachedRationalLinearEntry,
    /// Second binary slot in a binary-first layout, first in a placeholder layout.
    secondary: OnceLock<CachedRationalLinearEntry>,
    /// Second binary slot after a primary placeholder, otherwise eligible for
    /// a later unary link. Readers inspect its tag before using it.
    tertiary: OnceLock<CachedRationalLinearEntry>,
    /// First later-unary slot for placeholder layouts; a fallback unary slot otherwise.
    quaternary: OnceLock<CachedRationalLinearEntry>,
    /// Final fallback slot for later reciprocal/negation publication.
    quinary: OnceLock<CachedRationalLinearEntry>,
    /// Independent retained scale/radicand pair, initialized at most once.
    square_reduction: OnceLock<CachedRationalSquareReduction>,
}

/// Arithmetic-use observation permitting later binary-result retention; not a value proof.
const RETAINED_LINEAR_REUSE_SEEN: u32 = 1 << 0;
/// Observation used to admit reuse for repeated small powers.
const RETAINED_POWER_REUSE_SEEN: u32 = 1 << 1;
/// Observation used to retain square extraction after its first attempt.
const RETAINED_SQUARE_REUSE_SEEN: u32 = 1 << 2;
/// Exact finite binary64 representability learned by import or exact conversion.
/// The bit is evidence only; it does not store a float or replace the exact parts.
const RETAINED_EXACT_F64_VIEW: u32 = 1 << 3;
/// Denominator power-of-two classification is known; read with its value bit.
const RETAINED_DYADIC_KNOWN: u32 = 1 << 4;
/// Known denominator is a power of two, meaningful with the known bit.
const RETAINED_DYADIC_VALUE: u32 = 1 << 5;
/// One conflicted self-dot retention attempt has been admitted.
const RETAINED_SELF_DOT_CONFLICT_ATTEMPTED: u32 = 1 << 6;
/// Stored common factors may remain; canonical queries must use the reserved cache.
/// This representation marker is established at construction, not a learned
/// mathematical property of an arbitrary new value.
const RETAINED_UNREDUCED_INTERNAL: u32 = 1 << 7;
/// Observation bits cleared when uniquely owned negation changes the cache key.
/// Magnitude/representation facts outside this mask survive that sign change.
const RETAINED_REUSE_MASK: u32 = RETAINED_LINEAR_REUSE_SEEN
    | RETAINED_POWER_REUSE_SEEN
    | RETAINED_SQUARE_REUSE_SEEN
    | RETAINED_SELF_DOT_CONFLICT_ATTEMPTED;
/// First bit of the denominator-exponent field; low eight bits hold flags.
const RETAINED_DYADIC_SHIFT_OFFSET: u32 = 8;
/// A relative-error binary64 filter view has been learned by its consumer.
/// This scheduling evidence does not assert exact binary64 representability.
const RETAINED_RELATIVE_F64_FILTER_VIEW: u32 = 1 << 31;
/// Bits 8 through 30 for denominator exponent plus one; zero means not encoded.
/// The top relative-filter bit is excluded, as are the low flag bits.
const RETAINED_DYADIC_SHIFT_MASK: u32 =
    (RETAINED_RELATIVE_F64_FILTER_VIEW - 1) & !((1 << RETAINED_DYADIC_SHIFT_OFFSET) - 1);
/// Largest directly retained exponent: 2^23 - 2. Larger dyadic denominators
/// remain admissible but need exponent recovery instead of this packed field.
const RETAINED_DYADIC_SHIFT_MAX: u64 =
    ((RETAINED_DYADIC_SHIFT_MASK >> RETAINED_DYADIC_SHIFT_OFFSET) - 1) as u64;

/// Shared rational payload exposed as the target of `Rational`'s `Deref`.
///
/// Fields remain private: external callers cannot construct or mutate this
/// storage through the dereference. Sign and magnitudes describe the exact
/// value; cache/fact fields are implementation evidence, not serialization data.
/// Shared arithmetic may publish evidence concurrently. An exclusively owned
/// consuming operation can change the sign after clearing affected caches.
/// Its public visibility permits the associated trait target; the crate root
/// does not directly re-export this name. Callers can name the target through
/// `<Rational as Deref>::Target`. This is not a separate modeling API or a
/// stable memory layout.
pub struct RationalData {
    /// Sign of the value; valid zero uses `NoSign`, nonzero uses `Plus` or `Minus`.
    sign: Sign,
    /// Unsigned numerator magnitude; common factors may remain only under
    /// the explicit internal-unreduced representation contract.
    numerator: BigUint,
    /// Positive denominator magnitude; zero is invalid stored rational state.
    denominator: BigUint,
    /// One product or canonicalized unreduced fraction; write-once until
    /// exclusive invalidation, with weak operand keys and strong exact results.
    product_cache: OnceLock<CachedRationalProduct>,
    /// Optional lazily allocated multi-operation cache, published atomically.
    /// A fixed slot layout bounds entry count, not retained value/graph size.
    linear_cache: CompactOnceBox<CachedRationalArithmetic>,
    /// Packed representation, conversion and reuse facts, updated by relaxed OR.
    /// Facts grow monotonically while the represented value is shared; exclusive
    /// sign changes clear reuse observations. This atomic does not publish other
    /// memory and is not a precision, work, memory or cancellation budget.
    retained_facts: std::sync::atomic::AtomicU32,
}

/// Format canonical mathematical parts, omitting caches and allocation identity.
///
/// Formatting can initialize an unreduced value's canonicalization cache. Output
/// is diagnostic Rust formatting, not a stable replay format or bounded operation.
impl std::fmt::Debug for Rational {
    /// Write a `Rational` debug struct containing sign, numerator and denominator.
    /// Propagates formatter errors after obtaining canonical parts.
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

/// Share the same exact payload, learned facts and cached results through `Arc`.
///
/// No integer limbs are copied. The default `clone_from` replaces the destination
/// with a shared clone and releases its prior handle.
impl Clone for Rational {
    /// Increment the shared payload's strong reference count and return its handle.
    #[inline]
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

/// Borrow the payload without reduction, copying or cache population.
///
/// Only immutable dereferencing is provided; payload fields remain private.
impl Deref for Rational {
    /// Private-field payload, nameable externally through this associated type.
    type Target = RationalData;

    /// Borrow the exact shared payload for this handle's lifetime.
    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

/// Serialize canonical signed integer parts through the selected Serde format.
///
/// The three named fields are sign, numerator and denominator. Backend integer
/// encodings are supplied by num-bigint's Serde implementations. Allocation
/// identity, caches and retained bits are omitted. Canonicalization may populate
/// a cache and perform unbounded integer work; serialization is not a resource cap.
#[cfg(feature = "serde")]
impl Serialize for Rational {
    /// Obtain canonical parts and emit a three-field `Rational` struct.
    /// Propagate serializer failures from struct creation, each field and completion.
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

/// Read signed parts, reject a zero denominator and normalize the fraction.
///
/// Zero numerator or `NoSign` becomes canonical zero, including a nonzero
/// numerator paired with `NoSign`; inconsistent sign/zero inputs are normalized,
/// not all rejected. Nonzero parts are reduced exactly. Serde errors remain
/// deserializer errors rather than `Problem`. Decoding and reduction have no
/// local work/byte limit; caches and source allocation identity are not replayed.
#[cfg(feature = "serde")]
impl<'de> Deserialize<'de> for Rational {
    /// Decode the temporary wire record, validate the denominator, then reduce.
    /// The denominator check precedes sign/zero normalization, so even a zero-valued
    /// record with a zero denominator is rejected.
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        /// Temporary owned Serde record before rational validation.
        ///
        /// Derived deserialization reads the three required fields using their own
        /// Serde implementations, accepts sequence/map forms supported by the format,
        /// and by default ignores unknown map fields. It rejects missing/duplicate
        /// known fields and invalid component encodings, but does not check denominator
        /// nonzeroness or normalize sign/common factors; the enclosing method does that.
        #[derive(Deserialize)]
        struct RationalWire {
            /// Decoded sign tag, which may still disagree with the numerator.
            sign: Sign,
            /// Decoded unsigned magnitude, not yet normalized or reduced.
            numerator: BigUint,
            /// Decoded unsigned denominator, still permitted to be zero here.
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

/// Process-lifetime integer one, initialized on first access for borrowed comparisons.
static ONE: LazyLock<BigUint> = LazyLock::new(BigUint::one);
// Small positive constants use their narrow primitive source type; this keeps
// construction direct and avoids an intermediate `ToBigUint` conversion.
/// Lazily initialized integer two for radix/divisibility arithmetic.
static TWO: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(2_u8));
/// Lazily initialized integer five for decimal factor tests.
static FIVE: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(5_u8));
/// Lazily initialized integer ten for decimal scaling and digit conversion.
static TEN: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(10_u8));
/// Shared canonical zero (NoSign, 0/1), retained for the process lifetime.
static RATIONAL_ZERO: LazyLock<Rational> = LazyLock::new(|| {
    Rational::from_parts_raw(NoSign, BigUint::ZERO, BigUint::one())
});
/// Shared canonical positive unit (Plus, 1/1), retained for the process lifetime.
static RATIONAL_ONE: LazyLock<Rational> =
    LazyLock::new(|| Rational::from_parts_raw(Plus, BigUint::one(), BigUint::one()));
/// Shared canonical negative unit (Minus, 1/1), retained for the process lifetime.
static RATIONAL_MINUS_ONE: LazyLock<Rational> =
    LazyLock::new(|| Rational::from_parts_raw(Minus, BigUint::one(), BigUint::one()));
/// Lazy process-lifetime intern slots for positive integers 2 through 64.
/// Index is magnitude minus two; initialized values and their caches are retained.
static SMALL_POSITIVE_RATIONALS: [OnceLock<Rational>; 63] =
    [const { OnceLock::new() }; 63];
/// Lazy process-lifetime intern slots for negative integers -2 through -64.
/// Index is absolute magnitude minus two; there is no eviction or reset.
static SMALL_NEGATIVE_RATIONALS: [OnceLock<Rational>; 63] =
    [const { OnceLock::new() }; 63];
/// Number of positive odd magnitudes 1,3,...,63 eligible for each dyadic exponent.
const SMALL_DYADIC_ODD_MAGNITUDES: usize = 32;
/// Largest denominator exponent in the small dyadic intern pool, inclusive.
const SMALL_DYADIC_MAX_SHIFT: usize = 63;
/// 2,016 lazy slots per sign: 32 odd magnitudes times 63 exponents.
const SMALL_DYADIC_CACHE_LEN: usize =
    SMALL_DYADIC_ODD_MAGNITUDES * SMALL_DYADIC_MAX_SHIFT;
/// Lazy process-lifetime positive fractions m/2^s for odd m in 1..=63, s in 1..=63.
/// Index is (s-1)*32 + (m>>1); each populated slot retains its rational and caches.
static SMALL_POSITIVE_DYADICS: [OnceLock<Rational>; SMALL_DYADIC_CACHE_LEN] =
    [const { OnceLock::new() }; SMALL_DYADIC_CACHE_LEN];
/// Negative counterparts of the positive dyadic pool, with the same indexing
/// and process-lifetime retention; signs occupy separate pools.
static SMALL_NEGATIVE_DYADICS: [OnceLock<Rational>; SMALL_DYADIC_CACHE_LEN] =
    [const { OnceLock::new() }; SMALL_DYADIC_CACHE_LEN];
// Geometry repeatedly produces reduced word-sized fractions with small
// non-dyadic parts. Keep the same inclusive 63 boundary as the scalar and
// dyadic caches; lazy slots retain only values that a process actually uses.
/// Inclusive numerator bound for the reduced non-dyadic fraction pool.
const SMALL_GENERAL_MAX_MAGNITUDE: usize = 63;
/// Inclusive denominator bound for the reduced non-dyadic fraction pool.
const SMALL_GENERAL_MAX_DENOMINATOR: usize = 63;
/// 3,969 allocated slots per sign, including combinations never admitted.
const SMALL_GENERAL_CACHE_LEN: usize =
    SMALL_GENERAL_MAX_MAGNITUDE * SMALL_GENERAL_MAX_DENOMINATOR;
/// Lazy positive reduced non-dyadic fractions with numerator/denominator <=63.
/// Index is (denominator-1)*63 + numerator-1. Slots for unused or ineligible
/// combinations remain empty; populated rationals and their caches do not evict.
static SMALL_POSITIVE_GENERAL_RATIONALS: [OnceLock<Rational>; SMALL_GENERAL_CACHE_LEN] =
    [const { OnceLock::new() }; SMALL_GENERAL_CACHE_LEN];
/// Negative counterparts of the general fraction pool, using the same index.
/// The fixed slot count does not cap reachable arithmetic-cache memory.
static SMALL_NEGATIVE_GENERAL_RATIONALS: [OnceLock<Rational>; SMALL_GENERAL_CACHE_LEN] =
    [const { OnceLock::new() }; SMALL_GENERAL_CACHE_LEN];

/// Read and monotonically accumulate object-local bit evidence.
///
/// Relaxed atomics are sufficient for these self-contained bits; they do not
/// publish cache payloads. Callers must set only justified compatible flags and
/// encoded exponents. Exclusive value changes handle invalidation separately.
impl Rational {
    /// Return whether any requested bit is present in one relaxed snapshot.
    ///
    /// A multi-bit mask means any, not all; a zero mask returns false. No fact is
    /// derived from the value and no missing bit is interpreted as proof of falsity.
    #[inline]
    fn retained_fact(&self, fact: u32) -> bool {
        self.retained_facts
            .load(std::sync::atomic::Ordering::Relaxed)
            & fact
            != 0
    }

    /// OR justified bits into the shared word without changing the exact value.
    ///
    /// This does not validate the mask or merge different exponent encodings;
    /// callers must preserve the packed-field invariant. It publishes no other data.
    #[inline]
    fn retain_fact(&self, fact: u32) {
        self.retained_facts
            .fetch_or(fact, std::sync::atomic::Ordering::Relaxed);
    }

    /// Atomically OR a mask and report whether any of its bits were already set.
    ///
    /// For a single initially clear bit, exactly one racing fetch-or observes
    /// false before any exclusive reset; subsequent observers see true. A mask
    /// with several bits tests prior overlap, not whether all were present.
    /// A zero mask always returns false. Relaxed ordering publishes no unrelated
    /// state; callers must justify the evidence independently.
    #[inline]
    fn observe_retained_fact(&self, fact: u32) -> bool {
        self.retained_facts
            .fetch_or(fact, std::sync::atomic::Ordering::Relaxed)
            & fact
            != 0
    }

}

/// Record a rational-construction event when dispatch tracing is compiled.
///
/// The thread-local recorder counts only within an active recording scope.
/// Interned constant clones can count as temporaries, so this is not an
/// allocation counter or a measurement of retained bytes.
macro_rules! trace_rational_temporary {
    () => {{
        #[cfg(feature = "dispatch-trace")]
        crate::dispatch_trace::record_rational_temporary();
    }};
}

/// Record a reduction attempt and operand widths under dispatch tracing.
///
/// This does not perform reduction or prove a factor was removed. Arguments
/// are not evaluated when the feature is absent; with it enabled they evaluate
/// before the recorder checks whether the current thread is recording.
macro_rules! trace_rational_reduction {
    ($numerator:expr, $denominator:expr) => {{
        #[cfg(feature = "dispatch-trace")]
        crate::dispatch_trace::record_rational_reduction($numerator, $denominator);
    }};
}

/// Record a completed GCD observation, operand widths and divisor category.
///
/// The macro trusts the supplied operands/divisor and does not compute or verify
/// the GCD. Arguments disappear without dispatch tracing; otherwise evaluation
/// precedes the recorder's active-scope check.
macro_rules! trace_rational_gcd {
    ($left:expr, $right:expr, $divisor:expr) => {{
        #[cfg(feature = "dispatch-trace")]
        {
            crate::dispatch_trace::record_rational_gcd($left, $right, $divisor);
        }
    }};
}

/// Submit the inferred backend division label for an operation and operands.
///
/// With dispatch tracing disabled none of the arguments evaluate. Enabled
/// classification precedes the recording-scope check and observes operand shape,
/// not actual backend execution or successful division; zero is not validated.
macro_rules! trace_rational_division_algorithm {
    ($operation:expr, $dividend:expr, $divisor:expr) => {{
        #[cfg(feature = "dispatch-trace")]
        Rational::trace_backend_division($operation, $dividend, $divisor);
    }};
}

/// Submit the operand-width radix-output diagnostic under dispatch tracing.
///
/// The label does not observe conversion or identify its radix-specific backend
/// branch. The argument disappears when tracing is not compiled; otherwise it
/// evaluates even outside an active recording scope.
macro_rules! trace_rational_radix_output_algorithm {
    ($value:expr) => {{
        #[cfg(feature = "dispatch-trace")]
        Rational::trace_backend_radix_output($value);
    }};
}

/// Record whether an observed common binary shift is zero or positive.
///
/// No reduction or factor validation occurs here. Without dispatch tracing the
/// shift expression is not evaluated; with it enabled the thread-local recorder
/// accepts the event only while recording.
macro_rules! trace_rational_power_of_two_common_factor {
    ($shift:expr) => {{
        #[cfg(feature = "dispatch-trace")]
        crate::dispatch_trace::record_rational_power_of_two_common_factor($shift);
    }};
}
