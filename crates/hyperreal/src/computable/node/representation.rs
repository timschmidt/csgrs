/// Computable approximation of a Real number.
///
/// This is a demand-driven exact-real representation: every node can produce an
/// integer approximation at a requested binary precision, and caches store only
/// approximations proven for that node.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg(not(verus_keep_ghost))]
pub struct Computable {
    pub(super) internal: Arc<Node>,
    #[cfg_attr(feature = "serde", serde(skip))]
    pub(crate) signal: Option<Signal>,
}

/// Immutable expression and its shared, synchronized accelerator state.
/// Keeping these in one allocation makes a `Computable` clone one pointer and
/// one atomic reference-count update instead of two of each.
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize), serde(transparent))]
#[cfg(not(verus_keep_ghost))]
pub(super) struct Node {
    #[cfg_attr(feature = "serde", serde(skip, default))]
    facts: AtomicFacts,
    approximation: Approximation,
    #[cfg_attr(feature = "serde", serde(skip, default))]
    cache: ApproximationCache,
}

#[cfg(not(verus_keep_ghost))]
impl Node {
    pub(crate) fn new(
        approximation: Approximation,
        bound: BoundCache,
        exact_sign: ExactSignCache,
    ) -> Self {
        let (contains_inverse_trig_or_pi, linear_demand) = match &approximation {
            Approximation::Add(left, right) => {
                let (left_inverse, left_demand) = left.internal.construction_hints();
                let (right_inverse, right_demand) = right.internal.construction_hints();
                (
                    left_inverse || right_inverse,
                    left_demand.max(right_demand).saturating_add(2),
                )
            }
            Approximation::Negate(child) => child.internal.construction_hints(),
            Approximation::Offset(child, shift) => {
                let (inverse, demand) = child.internal.construction_hints();
                let demand = i32::from(demand)
                    .saturating_add(*shift)
                    .clamp(i32::from(i16::MIN), i32::from(i16::MAX));
                (inverse, demand as i16)
            }
            _ => (
                Self::approximation_contains_inverse_trig_or_pi(&approximation),
                0,
            ),
        };
        Self {
            facts: AtomicFacts::new(bound, exact_sign, contains_inverse_trig_or_pi, linear_demand),
            approximation,
            cache: ApproximationCache::new(),
        }
    }

    fn construction_hints(&self) -> (bool, i16) {
        // Read both retained hints in the same atomic snapshot. The inverse
        // trig flag already needs this load during Add construction.
        let bits = self.facts.0.load(std::sync::atomic::Ordering::Relaxed);
        let contains_inverse_trig_or_pi = if bits & AtomicFacts::INVERSE_TRIG_OR_PI_KNOWN != 0 {
            bits & AtomicFacts::CONTAINS_INVERSE_TRIG_OR_PI != 0
        } else {
            self.contains_inverse_trig_or_pi()
        };
        let demand = ((bits & AtomicFacts::LINEAR_DEMAND_MASK)
            >> AtomicFacts::LINEAR_DEMAND_SHIFT) as u16 as i16;
        (contains_inverse_trig_or_pi, demand)
    }

    fn approximation_contains_inverse_trig_or_pi(approximation: &Approximation) -> bool {
        match approximation {
            Approximation::Constant(SharedConstant::Pi)
            | Approximation::AtanRational(_)
            | Approximation::PrescaledAtan(_)
            | Approximation::AtanDeferred(_)
            | Approximation::AcosPositive(_)
            | Approximation::AcosPositiveRational(_)
            | Approximation::AcosNegativeRational(_) => true,
            Approximation::Negate(child)
            | Approximation::Offset(child, _)
            | Approximation::SincSmall(child)
            | Approximation::CoscSmall(child) => {
                child.internal.contains_inverse_trig_or_pi()
            }
            Approximation::Add(left, right) | Approximation::Multiply(left, right) => {
                left.internal.contains_inverse_trig_or_pi()
                    || right.internal.contains_inverse_trig_or_pi()
            }
            _ => false,
        }
    }

    pub(crate) fn contains_inverse_trig_or_pi(&self) -> bool {
        if let Some(value) = self.facts.contains_inverse_trig_or_pi() {
            return value;
        }
        let value = Self::approximation_contains_inverse_trig_or_pi(&self.approximation);
        self.facts.store_contains_inverse_trig_or_pi(value);
        value
    }

    pub(crate) fn cached_at_precision(&self, p: Precision) -> Option<BigInt> {
        self.cache.at_precision(p)
    }

    pub(crate) fn cached_value(&self) -> Option<(Precision, BigInt)> {
        self.cache.get()
    }

    pub(crate) fn store_cache_value(&self, p: Precision, value: BigInt) {
        self.cache.store(p, value);
    }

    #[cfg(test)]
    pub(crate) fn cache_snapshot(&self) -> Option<(Precision, BigInt)> {
        self.cache.get()
    }
}

#[cfg(not(verus_keep_ghost))]
struct CachedApproximation {
    precision: Precision,
    value: BigInt,
}

/// Lazily allocated synchronized single-value cache. Keeping the value directly
/// inside the lock avoids a second allocation and atomic reference-count update
/// for every published approximation. Readers clone or coarsen the integer
/// under the read lock; only the owned result escapes the guard.
#[cfg(not(verus_keep_ghost))]
struct ApproximationCache(
    std::sync::atomic::AtomicPtr<std::sync::RwLock<Option<CachedApproximation>>>,
);

#[cfg(not(verus_keep_ghost))]
impl ApproximationCache {
    fn new() -> Self {
        Self(std::sync::atomic::AtomicPtr::new(std::ptr::null_mut()))
    }

    fn cell(&self) -> Option<&std::sync::RwLock<Option<CachedApproximation>>> {
        let pointer = self.0.load(std::sync::atomic::Ordering::Acquire);
        if pointer.is_null() {
            None
        } else {
            // SAFETY: the pointed-to cell is installed once and remains alive
            // until the enclosing Node is exclusively dropped.
            Some(unsafe { &*pointer })
        }
    }

    fn cell_or_init(&self) -> &std::sync::RwLock<Option<CachedApproximation>> {
        if let Some(cell) = self.cell() {
            return cell;
        }

        let allocated = Box::into_raw(Box::new(std::sync::RwLock::new(None)));
        let pointer = match self.0.compare_exchange(
            std::ptr::null_mut(),
            allocated,
            std::sync::atomic::Ordering::AcqRel,
            std::sync::atomic::Ordering::Acquire,
        ) {
            Ok(_) => allocated,
            Err(installed) => {
                // SAFETY: this allocation lost the installation race and was
                // never published.
                unsafe { drop(Box::from_raw(allocated)) };
                installed
            }
        };
        // SAFETY: the winning pointer remains installed until Node::drop.
        unsafe { &*pointer }
    }

    fn get(&self) -> Option<(Precision, BigInt)> {
        let guard = self.cell()?.read().unwrap_or_else(|error| error.into_inner());
        let cached = guard.as_ref()?;
        Some((cached.precision, cached.value.clone()))
    }

    fn at_precision(&self, p: Precision) -> Option<BigInt> {
        let guard = self.cell()?.read().unwrap_or_else(|error| error.into_inner());
        Self::value_at_precision(guard.as_ref()?, p)
    }

    #[inline(always)]
    fn value_at_precision(cached: &CachedApproximation, p: Precision) -> Option<BigInt> {
        if p < cached.precision {
            None
        } else if p == cached.precision {
            Some(cached.value.clone())
        } else {
            // The positive gap can exceed i32::MAX even for valid precisions.
            let gap = p.abs_diff(cached.precision);
            let value = if gap == 1 {
                // Preserve scale's direct-clone path for a zero initial shift.
                cached.value.clone()
            } else if u64::from(gap) > cached.value.bits() {
                return Some(BigInt::zero());
            } else {
                &cached.value >> (gap - 1)
            };
            // Nearest integer, with halfway values rounded toward +infinity.
            Some((value + signed::ONE.deref()) >> 1)
        }
    }

    fn store(&self, p: Precision, value: BigInt) {
        let mut guard = self
            .cell_or_init()
            .write()
            .unwrap_or_else(|error| error.into_inner());
        // Concurrent evaluations may finish out of order. Never let a coarser
        // result evict a finer result already published.
        if guard
            .as_ref()
            .is_none_or(|cached| p < cached.precision)
        {
            *guard = Some(CachedApproximation {
                precision: p,
                value,
            });
        }
    }
}

#[cfg(not(verus_keep_ghost))]
impl Default for ApproximationCache {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(not(verus_keep_ghost))]
impl std::fmt::Debug for ApproximationCache {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.get().fmt(formatter)
    }
}

#[cfg(not(verus_keep_ghost))]
impl Drop for ApproximationCache {
    fn drop(&mut self) {
        let pointer = *self.0.get_mut();
        if !pointer.is_null() {
            // SAFETY: dropping the enclosing Node proves exclusive access and
            // no guard can still borrow this cell.
            unsafe { drop(Box::from_raw(pointer)) };
        }
    }
}

#[cfg(not(verus_keep_ghost))]
impl Deref for Node {
    type Target = Approximation;

    fn deref(&self) -> &Self::Target {
        &self.approximation
    }
}

#[derive(Debug)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
struct AtomicFacts(std::sync::atomic::AtomicU64);

#[cfg(not(verus_keep_ghost))]
impl Default for AtomicFacts {
    fn default() -> Self {
        Self(std::sync::atomic::AtomicU64::new(0))
    }
}

impl AtomicFacts {
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const TAG_INVALID: u64 = 0;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const TAG_UNKNOWN: u64 = 1;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const TAG_ZERO: u64 = 2;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const TAG_NONZERO: u64 = 3;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const SIGN_SHIFT: u32 = 2;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const MSD_PRESENT: u64 = 1 << 4;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const EXACT_MSD: u64 = 1 << 5;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const EXACT_SIGN_SHIFT: u32 = 6;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const EXACT_SIGN_MASK: u64 = 0b111 << Self::EXACT_SIGN_SHIFT;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const INVERSE_TRIG_OR_PI_KNOWN: u64 = 1 << 9;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const CONTAINS_INVERSE_TRIG_OR_PI: u64 = 1 << 10;
    // Signed saturated precision decrement along Add/Negate/Offset paths.
    // Used only for child ordering, never as a numerical bound. Deserialized
    // nodes default to zero and retain ordinary evaluation order.
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const LINEAR_DEMAND_SHIFT: u32 = 16;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const LINEAR_DEMAND_MASK: u64 = (u16::MAX as u64) << Self::LINEAR_DEMAND_SHIFT;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const NON_BOUND_MASK: u64 = Self::EXACT_SIGN_MASK
        | Self::INVERSE_TRIG_OR_PI_KNOWN
        | Self::CONTAINS_INVERSE_TRIG_OR_PI
        | Self::LINEAR_DEMAND_MASK;
    #[cfg_attr(verus_keep_ghost, verus_verify)]
    const MSD_SHIFT: u32 = 32;

    #[cfg(not(verus_keep_ghost))]
    fn new(
        bound: BoundCache,
        exact_sign: ExactSignCache,
        contains_inverse_trig_or_pi: bool,
        linear_demand: i16,
    ) -> Self {
        Self(std::sync::atomic::AtomicU64::new(
            Self::encode_bound(bound)
                | Self::encode_exact_sign(exact_sign)
                | ((linear_demand as u16 as u64) << Self::LINEAR_DEMAND_SHIFT)
                | Self::INVERSE_TRIG_OR_PI_KNOWN
                | if contains_inverse_trig_or_pi {
                    Self::CONTAINS_INVERSE_TRIG_OR_PI
                } else {
                    0
                },
        ))
    }

    #[cfg(not(verus_keep_ghost))]
    fn linear_demand(&self) -> i16 {
        ((self.0.load(std::sync::atomic::Ordering::Relaxed) & Self::LINEAR_DEMAND_MASK)
            >> Self::LINEAR_DEMAND_SHIFT) as u16 as i16
    }

    #[cfg(not(verus_keep_ghost))]
    fn contains_inverse_trig_or_pi(&self) -> Option<bool> {
        let bits = self.0.load(std::sync::atomic::Ordering::Relaxed);
        (bits & Self::INVERSE_TRIG_OR_PI_KNOWN != 0)
            .then_some(bits & Self::CONTAINS_INVERSE_TRIG_OR_PI != 0)
    }

    #[cfg(not(verus_keep_ghost))]
    fn store_contains_inverse_trig_or_pi(&self, value: bool) {
        let bits = Self::INVERSE_TRIG_OR_PI_KNOWN
            | if value {
                Self::CONTAINS_INVERSE_TRIG_OR_PI
            } else {
                0
            };
        self.0.fetch_or(bits, std::sync::atomic::Ordering::Relaxed);
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == encoded_bound_bits(value),
    ))]
    fn encode_bound(value: BoundCache) -> u64 {
        #[cfg(verus_keep_ghost)]
        proof! {
            assert(Self::MSD_PRESENT == 16u64) by (bit_vector);
            assert(Self::EXACT_MSD == 32u64) by (bit_vector);
        }
        match value {
            BoundCache::Invalid => Self::TAG_INVALID,
            BoundCache::Valid(BoundInfo::Unknown) => Self::TAG_UNKNOWN,
            BoundCache::Valid(BoundInfo::Zero) => Self::TAG_ZERO,
            BoundCache::Valid(BoundInfo::NonZero {
                sign,
                msd,
                exact_msd,
            }) => {
                let sign = match sign {
                    None => 0,
                    Some(Sign::Plus) => 1,
                    Some(Sign::Minus) => 2,
                    Some(Sign::NoSign) => 3,
                };
                let mut encoded =
                    Self::TAG_NONZERO | ((sign as u64) << Self::SIGN_SHIFT);
                if let Some(msd) = msd {
                    encoded |= Self::MSD_PRESENT | ((msd as u32 as u64) << Self::MSD_SHIFT);
                }
                if exact_msd {
                    encoded |= Self::EXACT_MSD;
                }
                encoded
            }
        }
    }

    #[cfg(not(verus_keep_ghost))]
    fn decode_bound(value: u64) -> BoundCache {
        match value & 0b11 {
            Self::TAG_INVALID => BoundCache::Invalid,
            Self::TAG_UNKNOWN => BoundCache::Valid(BoundInfo::Unknown),
            Self::TAG_ZERO => BoundCache::Valid(BoundInfo::Zero),
            Self::TAG_NONZERO => {
                let sign = match (value >> Self::SIGN_SHIFT) & 0b11 {
                    0 => None,
                    1 => Some(Sign::Plus),
                    2 => Some(Sign::Minus),
                    3 => Some(Sign::NoSign),
                    _ => unreachable!(),
                };
                let msd = (value & Self::MSD_PRESENT != 0)
                    .then_some(((value >> Self::MSD_SHIFT) as u32) as i32);
                BoundCache::Valid(BoundInfo::NonZero {
                    sign,
                    msd,
                    exact_msd: value & Self::EXACT_MSD != 0,
                })
            }
            _ => unreachable!(),
        }
    }

    #[cfg(not(verus_keep_ghost))]
    fn bound(&self) -> BoundCache {
        Self::decode_bound(
            self.0.load(std::sync::atomic::Ordering::Relaxed) & !Self::EXACT_SIGN_MASK,
        )
    }

    #[cfg(not(verus_keep_ghost))]
    fn snapshot(&self) -> (BoundCache, ExactSignCache) {
        let encoded = self.0.load(std::sync::atomic::Ordering::Relaxed);
        (
            Self::decode_bound(encoded & !Self::EXACT_SIGN_MASK),
            Self::decode_exact_sign(encoded),
        )
    }

    #[cfg(not(verus_keep_ghost))]
    fn set_bound(&self, value: BoundCache) {
        let encoded = Self::encode_bound(value);
        let mut current = self.0.load(std::sync::atomic::Ordering::Relaxed);
        loop {
            let updated = (current & Self::NON_BOUND_MASK) | encoded;
            match self.0.compare_exchange_weak(
                current,
                updated,
                std::sync::atomic::Ordering::Relaxed,
                std::sync::atomic::Ordering::Relaxed,
            ) {
                Ok(_) => return,
                Err(observed) => current = observed,
            }
        }
    }

    #[cfg(not(verus_keep_ghost))]
    fn set_bound_if_invalid(&self, value: BoundCache) {
        let encoded = Self::encode_bound(value);
        let mut current = self.0.load(std::sync::atomic::Ordering::Relaxed);
        loop {
            if current & 0b11 != Self::TAG_INVALID {
                return;
            }
            let updated = (current & Self::NON_BOUND_MASK) | encoded;
            match self.0.compare_exchange_weak(
                current,
                updated,
                std::sync::atomic::Ordering::Relaxed,
                std::sync::atomic::Ordering::Relaxed,
            ) {
                Ok(_) => return,
                Err(observed) => current = observed,
            }
        }
    }

    #[cfg(not(verus_keep_ghost))]
    fn set_bound_if_unresolved(&self, value: BoundCache) {
        let encoded = Self::encode_bound(value);
        let mut current = self.0.load(std::sync::atomic::Ordering::Relaxed);
        loop {
            let tag = current & 0b11;
            if tag != Self::TAG_INVALID && tag != Self::TAG_UNKNOWN {
                return;
            }
            let updated = (current & Self::NON_BOUND_MASK) | encoded;
            match self.0.compare_exchange_weak(
                current,
                updated,
                std::sync::atomic::Ordering::Relaxed,
                std::sync::atomic::Ordering::Relaxed,
            ) {
                Ok(_) => return,
                Err(observed) => current = observed,
            }
        }
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures exact_sign_denotation(result) == value,
            result & !0x1c0u64 == 0,
            ((result >> 6) & 7) <= 4,
    ))]
    fn encode_exact_sign(value: ExactSignCache) -> u64 {
        let encoded = match value {
            ExactSignCache::Invalid => 0,
            ExactSignCache::Unknown => 1,
            ExactSignCache::Valid(Sign::Minus) => 2,
            ExactSignCache::Valid(Sign::NoSign) => 3,
            ExactSignCache::Valid(Sign::Plus) => 4,
        };
        #[cfg(verus_keep_ghost)]
        proof! {
            assert((((encoded as u64) << 6u32) >> 6u32) & 7u64 == encoded) by (bit_vector)
                requires encoded <= 4u64;
            assert(((encoded as u64) << 6u32) & !0x1c0u64 == 0u64) by (bit_vector)
                requires encoded <= 4u64;
        }
        encoded << Self::EXACT_SIGN_SHIFT
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        requires ((value >> 6) & 7) <= 4,
        ensures result == exact_sign_denotation(value),
    ))]
    fn decode_exact_sign(value: u64) -> ExactSignCache {
        #[cfg(verus_keep_ghost)]
        proof! {
            assert(Self::EXACT_SIGN_MASK == 0x1c0u64) by (bit_vector);
            assert((value & 0x1c0u64) >> 6u32 == (value >> 6u32) & 7u64) by (bit_vector);
        }
        match (value & Self::EXACT_SIGN_MASK) >> Self::EXACT_SIGN_SHIFT {
            0 => ExactSignCache::Invalid,
            1 => ExactSignCache::Unknown,
            2 => ExactSignCache::Valid(Sign::Minus),
            3 => ExactSignCache::Valid(Sign::NoSign),
            4 => ExactSignCache::Valid(Sign::Plus),
            _ => unreachable!("invalid atomic exact-sign cache state"),
        }
    }

    #[cfg(not(verus_keep_ghost))]
    fn exact_sign(&self) -> ExactSignCache {
        Self::decode_exact_sign(self.0.load(std::sync::atomic::Ordering::Relaxed))
    }

    #[cfg(not(verus_keep_ghost))]
    fn replace_exact_sign(&self, value: ExactSignCache) -> ExactSignCache {
        let encoded = Self::encode_exact_sign(value);
        let mut current = self.0.load(std::sync::atomic::Ordering::Relaxed);
        loop {
            let updated = (current & !Self::EXACT_SIGN_MASK) | encoded;
            match self.0.compare_exchange_weak(
                current,
                updated,
                std::sync::atomic::Ordering::Relaxed,
                std::sync::atomic::Ordering::Relaxed,
            ) {
                Ok(_) => return Self::decode_exact_sign(current),
                Err(observed) => current = observed,
            }
        }
    }
}

#[cfg(not(verus_keep_ghost))]
pub(crate) mod signed {
    use num::{BigInt, One};
    use std::sync::LazyLock;

    // Use the narrow primitive that holds each literal so `BigInt::from`
    // dispatches directly instead of routing through the `ToBigInt` helper.
    pub(crate) static MINUS_ONE: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(-1));
    pub(crate) static ONE: LazyLock<BigInt> = LazyLock::new(BigInt::one);
    pub(crate) static TWO: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(2_u8));
    pub(crate) static FOUR: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(4_u8));
    pub(crate) static SIX: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(6_u8));
    pub(crate) static EIGHT: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(8_u8));
    pub(crate) static SIXTEEN: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(16_u8));
    pub(crate) static TWENTY_FOUR: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(24_u8));
    pub(crate) static SIXTY_FOUR: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(64_u8));
}

#[cfg(not(verus_keep_ghost))]
pub(crate) mod unsigned {
    use num::{BigUint, One};
    use std::sync::LazyLock;

    // These are small non-negative constants, so `u8` is the exact source type
    // and avoids the extra conversion trait path used before the bigint audit.
    pub(crate) static ONE: LazyLock<BigUint> = LazyLock::new(BigUint::one);
    pub(crate) static TWO: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(2_u8));
    pub(crate) static TEN: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(10_u8));
    pub(crate) static FIVE: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(5_u8));
    pub(crate) static SIX: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(6_u8));
}

#[cfg(not(verus_keep_ghost))]
static HALF_PI_SHORTCUT_RATIONAL_LIMIT: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(3, 2).unwrap());
#[cfg(not(verus_keep_ghost))]
static NEAR_LARGE_RATIONAL_TRIG_THRESHOLD: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(7, 2).unwrap());
#[cfg(not(verus_keep_ghost))]
static INVERSE_ENDPOINT_RATIONAL_THRESHOLD: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(7, 8).unwrap());
#[cfg(not(verus_keep_ghost))]
static THREE_HALVES_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(3, 2).unwrap());
#[cfg(not(verus_keep_ghost))]
static HALF_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::fraction(1, 2).unwrap());
