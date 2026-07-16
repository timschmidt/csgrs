/// Computable approximation of a Real number.
///
/// This is a demand-driven exact-real representation: every node can produce an
/// integer approximation at a requested binary precision, and caches store only
/// approximations proven for that node.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
pub(super) struct Node {
    #[cfg_attr(feature = "serde", serde(skip, default))]
    facts: AtomicFacts,
    approximation: Approximation,
    #[cfg_attr(feature = "serde", serde(skip, default))]
    cache: ApproximationCache,
}

impl Node {
    pub(crate) fn new(
        approximation: Approximation,
        bound: BoundCache,
        exact_sign: ExactSignCache,
    ) -> Self {
        Self {
            facts: AtomicFacts::new(bound, exact_sign),
            approximation,
            cache: ApproximationCache::new(),
        }
    }

    pub(crate) fn cached_at_precision(&self, p: Precision) -> Option<BigInt> {
        self.cache.at_precision(p)
    }

    pub(crate) fn cached_value(&self) -> Option<(Precision, BigInt)> {
        self.cache.get()
    }

    pub(crate) fn cached_sign(&self) -> Option<Sign> {
        self.cache.sign()
    }

    pub(crate) fn store_cache_value(&self, p: Precision, value: BigInt) {
        self.cache.store(p, value);
    }

    #[cfg(test)]
    pub(crate) fn cache_snapshot(&self) -> Option<(Precision, BigInt)> {
        self.cache.get()
    }
}

struct CachedApproximation {
    precision: Precision,
    value: BigInt,
}

/// Lazily allocated synchronized single-value cache. Keeping the value directly
/// inside the lock avoids a second allocation and atomic reference-count update
/// for every published approximation. The short read-side critical section only
/// clones the integer before releasing the lock.
struct ApproximationCache(
    std::sync::atomic::AtomicPtr<std::sync::RwLock<Option<CachedApproximation>>>,
);

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
            Some(scale(cached.value.clone(), cached.precision - p))
        }
    }

    fn sign(&self) -> Option<Sign> {
        let guard = self.cell()?.read().unwrap_or_else(|error| error.into_inner());
        guard.as_ref().map(|cached| cached.value.sign())
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

impl Default for ApproximationCache {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for ApproximationCache {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.get().fmt(formatter)
    }
}

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

impl Deref for Node {
    type Target = Approximation;

    fn deref(&self) -> &Self::Target {
        &self.approximation
    }
}

#[derive(Debug)]
struct AtomicFacts(std::sync::atomic::AtomicU64);

impl Default for AtomicFacts {
    fn default() -> Self {
        Self::new(BoundCache::Invalid, ExactSignCache::Invalid)
    }
}

impl AtomicFacts {
    const TAG_INVALID: u64 = 0;
    const TAG_UNKNOWN: u64 = 1;
    const TAG_ZERO: u64 = 2;
    const TAG_NONZERO: u64 = 3;
    const SIGN_SHIFT: u32 = 2;
    const MSD_PRESENT: u64 = 1 << 4;
    const EXACT_MSD: u64 = 1 << 5;
    const EXACT_SIGN_SHIFT: u32 = 6;
    const EXACT_SIGN_MASK: u64 = 0b111 << Self::EXACT_SIGN_SHIFT;
    const MSD_SHIFT: u32 = 32;

    fn new(bound: BoundCache, exact_sign: ExactSignCache) -> Self {
        Self(std::sync::atomic::AtomicU64::new(
            Self::encode_bound(bound) | Self::encode_exact_sign(exact_sign),
        ))
    }

    fn encode_bound(value: BoundCache) -> u64 {
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

    fn bound(&self) -> BoundCache {
        Self::decode_bound(
            self.0.load(std::sync::atomic::Ordering::Relaxed) & !Self::EXACT_SIGN_MASK,
        )
    }

    fn snapshot(&self) -> (BoundCache, ExactSignCache) {
        let encoded = self.0.load(std::sync::atomic::Ordering::Relaxed);
        (
            Self::decode_bound(encoded & !Self::EXACT_SIGN_MASK),
            Self::decode_exact_sign(encoded),
        )
    }

    fn set_bound(&self, value: BoundCache) {
        let encoded = Self::encode_bound(value);
        let mut current = self.0.load(std::sync::atomic::Ordering::Relaxed);
        loop {
            let updated = (current & Self::EXACT_SIGN_MASK) | encoded;
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

    fn set_bound_if_invalid(&self, value: BoundCache) {
        let encoded = Self::encode_bound(value);
        let mut current = self.0.load(std::sync::atomic::Ordering::Relaxed);
        loop {
            if current & 0b11 != Self::TAG_INVALID {
                return;
            }
            let updated = (current & Self::EXACT_SIGN_MASK) | encoded;
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

    fn encode_exact_sign(value: ExactSignCache) -> u64 {
        let encoded = match value {
            ExactSignCache::Invalid => 0,
            ExactSignCache::Unknown => 1,
            ExactSignCache::Valid(Sign::Minus) => 2,
            ExactSignCache::Valid(Sign::NoSign) => 3,
            ExactSignCache::Valid(Sign::Plus) => 4,
        };
        encoded << Self::EXACT_SIGN_SHIFT
    }

    fn decode_exact_sign(value: u64) -> ExactSignCache {
        match (value & Self::EXACT_SIGN_MASK) >> Self::EXACT_SIGN_SHIFT {
            0 => ExactSignCache::Invalid,
            1 => ExactSignCache::Unknown,
            2 => ExactSignCache::Valid(Sign::Minus),
            3 => ExactSignCache::Valid(Sign::NoSign),
            4 => ExactSignCache::Valid(Sign::Plus),
            _ => unreachable!("invalid atomic exact-sign cache state"),
        }
    }

    fn exact_sign(&self) -> ExactSignCache {
        Self::decode_exact_sign(self.0.load(std::sync::atomic::Ordering::Relaxed))
    }

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

static HALF_PI_SHORTCUT_RATIONAL_LIMIT: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(3, 2).unwrap());
static NEAR_LARGE_RATIONAL_TRIG_THRESHOLD: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(7, 2).unwrap());
static INVERSE_ENDPOINT_RATIONAL_THRESHOLD: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(7, 8).unwrap());
static THREE_HALVES_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(3, 2).unwrap());
static HALF_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::fraction(1, 2).unwrap());
