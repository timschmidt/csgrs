/// Computable approximation of a Real number.
///
/// This is a demand-driven exact-real representation: every node can produce an
/// integer approximation at a requested binary precision, and caches store only
/// approximations proven for that node. The model follows the constructive/exact
/// real arithmetic approach in Boehm et al., "Exact real arithmetic: a case
/// study in higher order programming", <https://doi.org/10.1145/319838.319860>.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Computable {
    pub(super) internal: Rc<Approximation>,
    #[cfg_attr(feature = "serde", serde(skip))]
    pub(crate) cache: RefCell<Cache>,
    #[cfg_attr(feature = "serde", serde(skip))]
    pub(crate) bound: Cell<BoundCache>,
    #[cfg_attr(feature = "serde", serde(skip))]
    pub(crate) exact_sign: Cell<ExactSignCache>,
    #[cfg_attr(feature = "serde", serde(skip))]
    pub(crate) signal: Option<Signal>,
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
    LazyLock::new(|| Rational::fraction(79, 20).unwrap());
static INVERSE_ENDPOINT_RATIONAL_THRESHOLD: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(7, 8).unwrap());
static THREE_HALVES_RATIONAL: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(3, 2).unwrap());
static HALF_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::fraction(1, 2).unwrap());
