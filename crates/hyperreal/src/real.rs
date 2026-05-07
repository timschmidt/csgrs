use crate::{
    Computable, MagnitudeBits, Problem, Rational, RealSign, RealStructuralFacts, ZeroKnowledge,
};
use num::bigint::{BigInt, BigUint, Sign};

mod convert;
mod test;

#[derive(Clone, Debug, Serialize, Deserialize)]
struct ConstProductClass {
    // Signed pi power lets reciprocal products such as 1/pi and e^q/pi remain
    // symbolic instead of falling into a generic inverse node.
    pi_power: i16,
    exp_power: Rational,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct ConstOffsetClass {
    // Invariant: the inner value pi^n*e^q + offset is constructed only when
    // cheaply certified positive. The outer Real.rational carries any sign.
    pi_power: i16,
    exp_power: Rational,
    offset: Rational,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct ConstProductSqrtClass {
    // Factored sqrt products are positive internally. Keeping the sqrt separate
    // allows later multiplication/division to cancel it exactly.
    pi_power: i16,
    exp_power: Rational,
    radicand: Rational,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct LnAffineClass {
    // Constructed only for positive offset + ln(base). This preserves the
    // nonzero/sign invariant shared by all non-Irrational classes.
    offset: Rational,
    base: Rational,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct LnProductClass {
    // Bases are sorted at construction so products compare and combine without
    // considering operand order.
    left: Rational,
    right: Rational,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
enum Class {
    // `Class` is a certificate, not the whole value: `Real.rational` scales the
    // mathematical value represented here. All variants except `Irrational` are
    // exact, nonzero, and positive internally unless a comment says otherwise.
    One,                // Exactly one
    Pi,                 // Exactly pi
    PiPow(u8),          // Exactly pi**n, n >= 2
    PiInv,              // Exactly 1 / pi
    PiExp(Rational),    // Exactly pi * e**Rational
    PiInvExp(Rational), // Exactly e**Rational / pi
    PiSqrt(Rational),   // Exactly pi * sqrt(Rational)
    // Boxed so uncommon combined constants do not bloat every `Real` value.
    // Dense algebra and borrowed-op benches are sensitive to enum size.
    ConstProduct(Box<ConstProductClass>), // Exactly pi**n * e**Rational, signed n
    ConstOffset(Box<ConstOffsetClass>),   // Exactly pi**n * e**Rational + Rational
    ConstProductSqrt(Box<ConstProductSqrtClass>), // Exactly pi**n * e**Rational * sqrt(Rational)
    Sqrt(Rational), // Square root of some positive integer without an integer square root
    Exp(Rational),  // Rational is never zero
    Ln(Rational),   // Rational > 1
    LnAffine(Box<LnAffineClass>), // Exactly Rational + ln(Rational), constructed positive only
    // Boxed for the same reason as `ConstProduct`: keep the common `Real`
    // representation small while still preserving a lightweight symbolic form.
    LnProduct(Box<LnProductClass>), // Product of two logarithms, ordered by base
    Log10(Rational),                // Rational > 1 and never a multiple of ten
    SinPi(Rational),                // 0 < Rational < 1/2 also never 1/6 or 1/4 or 1/3
    TanPi(Rational),                // 0 < Rational < 1/2 also never 1/6 or 1/4 or 1/3
    Irrational,
}

use Class::*;
use serde::Deserialize;
use serde::Serialize;

// We can't tell whether an Irrational value is ever equal to anything
impl PartialEq for Class {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (One, One) => true,
            (Pi, Pi) => true,
            (PiPow(r), PiPow(s)) => r == s,
            (PiInv, PiInv) => true,
            (PiExp(r), PiExp(s)) => r == s,
            (PiInvExp(r), PiInvExp(s)) => r == s,
            (PiSqrt(r), PiSqrt(s)) => r == s,
            (ConstProduct(left), ConstProduct(right)) => {
                left.pi_power == right.pi_power && left.exp_power == right.exp_power
            }
            (ConstOffset(left), ConstOffset(right)) => {
                left.pi_power == right.pi_power
                    && left.exp_power == right.exp_power
                    && left.offset == right.offset
            }
            (ConstProductSqrt(left), ConstProductSqrt(right)) => {
                left.pi_power == right.pi_power
                    && left.exp_power == right.exp_power
                    && left.radicand == right.radicand
            }
            (Sqrt(r), Sqrt(s)) => r == s,
            (Exp(r), Exp(s)) => r == s,
            (Ln(r), Ln(s)) => r == s,
            (LnAffine(left), LnAffine(right)) => {
                left.offset == right.offset && left.base == right.base
            }
            (LnProduct(left), LnProduct(right)) => {
                left.left == right.left && left.right == right.right
            }
            (Log10(r), Log10(s)) => r == s,
            (SinPi(r), SinPi(s)) => r == s,
            (TanPi(r), TanPi(s)) => r == s,
            (_, _) => false,
        }
    }
}

impl Class {
    // Could treat Exp specially for large negative exponents
    fn is_non_zero(&self) -> bool {
        // Every current symbolic class except the rational scale itself is
        // constructed as non-zero. Keeping this invariant lets zero/sign
        // queries short-circuit without touching the computable graph.
        true
    }

    // Any logarithmn can be added
    fn is_ln(&self) -> bool {
        matches!(self, Ln(_))
    }

    fn make_exp(br: Rational) -> (Class, Computable) {
        // `e^0` is the rational one. Normalizing here keeps exp-products from
        // leaving behind a symbolic class that would slow equality and sign checks.
        if br == *rationals::ZERO {
            (One, Computable::one())
        } else {
            (Exp(br.clone()), Computable::exp_rational(br))
        }
    }

    fn pi_power_computable(power: u16) -> Computable {
        // Pi powers are kept as shallow multiply chains only for symbolic
        // constants. Large generic powers still use the normal pow machinery.
        let mut value = Computable::pi();
        for _ in 1..power {
            value = value.multiply(Computable::pi());
        }
        value
    }

    fn make_pi_power(power: u8) -> (Class, Computable) {
        match power {
            0 => (One, Computable::one()),
            1 => (Pi, Computable::pi()),
            _ => (PiPow(power), Self::pi_power_computable(u16::from(power))),
        }
    }

    fn make_pi_exp(br: Rational) -> (Class, Computable) {
        Self::make_const_product(1, br)
    }

    fn make_const_product(pi_power: i16, exp_power: Rational) -> (Class, Computable) {
        // Normalize back to the smaller legacy variants whenever possible.
        // Rarer signed pi powers stay boxed; direct `pi`, `1/pi`, `e^q`,
        // and one-pi-factor products have smaller dedicated variants.
        if pi_power == 0 {
            return Self::make_exp(exp_power);
        }
        if exp_power == *rationals::ZERO {
            if let Ok(power) = u8::try_from(pi_power) {
                return Self::make_pi_power(power);
            }
            if pi_power == -1 {
                return (PiInv, Computable::pi().inverse());
            }
        }
        if pi_power == 1 {
            return (
                PiExp(exp_power.clone()),
                Computable::pi().multiply(Computable::exp_rational(exp_power)),
            );
        }
        if pi_power == -1 {
            return (
                PiInvExp(exp_power.clone()),
                Computable::pi()
                    .inverse()
                    .multiply(Computable::exp_rational(exp_power)),
            );
        }
        let pi = Self::signed_pi_power_computable(pi_power);
        let computable = if exp_power == *rationals::ZERO {
            pi
        } else {
            pi.multiply(Computable::exp_rational(exp_power.clone()))
        };
        (
            ConstProduct(Box::new(ConstProductClass {
                pi_power,
                exp_power: exp_power.clone(),
            })),
            computable,
        )
    }

    fn const_offset_positive_certified(
        pi_power: i16,
        exp_power: &Rational,
        offset: &Rational,
    ) -> bool {
        // `ConstOffset` participates in the same zero/sign fast path as the
        // purely multiplicative classes, so only construct it when the inner
        // value is cheaply certified positive. This deliberately covers the hot
        // `pi - 3` and `e - 2` style reductions and leaves near-cancellation
        // cases on the generic computable path.
        if offset.sign() != Sign::Minus {
            return true;
        }
        let threshold = -offset.clone();
        if exp_power == &*rationals::ZERO && pi_power == 1 {
            threshold <= Rational::new(3)
        } else if exp_power == &*rationals::ONE && pi_power == 0 {
            threshold <= Rational::new(2)
        } else {
            false
        }
    }

    fn make_const_offset(
        pi_power: i16,
        exp_power: Rational,
        offset: Rational,
    ) -> Option<(Class, Computable)> {
        if offset == *rationals::ZERO {
            return Some(Self::make_const_product(pi_power, exp_power));
        }
        if pi_power == 0 && exp_power == *rationals::ZERO {
            return None;
        }
        if !Self::const_offset_positive_certified(pi_power, &exp_power, &offset) {
            return None;
        }
        let (_, constant) = Self::make_const_product(pi_power, exp_power.clone());
        let computable = Computable::add(constant, Computable::rational(offset.clone()));
        Some((
            ConstOffset(Box::new(ConstOffsetClass {
                pi_power,
                exp_power,
                offset,
            })),
            computable,
        ))
    }

    fn signed_pi_power_computable(power: i16) -> Computable {
        // Negative pi powers are represented by inverting the positive cached
        // pi-chain. This is cheaper than creating an opaque reciprocal product
        // and lets later multiplication cancel back to the exact classes.
        if power >= 0 {
            return Self::pi_power_computable(power as u16);
        }
        Self::pi_power_computable(power.unsigned_abs()).inverse()
    }

    fn const_product_parts(&self) -> Option<(i16, Rational)> {
        // A lightweight deconstructor for the pi^n * e^q family. This is the
        // single point that lets newer classes (`1/pi`, `e^q/pi`) participate
        // in old multiplication/division reductions without growing more arms.
        match self {
            One => Some((0, Rational::zero())),
            Pi => Some((1, Rational::zero())),
            PiPow(power) => Some((i16::from(*power), Rational::zero())),
            PiInv => Some((-1, Rational::zero())),
            Exp(exp) => Some((0, exp.clone())),
            PiExp(exp) => Some((1, exp.clone())),
            PiInvExp(exp) => Some((-1, exp.clone())),
            ConstProduct(product) => Some((product.pi_power, product.exp_power.clone())),
            _ => None,
        }
    }

    fn const_offset_parts(&self) -> Option<(i16, Rational, Rational)> {
        // Treat plain constant products as offset-zero values. This lets
        // rational addition/subtraction share one normalization path while the
        // zero-offset case normalizes back to the smaller multiplicative class.
        match self {
            ConstOffset(offset) => Some((
                offset.pi_power,
                offset.exp_power.clone(),
                offset.offset.clone(),
            )),
            _ => {
                let (pi_power, exp_power) = self.const_product_parts()?;
                Some((pi_power, exp_power, Rational::zero()))
            }
        }
    }

    fn can_take_const_offset(&self) -> bool {
        // Gate the offset probe so ordinary sqrt/log/trig additions do not pay
        // for a const-product deconstruction they can never use. This gate was
        // added after borrowed matrix multiplication benchmarks caught the cost.
        matches!(
            self,
            Pi | PiPow(_)
                | PiInv
                | Exp(_)
                | PiExp(_)
                | PiInvExp(_)
                | ConstProduct(_)
                | ConstOffset(_)
        )
    }

    fn divide_const_products(left: &Class, right: &Class) -> Option<(Class, Computable)> {
        // General quotient closure for symbolic constants. The checked integer
        // arithmetic is a guardrail for rare constructed powers; overflow falls
        // back to the generic computable path rather than wrapping a certificate.
        let (left_pi, left_exp) = left.const_product_parts()?;
        let (right_pi, right_exp) = right.const_product_parts()?;
        let pi_power = left_pi.checked_sub(right_pi)?;
        Some(Self::make_const_product(pi_power, left_exp - right_exp))
    }

    fn multiply_const_products(left: &Class, right: &Class) -> Option<(Class, Computable)> {
        // Product closure mirrors division and is kept behind the exact special
        // forms in `Mul`; this retains very small hot arms while still catching
        // less common signed-power products.
        let (left_pi, left_exp) = left.const_product_parts()?;
        let (right_pi, right_exp) = right.const_product_parts()?;
        let pi_power = left_pi.checked_add(right_pi)?;
        Some(Self::make_const_product(pi_power, left_exp + right_exp))
    }

    fn make_pi_sqrt(r: Rational) -> (Class, Computable) {
        // `pi * sqrt(n)` appears in algebra kernels; keeping it symbolic lets
        // repeated products collapse before falling back to generic computables.
        (
            PiSqrt(r.clone()),
            Computable::pi().multiply(Computable::sqrt_rational(r)),
        )
    }

    fn make_const_product_sqrt(
        pi_power: i16,
        exp_power: Rational,
        radicand: Rational,
    ) -> (Class, Computable) {
        // This is a factored positive class: the rational scale carries all sign
        // information, while the inner pi/e/sqrt product stays non-zero. Keeping
        // the common `pi*sqrt(n)` and plain `sqrt(n)` variants avoids regressing
        // the old tight arms.
        if radicand == *rationals::ONE {
            return Self::make_const_product(pi_power, exp_power);
        }
        if pi_power == 0 && exp_power == *rationals::ZERO {
            return (Sqrt(radicand.clone()), Computable::sqrt_rational(radicand));
        }
        if pi_power == 1 && exp_power == *rationals::ZERO {
            return Self::make_pi_sqrt(radicand);
        }
        let (_, constant) = Self::make_const_product(pi_power, exp_power.clone());
        let computable = constant.multiply(Computable::sqrt_rational(radicand.clone()));
        (
            ConstProductSqrt(Box::new(ConstProductSqrtClass {
                pi_power,
                exp_power,
                radicand,
            })),
            computable,
        )
    }

    fn const_product_sqrt_parts(&self) -> Option<(i16, Rational, Rational)> {
        // Deconstructor for classes that contain an explicit sqrt factor. It is
        // intentionally narrower than `const_product_parts` so hot pi/e products
        // do not pay sqrt-specific reductions.
        match self {
            Sqrt(radicand) => Some((0, Rational::zero(), radicand.clone())),
            PiSqrt(radicand) => Some((1, Rational::zero(), radicand.clone())),
            ConstProductSqrt(product) => Some((
                product.pi_power,
                product.exp_power.clone(),
                product.radicand.clone(),
            )),
            _ => None,
        }
    }

    fn has_const_product_sqrt_factor(&self) -> bool {
        // Cheap prefilter before calling the heavier sqrt-product deconstructor
        // in multiply/divide fallbacks.
        matches!(self, Sqrt(_) | PiSqrt(_) | ConstProductSqrt(_))
    }

    fn ln_computable(base: &Rational) -> Computable {
        // Common logarithm constants share Computable caches. Dense symbolic
        // expressions repeatedly build ln(2), ln(3), etc.; routing them here
        // avoids independent approximation caches for the same mathematical value.
        if base == &Rational::new(2) {
            Computable::ln_constant(2).unwrap()
        } else if base == &Rational::new(3) {
            Computable::ln_constant(3).unwrap()
        } else if base == &Rational::new(5) {
            Computable::ln_constant(5).unwrap()
        } else if base == &Rational::new(6) {
            Computable::ln_constant(6).unwrap()
        } else if base == &Rational::new(7) {
            Computable::ln_constant(7).unwrap()
        } else if base == &Rational::new(10) {
            Computable::ln_constant(10).unwrap()
        } else {
            Computable::rational(base.clone()).ln()
        }
    }

    fn make_ln_affine(offset: Rational, base: Rational) -> Option<(Class, Computable)> {
        // Like `ConstOffset`, this additive form is only admitted when the
        // inner value is structurally positive. That keeps sign and zero queries
        // as cheap as other exact classes while avoiding a generic add node for
        // ln(a*e^q) with positive q.
        if base <= *rationals::ONE || offset.sign() == Sign::Minus {
            return None;
        }
        if offset == *rationals::ZERO {
            return Some((Ln(base.clone()), Self::ln_computable(&base)));
        }
        let computable = Computable::add(
            Computable::rational(offset.clone()),
            Self::ln_computable(&base),
        );
        Some((
            LnAffine(Box::new(LnAffineClass { offset, base })),
            computable,
        ))
    }

    fn make_ln_product(left: Rational, right: Rational) -> (Class, Computable) {
        // Sort factors so ln(a)*ln(b) and ln(b)*ln(a) compare equal and share
        // downstream structural facts.
        let (left, right) = if left <= right {
            (left, right)
        } else {
            (right, left)
        };
        (
            LnProduct(Box::new(LnProductClass {
                left: left.clone(),
                right: right.clone(),
            })),
            Self::ln_computable(&left).multiply(Self::ln_computable(&right)),
        )
    }
}

mod rationals {
    use crate::Rational;
    use std::sync::LazyLock;

    pub(super) static HALF: LazyLock<Rational> =
        LazyLock::new(|| Rational::fraction(1, 2).unwrap());
    pub(super) static ONE: LazyLock<Rational> = LazyLock::new(|| Rational::new(1));
    pub(super) static ZERO: LazyLock<Rational> = LazyLock::new(Rational::zero);
    pub(super) static TEN: LazyLock<Rational> = LazyLock::new(|| Rational::new(10));
}

mod constants {
    use crate::real::Class;
    use crate::{Computable, Rational, Real};
    thread_local! {
        // These are the canonical internal constants. Public constructors clone
        // these symbolic/computable forms instead of rebuilding exact classes and
        // caches on every call.
        static PI: Real = Real {
            rational: Rational::one(),
            class: Class::Pi,
            computable: Computable::pi(),
            signal: None,
        };
        static TAU: Real = Real {
            rational: Rational::new(2),
            class: Class::Pi,
            computable: Computable::pi(),
            signal: None,
        };
        static HALF: Real = Real::new(Rational::fraction(1, 2).unwrap());
        static SQRT_TWO_OVER_TWO: Real = Real {
            rational: Rational::fraction(1, 2).unwrap(),
            class: Class::Sqrt(Rational::new(2)),
            computable: Computable::sqrt_constant(2).unwrap(),
            signal: None,
        };
        static SQRT_THREE_OVER_TWO: Real = Real {
            rational: Rational::fraction(1, 2).unwrap(),
            class: Class::Sqrt(Rational::new(3)),
            computable: Computable::sqrt_constant(3).unwrap(),
            signal: None,
        };
        static SQRT_THREE: Real = Real {
            rational: Rational::one(),
            class: Class::Sqrt(Rational::new(3)),
            computable: Computable::sqrt_constant(3).unwrap(),
            signal: None,
        };
        static SQRT_THREE_OVER_THREE: Real = Real {
            rational: Rational::fraction(1, 3).unwrap(),
            class: Class::Sqrt(Rational::new(3)),
            computable: Computable::sqrt_constant(3).unwrap(),
            signal: None,
        };
        static LN2: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(2)),
            computable: Computable::ln_constant(2).unwrap(),
            signal: None,
        };
        static LN3: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(3)),
            computable: Computable::ln_constant(3).unwrap(),
            signal: None,
        };
        static LN5: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(5)),
            computable: Computable::ln_constant(5).unwrap(),
            signal: None,
        };
        static LN6: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(6)),
            computable: Computable::ln_constant(6).unwrap(),
            signal: None,
        };
        static LN7: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(7)),
            computable: Computable::ln_constant(7).unwrap(),
            signal: None,
        };
        static LN10: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(10)),
            computable: Computable::ln_constant(10).unwrap(),
            signal: None,
        };
        static E: Real = Real {
            rational: Rational::one(),
            class: Class::Exp(Rational::one()),
            computable: Computable::e_constant(),
            signal: None,
        };
    }

    pub(super) fn half() -> Real {
        HALF.with(|real| real.clone())
    }

    pub(super) fn pi() -> Real {
        PI.with(|real| real.clone())
    }

    pub(super) fn tau() -> Real {
        TAU.with(|real| real.clone())
    }

    pub(super) fn sqrt_two_over_two() -> Real {
        SQRT_TWO_OVER_TWO.with(|real| real.clone())
    }

    pub(super) fn sqrt_three_over_two() -> Real {
        SQRT_THREE_OVER_TWO.with(|real| real.clone())
    }

    pub(super) fn sqrt_three() -> Real {
        SQRT_THREE.with(|real| real.clone())
    }

    pub(super) fn sqrt_three_over_three() -> Real {
        SQRT_THREE_OVER_THREE.with(|real| real.clone())
    }

    pub(super) fn scaled_ln(base: u32, coefficient: i64) -> Option<Real> {
        // Return clones of canonical cached ln constants with only the rational
        // scale adjusted. This is cheaper than constructing a fresh Ln class and
        // computable cache for every recognized log power.
        let mut value = match base {
            2 => LN2.with(|real| real.clone()),
            3 => LN3.with(|real| real.clone()),
            5 => LN5.with(|real| real.clone()),
            6 => LN6.with(|real| real.clone()),
            7 => LN7.with(|real| real.clone()),
            10 => LN10.with(|real| real.clone()),
            _ => return None,
        };
        value.rational = Rational::new(coefficient);
        Some(value)
    }

    pub(super) fn e() -> Real {
        E.with(|real| real.clone())
    }
}

mod signed {
    use num::{BigInt, bigint::ToBigInt};
    use std::sync::LazyLock;

    pub(super) static ONE: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&1).unwrap());
}

mod unsigned {
    use num::{BigUint, bigint::ToBigUint};
    use std::sync::LazyLock;

    pub(super) static ONE: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&1).unwrap());
    pub(super) static TWO: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&2).unwrap());
    pub(super) static THREE: LazyLock<BigUint> =
        LazyLock::new(|| ToBigUint::to_biguint(&3).unwrap());
    pub(super) static FOUR: LazyLock<BigUint> =
        LazyLock::new(|| ToBigUint::to_biguint(&4).unwrap());
    pub(super) static SIX: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&6).unwrap());
}

use std::sync::Arc;
use std::sync::atomic::AtomicBool;

pub type Signal = Arc<AtomicBool>;

/// (More) Real numbers
///
/// This type is functionally the product of a [`Computable`] number
/// and a [`Rational`].
///
/// Internally the rational scale is kept beside a lightweight symbolic
/// [`Class`] certificate. Many hot operations inspect that certificate before
/// folding into the generic computable graph; do not eagerly combine the fields
/// unless a generic kernel really needs it.
///
/// # Examples
///
/// Even a normal rational can be parsed as a Real
/// ```
/// use hyperreal::{Real, Rational};
/// let half: Real = "0.5".parse().unwrap();
/// assert_eq!(half, Rational::fraction(1, 2).unwrap());
/// ```
///
/// Simple arithmetic
/// ```
/// use hyperreal::Real;
/// let two_pi = Real::pi() + Real::pi();
/// let four: Real = "4".parse().unwrap();
/// let four_pi = four * Real::pi();
/// let answer = (four_pi / two_pi).unwrap();
/// let two = hyperreal::Rational::new(2);
/// assert_eq!(answer, Real::new(two));
/// ```
///
/// Conversion
/// ```
/// use hyperreal::{Real, Rational};
/// let nine: Real = 9.into();
/// let three = Rational::new(3);
/// let answer = nine.sqrt().unwrap();
/// assert_eq!(answer, three);
/// ```
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Real {
    rational: Rational,
    class: Class,
    computable: Computable,
    #[serde(skip)]
    signal: Option<Signal>,
}

impl Real {
    /// Provide an atomic flag to signal early abort of calculations.
    /// The provided flag can be used e.g. from another execution thread.
    /// Aborted calculations may have incorrect results.
    pub fn abort(&mut self, s: Signal) {
        self.signal = Some(s.clone());
        self.computable.abort(s);
    }

    /// Zero, the additive identity.
    pub fn zero() -> Real {
        Self {
            rational: Rational::zero(),
            class: One,
            computable: Computable::one(),
            signal: None,
        }
    }

    /// The specified [`Rational`] as a Real.
    pub fn new(rational: Rational) -> Real {
        Self {
            rational,
            class: One,
            computable: Computable::one(),
            signal: None,
        }
    }

    /// π, the ratio of a circle's circumference to its diameter.
    pub fn pi() -> Real {
        constants::pi()
    }

    /// τ, the ratio of a circle's circumference to its radius.
    pub fn tau() -> Real {
        constants::tau()
    }

    /// e, Euler's number and the base of the natural logarithm function.
    pub fn e() -> Real {
        constants::e()
    }
}

impl AsRef<Real> for Real {
    fn as_ref(&self) -> &Real {
        self
    }
}

// Tan(r) is a repeating shape
// returns whether to negate, and the (if necessary reflected) fraction
// 0 < r < 0.5
// Never actually used for exact zero or half
fn tan_curve(r: Rational) -> (bool, Rational) {
    let mut s = r.fract();
    let mut flip = false;
    if s.sign() == Sign::Minus {
        flip = true;
        s = s.neg();
    }
    if s > *rationals::HALF {
        (!flip, Rational::one() - s)
    } else {
        (flip, s)
    }
}

// Sin(r) is a single curve, then reflected, then both halves negated
// returns whether to negate, and the (if necessary reflected) fraction
// 0 < r < 0.5
// Never actually used for exact zero or half
fn curve(r: Rational) -> (bool, Rational) {
    // Reduce rational multiples of pi to the first half-turn and record the
    // sign separately. SinPi/TanPi classes depend on this canonical argument so
    // equivalent exact trig values compare cheaply.
    if r.sign() == Sign::Minus {
        let (neg, s) = curve(r.neg());
        return (!neg, s);
    }
    let whole = r.shifted_big_integer(0);
    let mut s = r.fract();
    if s > *rationals::HALF {
        s = Rational::one() - s;
    }
    (whole.bit(0), s)
}

fn sin_pi_neg(r: Rational) -> bool {
    // For exact sin(q*pi) table entries, the integer part alone determines the
    // sign after the rational argument has been normalized.
    if r.sign() == Sign::Minus {
        return !sin_pi_neg(r.neg());
    }
    r.shifted_big_integer(0).bit(0)
}

impl Real {
    /// Is this Real exactly zero?
    #[inline]
    pub fn definitely_zero(&self) -> bool {
        self.rational.sign() == Sign::NoSign
    }

    /// Return this value as an owned exact rational when that is structurally known.
    pub fn exact_rational(&self) -> Option<Rational> {
        match self.class {
            One => Some(self.rational.clone()),
            _ => None,
        }
    }

    /// Conservatively inspect public structural facts about this value.
    #[inline]
    pub fn structural_facts(&self) -> RealStructuralFacts {
        if matches!(self.class, One) {
            return facts_from_rational(&self.rational, true);
        }

        let rational_sign = self.rational.sign();
        if rational_sign == Sign::NoSign {
            return facts_from_rational(&self.rational, false);
        }

        let computable = self.computable.structural_facts();
        let sign = match self.class {
            One => Some(real_sign_from_num(rational_sign)),
            Pi | PiPow(_) | PiInv | PiExp(_) | PiInvExp(_) | PiSqrt(_) | ConstProduct(_)
            | ConstOffset(_) | ConstProductSqrt(_) | Sqrt(_) | Exp(_) | Ln(_) | LnAffine(_)
            | LnProduct(_) | Log10(_) | SinPi(_) | TanPi(_) => {
                // Exact symbolic classes are positive by construction, so the
                // outer rational scale alone determines sign. Additive classes
                // such as ConstOffset/LnAffine are admitted only when this
                // invariant is certified.
                Some(real_sign_from_num(rational_sign))
            }
            Irrational => {
                multiply_public_sign(Some(real_sign_from_num(rational_sign)), computable.sign)
            }
        };

        let zero = match sign {
            Some(RealSign::Zero) => ZeroKnowledge::Zero,
            Some(RealSign::Negative | RealSign::Positive) => ZeroKnowledge::NonZero,
            None if matches!(computable.zero, ZeroKnowledge::NonZero) => ZeroKnowledge::NonZero,
            None => ZeroKnowledge::Unknown,
        };

        let magnitude = match (self.rational.msd_exact(), computable.magnitude) {
            (Some(rational_msd), Some(magnitude)) => Some(MagnitudeBits {
                msd: rational_msd + magnitude.msd,
                exact_msd: magnitude.exact_msd,
            }),
            _ => computable.magnitude,
        };

        RealStructuralFacts {
            sign,
            zero,
            exact_rational: false,
            magnitude,
        }
    }

    /// Conservatively report whether structural inspection proves this value is zero.
    #[inline]
    pub fn zero_status(&self) -> ZeroKnowledge {
        match self.rational.sign() {
            Sign::NoSign => ZeroKnowledge::Zero,
            // All named/exact classes are non-zero when their rational scale is
            // non-zero; only opaque computables need refinement. Keep this as a
            // negative test so adding another exact class does not lengthen this
            // predicate-heavy fast path.
            Sign::Minus | Sign::Plus if !matches!(self.class, Irrational) => ZeroKnowledge::NonZero,
            Sign::Minus | Sign::Plus => self.computable.zero_status(),
        }
    }

    /// Try to prove the sign without refining past `min_precision`.
    pub fn refine_sign_until(&self, min_precision: i32) -> Option<RealSign> {
        let facts = self.structural_facts();
        if let Some(sign) = facts.sign {
            return Some(sign);
        }
        if self.rational.sign() == Sign::NoSign {
            return Some(RealSign::Zero);
        }
        let computable_sign = self.computable.sign_until(min_precision)?;
        multiply_public_sign(
            Some(real_sign_from_num(self.rational.sign())),
            Some(computable_sign),
        )
    }

    /// Are two Reals definitely unequal?
    pub fn definitely_not_equal(&self, other: &Self) -> bool {
        if self.rational.sign() == Sign::NoSign {
            return other.class.is_non_zero() && other.rational.sign() != Sign::NoSign;
        }
        if other.rational.sign() == Sign::NoSign {
            return self.class.is_non_zero() && self.rational.sign() != Sign::NoSign;
        }
        false
        /* ... TODO add more cases which definitely aren't equal */
    }

    /// Our best attempt to discern the [`Sign`] of this Real.
    /// This will be accurate for trivial Rationals and many but not all other cases.
    pub fn best_sign(&self) -> Sign {
        if !matches!(self.class, Irrational) {
            self.rational.sign()
        } else {
            match (self.rational.sign(), self.computable.sign()) {
                (Sign::NoSign, _) => Sign::NoSign,
                (_, Sign::NoSign) => Sign::NoSign,
                (Sign::Plus, Sign::Plus) => Sign::Plus,
                (Sign::Plus, Sign::Minus) => Sign::Minus,
                (Sign::Minus, Sign::Plus) => Sign::Minus,
                (Sign::Minus, Sign::Minus) => Sign::Plus,
            }
        }
    }

    // Given a function which makes a [`Computable`] from another
    // Computable this method
    // returns a Real of Irrational class with that value.
    fn make_computable<F>(self, convert: F) -> Self
    where
        F: FnOnce(Computable) -> Computable,
    {
        // This is the boundary where exact/symbolic information is intentionally
        // discarded. Callers should exhaust local exact shortcuts before using it.
        let computable = convert(self.fold());

        Self {
            rational: Rational::one(),
            class: Irrational,
            computable,
            signal: None,
        }
    }

    fn sin_pi_rational(rational: Rational) -> Real {
        if rational.is_integer() {
            return Self::zero();
        }
        let mut exact: Option<Real> = None;
        let denominator = rational.denominator();
        // Small rational multiples of pi have compact exact forms. Keep these symbolic so
        // later algebra and predicate queries avoid generic trig evaluation.
        if denominator == unsigned::TWO.deref() {
            exact = Some(Self::new(Rational::one()));
        }
        if denominator == unsigned::THREE.deref() {
            exact = Some(constants::sqrt_three_over_two());
        }
        if denominator == unsigned::FOUR.deref() {
            exact = Some(constants::sqrt_two_over_two());
        }
        if denominator == unsigned::SIX.deref() {
            exact = Some(constants::half());
        }
        if let Some(real) = exact {
            return if sin_pi_neg(rational) {
                real.neg()
            } else {
                real
            };
        }

        let (negate, reduced) = curve(rational);
        // For non-tabulated rational multiples, reduce to the principal curve and store a
        // SinPi certificate rather than collapsing to an opaque computable.
        let argument =
            Computable::multiply(Computable::pi(), Computable::rational(reduced.clone()));
        let computable = Computable::prescaled_sin(argument);
        if negate {
            Self {
                rational: Rational::new(-1),
                class: SinPi(reduced),
                computable,
                signal: None,
            }
        } else {
            Self {
                rational: Rational::one(),
                class: SinPi(reduced),
                computable,
                signal: None,
            }
        }
    }

    /// The inverse of this Real, or a [`Problem`] if that's impossible,
    /// in particular Problem::DivideByZero if this real is zero.
    ///
    /// Example
    /// ```
    /// use hyperreal::{Rational,Real};
    /// let five = Real::new(Rational::new(5));
    /// let a_fifth = Real::new(Rational::fraction(1, 5).unwrap());
    /// assert_eq!(five.inverse(), Ok(a_fifth));
    /// ```
    pub fn inverse(self) -> Result<Self, Problem> {
        if self.definitely_zero() {
            return Err(Problem::DivideByZero);
        }
        match &self.class {
            One => {
                // Rational reciprocals remain exact.
                return Ok(Self {
                    rational: self.rational.inverse()?,
                    class: One,
                    computable: Computable::one(),
                    signal: None,
                });
            }
            Sqrt(sqrt) => {
                if let Some(sqrt) = sqrt.to_big_integer() {
                    // Rationalize 1/(a*sqrt(n)) when n is integral, keeping a sqrt form
                    // instead of an opaque inverse node.
                    let rational = (self.rational * Rational::from_bigint(sqrt)).inverse()?;
                    return Ok(Self {
                        rational,
                        class: self.class,
                        computable: self.computable,
                        signal: None,
                    });
                }
            }
            Pi => {
                // Consume the existing pi computable and only swap the lightweight class.
                // Rebuilding through `make_const_product` is measurably slower for `1/pi`.
                return Ok(Self {
                    rational: self.rational.inverse()?,
                    class: PiInv,
                    computable: self.computable.inverse(),
                    signal: None,
                });
            }
            PiInv => {
                // Reciprocal-pi is its own class; inverting it restores the
                // canonical cached pi class without generic const-product setup.
                return Ok(Self {
                    rational: self.rational.inverse()?,
                    class: Pi,
                    computable: self.computable.inverse(),
                    signal: None,
                });
            }
            Exp(exp) => {
                // e^x inverts to e^-x symbolically.
                let exp = Neg::neg(exp.clone());
                return Ok(Self {
                    rational: self.rational.inverse()?,
                    class: Exp(exp.clone()),
                    computable: Computable::exp_rational(exp),
                    signal: None,
                });
            }
            PiExp(exp) => {
                // pi*e^x inverts to e^-x/pi, preserving the one-pi-factor class
                // used by division/multiplication fast arms.
                return Ok(Self {
                    rational: self.rational.inverse()?,
                    class: PiInvExp(exp.clone().neg()),
                    computable: self.computable.inverse(),
                    signal: None,
                });
            }
            PiInvExp(exp) => {
                // The reciprocal of e^x/pi is pi*e^-x.
                return Ok(Self {
                    rational: self.rational.inverse()?,
                    class: PiExp(exp.clone().neg()),
                    computable: self.computable.inverse(),
                    signal: None,
                });
            }
            _ => (),
        }
        if let Some((pi_power, exp_power, radicand)) = self.class.const_product_sqrt_parts() {
            // Rationalize factored sqrt products as
            // 1 / (a*pi^n*e^q*sqrt(r)) = pi^-n*e^-q*sqrt(r) / (a*r).
            // Keeping the sqrt attached to the constant product lets later
            // multiplication cancel it without creating an opaque inverse node.
            let rational = (self.rational * radicand.clone()).inverse()?;
            let (class, computable) =
                Class::make_const_product_sqrt(-pi_power, exp_power.neg(), radicand);
            return Ok(Self {
                rational,
                class,
                computable,
                signal: None,
            });
        }
        if let Some((pi_power, exp_power)) = self.class.const_product_parts() {
            // Keep reciprocal constant products symbolic as pi^-n * e^-q. This matters
            // for scalar and matrix division by pi-heavy constants because the product
            // can later collapse back to `One`, `Exp`, `Pi`, or `PiExp`.
            let (class, computable) = Class::make_const_product(-pi_power, exp_power.neg());
            return Ok(Self {
                rational: self.rational.inverse()?,
                class,
                computable,
                signal: None,
            });
        }
        Ok(Self {
            rational: self.rational.inverse()?,
            class: Irrational,
            computable: Computable::inverse(self.computable),
            signal: None,
        })
    }

    /// The multiplicative inverse of this Real without consuming it.
    pub fn inverse_ref(&self) -> Result<Self, Problem> {
        if self.definitely_zero() {
            return Err(Problem::DivideByZero);
        }
        match &self.class {
            One => Ok(Self::new(self.rational.clone().inverse()?)),
            Sqrt(sqrt) => {
                if let Some(sqrt) = sqrt.to_big_integer() {
                    // Same rationalization as the owned path, but clone only the
                    // rational/computable pieces needed to leave `self` intact.
                    let rational = (&self.rational * Rational::from_bigint(sqrt)).inverse()?;
                    return Ok(Self {
                        rational,
                        class: self.class.clone(),
                        computable: self.computable.clone(),
                        signal: None,
                    });
                }
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: Irrational,
                    computable: Computable::inverse(self.computable.clone()),
                    signal: None,
                })
            }
            Pi => Ok(Self {
                // Preserve the dedicated reciprocal-pi class for borrowed scalar
                // division; rebuilding through the generic constant product costs more.
                rational: self.rational.clone().inverse()?,
                class: PiInv,
                computable: self.computable.clone().inverse(),
                signal: None,
            }),
            PiInv => Ok(Self {
                rational: self.rational.clone().inverse()?,
                class: Pi,
                computable: self.computable.clone().inverse(),
                signal: None,
            }),
            Exp(exp) => {
                // Borrowed inverse keeps e^x symbolic as e^-x, avoiding a generic
                // reciprocal node in matrix/vector scalar division.
                let exp = exp.clone().neg();
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: Exp(exp.clone()),
                    computable: Computable::exp_rational(exp),
                    signal: None,
                })
            }
            PiExp(exp) => Ok(Self {
                rational: self.rational.clone().inverse()?,
                class: PiInvExp(exp.clone().neg()),
                computable: self.computable.clone().inverse(),
                signal: None,
            }),
            PiInvExp(exp) => Ok(Self {
                rational: self.rational.clone().inverse()?,
                class: PiExp(exp.clone().neg()),
                computable: self.computable.clone().inverse(),
                signal: None,
            }),
            _ => {
                if let Some((pi_power, exp_power, radicand)) = self.class.const_product_sqrt_parts()
                {
                    // Borrowed path mirrors owned rationalization while cloning
                    // only the reduced rational radicand and symbolic powers.
                    let rational = (&self.rational * radicand.clone()).inverse()?;
                    let (class, computable) =
                        Class::make_const_product_sqrt(-pi_power, exp_power.neg(), radicand);
                    return Ok(Self {
                        rational,
                        class,
                        computable,
                        signal: None,
                    });
                }
                if let Some((pi_power, exp_power)) = self.class.const_product_parts() {
                    // Rare constant products still stay symbolic in the borrowed
                    // path so `a / (pi^n e^q)` can cancel in the following multiply.
                    let (class, computable) = Class::make_const_product(-pi_power, exp_power.neg());
                    return Ok(Self {
                        rational: self.rational.clone().inverse()?,
                        class,
                        computable,
                        signal: None,
                    });
                }
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: Irrational,
                    computable: Computable::inverse(self.computable.clone()),
                    signal: None,
                })
            }
        }
    }

    /// The square root of this Real, or a [`Problem`] if that's impossible,
    /// in particular Problem::SqrtNegative if this Real is negative.
    pub fn sqrt(self) -> Result<Real, Problem> {
        if self.best_sign() == Sign::Minus {
            return Err(Problem::SqrtNegative);
        }
        if self.definitely_zero() {
            return Ok(Self::zero());
        }
        match &self.class {
            One if self.rational.extract_square_will_succeed() => {
                // Extract rational square factors before creating sqrt nodes.
                let (square, rest) = self.rational.extract_square_reduced();
                if rest == *rationals::ONE {
                    return Ok(Self {
                        rational: square,
                        class: One,
                        computable: Computable::one(),
                        signal: None,
                    });
                } else {
                    return Ok(Self {
                        rational: square,
                        class: Sqrt(rest.clone()),
                        computable: Computable::sqrt_rational(rest),
                        signal: None,
                    });
                }
            }
            Pi if self.rational.extract_square_will_succeed() => {
                // If only the rational scale is a square, keep sqrt(pi) as a
                // computable sqrt rather than inventing a symbolic sqrt-pi class
                // that has not shown benchmark wins.
                let (square, rest) = self.rational.clone().extract_square_reduced();
                if rest == *rationals::ONE {
                    return Ok(Self {
                        rational: square,
                        class: Irrational,
                        computable: Computable::sqrt(self.computable),
                        signal: None,
                    });
                }
            }
            Exp(exp) if self.rational.extract_square_will_succeed() => {
                // sqrt(e^x) = e^(x/2) when the rational scale is also a square.
                let (square, rest) = self.rational.clone().extract_square_reduced();
                if rest == *rationals::ONE {
                    let exp = exp.clone() / Rational::new(2);
                    return Ok(Self {
                        rational: square,
                        class: Exp(exp.clone()),
                        computable: Computable::exp_rational(exp),
                        signal: None,
                    });
                }
            }
            _ => (),
        }

        Ok(self.make_computable(Computable::sqrt))
    }

    /// Apply the exponential function to this Real parameter.
    pub fn exp(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            return Ok(Self::new(Rational::one()));
        }
        match &self.class {
            One => {
                // exp(rational) is a first-class symbolic form used heavily by exact
                // constant products.
                return Ok(Self {
                    rational: Rational::one(),
                    class: Exp(self.rational.clone()),
                    computable: Computable::exp_rational(self.rational),
                    signal: None,
                });
            }
            Ln(ln) => {
                if let Some(int) = self.rational.to_big_integer() {
                    // exp(k ln n) folds to n^k when k is integral.
                    return Ok(Self {
                        rational: ln.clone().powi(int)?,
                        class: One,
                        computable: Computable::one(),
                        signal: None,
                    });
                }
            }
            _ => (),
        }

        Ok(self.make_computable(Computable::exp))
    }

    /// The base 10 logarithm of this Real or Problem::NotANumber if this Real is negative.
    pub fn log10(self) -> Result<Real, Problem> {
        // Use the cached ln(10) symbolic constant. Division recognizes ln/ln10
        // and can return a lightweight Log10 class for exact log inputs.
        self.ln()? / constants::scaled_ln(10, 1).unwrap()
    }

    // Find Some(m) integral log with respect to this base or else None
    // n should be positive (not zero) and base should be >= 2
    fn integer_log(n: &BigUint, base: u32) -> Option<u64> {
        use num::Integer;
        use num::bigint::ToBigUint;
        // TODO weed out some large failure cases early and return None

        // Build powers by repeated squaring, divide by the largest usable power,
        // then walk back down. This recognizes n = base^k without trial-dividing
        // by base k times.
        // Calculate base^2 base^4 base^8 base^16 and so on until it is bigger than next
        let mut result: Option<u64> = None;
        let mut powers: Vec<BigUint> = Vec::new();
        let mut next = ToBigUint::to_biguint(&base).unwrap();
        powers.push(next.clone());

        let mut reduced = n.clone();
        let mut i = 1;
        loop {
            // TODO Looping, may need to handle cancellation
            next = next.pow(2);
            if next.bits() > reduced.bits() {
                break;
            }

            let (div, rem) = reduced.div_rem(&next);
            if rem != BigUint::ZERO {
                return None;
            }
            powers.push(next.clone());
            result = Some(result.unwrap_or(0) + (1 << i));
            reduced = div;
            i += 1;
        }

        while let Some(power) = powers.pop() {
            if reduced == *unsigned::ONE {
                break;
            }
            i -= 1;
            if power.bits() > reduced.bits() {
                continue;
            }
            let (div, rem) = reduced.div_rem(&power);
            if rem != BigUint::ZERO {
                return None;
            }
            result = Some(result.unwrap_or(0) + (1 << i));
            reduced = div;
        }

        if reduced == *unsigned::ONE {
            result
        } else {
            None
        }
    }

    // For input y = ln(r) with r positive gives
    // Some(k ln(s)) where there is a small integer m such that r = s^k.
    // or None
    fn ln_small(r: &Rational) -> Option<Real> {
        let n = r.to_big_integer()?;
        let n = n.magnitude();

        // Recognize common integer powers so logs share cached scaled-ln constants
        // instead of creating many unrelated Ln nodes.
        for base in [2, 3, 5, 6, 7, 10] {
            if let Some(n) = Self::integer_log(n, base) {
                return constants::scaled_ln(base, n as i64);
            }
        }

        None
    }

    // Ensure the resulting Real uses r > 1 for Ln(r)
    // this is convenient elsewhere and makes commonality more frequent
    // e.g. use Ln(2) rather than Ln(0.5)
    // Must be called with r > 0
    fn ln_rational(r: Rational) -> Result<Real, Problem> {
        use std::cmp::Ordering::*;

        match r.partial_cmp(rationals::ONE.deref()) {
            Some(Less) => {
                let inv = r.inverse()?;
                if let Some(answer) = Self::ln_small(&inv) {
                    return Ok(-answer);
                }
                // Normalize ln(r<1) as -ln(1/r) to improve symbolic sharing.
                let new = Computable::rational(inv.clone());
                Ok(Self {
                    rational: Rational::new(-1),
                    class: Ln(inv),
                    computable: Computable::ln(new),
                    signal: None,
                })
            }
            Some(Equal) => Ok(Self::zero()),
            Some(Greater) => {
                if let Some(answer) = Self::ln_small(&r) {
                    return Ok(answer);
                }
                // Positive rationals above one get a lightweight Ln certificate.
                let new = Computable::rational(r.clone());
                Ok(Self {
                    rational: Rational::one(),
                    class: Ln(r),
                    computable: Computable::ln(new),
                    signal: None,
                })
            }
            _ => unreachable!(),
        }
    }

    fn try_add_rational_to_ln_term(term: &Real, offset: Rational) -> Option<Real> {
        // Normalize q + a*ln(b) as a * (q/a + ln(b)) when the inner affine log
        // is positive. If the sign certificate is not cheap, return None and let
        // ordinary addition build a generic computable sum.
        if offset == *rationals::ZERO {
            return Some(term.clone());
        }
        if term.class == One {
            return Some(Real::new(&term.rational + offset));
        }
        let Ln(base) = &term.class else {
            return None;
        };
        if term.rational.sign() == Sign::NoSign {
            return Some(Real::new(offset));
        }
        let class_offset = offset / &term.rational;
        let (class, computable) = Class::make_ln_affine(class_offset, base.clone())?;
        Some(Real {
            rational: term.rational.clone(),
            class,
            computable,
            signal: None,
        })
    }

    /// The natural logarithm of this Real or Problem::NotANumber if this Real is negative.
    pub fn ln(self) -> Result<Real, Problem> {
        if self.best_sign() != Sign::Plus {
            return Err(Problem::NotANumber);
        }
        match &self.class {
            One => return Self::ln_rational(self.rational),
            Exp(exp) => {
                if self.rational == *rationals::ONE {
                    // ln(e^x) collapses exactly for the pure exponential class.
                    return Ok(Self {
                        rational: exp.clone(),
                        class: One,
                        computable: Computable::one(),
                        signal: None,
                    });
                }
                // ln(a * e^x) = ln(a) + x for positive rational scale `a`.
                // The positive-offset case is stored as one factored `ln` class
                // so repeated predicates do not traverse a generic add graph.
                let log_scale = Self::ln_rational(self.rational)?;
                if let Some(answer) = Self::try_add_rational_to_ln_term(&log_scale, exp.clone()) {
                    return Ok(answer);
                }
                return Ok(log_scale + Self::new(exp.clone()));
            }
            _ => (),
        }

        Ok(self.make_computable(Computable::ln))
    }

    /// The sine of this Real.
    pub fn sin(self) -> Real {
        if self.definitely_zero() {
            return Self::zero();
        }
        match &self.class {
            One => {
                // Plain rational trig still uses Computable, not SinPi/TanPi:
                // those exact certificates are reserved for rational multiples
                // of pi where algebra can later invert them.
                let new = Computable::rational(self.rational.clone());
                return Self {
                    rational: Rational::one(),
                    class: Irrational,
                    computable: Computable::sin(new),
                    signal: None,
                };
            }
            Pi => {
                // sin(q*pi) has exact small-denominator and reusable SinPi handling.
                return Self::sin_pi_rational(self.rational);
            }
            _ => (),
        }

        self.make_computable(Computable::sin)
    }

    /// The cosine of this Real.
    pub fn cos(self) -> Real {
        if self.definitely_zero() {
            return Self::new(Rational::one());
        }
        match &self.class {
            One => {
                // Same policy as sine: generic rational cosine enters the
                // computable trig reducer, while pi-multiple exactness is below.
                let new = Computable::rational(self.rational.clone());
                return Self {
                    rational: Rational::one(),
                    class: Irrational,
                    computable: Computable::cos(new),
                    signal: None,
                };
            }
            Pi => {
                // cos(q*pi) is represented through the same SinPi machinery with a
                // half-turn shift, keeping exact identities in one place.
                return Self::sin_pi_rational(self.rational + Rational::fraction(1, 2).unwrap());
            }
            _ => (),
        }

        self.make_computable(Computable::cos)
    }

    /// The tangent of this Real.
    pub fn tan(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            return Ok(Self::zero());
        }

        match &self.class {
            One => {
                // For non-pi rational arguments there are no exact tangent
                // certificates, but Computable::tan still applies small/medium
                // argument reductions.
                let new = Computable::rational(self.rational.clone());
                return Ok(Self {
                    rational: Rational::one(),
                    class: Irrational,
                    computable: Computable::tan(new),
                    signal: None,
                });
            }
            Pi => {
                if self.rational.is_integer() {
                    return Ok(Self::zero());
                }
                // Rational multiples of pi get exact tangent values for the usual small
                // denominators, otherwise a compact TanPi certificate.
                let (neg, n) = tan_curve(self.rational);
                let mut r: Option<Real> = None;
                let d = n.denominator();
                if d == unsigned::TWO.deref() {
                    return Err(Problem::NotANumber);
                }
                if d == unsigned::THREE.deref() {
                    r = Some(constants::sqrt_three());
                }
                if d == unsigned::FOUR.deref() {
                    r = Some(Self::new(Rational::one()));
                }
                if d == unsigned::SIX.deref() {
                    r = Some(constants::sqrt_three_over_three());
                }
                if let Some(real) = r {
                    if neg {
                        return Ok(real.neg());
                    } else {
                        return Ok(real);
                    }
                } else {
                    let new =
                        Computable::multiply(Computable::pi(), Computable::rational(n.clone()));
                    let computable = Computable::prescaled_tan(new);
                    if neg {
                        return Ok(Self {
                            rational: Rational::new(-1),
                            class: TanPi(n),
                            computable,
                            signal: None,
                        });
                    } else {
                        return Ok(Self {
                            rational: Rational::one(),
                            class: TanPi(n),
                            computable,
                            signal: None,
                        });
                    }
                }
            }
            _ => (),
        }

        Ok(self.make_computable(Computable::tan))
    }

    fn pi_fraction(n: i64, d: u64) -> Real {
        Self::new(Rational::fraction(n, d).unwrap()) * Self::pi()
    }

    fn asin_exact(&self) -> Option<Real> {
        if self.definitely_zero() {
            return Some(Self::zero());
        }

        match &self.class {
            One => {
                // Exact inverse-trig table for rational endpoints and half-angle values.
                if self.rational == *rationals::ONE {
                    Some(Self::pi_fraction(1, 2))
                } else if self.rational == Rational::new(-1) {
                    Some(Self::pi_fraction(-1, 2))
                } else if self.rational == *rationals::HALF {
                    Some(Self::pi_fraction(1, 6))
                } else if self.rational == -rationals::HALF.clone() {
                    Some(Self::pi_fraction(-1, 6))
                } else {
                    None
                }
            }
            Sqrt(r) => {
                // Recognize sqrt(2)/2 and sqrt(3)/2 forms produced by exact trig.
                let sign = self.rational.sign();
                let magnitude = if sign == Sign::Minus {
                    self.rational.clone().neg()
                } else {
                    self.rational.clone()
                };
                let angle = if *r == Rational::new(2) && magnitude == *rationals::HALF {
                    Some(Rational::fraction(1, 4).unwrap())
                } else if *r == Rational::new(3) && magnitude == *rationals::HALF {
                    Some(Rational::fraction(1, 3).unwrap())
                } else {
                    None
                }?;

                let angle = if sign == Sign::Minus {
                    angle.neg()
                } else {
                    angle
                };
                Some(Self::new(angle) * Self::pi())
            }
            SinPi(r) => {
                // asin(sin(q*pi)) can reuse the stored angle when it is already in the
                // principal branch represented by SinPi.
                if self.rational == *rationals::ONE {
                    Some(Self::new(r.clone()) * Self::pi())
                } else if self.rational == Rational::new(-1) {
                    Some(Self::new(r.clone().neg()) * Self::pi())
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    fn atan_exact(&self) -> Option<Real> {
        if self.definitely_zero() {
            return Some(Self::zero());
        }

        match &self.class {
            One => {
                // atan(+/-1) is one of the few rational inputs with an exact
                // pi-fraction result; catching it avoids constructing an atan node.
                if self.rational == *rationals::ONE {
                    Some(Self::pi_fraction(1, 4))
                } else if self.rational == Rational::new(-1) {
                    Some(Self::pi_fraction(-1, 4))
                } else {
                    None
                }
            }
            Sqrt(r) => {
                if *r != Rational::new(3) {
                    return None;
                }
                // atan(sqrt(3)) and atan(sqrt(3)/3) have exact pi-fraction answers.
                let sign = self.rational.sign();
                let magnitude = if sign == Sign::Minus {
                    self.rational.clone().neg()
                } else {
                    self.rational.clone()
                };
                let angle = if magnitude == *rationals::ONE {
                    Some(Rational::fraction(1, 3).unwrap())
                } else if magnitude == Rational::fraction(1, 3).unwrap() {
                    Some(Rational::fraction(1, 6).unwrap())
                } else {
                    None
                }?;

                let angle = if sign == Sign::Minus {
                    angle.neg()
                } else {
                    angle
                };
                Some(Self::new(angle) * Self::pi())
            }
            TanPi(r) => {
                // Preserve exact inverse for TanPi certificates instead of going through
                // the generic atan kernel.
                if self.rational == *rationals::ONE {
                    Some(Self::new(r.clone()) * Self::pi())
                } else if self.rational == Rational::new(-1) {
                    Some(Self::new(r.clone().neg()) * Self::pi())
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    /// The inverse sine of this Real, or [`Problem::NotANumber`] outside [-1, 1].
    pub fn asin(self) -> Result<Real, Problem> {
        if let Some(exact) = self.asin_exact() {
            return Ok(exact);
        }
        if self.class == One {
            // Plain rationals use the computable asin kernel after cheap domain checks; it
            // has tiny/endpoint specializations that would be obscured by the atan formula.
            let magnitude = if self.rational.sign() == Sign::Minus {
                self.rational.clone().neg()
            } else {
                self.rational.clone()
            };
            if magnitude > *rationals::ONE {
                return Err(Problem::NotANumber);
            }

            return Ok(self.make_computable(|value| value.asin()));
        }
        if let Sqrt(r) = &self.class
            && self.rational.clone() * self.rational.clone() * r.clone() > *rationals::ONE
        {
            return Err(Problem::NotANumber);
        }
        if matches!(&self.class, Sqrt(_)) {
            // Sqrt inputs commonly arise from exact trig; keep them on the computable asin
            // path so recognizable forms survive longer.
            return Ok(self.make_computable(|value| value.asin()));
        }

        // Generic identity asin(x) = atan(x / sqrt(1-x^2)).
        let one = Self::new(Rational::one());
        let radicand = one.clone() - self.clone().powi(BigInt::from(2_u8))?;
        let denominator = radicand.sqrt().map_err(|problem| match problem {
            Problem::SqrtNegative => Problem::NotANumber,
            other => other,
        })?;
        (self / denominator)?.atan()
    }

    /// The inverse cosine of this Real, or [`Problem::NotANumber`] outside [-1, 1].
    pub fn acos(self) -> Result<Real, Problem> {
        if self.class == One {
            if self.rational == *rationals::ONE {
                // acos(1) is exactly zero and must not enter the generic kernel.
                return Ok(Self::zero());
            }
            if self.rational == Rational::new(-1) {
                // acos(-1) is exactly pi, using the cached internal constant.
                return Ok(Self::pi());
            }
            let magnitude = if self.rational.sign() == Sign::Minus {
                self.rational.clone().neg()
            } else {
                self.rational.clone()
            };
            if magnitude > *rationals::ONE {
                // Exact rational domain failures are rejected before any
                // approximation machinery is constructed.
                return Err(Problem::NotANumber);
            }
        }
        if let Some(asin) = self.asin_exact() {
            // acos(x) shares the exact asin table through pi/2 - asin(x).
            return Ok(Self::pi_fraction(1, 2) - asin);
        }
        if let Sqrt(r) = &self.class
            && self.rational.clone() * self.rational.clone() * r.clone() > *rationals::ONE
        {
            return Err(Problem::NotANumber);
        }

        Ok(self.make_computable(|value| value.acos()))
    }

    /// The inverse tangent of this Real.
    pub fn atan(self) -> Result<Real, Problem> {
        if let Some(exact) = self.atan_exact() {
            return Ok(exact);
        }

        Ok(self.make_computable(Computable::atan))
    }

    /// The inverse hyperbolic sine of this Real.
    pub fn asinh(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            return Ok(Self::zero());
        }
        if self.best_sign() == Sign::Minus {
            return Ok(self.neg().asinh()?.neg());
        }
        if self.fold_ref().approx(-4) <= BigInt::from(64_u8) {
            // Near zero, asinh(x) is evaluated with a log1p-style transform to avoid
            // cancellation in ln(x + sqrt(1+x^2)).
            return Ok(self.make_computable(|value| {
                let square = value.clone().square();
                let denominator = square
                    .clone()
                    .add(Computable::one())
                    .sqrt()
                    .add(Computable::one());
                value.add(square.multiply(denominator.inverse())).ln_1p()
            }));
        }
        Ok(self.make_computable(Computable::asinh))
    }

    /// The inverse hyperbolic cosine of this Real, or [`Problem::NotANumber`] for values < 1.
    pub fn acosh(self) -> Result<Real, Problem> {
        if self.class == One {
            if self.rational == *rationals::ONE {
                return Ok(Self::zero());
            }
            if self.rational < *rationals::ONE {
                return Err(Problem::NotANumber);
            }
        } else if let Sqrt(r) = &self.class {
            // Domain-check factored sqrt values exactly: (a*sqrt(r))^2 = a^2*r.
            if self.rational.sign() == Sign::Minus
                || self.rational.clone() * self.rational.clone() * r.clone() < *rationals::ONE
            {
                return Err(Problem::NotANumber);
            }
        } else {
            let one = Self::new(Rational::one());
            if (self.clone() - one).best_sign() == Sign::Minus {
                return Err(Problem::NotANumber);
            }
        }
        if self.fold_ref().approx(-4) <= BigInt::from(64_u8) {
            // Near one, acosh(x) uses ln1p on (x-1)+sqrt(x^2-1) for a smaller log input.
            return Ok(self.make_computable(|value| {
                let one = Computable::one();
                let shifted = value.clone().add(one.clone().negate());
                let radicand = value.square().add(one.negate());
                shifted.add(radicand.sqrt()).ln_1p()
            }));
        }
        Ok(self.make_computable(Computable::acosh))
    }

    /// The inverse hyperbolic tangent of this Real.
    ///
    /// Returns [`Problem::Infinity`] at the endpoints `-1` and `1`, or
    /// [`Problem::NotANumber`] outside `(-1, 1)`.
    pub fn atanh(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            return Ok(Self::zero());
        }
        let one_real = Self::new(Rational::one());
        if self == one_real || self == -one_real.clone() {
            return Err(Problem::Infinity);
        }
        if self.class == One {
            let magnitude = if self.rational.sign() == Sign::Minus {
                self.rational.clone().neg()
            } else {
                self.rational.clone()
            };
            if magnitude > *rationals::ONE {
                return Err(Problem::NotANumber);
            }
            if magnitude.msd_exact().is_some_and(|msd| msd <= -4) {
                // Tiny rational atanh is faster in the dedicated computable kernel than
                // building ln((1+x)/(1-x))/2.
                return Ok(self.make_computable(Computable::atanh));
            }

            let one = Rational::one();
            let ratio = (one.clone() + self.rational.clone()) / (one - self.rational);
            // Non-tiny rationals can remain an exact logarithm ratio.
            return Ok(Self::ln_rational(ratio)? * Self::new(Rational::fraction(1, 2).unwrap()));
        }
        if let Sqrt(r) = &self.class
            && self.rational.clone() * self.rational.clone() * r.clone() == *rationals::ONE
        {
            // Exact sqrt endpoint, e.g. sqrt(2)/2 scaled to magnitude one.
            return Err(Problem::Infinity);
        }
        if let Sqrt(r) = &self.class
            && self.rational.clone() * self.rational.clone() * r.clone() > *rationals::ONE
        {
            // Exact sqrt domain failure avoids an approximation sign query.
            return Err(Problem::NotANumber);
        }
        if matches!(&self.class, Sqrt(_)) {
            // In-domain sqrt inputs stay on the computable atanh path so the
            // factored radical can still be recognized by lower constructors.
            return Ok(self.make_computable(Computable::atanh));
        }
        let one = Self::new(Rational::one());
        let numerator = one.clone() + self.clone();
        let denominator = one - self;
        Ok((numerator / denominator)?.ln()? * Self::new(Rational::fraction(1, 2).unwrap()))
    }

    fn recursive_powi(base: &Real, exp: &BigUint) -> Self {
        // Fallback for sign-unknown integer powers: repeated squaring is cheaper and more
        // exact than forcing ln/exp through a value whose sign cannot be certified.
        let mut result = Self::new(Rational::one());
        let mut factor = base.clone();
        let bits = exp.bits();
        for b in 0..bits {
            if exp.bit(b) {
                result = result * factor.clone();
            }
            if b + 1 < bits {
                factor = factor.clone() * factor;
            }
        }
        result
    }

    fn compute_exp_ln_powi(value: Computable, exp: BigInt) -> Option<Computable> {
        match value.sign() {
            Sign::NoSign => None,
            Sign::Plus => Some(value.ln().multiply(Computable::integer(exp)).exp()),
            Sign::Minus => {
                // Take the power of the positive version and negate it afterwards.
                let value = value.negate();
                let odd = exp.bit(0);
                let exp = Computable::integer(exp);
                if odd {
                    Some(value.ln().multiply(exp).exp().negate())
                } else {
                    Some(value.ln().multiply(exp).exp())
                }
            }
        }
    }

    fn exp_ln_powi(self, exp: BigInt) -> Result<Self, Problem> {
        match self.best_sign() {
            Sign::NoSign => {
                // Unknown sign cannot safely use ln(base)*exp, so keep the exact
                // repeated-squaring fallback even though it may allocate more nodes.
                if exp.sign() == Sign::Minus {
                    Ok(Self::recursive_powi(&self, exp.magnitude()).neg())
                } else {
                    Ok(Self::recursive_powi(&self, exp.magnitude()))
                }
            }
            Sign::Plus => {
                // Known-positive generic powers use exp(exp*ln(base)) to avoid a long
                // multiplication chain for large exponents.
                let value = self.fold();
                let exp = Computable::integer(exp);

                Ok(Self {
                    rational: Rational::one(),
                    class: Irrational,
                    computable: value.ln().multiply(exp).exp(),
                    signal: None,
                })
            }
            Sign::Minus => {
                let odd = exp.bit(0);
                let value = self.fold();
                let exp = Computable::integer(exp);
                if odd {
                    Ok(Self {
                        rational: Rational::one(),
                        class: Irrational,
                        computable: value.ln().multiply(exp).exp().negate(),
                        signal: None,
                    })
                } else {
                    Ok(Self {
                        rational: Rational::one(),
                        class: Irrational,
                        computable: value.ln().multiply(exp).exp(),
                        signal: None,
                    })
                }
            }
        }
    }

    /// Raise this Real to some integer exponent.
    pub fn powi(self, exp: BigInt) -> Result<Self, Problem> {
        if exp == *signed::ONE {
            return Ok(self);
        }
        if exp.sign() == Sign::NoSign {
            if self.definitely_zero() {
                return Err(Problem::NotANumber);
            } else {
                return Ok(Self::new(Rational::one()));
            }
        }
        if exp.sign() == Sign::Minus && self.definitely_zero() {
            return Err(Problem::NotANumber);
        }
        if let Ok(rational) = self.rational.clone().powi(exp.clone()) {
            match &self.class {
                One => {
                    // Pure rationals stay exact under integer powers.
                    return Ok(Self {
                        rational,
                        class: One,
                        computable: self.computable,
                        signal: None,
                    });
                }
                Sqrt(sqrt) => 'quick: {
                    // (a*sqrt(n))^k can peel off n^(k/2); this preserves exact sqrt
                    // structure for odd powers and collapses even powers to rationals.
                    let odd = exp.bit(0);
                    let Ok(rf2) = sqrt.clone().powi(exp.clone() >> 1) else {
                        break 'quick;
                    };
                    let product = rational * rf2;
                    if odd {
                        let n = Self {
                            rational: product,
                            class: Sqrt(sqrt.clone()),
                            computable: self.computable,
                            signal: None,
                        };
                        return Ok(n);
                    } else {
                        return Ok(Self::new(product));
                    }
                }
                _ => {
                    if let Some(computable) =
                        Self::compute_exp_ln_powi(self.computable.clone(), exp.clone())
                    {
                        // Reuse the exact rational scale while moving the irrational part
                        // to the cheaper exp(ln(x)*k) representation.
                        return Ok(Self {
                            rational,
                            class: Irrational,
                            computable,
                            signal: None,
                        });
                    }
                }
            }
        }
        self.exp_ln_powi(exp)
    }

    /// Fractional (Non-integer) rational exponent.
    fn pow_fraction(self, exponent: Rational) -> Result<Self, Problem> {
        if exponent.denominator() == unsigned::TWO.deref() {
            // Half-integer powers are common enough to route through powi + sqrt, which
            // exposes exact-square simplifications.
            let n = exponent.shifted_big_integer(1);
            self.powi(n)?.sqrt()
        } else {
            self.pow_arb(Real::new(exponent))
        }
    }

    /// Arbitrary, possibly irrational exponent.
    /// NB: Assumed not to be integer
    fn pow_arb(self, exponent: Self) -> Result<Self, Problem> {
        match self.best_sign() {
            Sign::NoSign => {
                if exponent.best_sign() == Sign::Plus {
                    Ok(Real::zero())
                } else {
                    Err(Problem::NotAnInteger)
                }
            }
            Sign::Minus => Err(Problem::NotAnInteger),
            Sign::Plus => {
                let value = self.fold();
                let exp = exponent.fold();

                Ok(Self {
                    rational: Rational::one(),
                    class: Irrational,
                    computable: value.ln().multiply(exp).exp(),
                    signal: None,
                })
            }
        }
    }

    /// Raise this Real to some Real exponent.
    pub fn pow(self, exponent: Self) -> Result<Self, Problem> {
        if let Exp(ref n) = self.class
            && n == rationals::ONE.deref()
        {
            if self.rational == *rationals::ONE {
                // e^x with unit scale is just exp(x), preserving the symbolic exp path.
                return exponent.exp();
            } else {
                // (a*e)^x = a^x * e^x keeps the e^x part symbolic.
                let left = Real::new(self.rational).pow(exponent.clone())?;
                return Ok(left * exponent.exp()?);
            }
        }
        /* could handle self == 10 =>  10 ^ log10(exponent) specially */
        if exponent.class == One {
            let r = exponent.rational;
            match r.to_big_integer() {
                Some(n) => {
                    return self.powi(n);
                }
                None => {
                    return self.pow_fraction(r);
                }
            }
        }
        if exponent.definitely_zero() {
            return self.powi(BigInt::ZERO);
        }
        self.pow_arb(exponent)
    }

    /// Is this Real an integer ?
    pub fn is_integer(&self) -> bool {
        self.class == One && self.rational.is_integer()
    }

    /// Is this Real known to be rational ?
    pub fn is_rational(&self) -> bool {
        self.class == One
    }

    /// Should we display this Real as a fraction ?
    pub fn prefer_fraction(&self) -> bool {
        self.class == One && self.rational.prefer_fraction()
    }
}

fn real_sign_from_num(sign: Sign) -> RealSign {
    match sign {
        Sign::Minus => RealSign::Negative,
        Sign::NoSign => RealSign::Zero,
        Sign::Plus => RealSign::Positive,
    }
}

fn multiply_public_sign(left: Option<RealSign>, right: Option<RealSign>) -> Option<RealSign> {
    match (left?, right?) {
        (RealSign::Zero, _) | (_, RealSign::Zero) => Some(RealSign::Zero),
        (RealSign::Positive, RealSign::Positive) | (RealSign::Negative, RealSign::Negative) => {
            Some(RealSign::Positive)
        }
        (RealSign::Positive, RealSign::Negative) | (RealSign::Negative, RealSign::Positive) => {
            Some(RealSign::Negative)
        }
    }
}

fn facts_from_rational(rational: &Rational, exact_rational: bool) -> RealStructuralFacts {
    let sign = real_sign_from_num(rational.sign());
    let magnitude = rational.msd_exact().map(|msd| MagnitudeBits {
        msd,
        exact_msd: true,
    });

    RealStructuralFacts {
        sign: Some(sign),
        zero: if sign == RealSign::Zero {
            ZeroKnowledge::Zero
        } else {
            ZeroKnowledge::NonZero
        },
        exact_rational,
        magnitude,
    }
}

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
                PiPow(n) => write!(f, " x Pi**({})", &n),
                PiInv => f.write_str(" / Pi"),
                PiExp(n) => write!(f, " x Pi x e**({})", &n),
                PiInvExp(n) => write!(f, " x e**({}) / Pi", &n),
                PiSqrt(n) => write!(f, " x Pi x √({})", &n),
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
                Exp(n) => write!(f, " x e**({})", &n),
                Ln(n) => write!(f, " x ln({})", &n),
                LnAffine(term) => write!(f, " x ({} + ln({}))", term.offset, term.base),
                LnProduct(product) => {
                    write!(f, " x ln({}) x ln({})", product.left, product.right)
                }
                Log10(n) => write!(f, " x log10({})", &n),
                Sqrt(n) => write!(f, " √({})", &n),
                SinPi(n) => write!(f, " x sin({} x Pi)", &n),
                TanPi(n) => write!(f, " x tan({} x Pi)", &n),
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
            computable: Computable::one(),
            signal: None,
        })
    }
}

use std::ops::*;

impl Real {
    fn simple_log_sum(
        a: Rational,
        b: Rational,
        c: Rational,
        d: Rational,
    ) -> Result<Rational, Problem> {
        // Simplify a*ln(b) + c*ln(d) as ln(b^a*d^c) when the coefficients are
        // integral. This keeps log-heavy algebra in lightweight Ln forms.
        let Some(a) = a.to_big_integer() else {
            return Err(Problem::NotAnInteger);
        };
        let Some(c) = c.to_big_integer() else {
            return Err(Problem::NotAnInteger);
        };
        /* TODO: Should not attempt to simplify once a, b, c, d are too big */
        let left = b.powi(a)?;
        let right = d.powi(c)?;
        Ok(left * right)
    }

    fn try_add_rational_to_const_term(term: &Real, offset: Rational) -> Option<Real> {
        // Add rational offsets to a recognized pi/e constant without discarding
        // the symbolic certificate. This is the cheap path for facts on values
        // like pi - 3 and e - 2.
        if offset == *rationals::ZERO {
            return Some(term.clone());
        }
        if term.rational.sign() == Sign::NoSign {
            return Some(Real::new(offset));
        }
        let (pi_power, exp_power, existing_offset) = term.class.const_offset_parts()?;
        let class_offset = existing_offset + offset / &term.rational;
        let (class, computable) = Class::make_const_offset(pi_power, exp_power, class_offset)?;
        Some(Real {
            rational: term.rational.clone(),
            class,
            computable,
            signal: None,
        })
    }
}

impl<T: AsRef<Real>> Add<T> for &Real {
    type Output = Real;

    fn add(self, other: T) -> Self::Output {
        let other = other.as_ref();
        if self.class == other.class {
            // Same symbolic basis: combine only the rational scale and keep the existing
            // computable certificate.
            let rational = &self.rational + &other.rational;
            if rational.sign() == Sign::NoSign {
                return Self::Output::zero();
            }
            if self.class == One {
                return Self::Output::new(rational);
            }
            return Self::Output {
                rational,
                class: self.class.clone(),
                computable: self.computable.clone(),
                signal: self.signal.clone(),
            };
        }
        if self.definitely_zero() {
            return other.clone();
        }
        if other.definitely_zero() {
            return self.clone();
        }
        if self.class.is_ln() && other.class.is_ln() {
            // Log sums with integral coefficients can collapse to one Ln node, avoiding a
            // generic computable addition in log-heavy expressions.
            let Ln(b) = self.class.clone() else {
                unreachable!()
            };
            let Ln(d) = other.class.clone() else {
                unreachable!()
            };
            if let Ok(r) =
                Self::Output::simple_log_sum(self.rational.clone(), b, other.rational.clone(), d)
                && let Ok(simple) = Self::Output::ln_rational(r)
            {
                return simple;
            }
        }
        if other.class == One
            && self.class.can_take_const_offset()
            && let Some(sum) =
                Self::Output::try_add_rational_to_const_term(self, other.rational.clone())
        {
            // Preserve certified offsets such as `pi - 3` as exact structural
            // classes. This avoids paying generic addition during sign/MSD
            // predicates on almost-simple constants.
            return sum;
        }
        if self.class == One
            && other.class.can_take_const_offset()
            && let Some(sum) =
                Self::Output::try_add_rational_to_const_term(other, self.rational.clone())
        {
            return sum;
        }
        let left = self.fold_ref();
        let right = other.fold_ref();
        let computable = Computable::add(left, right);
        Self::Output {
            rational: Rational::one(),
            class: Irrational,
            computable,
            signal: None,
        }
    }
}

impl<T: AsRef<Real>> Add<T> for Real {
    type Output = Self;

    fn add(self, other: T) -> Self {
        &self + other.as_ref()
    }
}

impl Neg for Real {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            rational: -self.rational,
            ..self
        }
    }
}

impl Neg for &Real {
    type Output = Real;

    fn neg(self) -> Self::Output {
        let mut ret = self.clone();
        ret.rational = -ret.rational;
        ret
    }
}

impl<T: AsRef<Real>> Sub<T> for &Real {
    type Output = Real;

    fn sub(self, other: T) -> Self::Output {
        let other = other.as_ref();
        if self.class == other.class {
            // Same symbolic basis subtraction mirrors addition: update the scale only.
            let rational = &self.rational - &other.rational;
            if rational.sign() == Sign::NoSign {
                return Self::Output::zero();
            }
            if self.class == One {
                return Self::Output::new(rational);
            }
            return Self::Output {
                rational,
                class: self.class.clone(),
                computable: self.computable.clone(),
                signal: self.signal.clone(),
            };
        }
        if other.definitely_zero() {
            return self.clone();
        }
        if self.definitely_zero() {
            return -other;
        }
        if self.class.is_ln() && other.class.is_ln() {
            // Log differences use the same ln-product simplifier with a negated
            // coefficient for the right-hand term.
            let Ln(b) = self.class.clone() else {
                unreachable!()
            };
            let Ln(d) = other.class.clone() else {
                unreachable!()
            };
            if let Ok(r) =
                Self::Output::simple_log_sum(self.rational.clone(), b, -other.rational.clone(), d)
                && let Ok(simple) = Self::Output::ln_rational(r)
            {
                return simple;
            }
        }
        if other.class == One && self.class.can_take_const_offset() {
            if let Some(difference) =
                Self::Output::try_add_rational_to_const_term(self, -other.rational.clone())
            {
                return difference;
            }
        }
        if self.class == One && other.class.can_take_const_offset() {
            if let Some(difference) =
                Self::Output::try_add_rational_to_const_term(other, -self.rational.clone())
            {
                return -difference;
            }
        }
        let left = self.fold_ref();
        let right = other.fold_ref().negate();
        let computable = Computable::add(left, right);
        Self::Output {
            rational: Rational::one(),
            class: Irrational,
            computable,
            signal: None,
        }
    }
}

impl<T: AsRef<Real>> Sub<T> for Real {
    type Output = Self;

    fn sub(self, other: T) -> Self {
        &self - other.as_ref()
    }
}

impl Real {
    fn multiply_sqrts<T: AsRef<Rational>>(x: T, y: T) -> Self {
        let x = x.as_ref();
        let y = y.as_ref();
        if x == y {
            // sqrt(x)*sqrt(x) collapses to the exact rational x, eliminating an
            // otherwise expensive symbolic-irrational product.
            Self {
                rational: x.clone(),
                class: One,
                computable: Computable::one(),
                signal: None,
            }
        } else if (x == &Rational::new(2) && y == &Rational::new(3))
            || (x == &Rational::new(3) && y == &Rational::new(2))
        {
            // sqrt(2)*sqrt(3) is common enough in trig-derived matrices to keep
            // as sqrt(6) without running the general square-extraction code.
            Self {
                rational: Rational::one(),
                class: Sqrt(Rational::new(6)),
                computable: Computable::sqrt_rational(Rational::new(6)),
                signal: None,
            }
        } else {
            let product = x * y;
            if product == *rationals::ZERO {
                return Self {
                    rational: product,
                    class: One,
                    computable: Computable::one(),
                    signal: None,
                };
            }
            let (a, b) = product.extract_square_reduced();
            if b == *rationals::ONE {
                // The product contains a full square, so return only the exact
                // rational factor and keep subsequent sign/equality checks cheap.
                return Self {
                    rational: a,
                    class: One,
                    computable: Computable::one(),
                    signal: None,
                };
            }
            Self {
                rational: a,
                class: Sqrt(b.clone()),
                computable: Computable::sqrt_rational(b),
                signal: None,
            }
        }
    }
}

impl<T: AsRef<Real>> Mul<T> for &Real {
    type Output = Real;

    fn mul(self, other: T) -> Self::Output {
        let other = other.as_ref();
        if self.class == One && other.class == One {
            return Self::Output::new(&self.rational * &other.rational);
        }
        if self.definitely_zero() || other.definitely_zero() {
            return Self::Output::zero();
        }
        if self.class == One {
            let rational = &self.rational * &other.rational;
            if other.class == One {
                return Self::Output::new(rational);
            }
            return Self::Output {
                rational,
                class: other.class.clone(),
                computable: other.computable.clone(),
                signal: other.signal.clone(),
            };
        }
        if other.class == One {
            let rational = &self.rational * &other.rational;
            if self.class == One {
                return Self::Output::new(rational);
            }
            return Self::Output {
                rational,
                class: self.class.clone(),
                computable: self.computable.clone(),
                signal: self.signal.clone(),
            };
        }
        // The table below is deliberately explicit. The generic fallback can
        // represent every product, but these hot symbolic arms preserve exact
        // pi/e/sqrt/log structure and avoid building opaque Computable graphs.
        match (&self.class, &other.class) {
            (Sqrt(r), Sqrt(s)) => {
                let square = Self::Output::multiply_sqrts(r, s);
                Self::Output {
                    rational: &square.rational * &self.rational * &other.rational,
                    ..square
                }
            }
            (Exp(r), Exp(s)) => {
                // e^r * e^s = e^(r+s), keeping exponent arithmetic exact.
                let (class, computable) = Class::make_exp(r + s);
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (Pi, Pi) => {
                // pi*pi promotes to the pi-power family instead of a generic
                // irrational product.
                let (class, computable) = Class::make_pi_power(2);
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (PiPow(power), Pi) | (Pi, PiPow(power)) => {
                // Extend existing pi powers in-place; overflow falls back to a
                // generic Computable product rather than wrapping the exponent.
                let Some(power) = power.checked_add(1) else {
                    let rational = &self.rational * &other.rational;
                    return Self::Output {
                        rational,
                        class: Irrational,
                        computable: Computable::multiply(
                            self.computable.clone(),
                            other.computable.clone(),
                        ),
                        signal: None,
                    };
                };
                let (class, computable) = Class::make_pi_power(power);
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (PiPow(left), PiPow(right)) => {
                // Closed pi-power multiplication keeps dense algebra from
                // repeatedly allocating equivalent pi chains.
                let Some(power) = left.checked_add(*right) else {
                    let rational = &self.rational * &other.rational;
                    return Self::Output {
                        rational,
                        class: Irrational,
                        computable: Computable::multiply(
                            self.computable.clone(),
                            other.computable.clone(),
                        ),
                        signal: None,
                    };
                };
                let (class, computable) = Class::make_pi_power(power);
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (Pi, Exp(r)) | (Exp(r), Pi) => {
                // pi*e^q has a compact legacy class because it is a frequent
                // endpoint of exact transcendental simplification.
                let (class, computable) = Class::make_pi_exp(r.clone());
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (PiPow(power), Exp(exp)) | (Exp(exp), PiPow(power)) => {
                // Higher pi powers times e^q use the boxed const-product form so
                // common Real values do not grow to carry the rare fields inline.
                let (class, computable) = Class::make_const_product(i16::from(*power), exp.clone());
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (PiExp(r), Exp(s)) | (Exp(s), PiExp(r)) => {
                // Existing pi*e^q times another e^r only changes the exact
                // exponent; no new multiply node is needed.
                let (class, computable) = Class::make_pi_exp(r + s);
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (ConstProduct(product), Exp(exp)) | (Exp(exp), ConstProduct(product)) => {
                // Keep boxed pi^n*e^q products closed under another e^r factor.
                let (class, computable) =
                    Class::make_const_product(product.pi_power, product.exp_power.clone() + exp);
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (ConstProduct(product), Pi) | (Pi, ConstProduct(product)) => {
                // Multiplying by one more pi is a checked exponent bump. The
                // generic path is still available for deliberately huge powers.
                let Some(pi_power) = product.pi_power.checked_add(1) else {
                    let rational = &self.rational * &other.rational;
                    return Self::Output {
                        rational,
                        class: Irrational,
                        computable: Computable::multiply(
                            self.computable.clone(),
                            other.computable.clone(),
                        ),
                        signal: None,
                    };
                };
                let (class, computable) =
                    Class::make_const_product(pi_power, product.exp_power.clone());
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (ConstProduct(product), PiPow(power)) | (PiPow(power), ConstProduct(product)) => {
                // Same closure for pi^k factors; keeping it exact helps matrix
                // products cancel pi powers later in division.
                let Some(pi_power) = product.pi_power.checked_add(i16::from(*power)) else {
                    let rational = &self.rational * &other.rational;
                    return Self::Output {
                        rational,
                        class: Irrational,
                        computable: Computable::multiply(
                            self.computable.clone(),
                            other.computable.clone(),
                        ),
                        signal: None,
                    };
                };
                let (class, computable) =
                    Class::make_const_product(pi_power, product.exp_power.clone());
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (ConstProduct(left), ConstProduct(right)) => {
                // Fully factored pi^n*e^q products combine by exact exponent
                // arithmetic and retain their reusable computable cache.
                let Some(pi_power) = left.pi_power.checked_add(right.pi_power) else {
                    let rational = &self.rational * &other.rational;
                    return Self::Output {
                        rational,
                        class: Irrational,
                        computable: Computable::multiply(
                            self.computable.clone(),
                            other.computable.clone(),
                        ),
                        signal: None,
                    };
                };
                let (class, computable) =
                    Class::make_const_product(pi_power, left.exp_power.clone() + &right.exp_power);
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (Pi, Sqrt(r)) | (Sqrt(r), Pi) => {
                // pi*sqrt(r) has a compact direct class because it appears in
                // exact trig constants and BLAS-style products.
                let (class, computable) = Class::make_pi_sqrt(r.clone());
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (Exp(exp), Sqrt(r)) | (Sqrt(r), Exp(exp)) => {
                // e^q*sqrt(r) is kept factored so later multiply/divide can peel
                // off the exact exponential and radicand pieces.
                let (class, computable) = Class::make_const_product_sqrt(0, exp.clone(), r.clone());
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (PiExp(exp), Sqrt(r)) | (Sqrt(r), PiExp(exp)) => {
                // Keep the common `(pi*e^q)*sqrt(r)` construction out of the
                // generic fallback; scalar and BLAS kernels create this form
                // often enough that the direct arm pays for itself.
                let (class, computable) = Class::make_const_product_sqrt(1, exp.clone(), r.clone());
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (PiInvExp(exp), Sqrt(r)) | (Sqrt(r), PiInvExp(exp)) => {
                // The signed pi exponent is part of the factored sqrt class, so
                // e^q/pi times sqrt(r) remains easy to divide by pi or sqrt(r).
                let (class, computable) =
                    Class::make_const_product_sqrt(-1, exp.clone(), r.clone());
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (ConstProduct(product), Sqrt(r)) | (Sqrt(r), ConstProduct(product)) => {
                // Attach a sqrt factor to an existing pi/e product without
                // losing the separate radicand needed for rationalization.
                let (class, computable) = Class::make_const_product_sqrt(
                    product.pi_power,
                    product.exp_power.clone(),
                    r.clone(),
                );
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            (PiSqrt(r), Sqrt(s)) | (Sqrt(s), PiSqrt(r)) if r == s => {
                // pi*sqrt(r)*sqrt(r) collapses the sqrt pair into the rational
                // scale, leaving a plain pi certificate.
                let rational = &self.rational * &other.rational * r;
                Self::Output {
                    rational,
                    class: Pi,
                    computable: Computable::pi(),
                    signal: None,
                }
            }
            (Ln(r), Ln(s)) => {
                // Products of simple logs get a sorted symbolic class so
                // ln(a)*ln(b) and ln(b)*ln(a) share equality and sign facts.
                let (class, computable) = Class::make_ln_product(r.clone(), s.clone());
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable,
                    signal: None,
                }
            }
            _ => {
                if self.class.has_const_product_sqrt_factor()
                    || other.class.has_const_product_sqrt_factor()
                {
                    if let (
                        Some((left_pi, left_exp, left_rad)),
                        Some((right_pi, right_exp, right_rad)),
                    ) = (
                        self.class.const_product_sqrt_parts(),
                        other.class.const_product_sqrt_parts(),
                    ) {
                        if let Some(pi_power) = left_pi.checked_add(right_pi) {
                            let square = Self::Output::multiply_sqrts(&left_rad, &right_rad);
                            let rational = &square.rational * &self.rational * &other.rational;
                            let exp_power = left_exp + right_exp;
                            match square.class {
                                One => {
                                    let (class, computable) =
                                        Class::make_const_product(pi_power, exp_power);
                                    return Self::Output {
                                        rational,
                                        class,
                                        computable,
                                        signal: None,
                                    };
                                }
                                Sqrt(radicand) => {
                                    let (class, computable) = Class::make_const_product_sqrt(
                                        pi_power, exp_power, radicand,
                                    );
                                    return Self::Output {
                                        rational,
                                        class,
                                        computable,
                                        signal: None,
                                    };
                                }
                                _ => unreachable!(),
                            }
                        }
                    }
                    if let (Some((sqrt_pi, sqrt_exp, radicand)), Some((product_pi, product_exp))) = (
                        self.class.const_product_sqrt_parts(),
                        other.class.const_product_parts(),
                    ) {
                        if let Some(pi_power) = sqrt_pi.checked_add(product_pi) {
                            // General sqrt-product closure covers less common forms such as
                            // `(pi*sqrt(2))*e` without moving hot `pi*sqrt(n)` arms.
                            let (class, computable) = Class::make_const_product_sqrt(
                                pi_power,
                                sqrt_exp + product_exp,
                                radicand,
                            );
                            let rational = &self.rational * &other.rational;
                            return Self::Output {
                                rational,
                                class,
                                computable,
                                signal: None,
                            };
                        }
                    }
                    if let (Some((product_pi, product_exp)), Some((sqrt_pi, sqrt_exp, radicand))) = (
                        self.class.const_product_parts(),
                        other.class.const_product_sqrt_parts(),
                    ) {
                        if let Some(pi_power) = product_pi.checked_add(sqrt_pi) {
                            let (class, computable) = Class::make_const_product_sqrt(
                                pi_power,
                                product_exp + sqrt_exp,
                                radicand,
                            );
                            let rational = &self.rational * &other.rational;
                            return Self::Output {
                                rational,
                                class,
                                computable,
                                signal: None,
                            };
                        }
                    }
                }
                if let Some((class, computable)) =
                    Class::multiply_const_products(&self.class, &other.class)
                {
                    // Existing pi^n * e^q forms are closed under multiplication. Keep this
                    // fallback after the specialized arms so sqrt-heavy paths do not pay it.
                    let rational = &self.rational * &other.rational;
                    return Self::Output {
                        rational,
                        class,
                        computable,
                        signal: None,
                    };
                }
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class: Irrational,
                    computable: Computable::multiply(
                        self.computable.clone(),
                        other.computable.clone(),
                    ),
                    signal: None,
                }
            }
        }
    }
}

impl<T: AsRef<Real>> Mul<T> for Real {
    type Output = Self;

    fn mul(self, other: T) -> Self {
        &self * other.as_ref()
    }
}

impl<T: AsRef<Real>> Div<T> for &Real {
    type Output = Result<Real, Problem>;

    fn div(self, other: T) -> Self::Output {
        let other = other.as_ref();
        if other.definitely_zero() {
            return Err(Problem::DivideByZero);
        }
        if self.definitely_zero() {
            return Ok(Real::zero());
        }
        if self.class == other.class {
            let rational = &self.rational / &other.rational;
            return Ok(Real::new(rational));
        }
        if other.class == One {
            let rational = &self.rational / &other.rational;
            if self.class == One {
                return Ok(Real::new(rational));
            }
            return Ok(Real {
                rational,
                class: self.class.clone(),
                computable: self.computable.clone(),
                signal: self.signal.clone(),
            });
        }
        // These small constant-product quotient arms intentionally duplicate the
        // generalized helper below. A simpler "always use divide_const_products"
        // version improved rare deep products but regressed tiny hot cases such as
        // `e / pi`, so keep the fast arms for one-step pi/e reductions.
        match (&self.class, &other.class) {
            (PiPow(power), Pi) if *power > 1 => {
                let (class, computable) = Class::make_pi_power(power - 1);
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class,
                    computable,
                    signal: None,
                });
            }
            (ConstProduct(product), Exp(exp)) => {
                let (class, computable) =
                    Class::make_const_product(product.pi_power, &product.exp_power - exp);
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class,
                    computable,
                    signal: None,
                });
            }
            (ConstProduct(product), Pi) if product.pi_power > 0 => {
                let (class, computable) =
                    Class::make_const_product(product.pi_power - 1, product.exp_power.clone());
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class,
                    computable,
                    signal: None,
                });
            }
            (PiExp(exp), Exp(divisor_exp)) => {
                let (class, computable) = Class::make_pi_exp(exp - divisor_exp);
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class,
                    computable,
                    signal: None,
                });
            }
            (PiExp(exp), Pi) => {
                let (class, computable) = Class::make_exp(exp.clone());
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class,
                    computable,
                    signal: None,
                });
            }
            _ => {}
        }
        if let (Sqrt(left), Sqrt(right)) = (&self.class, &other.class)
            && let Some(right_integer) = right.to_big_integer()
        {
            // Rationalize sqrt(a)/sqrt(b) as sqrt(a*b)/b when b is an integer.
            // This keeps simple radical quotients exact instead of using
            // `other.inverse()` and losing the radicand certificate.
            let square = Real::multiply_sqrts(left, right);
            let denominator = &other.rational * Rational::from_bigint(right_integer);
            return Ok(Real {
                rational: &square.rational * &self.rational / denominator,
                ..square
            });
        }
        if let Some((class, computable)) = Class::divide_const_products(&self.class, &other.class) {
            // Keep the signed pi^n * e^q quotient ahead of the general sqrt
            // fallback. Tiny hot divisions such as `1 / pi` and `e / pi`
            // are more common than factored sqrt quotients in matrix kernels.
            return Ok(Real {
                rational: &self.rational / &other.rational,
                class,
                computable,
                signal: None,
            });
        }
        if self.class.has_const_product_sqrt_factor() || other.class.has_const_product_sqrt_factor()
        {
            if let (Some((left_pi, left_exp, left_rad)), Some((right_pi, right_exp, right_rad))) = (
                self.class.const_product_sqrt_parts(),
                other.class.const_product_sqrt_parts(),
            ) {
                if let Some(pi_power) = left_pi.checked_sub(right_pi) {
                    // Rationalize sqrt-heavy quotients before falling back to `other.inverse()`.
                    // This keeps `(pi*e*sqrt(2))/(e*sqrt(3))` as one factored sqrt
                    // product instead of an opaque division graph.
                    let square = Real::multiply_sqrts(&left_rad, &right_rad);
                    let denominator = &other.rational * right_rad;
                    let rational = &square.rational * &self.rational / denominator;
                    let exp_power = left_exp - right_exp;
                    return Ok(match square.class {
                        One => {
                            let (class, computable) =
                                Class::make_const_product(pi_power, exp_power);
                            Real {
                                rational,
                                class,
                                computable,
                                signal: None,
                            }
                        }
                        Sqrt(radicand) => {
                            let (class, computable) =
                                Class::make_const_product_sqrt(pi_power, exp_power, radicand);
                            Real {
                                rational,
                                class,
                                computable,
                                signal: None,
                            }
                        }
                        _ => unreachable!(),
                    });
                }
            }
            if let (Some((sqrt_pi, sqrt_exp, radicand)), Some((product_pi, product_exp))) = (
                self.class.const_product_sqrt_parts(),
                other.class.const_product_parts(),
            ) {
                if let Some(pi_power) = sqrt_pi.checked_sub(product_pi) {
                    // Divide out only the pi/e product and leave the sqrt factor
                    // intact for later exact radical products.
                    let (class, computable) =
                        Class::make_const_product_sqrt(pi_power, sqrt_exp - product_exp, radicand);
                    return Ok(Real {
                        rational: &self.rational / &other.rational,
                        class,
                        computable,
                        signal: None,
                    });
                }
            }
            if let (Some((product_pi, product_exp)), Some((sqrt_pi, sqrt_exp, radicand))) = (
                self.class.const_product_parts(),
                other.class.const_product_sqrt_parts(),
            ) {
                if let Some(pi_power) = product_pi.checked_sub(sqrt_pi) {
                    // Dividing by sqrt(r) multiplies numerator and denominator
                    // by sqrt(r); keep the remaining sqrt(r) factored.
                    let denominator = &other.rational * radicand.clone();
                    let rational = &self.rational / denominator;
                    let (class, computable) =
                        Class::make_const_product_sqrt(pi_power, product_exp - sqrt_exp, radicand);
                    return Ok(Real {
                        rational,
                        class,
                        computable,
                        signal: None,
                    });
                }
            }
        }
        // Simplify ln(x) / ln(10) to just log10(x)
        if other.class.is_ln() && self.class.is_ln() {
            if let Ln(s) = other.class.clone() {
                if s == *rationals::TEN {
                    // log10 is a smaller exact certificate than a quotient of
                    // two logs and gives equality/fact queries a direct shape.
                    let Ln(r) = &self.class else {
                        unreachable!();
                    };
                    let rational = &self.rational / &other.rational;
                    let ln10 = constants::scaled_ln(10, 1).unwrap();
                    let computable = self.computable.clone().multiply(ln10.computable.inverse());
                    return Ok(Real {
                        rational,
                        class: Log10(r.clone()),
                        computable: computable.clone(),
                        ..self.clone()
                    });
                }
            } else {
                unreachable!();
            }
        }

        let inverted = other.inverse_ref()?;
        Ok(self * inverted)
    }
}

impl<T: AsRef<Real>> Div<T> for Real {
    type Output = Result<Self, Problem>;

    fn div(self, other: T) -> Self::Output {
        &self / other.as_ref()
    }
}

// Best efforts only, definitely not adequate for Eq
// Requirements: PartialEq should be transitive and symmetric
// however it needn't be complete or reflexive.
impl PartialEq for Real {
    fn eq(&self, other: &Self) -> bool {
        self.rational == other.rational && self.class == other.class
    }
}

// For a rational this definitely works
impl PartialEq<Rational> for Real {
    fn eq(&self, other: &Rational) -> bool {
        self.class == Class::One && self.rational == *other
    }
}

// Symmetry
impl PartialEq<Real> for Rational {
    fn eq(&self, other: &Real) -> bool {
        other.class == Class::One && *self == other.rational
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn operations_work_on_refs() {
        let a = Real::new(Rational::new(2));
        let b = Real::new(Rational::new(3));
        let c = Real::new(Rational::new(6));
        assert_eq!(&a * &b, c.clone());
        assert_eq!(&c / &b, Ok(a.clone()));
        assert_eq!(&c - &a, Real::new(Rational::new(4)));
        assert_eq!(-&c, Real::new(Rational::new(-6)));
        assert_eq!(&a + &b, Real::new(Rational::new(5)));
    }
}
