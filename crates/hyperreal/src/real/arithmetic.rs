use crate::{
    Computable, DomainFacts, DomainStatus, IdentityFacts, MagnitudeBits, OrderingFacts,
    PrimitiveFacts, PrimitiveFloatStatus, Problem, Rational, RationalFacts, RationalStorageClass,
    RealDetailedFacts, RealSign, RealStructuralFacts, StructuralComparison, StructuralKind,
    SymbolicFacts, ZeroKnowledge, ZeroOneStatus,
};
use num::ToPrimitive;
use num::bigint::{BigInt, BigUint, Sign};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct ConstProductClass {
    // Signed pi power lets reciprocal products such as 1/pi and e^q/pi remain
    // symbolic instead of falling into a generic inverse node.
    pi_power: i16,
    exp_power: Rational,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct ConstOffsetClass {
    // Invariant: the inner value pi^n*e^q + offset is constructed only when
    // cheaply certified positive. The outer Real.rational carries any sign.
    pi_power: i16,
    exp_power: Rational,
    offset: Rational,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct ConstProductSqrtClass {
    // Factored sqrt products are positive internally. Keeping the sqrt separate
    // allows later multiplication/division to cancel it exactly.
    pi_power: i16,
    exp_power: Rational,
    radicand: Rational,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct LnAffineClass {
    // Constructed only for positive offset + ln(base). This preserves the
    // nonzero/sign invariant shared by all non-Irrational classes.
    offset: Rational,
    base: Rational,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct LnProductClass {
    // Bases are sorted at construction so products compare and combine without
    // considering operand order.
    left: Rational,
    right: Rational,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) enum Class {
    // `Class` is a certificate, not the whole value: `Real.rational` scales the
    // mathematical value represented here. All variants except `Irrational` are
    // exact, nonzero, and positive internally unless a comment says otherwise.
    // These symbolic forms are hyperreal-specific performance shortcuts layered
    // on top of the computable exact-real machinery; the invariants are kept
    // adjacent here because sign/zero/fact predicates rely on them for early
    // exits without approximation.
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

    fn is_ln(&self) -> bool {
        // Only simple `Ln(base)` values participate in the two-log sum/difference
        // collapse. Wider log classes intentionally skip that shortcut so the
        // simplifier stays cheap on non-log algebra.
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

    fn const_product_sqrt_computable(radicand: &Rational) -> Computable {
        // Composite `pi^n*e^q*sqrt(r)` values are often cloned through matrix
        // rows. For the dominant square-free residuals 2 and 3, reuse the
        // canonical shared sqrt computable so costly approximations are warmed
        // once and then multiplied by the symbolic pi/e factors. Plain `sqrt(2)`
        // and `pi*sqrt(2)` stay on their older direct constructors above; those
        // tighter one-node paths benchmark faster for isolated scalar use.
        radicand
            .to_integer_i64()
            .and_then(Computable::sqrt_constant)
            .unwrap_or_else(|| Computable::sqrt_rational(radicand.clone()))
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
        let computable = constant.multiply(Self::const_product_sqrt_computable(&radicand));
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
        // The integer extraction is deliberately structural and non-approximating:
        // keep exact reduction in front of numerical refinement, following the
        // exact-real model in Boehm, Cartwright, Riggle, and O'Donnell,
        // "Exact Real Arithmetic: A Case Study in Higher Order Programming",
        // LFP 1986, https://doi.org/10.1145/319838.319860.
        if let Some(base) = base.to_integer_i64() {
            match base {
                2 | 3 | 5 | 6 | 7 | 10 => return Computable::ln_constant(base as u32).unwrap(),
                _ => {}
            }
        }
        Computable::rational(base.clone()).ln()
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

    fn computable_certificate(&self) -> Computable {
        // Exact symbolic classes are small certificates for a computable value.
        // Cloning a cold `Real` in dense algebra should copy the certificate and
        // rebuild this lightweight wrapper instead of duplicating a larger
        // Computable payload whose cache is usually empty.
        match self {
            One => Computable::one(),
            Pi => Computable::pi(),
            PiPow(power) => Self::pi_power_computable(u16::from(*power)),
            PiInv => Computable::pi().inverse(),
            Exp(exp) => Computable::exp_rational(exp.clone()),
            PiExp(exp) => Computable::pi().multiply(Computable::exp_rational(exp.clone())),
            PiInvExp(exp) => Computable::pi()
                .inverse()
                .multiply(Computable::exp_rational(exp.clone())),
            ConstProduct(product) => {
                let pi = Self::signed_pi_power_computable(product.pi_power);
                if product.exp_power == *rationals::ZERO {
                    pi
                } else {
                    pi.multiply(Computable::exp_rational(product.exp_power.clone()))
                }
            }
            ConstOffset(offset) => {
                let constant =
                    Self::make_const_product(offset.pi_power, offset.exp_power.clone()).1;
                Computable::add(constant, Computable::rational(offset.offset.clone()))
            }
            Sqrt(radicand) => Computable::sqrt_rational(radicand.clone()),
            PiSqrt(radicand) => {
                Computable::pi().multiply(Computable::sqrt_rational(radicand.clone()))
            }
            ConstProductSqrt(product) => {
                let constant =
                    Self::make_const_product(product.pi_power, product.exp_power.clone()).1;
                constant.multiply(Computable::sqrt_rational(product.radicand.clone()))
            }
            Ln(base) => Self::ln_computable(base),
            LnAffine(affine) => Computable::add(
                Computable::rational(affine.offset.clone()),
                Self::ln_computable(&affine.base),
            ),
            LnProduct(product) => {
                Self::ln_computable(&product.left).multiply(Self::ln_computable(&product.right))
            }
            Log10(base) => {
                Self::ln_computable(base).multiply(Self::ln_computable(&*rationals::TEN).inverse())
            }
            SinPi(rational) => {
                let argument =
                    Computable::multiply(Computable::pi(), Computable::rational(rational.clone()));
                Computable::prescaled_sin(argument)
            }
            TanPi(rational) => {
                let argument =
                    Computable::multiply(Computable::pi(), Computable::rational(rational.clone()));
                Computable::prescaled_tan(argument)
            }
            Irrational => panic!("opaque irrational classes must carry their Computable payload"),
        }
    }
}

pub(crate) mod rationals {
    use crate::Rational;
    use std::sync::LazyLock;

    pub(crate) static HALF: LazyLock<Rational> =
        LazyLock::new(|| Rational::fraction(1, 2).unwrap());
    pub(crate) static ONE: LazyLock<Rational> = LazyLock::new(|| Rational::new(1));
    pub(crate) static THIRD: LazyLock<Rational> =
        LazyLock::new(|| Rational::fraction(1, 3).unwrap());
    pub(crate) static THREE: LazyLock<Rational> = LazyLock::new(|| Rational::new(3));
    pub(crate) static TWO: LazyLock<Rational> = LazyLock::new(|| Rational::new(2));
    // These tiny rationals sit on exact trig/inverse-trig hot paths. Reusing
    // them avoids repeated BigUint construction while preserving exact symbolic
    // dispatch. This is the same "cheap certificate before refinement" pattern
    // used by exact geometric predicates; see Shewchuk, "Adaptive Precision
    // Floating-Point Arithmetic and Fast Robust Geometric Predicates",
    // Discrete & Computational Geometry 1997.
    pub(crate) static QUARTER: LazyLock<Rational> =
        LazyLock::new(|| Rational::fraction(1, 4).unwrap());
    pub(crate) static SIXTH: LazyLock<Rational> =
        LazyLock::new(|| Rational::fraction(1, 6).unwrap());
    pub(crate) static SIX: LazyLock<Rational> = LazyLock::new(|| Rational::new(6));
    pub(crate) static SEVEN_EIGHTHS: LazyLock<Rational> =
        LazyLock::new(|| Rational::fraction(7, 8).unwrap());
    pub(crate) static ZERO: LazyLock<Rational> = LazyLock::new(Rational::zero);
    pub(crate) static TEN: LazyLock<Rational> = LazyLock::new(|| Rational::new(10));
}

mod constants {
    use super::{Class, ConstOffsetClass, LnAffineClass};
    use crate::{Computable, Rational, Real};
    thread_local! {
        // These are the canonical internal constants. Public constructors clone
        // these symbolic/computable forms instead of rebuilding exact classes and
        // caches on every call. This keeps common pi/log/sqrt certificates in
        // the symbolic layer and delays approximation until `Computable::approx`,
        // matching the lazy exact-real architecture described by Boehm et al.,
        // LFP 1986, https://doi.org/10.1145/319838.319860.
        static PI: Real = Real {
            rational: Rational::one(),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            signal: None,
        };
        static TAU: Real = Real {
            rational: Rational::new(2),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            signal: None,
        };
        static PI_OVER_TWO: Real = Real {
            rational: Rational::fraction(1, 2).unwrap(),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            signal: None,
        };
        static PI_OVER_THREE: Real = Real {
            rational: Rational::fraction(1, 3).unwrap(),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            signal: None,
        };
        static PI_OVER_FOUR: Real = Real {
            rational: Rational::fraction(1, 4).unwrap(),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            signal: None,
        };
        static PI_OVER_SIX: Real = Real {
            rational: Rational::fraction(1, 6).unwrap(),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            signal: None,
        };
        static HALF: Real = Real::new(Rational::fraction(1, 2).unwrap());
        static SQRT_TWO_OVER_TWO: Real = Real {
            rational: Rational::fraction(1, 2).unwrap(),
            class: Class::Sqrt(Rational::new(2)),
            computable: Some(Computable::sqrt_constant(2).unwrap()),
            signal: None,
        };
        static SQRT_THREE_OVER_TWO: Real = Real {
            rational: Rational::fraction(1, 2).unwrap(),
            class: Class::Sqrt(Rational::new(3)),
            computable: Some(Computable::sqrt_constant(3).unwrap()),
            signal: None,
        };
        static SQRT_THREE: Real = Real {
            rational: Rational::one(),
            class: Class::Sqrt(Rational::new(3)),
            computable: Some(Computable::sqrt_constant(3).unwrap()),
            signal: None,
        };
        static SQRT_THREE_OVER_THREE: Real = Real {
            rational: Rational::fraction(1, 3).unwrap(),
            class: Class::Sqrt(Rational::new(3)),
            computable: Some(Computable::sqrt_constant(3).unwrap()),
            signal: None,
        };
        static SQRT_SIX_OVER_THREE: Real = Real {
            rational: Rational::fraction(1, 3).unwrap(),
            class: Class::Sqrt(Rational::new(6)),
            computable: Some(Computable::sqrt_rational(Rational::new(6))),
            signal: None,
        };
        static PI_SQRT_TWO: Real = Real {
            rational: Rational::one(),
            class: Class::PiSqrt(Rational::new(2)),
            computable: Some(Computable::multiply(
                Computable::pi(),
                Computable::sqrt_constant(2).unwrap(),
            )),
            signal: None,
        };
        static LN2: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(2)),
            computable: Some(Computable::ln_constant(2).unwrap()),
            signal: None,
        };
        static LN3: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(3)),
            computable: Some(Computable::ln_constant(3).unwrap()),
            signal: None,
        };
        static HALF_LN3: Real = Real {
            rational: Rational::fraction(1, 2).unwrap(),
            class: Class::Ln(Rational::new(3)),
            computable: Some(Computable::ln_constant(3).unwrap()),
            signal: None,
        };
        static LN5: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(5)),
            computable: Some(Computable::ln_constant(5).unwrap()),
            signal: None,
        };
        static LN6: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(6)),
            computable: Some(Computable::ln_constant(6).unwrap()),
            signal: None,
        };
        static LN7: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(7)),
            computable: Some(Computable::ln_constant(7).unwrap()),
            signal: None,
        };
        static LN10: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(10)),
            computable: Some(Computable::ln_constant(10).unwrap()),
            signal: None,
        };
        static PI_MINUS_THREE: Real = Real {
            rational: Rational::one(),
            class: Class::ConstOffset(Box::new(ConstOffsetClass {
                pi_power: 1,
                exp_power: Rational::zero(),
                offset: Rational::new(-3),
            })),
            computable: Some(Computable::add(
                Computable::pi(),
                Computable::rational(Rational::new(-3)),
            )),
            signal: None,
        };
        static ONE_PLUS_LN2: Real = Real {
            rational: Rational::one(),
            class: Class::LnAffine(Box::new(LnAffineClass {
                offset: Rational::one(),
                base: Rational::new(2),
            })),
            computable: Some(Computable::add(
                Computable::one(),
                Computable::ln_constant(2).unwrap(),
            )),
            signal: None,
        };
        static E: Real = Real {
            rational: Rational::one(),
            class: Class::Exp(Rational::one()),
            computable: Some(Computable::e_constant()),
            signal: None,
        };
    }

    pub(super) fn half() -> Real {
        HALF.with(|real| real.clone())
    }

    pub(super) fn half_ln3() -> Real {
        HALF_LN3.with(|real| real.clone())
    }

    pub(super) fn pi() -> Real {
        PI.with(|real| real.clone())
    }

    pub(super) fn pi_fraction(n: i64, d: u64) -> Option<Real> {
        // Exact inverse trig repeatedly returns these pi fractions. Reusing the
        // cached symbolic Real avoids rational construction and a multiply by pi
        // on the dispatch path while preserving the public representation.
        match (n, d) {
            (1, 2) => Some(PI_OVER_TWO.with(|real| real.clone())),
            (-1, 2) => Some(PI_OVER_TWO.with(|real| -real.clone())),
            (1, 3) => Some(PI_OVER_THREE.with(|real| real.clone())),
            (-1, 3) => Some(PI_OVER_THREE.with(|real| -real.clone())),
            (1, 4) => Some(PI_OVER_FOUR.with(|real| real.clone())),
            (-1, 4) => Some(PI_OVER_FOUR.with(|real| -real.clone())),
            (1, 6) => Some(PI_OVER_SIX.with(|real| real.clone())),
            (-1, 6) => Some(PI_OVER_SIX.with(|real| -real.clone())),
            _ => None,
        }
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

    pub(super) fn sqrt_six_over_three() -> Real {
        SQRT_SIX_OVER_THREE.with(|real| real.clone())
    }

    pub(super) fn pi_sqrt_two() -> Real {
        PI_SQRT_TWO.with(|real| real.clone())
    }

    pub(super) fn sqrt_constant(n: i64) -> Option<Real> {
        match n {
            2 => Some(Real {
                rational: Rational::one(),
                class: Class::Sqrt(Rational::new(2)),
                computable: Some(Computable::sqrt_constant(2).unwrap()),
                signal: None,
            }),
            3 => Some(sqrt_three()),
            _ => None,
        }
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

    pub(super) fn pi_minus_three() -> Real {
        PI_MINUS_THREE.with(|real| real.clone())
    }

    pub(super) fn one_plus_ln2() -> Real {
        ONE_PLUS_LN2.with(|real| real.clone())
    }

    pub(super) fn e() -> Real {
        E.with(|real| real.clone())
    }
}

mod signed {
    use num::{BigInt, One};
    use std::sync::LazyLock;

    // The identity constructor is clearer and avoids a `ToBigInt` temporary.
    pub(super) static ONE: LazyLock<BigInt> = LazyLock::new(BigInt::one);
}

mod unsigned {
    use num::BigUint;
    use num::One;
    use std::sync::LazyLock;

    // Small exact constants use the narrow primitive that contains them; the
    // bigint constructors can widen directly from there.
    pub(super) static ONE: LazyLock<BigUint> = LazyLock::new(BigUint::one);
    pub(super) static TWO: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(2_u8));
    pub(super) static THREE: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(3_u8));
    pub(super) static FOUR: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(4_u8));
    pub(super) static SIX: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(6_u8));
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
#[derive(Debug, Serialize, Deserialize)]
pub struct Real {
    pub(super) rational: Rational,
    pub(super) class: Class,
    // Pure exact rationals do not need a computable payload. Leaving this empty
    // avoids allocating a fresh Computable::one() sentinel for every rational
    // scalar produced by dense algebra and matrix kernels; folding materializes
    // the rational leaf only when a generic approximation kernel actually needs it.
    pub(super) computable: Option<Computable>,
    #[serde(skip)]
    pub(super) signal: Option<Signal>,
}

impl Clone for Real {
    fn clone(&self) -> Self {
        // `Computable` caches are accelerators, not semantic state. Most Real
        // clones in realistic_blas matrix kernels are cold exact symbols, so
        // cloning the full payload just to preserve an empty cache is wasted
        // work. Rebuild exact symbolic computables from the compact class
        // certificate; keep opaque irrational payloads and abort-attached values
        // as true clones because their graph shape or signal cannot be inferred.
        let computable =
            if self.signal.is_some() || matches!(self.class, Irrational | ConstOffset(_)) {
                // ConstOffset payloads are shallow enough to clone and expensive
                // enough to rebuild that scalar rows like 1000*pi+eps regress if
                // every clone reconstructs cached pi plus a rational offset tree.
                self.computable.clone()
            } else if matches!(self.class, One) {
                None
            } else {
                Some(self.class.computable_certificate())
            };

        Self {
            rational: self.rational.clone(),
            class: self.class.clone(),
            computable,
            signal: self.signal.clone(),
        }
    }
}

impl Real {
    fn exact_rational_unchecked(rational: Rational) -> Real {
        Self {
            rational,
            class: One,
            computable: None,
            signal: None,
        }
    }

    fn computable_ref(&self) -> &Computable {
        self.computable
            .as_ref()
            .expect("non-rational Real classes carry a computable payload")
    }

    pub(super) fn computable_clone(&self) -> Computable {
        self.computable
            .clone()
            .unwrap_or_else(|| self.class.computable_certificate())
    }

    pub(super) fn into_computable(self) -> Computable {
        self.computable
            .unwrap_or_else(|| self.class.computable_certificate())
    }

    /// Provide an atomic flag to signal early abort of calculations.
    /// The provided flag can be used e.g. from another execution thread.
    /// Aborted calculations may have incorrect results.
    pub fn abort(&mut self, s: Signal) {
        self.signal = Some(s.clone());
        if let Some(computable) = &mut self.computable {
            computable.abort(s);
        }
    }

    /// Zero, the additive identity.
    pub fn zero() -> Real {
        crate::trace_dispatch!("real", "constructor", "zero");
        Self::exact_rational_unchecked(Rational::zero())
    }

    /// One, the multiplicative identity.
    pub fn one() -> Real {
        crate::trace_dispatch!("real", "constructor", "one");
        Self::exact_rational_unchecked(Rational::one())
    }

    /// The specified [`Rational`] as a Real.
    pub fn new(rational: Rational) -> Real {
        crate::trace_dispatch!("real", "constructor", "rational");
        Self::exact_rational_unchecked(rational)
    }

    /// π, the ratio of a circle's circumference to its diameter.
    pub fn pi() -> Real {
        crate::trace_dispatch!("real", "constructor", "cached-pi");
        constants::pi()
    }

    /// τ, the ratio of a circle's circumference to its radius.
    pub fn tau() -> Real {
        crate::trace_dispatch!("real", "constructor", "cached-tau");
        constants::tau()
    }

    /// e, Euler's number and the base of the natural logarithm function.
    pub fn e() -> Real {
        crate::trace_dispatch!("real", "constructor", "cached-e");
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
pub(crate) fn curve(r: Rational) -> (bool, Rational) {
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
        crate::trace_dispatch!("real", "definitely_zero", "rational-sign");
        self.rational.sign() == Sign::NoSign
    }

    /// Return this value as an owned exact rational when that is structurally known.
    #[inline]
    pub fn exact_rational(&self) -> Option<Rational> {
        match self.class {
            One => Some(self.rational.clone()),
            _ => None,
        }
    }

    /// Return a borrowed exact rational when that is structurally known.
    ///
    /// Higher-level dense algebra kernels use this to batch exact rational
    /// linear combinations without cloning every scalar. It deliberately
    /// exposes only the already-public exact-rational shape; symbolic and
    /// computable values still go through their normal arithmetic paths.
    #[inline]
    pub fn exact_rational_ref(&self) -> Option<&Rational> {
        match self.class {
            One => Some(&self.rational),
            _ => None,
        }
    }

    #[inline]
    fn scaled_by_rational(&self, scale: &Rational) -> Real {
        // Keep exact rational scaling as a structural operation. This is the
        // same fast path used by multiplication when one side is rational, and
        // the dot-product fallback below reuses it so mixed symbolic/rational
        // lanes do not build a generic product just to recover the same shape.
        if scale.sign() == Sign::NoSign || self.rational.sign() == Sign::NoSign {
            return Real::zero();
        }
        if scale.is_one() {
            return self.clone();
        }
        if scale.is_minus_one() {
            return -self;
        }

        let rational = scale * &self.rational;
        if matches!(self.class, One) {
            return Real::new(rational);
        }
        Real {
            rational,
            class: self.class.clone(),
            computable: self.computable.clone(),
            signal: self.signal.clone(),
        }
    }

    /// Returns true when this value is exactly rational with a dyadic denominator.
    ///
    /// This borrowed query exists for matrix and predicate kernels that need a
    /// representation heuristic without cloning the exact rational. Dyadic
    /// rationals reduce by shifts in `Rational`, so algorithms with more
    /// multiplications but fewer shared inverses can be profitable only on this
    /// structural class.
    #[inline]
    pub fn is_exact_dyadic_rational(&self) -> bool {
        matches!(self.class, One) && self.rational.is_dyadic()
    }

    /// Return a fused sum of signed exact-rational products.
    ///
    /// This is intentionally narrower than generic symbolic simplification:
    /// it succeeds only when every factor is already an exact rational. Dense
    /// algebra callers use it for fixed determinant/cofactor polynomials where
    /// reducing each product and partial sum dominates runtime. Non-rational
    /// symbolic and computable values keep their existing arithmetic trees so
    /// precision is still deferred in the established representation.
    pub fn exact_rational_signed_product_sum<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Real; FACTORS]; TERMS],
    ) -> Option<Real> {
        let mut rational_terms = [[rationals::ZERO.deref(); FACTORS]; TERMS];
        for i in 0..TERMS {
            for j in 0..FACTORS {
                rational_terms[i][j] = terms[i][j].exact_rational_ref()?;
            }
        }
        crate::trace_dispatch!("real", "product_sum", "exact-rational-shared-denom");
        Some(Real::new(Rational::signed_product_sum(
            positive_terms,
            rational_terms,
        )))
    }

    /// Conservatively inspect public structural facts about this value.
    #[inline]
    pub fn structural_facts(&self) -> RealStructuralFacts {
        if matches!(self.class, One) {
            crate::trace_dispatch!("real", "structural_facts", "exact-rational");
            return facts_from_rational(&self.rational, true);
        }

        let rational_sign = self.rational.sign();
        if rational_sign == Sign::NoSign {
            crate::trace_dispatch!("real", "structural_facts", "zero-scale");
            return facts_from_rational(&self.rational, false);
        }

        crate::trace_dispatch!(
            "real",
            "structural_facts",
            match self.class {
                One => "exact-rational",
                Irrational => "scaled-computable",
                Pi | PiPow(_) | PiInv | PiExp(_) | PiInvExp(_) | PiSqrt(_) | ConstProduct(_)
                | ConstOffset(_) | ConstProductSqrt(_) | Sqrt(_) | Exp(_) | Ln(_) | LnAffine(_)
                | LnProduct(_) | Log10(_) | SinPi(_) | TanPi(_) => "symbolic-nonzero-scale",
            }
        );

        let computable = self.computable_ref().structural_facts();
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

    /// Return richer opt-in structural facts for dispatch-heavy callers.
    ///
    /// This intentionally does not run approximation or refinement. It is
    /// derived from the same stored rational/class metadata as
    /// `structural_facts`, plus bit-length and denominator-shape checks. Keep
    /// expensive decomposition out of this query so solvers and matrix kernels
    /// can call it speculatively.
    #[inline]
    pub fn detailed_facts(&self) -> RealDetailedFacts {
        let base = self.structural_facts();
        let exact_rational = matches!(self.class, One);
        let cmp_one = if exact_rational {
            structural_cmp_from_ordering(self.rational.cmp_one_structural())
        } else {
            StructuralComparison::Unknown
        };
        let abs_cmp_one = if exact_rational {
            structural_cmp_from_ordering(self.rational.abs_cmp_one_structural())
        } else {
            StructuralComparison::Unknown
        };
        let identity = IdentityFacts {
            known_one: exact_rational && self.rational.is_one(),
            known_minus_one: exact_rational && self.rational.is_minus_one(),
            zero_or_one: if self.rational.sign() == Sign::NoSign {
                ZeroOneStatus::Zero
            } else if exact_rational && self.rational.is_one() {
                ZeroOneStatus::One
            } else {
                ZeroOneStatus::NeitherOrUnknown
            },
        };
        let rational = if exact_rational {
            self.rational.detailed_rational_facts()
        } else {
            RationalFacts {
                exact_integer: false,
                exact_small_integer_i64: false,
                exact_dyadic: false,
                power_of_two: false,
                storage: RationalStorageClass::VeryLarge,
            }
        };
        let ordering = OrderingFacts {
            cmp_one,
            abs_cmp_one,
        };
        let primitive = primitive_facts_from_base(&base);
        let domains = DomainFacts {
            sqrt: domain_from_sign_nonnegative(base.sign),
            log: domain_from_sign_positive(base.sign),
            unit_interval_closed: domain_abs_cmp_one(abs_cmp_one, true),
            unit_interval_open: domain_abs_cmp_one(abs_cmp_one, false),
            acosh: domain_cmp_one_ge(cmp_one),
        };
        let symbolic = SymbolicFacts {
            kind: structural_kind_for_class(&self.class),
            has_sqrt_factor: matches!(self.class, Sqrt(_) | PiSqrt(_) | ConstProductSqrt(_)),
            has_pi_factor: matches!(
                self.class,
                Pi | PiPow(_)
                    | PiInv
                    | PiExp(_)
                    | PiInvExp(_)
                    | PiSqrt(_)
                    | ConstProduct(_)
                    | ConstOffset(_)
                    | ConstProductSqrt(_)
            ),
            has_exp_factor: matches!(
                self.class,
                Exp(_)
                    | PiExp(_)
                    | PiInvExp(_)
                    | ConstProduct(_)
                    | ConstOffset(_)
                    | ConstProductSqrt(_)
            ),
            computable_required: self.computable.is_some() || matches!(self.class, Irrational),
        };

        crate::trace_dispatch!(
            "real",
            "detailed_facts",
            match symbolic.kind {
                StructuralKind::ExactRational => "exact-rational",
                StructuralKind::PiLike => "pi-like",
                StructuralKind::ExpLike => "exp-like",
                StructuralKind::SqrtLike => "sqrt-like",
                StructuralKind::LogLike => "log-like",
                StructuralKind::TrigExact => "trig-exact",
                StructuralKind::ProductConstant => "product-constant",
                StructuralKind::ComputableOpaque => "computable-opaque",
            }
        );

        RealDetailedFacts {
            base,
            identity,
            rational,
            primitive,
            ordering,
            domains,
            symbolic,
        }
    }

    /// Conservatively report whether structural inspection proves this value is zero.
    #[inline]
    pub fn zero_status(&self) -> ZeroKnowledge {
        match self.rational.sign() {
            Sign::NoSign => {
                crate::trace_dispatch!("real", "zero_status", "zero-scale");
                ZeroKnowledge::Zero
            }
            // All named/exact classes are non-zero when their rational scale is
            // non-zero; only opaque computables need refinement. Keep this as a
            // negative test so adding another exact class does not lengthen this
            // predicate-heavy fast path.
            Sign::Minus | Sign::Plus if !matches!(self.class, Irrational) => {
                crate::trace_dispatch!("real", "zero_status", "symbolic-nonzero-scale");
                ZeroKnowledge::NonZero
            }
            Sign::Minus | Sign::Plus => {
                crate::trace_dispatch!("real", "zero_status", "scaled-computable");
                self.computable_ref().zero_status()
            }
        }
    }

    /// Try to prove the sign without refining past `min_precision`.
    #[inline]
    pub fn refine_sign_until(&self, min_precision: i32) -> Option<RealSign> {
        let facts = self.structural_facts();
        if let Some(sign) = facts.sign {
            crate::trace_dispatch!("real", "refine_sign_until", "structural-facts");
            return Some(sign);
        }
        if self.rational.sign() == Sign::NoSign {
            crate::trace_dispatch!("real", "refine_sign_until", "zero-scale");
            return Some(RealSign::Zero);
        }
        crate::trace_dispatch!("real", "refine_sign_until", "computable-refine");
        let computable_sign = self.computable_ref().sign_until(min_precision)?;
        multiply_public_sign(
            Some(real_sign_from_num(self.rational.sign())),
            Some(computable_sign),
        )
    }

    /// Return the three-lane dot product of borrowed reals.
    ///
    /// Exact-rational lanes are accumulated with one shared denominator and a
    /// single final canonicalization. This is the vector/matrix analogue of the
    /// fraction-delaying exact linear-algebra algorithms discussed around
    /// Bareiss elimination and common factors in
    /// https://link.springer.com/article/10.1007/s11786-020-00495-9. The
    /// fallback intentionally preserves the previous product-then-pairwise-add
    /// tree for non-rational symbolic values; sharing that path with the
    /// rational fast path regressed expression-heavy scalar rows. Mixed
    /// symbolic/rational lanes use a narrower structural fallback: exact
    /// rational scales are applied directly and exact-zero terms are omitted,
    /// but dense symbolic lanes still take the original tree. 2026-05
    /// scalar_micro, 200 samples/8s: mixed dot3/dot4 moved from ~848 ns/~1.006
    /// us to ~697 ns/~753 ns; dense dot3/dot4 moved from ~4.01 us/~7.72 us
    /// to ~3.95 us/~7.11 us.
    pub fn dot3_refs(left: [&Real; 3], right: [&Real; 3]) -> Real {
        if let (Some(l0), Some(l1), Some(l2), Some(r0), Some(r1), Some(r2)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            left[2].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
            right[2].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "dot3-exact-rational-shared-denom");
            return Real::new(Rational::dot_products([l0, l1, l2], [r0, r1, r2]));
        }

        Self::dot3_refs_fallback(left, right)
    }

    /// Return a three-lane dot product whose lanes were already classified active.
    ///
    /// This is for callers that already paid for zero-lane facts. It preserves
    /// the shared-denominator exact-rational reducer while avoiding fresh
    /// scalar zero probes in fixed-size matrix lanes.
    pub fn active_dot3_refs(left: [&Real; 3], right: [&Real; 3]) -> Real {
        if let (Some(l0), Some(l1), Some(l2), Some(r0), Some(r1), Some(r2)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            left[2].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
            right[2].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "active-dot3-exact-rational");
            return Real::new(Rational::dot_products([l0, l1, l2], [r0, r1, r2]));
        }

        crate::trace_dispatch!("real", "dot_product", "active-dot3-real-tree");
        Self::sum_dot3_terms(
            Some(Self::dot_product_active_term(left[0], right[0])),
            Some(Self::dot_product_active_term(left[1], right[1])),
            Some(Self::dot_product_active_term(left[2], right[2])),
        )
    }

    #[inline(never)]
    fn dot3_refs_fallback(left: [&Real; 3], right: [&Real; 3]) -> Real {
        // Keep the symbolic fallback out of line so the matrix hot path that
        // exits through the exact-rational branch above remains small enough
        // for LLVM to inline consistently. An inline prototype improved mixed
        // symbolic dots but regressed realistic_blas hyperreal mat4 borrowed
        // multiply by ~2.6% through code layout alone.
        // Keep zero-sparse symbolic rows fast by skipping exact-zero lanes
        // before building intermediate symbolic terms.
        if Self::dot_product_has_structural_term(left[0], right[0])
            || Self::dot_product_has_structural_term(left[1], right[1])
            || Self::dot_product_has_structural_term(left[2], right[2])
        {
            crate::trace_dispatch!("real", "dot_product", "dot3-structural-real-tree");
            return Self::sum_dot3_terms(
                Self::dot_product_term(left[0], right[0]),
                Self::dot_product_term(left[1], right[1]),
                Self::dot_product_term(left[2], right[2]),
            );
        }

        if left[0].rational.sign() == Sign::NoSign
            || right[0].rational.sign() == Sign::NoSign
            || left[1].rational.sign() == Sign::NoSign
            || right[1].rational.sign() == Sign::NoSign
            || left[2].rational.sign() == Sign::NoSign
            || right[2].rational.sign() == Sign::NoSign
        {
            let p0 = Self::dot_product_term(left[0], right[0]);
            let p1 = Self::dot_product_term(left[1], right[1]);
            let p2 = Self::dot_product_term(left[2], right[2]);
            let active_terms =
                usize::from(p0.is_some()) + usize::from(p1.is_some()) + usize::from(p2.is_some());

            match active_terms {
                0 => {
                    crate::trace_dispatch!("real", "dot_product", "dot3-all-zero-real-tree");
                    return Real::zero();
                }
                1..=2 => {
                    crate::trace_dispatch!("real", "dot_product", "dot3-generic-real-tree-sparse");
                    return Self::sum_dot3_terms(p0, p1, p2);
                }
                _ => {
                    crate::trace_dispatch!("real", "dot_product", "dot3-generic-real-tree");
                    return Self::sum_dot3_terms(p0, p1, p2);
                }
            }
        }

        let p0 = left[0] * right[0];
        let p1 = left[1] * right[1];
        let p2 = left[2] * right[2];
        crate::trace_dispatch!("real", "dot_product", "dot3-generic-real-tree");
        let sum01 = &p0 + &p1;
        &sum01 + &p2
    }

    /// Return the four-lane dot product of borrowed reals.
    ///
    /// See [`Self::dot3_refs`] for the performance policy. Four-lane matrix
    /// multiplication gets the largest win from delaying rational
    /// canonicalization because each output cell otherwise builds four product
    /// rationals plus three partial-sum rationals.
    ///
    /// 2026-05 realistic_blas benchmarks: mat4 mul refs on hyperreal moved
    /// from roughly 10.46 us to 4.33 us after this path, and trace constructors
    /// for one borrowed mat4 multiply dropped from 448 rational Reals to 64.
    pub fn dot4_refs(left: [&Real; 4], right: [&Real; 4]) -> Real {
        if let (Some(l0), Some(l1), Some(l2), Some(l3), Some(r0), Some(r1), Some(r2), Some(r3)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            left[2].exact_rational_ref(),
            left[3].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
            right[2].exact_rational_ref(),
            right[3].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "dot4-exact-rational-shared-denom");
            return Real::new(Rational::dot_products([l0, l1, l2, l3], [r0, r1, r2, r3]));
        }

        Self::dot4_refs_fallback(left, right)
    }

    /// Return a four-lane dot product whose lanes were already classified active.
    ///
    /// See [`Self::active_dot3_refs`].
    pub fn active_dot4_refs(left: [&Real; 4], right: [&Real; 4]) -> Real {
        if let (Some(l0), Some(l1), Some(l2), Some(l3), Some(r0), Some(r1), Some(r2), Some(r3)) = (
            left[0].exact_rational_ref(),
            left[1].exact_rational_ref(),
            left[2].exact_rational_ref(),
            left[3].exact_rational_ref(),
            right[0].exact_rational_ref(),
            right[1].exact_rational_ref(),
            right[2].exact_rational_ref(),
            right[3].exact_rational_ref(),
        ) {
            crate::trace_dispatch!("real", "dot_product", "active-dot4-exact-rational");
            return Real::new(Rational::dot_products([l0, l1, l2, l3], [r0, r1, r2, r3]));
        }

        crate::trace_dispatch!("real", "dot_product", "active-dot4-real-tree");
        Self::sum_dot4_terms(
            Some(Self::dot_product_active_term(left[0], right[0])),
            Some(Self::dot_product_active_term(left[1], right[1])),
            Some(Self::dot_product_active_term(left[2], right[2])),
            Some(Self::dot_product_active_term(left[3], right[3])),
        )
    }

    /// Return the three-lane affine combination `c0 * x0 + c1 * x1 + c2 * x2`.
    ///
    /// The first increment keeps the representation boundary: these forms are
    /// currently delegates so existing transform callers can target a named
    /// constructor before stronger symbolic preservation is introduced.
    pub fn linear_combination3_refs(coeffs: [&Real; 3], values: [&Real; 3]) -> Real {
        Self::dot3_refs(coeffs, values)
    }

    /// Return a three-lane linear combination whose lanes were already classified active.
    pub fn active_linear_combination3_refs(coeffs: [&Real; 3], values: [&Real; 3]) -> Real {
        Self::active_dot3_refs(coeffs, values)
    }

    /// Return the four-lane affine combination `c0 * x0 + c1 * x1 + c2 * x2 + c3 * x3`.
    ///
    /// As with [`Self::linear_combination3_refs`], this is intentionally a
    /// thin constructor for the representation slotting work.
    pub fn linear_combination4_refs(coeffs: [&Real; 4], values: [&Real; 4]) -> Real {
        Self::dot4_refs(coeffs, values)
    }

    /// Return a four-lane linear combination whose lanes were already classified active.
    pub fn active_linear_combination4_refs(coeffs: [&Real; 4], values: [&Real; 4]) -> Real {
        Self::active_dot4_refs(coeffs, values)
    }

    /// Return the three-lane affine sum with an explicit offset.
    pub fn affine_combination3_refs(coeffs: [&Real; 3], values: [&Real; 3], offset: &Real) -> Real {
        let zero0 = coeffs[0].definitely_zero() || values[0].definitely_zero();
        let zero1 = coeffs[1].definitely_zero() || values[1].definitely_zero();
        let zero2 = coeffs[2].definitely_zero() || values[2].definitely_zero();
        if zero0 && zero1 && zero2 {
            crate::trace_dispatch!("real", "affine_combination", "affine-combination3-all-zero");
            return offset.clone();
        }

        if offset.definitely_zero() {
            crate::trace_dispatch!(
                "real",
                "affine_combination",
                "affine-combination3-offset-zero"
            );
            return Self::masked_linear_combination3_refs(coeffs, values, [zero0, zero1, zero2]);
        }

        let linear = Self::masked_linear_combination3_refs(coeffs, values, [zero0, zero1, zero2]);
        crate::trace_dispatch!("real", "affine_combination", "affine-combination3");
        offset + linear
    }

    /// Return the four-lane affine sum with an explicit offset.
    pub fn affine_combination4_refs(coeffs: [&Real; 4], values: [&Real; 4], offset: &Real) -> Real {
        let zero0 = coeffs[0].definitely_zero() || values[0].definitely_zero();
        let zero1 = coeffs[1].definitely_zero() || values[1].definitely_zero();
        let zero2 = coeffs[2].definitely_zero() || values[2].definitely_zero();
        let zero3 = coeffs[3].definitely_zero() || values[3].definitely_zero();
        if zero0 && zero1 && zero2 && zero3 {
            crate::trace_dispatch!("real", "affine_combination", "affine-combination4-all-zero");
            return offset.clone();
        }

        if offset.definitely_zero() {
            crate::trace_dispatch!(
                "real",
                "affine_combination",
                "affine-combination4-offset-zero"
            );
            return Self::masked_linear_combination4_refs(
                coeffs,
                values,
                [zero0, zero1, zero2, zero3],
            );
        }

        let linear =
            Self::masked_linear_combination4_refs(coeffs, values, [zero0, zero1, zero2, zero3]);
        crate::trace_dispatch!("real", "affine_combination", "affine-combination4");
        offset + linear
    }

    #[inline]
    fn masked_linear_combination3_refs(
        coeffs: [&Real; 3],
        values: [&Real; 3],
        zero: [bool; 3],
    ) -> Real {
        if !zero[0] && !zero[1] && !zero[2] {
            return Self::active_linear_combination3_refs(coeffs, values);
        }

        crate::trace_dispatch!(
            "real",
            "affine_combination",
            "active-linear-combination3-sparse"
        );
        Self::sum_dot3_terms(
            (!zero[0]).then(|| Self::dot_product_active_term(coeffs[0], values[0])),
            (!zero[1]).then(|| Self::dot_product_active_term(coeffs[1], values[1])),
            (!zero[2]).then(|| Self::dot_product_active_term(coeffs[2], values[2])),
        )
    }

    #[inline]
    fn masked_linear_combination4_refs(
        coeffs: [&Real; 4],
        values: [&Real; 4],
        zero: [bool; 4],
    ) -> Real {
        if !zero[0] && !zero[1] && !zero[2] && !zero[3] {
            return Self::active_linear_combination4_refs(coeffs, values);
        }

        crate::trace_dispatch!(
            "real",
            "affine_combination",
            "active-linear-combination4-sparse"
        );
        Self::sum_dot4_terms(
            (!zero[0]).then(|| Self::dot_product_active_term(coeffs[0], values[0])),
            (!zero[1]).then(|| Self::dot_product_active_term(coeffs[1], values[1])),
            (!zero[2]).then(|| Self::dot_product_active_term(coeffs[2], values[2])),
            (!zero[3]).then(|| Self::dot_product_active_term(coeffs[3], values[3])),
        )
    }

    #[inline(never)]
    fn dot4_refs_fallback(left: [&Real; 4], right: [&Real; 4]) -> Real {
        // See `dot3_refs_fallback` for the code-layout rationale.
        if Self::dot_product_has_structural_term(left[0], right[0])
            || Self::dot_product_has_structural_term(left[1], right[1])
            || Self::dot_product_has_structural_term(left[2], right[2])
            || Self::dot_product_has_structural_term(left[3], right[3])
        {
            crate::trace_dispatch!("real", "dot_product", "dot4-structural-real-tree");
            return Self::sum_dot4_terms(
                Self::dot_product_term(left[0], right[0]),
                Self::dot_product_term(left[1], right[1]),
                Self::dot_product_term(left[2], right[2]),
                Self::dot_product_term(left[3], right[3]),
            );
        }

        if left[0].rational.sign() == Sign::NoSign
            || right[0].rational.sign() == Sign::NoSign
            || left[1].rational.sign() == Sign::NoSign
            || right[1].rational.sign() == Sign::NoSign
            || left[2].rational.sign() == Sign::NoSign
            || right[2].rational.sign() == Sign::NoSign
            || left[3].rational.sign() == Sign::NoSign
            || right[3].rational.sign() == Sign::NoSign
        {
            let p0 = Self::dot_product_term(left[0], right[0]);
            let p1 = Self::dot_product_term(left[1], right[1]);
            let p2 = Self::dot_product_term(left[2], right[2]);
            let p3 = Self::dot_product_term(left[3], right[3]);
            let active_terms = usize::from(p0.is_some())
                + usize::from(p1.is_some())
                + usize::from(p2.is_some())
                + usize::from(p3.is_some());

            match active_terms {
                0 => {
                    crate::trace_dispatch!("real", "dot_product", "dot4-all-zero-real-tree");
                    return Real::zero();
                }
                1..=3 => {
                    crate::trace_dispatch!("real", "dot_product", "dot4-generic-real-tree-sparse");
                    return Self::sum_dot4_terms(p0, p1, p2, p3);
                }
                _ => {
                    crate::trace_dispatch!("real", "dot_product", "dot4-generic-real-tree");
                    return Self::sum_dot4_terms(p0, p1, p2, p3);
                }
            }
        }
        let p0 = left[0] * right[0];
        let p1 = left[1] * right[1];
        let p2 = left[2] * right[2];
        let p3 = left[3] * right[3];
        let sum01 = &p0 + &p1;
        let sum23 = &p2 + &p3;
        crate::trace_dispatch!("real", "dot_product", "dot4-generic-real-tree");
        &sum01 + &sum23
    }

    #[inline]
    fn dot_product_has_structural_term(left: &Real, right: &Real) -> bool {
        // Gate only on the symbolic class. A broader rational-sign precheck
        // also caught malformed zero-scaled symbolic terms, but the extra
        // field reads regressed the dense symbolic dot3 probe by about 4%.
        // Normal `Real` constructors canonicalize exact zero as `Class::One`,
        // so this still covers the practical zero-term shortcut.
        matches!(left.class, One) || matches!(right.class, One)
    }

    #[inline]
    fn dot_product_term(left: &Real, right: &Real) -> Option<Real> {
        if left.rational.sign() == Sign::NoSign || right.rational.sign() == Sign::NoSign {
            return None;
        }
        Some(Self::dot_product_active_term(left, right))
    }

    #[inline]
    fn dot_product_active_term(left: &Real, right: &Real) -> Real {
        if matches!(left.class, One) {
            return right.scaled_by_rational(&left.rational);
        }
        if matches!(right.class, One) {
            return left.scaled_by_rational(&right.rational);
        }
        left * right
    }

    #[inline]
    fn sum_dot3_terms(p0: Option<Real>, p1: Option<Real>, p2: Option<Real>) -> Real {
        match (p0, p1, p2) {
            (None, None, None) => Real::zero(),
            (Some(p), None, None) | (None, Some(p), None) | (None, None, Some(p)) => p,
            (Some(a), Some(b), None) | (Some(a), None, Some(b)) | (None, Some(a), Some(b)) => {
                &a + &b
            }
            (Some(p0), Some(p1), Some(p2)) => {
                let sum01 = &p0 + &p1;
                &sum01 + &p2
            }
        }
    }

    #[inline]
    fn sum_dot4_terms(
        p0: Option<Real>,
        p1: Option<Real>,
        p2: Option<Real>,
        p3: Option<Real>,
    ) -> Real {
        match (p0, p1, p2, p3) {
            (None, None, None, None) => Real::zero(),
            (Some(p0), Some(p1), Some(p2), Some(p3)) => {
                let sum01 = &p0 + &p1;
                let sum23 = &p2 + &p3;
                &sum01 + &sum23
            }
            (p0, p1, p2, p3) => Self::sum_dot_terms([p0, p1, p2, p3]),
        }
    }

    #[inline]
    fn sum_dot_terms<const N: usize>(terms: [Option<Real>; N]) -> Real {
        let mut total = None;
        for term in terms {
            let Some(term) = term else {
                continue;
            };
            total = Some(match total.take() {
                Some(total) => &total + &term,
                None => term,
            });
        }
        total.unwrap_or_else(Real::zero)
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
            crate::trace_dispatch!("real", "best_sign", "symbolic-or-rational");
            self.rational.sign()
        } else {
            crate::trace_dispatch!("real", "best_sign", "scaled-computable");
            match (self.rational.sign(), self.computable_ref().sign()) {
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
            computable: Some(computable),
            signal: None,
        }
    }

    fn irrational_from_computable(computable: Computable) -> Self {
        Self {
            rational: Rational::one(),
            class: Irrational,
            computable: Some(computable),
            signal: None,
        }
    }

    fn integer_pi_offset_residual(&self) -> Option<(bool, Rational)> {
        // `ConstOffset` stores values as scale * (pi + offset). When the scale
        // is an integer k, trig can use k*pi + r periodicity and evaluate only
        // the tiny rational residual r. This is the hot 1000*pi+eps scalar
        // family; falling through to generic computable reduction pays for a
        // half-pi quotient, residual tree, and cached pi wrappers.
        let ConstOffset(offset) = &self.class else {
            return None;
        };
        if offset.pi_power != 1 || offset.exp_power != *rationals::ZERO {
            return None;
        }
        // Only the parity and magnitude are needed here, so borrowing the
        // integer magnitude avoids constructing a temporary signed BigInt.
        let multiple_magnitude = self.rational.integer_magnitude()?;
        let negate_for_odd_multiple = multiple_magnitude.bit(0);
        let residual = &offset.offset
            * Rational::from_integer_magnitude(self.rational.sign(), multiple_magnitude.clone());
        Some((negate_for_odd_multiple, residual))
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
            exact = Some(Self::one());
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
                computable: Some(computable),
                signal: None,
            }
        } else {
            Self {
                rational: Rational::one(),
                class: SinPi(reduced),
                computable: Some(computable),
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
            crate::trace_dispatch!("real", "inverse", "div-by-zero");
            return Err(Problem::DivideByZero);
        }
        match &self.class {
            One => {
                // Rational reciprocals remain exact.
                crate::trace_dispatch!("real", "inverse", "one");
                return Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: One,
                    computable: None,
                    signal: None,
                });
            }
            Sqrt(sqrt) => {
                if let Some(sqrt) = sqrt.integer_magnitude() {
                    crate::trace_dispatch!("real", "inverse", "sqrt-rational-radical");
                    // Rationalize 1/(a*sqrt(n)) when n is integral, keeping a sqrt form
                    // instead of an opaque inverse node.
                    // Radicands are non-negative, so the borrowed BigUint is
                    // the exact type needed for the rational multiplier.
                    let rational = if self.rational.is_one() {
                        // Unit-scaled radicals are the hot path from sqrt table
                        // reductions. Avoid multiplying by one and then
                        // canonicalizing before inversion; see Yap, "Towards
                        // Exact Geometric Computation" (1997), on preserving
                        // exact algebraic structure to avoid unnecessary
                        // refinement/canonicalization work.
                        Rational::from_unsigned_integer(sqrt.clone()).inverse()?
                    } else {
                        (self.rational * Rational::from_unsigned_integer(sqrt.clone())).inverse()?
                    };
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
                crate::trace_dispatch!("real", "inverse", "pi");
                return Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: PiInv,
                    computable: Some(self.computable_clone().inverse()),
                    signal: None,
                });
            }
            PiInv => {
                // Reciprocal-pi is its own class; inverting it restores the
                // canonical cached pi class without generic const-product setup.
                crate::trace_dispatch!("real", "inverse", "pi-inverse");
                return Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: Pi,
                    computable: Some(self.computable_clone().inverse()),
                    signal: None,
                });
            }
            Exp(exp) => {
                // e^x inverts to e^-x symbolically.
                let exp = Neg::neg(exp.clone());
                crate::trace_dispatch!("real", "inverse", "exp");
                return Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: Exp(exp.clone()),
                    computable: Some(Computable::exp_rational(exp)),
                    signal: None,
                });
            }
            PiExp(exp) => {
                // pi*e^x inverts to e^-x/pi, preserving the one-pi-factor class
                // used by division/multiplication fast arms.
                crate::trace_dispatch!("real", "inverse", "pi-exp");
                return Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: PiInvExp(exp.clone().neg()),
                    computable: Some(self.computable_clone().inverse()),
                    signal: None,
                });
            }
            PiInvExp(exp) => {
                // The reciprocal of e^x/pi is pi*e^-x.
                crate::trace_dispatch!("real", "inverse", "pi-inv-exp");
                return Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: PiExp(exp.clone().neg()),
                    computable: Some(self.computable_clone().inverse()),
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
            crate::trace_dispatch!("real", "inverse", "const-product-sqrt");
            let rational = if self.rational.is_one() {
                // Most factored pi/e/sqrt products are unit-scaled. Skipping the
                // `1 * radicand` rational construction avoids one gcd while
                // preserving the exact rationalization identity above; see Yap
                // (1997) on delaying expensive exact-number normalization until
                // it is structurally required.
                radicand.clone().inverse()?
            } else {
                (self.rational * radicand.clone()).inverse()?
            };
            let (class, computable) =
                Class::make_const_product_sqrt(-pi_power, exp_power.neg(), radicand);
            return Ok(Self {
                rational,
                class,
                computable: Some(computable),
                signal: None,
            });
        }
        if let Some((pi_power, exp_power)) = self.class.const_product_parts() {
            // Keep reciprocal constant products symbolic as pi^-n * e^-q. This matters
            // for scalar and matrix division by pi-heavy constants because the product
            // can later collapse back to `One`, `Exp`, `Pi`, or `PiExp`.
            crate::trace_dispatch!("real", "inverse", "const-product");
            let (class, computable) = Class::make_const_product(-pi_power, exp_power.neg());
            return Ok(Self {
                rational: self.rational.inverse()?,
                class,
                computable: Some(computable),
                signal: None,
            });
        }
        crate::trace_dispatch!("real", "inverse", "generic");
        Ok(Self {
            rational: self.rational.clone().inverse()?,
            class: Irrational,
            computable: Some(Computable::inverse(self.computable_clone())),
            signal: None,
        })
    }

    /// The multiplicative inverse of this Real without consuming it.
    pub fn inverse_ref(&self) -> Result<Self, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "inverse_ref", "div-by-zero");
            return Err(Problem::DivideByZero);
        }
        match &self.class {
            One => {
                // Borrowed one-inverse keeps exact rational form and no extra cache.
                crate::trace_dispatch!("real", "inverse_ref", "one");
                Ok(Self::new(self.rational.clone().inverse()?))
            }
            Sqrt(sqrt) => {
                if let Some(sqrt) = sqrt.integer_magnitude() {
                    // Same rationalization as the owned path, but clone only the
                    // rational/computable pieces needed to leave `self` intact.
                    crate::trace_dispatch!("real", "inverse_ref", "sqrt-rational-radical");
                    let rational = if self.rational.is_one() {
                        // Borrowed unit-scaled sqrt inverses are common in
                        // vector normalization and matrix scalar division. The
                        // structural one fact lets us skip a rational multiply
                        // before exact inversion; see Yap (1997).
                        Rational::from_unsigned_integer(sqrt.clone()).inverse()?
                    } else {
                        (&self.rational * Rational::from_unsigned_integer(sqrt.clone()))
                            .inverse()?
                    };
                    return Ok(Self {
                        rational,
                        class: self.class.clone(),
                        computable: self.computable.clone(),
                        signal: None,
                    });
                }
                crate::trace_dispatch!("real", "inverse_ref", "sqrt-generic");
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: Irrational,
                    computable: Some(Computable::inverse(self.computable_clone())),
                    signal: None,
                })
            }
            Pi => {
                // Preserve the dedicated reciprocal-pi class for borrowed scalar
                // division; rebuilding through the generic constant product costs more.
                crate::trace_dispatch!("real", "inverse_ref", "pi");
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: PiInv,
                    computable: Some(self.computable_clone().inverse()),
                    signal: None,
                })
            }
            PiInv => {
                crate::trace_dispatch!("real", "inverse_ref", "pi-inverse");
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: Pi,
                    computable: Some(self.computable_clone().inverse()),
                    signal: None,
                })
            }
            Exp(exp) => {
                // Borrowed inverse keeps e^x symbolic as e^-x, avoiding a generic
                // reciprocal node in matrix/vector scalar division.
                let exp = exp.clone().neg();
                crate::trace_dispatch!("real", "inverse_ref", "exp");
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: Exp(exp.clone()),
                    computable: Some(Computable::exp_rational(exp)),
                    signal: None,
                })
            }
            PiExp(exp) => {
                crate::trace_dispatch!("real", "inverse_ref", "pi-exp");
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: PiInvExp(exp.clone().neg()),
                    computable: Some(self.computable_clone().inverse()),
                    signal: None,
                })
            }
            PiInvExp(exp) => {
                crate::trace_dispatch!("real", "inverse_ref", "pi-inv-exp");
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: PiExp(exp.clone().neg()),
                    computable: Some(self.computable_clone().inverse()),
                    signal: None,
                })
            }
            _ => {
                if let Some((pi_power, exp_power, radicand)) = self.class.const_product_sqrt_parts()
                {
                    // Borrowed path mirrors owned rationalization while cloning
                    // only the reduced rational radicand and symbolic powers.
                    crate::trace_dispatch!("real", "inverse_ref", "const-product-sqrt");
                    let rational = if self.rational.is_one() {
                        // Preserve the same symbolic rationalization but avoid
                        // constructing `1 * radicand` on the hot borrowed path;
                        // this follows the exact-structure-first strategy
                        // described by Yap (1997).
                        radicand.clone().inverse()?
                    } else {
                        (&self.rational * radicand.clone()).inverse()?
                    };
                    let (class, computable) =
                        Class::make_const_product_sqrt(-pi_power, exp_power.neg(), radicand);
                    return Ok(Self {
                        rational,
                        class,
                        computable: Some(computable),
                        signal: None,
                    });
                }
                if let Some((pi_power, exp_power)) = self.class.const_product_parts() {
                    // Rare constant products still stay symbolic in the borrowed
                    // path so `a / (pi^n e^q)` can cancel in the following multiply.
                    crate::trace_dispatch!("real", "inverse_ref", "const-product");
                    let (class, computable) = Class::make_const_product(-pi_power, exp_power.neg());
                    return Ok(Self {
                        rational: self.rational.clone().inverse()?,
                        class,
                        computable: Some(computable),
                        signal: None,
                    });
                }
                crate::trace_dispatch!("real", "inverse_ref", "generic");
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: Irrational,
                    computable: Some(Computable::inverse(self.computable_clone())),
                    signal: None,
                })
            }
        }
    }

    /// The square root of this Real, or a [`Problem`] if that's impossible,
    /// in particular Problem::SqrtNegative if this Real is negative.
    pub fn sqrt(self) -> Result<Real, Problem> {
        match self.best_sign() {
            Sign::Minus => {
                crate::trace_dispatch!("real", "sqrt", "domain-negative");
                return Err(Problem::SqrtNegative);
            }
            Sign::NoSign => {
                crate::trace_dispatch!("real", "sqrt", "exact-zero");
                return Ok(Self::zero());
            }
            Sign::Plus => {}
        }
        match &self.class {
            One if self.rational.extract_square_will_succeed() => {
                // Extract rational square factors before creating sqrt nodes.
                let (square, rest) = self.rational.extract_square_reduced();
                if rest.is_one() {
                    crate::trace_dispatch!("real", "sqrt", "rational-perfect-square");
                    return Ok(Self {
                        rational: square,
                        class: One,
                        computable: None,
                        signal: None,
                    });
                } else if !square.is_one()
                    && let Some(shared) = rest.to_integer_i64().and_then(constants::sqrt_constant)
                {
                    // sqrt(a^2 * r) = a*sqrt(r). For scaled sqrt(2)/sqrt(3),
                    // reuse the canonical shared computable so matrix/vector
                    // clones do not rebuild the same expensive approximation.
                    // Unscaled sqrt(2)/sqrt(3) keep the old local node because
                    // repeated cached approximation of one node is faster.
                    crate::trace_dispatch!("real", "sqrt", "scaled-shared-sqrt-constant");
                    return Ok(Self {
                        rational: square,
                        class: shared.class,
                        computable: shared.computable,
                        signal: None,
                    });
                } else {
                    crate::trace_dispatch!("real", "sqrt", "rational-sqrt-special-form");
                    return Ok(Self {
                        rational: square,
                        class: Sqrt(rest.clone()),
                        computable: Some(Computable::sqrt_rational(rest)),
                        signal: None,
                    });
                }
            }
            Pi if self.rational.extract_square_will_succeed() => {
                // If only the rational scale is a square, keep sqrt(pi) as a
                // computable sqrt rather than inventing a symbolic sqrt-pi class
                // that has not shown benchmark wins.
                let (square, rest) = self.rational.clone().extract_square_reduced();
                if rest.is_one() {
                    crate::trace_dispatch!("real", "sqrt", "pi-scale-computable-sqrt");
                    return Ok(Self {
                        rational: square,
                        class: Irrational,
                        computable: Some(Computable::sqrt(self.into_computable())),
                        signal: None,
                    });
                }
            }
            Exp(exp) if self.rational.extract_square_will_succeed() => {
                // sqrt(e^x) = e^(x/2) when the rational scale is also a square.
                // Square-free residual scales fall through to the factored
                // const-product sqrt path below.
                let (square, rest) = self.rational.clone().extract_square_reduced();
                if rest.is_one() {
                    let exp = exp.clone() / Rational::new(2);
                    crate::trace_dispatch!("real", "sqrt", "exp-half-special-form");
                    return Ok(Self {
                        rational: square,
                        class: Exp(exp.clone()),
                        computable: Some(Computable::exp_rational(exp)),
                        signal: None,
                    });
                }
            }
            _ => (),
        }
        crate::trace_dispatch!("real", "sqrt", "generic-computable");
        Ok(self.make_computable(Computable::sqrt))
    }

    /// Apply the exponential function to this Real parameter.
    pub fn exp(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "exp", "exact-zero-one");
            return Ok(Self::one());
        }
        match &self.class {
            One => {
                // exp(rational) is a first-class symbolic form used heavily by exact
                // constant products.
                crate::trace_dispatch!("real", "exp", "rational-exp-special-form");
                return Ok(Self {
                    rational: Rational::one(),
                    class: Exp(self.rational.clone()),
                    computable: Some(Computable::exp_rational(self.rational)),
                    signal: None,
                });
            }
            Ln(ln) => {
                if let Some(int) = self.rational.to_big_integer() {
                    // exp(k ln n) folds to n^k when k is integral.
                    crate::trace_dispatch!("real", "exp", "integer-log-collapse");
                    return Ok(Self {
                        rational: ln.clone().powi(int)?,
                        class: One,
                        computable: None,
                        signal: None,
                    });
                }
            }
            _ => (),
        }

        crate::trace_dispatch!("real", "exp", "generic-computable");
        Ok(self.make_computable(Computable::exp))
    }

    /// The base 10 logarithm of this Real or Problem::NotANumber if this Real is negative.
    pub fn log10(self) -> Result<Real, Problem> {
        if self.best_sign() != Sign::Plus {
            crate::trace_dispatch!("real", "log10", "domain-not-positive");
            return Err(Problem::NotANumber);
        }
        if let One = &self.class {
            // Scalar construction benches hit exact rationals here heavily.
            // Avoid building ln(x) and then simplifying ln(x)/ln(10).
            return Self::log10_rational(self.rational);
        }
        // Use the cached ln(10) symbolic constant. Division recognizes ln/ln10
        // and can return a lightweight Log10 class for exact log inputs.
        crate::trace_dispatch!("real", "log10", "ln-div-cached-ln10");
        self.ln()? / constants::scaled_ln(10, 1).unwrap()
    }

    fn log10_rational(r: Rational) -> Result<Real, Problem> {
        match r.cmp_one_structural() {
            std::cmp::Ordering::Less => {
                let inv = r.inverse()?;
                return Ok(-Self::log10_rational(inv)?);
            }
            std::cmp::Ordering::Equal => return Ok(Self::zero()),
            std::cmp::Ordering::Greater => {}
        }

        if let Some(n) = r.integer_magnitude()
            && let Some(log) = Self::integer_log(n, 10)
        {
            crate::trace_dispatch!("real", "log10", "rational-power-of-ten");
            return Ok(Self::new(Rational::new(log as i64)));
        }

        crate::trace_dispatch!("real", "log10", "rational-log10-special-form");
        let computable =
            Class::ln_computable(&r).multiply(Class::ln_computable(&*rationals::TEN).inverse());
        Ok(Self {
            rational: Rational::one(),
            class: Log10(r),
            computable: Some(computable),
            signal: None,
        })
    }

    // Find Some(m) integral log with respect to this base or else None
    // n should be positive (not zero) and base should be >= 2
    fn integer_log(n: &BigUint, base: u32) -> Option<u64> {
        use num::Integer;
        // TODO weed out some large failure cases early and return None

        if let Some(mut reduced) = n.to_u64() {
            // The scalar log benches mostly use decimal-sized inputs such as
            // 1e12. For values that fit in a machine word, repeated u64
            // division is much cheaper than allocating BigUint power ladders.
            if reduced <= 1 {
                return None;
            }
            let base = u64::from(base);
            let mut exponent = 0;
            while reduced % base == 0 {
                reduced /= base;
                exponent += 1;
            }
            return if reduced == 1 && exponent > 0 {
                Some(exponent)
            } else {
                None
            };
        }

        // Build powers by repeated squaring, divide by the largest usable power,
        // then walk back down. This recognizes n = base^k without trial-dividing
        // by base k times.
        // Calculate base^2 base^4 base^8 base^16 and so on until it is bigger than next
        let mut result: Option<u64> = None;
        let mut powers: Vec<BigUint> = Vec::new();
        let mut next = BigUint::from(base);
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
        let n = r.integer_magnitude()?;

        // Recognize common integer powers so logs share cached scaled-ln constants
        // instead of creating many unrelated Ln nodes.
        // Check base 10 first because log10/ln scalar benches include 1e12 and
        // 1e-12; probing 2, 3, 5, 6, and 7 first made those cases regress.
        for base in [10, 2, 3, 5, 6, 7] {
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
        match r.cmp_one_structural() {
            std::cmp::Ordering::Less => {
                let inv = r.inverse()?;
                if let Some(answer) = Self::ln_small(&inv) {
                    crate::trace_dispatch!("real", "ln", "rational-inverse-shared-log");
                    return Ok(-answer);
                }
                // Normalize ln(r<1) as -ln(1/r) to improve symbolic sharing.
                let new = Computable::rational(inv.clone());
                crate::trace_dispatch!("real", "ln", "rational-inverse-ln-special-form");
                Ok(Self {
                    rational: Rational::new(-1),
                    class: Ln(inv),
                    computable: Some(Computable::ln(new)),
                    signal: None,
                })
            }
            std::cmp::Ordering::Equal => {
                crate::trace_dispatch!("real", "ln", "rational-one-zero");
                Ok(Self::zero())
            }
            std::cmp::Ordering::Greater => {
                if let Some(answer) = Self::ln_small(&r) {
                    crate::trace_dispatch!("real", "ln", "rational-shared-log");
                    return Ok(answer);
                }
                // Positive rationals above one get a lightweight Ln certificate.
                let new = Computable::rational(r.clone());
                crate::trace_dispatch!("real", "ln", "rational-ln-special-form");
                Ok(Self {
                    rational: Rational::one(),
                    class: Ln(r),
                    computable: Some(Computable::ln(new)),
                    signal: None,
                })
            }
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
            computable: Some(computable),
            signal: None,
        })
    }

    /// The natural logarithm of this Real or Problem::NotANumber if this Real is negative.
    pub fn ln(self) -> Result<Real, Problem> {
        if self.best_sign() != Sign::Plus {
            crate::trace_dispatch!("real", "ln", "domain-not-positive");
            return Err(Problem::NotANumber);
        }
        match &self.class {
            One => return Self::ln_rational(self.rational),
            Exp(exp) => {
                if self.rational.is_one() {
                    // ln(e^x) collapses exactly for the pure exponential class.
                    crate::trace_dispatch!("real", "ln", "pure-exp-collapse");
                    return Ok(Self {
                        rational: exp.clone(),
                        class: One,
                        computable: None,
                        signal: None,
                    });
                }
                if exp == &*rationals::ONE && self.rational == *rationals::TWO {
                    crate::trace_dispatch!("real", "ln", "cached-one-plus-ln2");
                    return Ok(constants::one_plus_ln2());
                }
                // ln(a * e^x) = ln(a) + x for positive rational scale `a`.
                // The positive-offset case is stored as one factored `ln` class
                // so repeated predicates do not traverse a generic add graph.
                let log_scale = Self::ln_rational(self.rational)?;
                if let Some(answer) = Self::try_add_rational_to_ln_term(&log_scale, exp.clone()) {
                    crate::trace_dispatch!("real", "ln", "scaled-exp-affine-log-special-form");
                    return Ok(answer);
                }
                crate::trace_dispatch!("real", "ln", "scaled-exp-log-plus-exponent");
                return Ok(log_scale + Self::new(exp.clone()));
            }
            _ => (),
        }

        crate::trace_dispatch!("real", "ln", "generic-computable");
        Ok(self.make_computable(Computable::ln))
    }

    /// The sine of this Real.
    pub fn sin(self) -> Real {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "sin", "exact-zero");
            return Self::zero();
        }
        match &self.class {
            One => {
                // Plain rational trig still uses Computable, not SinPi/TanPi:
                // those exact certificates are reserved for rational multiples
                // of pi where algebra can later invert them. The owned helper
                // specializes before allocating a generic Ratio leaf.
                let computable = if self.rational.magnitude_at_least_power_of_two(3) {
                    // Keep the large-rational decision at the Real layer too:
                    // this path is already below 100 ns, so avoiding a second
                    // sign/MSD probe matters in Criterion.
                    crate::trace_dispatch!("real", "sin", "large-rational-deferred-node");
                    Computable::sin_large_rational_deferred(self.rational.clone())
                } else {
                    crate::trace_dispatch!("real", "sin", "rational-specialized-computable");
                    Computable::sin_rational(self.rational.clone())
                };
                return Self::irrational_from_computable(computable);
            }
            Pi => {
                // sin(q*pi) has exact small-denominator and reusable SinPi handling.
                crate::trace_dispatch!("real", "sin", "pi-rational-special-form");
                return Self::sin_pi_rational(self.rational);
            }
            _ => (),
        }
        if let Some((negate, residual)) = self.integer_pi_offset_residual() {
            crate::trace_dispatch!("real", "sin", "integer-pi-offset-rewrite");
            let reduced = Self::irrational_from_computable(Computable::sin_rational(residual));
            return if negate { reduced.neg() } else { reduced };
        }

        crate::trace_dispatch!("real", "sin", "generic-computable");
        self.make_computable(Computable::sin)
    }

    /// The cosine of this Real.
    pub fn cos(self) -> Real {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "cos", "exact-zero-one");
            return Self::one();
        }
        match &self.class {
            One => {
                // Same policy as sine: exact pi multiples stay symbolic, while
                // plain rationals enter the specialized computable constructor.
                let computable = if self.rational.magnitude_at_least_power_of_two(3) {
                    crate::trace_dispatch!("real", "cos", "large-rational-deferred-node");
                    Computable::cos_large_rational_deferred(self.rational.clone())
                } else {
                    crate::trace_dispatch!("real", "cos", "rational-specialized-computable");
                    Computable::cos_rational(self.rational.clone())
                };
                return Self::irrational_from_computable(computable);
            }
            Pi => {
                // cos(q*pi) is represented through the same SinPi machinery with a
                // half-turn shift, keeping exact identities in one place.
                crate::trace_dispatch!("real", "cos", "pi-rational-sinpi-rewrite");
                return Self::sin_pi_rational(self.rational + rationals::HALF.clone());
            }
            _ => (),
        }
        if let Some((negate, residual)) = self.integer_pi_offset_residual() {
            crate::trace_dispatch!("real", "cos", "integer-pi-offset-rewrite");
            let reduced = Self::irrational_from_computable(Computable::cos_rational(residual));
            return if negate { reduced.neg() } else { reduced };
        }

        crate::trace_dispatch!("real", "cos", "generic-computable");
        self.make_computable(Computable::cos)
    }

    /// The tangent of this Real.
    pub fn tan(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "tan", "exact-zero");
            return Ok(Self::zero());
        }

        match &self.class {
            One => {
                // For non-pi rational arguments there are no exact tangent
                // certificates, but Computable::tan still applies small/medium
                // argument reductions without first allocating a Ratio leaf.
                crate::trace_dispatch!("real", "tan", "rational-specialized-computable");
                return Ok(Self::irrational_from_computable(Computable::tan_rational(
                    self.rational.clone(),
                )));
            }
            Pi => {
                if self.rational.is_integer() {
                    crate::trace_dispatch!("real", "tan", "pi-integer-zero");
                    return Ok(Self::zero());
                }
                // Rational multiples of pi get exact tangent values for the usual small
                // denominators, otherwise a compact TanPi certificate.
                let (neg, n) = tan_curve(self.rational);
                let mut r: Option<Real> = None;
                let d = n.denominator();
                if d == unsigned::TWO.deref() {
                    crate::trace_dispatch!("real", "tan", "pi-half-pole");
                    return Err(Problem::NotANumber);
                }
                if d == unsigned::THREE.deref() {
                    r = Some(constants::sqrt_three());
                }
                if d == unsigned::FOUR.deref() {
                    r = Some(Self::one());
                }
                if d == unsigned::SIX.deref() {
                    r = Some(constants::sqrt_three_over_three());
                }
                if let Some(real) = r {
                    crate::trace_dispatch!("real", "tan", "pi-rational-exact-table");
                    if neg {
                        return Ok(real.neg());
                    } else {
                        return Ok(real);
                    }
                } else {
                    let new =
                        Computable::multiply(Computable::pi(), Computable::rational(n.clone()));
                    let computable = Computable::prescaled_tan(new);
                    crate::trace_dispatch!("real", "tan", "tanpi-special-form");
                    if neg {
                        return Ok(Self {
                            rational: Rational::new(-1),
                            class: TanPi(n),
                            computable: Some(computable),
                            signal: None,
                        });
                    } else {
                        return Ok(Self {
                            rational: Rational::one(),
                            class: TanPi(n),
                            computable: Some(computable),
                            signal: None,
                        });
                    }
                }
            }
            _ => (),
        }
        if let Some((_negate, residual)) = self.integer_pi_offset_residual() {
            crate::trace_dispatch!("real", "tan", "integer-pi-offset-rewrite");
            return Ok(Self::irrational_from_computable(Computable::tan_rational(
                residual,
            )));
        }

        crate::trace_dispatch!("real", "tan", "generic-computable");
        Ok(self.make_computable(Computable::tan))
    }

    fn pi_fraction(n: i64, d: u64) -> Real {
        if let Some(real) = constants::pi_fraction(n, d) {
            crate::trace_dispatch!("real", "pi_fraction", "cached-special-form");
            return real;
        }
        crate::trace_dispatch!("real", "pi_fraction", "constructed-generic");
        Self::new(Rational::fraction(n, d).unwrap()) * Self::pi()
    }

    fn asin_exact(&self) -> Option<Real> {
        if self.definitely_zero() {
            return Some(Self::zero());
        }

        match &self.class {
            One => {
                // Exact inverse-trig table for rational endpoints and half-angle values.
                if self.rational.is_one() {
                    Some(Self::pi_fraction(1, 2))
                } else if self.rational.is_minus_one() {
                    Some(Self::pi_fraction(-1, 2))
                } else if self.rational == *rationals::HALF {
                    Some(Self::pi_fraction(1, 6))
                } else if self.rational.sign() == Sign::Minus
                    && self.rational.compare_magnitude(&*rationals::HALF)
                        == std::cmp::Ordering::Equal
                {
                    Some(Self::pi_fraction(-1, 6))
                } else {
                    None
                }
            }
            Sqrt(r) => {
                // Recognize sqrt(2)/2 and sqrt(3)/2 forms produced by exact trig.
                // This is a structural table lookup, not a numerical comparison:
                // the radicand must be an exact small integer and the rational
                // scale must have exact magnitude 1/2.
                let sign = self.rational.sign();
                let half_magnitude =
                    self.rational.compare_magnitude(&*rationals::HALF) == std::cmp::Ordering::Equal;
                if !half_magnitude {
                    return None;
                }
                let angle = match r.to_integer_i64()? {
                    2 => rationals::QUARTER.clone(),
                    3 => rationals::THIRD.clone(),
                    _ => return None,
                };

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
                if self.rational.is_one() {
                    Some(Self::new(r.clone()) * Self::pi())
                } else if self.rational.is_minus_one() {
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
                if self.rational.is_one() {
                    Some(Self::pi_fraction(1, 4))
                } else if self.rational.is_minus_one() {
                    Some(Self::pi_fraction(-1, 4))
                } else {
                    None
                }
            }
            Sqrt(r) => {
                if r.to_integer_i64() != Some(3) {
                    return None;
                }
                // atan(sqrt(3)) and atan(sqrt(3)/3) have exact pi-fraction answers.
                let sign = self.rational.sign();
                let angle = if self.rational.abs_cmp_one_structural() == std::cmp::Ordering::Equal {
                    Some(rationals::THIRD.clone())
                } else if self.rational.compare_magnitude(&*rationals::THIRD)
                    == std::cmp::Ordering::Equal
                {
                    Some(rationals::SIXTH.clone())
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
                if self.rational.is_one() {
                    Some(Self::new(r.clone()) * Self::pi())
                } else if self.rational.is_minus_one() {
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
            crate::trace_dispatch!("real", "asin", "exact-special-form");
            return Ok(exact);
        }
        if self.class == One {
            // Plain rationals use the computable asin kernel after cheap domain checks; it
            // has tiny/endpoint specializations that would be obscured by the atan formula.
            if self.rational.abs_cmp_one_structural() == std::cmp::Ordering::Greater {
                crate::trace_dispatch!("real", "asin", "rational-domain-error");
                return Err(Problem::NotANumber);
            }

            crate::trace_dispatch!("real", "asin", "rational-computable");
            return Ok(self.make_computable(|value| value.asin()));
        }
        if let Sqrt(r) = &self.class
            && self
                .rational
                .compare_magnitude_squared_times(r, &*rationals::ONE)
                == std::cmp::Ordering::Greater
        {
            crate::trace_dispatch!("real", "asin", "sqrt-domain-error");
            return Err(Problem::NotANumber);
        }
        if matches!(&self.class, Sqrt(_)) {
            // Sqrt inputs commonly arise from exact trig; keep them on the computable asin
            // path so recognizable forms survive longer.
            crate::trace_dispatch!("real", "asin", "sqrt-computable");
            return Ok(self.make_computable(|value| value.asin()));
        }

        // Generic identity asin(x) = atan(x / sqrt(1-x^2)).
        crate::trace_dispatch!("real", "asin", "generic-atan-sqrt-rewrite");
        let one = Self::one();
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
            if self.rational.is_one() {
                // acos(1) is exactly zero and must not enter the generic kernel.
                crate::trace_dispatch!("real", "acos", "exact-one-zero");
                return Ok(Self::zero());
            }
            if self.rational.is_minus_one() {
                // acos(-1) is exactly pi, using the cached internal constant.
                crate::trace_dispatch!("real", "acos", "exact-minus-one-pi");
                return Ok(Self::pi());
            }
            if self.rational.abs_cmp_one_structural() == std::cmp::Ordering::Greater {
                // Exact rational domain failures are rejected before any
                // approximation machinery is constructed.
                crate::trace_dispatch!("real", "acos", "rational-domain-error");
                return Err(Problem::NotANumber);
            }
        }
        if let Some(asin) = self.asin_exact() {
            // acos(x) shares the exact asin table through pi/2 - asin(x).
            crate::trace_dispatch!("real", "acos", "asin-table-special-form");
            return Ok(Self::pi_fraction(1, 2) - asin);
        }
        if let Sqrt(r) = &self.class
            && self
                .rational
                .compare_magnitude_squared_times(r, &*rationals::ONE)
                == std::cmp::Ordering::Greater
        {
            crate::trace_dispatch!("real", "acos", "sqrt-domain-error");
            return Err(Problem::NotANumber);
        }

        crate::trace_dispatch!("real", "acos", "generic-computable");
        Ok(self.make_computable(|value| value.acos()))
    }

    /// The inverse tangent of this Real.
    pub fn atan(self) -> Result<Real, Problem> {
        if let Some(exact) = self.atan_exact() {
            crate::trace_dispatch!("real", "atan", "exact-special-form");
            return Ok(exact);
        }

        crate::trace_dispatch!("real", "atan", "generic-computable");
        Ok(self.make_computable(Computable::atan))
    }

    /// The inverse hyperbolic sine of this Real.
    pub fn asinh(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "asinh", "exact-zero");
            return Ok(Self::zero());
        }
        if self.class == One && self.rational.msd_exact().is_some_and(|msd| msd <= -4) {
            // Tiny exact rationals have a dedicated computable asinh series.
            // Enter it directly before Real-level odd symmetry expands the
            // expression into a larger ln1p graph.
            crate::trace_dispatch!("real", "asinh", "tiny-rational-computable");
            return Ok(self.make_computable(Computable::asinh));
        }
        let folded = self.fold_ref();
        let (known_sign, planning_msd) = folded.planning_sign_and_msd();
        if known_sign == Some(Sign::Minus) {
            crate::trace_dispatch!("real", "asinh", "negative-symmetry");
            return Ok(self.neg().asinh()?.neg());
        } else if known_sign.is_none() && self.best_sign() == Sign::Minus {
            // Fall back to the slower exact sign check only when the planning
            // layer cannot determine sign from symbolic structure.
            crate::trace_dispatch!("real", "asinh", "negative-symmetry-fallback");
            return Ok(self.neg().asinh()?.neg());
        }
        let is_near_zero = match planning_msd.flatten() {
            Some(msd) => msd < 3,
            None => folded.approx(-4) <= BigInt::from(64_u8),
        };
        if is_near_zero {
            // Near zero, delegate to the deferred computable ln1p reduction so
            // public construction stays cheap without giving up the stable
            // approximation identity.
            crate::trace_dispatch!("real", "asinh", "near-zero-deferred-node");
            return Ok(self.make_computable(Computable::asinh_near_zero_deferred));
        }
        crate::trace_dispatch!("real", "asinh", "direct-deferred-node");
        Ok(self.make_computable(Computable::asinh_direct_deferred))
    }

    /// The inverse hyperbolic cosine of this Real, or [`Problem::NotANumber`] for values < 1.
    pub fn acosh(self) -> Result<Real, Problem> {
        if self.class == One {
            match self.rational.cmp_one_structural() {
                std::cmp::Ordering::Equal => {
                    crate::trace_dispatch!("real", "acosh", "exact-one-zero");
                    return Ok(Self::zero());
                }
                std::cmp::Ordering::Less => {
                    crate::trace_dispatch!("real", "acosh", "rational-domain-error");
                    return Err(Problem::NotANumber);
                }
                std::cmp::Ordering::Greater => {}
            }
            if self.rational.msd_exact().is_some_and(|msd| msd >= 3) {
                // Large exact rationals cannot be in the cancellation-prone
                // neighborhood of one, so skip the low-precision proximity
                // probe and let the computable acosh kernel use its direct
                // large-input identity.
                crate::trace_dispatch!("real", "acosh", "large-rational-direct-deferred-node");
                return Ok(self.make_computable(Computable::acosh_direct_deferred));
            }
        } else if let Sqrt(r) = &self.class {
            // Domain-check factored sqrt values exactly: (a*sqrt(r))^2 = a^2*r.
            if self.rational.sign() == Sign::Minus
                || self
                    .rational
                    .compare_magnitude_squared_times(r, &*rationals::ONE)
                    == std::cmp::Ordering::Less
            {
                crate::trace_dispatch!("real", "acosh", "sqrt-domain-error");
                return Err(Problem::NotANumber);
            }
        } else {
            let one = Self::one();
            if (self.clone() - one).best_sign() == Sign::Minus {
                crate::trace_dispatch!("real", "acosh", "generic-domain-error");
                return Err(Problem::NotANumber);
            }
        }
        let folded = self.fold_ref();
        let planned_acosh_msd = folded.planning_sign_and_msd().1;
        let is_near_one = match planned_acosh_msd.flatten() {
            Some(msd) => msd < 3,
            None => folded.approx(-4) <= BigInt::from(64_u8),
        };
        if is_near_one {
            // Near one, delegate to the deferred computable ln1p/sqrt
            // reduction so public construction does not allocate the full
            // approximation graph.
            crate::trace_dispatch!("real", "acosh", "near-one-deferred-node");
            return Ok(self.make_computable(Computable::acosh_near_one_deferred));
        }
        crate::trace_dispatch!("real", "acosh", "direct-deferred-node");
        Ok(self.make_computable(Computable::acosh_direct_deferred))
    }

    /// The inverse hyperbolic tangent of this Real.
    ///
    /// Returns [`Problem::Infinity`] at the endpoints `-1` and `1`, or
    /// [`Problem::NotANumber`] outside `(-1, 1)`.
    pub fn atanh(self) -> Result<Real, Problem> {
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "atanh", "exact-zero");
            return Ok(Self::zero());
        }
        if self.class == One {
            if self.rational.is_one() || self.rational.is_minus_one() {
                crate::trace_dispatch!("real", "atanh", "endpoint-infinity");
                return Err(Problem::Infinity);
            }
            if self.rational.abs_cmp_one_structural() == std::cmp::Ordering::Greater {
                crate::trace_dispatch!("real", "atanh", "rational-domain-error");
                return Err(Problem::NotANumber);
            }
            if self.rational.msd_exact().is_some_and(|msd| msd <= -4) {
                // Tiny rational atanh is faster in the dedicated computable kernel than
                // building ln((1+x)/(1-x))/2.
                crate::trace_dispatch!("real", "atanh", "tiny-rational-computable");
                return Ok(self.make_computable(Computable::atanh));
            }
            if self.rational.compare_magnitude(&*rationals::SEVEN_EIGHTHS)
                != std::cmp::Ordering::Less
            {
                // Endpoint-adjacent rationals are hot in scalar predicates and
                // benchmarks; a deferred computable ln-ratio avoids eagerly
                // allocating the exact logarithm tree.
                crate::trace_dispatch!("real", "atanh", "endpoint-deferred-node");
                return Ok(self.make_computable(Computable::atanh_direct_deferred));
            }

            // This path deliberately keeps atanh(x) as the exact symbolic
            // `ln((1+x)/(1-x))/2` instead of approximating. Reuse the cached
            // unit rational so each construction only clones a tiny exact leaf
            // and does not rebuild/canonicalize it before the two rational
            // additions. This follows Boehm et al., "Exact Real Arithmetic: A
            // Case Study in Higher Order Programming" (1986), where symbolic
            // construction is kept separate from later numerical refinement.
            if self.rational == *rationals::HALF {
                crate::trace_dispatch!("real", "atanh", "rational-half-ln3-special-form");
                return Ok(constants::half_ln3());
            }
            if -&self.rational == *rationals::HALF {
                crate::trace_dispatch!("real", "atanh", "rational-minus-half-ln3-special-form");
                return Ok(-constants::half_ln3());
            }
            let one = rationals::ONE.clone();
            let ratio = (one.clone() + self.rational.clone()) / (one - self.rational);
            if ratio == *rationals::THREE {
                crate::trace_dispatch!("real", "atanh", "rational-half-ln3-special-form");
                return Ok(constants::half_ln3());
            }
            if ratio == *rationals::THIRD {
                crate::trace_dispatch!("real", "atanh", "rational-minus-half-ln3-special-form");
                return Ok(-constants::half_ln3());
            }
            // Non-tiny rationals can remain an exact logarithm ratio.
            crate::trace_dispatch!("real", "atanh", "rational-log-ratio-special-form");
            return Ok(Self::ln_rational(ratio)? * constants::half());
        }
        let one_real = Self::one();
        if self == one_real || self == -one_real.clone() {
            crate::trace_dispatch!("real", "atanh", "endpoint-infinity");
            return Err(Problem::Infinity);
        }
        if let Sqrt(r) = &self.class
            && self
                .rational
                .compare_magnitude_squared_times(r, &*rationals::ONE)
                == std::cmp::Ordering::Equal
        {
            // Exact sqrt endpoint, e.g. sqrt(2)/2 scaled to magnitude one.
            crate::trace_dispatch!("real", "atanh", "sqrt-endpoint-infinity");
            return Err(Problem::Infinity);
        }
        if let Sqrt(r) = &self.class
            && self
                .rational
                .compare_magnitude_squared_times(r, &*rationals::ONE)
                == std::cmp::Ordering::Greater
        {
            // Exact sqrt domain failure avoids an approximation sign query.
            crate::trace_dispatch!("real", "atanh", "sqrt-domain-error");
            return Err(Problem::NotANumber);
        }
        if matches!(&self.class, Sqrt(_)) {
            // In-domain sqrt inputs stay on the computable atanh path so the
            // factored radical can still be recognized by lower constructors.
            crate::trace_dispatch!("real", "atanh", "sqrt-computable");
            return Ok(self.make_computable(Computable::atanh));
        }
        crate::trace_dispatch!("real", "atanh", "generic-log-ratio-rewrite");
        let one = Self::one();
        let numerator = one.clone() + self.clone();
        let denominator = one - self;
        Ok((numerator / denominator)?.ln()? * constants::half())
    }

    fn recursive_powi(base: &Real, exp: &BigUint) -> Self {
        // Fallback for sign-unknown integer powers: repeated squaring is cheaper and more
        // exact than forcing ln/exp through a value whose sign cannot be certified.
        let mut result = Self::one();
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
                    computable: Some(value.ln().multiply(exp).exp()),
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
                        computable: Some(value.ln().multiply(exp).exp().negate()),
                        signal: None,
                    })
                } else {
                    Ok(Self {
                        rational: Rational::one(),
                        class: Irrational,
                        computable: Some(value.ln().multiply(exp).exp()),
                        signal: None,
                    })
                }
            }
        }
    }

    /// Raise this Real to some integer exponent.
    pub fn powi(self, exp: BigInt) -> Result<Self, Problem> {
        if exp == *signed::ONE {
            crate::trace_dispatch!("real", "powi", "exponent-one");
            return Ok(self);
        }
        if exp.sign() == Sign::NoSign {
            if self.definitely_zero() {
                crate::trace_dispatch!("real", "powi", "zero-to-zero-domain-error");
                return Err(Problem::NotANumber);
            } else {
                crate::trace_dispatch!("real", "powi", "exponent-zero-one");
                return Ok(Self::one());
            }
        }
        if exp.sign() == Sign::Minus && self.definitely_zero() {
            crate::trace_dispatch!("real", "powi", "zero-negative-exponent-domain-error");
            return Err(Problem::NotANumber);
        }
        if let Ok(rational) = self.rational.clone().powi(exp.clone()) {
            match &self.class {
                One => {
                    // Pure rationals stay exact under integer powers.
                    crate::trace_dispatch!("real", "powi", "rational-exact");
                    return Ok(Self {
                        rational,
                        class: One,
                        computable: None,
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
                        crate::trace_dispatch!("real", "powi", "sqrt-odd-special-form");
                        return Ok(n);
                    } else {
                        crate::trace_dispatch!("real", "powi", "sqrt-even-rational");
                        return Ok(Self::new(product));
                    }
                }
                _ => {
                    if let Some(computable) =
                        Self::compute_exp_ln_powi(self.computable_clone(), exp.clone())
                    {
                        // Reuse the exact rational scale while moving the irrational part
                        // to the cheaper exp(ln(x)*k) representation.
                        crate::trace_dispatch!("real", "powi", "irrational-exp-ln");
                        return Ok(Self {
                            rational,
                            class: Irrational,
                            computable: Some(computable),
                            signal: None,
                        });
                    }
                }
            }
        }
        crate::trace_dispatch!("real", "powi", "fallback-exp-ln-or-repeated-square");
        self.exp_ln_powi(exp)
    }

    /// Fractional (Non-integer) rational exponent.
    fn pow_fraction(self, exponent: Rational) -> Result<Self, Problem> {
        if exponent.denominator() == unsigned::TWO.deref() {
            // Half-integer powers are common enough to route through powi + sqrt, which
            // exposes exact-square simplifications.
            let n = exponent.shifted_big_integer(1);
            crate::trace_dispatch!("real", "pow", "half-integer-powi-sqrt");
            self.powi(n)?.sqrt()
        } else {
            crate::trace_dispatch!("real", "pow", "fractional-arbitrary");
            self.pow_arb(Real::new(exponent))
        }
    }

    /// Arbitrary, possibly irrational exponent.
    /// NB: Assumed not to be integer
    fn pow_arb(self, exponent: Self) -> Result<Self, Problem> {
        match self.best_sign() {
            Sign::NoSign => {
                if exponent.best_sign() == Sign::Plus {
                    crate::trace_dispatch!("real", "pow", "zero-positive-exponent");
                    Ok(Real::zero())
                } else {
                    crate::trace_dispatch!("real", "pow", "zero-nonpositive-domain-error");
                    Err(Problem::NotAnInteger)
                }
            }
            Sign::Minus => {
                crate::trace_dispatch!("real", "pow", "negative-arbitrary-domain-error");
                Err(Problem::NotAnInteger)
            }
            Sign::Plus => {
                let value = self.fold();
                let exp = exponent.fold();

                crate::trace_dispatch!("real", "pow", "positive-exp-ln");
                Ok(Self {
                    rational: Rational::one(),
                    class: Irrational,
                    computable: Some(value.ln().multiply(exp).exp()),
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
            if self.rational.is_one() {
                // e^x with unit scale is just exp(x), preserving the symbolic exp path.
                crate::trace_dispatch!("real", "pow", "e-base-exp");
                return exponent.exp();
            } else {
                // (a*e)^x = a^x * e^x keeps the e^x part symbolic.
                let left = Real::new(self.rational).pow(exponent.clone())?;
                crate::trace_dispatch!("real", "pow", "scaled-e-base-split");
                return Ok(left * exponent.exp()?);
            }
        }
        /* could handle self == 10 =>  10 ^ log10(exponent) specially */
        if exponent.class == One {
            let r = exponent.rational;
            if r.is_integer() {
                if let Some(n) = r.to_integer_i64() {
                    // Small integer exponents are a structural fact, not an
                    // approximation. Dispatching them before materializing the
                    // full BigInt avoids cloning arbitrary-precision storage on
                    // the common pow(x, 2/3/17) path while preserving exact
                    // repeated-squaring semantics; see Boehm et al.,
                    // "Exact Real Arithmetic: A Case Study in Higher Order
                    // Programming" (1986) on keeping exact symbolic structure
                    // ahead of numeric refinement.
                    crate::trace_dispatch!("real", "pow", "small-integer-exponent");
                    return self.powi(BigInt::from(n));
                }
                if let Some(n) = r.to_big_integer() {
                    crate::trace_dispatch!("real", "pow", "integer-exponent");
                    return self.powi(n);
                }
            }
            crate::trace_dispatch!("real", "pow", "rational-exponent");
            return self.pow_fraction(r);
        }
        if exponent.definitely_zero() {
            crate::trace_dispatch!("real", "pow", "zero-exponent");
            return self.powi(BigInt::ZERO);
        }
        crate::trace_dispatch!("real", "pow", "arbitrary-exponent");
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

fn structural_cmp_from_ordering(ordering: std::cmp::Ordering) -> StructuralComparison {
    match ordering {
        std::cmp::Ordering::Less => StructuralComparison::Less,
        std::cmp::Ordering::Equal => StructuralComparison::Equal,
        std::cmp::Ordering::Greater => StructuralComparison::Greater,
    }
}

fn domain_from_sign_nonnegative(sign: Option<RealSign>) -> DomainStatus {
    match sign {
        Some(RealSign::Positive | RealSign::Zero) => DomainStatus::Valid,
        Some(RealSign::Negative) => DomainStatus::Invalid,
        None => DomainStatus::Unknown,
    }
}

fn domain_from_sign_positive(sign: Option<RealSign>) -> DomainStatus {
    match sign {
        Some(RealSign::Positive) => DomainStatus::Valid,
        Some(RealSign::Negative | RealSign::Zero) => DomainStatus::Invalid,
        None => DomainStatus::Unknown,
    }
}

fn domain_abs_cmp_one(comparison: StructuralComparison, closed: bool) -> DomainStatus {
    match (comparison, closed) {
        (StructuralComparison::Less, _) => DomainStatus::Valid,
        (StructuralComparison::Equal, true) => DomainStatus::Valid,
        (StructuralComparison::Equal, false) | (StructuralComparison::Greater, _) => {
            DomainStatus::Invalid
        }
        (StructuralComparison::Unknown, _) => DomainStatus::Unknown,
    }
}

fn domain_cmp_one_ge(comparison: StructuralComparison) -> DomainStatus {
    match comparison {
        StructuralComparison::Equal | StructuralComparison::Greater => DomainStatus::Valid,
        StructuralComparison::Less => DomainStatus::Invalid,
        StructuralComparison::Unknown => DomainStatus::Unknown,
    }
}

#[inline]
fn primitive_facts_from_base(facts: &RealStructuralFacts) -> PrimitiveFacts {
    if facts.zero == ZeroKnowledge::Zero {
        return PrimitiveFacts {
            f32: PrimitiveFloatStatus::Zero,
            f64: PrimitiveFloatStatus::Zero,
        };
    }
    let Some(magnitude) = facts.magnitude else {
        return PrimitiveFacts {
            f32: PrimitiveFloatStatus::Unknown,
            f64: PrimitiveFloatStatus::Unknown,
        };
    };
    if !magnitude.exact_msd {
        return PrimitiveFacts {
            f32: PrimitiveFloatStatus::Unknown,
            f64: PrimitiveFloatStatus::Unknown,
        };
    }

    PrimitiveFacts {
        f32: primitive_float_status_from_msd(magnitude.msd, -150, -126, 127),
        f64: primitive_float_status_from_msd(magnitude.msd, -1075, -1022, 1023),
    }
}

#[inline]
fn primitive_float_status_from_msd(
    msd: i32,
    underflow_floor: i32,
    normal_floor: i32,
    overflow_ceiling: i32,
) -> PrimitiveFloatStatus {
    if msd < underflow_floor {
        PrimitiveFloatStatus::SubnormalOrUnderflows
    } else if msd > overflow_ceiling {
        PrimitiveFloatStatus::Overflows
    } else if msd < normal_floor {
        PrimitiveFloatStatus::SubnormalOrUnderflows
    } else {
        PrimitiveFloatStatus::NormalFinite
    }
}

fn structural_kind_for_class(class: &Class) -> StructuralKind {
    match class {
        One => StructuralKind::ExactRational,
        Pi | PiPow(_) | PiInv => StructuralKind::PiLike,
        Exp(_) | PiExp(_) | PiInvExp(_) => StructuralKind::ExpLike,
        Sqrt(_) | PiSqrt(_) => StructuralKind::SqrtLike,
        Ln(_) | LnAffine(_) | LnProduct(_) | Log10(_) => StructuralKind::LogLike,
        SinPi(_) | TanPi(_) => StructuralKind::TrigExact,
        ConstProduct(_) | ConstOffset(_) | ConstProductSqrt(_) => StructuralKind::ProductConstant,
        Irrational => StructuralKind::ComputableOpaque,
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
            computable: None,
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
            computable: Some(computable),
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
            computable: Some(computable),
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
        if self.class == Pi && self.rational.is_one() && other.class == One {
            if other.rational == *rationals::THREE {
                crate::trace_dispatch!("real", "sub", "cached-pi-minus-three");
                return constants::pi_minus_three();
            }
        }
        if self.class == One && self.rational == *rationals::THREE && other.class == Pi {
            if other.rational.is_one() {
                crate::trace_dispatch!("real", "sub", "cached-three-minus-pi");
                return -constants::pi_minus_three();
            }
        }
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
            computable: Some(computable),
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
                computable: None,
                signal: None,
            }
        } else if matches!(
            (x.to_integer_i64(), y.to_integer_i64()),
            (Some(2), Some(3)) | (Some(3), Some(2))
        ) {
            // sqrt(2)*sqrt(3) is common enough in trig-derived matrices to keep
            // as sqrt(6) without running the general square-extraction code.
            // The small-integer test is structural and allocation-light; the
            // general path still handles arbitrary radicands exactly when this
            // cheap certificate does not apply.
            Self {
                rational: Rational::one(),
                class: Sqrt(rationals::SIX.clone()),
                computable: Some(Computable::sqrt_rational(rationals::SIX.clone())),
                signal: None,
            }
        } else {
            let product = x * y;
            if product == *rationals::ZERO {
                return Self {
                    rational: product,
                    class: One,
                    computable: None,
                    signal: None,
                };
            }
            let (a, b) = product.extract_square_reduced();
            if b.is_one() {
                // The product contains a full square, so return only the exact
                // rational factor and keep subsequent sign/equality checks cheap.
                return Self {
                    rational: a,
                    class: One,
                    computable: None,
                    signal: None,
                };
            }
            Self {
                rational: a,
                class: Sqrt(b.clone()),
                computable: Some(Computable::sqrt_rational(b)),
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
            return other.scaled_by_rational(&self.rational);
        }
        if other.class == One {
            return self.scaled_by_rational(&other.rational);
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
                    computable: Some(computable),
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
                    computable: Some(computable),
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
                        computable: Some(Computable::multiply(
                            self.computable_clone(),
                            other.computable_clone(),
                        )),
                        signal: None,
                    };
                };
                let (class, computable) = Class::make_pi_power(power);
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable: Some(computable),
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
                        computable: Some(Computable::multiply(
                            self.computable_clone(),
                            other.computable_clone(),
                        )),
                        signal: None,
                    };
                };
                let (class, computable) = Class::make_pi_power(power);
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable: Some(computable),
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
                    computable: Some(computable),
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
                    computable: Some(computable),
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
                    computable: Some(computable),
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
                    computable: Some(computable),
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
                        computable: Some(Computable::multiply(
                            self.computable_clone(),
                            other.computable_clone(),
                        )),
                        signal: None,
                    };
                };
                let (class, computable) =
                    Class::make_const_product(pi_power, product.exp_power.clone());
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable: Some(computable),
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
                        computable: Some(Computable::multiply(
                            self.computable_clone(),
                            other.computable_clone(),
                        )),
                        signal: None,
                    };
                };
                let (class, computable) =
                    Class::make_const_product(pi_power, product.exp_power.clone());
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable: Some(computable),
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
                        computable: Some(Computable::multiply(
                            self.computable_clone(),
                            other.computable_clone(),
                        )),
                        signal: None,
                    };
                };
                let (class, computable) =
                    Class::make_const_product(pi_power, left.exp_power.clone() + &right.exp_power);
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable: Some(computable),
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
                    computable: Some(computable),
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
                    computable: Some(computable),
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
                    computable: Some(computable),
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
                    computable: Some(computable),
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
                    computable: Some(computable),
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
                    computable: Some(Computable::pi()),
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
                    computable: Some(computable),
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
                                        computable: Some(computable),
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
                                        computable: Some(computable),
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
                                computable: Some(computable),
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
                                computable: Some(computable),
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
                        computable: Some(computable),
                        signal: None,
                    };
                }
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class: Irrational,
                    computable: Some(Computable::multiply(
                        self.computable_clone(),
                        other.computable_clone(),
                    )),
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
            crate::trace_dispatch!("real", "div", "div-by-zero");
            return Err(Problem::DivideByZero);
        }
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "div", "zero");
            return Ok(Real::zero());
        }
        if self.class == other.class {
            crate::trace_dispatch!("real", "div", "same-class");
            let rational = &self.rational / &other.rational;
            return Ok(Real::new(rational));
        }
        if other.class == One {
            crate::trace_dispatch!("real", "div", "rhs-one");
            let rational = &self.rational / &other.rational;
            if self.class == One {
                crate::trace_dispatch!("real", "div", "rhs-one-class-one");
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
                crate::trace_dispatch!("real", "div", "pow-over-pi");
                let (class, computable) = Class::make_pi_power(power - 1);
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class,
                    computable: Some(computable),
                    signal: None,
                });
            }
            (ConstProduct(product), Exp(exp)) => {
                crate::trace_dispatch!("real", "div", "const-product-over-exp");
                let (class, computable) =
                    Class::make_const_product(product.pi_power, &product.exp_power - exp);
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class,
                    computable: Some(computable),
                    signal: None,
                });
            }
            (ConstProduct(product), Pi) if product.pi_power > 0 => {
                crate::trace_dispatch!("real", "div", "const-product-over-pi");
                let (class, computable) =
                    Class::make_const_product(product.pi_power - 1, product.exp_power.clone());
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class,
                    computable: Some(computable),
                    signal: None,
                });
            }
            (PiExp(exp), Exp(divisor_exp)) => {
                crate::trace_dispatch!("real", "div", "pi-exp-over-exp");
                let (class, computable) = Class::make_pi_exp(exp - divisor_exp);
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class,
                    computable: Some(computable),
                    signal: None,
                });
            }
            (PiExp(exp), Pi) => {
                crate::trace_dispatch!("real", "div", "pi-exp-over-pi");
                let (class, computable) = Class::make_exp(exp.clone());
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class,
                    computable: Some(computable),
                    signal: None,
                });
            }
            _ => {}
        }
        if let (Sqrt(left), Sqrt(right)) = (&self.class, &other.class)
            && let Some(right_integer) = right.integer_magnitude()
        {
            if self.rational.is_one()
                && other.rational.is_one()
                && left == &*rationals::TWO
                && right == &*rationals::THREE
            {
                crate::trace_dispatch!("real", "div", "cached-sqrt-six-over-three");
                return Ok(constants::sqrt_six_over_three());
            }
            // Rationalize sqrt(a)/sqrt(b) as sqrt(a*b)/b when b is an integer.
            // This keeps simple radical quotients exact instead of using
            // `other.inverse()` and losing the radicand certificate.
            let square = Real::multiply_sqrts(left, right);
            let denominator = if other.rational.is_one() {
                // Unit-scaled denominator radicals should not pay a rational
                // multiply/gcd just to form `1*b`; keep only the structural
                // radicand denominator. This is the same normalization-avoidance
                // principle used for exact geometric predicates in Yap (1997).
                Rational::from_unsigned_integer(right_integer.clone())
            } else {
                &other.rational * Rational::from_unsigned_integer(right_integer.clone())
            };
            return Ok(Real {
                rational: &square.rational * &self.rational / denominator,
                ..square
            });
        }
        if self.class.has_const_product_sqrt_factor() || other.class.has_const_product_sqrt_factor()
        {
            crate::trace_dispatch!("real", "div", "const-product-sqrt");
            if let (Some((left_pi, left_exp, left_rad)), Some((right_pi, right_exp, right_rad))) = (
                self.class.const_product_sqrt_parts(),
                other.class.const_product_sqrt_parts(),
            ) {
                if let Some(pi_power) = left_pi.checked_sub(right_pi) {
                    // Rationalize sqrt-heavy quotients before falling back to `other.inverse()`.
                    // This keeps `(pi*e*sqrt(2))/(e*sqrt(3))` as one factored sqrt
                    // product instead of an opaque division graph.
                    let square = Real::multiply_sqrts(&left_rad, &right_rad);
                    let denominator = if other.rational.is_one() {
                        // Preserve the factored sqrt quotient while skipping
                        // exact multiplication by one. Avoiding this gcd matters
                        // in matrix/vector scalar paths that divide by cached
                        // unit-scaled symbolic constants; see Yap (1997).
                        right_rad.clone()
                    } else {
                        &other.rational * right_rad
                    };
                    let rational = &square.rational * &self.rational / denominator;
                    let exp_power = left_exp - right_exp;
                    return Ok(match square.class {
                        One => {
                            let (class, computable) =
                                Class::make_const_product(pi_power, exp_power);
                            Real {
                                rational,
                                class,
                                computable: Some(computable),
                                signal: None,
                            }
                        }
                        Sqrt(radicand) => {
                            let (class, computable) =
                                Class::make_const_product_sqrt(pi_power, exp_power, radicand);
                            Real {
                                rational,
                                class,
                                computable: Some(computable),
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
                if self.rational.is_one()
                    && other.rational.is_one()
                    && sqrt_pi == 1
                    && sqrt_exp == *rationals::ONE
                    && radicand == *rationals::TWO
                    && product_pi == 0
                    && product_exp == *rationals::ONE
                {
                    crate::trace_dispatch!("real", "div", "cached-pi-sqrt-two");
                    return Ok(constants::pi_sqrt_two());
                }
                if let Some(pi_power) = sqrt_pi.checked_sub(product_pi) {
                    // Divide out only the pi/e product and leave the sqrt factor
                    // intact for later exact radical products.
                    let (class, computable) =
                        Class::make_const_product_sqrt(pi_power, sqrt_exp - product_exp, radicand);
                    return Ok(Real {
                        rational: &self.rational / &other.rational,
                        class,
                        computable: Some(computable),
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
                    let denominator = if other.rational.is_one() {
                        // The denominator is just the exact radicand for
                        // unit-scaled sqrt factors. Bypassing `1 * r` preserves
                        // the delayed-canonicalization invariant from Yap
                        // (1997) and keeps hot quotient paths flatter.
                        radicand.clone()
                    } else {
                        &other.rational * radicand.clone()
                    };
                    let rational = &self.rational / denominator;
                    let (class, computable) =
                        Class::make_const_product_sqrt(pi_power, product_exp - sqrt_exp, radicand);
                    return Ok(Real {
                        rational,
                        class,
                        computable: Some(computable),
                        signal: None,
                    });
                }
            }
        }
        if let Some((class, computable)) = Class::divide_const_products(&self.class, &other.class) {
            // Keep the signed pi^n * e^q quotient after const-product-sqrt
            // simplification. This avoids unnecessary sqrt factor
            // decomposition for cases where radical structure can be preserved.
            crate::trace_dispatch!("real", "div", "const-products");
            return Ok(Real {
                rational: &self.rational / &other.rational,
                class,
                computable: Some(computable),
                signal: None,
            });
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
                    let computable = self
                        .computable_clone()
                        .multiply(ln10.computable_clone().inverse());
                    return Ok(Real {
                        rational,
                        class: Log10(r.clone()),
                        computable: Some(computable),
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
