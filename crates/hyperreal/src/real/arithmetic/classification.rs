#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(crate) struct ConstProductClass {
    // Signed pi power lets reciprocal products such as 1/pi and e^q/pi remain
    // symbolic instead of falling into a generic inverse node.
    pi_power: i16,
    exp_power: Rational,
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(crate) struct ConstOffsetClass {
    // Invariant: the inner value pi^n*e^q + offset is constructed only when
    // cheaply certified positive. The outer Real.rational carries any sign.
    pi_power: i16,
    exp_power: Rational,
    offset: Rational,
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(crate) struct ConstProductSqrtClass {
    // Factored sqrt products are positive internally. Keeping the sqrt separate
    // allows later multiplication/division to cancel it exactly.
    pi_power: i16,
    exp_power: Rational,
    radicand: Rational,
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(crate) struct LnAffineClass {
    // Constructed only for positive offset + ln(base). This preserves the
    // nonzero/sign invariant shared by all non-Irrational classes.
    offset: Rational,
    base: Rational,
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(crate) struct LnProductClass {
    // Bases are sorted at construction so products compare and combine without
    // considering operand order.
    left: Rational,
    right: Rational,
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
    Log2(Rational),                 // Rational > 1 and never a power of two
    SinPi(Rational),                // 0 < Rational < 1/2 also never 1/6 or 1/4 or 1/3
    TanPi(Rational),                // 0 < Rational < 1/2 also never 1/6 or 1/4 or 1/3
    Irrational,
}

use Class::*;
#[derive(Clone, Copy, Debug, Default)]
pub(super) enum PrimitiveApproxCache {
    #[default]
    Empty,
    #[cfg(feature = "cached-f32-approx")]
    F32(Option<f32>),
    #[cfg(feature = "cached-f64-approx")]
    F64(Option<f64>),
}

/// Lock-free cache for lossy primitive conversions.
///
/// Exact-real values never convert to NaN, so reserved quiet-NaN bit patterns
/// can represent the empty and overflow states without increasing `Real`'s
/// layout. A tagged negative-NaN range carries cached finite `f32` values;
/// every other finite bit pattern is a cached `f64`.
pub(super) struct AtomicPrimitiveApproxCache(std::sync::atomic::AtomicU64);

impl AtomicPrimitiveApproxCache {
    const EMPTY: u64 = 0x7ff8_0000_0000_0001;
    #[cfg(feature = "cached-f32-approx")]
    const F32_NONE: u64 = 0x7ff8_0000_0000_0002;
    #[cfg(feature = "cached-f64-approx")]
    const F64_NONE: u64 = 0x7ff8_0000_0000_0003;
    #[cfg(feature = "cached-f32-approx")]
    const F32_TAG: u64 = 0xffff_ffff_0000_0000;

    pub(super) fn new(value: PrimitiveApproxCache) -> Self {
        Self(std::sync::atomic::AtomicU64::new(Self::encode(value)))
    }

    pub(super) fn get(&self) -> PrimitiveApproxCache {
        Self::decode(self.0.load(std::sync::atomic::Ordering::Relaxed))
    }

    pub(super) fn set(&self, value: PrimitiveApproxCache) {
        let encoded = Self::encode(value);
        if matches!(value, PrimitiveApproxCache::Empty) {
            self.0
                .store(encoded, std::sync::atomic::Ordering::Relaxed);
            return;
        }

        // Keep a concurrently-computed f64 in preference to f32. Conversion
        // caches are accelerators only, so relaxed ordering is sufficient.
        let mut current = self.0.load(std::sync::atomic::Ordering::Relaxed);
        while Self::rank(current) <= Self::rank(encoded) {
            match self.0.compare_exchange_weak(
                current,
                encoded,
                std::sync::atomic::Ordering::Relaxed,
                std::sync::atomic::Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(observed) => current = observed,
            }
        }
    }

    fn encode(value: PrimitiveApproxCache) -> u64 {
        match value {
            PrimitiveApproxCache::Empty => Self::EMPTY,
            #[cfg(feature = "cached-f32-approx")]
            PrimitiveApproxCache::F32(Some(value)) => Self::F32_TAG | u64::from(value.to_bits()),
            #[cfg(feature = "cached-f32-approx")]
            PrimitiveApproxCache::F32(None) => Self::F32_NONE,
            #[cfg(feature = "cached-f64-approx")]
            PrimitiveApproxCache::F64(Some(value)) => value.to_bits(),
            #[cfg(feature = "cached-f64-approx")]
            PrimitiveApproxCache::F64(None) => Self::F64_NONE,
        }
    }

    fn decode(value: u64) -> PrimitiveApproxCache {
        if value == Self::EMPTY {
            return PrimitiveApproxCache::Empty;
        }
        #[cfg(feature = "cached-f32-approx")]
        {
            if value == Self::F32_NONE {
                return PrimitiveApproxCache::F32(None);
            }
            if value & 0xffff_ffff_0000_0000 == Self::F32_TAG {
                return PrimitiveApproxCache::F32(Some(f32::from_bits(value as u32)));
            }
        }
        #[cfg(feature = "cached-f64-approx")]
        {
            if value == Self::F64_NONE {
                return PrimitiveApproxCache::F64(None);
            }
            return PrimitiveApproxCache::F64(Some(f64::from_bits(value)));
        }
        #[allow(unreachable_code)]
        PrimitiveApproxCache::Empty
    }

    fn rank(value: u64) -> u8 {
        match Self::decode(value) {
            PrimitiveApproxCache::Empty => 0,
            #[cfg(feature = "cached-f32-approx")]
            PrimitiveApproxCache::F32(_) => 1,
            #[cfg(feature = "cached-f64-approx")]
            PrimitiveApproxCache::F64(_) => 2,
        }
    }
}

impl Default for AtomicPrimitiveApproxCache {
    fn default() -> Self {
        Self::new(PrimitiveApproxCache::Empty)
    }
}

impl std::fmt::Debug for AtomicPrimitiveApproxCache {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.get().fmt(formatter)
    }
}

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
            (Log2(r), Log2(s)) => r == s,
            (SinPi(r), SinPi(s)) => r == s,
            (TanPi(r), TanPi(s)) => r == s,
            (_, _) => false,
        }
    }
}

impl Class {
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
        // Normalize back to the smaller dedicated variants whenever possible.
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
        // in multiplication/division reductions without growing more arms.
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
        // and `pi*sqrt(2)` stay on their direct constructors above; those
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
        // the tight scalar arms.
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
        // keep exact reduction in front of numerical refinement.
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
            Sqrt(radicand) => Computable::sqrt_squarefree_rational(radicand.clone()),
            PiSqrt(radicand) => {
                Computable::pi()
                    .multiply(Computable::sqrt_squarefree_rational(radicand.clone()))
            }
            ConstProductSqrt(product) => {
                let constant =
                    Self::make_const_product(product.pi_power, product.exp_power.clone()).1;
                constant.multiply(Computable::sqrt_squarefree_rational(
                    product.radicand.clone(),
                ))
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
                Self::ln_computable(base).multiply(Self::ln_computable(&rationals::TEN).inverse())
            }
            Log2(base) => {
                Self::ln_computable(base).multiply(Self::ln_computable(&rationals::TWO).inverse())
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
