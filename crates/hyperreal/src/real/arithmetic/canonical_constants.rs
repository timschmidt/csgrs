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
    use super::{Class, ConstOffsetClass, LnAffineClass, PrimitiveApproxCache};
    use crate::{Computable, Rational, Real};
    use std::cell::Cell;
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
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static TAU: Real = Real {
            rational: Rational::new(2),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static PI_OVER_TWO: Real = Real {
            rational: Rational::fraction(1, 2).unwrap(),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static PI_OVER_THREE: Real = Real {
            rational: Rational::fraction(1, 3).unwrap(),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static PI_OVER_FOUR: Real = Real {
            rational: Rational::fraction(1, 4).unwrap(),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static PI_OVER_SIX: Real = Real {
            rational: Rational::fraction(1, 6).unwrap(),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static HALF: Real = Real::new(Rational::fraction(1, 2).unwrap());
        static SQRT_TWO_OVER_TWO: Real = Real {
            rational: Rational::fraction(1, 2).unwrap(),
            class: Class::Sqrt(Rational::new(2)),
            computable: Some(Computable::sqrt_constant(2).unwrap()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static SQRT_THREE_OVER_TWO: Real = Real {
            rational: Rational::fraction(1, 2).unwrap(),
            class: Class::Sqrt(Rational::new(3)),
            computable: Some(Computable::sqrt_constant(3).unwrap()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static SQRT_THREE: Real = Real {
            rational: Rational::one(),
            class: Class::Sqrt(Rational::new(3)),
            computable: Some(Computable::sqrt_constant(3).unwrap()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static SQRT_THREE_OVER_THREE: Real = Real {
            rational: Rational::fraction(1, 3).unwrap(),
            class: Class::Sqrt(Rational::new(3)),
            computable: Some(Computable::sqrt_constant(3).unwrap()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static SQRT_SIX_OVER_THREE: Real = Real {
            rational: Rational::fraction(1, 3).unwrap(),
            class: Class::Sqrt(Rational::new(6)),
            computable: Some(Computable::sqrt_rational(Rational::new(6))),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static PI_SQRT_TWO: Real = Real {
            rational: Rational::one(),
            class: Class::PiSqrt(Rational::new(2)),
            computable: Some(Computable::multiply(
                Computable::pi(),
                Computable::sqrt_constant(2).unwrap(),
            )),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static LN2: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(2)),
            computable: Some(Computable::ln_constant(2).unwrap()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static LN3: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(3)),
            computable: Some(Computable::ln_constant(3).unwrap()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static HALF_LN3: Real = Real {
            rational: Rational::fraction(1, 2).unwrap(),
            class: Class::Ln(Rational::new(3)),
            computable: Some(Computable::ln_constant(3).unwrap()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static LN5: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(5)),
            computable: Some(Computable::ln_constant(5).unwrap()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static LN6: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(6)),
            computable: Some(Computable::ln_constant(6).unwrap()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static LN7: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(7)),
            computable: Some(Computable::ln_constant(7).unwrap()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static LN10: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(10)),
            computable: Some(Computable::ln_constant(10).unwrap()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        };
        static E: Real = Real {
            rational: Rational::one(),
            class: Class::Exp(Rational::one()),
            computable: Some(Computable::e_constant()),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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
                primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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

