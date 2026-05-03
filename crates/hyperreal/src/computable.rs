use crate::Rational;
use crate::computable::approximation::Approximation;
use core::cmp::Ordering;
use num::{BigInt, BigUint, bigint::Sign};
use num::{One, Zero};
use num::Signed;
use serde::{Deserialize, Serialize};
use std::{
    cell::RefCell,
    ops::{Deref, Neg},
};

mod approximation;
mod format;

pub type Precision = i32;

#[derive(Clone, Debug, PartialEq, Default)]
enum Cache {
    #[default]
    Invalid,
    Valid((Precision, BigInt)),
}

#[derive(Clone, Debug, PartialEq, Default)]
enum BoundCache {
    #[default]
    Invalid,
    Valid(BoundInfo),
}

#[derive(Clone, Debug, PartialEq, Default)]
enum ExactSignCache {
    #[default]
    Invalid,
    Valid(Sign),
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum BoundInfo {
    Unknown,
    Zero,
    NonZero {
        sign: Option<Sign>,
        msd: Option<Precision>,
        exact_msd: bool,
    },
}

impl BoundInfo {
    fn with_sign(sign: Sign, msd: Option<Precision>) -> Self {
        Self::with_sign_msd(sign, msd, true)
    }

    fn with_sign_msd(sign: Sign, msd: Option<Precision>, exact_msd: bool) -> Self {
        match sign {
            Sign::NoSign => Self::Zero,
            _ => Self::NonZero {
                sign: Some(sign),
                msd,
                exact_msd,
            },
        }
    }

    fn from_rational(r: &Rational) -> Self {
        match r.msd_exact() {
            Some(msd) => Self::with_sign(r.sign(), Some(msd)),
            None => Self::Zero,
        }
    }

    fn map_msd(self, f: impl FnOnce(Precision) -> Precision) -> Self {
        match self {
            Self::NonZero {
                sign,
                msd,
                exact_msd,
            } => Self::NonZero {
                sign,
                msd: msd.map(f),
                exact_msd,
            },
            other => other,
        }
    }

    fn negate(self) -> Self {
        match self {
            Self::NonZero {
                sign: Some(Sign::Plus),
                msd,
                exact_msd,
            } => Self::NonZero {
                sign: Some(Sign::Minus),
                msd,
                exact_msd,
            },
            Self::NonZero {
                sign: Some(Sign::Minus),
                msd,
                exact_msd,
            } => Self::NonZero {
                sign: Some(Sign::Plus),
                msd,
                exact_msd,
            },
            other => other,
        }
    }

    fn inverse(self) -> Self {
        match self {
            Self::NonZero { sign, msd, .. } => Self::NonZero {
                sign,
                msd: msd.map(|value| 1 - value),
                exact_msd: false,
            },
            other => other,
        }
    }

    fn square(self) -> Self {
        match self {
            Self::Zero => Self::Zero,
            Self::NonZero { msd, .. } => Self::NonZero {
                sign: Some(Sign::Plus),
                msd: msd.map(|value| value * 2),
                exact_msd: false,
            },
            Self::Unknown => Self::Unknown,
        }
    }

    fn sqrt(self) -> Self {
        match self {
            Self::Zero => Self::Zero,
            Self::NonZero {
                sign: Some(Sign::Plus),
                msd,
                ..
            } => Self::NonZero {
                sign: Some(Sign::Plus),
                msd: msd.map(|value| value / 2),
                exact_msd: false,
            },
            _ => Self::Unknown,
        }
    }

    fn multiply(self, other: Self) -> Self {
        match (self, other) {
            (Self::Zero, _) | (_, Self::Zero) => Self::Zero,
            (
                Self::NonZero {
                    sign: left_sign,
                    msd: left_msd,
                    ..
                },
                Self::NonZero {
                    sign: right_sign,
                    msd: right_msd,
                    ..
                },
            ) => {
                let sign = match (left_sign, right_sign) {
                    (Some(Sign::Plus), Some(Sign::Plus))
                    | (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Plus),
                    (Some(Sign::Plus), Some(Sign::Minus))
                    | (Some(Sign::Minus), Some(Sign::Plus)) => Some(Sign::Minus),
                    _ => None,
                };
                let msd = match (left_msd, right_msd) {
                    (Some(left), Some(right)) => Some(left + right),
                    _ => None,
                };
                Self::NonZero {
                    sign,
                    msd,
                    exact_msd: false,
                }
            }
            _ => Self::Unknown,
        }
    }

    fn add(self, other: Self) -> Self {
        match (self, other) {
            (Self::Zero, other) | (other, Self::Zero) => other,
            (
                Self::NonZero {
                    sign: left_sign,
                    msd: left_msd,
                    ..
                },
                Self::NonZero {
                    sign: right_sign,
                    msd: right_msd,
                    ..
                },
            ) => {
                let sign = match (left_sign.clone(), right_sign.clone()) {
                    (Some(left), Some(right)) if left == right => Some(left),
                    (Some(Sign::Plus), Some(Sign::Minus)) | (Some(Sign::Minus), Some(Sign::Plus)) => {
                        match (left_msd, right_msd) {
                            (Some(left), Some(right)) if left > right + 1 => left_sign,
                            (Some(left), Some(right)) if right > left + 1 => right_sign,
                            _ => None,
                        }
                    }
                    _ => None,
                };
                let msd = match (left_sign, right_sign, left_msd, right_msd) {
                    (_, _, Some(left), Some(right)) if left > right => Some(left),
                    (_, _, Some(left), Some(right)) if right > left => Some(right),
                    (Some(left_sign), Some(right_sign), Some(left), Some(right))
                        if left_sign != right_sign && left > right + 1 =>
                    {
                        Some(left)
                    }
                    (Some(left_sign), Some(right_sign), Some(left), Some(right))
                        if left_sign != right_sign && right > left + 1 =>
                    {
                        Some(right)
                    }
                    _ => None,
                };
                match sign {
                    Some(sign) => Self::NonZero {
                        sign: Some(sign),
                        msd,
                        exact_msd: false,
                    },
                    None if msd.is_some() => Self::NonZero {
                        sign: None,
                        msd,
                        exact_msd: false,
                    },
                    None => Self::Unknown,
                }
            }
            _ => Self::Unknown,
        }
    }

    fn known_msd(&self) -> Option<Option<Precision>> {
        match self {
            Self::Unknown => None,
            Self::Zero => Some(None),
            Self::NonZero {
                msd,
                exact_msd: true,
                ..
            } => Some(*msd),
            Self::NonZero { .. } => None,
        }
    }

    fn planning_msd(&self) -> Option<Option<Precision>> {
        match self {
            Self::Unknown => None,
            Self::Zero => Some(None),
            Self::NonZero { msd, .. } => Some(*msd),
        }
    }

    fn known_sign(&self) -> Option<Sign> {
        match self {
            Self::Zero => Some(Sign::NoSign),
            Self::NonZero { sign, .. } => sign.clone(),
            Self::Unknown => None,
        }
    }
}

fn negate_sign(sign: Sign) -> Sign {
    match sign {
        Sign::Plus => Sign::Minus,
        Sign::Minus => Sign::Plus,
        Sign::NoSign => Sign::NoSign,
    }
}

use std::sync::Arc;
use std::sync::atomic::AtomicBool;

pub type Signal = Arc<AtomicBool>;

fn should_stop(signal: &Option<Signal>) -> bool {
    use std::sync::atomic::Ordering::*;
    signal.as_ref().is_some_and(|s| s.load(Relaxed))
}

/// Computable approximation of a Real number.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Computable {
    internal: Box<Approximation>,
    #[serde(skip)]
    cache: RefCell<Cache>,
    #[serde(skip)]
    bound: RefCell<BoundCache>,
    #[serde(skip)]
    exact_sign: RefCell<ExactSignCache>,
    #[serde(skip)]
    signal: Option<Signal>,
}

mod signed {
    use num::One;
    use num::{BigInt, bigint::ToBigInt};
    use std::sync::LazyLock;

    pub(super) static MINUS_ONE: LazyLock<BigInt> =
        LazyLock::new(|| ToBigInt::to_bigint(&-1).unwrap());
    pub(super) static ONE: LazyLock<BigInt> = LazyLock::new(BigInt::one);
    pub(super) static TWO: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&2).unwrap());
    pub(super) static THREE: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&3).unwrap());
    pub(super) static FOUR: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&4).unwrap());
    pub(super) static FIVE: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&5).unwrap());
    pub(super) static SIX: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&6).unwrap());
    pub(super) static SEVEN: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&7).unwrap());
    pub(super) static EIGHT: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&8).unwrap());
    pub(super) static TWENTY_FOUR: LazyLock<BigInt> =
        LazyLock::new(|| ToBigInt::to_bigint(&24).unwrap());
    pub(super) static SIXTY_FOUR: LazyLock<BigInt> =
        LazyLock::new(|| ToBigInt::to_bigint(&64).unwrap());
    pub(super) static TWO_THREE_NINE: LazyLock<BigInt> =
        LazyLock::new(|| ToBigInt::to_bigint(&239).unwrap());
}

mod unsigned {
    use num::One;
    use num::{BigUint, bigint::ToBigUint};
    use std::sync::LazyLock;

    pub(super) static ONE: LazyLock<BigUint> = LazyLock::new(BigUint::one);
    pub(super) static TWO: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&2).unwrap());
    pub(super) static TEN: LazyLock<BigUint> =
        LazyLock::new(|| ToBigUint::to_biguint(&10).unwrap());
    pub(super) static FIVE: LazyLock<BigUint> =
        LazyLock::new(|| ToBigUint::to_biguint(&5).unwrap());
    pub(super) static SIX: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&6).unwrap());
}

impl Computable {
    /// Exactly one.
    pub fn one() -> Computable {
        Self {
            internal: Box::new(Approximation::Int(BigInt::one())),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    /// Approximate π, the ratio of a circle's circumference to its diameter.
    pub fn pi() -> Computable {
        let atan5 = Self::prescaled_atan(signed::FIVE.clone());
        let atan_239 = Self::prescaled_atan(signed::TWO_THREE_NINE.clone());
        let four = Self::integer(signed::FOUR.clone());
        let four_atan5 = Self::multiply(four, atan5);
        let neg = Self::negate(atan_239);
        let sum = Self::add(four_atan5, neg);
        let four = Self::integer(signed::FOUR.clone());
        Self::multiply(four, sum)
    }

    /// Any Rational.
    pub fn rational(r: Rational) -> Computable {
        Self {
            internal: Box::new(Approximation::Ratio(r)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }
}

impl Computable {
    pub(crate) fn e(r: Rational) -> Self {
        let rational = Self::rational(r);
        Self::exp(rational)
    }

    fn cached_bound(&self) -> Option<BoundInfo> {
        let bound = self.bound.borrow();
        match &*bound {
            BoundCache::Invalid => None,
            BoundCache::Valid(info) => Some(info.clone()),
        }
    }

    fn store_bound(&self, info: &BoundInfo) {
        if *info != BoundInfo::Unknown {
            self.bound.replace(BoundCache::Valid(info.clone()));
        }
    }

    fn bound_from_approx(prec: Precision, appr: &BigInt) -> BoundInfo {
        if appr.sign() == Sign::NoSign {
            BoundInfo::Zero
        } else {
            BoundInfo::with_sign(
                appr.sign(),
                Some(prec + appr.magnitude().bits() as Precision - 1),
            )
        }
    }

    fn cheap_bound_shallow(&self, budget: usize) -> Option<BoundInfo> {
        if let Some(info) = self.cached_bound() {
            return Some(info);
        }
        if budget == 0 {
            return None;
        }
        let info = match &*self.internal {
            Approximation::Int(n) => Some(if n.sign() == Sign::NoSign {
                BoundInfo::Zero
            } else {
                BoundInfo::with_sign(n.sign(), Some(n.magnitude().bits() as Precision - 1))
            }),
            Approximation::Ratio(r) => Some(BoundInfo::from_rational(r)),
            Approximation::Negate(child) => {
                child.cheap_bound_shallow(budget - 1).map(BoundInfo::negate)
            }
            Approximation::Offset(child, n) => child
                .cheap_bound_shallow(budget - 1)
                .map(|bound| bound.map_msd(|value| value + *n)),
            Approximation::Inverse(child) => {
                child.cheap_bound_shallow(budget - 1).map(BoundInfo::inverse)
            }
            Approximation::Square(child) => {
                child.cheap_bound_shallow(budget - 1).map(BoundInfo::square)
            }
            Approximation::Sqrt(child) => {
                child.cheap_bound_shallow(budget - 1).map(BoundInfo::sqrt)
            }
            Approximation::Multiply(left, right) => {
                let left = left.cheap_bound_shallow(budget - 1)?;
                let right = right.cheap_bound_shallow(budget - 1)?;
                Some(left.multiply(right))
            }
            Approximation::Add(left, right) => {
                let left = left.cheap_bound_shallow(budget - 1)?;
                let right = right.cheap_bound_shallow(budget - 1)?;
                Some(left.add(right))
            }
            _ => Some(if let Some((prec, appr)) = self.cached() {
                Self::bound_from_approx(prec, &appr)
            } else {
                BoundInfo::Unknown
            }),
        };
        if let Some(ref value) = info {
            self.store_bound(value);
        }
        info
    }

    fn cheap_bound(&self) -> BoundInfo {
        const SHALLOW_BOUND_BUDGET: usize = 24;

        if let Some(info) = self.cached_bound() {
            return info;
        }

        if let Some(bound) = self.cheap_bound_shallow(SHALLOW_BOUND_BUDGET) {
            return bound;
        }

        enum Frame<'a> {
            Eval(&'a Computable),
            FinishNegate,
            FinishOffset(i32),
            FinishInverse,
            FinishSquare,
            FinishSqrt,
            FinishAdd,
            FinishMultiply,
        }

        fn direct_bound(node: &Computable) -> Option<BoundInfo> {
            match &*node.internal {
                Approximation::Int(n) => Some(if n.sign() == Sign::NoSign {
                    BoundInfo::Zero
                } else {
                    BoundInfo::with_sign(n.sign(), Some(n.magnitude().bits() as Precision - 1))
                }),
                Approximation::Ratio(r) => Some(BoundInfo::from_rational(r)),
                Approximation::Negate(_)
                | Approximation::Offset(_, _)
                | Approximation::Inverse(_)
                | Approximation::Square(_)
                | Approximation::Sqrt(_)
                | Approximation::Add(_, _)
                | Approximation::Multiply(_, _) => None,
                _ => Some(if let Some((prec, appr)) = node.cached() {
                    Computable::bound_from_approx(prec, &appr)
                } else {
                    BoundInfo::Unknown
                }),
            }
        }

        let mut frames = vec![Frame::Eval(self)];
        let mut values: Vec<BoundInfo> = Vec::new();

        while let Some(frame) = frames.pop() {
            match frame {
                Frame::Eval(node) => {
                    if let Some(bound) = direct_bound(node) {
                        values.push(bound);
                        continue;
                    }

                    match &*node.internal {
                        Approximation::Negate(child) => {
                            frames.push(Frame::FinishNegate);
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Offset(child, n) => {
                            frames.push(Frame::FinishOffset(*n));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Inverse(child) => {
                            frames.push(Frame::FinishInverse);
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Square(child) => {
                            frames.push(Frame::FinishSquare);
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Sqrt(child) => {
                            frames.push(Frame::FinishSqrt);
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Add(left, right) => {
                            frames.push(Frame::FinishAdd);
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        Approximation::Multiply(left, right) => {
                            frames.push(Frame::FinishMultiply);
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        _ => unreachable!("direct_bound should handle non-structural nodes"),
                    }
                }
                Frame::FinishNegate => {
                    let value = values.pop().expect("negate bound should exist");
                    values.push(value.negate());
                }
                Frame::FinishOffset(offset) => {
                    let value = values.pop().expect("offset bound should exist");
                    values.push(value.map_msd(|msd| msd + offset));
                }
                Frame::FinishInverse => {
                    let value = values.pop().expect("inverse bound should exist");
                    values.push(value.inverse());
                }
                Frame::FinishSquare => {
                    let value = values.pop().expect("square bound should exist");
                    values.push(value.square());
                }
                Frame::FinishSqrt => {
                    let value = values.pop().expect("sqrt bound should exist");
                    values.push(value.sqrt());
                }
                Frame::FinishAdd => {
                    let right = values.pop().expect("add rhs bound should exist");
                    let left = values.pop().expect("add lhs bound should exist");
                    values.push(left.add(right));
                }
                Frame::FinishMultiply => {
                    let right = values.pop().expect("multiply rhs bound should exist");
                    let left = values.pop().expect("multiply lhs bound should exist");
                    values.push(left.multiply(right));
                }
            }
        }

        let result = values.pop().expect("bound evaluation should produce a result");
        self.store_bound(&result);
        result
    }

    fn exact_sign(&self) -> Option<Sign> {
        {
            let cache = self.exact_sign.borrow();
            if let ExactSignCache::Valid(sign) = &*cache {
                return Some(*sign);
            }
        }

        enum Frame<'a> {
            Eval(&'a Computable),
            FinishNegate(&'a Computable),
            FinishOffset(&'a Computable),
            FinishInverse(&'a Computable),
            FinishSquare(&'a Computable),
            FinishSqrt(&'a Computable),
            FinishAdd(&'a Computable),
            FinishMultiply(&'a Computable),
        }

        fn cached_exact_sign(node: &Computable) -> Option<Sign> {
            let cache = node.exact_sign.borrow();
            match &*cache {
                ExactSignCache::Invalid => None,
                ExactSignCache::Valid(sign) => Some(*sign),
            }
        }

        fn exact_sign_direct(node: &Computable) -> Option<Option<Sign>> {
            if let Some(sign) = cached_exact_sign(node) {
                return Some(Some(sign));
            }

            if let Some((_, appr)) = node.cached() {
                if appr.abs() > BigInt::one() {
                    return Some(Some(appr.sign()));
                }
            }

            match &*node.internal {
                Approximation::Int(n) => Some(Some(n.sign())),
                Approximation::Ratio(r) => Some(Some(r.sign())),
                Approximation::IntegralAtan(n) => Some(Some(n.sign())),
                Approximation::Negate(_)
                | Approximation::Offset(_, _)
                | Approximation::Inverse(_)
                | Approximation::Square(_)
                | Approximation::Sqrt(_)
                | Approximation::Add(_, _)
                | Approximation::Multiply(_, _) => None,
                _ => Some(None),
            }
        }

        let mut frames = vec![Frame::Eval(self)];
        let mut values: Vec<Option<Sign>> = Vec::new();

        while let Some(frame) = frames.pop() {
            match frame {
                Frame::Eval(node) => {
                    if let Some(sign) = exact_sign_direct(node) {
                        if let Some(sign) = sign {
                            node.exact_sign.replace(ExactSignCache::Valid(sign));
                        }
                        values.push(sign);
                        continue;
                    }

                    match &*node.internal {
                        Approximation::Negate(child) => {
                            frames.push(Frame::FinishNegate(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Offset(child, _) => {
                            frames.push(Frame::FinishOffset(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Inverse(child) => {
                            frames.push(Frame::FinishInverse(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Square(child) => {
                            frames.push(Frame::FinishSquare(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Sqrt(child) => {
                            frames.push(Frame::FinishSqrt(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Add(left, right) => {
                            frames.push(Frame::FinishAdd(node));
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        Approximation::Multiply(left, right) => {
                            frames.push(Frame::FinishMultiply(node));
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        _ => unreachable!("exact_sign_direct should handle non-structural nodes"),
                    }
                }
                Frame::FinishNegate(node) => {
                    let value = values.pop().expect("negate sign should exist");
                    let result = value.map(negate_sign);
                    if let Some(sign) = result {
                        node.exact_sign.replace(ExactSignCache::Valid(sign));
                    }
                    values.push(result);
                }
                Frame::FinishOffset(node) => {
                    let value = values.pop().expect("offset sign should exist");
                    if let Some(sign) = value {
                        node.exact_sign.replace(ExactSignCache::Valid(sign));
                    }
                    values.push(value);
                }
                Frame::FinishInverse(node) => {
                    let value = values.pop().expect("inverse sign should exist");
                    let result = match value {
                        Some(Sign::Plus) => Some(Sign::Plus),
                        Some(Sign::Minus) => Some(Sign::Minus),
                        _ => None,
                    };
                    if let Some(sign) = result {
                        node.exact_sign.replace(ExactSignCache::Valid(sign));
                    }
                    values.push(result);
                }
                Frame::FinishSquare(node) => {
                    let value = values.pop().expect("square sign should exist");
                    let result = match value {
                        Some(Sign::NoSign) => Some(Sign::NoSign),
                        Some(_) => Some(Sign::Plus),
                        None => None,
                    };
                    if let Some(sign) = result {
                        node.exact_sign.replace(ExactSignCache::Valid(sign));
                    }
                    values.push(result);
                }
                Frame::FinishSqrt(node) => {
                    let value = values.pop().expect("sqrt sign should exist");
                    let result = match value {
                        Some(Sign::NoSign) => Some(Sign::NoSign),
                        Some(Sign::Plus) => Some(Sign::Plus),
                        _ => None,
                    };
                    if let Some(sign) = result {
                        node.exact_sign.replace(ExactSignCache::Valid(sign));
                    }
                    values.push(result);
                }
                Frame::FinishAdd(node) => {
                    let right = values.pop().expect("add rhs sign should exist");
                    let left = values.pop().expect("add lhs sign should exist");
                    let result = match (left, right) {
                        (Some(Sign::NoSign), sign) | (sign, Some(Sign::NoSign)) => sign,
                        (Some(Sign::Plus), Some(Sign::Plus)) => Some(Sign::Plus),
                        (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Minus),
                        _ => None,
                    };
                    if let Some(sign) = result {
                        node.exact_sign.replace(ExactSignCache::Valid(sign));
                    }
                    values.push(result);
                }
                Frame::FinishMultiply(node) => {
                    let right = values.pop().expect("multiply rhs sign should exist");
                    let left = values.pop().expect("multiply lhs sign should exist");
                    let result = match (left, right) {
                        (Some(Sign::NoSign), _) | (_, Some(Sign::NoSign)) => Some(Sign::NoSign),
                        (Some(Sign::Plus), Some(Sign::Plus))
                        | (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Plus),
                        (Some(Sign::Plus), Some(Sign::Minus))
                        | (Some(Sign::Minus), Some(Sign::Plus)) => Some(Sign::Minus),
                        _ => None,
                    };
                    if let Some(sign) = result {
                        node.exact_sign.replace(ExactSignCache::Valid(sign));
                    }
                    values.push(result);
                }
            }
        }

        let result = values
            .pop()
            .expect("exact sign evaluation should produce a result");
        if let Some(sign) = result {
            self.exact_sign.replace(ExactSignCache::Valid(sign));
        }
        result
    }


    pub(super) fn planning_msd(&self) -> Option<Option<Precision>> {
        self.cheap_bound().planning_msd()
    }

    pub(super) fn planning_sign_and_msd(&self) -> (Option<Sign>, Option<Option<Precision>>) {
        let bound = self.cheap_bound();
        (bound.known_sign(), bound.planning_msd())
    }

    fn exact_rational(&self) -> Option<Rational> {
        match &*self.internal {
            Approximation::Int(n) => Some(Rational::from_bigint(n.clone())),
            Approximation::Ratio(r) => Some(r.clone()),
            _ => None,
        }
    }

    fn integer_ratio_nearest(&self, divisor: Computable) -> BigInt {
        let quotient = self.clone().multiply(divisor.inverse());
        scale(quotient.approx(-4), -4)
    }

    /// Natural Exponential function, raise Euler's Number to this number.
    pub fn exp(self) -> Computable {
        if self.exact_rational().is_some_and(|r| r.sign() == Sign::NoSign) {
            return Self::one();
        }
        let low_prec: Precision = -4;
        let rough_appr: BigInt = self.approx(low_prec);
        // At precision -4, an approximation outside +/-8 implies |x| > 0.5.
        if rough_appr > *signed::EIGHT || rough_appr < -signed::EIGHT.clone() {
            let ln2 = Self::ln2();
            let mut multiple = self.integer_ratio_nearest(ln2.clone());

            loop {
                let adjustment =
                    ln2.clone().multiply(Self::rational(Rational::from_bigint(multiple.clone())).negate());
                let reduced = self.clone().add(adjustment);
                let reduced_appr = reduced.approx(low_prec);

                if reduced_appr > *signed::EIGHT {
                    multiple += 1;
                    continue;
                }
                if reduced_appr < -signed::EIGHT.clone() {
                    multiple -= 1;
                    continue;
                }

                return Self {
                    internal: Box::new(Approximation::PrescaledExp(reduced)),
                    cache: RefCell::new(Cache::Invalid),
                    bound: RefCell::new(BoundCache::Invalid),
                    exact_sign: RefCell::new(ExactSignCache::Invalid),
                    signal: None,
                }
                .shift_left(multiple.try_into().expect("binary exponent should fit in i32"));
            }
        }

        Self {
            internal: Box::new(Approximation::PrescaledExp(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    /// Calculate nearby multiple of pi.
    fn pi_multiple(&self) -> BigInt {
        let mut multiple = self.integer_ratio_nearest(Self::pi());
        let adjustment =
            Self::pi().multiply(Self::rational(Rational::from_bigint(multiple.clone())).negate());
        let rough_appr = self.clone().add(adjustment).approx(-1);

        if rough_appr >= *signed::SIX {
            multiple += 1;
        } else if rough_appr <= -signed::SIX.clone() {
            multiple -= 1;
        }

        multiple
    }

    /// Calculate nearby multiple of pi/2.
    fn half_pi_multiple(&self) -> BigInt {
        let half_pi = Self::pi().shift_right(1);
        let mut multiple = self.integer_ratio_nearest(half_pi.clone());
        let adjustment =
            half_pi.multiply(Self::rational(Rational::from_bigint(multiple.clone())).negate());
        let rough_appr = self.clone().add(adjustment).approx(-1);

        if rough_appr >= *signed::TWO {
            multiple += 1;
        } else if rough_appr <= -signed::TWO.clone() {
            multiple -= 1;
        }

        multiple
    }

    fn medium_half_pi_multiple(rough_appr: &BigInt) -> BigInt {
        let positive = rough_appr.sign() != Sign::Minus;
        let magnitude = rough_appr.magnitude();
        let multiple = if magnitude < unsigned::FIVE.deref() {
            signed::ONE.clone()
        } else {
            signed::TWO.clone()
        };

        if positive { multiple } else { -multiple }
    }

    /// Cosine of this number.
    pub fn cos(self) -> Computable {
        if self.exact_rational().is_some_and(|r| r.sign() == Sign::NoSign) {
            return Self::one();
        }
        let rough_appr = self.approx(-1);
        let abs_rough_appr = rough_appr.magnitude();

        if abs_rough_appr >= unsigned::SIX.deref() {
            let multiplier = Self::pi_multiple(&self);
            let low_bit = multiplier.bit(0);

            let adjustment =
                Self::pi().multiply(Self::rational(Rational::from_bigint(multiplier)).negate());
            if low_bit {
                self.add(adjustment).cos().negate()
            } else {
                self.add(adjustment).cos()
            }
        } else if abs_rough_appr >= unsigned::TWO.deref() {
            // Scale further with double angle formula
            let cos_half = self.shift_right(1).cos();
            cos_half.square().shift_left(1).add(Self::one().negate())
        } else {
            Self {
                internal: Box::new(Approximation::PrescaledCos(self)),
                cache: RefCell::new(Cache::Invalid),
                bound: RefCell::new(BoundCache::Invalid),
                exact_sign: RefCell::new(ExactSignCache::Invalid),
                signal: None,
            }
        }
    }

    /// Sine of this number.
    pub fn sin(self) -> Computable {
        if self.exact_rational().is_some_and(|r| r.sign() == Sign::NoSign) {
            return Self::rational(Rational::zero());
        }
        let rough_appr = self.approx(-1);
        let abs_rough_appr = rough_appr.magnitude();

        if abs_rough_appr < unsigned::TWO.deref() {
            return Self {
                internal: Box::new(Approximation::PrescaledSin(self)),
                cache: RefCell::new(Cache::Invalid),
                bound: RefCell::new(BoundCache::Invalid),
                exact_sign: RefCell::new(ExactSignCache::Invalid),
                signal: None,
            };
        }

        if abs_rough_appr < unsigned::SIX.deref() {
            let multiplier = Self::medium_half_pi_multiple(&rough_appr);
            if multiplier == *signed::ONE {
                return Self::pi().shift_right(1).add(self.negate()).cos();
            } else if multiplier == *signed::MINUS_ONE {
                return Self::pi().shift_right(1).add(self).cos().negate();
            } else if multiplier == *signed::TWO {
                return Self::pi().add(self.negate()).sin();
            } else {
                return Self::pi().add(self).sin().negate();
            }
        }

        let multiplier = Self::half_pi_multiple(&self);
        let adjustment = Self::pi()
            .shift_right(1)
            .multiply(Self::rational(Rational::from_bigint(multiplier.clone())).negate());
        let reduced = self.add(adjustment);
        let quadrant =
            ((&multiplier % signed::FOUR.deref()) + signed::FOUR.deref()) % signed::FOUR.deref();

        if quadrant.is_zero() {
            reduced.sin()
        } else if quadrant == *signed::ONE {
            reduced.cos()
        } else if quadrant == *signed::TWO {
            reduced.sin().negate()
        } else {
            reduced.cos().negate()
        }
    }

    /// Tangent of this number.
    pub fn tan(self) -> Computable {
        if self.exact_rational().is_some_and(|r| r.sign() == Sign::NoSign) {
            return Self::rational(Rational::zero());
        }
        let rough_appr = self.approx(-1);
        if rough_appr.sign() == Sign::Minus {
            return self.negate().tan().negate();
        }

        let abs_rough_appr = rough_appr.magnitude();

        if abs_rough_appr < unsigned::TWO.deref() {
            return Self {
                internal: Box::new(Approximation::PrescaledTan(self)),
                cache: RefCell::new(Cache::Invalid),
                bound: RefCell::new(BoundCache::Invalid),
                exact_sign: RefCell::new(ExactSignCache::Invalid),
                signal: None,
            };
        }

        if abs_rough_appr < unsigned::FIVE.deref() {
            let complement = Self::pi().shift_right(1).add(self.negate());
            return Self {
                internal: Box::new(Approximation::PrescaledCot(complement)),
                cache: RefCell::new(Cache::Invalid),
                bound: RefCell::new(BoundCache::Invalid),
                exact_sign: RefCell::new(ExactSignCache::Invalid),
                signal: None,
            };
        }

        if abs_rough_appr < unsigned::SIX.deref() {
            return Self::pi().add(self.negate()).tan().negate();
        }

        let multiplier = Self::pi_multiple(&self);
        let adjustment =
            Self::pi().multiply(Self::rational(Rational::from_bigint(multiplier)).negate());
        self.add(adjustment).tan()
    }

    fn ln2() -> Self {
        let prescaled_9 = Self::rational(Rational::fraction(1, 9).unwrap()).prescaled_ln();
        let prescaled_24 = Self::rational(Rational::fraction(1, 24).unwrap()).prescaled_ln();
        let prescaled_80 = Self::rational(Rational::fraction(1, 80).unwrap()).prescaled_ln();

        let ln2_1 = Self::integer(signed::SEVEN.clone()).multiply(prescaled_9);
        let ln2_2 = Self::integer(signed::TWO.clone()).multiply(prescaled_24);
        let ln2_3 = Self::integer(signed::THREE.clone()).multiply(prescaled_80);

        ln2_1.add(ln2_2.negate()).add(ln2_3)
    }

    /// Natural logarithm of this number.
    pub fn ln(self) -> Computable {
        if self.exact_rational().is_some_and(|r| r == Rational::one()) {
            return Self::rational(Rational::zero());
        }
        if let Approximation::Ratio(r) = &*self.internal {
            if r.sign() == Sign::Plus {
                let (shift, reduced) = r.factor_two_powers();
                if shift != 0 {
                    let reduced_ln = if reduced == Rational::one() {
                        Self::integer(BigInt::zero())
                    } else {
                        Self::rational(reduced).ln()
                    };
                    let shift: BigInt = shift.into();
                    return reduced_ln.add(Self::integer(shift).multiply(Self::ln2()));
                }
            }
        }

        // Sixteenths, ie 8 == 0.5, 24 == 1.5
        let low_ln_limit = signed::EIGHT.deref();
        let high_ln_limit = signed::TWENTY_FOUR.deref();

        let low_prec = -4;
        let rough_appr = self.approx(low_prec);
        if rough_appr < BigInt::zero() {
            panic!("ArithmeticException");
        }
        if rough_appr <= *low_ln_limit {
            return self.inverse().ln().negate();
        }
        if rough_appr >= *high_ln_limit {
            // Sixteenths, ie 64 == 4.0
            let sixty_four = signed::SIXTY_FOUR.deref();

            if rough_appr <= *sixty_four {
                let quarter = self.sqrt().sqrt().ln();
                return quarter.shift_left(2);
            } else {
                let extra_bits: i32 = (rough_appr.bits() - 5).try_into().expect(
                    "Approximation should have few enough bits to fit in a 32-bit signed integer",
                );
                let scaled_result = self.shift_right(extra_bits).ln();
                let extra: BigInt = extra_bits.into();
                return scaled_result.add(Self::integer(extra).multiply(Self::ln2()));
            }
        }

        let minus_one = Self::integer(signed::MINUS_ONE.clone());
        let fraction = Self::add(self, minus_one);
        Self::prescaled_ln(fraction)
    }

    fn prescaled_ln(self) -> Self {
        Self {
            internal: Box::new(Approximation::PrescaledLn(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn sqrt_rational(r: Rational) -> Self {
        let rational = Self::rational(r);
        Self::sqrt(rational)
    }

    /// Square root of this number.
    pub fn sqrt(self) -> Computable {
        if let Approximation::Square(child) = self.internal.as_ref() {
            match child.exact_sign() {
                Some(Sign::Plus) => return child.clone(),
                Some(Sign::Minus) => return child.clone().negate(),
                Some(Sign::NoSign) => return Self::rational(Rational::zero()),
                None => {}
            }
        }
        if let Approximation::Multiply(left, right) = self.internal.as_ref() {
            let reduced = |scale: Rational, square_side: &Computable| {
                let (root, rest) = scale.extract_square_reduced();
                if rest != Rational::one() {
                    return None;
                }
                let Approximation::Square(child) = square_side.internal.as_ref() else {
                    return None;
                };
                match child.exact_sign() {
                    Some(Sign::Plus) => Some(child.clone().multiply(Self::rational(root))),
                    Some(Sign::Minus) => {
                        Some(child.clone().negate().multiply(Self::rational(root)))
                    }
                    Some(Sign::NoSign) => Some(Self::rational(Rational::zero())),
                    None => None,
                }
            };

            if let Some(scale) = left.exact_rational() {
                if let Some(value) = reduced(scale, right) {
                    return value;
                }
            }
            if let Some(scale) = right.exact_rational() {
                if let Some(value) = reduced(scale, left) {
                    return value;
                }
            }
        }
        if let Some(rational) = self.exact_rational() {
            if rational.sign() != Sign::Minus && rational.extract_square_will_succeed() {
                let (root, rest) = rational.extract_square_reduced();
                if rest == Rational::one() {
                    return Self::rational(root);
                }
            }
        }
        Self {
            internal: Box::new(Approximation::Sqrt(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn prescaled_atan(n: BigInt) -> Self {
        Self {
            internal: Box::new(Approximation::IntegralAtan(n)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    /// Negate this number.
    pub fn negate(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            return Self::rational(rational.neg());
        }
        if let Approximation::Negate(child) = self.internal.as_ref() {
            return child.clone();
        }
        Self {
            internal: Box::new(Approximation::Negate(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    /// Multiplicative inverse of this number.
    pub fn inverse(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            if let Ok(inverse) = rational.inverse() {
                return Self::rational(inverse);
            }
        }
        if let Approximation::Negate(child) = self.internal.as_ref() {
            if child.exact_sign().is_some_and(|sign| sign != Sign::NoSign) {
                return child.clone().inverse().negate();
            }
        }
        if let Approximation::Offset(child, n) = self.internal.as_ref() {
            if child.exact_sign().is_some_and(|sign| sign != Sign::NoSign) {
                return child.clone().inverse().shift_left(-n);
            }
        }
        if let Approximation::Inverse(child) = self.internal.as_ref() {
            if child.exact_sign().is_some_and(|sign| sign != Sign::NoSign) {
                return child.clone();
            }
        }
        Self {
            internal: Box::new(Approximation::Inverse(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn shift_left(self, n: i32) -> Self {
        if n == 0 {
            return self;
        }
        if let Approximation::Offset(child, inner) = self.internal.as_ref() {
            return child.clone().shift_left(inner + n);
        }
        Self {
            internal: Box::new(Approximation::Offset(self, n)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn shift_right(self, n: i32) -> Self {
        self.shift_left(-n)
    }

    /// Square of this number.
    pub fn square(self) -> Self {
        if let Some(rational) = self.exact_rational() {
            return Self::rational(rational.clone() * rational);
        }
        if let Approximation::Negate(child) = self.internal.as_ref() {
            return child.clone().square();
        }
        if let Approximation::Sqrt(child) = self.internal.as_ref() {
            match child.exact_sign() {
                Some(Sign::Plus) | Some(Sign::NoSign) => return child.clone(),
                _ => {}
            }
        }
        if let Approximation::Offset(child, n) = self.internal.as_ref() {
            return child.clone().square().shift_left(n * 2);
        }
        if let Approximation::Multiply(left, right) = &*self.internal {
            if let Some(scale) = left.exact_rational() {
                return right.clone().square().multiply(Self::rational(scale.clone() * scale));
            }
            if let Some(scale) = right.exact_rational() {
                return left.clone().square().multiply(Self::rational(scale.clone() * scale));
            }
        }
        Self {
            internal: Box::new(Approximation::Square(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    /// Multiply this number by some other number.
    pub fn multiply(self, other: Computable) -> Computable {
        let left_exact = self.exact_rational();
        let right_exact = other.exact_rational();

        if matches!(left_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign)
            || matches!(right_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign)
        {
            return Self::rational(Rational::zero());
        }
        if matches!(left_exact.as_ref(), Some(r) if *r == Rational::one()) {
            return other;
        }
        if matches!(right_exact.as_ref(), Some(r) if *r == Rational::one()) {
            return self;
        }
        if matches!(left_exact.as_ref(), Some(r) if *r == Rational::one().neg()) {
            return other.negate();
        }
        if matches!(right_exact.as_ref(), Some(r) if *r == Rational::one().neg()) {
            return self.negate();
        }
        if let Some((shift, sign)) = left_exact.as_ref().and_then(Rational::power_of_two_shift) {
            let shifted = other.shift_left(shift);
            return if sign == Sign::Minus {
                shifted.negate()
            } else {
                shifted
            };
        }
        if let Some((shift, sign)) = right_exact.as_ref().and_then(Rational::power_of_two_shift) {
            let shifted = self.shift_left(shift);
            return if sign == Sign::Minus {
                shifted.negate()
            } else {
                shifted
            };
        }
        if let (Some(left), Some(right)) = (left_exact.as_ref(), right_exact.as_ref()) {
            return Self::rational(left.clone() * right.clone());
        }
        if let Some(scale) = left_exact.as_ref() {
            if let Approximation::Multiply(inner_left, inner_right) = &*other.internal {
                if let Some(inner_scale) = inner_left.exact_rational() {
                    return inner_right
                        .clone()
                        .multiply(Self::rational(scale.clone() * inner_scale));
                }
                if let Some(inner_scale) = inner_right.exact_rational() {
                    return inner_left
                        .clone()
                        .multiply(Self::rational(scale.clone() * inner_scale));
                }
            }
        }
        if let Some(scale) = right_exact.as_ref() {
            if let Approximation::Multiply(inner_left, inner_right) = &*self.internal {
                if let Some(inner_scale) = inner_left.exact_rational() {
                    return inner_right
                        .clone()
                        .multiply(Self::rational(scale.clone() * inner_scale));
                }
                if let Some(inner_scale) = inner_right.exact_rational() {
                    return inner_left
                        .clone()
                        .multiply(Self::rational(scale.clone() * inner_scale));
                }
            }
        }
        Self {
            internal: Box::new(Approximation::Multiply(self, other)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    /// Add some other number to this number.
    #[allow(clippy::should_implement_trait)]
    pub fn add(self, other: Computable) -> Computable {
        let left_exact = self.exact_rational();
        let right_exact = other.exact_rational();

        if matches!(left_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign) {
            return other;
        }
        if matches!(right_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign) {
            return self;
        }
        if let (Some(left), Some(right)) = (left_exact, right_exact) {
            return Self::rational(left + right);
        }
        Self {
            internal: Box::new(Approximation::Add(self, other)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn integer(n: BigInt) -> Self {
        Self {
            internal: Box::new(Approximation::Int(n)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub fn abort(&mut self, s: Signal) {
        self.signal = Some(s);
    }

    /// An approximation of this Computable scaled to a specific precision
    ///
    /// The approximation is scaled (thus, a larger value for more negative p)
    /// and should be accurate to within +/- 1 at the scale provided.
    ///
    /// Example: 0.875 is between 0 and 1 with zero bits of extra precision
    /// ```
    /// use hyperreal::{Rational,Computable};
    /// use num::{Zero,One};
    /// use num::bigint::{BigInt,ToBigInt};
    /// let n = Rational::fraction(7, 8).unwrap();
    /// let comp = Computable::rational(n);
    /// assert!((BigInt::zero() ..= BigInt::one()).contains(&comp.approx(0)));
    /// ```
    ///
    /// Example: π * 2³ is a bit more than 25 but less than 26
    /// ```
    /// use hyperreal::{Rational,Computable};
    /// use num::{Zero,One};
    /// use num::bigint::{BigInt,ToBigInt};
    /// let pi = Computable::pi();
    /// let between_25_26 = (ToBigInt::to_bigint(&25).unwrap() ..= ToBigInt::to_bigint(&26).unwrap());
    /// assert!(between_25_26.contains(&pi.approx(-3)));
    /// ```
    pub fn approx(&self, p: Precision) -> BigInt {
        self.approx_signal(&self.signal, p)
    }

    /// Like `approx` but specifying an atomic abort/ stop signal.
    pub fn approx_signal(&self, signal: &Option<Signal>, p: Precision) -> BigInt {
        enum Frame<'a> {
            Eval(&'a Computable, Precision),
            FinishNegate(&'a Computable, Precision),
            FinishAdd(&'a Computable, Precision),
            FinishOffset(&'a Computable, Precision),
        }

        fn cached_at(node: &Computable, p: Precision) -> Option<BigInt> {
            let cache = node.cache.borrow();
            if let Cache::Valid((cache_prec, cache_appr)) = &*cache {
                if p >= *cache_prec {
                    return Some(scale(cache_appr.clone(), *cache_prec - p));
                }
            }
            None
        }

        if let Some(cached) = cached_at(self, p) {
            return cached;
        }

        if !matches!(
            &*self.internal,
            Approximation::Negate(_)
                | Approximation::Add(_, _)
                | Approximation::Offset(_, _)
        ) {
            let result = self.internal.approximate(signal, p);
            self.cache.replace(Cache::Valid((p, result.clone())));
            return result;
        }

        let mut frames = vec![Frame::Eval(self, p)];
        let mut values: Vec<BigInt> = Vec::new();

        while let Some(frame) = frames.pop() {
            match frame {
                Frame::Eval(node, prec) => {
                    if let Some(cached) = cached_at(node, prec) {
                        values.push(cached);
                        continue;
                    }

                    match &*node.internal {
                        Approximation::Negate(child) => {
                            frames.push(Frame::FinishNegate(node, prec));
                            frames.push(Frame::Eval(child, prec));
                        }
                        Approximation::Add(left, right) => {
                            frames.push(Frame::FinishAdd(node, prec));
                            frames.push(Frame::Eval(right, prec - 2));
                            frames.push(Frame::Eval(left, prec - 2));
                        }
                        Approximation::Offset(child, n) => {
                            frames.push(Frame::FinishOffset(node, prec));
                            frames.push(Frame::Eval(child, prec - *n));
                        }
                        _ => {
                            let result = node.internal.approximate(signal, prec);
                            node.cache.replace(Cache::Valid((prec, result.clone())));
                            values.push(result);
                        }
                    }
                }
                Frame::FinishNegate(node, prec) => {
                    let result = -values.pop().expect("negate child result should exist");
                    node.cache.replace(Cache::Valid((prec, result.clone())));
                    values.push(result);
                }
                Frame::FinishAdd(node, prec) => {
                    let right = values.pop().expect("add rhs result should exist");
                    let left = values.pop().expect("add lhs result should exist");
                    let result = scale(left + right, -2);
                    node.cache.replace(Cache::Valid((prec, result.clone())));
                    values.push(result);
                }
                Frame::FinishOffset(node, prec) => {
                    let result = values.pop().expect("offset child result should exist");
                    node.cache.replace(Cache::Valid((prec, result.clone())));
                    values.push(result);
                }
            }
        }

        values.pop().expect("evaluation should produce a result")
    }

    pub fn sign(&self) -> Sign {
        if let Some(sign) = self.exact_sign() {
            return sign;
        }
        {
            let cache = self.cache.borrow();
            if let Cache::Valid((_prec, cache_appr)) = &*cache {
                let sign = cache_appr.sign();
                if sign != Sign::NoSign {
                    self.exact_sign.replace(ExactSignCache::Valid(sign));
                    return sign;
                }
            }
        }
        let mut sign = Sign::NoSign;
        let mut p = 0;
        while p > -2000 && sign == Sign::NoSign {
            let appr = self.approx(p);
            p -= 10;
            sign = appr.sign();
        }
        if sign != Sign::NoSign {
            self.exact_sign.replace(ExactSignCache::Valid(sign));
        }
        sign
    }

    fn cached(&self) -> Option<(Precision, BigInt)> {
        let cache = self.cache.borrow();
        if let Cache::Valid((cache_prec, cache_appr)) = &*cache {
            Some((*cache_prec, cache_appr.clone()))
        } else {
            None
        }
    }

    /// Do not call this function if `self` and `other` may be the same.
    pub fn compare_to(&self, other: &Self) -> Ordering {
        if let (Some(left), Some(right)) = (self.exact_rational(), other.exact_rational()) {
            return left
                .partial_cmp(&right)
                .expect("exact rationals should be comparable");
        }
        if let (Some(left), Some(right)) = (self.exact_sign(), other.exact_sign()) {
            match (left, right) {
                (Sign::Minus, Sign::Plus | Sign::NoSign)
                | (Sign::NoSign, Sign::Plus) => return Ordering::Less,
                (Sign::Plus, Sign::Minus | Sign::NoSign)
                | (Sign::NoSign, Sign::Minus) => return Ordering::Greater,
                _ => {}
            }

            if matches!(left, Sign::Plus | Sign::Minus)
                && left == right
                && let (Some(Some(left_msd)), Some(Some(right_msd))) =
                    (self.cheap_bound().known_msd(), other.cheap_bound().known_msd())
                && left_msd != right_msd
            {
                return match left {
                    Sign::Plus => left_msd.cmp(&right_msd),
                    Sign::Minus => right_msd.cmp(&left_msd),
                    Sign::NoSign => unreachable!(),
                };
            }
        }
        let mut tolerance = -20;
        while tolerance > Precision::MIN {
            let order = self.compare_absolute(other, tolerance);
            if order != Ordering::Equal {
                return order;
            }
            tolerance *= 2;
        }
        panic!("Apparently called Computable::compare_to on equal values");
    }

    /// Compare two values to a specified tolerance (more negative numbers are more precise).
    pub fn compare_absolute(&self, other: &Self, tolerance: Precision) -> Ordering {
        if let (Some(left), Some(right)) = (self.exact_rational(), other.exact_rational()) {
            let left_abs = if left.sign() == Sign::Minus {
                left.neg()
            } else {
                left
            };
            let right_abs = if right.sign() == Sign::Minus {
                right.neg()
            } else {
                right
            };
            return left_abs
                .partial_cmp(&right_abs)
                .expect("exact rationals should be comparable");
        }
        match (self.exact_sign(), other.exact_sign()) {
            (Some(Sign::NoSign), Some(Sign::NoSign)) => return Ordering::Equal,
            (Some(Sign::NoSign), Some(_)) => return Ordering::Less,
            (Some(_), Some(Sign::NoSign)) => return Ordering::Greater,
            _ => {}
        }
        if let (Some(Some(left_msd)), Some(Some(right_msd))) =
            (self.cheap_bound().known_msd(), other.cheap_bound().known_msd())
        {
            if left_msd >= tolerance + 1 && right_msd <= tolerance - 1 {
                return Ordering::Greater;
            }
            if right_msd >= tolerance + 1 && left_msd <= tolerance - 1 {
                return Ordering::Less;
            }
        }
        let needed = tolerance - 1;
        let this = self.approx(needed);
        let alt = other.approx(needed);
        let max = alt.clone() + signed::ONE.deref();
        let min = alt.clone() - signed::ONE.deref();
        if this > max {
            Ordering::Greater
        } else if this < min {
            Ordering::Less
        } else {
            Ordering::Equal
        }
    }

    /// Most Significant Digit (Bit) ?
    /// May panic or give incorrect answers if not yet discovered.
    fn known_msd(&self) -> Precision {
        if let Some((prec, appr)) = self.cached() {
            let length = appr.magnitude().bits() as Precision;
            prec + length - 1
        } else {
            panic!("Expected valid cache state for known MSD but it's invalid")
        }
    }

    /// Most Significant Digit - or perhaps None if as yet undiscovered and less than p.
    fn msd(&self, p: Precision) -> Option<Precision> {
        if let Some(msd) = self.cheap_bound().known_msd() {
            return msd;
        }

        let cache = self.cached();
        let mut try_once = false;

        if cache.is_none() {
            try_once = true;
        } else if let Some((_prec, appr)) = cache {
            let one = signed::ONE.deref();
            let minus_one = signed::MINUS_ONE.deref();

            if appr > *minus_one && appr < *one {
                try_once = true;
            }
        }

        if try_once {
            let appr = self.approx(p - 1);
            if appr.magnitude() < &BigUint::one() {
                return None;
            }
        }

        Some(self.known_msd())
    }

    const STOP_PRECISION: Precision = Precision::MIN / 3;

    /// MSD iteratively: 0, -16, -40, -76 etc. or p if that's lower.
    /// You can choose p to avoid unnecessary work.
    pub(super) fn iter_msd_stop(&self, p: Precision) -> Option<Precision> {
        let mut prec = 0;

        loop {
            let msd = self.msd(prec);
            if msd.is_some() {
                return msd;
            }
            prec = (prec * 3) / 2 - 16;
            if prec <= p {
                break;
            }
            if should_stop(&self.signal) {
                break;
            }
        }
        self.msd(p)
    }

    /// MSD but iteratively without a guess as to precision.
    pub(super) fn iter_msd(&self) -> Precision {
        self.iter_msd_stop(Self::STOP_PRECISION)
            .unwrap_or(Self::STOP_PRECISION)
    }
}

fn shift(n: BigInt, p: Precision) -> BigInt {
    match 0.cmp(&p) {
        Ordering::Greater => n >> -p,
        Ordering::Equal => n,
        Ordering::Less => n << p,
    }
}

/// Scale n by p bits, rounding if this makes n smaller.
/// e.g. scale(10, 2) == 40
///      scale(10, -2) == 3
fn scale(n: BigInt, p: Precision) -> BigInt {
    if p >= 0 {
        n << p
    } else {
        let adj = shift(n, p + 1) + signed::ONE.deref();
        adj >> 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use num::bigint::BigUint;
    use num::Signed;

    #[test]
    fn compare() {
        let six: BigInt = "6".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let four: BigInt = "4".parse().unwrap();
        let six = Computable::integer(six.clone());
        let five = Computable::integer(five.clone());
        let four = Computable::integer(four.clone());

        assert_eq!(six.compare_to(&five), Ordering::Greater);
        assert_eq!(five.compare_to(&six), Ordering::Less);
        assert_eq!(four.compare_to(&six), Ordering::Less);
    }

    #[test]
    fn bigger() {
        let six: BigInt = "6".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let four: BigInt = "4".parse().unwrap();
        let a = Computable::integer(six.clone());
        let b = Computable::integer(five.clone());
        assert_eq!(a.compare_absolute(&b, 0), Ordering::Greater);
        let c = Computable::integer(four.clone());
        assert_eq!(c.compare_absolute(&a, 0), Ordering::Less);
        assert_eq!(b.compare_absolute(&b, 0), Ordering::Equal);
    }

    #[test]
    fn shifted() {
        let one = BigInt::one();
        let two = &one + &one;
        assert_eq!(one, shift(two, -1));
    }

    #[test]
    fn prec() {
        let nine: BigInt = "9".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let two: BigInt = "2".parse().unwrap();
        let one = BigInt::one();
        let a = Computable::integer(nine.clone());
        assert_eq!(nine, a.approx(0));
        assert_eq!(five, a.approx(1));
        assert_eq!(two, a.approx(2));
        assert_eq!(one, a.approx(3));
        assert_eq!(Cache::Valid((0, nine)), a.cache.into_inner());
    }

    #[test]
    fn prec_pi() {
        let three: BigInt = "3".parse().unwrap();
        let six: BigInt = "6".parse().unwrap();
        let thirteen: BigInt = "13".parse().unwrap();
        let four_zero_two: BigInt = "402".parse().unwrap();
        let a = Computable::pi();
        assert_eq!(four_zero_two, a.approx(-7));
        assert_eq!(three, a.approx(0));
        assert_eq!(six, a.approx(-1));
        assert_eq!(thirteen, a.approx(-2));
        assert_eq!(Cache::Valid((-7, four_zero_two)), a.cache.into_inner());
    }

    #[test]
    fn prec_atan_5() {
        let five: BigInt = "5".parse().unwrap();
        let atan_5 = Computable::prescaled_atan(five);
        let two_zero_two: BigInt = "202".parse().unwrap();
        assert_eq!(two_zero_two, atan_5.approx(-10));
        let at_twenty: BigInt = "206984".parse().unwrap();
        assert_eq!(at_twenty, atan_5.approx(-20));
    }

    #[test]
    fn prec_atan_239() {
        let two_three_nine: BigInt = "239".parse().unwrap();
        let atan_239 = Computable::prescaled_atan(two_three_nine);
        let four: BigInt = "4".parse().unwrap();
        assert_eq!(four, atan_239.approx(-10));
        let at_twenty: BigInt = "4387".parse().unwrap();
        assert_eq!(at_twenty, atan_239.approx(-20));
    }

    #[test]
    fn msd() {
        let one: BigInt = "1".parse().unwrap();
        let a = Computable::integer(one.clone());
        assert_eq!(Some(0), a.msd(-4));
        let three: BigInt = "3".parse().unwrap();
        let d = Computable::integer(three.clone());
        assert_eq!(Some(1), d.msd(-4));
        let five: BigInt = "5".parse().unwrap();
        let e = Computable::integer(five.clone());
        assert_eq!(Some(2), e.msd(-4));
        let seven: BigInt = "7".parse().unwrap();
        let f = Computable::integer(seven.clone());
        assert_eq!(Some(2), f.msd(-4));
        let eight: BigInt = "8".parse().unwrap();
        let g = Computable::integer(eight.clone());
        assert_eq!(Some(3), g.msd(-4));
    }

    #[test]
    fn iter_msd() {
        let one = Computable::one();
        assert_eq!(one.iter_msd(), 0);
        let pi = Computable::pi();
        assert_eq!(pi.iter_msd(), 1);
        let five = Rational::new(5);
        let e = Computable::e(five);
        assert_eq!(e.iter_msd(), 7);
    }

    #[test]
    fn negate() {
        let fifteen: BigInt = "15".parse().unwrap();
        let a = Computable::integer(fifteen.clone());
        let b = Computable::negate(a);
        let answer: BigInt = "-7".parse().unwrap();
        assert_eq!(answer, b.approx(1));
    }

    #[test]
    fn multiply() {
        let four: BigInt = "4".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(four);
        let b = Computable::prescaled_atan(five);
        let m = Computable::multiply(a, b);
        let answer: BigInt = "809".parse().unwrap();
        assert_eq!(answer, m.approx(-10));
    }

    #[test]
    fn multiply_opposite() {
        let four: BigInt = "4".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(four);
        let b = Computable::prescaled_atan(five);
        let m = Computable::multiply(b, a);
        let answer: BigInt = "809".parse().unwrap();
        assert_eq!(answer, m.approx(-10));
    }

    #[test]
    fn rational() {
        let sixth: Rational = "1/6".parse().unwrap();
        let c = Computable::rational(sixth);
        let zero = BigInt::zero();
        let one = BigInt::one();
        let ten: BigInt = "10".parse().unwrap();
        let eighty_five: BigInt = "85".parse().unwrap();
        assert_eq!(zero, c.approx(0));
        assert_eq!(zero, c.approx(-1));
        assert_eq!(zero, c.approx(-2));
        assert_eq!(one, c.approx(-3));
        assert_eq!(ten, c.approx(-6));
        assert_eq!(eighty_five, c.approx(-9));
    }

    #[test]
    fn scaled_ln1() {
        let zero = Computable::integer(BigInt::zero());
        let ln = Computable {
            internal: Box::new(Approximation::PrescaledLn(zero)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        };
        let zero = BigInt::zero();
        assert_eq!(zero, ln.approx(100));
    }

    #[test]
    fn scaled_ln1_4() {
        let zero_4: Rational = "0.4".parse().unwrap();
        let rational = Computable::rational(zero_4);
        let ln = Computable {
            internal: Box::new(Approximation::PrescaledLn(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        };
        let five: BigInt = "5".parse().unwrap();
        assert_eq!(five, ln.approx(-4));
    }

    #[test]
    fn ln() {
        let five: BigInt = "5".parse().unwrap();
        let integer = Computable::integer(five);
        let ln = Computable::ln(integer);
        let correct: BigInt = "1769595698905".parse().unwrap();
        assert_eq!(ln.approx(-40), correct);
    }

    #[test]
    fn exp_and_ln_round_trip() {
        let seven_fifths = Computable::rational(Rational::fraction(7, 5).unwrap());
        assert_close(seven_fifths.clone().exp().ln(), seven_fifths, -40, 2);
    }

    #[test]
    fn exact_transcendental_identities() {
        let zero = Computable::rational(Rational::zero());
        let one = Computable::rational(Rational::one());
        assert_close(zero.clone().exp(), one.clone(), -40, 0);
        assert_close(one.ln(), zero.clone(), -40, 0);
        assert_close(zero.clone().sin(), zero.clone(), -40, 0);
        assert_close(zero.clone().cos(), Computable::one(), -40, 0);
        assert_close(zero.tan(), Computable::rational(Rational::zero()), -40, 0);
    }

    #[test]
    fn compare_to_uses_exact_sign_and_rational_shortcuts() {
        let minus_pi = Computable::pi().negate();
        let pi = Computable::pi();
        assert_eq!(minus_pi.compare_to(&pi), Ordering::Less);

        let left = Computable::rational(Rational::fraction(7, 8).unwrap());
        let right = Computable::rational(Rational::fraction(9, 10).unwrap());
        assert_eq!(left.compare_to(&right), Ordering::Less);
    }

    #[test]
    fn compare_to_uses_exact_msd_gap_shortcut() {
        let base = Computable::pi();
        base.approx(-16);
        let huge = base
            .clone()
            .multiply(Computable::rational(Rational::from_bigint(BigInt::from(1_u8) << 200)));
        assert_eq!(huge.compare_to(&base), Ordering::Greater);
        assert_eq!(base.compare_to(&huge), Ordering::Less);

        let minus_base = base.negate();
        let minus_huge = huge.negate();
        assert_eq!(minus_huge.compare_to(&minus_base), Ordering::Less);
        assert_eq!(minus_base.compare_to(&minus_huge), Ordering::Greater);
    }

    #[test]
    fn compare_absolute_uses_exact_shortcuts() {
        let zero = Computable::rational(Rational::zero());
        let tiny = Computable::rational(Rational::fraction(1, 1024).unwrap());
        assert_eq!(zero.compare_absolute(&tiny, -40), Ordering::Less);

        let left = Computable::rational(Rational::fraction(-7, 8).unwrap());
        let right = Computable::rational(Rational::fraction(9, 10).unwrap());
        assert_eq!(left.compare_absolute(&right, -40), Ordering::Less);
    }

    #[test]
    fn compare_absolute_uses_exact_msd_gap_shortcut() {
        let base = Computable::pi();
        base.approx(-16);
        let huge = base
            .clone()
            .multiply(Computable::rational(Rational::from_bigint(BigInt::from(1_u8) << 200)));
        assert_eq!(huge.compare_absolute(&base, -40), Ordering::Greater);
        assert_eq!(base.compare_absolute(&huge, -40), Ordering::Less);
    }

    #[test]
    fn warmed_zero_sum_product_stays_zero() {
        let zero = Computable::pi().add(Computable::pi().negate());
        zero.approx(-128);
        let product = zero.multiply(Computable::pi());
        assert_eq!(product.approx(-128), BigInt::zero());
    }

    #[test]
    fn exp_negative_is_inverse() {
        let eleven_tenths = Computable::rational(Rational::fraction(11, 10).unwrap());
        let product = eleven_tenths
            .clone()
            .exp()
            .multiply(eleven_tenths.negate().exp());
        assert_close(product, Computable::one(), -40, 2);
    }

    #[test]
    fn exp_near_prescaled_limit_round_trip() {
        let half = Computable::rational(Rational::fraction(1, 2).unwrap());
        assert_close(half.clone().exp().ln(), half, -40, 2);
    }

    #[test]
    fn exp_large_argument_reduces_by_ln2() {
        let exponent = BigInt::from(200);
        let offset = Computable::rational(Rational::fraction(7, 20).unwrap());
        let value = Computable::ln2()
            .multiply(Computable::integer(exponent.clone()))
            .add(offset.clone());
        let expected = offset.exp().shift_left(200);

        assert_close(value.exp(), expected, -80, 2);
    }

    #[test]
    fn cos_zero() {
        let zero = Computable::rational(Rational::zero());
        let cos = zero.cos();
        let correct: BigInt = "4294967296".parse().unwrap();
        assert_eq!(cos.approx(-32), correct);
    }

    #[test]
    fn cos_one() {
        let one = Computable::one();
        let cos = one.cos();
        let correct: BigInt = "2320580734".parse().unwrap();
        assert_eq!(cos.approx(-32), correct);
    }

    fn assert_approx(c: Computable, p: Precision, expected: &str, max_error: i32) {
        let actual = c.approx(p);
        let expected: BigInt = expected.parse().unwrap();
        let error = (actual - expected).abs();
        let max_error = BigInt::from(max_error);
        assert!(error <= max_error);
    }

    fn assert_close(left: Computable, right: Computable, p: Precision, max_error: i32) {
        let error = (left.approx(p) - right.approx(p)).abs();
        let max_error = BigInt::from(max_error);
        assert!(error <= max_error);
    }

    fn pi_times(r: Rational) -> Computable {
        Computable::pi().multiply(Computable::rational(r))
    }

    fn shifted_cos_sin(c: Computable) -> Computable {
        pi_times(Rational::fraction(1, 2).unwrap())
            .add(c.negate())
            .cos()
    }

    #[test]
    fn sin_small_arguments() {
        let one_fifth = Computable::rational(Rational::fraction(1, 5).unwrap());
        assert_approx(one_fifth.sin(), -32, "853278278", 1);

        let zero = Computable::rational(Rational::zero());
        assert_eq!(BigInt::zero(), zero.sin().approx(-32));
    }

    #[test]
    fn sin_medium_arguments() {
        let three: BigInt = "3".parse().unwrap();
        let three = Computable::integer(three);
        assert_approx(three.sin(), -32, "606105819", 1);
    }

    #[test]
    fn sin_large_arguments() {
        let one_two_three: BigInt = "123".parse().unwrap();
        let one_two_three = Computable::integer(one_two_three);
        assert_approx(one_two_three.sin(), -32, "-1975270452", 1);
    }

    #[test]
    fn sin_negative_arguments() {
        let negative_three_fifths = Computable::rational(Rational::fraction(-3, 5).unwrap());
        assert_approx(negative_three_fifths.sin(), -32, "-2425120957", 1);
    }

    #[test]
    fn sin_near_pi_multiples() {
        let epsilon = Computable::rational(Rational::fraction(1, 64).unwrap());
        let pi_plus_epsilon = Computable::pi().add(epsilon.clone());
        let two_pi_minus_epsilon = pi_times(Rational::new(2)).add(epsilon.clone().negate());

        assert_approx(pi_plus_epsilon.sin(), -32, "-67106133", 1);
        assert_approx(two_pi_minus_epsilon.sin(), -32, "-67106133", 1);
    }

    #[test]
    fn sin_near_half_pi() {
        let epsilon = Computable::rational(Rational::fraction(1, 64).unwrap());
        let half_pi = pi_times(Rational::fraction(1, 2).unwrap());
        let half_pi_plus_epsilon = half_pi.clone().add(epsilon.clone());
        let half_pi_minus_epsilon = half_pi.add(epsilon.negate());

        assert_approx(half_pi_plus_epsilon.sin(), -32, "4294443019", 1);
        assert_approx(half_pi_minus_epsilon.sin(), -32, "4294443019", 1);
    }

    #[test]
    fn sin_matches_shifted_cos_identity() {
        for r in ["-12", "-3/5", "0", "1/5", "3", "123"] {
            let r: Rational = r.parse().unwrap();
            let c = Computable::rational(r);
            assert_close(c.clone().sin(), shifted_cos_sin(c), -40, 1);
        }

        for r in ["-7/3", "-1/2", "1/2", "2", "41/6"] {
            let r: Rational = r.parse().unwrap();
            let c = pi_times(r);
            assert_close(c.clone().sin(), shifted_cos_sin(c), -40, 1);
        }
    }

    #[test]
    fn deep_add_chain_approximates_without_recursive_walk() {
        let mut value = Computable::one();
        for _ in 0..5000 {
            value = value.add(Computable::one());
        }

        assert_eq!(value.approx(0), BigInt::from(5001));
    }

    #[test]
    fn deep_multiply_chain_of_ones_stays_exact() {
        let mut value = Computable::one();
        for _ in 0..5000 {
            value = value.multiply(Computable::one());
        }

        assert_eq!(value.approx(0), BigInt::from(1));
    }

    #[test]
    fn deep_multiply_chain_by_one_preserves_irrational() {
        let mut value = Computable::pi();
        for _ in 0..5000 {
            value = value.multiply(Computable::one());
        }

        assert_close(value, Computable::pi(), -40, 2);
    }

    #[test]
    fn rational_msd_exact_for_small_fraction() {
        let third = Computable::rational(Rational::fraction(1, 3).unwrap());
        assert_eq!(third.msd(-4), Some(-2));
    }

    #[test]
    fn multiply_combines_exact_scales() {
        let scale = Computable::rational(Rational::fraction(7, 8).unwrap());
        let combined = Computable::pi()
            .multiply(scale.clone())
            .multiply(scale.clone())
            .multiply(scale);
        let expected = Computable::pi().multiply(Computable::rational(
            Rational::fraction(343, 512).unwrap(),
        ));
        assert_close(combined, expected, -60, 2);
    }

    #[test]
    fn square_of_scaled_irrational_reuses_exact_scale() {
        let scaled =
            Computable::pi().multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        let expected = Computable::pi()
            .square()
            .multiply(Computable::rational(Rational::fraction(49, 64).unwrap()));
        assert_close(scaled.square(), expected, -60, 2);
    }

    #[test]
    fn inverse_of_exact_fraction_has_structural_bound() {
        let third = Computable::rational(Rational::fraction(1, 3).unwrap());
        let inverse = third.inverse();
        assert_eq!(inverse.sign(), Sign::Plus);
        assert_eq!(inverse.msd(-4), Some(1));
    }

    #[test]
    fn inverse_of_scaled_irrational_uses_structural_msd() {
        let scale = Rational::fraction(7, 8).unwrap();
        let base = Computable::pi();
        base.approx(-16);
        let value = base.multiply(Computable::rational(scale.clone()));
        assert_eq!(value.planning_msd(), Some(Some(0)));
        assert_eq!(value.msd(-4), Some(1));
        let inverse = value.inverse();
        let expected = Computable::pi()
            .inverse()
            .multiply(Computable::rational(scale.inverse().unwrap()));
        assert_close(inverse, expected, -60, 2);
    }

    #[test]
    fn square_of_negative_fraction_has_structural_bound() {
        let value = Computable::rational(Rational::fraction(-3, 8).unwrap()).square();
        assert_eq!(value.sign(), Sign::Plus);
        assert_eq!(value.msd(-4), Some(-3));
    }

    #[test]
    fn sqrt_of_scaled_square_tracks_structural_msd() {
        let value = Computable::pi()
            .multiply(Computable::rational(Rational::fraction(7, 8).unwrap()))
            .square()
            .sqrt();
        let expected = Computable::pi()
            .multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        assert_eq!(value.sign(), Sign::Plus);
        assert_eq!(value.msd(-4), Some(1));
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn sqrt_of_square_of_negative_value_returns_absolute_value() {
        let value = Computable::rational(Rational::fraction(-3, 8).unwrap())
            .square()
            .sqrt();
        assert_eq!(value.approx(-8), Computable::rational(Rational::fraction(3, 8).unwrap()).approx(-8));
    }

    #[test]
    fn double_negate_collapses_at_construction() {
        let value = Computable::pi().negate().negate();
        assert_close(value, Computable::pi(), -60, 2);
    }

    #[test]
    fn inverse_of_inverse_of_nonzero_value_collapses_at_construction() {
        let base = Computable::pi();
        base.approx(-16);
        let value = base.clone().inverse().inverse();
        assert_close(value, base, -60, 2);
    }

    #[test]
    fn nested_offsets_collapse_at_construction() {
        let value = Computable::pi().shift_left(5).shift_right(3);
        let expected = Computable::pi().shift_left(2);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn square_of_negative_value_collapses_to_square_of_positive_value() {
        let value = Computable::pi().negate().square();
        let expected = Computable::pi().square();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn inverse_of_negative_nonzero_value_normalizes_sign() {
        let base = Computable::pi();
        base.approx(-16);
        let value = base.clone().negate().inverse();
        let expected = base.inverse().negate();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn multiply_by_negative_one_collapses_to_negate() {
        let minus_one = Computable::rational(Rational::one().neg());
        let value = Computable::pi().multiply(minus_one);
        let expected = Computable::pi().negate();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn multiply_by_power_of_two_fraction_collapses_to_shift() {
        let value = Computable::pi().multiply(Computable::rational(Rational::fraction(1, 8).unwrap()));
        let expected = Computable::pi().shift_right(3);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn multiply_by_negative_power_of_two_fraction_collapses_to_shift_and_negate() {
        let value =
            Computable::pi().multiply(Computable::rational(Rational::fraction(-1, 8).unwrap()));
        let expected = Computable::pi().shift_right(3).negate();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn square_of_power_of_two_scaled_value_collapses_to_shifted_square() {
        let value = Computable::pi().multiply(Computable::rational(Rational::new(8))).square();
        let expected = Computable::pi().square().shift_left(6);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn sqrt_of_exactly_scaled_square_collapses_at_construction() {
        let value = Computable::pi()
            .multiply(Computable::rational(Rational::fraction(7, 8).unwrap()))
            .square()
            .sqrt();
        let expected =
            Computable::pi().multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn sqrt_of_exact_rational_square_is_exact() {
        let value = Computable::rational(Rational::fraction(49, 64).unwrap()).sqrt();
        let expected = Computable::rational(Rational::fraction(7, 8).unwrap());
        assert_close(value, expected, -60, 0);
    }

    #[test]
    fn square_of_sqrt_of_positive_value_collapses_at_construction() {
        let value = Computable::rational(Rational::new(2)).sqrt().square();
        let expected = Computable::rational(Rational::new(2));
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn inverse_of_shifted_nonzero_value_collapses_to_shifted_inverse() {
        let base = Computable::pi();
        base.approx(-16);
        let value = base.clone().shift_left(5).inverse();
        let expected = base.inverse().shift_right(5);
        assert_close(value, expected, -60, 2);
    }



    #[test]
    fn add_with_dominant_term_has_structural_bound() {
        let value = Computable::integer(BigInt::from(8))
            .add(Computable::rational(Rational::fraction(-1, 8).unwrap()));
        assert_eq!(value.sign(), Sign::Plus);
        assert_eq!(value.msd(-4), Some(2));
    }

    #[test]
    fn add_ignores_tiny_term_at_target_precision() {
        let big = Computable::pi();
        let tiny = Computable::rational(Rational::from_bigint_fraction(BigInt::from(1), BigUint::from(1_u8) << 200).unwrap());
        assert_eq!(
            big.clone().add(tiny).compare_absolute(&big, -128),
            Ordering::Equal
        );
    }

    #[test]
    fn add_does_not_ignore_tiny_opposite_sign_term() {
        let big = Computable::pi();
        let tiny = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(-1), BigUint::from(1_u8) << 200).unwrap(),
        );
        let sum = big.clone().add(tiny.clone());
        let delta = sum.add(big.negate());
        assert_eq!(delta.compare_absolute(&tiny, -180), Ordering::Equal);
    }

    #[test]
    fn deep_structural_bound_chain() {
        let scale = Computable::rational(Rational::fraction(-7, 8).unwrap());
        let mut value = Computable::pi();
        value.approx(-16);
        for _ in 0..2000 {
            value = value.multiply(scale.clone()).inverse().negate();
        }
        assert_eq!(value.sign(), Sign::Plus);
    }

    #[test]
    fn huge_trig_arguments_reduce_correctly() {
        let huge_multiple = BigInt::from(1_u8) << 200;
        let offset = Computable::rational(Rational::fraction(7, 5).unwrap());
        let huge = Computable::pi()
            .multiply(Computable::integer(huge_multiple))
            .add(offset.clone());

        assert_eq!(
            huge.clone().sin().compare_absolute(&offset.clone().sin(), -80),
            Ordering::Equal
        );
        assert_eq!(
            huge.clone().cos().compare_absolute(&offset.clone().cos(), -80),
            Ordering::Equal
        );
        assert_eq!(huge.tan().compare_absolute(&offset.tan(), -72), Ordering::Equal);
    }

    #[test]
    fn tan_small_and_medium_arguments() {
        let one_fifth = Computable::rational(Rational::fraction(1, 5).unwrap());
        assert_approx(one_fifth.tan(), -32, "870632973", 2);

        let seven_fifths = Computable::rational(Rational::fraction(7, 5).unwrap());
        assert_approx(seven_fifths.tan(), -32, "24901720944", 2);
    }

    #[test]
    fn tan_near_half_pi() {
        let epsilon = Computable::rational(Rational::fraction(1, 64).unwrap());
        let near_half_pi = pi_times(Rational::fraction(1, 2).unwrap()).add(epsilon.negate());
        assert_approx(near_half_pi.tan(), -32, "274855536959", 8);
    }

    #[test]
    fn ln_sqrt_pi() {
        let pi = Computable::pi();
        let sqrt = Computable::sqrt(pi);
        let ln = Computable::ln(sqrt);
        let correct: BigInt = "629321910077".parse().unwrap();
        assert_eq!(ln.approx(-40), correct);
    }

    #[test]
    fn ln_large_power_of_two() {
        let value = Computable::rational(Rational::new(1024));
        let ten = Computable::rational(Rational::new(10));
        assert_close(value.ln(), ten.multiply(Computable::ln2()), -40, 2);
    }

    #[test]
    fn ln_tiny_power_of_two() {
        let denominator = BigUint::from(1_u8) << 10;
        let value = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(1), denominator).unwrap(),
        );
        let ten = Computable::rational(Rational::new(10));
        assert_close(value.ln(), ten.multiply(Computable::ln2()).negate(), -40, 2);
    }

    #[test]
    fn ln_exact_binary_scaled_rational() {
        let denominator = BigUint::from(1_u8) << 10;
        let value = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(3), denominator).unwrap(),
        );
        let expected = Computable::rational(Rational::new(3))
            .ln()
            .add(Computable::rational(Rational::new(-10)).multiply(Computable::ln2()));
        assert_close(value.ln(), expected, -40, 2);
    }

    #[test]
    fn sqrt_square_round_trip() {
        let two = Computable::rational(Rational::new(2));
        let sqrt_two = two.clone().sqrt();
        assert_close(sqrt_two.square(), two, -40, 2);
    }

    #[test]
    fn ln_near_prescaled_limit_round_trip() {
        let value = Computable::rational(Rational::fraction(47, 32).unwrap());
        assert_close(value.clone().ln().exp(), value, -40, 2);
    }

    #[test]
    fn add() {
        let three: BigInt = "3".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(three);
        let b = Computable::integer(five);
        let c = Computable::add(a, b);
        let answer: BigInt = "256".parse().unwrap();
        assert_eq!(answer, c.approx(-5));
    }

    #[test]
    fn scale_up() {
        let ten: BigInt = "10".parse().unwrap();
        let three: BigInt = "3".parse().unwrap();
        assert_eq!(ten, scale(ten.clone(), 0));
        let a = scale(ten.clone(), -2);
        assert_eq!(three, a);
        let forty: BigInt = "40".parse().unwrap();
        let b = scale(ten.clone(), 2);
        assert_eq!(forty, b);
    }
}
