use crate::bounds::Bounds;
use crate::{DomainCoverage, IntervalError, RangeEnclosure, SignSet};
use num_bigint::BigInt;
use num_integer::Integer;
use num_traits::{One, Zero};

/// Caller-owned accounting for dyadic arithmetic and directed conversions.
///
/// Each stage reserves an upper bound `n` on its input/output magnitude bits.
/// Linear stages cost `n` units; product/division stages cost `n*n` units.
/// Square-root extraction reserves `2 * bit_length(n) + 4` quadratic stages,
/// with `n` the scaled quotient bit length plus two.
/// The formulas deliberately overestimate small or cancelling results. A size
/// limit bounds each reservation; a work limit bounds their cumulative cost.
/// Both limits are checked before the corresponding allocation/arithmetic.
/// These are deterministic accounting units, not CPU cycles, allocation bytes,
/// or an instruction-count proof for the BigInt implementation. Caller-owned
/// inputs, constructor normalization and Hyperreal evaluation are outside this
/// accounting. There is no cache, and a fresh budget permits an independent retry.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DyadicBudget {
    max_bits: u64,
    remaining_work: u64,
}

impl DyadicBudget {
    /// Sets the maximum reserved magnitude bits and cumulative work units.
    pub const fn new(max_bits: u64, work_units: u64) -> Self {
        Self {
            max_bits,
            remaining_work: work_units,
        }
    }

    /// Maximum magnitude-bit reservation for an individual stage.
    pub const fn max_bits(&self) -> u64 {
        self.max_bits
    }

    /// Unconsumed deterministic work units.
    pub const fn remaining_work(&self) -> u64 {
        self.remaining_work
    }

    pub(crate) fn reserve(
        &mut self,
        bits: u64,
        stages: u64,
        quadratic: bool,
    ) -> Result<(), IntervalError> {
        if bits > self.max_bits {
            return Err(IntervalError::SizeExhausted);
        }
        let cost = bits
            .max(1)
            .checked_mul(if quadratic { bits.max(1) } else { 1 })
            .and_then(|c| c.checked_mul(stages))
            .ok_or(IntervalError::WorkExhausted)?;
        self.remaining_work = self
            .remaining_work
            .checked_sub(cost)
            .ok_or(IntervalError::WorkExhausted)?;
        Ok(())
    }
}

/// An exact closed real interval `[lower, upper] * 2^exponent`, or empty.
///
/// Endpoints share a signed 32-bit exponent. Common powers of two are removed
/// up to the exponent's maximum; zero and empty have canonical exponent zero.
/// Normalization preserves the set, not the originally requested grid tag.
/// Arithmetic is exact except explicitly outward-rounded division/regridding.
/// Common-grid operations keep exact zero on the other operand's grid. Copying
/// zero still consumes work, but requires no magnitude-bit shift reservation;
/// an interval that merely contains zero receives ordinary alignment accounting.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DyadicInterval {
    bounds: Option<Bounds<BigInt>>,
    exponent: i32,
}

impl DyadicInterval {
    /// Encloses the principal square root on the grid `2^precision`.
    ///
    /// The endpoints are the tight outward grid bounds. Negative source points
    /// are excluded with `Partial` coverage; a strictly negative nonempty input
    /// returns `EmptyDomain`. Empty maps to empty with `Complete` coverage.
    /// Exact integer division, square root and a square check establish
    /// the directed bounds. Reservations precede scaling, root extraction and verification;
    /// see [`DyadicBudget`] for the accounting units and exclusions.
    pub fn sqrt(
        &self,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<RangeEnclosure<Self>, IntervalError> {
        let Some((a, b, exponent)) = self.bounds() else {
            return Ok(RangeEnclosure {
                range: Self::empty(),
                domain: DomainCoverage::Complete,
            });
        };
        let zero = BigInt::zero();
        if b < &zero {
            return Err(IntervalError::EmptyDomain);
        }
        let domain = if a < &zero {
            DomainCoverage::Partial
        } else {
            DomainCoverage::Complete
        };
        let shift = i64::from(exponent) - 2 * i64::from(precision);
        budget.reserve(1, 1, false)?;
        let denominator = BigInt::one();
        let (lower, lower_ceil) = scaled_sqrt(a.max(&zero), &denominator, shift, budget)?;
        let upper = if a == b {
            lower_ceil
        } else {
            scaled_sqrt(b, &denominator, shift, budget)?.1
        };
        Ok(RangeEnclosure {
            range: Self::new(lower, upper, precision)?,
            domain,
        })
    }

    /// Validates order and normalizes caller-owned mantissas without growing them.
    /// This ingress operation has no work budget; input parsing limits belong to
    /// the caller. Arithmetic methods require a separate explicit budget.
    pub fn new(lower: BigInt, upper: BigInt, exponent: i32) -> Result<Self, IntervalError> {
        let bounds = Bounds::new(lower, upper)?;
        let (mut lower, mut upper) = bounds.endpoints();
        if lower.is_zero() && upper.is_zero() {
            return Ok(Self {
                bounds: Some(Bounds::point(BigInt::zero())),
                exponent: 0,
            });
        }
        let shift = lower
            .trailing_zeros()
            .unwrap_or(u64::MAX)
            .min(upper.trailing_zeros().unwrap_or(u64::MAX))
            .min((i64::from(i32::MAX) - i64::from(exponent)) as u64);
        // shift is bounded by a nonzero input's already allocated bit length.
        let shift = usize::try_from(shift).map_err(|_| IntervalError::Overflow)?;
        lower >>= shift;
        upper >>= shift;
        Ok(Self {
            bounds: Some(Bounds::new(lower, upper)?),
            exponent: (i64::from(exponent) + shift as i64) as i32,
        })
    }

    /// The canonical empty set.
    pub const fn empty() -> Self {
        Self {
            bounds: None,
            exponent: 0,
        }
    }

    /// An exact singleton, normalized like `new`.
    pub fn point(mantissa: BigInt, exponent: i32) -> Result<Self, IntervalError> {
        Self::new(mantissa.clone(), mantissa, exponent)
    }

    /// Whether this set is empty.
    pub fn is_empty(&self) -> bool {
        self.bounds.is_none()
    }

    /// Whether this set contains exactly one real value.
    pub fn is_singleton(&self) -> bool {
        self.bounds
            .as_ref()
            .is_some_and(|b| b.as_refs().is_singleton())
    }

    /// Exact common-scale mantissas and exponent; `None` for empty.
    pub fn bounds(&self) -> Option<(&BigInt, &BigInt, i32)> {
        self.bounds.as_ref().map(|b| {
            let (a, b) = b.as_refs().endpoints();
            (a, b, self.exponent)
        })
    }

    /// Exact lower endpoint as mantissa and exponent, or `None` for empty.
    pub fn lower(&self) -> Option<(&BigInt, i32)> {
        self.bounds().map(|(lower, _, exponent)| (lower, exponent))
    }

    /// Exact upper endpoint as mantissa and exponent, or `None` for empty.
    pub fn upper(&self) -> Option<(&BigInt, i32)> {
        self.bounds().map(|(_, upper, exponent)| (upper, exponent))
    }

    /// Whether this set contains the exact point `mantissa * 2^exponent`.
    /// Alignment and temporary point storage use the caller's shared budget.
    pub fn contains_point(
        &self,
        mantissa: &BigInt,
        exponent: i32,
        budget: &mut DyadicBudget,
    ) -> Result<bool, IntervalError> {
        if self.is_empty() {
            return Ok(false);
        }
        budget.reserve(mantissa.bits(), 2, false)?;
        Self::point(mantissa.clone(), exponent)?.subset_of(self, budget)
    }

    /// Exact comparison of singleton values, sharing the caller's work/size
    /// budget and avoiding alignment of irrelevant exponent gaps. With sufficient
    /// budget, empty or nonsingleton operands return `NonSingletonEndpoint`.
    pub fn compare_singleton(
        &self,
        other: &Self,
        budget: &mut DyadicBudget,
    ) -> Result<core::cmp::Ordering, IntervalError> {
        let Some((a, b, e)) = self.bounds() else {
            return Err(IntervalError::NonSingletonEndpoint);
        };
        let Some((c, d, f)) = other.bounds() else {
            return Err(IntervalError::NonSingletonEndpoint);
        };
        budget.reserve(self.bits().max(other.bits()), 2, false)?;
        if a != b || c != d {
            return Err(IntervalError::NonSingletonEndpoint);
        }
        crate::dyadic_order::compare(a, e, c, f, budget)
    }

    /// Exact endpoint distance as a singleton interval; empty for empty input.
    pub fn width(&self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        self.measure(true, false, budget)
    }

    /// Exact midpoint as a singleton interval; empty for empty input.
    /// No rounding is performed. An unrepresentable half at the minimum
    /// exponent returns `Overflow` rather than choosing a nearby point.
    pub fn midpoint(&self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        self.measure(false, true, budget)
    }

    /// Partitions a nonsingleton into two closed halves meeting at its exact midpoint.
    /// Empty and singleton inputs return `None` after inspecting the stored bounds.
    /// If the midpoint needs a grid finer than `2^i32::MIN`, returns `Overflow`.
    ///
    /// Arithmetic uses at most one extra magnitude bit. The budget covers the
    /// endpoint sum, the two outer endpoints and the shared midpoint in both
    /// results.
    pub fn bisect(&self, budget: &mut DyadicBudget) -> Result<Option<[Self; 2]>, IntervalError> {
        let Some((a, b, exponent)) = self.bounds() else {
            return Ok(None);
        };
        if a == b {
            return Ok(None);
        }
        let bits = self.bits().saturating_add(1);
        budget.reserve(bits, 1, false)?;
        let sum = a + b;
        let finer = sum.is_odd();
        let exponent = exponent
            .checked_sub(i32::from(finer))
            .ok_or(IntervalError::Overflow)?;
        budget.reserve(bits, if finer { 3 } else { 4 }, false)?;
        let midpoint = if finer { sum } else { sum >> 1usize };
        let lower = a << usize::from(finer);
        let upper = b << usize::from(finer);
        // On a finer grid the midpoint is odd. On the original grid both
        // outer endpoints are odd, unless the exponent is already maximal.
        // Each child therefore already has the canonical common exponent.
        let half = |lo, hi| Self {
            bounds: Some(Bounds::ordered(lo, hi)),
            exponent,
        };
        Ok(Some([half(lower, midpoint.clone()), half(midpoint, upper)]))
    }

    /// Exact half-width as a singleton interval; empty for empty input.
    /// Like `midpoint`, this fails if the exact half cannot be represented.
    pub fn radius(&self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        self.measure(true, true, budget)
    }

    /// Exact singleton midpoint where representable. At the minimum stored
    /// exponent, an unrepresentable half-step gets its tight outward neighbors
    /// on that grid. Empty maps to empty; all work shares the caller's budget.
    pub fn midpoint_enclosure(&self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        self.half_measure_enclosure(false, budget)
    }

    /// Exact singleton half-width where representable, otherwise tight outward
    /// neighbors on the minimum stored grid. It measures the source radius,
    /// not a radius about a separately rounded midpoint.
    pub fn radius_enclosure(&self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        self.half_measure_enclosure(true, budget)
    }

    fn half_measure_enclosure(
        &self,
        difference: bool,
        budget: &mut DyadicBudget,
    ) -> Result<Self, IntervalError> {
        let Some((a, b, exponent)) = self.bounds() else {
            return Ok(Self::empty());
        };
        if exponent != i32::MIN {
            return self.measure(difference, true, budget);
        }
        budget.reserve(self.bits().saturating_add(1).max(2), 3, false)?;
        let value = if difference { b - a } else { a + b };
        let (lo, hi) = scaled_quotient(&value, &BigInt::from(2), 0, budget)?;
        Self::new(lo, hi, exponent)
    }

    fn measure(
        &self,
        difference: bool,
        half: bool,
        budget: &mut DyadicBudget,
    ) -> Result<Self, IntervalError> {
        let Some((a, b, exponent)) = self.bounds() else {
            return Ok(Self::empty());
        };
        budget.reserve(self.bits().saturating_add(1), 3, false)?;
        let value = if difference { b - a } else { a + b };
        Self::from_wide_exponent(
            value.clone(),
            value,
            i64::from(exponent) - i64::from(half),
            budget,
        )
    }

    // Normalize new exact results whose temporary exponent may exceed storage.
    // Remove only exact common factors; increasing an excessive exponent's
    // mantissas requires a separate reservation before shifting.
    fn from_wide_exponent(
        mut lower: BigInt,
        mut upper: BigInt,
        exponent: i64,
        budget: &mut DyadicBudget,
    ) -> Result<Self, IntervalError> {
        if lower.is_zero() && upper.is_zero() {
            return Self::new(lower, upper, 0);
        }
        if exponent < i64::from(i32::MIN) {
            let shift = (i64::from(i32::MIN) - exponent) as u64;
            if lower.trailing_zeros().unwrap_or(u64::MAX) < shift
                || upper.trailing_zeros().unwrap_or(u64::MAX) < shift
            {
                return Err(IntervalError::Overflow);
            }
            budget.reserve(lower.bits().max(upper.bits()), 2, false)?;
            let shift = usize::try_from(shift).map_err(|_| IntervalError::Overflow)?;
            lower >>= shift;
            upper >>= shift;
            Self::new(lower, upper, i32::MIN)
        } else if exponent > i64::from(i32::MAX) {
            let shift = (exponent - i64::from(i32::MAX)) as u64;
            budget.reserve(
                lower.bits().max(upper.bits()).saturating_add(shift),
                2,
                false,
            )?;
            let shift = usize::try_from(shift).map_err(|_| IntervalError::Overflow)?;
            Self::new(lower << shift, upper << shift, i32::MAX)
        } else {
            Self::new(lower, upper, exponent as i32)
        }
    }

    /// Signs of members, independent of the positive power-of-two scale.
    pub fn signs(&self) -> SignSet {
        self.bounds
            .as_ref()
            .map_or(SignSet::Empty, |b| b.as_refs().signs(&BigInt::zero()))
    }

    fn bits(&self) -> u64 {
        self.bounds().map_or(0, |(a, b, _)| a.bits().max(b.bits()))
    }

    // Nonempty operands share the finer grid unless one is exactly zero.
    // Canonical zero has exponent zero and is representable on every grid.
    fn common_exponent(&self, other: &Self) -> i32 {
        if self.exponent == other.exponent {
            self.exponent
        } else if self.exponent == 0 && self.bits() == 0 {
            other.exponent
        } else if other.exponent == 0 && other.bits() == 0 {
            self.exponent
        } else {
            self.exponent.min(other.exponent)
        }
    }

    // Align exactly to a no-coarser grid (zero permits any grid).
    // Reserve before cloning or shifting.
    fn align(
        &self,
        exponent: i32,
        budget: &mut DyadicBudget,
    ) -> Result<Option<Bounds<BigInt>>, IntervalError> {
        let Some((a, b, e)) = self.bounds() else {
            return Ok(None);
        };
        let bits = self.bits();
        if bits == 0 {
            budget.reserve(0, 2, false)?;
            return Ok(Some(Bounds::new(a.clone(), b.clone())?));
        }
        let shift = u64::try_from(i64::from(e) - i64::from(exponent))
            .map_err(|_| IntervalError::InvalidScale)?;
        let bits = bits
            .checked_add(shift)
            .ok_or(IntervalError::SizeExhausted)?;
        budget.reserve(bits, 2, false)?;
        let shift = usize::try_from(shift).map_err(|_| IntervalError::Overflow)?;
        Ok(Some(Bounds::new(a << shift, b << shift)?))
    }

    /// Outward enclosure on the grid `2^precision`; finer grids preserve the set.
    ///
    /// The result is normalized, so its stored exponent may be larger than
    /// `precision`. Precision specifies spacing, not a bound on interval width.
    pub fn regrid(&self, precision: i32, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        let Some((a, b, e)) = self.bounds() else {
            return Ok(Self::empty());
        };
        if precision <= e {
            let (a, b) = self
                .align(precision, budget)?
                .expect("nonempty input")
                .endpoints();
            return Self::new(a, b, precision);
        }
        budget.reserve(self.bits(), 4, false)?;
        let shift = usize::try_from(i64::from(precision) - i64::from(e))
            .map_err(|_| IntervalError::Overflow)?;
        // Arithmetic right shift is floor division. Negation gives the ceiling
        // without allocating a potentially enormous power-of-two denominator.
        Self::new(a >> shift, -((-b) >> shift), precision)
    }

    /// Exact sum on a shared grid, with bounded shift reservations.
    pub fn add(&self, other: &Self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        self.add_sub(other, budget, false)
    }

    /// Exact Minkowski difference; inputs are independent sets.
    pub fn sub(&self, other: &Self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        self.add_sub(other, budget, true)
    }

    fn add_sub(
        &self,
        other: &Self,
        budget: &mut DyadicBudget,
        subtract: bool,
    ) -> Result<Self, IntervalError> {
        if self.is_empty() || other.is_empty() {
            return Ok(Self::empty());
        }
        let exponent = self.common_exponent(other);
        let (a, b) = self
            .align(exponent, budget)?
            .expect("nonempty input")
            .endpoints();
        let (c, d) = other
            .align(exponent, budget)?
            .expect("nonempty input")
            .endpoints();
        budget.reserve(
            a.bits()
                .max(b.bits())
                .max(c.bits())
                .max(d.bits())
                .saturating_add(1),
            2,
            false,
        )?;
        if subtract {
            Self::new(a - d, b - c, exponent)
        } else {
            Self::new(a + c, b + d, exponent)
        }
    }

    /// Exact absolute-value image, with zero minimum across zero. Reserves
    /// space/work for retained mantissas before cloning or reflection.
    pub fn abs(&self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        let Some((a, b, e)) = self.bounds() else {
            return Ok(Self::empty());
        };
        budget.reserve(self.bits(), 2, false)?;
        if a >= &BigInt::zero() {
            Self::new(a.clone(), b.clone(), e)
        } else if b < &BigInt::zero() {
            Self::new(-b, -a, e)
        } else {
            Self::new(BigInt::zero(), (-a).max(b.clone()), e)
        }
    }

    /// Exact image of `min(x, y)` over independent operand members.
    /// Either empty operand gives empty without consuming budget. Alignment and
    /// result storage share the caller's size/work accounting.
    pub fn min_image(
        &self,
        other: &Self,
        budget: &mut DyadicBudget,
    ) -> Result<Self, IntervalError> {
        self.order_image(other, budget, false)
    }

    /// Exact image of `max(x, y)` over independent operand members.
    /// Either empty operand gives empty; this differs from hull.
    pub fn max_image(
        &self,
        other: &Self,
        budget: &mut DyadicBudget,
    ) -> Result<Self, IntervalError> {
        self.order_image(other, budget, true)
    }

    fn order_image(
        &self,
        other: &Self,
        budget: &mut DyadicBudget,
        maximum: bool,
    ) -> Result<Self, IntervalError> {
        if self.is_empty() || other.is_empty() {
            return Ok(Self::empty());
        }
        let exponent = self.common_exponent(other);
        let a = self.align(exponent, budget)?.expect("nonempty input");
        let b = other.align(exponent, budget)?.expect("nonempty input");
        let selected = if maximum {
            a.as_refs().max_image(b.as_refs())
        } else {
            a.as_refs().min_image(b.as_refs())
        };
        let (lo, hi) = selected.endpoints();
        budget.reserve(lo.bits().max(hi.bits()), 2, false)?;
        Self::new(lo.clone(), hi.clone(), exponent)
    }

    /// Exact reflected set.
    pub fn neg(&self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        let Some((a, b, e)) = self.bounds() else {
            return Ok(Self::empty());
        };
        budget.reserve(self.bits(), 2, false)?;
        Self::new(-b, -a, e)
    }

    /// Exact product hull. Temporary exponent overflow is normalized when the
    /// endpoints are representable, with mantissa growth reserved before shifting.
    /// Resource admission may fail before endpoint representability is decided.
    pub fn mul(&self, other: &Self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        let (Some((a, b, e)), Some((c, d, f))) = (self.bounds(), other.bounds()) else {
            return Ok(Self::empty());
        };
        if self.signs() == SignSet::Zero || other.signs() == SignSet::Zero {
            return Self::point(BigInt::zero(), 0);
        }
        let exponent = e.checked_add(f);
        budget.reserve(self.bits().saturating_add(other.bits()), 4, true)?;
        let products = [a * c, a * d, b * c, b * d];
        let lower = products.iter().min().expect("four products").clone();
        let upper = products.into_iter().max().expect("four products");
        match exponent {
            Some(exponent) => Self::new(lower, upper, exponent),
            None => Self::from_wide_exponent(lower, upper, i64::from(e) + i64::from(f), budget),
        }
    }

    /// Exact pointwise square, preserving dependence across zero.
    /// Unlike `self.mul(self)`, a zero-crossing input has lower bound zero.
    /// Temporary exponent overflow is normalized when representable, with any
    /// required mantissa growth charged before allocation.
    pub fn square(&self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        let Some((a, b, exponent)) = self.bounds() else {
            return Ok(Self::empty());
        };
        budget.reserve(self.bits().saturating_mul(2), 2, true)?;
        let aa = a * a;
        let bb = b * b;
        let (lower, upper) = if self.signs().contains_zero() {
            (BigInt::zero(), aa.max(bb))
        } else if a < &BigInt::zero() {
            (bb, aa)
        } else {
            (aa, bb)
        };
        Self::from_wide_exponent(lower, upper, i64::from(exponent) * 2, budget)
    }

    /// Exact image of a nonnegative integer power.
    ///
    /// Exponent zero maps every nonempty set to one, including zero; empty maps
    /// to empty. Odd powers preserve order and even powers retain a zero minimum
    /// when the source contains zero. For negative powers, explicitly compose
    /// with `reciprocal` at the caller's chosen output grid.
    ///
    /// Each nontrivial endpoint power reserves `input_bits * exponent` bits
    /// and at most twice the exponent's bit length in quadratic work stages
    /// before exponentiation. Zero and unit mantissas need only constant work.
    /// Exponents one and two reuse the existing copy and square paths.
    pub fn checked_pow(
        &self,
        exponent: u32,
        budget: &mut DyadicBudget,
    ) -> Result<Self, IntervalError> {
        let Some((a, b, scale)) = self.bounds() else {
            return Ok(Self::empty());
        };
        match exponent {
            0 => {
                budget.reserve(1, 2, false)?;
                return Self::point(BigInt::one(), 0);
            }
            1 => return self.regrid(scale, budget),
            2 => return self.square(budget),
            _ => {}
        }
        let aa = endpoint_power(a, exponent, budget)?;
        let bb = endpoint_power(b, exponent, budget)?;
        let (lower, upper) = if !exponent.is_multiple_of(2) {
            (aa, bb)
        } else if self.signs().contains_zero() {
            (BigInt::zero(), aa.max(bb))
        } else if a < &BigInt::zero() {
            (bb, aa)
        } else {
            (aa, bb)
        };
        // i32 * u32 fits i64, including both endpoint exponent extremes.
        Self::from_wide_exponent(lower, upper, i64::from(scale) * i64::from(exponent), budget)
    }

    /// Outward reciprocal on grid `2^precision`, using exact integer division.
    /// Empty maps to empty; a zero-containing input returns `ContainsZero`.
    pub fn reciprocal(
        &self,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<Self, IntervalError> {
        if self.signs().contains_zero() {
            return Err(IntervalError::ContainsZero);
        }
        if self.is_empty() {
            return Ok(Self::empty());
        }
        budget.reserve(1, 2, false)?;
        Self::point(BigInt::one(), 0)?.div(self, precision, budget)
    }

    /// Outward quotient hull on grid `2^precision`.
    ///
    /// Rejects a zero-containing denominator even for an empty numerator.
    /// All four rational endpoint quotients are rounded by exact integer
    /// floor/ceiling division; no approximate reciprocal is used.
    pub fn div(
        &self,
        other: &Self,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<Self, IntervalError> {
        if other.signs().contains_zero() {
            return Err(IntervalError::ContainsZero);
        }
        let (Some((a, b, e)), Some((c, d, f))) = (self.bounds(), other.bounds()) else {
            return Ok(Self::empty());
        };
        let shift = i64::from(e) - i64::from(f) - i64::from(precision);
        let mut range: Option<(BigInt, BigInt)> = None;
        for (n, d) in [(a, c), (a, d), (b, c), (b, d)] {
            let (lo, hi) = scaled_quotient(n, d, shift, budget)?;
            range = Some(match range {
                None => (lo, hi),
                Some((a, b)) => (a.min(lo), b.max(hi)),
            });
        }
        let (lower, upper) = range.expect("four quotients");
        Self::new(lower, upper, precision)
    }

    /// Exact hull, charging for common-grid alignment and retained mantissas.
    pub fn hull(&self, other: &Self, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        let exponent = if self.is_empty() {
            other.exponent
        } else if other.is_empty() {
            self.exponent
        } else {
            self.common_exponent(other)
        };
        let a = self.align(exponent, budget)?;
        let b = other.align(exponent, budget)?;
        match (a, b) {
            (Some(a), Some(b)) => {
                let (lo, hi) = a.as_refs().hull(b.as_refs()).endpoints();
                budget.reserve(lo.bits().max(hi.bits()), 2, false)?;
                Self::new(lo.clone(), hi.clone(), exponent)
            }
            (a, b) => match a.or(b) {
                Some(b) => {
                    let (a, b) = b.endpoints();
                    Self::new(a, b, exponent)
                }
                None => Ok(Self::empty()),
            },
        }
    }

    /// Exact intersection, or canonical empty when disjoint.
    pub fn intersection(
        &self,
        other: &Self,
        budget: &mut DyadicBudget,
    ) -> Result<Self, IntervalError> {
        if self.is_empty() || other.is_empty() {
            return Ok(Self::empty());
        }
        let exponent = self.common_exponent(other);
        let a = self.align(exponent, budget)?.expect("nonempty input");
        let b = other.align(exponent, budget)?.expect("nonempty input");
        match a.as_refs().intersection(b.as_refs()) {
            None => Ok(Self::empty()),
            Some(bounds) => {
                let (lo, hi) = bounds.endpoints();
                budget.reserve(lo.bits().max(hi.bits()), 2, false)?;
                Self::new(lo.clone(), hi.clone(), exponent)
            }
        }
    }

    /// Exact set inclusion, with explicit alignment accounting.
    pub fn subset_of(
        &self,
        other: &Self,
        budget: &mut DyadicBudget,
    ) -> Result<bool, IntervalError> {
        self.relation(other, budget, false, false)
    }

    /// Exact strict-endpoint inclusion in the real interior; empty is included.
    pub fn interior_subset_of(
        &self,
        other: &Self,
        budget: &mut DyadicBudget,
    ) -> Result<bool, IntervalError> {
        self.relation(other, budget, true, false)
    }

    /// Whether both sets are nonempty and this set lies strictly to the left.
    pub fn strictly_left_of(
        &self,
        other: &Self,
        budget: &mut DyadicBudget,
    ) -> Result<bool, IntervalError> {
        self.relation(other, budget, false, true)
    }

    fn relation(
        &self,
        other: &Self,
        budget: &mut DyadicBudget,
        interior: bool,
        left: bool,
    ) -> Result<bool, IntervalError> {
        if self.is_empty() {
            return Ok(!left);
        }
        if other.is_empty() {
            return Ok(false);
        }
        let exponent = self.common_exponent(other);
        let a = self.align(exponent, budget)?.expect("nonempty input");
        let b = other.align(exponent, budget)?.expect("nonempty input");
        Ok(if left {
            a.as_refs().left_of(b.as_refs())
        } else if interior {
            a.as_refs().interior_subset(b.as_refs())
        } else {
            a.as_refs().subset(b.as_refs())
        })
    }
}

// Nonnegative point powers use the integer backend, never interval products.
fn endpoint_power(
    value: &BigInt,
    exponent: u32,
    budget: &mut DyadicBudget,
) -> Result<BigInt, IntervalError> {
    if value.bits() <= 1 {
        budget.reserve(1, 1, false)?;
        return Ok(if exponent.is_multiple_of(2) && value < &BigInt::zero() {
            BigInt::one()
        } else {
            value.clone()
        });
    }
    let bits = value
        .bits()
        .checked_mul(u64::from(exponent))
        .ok_or(IntervalError::SizeExhausted)?;
    let stages = u64::from(u32::BITS - exponent.leading_zeros()) * 2;
    budget.reserve(bits, stages, true)?;
    Ok(value.pow(exponent))
}

// For n >= 0 and d > 0, floor(sqrt(n/d * 2^shift)) equals
// floor(sqrt(floor(n/d * 2^shift))). The upper endpoint equals this floor
// exactly when the quotient is integral and its floor is a perfect square.
// Domain checks belong to the public entry points.
fn scaled_sqrt(
    n: &BigInt,
    d: &BigInt,
    shift: i64,
    budget: &mut DyadicBudget,
) -> Result<(BigInt, BigInt), IntervalError> {
    if n.is_zero() {
        budget.reserve(1, 1, false)?;
        return Ok((BigInt::zero(), BigInt::zero()));
    }
    let (floor, ceil) = scaled_quotient(n, d, shift, budget)?;
    let bits = floor.bits().saturating_add(2);
    // Charge the integer root backend logarithmically many quadratic stages,
    // plus its exact square check, increment, and owned output. These are
    // deterministic accounting units, not a bound on backend CPU instructions.
    let stages = 2 * u64::from(u64::BITS - bits.leading_zeros()) + 4;
    budget.reserve(bits, stages, true)?;
    let lower = floor.sqrt();
    let upper = if floor == ceil && &lower * &lower == floor {
        lower.clone()
    } else {
        &lower + 1
    };
    Ok((lower, upper))
}

// Exact floor/ceiling of (numerator / denominator) * 2^shift.
// Guard bit growth before constructing either scaled operand.
pub(crate) fn scaled_quotient(
    n: &BigInt,
    d: &BigInt,
    shift: i64,
    budget: &mut DyadicBudget,
) -> Result<(BigInt, BigInt), IntervalError> {
    if d.is_zero() {
        return Err(IntervalError::ContainsZero);
    }
    let n_bits = n
        .bits()
        .saturating_add(if shift >= 0 { shift as u64 } else { 0 });
    let d_bits = d
        .bits()
        .saturating_add(if shift < 0 { shift.unsigned_abs() } else { 0 });
    budget.reserve(n_bits.max(d_bits).saturating_add(1), 4, true)?;
    let amount = usize::try_from(shift.unsigned_abs()).map_err(|_| IntervalError::Overflow)?;
    let (n, d) = if shift >= 0 {
        (n << amount, d.clone())
    } else {
        (n.clone(), d << amount)
    };
    Ok((n.div_floor(&d), n.div_ceil(&d)))
}

#[cfg(feature = "rational")]
mod bridge {
    use super::*;
    use crate::RationalInterval;
    use hyperreal::Rational;
    #[cfg(feature = "hyperreal")]
    use hyperreal::Real;
    use num_bigint::BigUint;
    use num_traits::One;

    impl RationalInterval {
        /// Tight outward grid enclosure of the principal square root.
        ///
        /// Requires the `dyadic` feature. Output spacing is `2^precision`.
        /// Negative source points are excluded with `Partial` coverage; a
        /// strictly negative nonempty input returns `EmptyDomain`. Empty maps
        /// to empty with `Complete` coverage. Scaling, integer division and
        /// root extraction use the caller's shared [`DyadicBudget`]. No rounded
        /// rational input or intermediate interval is substituted for the source.
        pub fn sqrt(
            &self,
            precision: i32,
            budget: &mut DyadicBudget,
        ) -> Result<RangeEnclosure<DyadicInterval>, IntervalError> {
            let Some((a, b)) = self.bounds() else {
                return Ok(RangeEnclosure {
                    range: DyadicInterval::empty(),
                    domain: DomainCoverage::Complete,
                });
            };
            if b.is_negative() {
                return Err(IntervalError::EmptyDomain);
            }
            let domain = if a.is_negative() {
                DomainCoverage::Partial
            } else {
                DomainCoverage::Complete
            };
            let endpoint = |value: &Rational, budget: &mut DyadicBudget| {
                if value.is_negative() || value.is_zero() {
                    budget.reserve(1, 1, false)?;
                    return Ok((BigInt::zero(), BigInt::zero()));
                }
                budget.reserve(
                    value.numerator().bits().max(value.denominator().bits()),
                    2,
                    false,
                )?;
                scaled_sqrt(
                    &BigInt::from(value.numerator().clone()),
                    &BigInt::from(value.denominator().clone()),
                    -2 * i64::from(precision),
                    budget,
                )
            };
            let (lower, lower_ceil) = endpoint(a, budget)?;
            let upper = if a == b {
                lower_ceil
            } else {
                endpoint(b, budget)?.1
            };
            Ok(RangeEnclosure {
                range: DyadicInterval::new(lower, upper, precision)?,
                domain,
            })
        }

        /// Outward enclosure with grid spacing `2^precision` and explicit
        /// conversion accounting. This does not call a floating-point backend.
        pub fn to_dyadic(
            &self,
            precision: i32,
            budget: &mut DyadicBudget,
        ) -> Result<DyadicInterval, IntervalError> {
            let Some((a, b)) = self.bounds() else {
                return Ok(DyadicInterval::empty());
            };
            let (lower, _) = round_rational(a, precision, budget)?;
            let (_, upper) = round_rational(b, precision, budget)?;
            DyadicInterval::new(lower, upper, precision)
        }
    }

    fn round_rational(
        value: &Rational,
        precision: i32,
        budget: &mut DyadicBudget,
    ) -> Result<(BigInt, BigInt), IntervalError> {
        let (sign, numerator, denominator) = value.ratio_parts();
        budget.reserve(numerator.bits().max(denominator.bits()), 2, false)?;
        let n = BigInt::from_biguint(sign, numerator.clone());
        let d = BigInt::from(denominator.clone());
        scaled_quotient(&n, &d, -i64::from(precision), budget)
    }

    impl DyadicInterval {
        /// Tight outward enclosure of an exact rational on grid `2^precision`.
        ///
        /// Reservations precede coefficient copies and scaled integer division.
        /// Stored ratios may be unreduced: admission uses their actual sizes
        /// without forcing a GCD or populating a canonicalization cache. The
        /// source stays exact; only this returned enclosure is rounded. Input
        /// construction and parsing are outside the arithmetic budget.
        pub fn from_rational(
            value: &Rational,
            precision: i32,
            budget: &mut DyadicBudget,
        ) -> Result<Self, IntervalError> {
            let (lower, upper) = round_rational(value, precision, budget)?;
            Self::new(lower, upper, precision)
        }

        /// Converts to exactly the represented rational endpoints.
        /// Budget reservations precede all power-of-two allocations.
        pub fn to_rational(
            &self,
            budget: &mut DyadicBudget,
        ) -> Result<RationalInterval, IntervalError> {
            let Some((a, b, e)) = self.bounds() else {
                return Ok(RationalInterval::empty());
            };
            let bits = self
                .bits()
                .saturating_add(i64::from(e).unsigned_abs())
                .saturating_add(1);
            budget.reserve(bits, 4, true)?;
            let amount = usize::try_from(i64::from(e).unsigned_abs())
                .map_err(|_| IntervalError::Overflow)?;
            if e >= 0 {
                Ok(RationalInterval::ordered(
                    Rational::from_bigint(a << amount),
                    Rational::from_bigint(b << amount),
                ))
            } else {
                let denominator = BigUint::one() << amount;
                Ok(RationalInterval::ordered(
                    Rational::from_bigint_fraction(a.clone(), denominator.clone())
                        .expect("nonzero power of two"),
                    Rational::from_bigint_fraction(b.clone(), denominator)
                        .expect("nonzero power of two"),
                ))
            }
        }

        /// Imports exact rational endpoints only when both lie on the specified
        /// grid. No purported dyadic certificate is trusted without validation.
        pub fn from_grid_bounds(
            lower: &Rational,
            upper: &Rational,
            precision: i32,
            budget: &mut DyadicBudget,
        ) -> Result<Self, IntervalError> {
            if lower > upper {
                return Err(IntervalError::ReversedEndpoints);
            }
            let (lo, lo_ceil) = round_rational(lower, precision, budget)?;
            let (hi_floor, hi) = round_rational(upper, precision, budget)?;
            if lo != lo_ceil || hi_floor != hi {
                return Err(IntervalError::NonDyadicEndpoint);
            }
            Self::new(lo, hi, precision)
        }

        /// Consumes Hyperreal's certified dyadic enclosure without reinterpreting
        /// its precision. Spacing `2^precision` is not an interval-width promise.
        ///
        /// Structurally rational values use [`Self::from_rational`], with
        /// reservations before coefficient copies or division. Other values
        /// retain the Hyperreal refinement path below.
        /// The budget governs conversion only. Hyperreal evaluation uses its
        /// existing precision/cancellation contract; this method does not impose
        /// a total-work bound on that evaluation. No independent cache is added.
        /// For a wholly budgeted pipeline, obtain certified bounds separately
        /// under the caller's evaluation policy and use `from_grid_bounds`.
        #[cfg(feature = "hyperreal")]
        pub fn from_real(
            value: &Real,
            precision: i32,
            budget: &mut DyadicBudget,
        ) -> Result<Self, IntervalError> {
            // Reject an unaffordable grid before asking Hyperreal to construct
            // its powers of two. This does not bound computable evaluation.
            budget.reserve(i64::from(precision).unsigned_abs() + 1, 1, false)?;
            if let Some(rational) = value.exact_rational_ref() {
                return Self::from_rational(rational, precision, budget);
            }
            let [lower, upper] = value
                .certified_dyadic_interval(precision)
                .ok_or(IntervalError::EnclosureUnavailable)?;
            Self::from_grid_bounds(&lower, &upper, precision, budget)
        }
    }
}

#[cfg(feature = "serde")]
impl serde::Serialize for DyadicInterval {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.bounds().serialize(serializer)
    }
}

#[cfg(feature = "serde")]
impl<'de> serde::Deserialize<'de> for DyadicInterval {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        match Option::<(BigInt, BigInt, i32)>::deserialize(deserializer)? {
            None => Ok(Self::empty()),
            Some((lower, upper, exponent)) => Self::new(lower, upper, exponent),
        }
        .map_err(serde::de::Error::custom)
    }
}

impl DyadicInterval {
    /// Import exactly the finite IEEE binary64 value, independent of the native
    /// arithmetic environment. NaN and infinity are typed validation errors.
    ///
    /// This adopts the supplied bits as a number; it certifies no relation to an
    /// earlier exact source or trajectory. In particular, `0.1_f64` is not 1/10.
    /// Bit validation/decoding precede native budget reservation, and reservation
    /// precedes mantissa allocation. No large power-of-two denominator is built.
    #[cfg(any(feature = "float-proposal", feature = "float-enclosure"))]
    pub fn from_f64(value: f64, budget: &mut DyadicBudget) -> Result<Self, IntervalError> {
        let bits = crate::ieee::BINARY64.finite(value.to_bits())?;
        crate::ieee::BINARY64.dyadic_point(bits, budget)
    }
}

#[cfg(test)]
#[path = "dyadic_quotient_tests.rs"]
mod quotient_tests;
