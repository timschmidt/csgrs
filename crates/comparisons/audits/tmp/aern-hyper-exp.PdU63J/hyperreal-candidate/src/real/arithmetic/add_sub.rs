use std::ops::*;

fn finite_f64_operand(value: f64) -> Real {
    Real::try_from(value).expect("Real arithmetic f64 operand must be finite")
}

impl Real {
    /// Returns the exact arithmetic mean of two real values.
    ///
    /// Values with the same symbolic basis keep that basis and average only
    /// their rational scales. This includes pure rationals and avoids building
    /// an addition node followed by a division node in the common geometry
    /// midpoint case. Mixed symbolic values retain the normal addition
    /// simplifications before applying the exact rational scale of one half.
    #[inline]
    pub fn average_pair(left: &Self, right: &Self) -> Self {
        if left.same_symbolic_basis(right) {
            crate::trace_dispatch!("real", "average_pair", "same-symbolic-basis");
            let rational = Rational::average_pair(&left.rational, &right.rational);
            if rational.sign() == Sign::NoSign {
                return Self::zero();
            }
            if left.class == One {
                return Self::new(rational);
            }
            return Self {
                rational,
                class: left.class.clone(),
                computable: left.computable.clone(),
                primitive_approx_cache: AtomicPrimitiveApproxCache::new(
                    PrimitiveApproxCache::Empty,
                ),
            };
        }

        if left.has_zero_scale() {
            crate::trace_dispatch!("real", "average_pair", "lhs-zero");
            return right.scaled_by_rational(&rationals::HALF);
        }
        if right.has_zero_scale() {
            crate::trace_dispatch!("real", "average_pair", "rhs-zero");
            return left.scaled_by_rational(&rationals::HALF);
        }

        crate::trace_dispatch!("real", "average_pair", "mixed-symbolic-basis");
        (left + right).scaled_by_rational(&rationals::HALF)
    }

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
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        })
    }
}

impl<T: AsRef<Real>> Add<T> for &Real {
    type Output = Real;

    fn add(self, other: T) -> Self::Output {
        let other = other.as_ref();
        if self.same_symbolic_basis(other) {
            crate::trace_dispatch!("real", "add", "same-symbolic-basis");
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
                primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
            };
        }
        if self.has_zero_scale() {
            crate::trace_dispatch!("real", "add", "lhs-zero");
            return other.clone();
        }
        if other.has_zero_scale() {
            crate::trace_dispatch!("real", "add", "rhs-zero");
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
                crate::trace_dispatch!("real", "add", "ln-combination");
                return simple;
            }
        }
        if other.class == One
            && self.class.can_take_const_offset()
            && let Some(sum) =
                Self::Output::try_add_rational_to_const_term(self, other.rational.clone())
        {
            crate::trace_dispatch!("real", "add", "rhs-rational-const-offset");
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
            crate::trace_dispatch!("real", "add", "lhs-rational-const-offset");
            return sum;
        }
        crate::trace_dispatch!("real", "add", "generic-computable");
        let left = self.fold_ref();
        let right = other.fold_ref();
        let computable = Computable::add(left, right);
        Self::Output {
            rational: Rational::one(),
            class: Irrational,
            computable: Some(computable),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        }
    }
}

impl<T: AsRef<Real>> Add<T> for Real {
    type Output = Self;

    fn add(self, other: T) -> Self {
        &self + other.as_ref()
    }
}

impl Add<f64> for Real {
    type Output = Self;

    fn add(self, other: f64) -> Self {
        &self + &finite_f64_operand(other)
    }
}

impl Add<f64> for &Real {
    type Output = Real;

    fn add(self, other: f64) -> Self::Output {
        self + &finite_f64_operand(other)
    }
}

impl Add<Real> for f64 {
    type Output = Real;

    fn add(self, other: Real) -> Self::Output {
        finite_f64_operand(self) + other
    }
}

impl<T: AsRef<Real>> AddAssign<T> for Real {
    #[inline]
    fn add_assign(&mut self, other: T) {
        let other = other.as_ref();
        if matches!(self.class, One) && matches!(other.class, One) {
            crate::trace_dispatch!("real", "add", "exact-rational-assign");
            let rational = &self.rational + &other.rational;
            if rational.sign() == Sign::NoSign {
                *self = Self::zero();
                return;
            }
            self.rational = rational;
            self.primitive_approx_cache
                .set(PrimitiveApproxCache::Empty);
            return;
        }
        *self = &*self + other;
    }
}

impl AddAssign<f64> for Real {
    fn add_assign(&mut self, other: f64) {
        *self = &*self + other;
    }
}

// Left folds keep short construction-only sums cheap. For longer sums, a
// streaming binary carry bounds the addition depth logarithmically, preventing
// the two guard bits requested by each computable Add node from accumulating
// linearly on the earliest terms. The lower size hint is authoritative: an
// iterator that selects the balanced path has at least this many values.
const BALANCED_REAL_SUM_THRESHOLD: usize = 256;

fn push_balanced_sum_partial(partials: &mut Vec<Option<Real>>, mut carry: Real) {
    let mut level = 0;
    loop {
        if level == partials.len() {
            partials.push(Some(carry));
            break;
        }
        if let Some(left) = partials[level].take() {
            carry = left + carry;
            level += 1;
        } else {
            partials[level] = Some(carry);
            break;
        }
    }
}

fn balanced_real_sum<I>(mut iter: I, guaranteed_len: usize) -> Real
where
    I: Iterator<Item = Real>,
{
    let levels = usize::try_from(usize::BITS - guaranteed_len.leading_zeros())
        .expect("usize bit count fits usize");
    let mut partials: Vec<Option<Real>> = Vec::with_capacity(levels);

    // Homogeneous rational and symbolic sums collapse to one scaled value and
    // do not accumulate computable guard bits. Preserve that cheaper path even
    // for long iterators. If a distinct basis appears, the collapsed prefix is
    // already one leaf and the remainder enters the balanced reducer normally.
    let Some(first) = iter.next() else {
        return Real::zero();
    };
    let mut homogeneous_prefix = Some(first);
    for value in iter.by_ref() {
        let prefix = homogeneous_prefix
            .as_mut()
            .expect("the homogeneous prefix exists until balancing begins");
        if prefix.same_symbolic_basis(&value) {
            *prefix += value;
            continue;
        }

        push_balanced_sum_partial(
            &mut partials,
            homogeneous_prefix
                .take()
                .expect("the first distinct basis consumes the prefix"),
        );
        push_balanced_sum_partial(&mut partials, value);
        break;
    }

    if let Some(prefix) = homogeneous_prefix {
        return prefix;
    }

    for value in iter {
        push_balanced_sum_partial(&mut partials, value);
    }

    partials
        .into_iter()
        .rev()
        .flatten()
        .reduce(|left, right| left + right)
        .unwrap_or_else(Real::zero)
}

fn real_sum<I>(iter: I) -> Real
where
    I: Iterator<Item = Real>,
{
    let guaranteed_len = iter.size_hint().0;
    if guaranteed_len < BALANCED_REAL_SUM_THRESHOLD {
        return iter.fold(Real::zero(), |sum, value| sum + value);
    }
    balanced_real_sum(iter, guaranteed_len)
}

// Keep the short borrowed path clone-free, but materialize owned values only
// after a proven-long iterator selects balancing. The owned entry point above
// can feed its values straight through without cloning them first.
fn real_sum_borrowed<'a, I>(iter: I) -> Real
where
    I: Iterator<Item = &'a Real>,
{
    let guaranteed_len = iter.size_hint().0;
    if guaranteed_len < BALANCED_REAL_SUM_THRESHOLD {
        return iter.fold(Real::zero(), |sum, value| sum + value);
    }
    balanced_real_sum(iter.cloned(), guaranteed_len)
}

impl std::iter::Sum for Real {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        real_sum(iter)
    }
}

impl<'a> std::iter::Sum<&'a Real> for Real {
    fn sum<I: Iterator<Item = &'a Real>>(iter: I) -> Self {
        real_sum_borrowed(iter)
    }
}

impl Neg for Real {
    type Output = Self;

    #[inline]
    fn neg(mut self) -> Self {
        self.rational = -self.rational;
        self.primitive_approx_cache.set(PrimitiveApproxCache::Empty);
        self
    }
}

impl Neg for &Real {
    type Output = Real;

    #[inline]
    fn neg(self) -> Self::Output {
        if matches!(self.class, One) && self.computable.is_none() {
            return Real {
                rational: -&self.rational,
                class: One,
                computable: None,
                primitive_approx_cache: AtomicPrimitiveApproxCache::new(
                    PrimitiveApproxCache::Empty,
                ),
            };
        }
        let mut ret = self.clone();
        ret.rational = -ret.rational;
        ret.primitive_approx_cache.set(PrimitiveApproxCache::Empty);
        ret
    }
}

impl<T: AsRef<Real>> Sub<T> for &Real {
    type Output = Real;

    fn sub(self, other: T) -> Self::Output {
        let other = other.as_ref();
        if self.same_symbolic_basis(other) {
            crate::trace_dispatch!("real", "sub", "same-symbolic-basis");
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
                primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
            };
        }
        if self.class == Pi
            && self.rational.is_one()
            && other.class == One
            && other.rational == *rationals::THREE
        {
            crate::trace_dispatch!("real", "sub", "cached-pi-minus-three");
            return constants::pi_minus_three();
        }
        if self.class == One
            && self.rational == *rationals::THREE
            && other.class == Pi
            && other.rational.is_one()
        {
            crate::trace_dispatch!("real", "sub", "cached-three-minus-pi");
            return -constants::pi_minus_three();
        }
        if other.has_zero_scale() {
            crate::trace_dispatch!("real", "sub", "rhs-zero");
            return self.clone();
        }
        if self.has_zero_scale() {
            crate::trace_dispatch!("real", "sub", "lhs-zero");
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
                crate::trace_dispatch!("real", "sub", "ln-combination");
                return simple;
            }
        }
        if other.class == One
            && self.class.can_take_const_offset()
            && let Some(difference) =
                Self::Output::try_add_rational_to_const_term(self, -other.rational.clone())
        {
            crate::trace_dispatch!("real", "sub", "rhs-rational-const-offset");
            return difference;
        }
        if self.class == One
            && other.class.can_take_const_offset()
            && let Some(difference) =
                Self::Output::try_add_rational_to_const_term(other, -self.rational.clone())
        {
            crate::trace_dispatch!("real", "sub", "lhs-rational-const-offset");
            return -difference;
        }
        crate::trace_dispatch!("real", "sub", "generic-computable");
        let left = self.fold_ref();
        let right = other.fold_ref().negate();
        let computable = Computable::add(left, right);
        Self::Output {
            rational: Rational::one(),
            class: Irrational,
            computable: Some(computable),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        }
    }
}

impl<T: AsRef<Real>> Sub<T> for Real {
    type Output = Self;

    fn sub(self, other: T) -> Self {
        &self - other.as_ref()
    }
}

impl Sub<f64> for Real {
    type Output = Self;

    fn sub(self, other: f64) -> Self {
        &self - &finite_f64_operand(other)
    }
}

impl Sub<f64> for &Real {
    type Output = Real;

    fn sub(self, other: f64) -> Self::Output {
        self - &finite_f64_operand(other)
    }
}

impl Sub<Real> for f64 {
    type Output = Real;

    fn sub(self, other: Real) -> Self::Output {
        finite_f64_operand(self) - other
    }
}

impl<T: AsRef<Real>> SubAssign<T> for Real {
    #[inline]
    fn sub_assign(&mut self, other: T) {
        let other = other.as_ref();
        if matches!(self.class, One) && matches!(other.class, One) {
            crate::trace_dispatch!("real", "sub", "exact-rational-assign");
            let rational = &self.rational - &other.rational;
            if rational.sign() == Sign::NoSign {
                *self = Self::zero();
                return;
            }
            self.rational = rational;
            self.primitive_approx_cache
                .set(PrimitiveApproxCache::Empty);
            return;
        }
        *self = &*self - other;
    }
}

impl SubAssign<f64> for Real {
    fn sub_assign(&mut self, other: f64) {
        *self = &*self - other;
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
                primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
            }
        } else {
            let product = x * y;
            if product == *rationals::ZERO {
                return Self {
                    rational: product,
                    class: One,
                    computable: None,
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                };
            }
            Self {
                rational: a,
                class: Sqrt(b.clone()),
                computable: Some(Computable::sqrt_squarefree_rational(b)),
                primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
            }
        }
    }
}
