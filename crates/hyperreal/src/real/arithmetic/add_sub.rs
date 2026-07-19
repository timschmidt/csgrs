use std::ops::*;

fn finite_f64_operand(value: f64) -> Real {
    Real::try_from(value).expect("Real arithmetic f64 operand must be finite")
}

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

impl std::iter::Sum for Real {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::zero(), |sum, value| sum + value)
    }
}

impl<'a> std::iter::Sum<&'a Real> for Real {
    fn sum<I: Iterator<Item = &'a Real>>(iter: I) -> Self {
        iter.fold(Self::zero(), |sum, value| sum + value)
    }
}

impl Neg for Real {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            rational: -self.rational,
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
            ..self
        }
    }
}

impl Neg for &Real {
    type Output = Real;

    fn neg(self) -> Self::Output {
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
