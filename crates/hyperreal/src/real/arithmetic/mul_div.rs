impl<T: AsRef<Real>> Mul<T> for &Real {
    type Output = Real;

    fn mul(self, other: T) -> Self::Output {
        let other = other.as_ref();
        if self.class == One && other.class == One {
            return Self::Output::new(&self.rational * &other.rational);
        }
        if self.has_zero_scale() || other.has_zero_scale() {
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                        primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                    };
                };
                let (class, computable) = Class::make_pi_power(power);
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable: Some(computable),
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                        primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                    };
                };
                let (class, computable) = Class::make_pi_power(power);
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable: Some(computable),
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                }
            }
            (Pi, Exp(r)) | (Exp(r), Pi) => {
                // pi*e^q has a compact class because it is a frequent
                // endpoint of exact transcendental simplification.
                let (class, computable) = Class::make_pi_exp(r.clone());
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable: Some(computable),
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                        primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                    };
                };
                let (class, computable) =
                    Class::make_const_product(pi_power, product.exp_power.clone());
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable: Some(computable),
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                        primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                    };
                };
                let (class, computable) =
                    Class::make_const_product(pi_power, product.exp_power.clone());
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable: Some(computable),
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                        primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                    };
                };
                let (class, computable) =
                    Class::make_const_product(pi_power, left.exp_power.clone() + &right.exp_power);
                let rational = &self.rational * &other.rational;
                Self::Output {
                    rational,
                    class,
                    computable: Some(computable),
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    ) && let Some(pi_power) = left_pi.checked_add(right_pi)
                    {
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
                                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                                };
                            }
                            Sqrt(radicand) => {
                                let (class, computable) =
                                    Class::make_const_product_sqrt(pi_power, exp_power, radicand);
                                return Self::Output {
                                    rational,
                                    class,
                                    computable: Some(computable),
                                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                                };
                            }
                            _ => unreachable!(),
                        }
                    }
                    if let (Some((sqrt_pi, sqrt_exp, radicand)), Some((product_pi, product_exp))) = (
                        self.class.const_product_sqrt_parts(),
                        other.class.const_product_parts(),
                    ) && let Some(pi_power) = sqrt_pi.checked_add(product_pi)
                    {
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
                            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                        };
                    }
                    if let (Some((product_pi, product_exp)), Some((sqrt_pi, sqrt_exp, radicand))) = (
                        self.class.const_product_parts(),
                        other.class.const_product_sqrt_parts(),
                    ) && let Some(pi_power) = product_pi.checked_add(sqrt_pi)
                    {
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
                            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                        };
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
                        primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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

impl Mul<f64> for Real {
    type Output = Self;

    fn mul(self, other: f64) -> Self {
        &self * &finite_f64_operand(other)
    }
}

impl Mul<f64> for &Real {
    type Output = Real;

    fn mul(self, other: f64) -> Self::Output {
        self * &finite_f64_operand(other)
    }
}

impl Mul<Real> for f64 {
    type Output = Real;

    fn mul(self, other: Real) -> Self::Output {
        finite_f64_operand(self) * other
    }
}

impl<T: AsRef<Real>> MulAssign<T> for Real {
    fn mul_assign(&mut self, other: T) {
        *self = &*self * other.as_ref();
    }
}

impl MulAssign<f64> for Real {
    fn mul_assign(&mut self, other: f64) {
        *self = &*self * other;
    }
}

impl<T: AsRef<Real>> Div<T> for &Real {
    type Output = Result<Real, Problem>;

    fn div(self, other: T) -> Self::Output {
        let other = other.as_ref();
        if self.rational.is_one()
            && other.rational.is_one()
            && let (Sqrt(left), Sqrt(right)) = (&self.class, &other.class)
            && left == &*rationals::TWO
            && right == &*rationals::THREE
        {
            crate::trace_dispatch!("real", "div", "cached-sqrt-six-over-three-prechecked");
            return Ok(constants::sqrt_six_over_three());
        }
        if other.has_zero_scale() {
            crate::trace_dispatch!("real", "div", "div-by-zero");
            return Err(Problem::DivideByZero);
        }
        if self.has_zero_scale() {
            crate::trace_dispatch!("real", "div", "zero");
            return Ok(Real::zero());
        }
        if self.same_symbolic_basis(other) {
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
                primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
            });
        }
        if self.class == One {
            if let Exp(exp) = &other.class {
                crate::trace_dispatch!("real", "div", "rational-over-exp");
                let exp = exp.clone().neg();
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class: Exp(exp.clone()),
                    computable: Some(Computable::exp_rational(exp)),
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                });
            }
            crate::trace_dispatch!("real", "div", "lhs-rational-symbolic-inverse");
            let inverted = other.inverse_ref()?;
            if self.rational.is_one() {
                return Ok(inverted);
            }
            return Ok(inverted.scaled_by_rational(&self.rational));
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                });
            }
            (PiExp(exp), Exp(divisor_exp)) => {
                crate::trace_dispatch!("real", "div", "pi-exp-over-exp");
                let (class, computable) = Class::make_pi_exp(exp - divisor_exp);
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class,
                    computable: Some(computable),
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                });
            }
            (PiExp(exp), Pi) => {
                crate::trace_dispatch!("real", "div", "pi-exp-over-pi");
                let (class, computable) = Class::make_exp(exp.clone());
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class,
                    computable: Some(computable),
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                });
            }
            (Exp(exp), Pi) => {
                crate::trace_dispatch!("real", "div", "exp-over-pi");
                let computable = Computable::pi_inverse_constant()
                    .multiply(Computable::exp_rational(exp.clone()));
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class: PiInvExp(exp.clone()),
                    computable: Some(computable),
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                });
            }
            (Pi, Exp(exp)) => {
                crate::trace_dispatch!("real", "div", "pi-over-exp");
                let (class, computable) = Class::make_pi_exp(exp.clone().neg());
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class,
                    computable: Some(computable),
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                });
            }
            (ConstProductSqrt(product), Exp(exp)) => {
                if self.rational.is_one()
                    && other.rational.is_one()
                    && product.pi_power == 1
                    && product.exp_power == *rationals::ONE
                    && exp == &*rationals::ONE
                    && product.radicand == *rationals::TWO
                {
                    crate::trace_dispatch!("real", "div", "cached-pi-sqrt-two-over-exp");
                    return Ok(constants::pi_sqrt_two());
                }
                crate::trace_dispatch!("real", "div", "const-product-sqrt-over-exp");
                let (class, computable) = Class::make_const_product_sqrt(
                    product.pi_power,
                    &product.exp_power - exp,
                    product.radicand.clone(),
                );
                return Ok(Real {
                    rational: &self.rational / &other.rational,
                    class,
                    computable: Some(computable),
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                });
            }
            _ => {}
        }
        if let (Sqrt(left), Sqrt(right)) = (&self.class, &other.class)
            && let Some(right_integer) = right.integer_magnitude()
        {
            // Rationalize sqrt(a)/sqrt(b) as sqrt(a*b)/b when b is an integer.
            // This keeps simple radical quotients exact instead of using
            // `other.inverse()` and losing the radicand certificate.
            let square = Real::multiply_sqrts(left, right);
            let denominator = if other.rational.is_one() {
                // Unit-scaled denominator radicals should not pay a rational
                // multiply/gcd just to form `1*b`; keep only the structural
                // radicand denominator, avoiding unnecessary normalization.
                Rational::from_unsigned_integer(right_integer.clone())
            } else {
                &other.rational * Rational::from_unsigned_integer(right_integer.clone())
            };
            return Ok(Real {
                rational: if square.rational.is_one() && self.rational.is_one() {
                    denominator.inverse()?
                } else {
                    &square.rational * &self.rational / denominator
                },
                ..square
            });
        }
        if self.class.has_const_product_sqrt_factor() || other.class.has_const_product_sqrt_factor()
        {
            crate::trace_dispatch!("real", "div", "const-product-sqrt");
            if let (Some((left_pi, left_exp, left_rad)), Some((right_pi, right_exp, right_rad))) = (
                self.class.const_product_sqrt_parts(),
                other.class.const_product_sqrt_parts(),
            ) && let Some(pi_power) = left_pi.checked_sub(right_pi)
            {
                // Rationalize sqrt-heavy quotients before falling back to `other.inverse()`.
                // This keeps `(pi*e*sqrt(2))/(e*sqrt(3))` as one factored sqrt
                // product instead of an opaque division graph.
                let square = Real::multiply_sqrts(&left_rad, &right_rad);
                let denominator = if other.rational.is_one() {
                    // Preserve the factored sqrt quotient while skipping
                    // exact multiplication by one. Avoiding this gcd matters
                    // in matrix/vector scalar paths that divide by cached
                    // unit-scaled symbolic constants.
                    right_rad.clone()
                } else {
                    &other.rational * right_rad
                };
                let rational = if square.rational.is_one() && self.rational.is_one() {
                    denominator.inverse()?
                } else {
                    &square.rational * &self.rational / denominator
                };
                let exp_power = left_exp - right_exp;
                return Ok(match square.class {
                    One => {
                        let (class, computable) = Class::make_const_product(pi_power, exp_power);
                        Real {
                            rational,
                            class,
                            computable: Some(computable),
                            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                        }
                    }
                    Sqrt(radicand) => {
                        let (class, computable) =
                            Class::make_const_product_sqrt(pi_power, exp_power, radicand);
                        Real {
                            rational,
                            class,
                            computable: Some(computable),
                            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                        }
                    }
                    _ => unreachable!(),
                });
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
                        primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                    });
                }
            }
            if let (Some((product_pi, product_exp)), Some((sqrt_pi, sqrt_exp, radicand))) = (
                self.class.const_product_parts(),
                other.class.const_product_sqrt_parts(),
            ) && let Some(pi_power) = product_pi.checked_sub(sqrt_pi)
            {
                // Dividing by sqrt(r) multiplies numerator and denominator
                // by sqrt(r); keep the remaining sqrt(r) factored.
                let denominator = if other.rational.is_one() {
                    // The denominator is just the exact radicand for
                    // unit-scaled sqrt factors. Bypassing `1 * r` preserves
                    // delayed canonicalization and keeps hot quotient paths flatter.
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
                    primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
                });
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
                primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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
                if s == *rationals::TWO {
                    // Same rationale as the log10 fold: keep two-log quotients
                    // anchored on a single Log2 certificate.
                    let Ln(r) = &self.class else {
                        unreachable!();
                    };
                    let rational = &self.rational / &other.rational;
                    let ln2 = constants::scaled_ln(2, 1).unwrap();
                    let computable = self
                        .computable_clone()
                        .multiply(ln2.computable_clone().inverse());
                    return Ok(Real {
                        rational,
                        class: Log2(r.clone()),
                        computable: Some(self.inherit_abort(computable)),
                        primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
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

impl Div<f64> for Real {
    type Output = Result<Self, Problem>;

    fn div(self, other: f64) -> Self::Output {
        &self / &finite_f64_operand(other)
    }
}

impl Div<f64> for &Real {
    type Output = Result<Real, Problem>;

    fn div(self, other: f64) -> Self::Output {
        self / &finite_f64_operand(other)
    }
}

impl Div<Real> for f64 {
    type Output = Result<Real, Problem>;

    fn div(self, other: Real) -> Self::Output {
        finite_f64_operand(self) / other
    }
}

impl<T: AsRef<Real>> DivAssign<T> for Real {
    fn div_assign(&mut self, other: T) {
        *self = (&*self / other.as_ref()).expect("division assignment by zero Real");
    }
}

impl DivAssign<f64> for Real {
    fn div_assign(&mut self, other: f64) {
        *self = (&*self / other).expect("division assignment by zero finite f64");
    }
}
