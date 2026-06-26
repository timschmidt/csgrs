impl Real {
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
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        }
    }

    fn irrational_from_computable(computable: Computable) -> Self {
        Self {
            rational: Rational::one(),
            class: Irrational,
            computable: Some(computable),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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
                primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
            }
        } else {
            Self {
                rational: Rational::one(),
                class: SinPi(reduced),
                computable: Some(computable),
                signal: None,
                primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
            }
        }
    }

    fn cos_pi_rational(rational: Rational) -> Option<Real> {
        if rational.is_integer() {
            let odd = rational
                .integer_magnitude()
                .expect("integer rational has integer magnitude")
                .bit(0);
            return Some(if odd { -Self::one() } else { Self::one() });
        }

        if rational.sign() == Sign::Plus && rational < *rationals::HALF {
            let denominator = rational.denominator();
            if denominator == unsigned::THREE.deref() {
                return Some(constants::half());
            }
            if denominator == unsigned::FOUR.deref() {
                return Some(constants::sqrt_two_over_two());
            }
            if denominator == unsigned::SIX.deref() {
                return Some(constants::sqrt_three_over_two());
            }
        }

        let mut reduced = rational.fract();
        if reduced.sign() == Sign::Minus {
            reduced = reduced.neg();
        }

        let negate = if reduced > *rationals::HALF {
            reduced = Rational::one() - reduced;
            true
        } else {
            false
        };

        let denominator = reduced.denominator();
        let exact = if reduced.is_zero() {
            Self::one()
        } else if denominator == unsigned::TWO.deref() {
            Self::zero()
        } else if denominator == unsigned::THREE.deref() {
            constants::half()
        } else if denominator == unsigned::FOUR.deref() {
            constants::sqrt_two_over_two()
        } else if denominator == unsigned::SIX.deref() {
            constants::sqrt_three_over_two()
        } else {
            return None;
        };

        Some(if negate { exact.neg() } else { exact })
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
        if self.rational.sign() != Sign::NoSign {
            match &self.class {
                One => {
                    crate::trace_dispatch!("real", "inverse", "prechecked-one");
                    return Ok(Self {
                        rational: self.rational.inverse()?,
                        class: One,
                        computable: None,
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
                Pi => {
                    crate::trace_dispatch!("real", "inverse", "prechecked-pi");
                    return Ok(Self {
                        rational: self.rational.clone().inverse()?,
                        class: PiInv,
                        computable: Some(Computable::pi_inverse_constant()),
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
                PiInv => {
                    crate::trace_dispatch!("real", "inverse", "prechecked-pi-inverse");
                    return Ok(Self {
                        rational: self.rational.clone().inverse()?,
                        class: Pi,
                        computable: Some(Computable::pi()),
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
                Sqrt(sqrt) => {
                    if let Some(sqrt) = sqrt.integer_magnitude() {
                        crate::trace_dispatch!(
                            "real",
                            "inverse",
                            "prechecked-sqrt-rational-radical"
                        );
                        let rational = if self.rational.is_one() {
                            Rational::from_unsigned_integer(sqrt.clone()).inverse()?
                        } else {
                            (self.rational * Rational::from_unsigned_integer(sqrt.clone()))
                                .inverse()?
                        };
                        return Ok(Self {
                            rational,
                            class: self.class,
                            computable: self.computable,
                            signal: None,
                            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                        });
                    }
                }
                ConstProductSqrt(product) => {
                    crate::trace_dispatch!("real", "inverse", "prechecked-const-product-sqrt");
                    let radicand = product.radicand.clone();
                    let rational = if self.rational.is_one() {
                        radicand.clone().inverse()?
                    } else {
                        (self.rational * radicand.clone()).inverse()?
                    };
                    let (class, computable) = Class::make_const_product_sqrt(
                        -product.pi_power,
                        product.exp_power.clone().neg(),
                        radicand,
                    );
                    return Ok(Self {
                        rational,
                        class,
                        computable: Some(computable),
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
                _ => {}
            }
        }
        if self.definitely_zero() {
            crate::trace_dispatch!("real", "inverse", "div-by-zero");
            return Err(Problem::DivideByZero);
        }
        match &self.class {
            One => {
                // Rational reciprocals remain exact.
                crate::trace_dispatch!("real", "inverse", "one");
                return Ok(Self {
                    rational: self.rational.inverse()?,
                    class: One,
                    computable: None,
                    signal: None,
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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
                    computable: Some(Computable::pi_inverse_constant()),
                    signal: None,
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                });
            }
            PiInv => {
                // Reciprocal-pi is its own class; inverting it restores the
                // canonical cached pi class without generic const-product setup.
                crate::trace_dispatch!("real", "inverse", "pi-inverse");
                return Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: Pi,
                    computable: Some(Computable::pi()),
                    signal: None,
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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
                primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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
                primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
            });
        }
        crate::trace_dispatch!("real", "inverse", "generic");
        Ok(Self {
            rational: self.rational.clone().inverse()?,
            class: Irrational,
            computable: Some(Computable::inverse(self.computable_clone())),
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        })
    }

    /// The multiplicative inverse of this Real without consuming it.
    pub fn inverse_ref(&self) -> Result<Self, Problem> {
        if self.rational.sign() != Sign::NoSign {
            match &self.class {
                One => {
                    crate::trace_dispatch!("real", "inverse_ref", "prechecked-one");
                    return Ok(Self {
                        rational: self.rational.clone().inverse()?,
                        class: One,
                        computable: None,
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
                Pi => {
                    crate::trace_dispatch!("real", "inverse_ref", "prechecked-pi");
                    return Ok(Self {
                        rational: self.rational.clone().inverse()?,
                        class: PiInv,
                        computable: Some(Computable::pi_inverse_constant()),
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
                PiInv => {
                    crate::trace_dispatch!("real", "inverse_ref", "prechecked-pi-inverse");
                    return Ok(Self {
                        rational: self.rational.clone().inverse()?,
                        class: Pi,
                        computable: Some(Computable::pi()),
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
                Sqrt(sqrt) => {
                    if let Some(sqrt) = sqrt.integer_magnitude() {
                        crate::trace_dispatch!(
                            "real",
                            "inverse_ref",
                            "prechecked-sqrt-rational-radical"
                        );
                        let rational = if self.rational.is_one() {
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
                            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                        });
                    }
                }
                ConstProductSqrt(product) => {
                    crate::trace_dispatch!("real", "inverse_ref", "prechecked-const-product-sqrt");
                    let radicand = product.radicand.clone();
                    let rational = if self.rational.is_one() {
                        radicand.clone().inverse()?
                    } else {
                        (&self.rational * radicand.clone()).inverse()?
                    };
                    let (class, computable) = Class::make_const_product_sqrt(
                        -product.pi_power,
                        product.exp_power.clone().neg(),
                        radicand,
                    );
                    return Ok(Self {
                        rational,
                        class,
                        computable: Some(computable),
                        signal: None,
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
                _ => {}
            }
        }
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
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
                crate::trace_dispatch!("real", "inverse_ref", "sqrt-generic");
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: Irrational,
                    computable: Some(Computable::inverse(self.computable_clone())),
                    signal: None,
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                })
            }
            Pi => {
                // Preserve the dedicated reciprocal-pi class for borrowed scalar
                // division; rebuilding through the generic constant product costs more.
                crate::trace_dispatch!("real", "inverse_ref", "pi");
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: PiInv,
                    computable: Some(Computable::pi_inverse_constant()),
                    signal: None,
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                })
            }
            PiInv => {
                crate::trace_dispatch!("real", "inverse_ref", "pi-inverse");
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: Pi,
                    computable: Some(Computable::pi()),
                    signal: None,
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                })
            }
            PiExp(exp) => {
                crate::trace_dispatch!("real", "inverse_ref", "pi-exp");
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: PiInvExp(exp.clone().neg()),
                    computable: Some(self.computable_clone().inverse()),
                    signal: None,
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                })
            }
            PiInvExp(exp) => {
                crate::trace_dispatch!("real", "inverse_ref", "pi-inv-exp");
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: PiExp(exp.clone().neg()),
                    computable: Some(self.computable_clone().inverse()),
                    signal: None,
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
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
                        primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                    });
                }
                crate::trace_dispatch!("real", "inverse_ref", "generic");
                Ok(Self {
                    rational: self.rational.clone().inverse()?,
                    class: Irrational,
                    computable: Some(Computable::inverse(self.computable_clone())),
                    signal: None,
                    primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
                })
            }
        }
    }

}
