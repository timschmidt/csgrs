impl Computable {
    /// A bounded sign certificate for rational linear combinations of real logs.
    /// After proving every argument positive and clearing denominators by D > 0,
    /// D * sum(c_i * ln(a_i)) = ln(P / N). Both products are positive, so the
    /// logarithmic sum has the sign of P - N. An algebraic certificate, never
    /// numeric proximity, is required to prove zero. Failure is not a decision.
    fn log_relation_sign(&self, min_precision: Precision) -> Option<RealSign> {
        use num::{Integer, ToPrimitive};

        fn possible_term(node: &Computable) -> bool {
            match &node.internal.approximation {
                Approximation::Add(..)
                | Approximation::Negate(..)
                | Approximation::Offset(..)
                | Approximation::Multiply(..)
                | Approximation::LinearCombination3(..)
                | Approximation::PrescaledLn(..)
                | Approximation::PrescaledLnRational(..)
                | Approximation::BinaryScaledLnRational { .. }
                | Approximation::Constant(
                    SharedConstant::Ln2
                    | SharedConstant::Ln3
                    | SharedConstant::Ln5
                    | SharedConstant::Ln6
                    | SharedConstant::Ln7
                    | SharedConstant::Ln10,
                ) => true,
                Approximation::Int(n) => n.is_zero(),
                Approximation::Ratio(r) => r.is_zero(),
                _ => false,
            }
        }

        fn collect(
            node: &Computable,
            coefficient: Rational,
            remaining: &mut usize,
            terms: &mut Vec<(Computable, Rational)>,
        ) -> Option<()> {
            *remaining = remaining.checked_sub(1)?;
            if coefficient.numerator().bits() > 8 || coefficient.denominator().bits() > 8 {
                return None;
            }
            let argument = match &node.internal.approximation {
                Approximation::Add(left, right) => {
                    collect(left, coefficient.clone(), remaining, terms)?;
                    return collect(right, coefficient, remaining, terms);
                }
                Approximation::Negate(child) => {
                    return collect(child, -coefficient, remaining, terms);
                }
                Approximation::Offset(child, shift) if (-8..=8).contains(shift) => {
                    return collect(
                        child,
                        coefficient * Computable::power_of_two_rational(*shift),
                        remaining,
                        terms,
                    );
                }
                Approximation::Multiply(left, right) => {
                    if let Some(scale) = left.exact_rational() {
                        return collect(right, coefficient * scale, remaining, terms);
                    }
                    return collect(
                        left,
                        coefficient * right.exact_rational()?,
                        remaining,
                        terms,
                    );
                }
                Approximation::LinearCombination3(combination) => {
                    for (child, scale) in combination.coefficients.iter().zip(&combination.values) {
                        collect(child, &coefficient * scale, remaining, terms)?;
                    }
                    return Some(());
                }
                Approximation::PrescaledLn(child) => child.clone().add(Computable::one()),
                Approximation::PrescaledLnRational(residual) => {
                    Computable::rational(residual + Rational::one())
                }
                Approximation::BinaryScaledLnRational { residual, shift } => {
                    if *shift != 0 {
                        collect(
                            &Computable::ln2(),
                            &coefficient * Rational::new(i64::from(*shift)),
                            remaining,
                            terms,
                        )?;
                    }
                    Computable::rational(residual + Rational::one())
                }
                Approximation::Constant(constant) => {
                    let base = match constant {
                        SharedConstant::Ln2 => 2,
                        SharedConstant::Ln3 => 3,
                        SharedConstant::Ln5 => 5,
                        SharedConstant::Ln6 => 6,
                        SharedConstant::Ln7 => 7,
                        SharedConstant::Ln10 => 10,
                        _ => return None,
                    };
                    Computable::rational(Rational::new(base))
                }
                _ => return node.exact_rational().filter(Rational::is_zero).map(|_| ()),
            };
            if terms.len() >= 16 {
                return None;
            }
            terms.push((argument, coefficient));
            Some(())
        }

        fn power(mut base: Computable, mut exponent: u64) -> Computable {
            let mut result = Computable::one();
            while exponent != 0 {
                if exponent & 1 != 0 {
                    result = result.multiply(base.clone());
                }
                exponent >>= 1;
                if exponent != 0 {
                    base = base.square();
                }
            }
            result
        }

        if should_stop(&self.signal) {
            return None;
        }
        // Reject common non-log differences before constructing rational
        // coefficients or allocating a collector. This is syntax only, not
        // a numerical shortcut or a cached claim about an unresolved value.
        if let Approximation::Add(left, right) = &self.internal.approximation
            && (!possible_term(left) || !possible_term(right))
        {
            return None;
        }
        let mut terms = Vec::new();
        if collect(self, Rational::one(), &mut 128, &mut terms).is_none() || terms.len() < 2 {
            return None;
        }
        let mut denominator = 1_u64;
        for (_, coefficient) in &terms {
            let d = coefficient.denominator().to_u64()?;
            let next = denominator.checked_mul(d / denominator.gcd(&d))?;
            if next > 256 {
                return None;
            }
            denominator = next;
        }
        let mut positive = Self::one();
        let mut negative = Self::one();
        let mut total_power = 0_u64;
        for (argument, coefficient) in terms {
            if argument.algebraic_separation_bound_bits().is_none()
                || argument.sign_until(min_precision) != Some(RealSign::Positive)
            {
                return None;
            }
            let Some(exponent) = (coefficient * Rational::new(denominator as i64)).to_integer_i64()
            else {
                return None;
            };
            let magnitude = exponent.unsigned_abs();
            if magnitude > 64 {
                return None;
            }
            total_power += magnitude;
            if total_power > 256 {
                return None;
            }
            let value = power(argument, magnitude);
            if exponent < 0 {
                negative = negative.multiply(value);
            } else {
                positive = positive.multiply(value);
            }
        }
        let difference = positive.add(negative.negate());
        // This also excludes nested logs from the recursive sign query, so
        // trying this certificate cannot create a log-proof recursion cycle.
        if difference.algebraic_separation_bound_bits().is_none() {
            return None;
        }
        let result = difference.sign_until(min_precision)?;
        (!should_stop(&self.signal)).then_some(result)
    }

    #[cfg(test)]
    fn log_relation_is_zero(&self, min_precision: Precision) -> bool {
        self.log_relation_sign(min_precision) == Some(RealSign::Zero)
    }
}
