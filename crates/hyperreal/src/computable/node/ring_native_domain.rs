// Query-local, non-numerical domain replay for the optional root strategy.
impl RingRecognition<'_> {
    fn native_atom_sign(&mut self, index: usize) -> Option<RealSign> {
        self.charge()?;
        if let Some(sign) = self.native_signs[index] {
            return Some(sign);
        }
        let value = self.atoms[index].as_ref().clone();
        let sign = self.replayed_sign(&value, Self::MAX_DEPTH).or_else(|| {
            self.relations[index]
                .as_ref()
                .and_then(|relation| relation.root_sign)
        });
        self.native_signs[index] = sign;
        sign
    }

    fn sign_product(a: RealSign, b: RealSign) -> RealSign {
        match (a, b) {
            (RealSign::Zero, _) | (_, RealSign::Zero) => RealSign::Zero,
            (a, b) if a == b => RealSign::Positive,
            _ => RealSign::Negative,
        }
    }

    fn sign_leaf(value: &Computable) -> Option<Rational> {
        match &value.internal.approximation {
            Approximation::One => Some(Rational::one()),
            Approximation::Int(n) if n.bits() < Self::MAX_COEFFICIENT_BITS => {
                Some(Rational::from_bigint(n.clone()))
            }
            Approximation::Ratio(r) if Self::coefficient_fits(r) => Some(r.clone()),
            _ => None,
        }
    }

    fn replayed_sign(&mut self, value: &Computable, depth: usize) -> Option<RealSign> {
        self.charge()?;
        if let Some(sign) = value.immediate_sign() {
            return Some(sign);
        }
        if depth == 0 {
            return None;
        }
        self.nodes_left = self.nodes_left.checked_sub(1)?;
        if let Some(rational) = Self::sign_leaf(value) {
            return Some(public_sign(rational.sign()));
        }
        match &value.internal.approximation {
            Approximation::Constant(_) | Approximation::PrescaledExp(_) => Some(RealSign::Positive),
            Approximation::Negate(child) => Some(Self::sign_product(
                RealSign::Negative,
                self.replayed_sign(child, depth - 1)?,
            )),
            Approximation::Offset(child, _) => self.replayed_sign(child, depth - 1),
            Approximation::Multiply(a, b) => Some(Self::sign_product(
                self.replayed_sign(a, depth - 1)?,
                self.replayed_sign(b, depth - 1)?,
            )),
            Approximation::Square(child) => {
                let sign = self.replayed_sign(child, depth - 1)?;
                Some(Self::sign_product(sign, sign))
            }
            Approximation::Sqrt(child) => self
                .replayed_sign(child, depth - 1)
                .filter(|sign| *sign != RealSign::Negative),
            Approximation::Inverse(child) => self
                .replayed_sign(child, depth - 1)
                .filter(|sign| *sign != RealSign::Zero),
            Approximation::Add(a, b) => {
                let a = self.replayed_sign(a, depth - 1)?;
                let b = self.replayed_sign(b, depth - 1)?;
                match (a, b) {
                    (RealSign::Zero, sign) | (sign, RealSign::Zero) => Some(sign),
                    (a, b) if a == b => Some(a),
                    _ => None,
                }
            }
            Approximation::PrescaledSinRational(r) | Approximation::PrescaledTanRational(r)
                if Self::coefficient_fits(r) && r >= &-Rational::one() && r <= &Rational::one() =>
            {
                Some(public_sign(r.sign()))
            }
            _ => None,
        }
    }

    fn native_polynomial_sign(&mut self, value: &RingPolynomial) -> Option<RealSign> {
        let (positive, negative) = self.split_nonnegative_terms(value)?;
        match (positive.terms.is_empty(), negative.terms.is_empty()) {
            (true, true) => return Some(RealSign::Zero),
            (false, true) => return Some(RealSign::Positive),
            (true, false) => return Some(RealSign::Negative),
            _ => {}
        }
        // Mixed signs may still separate using the existing fixed exact
        // intervals of shared constants. No atom is numerically refined.
        let (mut lower, mut upper) = (Rational::zero(), Rational::zero());
        for (monomial, coefficient) in &value.terms {
            let (mut lo, mut hi) = (Rational::one(), Rational::one());
            for &atom in monomial {
                self.charge()?;
                let Approximation::Constant(constant) =
                    self.atoms[usize::from(atom)].internal.approximation
                else {
                    return None;
                };
                let (a, b) = constant.interval();
                self.charge()?;
                lo *= a;
                self.charge()?;
                hi *= b;
                if !Self::coefficient_fits(&lo) || !Self::coefficient_fits(&hi) {
                    return None;
                }
            }
            self.charge()?;
            lo *= coefficient;
            self.charge()?;
            hi *= coefficient;
            if !Self::coefficient_fits(&lo) || !Self::coefficient_fits(&hi) {
                return None;
            }
            if coefficient.sign() == Sign::Minus {
                std::mem::swap(&mut lo, &mut hi);
            }
            self.charge()?;
            lower = lower + lo;
            self.charge()?;
            upper = upper + hi;
            if !Self::coefficient_fits(&lower) || !Self::coefficient_fits(&upper) {
                return None;
            }
        }
        if lower > Rational::zero() {
            Some(RealSign::Positive)
        } else if upper < Rational::zero() {
            Some(RealSign::Negative)
        } else {
            None
        }
    }

    fn native_quotient_sign(&mut self, value: &RingQuotient) -> Option<RealSign> {
        let numerator = self.native_polynomial_sign(&value.numerator)?;
        if numerator == RealSign::Zero {
            return Some(numerator);
        }
        let denominator = match &value.denominator {
            Some(value) => self.native_polynomial_sign(value)?,
            None => RealSign::Positive,
        };
        (denominator != RealSign::Zero).then(|| Self::sign_product(numerator, denominator))
    }

    fn admitted_quotient_nonzero(&mut self, value: &RingQuotient) -> Option<()> {
        if let Some(rational) = self.project_quotient(value) {
            return (!rational.is_zero()).then_some(());
        }
        (self.root_completion
            && matches!(
                self.native_quotient_sign(value),
                Some(RealSign::Positive | RealSign::Negative)
            ))
        .then_some(())
    }

    fn completed_root_relation(
        &mut self,
        value: &Computable,
        depth: usize,
    ) -> Option<NativeRingRelation> {
        if !self.root_completion || depth == 0 {
            return None;
        }
        let Approximation::Sqrt(radicand) = &value.internal.approximation else {
            return None;
        };
        let value = self.expand_relation_value(radicand, depth - 1)?;
        let sign = self.native_quotient_sign(&value)?;
        (sign != RealSign::Negative).then_some(NativeRingRelation {
            power: 2,
            value,
            root_sign: Some(sign),
        })
    }
}
