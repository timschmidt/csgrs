// Query-local, non-numerical domain replay for the optional root strategy.
/// Replay selected native sign/domain evidence locally within one bounded recognition attempt.
impl RingRecognition<'_> {
    /// Read or replay a sign for an existing atom index, caching success only in this query.
    /// Charge before even a cache hit. On any replay decline, try the admitted relation
    /// root_sign, including when replay spent its remaining allowance. None is stored
    /// as absence and may be tried again. The caller must provide an index valid in
    /// the parallel atom/sign/relation tables; no global node fact is published.
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

    /// Multiply two proved real signs, with exact zero annihilation.
    /// Operands must already denote defined real values; this helper checks no domains.
    fn sign_product(a: RealSign, b: RealSign) -> RealSign {
        match (a, b) {
            (RealSign::Zero, _) | (_, RealSign::Zero) => RealSign::Zero,
            (a, b) if a == b => RealSign::Positive,
            _ => RealSign::Negative,
        }
    }

    /// Read an exact One/Int/Ratio leaf within this recognizer's coefficient-size rules.
    /// Integers require bit length strictly below the coefficient cap; rationals use
    /// the stored-size and combined numerator/denominator check. Other shapes decline.
    /// No counter is charged here; the replay caller accounts for the visit.
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

    /// Replay selected sign rules without numerical refinement or global proof publication.
    /// Charge first, then accept an immediate sign before depth/node/leaf-size checks.
    /// Otherwise consume a node and recurse through supported arithmetic, requiring
    /// nonzero reciprocal children and nonnegative principal-root children. Add can
    /// decide same-sign/zero cases; mixed nonzero signs decline. Rational sine/tangent
    /// accept only coefficients within the size cap and the inclusive unit interval.
    ///
    /// Positive constants/exp and immediate facts assume valid native operands; this
    /// is not full recipe admission. Child order is preserved and even a zero product
    /// requires both child queries. Unsupported/unknown/capped/stopped outcomes all
    /// return None, without a typed reason or permanent failure cache. Charged steps
    /// poll the query root's stop handle, not independently every descendant handle.
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

    /// Decide an admitted polynomial sign from signed terms, then fixed constant intervals.
    /// Split terms using query-local atom signs; empty or one-sided support decides
    /// zero/strict sign. Mixed signs require every referenced atom to be a positive
    /// shared constant, whose rational enclosure is multiplied and summed outward
    /// with exact arithmetic. Coefficient caps are checked after each arithmetic step.
    /// Unseparated bounds, nonconstant mixed terms, or a failed charged step return
    /// None. No atom is refined, and malformed IDs/term invariants are caller errors;
    /// this helper is not a general polynomial sign decision procedure.
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

    /// Combine polynomial signs for a quotient whose denominator domain is already admitted.
    /// A zero numerator returns zero without inspecting the denominator; omitted
    /// denominator means one. Otherwise require a proved nonzero denominator sign.
    /// The admission precondition matters: this helper alone must not validate 0/0.
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

    /// Prove an already admitted quotient nonzero, preferring exact rational projection.
    /// A projected zero declines immediately. Only root-completion mode may then use
    /// the optional native sign proof. None combines undecided, zero and exhausted
    /// recognition; it is not itself a typed public domain error.
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

    /// Admit a principal square-root relation during the root-completion strategy.
    /// Require positive remaining depth, expand the radicand with the same query
    /// state, and prove its quotient nonnegative. Retain power two, that quotient,
    /// and the selected zero/positive root sign. No root is numerically evaluated;
    /// other recipes, disabled strategy, negative or unproved domains and spent
    /// substep limits decline. Existing operand admission remains a precondition.
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
