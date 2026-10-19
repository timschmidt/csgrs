// Candidate generation and sufficient exact-zero proofs; no numeric guesses.
impl RingRecognition<'_> {
    fn project_native_relations(&mut self, value: RingQuotient) -> Option<Rational> {
        // Jacobi's initial values (sn=0, cn=dn=1) suggest the rational
        // constant of a norm identity without expanding its common factors.
        // Other atoms take the arbitrary value 1 for this PROPOSAL only.
        // An assignment is never evidence: publish only after the complete
        // residual N-cD reduces to zero using admitted native relations.
        if let Some(candidate) = self.native_projection_candidate(&value) {
            let denominator = value
                .denominator
                .clone()
                .unwrap_or(Self::constant(Rational::one())?);
            let residual = self
                .scale(denominator, -candidate.clone())
                .and_then(|negative| self.add(value.numerator.clone(), negative));
            if let Some(residual) = residual
                && let Some(reduced) = self.reduce_native_relation_proof(
                    RingQuotient::polynomial(residual),
                    NativeReduction::ZeroResidual,
                )
                && self.project_quotient(&reduced).is_some_and(|r| r.is_zero())
            {
                return Some(candidate);
            }
        }
        // An already admitted denominator need not be normalized to prove a
        // zero numerator. If clearing relations on both sides exceeds a
        // structural limit, retry that smaller obligation with the remaining
        // work and the same atom identities. No inverse guard is bypassed.
        let numerator = value.numerator.clone();
        if let Some(reduced) = self.reduce_native_relations(value) {
            return self.project_quotient(&reduced);
        }
        let reduced = self.reduce_native_relation_proof(
            RingQuotient::polynomial(numerator),
            NativeReduction::ZeroResidual,
        )?;
        self.project_quotient(&reduced).filter(Rational::is_zero)
    }

    fn native_projection_candidate(&mut self, value: &RingQuotient) -> Option<Rational> {
        let numerator = self.native_initial_assignment(&value.numerator)?;
        let denominator = match &value.denominator {
            Some(denominator) => self.native_initial_assignment(denominator)?,
            None => Rational::one(),
        };
        self.charge()?;
        let candidate = numerator * denominator.inverse().ok()?;
        Self::coefficient_fits(&candidate).then_some(candidate)
    }

    fn native_initial_assignment(&mut self, value: &RingPolynomial) -> Option<Rational> {
        let mut result = Rational::zero();
        for (monomial, coefficient) in &value.terms {
            let mut zero = false;
            for &atom in monomial {
                self.charge()?;
                if matches!(
                    self.atoms[usize::from(atom)].internal.approximation,
                    Approximation::JacobiSn(_)
                ) {
                    zero = true;
                    break;
                }
            }
            if !zero {
                self.charge()?;
                result = result + coefficient;
                if !Self::coefficient_fits(&result) {
                    return None;
                }
            }
        }
        Some(result)
    }

    fn remove_common_monomial_for_zero_proof(&mut self, value: &mut RingPolynomial) -> Option<()> {
        self.charge()?;
        let Some((first, _)) = value.terms.first() else {
            return Some(());
        };
        let mut common = first.clone();
        for (term, _) in value.terms.iter().skip(1).rev() {
            if common.is_empty() {
                return Some(());
            }
            let (mut i, mut j) = (0, 0);
            let mut intersection = Vec::new();
            while i < common.len() && j < term.len() {
                self.charge()?;
                match common[i].cmp(&term[j]) {
                    Ordering::Less => i += 1,
                    Ordering::Greater => j += 1,
                    Ordering::Equal => {
                        intersection.push(common[i]);
                        i += 1;
                        j += 1;
                    }
                }
            }
            common = intersection;
        }
        if common.is_empty() {
            return Some(());
        }
        // Only a sufficient zero claim is retained: if the quotient is zero,
        // its product with the discarded monomial is zero even when that
        // factor can vanish. This never proves a nonzero value or an inverse.
        for (term, _) in &mut value.terms {
            let (mut removed, mut write) = (0, 0);
            for read in 0..term.len() {
                self.charge()?;
                if common.get(removed) == Some(&term[read]) {
                    removed += 1;
                } else {
                    term[write] = term[read];
                    write += 1;
                }
            }
            term.truncate(write);
            self.charge()?;
        }
        // Cancelling a common multiset need not preserve sorted-word order.
        // Ordering at most 64 bounded monomials cannot merge distinct terms.
        value.terms.sort_unstable_by(|(a, _), (b, _)| a.cmp(b));
        Some(())
    }
}

impl Computable {
    fn retained_rational_reduction(&self) -> Option<Rational> {
        let tower = self.internal.cache.quadratic_tower()?;
        (tower.even.scale.is_zero() && tower.odd.is_zero()).then_some(tower.even.rational)
    }

    pub(crate) fn bounded_native_relation_rational(&self) -> Option<Rational> {
        let mut recognition = RingRecognition::new(self);
        recognition.native_relations = true;
        recognition.collect_parameter_atoms(self)?;
        let quotient = recognition.expand_quotient(self, RingRecognition::MAX_DEPTH)?;
        let rational = recognition.project_native_relations(quotient)?;
        if should_stop(&self.signal) {
            return None;
        }
        self.internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(rational.sign()));
        // The result is a completed exact reduction, including all original
        // inverse domains. Keep its value, not just its sign, so a later
        // supported composition can consume it without unfolding this DAG.
        self.internal
            .cache
            .store_quadratic_tower(&Tower::rational(rational.clone()));
        crate::trace_dispatch!("computable", "normal_form", "native-rational-relation");
        Some(rational)
    }
}
