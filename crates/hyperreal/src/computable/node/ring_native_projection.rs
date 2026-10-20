// Candidate generation and sufficient exact-zero proofs; no numeric guesses.
impl<'a> RingRecognition<'a> {
    fn recognize_native(&mut self, value: &'a Computable) -> Option<Rational> {
        self.collect_parameter_atoms(value)?;
        let quotient = self.expand_quotient(value, Self::MAX_DEPTH)?;
        self.project_native_relations(quotient)
    }

    fn begin_root_completion(&mut self) -> Option<()> {
        self.charge()?;
        self.atoms.clear();
        self.quotients.clear();
        self.relations.clear();
        self.native_signs.clear();
        self.jacobi_bases.clear();
        self.parameter_atoms.clear();
        self.root_relation_seen = false;
        self.root_completion = true;
        // Strategy-local IDs cannot survive the changed atom decomposition.
        // Resource counters and cancellation remain those of the same query.
        Some(())
    }

    fn project_native_relations(&mut self, value: RingQuotient) -> Option<Rational> {
        let zero_mode = if self.root_completion {
            NativeReduction::SignedZeroResidual
        } else {
            NativeReduction::ZeroResidual
        };
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
                && let Some(reduced) =
                    self.reduce_native_relation_proof(RingQuotient::polynomial(residual), zero_mode)
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
        let reduced =
            self.reduce_native_relation_proof(RingQuotient::polynomial(numerator), zero_mode)?;
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

    // P,Q >= 0 and P²=Q² imply P=Q, including P=Q=0. Retained exact
    // signs partition this residual as P-Q; an unresolved term declines.
    // Squaring is only a sufficient zero proof, never a value rewrite.
    fn split_nonnegative_terms(
        &mut self,
        value: &RingPolynomial,
    ) -> Option<(RingPolynomial, RingPolynomial)> {
        let mut positive = RingPolynomial::default();
        let mut negative = RingPolynomial::default();
        for (monomial, coefficient) in &value.terms {
            self.charge()?;
            let mut sign = public_sign(coefficient.sign());
            for &atom in monomial {
                match self.native_atom_sign(usize::from(atom))? {
                    RealSign::Zero => {
                        sign = RealSign::Zero;
                        break;
                    }
                    RealSign::Negative => {
                        sign = match sign {
                            RealSign::Negative => RealSign::Positive,
                            RealSign::Positive => RealSign::Negative,
                            RealSign::Zero => RealSign::Zero,
                        }
                    }
                    RealSign::Positive => {}
                }
            }
            match sign {
                RealSign::Positive => {
                    self.insert(&mut positive, monomial.clone(), coefficient.clone())?
                }
                RealSign::Negative => {
                    self.insert(&mut negative, monomial.clone(), -coefficient.clone())?
                }
                RealSign::Zero => {}
            }
        }
        Some((positive, negative))
    }

    fn prove_zero_by_nonnegative_squares(&mut self, value: &RingPolynomial) -> Option<()> {
        let (mut positive, mut negative) = self.split_nonnegative_terms(value)?;
        if positive.terms.len() > 1 && negative.terms.len() > 1 {
            // Formal exact division proves P=R*Q (or Q=R*P). R=1 is
            // sufficient even when the discarded common polynomial vanishes;
            // no inverse or nonzero fact about that factor is published.
            let quotient = self
                .divide_exact_polynomial(&positive, &negative)
                .or_else(|| self.divide_exact_polynomial(&negative, &positive));
            if let Some(quotient) = quotient {
                let residual = self.add(quotient, Self::constant(Rational::from(-1))?)?;
                (positive, negative) = self.split_nonnegative_terms(&residual)?;
            }
        }
        let positive = self.multiply(&positive, &positive)?;
        let negative = self.multiply(&negative, &negative)?;
        let negative = self.scale(negative, Rational::from(-1))?;
        let norm = self.add(positive, negative)?;
        // The plain mode forbids recursive square-sign attempts and consumes
        // the same remaining work budget and admitted atom identities.
        self.reduce_native_relation_proof(
            RingQuotient::polynomial(norm),
            NativeReduction::ZeroResidual,
        )
        .map(|_| ())
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
        let rational = recognition.recognize_native(self).or_else(|| {
            recognition.begin_root_completion()?;
            recognition.recognize_native(self)
        })?;
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
