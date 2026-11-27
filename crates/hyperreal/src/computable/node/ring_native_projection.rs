// Candidate generation and sufficient exact-zero proofs; no numeric guesses.
/// Generate rational candidates and prove sufficient exact-zero obligations using admitted relations.
impl<'a> RingRecognition<'a> {
    /// Attempt rational projection after parameter collection and quotient expansion.
    /// Use the shared query state and MAX_DEPTH throughout, then prove any proposed
    /// rational with native relations. None leaves local work/state consumed; this
    /// helper itself publishes no scalar sign or retained reduction.
    fn recognize_native(&mut self, value: &'a Computable) -> Option<Rational> {
        self.collect_parameter_atoms(value)?;
        let quotient = self.expand_quotient(value, Self::MAX_DEPTH)?;
        self.project_native_relations(quotient)
    }

    /// Switch the native attempt to root-branch completion after one charged operation.
    /// Clear strategy-specific atoms, quotient memo entries, relations, signs and
    /// Jacobi/parameter choices because their IDs cannot survive decomposition changes.
    /// Preserve remaining node/operation allowances and the root stop handle. A failed
    /// charge leaves state unchanged; zero nodes alone does not prevent this handoff,
    /// although later traversal will decline. No resource allowance is replenished.
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

    /// Prove a rational candidate or exact zero from an already admitted native quotient.
    /// First propose a value by symbolic initial assignment, then require N-cD to
    /// reduce to zero. Use signed zero completion only in that strategy. Next try
    /// full quotient reduction; if it succeeds, return its optional rational projection.
    /// Only when reduction itself declines is the original numerator retried as a
    /// zero-only obligation, using the same remaining work and atom IDs. Denominator
    /// nonzero/domain proofs must already exist. No candidate assignment alone is evidence.
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

    /// Propose a rational by assigning Jacobi sn atoms zero and all other atoms one.
    /// Apply that assignment to numerator/denominator, charge division and enforce
    /// the coefficient-size cap. A zero assigned denominator or spent work declines.
    /// This is only a heuristic candidate and must never be published without proving
    /// the complete original residual zero.
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
    /// Partition an admitted polynomial as P-Q with both P and Q nonnegative at the selected values.
    /// Combine coefficient signs with query-local atom signs in stored monomial order.
    /// Drop zero terms, retaining a negative term's monomial with negated coefficient
    /// in Q; coefficients there need not themselves be positive. Unproved atom signs
    /// or charged insertion failures decline. Both partitions use the same atom IDs
    /// and caps. This does not decide whether two nonempty sides are equal.
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

    /// Prove a residual zero by equal squares of its established nonnegative sides.
    /// When both sides have multiple terms, first try formal polynomial division in
    /// either direction; equality of the resulting quotient to one suffices even if
    /// the discarded factor vanishes. Otherwise preserve the original sides. Square
    /// and subtract using admitted relations in plain zero mode, avoiding recursive
    /// sign-square attempts. Only the zero implication is returned, never a rewritten
    /// nonzero value or an inverse certificate. Limits and stop checks share this query.
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

    /// Evaluate a polynomial at the proposal-only assignment sn=0 and every other atom=1.
    /// Charge atom visits until a zero factor and each included coefficient addition;
    /// check coefficient size after addition. The result need not equal the actual
    /// polynomial value and gives no domain, sign or equality proof.
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

    /// Remove a common sorted multiset of atom IDs solely to simplify a sufficient zero obligation.
    /// If the quotient is zero, multiplying back proves the input zero even when the
    /// discarded monomial can vanish; the converse and nonzero values are not retained.
    /// Intersect terms, compact each monomial and re-sort the term list because word
    /// order can change. Charge explicit visits, not every sort comparison. A failed
    /// charge can leave a partially changed local candidate, which callers must discard.
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

/// Read or publish completed exact rational reductions while preserving the retained source graph.
impl Computable {
    /// Read a retained tower only when both its radical scale and odd part are zero.
    /// Return the exact rational part without rebuilding the original expression.
    /// No cache means no such retained reduction, even for an obvious rational leaf.
    /// Reading clones the stored tower; it does not refine or poll cancellation.
    fn retained_rational_reduction(&self) -> Option<Rational> {
        let tower = self.internal.cache.quadratic_tower()?;
        (tower.even.scale.is_zero() && tower.odd.is_zero()).then_some(tower.even.rational)
    }

    /// Try optional exact native-relation projection, then root completion with remaining work.
    /// The attempt admits original inverse domains along its recognized arithmetic
    /// skeleton and relies on defined native function atoms. It is not a full import
    /// validator. Success returns an exact rational without rewriting the source graph;
    /// a final root-signal check precedes sign publication and retained rational-tower
    /// storage, followed by the native-rational-relation dispatch trace. The tower
    /// cache retains its first successful value rather than replacing arbitrary state.
    ///
    /// None means this optional strategy declined (including unsupported forms, unknown
    /// domains, limits or cancellation), not irrationality or a permanent failure.
    /// Node/depth/term/degree/coefficient/operation caps bound selected recognition work,
    /// not total nested matcher work, peak allocation or elapsed time. Temporary proof
    /// state is owned by the attempt; no numerical approximation is used as a fallback.
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
