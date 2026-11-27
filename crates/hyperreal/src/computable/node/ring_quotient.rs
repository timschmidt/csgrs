/// A rational function over one recognition attempt's normalized opaque-atom polynomials.
/// An omitted denominator is one. Every explicit denominator must already be
/// proved nonzero at the actual atom values; a nonzero formal polynomial alone
/// is insufficient because atoms need not be algebraically independent. This
/// private carrier stores no separate proof receipt and relies on its admission
/// path. Clone copies the representation with the same query-relative atom IDs.
#[derive(Clone)]
struct RingQuotient {
    /// Exact polynomial above the fraction bar, with this attempt's atom numbering.
    /// An empty term list is zero; constructors may then drop an admitted denominator.
    numerator: RingPolynomial,
    /// None means one; an explicit polynomial carries an admission precondition
    /// that its selected atom values make it nonzero. This field does not enforce
    /// that precondition itself or encode a separate nonzero certificate.
    denominator: Option<RingPolynomial>,
}

/// Wrap already admitted fractions without performing domain proof search.
impl RingQuotient {
    /// Wrap an admitted quotient, discarding its denominator when the numerator is zero.
    /// This is representation normalization, not denominator validation: the caller
    /// must already have proved every explicit denominator nonzero at the atom values.
    /// No cancellation poll, work debit, factor reduction or size check occurs here.
    fn new(numerator: RingPolynomial, denominator: Option<RingPolynomial>) -> Self {
        // The denominator already has a nonzero witness. Once the numerator
        // cancels, retaining its denominator only creates irrelevant work.
        let denominator = if numerator.terms.is_empty() {
            None
        } else {
            denominator
        };
        Self {
            numerator,
            denominator,
        }
    }

    /// Embed a normalized polynomial with implicit denominator one.
    /// The atom IDs and coefficients must satisfy the enclosing recognition attempt's
    /// invariants. This wrapper does not validate them or spend recognition work.
    fn polynomial(numerator: RingPolynomial) -> Self {
        Self {
            numerator,
            denominator: None,
        }
    }
}

/// Expand, combine and project guarded rational functions within one proof attempt.
impl<'a> RingRecognition<'a> {
    /// Compare stored normalized polynomials over this attempt's atom IDs.
    /// Unequal term counts return false without a work debit; otherwise charge before
    /// each monomial/coefficient comparison. Equality is formal coefficient equality,
    /// not a complete test of equality at potentially dependent atom values. None
    /// means a charge declined; two empty polynomials compare equal without a poll.
    fn same_polynomial(&mut self, a: &RingPolynomial, b: &RingPolynomial) -> Option<bool> {
        if a.terms.len() != b.terms.len() {
            return Some(false);
        }
        for ((a, x), (b, y)) in a.terms.iter().zip(&b.terms) {
            self.charge()?;
            if a != b || x != y {
                return Some(false);
            }
        }
        Some(true)
    }

    /// Add two domain-admitted quotients, avoiding unnecessary denominator products.
    /// Zero operands return the other operand immediately. Equal stored denominators
    /// are shared; native mode next tries exact divisibility in each direction before
    /// cross multiplication. A cancelled cross numerator returns zero before building
    /// the denominator product. These shortcuts rely on both operands' earlier domain
    /// checks. None combines arithmetic limits and cancellation; failed optional
    /// factor searches consume work and do not restore the attempt's budget.
    fn quotient_add(&mut self, a: RingQuotient, b: RingQuotient) -> Option<RingQuotient> {
        // Both operands have been expanded, including all inverse guards.
        if a.numerator.terms.is_empty() {
            return Some(b);
        }
        if b.numerator.terms.is_empty() {
            return Some(a);
        }
        let (numerator, denominator) = match (a.denominator, b.denominator) {
            (None, None) => (self.add(a.numerator, b.numerator)?, None),
            (Some(d), None) => {
                let right = self.multiply(&b.numerator, &d)?;
                (self.add(a.numerator, right)?, Some(d))
            }
            (None, Some(d)) => {
                let left = self.multiply(&a.numerator, &d)?;
                (self.add(left, b.numerator)?, Some(d))
            }
            (Some(d), Some(e)) => {
                if self.same_polynomial(&d, &e)? {
                    (self.add(a.numerator, b.numerator)?, Some(d))
                } else {
                    if self.native_relations {
                        if let Some(factor) = self.divide_exact_polynomial(&e, &d) {
                            let left = self.multiply(&a.numerator, &factor)?;
                            return Some(RingQuotient::new(self.add(left, b.numerator)?, Some(e)));
                        }
                        if let Some(factor) = self.divide_exact_polynomial(&d, &e) {
                            let right = self.multiply(&b.numerator, &factor)?;
                            return Some(RingQuotient::new(self.add(a.numerator, right)?, Some(d)));
                        }
                    }
                    let left = self.multiply(&a.numerator, &e)?;
                    let right = self.multiply(&b.numerator, &d)?;
                    let numerator = self.add(left, right)?;
                    if numerator.terms.is_empty() {
                        return Some(RingQuotient::polynomial(numerator));
                    }
                    (numerator, Some(self.multiply(&d, &e)?))
                }
            }
        };
        Some(RingQuotient::new(numerator, denominator))
    }

    /// Multiply admitted quotients, with optional native cross-factor cancellation.
    /// When native relations are enabled and a denominator is explicit, clone the
    /// operands and try dividing each numerator by the opposite denominator in order.
    /// A failed division leaves that operand unchanged but can spend work. Delegate
    /// the resulting pair to the admitted product helper. No new nonzero witness is
    /// sought here; denominator removal uses the existing domain proof.
    fn quotient_multiply(&mut self, a: &RingQuotient, b: &RingQuotient) -> Option<RingQuotient> {
        if self.native_relations && (a.denominator.is_some() || b.denominator.is_some()) {
            let (mut a, mut b) = (a.clone(), b.clone());
            if let Some(denominator) = &b.denominator
                && let Some(reduced) = self.divide_exact_polynomial(&a.numerator, denominator)
            {
                a.numerator = reduced;
                b.denominator = None;
            }
            if let Some(denominator) = &a.denominator
                && let Some(reduced) = self.divide_exact_polynomial(&b.numerator, denominator)
            {
                b.numerator = reduced;
                a.denominator = None;
            }
            return self.quotient_multiply_admitted(&a, &b);
        }
        self.quotient_multiply_admitted(a, b)
    }

    /// Form a product from already domain-checked operands under the polynomial caps.
    /// Compute the numerator first. If it vanishes, return zero with denominator one;
    /// otherwise clone a sole explicit denominator or multiply the two denominators.
    /// The nonzero product proof is inherited from operand admission. No unconditional
    /// entry poll is added here; polynomial arithmetic controls charging and None.
    fn quotient_multiply_admitted(
        &mut self,
        a: &RingQuotient,
        b: &RingQuotient,
    ) -> Option<RingQuotient> {
        let numerator = self.multiply(&a.numerator, &b.numerator)?;
        if numerator.terms.is_empty() {
            return Some(RingQuotient::polynomial(numerator));
        }
        let denominator = match (&a.denominator, &b.denominator) {
            (None, None) => None,
            (Some(d), None) | (None, Some(d)) => Some(d.clone()),
            (Some(d), Some(e)) => Some(self.multiply(d, e)?),
        };
        Some(RingQuotient::new(numerator, denominator))
    }

    /// Scale an admitted numerator exactly and normalize a resulting zero quotient.
    /// Retain its existing nonzero denominator unless the numerator cancels. The
    /// polynomial scale path checks the rational coefficient and spends its usual
    /// work; zero scaling is safe only after the operand's domain was admitted.
    fn quotient_scale(&mut self, value: RingQuotient, scale: Rational) -> Option<RingQuotient> {
        Some(RingQuotient::new(
            self.scale(value.numerator, scale)?,
            value.denominator,
        ))
    }

    /// Try to extract a rational value from a domain-admitted quotient.
    /// If unit-circle pairs are recorded, first reduce cloned numerator and denominator
    /// with those relations, then apply proportional-coefficient projection. Otherwise
    /// project the original representation. Relations, denominator validity and atom
    /// identity must belong to this attempt; no numerical refinement or fact publication
    /// occurs here. None leaves rationality undecided and may also reflect spent limits.
    fn project_quotient(&mut self, value: &RingQuotient) -> Option<Rational> {
        if !self.unit_circle_pairs.is_empty() {
            let numerator = self.reduce_unit_circle_pairs(value.numerator.clone())?;
            let denominator = match &value.denominator {
                Some(denominator) => Some(self.reduce_unit_circle_pairs(denominator.clone())?),
                None => None,
            };
            return self.project_reduced_quotient(&RingQuotient::new(numerator, denominator));
        }
        self.project_reduced_quotient(value)
    }

    /// Recognize zero, a rational constant, or proportional numerator/denominator terms.
    /// Charge on entry, then accept an empty numerator using prior denominator admission.
    /// For an explicit denominator, require equal term counts, form the ratio of leading
    /// stored coefficients, enforce its coefficient cap, and check every matching
    /// monomial and scaled coefficient with a charge. This is sufficient, not necessary,
    /// for a rational value when atoms satisfy additional relations. Normalized nonzero
    /// coefficients and a nonzero denominator are caller invariants. None is not proof
    /// of irrationality; temporary coefficient arithmetic is not a total-memory bound.
    fn project_reduced_quotient(&mut self, value: &RingQuotient) -> Option<Rational> {
        self.charge()?;
        if value.numerator.terms.is_empty() {
            return Some(Rational::zero());
        }
        let Some(denominator) = &value.denominator else {
            return match value.numerator.terms.as_slice() {
                [(monomial, coefficient)] if monomial.is_empty() => Some(coefficient.clone()),
                _ => None,
            };
        };
        if value.numerator.terms.len() != denominator.terms.len() {
            return None;
        }
        let (_, a) = value.numerator.terms.first()?;
        let (_, b) = denominator.terms.first()?;
        // Coefficients are nonzero and size-checked at insertion. Division
        // has at most twice their bits temporarily, like multiplication.
        self.charge()?;
        let ratio = a * b.clone().inverse().ok()?;
        if !Self::coefficient_fits(&ratio) {
            return None;
        }
        for ((a, x), (b, y)) in value.numerator.terms.iter().zip(&denominator.terms) {
            self.charge()?;
            if a != b || x != &(y * &ratio) {
                return None;
            }
        }
        Some(ratio)
    }

    /// Expand an authoritative graph node into this attempt's exact quotient cache.
    /// Require positive depth even on a hit. A hit charges and polls before cloning a
    /// completed quotient; it spends no node visit and can succeed with no nodes left.
    /// A miss expands normally, then tries recovery only if a node was available on
    /// entry and child depth is positive. Both routes share spent work and partial local
    /// state. A seen root relation additionally reduces Square/Multiply results before
    /// insertion. Only completed results enter the live-pointer-keyed cache, using the
    /// same atom table and strategy flags; failed children may leave other completed
    /// entries. Entries are bounded by the node allowance, not by a byte budget.
    ///
    /// The graph and recognition strategy must remain compatible for this attempt.
    /// There is no cross-query memo here and no publication of a queried root fact.
    /// None combines unsupported recognition, unproved domains, limits and cancellation.
    fn expand_quotient(&mut self, value: &'a Computable, depth: usize) -> Option<RingQuotient> {
        if depth == 0 {
            return None;
        }
        let identity = Arc::as_ptr(&value.internal);
        if self.quotients.contains_key(&identity) {
            // A hit reuses a completed exact projection. Charge and check
            // cancellation before cloning it, without expanding this node
            // again. Atom IDs belong to this same recognition attempt.
            self.charge()?;
            return self.quotients.get(&identity).cloned();
        }
        let node_available = self.nodes_left != 0;
        let quotient = match self.expand_quotient_uncached(value, depth) {
            Some(quotient) => quotient,
            // Retry arithmetic only after the existing path declines. Its
            // children must already have completed domain-checked projections.
            // Reuse the remaining budget, including the parent's node debit;
            // neither cached children nor reduction bypass child depth zero.
            None if node_available && depth > 1 => self.recover_quotient(value, depth)?,
            None => return None,
        };
        let quotient = if self.root_relation_seen
            && matches!(
                value.internal.approximation,
                Approximation::Square(_) | Approximation::Multiply(..)
            ) {
            self.reduce_native_relation_proof(quotient, NativeReduction::Roots)?
        } else {
            quotient
        };
        // At most MAX_NODES entries: each miss consumes a node before it can
        // succeed. Failed or partial expansions are never inserted.
        self.quotients.insert(identity, quotient.clone());
        Some(quotient)
    }

    /// Perform quotient admission for one cache miss, in recipe evaluation order.
    /// The wrapper supplies positive depth. Native mode first reuses retained rational
    /// proofs, then recognizes guarded parameter/scalar-product atoms; other
    /// non-arithmetic recipes delegate to polynomial atom admission. Arithmetic spends
    /// an entry charge and node visit, expands left before right, and checks all three
    /// linear-combination children even when later scaling erases a term. Offset
    /// magnitudes must be below the coefficient-bit cap before dyadic materialization.
    ///
    /// An inverse expands its child before using an immediate nonzero sign or a local
    /// admitted-quotient nonzero proof; zero and unproved divisors decline. Opaque leaves
    /// otherwise retain the polynomial path's defined-real precondition. This is not a
    /// general imported-domain validator. None does not roll back spent work or local
    /// child projections, and does not distinguish their reasons for declining.
    fn expand_quotient_uncached(
        &mut self,
        value: &'a Computable,
        depth: usize,
    ) -> Option<RingQuotient> {
        if self.native_relations
            && let Some(rational) = value.retained_rational_reduction()
        {
            self.charge()?;
            self.nodes_left = self.nodes_left.checked_sub(1)?;
            return Self::constant(rational).map(RingQuotient::polynomial);
        }
        if self.native_relations
            && (self.is_parameter_atom(value)?
                || matches!(
                    value.internal.approximation,
                    Approximation::Multiply(..) | Approximation::Square(_)
                ) && self.scalar_product_shape(value, 8)?)
        {
            self.charge()?;
            self.nodes_left = self.nodes_left.checked_sub(1)?;
            return self
                .native_relation_atom(std::borrow::Cow::Borrowed(value), depth - 1)
                .map(RingQuotient::polynomial);
        }
        // Reuse the polynomial leaf/atom admission, with its own single node
        // charge, for every representation without rational arithmetic here.
        if !matches!(
            value.internal.approximation,
            Approximation::Add(..)
                | Approximation::Multiply(..)
                | Approximation::Square(..)
                | Approximation::Negate(..)
                | Approximation::Offset(..)
                | Approximation::LinearCombination3(..)
                | Approximation::Inverse(..)
        ) {
            return self.expand(value, depth).map(RingQuotient::polynomial);
        }
        self.charge()?;
        if depth == 0 {
            return None;
        }
        self.nodes_left = self.nodes_left.checked_sub(1)?;
        match &value.internal.approximation {
            Approximation::Add(a, b) => {
                let a = self.expand_quotient(a, depth - 1)?;
                let b = self.expand_quotient(b, depth - 1)?;
                self.quotient_add(a, b)
            }
            Approximation::Multiply(a, b) => {
                let a = self.expand_quotient(a, depth - 1)?;
                let b = self.expand_quotient(b, depth - 1)?;
                self.quotient_multiply(&a, &b)
            }
            Approximation::Square(child) => {
                let child = self.expand_quotient(child, depth - 1)?;
                self.quotient_multiply(&child, &child)
            }
            Approximation::Negate(child) => {
                let child = self.expand_quotient(child, depth - 1)?;
                self.quotient_scale(child, Rational::from(-1))
            }
            Approximation::Offset(child, shift) => {
                if u64::from(shift.unsigned_abs()) >= Self::MAX_COEFFICIENT_BITS {
                    return None;
                }
                let child = self.expand_quotient(child, depth - 1)?;
                self.quotient_scale(child, Computable::power_of_two_rational(*shift))
            }
            Approximation::LinearCombination3(combination) => {
                let mut sum = RingQuotient::polynomial(RingPolynomial::default());
                for (child, scale) in combination.coefficients.iter().zip(&combination.values) {
                    let child = self.expand_quotient(child, depth - 1)?;
                    let term = self.quotient_scale(child, scale.clone())?;
                    sum = self.quotient_add(sum, term)?;
                }
                Some(sum)
            }
            Approximation::Inverse(child) => {
                let quotient = self.expand_quotient(child, depth - 1)?;
                match child.immediate_sign() {
                    Some(RealSign::Zero) => return None,
                    Some(RealSign::Positive | RealSign::Negative) => {}
                    None => {
                        self.admitted_quotient_nonzero(&quotient)?;
                    }
                }
                if quotient.numerator.terms.is_empty() {
                    return None;
                }
                self.invert_admitted_quotient(quotient)
            }
            _ => unreachable!("non-arithmetic forms were handled by polynomial admission"),
        }
    }

    /// Invert a quotient already proved defined and nonzero by its caller.
    /// Native mode first tries a quadratic reciprocal of the numerator. A lone scaled
    /// square-root atom with a retained relation r²=N/H next uses 1/(c*r)=r*H/(c*N),
    /// optionally compacting the resulting denominator by another quadratic reciprocal.
    /// Otherwise swap numerator and denominator, supplying one for an omitted original
    /// denominator. Optional failed recognizers retain their work debits. Atom indices,
    /// parallel relation tables and square-root relation powers must be valid; this
    /// helper itself does not reject a zero value or independently prove a divisor.
    fn invert_admitted_quotient(&mut self, quotient: RingQuotient) -> Option<RingQuotient> {
        if self.native_relations
            && let Some(reciprocal) = self.reciprocal_quadratic_polynomial(&quotient.numerator)
        {
            let numerator = match &quotient.denominator {
                Some(denominator) => self.multiply(&reciprocal, denominator)?,
                None => reciprocal,
            };
            return Some(RingQuotient::polynomial(numerator));
        }
        // Both callers have already proved this value nonzero and expanded
        // every inverse it contains. A retained r²=N/H permits 1/r=r*H/N;
        // N is nonzero because r and H have nonzero witnesses.
        if self.native_relations
            && quotient.denominator.is_none()
            && let [(monomial, coefficient)] = quotient.numerator.terms.as_slice()
            && let [atom] = monomial.as_slice()
        {
            self.charge()?;
            let index = usize::from(*atom);
            if matches!(
                self.atoms[index].internal.approximation,
                Approximation::Sqrt(_)
                    | Approximation::Constant(SharedConstant::Sqrt2 | SharedConstant::Sqrt3)
            ) && let Some(relation) = self.relations[index].clone()
            {
                let root = RingPolynomial {
                    terms: vec![(vec![*atom], Rational::one())],
                };
                let numerator = match relation.value.denominator {
                    Some(denominator) => self.multiply(&root, &denominator)?,
                    None => root,
                };
                let denominator = self.scale(relation.value.numerator, coefficient.clone())?;
                // The root's radicand can itself lie in an admitted quadratic
                // field. Compact that already nonzero denominator before
                // repeated component squares raise its polynomial degree.
                if let Some(reciprocal) = self.reciprocal_quadratic_polynomial(&denominator) {
                    return self
                        .multiply(&numerator, &reciprocal)
                        .map(RingQuotient::polynomial);
                }
                return Some(RingQuotient::new(numerator, Some(denominator)));
            }
        }
        Some(RingQuotient {
            numerator: quotient
                .denominator
                .unwrap_or(Self::constant(Rational::one())?),
            denominator: Some(quotient.numerator),
        })
    }
}

/// Query guarded quotient proofs or rejection-only conditional domain identities.
impl Computable {
    #[cfg(feature = "serde")]
    /// Refute an imported argument domain using a conditional rational identity only.
    /// Assume opaque leaves denote reals, expand an arithmetic root, and apply `invalid`
    /// to its rational projection after the final stop check. If a leaf is undefined,
    /// the argument is already invalid; otherwise the projected value can refute its
    /// domain. The caller must supply a sound rejection predicate. This method never
    /// publishes the conditional rational or its sign, even if the predicate rejects it.
    ///
    /// Non-arithmetic roots, failed recognition, limits, cancellation or a false
    /// predicate return false, which does not validate the domain. The closure is
    /// called at most once and its own cost is outside recognition's work accounting.
    fn bounded_domain_contradiction(&self, invalid: impl FnOnce(&Rational) -> bool) -> bool {
        // The caller already tried exact leaves and defined rational forms.
        // Every other non-arithmetic root expands to one opaque atom, which
        // cannot project to a rational. Avoid allocating that doomed attempt.
        if !matches!(
            self.internal.approximation,
            Approximation::Add(..)
                | Approximation::Multiply(..)
                | Approximation::Square(..)
                | Approximation::Negate(..)
                | Approximation::Offset(..)
                | Approximation::LinearCombination3(..)
                | Approximation::Inverse(..)
        ) {
            return false;
        }
        let mut recognition = RingRecognition::new(self);
        let Some(quotient) = recognition.expand_quotient(self, RingRecognition::MAX_DEPTH) else {
            return false;
        };
        let Some(rational) = recognition.project_quotient(&quotient) else {
            return false;
        };
        !should_stop(&self.signal) && invalid(&rational)
    }

    /// Try an exact rational projection of a defined-real graph using guarded quotients.
    /// Cancel formal polynomial terms and divide only with nonzero witnesses, including
    /// already retained signs. Opaque leaves must already denote defined real values;
    /// use the rejection-only import path for conditional domain contradictions.
    /// Success publishes the root's exact sign and a rational-quotient dispatch event,
    /// not a retained rational tower or numerical sample. None conflates unrecognized
    /// forms, unproved divisors, resource limits and cancellation; a fresh call can retry.
    pub(crate) fn bounded_quotient_rational(&self) -> Option<Rational> {
        self.bounded_quotient_projection(false, false)
    }

    /// Try a guarded quotient proof that actually uses shared √2 or √3 square reduction.
    /// Use the ordinary quotient envelope with the two named root-square identities.
    /// A rational result reached without using either reduction still declines; this
    /// keeps the specialized dispatch meaningful. Defined-real inputs are required.
    /// Only completed, uncancelled success publishes the exact root sign and the
    /// root-square-quotient trace; no numerical sample or rational tower is retained.
    pub(crate) fn bounded_root_square_rational(&self) -> Option<Rational> {
        self.bounded_quotient_projection(true, false)
    }

    /// Evaluate an arithmetic graph whose admitted leaves must all be rational.
    /// Reuse quotient limits but disallow opaque atoms and require nonzero inverse
    /// arguments. Both multiplication children and every linear-combination child are
    /// checked before zero absorption, so cancellation cannot hide an undefined leaf.
    /// Success publishes the queried sign and rational-quotient event; None means this
    /// bounded rational-only route did not establish a value, not that it is undefined.
    fn bounded_defined_rational(&self) -> Option<Rational> {
        self.bounded_quotient_projection(false, true)
    }

    /// Run a fresh quotient strategy and publish only its completed rational proof.
    /// Set the root-square and rational-only flags once, expand under MAX_DEPTH, project,
    /// and check cancellation before publishing. Root-square mode additionally requires
    /// that its relation was used. Success stores only the root's exact sign and emits
    /// the selected dispatch event; query-local atoms, quotients and work state are then
    /// dropped. No policy-specific approximation is involved and failures are not cached.
    /// The shared node/work/degree/coefficient caps are distinct limits, not a bound on
    /// all allocation, elapsed time or total arithmetic cost.
    fn bounded_quotient_projection(
        &self,
        root_squares: bool,
        rational_only: bool,
    ) -> Option<Rational> {
        let mut recognition = RingRecognition::new(self);
        recognition.root_squares = root_squares;
        recognition.rational_only = rational_only;
        let quotient = recognition.expand_quotient(self, RingRecognition::MAX_DEPTH)?;
        let rational = recognition.project_quotient(&quotient)?;
        if should_stop(&self.signal) || (root_squares && !recognition.root_square_used) {
            return None;
        }
        self.internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(rational.sign()));
        if root_squares {
            crate::trace_dispatch!("computable", "normal_form", "root-square-quotient");
        } else {
            crate::trace_dispatch!("computable", "normal_form", "rational-quotient");
        }
        Some(rational)
    }
}
