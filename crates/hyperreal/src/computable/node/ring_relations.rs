/// Extend a query's guarded quotient arithmetic with selected exact scalar identities.
/// Admit sqrt(q)²=q only after establishing rational q≥0, and pi*InvPi=1 from
/// the built-in constants' definitions. Opaque native dependencies still require
/// prior defined-real admission; this is not a general domain-validation kernel.
impl<'a> RingRecognition<'a> {
    /// Admit a principal square root whose radicand has an exact nonnegative rational projection.
    /// Expand the radicand under the same quotient budget and reject negative or unproved
    /// values. After a charge, fold a perfect rational square to its nonnegative root
    /// and mark additional_relation_used. Otherwise obtain an opaque root atom, then
    /// reuse or append its (atom ID, rational square) entry; registering the entry alone
    /// does not mark the relation used. Later multiplication must actually reduce it.
    ///
    /// The caller supplies value=Sqrt(radicand), an enabled rational_squares table and
    /// compatible ordinary atom admission returning one degree-one monomial. This helper
    /// does not verify the recipe/argument correspondence or validate arbitrary opaque
    /// radicand dependencies. None combines domain/projection failure and spent limits;
    /// local atoms, child projections or usage state may already have changed. No root
    /// sample or global fact is published, and perfect-root arithmetic has no separate
    /// internal cancellation poll supplied by this method.
    fn rational_sqrt_atom(
        &mut self,
        value: &'a Computable,
        radicand: &'a Computable,
        depth: usize,
    ) -> Option<RingPolynomial> {
        let radicand = self.expand_quotient(radicand, depth)?;
        let square = self.project_quotient(&radicand)?;
        if square.sign() == Sign::Minus {
            return None;
        }
        self.charge()?;
        if let Some(root) = square.perfect_nth_root(2) {
            self.additional_relation_used = true;
            return Self::constant(root);
        }
        let atom = self.atom(value, depth)?;
        let index = atom.terms[0].0[0];
        for entry in 0..self.rational_squares.as_ref()?.len() {
            self.charge()?;
            if self.rational_squares.as_ref()?[entry].0 == index {
                return Some(atom);
            }
        }
        self.charge()?;
        self.rational_squares.as_mut()?.push((index, square));
        Some(atom)
    }

    /// Reduce registered rational root squares and exact built-in pi/inverse-pi pairs.
    /// The caller supplies a sorted monomial, valid IDs and a compatible rational_squares
    /// table. Scan adjacent equal IDs for a registered square, checking the incoming
    /// coefficient before multiplying by that square and checking the result afterwards.
    /// Thus an oversized incoming coefficient declines even when the square would shrink
    /// it. A completed pair sets additional_relation_used and is removed from the term.
    ///
    /// Then repeatedly scan the whole remaining monomial for built-in Pi and InvPi,
    /// which need not be adjacent, removing one pair per scan because pi*(1/pi)=1.
    /// This cancellation changes no coefficient and needs no approximate nonzero test.
    /// Scans and square products are charged. On None the coefficient, compacted prefix,
    /// monomial or usage flag may already be changed; discard that candidate. An empty
    /// term has no charge or stop poll. Successful pair removal does not replace the
    /// caller's final stored-coefficient guard; the pi-only path does not check it.
    fn reduce_additional_relations(
        &mut self,
        monomial: &mut Vec<u8>,
        coefficient: &mut Rational,
    ) -> Option<()> {
        let mut read = 0;
        let mut write = 0;
        while read < monomial.len() {
            self.charge()?;
            let atom = monomial[read];
            let mut square = None;
            if monomial.get(read + 1) == Some(&atom) {
                for index in 0..self.rational_squares.as_ref()?.len() {
                    self.charge()?;
                    let entry = &self.rational_squares.as_ref()?[index];
                    if entry.0 == atom {
                        square = Some(entry.1.clone());
                        break;
                    }
                }
            }
            if let Some(square) = square {
                // The incoming product may temporarily exceed the stored
                // coefficient limit. Check it before multiplying by another
                // admitted coefficient, bounding this temporary to twice it.
                if !Self::coefficient_fits(coefficient) {
                    return None;
                }
                self.charge()?;
                *coefficient = &*coefficient * square;
                if !Self::coefficient_fits(coefficient) {
                    return None;
                }
                self.additional_relation_used = true;
                read += 2;
            } else {
                monomial[write] = atom;
                write += 1;
                read += 1;
            }
        }
        monomial.truncate(write);

        // Sorted atom IDs need not put pi next to its reciprocal. Search the
        // bounded monomial, removing one certified pair on each iteration.
        loop {
            let (mut pi, mut inverse) = (None, None);
            for (index, atom) in monomial.iter().enumerate() {
                self.charge()?;
                match &self.atoms[usize::from(*atom)].internal.approximation {
                    Approximation::Constant(SharedConstant::Pi) => pi = Some(index),
                    Approximation::Constant(SharedConstant::InvPi) => inverse = Some(index),
                    _ => {}
                }
            }
            let (Some(pi), Some(inverse)) = (pi, inverse) else {
                break;
            };
            monomial.remove(pi.max(inverse));
            monomial.remove(pi.min(inverse));
            self.additional_relation_used = true;
        }
        Some(())
    }
}

/// Run ordinary or extended exact relation attempts with distinct completion gates.
impl Computable {
    /// Try a rational quotient proof using admitted roots, built-in reciprocals and trig relations.
    /// Start a fresh ordinary-limit attempt with shared sqrt(2)/sqrt(3) squares, rational
    /// root admission and trig argument recognition enabled. The latter also supplies
    /// exact rational trig values and certified unit-circle relations. A completed
    /// projection succeeds only if additional_relation_used was set; merely registering
    /// a nonsquare root, or using only a shared-root square, is insufficient for this
    /// strategy. The flag can be set by rational root folding, additional square/pi
    /// reduction, rational trig projection or unit-circle reduction.
    ///
    /// Defined native inputs and compatible inherited facts are preconditions. After
    /// the final cancellation check, store the exact root sign and emit the
    /// rational-root-reciprocal dispatch event. No numerical sample or retained rational
    /// tower is created. None supplies no negative fact: unsupported proof shapes,
    /// unproved domains, limits and cancellation all permit a later fresh retry.
    pub(crate) fn bounded_relation_rational(&self) -> Option<Rational> {
        let mut recognition = RingRecognition::new(self);
        recognition.root_squares = true;
        recognition.trig_arguments = Some(Vec::new());
        recognition.rational_squares = Some(Vec::new());
        let quotient = recognition.expand_quotient(self, RingRecognition::MAX_DEPTH)?;
        let rational = recognition.project_quotient(&quotient)?;
        if should_stop(&self.signal) || !recognition.additional_relation_used {
            return None;
        }
        self.internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(rational.sign()));
        crate::trace_dispatch!("computable", "normal_form", "rational-root-reciprocal");
        Some(rational)
    }

    /// Retry the relation strategy with degree twelve and 32,768 work charges.
    /// Keep the node, depth, atom, term and coefficient limits unchanged. Unlike the
    /// ordinary relation query, any completed rational projection is accepted without
    /// a relation-use flag. After quotient expansion (including its normal recoveries)
    /// declines, an Add root may try late factored-zero recovery with the same remaining
    /// work and child proofs. Projection failure after successful expansion does not
    /// invoke this fallback. The caller chooses when to attempt this more expensive
    /// strategy after cheaper exact proof routes; it does not run those routes itself.
    ///
    /// Require defined native inputs. A final stop check precedes exact sign publication
    /// and the extended-relation dispatch event; no numerical sample or rational tower
    /// is retained. Query-local state is dropped and failure is not cached. None combines
    /// unsuccessful recognition, unproved domains and limits. These size/work counters
    /// are not a total arithmetic, elapsed-time or allocation bound, and no approximation
    /// policy is consumed.
    pub(crate) fn bounded_extended_relation_rational(&self) -> Option<Rational> {
        let mut recognition = RingRecognition::new(self);
        recognition.root_squares = true;
        recognition.trig_arguments = Some(Vec::new());
        recognition.rational_squares = Some(Vec::new());
        recognition.extended_degree = true;
        recognition.operations_left = RingRecognition::EXTENDED_OPERATIONS;
        let quotient = recognition
            .expand_quotient(self, RingRecognition::MAX_DEPTH)
            .or_else(|| {
                // Preserve every original success and recovery. Only after
                // they decline may a root sum cancel common factors without
                // expanding their full product. Reuse the same completed
                // projections, inverse witnesses and remaining work budget.
                if let Approximation::Add(a, b) = &self.internal.approximation {
                    recognition.recover_factored_zero::<true>(a, b, RingRecognition::MAX_DEPTH - 1)
                } else {
                    None
                }
            })?;
        let rational = recognition.project_quotient(&quotient)?;
        if should_stop(&self.signal) {
            return None;
        }
        self.internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(rational.sign()));
        crate::trace_dispatch!("computable", "normal_form", "extended-relation");
        Some(rational)
    }
}
