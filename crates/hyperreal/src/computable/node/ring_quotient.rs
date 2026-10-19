// Rational functions over the same opaque atoms and polynomial arithmetic.
// A denominator is admitted only with an exact nonzero witness. Products of
// those denominators remain nonzero; no polynomial independence is assumed.
#[derive(Clone)]
struct RingQuotient {
    numerator: RingPolynomial,
    // None represents one. Every explicit denominator is proved nonzero at
    // the actual atom values, not merely a nonzero formal polynomial.
    denominator: Option<RingPolynomial>,
}

impl RingQuotient {
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

    fn polynomial(numerator: RingPolynomial) -> Self {
        Self {
            numerator,
            denominator: None,
        }
    }
}

impl<'a> RingRecognition<'a> {
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

    fn quotient_multiply_admitted(&mut self, a: &RingQuotient, b: &RingQuotient) -> Option<RingQuotient> {
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

    fn quotient_scale(&mut self, value: RingQuotient, scale: Rational) -> Option<RingQuotient> {
        Some(RingQuotient::new(
            self.scale(value.numerator, scale)?,
            value.denominator,
        ))
    }

    fn project_quotient(&mut self, value: &RingQuotient) -> Option<Rational> {
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
            && matches!(value.internal.approximation, Approximation::Square(_) | Approximation::Multiply(..))
        {
            self.reduce_native_relation_proof(quotient, NativeReduction::Roots)?
        } else { quotient };
        // At most MAX_NODES entries: each miss consumes a node before it can
        // succeed. Failed or partial expansions are never inserted.
        self.quotients.insert(identity, quotient.clone());
        Some(quotient)
    }

    fn expand_quotient_uncached(
        &mut self,
        value: &'a Computable,
        depth: usize,
    ) -> Option<RingQuotient> {
        if self.native_relations && let Some(rational) = value.retained_rational_reduction() {
            self.charge()?;
            self.nodes_left = self.nodes_left.checked_sub(1)?;
            return Self::constant(rational).map(RingQuotient::polynomial);
        }
        if self.native_relations
            && (self.is_parameter_atom(value)?
                || matches!(value.internal.approximation, Approximation::Multiply(..) | Approximation::Square(_))
                    && self.scalar_product_shape(value, 8)?)
        {
            self.charge()?;
            self.nodes_left = self.nodes_left.checked_sub(1)?;
            return self.native_relation_atom(std::borrow::Cow::Borrowed(value), depth - 1)
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

    fn invert_admitted_quotient(&mut self, quotient: RingQuotient) -> Option<RingQuotient> {
        if self.native_relations
            && let Some(reciprocal) = self.reciprocal_quadratic_polynomial(&quotient.numerator)
        {
            let numerator = match &quotient.denominator {
                Some(denominator) => self.multiply(&reciprocal,denominator)?,
                None => reciprocal,
            };
            return Some(RingQuotient::polynomial(numerator));
        }
        // Both callers have already proved this value nonzero and expanded
        // every inverse it contains. A retained r²=N/H permits 1/r=r*H/N;
        // N is nonzero because r and H have nonzero witnesses.
        if self.native_relations && quotient.denominator.is_none()
            && let [(monomial, coefficient)] = quotient.numerator.terms.as_slice()
            && let [atom] = monomial.as_slice()
        {
            self.charge()?;
            let index = usize::from(*atom);
            if matches!(self.atoms[index].internal.approximation,
                Approximation::Sqrt(_) | Approximation::Constant(SharedConstant::Sqrt2 | SharedConstant::Sqrt3))
                && let Some(relation) = self.relations[index].clone()
            {
                let root = RingPolynomial { terms: vec![(vec![*atom], Rational::one())] };
                let numerator = match relation.value.denominator {
                    Some(denominator) => self.multiply(&root, &denominator)?,
                    None => root,
                };
                let denominator = self.scale(relation.value.numerator, coefficient.clone())?;
                // The root's radicand can itself lie in an admitted quadratic
                // field. Compact that already nonzero denominator before
                // repeated component squares raise its polynomial degree.
                if let Some(reciprocal) = self.reciprocal_quadratic_polynomial(&denominator) {
                    return self.multiply(&numerator, &reciprocal).map(RingQuotient::polynomial);
                }
                return Some(RingQuotient::new(numerator, Some(denominator)));
            }
        }
        Some(RingQuotient {
            numerator: quotient.denominator.unwrap_or(Self::constant(Rational::one())?),
            denominator: Some(quotient.numerator),
        })
    }
}

impl Computable {
    pub(crate) fn bounded_quotient_rational(&self) -> Option<Rational> {
        self.bounded_quotient_projection(false)
    }

    pub(crate) fn bounded_root_square_rational(&self) -> Option<Rational> {
        self.bounded_quotient_projection(true)
    }

    fn bounded_quotient_projection(&self, root_squares: bool) -> Option<Rational> {
        let mut recognition = RingRecognition::new(self);
        recognition.root_squares = root_squares;
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
