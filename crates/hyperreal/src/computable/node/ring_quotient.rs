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
        let quotient = self.expand_quotient_uncached(value, depth)?;
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
                        if self.project_quotient(&quotient)?.is_zero() {
                            return None;
                        }
                    }
                }
                if quotient.numerator.terms.is_empty() {
                    return None;
                }
                Some(RingQuotient {
                    numerator: match quotient.denominator {
                        Some(denominator) => denominator,
                        None => Self::constant(Rational::one())?,
                    },
                    denominator: Some(quotient.numerator),
                })
            }
            _ => unreachable!("non-arithmetic forms were handled by polynomial admission"),
        }
    }
}

impl Computable {
    pub(crate) fn bounded_quotient_rational(&self) -> Option<Rational> {
        let mut recognition = RingRecognition::new(self);
        let quotient = recognition.expand_quotient(self, RingRecognition::MAX_DEPTH)?;
        let rational = recognition.project_quotient(&quotient)?;
        if should_stop(&self.signal) {
            return None;
        }
        self.internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(rational.sign()));
        crate::trace_dispatch!("computable", "normal_form", "rational-quotient");
        Some(rational)
    }
}
