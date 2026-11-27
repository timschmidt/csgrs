/// Recover declined quotient arithmetic through exact factor reduction.
/// Ordinary quotient recovery is deferred until expansion fails. Native quotient
/// addition and multiplication can also call exact division proactively. All
/// routes share atom identities, prior divisor witnesses and remaining work.
impl<'a> RingRecognition<'a> {
    /// Select the greatest monomial by total degree, then lexicographic factor order.
    /// Scan every stored term with one charge, preserving the polynomial's separate
    /// lexicographic storage order. Empty input or a declined charge returns None.
    /// Sorted factor lists make this a multiplicative monomial order suitable for
    /// long division; coefficients do not affect the selected index.
    fn leading_polynomial_term(&mut self, polynomial: &RingPolynomial) -> Option<usize> {
        polynomial.terms.first()?;
        let mut leading = 0;
        for (index, (term, _)) in polynomial.terms.iter().enumerate() {
            self.charge()?;
            let largest = &polynomial.terms[leading].0;
            // Total degree followed by lexicographic sorted factor lists is
            // a multiplicative monomial order. Division cancels its largest
            // term each step without changing the stored term ordering.
            if (term.len(), term) > (largest.len(), largest) {
                leading = index;
            }
        }
        Some(leading)
    }

    /// Try exact polynomial division without assuming independent atom values.
    /// Native mode delegates to formal graded division. Otherwise repeatedly cancel the
    /// greatest degree/lexicographic term, requiring the divisor monomial's sorted
    /// factors to be a submultiset and each stored quotient coefficient to fit.
    /// Return a polynomial only when the entire remainder vanishes; a leading term
    /// that cannot divide declines. Empty formal divisors, growth limits and spent
    /// charges also decline, and temporary clones/products are local to this call.
    /// Ordinary multiplication may apply enabled root/additional relations while
    /// subtracting products; with those flags off this is formal long division.
    ///
    /// The inputs must be normalized in this attempt's atom table. This helper does
    /// not prove the divisor nonzero at those atom values: fraction cancellation may
    /// consume it only after separate domain admission. Degree/term caps can reject
    /// an exact mathematical quotient; None is not a nondivisibility certificate.
    fn divide_exact_polynomial(
        &mut self,
        numerator: &RingPolynomial,
        denominator: &RingPolynomial,
    ) -> Option<RingPolynomial> {
        if self.native_relations {
            return self.divide_graded_polynomial(numerator, denominator);
        }
        self.charge()?;
        let leading = self.leading_polynomial_term(denominator)?;
        let (divisor_term, divisor_coefficient) = &denominator.terms[leading];
        self.charge()?;
        let inverse_coefficient = divisor_coefficient.clone().inverse().ok()?;
        let mut remainder = numerator.clone();
        let mut quotient = RingPolynomial::default();
        while !remainder.terms.is_empty() {
            let leading = self.leading_polynomial_term(&remainder)?;
            let (term, coefficient) = &remainder.terms[leading];
            let mut divisor_factors = divisor_term.iter().peekable();
            let mut factors = Vec::with_capacity(term.len());
            for atom in term {
                self.charge()?;
                if divisor_factors.peek() == Some(&atom) {
                    divisor_factors.next();
                } else {
                    factors.push(*atom);
                }
            }
            if divisor_factors.next().is_some() {
                return None;
            }
            self.charge()?;
            // Like coefficient products, division can temporarily use twice
            // the admitted bit count. Reject an oversized stored quotient.
            let scale = coefficient * &inverse_coefficient;
            if !Self::coefficient_fits(&scale) {
                return None;
            }
            self.insert(&mut quotient, factors.clone(), scale.clone())?;
            let factor = RingPolynomial {
                terms: vec![(factors, scale)],
            };
            let product = self.multiply(&factor, denominator)?;
            for (term, coefficient) in product.terms {
                self.charge()?;
                self.insert(&mut remainder, term, -coefficient)?;
            }
        }
        Some(quotient)
    }

    /// Retrieve a completed local quotient and try removing its explicit denominator.
    /// Charge before lookup by live node pointer. A successful exact division returns
    /// a polynomial and true; no denominator or failed optional division returns the
    /// original clone and false, even if the division spent its last work charge.
    /// Missing cache entries or a declined entry charge return None. The original
    /// cache is not overwritten. Its admission proof justifies factor cancellation.
    fn reduce_cached_quotient(&mut self, value: &Computable) -> Option<(RingQuotient, bool)> {
        self.charge()?;
        let quotient = self.quotients.get(&Arc::as_ptr(&value.internal))?.clone();
        // A cached projection has already discharged every inverse guard.
        // A formally nonzero polynomial alone would not suffice here.
        if let Some(denominator) = &quotient.denominator
            && let Some(polynomial) = self.divide_exact_polynomial(&quotient.numerator, denominator)
        {
            return Some((RingQuotient::polynomial(polynomial), true));
        }
        Some((quotient, false))
    }

    /// Retry a declined expansion using the same attempt's remaining proof resources.
    /// First reduce already cached child quotients and retry arithmetic. If that fails
    /// for an Add recipe, try ordinary factored-zero recovery at child depth. The caller
    /// must have admitted this recovery with depth greater than one and a node available
    /// at the original miss; this method does not reset budgets or refund previous work.
    /// Neither route publishes a root fact or bypasses inverse-domain obligations.
    fn recover_quotient(&mut self, value: &'a Computable, depth: usize) -> Option<RingQuotient> {
        self.recover_reduced_quotient(value).or_else(|| {
            if let Approximation::Add(a, b) = &value.internal.approximation {
                self.recover_factored_zero::<false>(a, b, depth - 1)
            } else {
                None
            }
        })
    }

    /// Retry selected arithmetic after cached child fractions become polynomials.
    /// Handle Add, Multiply, Square and LinearCombination3, retrieving every needed
    /// completed child and requiring at least one successful denominator removal.
    /// Linear combinations gather all children before applying scales. Negation,
    /// offsets, inverses and other recipes decline here; in particular no inverse
    /// guard is skipped. Use remaining charges, leave the child cache unchanged, and
    /// return None for missing children, no reduction or unsuccessful retry arithmetic.
    fn recover_reduced_quotient(&mut self, value: &Computable) -> Option<RingQuotient> {
        match &value.internal.approximation {
            Approximation::Add(a, b) => {
                let (a, changed_a) = self.reduce_cached_quotient(a)?;
                let (b, changed_b) = self.reduce_cached_quotient(b)?;
                if !changed_a && !changed_b {
                    return None;
                }
                self.quotient_add(a, b)
            }
            Approximation::Multiply(a, b) => {
                let (a, changed_a) = self.reduce_cached_quotient(a)?;
                let (b, changed_b) = self.reduce_cached_quotient(b)?;
                if !changed_a && !changed_b {
                    return None;
                }
                self.quotient_multiply(&a, &b)
            }
            Approximation::Square(child) => {
                let (child, changed) = self.reduce_cached_quotient(child)?;
                if !changed {
                    return None;
                }
                self.quotient_multiply(&child, &child)
            }
            Approximation::LinearCombination3(combination) => {
                let mut terms = Vec::with_capacity(3);
                let mut changed = false;
                for (child, scale) in combination.coefficients.iter().zip(&combination.values) {
                    let (quotient, reduced) = self.reduce_cached_quotient(child)?;
                    terms.push((quotient, scale));
                    changed |= reduced;
                }
                if !changed {
                    return None;
                }
                let mut sum = RingQuotient::polynomial(RingPolynomial::default());
                for (quotient, scale) in terms {
                    let term = self.quotient_scale(quotient, scale.clone())?;
                    sum = self.quotient_add(sum, term)?;
                }
                Some(sum)
            }
            // In particular, recovery never bypasses an inverse-domain guard.
            _ => None,
        }
    }
}
