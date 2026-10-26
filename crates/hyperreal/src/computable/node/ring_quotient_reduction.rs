// Optional recovery after quotient arithmetic declines. Existing successful
// expansions never pay for factor search. All work uses the same recognizer,
// atom identities, strict divisor witnesses and remaining operation budget.
impl<'a> RingRecognition<'a> {
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

    fn recover_quotient(&mut self, value: &'a Computable, depth: usize) -> Option<RingQuotient> {
        self.recover_reduced_quotient(value).or_else(|| {
            if let Approximation::Add(a, b) = &value.internal.approximation {
                self.recover_factored_zero::<false>(a, b, depth - 1)
            } else {
                None
            }
        })
    }

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
