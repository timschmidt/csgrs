// Exact division for the additional native-relation attempt. Keep the
// remainder in the same graded monomial order used by division, so each step
// can take its leading term without rescanning every remaining coefficient.
// The authoritative polynomial representation and older attempts stay intact.
/// Compact admitted quadratic reciprocals and perform exact formal division for native recognition.
impl<'a> RingRecognition<'a> {
    // For an admitted root with rational square, a nonzero rational norm gives
    // 1/(a+b*r) = (a-b*r)/(a²-b²*q), where r²=q. This keeps rational
    // parameter coefficients compact before they enter larger denominators.
    // A zero or oversized norm declines; no nonzero conjugate is assumed.
    /// Compact the inverse of a+b*r using an admitted rational square r²=q and nonzero norm.
    /// Require at most a constant and one root monomial, a seen root relation, and
    /// a denominator-free rational square relation. Return (a-b*r)/(a²-b²*q) as a
    /// polynomial with rational coefficients after charged exact arithmetic and
    /// size checks. This sufficient route can decline a nonzero value when its norm
    /// vanishes, as for 1+sqrt(1). Other shapes, missing relations, zero norm or
    /// exceeded work/size/stop limits return None. Valid atom IDs, normalized terms
    /// and root-domain admission are caller invariants; no root is approximated.
    fn reciprocal_quadratic_polynomial(
        &mut self,
        value: &RingPolynomial,
    ) -> Option<RingPolynomial> {
        if value.terms.len() > 2 || !self.root_relation_seen {
            return None;
        }
        let mut constant = Rational::zero();
        let mut radical = None;
        for (term, coefficient) in &value.terms {
            self.charge()?;
            match term.as_slice() {
                [] => constant = coefficient.clone(),
                [atom] if radical.is_none() => radical = Some((*atom, coefficient.clone())),
                _ => return None,
            }
        }
        let (atom, scale) = radical?;
        if !matches!(
            self.atoms[usize::from(atom)].internal.approximation,
            Approximation::Sqrt(_)
                | Approximation::Constant(SharedConstant::Sqrt2 | SharedConstant::Sqrt3)
        ) {
            return None;
        }
        let relation = self.relations[usize::from(atom)].as_ref()?;
        if relation.power != 2 || relation.value.denominator.is_some() {
            return None;
        }
        let square = match relation.value.numerator.terms.as_slice() {
            [] => Rational::zero(),
            [(term, coefficient)] if term.is_empty() => coefficient.clone(),
            _ => return None,
        };
        self.charge()?;
        let constant_square = &constant * &constant;
        self.charge()?;
        let scale_square = &scale * &scale;
        if !Self::coefficient_fits(&constant_square) || !Self::coefficient_fits(&scale_square) {
            return None;
        }
        self.charge()?;
        let radical_square = scale_square * square;
        if !Self::coefficient_fits(&radical_square) {
            return None;
        }
        self.charge()?;
        let norm = constant_square - radical_square;
        if !Self::coefficient_fits(&norm) {
            return None;
        }
        self.charge()?;
        let inverse = norm.inverse().ok()?;
        let conjugate = RingPolynomial {
            terms: value
                .terms
                .iter()
                .map(|(term, coefficient)| {
                    (
                        term.clone(),
                        if term.is_empty() {
                            coefficient.clone()
                        } else {
                            -coefficient.clone()
                        },
                    )
                })
                .collect(),
        };
        self.scale(conjugate, inverse)
    }

    /// Return the exact formal polynomial quotient only when long division leaves zero remainder.
    /// Use total degree then lexicographic sorted atom IDs as a multiplicative monomial
    /// order. Sort a local remainder once and use its final term thereafter; the
    /// returned quotient uses the normal stored order. This is division over formal
    /// rational polynomials, not proof that the divisor is nonzero at selected real
    /// atom values. Zero divisor, indivisible leading term, or a degree/coefficient/
    /// term/work/stop limit declines. Even zero numerator first needs a nonempty
    /// divisor and its invertible leading coefficient. Inputs must be normalized,
    /// with nonzero coefficients and sorted atom lists; no atom is evaluated.
    /// Charges cover explicit visits and arithmetic, not individual sort comparisons,
    /// all coefficient internals or peak allocation.
    fn divide_graded_polynomial(
        &mut self,
        numerator: &RingPolynomial,
        denominator: &RingPolynomial,
    ) -> Option<RingPolynomial> {
        self.charge()?;
        let leading = self.leading_polynomial_term(denominator)?;
        let (divisor_term, divisor_coefficient) = &denominator.terms[leading];
        self.charge()?;
        let inverse = divisor_coefficient.clone().inverse().ok()?;
        let mut remainder = numerator.terms.clone();
        // At most 64 terms, each with at most 12 small atom IDs. Admission of
        // each term pays for this bounded initial ordering; no DAG is compared.
        for _ in &remainder {
            self.charge()?;
        }
        remainder.sort_unstable_by(|(a, _), (b, _)| (a.len(), a).cmp(&(b.len(), b)));
        let mut quotient = RingPolynomial::default();
        while let Some((term, coefficient)) = remainder.last() {
            self.charge()?;
            let mut divisor = divisor_term.iter().peekable();
            let mut factors = Vec::with_capacity(term.len());
            for atom in term {
                self.charge()?;
                if divisor.peek() == Some(&atom) {
                    divisor.next();
                } else {
                    factors.push(*atom);
                }
            }
            if divisor.next().is_some() {
                return None;
            }
            self.charge()?;
            let scale = coefficient * &inverse;
            if !Self::coefficient_fits(&scale) {
                return None;
            }
            self.insert(&mut quotient, factors.clone(), scale.clone())?;
            for (monomial, coefficient) in &denominator.terms {
                self.charge()?;
                if factors.len() + monomial.len() > self.degree_limit() {
                    return None;
                }
                let mut product = factors.clone();
                product.extend_from_slice(monomial);
                product.sort_unstable();
                let coefficient = coefficient * &scale;
                if !Self::coefficient_fits(&coefficient) {
                    return None;
                }
                // This is the same bounded term insertion and coefficient
                // admission as RingPolynomial, with the graded ordering.
                self.charge()?;
                match remainder.binary_search_by(|(term, _)| {
                    (term.len(), term).cmp(&(product.len(), &product))
                }) {
                    Ok(index) => {
                        self.charge()?;
                        let difference = &remainder[index].1 - coefficient;
                        if !Self::coefficient_fits(&difference) {
                            return None;
                        }
                        if difference.is_zero() {
                            remainder.remove(index);
                        } else {
                            remainder[index].1 = difference;
                        }
                    }
                    Err(index) => {
                        if remainder.len() == Self::MAX_TERMS {
                            return None;
                        }
                        remainder.insert(index, (product, -coefficient));
                    }
                }
            }
        }
        Some(quotient)
    }
}
