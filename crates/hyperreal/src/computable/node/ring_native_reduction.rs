// Bounded substitution of admitted native relations in rational functions.
impl RingRecognition<'_> {
    fn polynomial_powers(
        &mut self,
        value: &RingPolynomial,
        count: usize,
    ) -> Option<Vec<RingPolynomial>> {
        let mut powers = vec![Self::constant(Rational::one())?];
        for power in 0..count {
            self.charge()?;
            powers.push(self.multiply(&powers[power], value)?);
        }
        Some(powers)
    }

    fn substitute_relation(
        &mut self,
        polynomial: RingPolynomial,
        atom: u8,
        power: usize,
        numerator_powers: &[RingPolynomial],
        denominator_powers: &[RingPolynomial],
    ) -> Option<RingPolynomial> {
        let mut result = RingPolynomial::default();
        let maximum = numerator_powers.len() - 1;
        for (monomial, coefficient) in polynomial.terms {
            let mut count = 0;
            let mut residual = Vec::with_capacity(monomial.len());
            for id in monomial {
                self.charge()?;
                if id == atom {
                    count += 1;
                } else {
                    residual.push(id);
                }
            }
            if count % power != 0 {
                residual.extend(std::iter::repeat_n(atom, count % power));
                residual.sort_unstable();
            }
            // Accumulate the substituted monomials directly. Building two
            // temporary polynomials would insert and merge the same terms
            // repeatedly, spending the proof budget on avoidable work.
            for (a, x) in &numerator_powers[count / power].terms {
                for (b, y) in &denominator_powers[maximum - count / power].terms {
                    self.charge()?;
                    if residual.len() + a.len() + b.len() > self.degree_limit() {
                        return None;
                    }
                    let mut monomial = residual.clone();
                    monomial.extend_from_slice(a);
                    monomial.extend_from_slice(b);
                    monomial.sort_unstable();
                    let mut scale = coefficient.clone();
                    for factor in [x, y] {
                        if !factor.is_one() {
                            self.charge()?;
                            scale *= factor;
                            if !Self::coefficient_fits(&scale) {
                                return None;
                            }
                        }
                    }
                    self.insert(&mut result, monomial, scale)?;
                }
            }
        }
        Some(result)
    }

    fn reduce_native_relations(&mut self, value: RingQuotient) -> Option<RingQuotient> {
        self.reduce_native_relation_proof(value, NativeReduction::Quotient)
    }

    fn reduce_native_relation_proof(
        &mut self,
        value: RingQuotient,
        mode: NativeReduction,
    ) -> Option<RingQuotient> {
        let roots_only = mode == NativeReduction::Roots;
        let zero_only = mode == NativeReduction::ZeroResidual;
        let mut numerator = value.numerator;
        let mut denominator = value
            .denominator
            .unwrap_or(Self::constant(Rational::one())?);
        loop {
            if zero_only {
                self.remove_common_monomial_for_zero_proof(&mut numerator)?;
            }
            let mut maxima = [0; Self::MAX_ATOMS];
            for (monomial, _) in numerator.terms.iter().chain(&denominator.terms) {
                let mut counts = [0; Self::MAX_ATOMS];
                for &atom in monomial {
                    self.charge()?;
                    let index = usize::from(atom);
                    counts[index] += 1;
                    maxima[index] = maxima[index].max(counts[index]);
                }
            }
            let mut selected = None;
            let mut priority = 0;
            for index in (0..self.relations.len()).rev() {
                self.charge()?;
                if let Some(relation) = &self.relations[index]
                    && maxima[index] >= relation.power
                {
                    let is_root = matches!(
                        self.atoms[index].internal.approximation,
                        Approximation::Sqrt(_)
                            | Approximation::Constant(
                                SharedConstant::Sqrt2 | SharedConstant::Sqrt3
                            )
                    );
                    if roots_only && !is_root {
                        continue;
                    }
                    let candidate_priority = if is_root {
                        3
                    } else {
                        match self.atoms[index].internal.approximation {
                            Approximation::JacobiDn(_) => 2,
                            Approximation::JacobiCn(_) => 1,
                            _ => 0,
                        }
                    };
                    if selected.is_none() || candidate_priority > priority {
                        selected = Some((index, maxima[index] / relation.power, relation.clone()));
                        priority = candidate_priority;
                    }
                    if is_root {
                        break;
                    }
                }
            }
            self.charge()?;
            let Some((index, maximum, relation)) = selected else {
                break;
            };
            let numerator_powers = self.polynomial_powers(&relation.value.numerator, maximum)?;
            let denominator_powers = self.polynomial_powers(
                &relation
                    .value
                    .denominator
                    .clone()
                    .unwrap_or(Self::constant(Rational::one())?),
                maximum,
            )?;
            numerator = self.substitute_relation(
                numerator,
                index as u8,
                relation.power,
                &numerator_powers,
                &denominator_powers,
            )?;
            if numerator.terms.is_empty() {
                // All inverse domains were admitted before substitution, and
                // the cleared relation denominator is proved nonzero. Its
                // zero numerator already proves the complete quotient zero.
                return Some(RingQuotient::polynomial(numerator));
            }
            if !zero_only {
                denominator = self.substitute_relation(
                    denominator,
                    index as u8,
                    relation.power,
                    &numerator_powers,
                    &denominator_powers,
                )?;
            }
            if let Some(factor) = &relation.value.denominator {
                for _ in 0..maximum {
                    let Some(n) = self.divide_exact_polynomial(&numerator, factor) else {
                        break;
                    };
                    if !zero_only {
                        let Some(d) = self.divide_exact_polynomial(&denominator, factor) else {
                            break;
                        };
                        denominator = d;
                    }
                    numerator = n;
                }
            }
            if matches!(
                self.atoms[index].internal.approximation,
                Approximation::Sqrt(_)
                    | Approximation::Constant(SharedConstant::Sqrt2 | SharedConstant::Sqrt3)
            ) {
                // A factor of the admitted nonzero denominator is itself
                // nonzero at these inputs. Cancel a root relation's numerator
                // only after exact division of BOTH sides establishes this.
                for _ in 0..maximum {
                    if zero_only {
                        let nonzero = match &self.atoms[index].internal.approximation {
                            Approximation::Sqrt(radicand) => {
                                radicand.immediate_sign() == Some(RealSign::Positive)
                            }
                            Approximation::Constant(
                                SharedConstant::Sqrt2 | SharedConstant::Sqrt3,
                            ) => true,
                            _ => false,
                        };
                        if !nonzero {
                            break;
                        }
                    }
                    let Some(n) =
                        self.divide_exact_polynomial(&numerator, &relation.value.numerator)
                    else {
                        break;
                    };
                    if !zero_only {
                        let Some(d) =
                            self.divide_exact_polynomial(&denominator, &relation.value.numerator)
                        else {
                            break;
                        };
                        denominator = d;
                    }
                    numerator = n;
                }
            }
        }
        if zero_only {
            // Only the zero claim survives multiplying/dividing a residual
            // by proved-nonzero factors. Never expose its rescaled nonzero
            // value as the value of the original expression.
            return numerator
                .terms
                .is_empty()
                .then(|| RingQuotient::polynomial(numerator));
        }
        let denominator = if matches!(denominator.terms.as_slice(), [(m, c)] if m.is_empty() && c.is_one())
        {
            None
        } else {
            Some(denominator)
        };
        let quotient = RingQuotient::new(numerator, denominator);
        if let Some(rational) = self.project_quotient(&quotient) {
            return Some(RingQuotient::polynomial(Self::constant(rational)?));
        }
        Some(quotient)
    }
}
