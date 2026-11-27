// Bounded substitution of admitted native relations in rational functions.
/// Substitute admitted relations under distinct quotient-value and sufficient-zero contracts.
impl RingRecognition<'_> {
    /// Build powers zero through count inclusive in this attempt's admitted polynomial algebra.
    /// Start at one and repeatedly multiply by the same input, sharing work/size
    /// limits. Count zero returns just one without an explicit work charge. The
    /// caller supplies a feasible count; this helper adds no independent vector cap.
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

    /// Substitute a^power=N/H into a polynomial after clearing H^M.
    /// For an a-exponent e, retain a^(e mod power) and multiply its other factors by
    /// N^(e/power)*H^(M-e/power), with M determined by the numerator power table.
    /// Accumulate exact terms directly under the existing degree/coefficient/term
    /// and charged-work limits. The caller must supply positive power, nonempty
    /// correct power tables through M, and e/power<=M for every input term; invalid
    /// tables can panic. H must already be proved nonzero for value-preserving
    /// quotient use. Sorting is bounded by admitted monomial size, not charged per comparison.
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

    /// Reduce an admitted quotient with all available native relations while preserving its value.
    /// Delegate to Quotient mode; None is an optional-recognition limit/stop decline.
    /// Denominator validity and triangular relation invariants must already hold.
    fn reduce_native_relations(&mut self, value: RingQuotient) -> Option<RingQuotient> {
        self.reduce_native_relation_proof(value, NativeReduction::Quotient)
    }

    /// Apply triangular native relations according to a value or zero-only proof mode.
    /// Inspect exponents in numerator and denominator, then scan relations from high
    /// IDs down, preferring roots, then Jacobi dn, then cn, then other relations.
    /// Within one priority, the highest ID wins. Clear relation denominators with
    /// matching powers on both quotient sides; zero modes retain only the zero
    /// implication for the numerator. An emptied numerator can finish immediately
    /// because all original and introduced denominators were already admitted.
    ///
    /// Try exact cancellation of introduced factors using remaining work. For root
    /// numerators, full quotient modes require formal division of both sides; zero
    /// modes instead require immediate positive-radicand evidence (or a shared
    /// positive root). Plain zero mode accepts only a vanished residual; signed zero
    /// mode may also use nonnegative-square completion. Their successful output is
    /// canonical zero, never the rescaled nonzero residual. Quotient/Roots modes may
    /// return a still-symbolic quotient and optionally project a rational result.
    ///
    /// The caller supplies valid bounded atom IDs, positive powers, lower-ID relation
    /// dependencies and nonzero denominator witnesses. Counters, coefficient/degree/
    /// term caps and cancellation are shared across nested operations; a failed
    /// optional cancellation does not reset them. None is not proof of nonzero or
    /// non-rationality. No global scalar cache is published and no approximation runs.
    fn reduce_native_relation_proof(
        &mut self,
        value: RingQuotient,
        mode: NativeReduction,
    ) -> Option<RingQuotient> {
        let roots_only = mode == NativeReduction::Roots;
        let zero_only = matches!(
            mode,
            NativeReduction::ZeroResidual | NativeReduction::SignedZeroResidual
        );
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
            if mode == NativeReduction::SignedZeroResidual
                && !numerator.terms.is_empty()
                && self.prove_zero_by_nonnegative_squares(&numerator).is_some()
            {
                return Some(RingQuotient::polynomial(RingPolynomial::default()));
            }
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
