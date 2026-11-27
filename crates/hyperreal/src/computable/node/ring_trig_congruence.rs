/// Reuse sine/cosine identities through exact argument quotients within one query.
/// Equality is proved algebraically, never inferred from sampled closeness. This
/// stage shares the quotient authority and preserves the original expression.
impl<'a> RingRecognition<'a> {
    /// Represent a sine/cosine using an admitted argument quotient and local exact proofs.
    /// Require positive depth and value's matching PrescaledSin/PrescaledCos argument.
    /// First scan successful rational function projections by structural recipe equality,
    /// charging before each lookup; a hit reuses prior argument admission. Otherwise
    /// expand the argument, including encountered inverse guards, before optional
    /// rational trig projection when the additional-relation strategy is enabled.
    /// A successful projection marks additional relation use and may enter the bounded
    /// positive cache; a failed optional projection can spend work before falling through.
    ///
    /// For remaining atoms, compare only the same function kind, in atom order, by
    /// subtracting its stored argument quotient and proving zero. A match sets
    /// trig_congruence_used and reuses that ID. Different functions or merely nearby
    /// arguments are not merged. Otherwise append an atom and argument record, then
    /// optionally register a unit-circle pair. Positive-cache capacity does not reject
    /// an uncached rational success; opaque atom capacity does reject new admission.
    ///
    /// The strategy requires a complete compatible trig_arguments table for existing
    /// trig atoms; a missing record declines immediately. Failure may leave atoms,
    /// arguments, proofs or flags changed locally. This method does not validate every
    /// opaque dependency or the prescaled numerical kernel's range. No numerical
    /// refinement or root fact publication occurs; None also covers depth/work/stop
    /// limits and unproved domains, and supplies no inequality certificate.
    fn trig_atom(
        &mut self,
        value: &'a Computable,
        argument: &'a Computable,
        depth: usize,
    ) -> Option<RingPolynomial> {
        if depth == 0 {
            return None;
        }
        // Only the later relation attempt populates this positive cache.
        // A structurally identical function has the already admitted argument
        // and domain; cold serialization need not preserve pointer sharing.
        for index in 0..self.rational_trig_values.len() {
            self.charge()?;
            let (known, rational) = &self.rational_trig_values[index];
            if Computable::internal_structural_eq(known, value) {
                return Self::constant(rational.clone());
            }
        }
        // Admit the argument before matching its function value. In particular,
        // an invalid or unproved inverse cannot disappear through congruence.
        let argument = self.expand_quotient(argument, depth)?;
        if self.rational_squares.is_some()
            && let Some(rational) = self.rational_trig_projection(value, &argument)
        {
            self.additional_relation_used = true;
            if self.rational_trig_values.len() < Self::MAX_ATOMS {
                self.charge()?;
                self.rational_trig_values.push((value, rational.clone()));
            }
            return Self::constant(rational);
        }
        for index in 0..self.atoms.len() {
            self.charge()?;
            if !matches!(
                (
                    &value.internal.approximation,
                    &self.atoms[index].internal.approximation
                ),
                (
                    Approximation::PrescaledSin(_),
                    Approximation::PrescaledSin(_)
                ) | (
                    Approximation::PrescaledCos(_),
                    Approximation::PrescaledCos(_)
                )
            ) {
                continue;
            }
            let previous = self
                .trig_arguments
                .as_ref()?
                .iter()
                .find(|(atom, _)| usize::from(*atom) == index)?
                .1
                .clone();
            let opposite = self.quotient_scale(previous, Rational::from(-1))?;
            let difference = self.quotient_add(argument.clone(), opposite)?;
            if self
                .project_quotient(&difference)
                .is_some_and(|r| r.is_zero())
            {
                self.trig_congruence_used = true;
                return Some(RingPolynomial {
                    terms: vec![(vec![index as u8], Rational::one())],
                });
            }
        }
        if self.atoms.len() == Self::MAX_ATOMS {
            return None;
        }
        let index = self.atoms.len() as u8;
        self.atoms.push(std::borrow::Cow::Borrowed(value));
        self.trig_arguments.as_mut()?.push((index, argument));
        if self.rational_squares.is_some() {
            self.register_unit_circle_pair(index)?;
        }
        Some(RingPolynomial {
            terms: vec![(vec![index], Rational::one())],
        })
    }
}

/// Run the specialized congruence attempt and publish only a completed exact proof.
impl Computable {
    /// Try a rational quotient proof that actually reuses equal sine/cosine arguments.
    /// Start a fresh ordinary-limit attempt with shared sqrt(2)/sqrt(3) squares and
    /// trig argument admission enabled, but without additional rational-root/trig-value
    /// or unit-circle relations. Require a completed rational projection and a set
    /// trig_congruence_used flag; structural quotient reuse alone need not set it.
    /// Defined native operands and compatible inherited facts remain preconditions.
    ///
    /// After a final stop check, publish only the exact root sign and the
    /// trig-congruence-quotient dispatch event, not a numerical sample or rational tower.
    /// None combines unsupported recognition, unproved domains, exhausted limits and
    /// cancellation. Local state is discarded and a fresh query can retry. The separate
    /// node/work/degree/coefficient caps do not establish total-time or heap bounds.
    pub(crate) fn bounded_trig_congruence_rational(&self) -> Option<Rational> {
        let mut recognition = RingRecognition::new(self);
        recognition.root_squares = true;
        recognition.trig_arguments = Some(Vec::new());
        let quotient = recognition.expand_quotient(self, RingRecognition::MAX_DEPTH)?;
        let rational = recognition.project_quotient(&quotient)?;
        if should_stop(&self.signal) || !recognition.trig_congruence_used {
            return None;
        }
        self.internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(rational.sign()));
        crate::trace_dispatch!("computable", "normal_form", "trig-congruence-quotient");
        Some(rational)
    }
}
