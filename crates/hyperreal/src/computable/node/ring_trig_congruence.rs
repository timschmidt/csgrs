// Function congruence: equal exact arguments give equal sine/cosine values.
// This later attempt reuses the quotient authority; it neither reconstructs
// the expression nor interprets numerical closeness as argument equality.
impl<'a> RingRecognition<'a> {
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

impl Computable {
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
