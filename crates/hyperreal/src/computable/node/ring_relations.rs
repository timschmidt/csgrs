// Additional exact relations over the existing domain-checked quotient ring.
// The principal square root satisfies sqrt(q)^2=q only for admitted q>=0.
// Pi is nonzero, and its built-in reciprocal denotes exactly 1/pi.
impl<'a> RingRecognition<'a> {
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

impl Computable {
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
}
