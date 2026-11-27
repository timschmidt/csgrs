/// Derive rational trig values and directed unit-circle rewrites from admitted quotients.
impl<'a> RingRecognition<'a> {
    /// Project sin/cos of ±atan(q)+k*pi/2 when q and sqrt(1+q²) are rational.
    /// Use the already admitted argument quotient, first requiring exact polynomial
    /// division if its denominator is explicit. Parse a constant plus pi and at most
    /// one AtanRational atom; the quarter-turn helper requires zero constant, atan
    /// coefficient ±1 and integral k. Pure quarter turns without an atan atom are not
    /// recognized by this route. The positive hypotenuse yields q/h and 1/h, with
    /// swaps/signs determined exactly from the quarter turn.
    ///
    /// The caller supplies a matching sine/cosine recipe, normalized terms, valid atom
    /// IDs and denominator evidence. Other recipe kinds would follow the cosine branch
    /// and are outside the contract. Charge explicit scans/arithmetic and enforce stored
    /// coefficient caps; perfect-root extraction and rational internals have no additional
    /// stop polls here. Unsupported forms, irrational hypotenuse and limits all return
    /// None. No approximate fallback, usage flag or global fact is published by this
    /// helper; its caller records a successful projection.
    fn rational_trig_projection(
        &mut self,
        value: &Computable,
        argument: &RingQuotient,
    ) -> Option<Rational> {
        let polynomial = match &argument.denominator {
            Some(denominator) => self.divide_exact_polynomial(&argument.numerator, denominator)?,
            None => argument.numerator.clone(),
        };
        let mut form = PiAtanLinearForm {
            constant: Rational::zero(),
            pi: Rational::zero(),
            atan: Rational::zero(),
            argument: None,
        };
        for (term, coefficient) in polynomial.terms {
            self.charge()?;
            match term.as_slice() {
                [] => form.constant = coefficient,
                [atom] => match &self.atoms[usize::from(*atom)].internal.approximation {
                    Approximation::Constant(SharedConstant::Pi) => form.pi = coefficient,
                    Approximation::AtanRational(q)
                        if form.argument.is_none() && Self::coefficient_fits(q) =>
                    {
                        form.atan = coefficient;
                        form.argument = Some(Computable::rational(q.clone()));
                    }
                    _ => return None,
                },
                _ => return None,
            }
        }
        // The shared quadrant recognizer performs at most one coefficient
        // multiplication (by two); reserve its charge before invoking it.
        self.charge()?;
        let (swap, sine_sign, cosine_sign, atan_argument) = form.quarter_turn_parts()?;
        let q = atan_argument.exact_rational()?;
        self.charge()?;
        let square = &q * &q;
        if !Self::coefficient_fits(&square) {
            return None;
        }
        self.charge()?;
        let squared_hypotenuse = Rational::one() + square;
        if !Self::coefficient_fits(&squared_hypotenuse) {
            return None;
        }
        self.charge()?;
        // A perfect rational root proves the principal positive hypotenuse.
        // General irrational hypotenuses remain opaque in this projection.
        let hypotenuse = squared_hypotenuse.perfect_nth_root(2)?;
        self.charge()?;
        let inverse = hypotenuse.inverse().ok()?;
        let sine = matches!(value.internal.approximation, Approximation::PrescaledSin(_));
        let rational = if sine == swap {
            inverse
        } else {
            self.charge()?;
            q * inverse
        };
        if !Self::coefficient_fits(&rational) {
            return None;
        }
        self.charge()?;
        let sign = if sine { sine_sign } else { cosine_sign };
        Some(if sign == Sign::Minus {
            -rational
        } else {
            rational
        })
    }

    /// Record at most one proved square-sum-one pair for the newly appended trig atom.
    /// Require new_atom to match the last admitted trig_arguments entry, with valid
    /// atom IDs and function kinds. Opposite sine/cosine kinds require equal argument
    /// quotients. Two sines or two cosines require their arguments to sum exactly to
    /// pi/2; denominator-free rational pi multiples use a cheaper coefficient test.
    /// Otherwise test the sum through quotient arithmetic. Stop after the first pair.
    ///
    /// The stored pair denotes retained and replaced square roles, not necessarily
    /// literal sine and cosine kinds. For complementary functions the older atom is
    /// retained and the newer is reduced, preserving the directed reduction order.
    /// Exact cos(pi*r) can arrive as sin(pi*(1/2-r)), motivating this same-kind rule.
    /// This is a sufficient test, not all trigonometric equalities (evenness and other
    /// periodic complements are not added). Some(()) also means no pair was proved;
    /// None covers missing state and failed charged substeps. Some false comparisons
    /// can include spent optional proof work. Registration itself does not mark relation
    /// use or publish facts; reduction must subsequently consume a recorded pair.
    fn register_unit_circle_pair(&mut self, new_atom: u8) -> Option<()> {
        let (new_id, argument) = self.trig_arguments.as_ref()?.last()?.clone();
        debug_assert_eq!(new_atom, new_id);
        let half = Rational::fraction(1, 2).expect("two is nonzero");
        for index in 0..self.trig_arguments.as_ref()?.len() - 1 {
            self.charge()?;
            let old_atom = self.trig_arguments.as_ref()?[index].0;
            let (sine, cosine, complementary) = match (
                &self.atoms[usize::from(new_atom)].internal.approximation,
                &self.atoms[usize::from(old_atom)].internal.approximation,
            ) {
                (Approximation::PrescaledSin(_), Approximation::PrescaledCos(_)) => {
                    (new_atom, old_atom, false)
                }
                (Approximation::PrescaledCos(_), Approximation::PrescaledSin(_)) => {
                    (old_atom, new_atom, false)
                }
                (Approximation::PrescaledSin(_), Approximation::PrescaledSin(_))
                | (Approximation::PrescaledCos(_), Approximation::PrescaledCos(_)) => {
                    (old_atom, new_atom, true)
                }
                _ => continue,
            };
            let related = if complementary {
                // Rational turns decide by their coefficients, without
                // cloning or adding quotients, which keeps a scan over many
                // unrelated turns cheap.
                let turn = |quotient: &RingQuotient| {
                    quotient
                        .denominator
                        .is_none()
                        .then(|| self.pi_multiple(&quotient.numerator))
                        .flatten()
                };
                let turns = turn(&argument).zip(turn(&self.trig_arguments.as_ref()?[index].1));
                if let Some((first, second)) = turns {
                    self.charge()?;
                    first + second == half
                } else {
                    let previous = self.trig_arguments.as_ref()?[index].1.clone();
                    let sum = self.quotient_add(argument.clone(), previous)?;
                    self.quotient_is_half_pi(&sum)?
                }
            } else {
                let previous = self.trig_arguments.as_ref()?[index].1.clone();
                let opposite = self.quotient_scale(previous, Rational::from(-1))?;
                let difference = self.quotient_add(argument.clone(), opposite)?;
                self.project_quotient(&difference)
                    .is_some_and(|r| r.is_zero())
            };
            if related {
                self.unit_circle_pairs.push((sine, cosine));
                break;
            }
        }
        Some(())
    }

    /// Recognize the stored one-term, degree-one shape r*pi and return r exactly.
    /// Require a valid atom ID naming the built-in Pi constant. Zero polynomials,
    /// multiterm expressions and equivalent forms needing simplification return None;
    /// this is not a complete value-equivalence test. No charge or stop poll occurs.
    fn pi_multiple(&self, polynomial: &RingPolynomial) -> Option<Rational> {
        let [(monomial, coefficient)] = polynomial.terms.as_slice() else {
            return None;
        };
        let [atom] = monomial.as_slice() else {
            return None;
        };
        matches!(
            self.atoms[usize::from(*atom)].internal.approximation,
            Approximation::Constant(SharedConstant::Pi)
        )
        .then(|| coefficient.clone())
    }

    /// Test a sufficient stored-shape proof that an admitted quotient equals pi/2.
    /// First try exact division for an explicit, already nonzero denominator. A declined
    /// division returns Some(false), including when it exhausted work; false means not
    /// proved, not inequality. Otherwise charge before testing the polynomial for its
    /// single half-pi term. A declined final charge returns None. No fact is published.
    fn quotient_is_half_pi(&mut self, value: &RingQuotient) -> Option<bool> {
        let polynomial = match &value.denominator {
            Some(denominator) => {
                match self.divide_exact_polynomial(&value.numerator, denominator) {
                    Some(polynomial) => polynomial,
                    None => return Some(false),
                }
            }
            None => value.numerator.clone(),
        };
        self.charge()?;
        Some(
            self.pi_multiple(&polynomial)
                == Some(Rational::fraction(1, 2).expect("two is nonzero")),
        )
    }

    /// Replace recorded second-role squares by one minus the first-role square.
    /// Scan terms in storage order, pairs in registration order and adjacent sorted
    /// factor IDs in position order. For c*rest*v² with recorded u²+v²=1, remove that
    /// term and insert c*rest and -c*rest*u² under the active polynomial limits. Repeat
    /// until no replaceable pair remains, setting additional_relation_used only after
    /// both insertions complete. The pair table must contain sound directed relations
    /// from compatible admitted atoms; arbitrary cycles are outside this contract.
    ///
    /// Work and intermediate term/coefficient growth are charged/guarded; None can
    /// leave the attempt's usage flag or other local state changed, while the owned
    /// partial polynomial is discarded. No matches (including an empty polynomial or
    /// pair list) can return without a stop poll. Success means these recorded rewrites
    /// finished, not that the polynomial is zero or globally canonical. The caller
    /// retains final projection, cancellation and publication responsibility.
    fn reduce_unit_circle_pairs(
        &mut self,
        mut polynomial: RingPolynomial,
    ) -> Option<RingPolynomial> {
        loop {
            let mut replacement = None;
            'terms: for (index, (term, _)) in polynomial.terms.iter().enumerate() {
                for pair in 0..self.unit_circle_pairs.len() {
                    self.charge()?;
                    let (sine, cosine) = self.unit_circle_pairs[pair];
                    for position in 0..term.len().saturating_sub(1) {
                        self.charge()?;
                        if term[position] == cosine && term[position + 1] == cosine {
                            replacement = Some((index, position, sine));
                            break 'terms;
                        }
                    }
                }
            }
            let Some((index, position, sine)) = replacement else {
                return Some(polynomial);
            };
            self.charge()?;
            let (mut rest, coefficient) = polynomial.terms.remove(index);
            rest.drain(position..position + 2);
            self.insert(&mut polynomial, rest.clone(), coefficient.clone())?;
            rest.extend([sine, sine]);
            rest.sort_unstable();
            self.insert(&mut polynomial, rest, -coefficient)?;
            self.additional_relation_used = true;
        }
    }
}
