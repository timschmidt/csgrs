// Exact trigonometric consequences of already admitted argument quotients.
impl<'a> RingRecognition<'a> {
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

    /// Records one pair of atoms whose squares sum to one.
    ///
    /// A sine and a cosine pair when their admitted arguments are equal. Two
    /// sines, or two cosines, pair when their arguments sum to exactly
    /// `pi / 2`, because `sin(pi/2 - x) = cos(x)` and `cos(pi/2 - x) = sin(x)`
    /// for every real `x`. Exact `cos(pi * r)` values are stored as
    /// `sin(pi * (1/2 - r))`, so this complementary form is how a rational
    /// turn's sine and cosine reach the recognizer. In a complementary pair
    /// the older atom takes the sine role and the newer one is reduced, so no
    /// chain of reductions returns to an atom it replaced.
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

    /// The rational `r` of a polynomial quotient that is exactly `r * pi`.
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

    /// Whether an admitted quotient is exactly `pi / 2`: after any exact
    /// division by its denominator, one term `(1/2) * pi`. Any other form,
    /// including an inexact division, is not proved equal.
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
