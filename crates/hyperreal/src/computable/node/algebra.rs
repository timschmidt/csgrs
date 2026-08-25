#[derive(Clone)]
struct PiAtanLinearForm {
    constant: Rational,
    pi: Rational,
    atan: Rational,
    argument: Option<Computable>,
}

#[derive(Clone)]
struct PiAcosLinearForm {
    constant: Rational,
    pi: Rational,
    acos: Rational,
    argument: Option<Computable>,
}

#[derive(Clone)]
struct PiAtanTermsForm {
    constant: Rational,
    pi: Rational,
    terms: Vec<(Computable, Rational)>,
}

#[derive(Clone)]
struct QuadraticCoefficient {
    rational: Rational,
    radical: Rational,
    atoms: Vec<(Computable, Rational)>,
}

impl QuadraticCoefficient {
    fn rational(value: Rational) -> Self {
        Self {
            rational: value,
            radical: Rational::zero(),
            atoms: Vec::new(),
        }
    }

    fn radical(value: Rational) -> Self {
        Self {
            rational: Rational::zero(),
            radical: value,
            atoms: Vec::new(),
        }
    }

    fn atom(value: Computable) -> Self {
        Self {
            rational: Rational::zero(),
            radical: Rational::zero(),
            atoms: vec![(value, Rational::one())],
        }
    }

    fn is_zero(&self) -> bool {
        self.rational.sign() == Sign::NoSign
            && self.radical.sign() == Sign::NoSign
            && self.atoms.is_empty()
    }

    fn add(mut self, other: Self) -> Self {
        self.rational = self.rational + other.rational;
        self.radical = self.radical + other.radical;
        for (atom, coefficient) in other.atoms {
            if let Some((_, existing)) = self
                .atoms
                .iter_mut()
                .find(|(candidate, _)| Computable::internal_structural_eq(candidate, &atom))
            {
                *existing = existing.clone() + coefficient;
            } else {
                self.atoms.push((atom, coefficient));
            }
        }
        self.atoms
            .retain(|(_, coefficient)| coefficient.sign() != Sign::NoSign);
        self
    }

    fn scaled(mut self, scale: &Rational) -> Self {
        self.rational *= scale;
        self.radical *= scale;
        for (_, coefficient) in &mut self.atoms {
            *coefficient *= scale;
        }
        self.atoms
            .retain(|(_, coefficient)| coefficient.sign() != Sign::NoSign);
        self
    }

    fn multiply(self, other: &Self, radicand: Option<&Rational>) -> Option<Self> {
        if (!self.atoms.is_empty()
            && (other.radical.sign() != Sign::NoSign || !other.atoms.is_empty()))
            || (!other.atoms.is_empty() && self.radical.sign() != Sign::NoSign)
        {
            return None;
        }
        let cross = &self.rational * &other.radical + &self.radical * &other.rational;
        let mut rational = &self.rational * &other.rational;
        let radical_product = &self.radical * &other.radical;
        if radical_product.sign() != Sign::NoSign {
            rational = rational + radical_product * radicand?;
        }
        let mut atoms = self
            .atoms
            .into_iter()
            .map(|(atom, coefficient)| (atom, coefficient * &other.rational))
            .collect::<Vec<_>>();
        for (atom, coefficient) in &other.atoms {
            let coefficient = coefficient * &self.rational;
            if let Some((_, existing)) = atoms
                .iter_mut()
                .find(|(candidate, _)| Computable::internal_structural_eq(candidate, atom))
            {
                *existing = existing.clone() + coefficient;
            } else {
                atoms.push((atom.clone(), coefficient));
            }
        }
        atoms.retain(|(_, coefficient)| coefficient.sign() != Sign::NoSign);
        Some(Self {
            rational,
            radical: cross,
            atoms,
        })
    }

    fn inverse(self, radicand: Option<&Rational>) -> Option<Self> {
        if !self.atoms.is_empty() {
            return None;
        }
        if self.radical.sign() == Sign::NoSign {
            return Some(Self::rational(self.rational.inverse().ok()?));
        }
        let radicand = radicand?;
        let norm =
            &self.rational * &self.rational - &self.radical * &self.radical * radicand;
        let inverse_norm = norm.inverse().ok()?;
        Some(Self {
            rational: self.rational * &inverse_norm,
            radical: -self.radical * inverse_norm,
            atoms: Vec::new(),
        })
    }

}

#[derive(Clone)]
struct PiQuadraticLaurentPolynomial {
    radicand: Option<Rational>,
    terms: Vec<(i16, QuadraticCoefficient)>,
}

impl PiQuadraticLaurentPolynomial {
    const MAX_TERMS: usize = 8;

    fn monomial(
        power: i16,
        coefficient: QuadraticCoefficient,
        radicand: Option<Rational>,
    ) -> Self {
        Self {
            radicand,
            terms: (!coefficient.is_zero())
                .then_some((power, coefficient))
                .into_iter()
                .collect(),
        }
    }

    fn normalize_radicand(&mut self) {
        if self
            .terms
            .iter()
            .all(|(_, coefficient)| coefficient.radical.sign() == Sign::NoSign)
        {
            self.radicand = None;
        }
    }

    fn merge_radicand(&mut self, other: &Self) -> Option<()> {
        match (&self.radicand, &other.radicand) {
            (Some(left), Some(right)) if left != right => None,
            (None, Some(right)) => {
                self.radicand = Some(right.clone());
                Some(())
            }
            _ => Some(()),
        }
    }

    fn insert(&mut self, power: i16, coefficient: QuadraticCoefficient) -> Option<()> {
        if coefficient.is_zero() {
            return Some(());
        }
        match self
            .terms
            .binary_search_by_key(&power, |(candidate, _)| *candidate)
        {
            Ok(index) => {
                let sum = self.terms[index].1.clone().add(coefficient);
                if sum.is_zero() {
                    self.terms.remove(index);
                } else {
                    self.terms[index].1 = sum;
                }
            }
            Err(index) => {
                if self.terms.len() == Self::MAX_TERMS {
                    return None;
                }
                self.terms.insert(index, (power, coefficient));
            }
        }
        Some(())
    }

    fn add(mut self, other: Self) -> Option<Self> {
        self.merge_radicand(&other)?;
        for (power, coefficient) in other.terms {
            self.insert(power, coefficient)?;
        }
        self.normalize_radicand();
        Some(self)
    }

    fn scaled(mut self, scale: Rational) -> Self {
        if scale.sign() == Sign::NoSign {
            self.terms.clear();
            self.radicand = None;
            return self;
        }
        for (_, coefficient) in &mut self.terms {
            *coefficient = coefficient.clone().scaled(&scale);
        }
        self
    }

    fn multiply(mut self, other: Self) -> Option<Self> {
        self.merge_radicand(&other)?;
        let radicand = self.radicand.clone();
        let mut product = Self {
            radicand,
            terms: Vec::new(),
        };
        for (left_power, left_coefficient) in self.terms {
            for (right_power, right_coefficient) in &other.terms {
                product.insert(
                    left_power.checked_add(*right_power)?,
                    left_coefficient
                        .clone()
                        .multiply(right_coefficient, product.radicand.as_ref())?,
                )?;
            }
        }
        product.normalize_radicand();
        Some(product)
    }

    fn inverse_monomial(self) -> Option<Self> {
        let [(power, coefficient)] = self.terms.as_slice() else {
            return None;
        };
        Some(Self::monomial(
            power.checked_neg()?,
            coefficient.clone().inverse(self.radicand.as_ref())?,
            self.radicand,
        ))
    }

}

impl PiAtanTermsForm {
    fn scaled(mut self, scale: Rational) -> Self {
        self.constant *= &scale;
        self.pi *= &scale;
        for (_, coefficient) in &mut self.terms {
            *coefficient *= &scale;
        }
        self
    }

    fn add(mut self, other: Self) -> Self {
        self.constant = self.constant + other.constant;
        self.pi = self.pi + other.pi;
        for (argument, coefficient) in other.terms {
            if let Some((_, existing)) = self
                .terms
                .iter_mut()
                .find(|(candidate, _)| Computable::internal_structural_eq(candidate, &argument))
            {
                *existing = existing.clone() + coefficient;
            } else {
                self.terms.push((argument, coefficient));
            }
        }
        self.terms
            .retain(|(_, coefficient)| coefficient.sign() != Sign::NoSign);
        self
    }
}

impl PiAcosLinearForm {
    fn scaled(mut self, scale: Rational) -> Self {
        self.constant *= &scale;
        self.pi *= &scale;
        self.acos *= scale;
        self
    }

    fn add(self, other: Self) -> Option<Self> {
        let argument = match (&self.argument, &other.argument) {
            (Some(left), Some(right)) if !Computable::internal_structural_eq(left, right) => {
                return None;
            }
            (Some(argument), _) | (_, Some(argument)) => Some(argument.clone()),
            (None, None) => None,
        };
        Some(Self {
            constant: self.constant + other.constant,
            pi: self.pi + other.pi,
            acos: self.acos + other.acos,
            argument,
        })
    }
}

impl PiAtanLinearForm {
    fn scaled(mut self, scale: Rational) -> Self {
        self.constant *= &scale;
        self.pi *= &scale;
        self.atan *= scale;
        self
    }

    fn add(self, other: Self) -> Option<Self> {
        let argument = match (&self.argument, &other.argument) {
            (Some(left), Some(right)) if !Computable::internal_structural_eq(left, right) => {
                return None;
            }
            (Some(argument), _) | (_, Some(argument)) => Some(argument.clone()),
            (None, None) => None,
        };
        Some(Self {
            constant: self.constant + other.constant,
            pi: self.pi + other.pi,
            atan: self.atan + other.atan,
            argument,
        })
    }
}

impl Computable {
    fn pi_laurent_polynomial(
        &self,
        budget: usize,
    ) -> Option<PiQuadraticLaurentPolynomial> {
        let mut memo = None;
        self.pi_laurent_polynomial_with_memo(budget, &mut memo)
    }

    fn pi_laurent_polynomial_with_memo(
        &self,
        budget: usize,
        memo: &mut Option<Vec<(usize, usize, Option<PiQuadraticLaurentPolynomial>)>>,
    ) -> Option<PiQuadraticLaurentPolynomial> {
        let key = Arc::as_ptr(&self.internal) as usize;
        let shared = Arc::strong_count(&self.internal) > 1;
        if shared
            && let Some((_, _, value)) = memo.as_ref().and_then(|memo| {
                memo.iter().find(|(candidate, candidate_budget, _)| {
                    *candidate == key && *candidate_budget == budget
                })
            })
        {
            return value.clone();
        }
        if budget == 0 {
            return None;
        }
        let result = if let Some(rational) = self.exact_rational() {
            Some(PiQuadraticLaurentPolynomial::monomial(
                0,
                QuadraticCoefficient::rational(rational),
                None,
            ))
        } else if let Some(polynomial) = match self.shared_constant_kind() {
            Some(SharedConstant::Pi) => Some(PiQuadraticLaurentPolynomial::monomial(
                    1,
                    QuadraticCoefficient::rational(Rational::one()),
                    None,
                )),
            Some(SharedConstant::InvPi) => Some(PiQuadraticLaurentPolynomial::monomial(
                    -1,
                    QuadraticCoefficient::rational(Rational::one()),
                    None,
                )),
            Some(SharedConstant::Tau) => Some(PiQuadraticLaurentPolynomial::monomial(
                    1,
                    QuadraticCoefficient::rational(Rational::new(2)),
                    None,
                )),
            Some(SharedConstant::Sqrt2) => Some(PiQuadraticLaurentPolynomial::monomial(
                    0,
                    QuadraticCoefficient::radical(Rational::one()),
                    Some(Rational::new(2)),
                )),
            Some(SharedConstant::Sqrt3) => Some(PiQuadraticLaurentPolynomial::monomial(
                    0,
                    QuadraticCoefficient::radical(Rational::one()),
                    Some(Rational::new(3)),
                )),
            _ => None,
        } {
            Some(polynomial)
        } else if self.atan_argument().is_some()
            || self.asin_argument().is_some()
            || self.acos_argument().is_some()
        {
            Some(PiQuadraticLaurentPolynomial::monomial(
                0,
                QuadraticCoefficient::atom(self.clone()),
                None,
            ))
        } else {
            let expanded = (|| match &self.internal.approximation {
                Approximation::Add(left, right) => left
                    .pi_laurent_polynomial_with_memo(budget - 1, memo)?
                    .add(right.pi_laurent_polynomial_with_memo(budget - 1, memo)?),
                Approximation::Multiply(left, right) => left
                    .pi_laurent_polynomial_with_memo(budget - 1, memo)?
                    .multiply(right.pi_laurent_polynomial_with_memo(budget - 1, memo)?),
                Approximation::Negate(child) => Some(
                    child
                        .pi_laurent_polynomial_with_memo(budget - 1, memo)?
                        .scaled(Rational::new(-1)),
                ),
                Approximation::Offset(child, shift) => Some(
                    child
                        .pi_laurent_polynomial_with_memo(budget - 1, memo)?
                        .scaled(Self::power_of_two_rational(*shift)),
                ),
                Approximation::Inverse(child) => child
                    .pi_laurent_polynomial_with_memo(budget - 1, memo)?
                    .inverse_monomial(),
                Approximation::Square(child) => {
                    let child = child.pi_laurent_polynomial_with_memo(budget - 1, memo)?;
                    child.clone().multiply(child)
                }
                Approximation::Sqrt(child) => {
                    let radicand = child.exact_rational()?;
                    Some(PiQuadraticLaurentPolynomial::monomial(
                        0,
                        QuadraticCoefficient::radical(Rational::one()),
                        Some(radicand),
                    ))
                }
                Approximation::LinearCombination3(combination) => {
                    let mut sum = PiQuadraticLaurentPolynomial::monomial(
                        0,
                        QuadraticCoefficient::rational(Rational::zero()),
                        None,
                    );
                    for (coefficient, value) in combination
                        .coefficients
                        .iter()
                        .zip(combination.values.iter())
                    {
                        sum = sum.add(
                            coefficient
                                .pi_laurent_polynomial_with_memo(budget - 1, memo)?
                                .scaled(value.clone()),
                        )?;
                    }
                    Some(sum)
                }
                _ => None,
            })();
            expanded.or_else(|| {
                Some(PiQuadraticLaurentPolynomial::monomial(
                    0,
                    QuadraticCoefficient::atom(self.clone()),
                    None,
                ))
            })
        };
        if shared {
            memo.get_or_insert_with(|| Vec::with_capacity(8))
                .push((key, budget, result.clone()));
        }
        result
    }

    pub(crate) fn bounded_laurent_rational(&self) -> Option<Rational> {
        self.bounded_laurent_rational_with_budget(48)
    }

    pub(crate) fn extended_laurent_rational(&self) -> Option<Rational> {
        self.bounded_laurent_rational_with_budget(64)
    }

    fn bounded_laurent_rational_with_budget(&self, budget: usize) -> Option<Rational> {
        let polynomial = self.pi_laurent_polynomial(budget)?;
        match polynomial.terms.as_slice() {
            [] => Some(Rational::zero()),
            [(0, coefficient)]
                if coefficient.radical.sign() == Sign::NoSign
                    && coefficient.atoms.is_empty() =>
            {
                Some(coefficient.rational.clone())
            }
            _ => None,
        }
    }

    fn without_scaled_shared_factor(&self, factor: SharedConstant) -> Option<Computable> {
        if self.shared_constant_kind() == Some(factor) {
            return Some(Self::one());
        }
        match &self.internal.approximation {
            Approximation::Multiply(left, right) => {
                if left.shared_constant_kind() == Some(factor) {
                    return Some(right.clone());
                }
                if right.shared_constant_kind() == Some(factor) {
                    return Some(left.clone());
                }
            }
            Approximation::Negate(child) => {
                return child
                    .without_scaled_shared_factor(factor)
                    .map(Computable::negate);
            }
            Approximation::Offset(child, shift) => {
                return child
                    .without_scaled_shared_factor(factor)
                    .map(|remainder| remainder.shift_left(*shift));
            }
            _ => {}
        }
        None
    }

    fn pi_atan_linear_form(&self, budget: usize) -> Option<PiAtanLinearForm> {
        if budget == 0 {
            return None;
        }
        if let Some(rational) = self.exact_rational() {
            return Some(PiAtanLinearForm {
                constant: rational,
                pi: Rational::zero(),
                atan: Rational::zero(),
                argument: None,
            });
        }
        if self.shared_constant_kind() == Some(SharedConstant::Pi) {
            return Some(PiAtanLinearForm {
                constant: Rational::zero(),
                pi: Rational::one(),
                atan: Rational::zero(),
                argument: None,
            });
        }
        if let Some(argument) = self.atan_argument() {
            return Some(PiAtanLinearForm {
                constant: Rational::zero(),
                pi: Rational::zero(),
                atan: Rational::one(),
                argument: Some(argument),
            });
        }
        match &self.internal.approximation {
            Approximation::Add(left, right) => left
                .pi_atan_linear_form(budget - 1)?
                .add(right.pi_atan_linear_form(budget - 1)?),
            Approximation::Negate(child) => Some(
                child
                    .pi_atan_linear_form(budget - 1)?
                    .scaled(Rational::new(-1)),
            ),
            Approximation::Offset(child, shift) => Some(
                child
                    .pi_atan_linear_form(budget - 1)?
                    .scaled(Self::power_of_two_rational(*shift)),
            ),
            Approximation::Multiply(left, right) => {
                if let Some(scale) = left.exact_rational() {
                    return Some(
                        right
                            .pi_atan_linear_form(budget - 1)?
                            .scaled(scale),
                    );
                }
                if let Some(scale) = right.exact_rational() {
                    return Some(left.pi_atan_linear_form(budget - 1)?.scaled(scale));
                }
                None
            }
            _ => None,
        }
    }

    pub(crate) fn signed_half_pi_minus_atan_argument(&self) -> Option<(Sign, Computable)> {
        let form = self.pi_atan_linear_form(32)?;
        if form.constant.sign() != Sign::NoSign || !form.atan.is_minus_one() {
            return None;
        }
        let orientation = if form.pi == Rational::fraction(1, 2).expect("two is nonzero") {
            Sign::Plus
        } else if form.pi == Rational::fraction(-1, 2).expect("two is nonzero") {
            Sign::Minus
        } else {
            return None;
        };
        let argument = form.argument?;
        (argument.exact_sign()? == orientation).then_some((orientation, argument))
    }

    pub(crate) fn integer_pi_plus_or_minus_atan_argument(
        &self,
    ) -> Option<(Sign, Sign, Computable)> {
        let form = self.pi_atan_linear_form(64)?;
        if form.constant.sign() != Sign::NoSign
            || (!form.atan.is_one() && !form.atan.is_minus_one())
        {
            return None;
        }
        let multiple = form.pi.integer_magnitude()?;
        let cosine_sign = if multiple.bit(0) {
            Sign::Minus
        } else {
            Sign::Plus
        };
        let sine_sign = if form.atan.is_minus_one() {
            match cosine_sign {
                Sign::Plus => Sign::Minus,
                Sign::Minus => Sign::Plus,
                Sign::NoSign => unreachable!("integer-pi parity has a nonzero orientation"),
            }
        } else {
            cosine_sign
        };
        Some((sine_sign, cosine_sign, form.argument?))
    }

    fn pi_acos_linear_form(&self, budget: usize) -> Option<PiAcosLinearForm> {
        if budget == 0 {
            return None;
        }
        if let Some(rational) = self.exact_rational() {
            return Some(PiAcosLinearForm {
                constant: rational,
                pi: Rational::zero(),
                acos: Rational::zero(),
                argument: None,
            });
        }
        if self.shared_constant_kind() == Some(SharedConstant::Pi) {
            return Some(PiAcosLinearForm {
                constant: Rational::zero(),
                pi: Rational::one(),
                acos: Rational::zero(),
                argument: None,
            });
        }
        if let Some(atan_argument) = self.atan_argument()
            && let Some((scale, radicand)) = atan_argument.exact_pure_quadratic_surd()
            && radicand == Rational::new(35)
        {
            let magnitude = if scale.sign() == Sign::Minus {
                -scale.clone()
            } else {
                scale.clone()
            };
            if magnitude == Rational::fraction(1, 35).expect("thirty-five is nonzero") {
                let orientation = if scale.sign() == Sign::Minus {
                    Rational::new(-1)
                } else {
                    Rational::one()
                };
                return Some(PiAcosLinearForm {
                    constant: Rational::zero(),
                    pi: orientation.clone()
                        * Rational::fraction(1, 2).expect("two is nonzero"),
                    acos: -orientation,
                    argument: Some(Self::rational(
                        Rational::fraction(1, 6).expect("six is nonzero"),
                    )),
                });
            }
        }
        if let Some(argument) = self.asin_argument() {
            if let Some(rational) = argument.exact_rational()
                && rational.sign() == Sign::Minus
            {
                return Some(PiAcosLinearForm {
                    constant: Rational::zero(),
                    pi: Rational::fraction(-1, 2).expect("two is nonzero"),
                    acos: Rational::one(),
                    argument: Some(Self::rational(-rational)),
                });
            }
            return Some(PiAcosLinearForm {
                constant: Rational::zero(),
                pi: Rational::fraction(1, 2).expect("two is nonzero"),
                acos: Rational::new(-1),
                argument: Some(argument),
            });
        }
        if let Approximation::AcosNegativeRational(magnitude) = &self.internal.approximation {
            return Some(PiAcosLinearForm {
                constant: Rational::zero(),
                pi: Rational::one(),
                acos: Rational::new(-1),
                argument: Some(Self::rational(magnitude.clone())),
            });
        }
        if let Some(argument) = self.acos_argument() {
            return Some(PiAcosLinearForm {
                constant: Rational::zero(),
                pi: Rational::zero(),
                acos: Rational::one(),
                argument: Some(argument),
            });
        }
        match &self.internal.approximation {
            Approximation::Add(left, right) => left
                .pi_acos_linear_form(budget - 1)?
                .add(right.pi_acos_linear_form(budget - 1)?),
            Approximation::Negate(child) => Some(
                child
                    .pi_acos_linear_form(budget - 1)?
                    .scaled(Rational::new(-1)),
            ),
            Approximation::Offset(child, shift) => Some(
                child
                    .pi_acos_linear_form(budget - 1)?
                    .scaled(Self::power_of_two_rational(*shift)),
            ),
            Approximation::Multiply(left, right) => {
                if let Some(scale) = left.exact_rational() {
                    return Some(
                        right
                            .pi_acos_linear_form(budget - 1)?
                            .scaled(scale),
                    );
                }
                if let Some(scale) = right.exact_rational() {
                    return Some(left.pi_acos_linear_form(budget - 1)?.scaled(scale));
                }
                None
            }
            _ => None,
        }
    }

    fn pi_atan_terms_form(&self, budget: usize) -> Option<PiAtanTermsForm> {
        if budget == 0 {
            return None;
        }
        if let Some(rational) = self.exact_rational() {
            return Some(PiAtanTermsForm {
                constant: rational,
                pi: Rational::zero(),
                terms: Vec::new(),
            });
        }
        if self.shared_constant_kind() == Some(SharedConstant::Pi) {
            return Some(PiAtanTermsForm {
                constant: Rational::zero(),
                pi: Rational::one(),
                terms: Vec::new(),
            });
        }
        if let Some(argument) = self.atan_argument() {
            return Some(PiAtanTermsForm {
                constant: Rational::zero(),
                pi: Rational::zero(),
                terms: vec![(argument, Rational::one())],
            });
        }
        match &self.internal.approximation {
            Approximation::Add(left, right) => Some(
                left.pi_atan_terms_form(budget - 1)?
                    .add(right.pi_atan_terms_form(budget - 1)?),
            ),
            Approximation::Negate(child) => Some(
                child
                    .pi_atan_terms_form(budget - 1)?
                    .scaled(Rational::new(-1)),
            ),
            Approximation::Offset(child, shift) => Some(
                child
                    .pi_atan_terms_form(budget - 1)?
                    .scaled(Self::power_of_two_rational(*shift)),
            ),
            Approximation::Multiply(left, right) => {
                if let Some(scale) = left.exact_rational() {
                    return Some(right.pi_atan_terms_form(budget - 1)?.scaled(scale));
                }
                if let Some(scale) = right.exact_rational() {
                    return Some(left.pi_atan_terms_form(budget - 1)?.scaled(scale));
                }
                None
            }
            _ => None,
        }
    }

    pub(crate) fn signed_acos_minus_half_pi_argument(&self) -> Option<(Sign, Computable)> {
        let form = self.pi_acos_linear_form(32)?;
        if form.constant.sign() != Sign::NoSign {
            return None;
        }
        let half = Rational::fraction(1, 2).expect("two is nonzero");
        let orientation = if form.acos.is_one() && form.pi == -half.clone() {
            Sign::Plus
        } else if form.acos.is_minus_one() && form.pi == half {
            Sign::Minus
        } else {
            return None;
        };
        Some((orientation, form.argument?))
    }

    fn collapsed_inverse_trig_linear_sum(
        &self,
        other: &Computable,
    ) -> Option<Rational> {
        if !self.internal.contains_inverse_trig_or_pi()
            && !other.internal.contains_inverse_trig_or_pi()
        {
            return None;
        }
        if let (Some(left), Some(right)) = (
            self.pi_atan_linear_form(32),
            other.pi_atan_linear_form(32),
        )
            && let Some(form) = left.add(right)
            && form.pi.sign() == Sign::NoSign
            && form.atan.sign() == Sign::NoSign
        {
            return Some(form.constant);
        }
        if let (Some(left), Some(right)) = (
            self.pi_acos_linear_form(32),
            other.pi_acos_linear_form(32),
        )
            && let Some(form) = left.add(right)
            && form.pi.sign() == Sign::NoSign
            && form.acos.sign() == Sign::NoSign
        {
            return Some(form.constant);
        }
        None
    }

    pub(crate) fn inverse_trig_linear_sign(&self) -> Option<Sign> {
        let product_sign = |left: Sign, right: Sign| match (left, right) {
            (Sign::NoSign, _) | (_, Sign::NoSign) => Sign::NoSign,
            (Sign::Plus, Sign::Plus) | (Sign::Minus, Sign::Minus) => Sign::Plus,
            _ => Sign::Minus,
        };
        let atan_term_sign = |argument: &Computable, coefficient: &Rational| {
            argument
                .exact_sign()
                .map(|argument_sign| product_sign(coefficient.sign(), argument_sign))
        };
        let magnitude = |value: &Rational| {
            if value.sign() == Sign::Minus {
                -value.clone()
            } else {
                value.clone()
            }
        };

        if let Some(form) = self.pi_atan_terms_form(32)
            && form.constant.sign() == Sign::NoSign
        {
            if form.pi.sign() == Sign::NoSign && form.terms.is_empty() {
                return Some(Sign::NoSign);
            }
            if form.pi.sign() == Sign::NoSign && form.terms.len() == 1 {
                return Some(product_sign(
                    form.terms[0].1.sign(),
                    form.terms[0].0.exact_sign()?,
                ));
            }
            if form.pi.sign() == Sign::NoSign
                && form.terms.len() == 2
                && form.terms[0].1 == -form.terms[1].1.clone()
            {
                let argument_order = form.terms[0]
                    .0
                    .clone()
                    .add(form.terms[1].0.clone().negate())
                    .exact_sign()?;
                return Some(product_sign(form.terms[0].1.sign(), argument_order));
            }
            let pi_sign = form.pi.sign();
            if pi_sign != Sign::NoSign {
                let terms_follow_pi =
                    form.terms
                        .iter()
                        .try_fold(true, |all_follow, (argument, coefficient)| {
                            if !all_follow {
                                return Some(false);
                            }
                            atan_term_sign(argument, coefficient)
                                .map(|term_sign| term_sign == Sign::NoSign || term_sign == pi_sign)
                        });
                if terms_follow_pi == Some(true) {
                    return Some(pi_sign);
                }
            }
            let atan_bound = form
                .terms
                .iter()
                .fold(Rational::zero(), |sum, (_, coefficient)| {
                    sum + magnitude(coefficient)
                });
            if pi_sign != Sign::NoSign && magnitude(&form.pi) * Rational::new(2) >= atan_bound {
                return Some(pi_sign);
            }
        }

        if let Some(form) = self.pi_atan_linear_form(32)
            && form.constant.sign() == Sign::NoSign
        {
            if form.pi.sign() == Sign::NoSign {
                return Some(product_sign(
                    form.atan.sign(),
                    form.argument?.exact_sign()?,
                ));
            }
            if form.atan.sign() == Sign::NoSign {
                return Some(form.pi.sign());
            }
            let atan_sign = form
                .argument
                .as_ref()
                .and_then(|argument| atan_term_sign(argument, &form.atan));
            if atan_sign == Some(Sign::NoSign) || atan_sign == Some(form.pi.sign()) {
                return Some(form.pi.sign());
            }
            if magnitude(&form.pi) * Rational::new(2) >= magnitude(&form.atan) {
                return Some(form.pi.sign());
            }
        }

        if let Some(form) = self.pi_acos_linear_form(32)
            && form.constant.sign() == Sign::NoSign
        {
            if form.pi.sign() == Sign::NoSign {
                let endpoint_distance = Self::one().add(form.argument?.negate()).exact_sign()?;
                return match endpoint_distance {
                    Sign::Plus => Some(form.acos.sign()),
                    Sign::NoSign => Some(Sign::NoSign),
                    Sign::Minus => None,
                };
            }
            if form.acos.sign() == Sign::NoSign {
                return Some(form.pi.sign());
            }
            if form.pi.sign() == form.acos.sign() {
                return Some(form.pi.sign());
            }
            let half = Rational::fraction(1, 2).expect("two is nonzero");
            let orientation = if form.acos.is_one() && form.pi == -half.clone() {
                Some(Sign::Plus)
            } else if form.acos.is_minus_one() && form.pi == half {
                Some(Sign::Minus)
            } else {
                None
            };
            if let Some(orientation) = orientation {
                let argument_sign = form.argument?.exact_sign()?;
                return Some(product_sign(
                    orientation,
                    match argument_sign {
                        Sign::Plus => Sign::Minus,
                        Sign::Minus => Sign::Plus,
                        Sign::NoSign => Sign::NoSign,
                    },
                ));
            }
        }
        None
    }

    pub(crate) fn pi_rational_multiple(&self) -> Option<Rational> {
        self.retained_pi_rational_multiple(32)
            .or_else(|| self.pi_laurent_rational_multiple())
    }

    fn pi_laurent_rational_multiple(&self) -> Option<Rational> {
        let polynomial = self.pi_laurent_polynomial(48)?;
        match polynomial.terms.as_slice() {
            [] => Some(Rational::zero()),
            [(1, coefficient)]
                if coefficient.radical.sign() == Sign::NoSign
                    && coefficient.atoms.is_empty() =>
            {
                Some(coefficient.rational.clone())
            }
            _ => None,
        }
    }

    fn retained_pi_rational_multiple(&self, budget: usize) -> Option<Rational> {
        if budget == 0 {
            return None;
        }
        if self.shared_constant_kind() == Some(SharedConstant::Pi) {
            return Some(Rational::one());
        }
        match &self.internal.approximation {
            Approximation::Multiply(left, right) => {
                if let Some(scale) = left.exact_rational() {
                    Some(right.retained_pi_rational_multiple(budget - 1)? * scale)
                } else if let Some(scale) = right.exact_rational() {
                    Some(left.retained_pi_rational_multiple(budget - 1)? * scale)
                } else {
                    None
                }
            }
            Approximation::Negate(child) => {
                Some(-child.retained_pi_rational_multiple(budget - 1)?)
            }
            Approximation::Offset(child, shift) => Some(
                child.retained_pi_rational_multiple(budget - 1)?
                    * Self::power_of_two_rational(*shift),
            ),
            Approximation::Add(left, right) => Some(
                left.retained_pi_rational_multiple(budget - 1)?
                    + right.retained_pi_rational_multiple(budget - 1)?,
            ),
            _ => None,
        }
    }

    fn canonical_sin_pi_term(&self) -> Option<(Rational, Rational)> {
        match &self.internal.approximation {
            Approximation::PrescaledSin(argument) => {
                let argument = argument.pi_rational_multiple()?;
                (argument.sign() == Sign::Plus
                    && argument < Rational::fraction(1, 2).expect("two is nonzero"))
                .then(|| (argument, Rational::one()))
            }
            Approximation::Negate(child) => {
                let (argument, scale) = child.canonical_sin_pi_term()?;
                Some((argument, scale.neg()))
            }
            Approximation::Offset(child, shift) => {
                let (argument, scale) = child.canonical_sin_pi_term()?;
                Some((argument, scale * Self::power_of_two_rational(*shift)))
            }
            Approximation::Multiply(left, right) => {
                if let Some(scale) = left.exact_rational() {
                    let (argument, inner_scale) = right.canonical_sin_pi_term()?;
                    return Some((argument, scale * inner_scale));
                }
                if let Some(scale) = right.exact_rational() {
                    let (argument, inner_scale) = left.canonical_sin_pi_term()?;
                    return Some((argument, scale * inner_scale));
                }
                None
            }
            _ => None,
        }
    }

    fn sin_pi_rational_sum_sign(&self, rational: &Rational) -> Option<Sign> {
        let (_, scale) = self.canonical_sin_pi_term()?;
        let endpoint = rational + &scale;
        match scale.sign() {
            Sign::Plus if rational.sign() != Sign::Minus => Some(Sign::Plus),
            Sign::Plus if endpoint.sign() != Sign::Plus => Some(Sign::Minus),
            Sign::Minus if rational.sign() != Sign::Plus => Some(Sign::Minus),
            Sign::Minus if endpoint.sign() != Sign::Minus => Some(Sign::Plus),
            Sign::NoSign => Some(rational.sign()),
            Sign::Plus | Sign::Minus => None,
        }
    }

    /// Negate this number.
    pub fn negate(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            // Keep exact leaves exact; a Negate node would hide cheap rational
            // sign/MSD facts.
            return Self::rational(rational.neg());
        }
        if let Approximation::Negate(child) = &self.internal.approximation {
            // Double negation cancels at construction time so exact sign walks
            // and approximation stacks stay shallow.
            return child.clone();
        }
        if let Approximation::Multiply(left, right) = &self.internal.approximation {
            if let Some(scale) = left.exact_rational()
                && scale.sign() == Sign::Minus
            {
                crate::trace_dispatch!("computable", "negate", "exact-scale-fold");
                return right.clone().multiply_rational(scale.neg());
            }
            if let Some(scale) = right.exact_rational()
                && scale.sign() == Sign::Minus
            {
                crate::trace_dispatch!("computable", "negate", "exact-scale-fold");
                return left.clone().multiply_rational(scale.neg());
            }
        }
        let exact_sign = match self.internal.facts.exact_sign() {
            // Preserve known exact signs for the cheap sign-first path in
            // predicates; this avoids a recursive sign walk on first query.
            ExactSignCache::Valid(sign) => ExactSignCache::Valid(negate_sign(sign)),
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(Approximation::Negate(self), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    /// Multiplicative inverse of this number.
    pub fn inverse(self) -> Computable {
        if self.shared_constant_kind() == Some(SharedConstant::Pi) {
            crate::trace_dispatch!("computable", "inverse", "shared-pi");
            return Self::pi_inverse_constant();
        }
        if self.shared_constant_kind() == Some(SharedConstant::InvPi) {
            crate::trace_dispatch!("computable", "inverse", "shared-inv-pi");
            return Self::pi();
        }
        if let Some(rational) = self.exact_rational()
            && let Ok(inverse) = rational.inverse()
        {
            // Exact rational reciprocals stay exact; this is common in BLAS
            // division by scalar constants.
            return Self::rational(inverse);
        }
        if let Approximation::Negate(child) = &self.internal.approximation
            && child.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
        {
            // 1/(-x) = -(1/x). The nonzero sign guard avoids manufacturing a
            // reciprocal of a value that may be zero.
            return child.clone().inverse().negate();
        }
        if let Approximation::Offset(child, n) = &self.internal.approximation
            && child.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
        {
            // 1/(x*2^n) = (1/x)*2^-n, preserving the cheap binary scale.
            return child.clone().inverse().shift_left(-n);
        }
        if let Approximation::Multiply(left, right) = &self.internal.approximation {
            if let Some(scale) = left.exact_rational()
                && let Ok(inverse_scale) = scale.inverse()
                && right.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
            {
                // 1/(q*x) = (1/q)/x. Peeling the exact scale lets chains like
                // negate(inverse(x * -7/8)) collapse every other step instead
                // of building deep multiply/inverse/negate stacks.
                return right.clone().inverse().multiply_rational(inverse_scale);
            }
            if let Some(scale) = right.exact_rational()
                && let Ok(inverse_scale) = scale.inverse()
                && left.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
            {
                return left.clone().inverse().multiply_rational(inverse_scale);
            }
        }
        if let Approximation::Inverse(child) = &self.internal.approximation
            && child.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
        {
            // Inverse of inverse collapses only when the inner value is
            // structurally nonzero.
            return child.clone();
        }
        if let Approximation::Square(child) = &self.internal.approximation
            && child.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
        {
            crate::trace_dispatch!("computable", "inverse", "square-of-inverse");
            return child.clone().inverse().square();
        }
        let exact_sign = match self.internal.facts.exact_sign() {
            // Reciprocal preserves sign for structurally nonzero values and lets
            // sign queries remain structural through inverse chains.
            ExactSignCache::Valid(sign) if sign != Sign::NoSign => ExactSignCache::Valid(sign),
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(Approximation::Inverse(self), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    pub(crate) fn shift_left(self, n: i32) -> Self {
        if n == 0 {
            return self;
        }
        if let Approximation::Negate(child) = &self.internal.approximation {
            // Keep the sign outside an exact binary scale so independently
            // expanded products retain the common unsigned expression.
            return child.clone().shift_left(n).negate();
        }
        if let Approximation::Offset(child, inner) = &self.internal.approximation {
            // Combine nested binary offsets rather than growing a chain of
            // no-op-ish wrappers.
            return child.clone().shift_left(inner + n);
        }
        // Exact sign is unchanged by binary scaling when the inner sign is
        // already proven; this makes compare/sign predicates avoid descending
        // into a one-step structural walk on hot paths.
        let exact_sign = match self.internal.facts.exact_sign() {
            ExactSignCache::Valid(sign) => ExactSignCache::Valid(sign),
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(Approximation::Offset(self, n), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    fn shift_right(self, n: i32) -> Self {
        self.shift_left(-n)
    }

    /// Square of this number.
    pub fn square(self) -> Self {
        if let Some(rational) = self.exact_rational() {
            // Exact rationals can square without approximation or expression growth.
            return Self::rational(rational.clone() * rational);
        }
        if let Approximation::Negate(child) = &self.internal.approximation {
            // (-x)^2 is x^2; dropping the negate avoids an extra node in repeated products.
            return child.clone().square();
        }
        if let Approximation::Sqrt(child) = &self.internal.approximation {
            match child.exact_sign() {
                // sqrt(x)^2 can collapse only when x is structurally known nonnegative.
                Some(Sign::Plus) | Some(Sign::NoSign) => return child.clone(),
                _ => {}
            }
        }
        if let Approximation::Offset(child, n) = &self.internal.approximation {
            // (x * 2^n)^2 is x^2 * 2^(2n); keeping powers of two as offsets is much
            // cheaper than multiplying by an exact rational scale.
            return child.clone().square().shift_left(n * 2);
        }
        if let Approximation::Multiply(left, right) = &self.internal.approximation {
            if let Some(scale) = left.exact_rational() {
                // Peel exact scales out of products before squaring symbolic factors.
                return right
                    .clone()
                    .square()
                    .multiply(Self::rational(scale.clone() * scale));
            }
            if let Some(scale) = right.exact_rational() {
                return left
                    .clone()
                    .square()
                    .multiply(Self::rational(scale.clone() * scale));
            }
        }
        let exact_sign = match self.internal.facts.exact_sign() {
            // Squared values are nonnegative when defined; structural zero is
            // preserved as exact zero.
            ExactSignCache::Valid(Sign::NoSign) => ExactSignCache::Valid(Sign::NoSign),
            ExactSignCache::Valid(Sign::Plus) | ExactSignCache::Valid(Sign::Minus) => {
                ExactSignCache::Valid(Sign::Plus)
            }
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(Approximation::Square(self), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    /// Multiply this number by some other number.
    pub fn multiply(self, other: Computable) -> Computable {
        if let Approximation::Negate(child) = &self.internal.approximation {
            return child.clone().multiply(other).negate();
        }
        if let Approximation::Negate(child) = &other.internal.approximation {
            return self.multiply(child.clone()).negate();
        }
        match self.shared_constant_kind() {
            Some(SharedConstant::Pi) => {
                if let Approximation::Add(left, right) = &other.internal.approximation {
                    if let Some(remainder) =
                        left.without_scaled_shared_factor(SharedConstant::InvPi)
                    {
                        return remainder.add(self.multiply(right.clone()));
                    }
                    if let Some(remainder) =
                        right.without_scaled_shared_factor(SharedConstant::InvPi)
                    {
                        return self.multiply(left.clone()).add(remainder);
                    }
                }
                if let Some(remainder) = other.without_scaled_shared_factor(SharedConstant::InvPi)
                {
                    return remainder;
                }
            }
            Some(SharedConstant::InvPi) => {
                if let Some(remainder) = other.without_scaled_shared_factor(SharedConstant::Pi) {
                    return remainder;
                }
            }
            _ => {}
        }
        match other.shared_constant_kind() {
            Some(SharedConstant::Pi) => {
                if let Approximation::Add(left, right) = &self.internal.approximation {
                    if let Some(remainder) =
                        left.without_scaled_shared_factor(SharedConstant::InvPi)
                    {
                        return remainder.add(other.multiply(right.clone()));
                    }
                    if let Some(remainder) =
                        right.without_scaled_shared_factor(SharedConstant::InvPi)
                    {
                        return other.multiply(left.clone()).add(remainder);
                    }
                }
                if let Some(remainder) = self.without_scaled_shared_factor(SharedConstant::InvPi)
                {
                    return remainder;
                }
            }
            Some(SharedConstant::InvPi) => {
                if let Some(remainder) = self.without_scaled_shared_factor(SharedConstant::Pi) {
                    return remainder;
                }
            }
            _ => {}
        }
        let left_exact = self.exact_rational();
        let right_exact = other.exact_rational();

        if matches!(left_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign)
            || matches!(right_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign)
        {
            // Zero annihilates without preserving the other expression tree.
            return Self::zero();
        }
        if matches!(left_exact.as_ref(), Some(r) if r.is_one()) {
            // Multiplication by +/-1 stays as identity/negate so downstream exact-sign
            // queries still see the original structure.
            return other;
        }
        if matches!(right_exact.as_ref(), Some(r) if r.is_one()) {
            return self;
        }
        if matches!(left_exact.as_ref(), Some(r) if r.is_minus_one()) {
            return other.negate();
        }
        if matches!(right_exact.as_ref(), Some(r) if r.is_minus_one()) {
            return self.negate();
        }
        let exact_sign = {
            let left_sign = left_exact.as_ref().map(Rational::sign).or_else(|| {
                match self.internal.facts.exact_sign() {
                    ExactSignCache::Valid(sign) => Some(sign),
                    _ => None,
                }
            });
            let right_sign = right_exact.as_ref().map(Rational::sign).or_else(|| {
                match other.internal.facts.exact_sign() {
                    ExactSignCache::Valid(sign) => Some(sign),
                    _ => None,
                }
            });
            match (left_sign, right_sign) {
                (Some(Sign::NoSign), Some(_)) | (Some(_), Some(Sign::NoSign)) => {
                    ExactSignCache::Valid(Sign::NoSign)
                }
                (Some(left), Some(right)) => ExactSignCache::Valid(if left == right {
                    Sign::Plus
                } else {
                    Sign::Minus
                }),
                _ => ExactSignCache::Invalid,
            }
        };
        if let Some((shift, sign)) = left_exact.as_ref().and_then(Rational::power_of_two_shift) {
            // Dyadic scales are represented as binary offsets, avoiding generic multiply
            // evaluation during approximation.
            let shifted = other.shift_left(shift);
            return if sign == Sign::Minus {
                shifted.negate()
            } else {
                shifted
            };
        }
        if let Some((shift, sign)) = right_exact.as_ref().and_then(Rational::power_of_two_shift) {
            let shifted = self.shift_left(shift);
            return if sign == Sign::Minus {
                shifted.negate()
            } else {
                shifted
            };
        }
        if let (Some(left), Some(right)) = (left_exact.as_ref(), right_exact.as_ref()) {
            // Collapse purely exact products immediately.
            return Self::rational(left.clone() * right.clone());
        }
        if let Some(scale) = left_exact.as_ref()
            && let Approximation::Multiply(inner_left, inner_right) = &other.internal.approximation
        {
            if let Some(inner_scale) = inner_left.exact_rational() {
                // Combine adjacent exact scales so factored symbolic products stay shallow.
                return inner_right
                    .clone()
                    .multiply(Self::rational(scale.clone() * inner_scale));
            }
            if let Some(inner_scale) = inner_right.exact_rational() {
                return inner_left
                    .clone()
                    .multiply(Self::rational(scale.clone() * inner_scale));
            }
        }
        if let Some(scale) = right_exact.as_ref()
            && let Approximation::Multiply(inner_left, inner_right) = &self.internal.approximation
        {
            if let Some(inner_scale) = inner_left.exact_rational() {
                return inner_right
                    .clone()
                    .multiply(Self::rational(scale.clone() * inner_scale));
            }
            if let Some(inner_scale) = inner_right.exact_rational() {
                return inner_left
                    .clone()
                    .multiply(Self::rational(scale.clone() * inner_scale));
            }
        }
        Self {
            internal: Arc::new(Node::new(Approximation::Multiply(self, other), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    pub(crate) fn linear_combination3(
        coefficients: [Computable; 3],
        values: [Rational; 3],
    ) -> Computable {
        Self {
            internal: Arc::new(Node::new(
                Approximation::LinearCombination3(Box::new(
                    LinearCombination3 {
                        coefficients,
                        values,
                    },
                )),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    pub(crate) fn multiply_rational(self, scale: Rational) -> Computable {
        if scale.sign() == Sign::NoSign {
            // Multiplying by zero drops the expression tree, including any
            // pending expensive approximation work.
            return Self::zero();
        }
        if let Some(value) = self.exact_rational() {
            // Exact symbolic leaves and exact-rational factors collapse directly.
            // This preserves cheap structural facts for chained scaling in
            // `fold_ref` and avoids building a new Multiply node.
            return Self::rational(value * scale);
        }
        if scale.is_one() {
            return self;
        }
        if scale.is_minus_one() {
            return self.negate();
        }
        if let Some((shift, sign)) = scale.power_of_two_shift() {
            // The borrowed Real fold path calls this often; recognize dyadic
            // scales before building a generic Multiply node.
            let shifted = self.shift_left(shift);
            return if sign == Sign::Minus {
                shifted.negate()
            } else {
                shifted
            };
        }
        if let Approximation::Multiply(left, right) = &self.internal.approximation {
            // Peel and combine exact rational factors from existing multiply
            // nodes so repeated scalar rebalances stay shallow.
            if let Some(inner_scale) = left.exact_rational() {
                return right.clone().multiply_rational(inner_scale.clone() * scale);
            }
            if let Some(inner_scale) = right.exact_rational() {
                return left.clone().multiply_rational(inner_scale.clone() * scale);
            }
        }
        let scale_sign = scale.sign();
        let exact_sign = match (self.internal.facts.exact_sign(), scale_sign) {
            (ExactSignCache::Valid(Sign::NoSign), _) => ExactSignCache::Valid(Sign::NoSign),
            (ExactSignCache::Valid(Sign::Plus), Sign::Plus) => ExactSignCache::Valid(Sign::Plus),
            (ExactSignCache::Valid(Sign::Plus), Sign::Minus) => ExactSignCache::Valid(Sign::Minus),
            (ExactSignCache::Valid(Sign::Minus), Sign::Plus) => ExactSignCache::Valid(Sign::Minus),
            (ExactSignCache::Valid(Sign::Minus), Sign::Minus) => ExactSignCache::Valid(Sign::Plus),
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(Approximation::Multiply(Self::rational(scale), self), BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    /// Add some other number to this number.
    #[allow(clippy::should_implement_trait)]
    pub fn add(self, other: Computable) -> Computable {
        if let Approximation::Negate(cancelled) = &other.internal.approximation
            && Self::internal_structural_eq(&self, cancelled)
        {
            crate::trace_dispatch!("computable", "add", "direct-cancellation");
            return Self::zero();
        }
        if let Approximation::Negate(cancelled) = &self.internal.approximation
            && Self::internal_structural_eq(cancelled, &other)
        {
            crate::trace_dispatch!("computable", "add", "direct-cancellation");
            return Self::zero();
        }
        if let Some(rational) = self.collapsed_inverse_trig_linear_sum(&other) {
            crate::trace_dispatch!("computable", "add", "inverse-trig-linear-collapse");
            return Self::rational(rational);
        }
        if let Approximation::Add(left, right) = &self.internal.approximation
            && let Some(other_rational) = other.exact_rational()
        {
            if let Some(left_rational) = left.exact_rational() {
                crate::trace_dispatch!("computable", "add", "nested-left-rational-fold");
                return right
                    .clone()
                    .add(Self::rational(left_rational + other_rational));
            }
            if let Some(right_rational) = right.exact_rational() {
                crate::trace_dispatch!("computable", "add", "nested-right-rational-fold");
                return left
                    .clone()
                    .add(Self::rational(right_rational + other_rational));
            }
        }
        if let Some(self_rational) = self.exact_rational()
            && let Approximation::Add(left, right) = &other.internal.approximation
        {
            if let Some(left_rational) = left.exact_rational() {
                crate::trace_dispatch!("computable", "add", "nested-left-rational-fold");
                return right
                    .clone()
                    .add(Self::rational(self_rational + left_rational));
            }
            if let Some(right_rational) = right.exact_rational() {
                crate::trace_dispatch!("computable", "add", "nested-right-rational-fold");
                return left
                    .clone()
                    .add(Self::rational(self_rational + right_rational));
            }
        }
        if let Approximation::Add(left, right) = &self.internal.approximation
            && let Approximation::Negate(cancelled) = &other.internal.approximation
        {
            if Self::internal_structural_eq(left, cancelled) {
                crate::trace_dispatch!("computable", "add", "nested-left-cancellation");
                return right.clone();
            }
            if Self::internal_structural_eq(right, cancelled) {
                crate::trace_dispatch!("computable", "add", "nested-right-cancellation");
                return left.clone();
            }
        }
        if let Approximation::Negate(cancelled) = &self.internal.approximation
            && let Approximation::Add(left, right) = &other.internal.approximation
        {
            if Self::internal_structural_eq(cancelled, left) {
                crate::trace_dispatch!("computable", "add", "nested-left-cancellation");
                return right.clone();
            }
            if Self::internal_structural_eq(cancelled, right) {
                crate::trace_dispatch!("computable", "add", "nested-right-cancellation");
                return left.clone();
            }
        }

        let left_exact = self.exact_rational();
        let right_exact = other.exact_rational();

        if matches!(left_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign) {
            // Exact zero leaves are common after symbolic cancellation; avoid
            // wrapping the surviving operand in an Add node.
            return other;
        }
        if matches!(right_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign) {
            // Symmetric exact-zero fast path for borrowed and owned additions.
            return self;
        }
        if matches!(right_exact.as_ref(), Some(r) if r.is_one())
            && let Some(left) = left_exact.as_ref()
        {
            return Self::rational(left.add_one());
        }
        if matches!(left_exact.as_ref(), Some(r) if r.is_one())
            && let Some(right) = right_exact.as_ref()
        {
            return Self::rational(right.add_one());
        }
        if let (Some(left), Some(right)) = (left_exact.as_ref(), right_exact.as_ref()) {
            // Fold exact leaf sums immediately so rational imports and parsed
            // dyadics stay outside the approximation graph.
            return Self::rational(left.clone() + right.clone());
        }
        let certified_bound = if let Some(rational) = right_exact.as_ref()
            && let Some(term) = self.shared_constant_term()
        {
            Self::constant_rational_sum_bound(&term, rational)
        } else if let Some(rational) = left_exact.as_ref()
            && let Some(term) = other.shared_constant_term()
        {
            Self::constant_rational_sum_bound(&term, rational)
        } else {
            BoundInfo::Unknown
        };
        // Store any c*K+q certificate directly on the Add node. The arithmetic
        // still falls back to a generic sum, but structural sign/fact queries
        // can answer from the certificate.
        let child_sign = {
            let left_sign = left_exact.as_ref().map(Rational::sign).or_else(|| {
                match self.internal.facts.exact_sign() {
                    ExactSignCache::Valid(sign) => Some(sign),
                    _ => None,
                }
            });
            let right_sign = right_exact.as_ref().map(Rational::sign).or_else(|| {
                match other.internal.facts.exact_sign() {
                    ExactSignCache::Valid(sign) => Some(sign),
                    _ => None,
                }
            });
            match (left_sign, right_sign) {
                (Some(Sign::NoSign), Some(sign)) | (Some(sign), Some(Sign::NoSign)) => Some(sign),
                (Some(Sign::Plus), Some(Sign::Plus)) => Some(Sign::Plus),
                (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Minus),
                _ => None,
            }
        };
        let certified_sign = certified_bound.known_sign().or(child_sign);
        let certified_sign = certified_sign
            .or_else(|| {
                right_exact
                    .as_ref()
                    .and_then(|rational| self.sin_pi_rational_sum_sign(rational))
            })
            .or_else(|| {
                left_exact
                    .as_ref()
                    .and_then(|rational| other.sin_pi_rational_sum_sign(rational))
            });
        Self {
            internal: Arc::new(Node::new(Approximation::Add(self, other), if certified_bound == BoundInfo::Unknown {
                    BoundCache::Invalid
                } else {
                    BoundCache::Valid(certified_bound)
                },
                match certified_sign {
                    Some(sign) => ExactSignCache::Valid(sign),
                    None => ExactSignCache::Invalid,
                },
            )),
            signal: None,
        }
    }

    pub(crate) fn integer(n: BigInt) -> Self {
        if n == *signed::ONE {
            return Self::one();
        }
        Self {
            internal: Arc::new(Node::new(Approximation::Int(n), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        }
    }

    /// Error function, erf(x).
    pub fn erf(self) -> Computable {
        if let Some(rational) = self.exact_rational()
            && rational.sign() == Sign::NoSign
        {
            return Self::zero();
        }
        let series = Self {
            internal: Arc::new(Node::new(Approximation::ErfSeries(self.clone()), BoundCache::Invalid, ExactSignCache::Invalid)),
            signal: None,
        };
        let gaussian = self.square().negate().exp();
        let two_over_sqrt_pi = Self::pi().sqrt().inverse().shift_left(1);
        two_over_sqrt_pi.multiply(gaussian).multiply(series)
    }

    /// Complementary error function, erfc(x) = 1 - erf(x).
    pub fn erfc(self) -> Computable {
        if let Some(rational) = self.exact_rational()
            && rational.sign() == Sign::NoSign
        {
            return Self::one();
        }
        Self {
            internal: Arc::new(Node::new(Approximation::Erfc(self), BoundCache::Invalid, ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    /// Scaled complementary error function, erfcx(x) = exp(x^2) * erfc(x).
    pub fn erfcx(self) -> Computable {
        if let Some(rational) = self.exact_rational()
            && rational.sign() == Sign::NoSign
        {
            return Self::one();
        }
        self.clone().square().exp().multiply(self.erfc())
    }

    /// Standard normal CDF.
    pub fn pnorm(self) -> Computable {
        if let Some(rational) = self.exact_rational()
            && rational.sign() == Sign::NoSign
        {
            return Self::rational(HALF_RATIONAL.clone());
        }
        let sqrt2 = Self::sqrt_constant(2).unwrap_or_else(|| Self::integer(BigInt::from(2)).sqrt());
        let z = self.multiply(sqrt2.inverse());
        Self::one().add(z.erf()).shift_right(1)
    }

    /// Standard normal density.
    pub fn dnorm(self) -> Computable {
        let neg_half_x_sq = self.square().shift_right(1).negate();
        let sqrt_2pi = Self::pi().shift_left(1).sqrt();
        neg_half_x_sq.exp().multiply(sqrt_2pi.inverse())
    }

    /// Standard normal upper-tail probability, 1 - pnorm(x).
    pub fn normal_sf(self) -> Computable {
        if let Some(rational) = self.exact_rational()
            && rational.sign() == Sign::NoSign
        {
            return Self::rational(HALF_RATIONAL.clone());
        }
        Self {
            internal: Arc::new(Node::new(Approximation::NormalSf(self), BoundCache::Invalid, ExactSignCache::Valid(Sign::Plus))),
            signal: None,
        }
    }

    /// Standard normal probability mass over [lo, hi].
    pub fn normal_interval(lo: Computable, hi: Computable) -> Computable {
        if Computable::internal_structural_eq(&lo, &hi) {
            return Self::zero();
        }
        let exact_order = lo
            .bounded_laurent_rational()
            .zip(hi.bounded_laurent_rational())
            .map(|(lo, hi)| {
                lo.partial_cmp(&hi)
                    .expect("exact rational ordering is total")
            });
        if exact_order == Some(Ordering::Equal) {
            return Self::zero();
        }
        let exact_sign = match exact_order {
            Some(Ordering::Less) => ExactSignCache::Valid(Sign::Plus),
            Some(Ordering::Greater) => ExactSignCache::Valid(Sign::Minus),
            Some(Ordering::Equal) => unreachable!("equal exact endpoints returned above"),
            None => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(Approximation::NormalInterval { lo, hi }, BoundCache::Invalid, exact_sign)),
            signal: None,
        }
    }

    /// Natural logarithm of the standard normal CDF.
    pub fn log_pnorm(self) -> Computable {
        Self {
            internal: Arc::new(Node::new(Approximation::LogPnorm(self), BoundCache::Invalid, ExactSignCache::Valid(Sign::Minus))),
            signal: None,
        }
    }

    /// Natural logarithm of the standard normal upper-tail probability.
    pub fn log_normal_sf(self) -> Computable {
        Self {
            internal: Arc::new(Node::new(Approximation::LogNormalSf(self), BoundCache::Invalid, ExactSignCache::Valid(Sign::Minus))),
            signal: None,
        }
    }

    /// Natural logarithm of the standard normal density.
    pub fn log_dnorm(self) -> Computable {
        Self {
            internal: Arc::new(Node::new(Approximation::LogDnorm(self), BoundCache::Invalid, ExactSignCache::Valid(Sign::Minus))),
            signal: None,
        }
    }

    /// Standard normal quantile by Newton iteration with the analytic density.
    pub fn normal_quantile(p: Computable, seed: BigInt, seed_prec: Precision) -> Computable {
        Self {
            internal: Arc::new(Node::new(
                Approximation::NormalQuantile(Box::new(NormalQuantileData {
                    p,
                    seed,
                    seed_prec,
                })), BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Attach an abort signal checked by long-running approximation routines.
    pub fn abort(&mut self, s: Signal) {
        self.signal = Some(s);
    }

}
