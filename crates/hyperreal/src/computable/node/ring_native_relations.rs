// Optional rational projection in a triangular algebraic extension. Each
// retained relation a^p=N/H proves H nonzero. Substitution clears the same
// power of H from both sides of a quotient; individual numerical expressions
// are never rewritten into cancellation-prone square differences.
#[derive(Clone)]
struct NativeRingRelation {
    power: usize,
    value: RingQuotient,
    root_sign: Option<RealSign>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum NativeReduction {
    Quotient,
    Roots,
    ZeroResidual,
    SignedZeroResidual,
}

impl<'a> RingRecognition<'a> {
    // Merge admitted monomials while collapsing rational root squares. No
    // retained monomial exceeds the existing degree cap, even when an
    // unreduced product would exceed it. All visits and products are charged.
    fn multiply_native_root_terms(
        &mut self,
        a: &[u8],
        b: &[u8],
        mut coefficient: Rational,
    ) -> Option<(Vec<u8>, Rational)> {
        self.charge()?;
        if !Self::coefficient_fits(&coefficient) {
            return None;
        }
        let (mut i, mut j) = (0, 0);
        let mut result = Vec::with_capacity((a.len() + b.len()).min(self.degree_limit()));
        while i < a.len() || j < b.len() {
            self.charge()?;
            let atom = match (a.get(i), b.get(j)) {
                (Some(x), Some(y)) => (*x).min(*y),
                (Some(x), None) | (None, Some(x)) => *x,
                (None, None) => unreachable!("at least one factor remains"),
            };
            let mut count = 0;
            while a.get(i) == Some(&atom) {
                self.charge()?;
                i += 1;
                count += 1;
            }
            while b.get(j) == Some(&atom) {
                self.charge()?;
                j += 1;
                count += 1;
            }
            let index = usize::from(atom);
            let square = if count >= 2
                && matches!(
                    self.atoms[index].internal.approximation,
                    Approximation::Sqrt(_)
                        | Approximation::Constant(SharedConstant::Sqrt2 | SharedConstant::Sqrt3)
                ) {
                self.relations[index].as_ref().and_then(|relation| {
                    if relation.power != 2 || relation.value.denominator.is_some() {
                        return None;
                    }
                    match relation.value.numerator.terms.as_slice() {
                        [] => Some(Rational::zero()),
                        [(term, value)] if term.is_empty() => Some(value.clone()),
                        _ => None,
                    }
                })
            } else {
                None
            };
            if let Some(square) = square {
                for _ in 0..count / 2 {
                    self.charge()?;
                    coefficient *= &square;
                    if !Self::coefficient_fits(&coefficient) {
                        return None;
                    }
                }
                count %= 2;
            }
            if result.len() + count > self.degree_limit() {
                return None;
            }
            result.extend(std::iter::repeat_n(atom, count));
        }
        Some((result, coefficient))
    }

    fn scalar_product_shape(&mut self, value: &Computable, depth: usize) -> Option<bool> {
        self.charge()?;
        if depth == 0 {
            return Some(false);
        }
        match &value.internal.approximation {
            Approximation::Multiply(a, b) => Some(
                self.scalar_product_shape(a, depth - 1)?
                    && self.scalar_product_shape(b, depth - 1)?,
            ),
            Approximation::Square(child) => self.scalar_product_shape(child, depth - 1),
            Approximation::One
            | Approximation::Int(_)
            | Approximation::Ratio(_)
            | Approximation::Add(..)
            | Approximation::Negate(_)
            | Approximation::Offset(..)
            | Approximation::LinearCombination3(_)
            | Approximation::Inverse(_)
            | Approximation::JacobiSn(_)
            | Approximation::JacobiCn(_)
            | Approximation::JacobiDn(_) => Some(false),
            Approximation::Sqrt(_)
            | Approximation::Constant(SharedConstant::Sqrt2 | SharedConstant::Sqrt3) => {
                Some(!self.root_completion)
            }
            _ => Some(true),
        }
    }

    fn native_relation_atom(
        &mut self,
        value: std::borrow::Cow<'a, Computable>,
        depth: usize,
    ) -> Option<RingPolynomial> {
        for index in 0..self.atoms.len() {
            self.charge()?;
            if Computable::internal_structural_eq(self.atoms[index].as_ref(), &value) {
                return Some(RingPolynomial {
                    terms: vec![(vec![index as u8], Rational::one())],
                });
            }
        }
        // Dependencies enter first. Therefore every relation refers only to
        // lower atom IDs, even when its coefficients contain other Jacobi
        // functions or roots. A declined relation leaves this atom opaque.
        let relation = self
            .native_square_relation(&value, depth)
            .map(|value| NativeRingRelation {
                power: 2,
                value,
                root_sign: None,
            })
            .or_else(|| self.completed_root_relation(&value, depth))
            .or_else(|| self.native_inverse_exp_relation(&value));
        self.root_relation_seen |= relation.is_some()
            && matches!(
                value.internal.approximation,
                Approximation::Sqrt(_)
                    | Approximation::Constant(SharedConstant::Sqrt2 | SharedConstant::Sqrt3)
            );
        self.charge()?;
        if self.atoms.len() == Self::MAX_ATOMS {
            return None;
        }
        let index = self.atoms.len() as u8;
        self.atoms.push(value);
        self.relations.push(relation);
        self.native_signs.push(None);
        Some(RingPolynomial {
            terms: vec![(vec![index], Rational::one())],
        })
    }

    fn native_inverse_exp_relation(&mut self, value: &Computable) -> Option<NativeRingRelation> {
        let is_exp = |value: &Computable| {
            matches!(
                value.internal.approximation,
                Approximation::PrescaledExp(_) | Approximation::Constant(SharedConstant::E)
            )
        };
        if !is_exp(value) {
            return None;
        }
        for index in 0..self.atoms.len() {
            self.charge()?;
            let other = self.atoms[index].as_ref();
            if !is_exp(other) {
                continue;
            }
            // Reuse the existing bounded log-of-positive-exponential proof.
            // Binary range reduction may leave a+b=k*ln(2), not just zero.
            // Retained logarithms of rationals or pi are not part of this
            // relation; only ln(2) offsets and cancelled terms qualify.
            let mut form = ExpRelationForm::new(false);
            if form.logarithm(value, Rational::one()).is_none()
                || form.logarithm(other, Rational::one()).is_none()
                || !form.constant.is_zero()
                || !form.logarithms.is_empty()
            {
                continue;
            }
            let scale = match form.terms.as_slice() {
                [] => Rational::one(),
                [(None, coefficient)] => {
                    let Some(shift) = coefficient
                        .to_integer_i64()
                        .filter(|s| s.unsigned_abs() < Self::MAX_COEFFICIENT_BITS)
                    else {
                        continue;
                    };
                    Computable::power_of_two_rational(shift as i32)
                }
                _ => continue,
            };
            // The admitted exponential syntax proves the denominator positive.
            return Some(NativeRingRelation {
                power: 1,
                root_sign: None,
                value: RingQuotient {
                    numerator: Self::constant(scale)?,
                    denominator: Some(RingPolynomial {
                        terms: vec![(vec![index as u8], Rational::one())],
                    }),
                },
            });
        }
        None
    }

    fn native_square_relation(&mut self, value: &Computable, depth: usize) -> Option<RingQuotient> {
        if depth == 0 {
            return None;
        }
        match &value.internal.approximation {
            Approximation::JacobiSn(arguments)
            | Approximation::JacobiCn(arguments)
            | Approximation::JacobiDn(arguments) => {
                let cosine_basis = self.jacobi_cosine_basis(&arguments[1])?;
                if matches!(value.internal.approximation, Approximation::JacobiSn(_))
                    && !cosine_basis
                    || matches!(value.internal.approximation, Approximation::JacobiCn(_))
                        && cosine_basis
                {
                    return None;
                }
                let base = arguments[0]
                    .clone()
                    .jacobi_component(arguments[1].clone(), usize::from(cosine_basis));
                let base = self.expand_relation_value(&base, depth - 1)?;
                let square = self.quotient_multiply(&base, &base)?;
                let one = RingQuotient::polynomial(Self::constant(Rational::one())?);
                let (constant, coefficient) =
                    if matches!(value.internal.approximation, Approximation::JacobiDn(_)) {
                        let complement = arguments[1].retained_log_argument(&mut 64)?;
                        let complement = self.expand_relation_value(&complement, depth - 1)?;
                        if cosine_basis {
                            // dn² = h + (1-h)cn². This basis keeps the stable
                            // negative-parameter complex denominator compact.
                            let negative_one =
                                self.quotient_scale(one.clone(), Rational::from(-1))?;
                            let coefficient =
                                self.quotient_add(complement.clone(), negative_one)?;
                            (complement, coefficient)
                        } else {
                            let negative = self.quotient_scale(complement, Rational::from(-1))?;
                            (one.clone(), self.quotient_add(one.clone(), negative)?)
                        }
                    } else {
                        (one.clone(), one)
                    };
                let term = self.quotient_multiply(&coefficient, &square)?;
                let term = self.quotient_scale(term, Rational::from(-1))?;
                self.quotient_add(constant, term)
            }
            Approximation::Sqrt(radicand)
                if matches!(
                    radicand.immediate_sign(),
                    Some(RealSign::Zero | RealSign::Positive)
                ) =>
            {
                self.expand_relation_value(radicand, depth - 1)
            }
            Approximation::Constant(SharedConstant::Sqrt2) => {
                Some(RingQuotient::polynomial(Self::constant(Rational::from(2))?))
            }
            Approximation::Constant(SharedConstant::Sqrt3) => {
                Some(RingQuotient::polynomial(Self::constant(Rational::from(3))?))
            }
            _ => None,
        }
    }

    fn jacobi_cosine_basis(&mut self, log_complement: &Computable) -> Option<bool> {
        // One choice per structurally equal parameter for the whole query.
        // Differently warmed copies must not introduce opposite rewrite rules.
        for index in 0..self.jacobi_bases.len() {
            self.charge()?;
            if Computable::internal_structural_eq(&self.jacobi_bases[index].0, log_complement) {
                return Some(self.jacobi_bases[index].1);
            }
        }
        self.charge()?;
        if self.jacobi_bases.len() == Self::MAX_ATOMS {
            return None;
        }
        let cosine = log_complement.immediate_sign() == Some(RealSign::Positive);
        self.jacobi_bases.push((log_complement.clone(), cosine));
        Some(cosine)
    }

    // Derived expressions are temporary, so their pointers must never enter
    // the memo table for the authoritative borrowed DAG. Retain opaque leaves
    // by ownership and parse arithmetic within the same visit/work budgets.
    fn expand_relation_value(&mut self, value: &Computable, depth: usize) -> Option<RingQuotient> {
        self.charge()?;
        if depth == 0 {
            return None;
        }
        self.nodes_left = self.nodes_left.checked_sub(1)?;
        if let Some(rational) = value.retained_rational_reduction() {
            return Self::constant(rational).map(RingQuotient::polynomial);
        }
        if self.is_parameter_atom(value)?
            || matches!(
                value.internal.approximation,
                Approximation::Multiply(..) | Approximation::Square(_)
            ) && self.scalar_product_shape(value, 8)?
        {
            return self
                .native_relation_atom(std::borrow::Cow::Owned(value.clone()), depth - 1)
                .map(RingQuotient::polynomial);
        }
        let result = match &value.internal.approximation {
            Approximation::One => RingQuotient::polynomial(Self::constant(Rational::one())?),
            Approximation::Int(integer) => {
                if integer.bits() >= Self::MAX_COEFFICIENT_BITS {
                    return None;
                }
                RingQuotient::polynomial(Self::constant(Rational::from_bigint(integer.clone()))?)
            }
            Approximation::Ratio(rational) => {
                RingQuotient::polynomial(Self::constant(rational.clone())?)
            }
            Approximation::Add(a, b) => {
                let a = self.expand_relation_value(a, depth - 1)?;
                let b = self.expand_relation_value(b, depth - 1)?;
                self.quotient_add(a, b)?
            }
            Approximation::Multiply(a, b) => {
                let a = self.expand_relation_value(a, depth - 1)?;
                let b = self.expand_relation_value(b, depth - 1)?;
                self.quotient_multiply(&a, &b)?
            }
            Approximation::Square(child) => {
                let child = self.expand_relation_value(child, depth - 1)?;
                self.quotient_multiply(&child, &child)?
            }
            Approximation::Negate(child) => {
                let child = self.expand_relation_value(child, depth - 1)?;
                self.quotient_scale(child, Rational::from(-1))?
            }
            Approximation::Offset(child, shift) => {
                if u64::from(shift.unsigned_abs()) >= Self::MAX_COEFFICIENT_BITS {
                    return None;
                }
                let child = self.expand_relation_value(child, depth - 1)?;
                self.quotient_scale(child, Computable::power_of_two_rational(*shift))?
            }
            Approximation::LinearCombination3(combination) => {
                let mut sum = RingQuotient::polynomial(RingPolynomial::default());
                for (child, scale) in combination.coefficients.iter().zip(&combination.values) {
                    let child = self.expand_relation_value(child, depth - 1)?;
                    let term = self.quotient_scale(child, scale.clone())?;
                    sum = self.quotient_add(sum, term)?;
                }
                sum
            }
            Approximation::Inverse(child) => {
                let quotient = self.expand_relation_value(child, depth - 1)?;
                match child.immediate_sign() {
                    Some(RealSign::Zero) => return None,
                    Some(RealSign::Positive | RealSign::Negative) => {}
                    None => {
                        self.admitted_quotient_nonzero(&quotient)?;
                    }
                }
                if quotient.numerator.terms.is_empty() {
                    return None;
                }
                self.invert_admitted_quotient(quotient)?
            }
            _ => RingQuotient::polynomial(
                self.native_relation_atom(std::borrow::Cow::Owned(value.clone()), depth - 1)?,
            ),
        };
        self.reduce_native_relations(result)
    }
}
