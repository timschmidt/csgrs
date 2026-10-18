// A proof-only normal form for positive exponential expressions. Nothing here
// replaces an approximation node or observes numeric caches. Consequently a
// failed structural attempt may share the existing exact-sign Unknown cache.
struct ExpRelationForm<'a> {
    // None denotes ln(2), including binary offsets outside an exponential.
    terms: Vec<(Option<&'a Computable>, Rational)>,
    logarithms: Vec<(PositiveLogAtom, Rational)>,
    constant: Rational,
    remaining: usize,
    require_defined_arguments: bool,
}

#[derive(PartialEq)]
enum PositiveLogAtom {
    Pi,
    Rational(Rational),
}

impl<'a> ExpRelationForm<'a> {
    fn new(require_defined_arguments: bool) -> Self {
        Self {
            terms: Vec::new(),
            logarithms: Vec::new(),
            constant: Rational::zero(),
            remaining: 128,
            require_defined_arguments,
        }
    }

    fn bounded(coefficient: &Rational) -> bool {
        coefficient
            .numerator()
            .bits()
            .max(coefficient.denominator().bits())
            <= 1024
    }

    fn leaf(node: &Computable) -> Option<Rational> {
        match &node.internal.approximation {
            Approximation::One => Some(Rational::one()),
            Approximation::Int(n) if n.bits() <= 1024 => Some(Rational::from_bigint(n.clone())),
            Approximation::Ratio(q) if Self::bounded(q) => Some(q.clone()),
            _ => None,
        }
    }

    fn charge(&mut self, coefficient: &Rational) -> Option<()> {
        self.remaining = self.remaining.checked_sub(1)?;
        Self::bounded(coefficient).then_some(())
    }

    fn term(&mut self, atom: Option<&'a Computable>, coefficient: Rational) -> Option<()> {
        if !Self::bounded(&coefficient) {
            return None;
        }
        if coefficient.sign() == Sign::NoSign {
            return Some(());
        }
        for index in 0..self.terms.len() {
            let same = match (atom, self.terms[index].0) {
                (None, None) => true,
                (Some(a), Some(b)) => exp_atom_structural_eq(a, b),
                _ => false,
            };
            if same {
                self.terms[index].1 = &self.terms[index].1 + coefficient;
                if !Self::bounded(&self.terms[index].1) {
                    return None;
                }
                if self.terms[index].1.sign() == Sign::NoSign {
                    self.terms.swap_remove(index);
                }
                return Some(());
            }
        }
        if self.terms.len() + self.logarithms.len() >= 16 {
            return None;
        }
        self.terms.push((atom, coefficient));
        Some(())
    }

    fn log_term(&mut self, atom: PositiveLogAtom, coefficient: Rational) -> Option<()> {
        if !Self::bounded(&coefficient) {
            return None;
        }
        for index in 0..self.logarithms.len() {
            if self.logarithms[index].0 == atom {
                let sum = &self.logarithms[index].1 + coefficient;
                if !Self::bounded(&sum) {
                    return None;
                }
                if sum.sign() == Sign::NoSign {
                    self.logarithms.swap_remove(index);
                } else {
                    self.logarithms[index].1 = sum;
                }
                return Some(());
            }
        }
        if coefficient.sign() != Sign::NoSign {
            if self.terms.len() + self.logarithms.len() >= 16 {
                return None;
            }
            self.logarithms.push((atom, coefficient));
        }
        Some(())
    }

    fn log_rational(&mut self, value: Rational, coefficient: Rational) -> Option<()> {
        if value.sign() != Sign::Plus || !Self::bounded(&value) {
            return None;
        }
        if value.is_one() {
            return Some(());
        }
        let numerator = value.numerator();
        let denominator = value.denominator();
        let n = numerator.trailing_zeros()?;
        let d = denominator.trailing_zeros()?;
        if numerator.bits() == n + 1 && denominator.bits() == d + 1 {
            return self.term(None, coefficient * Rational::new(n as i64 - d as i64));
        }
        self.log_term(PositiveLogAtom::Rational(value), coefficient)
    }

    fn linear_atom(&mut self, node: &'a Computable, coefficient: Rational) -> Option<()> {
        // Ordinary construction already requires real operands. Imported graphs
        // have no such proof: even a zero coefficient must not erase an unknown
        // or undefined child before validating its domain.
        if self.require_defined_arguments
            && !matches!(node.internal.approximation, Approximation::Constant(_))
        {
            return None;
        }
        self.term(Some(node), coefficient)
    }

    fn linear(&mut self, node: &'a Computable, coefficient: Rational) -> Option<()> {
        self.charge(&coefficient)?;
        if let Some(q) = Self::leaf(node) {
            self.constant = &self.constant + q * coefficient;
            return Self::bounded(&self.constant).then_some(());
        }
        match &node.internal.approximation {
            Approximation::Constant(SharedConstant::Ln2) => self.term(None, coefficient),
            Approximation::Add(a, b) => {
                self.linear(a, coefficient.clone())?;
                self.linear(b, coefficient)
            }
            Approximation::Negate(x) => self.linear(x, -coefficient),
            Approximation::Offset(x, shift) if shift.unsigned_abs() <= 1023 => {
                self.linear(x, coefficient * Computable::power_of_two_rational(*shift))
            }
            Approximation::Multiply(a, b) => {
                if let Some(q) = Self::leaf(a) {
                    self.linear(b, coefficient * q)
                } else if let Some(q) = Self::leaf(b) {
                    self.linear(a, coefficient * q)
                } else {
                    self.linear_atom(node, coefficient)
                }
            }
            _ => self.linear_atom(node, coefficient),
        }
    }

    fn logarithm(&mut self, node: &'a Computable, coefficient: Rational) -> Option<()> {
        self.charge(&coefficient)?;
        match &node.internal.approximation {
            Approximation::PrescaledExp(x) => self.linear(x, coefficient),
            Approximation::Constant(SharedConstant::E) => {
                self.constant = &self.constant + coefficient;
                Self::bounded(&self.constant).then_some(())
            }
            Approximation::One => Some(()),
            Approximation::Int(_) | Approximation::Ratio(_) => {
                self.log_rational(Self::leaf(node)?, coefficient)
            }
            Approximation::Constant(SharedConstant::Pi) => {
                self.log_term(PositiveLogAtom::Pi, coefficient)
            }
            Approximation::Constant(SharedConstant::InvPi) => {
                self.log_term(PositiveLogAtom::Pi, -coefficient)
            }
            Approximation::Constant(SharedConstant::Tau) => {
                self.term(None, coefficient.clone())?;
                self.log_term(PositiveLogAtom::Pi, coefficient)
            }
            Approximation::Constant(SharedConstant::Sqrt2) => {
                self.term(None, coefficient * HALF_RATIONAL.clone())
            }
            Approximation::Constant(SharedConstant::Sqrt3) => {
                self.log_rational(Rational::new(3), coefficient * HALF_RATIONAL.clone())
            }
            Approximation::Offset(x, shift) => {
                self.term(None, &coefficient * Rational::new(i64::from(*shift)))?;
                self.logarithm(x, coefficient)
            }
            Approximation::Sqrt(x) => self.logarithm(x, coefficient * HALF_RATIONAL.clone()),
            Approximation::Square(x) => self.logarithm(x, coefficient * Rational::new(2)),
            Approximation::Inverse(x) => self.logarithm(x, -coefficient),
            Approximation::Multiply(a, b) => {
                self.logarithm(a, coefficient.clone())?;
                self.logarithm(b, coefficient)
            }
            // Acceptance of every leaf proves positivity; do not use this
            // log-of-product/root rule on an arbitrary signed or unknown value.
            _ => None,
        }
    }
}

impl Computable {
    fn exact_positive_exp_difference_sign(&self) -> Option<Sign> {
        let Approximation::Add(a, b) = &self.internal.approximation else {
            return None;
        };
        let (a, b) = match (&a.internal.approximation, &b.internal.approximation) {
            (_, Approximation::Negate(b)) => (a, b),
            (Approximation::Negate(a), _) => (b, a),
            _ => return None,
        };
        let mut form = ExpRelationForm::new(false);
        form.logarithm(a, Rational::one())?;
        form.logarithm(b, Rational::new(-1))?;
        // Both values are positive and log is strictly increasing. Once all
        // opaque terms cancel, sign(a - b) = sign(log(a) - log(b)); preserve
        // a nonzero rational remainder rather than falling back to refinement.
        (form.terms.is_empty() && form.logarithms.is_empty()).then(|| form.constant.sign())
    }

    /// Compare positive product recipes while proving every traversed operand
    /// is defined. A declined proof is not a claim of inequality. This uses the
    /// same 128 visits, 16 atoms and 1024-bit coefficient limits as the native
    /// exponential relation; it neither refines nor changes either graph.
    #[cfg(feature = "serde")]
    pub(crate) fn compare_defined_positive_products(&self, other: &Self) -> Option<Sign> {
        let mut form = ExpRelationForm::new(true);
        form.logarithm(self, Rational::one())?;
        form.logarithm(other, Rational::new(-1))?;
        (form.terms.is_empty() && form.logarithms.is_empty()).then(|| form.constant.sign())
    }

    #[cfg(test)]
    fn exact_positive_exp_difference_zero(&self) -> bool {
        self.exact_positive_exp_difference_sign() == Some(Sign::NoSign)
    }
}
