/// Proof accumulator for rational linear combinations of logarithms of recognized positive expressions.
/// The represented logarithmic difference is a rational constant plus weighted
/// opaque real terms, a distinguished ln(2) term, and logarithms of positive atoms.
/// Only a remainder with no terms or logarithms supplies a sign certificate.
///
/// The native mode assumes opaque arguments already denote valid reals; import
/// mode admits only its restricted defined grammar. Neither mode changes graph
/// nodes or requests numerical approximations. Rational access/equality may
/// normalize exact coefficients, and atom comparison may update a separate TLS cache.
///
/// Limits are 128 linear/logarithm visits, sixteen live combined atom slots and
/// 1024 bits per checked numerator or denominator. They do not bound all nested
/// structural comparisons, intermediate arithmetic, allocation or cancellation.
/// Methods may partially mutate this accumulator before returning None; callers
/// discard the entire failed attempt. A decline is not a proof of inequality.
struct ExpRelationForm<'a> {
    /// Weighted exponent atoms: None denotes ln(2), Some borrows an immutable expression.
    /// Structural matches combine and exact zero coefficients disappear. No canonical
    /// order is promised; traversal and removal order still affect recognition work.
    terms: Vec<(Option<&'a Computable>, Rational)>,
    /// Weighted logarithms of selected positive constants/rationals, sharing the sixteen-slot cap with terms.
    logarithms: Vec<(PositiveLogAtom, Rational)>,
    /// Exact rational remainder after collecting exponent leaves and ln(e) contributions.
    constant: Rational,
    /// Remaining entry charges for linear/logarithm recursion; initialized to 128.
    /// Atom searches and structural comparison work are not charged against this counter.
    remaining: usize,
    /// Whether opaque linear atoms must be trusted shared constants rather than arbitrary presumed-real graphs.
    require_defined_arguments: bool,
}

#[derive(PartialEq)]
/// Positive bases whose logarithms are collected without numerical evaluation.
/// Equality matches Pi or exact rational values; it does not discover multiplicative
/// relations between different rational bases. Powers of two use the separate ln(2) atom.
enum PositiveLogAtom {
    /// The positive constant pi, representing ln(pi).
    Pi,
    /// A positive bounded rational base, distinct from the explicit ln(2) representation.
    Rational(
        /// Exact base admitted by log_rational; this enum itself does not validate positivity.
        Rational,
    ),
}

impl<'a> ExpRelationForm<'a> {
    /// Start an empty logarithmic form with 128 recursive-entry charges.
    /// The flag selects native presumed-real arguments or conservative import definedness checks.
    fn new(require_defined_arguments: bool) -> Self {
        Self {
            terms: Vec::new(),
            logarithms: Vec::new(),
            constant: Rational::zero(),
            remaining: 128,
            require_defined_arguments,
        }
    }

    /// Check that each canonical numerator and denominator has at most 1024 bits.
    /// The maximum, not their sum, is bounded. Accessors can normalize the rational
    /// before this check; the test is not an admission bound on that work or temporary storage.
    fn bounded(coefficient: &Rational) -> bool {
        coefficient
            .numerator()
            .bits()
            .max(coefficient.denominator().bits())
            <= 1024
    }

    /// Read One or a directly stored integer/rational within the coefficient-size guard.
    /// Other tags and oversized leaves return None; no recursive rational reduction
    /// or numerical approximation is attempted. Rational access may canonicalize its value.
    fn leaf(node: &Computable) -> Option<Rational> {
        match &node.internal.approximation {
            Approximation::One => Some(Rational::one()),
            Approximation::Int(n) if n.bits() <= 1024 => Some(Rational::from_bigint(n.clone())),
            Approximation::Ratio(q) if Self::bounded(q) => Some(q.clone()),
            _ => None,
        }
    }

    /// Consume one recursive-entry charge, then apply the coefficient-size guard.
    /// The charge remains consumed if the size check fails. Neither failure has a
    /// separate typed reason here, and this counter does not measure total computation.
    fn charge(&mut self, coefficient: &Rational) -> Option<()> {
        self.remaining = self.remaining.checked_sub(1)?;
        Self::bounded(coefficient).then_some(())
    }

    /// Add a bounded coefficient of ln(2) or an opaque expression atom.
    /// Drop exact zero, merge a recognized structural match, and swap-remove a
    /// cancelled entry; a new atom requires fewer than sixteen combined live slots.
    /// A merged coefficient is written before its size check, so None can leave a
    /// partially changed form. Structural comparison has its own uncharged traversal cost.
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

    /// Add a weighted logarithm, matching bases by PositiveLogAtom equality.
    /// Merge exact coefficients and swap-remove cancellations; otherwise append a
    /// nonzero term within the shared sixteen-slot cap. Size failure for a merged
    /// sum is checked before replacing that entry. No general logarithm identities are solved.
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

    /// Expand the logarithm of a positive rational with bounded numerator/denominator.
    /// One contributes nothing; a ratio of binary powers contributes its integral
    /// exponent times ln(2). Other values remain a single exact rational log atom,
    /// without prime factorization. Reject nonpositive or oversized bases, even with zero weight.
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

    /// Retain an unexpanded linear expression under the chosen definedness contract.
    /// Native mode assumes its real domain; import mode permits only shared constants.
    /// Perform that admission before term can erase a zero coefficient, so an opaque
    /// undefined child cannot disappear merely because its weight is zero.
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

    /// Collect a rationally weighted exponent through leaves, sums, negation, small binary offsets and scalar products.
    /// Charge every entry. Offset expansion requires absolute shift at most 1023;
    /// otherwise the node remains opaque in native mode and is declined in import mode.
    /// A product is distributed only when one direct operand is an admitted rational
    /// leaf, testing the left first. Unknown syntax becomes an atom under linear_atom.
    /// This is a partial linear recognizer, not a general symbolic normalizer.
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

    /// Collect a weighted logarithm of a recognized positive product/root/inverse recipe.
    /// Use ln(exp(x)) = x, ln(e) = 1, selected pi/root constants and positive rational
    /// leaves; binary offsets add multiples of ln(2). Roots halve, squares double,
    /// inverses negate and products add coefficients. Every recursively accepted
    /// base must be positive under the selected argument-domain contract: a square
    /// of a negative or unknown base is not admitted by squaring alone.
    /// Charge every entry, preserve left-to-right traversal and propagate None without rollback.
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
    /// Prove the sign of a syntactic difference between recognized positive expressions.
    /// Accept Add(a, Negate(b)) or Add(Negate(b), a), checking the right negation first.
    /// Strict monotonicity of log transfers the rational remainder sign only after all
    /// atoms cancel. Native mode assumes any opaque exponent arguments denote reals.
    /// None combines unrecognized syntax, unresolved atoms and fixed-guard failures;
    /// it is not a sign or a typed resource outcome. This helper does not poll signals
    /// or alter numerical caches; its caller may retain the returned sign or Unknown.
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

    /// Compare positive recipes while proving definedness within a restricted import grammar.
    /// Return the sign of self-other when all logarithmic atoms cancel, including
    /// NoSign for equality. Opaque exponent atoms must be shared constants; even a
    /// zero coefficient cannot bypass this rule. Valid expressions outside this
    /// grammar can be declined, including structurally identical operands.
    /// Use the same fixed guards as the native relation without numerical refinement
    /// or graph rewriting. None does not distinguish grammar, size, visit or unresolved
    /// atom failures, and these guards do not bound all nested work or cancellation.
    #[cfg(feature = "serde")]
    pub(crate) fn compare_defined_positive_products(&self, other: &Self) -> Option<Sign> {
        let mut form = ExpRelationForm::new(true);
        form.logarithm(self, Rational::one())?;
        form.logarithm(other, Rational::new(-1))?;
        (form.terms.is_empty() && form.logarithms.is_empty()).then(|| form.constant.sign())
    }

    #[cfg(test)]
    /// Test-only projection that returns true only for a proved zero exponential difference.
    /// False combines proved nonzero with an unresolved/declined proof; it cannot certify inequality.
    fn exact_positive_exp_difference_zero(&self) -> bool {
        self.exact_positive_exp_difference_sign() == Some(Sign::NoSign)
    }
}
