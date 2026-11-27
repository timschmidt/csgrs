/// Recognized constant + pi*pi_coefficient + atan_coefficient*atan(argument).
/// All coefficients are exact rationals; at most one structurally recognized
/// principal-atan argument is retained. A zero coefficient may keep its argument.
/// This query form assumes defined operands and is not a domain validator.
/// Clone copies coefficients and shares the Computable argument and its facts.
#[derive(Clone)]
struct PiAtanLinearForm {
    /// Exact rational offset, which quarter-turn recognition requires to be zero.
    constant: Rational,
    /// Exact coefficient of pi, not the represented angle itself.
    pi: Rational,
    /// Exact multiplier of the retained principal arctangent; it may be zero.
    atan: Rational,
    /// Retained atan operand; absent only when no operand is needed by the form.
    argument: Option<Computable>,
}

/// Exact trigonometric composition fixtures for recognized quarter and diagonal rotations.
/// Raw reconstruction, refusal/retry, and optional same-process serde replay exercise
/// the retained angle without sampling its root. These tests do not build a geometric
/// pcurve or establish fresh-process replay or universal recognition completeness.
#[cfg(test)]
mod quarter_turn_atan_tests {
    use super::*;

    /// Build an unreduced expression with invalid bound evidence, Unknown exact sign and no signal.
    /// This exposes the reconstruction path instead of constructor simplifications;
    /// fixture authors supply defined operands, since the helper does not validate domains.
    fn raw(approximation: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Unknown,
            )),
            signal: None,
        }
    }

    /// Recover sin=-4/5 and cos=-3/5 from a reconstructed angle a-pi, with a=atan(4/3).
    /// The raw interpolation uses q=(pi-2*pi)/(2*pi)=-1/2 and must leave both queried
    /// angle and q unsampled. With serde enabled, an in-process JSON round trip must
    /// recover the same rational coordinates without sampling the replayed angle.
    #[test]
    fn reconstructed_periodic_interpolation_retains_exact_trigonometry() {
        // Splitting and replaying a periodic pcurve retains this expression:
        // q = (pi - 2*pi)/(2*pi), a*(1+q) - (a-2*pi)*q = a-pi.
        // Build its uncached representation, without constructor reduction.
        let a = Computable::rational(Rational::fraction(4, 3).unwrap()).atan();
        let pi = Computable::pi();
        let q = raw(Approximation::Multiply(
            raw(Approximation::Add(
                pi.clone(),
                pi.clone().shift_left(1).negate(),
            )),
            Computable::pi_inverse_constant().shift_right(1),
        ));
        assert!(q.exact_rational().is_none());
        let angle = raw(Approximation::Add(
            raw(Approximation::Multiply(
                a.clone(),
                raw(Approximation::Add(Computable::one(), q.clone())),
            )),
            raw(Approximation::Multiply(
                raw(Approximation::Add(a, pi.shift_left(1).negate())),
                q.clone(),
            ))
            .negate(),
        ));
        assert!(angle.cached().is_none());
        assert_eq!(
            angle.clone().sin().exact_rational(),
            Some(Rational::fraction(-4, 5).unwrap())
        );
        assert_eq!(
            angle.clone().cos().exact_rational(),
            Some(Rational::fraction(-3, 5).unwrap())
        );
        assert!(angle.cached().is_none());
        assert!(q.cached().is_none());
        #[cfg(feature = "serde")]
        {
            let serialized = serde_json::to_string(&angle).unwrap();
            let replay: Computable = serde_json::from_str(&serialized).unwrap();
            assert_eq!(
                replay.clone().sin().exact_rational(),
                Some(Rational::fraction(-4, 5).unwrap())
            );
            assert_eq!(
                replay.clone().cos().exact_rational(),
                Some(Rational::fraction(-3, 5).unwrap())
            );
            assert!(replay.cached().is_none());
        }
    }

    /// Check atan(±4/3)+k*pi/2 for all integers k from -9 through 9.
    /// Rotate the independent 3-4-5 coordinates by k modulo four and require exact
    /// rational sine/cosine values while the angle root stays unsampled (38 cases).
    #[test]
    fn direct_computable_quarter_turns_keep_exact_rational_coordinates() {
        for numerator in [-4, 4] {
            let argument = Computable::rational(Rational::fraction(numerator, 3).unwrap());
            for turns in -9_i64..=9 {
                let angle = argument
                    .clone()
                    .atan()
                    .add(Computable::pi().multiply_rational(Rational::fraction(turns, 2).unwrap()));
                let s = Rational::fraction(numerator, 5).unwrap();
                let c = Rational::fraction(3, 5).unwrap();
                let (s, c) = match turns.rem_euclid(4) {
                    0 => (s, c),
                    1 => (c, -s),
                    2 => (-s, -c),
                    _ => (-c, s),
                };
                assert!(angle.cached().is_none());
                assert_eq!(angle.clone().sin().exact_rational(), Some(s));
                assert_eq!(angle.clone().cos().exact_rational(), Some(c));
                assert!(angle.cached().is_none());
            }
        }
    }

    /// Require five unsupported quarter-turn forms to decline without sampling the root.
    /// Cases retain a pi/3 shift, a 10^-12 offset, doubled atan multiplicity, an
    /// irrational atan scale, or two distinct atan arguments. None is a recognition
    /// refusal, not proof that the expression lacks exact trigonometric values.
    #[test]
    fn quarter_turn_recognizer_does_not_admit_other_angles_or_discard_offsets() {
        let atan = Computable::rational(Rational::fraction(4, 3).unwrap()).atan();
        let quarter = atan.clone().add(Computable::pi().shift_right(1));
        for value in [
            atan.clone()
                .add(Computable::pi().multiply_rational(Rational::fraction(1, 3).unwrap())),
            quarter.clone().add(Computable::rational(
                Rational::fraction(1, 1_000_000_000_000).unwrap(),
            )),
            quarter.shift_left(1),
            raw(Approximation::Multiply(
                atan.clone(),
                Computable::sqrt_rational(Rational::new(2)),
            )),
            atan.add(Computable::rational(Rational::fraction(5, 12).unwrap()).atan()),
        ] {
            assert!(value.cached().is_none());
            assert!(value.quarter_turn_plus_or_minus_atan_argument().is_none());
            assert!(value.cached().is_none());
        }
    }

    /// Preserve a 10^-12 coefficient perturbation and retry an exact reconstructed quarter turn.
    /// The perturbed angle must decline; depth-entry budgets zero and one fail on the
    /// unperturbed form, whose later ordinary recognition succeeds without sampling.
    /// This checks refusal/retry, not a total-work bound or negative cached evidence.
    #[test]
    fn unrecognized_or_exhausted_coefficients_do_not_invent_quarter_turns() {
        let atan = Computable::rational(Rational::fraction(4, 3).unwrap()).atan();
        let ratio = raw(Approximation::Multiply(
            Computable::pi(),
            Computable::pi_inverse_constant().shift_right(1),
        ));
        let tiny = Computable::rational(Rational::fraction(1, 1_000_000_000_000).unwrap());
        let off_quarter = raw(Approximation::Add(
            atan.clone(),
            raw(Approximation::Multiply(
                raw(Approximation::Add(ratio.clone(), tiny)),
                Computable::pi(),
            )),
        ));
        assert!(
            off_quarter
                .quarter_turn_plus_or_minus_atan_argument()
                .is_none()
        );
        assert!(off_quarter.cached().is_none());
        let quarter = raw(Approximation::Add(
            atan,
            raw(Approximation::Multiply(ratio, Computable::pi())),
        ));
        assert!(quarter.pi_atan_linear_form(0).is_none());
        assert!(quarter.pi_atan_linear_form(1).is_none());
        // A smaller unsuccessful search must not poison the later certificate.
        assert!(quarter.quarter_turn_plus_or_minus_atan_argument().is_some());
        assert!(quarter.cached().is_none());
    }

    /// Recognize atan(4/3)+pi/4 after a zero-budget attempt declines.
    /// Require positive sine, negative cosine and exact squared values 49/50 and 1/50
    /// via bounded Laurent proofs. The generated values need not be rational leaves;
    /// the original angle must remain unsampled.
    #[test]
    fn diagonal_rotation_reuses_exact_coefficients_without_refinement() {
        let atan = Computable::rational(Rational::fraction(4, 3).unwrap()).atan();
        let angle = raw(Approximation::Add(atan, Computable::pi().shift_right(2)));
        assert!(angle.pi_atan_linear_form(0).is_none());
        assert!(angle.diagonal_turn_plus_or_minus_atan_argument().is_some());
        let sine = angle.clone().sin();
        let cosine = angle.clone().cos();
        assert_eq!(sine.exact_sign(), Some(Sign::Plus));
        assert_eq!(cosine.exact_sign(), Some(Sign::Minus));
        // These are composed quadratic values, not rational leaves. Require
        // the exact value certificate, without claiming leaf representation.
        assert_eq!(
            sine.square().bounded_laurent_rational(64),
            Some(Rational::fraction(49, 50).unwrap())
        );
        assert_eq!(
            cosine.square().bounded_laurent_rational(64),
            Some(Rational::fraction(1, 50).unwrap())
        );
        assert!(angle.cached().is_none());
    }

    /// Decline five forms outside the odd-pi/4, unit-atan recognizer while retaining unsampled roots.
    /// Keep a 10^-12 offset, doubled atan multiplicity, pi/3 and pi/2 shifts, and a
    /// second distinct atan argument explicit. A quarter turn belongs to another recognizer.
    #[test]
    fn diagonal_rotation_never_discards_offsets_or_changes_atan_multiplicity() {
        let atan = Computable::rational(Rational::fraction(4, 3).unwrap()).atan();
        let diagonal = atan.clone().add(Computable::pi().shift_right(2));
        for value in [
            diagonal.clone().add(Computable::rational(
                Rational::fraction(1, 1_000_000_000_000).unwrap(),
            )),
            diagonal.shift_left(1),
            atan.clone()
                .add(Computable::pi().multiply_rational(Rational::fraction(1, 3).unwrap())),
            atan.clone().add(Computable::pi().shift_right(1)),
            atan.add(Computable::rational(Rational::fraction(5, 12).unwrap()).atan()),
        ] {
            assert!(value.diagonal_turn_plus_or_minus_atan_argument().is_none());
            assert!(value.cached().is_none());
        }
    }
}

/// Check optional Laurent reconstruction's success/refusal without sampling the queried root.
/// This fixture distinguishes a covered radical polynomial from a surviving opaque sine;
/// semantic equality and evidence-retention consumers have separate tests.
#[cfg(test)]
mod pi_laurent_compaction_node_tests {
    use super::*;

    /// Require compaction of (pi+sqrt(13))²-pi² and refusal of sin(7)+sqrt(13).
    /// Both roots stay unsampled before and after the call. The test checks admission
    /// and cache state, not the reconstructed value's equality by itself.
    #[test]
    fn compact_pi_laurent_uses_no_approximation_for_success_or_failure() {
        let root = Computable::sqrt_rational(Rational::from(13));
        let binomial = Computable::pi().add(root.clone());
        let covered = binomial.square().add(Computable::pi().square().negate());
        let opaque = Computable::rational(Rational::from(7)).sin().add(root);
        for (value, supported) in [(covered, true), (opaque, false)] {
            assert!(value.cached().is_none());
            assert_eq!(value.compact_pi_laurent().is_some(), supported);
            assert!(value.cached().is_none());
        }
    }
}

/// Recognized rational offset plus rational multiples of pi and one principal acos.
/// The optional retained argument must already be admitted in [-1,1]. Complementary
/// nonnegative rational arguments may share this form through their exact unit-circle
/// relation. Clone copies coefficients and shares the argument; it adds no domain proof.
#[derive(Clone)]
struct PiAcosLinearForm {
    /// Exact rational offset independent of the inverse angle and pi.
    constant: Rational,
    /// Exact rational coefficient of pi, including complementary-angle adjustments.
    pi: Rational,
    /// Exact coefficient of the retained principal acos, possibly zero.
    acos: Rational,
    /// Retained acos operand; zero coefficients may still retain an argument.
    argument: Option<Computable>,
}

/// Rational offset and pi coefficient with a linear list of principal arctangent terms.
/// Distinct structurally recognized operands remain separate for sign/order deductions.
/// This is not a minimal transcendence basis or a general atan addition formula.
/// Clone copies the list and coefficients while sharing its Computable operands.
#[derive(Clone)]
struct PiAtanTermsForm {
    /// Exact rational offset used by the sign recognizer.
    constant: Rational,
    /// Exact rational multiplier of pi.
    pi: Rational,
    /// Pairs of retained atan operand and rational multiplier; scaling may leave zero multipliers.
    terms: Vec<(Computable, Rational)>,
}

// A bounded recognition form for exact rational combinations of positive
// rational square roots and opaque linear atoms. Independent radicals retain
// their identities instead of turning their entire sum into one opaque atom.
/// Exact rational linear combination of principal rational roots and opaque scalar atoms.
/// Root entries hold (radicand, multiplier); atom entries hold (value, multiplier).
/// Operands must already be defined and roots nonnegative. Square extraction is
/// partial, so distinct stored radicals need not be algebraically independent.
/// Clone copies vectors and rationals while sharing opaque nodes and retained facts.
#[derive(Clone)]
struct RadicalCoefficient {
    /// Exact root-free and atom-free summand.
    rational: Rational,
    /// Nonzero coefficients keyed by retained radicands; normally at most eight, without full factorization.
    radicals: Vec<(Rational, Rational)>,
    /// Opaque value/coefficient pairs merged by structural recognition; no local atom-count cap.
    atoms: Vec<(Computable, Rational)>,
}

/// Optional exact coefficient arithmetic with restricted nonlinear combinations.
/// None declines this representation; it is not a typed domain/resource diagnosis.
/// These helpers neither poll cancellation nor impose a coefficient-bit or total-work limit.
impl RadicalCoefficient {
    // Bounds optional symbolic recognition, never the underlying exact DAG.
    /// Maximum distinct retained radical entries accepted by insertion; not a bound on opaque atoms.
    const MAX_RADICAL_TERMS: usize = 8;

    /// Create a coefficient containing only the exact rational summand, including canonical zero.
    fn rational(value: Rational) -> Self {
        Self {
            rational: value,
            radicals: Vec::new(),
            atoms: Vec::new(),
        }
    }

    /// Represent value*sqrt(radicand), requiring a nonnegative rational radicand.
    /// Reuse retained partial square extraction, absorb its scale and fold residual one
    /// or a zero multiplier into the rational part. This does not fully factor every radicand.
    fn radical(value: Rational, radicand: Rational) -> Self {
        let (scale, radicand) = radicand.extract_square_reduced_retained();
        let value = value * scale;
        if radicand.is_one() || value.sign() == Sign::NoSign {
            return Self::rational(value);
        }
        Self {
            rational: Rational::zero(),
            radicals: vec![(radicand, value)],
            atoms: Vec::new(),
        }
    }

    /// Retain an already-defined scalar as one opaque atom with coefficient one.
    /// No rational/zero recognition or domain validation is attempted here.
    fn atom(value: Computable) -> Self {
        Self {
            rational: Rational::zero(),
            radicals: Vec::new(),
            atoms: vec![(value, Rational::one())],
        }
    }

    /// Recognize the empty exact form: zero rational part and no radical or atom entries.
    /// False means this stored form is not empty, not proof that its scalar value is nonzero.
    fn is_zero(&self) -> bool {
        self.rational.sign() == Sign::NoSign && self.radicals.is_empty() && self.atoms.is_empty()
    }

    /// Merge a normalized radical term by exact radicand equality, dropping zero coefficients.
    /// Residual one contributes to the rational part; other radicands are not reduced here.
    /// At eight distinct entries a new key returns None without mutation, while existing
    /// keys can still cancel. Callers maintain the cap and fold zero radicands beforehand.
    fn insert_radical(&mut self, radicand: Rational, coefficient: Rational) -> Option<()> {
        if coefficient.sign() == Sign::NoSign {
            return Some(());
        }
        if radicand.is_one() {
            self.rational = &self.rational + coefficient;
            return Some(());
        }
        if let Some(index) = self
            .radicals
            .iter()
            .position(|(candidate, _)| *candidate == radicand)
        {
            self.radicals[index].1 = &self.radicals[index].1 + coefficient;
            if self.radicals[index].1.sign() == Sign::NoSign {
                self.radicals.remove(index);
            }
        } else {
            if self.radicals.len() == Self::MAX_RADICAL_TERMS {
                return None;
            }
            self.radicals.push((radicand, coefficient));
        }
        Some(())
    }

    /// Merge a linear atom by internal structural equality and drop a cancelled coefficient.
    /// A zero input coefficient is ignored. Distinct but mathematically equal opaque
    /// values need not merge; this method has no separate atom-count limit.
    fn insert_atom(&mut self, atom: Computable, coefficient: Rational) {
        if coefficient.sign() == Sign::NoSign {
            return;
        }
        if let Some(index) = self
            .atoms
            .iter()
            .position(|(candidate, _)| Computable::internal_structural_eq(candidate, &atom))
        {
            self.atoms[index].1 = &self.atoms[index].1 + coefficient;
            if self.atoms[index].1.sign() == Sign::NoSign {
                self.atoms.remove(index);
            }
        } else {
            self.atoms.push((atom, coefficient));
        }
    }

    /// Consume two coefficients, merging rationals, radicals and then opaque atoms in order.
    /// Decline if an intermediate radical insertion exceeds its cap, even if a later term
    /// could cancel it. Partial local work is discarded with the consumed value on None.
    fn add(mut self, other: Self) -> Option<Self> {
        self.rational = self.rational + other.rational;
        for (radicand, coefficient) in other.radicals {
            self.insert_radical(radicand, coefficient)?;
        }
        for (atom, coefficient) in other.atoms {
            self.insert_atom(atom, coefficient);
        }
        Some(self)
    }

    /// Multiply every summand by an exact rational, removing zero radical/atom entries.
    /// Scaling by zero produces the empty coefficient form. No domain validation is added.
    fn scale_in_place(&mut self, scale: &Rational) {
        self.rational *= scale;
        for (_, coefficient) in &mut self.radicals {
            *coefficient *= scale;
        }
        for (_, coefficient) in &mut self.atoms {
            *coefficient *= scale;
        }
        self.radicals
            .retain(|(_, coefficient)| coefficient.sign() != Sign::NoSign);
        self.atoms
            .retain(|(_, coefficient)| coefficient.sign() != Sign::NoSign);
    }

    /// Distribute supported products, using equal-root squares and partial square extraction.
    /// Rational factors may scale opaque atoms, but atom*atom and atom*radical products
    /// decline before expansion. Intermediate radical-cap failures also decline;
    /// None does not imply the exact product is undefined or lacks another representation.
    fn multiply(self, other: &Self) -> Option<Self> {
        if (!self.atoms.is_empty() && (!other.radicals.is_empty() || !other.atoms.is_empty()))
            || (!other.atoms.is_empty() && !self.radicals.is_empty())
        {
            return None;
        }
        let mut result = Self::rational(&self.rational * &other.rational);
        for (radicand, coefficient) in &self.radicals {
            result.insert_radical(radicand.clone(), coefficient * &other.rational)?;
        }
        for (radicand, coefficient) in &other.radicals {
            result.insert_radical(radicand.clone(), coefficient * &self.rational)?;
        }
        for (left, left_coefficient) in self.radicals {
            for (right, right_coefficient) in &other.radicals {
                let coefficient = &left_coefficient * right_coefficient;
                if left == *right {
                    result.rational = result.rational + coefficient * &left;
                } else {
                    let (scale, radicand) = (&left * right).extract_square_reduced_retained();
                    result.insert_radical(radicand, coefficient * scale)?;
                }
            }
        }
        for (atom, coefficient) in self.atoms {
            result.insert_atom(atom, coefficient * &other.rational);
        }
        for (atom, coefficient) in &other.atoms {
            result.insert_atom(atom.clone(), coefficient * &self.rational);
        }
        Some(result)
    }

    /// Invert a nonzero rational or an atom-free single-radical form using its conjugate norm.
    /// For a+b*sqrt(d), require invertible rational norm a²-b²*d, then return
    /// (a-b*sqrt(d))/norm. Atoms, multiple radicals and zero norm decline; zero norm
    /// alone is not a proof that the original value is zero. Caller domain assumptions persist.
    fn inverse(self) -> Option<Self> {
        if !self.atoms.is_empty() {
            return None;
        }
        match self.radicals.as_slice() {
            [] => Some(Self::rational(self.rational.inverse().ok()?)),
            [(radicand, coefficient)] => {
                let norm = &self.rational * &self.rational - coefficient * coefficient * radicand;
                let inverse_norm = norm.inverse().ok()?;
                Some(Self {
                    rational: self.rational * &inverse_norm,
                    radicals: vec![(radicand.clone(), -coefficient * inverse_norm)],
                    atoms: Vec::new(),
                })
            }
            _ => None,
        }
    }
}

/// Sorted finite sum of pi^power times restricted exact RadicalCoefficient values.
/// Powers are signed i16 and distinct; structurally empty coefficients are omitted.
/// The empty vector denotes zero. This bounded recognition form does not replace
/// the source DAG or prove independence of opaque atoms; Clone copies coefficient
/// storage while retaining shared opaque dependencies.
#[derive(Clone)]
struct PiLaurentPolynomial {
    /// Increasing signed powers paired with nonempty coefficients; normal insertion admits at most eight.
    terms: Vec<(i16, RadicalCoefficient)>,
}

// This normal form is a bounded recognition fast path, not an exactness
// authority. A depth limit alone still permits exponential traversal of a
// wide, mostly unshared arithmetic DAG before the query returns `None`.
// Bound aggregate node visits as well; callers retain their ordinary exact
// representation when this recognition budget is exhausted.
/// Limit charged uncached node visits in one Laurent recognition query to 512.
/// Shared-node memo hits bypass the charge; recursive depth is a separate input.
/// Coefficient arithmetic, allocations and structural comparisons are not fully
/// metered by this count, and exhaustion can leave an exact opaque remainder.
const MAX_PI_LAURENT_NORMAL_FORM_NODES: usize = 512;

/// Preserve sorted Laurent terms with checked exponents and restricted coefficient arithmetic.
/// Optional failure keeps the source DAG authoritative; these helpers do not poll
/// cancellation or provide total-work, coefficient-growth or memory accounting.
impl PiLaurentPolynomial {
    /// Maximum distinct pi powers admitted during insertion, including intermediate expansion.
    const MAX_TERMS: usize = 8;

    /// Create one term at any representable signed power, or the empty polynomial for structural zero.
    /// The coefficient is trusted to satisfy its own domain/storage invariants.
    fn monomial(power: i16, coefficient: RadicalCoefficient) -> Self {
        Self {
            terms: (!coefficient.is_zero())
                .then_some((power, coefficient))
                .into_iter()
                .collect(),
        }
    }

    /// Insert in increasing power order or merge an existing coefficient, removing structural zero.
    /// Decline a ninth distinct power or failed coefficient addition. The receiver stays
    /// unchanged on either failure because a matching coefficient is cloned before merging.
    fn insert(&mut self, power: i16, coefficient: RadicalCoefficient) -> Option<()> {
        if coefficient.is_zero() {
            return Some(());
        }
        match self
            .terms
            .binary_search_by_key(&power, |(candidate, _)| *candidate)
        {
            Ok(index) => {
                let sum = self.terms[index].1.clone().add(coefficient)?;
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

    /// Consume and merge terms in the right operand's stored order, preserving sorted powers.
    /// An intermediate term/coefficient cap can decline before later cancellations;
    /// partial changes are confined to the consumed local value.
    fn add(mut self, other: Self) -> Option<Self> {
        for (power, coefficient) in other.terms {
            self.insert(power, coefficient)?;
        }
        Some(self)
    }

    /// Scale every coefficient by an exact rational, clearing the polynomial for scale zero.
    /// For admitted nonzero coefficients, a nonzero rational scale preserves stored powers.
    fn scaled(mut self, scale: Rational) -> Self {
        if scale.sign() == Sign::NoSign {
            self.terms.clear();
            return self;
        }
        for (_, coefficient) in &mut self.terms {
            coefficient.scale_in_place(&scale);
        }
        self
    }

    /// Convolve term pairs with checked i16 exponent sums and restricted coefficient products.
    /// Decline exponent overflow, unsupported products or intermediate term/radical caps.
    /// A single right term consumes each left coefficient once, preserving insertion
    /// order; other products clone it per right term. Empty factors yield empty output.
    fn multiply(self, other: Self) -> Option<Self> {
        let mut product = Self { terms: Vec::new() };
        if let [(right_power, right_coefficient)] = other.terms.as_slice() {
            // Each owned left coefficient is consumed exactly once when the
            // right polynomial has one term. Preserve the ordinary insertion
            // order and all recognition limits without copying its vectors.
            for (left_power, left_coefficient) in self.terms {
                product.insert(
                    left_power.checked_add(*right_power)?,
                    left_coefficient.multiply(right_coefficient)?,
                )?;
            }
            return Some(product);
        }
        for (left_power, left_coefficient) in self.terms {
            for (right_power, right_coefficient) in &other.terms {
                product.insert(
                    left_power.checked_add(*right_power)?,
                    left_coefficient.clone().multiply(right_coefficient)?,
                )?;
            }
        }
        Some(product)
    }

    /// Invert exactly one term by checked exponent negation and coefficient inversion.
    /// Empty or multi-term polynomials, i16::MIN exponents and unrecognized coefficient
    /// inverses decline; this is not a general rational-function inversion algorithm.
    fn inverse_monomial(mut self) -> Option<Self> {
        if self.terms.len() != 1 {
            return None;
        }
        let (power, coefficient) = self.terms.pop()?;
        Some(Self::monomial(power.checked_neg()?, coefficient.inverse()?))
    }
}

/// Combine linear atan evidence without evaluating inverse functions or polling cancellation.
/// Structural matching and rational arithmetic have no independent list-size or work cap.
impl PiAtanTermsForm {
    /// Multiply the offset, pi multiplier and every atan coefficient by the rational scale.
    /// Retain the operand list even when coefficients become zero; add performs pruning.
    fn scaled(mut self, scale: Rational) -> Self {
        self.constant *= &scale;
        self.pi *= &scale;
        for (_, coefficient) in &mut self.terms {
            *coefficient *= &scale;
        }
        self
    }

    /// Merge structurally matching atan operands, add offset/pi coefficients and remove zero terms.
    /// Preserve existing order, appending new operands. Equality is structural recognition,
    /// not a decision procedure for all mathematically equal arctangents.
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

/// Combine a single principal-acos basis with exact rational scaling and complementary angles.
/// Inputs must already satisfy their real domains; no approximation or cancellation poll occurs.
impl PiAcosLinearForm {
    /// Scale all rational coefficients while retaining the optional argument, even at zero scale.
    fn scaled(mut self, scale: Rational) -> Self {
        self.constant *= &scale;
        self.pi *= &scale;
        self.acos *= scale;
        self
    }

    /// Merge the same structural acos operand or convert a nonnegative rational complement.
    /// For distinct arguments a,b, require a,b>=0 and a²+b²=1, then substitute
    /// acos(b)=pi/2-acos(a) and retain a. Signs are essential to select the principal
    /// branches. Unsupported distinct arguments decline even if a coefficient is zero;
    /// the result may retain an argument whose final coefficient cancels.
    fn add(self, mut other: Self) -> Option<Self> {
        let argument = match (&self.argument, &other.argument) {
            (Some(left), Some(right)) if !Computable::internal_structural_eq(left, right) => {
                // In the first quadrant complementary sine/cosine arguments
                // give acos(a)+acos(b)=pi/2. The exact unit-circle equation
                // and both signs are necessary; squares alone lose the branch.
                let a = left.exact_rational()?;
                let b = right.exact_rational()?;
                if a.sign() == Sign::Minus
                    || b.sign() == Sign::Minus
                    || &a * &a + &b * &b != Rational::one()
                {
                    return None;
                }
                other.pi =
                    other.pi + &other.acos * Rational::fraction(1, 2).expect("two is nonzero");
                other.acos = -other.acos;
                Some(left.clone())
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

/// Combine a single principal-atan basis and extract exact quarter-turn coordinate permutations.
/// Defined operands are a precondition; these helpers do not refine or poll cancellation.
impl PiAtanLinearForm {
    /// Recognize k*pi/2 ± atan(x), returning (swap, sine_sign, cosine_sign, x).
    /// Require zero rational offset, atan coefficient ±1 and an existing operand.
    /// For even k, signs multiply sin(atan(x)) and cos(atan(x)); for odd k, swap those
    /// base coordinates first. Exact parity handles negative turns without reducing x.
    /// Other coefficients/turns decline, rather than being rounded to a supported angle.
    fn quarter_turn_parts(self) -> Option<(bool, Sign, Sign, Computable)> {
        if self.constant.sign() != Sign::NoSign
            || (!self.atan.is_one() && !self.atan.is_minus_one())
        {
            return None;
        }
        // Keep the integer-pi path free of a new rational scaling step. Odd multiples
        // of pi/2 exchange the sine and cosine numerators; their signs follow
        // the exact quarter-turn rotation, without refining either angle.
        let Some(multiple) = self.pi.integer_magnitude() else {
            let doubled = &self.pi * Rational::new(2);
            let quarter_turns = doubled.integer_magnitude()?;
            let sine_sign = if quarter_turns.bit(1) ^ (self.pi.sign() == Sign::Minus) {
                Sign::Minus
            } else {
                Sign::Plus
            };
            let cosine_sign = if self.atan.is_minus_one() {
                sine_sign
            } else if sine_sign == Sign::Plus {
                Sign::Minus
            } else {
                Sign::Plus
            };
            return Some((true, sine_sign, cosine_sign, self.argument?));
        };
        let cosine_sign = if multiple.bit(0) {
            Sign::Minus
        } else {
            Sign::Plus
        };
        let sine_sign = if self.atan.is_minus_one() {
            match cosine_sign {
                Sign::Plus => Sign::Minus,
                Sign::Minus => Sign::Plus,
                Sign::NoSign => unreachable!("integer-pi parity has a nonzero orientation"),
            }
        } else {
            cosine_sign
        };
        Some((false, sine_sign, cosine_sign, self.argument?))
    }

    /// Scale the rational offset, pi and atan coefficients without dropping the retained argument.
    fn scaled(mut self, scale: Rational) -> Self {
        self.constant *= &scale;
        self.pi *= &scale;
        self.atan *= scale;
        self
    }

    /// Add coefficients only when retained atan arguments match structurally or one is absent.
    /// Different arguments decline even with zero multipliers. Cancellation may leave a
    /// zero atan coefficient with its original operand still present.
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
    /// Recognize a restricted Laurent form using the supplied depth budget and 512 node charges.
    /// Start with no shared-node memo; unsupported subexpressions may survive as exact
    /// opaque atoms. None is a recognition refusal, not a proof of irrationality or
    /// invalidity. Operands must already be defined; this query is not a domain validator
    /// and neither approximates nor polls the signal. The counters are not total resource bounds.
    fn pi_laurent_polynomial(&self, budget: usize) -> Option<PiLaurentPolynomial> {
        let mut memo = None;
        let mut remaining_work = MAX_PI_LAURENT_NORMAL_FORM_NODES;
        self.pi_laurent_polynomial_with_memo(budget, &mut remaining_work, &mut memo)
    }

    /// Recursively retain exact rational, shared-constant, radical and opaque-atom structure.
    /// Use a query-local allocation-keyed memo only for shared nodes; a hit returns before
    /// depth/work checks. Otherwise charge one visit, then descend with depth minus one.
    /// Inverse trig leaves remain atoms; supported arithmetic expands within coefficient
    /// and exponent limits. Expansion failure becomes an opaque atom for that whole node,
    /// so a successful form need not be fully expanded or fit reconstruction's envelope.
    ///
    /// Nonnegative rational sqrt arguments can become radical coefficients. Inverses
    /// expand only for admitted monomials with a recognized coefficient inverse;
    /// unrecognized inverses can remain opaque. Later structural cancellation assumes
    /// all original operands are defined, rather than independently validating them.
    /// Memo entries include the current query's result, potentially an opaque fallback;
    /// no failed recognition is stored as a persistent negative mathematical fact.
    fn pi_laurent_polynomial_with_memo(
        &self,
        budget: usize,
        remaining_work: &mut usize,
        memo: &mut Option<Vec<(usize, Option<PiLaurentPolynomial>)>>,
    ) -> Option<PiLaurentPolynomial> {
        let key = Arc::as_ptr(&self.internal) as usize;
        let shared = Arc::strong_count(&self.internal) > 1;
        if shared
            && let Some((_, value)) = memo
                .as_ref()
                .and_then(|memo| memo.iter().find(|(candidate, _)| *candidate == key))
        {
            return value.clone();
        }
        if budget == 0 || *remaining_work == 0 {
            return None;
        }
        *remaining_work -= 1;
        let result = if let Some(rational) = self.exact_rational() {
            Some(PiLaurentPolynomial::monomial(
                0,
                RadicalCoefficient::rational(rational),
            ))
        } else if let Some(polynomial) = match self.shared_constant_kind() {
            Some(SharedConstant::Pi) => Some(PiLaurentPolynomial::monomial(
                1,
                RadicalCoefficient::rational(Rational::one()),
            )),
            Some(SharedConstant::InvPi) => Some(PiLaurentPolynomial::monomial(
                -1,
                RadicalCoefficient::rational(Rational::one()),
            )),
            Some(SharedConstant::Tau) => Some(PiLaurentPolynomial::monomial(
                1,
                RadicalCoefficient::rational(Rational::new(2)),
            )),
            Some(SharedConstant::Sqrt2) => Some(PiLaurentPolynomial::monomial(
                0,
                RadicalCoefficient::radical(Rational::one(), Rational::new(2)),
            )),
            Some(SharedConstant::Sqrt3) => Some(PiLaurentPolynomial::monomial(
                0,
                RadicalCoefficient::radical(Rational::one(), Rational::new(3)),
            )),
            _ => None,
        } {
            Some(polynomial)
        } else if self.atan_argument().is_some()
            || self.asin_argument().is_some()
            || self.acos_argument().is_some()
        {
            Some(PiLaurentPolynomial::monomial(
                0,
                RadicalCoefficient::atom(self.clone()),
            ))
        } else {
            let expanded = (|| match &self.internal.approximation {
                Approximation::Add(left, right) => left
                    .pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?
                    .add(right.pi_laurent_polynomial_with_memo(
                        budget - 1,
                        remaining_work,
                        memo,
                    )?),
                Approximation::Multiply(left, right) => left
                    .pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?
                    .multiply(right.pi_laurent_polynomial_with_memo(
                        budget - 1,
                        remaining_work,
                        memo,
                    )?),
                Approximation::Negate(child) => Some(
                    child
                        .pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?
                        .scaled(Rational::new(-1)),
                ),
                Approximation::Offset(child, shift) => Some(
                    child
                        .pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?
                        .scaled(Self::power_of_two_rational(*shift)),
                ),
                Approximation::Inverse(child) => child
                    .pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?
                    .inverse_monomial(),
                Approximation::Square(child) => {
                    let child =
                        child.pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?;
                    child.clone().multiply(child)
                }
                Approximation::Sqrt(child) => {
                    let child =
                        child.pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?;
                    let radicand = match child.terms.as_slice() {
                        [] => Rational::zero(),
                        [(0, coefficient)]
                            if coefficient.radicals.is_empty() && coefficient.atoms.is_empty() =>
                        {
                            coefficient.rational.clone()
                        }
                        _ => return None,
                    };
                    if radicand.sign() == Sign::Minus {
                        return None;
                    }
                    Some(PiLaurentPolynomial::monomial(
                        0,
                        RadicalCoefficient::radical(Rational::one(), radicand),
                    ))
                }
                Approximation::LinearCombination3(combination) => {
                    let mut sum = PiLaurentPolynomial::monomial(
                        0,
                        RadicalCoefficient::rational(Rational::zero()),
                    );
                    for (coefficient, value) in combination
                        .coefficients
                        .iter()
                        .zip(combination.values.iter())
                    {
                        sum = sum.add(
                            coefficient
                                .pi_laurent_polynomial_with_memo(budget - 1, remaining_work, memo)?
                                .scaled(value.clone()),
                        )?;
                    }
                    Some(sum)
                }
                _ => None,
            })();
            expanded.or_else(|| {
                Some(PiLaurentPolynomial::monomial(
                    0,
                    RadicalCoefficient::atom(self.clone()),
                ))
            })
        };
        if shared {
            memo.get_or_insert_with(|| Vec::with_capacity(8))
                .push((key, result.clone()));
        }
        result
    }

    /// Return a rational only when the optional Laurent form is empty or purely rational at power zero.
    /// On success retain its exact sign in the shared root facts; do not change the source
    /// representation or retain the rational value itself here. None leaves no negative
    /// proof and may reflect limits or a surviving radical/atom/power. Defined input
    /// operands are required; this recognizer does not certify arbitrary imported domains.
    /// For example, a raw inverse-zero expression minus itself can cancel here and
    /// publish zero; callers must validate domains before invoking this proof path.
    pub(crate) fn bounded_laurent_rational(&self, budget: usize) -> Option<Rational> {
        let polynomial = self.pi_laurent_polynomial(budget)?;
        let rational = match polynomial.terms.as_slice() {
            [] => Some(Rational::zero()),
            [(0, coefficient)]
                if coefficient.radicals.is_empty() && coefficient.atoms.is_empty() =>
            {
                Some(coefficient.rational.clone())
            }
            _ => None,
        }?;
        // A successful symbolic proof remains valid for every clone. Retain
        // its sign so later predicates do not rebuild the same normal form.
        self.internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(rational.sign()));
        Some(rational)
    }

    /// Reconstruct a depth-64 recognized Laurent form only when no opaque atoms survive.
    /// Build exact rational/root coefficients and signed pi powers by repeated squaring,
    /// then add terms. An empty polynomial reconstructs scalar zero. Preserve an already
    /// valid root sign and the original outer abort signal.
    ///
    /// None means recognition or reconstruction eligibility declined, leaving the original
    /// value available. No numerical refinement or signal poll occurs, but construction
    /// can allocate and reuse/store exact facts. The depth/node/term limits do not bound
    /// all coefficient work or retained memory; defined source operands remain a prerequisite.
    pub(crate) fn compact_pi_laurent(&self) -> Option<Self> {
        let polynomial = self.pi_laurent_polynomial(64)?;
        // Rebuild only coefficients in the admitted rational/radical basis.
        // An opaque remainder may hide an arbitrarily large graph, including
        // one left opaque after the recognition budget was exhausted.
        if polynomial
            .terms
            .iter()
            .any(|(_, coefficient)| !coefficient.atoms.is_empty())
        {
            return None;
        }
        let mut result = Self::zero();
        for (power, coefficient) in polynomial.terms {
            let mut value = Self::rational(coefficient.rational);
            for (radicand, scale) in coefficient.radicals {
                value = value.add(Self::sqrt_rational(radicand).multiply_rational(scale));
            }
            let mut exponent = power.unsigned_abs();
            let mut factor = if power < 0 {
                Self::pi_inverse_constant()
            } else {
                Self::pi()
            };
            while exponent != 0 {
                if exponent & 1 != 0 {
                    value = value.multiply(factor.clone());
                }
                exponent >>= 1;
                if exponent != 0 {
                    factor = factor.square();
                }
            }
            result = result.add(value);
        }
        // Reconstruction proves value equality, so an already certified sign
        // remains valid. Losing it could turn a later low-budget predicate
        // into Unknown even though the original expression decided exactly.
        if let ExactSignCache::Valid(sign) = self.internal.facts.exact_sign() {
            result
                .internal
                .facts
                .replace_exact_sign(ExactSignCache::Valid(sign));
        }
        result.signal = self.signal.clone();
        Some(result)
    }

    /// Extract one directly retained shared factor through negation and binary scaling.
    /// Return r only for recognized self=factor*r: a direct constant gives one, and a
    /// product with a direct constant child gives the other child. Do not search nested
    /// products or sums. Multiplication uses this to cancel pi with its reciprocal.
    /// Defined source operands are required; this is not domain validation.
    /// There is no explicit recursion/work limit or signal poll; offset arithmetic and
    /// allocation must remain admissible. A directly returned child retains its own
    /// signal; reconstructed wrappers need not transport the outer signal or become
    /// rational leaves, even when their exact value is rational.
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

    /// Recognize a rational offset plus rational multiples of pi and one principal atan.
    /// At positive depth, try rational metadata, direct pi and retained atan operands,
    /// then Add, Negate, Offset and products with an exact-rational child, in that order.
    /// Distinct atan operands must merge structurally; multiplication prefers the left
    /// rational scale. Unsupported forms or depth zero decline without refinement.
    /// Budget limits recursive depth, not aggregate visits, coefficient growth or nested
    /// helper work. Defined operands and admissible offset shifts are preconditions;
    /// this optional reader adds no cancellation poll or independent domain validation.
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
                    return Some(right.pi_atan_linear_form(budget - 1)?.scaled(scale));
                }
                if let Some(scale) = right.exact_rational() {
                    return Some(left.pi_atan_linear_form(budget - 1)?.scaled(scale));
                }
                None
            }
            _ => None,
        }
    }

    /// Recognize orientation*pi/2-atan(x), with orientation exactly ±1, at depth 32.
    /// Require zero rational offset and atan coefficient -1, returning the orientation
    /// and retained operand. This reader alone does not require x to have that sign
    /// and does not prove the resulting angle lies on a reciprocal atan's principal branch.
    fn half_pi_minus_atan_argument(&self) -> Option<(Sign, Computable)> {
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
        Some((orientation, form.argument?))
    }

    /// Recognize orientation*pi/2-atan(x) only when x has the same proved nonzero sign.
    /// Return the orientation and retained operand, establishing the principal identity
    /// atan(1/x) without evaluating the angle. A missing sign or structural form declines;
    /// the underlying depth-32 reader and exact-sign helpers are not a total-work budget.
    pub(crate) fn signed_half_pi_minus_atan_argument(&self) -> Option<(Sign, Computable)> {
        let (orientation, argument) = self.half_pi_minus_atan_argument()?;
        (argument.exact_sign()? == orientation).then_some((orientation, argument))
    }

    /// Recover (swap, sine_sign, cosine_sign, x) for recognized k*pi/2 ± atan(x).
    /// Signs multiply the original atan(x) coordinates, after swapping them when requested.
    /// Try direct linear recognition before Laurent recovery, each with depth 64.
    /// Disable pure-pi anchor recovery on this entry point to retain Real's established
    /// recognition path; None does not deny an exact trig identity. Defined inputs and
    /// the underlying resource preconditions remain required; no approximation is used.
    pub(crate) fn quarter_turn_plus_or_minus_atan_argument(
        &self,
    ) -> Option<(bool, Sign, Sign, Computable)> {
        // Real already has canonical rational-pi trig forms. Preserve that
        // consumer's established argument recognition and representation path.
        self.pi_atan_trig_argument(false)
    }

    /// Try the direct atan linear form, then its Laurent recovery, both at depth 64.
    /// For a recovered form, quarter_turn_parts supplies the signed coordinate mapping.
    /// If requested, a zero-offset pure-pi form with no retained argument first uses the
    /// finite AtanPiAnchor table and returns an unswapped mapping. Failure of that anchor
    /// lookup returns None. A retained zero-coefficient argument does not qualify as pure pi.
    /// The fallback runs when direct form construction fails, not when later projection
    /// of an already recognized form fails. No signal poll or numerical refinement occurs.
    fn pi_atan_trig_argument(
        &self,
        recover_pi_anchor: bool,
    ) -> Option<(bool, Sign, Sign, Computable)> {
        let form = self
            .pi_atan_linear_form(64)
            .or_else(|| self.pi_atan_laurent_form(64))?;
        if recover_pi_anchor
            && form.constant.sign() == Sign::NoSign
            && form.atan.is_zero()
            && form.argument.is_none()
        {
            let (sine_sign, cosine_sign, argument) = AtanPiAnchor::periodic_trig_argument(form.pi)?;
            return Some((false, sine_sign, cosine_sign, argument));
        }
        form.quarter_turn_parts()
    }

    /// Recognize k*pi/4 ± atan(x) for odd integer k, returning three signs and x.
    /// The tuple contains the signs of sin(k*pi/4), cos(k*pi/4), and the atan multiplier;
    /// the first two diagonal components each have magnitude 1/sqrt(2). Consumers use
    /// these signs in exact angle-addition formulas, not as signs of the final coordinates.
    /// Require zero rational offset and atan coefficient ±1; even k belongs to the
    /// separate quarter-turn recognizer. Try direct then Laurent forms at depth 64.
    /// None covers unsupported structure or limits, not disproved identities. Defined
    /// operands, fitting offset arithmetic and nested resource assumptions still apply.
    pub(crate) fn diagonal_turn_plus_or_minus_atan_argument(
        &self,
    ) -> Option<(Sign, Sign, Sign, Computable)> {
        let form = self
            .pi_atan_linear_form(64)
            .or_else(|| self.pi_atan_laurent_form(64))?;
        if form.constant.sign() != Sign::NoSign
            || (!form.atan.is_one() && !form.atan.is_minus_one())
        {
            return None;
        }
        let quadrupled = &form.pi * Rational::new(4);
        let turns = quadrupled.integer_magnitude()?;
        if !turns.bit(0) {
            return None;
        }
        let sine_sign = if turns.bit(2) ^ (form.pi.sign() == Sign::Minus) {
            Sign::Minus
        } else {
            Sign::Plus
        };
        let cosine_sign = if turns.bit(1) ^ turns.bit(2) {
            Sign::Minus
        } else {
            Sign::Plus
        };
        Some((sine_sign, cosine_sign, form.atan.sign(), form.argument?))
    }

    /// Recover one atan basis after cancelling recognized pi-Laurent arithmetic.
    /// First require inverse-trig/pi family metadata. Admit rational power-zero terms,
    /// power-zero atoms readable as atan operands, and atom-free rational pi terms at
    /// power one. Radical coefficients, other powers and unmergeable atan operands decline.
    /// The supplied depth reaches the shared node-limited Laurent recognizer; it is not
    /// a total-work cap. Existing domain assumptions apply to opaque cancellations;
    /// this reader neither refines nor stores a new root sign.
    fn pi_atan_laurent_form(&self, budget: usize) -> Option<PiAtanLinearForm> {
        if !self.internal.contains_inverse_trig_or_pi() {
            return None;
        }
        // Reconstructed angular interpolation can retain rational scales such
        // as (pi-2*pi)/(2*pi). Normalize the complete angle once, only after
        // the cheap trigonometric recognizer fails. Reuse the existing bounded
        // Laurent certificate instead of searching at each scalar addition.
        let polynomial = self.pi_laurent_polynomial(budget)?;
        let mut form = PiAtanLinearForm {
            constant: Rational::zero(),
            pi: Rational::zero(),
            atan: Rational::zero(),
            argument: None,
        };
        for (power, coefficient) in polynomial.terms {
            if !coefficient.radicals.is_empty() {
                return None;
            }
            match power {
                0 => {
                    form.constant = form.constant + coefficient.rational;
                    for (atom, scale) in coefficient.atoms {
                        form = form.add(PiAtanLinearForm {
                            constant: Rational::zero(),
                            pi: Rational::zero(),
                            atan: scale,
                            argument: Some(atom.atan_argument()?),
                        })?;
                    }
                }
                1 if coefficient.atoms.is_empty() => form.pi = form.pi + coefficient.rational,
                _ => return None,
            }
        }
        Some(form)
    }

    /// Recognize a rational offset plus rational multiples of pi and one principal acos.
    /// At positive depth, try rational metadata and pi, then atan, asin, the stored
    /// negative-rational acos complement, and other acos operands before arithmetic.
    /// For atan(t), require rational or pure-quadratic t and a rational exact root
    /// s=sqrt(t²/(1+t²)); use sign(t)*(pi/2-acos(s)), taking positive orientation at zero.
    /// A recognized atan whose conversion fails declines immediately, without trying
    /// later families. For asin, use pi/2-acos(x), with the reflected formula for an
    /// exact negative rational x. The complement tag stores acos(-r)=pi-acos(r).
    ///
    /// Recurse through Add, Negate, Offset and exact-rational products; form addition
    /// may use the nonnegative rational unit-circle complement relation. Principal
    /// real domains must already be admitted. Depth bounds do not meter aggregate or
    /// nested work, rational size or allocations, and no signal poll occurs here.
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
        if let Some(argument) = self.atan_argument() {
            // atan(t) has the sign of t and lies strictly between -pi/2 and
            // pi/2. Its sine has squared magnitude t^2/(1+t^2). Admit this
            // cross-family form only when that positive root is exactly
            // rational, including pure quadratic tangents. No root is fitted.
            let (sign, square) = if let Some(value) = argument.exact_rational() {
                (value.sign(), &value * &value)
            } else {
                let (scale, radicand) = argument.exact_pure_quadratic_surd()?;
                (scale.sign(), &scale * &scale * radicand)
            };
            let denominator = Rational::one() + &square;
            let sine = (square / denominator).perfect_nth_root(2)?;
            let orientation = if sign == Sign::Minus {
                Rational::new(-1)
            } else {
                Rational::one()
            };
            return Some(PiAcosLinearForm {
                constant: Rational::zero(),
                pi: orientation.clone() * Rational::fraction(1, 2).expect("two is nonzero"),
                acos: -orientation,
                argument: Some(Self::rational(sine)),
            });
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
                    return Some(right.pi_acos_linear_form(budget - 1)?.scaled(scale));
                }
                if let Some(scale) = right.exact_rational() {
                    return Some(left.pi_acos_linear_form(budget - 1)?.scaled(scale));
                }
                None
            }
            _ => None,
        }
    }

    /// Project recognized Laurent arithmetic back into one principal-acos linear basis.
    /// Require positive depth and inverse-trig/pi metadata, then admit rational power-zero
    /// terms, inverse-angle atoms convertible at depth minus one, and atom-free rational
    /// power-one terms. Decline radicals, other powers or incompatible arguments.
    /// Atom conversion has its own recursive work within that depth; the outer Laurent
    /// node counter is not a total budget. Domain validity remains a caller prerequisite.
    fn pi_acos_laurent_form(&self, budget: usize) -> Option<PiAcosLinearForm> {
        if budget == 0 || !self.internal.contains_inverse_trig_or_pi() {
            return None;
        }
        // Replay may retain an angle inside cancelling pi factors. Project
        // once through the existing bounded Laurent authority, then interpret
        // only rational scales, pi, and admitted inverse-angle atoms.
        let polynomial = self.pi_laurent_polynomial(budget)?;
        let mut form = PiAcosLinearForm {
            constant: Rational::zero(),
            pi: Rational::zero(),
            acos: Rational::zero(),
            argument: None,
        };
        for (power, coefficient) in polynomial.terms {
            if !coefficient.radicals.is_empty() {
                return None;
            }
            match power {
                0 => {
                    form.constant = form.constant + coefficient.rational;
                    for (atom, scale) in coefficient.atoms {
                        form = form.add(atom.pi_acos_linear_form(budget - 1)?.scaled(scale))?;
                    }
                }
                1 if coefficient.atoms.is_empty() => form.pi = form.pi + coefficient.rational,
                _ => return None,
            }
        }
        Some(form)
    }

    /// Collect rational offset/pi terms and a linear list of retained principal-atan operands.
    /// Follow the direct linear reader's rational/pi/atan precedence and supported
    /// Add, Negate, Offset and rational-product grammar, but preserve distinct arguments
    /// for later monotonicity and sign deductions. Addition merges structural matches.
    /// Depth zero or an unsupported branch declines. Depth does not cap list size,
    /// aggregate traversal or coefficient work, and this reader does not poll cancellation.
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

    /// Recognize orientation*(acos(x)-pi/2), with orientation exactly ±1, at depth 32.
    /// Require a zero rational offset and matching opposite pi/acos coefficients.
    /// Return the coefficient orientation and operand; orientation is not the sign of
    /// the resulting angle, and no sign restriction on x is imposed. The principal
    /// acos domain must already be valid. This entry point uses no Laurent fallback.
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

    /// Return the rational remainder when two admitted inverse-angle linear forms cancel.
    /// Require inverse-trig/pi metadata on at least one operand, then try paired direct
    /// atan forms before paired acos forms, each at depth 32. Success requires both pi
    /// and inverse-angle coefficients to vanish after merging; no Laurent recovery or
    /// numerical refinement is attempted. None is recognition failure, not inequality;
    /// this helper does not independently validate domains or publish a root sign.
    fn collapsed_inverse_trig_linear_sum(&self, other: &Computable) -> Option<Rational> {
        if !self.internal.contains_inverse_trig_or_pi()
            && !other.internal.contains_inverse_trig_or_pi()
        {
            return None;
        }
        if let (Some(left), Some(right)) =
            (self.pi_atan_linear_form(32), other.pi_atan_linear_form(32))
            && let Some(form) = left.add(right)
            && form.pi.sign() == Sign::NoSign
            && form.atan.sign() == Sign::NoSign
        {
            return Some(form.constant);
        }
        if let (Some(left), Some(right)) =
            (self.pi_acos_linear_form(32), other.pi_acos_linear_form(32))
            && let Some(form) = left.add(right)
            && form.pi.sign() == Sign::NoSign
            && form.acos.sign() == Sign::NoSign
        {
            return Some(form.constant);
        }
        None
    }

    /// Prove selected signs using principal inverse-angle ranges and retained argument signs.
    /// For zero-offset forms, try the atan-term list, a single atan basis, then direct
    /// or Laurent acos bases, using depth 32. Atan cases include exact cancellation,
    /// monotonicity for opposite equal coefficients, aligned signs, and pi dominance.
    /// For a nonzero pi coefficient,
    /// 2*abs(pi_coefficient)>=sum(abs(atan_coefficients)) gives a strict sign because
    /// abs(atan(x))<pi/2 for every finite x. Acos cases use its nonnegative principal
    /// range, the endpoint x=1, aligned pi/acos coefficients and signed half-pi shifts.
    ///
    /// Return a proved Sign or None, which means this search did not decide. Some
    /// required missing argument signs end the search early; do not infer that every
    /// later rule ran. Defined real operands are assumed: the endpoint check alone
    /// is not validation of the full acos domain. Nested exact-sign/arithmetic helpers
    /// may reuse or retain facts, but this method does not publish the queried sign
    /// itself, numerically refine, or impose a total-work/cancellation contract.
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
                return Some(product_sign(form.atan.sign(), form.argument?.exact_sign()?));
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

        if let Some(form) = self
            .pi_acos_linear_form(32)
            .or_else(|| self.pi_acos_laurent_form(32))
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

    /// Recover an exact rational r for a recognized value r*pi.
    /// Try retained linear pi structure at depth 32 before depth-48 Laurent projection.
    /// Zero is admitted by the Laurent path even when the direct reader has no pi leaf.
    /// None does not prove a non-pi value; defined source operands, admissible shifts
    /// and the underlying resource restrictions remain required. No refinement occurs.
    pub(crate) fn pi_rational_multiple(&self) -> Option<Rational> {
        self.retained_pi_rational_multiple(32)
            .or_else(|| self.pi_laurent_rational_multiple())
    }

    /// Project a depth-48 Laurent form only if empty or a purely rational coefficient at power one.
    /// Return zero for an empty form and the pi coefficient otherwise. Surviving atoms,
    /// radicals or any other powers decline; no new root sign is stored here. Opaque
    /// cancellation still assumes defined operands and the recognizer's resource contract.
    fn pi_laurent_rational_multiple(&self) -> Option<Rational> {
        let polynomial = self.pi_laurent_polynomial(48)?;
        match polynomial.terms.as_slice() {
            [] => Some(Rational::zero()),
            [(1, coefficient)]
                if coefficient.radicals.is_empty() && coefficient.atoms.is_empty() =>
            {
                Some(coefficient.rational.clone())
            }
            _ => None,
        }
    }

    /// Read direct pi through rational products, negation, binary scaling and addition.
    /// Require positive remaining depth at every visited node; try the left rational
    /// factor before the right. Pure rational leaves, including zero, decline this
    /// route. The returned coefficient is exact for admitted operands and shifts.
    /// No memo or aggregate-work counter is used, so depth alone does not bound the
    /// whole traversal, coefficient growth or allocations; no signal poll occurs.
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
            Approximation::Negate(child) => Some(-child.retained_pi_rational_multiple(budget - 1)?),
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

    /// Recover (r, scale) from a stored scale*sin(pi*r) with 0<r<1/2.
    /// The base must be PrescaledSin with a recognized rational-pi argument; only
    /// negation, binary scaling and exact-rational products are peeled. Neither the
    /// name nor this reader normalizes arbitrary periodic angles or cosine tags.
    /// The strict interval establishes 0<sin(pi*r)<1 for subsequent sign bounds.
    /// Wrapper recursion has no explicit depth/work limit or cancellation poll; nested
    /// pi recognition and power-of-two allocation retain their own restrictions.
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

    /// Use the strict range 0<sin(pi*r)<1 to sign rational+scale*sin(pi*r).
    /// Recognize the canonical sine term, then compare its rational interval endpoints
    /// rational and rational+scale. A nonzero scale can give a strict sign even when
    /// one endpoint is zero, because the sine range is open; zero scale returns the
    /// rational sign. Opposite endpoint signs decline. This is a sufficient bound,
    /// not an exact sine evaluation, and it neither validates arbitrary domains nor
    /// publishes a sign itself; the addition constructor may retain the returned proof.
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
            internal: Arc::new(Node::new(
                Approximation::Negate(self),
                BoundCache::Invalid,
                exact_sign,
            )),
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
            && let Some(inverse_offset) = n.checked_neg()
            && child.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
        {
            // 1/(x*2^n) = (1/x)*2^-n, preserving the cheap binary scale.
            // Keep the inverse node below when the new exponent does not fit.
            return child.clone().inverse().shift_left(inverse_offset);
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
            internal: Arc::new(Node::new(
                Approximation::Inverse(self),
                BoundCache::Invalid,
                exact_sign,
            )),
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
        if let Approximation::Offset(child, inner) = &self.internal.approximation
            && let Some(combined) = inner.checked_add(n)
        {
            // Combine nested binary offsets rather than growing a chain of
            // wrappers, but retain both when their exact sum does not fit.
            return child.clone().shift_left(combined);
        }
        // Exact sign is unchanged by binary scaling when the inner sign is
        // already proven; this makes compare/sign predicates avoid descending
        // into a one-step structural walk on hot paths.
        let exact_sign = match self.internal.facts.exact_sign() {
            ExactSignCache::Valid(sign) => ExactSignCache::Valid(sign),
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(
                Approximation::Offset(self, n),
                BoundCache::Invalid,
                exact_sign,
            )),
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
        if let Approximation::Offset(child, n) = &self.internal.approximation
            && let Some(squared_offset) = n.checked_mul(2)
        {
            // (x * 2^n)^2 is x^2 * 2^(2n); keeping powers of two as offsets is much
            // cheaper than multiplying by an exact rational scale. Retain the
            // square node below when the doubled exponent does not fit.
            return child.clone().square().shift_left(squared_offset);
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
            internal: Arc::new(Node::new(
                Approximation::Square(self),
                BoundCache::Invalid,
                exact_sign,
            )),
            signal: None,
        }
    }

    /// Multiply this number by some other number.
    pub fn multiply(self, other: Computable) -> Computable {
        if let Some(result) = self.shared_reciprocal_product(&other) {
            crate::trace_dispatch!("computable", "multiply", "shared-reciprocal-product");
            return result;
        }
        if let Some(result) = self.shared_square_quotient(&other) {
            crate::trace_dispatch!("computable", "multiply", "shared-square-quotient");
            return result;
        }
        if self.jacobi_square_product(&other) {
            return self.square();
        }
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
                if let Some(remainder) = other.without_scaled_shared_factor(SharedConstant::InvPi) {
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
                if let Some(remainder) = self.without_scaled_shared_factor(SharedConstant::InvPi) {
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
            internal: Arc::new(Node::new(
                Approximation::Multiply(self, other),
                BoundCache::Invalid,
                exact_sign,
            )),
            signal: None,
        }
    }

    pub(crate) fn linear_combination3(
        coefficients: [Computable; 3],
        values: [Rational; 3],
    ) -> Computable {
        Self {
            internal: Arc::new(Node::new(
                Approximation::LinearCombination3(Box::new(LinearCombination3 {
                    coefficients,
                    values,
                })),
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
            internal: Arc::new(Node::new(
                Approximation::Multiply(Self::rational(scale), self),
                BoundCache::Invalid,
                exact_sign,
            )),
            signal: None,
        }
    }

    // Expose an exact affine offset only when cancellation needs it. Keep
    // ordinary sums and their shared approximation caches intact. Bound the
    // total traversal and scale materialization; never approximate here.
    fn rational_offset_parts(&self, budget: &mut usize) -> Option<(&Self, Rational, Rational)> {
        *budget = budget.checked_sub(1)?;
        let (child, scale) = match &self.internal.approximation {
            Approximation::Add(left, right) => {
                if let Some(offset) = left.exact_rational() {
                    return Some((right, Rational::one(), offset));
                }
                if let Some(offset) = right.exact_rational() {
                    return Some((left, Rational::one(), offset));
                }
                let (left, left_scale, left_offset) = left.rational_offset_parts(budget)?;
                let (right, right_scale, right_offset) = right.rational_offset_parts(budget)?;
                return Self::internal_structural_eq(left, right)
                    .then(|| (left, left_scale + right_scale, left_offset + right_offset));
            }
            Approximation::Negate(child) => (child, Rational::new(-1)),
            Approximation::Offset(child, shift) if shift.unsigned_abs() <= 4096 => {
                (child, Self::power_of_two_rational(*shift))
            }
            Approximation::Multiply(left, right) => {
                if let Some(scale) = left.exact_rational() {
                    (right, scale)
                } else {
                    (left, right.exact_rational()?)
                }
            }
            _ => return None,
        };
        if scale.numerator().bits().max(scale.denominator().bits()) > 4097 {
            return None;
        }
        let (base, coefficient, offset) = child.rational_offset_parts(budget)?;
        Some((base, coefficient * &scale, offset * scale))
    }

    /// Add some other number to this number.
    #[allow(clippy::should_implement_trait)]
    pub fn add(self, other: Computable) -> Computable {
        if let Some(value) = self.jacobi_square_sum(&other) {
            return value;
        }
        fn affine_offset_eq(left: &Computable, right: &Computable) -> bool {
            let left_parts = left.rational_offset_parts(&mut 8);
            let right_parts = right.rational_offset_parts(&mut 8);
            let (left, left_scale, right, right_scale) = match (left_parts, right_parts) {
                (
                    Some((left, left_scale, left_offset)),
                    Some((right, right_scale, right_offset)),
                ) if left_offset == right_offset => (left, left_scale, right, right_scale),
                (Some((base, scale, offset)), None) if offset.sign() == Sign::NoSign => {
                    (base, scale, right, Rational::one())
                }
                (None, Some((base, scale, offset))) if offset.sign() == Sign::NoSign => {
                    (left, Rational::one(), base, scale)
                }
                _ => return false,
            };
            if left_scale == right_scale {
                Computable::internal_structural_eq(left, right)
            } else {
                Computable::internal_structural_eq(
                    &left.clone().multiply_rational(left_scale),
                    &right.clone().multiply_rational(right_scale),
                )
            }
        }

        fn commuted_add_pair_eq(left: &Computable, right: &Computable) -> bool {
            matches!(
                (&left.internal.approximation, &right.internal.approximation),
                (
                    Approximation::Add(left_first, left_second),
                    Approximation::Add(right_first, right_second),
                ) if Computable::internal_structural_eq(left_first, right_second)
                    && Computable::internal_structural_eq(left_second, right_first)
            )
        }

        if let Approximation::Negate(cancelled) = &other.internal.approximation {
            if Self::internal_structural_eq(&self, cancelled) {
                crate::trace_dispatch!("computable", "add", "direct-cancellation");
                return Self::zero();
            }
            if commuted_add_pair_eq(&self, cancelled) {
                crate::trace_dispatch!("computable", "add", "commuted-pair-cancellation");
                return Self::zero();
            }
        }
        if let Approximation::Negate(cancelled) = &self.internal.approximation {
            if Self::internal_structural_eq(cancelled, &other) {
                crate::trace_dispatch!("computable", "add", "direct-cancellation");
                return Self::zero();
            }
            if commuted_add_pair_eq(cancelled, &other) {
                crate::trace_dispatch!("computable", "add", "commuted-pair-cancellation");
                return Self::zero();
            }
        }
        if let Some(rational) = self.collapsed_inverse_trig_linear_sum(&other) {
            crate::trace_dispatch!("computable", "add", "inverse-trig-linear-collapse");
            return Self::rational(rational);
        }
        let left_exact = self.exact_rational();
        let right_exact = other.exact_rational();
        if let Some(rational) = &right_exact
            && let Some((base, scale, offset)) = self.rational_offset_parts(&mut 8)
        {
            crate::trace_dispatch!("computable", "add", "rational-offset-fold");
            return base
                .clone()
                .multiply_rational(scale)
                .add(Self::rational(offset + rational));
        }
        if let Some(rational) = &left_exact
            && let Some((base, scale, offset)) = other.rational_offset_parts(&mut 8)
        {
            crate::trace_dispatch!("computable", "add", "rational-offset-fold");
            return base
                .clone()
                .multiply_rational(scale)
                .add(Self::rational(offset + rational));
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

        let cancelled_pair = match (&self.internal.approximation, &other.internal.approximation) {
            (_, Approximation::Negate(cancelled)) => Some((&self, cancelled)),
            (Approximation::Negate(cancelled), _) => Some((&other, cancelled)),
            _ => None,
        };
        if cancelled_pair.is_some_and(|(left, right)| affine_offset_eq(left, right)) {
            crate::trace_dispatch!("computable", "add", "affine-offset-cancellation");
            return Self::zero();
        }

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
            internal: Arc::new(Node::new(
                Approximation::Add(self, other),
                if certified_bound == BoundInfo::Unknown {
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
            internal: Arc::new(Node::new(
                Approximation::Int(n),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
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
            internal: Arc::new(Node::new(
                Approximation::ErfSeries(self.clone()),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
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
            internal: Arc::new(Node::new(
                Approximation::Erfc(self),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
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
            internal: Arc::new(Node::new(
                Approximation::NormalSf(self),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal: None,
        }
    }

    /// Standard normal probability mass over [lo, hi].
    pub fn normal_interval(lo: Computable, hi: Computable) -> Computable {
        if Computable::internal_structural_eq(&lo, &hi) {
            return Self::zero();
        }
        let exact_order = lo
            .bounded_laurent_rational(48)
            .zip(hi.bounded_laurent_rational(48))
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
            internal: Arc::new(Node::new(
                Approximation::NormalInterval { lo, hi },
                BoundCache::Invalid,
                exact_sign,
            )),
            signal: None,
        }
    }

    /// Natural logarithm of the standard normal CDF.
    pub fn log_pnorm(self) -> Computable {
        Self {
            internal: Arc::new(Node::new(
                Approximation::LogPnorm(self),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Minus),
            )),
            signal: None,
        }
    }

    /// Natural logarithm of the standard normal upper-tail probability.
    pub fn log_normal_sf(self) -> Computable {
        Self {
            internal: Arc::new(Node::new(
                Approximation::LogNormalSf(self),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Minus),
            )),
            signal: None,
        }
    }

    /// Natural logarithm of the standard normal density.
    pub fn log_dnorm(self) -> Computable {
        Self {
            internal: Arc::new(Node::new(
                Approximation::LogDnorm(self),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Minus),
            )),
            signal: None,
        }
    }

    /// Standard normal quantile for a defined probability strictly between zero and one.
    ///
    /// The seed is an evaluation hint. A retained CDF identity or numerical
    /// enclosures certify the result; safeguarded Newton iteration and monotone
    /// search repair inaccurate seeds.
    /// This native constructor assumes the probability domain; it does not return
    /// a typed validation error for endpoints or invalid imported graphs. A
    /// recognized rational CDF inverse returns immediately; otherwise retain p,
    /// the raw seed integer and its binary exponent in a boxed payload. The outer
    /// signal starts unset, including
    /// on exact shortcuts; retaining a child is not automatic abort inheritance.
    /// Numerical evaluation has no new total-work or heap-byte limit, and extreme
    /// output precision can encounter unchecked integer overflow. Public Real
    /// entry points apply their own admission and practical-window rules.
    pub fn normal_quantile(p: Computable, seed: BigInt, seed_prec: Precision) -> Computable {
        if let Some(exact) = p.exact_normal_quantile() {
            return Self::rational(exact);
        }
        Self {
            internal: Arc::new(Node::new(
                Approximation::NormalQuantile(Box::new(NormalQuantileData { p, seed, seed_prec })),
                BoundCache::Invalid,
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
