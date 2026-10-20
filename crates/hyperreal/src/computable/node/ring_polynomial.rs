// Rational projection of a commutative polynomial over opaque real atoms.
// This optional proof follows the established Laurent recognizer; it never
// replaces an existing successful proof, approximates an atom, or rebuilds the
// authoritative expression. Atom equality uses the shared structural matcher.

#[derive(Clone, Default)]
struct RingPolynomial {
    // Sorted monomials, with sorted atom IDs and nonzero rational coefficients.
    terms: Vec<(Vec<u8>, Rational)>,
}

struct RingRecognition<'a> {
    atoms: Vec<&'a Computable>,
    // Used only by quotient recognition. The borrowed root keeps these node
    // identities alive for the attempt; no entry survives the query.
    quotients: std::collections::HashMap<*const Node, RingQuotient>,
    nodes_left: usize,
    operations_left: usize,
    signal: &'a Option<Signal>,
    // An additional attempt may use only these built-in exact relations.
    // Earlier polynomial/quotient attempts keep their original arithmetic.
    root_squares: bool,
    root_square_used: bool,
    // Imported endpoint proofs cannot assume opaque atoms denote valid reals.
    rational_only: bool,
    // Present only in the later function-congruence attempt. Argument
    // projections share this attempt's atoms, budgets and lifetime.
    trig_arguments: Option<Vec<(u8, RingQuotient)>>,
    trig_congruence_used: bool,
    // Present only in the later rational-root/reciprocal attempt. Each
    // square is certified from an admitted nonnegative rational radicand.
    rational_squares: Option<Vec<(u8, Rational)>>,
    unit_circle_pairs: Vec<(u8, u8)>,
    rational_trig_values: Vec<(&'a Computable, Rational)>,
    additional_relation_used: bool,
}

impl<'a> RingRecognition<'a> {
    const MAX_NODES: usize = 512;
    const MAX_DEPTH: usize = 64;
    const MAX_ATOMS: usize = 32;
    const MAX_TERMS: usize = 64;
    const MAX_DEGREE: usize = 8;
    const MAX_COEFFICIENT_BITS: u64 = 4096;
    const MAX_OPERATIONS: usize = 8192;

    fn new(value: &'a Computable) -> Self {
        Self {
            atoms: Vec::new(),
            quotients: std::collections::HashMap::new(),
            nodes_left: Self::MAX_NODES,
            operations_left: Self::MAX_OPERATIONS,
            signal: &value.signal,
            root_squares: false,
            root_square_used: false,
            rational_only: false,
            trig_arguments: None,
            trig_congruence_used: false,
            rational_squares: None,
            unit_circle_pairs: Vec::new(),
            rational_trig_values: Vec::new(),
            additional_relation_used: false,
        }
    }

    // Charges coefficient operations, term insertion and atom comparisons.
    // This bounds their count, not the work inside the existing DAG matcher.
    fn charge(&mut self) -> Option<()> {
        if should_stop(self.signal) {
            return None;
        }
        self.operations_left = self.operations_left.checked_sub(1)?;
        Some(())
    }

    fn coefficient_fits(value: &Rational) -> bool {
        // Check stored size before canonicalizing an unreduced rational. The
        // shared storage classifier rejects inputs above 4096 stored bits.
        value.is_zero()
            || (!matches!(
                value.storage_class(),
                crate::RationalStorageClass::VeryLarge
            ) && value.numerator().bits() + value.denominator().bits()
                <= Self::MAX_COEFFICIENT_BITS)
    }

    fn constant(value: Rational) -> Option<RingPolynomial> {
        if !Self::coefficient_fits(&value) {
            return None;
        }
        Some(RingPolynomial {
            terms: if value.is_zero() {
                Vec::new()
            } else {
                vec![(Vec::new(), value)]
            },
        })
    }

    fn insert(
        &mut self,
        result: &mut RingPolynomial,
        monomial: Vec<u8>,
        coefficient: Rational,
    ) -> Option<()> {
        self.charge()?;
        if !Self::coefficient_fits(&coefficient) || monomial.len() > Self::MAX_DEGREE {
            return None;
        }
        if coefficient.is_zero() {
            return Some(());
        }
        match result
            .terms
            .binary_search_by(|(term, _)| term.cmp(&monomial))
        {
            Ok(index) => {
                self.charge()?;
                let sum = &result.terms[index].1 + coefficient;
                if !Self::coefficient_fits(&sum) {
                    return None;
                }
                if sum.is_zero() {
                    result.terms.remove(index);
                } else {
                    result.terms[index].1 = sum;
                }
            }
            Err(index) => {
                if result.terms.len() == Self::MAX_TERMS {
                    return None;
                }
                result.terms.insert(index, (monomial, coefficient));
            }
        }
        Some(())
    }

    fn add(&mut self, mut left: RingPolynomial, right: RingPolynomial) -> Option<RingPolynomial> {
        for (monomial, coefficient) in right.terms {
            self.insert(&mut left, monomial, coefficient)?;
        }
        Some(left)
    }

    fn multiply(
        &mut self,
        left: &RingPolynomial,
        right: &RingPolynomial,
    ) -> Option<RingPolynomial> {
        let mut result = RingPolynomial::default();
        for (a, x) in &left.terms {
            for (b, y) in &right.terms {
                self.charge()?;
                if a.len() + b.len() > Self::MAX_DEGREE {
                    return None;
                }
                let mut monomial = Vec::with_capacity(a.len() + b.len());
                monomial.extend_from_slice(a);
                monomial.extend_from_slice(b);
                monomial.sort_unstable();
                // Both operands were admitted; temporary rational arithmetic
                // can use twice the coefficient bits before the next check;
                // a subsequent coefficient sum may also require a carry bit.
                let mut coefficient = x * y;
                if self.root_squares {
                    self.reduce_root_squares(&mut monomial, &mut coefficient)?;
                }
                if self.rational_squares.is_some() {
                    self.reduce_additional_relations(&mut monomial, &mut coefficient)?;
                }
                self.insert(&mut result, monomial, coefficient)?;
            }
        }
        Some(result)
    }

    fn reduce_root_squares(
        &mut self,
        monomial: &mut Vec<u8>,
        coefficient: &mut Rational,
    ) -> Option<()> {
        let mut read = 0;
        let mut write = 0;
        while read < monomial.len() {
            self.charge()?;
            let atom = monomial[read];
            let square = match &self.atoms[usize::from(atom)].internal.approximation {
                Approximation::Constant(SharedConstant::Sqrt2) => Some(2),
                Approximation::Constant(SharedConstant::Sqrt3) => Some(3),
                _ => None,
            };
            if let Some(square) = square
                && monomial.get(read + 1) == Some(&atom)
            {
                self.charge()?;
                *coefficient = &*coefficient * Rational::from(square);
                if !Self::coefficient_fits(coefficient) {
                    return None;
                }
                self.root_square_used = true;
                read += 2;
            } else {
                monomial[write] = atom;
                write += 1;
                read += 1;
            }
        }
        monomial.truncate(write);
        Some(())
    }

    fn scale(&mut self, polynomial: RingPolynomial, value: Rational) -> Option<RingPolynomial> {
        self.multiply(&polynomial, &Self::constant(value)?)
    }

    fn atom(&mut self, value: &'a Computable) -> Option<RingPolynomial> {
        if self.rational_only {
            return None;
        }
        for index in 0..self.atoms.len() {
            self.charge()?;
            if Computable::internal_structural_eq(self.atoms[index], value) {
                return Some(RingPolynomial {
                    terms: vec![(vec![index as u8], Rational::one())],
                });
            }
        }
        if self.atoms.len() == Self::MAX_ATOMS {
            return None;
        }
        let index = self.atoms.len() as u8;
        self.atoms.push(value);
        Some(RingPolynomial {
            terms: vec![(vec![index], Rational::one())],
        })
    }

    fn expand(&mut self, value: &'a Computable, depth: usize) -> Option<RingPolynomial> {
        self.charge()?;
        if depth == 0 {
            return None;
        }
        self.nodes_left = self.nodes_left.checked_sub(1)?;
        match &value.internal.approximation {
            Approximation::One => Self::constant(Rational::one()),
            Approximation::Int(integer) => {
                if integer.bits() >= Self::MAX_COEFFICIENT_BITS {
                    return None;
                }
                Self::constant(Rational::from_bigint(integer.clone()))
            }
            Approximation::Ratio(rational) => Self::constant(rational.clone()),
            Approximation::Add(left, right) => {
                let left = self.expand(left, depth - 1)?;
                let right = self.expand(right, depth - 1)?;
                self.add(left, right)
            }
            Approximation::Multiply(left, right) => {
                let left = self.expand(left, depth - 1)?;
                let right = self.expand(right, depth - 1)?;
                self.multiply(&left, &right)
            }
            Approximation::Square(child) => {
                let child = self.expand(child, depth - 1)?;
                self.multiply(&child, &child)
            }
            Approximation::Negate(child) => {
                let child = self.expand(child, depth - 1)?;
                self.scale(child, Rational::from(-1))
            }
            Approximation::Offset(child, shift) => {
                if u64::from(shift.unsigned_abs()) >= Self::MAX_COEFFICIENT_BITS {
                    return None;
                }
                let child = self.expand(child, depth - 1)?;
                self.scale(child, Computable::power_of_two_rational(*shift))
            }
            Approximation::LinearCombination3(combination) => {
                let mut sum = RingPolynomial::default();
                for (child, scale) in combination.coefficients.iter().zip(&combination.values) {
                    let child = self.expand(child, depth - 1)?;
                    let term = self.scale(child, scale.clone())?;
                    sum = self.add(sum, term)?;
                }
                Some(sum)
            }
            Approximation::PrescaledSin(argument) | Approximation::PrescaledCos(argument)
                if self.trig_arguments.is_some() =>
            {
                self.trig_atom(value, argument, depth - 1)
            }
            Approximation::Sqrt(radicand) if self.rational_squares.is_some() => {
                self.rational_sqrt_atom(value, radicand, depth - 1)
            }
            // Roots, other transcendental functions and opaque inverses retain
            // their identity. No algebraic independence is assumed: a zero
            // polynomial is zero for any valid real values of its atoms.
            _ => self.atom(value),
        }
    }
}

impl Computable {
    pub(crate) fn bounded_ring_rational(&self) -> Option<Rational> {
        let mut recognition = RingRecognition::new(self);
        let polynomial = recognition.expand(self, RingRecognition::MAX_DEPTH)?;
        let rational = match polynomial.terms.as_slice() {
            [] => Rational::zero(),
            [(monomial, coefficient)] if monomial.is_empty() => coefficient.clone(),
            _ => return None,
        };
        if should_stop(&self.signal) {
            return None;
        }
        // Publish only a completed exact projection. A budget decline, opaque
        // remainder or cancellation supplies no cacheable negative evidence.
        self.internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(rational.sign()));
        crate::trace_dispatch!("computable", "normal_form", "rational-polynomial");
        Some(rational)
    }
}
