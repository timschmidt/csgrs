// Rational projection of a commutative polynomial over opaque real atoms.
// This optional proof follows the established Laurent recognizer; it never
// replaces an existing successful proof, approximates an atom, or rebuilds the
// authoritative expression. Atom equality uses the shared structural matcher.

#[derive(Clone, Default)]
/// Sparse commutative polynomial with exact rational coefficients over query-relative opaque atoms.
/// The empty term list is exact zero; the empty monomial denotes one. No algebraic
/// independence is assumed: formal cancellation is valid for any defined atom
/// values, but a nonzero formal polynomial may still vanish at those values.
/// Clone copies its representation without revalidating IDs; Default produces zero.
struct RingPolynomial {
    // Sorted monomials, with sorted atom IDs and nonzero rational coefficients.
    /// Lexicographically sorted distinct monomials, each a sorted multiset of atom IDs with a nonzero rational coefficient.
    terms: Vec<(Vec<u8>, Rational)>,
}

/// Disposable proof-attempt state shared by polynomial, quotient and later relation strategies.
/// Borrowed inputs keep authoritative pointer keys alive; owned derived atoms
/// stay in this state. Counters and proof tables are local to the attempt.
/// Adapters must configure compatible flags/tables and preserve their ID
/// invariants; arbitrary flag combinations are not independently validated.
/// Opaque leaves are assumed to denote defined reals unless the selected
/// admission strategy explicitly checks their domains. Limits meter selected
/// work/storage dimensions, not all nested computation, memory or cancellation.
struct RingRecognition<'a> {
    /// Borrowed authoritative or owned derived recipes indexed by u8 IDs; structural matches share an ID within this attempt.
    atoms: Vec<std::borrow::Cow<'a, Computable>>,
    // Used only by quotient recognition. The borrowed root keeps these node
    // identities alive for the attempt; no entry survives the query.
    /// Completed quotient projections keyed by live authoritative node addresses; temporary derived pointers must not enter this query-local map.
    quotients: std::collections::HashMap<*const Node, RingQuotient>,
    /// Remaining charged node expansions shared by the configured strategy; this is not a byte or total-DAG-size bound.
    nodes_left: usize,
    /// Remaining explicit work charges; nested rational/matcher internals are not individually metered.
    operations_left: usize,
    /// Borrowed stop handle of the original query wrapper, polled at charge/publication boundaries.
    signal: &'a Option<Signal>,
    // An additional attempt may use only these built-in exact relations.
    // Earlier polynomial/quotient attempts keep their original arithmetic.
    /// Enable the special sqrt(2)^2=2 and sqrt(3)^2=3 reductions in ordinary multiplication.
    root_squares: bool,
    /// Record that a built-in square pair was reduced during this attempt, for the root-square adapter's success gate.
    root_square_used: bool,
    // Imported endpoint proofs cannot assume opaque atoms denote valid reals.
    /// Forbid opaque atom admission in the defined-rational quotient strategy; arithmetic alone must discharge its domains.
    rational_only: bool,
    // Present only in the later function-congruence attempt. Argument
    // projections share this attempt's atoms, budgets and lifetime.
    /// Optional trig-congruence table pairing atom IDs with already admitted exact argument quotients; None disables that strategy.
    trig_arguments: Option<Vec<(u8, RingQuotient)>>,
    /// Record a successful equality match between same-function trig arguments for the congruence adapter's gate.
    trig_congruence_used: bool,
    // Present only in the later rational-root/reciprocal attempt. Each
    // square is certified from an admitted nonnegative rational radicand.
    /// Optional admitted (root atom ID, nonnegative rational square) table; presence also enables the additional relation strategy.
    rational_squares: Option<Vec<(u8, Rational)>>,
    /// Atom pairs proved to have squared values summing to one, including complementary same-function angles.
    unit_circle_pairs: Vec<(u8, u8)>,
    /// Successful rational trig projections associated with borrowed source recipes for structural reuse in this query.
    rational_trig_values: Vec<(&'a Computable, Rational)>,
    /// Record that an additional root/reciprocal/trig relation was consumed; the ordinary relation adapter requires this, the extended one does not.
    additional_relation_used: bool,
    // A separate optional attempt admits triangular native square relations.
    // Ordinary ring and quotient recognition retain their original limits.
    /// Enable triangular native relation admission, native multiplication/division and degree twelve.
    native_relations: bool,
    // Root-branch completion follows the original native strategy and uses
    // only its remaining node and operation budget.
    /// Enable the later native root-branch/sign strategy while retaining the same remaining query allowances.
    root_completion: bool,
    // The final extended relation attempt raises the degree limit; every
    // earlier attempt keeps its original limits.
    /// Select the extended relation degree cap when native mode is disabled; its adapter sets the larger work allowance separately.
    extended_degree: bool,
    /// Optional query-local signs parallel to native atoms; missing entries remain retryable rather than negative evidence.
    native_signs: Vec<Option<RealSign>>,
    /// Optional admitted triangular relation parallel to each native atom, referring only to earlier atom IDs.
    relations: Vec<Option<NativeRingRelation>>,
    /// Hint that native relation construction encountered a usable square-root relation; gates early root reductions.
    root_relation_seen: bool,
    /// One retained sn/cn basis choice per structural log-complement recipe, stable across differently warmed copies in this query.
    jacobi_bases: Vec<(Computable, bool)>,
    /// Already guarded parameter quotient recipes kept opaque to avoid expanding unrelated native constants.
    parameter_atoms: Vec<Computable>,
}

/// Shared exact polynomial arithmetic and admission state for the configured optional recognition strategy.
impl<'a> RingRecognition<'a> {
    /// Initial node-expansion allowance for ordinary recognition attempts (512).
    const MAX_NODES: usize = 512;
    /// Initial recursive expansion depth (64), independent of total charged node visits.
    const MAX_DEPTH: usize = 64;
    /// Maximum query-relative opaque atom count (32); shared tables use the same ID envelope.
    const MAX_ATOMS: usize = 32;
    /// Maximum retained nonzero monomial count in an admitted polynomial (64).
    const MAX_TERMS: usize = 64;
    /// Ordinary retained monomial degree limit (eight), counting repeated atom factors.
    const MAX_DEGREE: usize = 8;
    /// Stored rational coefficient cap (4096 combined numerator/denominator bits for nonzero admitted values).
    const MAX_COEFFICIENT_BITS: u64 = 4096;
    /// Initial count of explicit work charges for an ordinary attempt (8192), not a total computation bound.
    const MAX_OPERATIONS: usize = 8192;
    // Limits of the final extended relation attempt, which runs only after
    // every other exact sign stage declined.
    /// Retained monomial degree allowance for the extended relation strategy (twelve).
    const EXTENDED_DEGREE: usize = 12;
    /// Explicit work allowance selected by the extended relation adapter (32768); setting the degree flag alone does not apply it.
    const EXTENDED_OPERATIONS: usize = 32_768;

    /// Create a plain-polynomial recognition attempt borrowing the input wrapper's stop handle.
    /// Initialize standard node/work allowances and empty query-local tables, with
    /// all optional strategies disabled. This does not inspect the graph, poll the
    /// stop handle or validate its native domains. Strategy adapters configure the
    /// relevant flags/tables before invoking shared arithmetic.
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
            native_relations: false,
            root_completion: false,
            extended_degree: false,
            native_signs: Vec::new(),
            relations: Vec::new(),
            root_relation_seen: false,
            jacobi_bases: Vec::new(),
            parameter_atoms: Vec::new(),
        }
    }

    /// Choose the retained monomial-degree cap for the active strategy.
    /// Native relations take precedence at twelve, extended relations use their
    /// twelve-degree constant, and ordinary attempts use eight. This is a stored
    /// polynomial degree, not algebraic field degree or expression-tree depth.
    fn degree_limit(&self) -> usize {
        if self.native_relations {
            12
        } else if self.extended_degree {
            Self::EXTENDED_DEGREE
        } else {
            Self::MAX_DEGREE
        }
    }

    // Charges coefficient operations, term insertion and atom comparisons.
    // This bounds their count, not the work inside the existing DAG matcher.
    /// Poll the attempt's borrowed root signal, then consume one remaining operation charge.
    /// A stop or zero allowance returns None without decrement; no reason is retained.
    /// A charge counts an explicit visit/arithmetic/insertion/comparison step, not
    /// all work within a rational operation, allocation or structural DAG comparison.
    fn charge(&mut self) -> Option<()> {
        if should_stop(self.signal) {
            return None;
        }
        self.operations_left = self.operations_left.checked_sub(1)?;
        Some(())
    }

    /// Accept exact zero or a rational fitting the stored-size and component-bit rules.
    /// For nonzero values, reject VeryLarge storage before reading canonical numerator
    /// and denominator magnitudes, then require their bit-length sum at most 4096.
    /// This static guard does not charge work, cap transient arithmetic or prove that
    /// a whole computation fits a memory budget.
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

    /// Construct canonical polynomial zero or a single constant term after coefficient admission.
    /// An empty monomial means one; an empty term list means zero. No work charge or
    /// stop check is performed here, and None denotes only failed coefficient admission.
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

    /// Insert or merge one sorted monomial while preserving lexicographic term order.
    /// Charge first and check coefficient/degree before ignoring exact zero. On a
    /// match, charge the coefficient sum, reject excessive size, and remove a zero
    /// sum; a new term requires room below the term cap. The caller supplies sorted
    /// valid atom IDs and an already normalized result. A failure spends work but
    /// leaves this insertion's result unchanged; it never validates atom domains.
    fn insert(
        &mut self,
        result: &mut RingPolynomial,
        monomial: Vec<u8>,
        coefficient: Rational,
    ) -> Option<()> {
        self.charge()?;
        if !Self::coefficient_fits(&coefficient) || monomial.len() > self.degree_limit() {
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

    /// Accumulate right-hand terms into an admitted left polynomial using charged insertion.
    /// Process terms in stored order. Empty right input returns left without an
    /// extra charge or stop poll; any failed insertion discards the owned candidate
    /// and returns None with the attempt's work already spent.
    fn add(&mut self, mut left: RingPolynomial, right: RingPolynomial) -> Option<RingPolynomial> {
        for (monomial, coefficient) in right.terms {
            self.insert(&mut left, monomial, coefficient)?;
        }
        Some(left)
    }

    /// Expand admitted polynomial products under the active relation strategy and limits.
    /// Native mode first charges and reuses either exact-one operand by cloning the
    /// other. Otherwise process term pairs in left/right stored order, merge sorted
    /// IDs, multiply coefficients and insert/merge the result. Native root completion
    /// with a seen root relation can collapse rational root pairs before the degree
    /// check; other paths check the concatenated degree before optional built-in or
    /// additional relation reduction. Empty operands can return zero without pair
    /// charges; the native entry charge still applies.
    ///
    /// Callers must supply admitted normalized operands and coherent strategy tables.
    /// Identity reuse does not revalidate them. Coefficient arithmetic can temporarily
    /// exceed stored caps before insertion/reduction checks. None means a local limit
    /// or stop decline; no factor is approximated and no global proof is published.
    fn multiply(
        &mut self,
        left: &RingPolynomial,
        right: &RingPolynomial,
    ) -> Option<RingPolynomial> {
        if self.native_relations {
            self.charge()?;
            // Both polynomial operands already passed admission. Reusing an
            // identity operand avoids reconstructing and reinserting its
            // unchanged coefficients during native relation substitution.
            let is_one = |value: &RingPolynomial| {
                matches!(value.terms.as_slice(),
                [(monomial, coefficient)] if monomial.is_empty() && coefficient.is_one())
            };
            if is_one(left) {
                return Some(right.clone());
            }
            if is_one(right) {
                return Some(left.clone());
            }
        }
        let mut result = RingPolynomial::default();
        for (a, x) in &left.terms {
            for (b, y) in &right.terms {
                self.charge()?;
                if self.native_relations && self.root_completion && self.root_relation_seen {
                    let (monomial, coefficient) = self.multiply_native_root_terms(a, b, x * y)?;
                    self.insert(&mut result, monomial, coefficient)?;
                    continue;
                }
                if a.len() + b.len() > self.degree_limit() {
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

    /// Remove adjacent pairs of shared sqrt(2)/sqrt(3) atom IDs from a sorted monomial.
    /// Multiply their rational squares into the coefficient, enforcing coefficient
    /// admission and marking root_square_used after a completed pair. Other roots
    /// remain unchanged. Explicit scans/products are charged. On failure, coefficient,
    /// monomial prefix or usage flag may already be changed; discard that candidate.
    /// The caller supplies valid IDs and a sorted monomial. An incoming product
    /// coefficient may already exceed the stored cap; it is checked after a
    /// reduced pair and by the caller's subsequent insertion.
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

    /// Multiply an admitted polynomial by a size-checked rational constant.
    /// Delegate to the active multiplication strategy, including its identity, zero,
    /// relation and charging behavior. A failed constant guard declines before multiplication.
    fn scale(&mut self, polynomial: RingPolynomial, value: Rational) -> Option<RingPolynomial> {
        self.multiply(&polynomial, &Self::constant(value)?)
    }

    /// Represent a defined native value as a query-relative opaque indeterminate.
    /// Rational-only mode declines; native mode delegates relation admission with
    /// the supplied depth. Otherwise compare existing recipes structurally in ID
    /// order, charging each comparison, then append if fewer than 32 atoms exist.
    /// The ordinary insertion has no additional charge, depth check or stop poll;
    /// its expansion caller accounts for entry. Structural matching is not a complete
    /// value-equality test, and opaque admission does not check descendant domains.
    fn atom(&mut self, value: &'a Computable, depth: usize) -> Option<RingPolynomial> {
        if self.rational_only {
            return None;
        }
        if self.native_relations {
            return self.native_relation_atom(std::borrow::Cow::Borrowed(value), depth);
        }
        for index in 0..self.atoms.len() {
            self.charge()?;
            if Computable::internal_structural_eq(self.atoms[index].as_ref(), value) {
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
        Some(RingPolynomial {
            terms: vec![(vec![index], Rational::one())],
        })
    }

    /// Expand selected arithmetic into a rational polynomial over admitted opaque real atoms.
    /// Charge entry, reject depth zero, then consume a node visit. Parse arithmetic
    /// children left before right and all three linear-form coefficients even when
    /// a later coefficient or factor can cancel. Reject excessive binary shifts
    /// before materializing powers of two; enforce coefficient limits on leaves.
    /// Enabled strategies may separately admit trig arguments or rational roots.
    /// Other shapes, including inverses in plain mode, are opaque atoms: definedness
    /// is a caller precondition, not something this walk establishes.
    ///
    /// The plain walk does not memoize DAG expansions, so shared children can spend
    /// visits again even though equal opaque atoms reuse IDs. None combines unsupported
    /// admission, depth/node/work/size caps and cancellation; no negative fact is cached.
    /// All returned polynomials are exact relative to their admitted atom values.
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
            _ => self.atom(value, depth - 1),
        }
    }
}

/// Publish successful plain-polynomial rational projections without numerical approximation.
impl Computable {
    /// Try a fresh bounded polynomial expansion and return only a fully rational projection.
    /// Accept an empty polynomial as zero or a single constant term as its exact
    /// rational. Other residual terms or exhausted limits return None, not a proof
    /// of irrationality. The plain strategy assumes defined native atoms, including
    /// opaque inverses, so it is not an import-domain validator.
    ///
    /// After a final root-signal check, publish the exact sign and the rational-polynomial
    /// dispatch trace. This adapter neither approximates nor retains the rational
    /// value in the tower cache; later calls rebuild the optional projection. Node,
    /// depth, atom, term, degree, coefficient and operation caps have distinct scopes
    /// and do not bound all nested matching work or total memory.
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
