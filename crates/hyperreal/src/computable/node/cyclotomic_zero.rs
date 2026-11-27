// Exact zero certificates over cyclotomic constants.
//
// Rationals, sin(pi*r) and cos(pi*r) for rational r, sqrt(2), sqrt(3) and
// square roots of rationals whose square-free part divides 6 all lie in one
// cyclotomic field Q(zeta_N), with N read from the leaves. Sums, products and
// reciprocals of them stay in that field, so these values satisfy every
// algebraic relation that holds there. The relations include the multiple-angle
// and complementary-angle identities that sampled circles and spheres create,
// which the opaque-atom recognizers cannot see.
//
// The proof does not approximate. Every node becomes a quotient A / prod F_k^e
// of field elements, where each factor F_k is the numerator of a reciprocal's
// operand, proven nonzero. Sums use the least common multiple of the factor
// multisets, so shared reciprocals such as a segment parameter do not inflate
// the bound. For each prime p = 1 (mod N), reduction modulo the primes above p
// sends zeta_N to the primitive N-th roots of unity modulo p. If A reduces to
// zero at all phi(N) of them, then A lies in p Z[zeta_N] after its integrality
// denominator d is cleared. Every embedding of A is at most 2^m in absolute
// value, a bound carried through the graph. A nonzero algebraic integer
// divisible by the product P of the chosen primes has norm at least P^phi(N),
// so some embedding is at least P. Once P > d * 2^m, the residues prove A = 0,
// and so the value is zero. A nonzero residue proves the value nonzero, so the
// attempt declines. Primality uses deterministic Miller-Rabin, which is exact
// below 2^64.
//
// The attempt visits distinct DAG nodes once, so shared subexpressions cost
// one evaluation per prime. Node count, field order and degree, and modular
// work have explicit limits. Exhaustion, cancellation, an unsupported leaf or
// an unproved reciprocal normally declines; none of these is retained as negative
// evidence. Publication is intended to require a completed zero proof. Extreme
// turn-order LCM overflow can violate field admission and publish an incorrect
// zero with overflow checks disabled; see the documented existing defect below.

/// Track an integrality denominator and a uniform complex-embedding magnitude bound.
/// For the represented field element A, denominator*A is an algebraic integer and
/// |sigma(A)|≤2^magnitude_bits for every embedding. These are conservative proof
/// bounds, not an evaluation or a bound on the full quotient. Clone copies the evidence.
#[derive(Clone)]
struct CyclotomicBound {
    /// Positive integer clearing the represented numerator into the ring of integers.
    denominator: BigUint,
    /// Conservative base-two magnitude exponent, possibly negative; arithmetic saturates.
    magnitude_bits: i64,
}

/// Propagate conservative numerator size and integrality evidence without modular work charges.
impl CyclotomicBound {
    /// Return the exact unit bound with integrality denominator one and magnitude exponent zero.
    fn one() -> Self {
        Self {
            denominator: BigUint::one(),
            magnitude_bits: 0,
        }
    }

    /// Bound a rational using numerator/denominator bit lengths and its positive denominator.
    /// The bit-length inequality is conservative, including for zero. Conversions and
    /// exponent arithmetic saturate; cloning the denominator is not charged here.
    fn rational(value: &Rational) -> Self {
        let numerator_bits = i64::try_from(value.numerator().bits()).unwrap_or(i64::MAX);
        let denominator_bits = i64::try_from(value.denominator().bits()).unwrap_or(i64::MAX);
        // |u/v| < 2^bits(u) / 2^(bits(v)-1).
        Self {
            denominator: value.denominator().clone(),
            magnitude_bits: numerator_bits
                .saturating_sub(denominator_bits)
                .saturating_add(1),
        }
    }

    /// Multiply integrality denominators and add magnitude exponents with saturation.
    /// The caller supplies admissible bounds and applies size guards after this allocation.
    fn product(&self, other: &Self) -> Self {
        Self {
            denominator: &self.denominator * &other.denominator,
            magnitude_bits: self.magnitude_bits.saturating_add(other.magnitude_bits),
        }
    }

    /// Use the LCM integrality denominator and one plus the larger embedding exponent.
    /// This triangle-inequality bound ignores cancellation and performs uncharged BigUint arithmetic.
    fn sum(&self, other: &Self) -> Self {
        Self {
            denominator: num::integer::Integer::lcm(&self.denominator, &other.denominator),
            magnitude_bits: self
                .magnitude_bits
                .max(other.magnitude_bits)
                .saturating_add(1),
        }
    }

    // Upper bound on log2(denominator * 2^magnitude_bits), the size a zero
    // proof must exceed.
    /// Return a conservative integer exponent for denominator*2^magnitude_bits.
    /// Use denominator bit length plus the magnitude exponent with saturation; the
    /// proof loop adds an extra bit before accumulating prime coverage.
    fn certificate_bits(&self) -> i64 {
        i64::try_from(self.denominator.bits())
            .unwrap_or(i64::MAX)
            .saturating_add(self.magnitude_bits)
    }
}

/// Exact generators interpreted consistently in a common cyclotomic field.
/// Rational pi turns use roots of unity; Root2/Root3 select the positive real roots
/// in the intended embedding. Modular proof evaluation also visits their conjugates.
#[derive(Clone)]
enum CyclotomicLeaf {
    // cos(pi * turn) or sin(pi * turn).
    /// Cosine of an exact rational multiple of pi.
    Cos(#[doc = "Exact turn r in cos(pi*r)."] Rational),
    /// Sine of an exact rational multiple of pi.
    Sin(#[doc = "Exact turn r in sin(pi*r)."] Rational),
    /// Positive sqrt(2), represented using eighth roots of unity.
    Root2,
    /// Positive sqrt(3), represented using twelfth roots of unity.
    Root3,
}

/// An admitted graph instruction whose operand indices precede its node.
/// Evaluation carries a numerator and a multiset of reciprocal-operand numerator
/// factors, so an Inverse instruction still requires a nonzero witness during proof.
/// Clone copies instruction data and indices without copying or validating the graph.
#[derive(Clone)]
enum CyclotomicTerm {
    /// Exact rational numerator with no symbolic denominator factors.
    Rational(
        #[doc = "Rational value; its denominator must be invertible modulo each selected prime."]
        Rational,
    ),
    // A rational multiple of a product of leaves.
    /// Rational coefficient times a product of admitted cyclotomic generators.
    Leaf(
        #[doc = "Exact rational scale."] Rational,
        #[doc = "Generators whose common field order has already been admitted."]
        Vec<CyclotomicLeaf>,
    ),
    /// Negate a prior numerator, preserving its denominator factors.
    Negate(#[doc = "Operand node index."] usize),
    /// Add prior quotients over the LCM of their symbolic denominator multisets.
    Add(
        #[doc = "Left operand index."] usize,
        #[doc = "Right operand index."] usize,
    ),
    /// Multiply prior numerators and add denominator-factor multiplicities.
    Multiply(
        #[doc = "Left operand index."] usize,
        #[doc = "Right operand index."] usize,
    ),
    /// Multiply a prior numerator by an exact rational coefficient.
    Scale(
        #[doc = "Operand node index."] usize,
        #[doc = "Exact scale, independently reduced modulo each prime."] Rational,
    ),
    /// Exchange an operand numerator with its denominator product.
    Inverse(
        #[doc = "Prior operand index; its numerator must obtain a nonzero residue witness."] usize,
    ),
}

// Sorted (reciprocal node, exponent) pairs. The factor of reciprocal node k
// is the numerator of its operand.
/// Sorted distinct (reciprocal node, positive exponent) pairs for a symbolic denominator.
/// Each factor is that reciprocal operand's numerator. Equality is by graph node ID,
/// not factor-value equality; no general algebraic factorization is performed.
type CyclotomicFactors = Vec<(usize, u32)>;

/// One topologically ordered instruction with bounds on its represented numerator.
/// The associated denominator factors are symbolic obligations, not yet nonzero
/// certificates; modular evaluation discharges each encountered reciprocal.
struct CyclotomicNode {
    /// Instruction defining this node from prior nodes or exact leaf data.
    term: CyclotomicTerm,
    /// Integrality and embedding bound for the numerator after symbolic denominator clearing.
    numerator: CyclotomicBound,
    /// Sorted reciprocal-node/exponent pairs whose product is the quotient's denominator.
    denominator: CyclotomicFactors,
}

/// Query-local cyclotomic program with its selected common root-of-unity order.
/// The last node is the queried result. Shared source allocation identities reuse
/// nodes; synthetic linear-form instructions also count toward the node cap.
struct CyclotomicGraph {
    /// Topologically ordered admitted instructions, with the queried root last.
    nodes: Vec<CyclotomicNode>,
    /// Positive common N for Q(zeta_N); construction starts at one and grows by LCM.
    order: u64,
}

/// A splitting prime and chosen primitive root power table for exact residue evaluation.
/// The admitted order divides prime-1; the table represents every power in that
/// cyclic group. This finite-field image alone is not an exact zero certificate.
struct CyclotomicModulus {
    /// Odd splitting prime in the proof search interval, below 2^62.
    prime: u64,
    // Primitive order-th root of unity modulo `prime`, as a power table.
    /// Order-many powers of one primitive root, beginning with one; indices wrap modulo order.
    powers: Vec<u64>,
    /// The residue of 1/2 modulo the odd prime, computed as ceil(prime/2).
    half: u64,
}

/// Attempt-local modular work allowance borrowing the queried root cancellation signal.
/// All-conjugate residues plus the graph numerator bound establish zero after enough
/// distinct splitting primes. Graph construction and nested coefficient operations
/// are not fully metered by this counter; no negative proof is cached.
struct CyclotomicZeroProof<'a> {
    /// Borrowed cancellation handle of the queried root, not a merged set of descendant handles.
    signal: &'a Option<Signal>,
    /// Remaining charged modular operations; this is neither precision nor an allocation budget.
    work_left: u64,
}

/// Admit a cyclotomic graph and publish only a completed exact zero certificate.
impl Computable {
    /// Maximum stored instructions, 4096, including synthetic linear-form nodes.
    const CYCLOTOMIC_MAX_NODES: usize = 4096;
    /// Maximum admitted root-of-unity order N, 16384, after order reconciliation.
    const CYCLOTOMIC_MAX_ORDER: u64 = 1 << 14;
    /// Maximum admitted field degree phi(N), 1024; distinct from N and source node count.
    const CYCLOTOMIC_MAX_DEGREE: u64 = 1024;
    /// Initial modular charge allowance, 2^25; graph admission uses separate limits.
    const CYCLOTOMIC_MAX_WORK: u64 = 1 << 25;
    // Size limits on the carried bound and on reciprocal exponents keep the
    // bound arithmetic itself bounded before any modular work is charged.
    /// Maximum carried certificate exponent, 65536; not a complete temporary-memory bound.
    const CYCLOTOMIC_MAX_CERTIFICATE_BITS: i64 = 1 << 16;
    /// Maximum retained reciprocal-factor exponent, 1024.
    const CYCLOTOMIC_MAX_EXPONENT: u32 = 1 << 10;
    /// Inclusive splitting-prime search floor 2^61, supporting 61 certified bits per prime.
    const CYCLOTOMIC_PRIME_FLOOR: u64 = 1 << 61;
    /// Initial exclusive splitting-prime search ceiling 2^62, also keeping residue sums in u64.
    const CYCLOTOMIC_PRIME_CEILING: u64 = 1 << 62;

    /// Attempt an exact zero certificate through all-conjugate modular evaluation.
    /// Admit rational arithmetic, rational-pi trig leaves and selected radical leaves
    /// into one common cyclotomic field. Every encountered inverse must acquire a
    /// nonzero operand-numerator residue witness; zero multiplication does not erase it.
    /// A final root stop check precedes exact zero-sign publication and the cyclotomic-zero
    /// trace. No numerical sample or retained rational tower is created.
    ///
    /// None combines unsupported leaves, nonzero residues, unproved inverse domains,
    /// size/work limits, unsuitable primes and cancellation; it is not a retained
    /// nonzero/undecidable fact. Graph admission and nested leaf/bound arithmetic are
    /// not wholly charged by the modular allowance. Existing limitation: very large
    /// rational-turn denominators can overflow unchecked u64 LCM arithmetic before
    /// order rejection. Debug checks panic; with overflow checks disabled, wrapped orders
    /// can incorrectly publish zero, e.g. for sin(pi/(2^62+1)). This is an unresolved
    /// correctness defect, not an admitted exact success.
    pub(crate) fn bounded_cyclotomic_zero(&self) -> Option<Rational> {
        let graph = self.cyclotomic_graph()?;
        let mut proof = CyclotomicZeroProof {
            signal: &self.signal,
            work_left: Self::CYCLOTOMIC_MAX_WORK,
        };
        proof.prove_zero(&graph)?;
        if should_stop(&self.signal) {
            return None;
        }
        self.internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(Sign::NoSign));
        crate::trace_dispatch!("computable", "normal_form", "cyclotomic-zero");
        Some(Rational::zero())
    }

    /// Build an iterative postorder program keyed by live source-node allocation identity.
    /// A LIFO stack pushes children in stored order, so siblings are ordinarily finalized
    /// right before left. Completed shared nodes are reused; distinct equal recipes need
    /// not merge. Expand linear forms into synthetic scales/sums, checking every operand.
    /// Poll the root signal on each stack pop; bound pending stack length and graph nodes.
    /// Leaf parsing and bound construction happen before modular work accounting.
    ///
    /// The resulting root is last. Failure discards local state and publishes nothing;
    /// leaf helpers may have their own retained facts. Large turn LCM arithmetic can
    /// panic or wrap to an invalid accepted order before its cap test. Nested exact
    /// recognition/square extraction is not
    /// a total-resource-bounded admission procedure.
    fn cyclotomic_graph(&self) -> Option<CyclotomicGraph> {
        let mut graph = CyclotomicGraph {
            nodes: Vec::new(),
            order: 1,
        };
        let mut index = std::collections::HashMap::<*const Node, usize>::new();
        let mut stack: Vec<(&Computable, bool)> = vec![(self, false)];
        while let Some((value, expanded)) = stack.pop() {
            if should_stop(&self.signal) {
                return None;
            }
            let key = Arc::as_ptr(&value.internal);
            if index.contains_key(&key) {
                continue;
            }
            if !expanded {
                if stack.len() > 4 * Self::CYCLOTOMIC_MAX_NODES {
                    return None;
                }
                stack.push((value, true));
                for child in value.cyclotomic_children()? {
                    if !index.contains_key(&Arc::as_ptr(&child.internal)) {
                        stack.push((child, false));
                    }
                }
                continue;
            }
            let child = |child: &Computable| index.get(&Arc::as_ptr(&child.internal)).copied();
            let term = match &value.internal.approximation {
                Approximation::One => CyclotomicTerm::Rational(Rational::one()),
                Approximation::Int(integer) => {
                    CyclotomicTerm::Rational(Rational::from_bigint(integer.clone()))
                }
                Approximation::Ratio(rational) => CyclotomicTerm::Rational(rational.clone()),
                Approximation::Negate(operand) => CyclotomicTerm::Negate(child(operand)?),
                Approximation::Add(left, right) => CyclotomicTerm::Add(child(left)?, child(right)?),
                Approximation::Multiply(left, right) => {
                    CyclotomicTerm::Multiply(child(left)?, child(right)?)
                }
                Approximation::Square(operand) => {
                    let operand = child(operand)?;
                    CyclotomicTerm::Multiply(operand, operand)
                }
                Approximation::Offset(operand, shift) => {
                    if shift.unsigned_abs() >= 1 << 16 {
                        return None;
                    }
                    CyclotomicTerm::Scale(child(operand)?, Self::power_of_two_rational(*shift))
                }
                Approximation::Inverse(operand) => CyclotomicTerm::Inverse(child(operand)?),
                Approximation::LinearCombination3(combination) => {
                    // Expand into scaled sums; the intermediate nodes have no
                    // DAG identity of their own.
                    let mut sum = None;
                    for (operand, scale) in combination.coefficients.iter().zip(&combination.values)
                    {
                        let scaled =
                            graph.push(CyclotomicTerm::Scale(child(operand)?, scale.clone()))?;
                        sum = Some(match sum {
                            None => scaled,
                            Some(previous) => graph.push(CyclotomicTerm::Add(previous, scaled))?,
                        });
                    }
                    index.insert(key, sum?);
                    continue;
                }
                _ => {
                    let (scale, leaves) = value.cyclotomic_leaf()?;
                    for leaf in &leaves {
                        graph.order = graph.require_order(leaf)?;
                    }
                    CyclotomicTerm::Leaf(scale, leaves)
                }
            };
            let node = graph.push(term)?;
            index.insert(key, node);
        }
        index.get(&Arc::as_ptr(&self.internal))?;
        Some(graph)
    }

    /// Return borrowed arithmetic children in recipe order, or no children for an admitted leaf.
    /// Classify non-arithmetic leaves here and again during finalization. This helper
    /// does not poll cancellation; leaf recognizers have separate costs and preconditions.
    fn cyclotomic_children(&self) -> Option<Vec<&Computable>> {
        Some(match &self.internal.approximation {
            Approximation::One | Approximation::Int(_) | Approximation::Ratio(_) => Vec::new(),
            Approximation::Negate(operand)
            | Approximation::Square(operand)
            | Approximation::Offset(operand, _)
            | Approximation::Inverse(operand) => vec![operand],
            Approximation::Add(left, right) | Approximation::Multiply(left, right) => {
                vec![left, right]
            }
            Approximation::LinearCombination3(combination) => {
                combination.coefficients.iter().collect()
            }
            // Leaves are classified again when the node is finalized.
            _ => {
                self.cyclotomic_leaf()?;
                Vec::new()
            }
        })
    }

    // A leaf is a rational multiple of a product of cyclotomic constants.
    /// Recognize a rational scale times rational-pi trig and selected radical generators.
    /// Use exact pi-multiple recognition for sine/cosine, and nonnegative exact rational
    /// recognition plus square extraction for sqrt. Admit extracted residuals 1,2,3,6;
    /// zero gives zero scale and no generators. This is a representation-dependent
    /// sufficient recognizer, not all cyclotomic numbers or all algebraic roots.
    /// No prescaled numerical kernel range is certified. Exact leaf helpers and square
    /// extraction precede modular charges and may allocate or use their own caches.
    fn cyclotomic_leaf(&self) -> Option<(Rational, Vec<CyclotomicLeaf>)> {
        match &self.internal.approximation {
            Approximation::Constant(SharedConstant::Sqrt2) => {
                Some((Rational::one(), vec![CyclotomicLeaf::Root2]))
            }
            Approximation::Constant(SharedConstant::Sqrt3) => {
                Some((Rational::one(), vec![CyclotomicLeaf::Root3]))
            }
            Approximation::PrescaledSin(argument) => Some((
                Rational::one(),
                vec![CyclotomicLeaf::Sin(argument.pi_rational_multiple()?)],
            )),
            Approximation::PrescaledCos(argument) => Some((
                Rational::one(),
                vec![CyclotomicLeaf::Cos(argument.pi_rational_multiple()?)],
            )),
            Approximation::Sqrt(radicand) => {
                // Only an exact nonnegative rational radicand denotes a real
                // root; sqrt(s^2 * m) = s * sqrt(m) for square-free m | 6.
                let radicand = radicand.exact_rational()?;
                if radicand.sign() == Sign::Minus {
                    return None;
                }
                let (scale, rest) = radicand.extract_square_reduced();
                if rest.is_zero() {
                    return Some((Rational::zero(), Vec::new()));
                }
                let leaves = match rest.to_integer_i64()? {
                    1 => Vec::new(),
                    2 => vec![CyclotomicLeaf::Root2],
                    3 => vec![CyclotomicLeaf::Root3],
                    6 => vec![CyclotomicLeaf::Root2, CyclotomicLeaf::Root3],
                    _ => return None,
                };
                Some((scale, leaves))
            }
            _ => None,
        }
    }
}

/// Build bounded numerator instructions and reconcile their reciprocal-factor multisets.
impl CyclotomicGraph {
    // The factor contributed by reciprocal node `node`: its operand's numerator.
    /// Resolve a denominator-factor ID to the reciprocal operand whose numerator it names.
    /// Indices must exist and name Inverse instructions; other tags panic as an invariant violation.
    fn factor_operand(&self, node: usize) -> usize {
        match self.nodes[node].term {
            CyclotomicTerm::Inverse(operand) => operand,
            _ => unreachable!("denominator factors name reciprocal nodes"),
        }
    }

    /// Multiply integrality/magnitude bounds for prior reciprocal-operand numerator powers.
    /// Check each exponent-scaled certificate estimate before materializing its denominator
    /// power and use checked magnitude scaling. The aggregate bound is checked later by
    /// push; this helper supplies no nonzero evidence or modular charge accounting.
    fn factor_bound(&self, factors: &[(usize, u32)]) -> Option<CyclotomicBound> {
        let mut bound = CyclotomicBound::one();
        for &(node, exponent) in factors {
            let factor = &self.nodes[self.factor_operand(node)].numerator;
            let bits = factor
                .certificate_bits()
                .max(1)
                .checked_mul(i64::from(exponent))?;
            if bits > Computable::CYCLOTOMIC_MAX_CERTIFICATE_BITS {
                return None;
            }
            bound = bound.product(&CyclotomicBound {
                denominator: factor.denominator.pow(exponent),
                magnitude_bits: factor.magnitude_bits.checked_mul(i64::from(exponent))?,
            });
        }
        Some(bound)
    }

    /// Append one topologically valid instruction after constructing its numerator bound.
    /// Sums reconcile denominator multisets, products add exponents, and an inverse
    /// introduces its own prospective node ID as a denominator factor. No divisor is
    /// proved nonzero yet. Check node count, final certificate size and exponent caps;
    /// None leaves nodes unchanged, though temporary bound arithmetic already occurred.
    /// The caller supplies valid earlier indices and compatible leaf/order data. These
    /// checks do not bound every allocation made before admission.
    fn push(&mut self, term: CyclotomicTerm) -> Option<usize> {
        if self.nodes.len() == Computable::CYCLOTOMIC_MAX_NODES {
            return None;
        }
        let (numerator, denominator) = match &term {
            CyclotomicTerm::Rational(value) => (CyclotomicBound::rational(value), Vec::new()),
            CyclotomicTerm::Leaf(scale, leaves) => {
                let mut bound = CyclotomicBound::rational(scale);
                for leaf in leaves {
                    // 2 cos and 2 sin are algebraic integers of magnitude at
                    // most 2; sqrt(2) and sqrt(3) are algebraic integers below
                    // 2 in every embedding.
                    let leaf_bound = match leaf {
                        CyclotomicLeaf::Cos(_) | CyclotomicLeaf::Sin(_) => CyclotomicBound {
                            denominator: BigUint::from(2_u8),
                            magnitude_bits: 0,
                        },
                        CyclotomicLeaf::Root2 | CyclotomicLeaf::Root3 => CyclotomicBound {
                            denominator: BigUint::one(),
                            magnitude_bits: 1,
                        },
                    };
                    bound = bound.product(&leaf_bound);
                }
                (bound, Vec::new())
            }
            CyclotomicTerm::Negate(operand) => {
                let operand = &self.nodes[*operand];
                (operand.numerator.clone(), operand.denominator.clone())
            }
            CyclotomicTerm::Add(left, right) => {
                let (common, left_missing, right_missing) = cyclotomic_lcm(
                    &self.nodes[*left].denominator,
                    &self.nodes[*right].denominator,
                );
                let left_term = self.nodes[*left]
                    .numerator
                    .product(&self.factor_bound(&left_missing)?);
                let right_term = self.nodes[*right]
                    .numerator
                    .product(&self.factor_bound(&right_missing)?);
                (left_term.sum(&right_term), common)
            }
            CyclotomicTerm::Multiply(left, right) => {
                let (left, right) = (&self.nodes[*left], &self.nodes[*right]);
                (
                    left.numerator.product(&right.numerator),
                    cyclotomic_merge(&left.denominator, &right.denominator)?,
                )
            }
            CyclotomicTerm::Scale(operand, scale) => {
                let operand = &self.nodes[*operand];
                (
                    operand.numerator.product(&CyclotomicBound::rational(scale)),
                    operand.denominator.clone(),
                )
            }
            CyclotomicTerm::Inverse(operand) => {
                // 1/(A / prod F) = prod F / A, and A becomes this node's factor.
                let bound = self.factor_bound(&self.nodes[*operand].denominator)?;
                (bound, vec![(self.nodes.len(), 1)])
            }
        };
        if numerator.certificate_bits() > Computable::CYCLOTOMIC_MAX_CERTIFICATE_BITS
            || denominator
                .iter()
                .any(|&(_, exponent)| exponent > Computable::CYCLOTOMIC_MAX_EXPONENT)
        {
            return None;
        }
        self.nodes.push(CyclotomicNode {
            term,
            numerator,
            denominator,
        });
        Some(self.nodes.len() - 1)
    }

    // cos(pi u/v) needs zeta_{2v}; sine also needs i = zeta_4.
    /// Compute a sufficient common N by LCM, then require N≤16384 and phi(N)≤1024.
    /// Cos(pi*u/v) requires 2v; sine additionally requires four, and the root generators
    /// require eight or twelve. Return the new order without modifying this graph.
    /// Denominator conversion/doubling is checked, but both LCM operations are not:
    /// large finite rational denominators can panic with debug checks or wrap when
    /// overflow checks are disabled. For sin(pi/(2^62+1)), wrapping admits order four
    /// and can produce an incorrect zero certificate. This existing defect remains
    /// unfixed; None/typed exhaustion is not guaranteed for that input envelope.
    fn require_order(&self, leaf: &CyclotomicLeaf) -> Option<u64> {
        let leaf_order = match leaf {
            CyclotomicLeaf::Cos(turn) | CyclotomicLeaf::Sin(turn) => {
                let twice = u64::try_from(turn.denominator()).ok()?.checked_mul(2)?;
                if matches!(leaf, CyclotomicLeaf::Sin(_)) {
                    num::integer::Integer::lcm(&twice, &4)
                } else {
                    twice
                }
            }
            CyclotomicLeaf::Root2 => 8,
            CyclotomicLeaf::Root3 => 12,
        };
        let order = num::integer::Integer::lcm(&self.order, &leaf_order);
        (order <= Computable::CYCLOTOMIC_MAX_ORDER
            && cyclotomic_totient(order) <= Computable::CYCLOTOMIC_MAX_DEGREE)
            .then_some(order)
    }
}

// Least common multiple of two factor multisets, with the factors each side
// lacks.
/// Reconcile sorted unique factor/exponent lists by pointwise maximum.
/// Return (common, missing_from_left, missing_from_right); missing entries are exponent
/// differences. This performs no polynomial factorization, size guard or cancellation poll.
fn cyclotomic_lcm(
    left: &[(usize, u32)],
    right: &[(usize, u32)],
) -> (CyclotomicFactors, CyclotomicFactors, CyclotomicFactors) {
    let (mut common, mut left_missing, mut right_missing) = (Vec::new(), Vec::new(), Vec::new());
    let (mut i, mut j) = (0, 0);
    while i < left.len() || j < right.len() {
        let order = match (left.get(i), right.get(j)) {
            (Some(a), Some(b)) => a.0.cmp(&b.0),
            (Some(_), None) => Ordering::Less,
            (None, _) => Ordering::Greater,
        };
        match order {
            Ordering::Less => {
                common.push(left[i]);
                right_missing.push(left[i]);
                i += 1;
            }
            Ordering::Greater => {
                common.push(right[j]);
                left_missing.push(right[j]);
                j += 1;
            }
            Ordering::Equal => {
                let (node, a, b) = (left[i].0, left[i].1, right[j].1);
                common.push((node, a.max(b)));
                if a < b {
                    left_missing.push((node, b - a));
                } else if b < a {
                    right_missing.push((node, a - b));
                }
                i += 1;
                j += 1;
            }
        }
    }
    (common, left_missing, right_missing)
}

// Product of two factor multisets.
/// Multiply sorted unique factor multisets by adding matching u32 exponents.
/// Checked exponent overflow returns None; other size/exponent caps belong to graph
/// admission. Inputs are unchanged and the merged allocation is local.
fn cyclotomic_merge(left: &[(usize, u32)], right: &[(usize, u32)]) -> Option<CyclotomicFactors> {
    let mut merged = left.to_vec();
    for &(node, exponent) in right {
        match merged.binary_search_by_key(&node, |&(factor, _)| factor) {
            Ok(position) => merged[position].1 = merged[position].1.checked_add(exponent)?,
            Err(position) => merged.insert(position, (node, exponent)),
        }
    }
    Some(merged)
}

/// Search splitting primes and evaluate numerator conjugates under one shared modular allowance.
impl CyclotomicZeroProof<'_> {
    /// Poll the root stop signal, then subtract amount with checked arithmetic.
    /// A declined charge leaves work_left unchanged; zero still polls. None has no reason tag.
    fn charge(&mut self, amount: u64) -> Option<()> {
        if should_stop(self.signal) {
            return None;
        }
        self.work_left = self.work_left.checked_sub(amount)?;
        Some(())
    }

    /// Certify the final numerator zero using all primitive conjugates and its carried bound.
    /// Require a nonempty valid graph with positive admitted order and sound numerator
    /// bounds. Enumerate unit exponents modulo N, then choose decreasing splitting primes.
    /// On the first prime, evaluate one conjugate as a cheap rejection test before the
    /// full set (that conjugate is evaluated again). Always process at least one prime.
    /// Each completed prime contributes 61 conservative bits; exceed the integrality-
    /// cleared embedding bound to force zero by the algebraic-integer norm argument.
    ///
    /// Any nonzero numerator residue declines. An unproved reciprocal or a rational
    /// denominator divisible by the selected prime also declines immediately, rather
    /// than trying another prime. None includes exhausted search/work and cancellation.
    /// This local certificate does not publish facts or supply the final caller stop poll.
    fn prove_zero(&mut self, graph: &CyclotomicGraph) -> Option<()> {
        let root = graph.nodes.last()?;
        let required_bits = root.numerator.certificate_bits().checked_add(1)?;
        let exponents = (0..graph.order)
            .filter(|exponent| num::integer::gcd(*exponent, graph.order) == 1)
            .collect::<Vec<_>>();
        let factors = cyclotomic_prime_factors(graph.order);
        let mut covered_bits = 0_i64;
        let mut below = Computable::CYCLOTOMIC_PRIME_CEILING;
        let mut first = true;
        // At least one prime is always evaluated.
        while first || covered_bits < required_bits {
            let modulus = self.modulus(graph.order, &factors, below)?;
            below = modulus.prime;
            let rationals = self.rational_residues(graph, modulus.prime)?;
            // A nonzero value is usually exposed by one conjugate; check it
            // before paying for all phi(N) of them.
            if first {
                first = false;
                if !self.evaluate(graph, &modulus, &rationals, &exponents[..1])? {
                    return None;
                }
            }
            if !self.evaluate(graph, &modulus, &rationals, &exponents)? {
                return None;
            }
            covered_bits = covered_bits.checked_add(61)?;
        }
        Some(())
    }

    /// Find a splitting prime below the supplied ceiling and an order-N root power table.
    /// Require positive bounded order, its complete distinct prime factors, and a ceiling
    /// large enough for below-2. Search p=1 mod N down to 2^61, with exact primality tests.
    /// For the first prime, try only bases 2 through 65 for a primitive root; failure
    /// of that bounded search declines rather than advancing to another prime.
    /// Charge candidates, a fixed root-search allowance and the power-table length;
    /// these charges are not individual arithmetic-operation or peak-byte counts.
    fn modulus(&mut self, order: u64, factors: &[u64], below: u64) -> Option<CyclotomicModulus> {
        let mut candidate = (below - 2) / order * order + 1;
        while candidate >= Computable::CYCLOTOMIC_PRIME_FLOOR {
            self.charge(1)?;
            if cyclotomic_is_prime(candidate) {
                self.charge(1024)?;
                let root = (2..).take(64).find_map(|base: u64| {
                    let root = cyclotomic_pow(base, (candidate - 1) / order, candidate);
                    factors
                        .iter()
                        .all(|factor| cyclotomic_pow(root, order / factor, candidate) != 1)
                        .then_some(root)
                })?;
                self.charge(order)?;
                let mut powers = Vec::with_capacity(order as usize);
                let mut power = 1;
                for _ in 0..order {
                    powers.push(power);
                    power = cyclotomic_mul(power, root, candidate);
                }
                return Some(CyclotomicModulus {
                    prime: candidate,
                    powers,
                    half: candidate.div_ceil(2),
                });
            }
            candidate = candidate.checked_sub(order)?;
        }
        None
    }

    // Residues of the rational in each rational, scale and leaf term. One
    // modular inversion serves every denominator (Montgomery's batch trick).
    /// Reduce rational coefficients for Rational, Leaf and Scale instructions modulo an odd prime.
    /// Leave unused result slots zero. Batch all nonzero denominator residues using one
    /// Fermat inversion and prefix products; the prime must actually be prime. A zero
    /// denominator residue returns None without searching another prime. Explicit charges
    /// cover coefficient visits and the batch estimate, not all BigUint reduction work.
    fn rational_residues(&mut self, graph: &CyclotomicGraph, prime: u64) -> Option<Vec<u64>> {
        let mut residues = vec![0; graph.nodes.len()];
        let mut denominators = Vec::new();
        for (index, node) in graph.nodes.iter().enumerate() {
            let value = match &node.term {
                CyclotomicTerm::Rational(value)
                | CyclotomicTerm::Leaf(value, _)
                | CyclotomicTerm::Scale(_, value) => value,
                _ => continue,
            };
            self.charge(8)?;
            let numerator = cyclotomic_magnitude_residue(value.numerator(), prime);
            residues[index] = if value.sign() == Sign::Minus {
                cyclotomic_sub(0, numerator, prime)
            } else {
                numerator
            };
            let denominator = cyclotomic_magnitude_residue(value.denominator(), prime);
            // A denominator divisible by the prime has no residue; decline
            // rather than search for another prime.
            if denominator == 0 {
                return None;
            }
            denominators.push((index, denominator));
        }
        let mut prefix = Vec::with_capacity(denominators.len());
        let mut product = 1;
        for &(_, denominator) in &denominators {
            prefix.push(product);
            product = cyclotomic_mul(product, denominator, prime);
        }
        self.charge(128 + 3 * denominators.len() as u64)?;
        let mut inverse = cyclotomic_pow(product, prime - 2, prime);
        for (&(index, denominator), &before) in denominators.iter().zip(&prefix).rev() {
            residues[index] = cyclotomic_mul(
                residues[index],
                cyclotomic_mul(inverse, before, prime),
                prime,
            );
            inverse = cyclotomic_mul(inverse, denominator, prime);
        }
        Some(residues)
    }

    // Product of factor residues raised to their exponents.
    /// Evaluate products of stored numerator-factor residues to their positive exponents.
    /// Use matching conjugate vectors from a valid earlier graph evaluation. Outer None
    /// means a declined charge; Some(None) is the empty factor product one, avoiding a
    /// vector allocation. Some(Some(v)) is the materialized product. Width/vector lengths
    /// and valid reciprocal IDs are caller invariants; this helper proves no nonzero fact.
    fn factor_residues(
        &mut self,
        graph: &CyclotomicGraph,
        values: &[Vec<u64>],
        factors: &[(usize, u32)],
        prime: u64,
        width: usize,
    ) -> Option<Option<Vec<u64>>> {
        let mut product: Option<Vec<u64>> = None;
        for &(node, exponent) in factors {
            self.charge(2 * u64::from(u32::BITS - exponent.leading_zeros()) * width as u64)?;
            let power = values[graph.factor_operand(node)]
                .iter()
                .map(|&value| cyclotomic_pow(value, u64::from(exponent), prime))
                .collect::<Vec<_>>();
            product = Some(match product {
                None => power,
                Some(product) => cyclotomic_zip(&product, &power, prime, cyclotomic_mul),
            });
        }
        Some(product)
    }

    // Returns whether the root numerator vanishes at every listed conjugate.
    /// Evaluate graph numerators at the listed primitive conjugates of one splitting prime.
    /// Maintain one vector per graph node; addition clears missing denominator factors,
    /// while inverse substitutes its operand's denominator product after finding at least
    /// one nonzero operand-numerator residue among the selected conjugates. All-zero
    /// operand residues decline, even if another prime or conjugate could prove nonzero.
    /// No modular division by those symbolic factors is performed.
    ///
    /// Return Some(true) when the root numerator vanishes at every supplied conjugate,
    /// Some(false) otherwise; neither alone is the complete height-bounded zero proof.
    /// Require nonempty conjugate lists for meaningful inverse/root decisions, matching
    /// rational slots and a valid graph/modulus. Charging is per graph/vector work, not
    /// all allocation or nested integer work; no facts or negative result are retained.
    fn evaluate(
        &mut self,
        graph: &CyclotomicGraph,
        modulus: &CyclotomicModulus,
        rationals: &[u64],
        exponents: &[u64],
    ) -> Option<bool> {
        let prime = modulus.prime;
        let width = exponents.len();
        let mut values: Vec<Vec<u64>> = Vec::with_capacity(graph.nodes.len());
        for (index, node) in graph.nodes.iter().enumerate() {
            self.charge(3 * width as u64)?;
            let value = match &node.term {
                CyclotomicTerm::Rational(_) => vec![rationals[index]; width],
                CyclotomicTerm::Leaf(_, leaves) => exponents
                    .iter()
                    .map(|&exponent| {
                        leaves.iter().fold(rationals[index], |product, leaf| {
                            let leaf =
                                cyclotomic_leaf_residue(leaf, exponent, graph.order, modulus);
                            cyclotomic_mul(product, leaf, prime)
                        })
                    })
                    .collect(),
                CyclotomicTerm::Negate(operand) => values[*operand]
                    .iter()
                    .map(|&value| cyclotomic_sub(0, value, prime))
                    .collect(),
                CyclotomicTerm::Add(left, right) => {
                    let (_, left_missing, right_missing) = cyclotomic_lcm(
                        &graph.nodes[*left].denominator,
                        &graph.nodes[*right].denominator,
                    );
                    let scaled = |proof: &mut Self, operand: usize, missing| {
                        Some(
                            match proof.factor_residues(graph, &values, missing, prime, width)? {
                                Some(factor) => {
                                    cyclotomic_zip(&values[operand], &factor, prime, cyclotomic_mul)
                                }
                                None => values[operand].clone(),
                            },
                        )
                    };
                    let left = scaled(self, *left, &left_missing)?;
                    let right = scaled(self, *right, &right_missing)?;
                    cyclotomic_zip(&left, &right, prime, cyclotomic_add)
                }
                CyclotomicTerm::Multiply(left, right) => {
                    cyclotomic_zip(&values[*left], &values[*right], prime, cyclotomic_mul)
                }
                CyclotomicTerm::Scale(operand, _) => values[*operand]
                    .iter()
                    .map(|&value| cyclotomic_mul(value, rationals[index], prime))
                    .collect(),
                CyclotomicTerm::Inverse(operand) => {
                    // A nonzero residue proves the operand's numerator, and so
                    // the operand, nonzero: a zero field element reduces to
                    // zero modulo every prime. Otherwise the reciprocal is
                    // unproved and the attempt declines.
                    if values[*operand].iter().all(|&value| value == 0) {
                        return None;
                    }
                    let denominator = &graph.nodes[*operand].denominator;
                    self.factor_residues(graph, &values, denominator, prime, width)?
                        .unwrap_or_else(|| vec![1; width])
                }
            };
            values.push(value);
        }
        Some(values.last()?.iter().all(|&value| value == 0))
    }
}

/// Evaluate a selected generator under zeta_N mapped to the exponent-th primitive root.
/// Use (z^a+z^-a)/2 for cosine and (z^a-z^-a)/(2i) for sine, including signed
/// rational turns; root2/root3 are conjugate sums at orders eight/twelve. Require
/// admitted nonzero order, compatible powers/prime and all leaf-order divisibilities.
/// This is a conjugate residue, not necessarily the intended real embedding's value.
/// No charge or stop poll occurs inside this helper.
fn cyclotomic_leaf_residue(
    leaf: &CyclotomicLeaf,
    exponent: u64,
    order: u64,
    modulus: &CyclotomicModulus,
) -> u64 {
    let prime = modulus.prime;
    // zeta^(exponent * k) for the conjugate selected by `exponent`.
    let power = |k: u64| {
        modulus.powers[((u128::from(exponent) * u128::from(k)) % u128::from(order)) as usize]
    };
    let conjugate_pair = |k: u64, sum: bool| {
        let forward = power(k);
        let backward = power(order - k % order);
        if sum {
            cyclotomic_add(forward, backward, prime)
        } else {
            cyclotomic_sub(forward, backward, prime)
        }
    };
    match leaf {
        CyclotomicLeaf::Cos(turn) | CyclotomicLeaf::Sin(turn) => {
            // cos(pi u/v) = (zeta^a + zeta^-a)/2 and
            // sin(pi u/v) = (zeta^a - zeta^-a)/(2i), with zeta^a = zeta_{2v}^u
            // and 1/i = -i = zeta^(3N/4).
            let twice = 2 * u64::try_from(turn.denominator())
                .expect("the admitted order bounds the denominator");
            let numerator = u64::try_from(turn.numerator() % BigUint::from(twice))
                .expect("a residue is below its modulus");
            let a = numerator * (order / twice) % order;
            let a = if turn.sign() == Sign::Minus {
                (order - a) % order
            } else {
                a
            };
            if matches!(leaf, CyclotomicLeaf::Cos(_)) {
                cyclotomic_mul(conjugate_pair(a, true), modulus.half, prime)
            } else {
                let reciprocal_i = power(3 * order / 4);
                cyclotomic_mul(
                    cyclotomic_mul(conjugate_pair(a, false), reciprocal_i, prime),
                    modulus.half,
                    prime,
                )
            }
        }
        // sqrt(2) = zeta_8 + zeta_8^-1 and sqrt(3) = zeta_12 + zeta_12^-1.
        CyclotomicLeaf::Root2 => conjugate_pair(order / 8, true),
        CyclotomicLeaf::Root3 => conjugate_pair(order / 12, true),
    }
}

/// Reduce an unsigned magnitude modulo a nonzero prime, using u64 when it fits.
/// Larger values allocate a BigUint remainder; this helper has no charge or stop poll.
fn cyclotomic_magnitude_residue(magnitude: &BigUint, prime: u64) -> u64 {
    match u64::try_from(magnitude) {
        Ok(small) => small % prime,
        Err(_) => u64::try_from(magnitude % BigUint::from(prime)).expect("below the prime"),
    }
}

/// Apply a supplied modular binary operation pairwise to equal-width residue vectors.
/// The caller must ensure equal lengths; iterator zip otherwise silently truncates
/// to the shorter slice. Allocate the result without internal charging or polling.
fn cyclotomic_zip(
    left: &[u64],
    right: &[u64],
    prime: u64,
    op: fn(u64, u64, u64) -> u64,
) -> Vec<u64> {
    left.iter()
        .zip(right)
        .map(|(&left, &right)| op(left, right, prime))
        .collect()
}

/// Add normalized residues modulo a positive prime below 2^62.
/// The caller guarantees operands below the prime, so one subtraction suffices and
/// u64 addition cannot overflow. No primality check, charge or poll is performed.
fn cyclotomic_add(left: u64, right: u64, prime: u64) -> u64 {
    let sum = left + right;
    if sum >= prime { sum - prime } else { sum }
}

/// Subtract normalized residues modulo a positive prime below 2^62.
/// Caller bounds make the conditional addition safe in u64; no validation or poll occurs.
fn cyclotomic_sub(left: u64, right: u64, prime: u64) -> u64 {
    if left >= right {
        left - right
    } else {
        left + prime - right
    }
}

/// Multiply two u64 residues using a u128 product and reduce modulo a nonzero modulus.
/// The modulus need not be prime for this arithmetic identity. Zero modulus panics;
/// this primitive performs no charge or cancellation poll.
fn cyclotomic_mul(left: u64, right: u64, prime: u64) -> u64 {
    ((u128::from(left) * u128::from(right)) % u128::from(prime)) as u64
}

/// Compute modular exponentiation by repeated squaring for a nonzero modulus.
/// Normalize the base, return 1 mod modulus at exponent zero, and perform no
/// primality check, work debit or internal cancellation poll.
fn cyclotomic_pow(mut base: u64, mut exponent: u64, prime: u64) -> u64 {
    let mut result = 1 % prime;
    base %= prime;
    while exponent > 0 {
        if exponent & 1 == 1 {
            result = cyclotomic_mul(result, base, prime);
        }
        base = cyclotomic_mul(base, base, prime);
        exponent >>= 1;
    }
    result
}

/// Decide primality for every u64 using small-prime division and twelve Miller–Rabin bases.
/// The first counterexample for these bases is 318665857834031151167461, above u64::MAX;
/// see [Sorenson and Webster, Theorem 1.1](https://arxiv.org/html/1509.00864v1).
/// The former 3.3e24 comment corresponded to thirteen bases, not these twelve.
/// Values below two are false; listed small primes are handled before witness tests.
/// Uses exact u128 modular multiplication with no internal work debit or stop poll.
fn cyclotomic_is_prime(candidate: u64) -> bool {
    /// The first twelve prime bases, 2 through 37, for the deterministic u64 witness test.
    const BASES: [u64; 12] = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37];
    if candidate < 2 {
        return false;
    }
    for base in BASES {
        if candidate.is_multiple_of(base) {
            return candidate == base;
        }
    }
    let twos = (candidate - 1).trailing_zeros();
    let odd = (candidate - 1) >> twos;
    BASES.iter().all(|&base| {
        let mut value = cyclotomic_pow(base, odd, candidate);
        if value == 1 || value == candidate - 1 {
            return true;
        }
        for _ in 1..twos {
            value = cyclotomic_mul(value, value, candidate);
            if value == candidate - 1 {
                return true;
            }
        }
        false
    })
}

/// Return ascending distinct prime factors by trial division on a positive admitted order.
/// One gives an empty list. Callers restrict values to the small field-order envelope;
/// the unchecked factor*factor loop is not a safe general near-u64::MAX factorizer.
/// No cancellation or work accounting occurs inside this helper.
fn cyclotomic_prime_factors(mut value: u64) -> Vec<u64> {
    let mut factors = Vec::new();
    let mut factor = 2;
    while factor * factor <= value {
        if value.is_multiple_of(factor) {
            factors.push(factor);
            while value.is_multiple_of(factor) {
                value /= factor;
            }
        }
        factor += 1;
    }
    if value > 1 {
        factors.push(value);
    }
    factors
}

/// Compute Euler phi for a positive admitted order from its distinct prime factors.
/// One yields one. This inherits the small-order trial-division precondition and
/// does not provide charged general integer factorization.
fn cyclotomic_totient(value: u64) -> u64 {
    cyclotomic_prime_factors(value)
        .into_iter()
        .fold(value, |totient, factor| totient / factor * (factor - 1))
}
