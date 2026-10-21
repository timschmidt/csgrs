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
// an unproved reciprocal declines; none of these is retained as negative
// evidence. Only a completed zero proof is published.

/// A field element tracked symbolically: `denominator * value` is an
/// algebraic integer and every complex embedding has magnitude at most
/// `2^magnitude_bits`.
#[derive(Clone)]
struct CyclotomicBound {
    denominator: BigUint,
    magnitude_bits: i64,
}

impl CyclotomicBound {
    fn one() -> Self {
        Self {
            denominator: BigUint::one(),
            magnitude_bits: 0,
        }
    }

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

    fn product(&self, other: &Self) -> Self {
        Self {
            denominator: &self.denominator * &other.denominator,
            magnitude_bits: self.magnitude_bits.saturating_add(other.magnitude_bits),
        }
    }

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
    fn certificate_bits(&self) -> i64 {
        i64::try_from(self.denominator.bits())
            .unwrap_or(i64::MAX)
            .saturating_add(self.magnitude_bits)
    }
}

#[derive(Clone)]
enum CyclotomicLeaf {
    // cos(pi * turn) or sin(pi * turn).
    Cos(Rational),
    Sin(Rational),
    Root2,
    Root3,
}

#[derive(Clone)]
enum CyclotomicTerm {
    Rational(Rational),
    // A rational multiple of a product of leaves.
    Leaf(Rational, Vec<CyclotomicLeaf>),
    Negate(usize),
    Add(usize, usize),
    Multiply(usize, usize),
    Scale(usize, Rational),
    Inverse(usize),
}

// Sorted (reciprocal node, exponent) pairs. The factor of reciprocal node k
// is the numerator of its operand.
type CyclotomicFactors = Vec<(usize, u32)>;

struct CyclotomicNode {
    term: CyclotomicTerm,
    numerator: CyclotomicBound,
    denominator: CyclotomicFactors,
}

struct CyclotomicGraph {
    nodes: Vec<CyclotomicNode>,
    order: u64,
}

struct CyclotomicModulus {
    prime: u64,
    // Primitive order-th root of unity modulo `prime`, as a power table.
    powers: Vec<u64>,
    half: u64,
}

struct CyclotomicZeroProof<'a> {
    signal: &'a Option<Signal>,
    work_left: u64,
}

impl Computable {
    const CYCLOTOMIC_MAX_NODES: usize = 4096;
    const CYCLOTOMIC_MAX_ORDER: u64 = 1 << 14;
    const CYCLOTOMIC_MAX_DEGREE: u64 = 1024;
    const CYCLOTOMIC_MAX_WORK: u64 = 1 << 25;
    // Size limits on the carried bound and on reciprocal exponents keep the
    // bound arithmetic itself bounded before any modular work is charged.
    const CYCLOTOMIC_MAX_CERTIFICATE_BITS: i64 = 1 << 16;
    const CYCLOTOMIC_MAX_EXPONENT: u32 = 1 << 10;
    const CYCLOTOMIC_PRIME_FLOOR: u64 = 1 << 61;
    const CYCLOTOMIC_PRIME_CEILING: u64 = 1 << 62;

    /// Prove that this expression is exactly zero when all of its leaves are
    /// cyclotomic constants. Returns `Some(0)` only with a completed proof.
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

impl CyclotomicGraph {
    // The factor contributed by reciprocal node `node`: its operand's numerator.
    fn factor_operand(&self, node: usize) -> usize {
        match self.nodes[node].term {
            CyclotomicTerm::Inverse(operand) => operand,
            _ => unreachable!("denominator factors name reciprocal nodes"),
        }
    }

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

impl CyclotomicZeroProof<'_> {
    fn charge(&mut self, amount: u64) -> Option<()> {
        if should_stop(self.signal) {
            return None;
        }
        self.work_left = self.work_left.checked_sub(amount)?;
        Some(())
    }

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

fn cyclotomic_magnitude_residue(magnitude: &BigUint, prime: u64) -> u64 {
    match u64::try_from(magnitude) {
        Ok(small) => small % prime,
        Err(_) => u64::try_from(magnitude % BigUint::from(prime)).expect("below the prime"),
    }
}

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

fn cyclotomic_add(left: u64, right: u64, prime: u64) -> u64 {
    let sum = left + right;
    if sum >= prime { sum - prime } else { sum }
}

fn cyclotomic_sub(left: u64, right: u64, prime: u64) -> u64 {
    if left >= right {
        left - right
    } else {
        left + prime - right
    }
}

fn cyclotomic_mul(left: u64, right: u64, prime: u64) -> u64 {
    ((u128::from(left) * u128::from(right)) % u128::from(prime)) as u64
}

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

// Deterministic Miller-Rabin: the first twelve prime bases are exact for
// every n < 3.3 * 10^24 (Sorenson and Webster, 2015), hence for all u64.
fn cyclotomic_is_prime(candidate: u64) -> bool {
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

fn cyclotomic_totient(value: u64) -> u64 {
    cyclotomic_prime_factors(value)
        .into_iter()
        .fold(value, |totient, factor| totient / factor * (factor - 1))
}
