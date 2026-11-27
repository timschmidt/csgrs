/// Recursive coefficient tree over a query-local sequence of principal square roots.
///
/// Each extension represents `low + high*sqrt(radicands[level])`, with both
/// coefficients over lower levels in the same RadicalTower. Levels have no
/// meaning without that tower; separately built towers cannot exchange elements
/// merely because their indices agree. Dependent generators are permitted, so
/// equal scalar values need not have equal trees or structurally zero differences.
/// Derived equality compares presentation, including exact rational leaf values;
/// comparing distinct rational presentations can require arbitrary-width arithmetic.
/// Cloning duplicates the boxed coefficient tree while rational storage can share
/// its own data; this representation is not a shared construction DAG.
#[derive(Clone, Debug, PartialEq)]
enum RadicalElement {
    /// Exact rational coefficient with no root generator in use.
    Rational(
        /// Signed rational value; not a numerical approximation or root selector.
        Rational,
    ),
    /// `low + high * sqrt(generator level)`; both parts use lower levels.
    Extension {
        /// Index of this expression's highest used generator in the owning tower.
        level: usize,
        /// Coefficient independent of this generator.
        low: Box<RadicalElement>,
        /// Multiplier of its selected positive root, possibly over earlier generators.
        high: Box<RadicalElement>,
    },
}

/// Query-local square relations and the arithmetic that consumes them.
///
/// Each appended radicand has a proved positive sign and uses only earlier
/// generators. Sign comparison by squared magnitudes stays valid for dependent
/// roots; conjugate inversion can nevertheless decline a nonzero value when its
/// retained norm vanishes. The carrier is not a complete algebraic-number solver.
struct RadicalTower {
    /// Positive root squares in dependency order; elements refer to these by index.
    radicands: Vec<RadicalElement>,
}

/// Maximum completed distinct-node entries in one traversal memo.
/// This does not bound pending frames, depth before the first completed node,
/// helper recognizer visits, coefficient arithmetic or total work/memory.
const RADICAL_TOWER_MAX_NODES: usize = 256;
/// Maximum appended positive-root generators, after rational-root and exact-tree reuse attempts.
const RADICAL_TOWER_MAX_GENERATORS: usize = 10;
/// Maximum recursive rational-leaf count at element guard sites, including zero leaves.
/// Checks after construction do not bound every intermediate allocation or arithmetic operation.
const RADICAL_TOWER_MAX_TERMS: usize = 256;
/// Maximum numerator-plus-denominator bits of any leaf at element guard sites.
/// This is neither a sum across all coefficients nor a universal input-size cap;
/// exact-rational traversal shortcuts bypass the element guard.
const RADICAL_TOWER_MAX_COEFFICIENT_BITS: u64 = 4_096;

/// Coefficient-tree operations independent of the owning tower's square relations.
impl RadicalElement {
    /// Construct rational zero without introducing a generator.
    fn zero() -> Self {
        Self::Rational(Rational::zero())
    }

    /// Return the outer stored level, or None for a rational leaf.
    /// Valid trees place every coefficient at a strictly lower level.
    fn level(&self) -> Option<usize> {
        match self {
            Self::Rational(_) => None,
            Self::Extension { level, .. } => Some(*level),
        }
    }

    /// Count rational leaves recursively, including explicit zeros and repeated subtrees.
    /// This is a representation-size measure, not the number of nonzero monomials.
    fn terms(&self) -> usize {
        match self {
            Self::Rational(_) => 1,
            Self::Extension { low, high, .. } => low.terms() + high.terms(),
        }
    }

    /// Take the maximum canonical numerator-plus-denominator bit count among rational leaves.
    /// The rational accessors may canonicalize and cache unreduced fractions; do not
    /// sum across the tree or treat this query as allocation-free inspection.
    fn coefficient_bits(&self) -> u64 {
        match self {
            Self::Rational(value) => value.numerator().bits() + value.denominator().bits(),
            Self::Extension { low, high, .. } => {
                low.coefficient_bits().max(high.coefficient_bits())
            }
        }
    }

    /// Check this completed tree against the leaf-count and per-leaf bit thresholds.
    /// Despite the name, this is not a cumulative work, memory, precision or cancellation budget.
    /// Callers choose the guard sites; constructing or visiting the tree already costs work.
    fn within_budget(&self) -> bool {
        self.terms() <= RADICAL_TOWER_MAX_TERMS
            && self.coefficient_bits() <= RADICAL_TOWER_MAX_COEFFICIENT_BITS
    }

    /// Require every rational leaf to be zero, without using any root-square relation.
    /// Dependent-root cancellation may have scalar value zero while this returns false.
    fn is_structurally_zero(&self) -> bool {
        match self {
            Self::Rational(value) => value.sign() == Sign::NoSign,
            Self::Extension { low, high, .. } => {
                low.is_structurally_zero() && high.is_structurally_zero()
            }
        }
    }

    /// Build the formal element `0 + 1*sqrt(radicands[level])`.
    /// The caller must supply a valid positive-root index in the same tower.
    fn generator(level: usize) -> Self {
        Self::Extension {
            level,
            low: Box::new(Self::zero()),
            high: Box::new(Self::Rational(Rational::one())),
        }
    }

    /// Split into coefficients over a requested level at least as high as this tree's level.
    /// An exact level match unwraps the extension; otherwise return this value as
    /// low and zero as high. The level-order precondition is supplied by callers,
    /// not checked here; this is not arbitrary coefficient extraction.
    fn split(self, level: usize) -> (Self, Self) {
        match self {
            Self::Extension {
                level: own,
                low,
                high,
            } if own == level => (*low, *high),
            other => (other, Self::zero()),
        }
    }

    /// Assemble an extension, dropping it only when high is structurally zero.
    /// The caller supplies lower-level coefficients; this neither validates levels nor applies size guards.
    fn join(level: usize, low: Self, high: Self) -> Self {
        if high.is_structurally_zero() {
            low
        } else {
            Self::Extension {
                level,
                low: Box::new(low),
                high: Box::new(high),
            }
        }
    }

    /// Negate every rational leaf while retaining the tree shape and selected generators.
    fn neg(self) -> Self {
        match self {
            Self::Rational(value) => Self::Rational(-value),
            Self::Extension { level, low, high } => Self::Extension {
                level,
                low: Box::new(low.neg()),
                high: Box::new(high.neg()),
            },
        }
    }

    /// Scale every leaf by an exact rational and collapse structurally zero high parts.
    /// No size guard is applied here; zero factors still traverse the coefficient tree.
    fn scale(self, factor: &Rational) -> Self {
        match self {
            Self::Rational(value) => Self::Rational(&value * factor),
            Self::Extension { level, low, high } => {
                Self::join(level, low.scale(factor), high.scale(factor))
            }
        }
    }

    /// Align operands at their highest stored level, add corresponding coefficients and collapse unused extensions.
    /// Both operands must belong to the same tower. Addition is exact but does not apply
    /// size guards or semantic zero reduction; temporary/result trees may exceed guard thresholds.
    fn add(self, other: Self) -> Self {
        match (self.level(), other.level()) {
            (None, None) => {
                let (Self::Rational(left), Self::Rational(right)) = (self, other) else {
                    unreachable!("levelless radical elements are rational")
                };
                Self::Rational(&left + &right)
            }
            (left, right) => {
                let level = left.max(right).expect("one element has a generator");
                let (left_low, left_high) = self.split(level);
                let (right_low, right_high) = other.split(level);
                Self::join(level, left_low.add(right_low), left_high.add(right_high))
            }
        }
    }
}

/// Arithmetic and selected-root sign proofs using this query's ordered square relations.
impl RadicalTower {
    /// Multiply by aligning the highest generator and replacing its square with the retained radicand.
    /// Use `(a+b*s)*(c+d*s) = a*c+b*d*r + (a*d+b*c)*s`, where `s^2=r`.
    /// Recursive products check the resulting element thresholds, including rational
    /// products and scaling paths; these checks occur after arithmetic/construction.
    /// None reports a guard decline. Invalid generator indices violate internal invariants.
    fn mul(&self, left: RadicalElement, right: RadicalElement) -> Option<RadicalElement> {
        let product = match (left.level(), right.level()) {
            (None, None) => {
                let (RadicalElement::Rational(left), RadicalElement::Rational(right)) =
                    (left, right)
                else {
                    unreachable!("levelless radical elements are rational")
                };
                RadicalElement::Rational(&left * &right)
            }
            (None, Some(_)) => {
                let RadicalElement::Rational(factor) = left else {
                    unreachable!("levelless radical elements are rational")
                };
                right.scale(&factor)
            }
            (Some(_), None) => {
                let RadicalElement::Rational(factor) = right else {
                    unreachable!("levelless radical elements are rational")
                };
                left.scale(&factor)
            }
            (Some(left_level), Some(right_level)) => {
                let level = left_level.max(right_level);
                let (left_low, left_high) = left.split(level);
                let (right_low, right_high) = right.split(level);
                // (a + b s)(c + d s) = ac + bd r + (ad + bc) s, with s^2 = r.
                let high_product = self.mul(left_high.clone(), right_high.clone())?;
                let low = self
                    .mul(left_low.clone(), right_low.clone())?
                    .add(self.mul(high_product, self.radicands[level].clone())?);
                let high = self
                    .mul(left_low, right_high)?
                    .add(self.mul(left_high, right_low)?);
                RadicalElement::join(level, low, high)
            }
        };
        product.within_budget().then_some(product)
    }

    /// Prove the sign recursively using positive selected roots and exact lower-level arithmetic.
    /// A zero high part or equal term signs decide immediately. Opposed signs reduce
    /// to the sign of `low^2 - high^2*radicand`, with the low sign fixing orientation.
    /// This remains valid for dependent generators, but multiplication guards can decline.
    /// There is no separate guard on the difference after addition or on recursive sign
    /// work; rational leaves return their sign regardless of their bit length.
    fn sign(&self, value: &RadicalElement) -> Option<RealSign> {
        let RadicalElement::Extension { level, low, high } = value else {
            let RadicalElement::Rational(value) = value else {
                unreachable!("levelless radical elements are rational")
            };
            return Some(rational_sign(value));
        };
        let low_sign = self.sign(low)?;
        let high_sign = self.sign(high)?;
        if high_sign == RealSign::Zero || low_sign == high_sign {
            return Some(low_sign);
        }
        if low_sign == RealSign::Zero {
            // Radicands are certified positive, so the root is positive.
            return Some(high_sign);
        }
        // a + b s with opposite signs: compare a^2 with b^2 r.
        let low_square = self.mul((**low).clone(), (**low).clone())?;
        let high_square = self.mul((**high).clone(), (**high).clone())?;
        let scaled = self.mul(high_square, self.radicands[*level].clone())?;
        let difference = low_square.add(scaled.neg());
        let difference_sign = self.sign(&difference)?;
        Some(match (low_sign, difference_sign) {
            (_, RealSign::Zero) => RealSign::Zero,
            (RealSign::Positive, sign) => sign,
            (RealSign::Negative, RealSign::Positive) => RealSign::Negative,
            (RealSign::Negative, RealSign::Negative) => RealSign::Positive,
            (RealSign::Zero, _) => unreachable!("a zero low part returned above"),
        })
    }

    /// Invert by the outer conjugate divided by `low^2 - high^2*radicand`, recursively.
    /// Rational zero, zero norm and undecided guarded arithmetic return None. A dependent
    /// generator can give zero norm for a nonzero value, so decline is not a zero proof.
    /// Direct rational inversion has no element guard here; later traversal may apply one.
    fn inverse(&self, value: RadicalElement) -> Option<RadicalElement> {
        match value {
            RadicalElement::Rational(value) => {
                if value.sign() == Sign::NoSign {
                    return None;
                }
                Some(RadicalElement::Rational(Rational::one() / value))
            }
            RadicalElement::Extension { level, low, high } => {
                // 1 / (a + b s) = (a - b s) / (a^2 - b^2 r).
                let low_square = self.mul((*low).clone(), (*low).clone())?;
                let high_square = self.mul((*high).clone(), (*high).clone())?;
                let norm =
                    low_square.add(self.mul(high_square, self.radicands[level].clone())?.neg());
                // A zero norm prevents conjugate inversion. It can arise from
                // a zero value or a dependent generator representing a nonzero value.
                if self.sign(&norm)? == RealSign::Zero {
                    return None;
                }
                let inverse_norm = self.inverse(norm)?;
                let conjugate = RadicalElement::join(level, *low, high.neg());
                self.mul(conjugate, inverse_norm)
            }
        }
    }

    /// Select the principal root after proving the radicand sign: zero collapses and negative declines.
    /// For positive input, first try a rational root, then reuse an exactly equal retained
    /// radicand tree, then append a generator if its count permits. Tree comparison is
    /// within this tower and does not identify all semantically equal radicands. The new
    /// relation is positive and depends only on earlier levels; no full field-minimization
    /// or independent-generator proof is required. Sign/size recognition can decline.
    fn square_root(&mut self, radicand: RadicalElement) -> Option<RadicalElement> {
        match self.sign(&radicand)? {
            RealSign::Zero => Some(RadicalElement::zero()),
            RealSign::Negative => None,
            RealSign::Positive => {
                if let RadicalElement::Rational(value) = &radicand
                    && let Some(root) = rational_square_root(value)
                {
                    return Some(RadicalElement::Rational(root));
                }
                // Separately built roots of one radicand name one generator,
                // so cancellations between them reduce structurally.
                if let Some(level) = self.radicands.iter().position(|known| known == &radicand) {
                    return Some(RadicalElement::generator(level));
                }
                if self.radicands.len() >= RADICAL_TOWER_MAX_GENERATORS {
                    return None;
                }
                self.radicands.push(radicand);
                Some(RadicalElement::generator(self.radicands.len() - 1))
            }
        }
    }
}

/// Recognize a rational principal root from square factors of its reduced numerator and denominator.
/// The caller supplies nonnegative input: the sign is not checked here. Both
/// residuals must reduce to one through the shared partial square extractor.
/// A decline need not prove nonsquareness, especially beyond that extractor's
/// size/probe envelope. Numerator/denominator accessors can canonicalize and cache
/// the input fraction before the square-extraction attempts.
fn rational_square_root(value: &Rational) -> Option<Rational> {
    let (root, remainder) = square_parts(value.numerator());
    let (denominator_root, denominator_remainder) = square_parts(value.denominator());
    (remainder.is_one() && denominator_remainder.is_one()).then(|| &root / &denominator_root)
}

/// Attempt an exact sign in an iterated principal-square-root tower using a fresh local basis.
///
/// Iterative postorder memoizes each completed shared node by pointer identity.
/// Recognize exact rational shortcuts, sqrt(2) and sqrt(3) constants, ring operations,
/// inverse, binary shifts, square roots (including NthRoot index two), and rational
/// three-term combinations. Zero weights still require coefficient reduction.
/// Retained PrescaledSin leaves admit only arguments recognized as pi/8 or 3*pi/8;
/// positive half-angle roots prove their values without replacing the sine nodes.
/// The separate pi-multiple recognizer owns its own traversal limits.
///
/// The node guard counts completed memo entries, not pending stack depth or total
/// work. Exact-rational shortcuts bypass element-size checks; other completed nodes
/// are checked after construction. Multiplication checks some recursive intermediates,
/// but addition, sign differences and generator relations are not a cumulative budget.
/// Offsets construct `2^shift` independently before scaling: unlike coefficient-aware
/// quadratic shifts, that factor guard cannot cancel powers already in the child.
/// No numerical refinement or cancellation receipt is provided by this query.
///
/// Unsupported syntax, invalid domains, dependent-root inversion failures and size
/// guards all return None. The local basis/memo are discarded; no radical reduction
/// is retained on source nodes. Exact rational helper caches may still be populated.
fn radical_tower_sign_of(value: &Computable) -> Option<RealSign> {
    // Traverse the immutable DAG in postorder, as the quadratic tower does.
    // Shared subexpressions reduce once.
    let key = |value: &Computable| Arc::as_ptr(&value.internal);
    let mut tower = RadicalTower {
        radicands: Vec::new(),
    };
    let mut memo = std::collections::HashMap::<*const Node, RadicalElement>::new();
    let mut pending = vec![(value, false)];
    while let Some((current, ready)) = pending.pop() {
        let current_key = key(current);
        if memo.contains_key(&current_key) {
            continue;
        }
        if memo.len() >= RADICAL_TOWER_MAX_NODES {
            return None;
        }
        if !ready {
            if let Some(rational) = current.exact_rational() {
                memo.insert(current_key, RadicalElement::Rational(rational));
                continue;
            }
            pending.push((current, true));
            match &current.internal.approximation {
                Approximation::Constant(SharedConstant::Sqrt2 | SharedConstant::Sqrt3) => {}
                // These retained trigonometric leaves have a selected exact
                // radical value. Their pi-scaled arguments are recognized
                // separately with the existing bounded scalar recognizer.
                Approximation::PrescaledSin(_) => {}
                Approximation::Negate(child)
                | Approximation::Offset(child, _)
                | Approximation::Inverse(child)
                | Approximation::Square(child)
                | Approximation::Sqrt(child)
                | Approximation::NthRoot(child, 2) => pending.push((child, false)),
                Approximation::Add(left, right) | Approximation::Multiply(left, right) => {
                    pending.push((right, false));
                    pending.push((left, false));
                }
                Approximation::LinearCombination3(combination) => {
                    pending.extend(
                        combination
                            .coefficients
                            .iter()
                            .rev()
                            .map(|child| (child, false)),
                    );
                }
                _ => return None,
            }
            continue;
        }
        let child = |value: &Computable| memo.get(&key(value)).cloned();
        let element = match &current.internal.approximation {
            Approximation::Constant(SharedConstant::Sqrt2) => {
                tower.square_root(RadicalElement::Rational(Rational::new(2)))?
            }
            Approximation::Constant(SharedConstant::Sqrt3) => {
                tower.square_root(RadicalElement::Rational(Rational::new(3)))?
            }
            Approximation::PrescaledSin(argument) => {
                let angle = argument.pi_rational_multiple()?;
                let root_sign = if angle == Rational::fraction(1, 8).expect("eight is nonzero") {
                    -1
                } else if angle == Rational::fraction(3, 8).expect("eight is nonzero") {
                    1
                } else {
                    return None;
                };
                // Both angles lie in (0, pi/2), so the half-angle identity
                // selects the principal root of (2 +/- sqrt(2))/4. This is a
                // proof projection only: the retained SinPi value is intact.
                let root = tower.square_root(RadicalElement::Rational(Rational::new(2)))?;
                let radicand = RadicalElement::Rational(Rational::new(2))
                    .add(root.scale(&Rational::from(root_sign)))
                    .scale(&Rational::fraction(1, 4).expect("four is nonzero"));
                tower.square_root(radicand)?
            }
            Approximation::Negate(value) => child(value)?.neg(),
            Approximation::Offset(value, shift) => {
                let factor = shifted_coefficient(&Rational::one(), *shift)?;
                child(value)?.scale(&factor)
            }
            Approximation::Add(left, right) => child(left)?.add(child(right)?),
            Approximation::Multiply(left, right) => tower.mul(child(left)?, child(right)?)?,
            Approximation::Inverse(value) => tower.inverse(child(value)?)?,
            Approximation::Square(value) => {
                let value = child(value)?;
                tower.mul(value.clone(), value)?
            }
            Approximation::Sqrt(value) | Approximation::NthRoot(value, 2) => {
                tower.square_root(child(value)?)?
            }
            Approximation::LinearCombination3(combination) => {
                let mut sum = RadicalElement::zero();
                for (coefficient, weight) in combination
                    .coefficients
                    .iter()
                    .zip(combination.values.iter())
                {
                    sum = sum.add(child(coefficient)?.scale(weight));
                }
                sum
            }
            _ => return None,
        };
        if !element.within_budget() {
            return None;
        }
        memo.insert(current_key, element);
    }
    let value = memo.remove(&key(value))?;
    tower.sign(&value)
}

/// Publish exact radical signs or memoize a declined query on the shared expression root.
impl Computable {
    /// Exact sign by reduction in an iterated square-root tower.
    ///
    /// A previously memoized decline returns None immediately. Otherwise build
    /// a fresh basis and retain an exact sign on success; this method does not
    /// short-circuit a successful retry from the previously retained sign.
    ///
    /// None combines unsupported syntax, invalid domains, failed dependent-root
    /// inversion and recognition guards. It does not prove mathematical
    /// nonmembership in every equivalent tower or universal undecidability.
    /// Every decline is memoized on this node, with no caller budget/policy key
    /// or retry override here. The exact value is unchanged; other sign-query
    /// paths remain available. The guards are not a total-work bound and this
    /// interface exposes no typed exhaustion/cancellation receipt.
    pub(crate) fn radical_tower_sign(&self) -> Option<RealSign> {
        // Remember a declined query on this root to avoid repeating traversal.
        // This flag does not distinguish the different reasons for declining.
        if self.internal.cache.radical_tower_declined() {
            return None;
        }
        let Some(sign) = radical_tower_sign_of(self) else {
            self.internal.cache.mark_radical_tower_declined();
            return None;
        };
        self.internal
            .facts
            .replace_exact_sign(ExactSignCache::Valid(private_sign(sign)));
        Some(sign)
    }
}
