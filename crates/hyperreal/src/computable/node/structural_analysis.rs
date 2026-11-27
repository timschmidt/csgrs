#[derive(Clone, Copy)]
/// Finite outward enclosure used only as a cheap certified filter for exact expressions.
/// Endpoints are auxiliary bounds, not authoritative scalar data. Clone/Copy
/// duplicate the endpoints; constructors validate shape rather than source inclusion.
struct Binary64Interval {
    /// Finite lower bound on the exact value, inclusive.
    lower: f64,
    /// Finite upper bound, at least lower, inclusive.
    upper: f64,
}

#[derive(Clone)]
/// Retained root recipe and an upper bound on the extension degree it introduces.
/// Clone shares the underlying expression; allocation identity is not field equality.
struct AlgebraicGenerator {
    /// Exact selected root whose presence establishes the required field extension.
    root: Computable,
    /// Positive extension-degree bound supplied by the admitted root recipe.
    degree: u32,
}

/// A bounded zero-separation certificate for `alpha = beta / gamma`.
///
/// `beta` and nonzero `gamma` are algebraic integers, and the two logarithms
/// bound the magnitudes of every conjugate of each. If `D` is the product
/// of the retained generator degrees and `alpha != 0`, the norm of the integral
/// numerator beta is a nonzero integer, so
///
/// `|alpha| >= 2^-(denominator_log2 + (D - 1) * conjugate_log2)`.
///
/// Generator degrees deliberately overestimate the actual field degree. That
/// only weakens the bound; exact rational-root dependency checks may remove a
/// generator when membership in the existing field is proved. Clone copies
/// metadata and shares retained recipes; these records do not validate their own
/// algebraic/domain invariants or supply a global computation budget.
#[derive(Clone)]
struct AlgebraicSeparation {
    // Keeping an algebraic-integer denominator avoids taking its integer norm
    // at each inverse. Both logarithms bound all conjugate magnitudes; gamma
    // need not be positive, but must be nonzero in the selected embedding.
    /// Upper bound on log2 of every conjugate magnitude of the nonzero integral denominator.
    denominator_log2: u64,
    /// Upper bound on log2 of every conjugate magnitude of the integral numerator.
    conjugate_log2: u64,
    /// Retained roots used for a conservative product-degree bound, with proved reuse where available.
    generators: Vec<AlgebraicGenerator>,
}

/// Propagate bounded algebraic-integer fraction metadata for optional zero separation.
impl AlgebraicSeparation {
    /// Maximum retained generator count for this optional norm certificate (sixteen).
    const MAX_GENERATORS: usize = 16;
    /// Maximum conservative product of extension degrees (2^20), not a mathematical degree limit.
    const MAX_FIELD_DEGREE: u64 = 1 << 20;
    /// Maximum numerator/denominator log bound and final separation exponent (2^17).
    const MAX_LOG2: u64 = 1 << 17;
    /// Maximum same-degree exponent vectors considered by the rational-root membership shortcut.
    const MAX_DEPENDENCY_COMBINATIONS: u64 = 4_096;
    /// Maximum bit length of root-membership inputs and intermediate products; not a total scratch bound.
    const MAX_DEPENDENCY_BITS: u64 = 4_096;

    /// Return the ceiling binary logarithm of a positive magnitude, with zero also mapped to zero.
    /// The zero convention supplies a harmless upper bound for a zero numerator.
    fn ceil_log2(value: &BigUint) -> u64 {
        if value <= &BigUint::one() {
            0
        } else if value.trailing_zeros() == Some(value.bits() - 1) {
            value.bits() - 1
        } else {
            value.bits()
        }
    }

    /// Describe a reduced rational as integral numerator/denominator with no generators.
    /// Use magnitude ceilings and decline if either logarithm exceeds the metadata cap.
    fn rational(value: &Rational) -> Option<Self> {
        let result = Self {
            denominator_log2: Self::ceil_log2(value.denominator()),
            conjugate_log2: Self::ceil_log2(value.numerator()),
            generators: Vec::new(),
        };
        result.within_limits().then_some(result)
    }

    /// Check logarithm, generator-count and product-degree caps for this candidate metadata.
    /// These are representation/search limits, not a bound on all arithmetic or allocation work.
    fn within_limits(&self) -> bool {
        self.denominator_log2 <= Self::MAX_LOG2
            && self.conjugate_log2 <= Self::MAX_LOG2
            && self.generators.len() <= Self::MAX_GENERATORS
            && self.field_degree().is_some()
    }

    /// Multiply retained extension-degree upper bounds with checked arithmetic and the degree cap.
    /// An empty list gives degree one; the product need not be the minimal field degree.
    fn field_degree(&self) -> Option<u64> {
        self.generators.iter().try_fold(1_u64, |degree, generator| {
            degree
                .checked_mul(u64::from(generator.degree))
                .filter(|degree| *degree <= Self::MAX_FIELD_DEGREE)
        })
    }

    /// Recognize a retained square/nth root with an exactly recognized rational radicand.
    /// Also recognize the shared sqrt(2) and sqrt(3) recipes. This does not validate a
    /// root domain or decide arbitrary algebraic membership.
    fn rational_root(root: &Computable) -> Option<(u32, Rational)> {
        match &root.internal.approximation {
            Approximation::Sqrt(child) => Some((2, child.exact_rational()?)),
            Approximation::NthRoot(child, degree) => Some((*degree, child.exact_rational()?)),
            Approximation::Constant(SharedConstant::Sqrt2) => Some((2, Rational::new(2))),
            Approximation::Constant(SharedConstant::Sqrt3) => Some((2, Rational::new(3))),
            _ => None,
        }
    }

    /// Try to prove a positive rational root belongs to the already retained same-degree root field.
    /// Enumerate exponent vectors modulo the degree and test whether the remaining
    /// rational quotient has an exact nth root. Candidate/base and intermediate-product
    /// sizes and the combination count are capped; a cap or missed shape returns false,
    /// not independence. Quotient/root-test work is not globally metered, and this
    /// helper does not poll cancellation. Callers supply a valid positive root degree.
    fn rational_root_depends_on_existing(&self, root: &Computable, degree: u32) -> bool {
        let Some((candidate_degree, candidate)) = Self::rational_root(root) else {
            return false;
        };
        if candidate_degree != degree || candidate.sign() != Sign::Plus {
            return false;
        }
        if candidate.numerator().bits() > Self::MAX_DEPENDENCY_BITS
            || candidate.denominator().bits() > Self::MAX_DEPENDENCY_BITS
        {
            return false;
        }

        let bases = self
            .generators
            .iter()
            .filter_map(|generator| {
                let (generator_degree, radicand) = Self::rational_root(&generator.root)?;
                (generator_degree == degree
                    && radicand.sign() == Sign::Plus
                    && radicand.numerator().bits() <= Self::MAX_DEPENDENCY_BITS
                    && radicand.denominator().bits() <= Self::MAX_DEPENDENCY_BITS)
                    .then_some(radicand)
            })
            .collect::<Vec<_>>();
        let combinations = bases.iter().try_fold(1_u64, |count, _| {
            count
                .checked_mul(u64::from(degree))
                .filter(|count| *count <= Self::MAX_DEPENDENCY_COMBINATIONS)
        });
        let Some(combinations) = combinations else {
            return false;
        };

        // Positive rational nth roots generate a multiplicative extension.
        // If r / product(b_i^e_i) is an exact nth power for exponents modulo n,
        // root_n(r) already belongs to the field generated by root_n(b_i).
        for mut combination in 0..combinations {
            let mut product = Rational::one();
            for base in &bases {
                let exponent = combination % u64::from(degree);
                combination /= u64::from(degree);
                for _ in 0..exponent {
                    product *= base;
                    if product.numerator().bits() > Self::MAX_DEPENDENCY_BITS
                        || product.denominator().bits() > Self::MAX_DEPENDENCY_BITS
                    {
                        return false;
                    }
                }
            }
            if (&candidate / &product).perfect_nth_root(degree).is_some() {
                return true;
            }
        }
        false
    }

    /// Reuse a structurally equal or proved dependent root; otherwise append its degree witness.
    /// Returns None when the resulting metadata exceeds a cap. The append is not
    /// rolled back on failure, so the caller must discard a failed candidate. Structural
    /// equality and dependency recognition are sufficient tests, not canonical field identity.
    fn insert_generator(&mut self, root: Computable, degree: u32) -> Option<()> {
        if self
            .generators
            .iter()
            .any(|generator| Computable::internal_structural_eq(&generator.root, &root))
            || self.rational_root_depends_on_existing(&root, degree)
        {
            return Some(());
        }
        self.generators.push(AlgebraicGenerator { root, degree });
        self.within_limits().then_some(())
    }

    /// Insert the other certificate's roots in order, retaining shared/dependent generators once.
    /// Stops on the first cap failure; the partially mutated receiver must then be discarded.
    fn merge_generators(&mut self, other: Self) -> Option<()> {
        for generator in other.generators {
            self.insert_generator(generator.root, generator.degree)?;
        }
        Some(())
    }

    /// Bound the integral numerator of a sum over the product denominator.
    /// Cross-multiply magnitude bounds, add one bit for addition, and merge generators.
    /// Checked arithmetic or any metadata cap failure declines the optional certificate.
    fn add(mut self, other: Self) -> Option<Self> {
        let conjugate_log2 = other
            .denominator_log2
            .checked_add(self.conjugate_log2)?
            .max(self.denominator_log2.checked_add(other.conjugate_log2)?)
            .checked_add(1)?;
        self.denominator_log2 = self.denominator_log2.checked_add(other.denominator_log2)?;
        self.conjugate_log2 = conjugate_log2;
        self.merge_generators(other)?;
        self.within_limits().then_some(self)
    }

    /// Add numerator/denominator logarithms and merge generators for a product.
    /// Return None on checked-arithmetic or metadata-cap failure; no value is approximated.
    fn multiply(mut self, other: Self) -> Option<Self> {
        self.denominator_log2 = self.denominator_log2.checked_add(other.denominator_log2)?;
        self.conjugate_log2 = self.conjugate_log2.checked_add(other.conjugate_log2)?;
        self.merge_generators(other)?;
        self.within_limits().then_some(self)
    }

    /// Double both integral magnitude bounds without adding field generators.
    /// Checked arithmetic and the usual metadata caps can decline the result.
    fn square(mut self) -> Option<Self> {
        self.denominator_log2 = self.denominator_log2.checked_mul(2)?;
        self.conjugate_log2 = self.conjugate_log2.checked_mul(2)?;
        self.within_limits().then_some(self)
    }

    /// Swap integral numerator and denominator magnitude bounds for an already proved nonzero value.
    /// The parser supplies that nonzero proof; this metadata method does not test it.
    /// No integer norm is materialized, and limits are checked after the swap.
    fn inverse(mut self) -> Option<Self> {
        // The parser admits an inverse only after proving its input nonzero,
        // so beta is a valid denominator for gamma / beta.
        std::mem::swap(&mut self.denominator_log2, &mut self.conjugate_log2);
        self.within_limits().then_some(self)
    }

    /// Extend metadata by a valid degree-n root while retaining its exact recipe.
    /// For alpha=beta/gamma, gamma*root_n(alpha) is integral with nth power
    /// beta*gamma^(n-1), even when the selected gamma is negative. Round the resulting
    /// conjugate-log bound upward, then insert the root generator. The caller must
    /// supply a defined real root; domain validation is not performed here. Degree zero,
    /// checked-arithmetic failure or exceeded metadata limits return None.
    fn root(mut self, root: Computable, degree: u32) -> Option<Self> {
        // If alpha = beta/gamma, then gamma * root_n(alpha) is integral:
        // its nth power is beta * gamma^(n-1). This holds even when the
        // selected gamma is negative; no principal-root choice for this
        // integral numerator is needed by the conjugate-magnitude bound.
        let degree_u64 = u64::from(degree);
        let numerator = self
            .denominator_log2
            .checked_mul(degree_u64.checked_sub(1)?)?
            .checked_add(self.conjugate_log2)?;
        self.conjugate_log2 = numerator.checked_add(degree_u64.checked_sub(1)?)? / degree_u64;
        self.insert_generator(root, degree)?;
        self.within_limits().then_some(self)
    }

    /// Account for exact multiplication by 2^shift without constructing that integer.
    /// Positive shifts enlarge the numerator bound and negative shifts the denominator;
    /// unsigned magnitude handles i32::MIN. Exceeded metadata limits return None.
    fn offset(mut self, shift: Precision) -> Option<Self> {
        if shift >= 0 {
            self.conjugate_log2 = self
                .conjugate_log2
                .checked_add(u64::from(shift.unsigned_abs()))?;
        } else {
            self.denominator_log2 = self
                .denominator_log2
                .checked_add(u64::from(shift.unsigned_abs()))?;
        }
        self.within_limits().then_some(self)
    }

    /// Return B such that a nonzero represented alpha satisfies abs(alpha) >= 2^-B.
    /// Combine the denominator bound with the numerator conjugate bound times D-1.
    /// This is conditional on valid retained metadata and is not itself a zero test;
    /// checked arithmetic or an excessive degree product returns None.
    fn separation_bits(&self) -> Option<u64> {
        self.denominator_log2.checked_add(
            self.field_degree()?
                .checked_sub(1)?
                .checked_mul(self.conjugate_log2)?,
        )
    }
}

/// Finite outward arithmetic for filters; failure declines a filter rather than changing the exact source.
impl Binary64Interval {
    /// Accept finite ordered endpoints, otherwise decline the enclosure.
    /// This checks interval shape, not that an intended exact value lies inside it.
    fn new(lower: f64, upper: f64) -> Option<Self> {
        (lower.is_finite() && upper.is_finite() && lower <= upper).then_some(Self { lower, upper })
    }

    /// Enclose a finite binary64 value as a singleton; reject infinities and NaN.
    fn exact(value: f64) -> Option<Self> {
        Self::new(value, value)
    }

    /// Construct a binary64 power of two directly from IEEE-754 exponent/fraction bits.
    /// The representable range -1074..=1023 is exact; smaller exponents return zero
    /// and larger exponents positive infinity. Those sentinels support conservative
    /// threshold tests and are not finite exact scalar conversions.
    fn power_of_two(exponent: Precision) -> f64 {
        // Construct the exact IEEE-754 value directly. This covers every
        // representable normal and subnormal power of two without relying on
        // a libm implementation or rounding mode.
        if exponent > 1_023 {
            f64::INFINITY
        } else if exponent >= -1_022 {
            f64::from_bits(u64::try_from(exponent + 1_023).unwrap() << 52)
        } else if exponent >= -1_074 {
            f64::from_bits(1_u64 << u32::try_from(exponent + 1_074).unwrap())
        } else {
            0.0
        }
    }

    /// Enclose zero exactly or a rational whose numerator and denominator are at most 2^53.
    /// Both integer conversions are then exact; expand the rounded quotient by one
    /// adjacent float on each side and restore the sign. Larger components decline,
    /// even when the rational could have a narrow binary64 enclosure by another method.
    fn from_rational(value: &Rational) -> Option<Self> {
        if value.sign() == Sign::NoSign {
            return Self::exact(0.0);
        }
        let numerator = num::ToPrimitive::to_u64(value.numerator())?;
        let denominator = num::ToPrimitive::to_u64(value.denominator())?;
        /// Largest integer whose conversion is guaranteed exact here, including 2^53.
        const MAX_EXACT_BINARY64_INTEGER: u64 = 1_u64 << 53;
        if numerator > MAX_EXACT_BINARY64_INTEGER || denominator > MAX_EXACT_BINARY64_INTEGER {
            return None;
        }
        // Both integer-to-binary64 conversions are exact. Division is the only
        // rounding operation, so one adjacent value on either side encloses
        // the exact rational without allocating big-integer scratch storage.
        let magnitude = numerator as f64 / denominator as f64;
        let lower = magnitude.next_down();
        let upper = magnitude.next_up();
        if value.sign() == Sign::Minus {
            Self::new(-upper, -lower)
        } else {
            Self::new(lower, upper)
        }
    }

    /// Reverse and negate the endpoints exactly, preserving the represented enclosure.
    fn negate(self) -> Self {
        Self {
            lower: -self.upper,
            upper: -self.lower,
        }
    }

    /// Add lower and upper endpoints and expand each rounded result outward by one float.
    /// Return None if expansion produces a nonfinite or unordered endpoint.
    fn add(self, other: Self) -> Option<Self> {
        // Basic binary64 operations are correctly rounded. Expand each result
        // by one representable value to turn the rounded endpoint operation
        // into an outward enclosure.
        Self::new(
            (self.lower + other.lower).next_down(),
            (self.upper + other.upper).next_up(),
        )
    }

    /// Enclose all four endpoint products and expand their extrema outward by one float.
    /// Dependencies between operands are not tracked. Nonfinite expanded bounds decline.
    fn multiply(self, other: Self) -> Option<Self> {
        let products = [
            self.lower * other.lower,
            self.lower * other.upper,
            self.upper * other.lower,
            self.upper * other.upper,
        ];
        let lower = products.into_iter().fold(f64::INFINITY, f64::min);
        let upper = products.into_iter().fold(f64::NEG_INFINITY, f64::max);
        Self::new(lower.next_down(), upper.next_up())
    }

    /// Enclose the reciprocal by reversing endpoint reciprocals and expanding outward.
    /// An interval containing zero or a nonfinite expanded result returns None.
    fn inverse(self) -> Option<Self> {
        if self.lower <= 0.0 && self.upper >= 0.0 {
            return None;
        }
        Self::new((1.0 / self.upper).next_down(), (1.0 / self.lower).next_up())
    }

    /// Enclose a square, using zero as the lower bound when the source crosses zero.
    /// Otherwise use interval multiplication; nonfinite expanded bounds decline.
    fn square(self) -> Option<Self> {
        if self.lower <= 0.0 && self.upper >= 0.0 {
            let magnitude = self.lower.abs().max(self.upper.abs());
            let upper = (magnitude * magnitude).next_up();
            return Self::new(0.0, upper);
        }
        self.multiply(self)
    }
}

/// Structural recognition, cache publication and optional exact sign/magnitude certificates.
impl Computable {
    /// Construct exp(r), routing exactly r=1 to the shared e recipe/cache.
    /// Other finite rationals use the ordinary exact expression constructor and its
    /// existing reduction behavior; no approximate rational substitution occurs.
    pub(crate) fn exp_rational(r: Rational) -> Self {
        if r.is_one() {
            // e^1 is hot enough to route to the shared e cache.
            Self::e_constant()
        } else {
            let rational = Self::rational(r);
            Self::exp(rational)
        }
    }

    /// Read a directly retained shared-constant tag, without looking through wrappers.
    fn shared_constant_kind(&self) -> Option<SharedConstant> {
        match &self.internal.approximation {
            Approximation::Constant(constant) => Some(*constant),
            _ => None,
        }
    }

    /// Materialize the exact rational 2^shift using an allocated integer numerator or denominator.
    /// This helper has no allocation/work cap or cancellation check. Its negative
    /// branch negates shift in i32, so i32::MIN is not supported (debug overflow);
    /// callers must constrain shifts and ensure the resulting allocation is affordable.
    fn power_of_two_rational(shift: Precision) -> Rational {
        if shift >= 0 {
            Rational::from_bigint(BigInt::one() << shift as usize)
        } else {
            Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << (-shift) as usize)
                .unwrap()
        }
    }

    /// Build optional norm-separation metadata for the admitted algebraic recipe subset.
    /// Traverse rational leaves, the shared square roots, arithmetic wrappers, roots
    /// and three-term linear forms; inverses require a proved nonzero child first.
    /// Assume all other native recipe domains are valid. A successful result B means
    /// nonzero values have magnitude at least 2^-B; it does not say this value is zero.
    ///
    /// Memoize completed nodes by live Arc identity and charge at most 256 previously
    /// unmemoized visits. Generator/degree/log bounds and the final B are capped.
    /// Nested sign/dependency tests, rational sizes and allocations are not a total
    /// work budget, and traversal does not poll cancellation. Unsupported shapes or
    /// any unavailable/capped metadata return None without a persistent failure cache.
    fn algebraic_separation_bound_bits(&self) -> Option<u64> {
        /// Maximum nonmemoized node visits in this optional separation parse, excluding nested proof work.
        const NODE_BUDGET: usize = 256;

        /// Recursively derive one candidate separation certificate with successful-node memoization.
        /// A memo hit is free; otherwise charge a visit before left-to-right child parsing.
        /// Inverse sign proofs and generator dependency tests have their own work. Store
        /// only successful metadata, and decline unsupported shapes or exceeded caps.
        fn parse(
            node: &Computable,
            remaining: &mut usize,
            memo: &mut Vec<(usize, AlgebraicSeparation)>,
        ) -> Option<AlgebraicSeparation> {
            let key = Arc::as_ptr(&node.internal) as usize;
            if let Some((_, value)) = memo.iter().find(|(candidate, _)| *candidate == key) {
                return Some(value.clone());
            }
            *remaining = remaining.checked_sub(1)?;
            let result = match &node.internal.approximation {
                Approximation::One => AlgebraicSeparation::rational(&Rational::one()),
                Approximation::Int(value) => {
                    AlgebraicSeparation::rational(&Rational::from_bigint(value.clone()))
                }
                Approximation::Ratio(value) => AlgebraicSeparation::rational(value),
                Approximation::Constant(SharedConstant::Sqrt2)
                | Approximation::Constant(SharedConstant::Sqrt3) => {
                    let radicand = match &node.internal.approximation {
                        Approximation::Constant(SharedConstant::Sqrt2) => 2,
                        Approximation::Constant(SharedConstant::Sqrt3) => 3,
                        _ => unreachable!(),
                    };
                    AlgebraicSeparation::rational(&Rational::new(radicand))?.root(node.clone(), 2)
                }
                Approximation::Negate(child) => parse(child, remaining, memo),
                Approximation::Offset(child, shift) => {
                    parse(child, remaining, memo)?.offset(*shift)
                }
                Approximation::Add(left, right) => {
                    parse(left, remaining, memo)?.add(parse(right, remaining, memo)?)
                }
                Approximation::Multiply(left, right) => {
                    parse(left, remaining, memo)?.multiply(parse(right, remaining, memo)?)
                }
                Approximation::Inverse(child)
                    if child.exact_sign().is_some_and(|sign| sign != Sign::NoSign) =>
                {
                    parse(child, remaining, memo)?.inverse()
                }
                Approximation::Square(child) => parse(child, remaining, memo)?.square(),
                Approximation::Sqrt(child) => parse(child, remaining, memo)?.root(node.clone(), 2),
                Approximation::NthRoot(child, degree) => {
                    parse(child, remaining, memo)?.root(node.clone(), *degree)
                }
                Approximation::LinearCombination3(combination) => {
                    let mut sum = AlgebraicSeparation::rational(&Rational::zero())?;
                    for (coefficient, value) in combination
                        .coefficients
                        .iter()
                        .zip(combination.values.iter())
                    {
                        sum = sum.add(
                            parse(coefficient, remaining, memo)?
                                .multiply(AlgebraicSeparation::rational(value)?)?,
                        )?;
                    }
                    Some(sum)
                }
                _ => None,
            }?;
            memo.push((key, result.clone()));
            Some(result)
        }

        let mut remaining = NODE_BUDGET;
        let metadata = parse(self, &mut remaining, &mut Vec::with_capacity(16))?;
        metadata
            .separation_bits()
            .filter(|bits| *bits <= AlgebraicSeparation::MAX_LOG2)
    }

    /// Choose precision -(B+2) when an algebraic separation bound B fits the caller floor.
    /// At that scale an approximation of magnitude at most one, with its one-unit
    /// error contract, cannot represent a nonzero value of magnitude at least 2^-B.
    /// This helper only plans that test; it neither evaluates the node nor proves zero.
    fn algebraic_zero_precision(&self, min_precision: Precision) -> Option<Precision> {
        let bound = i64::try_from(self.algebraic_separation_bound_bits()?).ok()?;
        let target = -bound.checked_add(2)?;
        (target >= i64::from(min_precision))
            .then(|| Precision::try_from(target).ok())
            .flatten()
    }

    /// Recognize an exact rational multiple of one shared constant through selected wrappers.
    /// Accept negation, binary scaling, and multiplication with an exactly recognized
    /// rational factor in either position. Do not normalize sums. Recursive traversal
    /// has no node/stop budget, and binary shifts materialize rational coefficients
    /// through power_of_two_rational with its range/allocation limitations.
    fn shared_constant_term(&self) -> Option<(SharedConstant, Rational)> {
        // Recognize "exact rational scale times one shared constant" through
        // lightweight wrappers. This supports pi-3/e-2 style sign certificates
        // without needing a full symbolic Real class.
        match &self.internal.approximation {
            Approximation::Constant(constant) => Some((*constant, Rational::one())),
            Approximation::Negate(child) => {
                let (constant, scale) = child.shared_constant_term()?;
                Some((constant, scale.neg()))
            }
            Approximation::Offset(child, shift) => {
                let (constant, scale) = child.shared_constant_term()?;
                Some((constant, scale * Self::power_of_two_rational(*shift)))
            }
            Approximation::Multiply(left, right) => {
                if let Some(scale) = left.exact_rational() {
                    let (constant, inner_scale) = right.shared_constant_term()?;
                    return Some((constant, scale * inner_scale));
                }
                if let Some(scale) = right.exact_rational() {
                    let (constant, inner_scale) = left.shared_constant_term()?;
                    return Some((constant, scale * inner_scale));
                }
                None
            }
            _ => None,
        }
    }

    /// Recognize a direct Add of integer*pi and a kernel-sized rational residual.
    /// Try the stored operand order before swapping; Tau contributes twice its scale.
    /// Residuals must be zero, have magnitude below one, or fit the rational half-pi
    /// shortcut. This is selective recipe recognition, not general period reduction.
    /// The shared-constant recognizer can materialize coefficients and has no work cap.
    fn integer_pi_plus_rational(&self) -> Option<(BigInt, Rational)> {
        // Trig reducers often see values like k*pi + r after symbolic algebra.
        // If k is an exact integer, the period/parity can be handled without
        // estimating a quotient or building a cancellation-prone residual.
        /// Try one ordered period term/residual pair, checking residual size before period extraction.
        /// Require a shared Pi/Tau term with an integral pi scale; decline other shapes.
        fn extract(term: &Computable, offset: &Computable) -> Option<(BigInt, Rational)> {
            let rational = offset.exact_rational()?;
            let residual_is_kernel_sized = rational.sign() == Sign::NoSign
                || rational.msd_exact().is_some_and(|msd| msd < 0)
                || Computable::exact_rational_half_pi_shortcut_magnitude(&rational).is_some();
            if !residual_is_kernel_sized {
                return None;
            }
            let (constant, scale) = term.shared_constant_term()?;
            let pi_scale = match constant {
                SharedConstant::Pi => scale,
                SharedConstant::Tau => scale * Rational::new(2),
                _ => return None,
            };
            pi_scale
                .to_big_integer()
                .map(|multiple| (multiple, rational))
        }

        match &self.internal.approximation {
            Approximation::Add(left, right) => {
                extract(left, right).or_else(|| extract(right, left))
            }
            _ => None,
        }
    }

    /// Derive sign and optional magnitude metadata from ordered exact rational endpoints.
    /// The caller must supply a valid enclosure. If zero is not excluded, return
    /// Unknown; otherwise equal endpoint binades certify the exponent and different
    /// binades give only an inexact planning estimate. This does not validate ordering.
    fn bound_from_strict_interval(lower: Rational, upper: Rational) -> BoundInfo {
        // Convert an interval that excludes zero into a reusable sign/MSD
        // certificate. If the interval crosses zero, preserve correctness by
        // returning Unknown.
        let zero = Rational::zero();
        let (sign, magnitude_lower, magnitude_upper) = if lower > zero {
            (Sign::Plus, lower, upper)
        } else if upper < zero {
            (Sign::Minus, upper.neg(), lower.neg())
        } else {
            return BoundInfo::Unknown;
        };

        let lower_msd = magnitude_lower.msd_exact();
        let upper_msd = magnitude_upper.msd_exact();
        let (msd, exact_msd) = match (lower_msd, upper_msd) {
            (Some(lower), Some(upper)) if lower == upper => (Some(lower), true),
            (Some(lower), Some(upper)) => (Some(lower.max(upper)), false),
            _ => (None, false),
        };

        BoundInfo::with_sign_msd(sign, msd, exact_msd)
    }

    /// Bound scale*K+q using the shared constant's exact rational enclosure.
    /// Order the scaled endpoints to handle negative scale, translate by q, then
    /// extract a sign/exponent only when the resulting interval excludes zero.
    fn constant_rational_sum_bound(
        term: &(SharedConstant, Rational),
        rational: &Rational,
    ) -> BoundInfo {
        // Specialized structural bound for c*K + q where K is a shared constant.
        // This is the computable-side companion to Real's ConstOffset class and
        // keeps generic Add nodes for pi-3 from needing approximation refinement.
        let (constant, scale) = term;
        let (lower, upper) = constant.interval();
        let scaled_lower = lower * scale;
        let scaled_upper = upper * scale;
        let (lower, upper) = if scaled_lower <= scaled_upper {
            (scaled_lower, scaled_upper)
        } else {
            (scaled_upper, scaled_lower)
        };

        Self::bound_from_strict_interval(lower + rational, upper + rational)
    }

    /// Reuse a numeric sample at this precision or a coarser one, without refinement.
    /// Direct constants consult process-wide caches (including Pi/Tau identity reuse);
    /// other expressions consult their node cache. Rescaling rounds a stored integer,
    /// not the exact real. This lookup does not poll the wrapper stop handle.
    pub(crate) fn cached_at_precision(&self, p: Precision) -> Option<BigInt> {
        // A cached value at precision q can answer any less precise request p
        // by shifting, but not a more precise one. Shared constants use the
        // process-wide cache; other nodes keep their cache beside the node.
        if let Approximation::Constant(constant) = &self.internal.approximation {
            return Self::cached_constant_at_precision(*constant, p);
        }

        self.internal.cached_at_precision(p)
    }

    /// Try the requested shared cache, then transport a Pi/Tau sample using Tau=2*Pi.
    /// A transported hit is also stored under the requested constant. Checked exponent
    /// adjustment prevents i32 overflow; lack of a sufficiently fine sample returns
    /// None. No kernel runs and no cancellation check is made.
    fn cached_constant_at_precision(constant: SharedConstant, p: Precision) -> Option<BigInt> {
        if let Some(cached) = Self::cached_shared_constant_at_precision(constant, p) {
            return Some(cached);
        }
        if constant == SharedConstant::Tau
            && let Some(pi_precision) = p.checked_sub(1)
            && let Some(cached) =
                Self::cached_shared_constant_at_precision(SharedConstant::Pi, pi_precision)
        {
            // tau is exactly 2*pi, so a pi approximation at precision p-1
            // is already a tau approximation at precision p. Populate the
            // tau cache from pi instead of re-running the Machin pi kernel
            // when callers ask for tau after pi has been warmed.
            Self::store_shared_constant_cache_value(SharedConstant::Tau, p, cached.clone());
            return Some(cached);
        }
        if constant == SharedConstant::Pi
            && let Some(tau_precision) = p.checked_add(1)
            && let Some(cached) =
                Self::cached_shared_constant_at_precision(SharedConstant::Tau, tau_precision)
        {
            // The same identity works in reverse: a tau approximation at
            // precision p+1 is already a pi approximation at precision p.
            // This matters for applications that use tau for trig
            // construction and later format pi; reuse the costly Machin
            // approximation instead of recomputing it under a different
            // shared-constant key.
            Self::store_shared_constant_cache_value(SharedConstant::Pi, p, cached.clone());
            return Some(cached);
        }
        None
    }

    /// Read/rescale the process-wide cache selected by the constant's stable cache index.
    /// A missing or insufficiently fine sample returns None; no approximation is computed.
    fn cached_shared_constant_at_precision(
        constant: SharedConstant,
        p: Precision,
    ) -> Option<BigInt> {
        SHARED_CONSTANT_CACHES[constant.cache_index()].at_precision(p)
    }

    /// Publish a caller-certified sample to the named constant's process-wide cache.
    /// The cache retains the finest sample; this raw store has no stop or validity check.
    fn store_shared_constant_cache_value(constant: SharedConstant, p: Precision, value: BigInt) {
        SHARED_CONSTANT_CACHES[constant.cache_index()].store(p, value);
    }

    /// Publish a completed one-unit approximation unless the supplied stop handle is set.
    /// First strengthen only an unresolved bound from a separated sample, then route
    /// the integer to the shared-constant or node cache. The caller must supply a valid
    /// sample at p, not an intermediate scaled value; the check does not establish
    /// sample correctness or provide a cancellation receipt. It checks the supplied
    /// handle rather than independently polling every handle in the expression.
    fn store_cache_value(&self, signal: &Option<Signal>, p: Precision, value: BigInt) {
        // Store only exact node approximation results, not temporary scaled
        // values. For shared constants this updates the process-wide cache so
        // every cloned constant wrapper and worker thread benefits.
        //
        // Abort-aware kernels may return an intentionally incomplete value.
        // Never publish one into shared state where an un-aborted clone could
        // later mistake it for a certified approximation.
        if should_stop(signal) {
            return;
        }
        let approximation_bound = Self::bound_from_approx(p, &value);
        if approximation_bound != BoundInfo::Unknown {
            // A separated approximation is a stronger certificate than an
            // absent or conservative Unknown result. Never replace an existing
            // structural certificate: it may carry an exact MSD that the
            // approximation error band cannot recover.
            self.internal
                .facts
                .set_bound_if_unresolved(BoundCache::Valid(approximation_bound));
        }
        if let Some(constant) = self.shared_constant_kind() {
            Self::store_shared_constant_cache_value(constant, p, value);
        } else {
            self.internal.store_cache_value(p, value);
        }
    }

    /// Read the bound slot: None means uninitialized, while Some(Unknown) is a recorded decline.
    fn cached_bound(&self) -> Option<BoundInfo> {
        match self.internal.facts.bound() {
            BoundCache::Invalid => None,
            BoundCache::Valid(info) => Some(info),
        }
    }

    /// Publish a structural bound, preserving an existing slot against an incoming Unknown.
    /// A definite bound uses the unconditional setter and can replace existing
    /// metadata, including a stronger magnitude certificate. Callers must justify
    /// that proof. This is separate from approximation publication's unresolved-only rule.
    fn store_bound(&self, info: &BoundInfo) {
        if *info == BoundInfo::Unknown {
            // The expression is immutable, so repeating the same conservative
            // structural walk cannot improve this result. Cache Unknown only
            // while the slot is invalid; a concurrent or later separated
            // approximation atomically upgrades it through `set_bound`.
            self.internal
                .facts
                .set_bound_if_invalid(BoundCache::Valid(BoundInfo::Unknown));
        } else {
            self.internal.facts.set_bound(BoundCache::Valid(*info));
        }
    }

    /// Extract sign and a nonexact MSD hint from a caller-certified one-unit sample.
    /// Integer magnitudes zero or one cannot separate zero and return Unknown.
    /// Larger magnitudes certify sign; an unrepresentable exponent is omitted.
    /// The sample's precision and error contract are trusted rather than revalidated.
    fn bound_from_approx(prec: Precision, appr: &BigInt) -> BoundInfo {
        // Approximation values with magnitude <= 1 are within the allowed error
        // band, so they cannot certify sign or nonzero status.
        let magnitude_bits = appr.magnitude().bits();
        if magnitude_bits <= 1 {
            BoundInfo::Unknown
        } else {
            BoundInfo::with_sign_msd(
                appr.sign(),
                crate::verified::word::checked_bit_length_msd(magnitude_bits, 1, false, prec),
                false,
            )
        }
    }

    /// Try cached metadata and a bounded-depth structural walk, publishing computed bounds.
    /// The budget counts recursive depth per branch, not total nodes; cached results
    /// remain usable at zero budget. Arithmetic wrappers visit children left to right.
    /// Direct specializations trust their native domains. Other shapes consume an
    /// existing separated sample or record Unknown, without starting approximation.
    /// None requests the iterative fallback, not a proof of invalid input.
    fn cheap_bound_shallow(&self, budget: usize) -> Option<BoundInfo> {
        // First try a shallow recursive walk. It is faster for common small
        // trees and avoids allocating the explicit stack used by deep chains.
        if let Some(info) = self.cached_bound() {
            return Some(info);
        }
        if budget == 0 {
            return None;
        }
        let info = match &self.internal.approximation {
            Approximation::One => Some(BoundInfo::with_sign(Sign::Plus, Some(0))),
            Approximation::Int(n) => Some(if n.sign() == Sign::NoSign {
                BoundInfo::Zero
            } else {
                BoundInfo::with_sign(
                    n.sign(),
                    crate::verified::word::checked_bit_length_msd(
                        n.magnitude().bits(),
                        1,
                        false,
                        0,
                    ),
                )
            }),
            Approximation::Constant(constant) => Some(constant.bound_info()),
            Approximation::Ratio(r) => Some(BoundInfo::from_rational(r)),
            Approximation::AtanRational(r) => Some(BoundInfo::with_sign_msd(r.sign(), None, false)),
            Approximation::AsinRational(r) => Some(BoundInfo::with_sign_msd(r.sign(), None, false)),
            Approximation::AsinhRational(r) | Approximation::AtanhRational(r) => {
                Some(BoundInfo::with_sign_msd(r.sign(), None, false))
            }
            Approximation::PrescaledSinRational(r) | Approximation::PrescaledTanRational(r) => {
                Some(BoundInfo::with_sign_msd(r.sign(), None, false))
            }
            Approximation::PrescaledCosRational(_)
            | Approximation::SincSmall(_)
            | Approximation::CoscSmall(_) => {
                Some(BoundInfo::with_sign_msd(Sign::Plus, None, false))
            }
            Approximation::PrescaledCosHalfPiMinusRational(_)
            | Approximation::PrescaledSinHalfPiMinusRational(_)
            | Approximation::PrescaledCotHalfPiMinusRational(_) => {
                Some(BoundInfo::with_sign_msd(Sign::Plus, None, false))
            }
            Approximation::Negate(child) => {
                child.cheap_bound_shallow(budget - 1).map(BoundInfo::negate)
            }
            Approximation::Offset(child, n) => child
                .cheap_bound_shallow(budget - 1)
                .map(|bound| bound.map_msd(|value| value.checked_add(*n))),
            Approximation::Inverse(child) => child
                .cheap_bound_shallow(budget - 1)
                .map(BoundInfo::inverse),
            Approximation::Square(child) => {
                child.cheap_bound_shallow(budget - 1).map(BoundInfo::square)
            }
            Approximation::Sqrt(child) => {
                child.cheap_bound_shallow(budget - 1).map(BoundInfo::sqrt)
            }
            Approximation::Multiply(left, right) => {
                let left = left.cheap_bound_shallow(budget - 1)?;
                let right = right.cheap_bound_shallow(budget - 1)?;
                Some(left.multiply(right))
            }
            Approximation::Add(left, right) => {
                let left = left.cheap_bound_shallow(budget - 1)?;
                let right = right.cheap_bound_shallow(budget - 1)?;
                Some(left.add(right))
            }
            _ => Some(if let Some((prec, appr)) = self.cached() {
                Self::bound_from_approx(prec, &appr)
            } else {
                BoundInfo::Unknown
            }),
        };
        if let Some(ref value) = info {
            self.store_bound(value);
        }
        info
    }

    /// Return cached bounds or try a 24-level recursive walk before an explicit-stack walk.
    /// The fallback visits arithmetic children left to right and publishes bound
    /// metadata along the way. Unsupported shapes may yield Unknown; inexact exponents
    /// are planning hints only. Valid native domains are assumed, including for zero
    /// annihilation. Heap stacks avoid deep recursion for these arithmetic wrappers,
    /// but there is no total-node, memory or cancellation bound.
    fn cheap_bound(&self) -> BoundInfo {
        /// Depth allowance for the recursive fast path before switching to an unbounded explicit stack.
        const SHALLOW_BOUND_BUDGET: usize = 24;

        // The public structural API leans on this method heavily. It must stay
        // conservative: a false NonZero or sign certificate is a correctness
        // bug, while Unknown only costs later refinement.
        if let Some(info) = self.cached_bound() {
            return info;
        }

        if let Some(bound) = self.cheap_bound_shallow(SHALLOW_BOUND_BUDGET) {
            return bound;
        }

        /// Deferred steps for the iterative bound traversal; operand values live on a parallel stack.
        enum Frame<'a> {
            /// Visit a node or reuse its completed bound query.
            Eval(
                /// Expression to inspect.
                &'a Computable,
            ),
            /// Negate the completed child bound and publish it on the parent.
            FinishNegate(
                /// Parent whose result is published after its children complete.
                &'a Computable,
            ),
            /// Transport the completed child bound through binary scaling.
            FinishOffset(
                /// Parent whose result is published after its children complete.
                &'a Computable,
                /// Exact binary shift applied to a representable magnitude exponent.
                i32,
            ),
            /// Derive reciprocal bound evidence from the completed child.
            FinishInverse(
                /// Parent whose result is published after its children complete.
                &'a Computable,
            ),
            /// Derive square bound evidence from the completed child.
            FinishSquare(
                /// Parent whose result is published after its children complete.
                &'a Computable,
            ),
            /// Derive principal-square-root bound evidence from the completed child.
            FinishSqrt(
                /// Parent whose result is published after its children complete.
                &'a Computable,
            ),
            /// Combine left/right child bound results after visiting them in that order.
            FinishAdd(
                /// Parent whose result is published after its children complete.
                &'a Computable,
            ),
            /// Combine left/right child bound results after visiting them in that order.
            FinishMultiply(
                /// Parent whose result is published after its children complete.
                &'a Computable,
            ),
        }

        /// Resolve cached/leaf/specialized bounds or consume an existing separated numeric sample.
        /// Return None only for arithmetic wrappers that need explicit-stack child traversal;
        /// Some(Unknown) is a completed unsuccessful bound query. Native domains are trusted.
        fn direct_bound(node: &Computable) -> Option<BoundInfo> {
            if let Some(bound) = node.cached_bound() {
                return Some(bound);
            }
            match &node.internal.approximation {
                Approximation::One => Some(BoundInfo::with_sign(Sign::Plus, Some(0))),
                Approximation::Int(n) => Some(if n.sign() == Sign::NoSign {
                    BoundInfo::Zero
                } else {
                    BoundInfo::with_sign(
                        n.sign(),
                        crate::verified::word::checked_bit_length_msd(
                            n.magnitude().bits(),
                            1,
                            false,
                            0,
                        ),
                    )
                }),
                Approximation::Constant(constant) => Some(constant.bound_info()),
                Approximation::Ratio(r) => Some(BoundInfo::from_rational(r)),
                Approximation::AtanRational(r) => {
                    Some(BoundInfo::with_sign_msd(r.sign(), None, false))
                }
                Approximation::AsinRational(r) => {
                    Some(BoundInfo::with_sign_msd(r.sign(), None, false))
                }
                Approximation::AsinhRational(r) | Approximation::AtanhRational(r) => {
                    Some(BoundInfo::with_sign_msd(r.sign(), None, false))
                }
                Approximation::PrescaledSinRational(r) | Approximation::PrescaledTanRational(r) => {
                    Some(BoundInfo::with_sign_msd(r.sign(), None, false))
                }
                Approximation::PrescaledCosRational(_)
                | Approximation::SincSmall(_)
                | Approximation::CoscSmall(_) => {
                    Some(BoundInfo::with_sign_msd(Sign::Plus, None, false))
                }
                Approximation::PrescaledCosHalfPiMinusRational(_)
                | Approximation::PrescaledSinHalfPiMinusRational(_)
                | Approximation::PrescaledCotHalfPiMinusRational(_) => {
                    Some(BoundInfo::with_sign_msd(Sign::Plus, None, false))
                }
                Approximation::Negate(_)
                | Approximation::Offset(_, _)
                | Approximation::Inverse(_)
                | Approximation::Square(_)
                | Approximation::Sqrt(_)
                | Approximation::Add(_, _)
                | Approximation::Multiply(_, _) => None,
                _ => Some(if let Some((prec, appr)) = node.cached() {
                    Computable::bound_from_approx(prec, &appr)
                } else {
                    BoundInfo::Unknown
                }),
            }
        }

        // Reserve small fixed-size stacks because bound queries are often called
        // on long symbolic chains and should not allocate repeatedly under
        // repeated structural fact traffic.
        let mut frames = Vec::with_capacity(16);
        let mut values: Vec<BoundInfo> = Vec::with_capacity(8);
        frames.push(Frame::Eval(self));

        // Deep addition/multiplication chains are common after algebra kernels.
        // Use an explicit stack so structural fact discovery cannot recurse
        // through thousands of nodes.
        while let Some(frame) = frames.pop() {
            match frame {
                Frame::Eval(node) => {
                    if let Some(bound) = direct_bound(node) {
                        node.store_bound(&bound);
                        values.push(bound);
                        continue;
                    }

                    match &node.internal.approximation {
                        Approximation::Negate(child) => {
                            frames.push(Frame::FinishNegate(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Offset(child, n) => {
                            frames.push(Frame::FinishOffset(node, *n));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Inverse(child) => {
                            frames.push(Frame::FinishInverse(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Square(child) => {
                            frames.push(Frame::FinishSquare(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Sqrt(child) => {
                            frames.push(Frame::FinishSqrt(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Add(left, right) => {
                            frames.push(Frame::FinishAdd(node));
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        Approximation::Multiply(left, right) => {
                            frames.push(Frame::FinishMultiply(node));
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        _ => unreachable!("direct_bound should handle non-structural nodes"),
                    }
                }
                Frame::FinishNegate(node) => {
                    let value = values.pop().expect("negate bound should exist");
                    let result = value.negate();
                    node.store_bound(&result);
                    values.push(result);
                }
                Frame::FinishOffset(node, offset) => {
                    let value = values.pop().expect("offset bound should exist");
                    let result = value.map_msd(|msd| msd.checked_add(offset));
                    node.store_bound(&result);
                    values.push(result);
                }
                Frame::FinishInverse(node) => {
                    let value = values.pop().expect("inverse bound should exist");
                    let result = value.inverse();
                    node.store_bound(&result);
                    values.push(result);
                }
                Frame::FinishSquare(node) => {
                    let value = values.pop().expect("square bound should exist");
                    let result = value.square();
                    node.store_bound(&result);
                    values.push(result);
                }
                Frame::FinishSqrt(node) => {
                    let value = values.pop().expect("sqrt bound should exist");
                    let result = value.sqrt();
                    node.store_bound(&result);
                    values.push(result);
                }
                Frame::FinishAdd(node) => {
                    let right = values.pop().expect("add rhs bound should exist");
                    let left = values.pop().expect("add lhs bound should exist");
                    let result = left.add(right);
                    node.store_bound(&result);
                    values.push(result);
                }
                Frame::FinishMultiply(node) => {
                    let right = values.pop().expect("multiply rhs bound should exist");
                    let left = values.pop().expect("multiply lhs bound should exist");
                    let result = left.multiply(right);
                    node.store_bound(&result);
                    values.push(result);
                }
            }
        }

        let result = values
            .pop()
            .expect("bound evaluation should produce a result");
        self.store_bound(&result);
        result
    }

    /// Build an outward binary64 enclosure for a selective recipe subset without warming caches.
    /// Charge one visit per recursive node, including shared nodes revisited through
    /// another edge. Accept small integers/rationals, Pi/E enclosures, arithmetic
    /// wrappers and three-term forms; binary scaling accepts only normal powers
    /// with shifts -1022..=1023. Decline missing budget, unsupported shapes, oversized
    /// rationals, zero-crossing inverses or nonfinite bounds. No cancellation polling
    /// or proof of arbitrary native recipe validity is supplied here.
    fn binary64_filter_interval(&self, remaining: &mut usize) -> Option<Binary64Interval> {
        // The filter is deliberately on-demand and bounded. It carries no
        // fields in every node and gives up on unsupported, oversized, deep,
        // non-finite, or out-of-range scale expressions; sign_until then runs
        // its unchanged arbitrary-precision path.
        *remaining = remaining.checked_sub(1)?;
        match &self.internal.approximation {
            Approximation::One => Binary64Interval::exact(1.0),
            Approximation::Int(value) => {
                if value.magnitude().bits() > 53 {
                    return None;
                }
                Binary64Interval::exact(num::ToPrimitive::to_f64(value)?)
            }
            Approximation::Ratio(value) => Binary64Interval::from_rational(value),
            // These explicit adjacent-binary64 enclosures are independently
            // checked against high-precision MPFR constants in the unit tests.
            Approximation::Constant(SharedConstant::E) => Binary64Interval::new(
                f64::from_bits(0x4005_bf0a_8b14_5768),
                f64::from_bits(0x4005_bf0a_8b14_576a),
            ),
            Approximation::Constant(SharedConstant::Pi) => Binary64Interval::new(
                f64::from_bits(0x4009_21fb_5444_2d17),
                f64::from_bits(0x4009_21fb_5444_2d19),
            ),
            Approximation::Constant(_) => None,
            Approximation::Negate(child) => {
                Some(child.binary64_filter_interval(remaining)?.negate())
            }
            Approximation::Offset(child, shift) => {
                let biased_exponent = shift.checked_add(1_023)?;
                if !(1..=2_046).contains(&biased_exponent) {
                    return None;
                }
                let scale = f64::from_bits(u64::try_from(biased_exponent).ok()? << 52);
                child
                    .binary64_filter_interval(remaining)?
                    .multiply(Binary64Interval::exact(scale)?)
            }
            Approximation::Inverse(child) => child.binary64_filter_interval(remaining)?.inverse(),
            Approximation::Square(child) => child.binary64_filter_interval(remaining)?.square(),
            Approximation::Add(left, right) => left
                .binary64_filter_interval(remaining)?
                .add(right.binary64_filter_interval(remaining)?),
            Approximation::Multiply(left, right) => left
                .binary64_filter_interval(remaining)?
                .multiply(right.binary64_filter_interval(remaining)?),
            Approximation::LinearCombination3(form) => {
                let mut result = Binary64Interval::exact(0.0)?;
                for (coefficient, value) in form.coefficients.iter().zip(&form.values) {
                    let coefficient = coefficient.binary64_filter_interval(remaining)?;
                    let value = Binary64Interval::from_rational(value)?;
                    result = result.add(coefficient.multiply(value)?)?;
                }
                Some(result)
            }
            _ => None,
        }
    }

    #[inline(never)]
    /// Try a 32-visit interval filter and require separation beyond the caller's 2^p band.
    /// A singleton zero interval proves zero; all other unseparated cases return None.
    /// Threshold overflow gives infinity and underflow gives zero, conservatively
    /// relative to representable endpoints. This helper does not publish sign facts,
    /// warm numeric caches or poll cancellation; the predicate caller owns publication.
    fn binary64_filter_sign_until(&self, min_precision: Precision) -> Option<Sign> {
        let mut remaining = 32;
        let interval = self.binary64_filter_interval(&mut remaining)?;
        // Do not let the fixed-width filter silently exceed the caller's
        // precision floor. The interval must lie beyond the same 2^p band that
        // the scaled arbitrary-precision approximation uses for a decision.
        // This preserves Unknown at p=0 for close values such as pi-103993/33102.
        let threshold = Binary64Interval::power_of_two(min_precision);
        if interval.lower > threshold {
            Some(Sign::Plus)
        } else if interval.upper < -threshold {
            Some(Sign::Minus)
        } else if interval.lower == 0.0 && interval.upper == 0.0 {
            Some(Sign::NoSign)
        } else {
            None
        }
    }

    /// Prove the sign of a retained nonzero q*pi with abs(q)<1/2.
    /// Accept one exactly recognized rational factor, binary shifts and negations,
    /// charging at most 32 inspections (including both multiplication leaves).
    /// Magnitude metadata avoids expanding shifted coefficients; checked shift overflow,
    /// zero, unsupported forms or the cutoff return None. Input rational size and
    /// nested exact recognition are not bounded by this counter. No stop polling or
    /// numeric cache warming occurs here.
    fn reduced_pi_multiple_sign(&self) -> Option<Sign> {
        let mut remaining = 32_usize;
        let mut node = self;
        let mut factor = None;
        let mut shift = 0_i32;
        let mut negative = false;
        loop {
            remaining = remaining.checked_sub(1)?;
            match &node.internal.approximation {
                Approximation::Constant(SharedConstant::Pi) => {
                    let factor = factor.unwrap_or_else(Rational::one);
                    if factor.msd_exact_with_offset(shift)?.0 >= -1 {
                        return None;
                    }
                    return Some(if negative {
                        negate_sign(factor.sign())
                    } else {
                        factor.sign()
                    });
                }
                Approximation::Negate(child) => {
                    negative = !negative;
                    node = child;
                }
                Approximation::Offset(child, offset) => {
                    shift = shift.checked_add(*offset)?;
                    node = child;
                }
                Approximation::Multiply(left, right) if factor.is_none() => {
                    // Charge both leaf inspections, including the unsuccessful
                    // one when the rational is on the right.
                    remaining = remaining.checked_sub(2)?;
                    if let Some(rational) = left.exact_rational() {
                        factor = Some(rational);
                        node = right;
                    } else {
                        factor = Some(right.exact_rational()?);
                        node = left;
                    }
                }
                _ => return None,
            }
        }
    }

    /// Prove both abs(x)<=1 and its sign without expanding outer binary shifts.
    /// Peel negations/shifts with an i64 exponent sum, then use the remaining portion
    /// of a 32-visit binary64 enclosure allowance. Poll stop handles on the peeled
    /// path; the inner interval walk has no independent polling. Zero-crossing
    /// uncertainty, excessive range, unsupported shapes or exhausted visits return
    /// None without publishing a failure. On this range sine/tangent preserve sign,
    /// cosine is positive, and cotangent preserves every nonzero argument's sign.
    fn small_trig_argument_sign(&self) -> Option<Sign> {
        let mut remaining = 32_usize;
        let mut node = self;
        let mut shift = 0_i64;
        let mut negative = false;
        loop {
            if remaining == 0 || should_stop(&node.signal) {
                return None;
            }
            match &node.internal.approximation {
                Approximation::Negate(child) => {
                    remaining -= 1;
                    negative = !negative;
                    node = child;
                }
                Approximation::Offset(child, offset) => {
                    remaining -= 1;
                    shift = shift.checked_add(i64::from(*offset))?;
                    node = child;
                }
                _ => break,
            }
        }
        let interval = node.binary64_filter_interval(&mut remaining)?;
        let magnitude = interval.lower.abs().max(interval.upper.abs());
        // All finite binary64 magnitudes are below 2^1024. Below 2^-1074,
        // only an exactly zero enclosure can pass this conservative test.
        let limit = if shift < -1_023 {
            f64::INFINITY
        } else if shift > 1_074 {
            0.0
        } else {
            Binary64Interval::power_of_two(-shift as i32)
        };
        if magnitude > limit {
            return None;
        }
        let sign = if interval.lower > 0.0 {
            Sign::Plus
        } else if interval.upper < 0.0 {
            Sign::Minus
        } else if magnitude == 0.0 {
            Sign::NoSign
        } else {
            return None;
        };
        Some(if negative { negate_sign(sign) } else { sign })
    }

    /// Try retained sign evidence, separated samples and structural sign proofs for valid recipes.
    /// At the root, try cached certification, sample sign, then the Add inverse-trig
    /// relation before honoring a cached Unknown. The explicit-stack arithmetic walk
    /// visits left before right, publishes results, and tries a quadratic projection
    /// if the final result is unresolved. Direct function cases may recursively query
    /// children. It does not validate all native domains: annihilating zeros and
    /// positive-function shortcuts assume defined operands and admitted kernel ranges.
    ///
    /// Some(NoSign) certifies zero; None records an unresolved route, not mathematical
    /// undecidability. A later separated sample can strengthen Unknown. No numeric
    /// refinement is initiated here, but recognizers can do exact algebra and mutate
    /// proof caches. There is no blanket work/depth bound or cancellation polling.
    fn exact_sign(&self) -> Option<Sign> {
        // `exact_sign` is stronger than "current approximation sign": it means
        // the expression shape or a separated cached approximation proves the
        // sign. Unknown is cached separately so impossible structural proofs do
        // not repeat on every predicate query.
        let cached_sign = self.internal.facts.exact_sign();
        if let ExactSignCache::Valid(sign) = cached_sign {
            return Some(sign);
        }
        if let Some((_, appr)) = self.cached()
            && appr.abs() > BigInt::one()
        {
            let sign = appr.sign();
            self.internal
                .facts
                .replace_exact_sign(ExactSignCache::Valid(sign));
            return Some(sign);
        }
        if matches!(self.internal.approximation, Approximation::Add(_, _))
            && let Some(sign) = self.inverse_trig_linear_sign()
        {
            self.internal
                .facts
                .replace_exact_sign(ExactSignCache::Valid(sign));
            return Some(sign);
        }
        if cached_sign == ExactSignCache::Unknown {
            return None;
        }
        /// Deferred steps for the iterative sign traversal; operand values live on a parallel stack.
        enum Frame<'a> {
            /// Visit a node or reuse its completed sign query.
            Eval(
                /// Expression to inspect.
                &'a Computable,
            ),
            /// Negate the completed child sign and publish it on the parent.
            FinishNegate(
                /// Parent whose result is published after its children complete.
                &'a Computable,
            ),
            /// Transport the completed child sign through binary scaling.
            FinishOffset(
                /// Parent whose result is published after its children complete.
                &'a Computable,
            ),
            /// Derive reciprocal sign evidence from the completed child.
            FinishInverse(
                /// Parent whose result is published after its children complete.
                &'a Computable,
            ),
            /// Derive square sign evidence from the completed child.
            FinishSquare(
                /// Parent whose result is published after its children complete.
                &'a Computable,
            ),
            /// Derive principal-square-root sign evidence from the completed child.
            FinishSqrt(
                /// Parent whose result is published after its children complete.
                &'a Computable,
            ),
            /// Combine left/right child sign results after visiting them in that order.
            FinishAdd(
                /// Parent whose result is published after its children complete.
                &'a Computable,
            ),
            /// Combine left/right child sign results after visiting them in that order.
            FinishMultiply(
                /// Parent whose result is published after its children complete.
                &'a Computable,
            ),
        }

        /// Read a nested sign slot, strengthening a recorded Unknown from a separated numeric sample.
        /// None requests structural traversal; Some(None) is a recorded unresolved query;
        /// Some(Some(sign)) is usable evidence. Uninitialized slots are handled by the caller.
        fn cached_exact_sign(node: &Computable) -> Option<Option<Sign>> {
            let cached_sign = node.internal.facts.exact_sign();
            match cached_sign {
                ExactSignCache::Invalid => None,
                ExactSignCache::Unknown => {
                    if let Some((_, appr)) = node.cached()
                        && appr.abs() > BigInt::one()
                    {
                        let sign = appr.sign();
                        node.internal
                            .facts
                            .replace_exact_sign(ExactSignCache::Valid(sign));
                        Some(Some(sign))
                    } else {
                        Some(None)
                    }
                }
                ExactSignCache::Valid(sign) => Some(Some(sign)),
            }
        }

        /// Resolve cached/sample/direct function signs or request arithmetic child traversal.
        /// Outer None selects an explicit-stack wrapper; Some(None) is an unresolved
        /// leaf or specialization. Several direct cases recursively ask child signs or
        /// exact recognizers; this is not a domain validator or a recursion/work bound.
        fn exact_sign_direct(node: &Computable) -> Option<Option<Sign>> {
            // Direct cases either know their sign structurally or are known not
            // to be structurally decidable without visiting children.
            if let Some(sign) = cached_exact_sign(node) {
                return Some(sign);
            }

            if let Some((_, appr)) = node.cached()
                && appr.abs() > BigInt::one()
            {
                return Some(Some(appr.sign()));
            }

            match &node.internal.approximation {
                Approximation::One => Some(Some(Sign::Plus)),
                Approximation::Int(n) => Some(Some(n.sign())),
                Approximation::Constant(_) => Some(Some(Sign::Plus)),
                Approximation::Ratio(r) => Some(Some(r.sign())),
                Approximation::IntegralAtan(n) => Some(Some(n.sign())),
                Approximation::AtanRational(r) => Some(Some(r.sign())),
                Approximation::AsinRational(r) => Some(Some(r.sign())),
                Approximation::AsinhRational(r) | Approximation::AtanhRational(r) => {
                    Some(Some(r.sign()))
                }
                Approximation::PrescaledSinRational(r) | Approximation::PrescaledTanRational(r) => {
                    Some(Some(r.sign()))
                }
                // Sine and tangent share the argument's sign on (-pi/2, pi/2).
                // Recover the witness from the graph after lazy reconstruction
                // or import; serialized class tags and cache hints are not proof.
                Approximation::PrescaledSin(argument) | Approximation::PrescaledTan(argument) => {
                    Some(
                        argument
                            .reduced_pi_multiple_sign()
                            .or_else(|| argument.small_trig_argument_sign()),
                    )
                }
                Approximation::PrescaledCos(argument) => {
                    Some(argument.small_trig_argument_sign().map(|_| Sign::Plus))
                }
                Approximation::PrescaledCot(argument) => Some(
                    argument
                        .small_trig_argument_sign()
                        .filter(|sign| *sign != Sign::NoSign),
                ),
                Approximation::PrescaledCosRational(_)
                | Approximation::SincSmall(_)
                | Approximation::CoscSmall(_) => Some(Some(Sign::Plus)),
                Approximation::PrescaledCosHalfPiMinusRational(_)
                | Approximation::PrescaledSinHalfPiMinusRational(_)
                | Approximation::PrescaledCotHalfPiMinusRational(_) => Some(Some(Sign::Plus)),
                Approximation::AcosPositiveRational(r) => {
                    Some(Some(if r.is_one() { Sign::NoSign } else { Sign::Plus }))
                }
                Approximation::NormalQuantile(_) => Some(node.exact_rational().map(|r| r.sign())),
                Approximation::AcoshNearOne(argument) | Approximation::AcoshDirect(argument) => {
                    // The representation alone does not prove x>1. A checked
                    // rational expression proves the endpoint and its domain;
                    // a cached sign alone could hide an undefined child.
                    Some(argument.bounded_defined_rational().and_then(|r| {
                        if r.is_one() {
                            Some(Sign::NoSign)
                        } else if r > Rational::one() {
                            Some(Sign::Plus)
                        } else {
                            None
                        }
                    }))
                }
                Approximation::AcosNegativeRational(_)
                | Approximation::Erfc(_)
                | Approximation::NormalSf(_) => Some(Some(Sign::Plus)),
                Approximation::AcosPositive(_) | Approximation::NormalInterval { .. } => Some(None),
                Approximation::LogPnorm(_)
                | Approximation::LogNormalSf(_)
                | Approximation::LogDnorm(_) => Some(Some(Sign::Minus)),
                Approximation::PrescaledAtan(child)
                | Approximation::AtanDeferred(child)
                | Approximation::PrescaledAsin(child)
                | Approximation::AsinDeferred(child)
                | Approximation::AsinhNearZero(child)
                | Approximation::AsinhDirect(child)
                | Approximation::PrescaledAsinh(child)
                | Approximation::AtanhDirect(child)
                | Approximation::PrescaledAtanh(child)
                | Approximation::InverseIsometricLatitude { psi: child, .. }
                | Approximation::Expm1(child) => Some(child.exact_sign()),
                Approximation::EllipticF(arguments)
                | Approximation::EllipticE(arguments)
                | Approximation::JacobiAmplitude(arguments)
                | Approximation::InverseEllipticE(arguments) => Some(arguments[0].exact_sign()),
                Approximation::PrescaledExp(_)
                | Approximation::EllipticRf(_)
                | Approximation::JacobiDn(_)
                | Approximation::EllipticRd(_) => Some(Some(Sign::Plus)),
                Approximation::Negate(_)
                | Approximation::Offset(_, _)
                | Approximation::Inverse(_)
                | Approximation::Square(_)
                | Approximation::Sqrt(_)
                | Approximation::Add(_, _)
                | Approximation::Multiply(_, _) => None,
                _ => Some(None),
            }
        }

        /// Publish Valid(sign) or Unknown through the monotone exact-sign cache setter.
        /// An existing valid sign survives Unknown; the caller must justify every certificate.
        fn store_exact_sign(node: &Computable, sign: Option<Sign>) {
            node.internal.facts.replace_exact_sign(match sign {
                Some(sign) => ExactSignCache::Valid(sign),
                None => ExactSignCache::Unknown,
            });
        }

        // Structural sign on deep chains stays allocation-light so predicate-heavy
        // code does not needlessly allocate during exact-sign walks.
        let mut frames = Vec::with_capacity(16);
        let mut values: Vec<Option<Sign>> = Vec::with_capacity(8);
        frames.push(Frame::Eval(self));

        // Mirror cheap_bound's nonrecursive traversal for deep structural
        // expressions. This matters for predicate-heavy code that asks only for
        // sign and never needs numeric approximation.
        while let Some(frame) = frames.pop() {
            match frame {
                Frame::Eval(node) => {
                    if let Some(sign) = exact_sign_direct(node) {
                        store_exact_sign(node, sign);
                        values.push(sign);
                        continue;
                    }

                    match &node.internal.approximation {
                        Approximation::Negate(child) => {
                            frames.push(Frame::FinishNegate(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Offset(child, _) => {
                            frames.push(Frame::FinishOffset(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Inverse(child) => {
                            frames.push(Frame::FinishInverse(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Square(child) => {
                            frames.push(Frame::FinishSquare(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Sqrt(child) => {
                            frames.push(Frame::FinishSqrt(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Add(left, right) => {
                            frames.push(Frame::FinishAdd(node));
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        Approximation::Multiply(left, right) => {
                            frames.push(Frame::FinishMultiply(node));
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        _ => unreachable!("exact_sign_direct should handle non-structural nodes"),
                    }
                }
                Frame::FinishNegate(node) => {
                    let value = values.pop().expect("negate sign should exist");
                    let result = value.map(negate_sign);
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishOffset(node) => {
                    let value = values.pop().expect("offset sign should exist");
                    store_exact_sign(node, value);
                    values.push(value);
                }
                Frame::FinishInverse(node) => {
                    let value = values.pop().expect("inverse sign should exist");
                    let result = match value {
                        Some(Sign::Plus) => Some(Sign::Plus),
                        Some(Sign::Minus) => Some(Sign::Minus),
                        _ => None,
                    };
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishSquare(node) => {
                    let value = values.pop().expect("square sign should exist");
                    let result = match value {
                        Some(Sign::NoSign) => Some(Sign::NoSign),
                        Some(_) => Some(Sign::Plus),
                        None => None,
                    };
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishSqrt(node) => {
                    let value = values.pop().expect("sqrt sign should exist");
                    let result = match value {
                        Some(Sign::NoSign) => Some(Sign::NoSign),
                        Some(Sign::Plus) => Some(Sign::Plus),
                        _ => None,
                    };
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishAdd(node) => {
                    let right = values.pop().expect("add rhs sign should exist");
                    let left = values.pop().expect("add lhs sign should exist");
                    let result = match (left, right) {
                        (Some(Sign::NoSign), sign) | (sign, Some(Sign::NoSign)) => sign,
                        (Some(Sign::Plus), Some(Sign::Plus)) => Some(Sign::Plus),
                        (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Minus),
                        _ => None,
                    }
                    .or_else(|| node.exact_positive_exp_difference_sign());
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishMultiply(node) => {
                    let right = values.pop().expect("multiply rhs sign should exist");
                    let left = values.pop().expect("multiply lhs sign should exist");
                    let result = match (left, right) {
                        (Some(Sign::NoSign), _) | (_, Some(Sign::NoSign)) => Some(Sign::NoSign),
                        (Some(Sign::Plus), Some(Sign::Plus))
                        | (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Plus),
                        (Some(Sign::Plus), Some(Sign::Minus))
                        | (Some(Sign::Minus), Some(Sign::Plus)) => Some(Sign::Minus),
                        _ => None,
                    };
                    store_exact_sign(node, result);
                    values.push(result);
                }
            }
        }

        let mut result = values
            .pop()
            .expect("exact sign evaluation should produce a result");
        if result.is_none() {
            result = self.exact_quadratic_surd_sign();
        }
        store_exact_sign(self, result);
        result
    }

    /// Decide the sign of a recognized rational-plus-one-radical normal form exactly.
    /// After trivial zero/equal-sign cases, compare rational squares to decide opposite
    /// terms without approximating the root. Unsupported projections return None;
    /// the normal form supplies the positive-radicand and definedness invariants.
    fn exact_quadratic_surd_sign(&self) -> Option<Sign> {
        let (rational, scale, radicand) = self.exact_quadratic_surd_parts()?;
        if scale.sign() == Sign::NoSign {
            return Some(rational.sign());
        }
        if rational.sign() == Sign::NoSign || rational.sign() == scale.sign() {
            return Some(scale.sign());
        }
        let rational_square = &rational * &rational;
        let radical_square = &scale * &scale * radicand?;
        Some(
            match rational_square
                .partial_cmp(&radical_square)
                .expect("finite exact rationals are totally ordered")
            {
                Ordering::Less => scale.sign(),
                Ordering::Equal => Sign::NoSign,
                Ordering::Greater => rational.sign(),
            },
        )
    }

    /// Project scale*sqrt(radicand) only when the normalized rational part is zero.
    /// Return the retained rational scale and radicand. Pure rational/zero projections
    /// without a radical decline; no numerical fitting or approximate detection occurs.
    pub(crate) fn exact_pure_quadratic_surd(&self) -> Option<(Rational, Rational)> {
        let (rational, scale, radicand) = self.exact_quadratic_surd_parts()?;
        if rational.sign() != Sign::NoSign {
            return None;
        }
        Some((scale, radicand?))
    }

    /// Project a rational or rational-plus-one-radical value after bounded Laurent normalization.
    /// Request the shared normal form with depth allowance 64, then require only the degree-zero
    /// term, no atoms and at most one radical. Cancellation may remove intermediate
    /// radicals. Return (rational, scale, optional radicand), emitting the quadratic-surd
    /// dispatch trace only on success. None means this projection declined; its local
    /// budget is not a general bound on all nested arithmetic or allocations.
    pub(crate) fn exact_quadratic_surd_parts(
        &self,
    ) -> Option<(Rational, Rational, Option<Rational>)> {
        // Project the shared bounded normal form after cancellation. Requiring
        // every intermediate expression to use one radical would miss exact
        // quadratic values assembled from independent geometric directions.
        let polynomial = self.pi_laurent_polynomial(64)?;
        let value = match polynomial.terms.as_slice() {
            [] => (Rational::zero(), Rational::zero(), None),
            [(0, coefficient)] if coefficient.atoms.is_empty() => {
                match coefficient.radicals.as_slice() {
                    [] => (coefficient.rational.clone(), Rational::zero(), None),
                    [(radicand, scale)] => (
                        coefficient.rational.clone(),
                        scale.clone(),
                        Some(radicand.clone()),
                    ),
                    _ => return None,
                }
            }
            _ => return None,
        };
        crate::trace_dispatch!("computable", "structural", "quadratic-surd");
        Some(value)
    }

    #[cfg(test)]
    /// Expose the bound's optional planning exponent to tests, including inexact hints.
    /// The outer None is unavailable magnitude; Some(None) denotes proved zero.
    pub(super) fn planning_msd(&self) -> Option<Option<Precision>> {
        self.cheap_bound().planning_msd()
    }

    /// Return known sign and optional planning exponent from structural bound discovery.
    /// A present exponent may be inexact; magnitude None is unavailable and Some(None)
    /// is proved zero. Queries can populate bound caches and must not be treated as
    /// error certificates merely because an exponent was returned.
    pub(crate) fn planning_sign_and_msd(&self) -> (Option<Sign>, Option<Option<Precision>>) {
        let bound = self.cheap_bound();
        (bound.known_sign(), bound.planning_msd())
    }

    /// Return known sign and only an exact magnitude certificate from bound discovery.
    /// Inexact planning hints are withheld; magnitude Some(None) denotes proved zero.
    /// This does not refine numerically when a certificate is missing.
    pub(crate) fn certified_sign_and_msd(&self) -> (Option<Sign>, Option<Option<Precision>>) {
        // Inexact hints can drift through sums or become severe underestimates
        // after cancellation and inversion. Error-sensitive precision choices
        // must use a certificate or obtain a fresh approximation instead.
        self.cheap_bound().certified_sign_and_msd()
    }

    /// Recover a rational inverse from exact one-half or a fully matching retained CDF graph.
    /// First recognize rational 1/2 as zero. Otherwise the Offset/Add/Multiply/
    /// ErfSeries shape suggests an erf argument; a pure quadratic projection and
    /// a perfect square root of twice its radicand suggest a rational inverse.
    /// Reconstruct the entire native CDF and require structural equality before
    /// accepting it. The embedded argument alone is insufficient, as a changed
    /// additive constant demonstrates. No numerical tail probability is evaluated.
    /// None means this optional recognition declined, not an invalid probability
    /// or a proof that its inverse is irrational. This helper assumes defined
    /// children and adds no total-work, memory or cancellation contract.
    pub(crate) fn exact_normal_quantile(&self) -> Option<Rational> {
        if self.exact_rational().as_ref() == Some(&*HALF_RATIONAL) {
            return Some(Rational::zero());
        }
        // Native CDF construction retains erf's argument. It suggests an
        // inverse, but only reconstructing the complete CDF proves the identity:
        // a similar-looking expression or an untrusted seed is not a witness.
        let Approximation::Offset(sum, -1) = &self.internal.approximation else {
            return None;
        };
        let Approximation::Add(_, erf) = &sum.internal.approximation else {
            return None;
        };
        let Approximation::Multiply(_, series) = &erf.internal.approximation else {
            return None;
        };
        let Approximation::ErfSeries(argument) = &series.internal.approximation else {
            return None;
        };
        let (scale, radicand) = argument.exact_pure_quadratic_surd()?;
        let inverse = scale * (radicand * Rational::new(2)).perfect_nth_root(2)?;
        let reconstructed = Self::rational(inverse.clone()).pnorm();
        Self::internal_structural_eq(self, &reconstructed).then_some(inverse)
    }

    /// Recognize exact leaves and selected defined inverse-cosine, acosh and quantile identities.
    /// Accept One/Int/Ratio, acos(1), acosh of a bounded-defined rational one, and a
    /// fully checked retained rational normal-quantile inverse. Do not promote a
    /// cached zero or arbitrary arithmetic expression to rational form; unresolved
    /// child domains may remain beneath cached signs. None is a selective recognition
    /// decline, not irrationality. Nested recognizers have their own limits.
    pub(crate) fn exact_rational(&self) -> Option<Rational> {
        // Read exact leaves and defined inverse-cosine/quantile identities. Do not turn
        // an arbitrary cached zero into a rational: imported graphs may still
        // contain unresolved domain obligations beneath that sign fact.
        match &self.internal.approximation {
            Approximation::One => Some(Rational::one()),
            Approximation::Int(n) => Some(Rational::from_bigint(n.clone())),
            Approximation::Ratio(r) => Some(r.clone()),
            Approximation::AcosPositiveRational(r) if r.is_one() => Some(Rational::zero()),
            Approximation::NormalQuantile(data) => data.p.exact_normal_quantile(),
            Approximation::AcoshNearOne(argument) | Approximation::AcoshDirect(argument) => {
                argument
                    .bounded_defined_rational()?
                    .is_one()
                    .then(Rational::zero)
            }
            _ => None,
        }
    }
}
