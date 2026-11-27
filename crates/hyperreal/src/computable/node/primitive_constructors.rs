/// Leaf constructors, optional structural recognition and deferred elementary recipes.
///
/// Raw helpers retain exact definitions; their tags do not establish real-domain
/// validity or a global resource budget. Unless stated otherwise below, a new
/// wrapper has no outer cancellation signal even when a stored child has one;
/// adapters must arrange the intended evaluation control explicitly.
impl Computable {
    /// Construct an exact one-half leaf from the shared rational payload.
    /// Log-ratio reductions reuse the rational value; each call still constructs
    /// its own `Computable` wrapper through `rational`, with no attached signal.
    #[inline]
    fn half() -> Self {
        Self::rational(HALF_RATIONAL.clone())
    }

    /// Recognize matching immutable expression structure without numerical refinement.
    ///
    /// Shared node allocation proves a match immediately. Otherwise compare the
    /// handled tags, exact payloads and ordered children, ignoring caches and
    /// wrapper signals. This is a sufficient recognition test on valid graphs,
    /// not value inequality when false: algebraically equivalent forms can differ,
    /// quantile seeds must match, and independently allocated `LinearCombination3`
    /// nodes have no matching arm. No domain validity is established here.
    ///
    /// Try at most 64 non-pointer-shortcut comparisons recursively, then restart
    /// with an iterative stack and a set of visited node-identity pairs. The limit
    /// bounds only the initial walk, not total work, coefficient comparison cost
    /// or retained memory. Neither phase polls cancellation or returns exhaustion.
    pub(crate) fn internal_structural_eq(left: &Self, right: &Self) -> bool {
        /// Result of comparing one pair of tags and non-child payloads.
        /// Child obligations retain source order without allocating a work vector.
        enum NodeComparison<'a> {
            /// A terminal leaf match or an unrecognized/mismatched form.
            Decided(
                /// True proves the handled structure matches; false is not value inequality.
                bool,
            ),
            /// One child obligation, also used for duplicated binary edges.
            One(
                /// Left and right children to compare next.
                (&'a Computable, &'a Computable),
            ),
            /// Two ordered obligations for binary expressions or paired payloads.
            Two(
                /// First child pair, visited before the second.
                (&'a Computable, &'a Computable),
                /// Second pair, required only after the first matches.
                (&'a Computable, &'a Computable),
            ),
            /// Three ordered obligations for a Carlson-integral payload.
            Three(
                /// Corresponding children in their retained payload order.
                [(&'a Computable, &'a Computable); 3],
            ),
        }

        /// Compare matching tag payloads and emit remaining ordered child obligations.
        /// Quantile hints participate; LinearCombination3 has no structural arm.
        /// This helper performs neither recursion nor a pointer-identity shortcut.
        #[inline(always)]
        fn compare_nodes<'a>(left: &'a Computable, right: &'a Computable) -> NodeComparison<'a> {
            match (&left.internal.approximation, &right.internal.approximation) {
                (Approximation::One, Approximation::One) => NodeComparison::Decided(true),
                (Approximation::Int(left), Approximation::Int(right))
                | (Approximation::IntegralAtan(left), Approximation::IntegralAtan(right)) => {
                    NodeComparison::Decided(left == right)
                }
                (Approximation::Constant(left), Approximation::Constant(right)) => {
                    NodeComparison::Decided(left == right)
                }
                (Approximation::Inverse(left), Approximation::Inverse(right))
                | (Approximation::Negate(left), Approximation::Negate(right))
                | (Approximation::Square(left), Approximation::Square(right))
                | (Approximation::PrescaledExp(left), Approximation::PrescaledExp(right))
                | (Approximation::Expm1(left), Approximation::Expm1(right))
                | (Approximation::Sqrt(left), Approximation::Sqrt(right))
                | (Approximation::PrescaledLn(left), Approximation::PrescaledLn(right))
                | (Approximation::PrescaledAtan(left), Approximation::PrescaledAtan(right))
                | (Approximation::AtanDeferred(left), Approximation::AtanDeferred(right))
                | (Approximation::PrescaledAsin(left), Approximation::PrescaledAsin(right))
                | (Approximation::AsinDeferred(left), Approximation::AsinDeferred(right))
                | (Approximation::AcosPositive(left), Approximation::AcosPositive(right))
                | (Approximation::AcoshNearOne(left), Approximation::AcoshNearOne(right))
                | (Approximation::AcoshDirect(left), Approximation::AcoshDirect(right))
                | (Approximation::AsinhNearZero(left), Approximation::AsinhNearZero(right))
                | (Approximation::AsinhDirect(left), Approximation::AsinhDirect(right))
                | (Approximation::PrescaledAsinh(left), Approximation::PrescaledAsinh(right))
                | (Approximation::AtanhDirect(left), Approximation::AtanhDirect(right))
                | (Approximation::PrescaledAtanh(left), Approximation::PrescaledAtanh(right))
                | (Approximation::PrescaledCos(left), Approximation::PrescaledCos(right))
                | (Approximation::PrescaledSin(left), Approximation::PrescaledSin(right))
                | (Approximation::PrescaledTan(left), Approximation::PrescaledTan(right))
                | (Approximation::PrescaledCot(left), Approximation::PrescaledCot(right))
                | (Approximation::ErfSeries(left), Approximation::ErfSeries(right))
                | (Approximation::Erfc(left), Approximation::Erfc(right))
                | (Approximation::NormalSf(left), Approximation::NormalSf(right))
                | (Approximation::LogPnorm(left), Approximation::LogPnorm(right))
                | (Approximation::LogNormalSf(left), Approximation::LogNormalSf(right))
                | (Approximation::LogDnorm(left), Approximation::LogDnorm(right))
                | (Approximation::SincSmall(left), Approximation::SincSmall(right))
                | (Approximation::CoscSmall(left), Approximation::CoscSmall(right)) => {
                    NodeComparison::One((left, right))
                }
                (Approximation::Add(left, right), Approximation::Add(left_rhs, right_rhs))
                | (
                    Approximation::Multiply(left, right),
                    Approximation::Multiply(left_rhs, right_rhs),
                ) => {
                    if Arc::ptr_eq(&left.internal, &right.internal)
                        && Arc::ptr_eq(&left_rhs.internal, &right_rhs.internal)
                    {
                        NodeComparison::One((left, left_rhs))
                    } else {
                        NodeComparison::Two((left, left_rhs), (right, right_rhs))
                    }
                }
                (Approximation::Ratio(left), Approximation::Ratio(right))
                | (
                    Approximation::PrescaledLnRational(left),
                    Approximation::PrescaledLnRational(right),
                )
                | (Approximation::AtanRational(left), Approximation::AtanRational(right))
                | (Approximation::AsinRational(left), Approximation::AsinRational(right))
                | (
                    Approximation::AcosPositiveRational(left),
                    Approximation::AcosPositiveRational(right),
                )
                | (
                    Approximation::AcosNegativeRational(left),
                    Approximation::AcosNegativeRational(right),
                )
                | (Approximation::AsinhRational(left), Approximation::AsinhRational(right))
                | (Approximation::AtanhRational(left), Approximation::AtanhRational(right))
                | (
                    Approximation::PrescaledCosRational(left),
                    Approximation::PrescaledCosRational(right),
                )
                | (Approximation::CosLargeRational(left), Approximation::CosLargeRational(right))
                | (
                    Approximation::PrescaledCosHalfPiMinusRational(left),
                    Approximation::PrescaledCosHalfPiMinusRational(right),
                )
                | (
                    Approximation::PrescaledSinRational(left),
                    Approximation::PrescaledSinRational(right),
                )
                | (Approximation::SinLargeRational(left), Approximation::SinLargeRational(right))
                | (
                    Approximation::PrescaledSinHalfPiMinusRational(left),
                    Approximation::PrescaledSinHalfPiMinusRational(right),
                )
                | (
                    Approximation::PrescaledCotHalfPiMinusRational(left),
                    Approximation::PrescaledCotHalfPiMinusRational(right),
                )
                | (Approximation::TanLargeRational(left), Approximation::TanLargeRational(right))
                | (
                    Approximation::PrescaledTanRational(left),
                    Approximation::PrescaledTanRational(right),
                ) => NodeComparison::Decided(left == right),
                (
                    Approximation::Offset(left, left_shift),
                    Approximation::Offset(right, right_shift),
                ) if left_shift == right_shift => NodeComparison::One((left, right)),
                (
                    Approximation::NthRoot(left, left_degree),
                    Approximation::NthRoot(right, right_degree),
                ) if left_degree == right_degree => NodeComparison::One((left, right)),
                (
                    Approximation::BinaryScaledLnRational {
                        residual: left_residual,
                        shift: left_shift,
                    },
                    Approximation::BinaryScaledLnRational {
                        residual: right_residual,
                        shift: right_shift,
                    },
                ) => NodeComparison::Decided(
                    left_residual == right_residual && left_shift == right_shift,
                ),
                (
                    Approximation::NormalInterval {
                        lo: left_lo,
                        hi: left_hi,
                    },
                    Approximation::NormalInterval {
                        lo: right_lo,
                        hi: right_hi,
                    },
                ) => NodeComparison::Two((left_lo, right_lo), (left_hi, right_hi)),
                (Approximation::NormalQuantile(left), Approximation::NormalQuantile(right))
                    if left.seed == right.seed && left.seed_prec == right.seed_prec =>
                {
                    NodeComparison::One((&left.p, &right.p))
                }
                (
                    Approximation::InverseIsometricLatitude {
                        psi: l,
                        log_axis_ratio: lr,
                    },
                    Approximation::InverseIsometricLatitude {
                        psi: r,
                        log_axis_ratio: rr,
                    },
                ) => NodeComparison::Two((l, r), (lr, rr)),
                (Approximation::EllipticRf(left), Approximation::EllipticRf(right))
                | (Approximation::EllipticRd(left), Approximation::EllipticRd(right)) => {
                    NodeComparison::Three(std::array::from_fn(|i| (&left[i], &right[i])))
                }
                (Approximation::EllipticF(left), Approximation::EllipticF(right))
                | (Approximation::EllipticE(left), Approximation::EllipticE(right))
                | (Approximation::JacobiAmplitude(left), Approximation::JacobiAmplitude(right))
                | (Approximation::JacobiSn(left), Approximation::JacobiSn(right))
                | (Approximation::JacobiCn(left), Approximation::JacobiCn(right))
                | (Approximation::JacobiDn(left), Approximation::JacobiDn(right))
                | (Approximation::InverseEllipticE(left), Approximation::InverseEllipticE(right)) => {
                    NodeComparison::Two((&left[0], &right[0]), (&left[1], &right[1]))
                }
                _ => NodeComparison::Decided(false),
            }
        }

        /// Walk the initial comparison without a heap worklist, charging non-pointer pairs.
        /// Shared allocations succeed before checking the remaining visit allowance.
        /// Return None on depletion so the caller can restart iteratively; a false
        /// result is only a failed recognition. Child order and early returns matter.
        fn compare_bounded(
            left: &Computable,
            right: &Computable,
            remaining: &mut usize,
        ) -> Option<bool> {
            if Arc::ptr_eq(&left.internal, &right.internal) {
                return Some(true);
            }
            if *remaining == 0 {
                return None;
            }
            *remaining -= 1;
            match compare_nodes(left, right) {
                NodeComparison::Decided(equal) => Some(equal),
                NodeComparison::One((left, right)) => compare_bounded(left, right, remaining),
                NodeComparison::Three(pairs) => {
                    for (left, right) in pairs {
                        match compare_bounded(left, right, remaining) {
                            Some(true) => {}
                            other => return other,
                        }
                    }
                    Some(true)
                }
                NodeComparison::Two((left, left_rhs), (right, right_rhs)) => {
                    match compare_bounded(left, left_rhs, remaining) {
                        Some(true) => compare_bounded(right, right_rhs, remaining),
                        Some(false) => Some(false),
                        None => None,
                    }
                }
            }
        }

        // Preserve the allocation-free path for ordinary shallow comparisons.
        // A computable is an immutable DAG, not necessarily a tree, so cap that
        // walk before shared binary edges can cause exponential revisitation.
        let mut remaining = 64;
        if let Some(equal) = compare_bounded(left, right, &mut remaining) {
            return equal;
        }

        // Retain one identity pair per already-compared node and finish the
        // comparison iteratively. This also makes unusually deep graphs safe.
        let mut pending = vec![(left, right)];
        let mut compared = HashSet::new();
        while let Some((left, right)) = pending.pop() {
            if Arc::ptr_eq(&left.internal, &right.internal) {
                continue;
            }
            match compare_nodes(left, right) {
                NodeComparison::Decided(true) => continue,
                NodeComparison::Decided(false) => return false,
                children => {
                    let identity = (Arc::as_ptr(&left.internal), Arc::as_ptr(&right.internal));
                    if !compared.insert(identity) {
                        continue;
                    }
                    match children {
                        NodeComparison::One(pair) => pending.push(pair),
                        NodeComparison::Three(pairs) => pending.extend(pairs.into_iter().rev()),
                        NodeComparison::Two(first, second) => {
                            pending.push(second);
                            pending.push(first);
                        }
                        NodeComparison::Decided(_) => unreachable!("matched above"),
                    }
                }
            }
        }
        true
    }

    /// Order a recognized `base + perturbation` against the same base exactly.
    /// Try either addend of either operand in source order; reverse the result
    /// when the sum is the other operand. The first matched base delegates to its
    /// certified perturbation sign without tolerance or numerical refinement.
    /// `None` means the form or sign was not recognized, not equality.
    fn exact_shared_perturbation_order(&self, other: &Self) -> Option<Ordering> {
        if let Approximation::Add(left, right) = &self.internal.approximation {
            if Computable::internal_structural_eq(left, other) {
                return Self::dominant_perturbation_order(left, right, other, None);
            }
            if Computable::internal_structural_eq(right, other) {
                return Self::dominant_perturbation_order(right, left, other, None);
            }
        }
        if let Approximation::Add(left, right) = &other.internal.approximation {
            if Computable::internal_structural_eq(left, self) {
                return Self::dominant_perturbation_order(left, right, self, None)
                    .map(Ordering::reverse);
            }
            if Computable::internal_structural_eq(right, self) {
                return Self::dominant_perturbation_order(right, left, self, None)
                    .map(Ordering::reverse);
            }
        }
        None
    }

    /// Use a certified perturbation to compare `base + perturbation` with `comparable`.
    /// Require a structural match between base and comparable and a known
    /// perturbation sign. With a tolerance exponent, a certified nonzero magnitude
    /// below that exponent returns tolerance `Equal`, not mathematical equality.
    /// Otherwise the sign alone orders the difference, independently of the base's
    /// sign. Missing recognition/sign returns `None`; no numerical refinement,
    /// cancellation poll or total-work bound is added by this shortcut.
    fn dominant_perturbation_order(
        base: &Self,
        perturbation: &Self,
        comparable: &Self,
        tolerance: Option<Precision>,
    ) -> Option<Ordering> {
        if !Computable::internal_structural_eq(base, comparable) {
            return None;
        }

        // Only a certified magnitude can prove that the perturbation fits
        // the tolerance. An inexact hint may drift through a long sum; its
        // known sign still gives a valid ordering without approximation.
        let (perturb_sign, perturb_msd) = perturbation.certified_sign_and_msd();
        let perturb_sign = perturb_sign?;
        if tolerance
            .is_some_and(|tolerance| perturb_msd.flatten().is_some_and(|msd| msd < tolerance))
        {
            return Some(Ordering::Equal);
        }

        // `(base + perturbation) - base` is exactly the perturbation. The
        // ordering therefore depends only on its sign; the sign or magnitude
        // of a negative base must not reverse the comparison. Exact comparison
        // passes no tolerance and can retain that proof below its refinement
        // floor; absolute-tolerance comparison may deliberately collapse a
        // smaller perturbation to `Equal` above.
        match perturb_sign {
            Sign::Minus => Some(Ordering::Less),
            Sign::NoSign => Some(Ordering::Equal),
            Sign::Plus => Some(Ordering::Greater),
        }
    }

    /// Construct exact zero as a fresh integer leaf with certified zero facts.
    /// No approximation or cancellation signal is attached.
    pub fn zero() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "zero");
        Self {
            internal: Arc::new(Node::new(
                Approximation::Int(BigInt::zero()),
                BoundCache::Valid(BoundInfo::Zero),
                ExactSignCache::Valid(Sign::NoSign),
            )),
            signal: None,
        }
    }

    /// Construct exact one using the dedicated leaf with positive sign and binade zero.
    /// The wrapper is fresh and has no attached cancellation signal.
    pub fn one() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "one");
        Self {
            internal: Arc::new(Node::new(
                Approximation::One,
                BoundCache::Valid(BoundInfo::with_sign(Sign::Plus, Some(0))),
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal: None,
        }
    }

    /// Retain exact pi, the ratio of circumference to diameter, as a named recipe.
    /// Construct a fresh positive node; requested approximations reuse the
    /// cross-thread constant cache. Construction performs no approximation and
    /// attaches no cancellation signal.
    pub fn pi() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-pi");
        Self::shared_constant(SharedConstant::Pi)
    }

    /// Retain exact `1/pi` for symbolic reciprocal-pi normalization.
    /// Use a fresh positive wrapper, shared constant approximations and no signal.
    pub(crate) fn pi_inverse_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-inv-pi");
        Self::shared_constant(SharedConstant::InvPi)
    }

    /// Retain exact principal `atan(1/5)` as a reduction/Machin anchor.
    /// Use a fresh positive wrapper, shared constant approximations and no signal.
    pub(crate) fn atan_inv5_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-atan-inv5");
        Self::shared_constant(SharedConstant::AtanInv5)
    }

    /// Retain exact principal `atan(1/2)` as a rational arctangent anchor.
    /// Use a fresh positive wrapper, shared constant approximations and no signal.
    pub(crate) fn atan_inv2_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-atan-inv2");
        Self::shared_constant(SharedConstant::AtanInv2)
    }

    /// Retain exact principal `atan(2)`, the single-argument arctangent anchor.
    /// Use a fresh positive wrapper, shared constant approximations and no signal.
    pub(crate) fn atan2_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-atan2");
        Self::shared_constant(SharedConstant::Atan2)
    }

    /// Retain exact principal `atan(3/2)` for rational arctangent reduction.
    /// Use a fresh positive wrapper, shared constant approximations and no signal.
    pub(crate) fn atan_three_halves_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-atan-three-halves");
        Self::shared_constant(SharedConstant::AtanThreeHalves)
    }

    /// Retain exact `tau = 2*pi`, the ratio of circumference to radius.
    /// Use a fresh positive named node, shared constant approximations and no
    /// attached cancellation signal; construction does not sample the value.
    pub fn tau() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-tau");
        Self::shared_constant(SharedConstant::Tau)
    }

    /// Retain exact Euler's number, the base of the natural logarithm.
    /// Delegate to the named constant constructor after recording this public
    /// entry's dispatch event. The fresh wrapper shares approximations across
    /// threads and has no attached cancellation signal.
    pub fn e() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-e");
        Self::e_constant()
    }

    /// Construct the internal exact Euler-number anchor used by exp identities.
    /// Use a fresh positive wrapper, shared constant approximations and no signal.
    pub(crate) fn e_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-e-internal");
        Self::shared_constant(SharedConstant::E)
    }

    /// Recognize a named natural logarithm for arguments 2, 3, 5, 6, 7 or 10.
    /// `None` is a catalog miss, not proof that the logarithm is invalid. A match
    /// creates a fresh positive wrapper with shared approximations and no signal;
    /// `base` denotes the argument, not a change of logarithm base.
    pub(crate) fn ln_constant(base: u32) -> Option<Computable> {
        // Common logarithms are shared constants so repeated symbolic ln forms
        // reuse one approximation cache across cloned Real values.
        crate::trace_dispatch!("computable", "constructor", "shared-log-constant-probe");
        let constant = match base {
            2 => SharedConstant::Ln2,
            3 => SharedConstant::Ln3,
            5 => SharedConstant::Ln5,
            6 => SharedConstant::Ln6,
            7 => SharedConstant::Ln7,
            10 => SharedConstant::Ln10,
            _ => return None,
        };
        Some(Self::shared_constant(constant))
    }

    /// Recognize the named positive square roots of two and three.
    /// Other integers return `None` as a catalog miss, not a domain verdict.
    /// A match has shared constant approximations and no attached signal.
    pub(crate) fn sqrt_constant(n: i64) -> Option<Computable> {
        // sqrt(2) and sqrt(3) are exact trig outputs; caching them prevents
        // fresh sqrt kernels in every sin/cos special form.
        crate::trace_dispatch!("computable", "constructor", "shared-sqrt-constant-probe");
        let constant = match n {
            2 => SharedConstant::Sqrt2,
            3 => SharedConstant::Sqrt3,
            _ => return None,
        };
        Some(Self::shared_constant(constant))
    }

    /// Retain exact `acosh(2) = ln(2 + sqrt(3))` as an inverse-hyperbolic anchor.
    /// Use a fresh positive wrapper, shared constant approximations and no signal.
    pub(crate) fn acosh2_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-acosh2");
        Self::shared_constant(SharedConstant::Acosh2)
    }

    /// Retain exact `asinh(1) = ln(1 + sqrt(2))` as an inverse-hyperbolic anchor.
    /// Use a fresh positive wrapper, shared constant approximations and no signal.
    pub(crate) fn asinh1_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-asinh1");
        Self::shared_constant(SharedConstant::Asinh1)
    }

    /// Construct reduced sine under the caller's nominal `abs(value) < 1` contract.
    /// First recover the exact inverse-cosine complement when recognized;
    /// otherwise retain the source in a sine node and copy a reduced-pi sign proof
    /// if available. No range check is performed. A new node has no outer signal;
    /// the rewrite returns the recovered argument or its negation.
    pub(crate) fn prescaled_sin(value: Computable) -> Computable {
        // Caller promises argument reduction; visibility alone is not a range proof.
        if let Some((orientation, argument)) = value.signed_acos_minus_half_pi_argument() {
            crate::trace_dispatch!(
                "computable",
                "constructor",
                "prescaled-sin-acos-minus-half-pi"
            );
            return if orientation == Sign::Plus {
                argument.negate()
            } else {
                argument
            };
        }
        crate::trace_dispatch!("computable", "constructor", "prescaled-sin");
        let sign = value
            .reduced_pi_multiple_sign()
            .map_or(ExactSignCache::Invalid, ExactSignCache::Valid);
        Self {
            internal: Arc::new(Node::new(
                Approximation::PrescaledSin(value),
                BoundCache::Invalid,
                sign,
            )),
            signal: None,
        }
    }

    /// Construct reduced cosine under the caller's nominal `abs(value) < 1` contract.
    /// A recognized signed `acos(x)-pi/2` becomes `sqrt(1-x^2)` exactly; otherwise
    /// retain the source in a cosine node with initially unknown sign and bound.
    /// This constructor does not validate the range. The ordinary node has no
    /// outer signal; the identity path follows its recovered argument's graph.
    pub(crate) fn prescaled_cos(value: Computable) -> Computable {
        // Same reduced-argument contract as prescaled_sin. Cosine has exact
        // zero/one shortcuts in the public constructor, so this stays a raw
        // approximation node for already-small residuals.
        if let Some((_, argument)) = value.signed_acos_minus_half_pi_argument() {
            crate::trace_dispatch!(
                "computable",
                "constructor",
                "prescaled-cos-acos-minus-half-pi"
            );
            return Self::one().add(argument.square().negate()).sqrt();
        }
        crate::trace_dispatch!("computable", "constructor", "prescaled-cos");
        Self {
            internal: Arc::new(Node::new(
                Approximation::PrescaledCos(value),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Prove `abs(self) <= 3/4` from a completed sample at binary precision -2.
    /// An integer sample of magnitude at most two plus its one-unit error implies
    /// this bound. Explicitly evaluate with no cancellation signal, even if this
    /// wrapper has one; that evaluation can warm caches and has no total-work
    /// limit. False means this particular sufficient test failed, not that the
    /// angle necessarily lies outside the small interval.
    fn has_certified_small_angle(&self) -> bool {
        // approx(-2) differs from 4*x by at most one. Therefore |a| <= 2
        // certifies |x| <= 3/4. Use an explicit un-aborted evaluation because
        // this result authorizes a kernel precondition rather than merely
        // choosing an optional optimization.
        self.approx_signal(&None, -2).magnitude() <= signed::TWO.magnitude()
    }

    /// Retain analytic `sin(x)/x`, including value one at zero, after a small-angle proof.
    /// The proof helper samples with cancellation disabled and admits
    /// `abs(x) <= 3/4`. Failure returns `None` without constructing this quotient;
    /// success retains the exact child and its outer signal, certifies positivity
    /// and leaves magnitude unknown. No decision that x is nonzero is required.
    pub(crate) fn sinc_small_if_certified(self) -> Option<Computable> {
        if !self.has_certified_small_angle() {
            return None;
        }
        let signal = self.signal.clone();
        crate::trace_dispatch!("computable", "constructor", "certified-small-sinc");
        Some(Self {
            internal: Arc::new(Node::new(
                Approximation::SincSmall(self),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal,
        })
    }

    /// Retain analytic `(1-cos(x))/x^2`, including value one half at zero.
    /// Use the same sufficient small-angle proof as sinc, evaluating that proof
    /// without a cancellation signal. On success retain the exact child and its
    /// signal with positive sign and unknown magnitude; otherwise return `None`.
    /// The removable value needs no zero predicate or rounded denominator.
    pub(crate) fn cosc_small_if_certified(self) -> Option<Computable> {
        if !self.has_certified_small_angle() {
            return None;
        }
        let signal = self.signal.clone();
        crate::trace_dispatch!("computable", "constructor", "certified-small-cosc");
        Some(Self {
            internal: Arc::new(Node::new(
                Approximation::CoscSmall(self),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal,
        })
    }

    /// Retain cosine of a caller-admitted exact rational with `abs(r) < 1`.
    /// Store the rational directly, avoiding a child node, and initialize a positive
    /// sign with unknown magnitude. There is no range validation or outer signal.
    fn prescaled_cos_rational(rational: Rational) -> Computable {
        // Small exact-rational cosine construction is a scalar hot path. Store
        // the rational directly so construction avoids a child Ratio node; the
        // approximation dispatcher materializes the same kernel input later if
        // digits are requested.
        crate::trace_dispatch!("computable", "constructor", "prescaled-cos-rational");
        Self {
            internal: Arc::new(Node::new(
                Approximation::PrescaledCosRational(rational),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal: None,
        }
    }

    /// Retain cosine of an exact rational and defer half-pi reduction until evaluation.
    /// The exact payload remains authoritative; construction makes no size/range
    /// check, sign decision or numerical sample. The fresh node has unknown facts
    /// and no attached signal. Deferral does not bound later reduction work.
    pub(crate) fn cos_large_rational_deferred(rational: Rational) -> Computable {
        // Real::cos for large plain rationals defers the expensive half-pi
        // reduction until digits are requested. This keeps construction and
        // structural queries cheap; the approximation node then performs direct
        // residual arithmetic without allocating the generic reducer graph.
        crate::trace_dispatch!("computable", "constructor", "cos-large-rational-deferred");
        Self {
            internal: Arc::new(Node::new(
                Approximation::CosLargeRational(rational),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Retain `cos(pi/2-r) = sin(r)` for a caller-admitted rational `1 <= r < 3/2`.
    /// Store r itself, avoiding a composite irrational residual graph. Initialize
    /// positive sign, unknown magnitude and no outer signal; do not validate the
    /// window here. The caller restores any original negative-input symmetry.
    fn prescaled_cos_half_pi_minus_rational(rational: Rational) -> Computable {
        // sin(x) for exact medium rational x is cos(pi/2 - x). Keeping the
        // residual as one node avoids the generic Add/Offset/Negate stack in
        // the cold scalar f64 and 7/5 benchmarks.
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "prescaled-cos-half-pi-minus-rational"
        );
        let internal = Approximation::PrescaledCosHalfPiMinusRational(rational);
        Self {
            internal: Arc::new(Node::new(
                internal,
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal: None,
        }
    }

    /// Retain `sin(pi/2-r) = cos(r)` for a caller-admitted rational `1 <= r < 3/2`.
    /// Store the original positive magnitude rather than the irrational residual.
    /// The sign is positive, magnitude unknown and outer signal absent; this
    /// constructor performs no range check.
    fn prescaled_sin_half_pi_minus_rational(rational: Rational) -> Computable {
        // cos(x) for exact medium rational x is sin(pi/2 - x). This mirrors the
        // cosine shortcut above and keeps common dyadic imports off the generic
        // composite residual path.
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "prescaled-sin-half-pi-minus-rational"
        );
        let internal = Approximation::PrescaledSinHalfPiMinusRational(rational);
        Self {
            internal: Arc::new(Node::new(
                internal,
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal: None,
        }
    }

    /// Retain `cot(pi/2-r) = tan(r)` for a caller-admitted rational `1 <= r < 3/2`.
    /// Store r directly with positive sign, unknown magnitude and no signal.
    /// No range check occurs here. This exact definition does not repair the
    /// cotangent evaluator's documented coarse-zero accuracy gap.
    fn prescaled_cot_half_pi_minus_rational(rational: Rational) -> Computable {
        // tan(x) near pi/2 is cot(pi/2 - x). Keeping the residual exact avoids
        // the generic complement tree and lets the approximation layer evaluate
        // the local quotient directly.
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "prescaled-cot-half-pi-minus-rational"
        );
        let internal = Approximation::PrescaledCotHalfPiMinusRational(rational);
        Self {
            internal: Arc::new(Node::new(
                internal,
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal: None,
        }
    }

    /// Retain sine of an exact rational for lazy half-pi reduction and quadrant selection.
    /// Store the original rational with unknown sign and magnitude and no signal.
    /// Construction neither enforces a minimum magnitude nor samples the angle;
    /// subsequent reduction has no total-work bound supplied by this wrapper.
    pub(crate) fn sin_large_rational_deferred(rational: Rational) -> Computable {
        // Same lazy-construction policy as cos_large_rational_deferred. The
        // approximation node evaluates the direct half-pi residual itself, so
        // exact 1e6/1e30 scalar rows avoid eager reducer graph construction.
        crate::trace_dispatch!("computable", "constructor", "sin-large-rational-deferred");
        Self {
            internal: Arc::new(Node::new(
                Approximation::SinLargeRational(rational),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Retain tangent of an exact rational for deferred sine/cosine reduction.
    /// Finite rational angles are not odd half-pi poles, but this supplies no
    /// uniform evaluation-cost bound. Store unknown facts and no outer signal.
    /// The evaluator's documented coarse-zero gap remains independent of this
    /// exact payload; construction performs no approximation or size check.
    pub(crate) fn tan_large_rational_deferred(rational: Rational) -> Computable {
        // Tangent used to run through generic pi reduction even for exact large
        // rationals. Deferring it into a dedicated approximation node lets the
        // hot 1e6/1e30 rows share the direct half-pi residual used by sin/cos.
        crate::trace_dispatch!("computable", "constructor", "tan-large-rational-deferred");
        Self {
            internal: Arc::new(Node::new(
                Approximation::TanLargeRational(rational),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Retain reduced tangent under the caller's nominal `abs(value) < 1` contract.
    /// Copy a recognized reduced-pi sign; otherwise leave facts unknown. This
    /// constructor neither proves the range nor separates poles and attaches no
    /// outer signal. Wider native schedules and coarse-zero limitations are
    /// documented at the tangent evaluator, not certified by this tag.
    pub(crate) fn prescaled_tan(value: Computable) -> Computable {
        // Same reduced-argument contract as prescaled_sin; tangent additionally
        // relies on the public constructor to handle near-pole complements.
        crate::trace_dispatch!("computable", "constructor", "prescaled-tan");
        let sign = value
            .reduced_pi_multiple_sign()
            .map_or(ExactSignCache::Invalid, ExactSignCache::Valid);
        Self {
            internal: Arc::new(Node::new(
                Approximation::PrescaledTan(value),
                BoundCache::Invalid,
                sign,
            )),
            signal: None,
        }
    }

    /// Retain sine of a caller-admitted rational with `abs(r) < 1` directly.
    /// Its sign is the rational's sign on that interval. Store that fact without
    /// checking the range; magnitude is unknown and no signal is attached.
    fn prescaled_sin_rational(rational: Rational) -> Computable {
        // Small exact-rational sine construction mirrors cosine and preserves
        // the exact sign without allocating a child Computable.
        crate::trace_dispatch!("computable", "constructor", "prescaled-sin-rational");
        let sign = rational.sign();
        Self {
            internal: Arc::new(Node::new(
                Approximation::PrescaledSinRational(rational),
                BoundCache::Invalid,
                ExactSignCache::Valid(sign),
            )),
            signal: None,
        }
    }

    /// Retain tangent of a caller-admitted rational with `abs(r) < 1` directly.
    /// Initialize the rational's sign, which agrees with tangent in this interval,
    /// and leave magnitude unknown. No range check or outer signal is supplied.
    fn prescaled_tan_rational(rational: Rational) -> Computable {
        // Small exact-rational tangent uses the same construction shortcut as
        // sine; sign follows the rational argument on the reduced interval.
        crate::trace_dispatch!("computable", "constructor", "prescaled-tan-rational");
        let sign = rational.sign();
        Self {
            internal: Arc::new(Node::new(
                Approximation::PrescaledTanRational(rational),
                BoundCache::Invalid,
                ExactSignCache::Valid(sign),
            )),
            signal: None,
        }
    }

    /// Schedule asinh's local odd series while retaining the exact child.
    /// Construction performs no range proof and leaves sign/magnitude unknown.
    /// The evaluator proves its half-unit series range or falls back by sign to
    /// the logarithmic identity, so a defined finite child remains required.
    /// The new outer wrapper has no cancellation signal.
    fn prescaled_asinh(value: Computable) -> Computable {
        // The evaluator must establish the series range before consuming this tag.
        crate::trace_dispatch!("computable", "constructor", "prescaled-asinh");
        Self {
            internal: Arc::new(Node::new(
                Approximation::PrescaledAsinh(value),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Schedule principal asin's tiny series for a caller-admitted child in `[-1,1]`.
    /// Copy a known exact sign without sampling the value here. The evaluator
    /// checks the series range and falls back to a half-angle identity if needed;
    /// this constructor neither validates the real domain nor attaches a signal.
    fn prescaled_asin(value: Computable) -> Computable {
        // Tiny non-rational asin inputs use the direct odd-power series. This
        // mirrors prescaled atan/asinh dispatch and avoids building the generic
        // atan/sqrt transform once the argument is structurally small.
        crate::trace_dispatch!("computable", "constructor", "prescaled-asin");
        let sign = value.exact_sign();
        Self {
            internal: Arc::new(Node::new(
                Approximation::PrescaledAsin(value),
                BoundCache::Invalid,
                sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid),
            )),
            signal: None,
        }
    }

    /// Retain asinh of an exact rational without an intermediate child node.
    /// Initialize its exact sign and leave magnitude unknown. The evaluator
    /// selects the small series or a sign-normalized logarithmic fallback, so no
    /// smallness proof is implied by construction. No signal is attached.
    fn asinh_rational_deferred(rational: Rational) -> Computable {
        // Same series as `prescaled_asinh`, but exact rationals can skip the
        // child Computable wrapper and feed the kernel directly.
        crate::trace_dispatch!("computable", "constructor", "asinh-rational-deferred");
        let sign = rational.sign();
        Self {
            internal: Arc::new(Node::new(
                Approximation::AsinhRational(rational),
                BoundCache::Invalid,
                ExactSignCache::Valid(sign),
            )),
            signal: None,
        }
    }

    /// Schedule atanh's tiny series for an admitted child strictly between -1 and 1.
    /// The evaluator independently checks its local series range and may use the
    /// log-ratio identity. This constructor only copies a known exact sign,
    /// leaves magnitude unknown and attaches no signal; it does not check domain.
    fn prescaled_atanh(value: Computable) -> Computable {
        // Tiny non-rational atanh inputs use the direct odd-power series. This
        // keeps parity with exact-rational AtanhRational and avoids the heavier
        // log-ratio graph for already-small symbolic arguments.
        crate::trace_dispatch!("computable", "constructor", "prescaled-atanh");
        let sign = value.exact_sign();
        Self {
            internal: Arc::new(Node::new(
                Approximation::PrescaledAtanh(value),
                BoundCache::Invalid,
                sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid),
            )),
            signal: None,
        }
    }

    /// Retain principal atanh of an admitted rational strictly between -1 and 1.
    /// Store the rational directly with its sign, unknown magnitude and no signal.
    /// The evaluator selects the tiny series or log-ratio reduction; neither the
    /// open real domain nor series smallness is checked during construction.
    fn atanh_rational_deferred(rational: Rational) -> Computable {
        // Tiny exact-rational atanh uses the odd series directly. Keeping the
        // Rational payload avoids rebuilding a Ratio node in cold approximation
        // benches while preserving the symbolic value until the final request.
        crate::trace_dispatch!("computable", "constructor", "atanh-rational-deferred");
        let sign = rational.sign();
        Self {
            internal: Arc::new(Node::new(
                Approximation::AtanhRational(rational),
                BoundCache::Invalid,
                ExactSignCache::Valid(sign),
            )),
            signal: None,
        }
    }

    /// Retain principal acos for an admitted `0 <= value <= 1` through a half-angle recipe.
    /// Evaluation uses `2*atan(sqrt((1-value)/(1+value)))`. Leave sign unknown:
    /// the endpoint one yields zero even though the source is positive. No domain
    /// check, magnitude proof or outer cancellation signal is supplied here.
    fn acos_positive(value: Computable) -> Computable {
        // For x >= 0, acos(x) is reduced with 2*atan(sqrt((1-x)/(1+x))).
        // A single deferred node avoids allocating that whole formula during
        // public construction of endpoint-heavy inverse trig expressions.
        crate::trace_dispatch!("computable", "constructor", "acos-positive-deferred");
        Self {
            // A positive argument may still be exactly one, where acos is
            // zero. The public constructor normalizes covered exact replay
            // forms first; unsupported forms must remain Unknown rather than
            // carrying a false strictly-positive certificate.
            internal: Arc::new(Node::new(
                Approximation::AcosPositive(value),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Retain principal acos of a rational admitted strictly below one and at least zero.
    /// This constructor initializes a strictly positive sign, so the zero-result
    /// endpoint one must be handled by the caller before entry. Store unknown
    /// magnitude and no signal; the rational domain is not validated here.
    fn acos_positive_rational_deferred(rational: Rational) -> Computable {
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "acos-positive-rational-deferred"
        );
        Self {
            internal: Arc::new(Node::new(
                Approximation::AcosPositiveRational(rational),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal: None,
        }
    }

    /// Retain `acos(-magnitude) = pi-acos(magnitude)` for magnitude in `[0,1]`.
    /// The payload is nonnegative, not the signed acos argument. Initialize
    /// positive sign, unknown magnitude and no signal without validating the
    /// caller's domain obligation.
    fn acos_negative_rational_deferred(magnitude: Rational) -> Computable {
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "acos-negative-rational-deferred"
        );
        Self {
            internal: Arc::new(Node::new(
                Approximation::AcosNegativeRational(magnitude),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal: None,
        }
    }

    /// Retain principal asin on `[-1,1]` through the deferred half-angle atan/sqrt identity.
    /// The exact child remains available to symbolic consumers. Copy its known
    /// sign, leave magnitude unknown and attach no outer signal; the caller must
    /// establish the real domain before this unchecked construction.
    fn asin_deferred(value: Computable) -> Computable {
        // Generic asin uses a stable atan/sqrt half-angle transform. Deferring
        // that formula keeps symbolic-radical construction lightweight and
        // leaves the exact input graph intact until approximation is requested.
        crate::trace_dispatch!("computable", "constructor", "asin-deferred");
        let sign = value.exact_sign();
        Self {
            internal: Arc::new(Node::new(
                Approximation::AsinDeferred(value),
                BoundCache::Invalid,
                sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid),
            )),
            signal: None,
        }
    }

    /// Retain principal atanh as `ln((1+x)/(1-x))/2` for caller-admitted `-1 < x < 1`.
    /// Defer reduction-graph construction until evaluation and leave sign/magnitude
    /// unknown. This wrapper neither checks the open domain nor attaches an outer
    /// signal; endpoints cannot represent finite values of this recipe.
    pub(crate) fn atanh_direct_deferred(value: Computable) -> Computable {
        // Endpoint atanh uses a deferred ln-ratio node. This keeps construction
        // cheap for predicate/scalar benches while preserving the same
        // approximation identity when a numeric value is requested.
        crate::trace_dispatch!("computable", "constructor", "atanh-direct-deferred");
        Self {
            internal: Arc::new(Node::new(
                Approximation::AtanhDirect(value),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Retain acosh via `ln1p((x-1)+sqrt(x^2-1))` for caller-admitted `x > 1`.
    /// Near-one describes the preferred schedule, not an enforced upper bound.
    /// Initialize strictly positive sign, unknown magnitude and no outer signal;
    /// the caller must handle x=1 separately before using this constructor.
    pub(crate) fn acosh_near_one_deferred(value: Computable) -> Computable {
        // Near-one acosh uses a deferred ln1p/sqrt reduction. That avoids
        // building the reduction graph during scalar construction while keeping
        // the cancellation-resistant approximation path.
        crate::trace_dispatch!("computable", "constructor", "acosh-near-one-deferred");
        Self {
            internal: Arc::new(Node::new(
                Approximation::AcoshNearOne(value),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal: None,
        }
    }

    /// Retain acosh via `ln(x+sqrt(x^2-1))` for caller-admitted `x > 1`.
    /// Defer the sqrt/log graph, storing positive sign and unknown magnitude
    /// without checking the domain. The caller must handle x=1 separately to
    /// justify the sign fact. No outer signal is attached.
    pub(crate) fn acosh_direct_deferred(value: Computable) -> Computable {
        // Large acosh uses a deferred direct ln/sqrt identity so construction
        // paths do not eagerly allocate the sqrt/log graph.
        crate::trace_dispatch!("computable", "constructor", "acosh-direct-deferred");
        Self {
            internal: Arc::new(Node::new(
                Approximation::AcoshDirect(value),
                BoundCache::Invalid,
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal: None,
        }
    }

    /// Retain asinh via `ln1p(x+x^2/(sqrt(1+x^2)+1))` for a defined finite child.
    /// Rationalizing the square-root difference avoids near-zero cancellation;
    /// the identity has no separate upper domain bound. Copy a known exact sign,
    /// leave magnitude unknown and attach no outer signal. Reduction is deferred.
    pub(crate) fn asinh_near_zero_deferred(value: Computable) -> Computable {
        // Moderate/tiny asinh inputs use a deferred ln1p reduction so public
        // construction stays lightweight while approximation still avoids
        // cancellation near zero.
        crate::trace_dispatch!("computable", "constructor", "asinh-near-zero-deferred");
        let sign = value.exact_sign();
        Self {
            internal: Arc::new(Node::new(
                Approximation::AsinhNearZero(value),
                BoundCache::Invalid,
                sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid),
            )),
            signal: None,
        }
    }

    /// Retain asinh via `ln(x+sqrt(1+x^2))` for a defined finite child.
    /// Callers may normalize negative inputs to avoid cancellation; this wrapper
    /// does not change the sign or prove a size condition. Copy a known sign,
    /// leave magnitude unknown and attach no outer signal; expand on evaluation.
    pub(crate) fn asinh_direct_deferred(value: Computable) -> Computable {
        // Large asinh inputs use a deferred direct ln/sqrt identity. The caller
        // chooses this only after sign and size reduction, so no extra probing
        // is needed during construction.
        crate::trace_dispatch!("computable", "constructor", "asinh-direct-deferred");
        let sign = value.exact_sign();
        Self {
            internal: Arc::new(Node::new(
                Approximation::AsinhDirect(value),
                BoundCache::Invalid,
                sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid),
            )),
            signal: None,
        }
    }

    /// Wrap a named exact constant in a fresh node with its known positive sign and bound facts.
    /// Approximation reuse belongs to the cross-thread `SHARED_CONSTANT_CACHES`
    /// table; this wrapper has no abort signal and is distinct from thread-local
    /// Real templates. Construction neither computes digits nor establishes a
    /// precision, total-work or memory budget for later requests.
    fn shared_constant(constant: SharedConstant) -> Computable {
        // Initialize per-node facts directly; approximation values live in the
        // separate shared constant cache rather than in a thread-local table.
        crate::trace_dispatch!("computable", "constructor", "shared-constant-wrapper");
        Self {
            internal: Arc::new(Node::new(
                Approximation::Constant(constant),
                BoundCache::Valid(constant.bound_info()),
                ExactSignCache::Valid(Sign::Plus),
            )),
            signal: None,
        }
    }

    /// Retain any exact rational, canonicalizing zero, one and integral values first.
    /// Zero and one use their dedicated constructors; other integers delegate to
    /// `integer`. Nonintegral values retain the rational payload with initially
    /// unknown sign and magnitude caches, allowing later exact recognition.
    /// No numerical approximation or outer cancellation signal is introduced.
    pub fn rational(r: Rational) -> Computable {
        if r.sign() == Sign::NoSign {
            // Canonicalize rational zero at construction time. This exposes
            // exact sign/zero facts immediately and avoids a Ratio leaf in the
            // many higher-level code paths that still call `rational(0)`.
            crate::trace_dispatch!("computable", "constructor", "rational-zero-canonicalized");
            return Self::zero();
        }
        if r.is_one() {
            // Route rational one through the dedicated One node so callers that
            // import binary64-derived dyadic/integer identities get the same cheap constructor
            // and structural facts as `Computable::one()`.
            crate::trace_dispatch!("computable", "constructor", "rational-one-canonicalized");
            return Self::one();
        }
        if r.is_integer() {
            crate::trace_dispatch!(
                "computable",
                "constructor",
                "rational-integer-canonicalized"
            );
            return Self::integer(BigInt::from_biguint(r.sign(), r.numerator().clone()));
        }
        crate::trace_dispatch!("computable", "constructor", "rational-node");
        Self {
            internal: Arc::new(Node::new(
                Approximation::Ratio(r),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }
}
