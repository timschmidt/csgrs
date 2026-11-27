/// LIFO storage for common small evaluator stacks, with an unbounded heap spill.
///
/// Occupied inline slots form a prefix and precede every overflow entry. Pop
/// drains overflow first, then takes inline values; entries are moved, not cloned.
/// N is an inline-capacity choice, not a work or memory limit, and zero capacity
/// simply uses the vector. Ordinary ownership drops any entries left on return.
struct InlineStack<T, const N: usize> {
    /// Prefix storage for the earliest live entries; unused slots are None.
    inline: [Option<T>; N],
    /// Number of occupied prefix slots, always at most N.
    inline_len: usize,
    /// Later live entries once the prefix fills; no fixed allocation cap.
    overflow: Vec<T>,
}

/// Move-only stack operations preserving inline-prefix and overflow ordering.
impl<T, const N: usize> InlineStack<T, N> {
    /// Create an empty stack without allocating overflow capacity.
    fn new() -> Self {
        Self {
            inline: std::array::from_fn(|_| None),
            inline_len: 0,
            overflow: Vec::new(),
        }
    }

    /// Append to the free inline prefix only while overflow is empty; otherwise append to the vector.
    fn push(&mut self, value: T) {
        if self.overflow.is_empty() && self.inline_len < N {
            self.inline[self.inline_len] = Some(value);
            self.inline_len += 1;
        } else {
            self.overflow.push(value);
        }
    }

    /// Remove the newest overflow entry, or take the last inline slot; return None when empty.
    fn pop(&mut self) -> Option<T> {
        if let Some(value) = self.overflow.pop() {
            return Some(value);
        }
        self.inline_len = self.inline_len.checked_sub(1)?;
        self.inline[self.inline_len].take()
    }
}

/// Evaluate exact expression graphs numerically and consume retained sign, order and magnitude evidence.
impl Computable {
    /// Return an exact sign already retained at this expression's root.
    ///
    /// Unlike [`Self::structural_facts`], this query never walks the expression
    /// graph and never evaluates an approximation. It is intended for bounded
    /// accelerators whose inconclusive result has an authoritative fallback.
    /// Prefer a retained exact sign, then the sign of a retained bound. Unknown
    /// and invalid sign slots both permit that bound fallback; no fact is published.
    pub(crate) fn immediate_sign(&self) -> Option<RealSign> {
        let (bound, sign) = self.internal.facts.snapshot();
        match sign {
            ExactSignCache::Valid(sign) => Some(public_sign(sign)),
            ExactSignCache::Invalid | ExactSignCache::Unknown => match bound {
                BoundCache::Valid(bound) => bound.known_sign().map(public_sign),
                BoundCache::Invalid => None,
            },
        }
    }

    /// Approximate this value x by an integer at binary scale `2^-p`.
    ///
    /// On completed, valid evaluation, `|result - x*2^-p| <= 1`: more negative p
    /// requests more fractional bits. This is not a
    /// promised correctly rounded conversion or a canonical choice of integer.
    /// Retained approximations may satisfy a request by certified rescaling.
    /// Uses this wrapper's attached signal; see [`Self::approx_signal`] for
    /// interruption, cache publication and precision/resource limitations.
    ///
    /// At precision zero, 7/8 admits either adjacent integer:
    /// ```
    /// use hyperreal::{Computable, Rational};
    /// use num::BigInt;
    /// let value = Computable::rational(Rational::fraction(7, 8).unwrap());
    /// assert!((BigInt::from(0)..=BigInt::from(1)).contains(&value.approx(0)));
    /// ```
    /// At precision -3, the integer approximates eight times pi:
    /// ```
    /// use hyperreal::Computable;
    /// use num::BigInt;
    /// assert!((BigInt::from(25)..=BigInt::from(26)).contains(&Computable::pi().approx(-3)));
    /// ```
    pub fn approx(&self, p: Precision) -> BigInt {
        self.approx_signal(&self.signal, p)
    }

    /// Return either the floor or the ceiling after a completed approximation.
    ///
    /// Request precision -2 once, divide the resulting integer by four and round.
    /// The combined error is at most 3/4, which makes the returned integer
    /// adjacent to the exact value without deciding whether that value is integral.
    /// This is not specified as nearest-integer or directed rounding. One outer
    /// approximation request can still perform substantial nested work; it is not
    /// a total-work bound. Interruption inherits [`Self::approx_signal`]'s incomplete
    /// result behavior, so the adjacency guarantee requires completed evaluation.
    pub fn near_integer(&self) -> BigInt {
        // `approx(-2)` differs from 4*x by at most one. Rounding it after a
        // division by four adds at most another half, so the returned integer
        // differs from x by at most 3/4 and must be floor(x) or ceil(x).
        scale(self.approx(-2), -2)
    }

    /// Evaluate at binary precision p with an explicit signal instead of this wrapper's attachment.
    ///
    /// Exact integer leaves scale directly. Shared constants consult process-wide
    /// caches; other nodes try their numeric cache and a retained quadratic tower.
    /// Negate/Add/Offset chains use explicit stacks; other kernels can still recurse.
    /// Addition requests two guard bits per child, evaluates the larger linear-demand
    /// hint first, then rounds once. Offsets translate child precision. These
    /// scheduling hints change cache reuse, not the represented expression.
    ///
    /// Completed results are published through the signal-aware cache writer,
    /// which preserves stronger existing facts and the finest approximation.
    /// An asserted signal suppresses publication, but this BigInt return type has
    /// no cancellation status: lower kernels may return incomplete values and
    /// exact/cache shortcuts may still finish. Do not consume an interrupted
    /// result as a certified enclosure solely because this method returned.
    ///
    /// Precision is not a cumulative work or allocation budget. Derived exponents
    /// such as `p-2` and `p-offset` use ordinary i32 arithmetic here; extreme inputs
    /// are not uniformly checked or reported as typed errors. Large valid shifts
    /// can also request large integers. The underlying graph must satisfy each
    /// kernel's mathematical domain requirements.
    pub fn approx_signal(&self, signal: &Option<Signal>, p: Precision) -> BigInt {
        /// Continuation stack for linear graph nodes; completed child integers use a separate value stack.
        enum Frame<'a> {
            /// Reuse retained evidence or schedule evaluation of this node.
            Eval(
                /// Borrowed immutable expression to evaluate.
                &'a Computable,
                /// Requested binary precision at this node.
                Precision,
            ),
            /// Negate the single completed child and publish the parent's result.
            FinishNegate(
                /// Parent whose cache receives the negated result.
                &'a Computable,
                /// Parent precision, unchanged for its negated child.
                Precision,
            ),
            /// Sum two child results at two guard bits, round and publish the parent.
            FinishAdd(
                /// Parent sum; child scheduling may have reversed operand evaluation order.
                &'a Computable,
                /// Parent precision after discarding the two child guard bits.
                Precision,
            ),
            /// Forward the child's integer unchanged because its precision already incorporates the offset.
            FinishOffset(
                /// Offset parent whose cache receives the result.
                &'a Computable,
                /// Parent precision at which that unchanged integer represents its value.
                Precision,
            ),
        }

        // Exact integer leaves scale directly without a synchronized cache.
        // Required storage depends on both input magnitude and requested precision.
        let uses_node_cache = match &self.internal.approximation {
            Approximation::Int(value) => return scale_at_precision(value.clone(), p),
            Approximation::One => return scale_at_precision(signed::ONE.deref().clone(), p),
            Approximation::Constant(constant) => {
                if let Some(cached) = Self::cached_constant_at_precision(*constant, p) {
                    return cached;
                }
                false
            }
            _ => true,
        };

        if uses_node_cache && let Some(cached) = self.internal.cached_at_precision(p) {
            return cached;
        }
        if uses_node_cache
            && let Some(result) = self.cached_quadratic_tower_approximation(signal, p)
        {
            self.store_cache_value(signal, p, result.clone());
            return result;
        }

        if !matches!(
            &self.internal.approximation,
            Approximation::Negate(_) | Approximation::Add(_, _) | Approximation::Offset(_, _)
        ) {
            // Most node kinds evaluate as one kernel call. Only Negate/Add/Offset
            // are flattened below because they form the long chains seen in
            // parser, matrix, and structural-reduction workloads.
            let result = self.internal.approximate(signal, p);
            self.store_cache_value(signal, p, result.clone());
            return result;
        }

        // Common expression chains fit inline; unusually deep trees spill to
        // heap storage without changing the iterative traversal.
        let mut frames = InlineStack::<Frame<'_>, 16>::new();
        let mut values = InlineStack::<BigInt, 8>::new();
        frames.push(Frame::Eval(self, p));

        while let Some(frame) = frames.pop() {
            match frame {
                Frame::Eval(node, prec) => {
                    if let Some(cached) = node.cached_at_precision(prec) {
                        values.push(cached);
                        continue;
                    }

                    if let Some(result) = node.cached_quadratic_tower_approximation(signal, prec) {
                        node.store_cache_value(signal, prec, result.clone());
                        values.push(result);
                        continue;
                    }

                    match &node.internal.approximation {
                        Approximation::Negate(child) => {
                            // Flatten sign wrappers so a deep chain of negated
                            // sums does not recurse through approx_signal.
                            frames.push(Frame::FinishNegate(node, prec));
                            frames.push(Frame::Eval(child, prec));
                        }
                        Approximation::Add(left, right) => {
                            // Evaluate add children at two guard bits, then
                            // round once. This mirrors the recursive add kernel
                            // but avoids stack growth for chained additions.
                            frames.push(Frame::FinishAdd(node, prec));
                            // Evaluate larger predicted precision demands
                            // first, allowing siblings to reuse finer caches.
                            // Binary scaling participates in the hint: depth
                            // alone reverses the right order for decaying terms.
                            // Both operands retain exactly the same precision;
                            // this hint cannot alter the enclosure contract.
                            let (first, second) = if right.internal.facts.linear_demand()
                                > left.internal.facts.linear_demand()
                            {
                                (right, left)
                            } else {
                                (left, right)
                            };
                            frames.push(Frame::Eval(second, prec - 2));
                            frames.push(Frame::Eval(first, prec - 2));
                        }
                        Approximation::Offset(child, n) => {
                            // Binary offsets translate the requested precision
                            // instead of doing any arithmetic at finish time.
                            frames.push(Frame::FinishOffset(node, prec));
                            frames.push(Frame::Eval(child, prec - *n));
                        }
                        _ => {
                            let result = node.internal.approximate(signal, prec);
                            node.store_cache_value(signal, prec, result.clone());
                            values.push(result);
                        }
                    }
                }
                Frame::FinishNegate(node, prec) => {
                    let result = -values.pop().expect("negate child result should exist");
                    node.store_cache_value(signal, prec, result.clone());
                    values.push(result);
                }
                Frame::FinishAdd(node, prec) => {
                    let right = values.pop().expect("add rhs result should exist");
                    let left = values.pop().expect("add lhs result should exist");
                    let result = scale(left + right, -2);
                    node.store_cache_value(signal, prec, result.clone());
                    values.push(result);
                }
                Frame::FinishOffset(node, prec) => {
                    let result = values.pop().expect("offset child result should exist");
                    node.store_cache_value(signal, prec, result.clone());
                    values.push(result);
                }
            }
        }

        values.pop().expect("evaluation should produce a result")
    }

    /// Report a snapshot of structural/retained sign, zero, rational-availability and magnitude facts.
    ///
    /// Try exact-sign analysis, a separated numeric cache, cheap bounds and deferred
    /// exact-rational bounds in that order. A sign establishes zero/nonzero status;
    /// unsigned nonzero bounds can also prove nonzero while leaving sign unknown.
    /// Magnitude comes from bounds and retains its exactness qualification.
    /// No new approximation is requested here, but structural traversal, rational
    /// recognition and cache updates can occur. This is not allocation-free or a
    /// source-bound proof receipt; absent rational evidence does not prove irrationality.
    pub fn structural_facts(&self) -> RealStructuralFacts {
        let exact = self.exact_rational();
        let (cached_bound, cached_sign) = self.internal.facts.snapshot();

        let mut sign = match cached_sign {
            ExactSignCache::Valid(sign) => Some(public_sign(sign)),
            ExactSignCache::Invalid | ExactSignCache::Unknown => self.exact_sign().map(public_sign),
        };
        #[cfg(feature = "dispatch-trace")]
        if sign.is_some() {
            crate::trace_dispatch!("computable", "structural_facts", "exact-sign-cache");
        }
        if sign.is_none()
            && let Some((_, appr)) = self.cached()
            && appr.abs() > BigInt::one()
        {
            crate::trace_dispatch!("computable", "structural_facts", "approximation-cache-sign");
            sign = Some(public_sign(appr.sign()));
        }

        let bound = match cached_bound {
            BoundCache::Valid(bound) => bound,
            BoundCache::Invalid => self.cheap_bound(),
        };
        if sign.is_none() {
            let bound_sign = bound.known_sign();
            #[cfg(feature = "dispatch-trace")]
            if bound_sign.is_some() {
                crate::trace_dispatch!("computable", "structural_facts", "cheap-bound-sign");
            }
            sign = bound_sign.map(public_sign);
        }
        if sign.is_none() {
            let exact_bound_sign = exact
                .as_ref()
                .map(BoundInfo::from_rational)
                .as_ref()
                .and_then(BoundInfo::known_sign);
            #[cfg(feature = "dispatch-trace")]
            if exact_bound_sign.is_some() {
                crate::trace_dispatch!("computable", "structural_facts", "exact-rational-bound");
            }
            sign = exact_bound_sign.map(public_sign);
        }
        let exact_bound = if sign.is_none() {
            // Keep exact-rational bounds deferred until sign could not be proven by
            // cheaper structural facts. This avoids unnecessary conversion work.
            exact.as_ref().map(BoundInfo::from_rational)
        } else {
            None
        };

        let zero = match sign {
            Some(RealSign::Zero) => ZeroKnowledge::Zero,
            Some(RealSign::Negative | RealSign::Positive) => ZeroKnowledge::NonZero,
            None => {
                if matches!(&bound, BoundInfo::Zero)
                    || matches!(&exact_bound, Some(BoundInfo::Zero))
                {
                    ZeroKnowledge::Zero
                } else if matches!(&bound, BoundInfo::NonZero { .. })
                    || matches!(&exact_bound, Some(BoundInfo::NonZero { .. }))
                {
                    ZeroKnowledge::NonZero
                } else {
                    ZeroKnowledge::Unknown
                }
            }
        };

        let magnitude = bound
            .magnitude_bits()
            .or_else(|| exact_bound.as_ref().and_then(BoundInfo::magnitude_bits));

        RealStructuralFacts {
            sign,
            zero,
            exact_rational: exact.is_some(),
            magnitude,
        }
    }

    /// Conservatively report Zero, NonZero or Unknown using exact-sign analysis, then cheap bounds.
    /// Both analyses can derive and retain facts without requesting a new approximation.
    /// This narrower path does not perform structural_facts' separate numeric-cache
    /// inspection or general algebraic reduction; Unknown is not a nonzero answer.
    #[inline]
    pub fn zero_status(&self) -> ZeroKnowledge {
        if let Some(sign) = self.exact_sign() {
            crate::trace_dispatch!("computable", "zero_status", "exact-sign-cache");
            return if sign == Sign::NoSign {
                ZeroKnowledge::Zero
            } else {
                ZeroKnowledge::NonZero
            };
        }

        match self.cheap_bound() {
            BoundInfo::Zero => {
                crate::trace_dispatch!("computable", "zero_status", "cheap-bound-zero");
                ZeroKnowledge::Zero
            }
            BoundInfo::NonZero { .. } => {
                crate::trace_dispatch!("computable", "zero_status", "cheap-bound-nonzero");
                ZeroKnowledge::NonZero
            }
            BoundInfo::Unknown => {
                crate::trace_dispatch!("computable", "zero_status", "unknown");
                ZeroKnowledge::Unknown
            }
        }
    }

    /// Try to prove a sign with outer refinement requests no finer than min_precision.
    ///
    /// Preserve the cascade: exact-sign analysis, a separated retained approximation,
    /// cheap bounds, certified binary64 filtering, then integer refinement. Previously
    /// proved facts can come from finer precision than this request. Kernel guard bits
    /// and child precisions are not bounded by the outer request floor.
    ///
    /// Refinement starts at max(0,min_precision), then uses `3*p/2-16` clamped to the
    /// floor and any admitted algebraic zero target. Metadata is deferred until -64
    /// or an exhausted floor at most -2; a proved separation bound can certify zero.
    /// Ordinary separated approximations certify only nonzero signs. The last fallback
    /// recognizes exact rational zero; otherwise None preserves an unresolved result.
    ///
    /// Some signs are retained for later calls. The floor is not a total-work bound,
    /// and extreme recurrence/child exponents are not uniformly checked. The attached
    /// signal is polled between attempts, not at every return. This Option carries
    /// no cancellation receipt; as with approx_signal, interrupted numerical work
    /// must not be treated as certified merely because a value was returned.
    pub fn sign_until(&self, min_precision: Precision) -> Option<RealSign> {
        if let Some(sign) = self.exact_sign() {
            crate::trace_dispatch!("computable", "sign_until", "exact-sign-cache");
            return Some(public_sign(sign));
        }
        if let Some((_, appr)) = self.cached()
            && appr.abs() > BigInt::one()
        {
            let sign = appr.sign();
            self.internal
                .facts
                .replace_exact_sign(ExactSignCache::Valid(sign));
            crate::trace_dispatch!("computable", "sign_until", "approximation-cache-sign");
            return Some(public_sign(sign));
        }

        // Prefer structural facts before touching extra approximation work.
        // This keeps sign queries cheap when bounds are already strong enough.
        if let Some(sign) = self.cheap_bound().known_sign() {
            crate::trace_dispatch!("computable", "sign_until", "cheap-bound-sign");
            return Some(public_sign(sign));
        }

        if let Some(sign) = self.binary64_filter_sign_until(min_precision) {
            self.internal
                .facts
                .replace_exact_sign(ExactSignCache::Valid(sign));
            crate::trace_dispatch!("computable", "sign_until", "binary64-filter-sign");
            return Some(public_sign(sign));
        }

        crate::trace_dispatch!("computable", "sign_until", "precision-refinement");
        let start = if min_precision > 0 { min_precision } else { 0 };
        let mut p = start;
        let mut checked_algebraic_bound = false;
        let mut algebraic_zero_precision = None;
        loop {
            let appr = self.approx(p);
            #[cfg(feature = "dispatch-trace")]
            crate::dispatch_trace::record_sign_refinement_precision(
                "sign_until_attempt_precision",
                p,
            );
            if appr.abs() > BigInt::one() {
                let sign = appr.sign();
                self.internal
                    .facts
                    .replace_exact_sign(ExactSignCache::Valid(sign));
                #[cfg(feature = "dispatch-trace")]
                crate::dispatch_trace::record_sign_refinement_precision(
                    "sign_until_decision_precision",
                    p,
                );
                return Some(public_sign(sign));
            }

            // Ordinary nonzero inputs usually separate before 64 bits and pay
            // no algebraic-metadata cost. At a shallower negative caller floor,
            // inspect metadata only after ordinary refinement is exhausted;
            // this still makes every certificate that fits that floor usable.
            // A proved target then stops refinement without crossing the floor.
            if p <= -64 || (p <= min_precision && min_precision <= -2) {
                if !checked_algebraic_bound {
                    checked_algebraic_bound = true;
                    algebraic_zero_precision = self.algebraic_zero_precision(min_precision);
                    if algebraic_zero_precision.is_some() {
                        crate::trace_dispatch!(
                            "computable",
                            "sign_until",
                            "algebraic-separation-target"
                        );
                    } else {
                        crate::trace_dispatch!(
                            "computable",
                            "sign_until",
                            "algebraic-separation-unavailable"
                        );
                    }
                }
                if algebraic_zero_precision.is_some_and(|target| p <= target) {
                    self.internal
                        .facts
                        .replace_exact_sign(ExactSignCache::Valid(Sign::NoSign));
                    crate::trace_dispatch!("computable", "sign_until", "algebraic-separation-zero");
                    return Some(RealSign::Zero);
                }
            }

            if p <= min_precision {
                break;
            }
            let mut next = (p * 3) / 2 - 16;
            if let Some(target) = algebraic_zero_precision
                && target < p
                && next < target
            {
                next = target;
            }
            p = if next < min_precision {
                min_precision
            } else {
                next
            };
            if should_stop(&self.signal) {
                break;
            }
        }

        if self
            .exact_rational()
            .is_some_and(|r| r.sign() == Sign::NoSign)
        {
            crate::trace_dispatch!("computable", "sign_until", "exact-rational-zero");
            Some(RealSign::Zero)
        } else {
            #[cfg(feature = "dispatch-trace")]
            crate::dispatch_trace::record_sign_refinement_precision(
                "sign_until_unresolved_precision",
                p,
            );
            crate::trace_dispatch!("computable", "sign_until", "unknown");
            None
        }
    }

    /// Clone the finest stored approximation without rescaling or starting evaluation.
    /// Shared constants read their process-wide slot; other nodes read their own cache.
    /// Absence is not a zero or a failed-evaluation certificate.
    fn cached(&self) -> Option<(Precision, BigInt)> {
        if let Some(constant) = self.shared_constant_kind() {
            SHARED_CONSTANT_CACHES[constant.cache_index()].get()
        } else {
            self.internal.cached_value()
        }
    }

    /// Try to compare two computable values exactly.
    ///
    /// Return Some(Equal) for an established equality and None for unresolved
    /// equality/order after the fallback tolerance floor of -4096. Preserve None
    /// when comparing semantically equal but differently represented expressions.
    ///
    /// Structural identity, exact leaf/sign/rational comparisons and exact magnitude
    /// gaps run before the caller-floor path. That path can use shared perturbation
    /// proofs and further bounds before numerical comparison. This duplicated fast
    /// prefix and its predicate order are intentional. Cached finer evidence remains
    /// usable; the default floor is not a total-work, allocation or cancellation bound.
    /// Evaluation must complete within the underlying domains for numerical decisions
    /// to be certified; interruption inherits approx_signal's unqualified return behavior.
    pub fn try_compare_to(&self, other: &Self) -> Option<Ordering> {
        // Keep the default entry point's certified fast cascade in this
        // function. Routing these overwhelmingly common cases through the
        // caller-configurable API would add an extra entry-point call.
        if Self::internal_structural_eq(self, other) {
            return Some(Ordering::Equal);
        }
        if let Some(order) = self.exact_rational_leaf_cmp(other) {
            crate::trace_dispatch!("computable", "compare_to", "exact-rational");
            return Some(order);
        }
        let exact_signs = (self.exact_sign(), other.exact_sign());
        if let (Some(left), Some(right)) = exact_signs {
            match (left, right) {
                (Sign::Minus, Sign::Plus | Sign::NoSign) | (Sign::NoSign, Sign::Plus) => {
                    crate::trace_dispatch!("computable", "compare_to", "exact-sign-opposite");
                    return Some(Ordering::Less);
                }
                (Sign::Plus, Sign::Minus | Sign::NoSign) | (Sign::NoSign, Sign::Minus) => {
                    crate::trace_dispatch!("computable", "compare_to", "exact-sign-opposite");
                    return Some(Ordering::Greater);
                }
                _ => {}
            }
        }
        if let (Some(left), Some(right)) = (self.exact_rational(), other.exact_rational()) {
            crate::trace_dispatch!("computable", "compare_to", "exact-rational");
            return Some(
                left.partial_cmp(&right)
                    .expect("exact rationals should be comparable"),
            );
        }
        if let (Some(left), Some(right)) = exact_signs
            && matches!(left, Sign::Plus | Sign::Minus)
            && left == right
            && let (Some(Some(left_msd)), Some(Some(right_msd))) = (
                self.cheap_bound().known_msd(),
                other.cheap_bound().known_msd(),
            )
            && left_msd != right_msd
        {
            crate::trace_dispatch!("computable", "compare_to", "cheap-bound-msd-gap");
            return Some(match left {
                Sign::Plus => left_msd.cmp(&right_msd),
                Sign::Minus => right_msd.cmp(&left_msd),
                Sign::NoSign => Ordering::Equal,
            });
        }
        self.try_compare_to_until(other, DEFAULT_COMPARE_REFINEMENT_FLOOR)
    }

    /// Try exact ordering, then lower comparison tolerances as far as min_precision.
    ///
    /// Structural equality, exact leaves/shared perturbations, signs, rationals and
    /// bounds may decide without refinement. Otherwise start at max(-20,min_precision),
    /// subtract sixteen while nonnegative and double negative tolerances, saturating
    /// and clamping to the floor. A separated comparison returns order; unresolved
    /// comparison at the floor returns None, never an equality guess.
    ///
    /// The floor applies to comparison tolerance: compare_absolute may request an
    /// approximation at tolerance-1, and its kernels may need further guard bits.
    /// This does not bound all nested precision, work or memory. There is no dedicated
    /// stop poll in this loop or typed domain/overflow/cancellation result. Existing
    /// proofs can decide beyond the precision of the current request; numerical
    /// guarantees require completed evaluation with valid intermediate exponents.
    pub fn try_compare_to_until(&self, other: &Self, min_precision: Precision) -> Option<Ordering> {
        if Self::internal_structural_eq(self, other) {
            return Some(Ordering::Equal);
        }

        // Borrow exact leaves without constructing temporary rational wrappers.
        // The underlying rational comparison may still allocate arithmetic temporaries.
        if let Some(order) = self.exact_rational_leaf_cmp(other) {
            crate::trace_dispatch!("computable", "compare_to", "exact-rational");
            return Some(order);
        }

        if let Some(order) = self.exact_shared_perturbation_order(other) {
            crate::trace_dispatch!("computable", "compare_to", "exact-shared-perturbation");
            return Some(order);
        }

        let exact_signs = (self.exact_sign(), other.exact_sign());
        if let (Some(left), Some(right)) = exact_signs {
            match (left, right) {
                (Sign::Minus, Sign::Plus | Sign::NoSign) | (Sign::NoSign, Sign::Plus) => {
                    crate::trace_dispatch!("computable", "compare_to", "exact-sign-opposite");
                    return Some(Ordering::Less);
                }
                (Sign::Plus, Sign::Minus | Sign::NoSign) | (Sign::NoSign, Sign::Minus) => {
                    crate::trace_dispatch!("computable", "compare_to", "exact-sign-opposite");
                    return Some(Ordering::Greater);
                }
                _ => {}
            }
        }

        if let (Some(left), Some(right)) = (self.exact_rational(), other.exact_rational()) {
            // Exact rationals compare directly; escalating to approximate comparison here is
            // both slower and can burn cache precision unnecessarily.
            crate::trace_dispatch!("computable", "compare_to", "exact-rational");
            return Some(
                left.partial_cmp(&right)
                    .expect("exact rationals should be comparable"),
            );
        }

        if let (Some(left), Some(right)) = exact_signs
            && matches!(left, Sign::Plus | Sign::Minus)
            && left == right
            && let (Some(Some(left_msd)), Some(Some(right_msd))) = (
                self.cheap_bound().known_msd(),
                other.cheap_bound().known_msd(),
            )
            && left_msd != right_msd
        {
            // Same-sign values with different most-significant digits have a known
            // order without evaluating either value to a requested precision.
            crate::trace_dispatch!("computable", "compare_to", "exact-sign-msd-gap");
            return Some(match left {
                Sign::Plus => left_msd.cmp(&right_msd),
                Sign::Minus => right_msd.cmp(&left_msd),
                Sign::NoSign => unreachable!(),
            });
        }

        let self_bound = self.cheap_bound();
        let other_bound = other.cheap_bound();
        let self_bound_sign = self_bound.known_sign();
        let other_bound_sign = other_bound.known_sign();
        if let (Some(left), Some(right)) = (self_bound_sign, other_bound_sign) {
            match (left, right) {
                (Sign::Minus, Sign::NoSign | Sign::Plus) | (Sign::NoSign, Sign::Plus) => {
                    crate::trace_dispatch!("computable", "compare_to", "cheap-bound-opposite-sign");
                    return Some(Ordering::Less);
                }
                (Sign::Plus, Sign::Minus | Sign::NoSign) | (Sign::NoSign, Sign::Minus) => {
                    crate::trace_dispatch!("computable", "compare_to", "cheap-bound-opposite-sign");
                    return Some(Ordering::Greater);
                }
                (Sign::NoSign, Sign::NoSign) => return Some(Ordering::Equal),
                _ => {}
            }
            if left == right
                && let (Some(Some(left_msd)), Some(Some(right_msd))) =
                    (self_bound.known_msd(), other_bound.known_msd())
                && left_msd != right_msd
            {
                // Same-sign structural bounds can decide exact ordering
                // before entering tolerance refinement.
                crate::trace_dispatch!("computable", "compare_to", "cheap-bound-msd-gap");
                return Some(match left {
                    Sign::Plus => left_msd.cmp(&right_msd),
                    Sign::Minus => right_msd.cmp(&left_msd),
                    Sign::NoSign => Ordering::Equal,
                });
            }
        }
        crate::trace_dispatch!("computable", "compare_to", "approx-refinement");
        let mut tolerance = (-20).max(min_precision);
        loop {
            let order = self.compare_absolute(other, tolerance);
            if order != Ordering::Equal {
                return Some(order);
            }
            if tolerance <= min_precision {
                return None;
            }
            let next = if tolerance >= 0 {
                tolerance.saturating_sub(16)
            } else {
                tolerance.saturating_mul(2)
            };
            tolerance = next.max(min_precision);
        }
    }

    /// Compare two values to a specified absolute tolerance.
    ///
    /// More negative tolerances are more precise. `Equal` generally means that
    /// the requested tolerance did not separate the values; it is not an exact
    /// equality certificate. Predicate callers must use
    /// [`Computable::try_compare_to_until`] and preserve `None`.
    ///
    /// Compare signed values, not their absolute magnitudes. Exact leaves and
    /// sign/magnitude proofs can distinguish arbitrarily close inputs even at a
    /// coarse tolerance; a shared perturbation below tolerance may instead return
    /// Equal. Thus this is not a uniform approximate-equivalence relation.
    ///
    /// The numerical fallback requests both integers at tolerance-1 and returns
    /// order when their difference exceeds one integer unit, otherwise Equal.
    /// Tolerance is a binary exponent, not a work limit or a bound on all internal
    /// precision. In particular, subtracting one from i32::MIN is unchecked here
    /// and overflows on that fallback path; there is no typed error. Each operand
    /// uses its own attached signal and interrupted kernel results carry no receipt.
    /// Numerical order claims require valid, completed evaluation.
    pub fn compare_absolute(&self, other: &Self, tolerance: Precision) -> Ordering {
        // Fast-path exact leafs before structural perturbation checks.
        if let Some(order) = self.exact_rational_leaf_cmp(other) {
            crate::trace_dispatch!("computable", "compare_absolute", "exact-rational");
            return order;
        }

        if let Approximation::Add(left, right) = &self.internal.approximation
            && let Some(order) = if Self::internal_structural_eq(left, other) {
                crate::trace_dispatch!(
                    "computable",
                    "compare_absolute",
                    "dominant-perturbation-self"
                );
                Self::dominant_perturbation_order(left, right, other, Some(tolerance))
            } else if Self::internal_structural_eq(right, other) {
                crate::trace_dispatch!(
                    "computable",
                    "compare_absolute",
                    "dominant-perturbation-self-reversed"
                );
                Self::dominant_perturbation_order(right, left, other, Some(tolerance))
            } else {
                None
            }
        {
            return order;
        }
        if let Approximation::Add(left, right) = &other.internal.approximation
            && let Some(order) = if Self::internal_structural_eq(left, self) {
                crate::trace_dispatch!(
                    "computable",
                    "compare_absolute",
                    "dominant-perturbation-other"
                );
                Self::dominant_perturbation_order(left, right, self, Some(tolerance))
            } else if Self::internal_structural_eq(right, self) {
                crate::trace_dispatch!(
                    "computable",
                    "compare_absolute",
                    "dominant-perturbation-other-reversed"
                );
                Self::dominant_perturbation_order(right, left, self, Some(tolerance))
            } else {
                None
            }
        {
            return order.reverse();
        }

        if let (Some(left), Some(right)) = (self.exact_rational(), other.exact_rational()) {
            // Compare exact rationals without normalizing both operands. The
            // method name refers to its absolute error tolerance, not to
            // absolute values: opposite signs and comparisons with zero must
            // therefore be decided by signed order before comparing magnitudes.
            crate::trace_dispatch!("computable", "compare_absolute", "exact-rational");
            return match (left.sign(), right.sign()) {
                (Sign::Minus, Sign::Minus) => right.compare_magnitude(&left),
                (Sign::Minus, Sign::NoSign | Sign::Plus) => Ordering::Less,
                (Sign::NoSign, Sign::Minus) => Ordering::Greater,
                (Sign::NoSign, Sign::NoSign) => Ordering::Equal,
                (Sign::NoSign, Sign::Plus) => Ordering::Less,
                (Sign::Plus, Sign::Minus | Sign::NoSign) => Ordering::Greater,
                (Sign::Plus, Sign::Plus) => left.compare_magnitude(&right),
            };
        }

        let self_sign = self.exact_sign();
        let other_sign = other.exact_sign();
        match (self_sign, other_sign) {
            // Exact signs can order zero and opposite-signed values directly.
            (Some(Sign::Minus), Some(Sign::NoSign | Sign::Plus))
            | (Some(Sign::NoSign), Some(Sign::Plus)) => return Ordering::Less,
            (Some(Sign::Plus), Some(Sign::Minus | Sign::NoSign))
            | (Some(Sign::NoSign), Some(Sign::Minus)) => return Ordering::Greater,
            (Some(Sign::NoSign), Some(Sign::NoSign)) => return Ordering::Equal,
            _ => {}
        }

        // Keep bound derivation lazy: only ask cheap_bound when exact sign facts
        // cannot already determine the ordering.
        if self_sign.is_none() || other_sign.is_none() {
            let self_bound = self.cheap_bound();
            let other_bound = other.cheap_bound();
            let self_structural_sign = self_sign.or(self_bound.known_sign());
            let other_structural_sign = other_sign.or(other_bound.known_sign());
            let self_msd = self_bound.known_msd();
            let other_msd = other_bound.known_msd();

            if let (BoundInfo::Zero, BoundInfo::Zero) = (&self_bound, &other_bound) {
                return Ordering::Equal;
            }
            match (self_structural_sign, other_structural_sign) {
                (Some(Sign::Minus), Some(Sign::NoSign | Sign::Plus))
                | (Some(Sign::NoSign), Some(Sign::Plus)) => return Ordering::Less,
                (Some(Sign::Plus), Some(Sign::Minus | Sign::NoSign))
                | (Some(Sign::NoSign), Some(Sign::Minus)) => return Ordering::Greater,
                (Some(Sign::NoSign), Some(Sign::NoSign)) => return Ordering::Equal,
                _ => {}
            }
            if let (Some(left_sign), Some(right_sign), Some(left_msd), Some(right_msd)) = (
                self_structural_sign,
                other_structural_sign,
                self_msd,
                other_msd,
            ) && left_msd != right_msd
            {
                crate::trace_dispatch!("computable", "compare_absolute", "exact-sign-msd-gap");
                match (left_sign, right_sign) {
                    (Sign::Plus, Sign::Plus) => return left_msd.cmp(&right_msd),
                    (Sign::Minus, Sign::Minus) => return right_msd.cmp(&left_msd),
                    _ => {}
                }
            }
            // A known magnitude without a known sign cannot order values.
            // Same-sign values with exact, distinct MSDs already returned
            // above with the correct negative-value reversal.
        } else if self_sign == other_sign {
            let self_bound = self.cheap_bound();
            let other_bound = other.cheap_bound();
            if let (Some(Some(self_msd)), Some(Some(other_msd))) =
                (self_bound.known_msd(), other_bound.known_msd())
                && let (Some(left_sign), Some(right_sign)) = (self_sign, other_sign)
                && left_sign == right_sign
                && self_msd != other_msd
            {
                crate::trace_dispatch!("computable", "compare_absolute", "exact-sign-msd-gap");
                return match (left_sign, right_sign) {
                    (Sign::Plus, Sign::Plus) => self_msd.cmp(&other_msd),
                    (Sign::Minus, Sign::Minus) => other_msd.cmp(&self_msd),
                    _ => Ordering::Equal,
                };
            }
        }
        crate::trace_dispatch!("computable", "compare_absolute", "approx-refinement");
        let needed = tolerance - 1;
        let this = self.approx(needed);
        let alt = other.approx(needed);
        let max = alt.clone() + signed::ONE.deref();
        let min = alt.clone() - signed::ONE.deref();
        if this > max {
            Ordering::Greater
        } else if this < min {
            Ordering::Less
        } else {
            Ordering::Equal
        }
    }

    #[inline]
    /// Compare directly borrowed Ratio/Ratio, Int/Int, One/One and One/Int pairs.
    /// Other tag combinations decline for the later exact-rational path. Rational
    /// comparison can allocate arithmetic temporaries; no new wrapper rationals are built.
    fn exact_rational_leaf_cmp(&self, other: &Self) -> Option<Ordering> {
        match (&self.internal.approximation, &other.internal.approximation) {
            (Approximation::Ratio(left), Approximation::Ratio(right)) => left.partial_cmp(right),
            (Approximation::Int(left), Approximation::Int(right)) => Some(left.cmp(right)),
            (Approximation::One, Approximation::One) => Some(Ordering::Equal),
            (Approximation::One, Approximation::Int(right)) => Some(BigInt::one().cmp(right)),
            (Approximation::Int(left), Approximation::One) => Some(left.cmp(&BigInt::one())),
            _ => None,
        }
    }

    /// Return the cached integer's leading-bit exponent translated by its precision.
    /// The caller must already have a cache separated from zero. This helper
    /// computes `bit_length(abs(integer))-1+precision`, not a new exact-binade
    /// proof for the represented real near a power-of-two boundary.
    /// Panics when the cache is absent or the resulting exponent is outside i32.
    fn known_msd(&self) -> Precision {
        if let Some((prec, appr)) = self.cached() {
            crate::verified::word::checked_bit_length_msd(appr.magnitude().bits(), 1, false, prec)
                .expect("Cached magnitude is outside the precision range")
        } else {
            panic!("Expected valid cache state for known MSD but it's invalid")
        }
    }

    /// Obtain an exact structural magnitude or an approximation-derived leading-bit estimate.
    /// A known zero bound returns None immediately. Otherwise reuse a cache with
    /// integer magnitude above one, or request p-1 once to separate it from zero;
    /// an ambiguous result returns None. A cache-based exponent need not be the
    /// exact binade at a boundary, unlike a bound marked exact. None combines known
    /// zero and unresolved magnitude; it is not by itself a zero certificate.
    /// Cache publication, signal handling and representable precision assumptions
    /// are inherited from approximation; this helper has no typed failure receipt.
    pub(crate) fn msd(&self, p: Precision) -> Option<Precision> {
        if let Some(msd) = self.cheap_bound().known_msd() {
            return msd;
        }

        let cache = self.cached();
        let mut try_once = false;

        if cache.is_none() {
            try_once = true;
        } else if let Some((_prec, appr)) = cache {
            let one = signed::ONE.deref();
            let minus_one = signed::MINUS_ONE.deref();

            // Approximation results carry an absolute error of at most one at
            // their scale. Magnitude one therefore does not prove a leading
            // bit; refine until the approximation is separated from zero.
            if appr >= *minus_one && appr <= *one {
                try_once = true;
            }
        }

        if try_once {
            let appr = self.approx(p - 1);
            if appr.magnitude() <= &BigUint::one() {
                return None;
            }
        }

        Some(self.known_msd())
    }

    /// Default sentinel/floor for magnitude search, leaving headroom for its precision recurrence.
    /// It is neither a smallest nonzero magnitude nor a total resource limit; iter_msd
    /// uses the same value to represent an unresolved search.
    const STOP_PRECISION: Precision = Precision::MIN / 3;

    /// Search for a magnitude at requests 0, -16, -40, -76, then finally p when the next step reaches it.
    /// Always start at zero, even if p is positive. Poll the attached signal between
    /// steps, but still perform the final msd(p) call after a stop or floor break.
    /// Each msd request can evaluate one bit finer and consume finer cached evidence.
    /// None means unresolved or known zero; this is not a cancellation receipt or
    /// total resource bound, and extreme caller floors are not uniformly checked.
    pub(crate) fn iter_msd_stop(&self, p: Precision) -> Option<Precision> {
        let mut prec = 0;

        loop {
            let msd = self.msd(prec);
            if msd.is_some() {
                return msd;
            }
            prec = (prec * 3) / 2 - 16;
            if prec <= p {
                break;
            }
            if should_stop(&self.signal) {
                break;
            }
        }
        self.msd(p)
    }

    /// Search using STOP_PRECISION and return that sentinel when no magnitude is obtained.
    /// The plain integer result does not distinguish a real exponent equal to the
    /// sentinel from known zero or unresolved search. Arithmetic-kernel consumers
    /// use it for precision planning, not as an independent nonzero certificate.
    pub(super) fn iter_msd(&self) -> Precision {
        self.iter_msd_stop(Self::STOP_PRECISION)
            .unwrap_or(Self::STOP_PRECISION)
    }
}
