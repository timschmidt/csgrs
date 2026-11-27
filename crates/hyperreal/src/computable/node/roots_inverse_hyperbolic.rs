/// Retained roots, inverse-angle reductions and inverse-hyperbolic compositions.
/// Native construction assumes valid real operands; checked Real entry points
/// own domain admission. Branch recognition, approximation and cancellation
/// remain distinct, and no finite expression guarantees bounded total work.
impl Computable {
    /// Retain the unique geodetic latitude for a finite isometric latitude and log axis ratio.
    /// `self` is psi and `log_axis_ratio` is ln(b/a), encoding a positive axis ratio
    /// when the child is defined and finite. The result lies in `(-pi/2,pi/2)` and
    /// has psi's sign. See [`crate::Real::inverse_isometric_latitude`] for the
    /// oblate/prolate defining equations. No equation solving or domain validation
    /// occurs here. Copy a known sign and prefer self's signal over the ratio's;
    /// retain both exact children for subsequent refinement.
    pub(crate) fn inverse_isometric_latitude(self, log_axis_ratio: Self) -> Self {
        let exact_sign = self
            .exact_sign()
            .map_or(ExactSignCache::Invalid, ExactSignCache::Valid);
        let signal = self
            .signal
            .clone()
            .or_else(|| log_axis_ratio.signal.clone());
        Self {
            internal: Arc::new(Node::new(
                Approximation::InverseIsometricLatitude {
                    psi: self,
                    log_axis_ratio,
                },
                BoundCache::Invalid,
                exact_sign,
            )),
            signal,
        }
    }

    /// Borrow the immediate child of a retained Square node, otherwise return None.
    /// This syntactic inspection performs no refinement or algebraic recognition;
    /// a product of equal children or a square-valued expression need not match.
    pub(super) fn square_operand(&self) -> Option<&Computable> {
        match &self.internal.approximation {
            Approximation::Square(child) => Some(child),
            _ => None,
        }
    }

    /// Largest degree scheduled as a direct nonnegative NthRoot node (minimum three).
    /// Native replay and evaluation use this implementation bound; larger Real
    /// roots have a rational-exponent path. It is not a mathematical degree limit.
    pub(crate) const MAX_DIRECT_NTH_ROOT_DEGREE: u32 = 9;

    /// Construct the nonnegative square root of an admitted nonnegative rational.
    /// Route through the ordinary rational and sqrt constructors so recognized
    /// perfect squares and scaled shared radicals can simplify. Negative inputs
    /// are outside this unchecked helper's contract; no typed domain error is returned.
    pub(crate) fn sqrt_rational(r: Rational) -> Self {
        // Preserve the rational leaf so sqrt can still collapse perfect
        // rational squares before allocating a generic Sqrt node.
        let rational = Self::rational(r);
        Self::sqrt(rational)
    }

    /// Wrap a previously decomposed nonnegative rational radicand directly in Sqrt.
    /// Real's retained radical classes use this to avoid repeating extraction.
    /// The name assumes the caller's decomposition; this helper neither verifies
    /// squarefreeness nor collapses perfect squares. A debug assertion rejects
    /// negative input, but release construction does not validate it. Initialize
    /// zero sign for zero, positive sign otherwise, unknown magnitude and no signal.
    pub(crate) fn sqrt_squarefree_rational(radicand: Rational) -> Self {
        debug_assert!(radicand.sign() != Sign::Minus);
        let child = Self::rational(radicand);
        let exact_sign = match child.internal.facts.exact_sign() {
            ExactSignCache::Valid(Sign::NoSign) => ExactSignCache::Valid(Sign::NoSign),
            _ => ExactSignCache::Valid(Sign::Plus),
        };
        Self {
            internal: Arc::new(Node::new(
                Approximation::Sqrt(child),
                BoundCache::Invalid,
                exact_sign,
            )),
            signal: None,
        }
    }

    /// Retain the principal nonnegative square root of a caller-admitted nonnegative value.
    ///
    /// Collapse a retained square to the absolute value of its child when the sign
    /// is known, and recognize exact-square rational scales around such squares.
    /// Try rational square extraction before retaining a generic Sqrt. Scaled
    /// sqrt(2)/sqrt(3) residuals can reuse named constants; plain sqrt(2)/sqrt(3)
    /// retain their ordinary node. An unresolved child sign keeps the square root
    /// rather than choosing an unjustified sign.
    ///
    /// This native constructor does not return typed negative-domain errors; use
    /// [`crate::Real::sqrt`] for checked admission. Its sign facts presuppose a
    /// valid radicand. A generic node has no outer signal, and simplification paths
    /// follow the returned child's/composition's controls rather than promising
    /// transport of the source wrapper's signal. There is no total-work budget.
    pub fn sqrt(self) -> Computable {
        if let Approximation::Square(child) = &self.internal.approximation {
            // sqrt(x^2) can collapse to abs(x) when the sign is structurally known.
            match child.exact_sign() {
                Some(Sign::Plus) => {
                    crate::trace_dispatch!("computable", "sqrt", "square-positive-collapse");
                    return child.clone();
                }
                Some(Sign::Minus) => {
                    crate::trace_dispatch!("computable", "sqrt", "square-negative-abs-collapse");
                    return child.clone().negate();
                }
                Some(Sign::NoSign) => {
                    crate::trace_dispatch!("computable", "sqrt", "square-zero-collapse");
                    return Self::zero();
                }
                None => {}
            }
        }
        if let Approximation::Multiply(left, right) = &self.internal.approximation {
            let reduced = |scale: Rational, square_side: &Computable| {
                // Recognize c*x^2 where c is an exact square, preserving the symbolic x
                // instead of introducing a generic sqrt node.
                let (root, rest) = scale.extract_square_reduced();
                if !rest.is_one() {
                    return None;
                }
                let Approximation::Square(child) = &square_side.internal.approximation else {
                    return None;
                };
                match child.exact_sign() {
                    Some(Sign::Plus) => Some(child.clone().multiply(Self::rational(root))),
                    Some(Sign::Minus) => {
                        Some(child.clone().negate().multiply(Self::rational(root)))
                    }
                    Some(Sign::NoSign) => Some(Self::zero()),
                    None => None,
                }
            };

            if let Some(scale) = left.exact_rational()
                && let Some(value) = reduced(scale, right)
            {
                crate::trace_dispatch!("computable", "sqrt", "scaled-square-collapse");
                return value;
            }
            if let Some(scale) = right.exact_rational()
                && let Some(value) = reduced(scale, left)
            {
                crate::trace_dispatch!("computable", "sqrt", "scaled-square-collapse");
                return value;
            }
        }
        if let Some(rational) = self.exact_rational()
            && rational.sign() != Sign::Minus
            && rational.extract_square_will_succeed()
        {
            // Perfect rational squares stay exact. For scaled sqrt(2)/sqrt(3)
            // residuals, keep the irrational part shared and the exact scale
            // symbolic. Plain sqrt(2) and sqrt(3) deliberately stay on the
            // generic node; named radical constants use the separate process-wide
            // approximation cache.
            let (root, rest) = rational.extract_square_reduced();
            if rest.is_one() {
                crate::trace_dispatch!("computable", "sqrt", "exact-rational-square");
                return Self::rational(root);
            }
            if !root.is_one()
                && let Some(shared_radicand @ (2 | 3)) = rest.to_integer_i64()
            {
                // For scaled sqrt(2)/sqrt(3), reuse the shared constant cache
                // for the irrational factor and keep the exact rational scale
                // separate. This is a measured construction-time win for scaled
                // square-free inputs without changing the single-node path for
                // plain sqrt(2)/sqrt(3).
                crate::trace_dispatch!("computable", "sqrt", "shared-squarefree-rational");
                let constant = Self::sqrt_constant(shared_radicand)
                    .expect("sqrt(2) and sqrt(3) are shared constants");
                return constant.multiply(Self::rational(root));
            }
        }
        crate::trace_dispatch!("computable", "sqrt", "generic-sqrt-node");
        let exact_sign = match self.internal.facts.exact_sign() {
            // Square roots are nonnegative where defined and preserve structural zero.
            ExactSignCache::Valid(Sign::NoSign) => ExactSignCache::Valid(Sign::NoSign),
            ExactSignCache::Valid(Sign::Plus) | ExactSignCache::Valid(Sign::Minus) => {
                ExactSignCache::Valid(Sign::Plus)
            }
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(
                Approximation::Sqrt(self),
                BoundCache::Invalid,
                exact_sign,
            )),
            signal: None,
        }
    }

    /// Retain a nonnegative nth root for an admitted nonnegative child and degree 3 through 9.
    /// Negative odd roots are handled by Real's sign symmetry outside this node.
    /// A debug assertion checks the degree here; native replay and the evaluator
    /// have their own degree checks. This implementation range is not a mathematical
    /// completeness or total-resource bound. Copy only already-stored zero/positive
    /// sign facts, leave magnitude unknown and attach no outer signal.
    pub(crate) fn nth_root(self, degree: u32) -> Computable {
        debug_assert!((3..=Self::MAX_DIRECT_NTH_ROOT_DEGREE).contains(&degree));
        let exact_sign = match self.internal.facts.exact_sign() {
            ExactSignCache::Valid(Sign::NoSign) => ExactSignCache::Valid(Sign::NoSign),
            ExactSignCache::Valid(Sign::Plus) => ExactSignCache::Valid(Sign::Plus),
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Arc::new(Node::new(
                Approximation::NthRoot(self, degree),
                BoundCache::Invalid,
                exact_sign,
            )),
            signal: None,
        }
    }

    /// Retain principal `atan(1/n)` from an exact nonzero integer denominator.
    /// Machin pi and arctangent reductions use this payload to avoid a rational
    /// child and perform division-only series work. Magnitudes at least two use
    /// the series; unit denominators have a separate evaluator identity. No
    /// nonzero check occurs here, and sign/magnitude start unknown with no signal.
    pub(crate) fn prescaled_atan(n: BigInt) -> Self {
        // atan(1/n) kernel used by pi and atan reduction constants. Passing the
        // denominator as an integer keeps the series loop division-only.
        Self {
            internal: Arc::new(Node::new(
                Approximation::IntegralAtan(n),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Retain principal atan of any exact rational for evaluation-time anchor reduction.
    /// Avoid allocating the intermediate quotient/reduction graph during construction.
    /// The rational remains authoritative; sign/magnitude start unknown and the
    /// fresh wrapper has no signal. Deferral supplies no total-work or size budget.
    fn atan_rational_deferred(rational: Rational) -> Self {
        // Exact rational atan reductions used to allocate intermediate
        // add/multiply/inverse nodes before reaching the small atan series. This
        // deferred node keeps the public constructor compact and performs the
        // same range reductions directly in the approximation kernel.
        crate::trace_dispatch!("computable", "constructor", "atan-rational-deferred");
        Self {
            internal: Arc::new(Node::new(
                Approximation::AtanRational(rational),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Retain principal atan of a defined finite child without eagerly reducing its range.
    /// Copy a known exact sign and the child's outer signal, leaving magnitude
    /// unknown. Evaluation invokes atan_reduced on the retained source; no numerical
    /// sample or uniform convergence-cost bound is supplied by this constructor.
    fn atan_deferred(value: Computable) -> Self {
        crate::trace_dispatch!("computable", "constructor", "atan-deferred");
        let sign = value.exact_sign();
        let signal = value.signal.clone();
        Self {
            internal: Arc::new(Node::new(
                Approximation::AtanDeferred(value),
                BoundCache::Invalid,
                sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid),
            )),
            signal,
        }
    }

    /// Retain principal asin of a caller-admitted rational in `[-1,1]`.
    /// Keep the signed exact payload for the evaluator's tiny series or
    /// endpoint-stable acos complement. Initialize its exact sign and unknown
    /// magnitude, without checking the domain or attaching a signal.
    fn asin_rational_deferred(rational: Rational) -> Self {
        // Exact rational asin stores the signed input directly. Tiny values use
        // the odd series, while larger values select the endpoint-stable acos
        // complement inside the approximation kernel. Keeping both schedules
        // behind one node avoids constructing and then negating a pi/2 graph.
        crate::trace_dispatch!("computable", "constructor", "asin-rational-deferred");
        let sign = rational.sign();
        Self {
            internal: Arc::new(Node::new(
                Approximation::AsinRational(rational),
                BoundCache::Invalid,
                ExactSignCache::Valid(sign),
            )),
            signal: None,
        }
    }

    /// Retain principal arctangent in `(-pi/2,pi/2)` for a defined finite real input.
    /// Recognize the sqrt(35)/35 asin anchor and supported sqrt(3) pi anchors,
    /// then exact rationals, including a bounded Laurent recognition attempt.
    /// Zero and +/-one yield exact angular constants; other rationals use a
    /// deferred node with negative symmetry. Remaining inputs retain a deferred
    /// source and its signal. Constant/rational shortcuts do not promise to retain
    /// the original outer signal. No numerical range reduction is forced here,
    /// and the recognition limits do not bound all future evaluation work.
    pub fn atan(self) -> Computable {
        if let Some((scale, radicand)) = self.exact_pure_quadratic_surd()
            && radicand == Rational::new(35)
        {
            let magnitude = if scale.sign() == Sign::Minus {
                scale.clone().neg()
            } else {
                scale.clone()
            };
            if magnitude == Rational::fraction(1, 35).expect("thirty-five is nonzero") {
                let argument =
                    Rational::fraction(if scale.sign() == Sign::Minus { -1 } else { 1 }, 6)
                        .expect("six is nonzero");
                crate::trace_dispatch!("computable", "atan", "quadratic-surd-asin-anchor");
                return Self::asin_rational_deferred(argument);
            }
        }
        if let Some((scale, radicand)) = self.exact_pure_quadratic_surd()
            && radicand == Rational::new(3)
        {
            let magnitude = if scale.sign() == Sign::Minus {
                scale.clone().neg()
            } else {
                scale.clone()
            };
            if let Some(anchor) = AtanPiAnchor::from_root_three_scale(&magnitude) {
                crate::trace_dispatch!("computable", "atan", "quadratic-surd-pi-anchor");
                return anchor.angle(scale.sign());
            }
        }
        if let Some(rational) = self
            .exact_rational()
            .or_else(|| self.bounded_laurent_rational(48))
        {
            if rational.sign() == Sign::NoSign {
                crate::trace_dispatch!("computable", "atan", "exact-zero");
                return Self::zero();
            }
            if rational.is_one() || rational.is_minus_one() {
                crate::trace_dispatch!("computable", "atan", "exact-quarter-turn");
                return AtanPiAnchor::Unit.angle(rational.sign());
            }
            match rational.sign() {
                Sign::Plus => {
                    crate::trace_dispatch!("computable", "atan", "exact-rational-deferred");
                    return Self::atan_rational_deferred(rational);
                }
                Sign::Minus => {
                    crate::trace_dispatch!("computable", "atan", "negative-rational-deferred");
                    return Self::atan_rational_deferred(rational.neg()).negate();
                }
                Sign::NoSign => unreachable!("zero rational returned above"),
            }
        }
        Self::atan_deferred(self)
    }

    /// Choose an exact convergence reduction for a defined finite arctangent source.
    /// Rationals delegate to atan. Normalize a known negative sign, then use only
    /// a certified small binade or a certified large binade with positive sign
    /// for structural series/reciprocal scheduling. Otherwise sample at precision
    /// -4: magnitude at most seven proves `abs(x) <= 1/2` for a completed sample;
    /// a separated negative sample uses odd symmetry. Positive middle inputs use
    /// `atan(1/2) + atan((x-1/2)/(1+x/2))`; larger ones use the reciprocal identity.
    ///
    /// The series evaluator independently checks its working range. Construction
    /// may refine the source using its attached signal, while fresh prescaled
    /// wrappers have no outer signal. Interrupted samples are not certificates;
    /// callers retain responsibility for abort publication. No total-work limit
    /// or typed exhaustion outcome is introduced by this reduction.
    pub(crate) fn atan_reduced(self) -> Computable {
        if self.exact_rational().is_some() {
            return self.atan();
        }
        let bound = self.cheap_bound();
        let known_sign = bound.known_sign();
        if known_sign == Some(Sign::Minus) {
            crate::trace_dispatch!("computable", "atan", "known-negative-symmetry");
            return self.negate().atan().negate();
        }
        if known_sign.is_none() && self.exact_sign() == Some(Sign::Minus) {
            crate::trace_dispatch!("computable", "atan", "known-negative-symmetry-fallback");
            return self.negate().atan().negate();
        }
        // An estimated magnitude is a scheduling hint, not a series-domain
        // certificate: repeated additions can outgrow the retained estimate.
        if let Some(msd) = bound.known_msd().flatten() {
            if msd < -1 {
                crate::trace_dispatch!("computable", "atan", "structural-small-prescaled");
                return Self {
                    internal: Arc::new(Node::new(
                        Approximation::PrescaledAtan(self),
                        BoundCache::Invalid,
                        ExactSignCache::Invalid,
                    )),
                    signal: None,
                };
            }
            if msd >= 5 && known_sign == Some(Sign::Plus) {
                crate::trace_dispatch!("computable", "atan", "large-reciprocal-structural");
                return Self::pi()
                    .shift_right(1)
                    .add(self.inverse().atan().negate());
            }
        }

        let rough_appr = self.approx(-4);
        if rough_appr.magnitude() < signed::EIGHT.magnitude() {
            // |approx(-4)| <= 7 and its one-unit error prove |self| <= 1/2.
            crate::trace_dispatch!("computable", "atan", "rough-small-prescaled");
            return Self {
                internal: Arc::new(Node::new(
                    Approximation::PrescaledAtan(self),
                    BoundCache::Invalid,
                    ExactSignCache::Invalid,
                )),
                signal: None,
            };
        }
        if rough_appr.sign() == Sign::Minus {
            // The small branch handled the possible-zero interval. Here the
            // rough sample certifies negativity, so odd symmetry reaches the
            // positive range reductions without asking for an exact comparison.
            crate::trace_dispatch!("computable", "atan", "rough-negative-symmetry");
            return self.negate().atan().negate();
        }

        let one = Self::one();
        let half = one.clone().shift_right(1);
        if rough_appr <= *signed::SIXTEEN {
            // For middle-sized arguments, subtract atan(1/2) before recursing. This keeps
            // the residual small without jumping all the way to the reciprocal identity.
            let numerator = self.clone().add(half.clone().negate());
            let denominator = one.add(self.multiply(half));
            crate::trace_dispatch!("computable", "atan", "medium-atan-half-reduction");
            return Self::prescaled_atan(BigInt::from(2_u8))
                .add(numerator.multiply(denominator.inverse()).atan());
        }

        // Large positive atan uses pi/2 - atan(1/x), which converges faster.
        crate::trace_dispatch!("computable", "atan", "large-reciprocal");
        Self::pi()
            .shift_right(1)
            .add(self.inverse().atan().negate())
    }

    /// Retain the angle of `(x,self)` counterclockwise from the positive x axis.
    /// The principal range is `(-pi,pi]`, with self denoting y. Axes produce exact
    /// zero, +/-pi/2 or pi; the origin is defined as zero. These real values do not
    /// distinguish IEEE signed-zero cases. Known quadrants use atan(y/x) with a
    /// pi correction, while a known nonzero y permits a half-angle fallback when
    /// x's sign remains unresolved.
    ///
    /// Use the sign-refinement floor -4096 through [`Self::try_atan2_until`]. That
    /// floor is not a total-work/memory bound. Both inputs must be defined finite
    /// reals; this constructor has no typed domain or cancellation receipt, and
    /// axis constants do not inherit operand signals.
    ///
    /// # Panics
    ///
    /// Panics if the bounded sign search and half-angle fallback cannot establish
    /// an applicable branch. Use [`Self::try_atan2_until`] to retain that uncertainty.
    #[inline]
    pub fn atan2(self, x: Computable) -> Computable {
        self.try_atan2_until(x, ATAN2_SIGN_REFINEMENT_FLOOR).expect(
            "atan2 quadrant could not be certified; use Computable::try_atan2_until to handle uncertainty",
        )
    }

    /// Construct principal atan2 with a chosen binary sign-refinement floor.
    /// Inspect structural signs and exact axes first. Refine x only if needed;
    /// positive x needs no y sign, whereas negative/zero x requires y's sign.
    /// When x remains unresolved, an already-known nonzero y permits the exact
    /// half-angle identity without selecting a quadrant from a rounded sample.
    /// Otherwise return None, never an axis value inferred from an unknown sign.
    ///
    /// For defined finite operands, successful results have the same principal
    /// branch and zero-origin convention as [`Self::atan2`]. The floor limits sign
    /// refinement precision, not total work, allocations or cancellation latency.
    /// There is no separate abort receipt or domain validation; result controls
    /// follow the chosen composition and axis constants have no attached signal.
    #[inline]
    pub fn try_atan2_until(self, x: Computable, min_precision: Precision) -> Option<Computable> {
        let y_sign = self.structural_facts().sign.map(private_sign);
        let x_sign = x.structural_facts().sign.map(private_sign);
        match (y_sign, x_sign) {
            (Some(Sign::NoSign), Some(Sign::NoSign)) | (Some(Sign::NoSign), Some(Sign::Plus)) => {
                crate::trace_dispatch!("computable", "atan2", "axis-zero-y");
                return Some(Self::zero());
            }
            (Some(Sign::NoSign), Some(Sign::Minus)) => {
                crate::trace_dispatch!("computable", "atan2", "axis-negative-x");
                return Some(Self::pi());
            }
            (Some(Sign::Plus), Some(Sign::NoSign)) => {
                crate::trace_dispatch!("computable", "atan2", "axis-positive-y");
                return Some(Self::pi().shift_right(1));
            }
            (Some(Sign::Minus), Some(Sign::NoSign)) => {
                crate::trace_dispatch!("computable", "atan2", "axis-negative-y");
                return Some(Self::pi().shift_right(1).negate());
            }
            _ => {}
        }

        let x_sign = x_sign.or_else(|| x.sign_until(min_precision).map(private_sign));
        match x_sign {
            Some(Sign::Plus) => {
                crate::trace_dispatch!("computable", "atan2", "quadrant-right");
                Some(self.multiply(x.inverse()).atan())
            }
            Some(Sign::Minus) => {
                let y_sign = y_sign.or_else(|| self.sign_until(min_precision).map(private_sign))?;
                let base = self.multiply(x.inverse()).atan();
                match y_sign {
                    Sign::NoSign => {
                        crate::trace_dispatch!("computable", "atan2", "axis-negative-x");
                        Some(Self::pi())
                    }
                    Sign::Plus => {
                        crate::trace_dispatch!("computable", "atan2", "quadrant-upper-left");
                        Some(base.add(Self::pi()))
                    }
                    Sign::Minus => {
                        crate::trace_dispatch!("computable", "atan2", "quadrant-lower-left");
                        Some(base.add(Self::pi().negate()))
                    }
                }
            }
            Some(Sign::NoSign) => {
                let y_sign = y_sign.or_else(|| self.sign_until(min_precision).map(private_sign))?;
                match y_sign {
                    Sign::NoSign => Some(Self::zero()),
                    Sign::Plus => Some(Self::pi().shift_right(1)),
                    Sign::Minus => Some(Self::pi().shift_right(1).negate()),
                }
            }
            None if y_sign == Some(Sign::Plus) => {
                crate::trace_dispatch!("computable", "atan2", "half-angle-positive-y");
                Some(Self::atan2_half_angle(self, x))
            }
            None if y_sign == Some(Sign::Minus) => {
                crate::trace_dispatch!("computable", "atan2", "half-angle-negative-y");
                Some(Self::atan2_half_angle(self, x))
            }
            None => None,
        }
    }

    /// Construct `2*atan(y/(sqrt(x^2+y^2)+x))` on the principal atan2 branch.
    /// The caller must ensure finite defined inputs and a positive denominator;
    /// atan2's fallback supplies nonzero y. This excludes the origin and negative
    /// x axis, where the formula would divide by zero. No sign check or branch
    /// selection is performed here; multiplication by two is an exact binary shift.
    fn atan2_half_angle(y: Computable, x: Computable) -> Computable {
        Self::atan2_half_angle_argument(y, x).atan().shift_left(1)
    }

    /// Build the exact half-angle ratio `y/(sqrt(x^2+y^2)+x)`.
    /// Require finite defined operands with a positive denominator: nonzero y or
    /// positive x suffices. Retain squares, root and reciprocal so numerical
    /// cancellation does not replace the source by rounded coordinates. The
    /// helper itself does not check this precondition or impose a work budget.
    fn atan2_half_angle_argument(y: Computable, x: Computable) -> Computable {
        let radius = x.clone().square().add(y.clone().square()).sqrt();
        y.multiply(radius.add(x).inverse())
    }

    /// Reduce atan of a finite real source by two exact half-angle steps.
    /// Use x=1 for both ratios, making each denominator positive even at y=0.
    /// For theta=atan(self), the retained quarter-angle tangent has magnitude
    /// below `tan(pi/8)=sqrt(2)-1 < 1/2`, independently of a borderline coarse
    /// sample. Apply atan there and scale by four with the binary shift's precision
    /// transport. This prevents reselection of the same borderline series range;
    /// it does not establish a total-work or cancellation bound.
    pub(crate) fn atan_with_half_angle_reduction(self) -> Computable {
        // For theta = atan(self), each step maps tan(theta) to tan(theta/2).
        // Since |theta| < pi/2, two steps give an argument of magnitude
        // below tan(pi/8) = sqrt(2)-1 < 1/2. Both denominators are positive,
        // and the factor four retains the native approximation error budget.
        let half = Self::atan2_half_angle_argument(self, Self::one());
        let quarter = Self::atan2_half_angle_argument(half, Self::one());
        quarter.atan().shift_left(2)
    }

    /// Retain principal arcsine on the caller-admitted closed real interval `[-1,1]`.
    /// Zero simplifies exactly. Other rationals retain their signed payload for
    /// series/endpoint evaluation. Normalize a known negative nonrational input;
    /// a small planning exponent only schedules a candidate series whose kernel
    /// must independently prove its range. Otherwise defer the half-angle identity.
    /// This native API does not return typed domain errors; use [`crate::Real::asin`]
    /// for checked admission. New deferred nodes do not retain an outer signal,
    /// and no global work/memory bound follows from lazy construction.
    pub fn asin(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            match rational.sign() {
                Sign::NoSign => {
                    crate::trace_dispatch!("computable", "asin", "exact-zero");
                    return Self::zero();
                }
                Sign::Minus => {
                    crate::trace_dispatch!("computable", "asin", "signed-rational-deferred");
                    return Self::asin_rational_deferred(rational);
                }
                Sign::Plus => {
                    if rational.msd_exact().is_some_and(|msd| msd <= -4) {
                        // Tiny asin(x) is handled by its dedicated series; the generic
                        // atan transform builds extra sqrt/division nodes.
                        crate::trace_dispatch!("computable", "asin", "exact-tiny-rational-series");
                        return Self::asin_rational_deferred(rational);
                    }
                    if rational >= *INVERSE_ENDPOINT_RATIONAL_THRESHOLD {
                        crate::trace_dispatch!("computable", "asin", "endpoint-rational-deferred");
                        return Self::asin_rational_deferred(rational);
                    }
                    crate::trace_dispatch!("computable", "asin", "positive-rational-deferred");
                    return Self::asin_rational_deferred(rational);
                }
            }
        }
        if self.exact_sign() == Some(Sign::Minus) {
            crate::trace_dispatch!("computable", "asin", "known-negative-symmetry");
            return self.negate().asin().negate();
        }
        // The estimate only schedules a lazy candidate: the approximation
        // kernel independently proves the series domain before evaluating it.
        let (_, planned_msd) = self.planning_sign_and_msd();
        if planned_msd.flatten().is_some_and(|msd| msd <= -4) {
            crate::trace_dispatch!("computable", "asin", "structural-tiny-prescaled");
            return Self::prescaled_asin(self);
        }

        crate::trace_dispatch!("computable", "asin", "generic-atan-sqrt-transform");
        Self::asin_deferred(self)
    }

    /// Retain principal arccosine in `[0,pi]` for a caller-admitted input in `[-1,1]`.
    /// Normalize rational endpoints, zero and +/-one-half to exact pi multiples.
    /// Small rational magnitudes use pi/2-asin; other rational inputs retain a
    /// positive argument or a separate negative-input magnitude. A bounded Laurent
    /// recognizer can recover another rational before the positive-input
    /// half-angle or generic pi/2-asin path.
    ///
    /// Unknown equality to one must remain compatible with a zero result, so the
    /// generic positive-input helper does not assert a strictly positive sign.
    /// No typed domain check or uniform signal transport occurs here; checked
    /// admission belongs to [`crate::Real::acos`]. Recognition bounds are not a
    /// total evaluation-work or memory budget.
    pub fn acos(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            if rational.is_one() {
                crate::trace_dispatch!("computable", "acos", "exact-one-zero");
                return Self::zero();
            }
            if rational.is_minus_one() {
                crate::trace_dispatch!("computable", "acos", "exact-minus-one-pi");
                return Self::pi();
            }
            if rational.sign() == Sign::NoSign {
                crate::trace_dispatch!("computable", "acos", "exact-zero-half-pi");
                return Self::pi().shift_right(1);
            }
            let rational_sign = rational.sign();
            let magnitude = if rational_sign == Sign::Minus {
                rational.neg()
            } else {
                rational
            };
            if magnitude == *HALF_RATIONAL {
                crate::trace_dispatch!("computable", "acos", "exact-rational-half-pi-third");
                let numerator = if rational_sign == Sign::Minus { 2 } else { 1 };
                return Self::pi().multiply_rational(
                    Rational::fraction(numerator, 3).expect("three is nonzero"),
                );
            }
            if magnitude.msd_exact().is_some_and(|msd| msd <= -4) {
                crate::trace_dispatch!("computable", "acos", "tiny-via-asin");
                return Self::pi().shift_right(1).add(self.asin().negate());
            }
            if rational_sign == Sign::Minus {
                // Every negative rational uses acos(-x) = pi - acos(x). Store
                // the magnitude directly so mid-range values do not expand
                // through nested half-pi-minus-asin identities before trig
                // composition can inspect their exact [pi/2, pi] range.
                crate::trace_dispatch!("computable", "acos", "negative-rational-deferred");
                return Self::acos_negative_rational_deferred(magnitude);
            }
            if rational_sign == Sign::Plus {
                crate::trace_dispatch!("computable", "acos", "positive-rational-deferred");
                return Self::acos_positive_rational_deferred(magnitude);
            }
        }

        if let Some(rational) = self.bounded_laurent_rational(48) {
            crate::trace_dispatch!("computable", "acos", "exact-normal-form");
            return Self::rational(rational).acos();
        }

        if self.exact_sign() == Some(Sign::Plus) {
            // For positive values, acos(x) = 2 atan(sqrt((1-x)/(1+x))). This is the
            // endpoint-friendly path for values near 1.
            crate::trace_dispatch!("computable", "acos", "positive-endpoint-deferred");
            return Self::acos_positive(self);
        }

        crate::trace_dispatch!("computable", "acos", "generic-half-pi-minus-asin");
        Self::pi().shift_right(1).add(self.asin().negate())
    }

    /// Retain real inverse hyperbolic sine for a defined finite input.
    /// Normalize exact zero and known negative inputs. Recognized rational binades
    /// at most -2 select the odd series; those at least 3 select the direct
    /// log/sqrt graph. Other inputs choose between the rationalized log1p identity
    /// and `ln(x+sqrt(1+x^2))` using planning magnitude or a precision -4 sample.
    /// That sample schedules a mathematically equivalent identity, not a domain
    /// proof. Construction can evaluate the source and builds the selected graph
    /// eagerly; branch-dependent signal propagation and existing kernel resource
    /// limits apply, with no new typed exhaustion or total-work budget.
    pub fn asinh(self) -> Computable {
        let exact_rational = self.exact_rational();
        if exact_rational
            .as_ref()
            .is_some_and(|r| r.sign() == Sign::NoSign)
        {
            crate::trace_dispatch!("computable", "asinh", "exact-zero");
            return Self::zero();
        }
        let (known_sign, planned_msd) = self.planning_sign_and_msd();
        if exact_rational
            .as_ref()
            .is_some_and(|r| r.sign() == Sign::Minus)
            || known_sign == Some(Sign::Minus)
        {
            crate::trace_dispatch!("computable", "asinh", "known-negative-symmetry");
            return self.negate().asinh().negate();
        }
        let exact_small = exact_rational
            .as_ref()
            .and_then(Rational::msd_exact)
            .is_some_and(|msd| msd <= -2);
        let exact_large = exact_rational
            .as_ref()
            .and_then(Rational::msd_exact)
            .is_some_and(|msd| msd >= 3);
        if exact_small {
            crate::trace_dispatch!("computable", "asinh", "exact-small-rational-series");
            if let Some(rational) = exact_rational {
                return Self::asinh_rational_deferred(rational);
            }
            return Self::prescaled_asinh(self);
        }
        if exact_large {
            let radicand = self.clone().square().add(Self::one());
            crate::trace_dispatch!("computable", "asinh", "exact-large-direct-ln-sqrt");
            return self.add(radicand.sqrt()).ln();
        }
        let known_msd = planned_msd.flatten();
        let is_near_zero = match known_msd {
            Some(msd) => msd < 3,
            None => self.approx(-4) <= BigInt::from(64_u8),
        };
        if is_near_zero {
            // Direct Computable approximation benches include construction in
            // the measured work, and the eager graph caches its children better
            // than a deferred Real-only wrapper.
            let square = self.clone().square();
            let one = Self::one();
            let denominator = square.clone().add(one.clone()).sqrt().add(one);
            crate::trace_dispatch!("computable", "asinh", "near-zero-ln1p-transform");
            return self.add(square.multiply(denominator.inverse())).ln_1p();
        }

        let radicand = self.clone().square().add(Self::one());
        crate::trace_dispatch!("computable", "asinh", "generic-direct-ln-sqrt");
        self.add(radicand.sqrt()).ln()
    }

    /// Retain principal inverse hyperbolic cosine for a caller-admitted real `x >= 1`.
    /// Recognize immediate one/two and sqrt(2) anchors. For immediate rational or
    /// integer x at least two, form x^2-1 rationally before constructing the root.
    /// Otherwise planning magnitude or a precision -4 sample selects the near-one
    /// `ln1p((x-1)+sqrt(x^2-1))` or direct `ln(x+sqrt(x^2-1))` graph. Both identities
    /// hold throughout the admitted domain; scheduling does not validate it.
    ///
    /// Use [`crate::Real::acosh`] for checked domain failures. Native construction
    /// may refine and allocate a reduction graph, and signal transport depends on
    /// the selected path. Precision/exponent arithmetic and underlying kernels keep
    /// their existing limitations; no total-work or typed resource contract is added.
    pub fn acosh(self) -> Computable {
        let exact_rational_msd = match &self.internal.approximation {
            Approximation::One => {
                crate::trace_dispatch!("computable", "acosh", "exact-one-zero");
                return Self::zero();
            }
            Approximation::Ratio(r) => {
                if r.is_one() {
                    crate::trace_dispatch!("computable", "acosh", "exact-one-zero");
                    return Self::zero();
                }
                if r == &Rational::new(2) {
                    crate::trace_dispatch!("computable", "acosh", "exact-two-constant");
                    return Self::acosh2_constant();
                }
                if r >= &Rational::new(2) {
                    let radicand = r.clone() * r.clone() - Rational::one();
                    crate::trace_dispatch!(
                        "computable",
                        "acosh",
                        "exact-rational-at-least-two-direct-radicand"
                    );
                    return self.add(Self::sqrt_rational(radicand)).ln();
                }
                r.msd_exact()
            }
            Approximation::Int(n) => {
                if n == signed::ONE.deref() {
                    crate::trace_dispatch!("computable", "acosh", "exact-one-zero");
                    return Self::zero();
                }
                if n == signed::TWO.deref() {
                    crate::trace_dispatch!("computable", "acosh", "exact-two-constant");
                    return Self::acosh2_constant();
                }
                if n >= signed::TWO.deref() {
                    let r = Rational::from_bigint(n.clone());
                    let radicand = r.clone() * r - Rational::one();
                    crate::trace_dispatch!(
                        "computable",
                        "acosh",
                        "exact-integer-at-least-two-direct-radicand"
                    );
                    return self.add(Self::sqrt_rational(radicand)).ln();
                }
                if n.sign() == Sign::NoSign {
                    None
                } else {
                    Some(n.magnitude().bits() as Precision - 1)
                }
            }
            _ => None,
        };
        if let Approximation::Sqrt(child) = &self.internal.approximation
            && child
                .exact_rational()
                .is_some_and(|r| r == Rational::new(2))
        {
            crate::trace_dispatch!("computable", "acosh", "sqrt-two-asinh-one");
            return Self::asinh1_constant();
        }
        if exact_rational_msd.is_some_and(|msd| msd >= 3) {
            // Large exact rationals skip the low-precision near-one probe and
            // use the direct acosh identity.
            let one = Self::one();
            let radicand = self.clone().square().add(one.negate());
            crate::trace_dispatch!("computable", "acosh", "exact-large-direct-ln-sqrt");
            return self.add(radicand.sqrt()).ln();
        }
        let known_msd = self.planning_sign_and_msd().1.flatten();
        let is_near_one = match known_msd {
            Some(msd) => msd < 3,
            None => self.approx(-4) <= BigInt::from(64_u8),
        };
        if is_near_one {
            // Keep the public Computable kernel eager for approximation-heavy
            // benches; Real uses a deferred wrapper when construction alone is
            // the hot path.
            let one = Self::one();
            let shifted = self.clone().add(one.clone().negate());
            let radicand = self.square().add(one.negate());
            crate::trace_dispatch!("computable", "acosh", "near-one-ln1p-transform");
            return shifted.add(radicand.sqrt()).ln_1p();
        }

        // Generic identity for already validated large inputs.
        let one = Self::one();
        let radicand = self.clone().square().add(one.negate());
        crate::trace_dispatch!("computable", "acosh", "generic-direct-ln-sqrt");
        self.add(radicand.sqrt()).ln()
    }

    /// Retain principal inverse hyperbolic tangent for caller-admitted `-1 < x < 1`.
    /// Normalize zero and negative symmetry, retain a tiny rational series, and
    /// recognize one-half as ln(3)/2. Other admitted positive rationals form the
    /// exact ratio `(1+x)/(1-x)` before logarithm construction. Nonrational small
    /// planning estimates schedule a series with an independent evaluation-time
    /// range check; the remaining path builds the half-log-ratio graph.
    ///
    /// The endpoints are not finite values and inputs outside the interval are
    /// outside this native contract. Use [`crate::Real::atanh`] for typed admission.
    /// No uniform outer-signal transport or total-work budget is supplied, and
    /// planning smallness is not an exact domain certificate.
    pub fn atanh(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            match rational.sign() {
                Sign::NoSign => {
                    crate::trace_dispatch!("computable", "atanh", "exact-zero");
                    return Self::zero();
                }
                Sign::Minus => {
                    crate::trace_dispatch!("computable", "atanh", "exact-negative-symmetry");
                    return self.negate().atanh().negate();
                }
                Sign::Plus => {
                    if rational.msd_exact().is_some_and(|msd| msd <= -4) {
                        // Tiny atanh(x) is best served by the direct odd series.
                        crate::trace_dispatch!("computable", "atanh", "exact-tiny-prescaled");
                        return Self::atanh_rational_deferred(rational);
                    }
                    if rational.is_one_half() {
                        crate::trace_dispatch!("computable", "atanh", "exact-half-ln3");
                        return Self::ln_constant(3)
                            .expect("ln3 is a shared log constant")
                            .multiply(Self::half());
                    }
                    if !rational.is_one() {
                        // For exact rationals, atanh(x) is one exact ln ratio.
                        // That keeps common factors in the logarithm constructor
                        // instead of building a generic quotient Computable first.
                        // The final multiply reuses the shared exact half rational,
                        // while half() still constructs a fresh Computable wrapper.
                        let one = Rational::one();
                        let ratio = (one.clone() + rational.clone()) / (one - rational);
                        crate::trace_dispatch!("computable", "atanh", "exact-log-ratio");
                        return Self::ln_exact_rational(ratio).multiply(Self::half());
                    }
                }
            }
        }
        if self.exact_sign() == Some(Sign::Minus) {
            crate::trace_dispatch!("computable", "atanh", "known-negative-symmetry");
            return self.negate().atanh().negate();
        }
        // Planning chooses a lazy schedule, never a series-domain certificate.
        let (_, planned_msd) = self.planning_sign_and_msd();
        if planned_msd.flatten().is_some_and(|msd| msd <= -4) {
            crate::trace_dispatch!("computable", "atanh", "structural-tiny-prescaled");
            return Self::prescaled_atanh(self);
        }

        // General formula 1/2 * ln((1+x)/(1-x)). Tiny exact rationals avoid
        // this path because the odd atanh series has much less setup.
        crate::trace_dispatch!("computable", "atanh", "generic-log-ratio");
        let one = Self::one();
        let numerator = one.clone().add(self.clone());
        let denominator = one.add(self.negate());
        numerator
            .multiply(denominator.inverse())
            .ln()
            .multiply(Self::half())
    }

    /// Sufficiently prove `abs(self) <= 1/8` for coarse inverse-series scheduling.
    /// First inspect only the cached certified nonzero binade: an exponent at most
    /// -4 implies strict magnitude below one eighth. Otherwise a precision -8
    /// sample of magnitude at most 31 plus its one-unit error proves the closed
    /// bound. Missing facts, zero metadata and estimated exponents take this sample
    /// path. False means the sufficient test declined, not a proof of large input.
    ///
    /// The explicit signal is forwarded to sampling, but this helper does not poll
    /// it or return an abort receipt. Its boolean is a range certificate only for
    /// completed evaluation; callers must reject interrupted placeholder results.
    /// This fixed precision does not bound total child work or memory.
    pub(crate) fn inverse_series_argument_is_small(&self, signal: &Option<Signal>) -> bool {
        // Some inputs retain an exact magnitude. Deserialization drops caches,
        // and a constructor may schedule this series from an estimate alone.
        // Avoid a graph walk on the certified path; otherwise |256*x - a| <= 1
        // and |a| <= 31 prove |x| <= 1/8 independently of structural hints.
        self.cached_bound()
            .and_then(|bound| bound.known_msd())
            .flatten()
            .is_some_and(|msd| msd <= -4)
            || self.approx_signal(signal, -8).magnitude() <= &BigUint::from(31_u8)
    }
}
