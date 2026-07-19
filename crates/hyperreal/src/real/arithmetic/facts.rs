impl Real {
    #[inline]
    fn has_zero_scale(&self) -> bool {
        self.rational.sign() == Sign::NoSign
    }

    /// Is this Real exactly zero?
    #[inline]
    pub fn definitely_zero(&self) -> bool {
        crate::trace_dispatch!("real", "definitely_zero", "rational-sign");
        self.has_zero_scale()
    }

    /// Returns whether this value is structurally known to be exactly one.
    #[inline]
    pub fn definitely_one(&self) -> bool {
        crate::trace_dispatch!("real", "definitely_one", "identity-facts");
        matches!(
            self.detailed_facts().identity.zero_or_one,
            ZeroOneStatus::One
        )
    }

    /// Classifies the value as exact zero or exact one when structural facts prove it.
    ///
    /// Returns `Some(false)` for zero, `Some(true)` for one, and `None` for
    /// every other value or values whose identity status cannot be decided
    /// without refinement.
    #[inline]
    pub fn zero_or_one(&self) -> Option<bool> {
        crate::trace_dispatch!("real", "zero_or_one", "identity-facts");
        match self.detailed_facts().identity.zero_or_one {
            ZeroOneStatus::Zero => Some(false),
            ZeroOneStatus::One => Some(true),
            ZeroOneStatus::NeitherOrUnknown => None,
        }
    }

    /// Classifies this value as exact zero, one, or minus one when structural
    /// facts prove it.
    ///
    /// This combined query serves hot dispatch paths that need all three
    /// identity cases, such as sparse vectors, homogeneous transforms, and
    /// signed-permutation matrices. It performs no approximation or refinement;
    /// callers get only stored exact scalar structure.
    #[inline]
    pub fn zero_one_or_minus_one(&self) -> ZeroOneMinusOneStatus {
        crate::trace_dispatch!("real", "zero_one_or_minus_one", "identity-facts");
        self.detailed_facts().identity.zero_one_or_minus_one
    }

    /// Return this value as an owned exact rational when that is structurally known.
    #[inline]
    pub fn exact_rational(&self) -> Option<Rational> {
        match self.class {
            One => Some(self.rational.clone()),
            _ => None,
        }
    }

    /// Return a borrowed exact rational when that is structurally known.
    ///
    /// Higher-level dense algebra kernels use this to batch exact rational
    /// linear combinations without cloning every scalar. It deliberately
    /// exposes only the already-public exact-rational shape; symbolic and
    /// computable values still go through their normal arithmetic paths.
    #[inline]
    pub fn exact_rational_ref(&self) -> Option<&Rational> {
        match self.class {
            One => Some(&self.rational),
            _ => None,
        }
    }

    /// Returns storage-level reuse evidence when this value is an exact rational.
    ///
    /// This is an advisory scheduling fact for aggregate arithmetic. It is true
    /// when the rational storage is shared or already carries retained arithmetic
    /// state, `Some(false)` for the first observation of an isolated exact
    /// rational, and `None` for a symbolic value. The first isolated observation
    /// is remembered so a repeated borrowed call can select a retained schedule.
    /// It does not inspect an approximation or affect exact decisions.
    #[inline]
    pub fn exact_rational_reuse_evidence(&self) -> Option<bool> {
        matches!(self.class, One).then(|| self.rational.has_arithmetic_reuse_evidence())
    }

    #[inline]
    fn scaled_by_rational(&self, scale: &Rational) -> Real {
        // Keep exact rational scaling as a structural operation. This is the
        // same fast path used by multiplication when one side is rational, and
        // the dot-product fallback below reuses it so mixed symbolic/rational
        // lanes do not build a generic product just to recover the same shape.
        if scale.sign() == Sign::NoSign || self.rational.sign() == Sign::NoSign {
            return Real::zero();
        }
        if scale.is_one() {
            return self.clone();
        }
        if scale.is_minus_one() {
            return -self;
        }

        let rational = scale * &self.rational;
        if matches!(self.class, One) {
            return Real::new(rational);
        }
        Real {
            rational,
            class: self.class.clone(),
            computable: self.computable.clone(),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        }
    }

    /// Returns true when this value is exactly rational with a dyadic denominator.
    ///
    /// This borrowed query exists for matrix and predicate kernels that need a
    /// representation heuristic without cloning the exact rational. Dyadic
    /// rationals reduce by shifts in `Rational`, so algorithms with more
    /// multiplications but fewer shared inverses can be profitable only on this
    /// structural class.
    #[inline]
    pub fn is_exact_dyadic_rational(&self) -> bool {
        matches!(self.class, One) && self.rational.is_dyadic()
    }

    /// Return exact-rational facts for a borrowed set of values.
    ///
    /// This is the scalar-layer entry point for object crates that want to
    /// carry common-scale eligibility without inspecting rational internals.
    /// Scalar representation facts are produced here, while geometry crates
    /// decide how long to retain them.
    pub fn exact_set_facts<'a, I>(values: I) -> crate::real::RealExactSetFacts
    where
        I: IntoIterator<Item = &'a Real>,
    {
        crate::real::RealExactSetFacts::from_reals(values)
    }

    /// Return a fused sum of signed exact-rational products.
    ///
    /// This is intentionally narrower than generic symbolic simplification:
    /// it succeeds only when every factor is already an exact rational. Dense
    /// algebra callers use it for fixed determinant/cofactor polynomials where
    /// reducing each product and partial sum dominates runtime. Non-rational
    /// symbolic and computable values keep their existing arithmetic trees so
    /// precision is still deferred in the established representation.
    pub fn exact_rational_signed_product_sum<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Real; FACTORS]; TERMS],
    ) -> Option<Real> {
        let mut rational_terms = [[rationals::ZERO.deref(); FACTORS]; TERMS];
        for i in 0..TERMS {
            for j in 0..FACTORS {
                rational_terms[i][j] = terms[i][j].exact_rational_ref()?;
            }
        }
        crate::trace_dispatch!("real", "product_sum", "exact-rational-shared-denom");
        Some(Real::new(Rational::signed_product_sum(
            positive_terms,
            rational_terms,
        )))
    }

    /// Return a fused sum of signed exact-rational products after the caller
    /// has already proved every factor is an exact rational.
    ///
    /// This deliberately bypasses per-factor class checks. It is for prepared
    /// exact matrix kernels that cache an aggregate exact-rational certificate
    /// before entering dense cofactor algebra.
    pub fn exact_rational_signed_product_sum_known_exact<
        const TERMS: usize,
        const FACTORS: usize,
    >(
        positive_terms: [bool; TERMS],
        terms: [[&Real; FACTORS]; TERMS],
    ) -> Real {
        let rational_terms = terms.map(|term| term.map(|factor| &factor.rational));
        crate::trace_dispatch!("real", "product_sum", "exact-rational-known-shared-denom");
        Real::new(Rational::signed_product_sum(positive_terms, rational_terms))
    }

    /// Return a two-term exact-rational product sum after the caller has
    /// already proved all four factors exact.
    ///
    /// This fixed-shape entry point lets complex and 2x2 determinant kernels
    /// attempt the scalar word reducer without first entering the generic
    /// product-shape planner.
    pub fn exact_rational_signed_product_sum2_known_exact(
        positive_terms: [bool; 2],
        terms: [[&Real; 2]; 2],
    ) -> Real {
        let rational_terms = terms.map(|term| term.map(|factor| &factor.rational));
        Real::new(Rational::signed_product_sum2(
            positive_terms,
            rational_terms,
        ))
    }

    /// Multiply exact-rational complex component pairs after the caller has
    /// proved all four factors exact.
    ///
    /// The scalar layer converts each rational storage value once and returns
    /// `(ac - bd, ad + bc)` without exposing numerator or denominator storage
    /// to the complex-number layer.
    pub fn exact_rational_complex_product_known_exact(
        left: [&Real; 2],
        right: [&Real; 2],
    ) -> (Real, Real) {
        let (re, im) = Rational::complex_product_components(
            left.map(|value| &value.rational),
            right.map(|value| &value.rational),
        );
        (Real::new(re), Real::new(im))
    }

    /// Divide exact-rational complex component pairs after the caller has
    /// proved all four factors exact.
    ///
    /// This keeps conjugate-product formation, norm construction, and the two
    /// final quotient reductions in the scalar layer. Division by an exact
    /// zero norm returns [`Problem::DivideByZero`](crate::Problem::DivideByZero).
    pub fn exact_rational_complex_quotient_known_exact(
        left: [&Real; 2],
        right: [&Real; 2],
    ) -> Result<(Real, Real), crate::Problem> {
        let (re, im) = Rational::complex_quotient_components(
            left.map(|value| &value.rational),
            right.map(|value| &value.rational),
        )?;
        Ok((Real::new(re), Real::new(im)))
    }

    /// Return a fused exact-rational product sum using a carried shared-scale
    /// certificate.
    ///
    /// This is the denominator-specialized counterpart to
    /// [`Self::exact_rational_signed_product_sum_known_exact`]. It is intended
    /// for geometric objects that already retained a common reduced
    /// denominator fact across all factors. `Rational` still validates the
    /// certificate before using the faster schedule, so stale or over-broad
    /// object facts fall back to the generic exact reducer instead of becoming
    /// arithmetic assumptions. Object structure is preserved until a certified
    /// arithmetic package can consume it, with fraction normalization delayed.
    pub fn exact_rational_signed_product_sum_known_shared_denominator<
        const TERMS: usize,
        const FACTORS: usize,
    >(
        positive_terms: [bool; TERMS],
        terms: [[&Real; FACTORS]; TERMS],
    ) -> Real {
        let rational_terms = terms.map(|term| term.map(|factor| &factor.rational));
        if let Some(value) =
            Rational::signed_product_sum_shared_denominator(positive_terms, rational_terms)
        {
            crate::trace_dispatch!("real", "product_sum", "exact-rational-known-common-scale");
            return Real::new(value);
        }
        crate::trace_dispatch!(
            "real",
            "product_sum",
            "exact-rational-known-common-scale-fallback"
        );
        Real::new(Rational::signed_product_sum(positive_terms, rational_terms))
    }

    /// Return a fixed-size signed sum of products while preserving its shape.
    ///
    /// This is the general expression-layer counterpart to the matrix dot
    /// helpers. It first attempts the exact rational reducer, which keeps one
    /// shared denominator and performs one final canonicalization for the whole
    /// determinant/cofactor polynomial. If any factor is symbolic or computable,
    /// it falls back to a bounded expression tree that still prunes exact-zero
    /// factors and applies exact-rational scales directly.
    ///
    /// The API is intentionally fixed-arity and caller-directed rather than a
    /// general symbolic optimizer. Predicate, matrix, and solver crates should
    /// pass known geometric polynomials here before expanding them. The
    /// exact-rational route uses delayed normalization.
    pub fn signed_product_sum<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Real; FACTORS]; TERMS],
    ) -> Real {
        if let Some(sum) = Self::exact_rational_signed_product_sum(positive_terms, terms) {
            crate::trace_dispatch!("real", "product_sum", "fixed-exact-rational");
            return sum;
        }

        crate::trace_dispatch!("real", "product_sum", "fixed-real-tree");
        Self::active_signed_product_sum(positive_terms, terms)
    }

    /// Return a fixed-size signed sum of products whose terms are already active.
    ///
    /// Callers use this after object-level facts have already removed
    /// structurally zero terms. Exact-rational inputs still go through the
    /// whole-polynomial reducer; mixed symbolic inputs keep rational factors as
    /// scales and avoid introducing unrelated optimization passes.
    pub fn active_signed_product_sum<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Real; FACTORS]; TERMS],
    ) -> Real {
        if let Some(sum) = Self::exact_rational_signed_product_sum(positive_terms, terms) {
            crate::trace_dispatch!("real", "product_sum", "active-fixed-exact-rational");
            return sum;
        }

        let mut total = None;
        for i in 0..TERMS {
            let Some(product) = Self::product_term(terms[i]) else {
                continue;
            };
            let signed = if positive_terms[i] { product } else { -product };
            total = Some(match total.take() {
                Some(total) => &total + &signed,
                None => signed,
            });
        }
        total.unwrap_or_else(Real::zero)
    }

    /// Conservatively inspect public structural facts about this value.
    #[inline]
    pub fn structural_facts(&self) -> RealStructuralFacts {
        if matches!(self.class, One) {
            crate::trace_dispatch!("real", "structural_facts", "exact-rational");
            return facts_from_rational(&self.rational, true);
        }

        let rational_sign = self.rational.sign();
        if rational_sign == Sign::NoSign {
            crate::trace_dispatch!("real", "structural_facts", "zero-scale");
            return facts_from_rational(&self.rational, false);
        }

        crate::trace_dispatch!(
            "real",
            "structural_facts",
            match self.class {
                One => "exact-rational",
                Irrational => "scaled-computable",
                Pi | PiPow(_) | PiInv | PiExp(_) | PiInvExp(_) | PiSqrt(_) | ConstProduct(_)
                | ConstOffset(_) | ConstProductSqrt(_) | Sqrt(_) | Exp(_) | Ln(_) | LnAffine(_)
                | LnProduct(_) | Log10(_) | Log2(_) | SinPi(_) | TanPi(_) =>
                    "symbolic-nonzero-scale",
            }
        );

        let computable = self.computable_ref().structural_facts();
        let sign = match self.class {
            One => Some(real_sign_from_num(rational_sign)),
            Pi | PiPow(_) | PiInv | PiExp(_) | PiInvExp(_) | PiSqrt(_) | ConstProduct(_)
            | ConstOffset(_) | ConstProductSqrt(_) | Sqrt(_) | Exp(_) | Ln(_) | LnAffine(_)
            | LnProduct(_) | Log10(_) | Log2(_) | SinPi(_) | TanPi(_) => {
                // Exact symbolic classes are positive by construction, so the
                // outer rational scale alone determines sign. Additive classes
                // such as ConstOffset/LnAffine are admitted only when this
                // invariant is certified.
                Some(real_sign_from_num(rational_sign))
            }
            Irrational => {
                multiply_public_sign(Some(real_sign_from_num(rational_sign)), computable.sign)
            }
        };

        let zero = match sign {
            Some(RealSign::Zero) => ZeroKnowledge::Zero,
            Some(RealSign::Negative | RealSign::Positive) => ZeroKnowledge::NonZero,
            None if matches!(computable.zero, ZeroKnowledge::NonZero) => ZeroKnowledge::NonZero,
            None => ZeroKnowledge::Unknown,
        };

        let magnitude = match (self.rational.msd_exact(), computable.magnitude) {
            (Some(rational_msd), Some(magnitude)) => Some(MagnitudeBits {
                msd: rational_msd + magnitude.msd,
                exact_msd: magnitude.exact_msd,
            }),
            _ => computable.magnitude,
        };

        RealStructuralFacts {
            sign,
            zero,
            exact_rational: false,
            magnitude,
        }
    }

    /// Return richer opt-in structural facts for dispatch-heavy callers.
    ///
    /// This intentionally does not run approximation or refinement. It is
    /// derived from the same stored rational/class metadata as
    /// `structural_facts`, plus bit-length and denominator-shape checks. Keep
    /// expensive decomposition out of this query so solvers and matrix kernels
    /// can call it speculatively.
    ///
    /// Structural-dispatch note: this fact set is the right boundary for future
    /// algorithm selection. Keep additions descriptive, such as integer grid,
    /// dyadic scale, algebraic class, or magnitude envelope, so higher crates
    /// can choose faster exact kernels without learning `Real` internals.
    #[inline]
    pub fn detailed_facts(&self) -> RealDetailedFacts {
        let base = self.structural_facts();
        let exact_rational = matches!(self.class, One);
        let cmp_one = if exact_rational {
            structural_cmp_from_ordering(self.rational.cmp_one_structural())
        } else if self.rational.is_one() || self.rational.is_minus_one() {
            // Multiplying two exact MSDs can carry into the next binade. Only
            // unit-magnitude outer scales preserve the computable's exact MSD
            // strongly enough to certify a comparison with one.
            structural_cmp_one_from_base(&base)
        } else {
            StructuralComparison::Unknown
        };
        let abs_cmp_one = if exact_rational {
            structural_cmp_from_ordering(self.rational.abs_cmp_one_structural())
        } else if self.rational.is_one() || self.rational.is_minus_one() {
            structural_abs_cmp_one_from_base(&base)
        } else {
            StructuralComparison::Unknown
        };
        let identity = IdentityFacts {
            known_one: exact_rational && self.rational.is_one(),
            known_minus_one: exact_rational && self.rational.is_minus_one(),
            zero_or_one: if self.rational.sign() == Sign::NoSign {
                ZeroOneStatus::Zero
            } else if exact_rational && self.rational.is_one() {
                ZeroOneStatus::One
            } else {
                ZeroOneStatus::NeitherOrUnknown
            },
            zero_one_or_minus_one: if self.rational.sign() == Sign::NoSign {
                ZeroOneMinusOneStatus::Zero
            } else if exact_rational && self.rational.is_one() {
                ZeroOneMinusOneStatus::One
            } else if exact_rational && self.rational.is_minus_one() {
                ZeroOneMinusOneStatus::MinusOne
            } else {
                ZeroOneMinusOneStatus::NeitherOrUnknown
            },
        };
        let rational = if exact_rational {
            self.rational.detailed_rational_facts()
        } else {
            RationalFacts {
                exact_integer: false,
                exact_small_integer_i64: false,
                exact_dyadic: false,
                power_of_two: false,
                storage: RationalStorageClass::VeryLarge,
            }
        };
        let ordering = OrderingFacts {
            cmp_one,
            abs_cmp_one,
        };
        let primitive = primitive_facts_from_base(&base);
        let domains = DomainFacts {
            reciprocal: domain_from_zero_nonzero(base.zero),
            sqrt: domain_from_sign_nonnegative(base.sign),
            log: domain_from_sign_positive(base.sign),
            asin_acos: domain_abs_cmp_one(abs_cmp_one, true),
            unit_interval_closed: domain_abs_cmp_one(abs_cmp_one, true),
            unit_interval_open: domain_abs_cmp_one(abs_cmp_one, false),
            acosh: domain_cmp_one_ge(cmp_one),
            atanh: domain_abs_cmp_one(abs_cmp_one, false),
        };
        let dependencies = symbolic_dependencies_for_class(&self.class);
        let symbolic = SymbolicFacts {
            kind: structural_kind_for_class(&self.class),
            degree: symbolic_degree_for_class(&self.class),
            dependencies,
            has_sqrt_factor: matches!(self.class, Sqrt(_) | PiSqrt(_) | ConstProductSqrt(_)),
            has_pi_factor: dependencies.contains(SymbolicDependencyMask::PI),
            has_exp_factor: dependencies.contains(SymbolicDependencyMask::EXP),
            has_log_factor: dependencies.contains(SymbolicDependencyMask::LOG),
            has_trig_factor: dependencies.contains(SymbolicDependencyMask::TRIG),
            computable_required: self.computable.is_some() || matches!(self.class, Irrational),
        };

        crate::trace_dispatch!(
            "real",
            "detailed_facts",
            match symbolic.kind {
                StructuralKind::ExactRational => "exact-rational",
                StructuralKind::PiLike => "pi-like",
                StructuralKind::ExpLike => "exp-like",
                StructuralKind::SqrtLike => "sqrt-like",
                StructuralKind::LogLike => "log-like",
                StructuralKind::TrigExact => "trig-exact",
                StructuralKind::ProductConstant => "product-constant",
                StructuralKind::ComputableOpaque => "computable-opaque",
            }
        );

        RealDetailedFacts {
            base,
            identity,
            rational,
            primitive,
            ordering,
            domains,
            symbolic,
        }
    }

    /// Return structural domain certificates for common unary operations.
    ///
    /// This is a convenience accessor around [`Real::detailed_facts`]. It does
    /// not approximate or refine the value. Domain facts are conservative:
    /// `Valid` and `Invalid` are certificates, while `Unknown` means callers
    /// need an exact predicate, a symbolic rewrite, or an explicit refinement
    /// policy.
    #[inline]
    pub fn domain_facts(&self) -> DomainFacts {
        self.detailed_facts().domains
    }

    /// Return whether reciprocal/inverse is structurally in-domain.
    #[inline]
    pub fn reciprocal_domain(&self) -> DomainStatus {
        self.domain_facts().reciprocal
    }

    /// Return whether square root is structurally in-domain.
    #[inline]
    pub fn sqrt_domain(&self) -> DomainStatus {
        self.domain_facts().sqrt
    }

    /// Return whether natural logarithm is structurally in-domain.
    #[inline]
    pub fn log_domain(&self) -> DomainStatus {
        self.domain_facts().log
    }

    /// Return whether asin/acos are structurally in-domain.
    #[inline]
    pub fn asin_acos_domain(&self) -> DomainStatus {
        self.domain_facts().asin_acos
    }

    /// Return whether acosh is structurally in-domain.
    #[inline]
    pub fn acosh_domain(&self) -> DomainStatus {
        self.domain_facts().acosh
    }

    /// Return whether atanh is structurally in-domain.
    #[inline]
    pub fn atanh_domain(&self) -> DomainStatus {
        self.domain_facts().atanh
    }

    /// Conservatively report whether structural inspection proves this value is zero.
    #[inline]
    pub fn zero_status(&self) -> ZeroKnowledge {
        match self.rational.sign() {
            Sign::NoSign => {
                crate::trace_dispatch!("real", "zero_status", "zero-scale");
                ZeroKnowledge::Zero
            }
            // All named/exact classes are non-zero when their rational scale is
            // non-zero; only opaque computables need refinement. Keep this as a
            // negative test so adding another exact class does not lengthen this
            // predicate-heavy fast path.
            Sign::Minus | Sign::Plus if !matches!(self.class, Irrational) => {
                crate::trace_dispatch!("real", "zero_status", "symbolic-nonzero-scale");
                ZeroKnowledge::NonZero
            }
            Sign::Minus | Sign::Plus => {
                crate::trace_dispatch!("real", "zero_status", "scaled-computable");
                self.computable_ref().zero_status()
            }
        }
    }

    /// Try to prove the sign without refining past `min_precision`.
    #[inline]
    pub fn refine_sign_until(&self, min_precision: i32) -> Option<RealSign> {
        self.certified_sign_until(min_precision).sign()
    }

    /// Try to prove the sign and report the proof route used.
    ///
    /// This method is the certificate-bearing counterpart to
    /// [`Real::refine_sign_until`]. It never returns a primitive-float
    /// approximation and must not be interpreted as a tolerance policy. A known
    /// result is an exact sign proof from structural facts, exact zero scale, or
    /// bounded exact-real refinement. This matches Yap's EGC requirement that
    /// combinatorial predicates consume certified facts or explicit uncertainty.
    #[inline]
    pub fn certified_sign_until(&self, min_precision: i32) -> CertifiedRealSign {
        let facts = self.structural_facts();
        if let Some(sign) = facts.sign {
            crate::trace_dispatch!("real", "certified_sign_until", "structural-facts");
            return CertifiedRealSign::Known {
                sign,
                certificate: RealSignCertificate::StructuralFacts,
            };
        }
        if self.rational.sign() == Sign::NoSign {
            crate::trace_dispatch!("real", "certified_sign_until", "zero-scale");
            return CertifiedRealSign::Known {
                sign: RealSign::Zero,
                certificate: RealSignCertificate::ExactZeroScale,
            };
        }
        crate::trace_dispatch!("real", "certified_sign_until", "computable-refine");
        let Some(computable_sign) = self.computable_ref().sign_until(min_precision) else {
            crate::trace_dispatch!("real", "certified_sign_until", "unknown");
            return CertifiedRealSign::Unknown { min_precision };
        };
        let Some(sign) = multiply_public_sign(
            Some(real_sign_from_num(self.rational.sign())),
            Some(computable_sign),
        ) else {
            crate::trace_dispatch!("real", "certified_sign_until", "scale-unknown");
            return CertifiedRealSign::Unknown { min_precision };
        };
        CertifiedRealSign::Known {
            sign,
            certificate: RealSignCertificate::BoundedRefinement { min_precision },
        }
    }

    /// Try to prove whether two values are equal without refining past
    /// `min_precision`.
    ///
    /// This is intentionally separate from [`PartialEq`]. `PartialEq` remains a
    /// fast structural relation, while this method can prove additional semantic
    /// equalities and inequalities by comparing exact rationals, checking cheap
    /// structural facts, and finally proving the sign of `self - other` through
    /// bounded exact-real refinement. `Unknown` means the requested refinement
    /// budget did not decide the result.
    #[inline]
    pub fn certified_eq_until(&self, other: &Self, min_precision: i32) -> CertifiedRealEquality {
        if self == other {
            crate::trace_dispatch!("real", "certified_eq_until", "structural-equality");
            return CertifiedRealEquality::Equal {
                certificate: RealEqualityCertificate::StructuralEquality,
            };
        }

        if let (Some(left), Some(right)) = (self.exact_rational_ref(), other.exact_rational_ref()) {
            crate::trace_dispatch!("real", "certified_eq_until", "exact-rational-comparison");
            return if left == right {
                CertifiedRealEquality::Equal {
                    certificate: RealEqualityCertificate::ExactRationalComparison,
                }
            } else {
                CertifiedRealEquality::NotEqual {
                    certificate: RealEqualityCertificate::ExactRationalComparison,
                }
            };
        }

        if self.class == other.class && !matches!(self.class, Irrational) {
            crate::trace_dispatch!(
                "real",
                "certified_eq_until",
                "same-exact-class-different-scale"
            );
            return CertifiedRealEquality::NotEqual {
                certificate: RealEqualityCertificate::StructuralFacts,
            };
        }

        let left_facts = self.structural_facts();
        let right_facts = other.structural_facts();

        match (left_facts.zero, right_facts.zero) {
            (ZeroKnowledge::Zero, ZeroKnowledge::Zero) => {
                crate::trace_dispatch!("real", "certified_eq_until", "structural-zero-equality");
                return CertifiedRealEquality::Equal {
                    certificate: RealEqualityCertificate::StructuralFacts,
                };
            }
            (ZeroKnowledge::Zero, ZeroKnowledge::NonZero)
            | (ZeroKnowledge::NonZero, ZeroKnowledge::Zero) => {
                crate::trace_dispatch!("real", "certified_eq_until", "structural-zero-inequality");
                return CertifiedRealEquality::NotEqual {
                    certificate: RealEqualityCertificate::StructuralFacts,
                };
            }
            _ => {}
        }

        if let (Some(left), Some(right)) = (left_facts.sign, right_facts.sign)
            && left != right
        {
            crate::trace_dispatch!("real", "certified_eq_until", "structural-sign-inequality");
            return CertifiedRealEquality::NotEqual {
                certificate: RealEqualityCertificate::StructuralFacts,
            };
        }

        if let (Some(left), Some(right)) = (left_facts.magnitude, right_facts.magnitude)
            && left.exact_msd
            && right.exact_msd
            && left.msd != right.msd
        {
            crate::trace_dispatch!(
                "real",
                "certified_eq_until",
                "structural-magnitude-inequality"
            );
            return CertifiedRealEquality::NotEqual {
                certificate: RealEqualityCertificate::StructuralFacts,
            };
        }

        let difference = self - other;
        match difference.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                certificate,
            } => {
                crate::trace_dispatch!("real", "certified_eq_until", "difference-zero");
                CertifiedRealEquality::Equal {
                    certificate: equality_certificate_from_sign_certificate(certificate),
                }
            }
            CertifiedRealSign::Known { certificate, .. } => {
                crate::trace_dispatch!("real", "certified_eq_until", "difference-nonzero");
                CertifiedRealEquality::NotEqual {
                    certificate: equality_certificate_from_sign_certificate(certificate),
                }
            }
            CertifiedRealSign::Unknown { .. } => {
                crate::trace_dispatch!("real", "certified_eq_until", "unknown");
                CertifiedRealEquality::Unknown { min_precision }
            }
        }
    }

    /// Compare two values with a certified structural/refinement predicate.
    ///
    /// The comparison first uses representation equality and exact-rational
    /// ordering, then proves the sign of `self - other` through the same
    /// structural facts and bounded exact-real refinement used by
    /// [`Real::certified_sign_until`]. `Unknown` means the requested refinement
    /// budget did not certify an ordering; it is not an approximate equality.
    #[inline]
    pub fn certified_cmp_until(&self, other: &Self, min_precision: i32) -> CertifiedRealOrdering {
        if self == other {
            crate::trace_dispatch!("real", "certified_cmp_until", "structural-equality");
            return CertifiedRealOrdering::Known {
                ordering: Ordering::Equal,
                certificate: RealOrderingCertificate::StructuralEquality,
            };
        }

        if let (Some(left), Some(right)) = (self.exact_rational_ref(), other.exact_rational_ref()) {
            crate::trace_dispatch!("real", "certified_cmp_until", "exact-rational-comparison");
            return CertifiedRealOrdering::Known {
                ordering: left
                    .partial_cmp(right)
                    .expect("exact rationals should be comparable"),
                certificate: RealOrderingCertificate::ExactRationalComparison,
            };
        }

        let difference = self - other;
        match difference.certified_sign_until(min_precision) {
            CertifiedRealSign::Known { sign, certificate } => {
                crate::trace_dispatch!("real", "certified_cmp_until", "difference-sign");
                CertifiedRealOrdering::Known {
                    ordering: ordering_from_real_sign(sign),
                    certificate: ordering_certificate_from_sign_certificate(certificate),
                }
            }
            CertifiedRealSign::Unknown { .. } => {
                crate::trace_dispatch!("real", "certified_cmp_until", "unknown");
                CertifiedRealOrdering::Unknown { min_precision }
            }
        }
    }

    /// Returns a closed dyadic-rational interval certified to contain this
    /// value at the requested computable precision.
    ///
    /// A computable approximation at precision `p` is within one integer unit
    /// of `value * 2^-p`. Expanding that integer by one on both sides, restoring
    /// the dyadic scale, and multiplying by this real's exact rational scale
    /// therefore yields conservative exact bounds. This is useful for broad
    /// phases that need a cheap separation certificate but do not need the
    /// exact ordering of overlapping values.
    ///
    /// Returns `None` if evaluation was aborted; aborted approximations are not
    /// certificates.
    pub fn certified_dyadic_interval(&self, precision: i32) -> Option<[Rational; 2]> {
        fn scaled_integer(value: BigInt, precision: i32) -> Rational {
            if precision < 0 {
                let shift = usize::try_from(precision.unsigned_abs())
                    .expect("u32 precision magnitude should fit usize");
                Rational::from_bigint_fraction(value, BigUint::from(1_u8) << shift)
                    .expect("a power-of-two denominator is nonzero")
            } else {
                let shift = usize::try_from(precision as u32)
                    .expect("u32 precision should fit usize");
                Rational::from_bigint(value << shift)
            }
        }

        if let Some(exact) = self.exact_rational_ref() {
            return Some([exact.clone(), exact.clone()]);
        }
        if self.is_aborted() {
            return None;
        }
        let approximation = self.computable_clone().approx(precision);
        if self.is_aborted() {
            return None;
        }
        let lower = scaled_integer(&approximation - BigInt::from(1_u8), precision);
        let upper = scaled_integer(approximation + BigInt::from(1_u8), precision);
        let scaled_lower = &self.rational * &lower;
        let scaled_upper = &self.rational * &upper;
        Some(if self.rational.sign() == Sign::Minus {
            [scaled_upper, scaled_lower]
        } else {
            [scaled_lower, scaled_upper]
        })
    }
}
