impl Computable {
    // Call only after independently proving this operand is real. A literal
    // or retained sign needs neither rational recognition nor a graph walk.
    // Unreduced rationals keep their existing metered recognition path.
    fn available_domain_sign(&self) -> Option<Sign> {
        let sign = match &self.internal.approximation {
            Approximation::One | Approximation::Constant(_) => Sign::Plus,
            Approximation::Int(value) => value.sign(),
            Approximation::Ratio(value) if !value.is_internally_unreduced() => value.sign(),
            _ => return self.immediate_sign().map(private_sign),
        };
        // The former exact-sign fallback retained this proof for wide leaves.
        // Keep it available to subsequent zero-work queries on the child.
        self.internal
            .facts
            .publish_exact_sign(ExactSignCache::Valid(sign));
        Some(sign)
    }

    // Use only after independently establishing that this operand is real.
    // Reading a retained bound performs no graph walk or coefficient work.
    fn available_bound_proves_domain(&self, domain: RequiredRealDomain) -> bool {
        let Some(bound) = self.cached_bound() else {
            return false;
        };
        use RequiredRealDomain::*;
        match domain {
            AboveMinusOne | ClosedUnit | OpenUnit | Probability => {
                // An upper MSD e < 0 proves |x| < 2^(e+1) <= 1;
                // an exact binade is unnecessary for this upper bound.
                bound
                    .planning_msd()
                    .is_some_and(|msd| msd.is_none_or(|msd| msd < 0))
                    && (!matches!(domain, Probability) || bound.known_sign() == Some(Sign::Plus))
            }
            AtLeastOne => {
                // Here a lower bound is essential: only an exact binade
                // supplies |x| >= 2^e. An upper estimate cannot prove it.
                bound.known_sign() == Some(Sign::Plus)
                    && bound.known_msd().flatten().is_some_and(|msd| msd >= 0)
            }
            Nonzero | Nonnegative | Positive => false,
        }
    }

    // These range theorems require existence, which callers establish first.
    // A square need not have a known sign/nonzero certificate for its real
    // value to be nonnegative.
    fn has_nonnegative_range_when_real(&self) -> bool {
        matches!(
            self.internal.approximation,
            Approximation::Square(_)
                | Approximation::Sqrt(_)
                | Approximation::NthRoot(_, _)
                | Approximation::AcoshNearOne(_)
                | Approximation::AcoshDirect(_)
                | Approximation::AcosPositive(_)
                | Approximation::AcosPositiveRational(_)
                | Approximation::AcosNegativeRational(_)
                | Approximation::CoscSmall(_)
                | Approximation::PrescaledExp(_)
                | Approximation::Erfc(_)
                | Approximation::NormalSf(_)
        )
    }

    /// Prove that this expression denotes a finite real before using a theorem
    /// whose conclusion discards the child's sign or value. Failure to prove
    /// the domain is Unknown, including when the structural budget is spent.
    ///
    /// Numeric caches and sign caches do not by themselves prove existence.
    /// Sign/range facts are consulted only after the operand's real domain has
    /// been established independently. This proves mathematical domains, not
    /// private series convergence ranges or aggregate evaluation work bounds.
    fn has_finite_real_certificate(&self) -> bool {
        self.has_finite_real_certificate_with_budget(32, &mut 64)
    }

    fn has_finite_real_certificate_with_budget(&self, depth: u16, work: &mut usize) -> bool {
        fn sign_of_real(child: &Computable) -> Option<Sign> {
            child
                .available_domain_sign()
                .or_else(|| child.bounded_rational_value().map(|value| value.sign()))
                .or_else(|| child.exact_sign())
        }

        fn in_unit_interval(child: &Computable, closed: bool) -> bool {
            if child.available_bound_proves_domain(RequiredRealDomain::OpenUnit) {
                return true;
            }
            if let Some(value) = child.bounded_rational_value() {
                return value.numerator() < value.denominator()
                    || (closed && value.numerator() == value.denominator());
            }
            child
                .cheap_bound_shallow(8)
                .and_then(|bound| bound.known_msd())
                .is_some_and(|msd| msd.is_none_or(|msd| msd < 0))
        }

        fn at_least_one(child: &Computable) -> bool {
            if child.available_bound_proves_domain(RequiredRealDomain::AtLeastOne) {
                return true;
            }
            if let Some(value) = child.bounded_rational_value() {
                return value.sign() == Sign::Plus && value.numerator() >= value.denominator();
            }
            child.cheap_bound_shallow(8).is_some_and(|bound| {
                bound.known_sign() == Some(Sign::Plus)
                    && bound.known_msd().flatten().is_some_and(|msd| msd >= 0)
            })
        }

        fn visit(node: &Computable, depth: u16, work: &mut usize) -> bool {
            if node.internal.facts.finite_real() {
                return true;
            }
            if depth == 0 || *work == 0 || !charge_evaluation_work() {
                return false;
            }
            *work -= 1;
            let depth = depth - 1;
            use Approximation::*;
            match &node.internal.approximation {
                One | Int(_) | Ratio(_) | Constant(_) => true,
                IntegralAtan(n) => !n.is_zero(),
                AtanRational(_)
                | AsinhRational(_)
                | PrescaledCosRational(_)
                | CosLargeRational(_)
                | PrescaledCosHalfPiMinusRational(_)
                | PrescaledSinRational(_)
                | SinLargeRational(_)
                | PrescaledSinHalfPiMinusRational(_)
                // A rational angle cannot equal an odd multiple of pi/2.
                | PrescaledCotHalfPiMinusRational(_)
                | TanLargeRational(_)
                | PrescaledTanRational(_) => true,
                AsinRational(value)
                | AcosPositiveRational(value)
                | AcosNegativeRational(value) => {
                    SymbolicWork::current().read(value).is_some()
                        && value.numerator() <= value.denominator()
                }
                AtanhRational(value) => {
                    SymbolicWork::current().read(value).is_some()
                        && value.numerator() < value.denominator()
                }
                PrescaledLnRational(value)
                | BinaryScaledLnRational { residual: value, .. } => {
                    value.sign() != Sign::Minus
                        || (SymbolicWork::current().read(value).is_some()
                            && value.numerator() < value.denominator())
                }
                Negate(child)
                | Offset(child, _)
                | Square(child)
                | PrescaledExp(child)
                | Expm1(child)
                | PrescaledAtan(child)
                | AtanDeferred(child)
                | AsinhNearZero(child)
                | AsinhDirect(child)
                | PrescaledAsinh(child)
                | PrescaledCos(child)
                | PrescaledSin(child)
                | ErfSeries(child)
                | Erfc(child)
                | NormalSf(child)
                | LogPnorm(child)
                | LogNormalSf(child)
                | LogDnorm(child)
                | SincSmall(child)
                | CoscSmall(child)
                | SinDeferred(child)
                | CosDeferred(child) => visit(child, depth, work),
                Add(left, right) | Multiply(left, right) => {
                    // Even zero multiplication and x-x require both operands
                    // to exist. Never use an annihilator as a domain proof.
                    visit(left, depth, work) && visit(right, depth, work)
                }
                LinearCombination3(data) => data
                    .coefficients
                    .iter()
                    .all(|child| visit(child, depth, work)),
                Inverse(child) => {
                    visit(child, depth, work)
                        && matches!(sign_of_real(child), Some(Sign::Plus | Sign::Minus))
                }
                Sqrt(child) => {
                    visit(child, depth, work)
                        && (child.has_nonnegative_range_when_real()
                            || matches!(sign_of_real(child), Some(Sign::Plus | Sign::NoSign)))
                }
                NthRoot(child, degree) => {
                    (3..=Computable::MAX_DIRECT_NTH_ROOT_DEGREE).contains(degree)
                        && visit(child, depth, work)
                        && (child.has_nonnegative_range_when_real()
                            || matches!(sign_of_real(child), Some(Sign::Plus | Sign::NoSign)))
                }
                LnDeferred(child) => {
                    visit(child, depth, work) && sign_of_real(child) == Some(Sign::Plus)
                }
                PrescaledLn(child) => {
                    visit(child, depth, work)
                        && (matches!(sign_of_real(child), Some(Sign::Plus | Sign::NoSign))
                            || in_unit_interval(child, false))
                }
                PrescaledAsin(child) | AsinDeferred(child) | AcosPositive(child) => {
                    visit(child, depth, work) && in_unit_interval(child, true)
                }
                AtanhDirect(child) | PrescaledAtanh(child) => {
                    visit(child, depth, work) && in_unit_interval(child, false)
                }
                AcoshNearOne(child) | AcoshDirect(child) => {
                    visit(child, depth, work) && at_least_one(child)
                }
                PrescaledTan(child) | TanDeferred(child) => {
                    visit(child, depth, work)
                        && (child.available_bound_proves_domain(RequiredRealDomain::OpenUnit)
                            || child.bounded_rational_value().is_some()
                            || in_unit_interval(child, true))
                }
                PrescaledCot(child) => {
                    visit(child, depth, work)
                        && matches!(sign_of_real(child), Some(Sign::Plus | Sign::Minus))
                        && (child.available_bound_proves_domain(RequiredRealDomain::OpenUnit)
                            || child.bounded_rational_value().is_some()
                            || in_unit_interval(child, true))
                }
                NormalInterval { lo, hi } => visit(lo, depth, work) && visit(hi, depth, work),
                NormalQuantile(data) => {
                    visit(&data.p, depth, work)
                        && sign_of_real(&data.p) == Some(Sign::Plus)
                        && in_unit_interval(&data.p, false)
                }
            }
        }
        visit(self, depth, work)
    }
}
