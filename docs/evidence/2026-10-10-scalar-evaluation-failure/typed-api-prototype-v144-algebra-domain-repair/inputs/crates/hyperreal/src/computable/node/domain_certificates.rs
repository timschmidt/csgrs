impl Computable {
    /// Prove that this expression denotes a finite real before using a theorem
    /// whose conclusion discards the child's sign or value. Failure to prove
    /// the domain is Unknown, including when the structural budget is spent.
    ///
    /// Numeric caches and sign caches do not by themselves prove existence.
    /// Sign/range facts are consulted only after the operand's real domain has
    /// been established independently. This proves mathematical domains, not
    /// private series convergence ranges or aggregate evaluation work bounds.
    fn has_finite_real_certificate(&self) -> bool {
        fn sign_of_real(child: &Computable) -> Option<Sign> {
            child
                .bounded_rational_value()
                .map(|value| value.sign())
                .or_else(|| child.exact_sign())
        }

        fn in_unit_interval(child: &Computable, closed: bool) -> bool {
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
            if let Some(value) = child.bounded_rational_value() {
                return value.sign() == Sign::Plus && value.numerator() >= value.denominator();
            }
            child.cheap_bound_shallow(8).is_some_and(|bound| {
                bound.known_sign() == Some(Sign::Plus)
                    && bound.known_msd().flatten().is_some_and(|msd| msd >= 0)
            })
        }

        fn visit(node: &Computable, depth: u8, work: &mut u8) -> bool {
            if node.internal.facts.finite_real() {
                return true;
            }
            if depth == 0 || *work == 0 {
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
                | AcosNegativeRational(value) => value.numerator() <= value.denominator(),
                AtanhRational(value) => value.numerator() < value.denominator(),
                PrescaledLnRational(value)
                | BinaryScaledLnRational { residual: value, .. } => {
                    value.sign() != Sign::Minus || value.numerator() < value.denominator()
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
                        && matches!(sign_of_real(child), Some(Sign::Plus | Sign::NoSign))
                }
                NthRoot(child, degree) => {
                    (3..=Computable::MAX_DIRECT_NTH_ROOT_DEGREE).contains(degree)
                        && visit(child, depth, work)
                        && matches!(sign_of_real(child), Some(Sign::Plus | Sign::NoSign))
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
                        && (child.bounded_rational_value().is_some()
                            || in_unit_interval(child, true))
                }
                PrescaledCot(child) => {
                    visit(child, depth, work)
                        && matches!(sign_of_real(child), Some(Sign::Plus | Sign::Minus))
                        && (child.bounded_rational_value().is_some()
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
        visit(self, 32, &mut 64)
    }
}
