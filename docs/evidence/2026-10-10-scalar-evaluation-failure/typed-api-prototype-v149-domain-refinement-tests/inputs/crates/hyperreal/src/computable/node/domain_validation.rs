#[derive(Clone, Copy)]
enum RequiredRealDomain {
    Nonzero,
    Nonnegative,
    Positive,
    AboveMinusOne,
    AtLeastOne,
    ClosedUnit,
    OpenUnit,
    Probability,
}

impl RequiredRealDomain {
    fn accepts_rational(self, value: &Rational) -> bool {
        use RequiredRealDomain::*;
        match self {
            Nonzero => value.sign() != Sign::NoSign,
            Nonnegative => value.sign() != Sign::Minus,
            Positive => value.sign() == Sign::Plus,
            AboveMinusOne => value.sign() != Sign::Minus || value.numerator() < value.denominator(),
            AtLeastOne => value.sign() == Sign::Plus && value.numerator() >= value.denominator(),
            ClosedUnit => value.numerator() <= value.denominator(),
            OpenUnit => value.numerator() < value.denominator(),
            Probability => value.sign() == Sign::Plus && value.numerator() < value.denominator(),
        }
    }

    fn accepts_sign(self, sign: Sign) -> Option<bool> {
        use RequiredRealDomain::*;
        match (self, sign) {
            (Nonzero, sign) => Some(sign != Sign::NoSign),
            (Nonnegative, sign) => Some(sign != Sign::Minus),
            (Positive, sign) => Some(sign == Sign::Plus),
            (AboveMinusOne, Sign::Plus | Sign::NoSign) => Some(true),
            (AtLeastOne | Probability, Sign::Minus | Sign::NoSign) => Some(false),
            (ClosedUnit | OpenUnit, Sign::NoSign) => Some(true),
            _ => None,
        }
    }
}

impl Computable {
    // Called only after this operand has independently been proved real. Each
    // numeric sample then provides a legitimate one-unit interval for deciding
    // the additional partial-function domain. Boundary ambiguity stays typed.
    fn certify_required_domain(
        &self,
        signal: &Option<Signal>,
        domain: RequiredRealDomain,
        invalid: Problem,
    ) -> Result<(), Problem> {
        if should_stop(signal) {
            return Err(Problem::Exhausted);
        }
        if matches!(domain, RequiredRealDomain::Nonnegative)
            && self.has_nonnegative_range_when_real()
        {
            return Ok(());
        }
        if let Some(value) = self.bounded_rational_value() {
            return domain.accepts_rational(&value).then_some(()).ok_or(invalid);
        }
        if let Some(accepted) = self.exact_sign().and_then(|sign| domain.accepts_sign(sign)) {
            return accepted.then_some(()).ok_or(invalid);
        }
        let mut precision: Precision = -8;
        loop {
            let sample = self.approx_signal(signal, precision)?;
            let unit = BigInt::one() << precision.unsigned_abs();
            let lower = &sample - 1_u8;
            let upper = &sample + 1_u8;
            use RequiredRealDomain::*;
            let (accepted, rejected) = match domain {
                Nonzero => (lower > BigInt::zero() || upper < BigInt::zero(), false),
                Nonnegative => (lower >= BigInt::zero(), upper < BigInt::zero()),
                Positive => (lower > BigInt::zero(), upper <= BigInt::zero()),
                AboveMinusOne => (lower > -&unit, upper <= -&unit),
                AtLeastOne => (lower >= unit, upper < unit),
                ClosedUnit => (lower >= -&unit && upper <= unit, upper < -&unit || lower > unit),
                OpenUnit => (lower > -&unit && upper < unit, upper <= -&unit || lower >= unit),
                Probability => (lower > BigInt::zero() && upper < unit, upper <= BigInt::zero() || lower >= unit),
            };
            if accepted {
                return Ok(());
            }
            if rejected {
                return Err(invalid);
            }
            if precision <= -(Self::MAX_APPROXIMATION_BITS / 2) {
                return Err(Problem::Exhausted);
            }
            precision *= 2;
        }
    }

    /// Establish existence before a numeric shortcut discards an operand.
    /// Children are validated before their values can prove an outer domain.
    /// Proven domains are retained independently of numeric/sign caches.
    pub(crate) fn ensure_finite_real(&self, signal: &Option<Signal>) -> Result<(), Problem> {
        const MAX_DOMAIN_VISITS: usize = 1_048_576;
        if should_stop(signal) {
            return Err(Problem::Exhausted);
        }
        if self.has_finite_real_certificate() {
            self.internal.facts.store_finite_real();
            return Ok(());
        }
        // References keep this walk allocation-light and avoid recursive stack
        // growth. A shared child is skipped once its immutable domain is proved.
        let mut frames = vec![(self, false)];
        let mut remaining = MAX_DOMAIN_VISITS;
        while let Some((node, finish)) = frames.pop() {
            if should_stop(signal) {
                return Err(Problem::Exhausted);
            }
            if node.internal.facts.finite_real() {
                continue;
            }
            use Approximation::*;
            if !finish {
                remaining = remaining.checked_sub(1).ok_or(Problem::Exhausted)?;
                frames.push((node, true));
                match &node.internal.approximation {
                    Inverse(child) | Negate(child) | Square(child) | Offset(child, _)
                    | PrescaledExp(child) | Expm1(child) | Sqrt(child) | PrescaledLn(child)
                    | PrescaledAtan(child) | AtanDeferred(child) | PrescaledAsin(child)
                    | AsinDeferred(child) | AcosPositive(child) | AcoshNearOne(child)
                    | AcoshDirect(child) | AsinhNearZero(child) | AsinhDirect(child)
                    | PrescaledAsinh(child) | AtanhDirect(child) | PrescaledAtanh(child)
                    | PrescaledCos(child) | PrescaledSin(child) | PrescaledTan(child)
                    | PrescaledCot(child) | ErfSeries(child) | Erfc(child) | NormalSf(child)
                    | LogPnorm(child) | LogNormalSf(child) | LogDnorm(child) | NthRoot(child, _)
                    | SincSmall(child) | CoscSmall(child) | SinDeferred(child)
                    | CosDeferred(child) | TanDeferred(child) | LnDeferred(child) => frames.push((child, false)),
                    Add(left, right) | Multiply(left, right) => {
                        frames.push((right, false));
                        frames.push((left, false));
                    }
                    LinearCombination3(data) => {
                        for child in data.coefficients.iter().rev() {
                            frames.push((child, false));
                        }
                    }
                    NormalInterval { lo, hi } => {
                        frames.push((hi, false));
                        frames.push((lo, false));
                    }
                    NormalQuantile(data) => frames.push((&data.p, false)),
                    One | Int(_) | Ratio(_) | Constant(_) | PrescaledLnRational(_)
                    | BinaryScaledLnRational { .. } | IntegralAtan(_) | AtanRational(_)
                    | AsinRational(_) | AcosPositiveRational(_) | AcosNegativeRational(_)
                    | AsinhRational(_) | AtanhRational(_) | PrescaledCosRational(_)
                    | CosLargeRational(_) | PrescaledCosHalfPiMinusRational(_)
                    | PrescaledSinRational(_) | SinLargeRational(_)
                    | PrescaledSinHalfPiMinusRational(_) | PrescaledCotHalfPiMinusRational(_)
                    | TanLargeRational(_) | PrescaledTanRational(_) => {}
                }
                continue;
            }
            if !node.has_finite_real_certificate() {
                use RequiredRealDomain as Domain;
                let requirement = match &node.internal.approximation {
                    Inverse(child) => Some((child, Domain::Nonzero, Problem::DivideByZero)),
                    Sqrt(child) => Some((child, Domain::Nonnegative, Problem::SqrtNegative)),
                    NthRoot(child, degree) => {
                        if !(3..=Self::MAX_DIRECT_NTH_ROOT_DEGREE).contains(degree) {
                            return Err(Problem::OutOfRange);
                        }
                        Some((child, Domain::Nonnegative, Problem::NotANumber))
                    }
                    LnDeferred(child) => Some((child, Domain::Positive, Problem::NotANumber)),
                    PrescaledLn(child) => Some((child, Domain::AboveMinusOne, Problem::NotANumber)),
                    PrescaledAsin(child) | AsinDeferred(child) | AcosPositive(child) => Some((child, Domain::ClosedUnit, Problem::NotANumber)),
                    AtanhDirect(child) | PrescaledAtanh(child) => Some((child, Domain::OpenUnit, Problem::NotANumber)),
                    AcoshNearOne(child) | AcoshDirect(child) => Some((child, Domain::AtLeastOne, Problem::NotANumber)),
                    NormalQuantile(data) => Some((&data.p, Domain::Probability, Problem::NotANumber)),
                    PrescaledTan(child) | TanDeferred(child) | PrescaledCot(child) => {
                        // Public trig construction can attempt reduction and
                        // defer after failure. Build the deferred node directly
                        // so an observed cancellation cannot be swallowed here.
                        let kind = if matches!(node.internal.approximation, PrescaledCot(_)) {
                            SinDeferred(child.clone())
                        } else {
                            CosDeferred(child.clone())
                        };
                        let denominator = Computable {
                            internal: Arc::new(Node::new(kind, BoundCache::Invalid, ExactSignCache::Invalid)),
                            signal: signal.clone(),
                        };
                        denominator.certify_required_domain(signal, Domain::Nonzero, Problem::DivideByZero)?;
                        None
                    }
                    IntegralAtan(_) => return Err(Problem::DivideByZero),
                    PrescaledLnRational(_) | BinaryScaledLnRational { .. }
                    | AsinRational(_) | AcosPositiveRational(_) | AcosNegativeRational(_)
                    | AtanhRational(_) => return Err(Problem::NotANumber),
                    _ => return Err(Problem::Exhausted),
                };
                if let Some((child, domain, invalid)) = requirement {
                    child.certify_required_domain(signal, domain, invalid)?;
                }
            }
            node.internal.facts.store_finite_real();
        }
        Ok(())
    }
}
