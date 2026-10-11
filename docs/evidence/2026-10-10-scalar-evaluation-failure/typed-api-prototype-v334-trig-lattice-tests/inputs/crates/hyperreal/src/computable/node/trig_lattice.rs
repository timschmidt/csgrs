impl Computable {
    /// Prove `self = k*pi/2 (mod 2*pi)` without expanding binary offsets.
    /// The accepted grammar has only finite rational/pi leaves and total
    /// arithmetic, so a successful result also proves operand existence.
    /// Limits decline recognition; they never assert a negative fact.
    pub(crate) fn exact_half_pi_index(&self) -> Option<u8> {
        const MAX_BITS: u64 = 4096;
        const MAX_TERMS: usize = 16;

        // Ordinary rational trig inputs need no symbolic allocation.
        match &self.internal.approximation {
            Approximation::One => return None,
            Approximation::Int(n) => return n.is_zero().then_some(0),
            Approximation::Ratio(q) => return q.is_zero().then_some(0),
            _ => {}
        }

        #[derive(Clone)]
        struct Term {
            pi: bool,
            coefficient: Rational,
            exponent: i64,
        }

        fn bounded(q: &Rational) -> bool {
            q.numerator().bits() <= MAX_BITS && q.denominator().bits() <= MAX_BITS
        }

        impl Term {
            fn normalize(&mut self) -> Option<()> {
                if !bounded(&self.coefficient) {
                    return None;
                }
                if self.coefficient.is_zero() {
                    return Some(());
                }
                let numerator_twos = self.coefficient.numerator().trailing_zeros()?;
                let denominator_twos = self.coefficient.denominator().trailing_zeros()?;
                self.exponent = self.exponent
                    .checked_add(i64::try_from(numerator_twos).ok()?)?
                    .checked_sub(i64::try_from(denominator_twos).ok()?)?;
                self.coefficient = Rational::from_bigint_fraction(
                    BigInt::from_biguint(self.coefficient.sign(),
                        self.coefficient.numerator() >> numerator_twos),
                    self.coefficient.denominator() >> denominator_twos,
                ).ok()?;
                Some(())
            }
        }

        #[derive(Default)]
        struct Form(Vec<Term>);

        impl Form {
            fn insert(&mut self, mut term: Term) -> Option<()> {
                loop {
                    term.normalize()?;
                    if term.coefficient.is_zero() {
                        return Some(());
                    }
                    if let Some(index) = self.0.iter().position(|old|
                        old.pi == term.pi && old.exponent == term.exponent) {
                        // Each operand has at most MAX_BITS bits; the bounded
                        // temporary sum is checked before it can be reused.
                        term.coefficient = term.coefficient + self.0.swap_remove(index).coefficient;
                    } else {
                        if self.0.len() == MAX_TERMS {
                            return None;
                        }
                        self.0.push(term);
                        return Some(());
                    }
                }
            }

            fn leaf(pi: bool, coefficient: Rational) -> Option<Self> {
                let mut result = Self::default();
                result.insert(Term { pi, coefficient, exponent: 0 })?;
                Some(result)
            }
        }

        fn visit(node: &Computable, depth: u16, work: &mut usize) -> Option<Form> {
            if depth == 0 || *work == 0 {
                return None;
            }
            *work -= 1;
            use Approximation::*;
            match &node.internal.approximation {
                One => Form::leaf(false, Rational::one()),
                Int(n) if n.bits() <= MAX_BITS => Form::leaf(false, Rational::from_bigint(n.clone())),
                Ratio(q) if bounded(q) => Form::leaf(false, q.clone()),
                Constant(SharedConstant::Pi) => Form::leaf(true, Rational::one()),
                Constant(SharedConstant::Tau) => Form::leaf(true, Rational::new(2)),
                Negate(child) => {
                    let mut form = visit(child, depth - 1, work)?;
                    for term in &mut form.0 {
                        term.coefficient = -term.coefficient.clone();
                    }
                    Some(form)
                }
                Offset(child, shift) => {
                    let mut form = visit(child, depth - 1, work)?;
                    for term in &mut form.0 {
                        term.exponent = term.exponent.checked_add(i64::from(*shift))?;
                    }
                    Some(form)
                }
                Add(left, right) => {
                    let mut form = visit(left, depth - 1, work)?;
                    for term in visit(right, depth - 1, work)?.0 {
                        form.insert(term)?;
                    }
                    Some(form)
                }
                Multiply(left, right) => {
                    // Visit both children even for a zero factor. Unsupported
                    // partial functions cannot be erased by an annihilator.
                    let left = visit(left, depth - 1, work)?;
                    let right = visit(right, depth - 1, work)?;
                    let mut result = Form::default();
                    for a in &left.0 {
                        for b in &right.0 {
                            *work = work.checked_sub(1)?;
                            if a.pi && b.pi {
                                return None;
                            }
                            result.insert(Term {
                                pi: a.pi || b.pi,
                                coefficient: &a.coefficient * &b.coefficient,
                                exponent: a.exponent.checked_add(b.exponent)?,
                            })?;
                        }
                    }
                    Some(result)
                }
                _ => None,
            }
        }

        let form = visit(self, 128, &mut 256)?;
        // Every normalized coefficient has odd numerator and denominator;
        // distinct binary exponents therefore cannot cancel the lowest one.
        // Rational residuals cannot equal a nonzero rational multiple of pi.
        // A pi term below exponent -1 prevents an integral half-pi index.
        if form.0.iter().any(|term| !term.pi || term.exponent < -1) {
            return None;
        }
        let mut residue = Rational::zero();
        for term in form.0 {
            // Compute 2*q modulo 4. Modular powering depends on the exponent's
            // bit length, not its magnitude; no 2^exponent allocation occurs.
            let denominator = term.coefficient.denominator();
            let modulus = denominator << 2_usize;
            let power = BigUint::from(u64::try_from(term.exponent.checked_add(1)?).ok()?);
            let mut numerator = (term.coefficient.numerator()
                * BigUint::from(2_u8).modpow(&power, &modulus)) % &modulus;
            if term.coefficient.sign() == Sign::Minus && !numerator.is_zero() {
                numerator = &modulus - numerator;
            }
            let value = Rational::from_bigint_fraction(
                BigInt::from_biguint(Sign::Plus, numerator), denominator.clone()).ok()?;
            residue = residue + value;
            if !bounded(&residue) {
                return None;
            }
        }
        let index = residue.to_big_integer()?;
        Some(u8::from(index.magnitude().bit(0)) + 2 * u8::from(index.magnitude().bit(1)))
    }

    fn exact_trig_integer(&self) -> Option<Result<i8, Problem>> {
        use Approximation::*;
        let (argument, kind) = match &self.internal.approximation {
            PrescaledSin(c) | SinDeferred(c) => (c, 0),
            PrescaledCos(c) | CosDeferred(c) => (c, 1),
            PrescaledTan(c) | TanDeferred(c) => (c, 2),
            PrescaledCot(c) => (c, 3),
            _ => return None,
        };
        let index = usize::from(argument.exact_half_pi_index()?);
        let sine = [0, 1, 0, -1][index];
        let cosine = [1, 0, -1, 0][index];
        Some(match kind {
            0 => Ok(sine),
            1 => Ok(cosine),
            2 if cosine == 0 => Err(Problem::DivideByZero),
            3 if sine == 0 => Err(Problem::DivideByZero),
            _ => Ok(0),
        })
    }
}
