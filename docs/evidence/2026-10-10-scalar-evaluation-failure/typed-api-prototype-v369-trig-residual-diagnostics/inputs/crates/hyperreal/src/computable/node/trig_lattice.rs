const MAX_TRIG_COEFFICIENT_BITS: u64 = 4096;
const MAX_TRIG_TERMS: usize = 16;

#[derive(Clone)]
struct PiLinearTerm {
    pi: bool,
    coefficient: Rational,
    exponent: i64,
}

fn bounded_trig_coefficient(q: &Rational) -> bool {
    q.numerator().bits() <= MAX_TRIG_COEFFICIENT_BITS && q.denominator().bits() <= MAX_TRIG_COEFFICIENT_BITS
}

impl PiLinearTerm {
    /// Materialize only bounded rational residuals; huge periodic coefficients
    /// stay compressed and are handled by modular powering instead.
    fn rational_value(&self) -> Option<Rational> {
        let shift = self.exponent.unsigned_abs();
        let width = if self.exponent >= 0 {
            self.coefficient.numerator().bits()
        } else {
            self.coefficient.denominator().bits()
        };
        if width.checked_add(shift)? > MAX_TRIG_COEFFICIENT_BITS {
            return None;
        }
        let shift = usize::try_from(shift).ok()?;
        let (mut numerator, mut denominator) = (
            self.coefficient.numerator().clone(),
            self.coefficient.denominator().clone(),
        );
        if self.exponent >= 0 {
            numerator <<= shift;
        } else {
            denominator <<= shift;
        }
        Rational::from_bigint_fraction(
            BigInt::from_biguint(self.coefficient.sign(), numerator),
            denominator,
        ).ok()
    }

    /// For a pi term q*pi, return 2*q modulo four in [0, 4).
    fn twice_pi_residue(&self) -> Option<Rational> {
        let power = self.exponent.checked_add(1)?;
        let mut denominator = self.coefficient.denominator().clone();
        let power = if power < 0 {
            let shift = power.unsigned_abs();
            if denominator.bits().checked_add(shift)? > MAX_TRIG_COEFFICIENT_BITS {
                return None;
            }
            denominator <<= usize::try_from(shift).ok()?;
            0
        } else {
            u64::try_from(power).ok()?
        };
        // Modular powering depends on the exponent's bit length, not its
        // magnitude. The modulus needs at most the coefficient bit cap plus two.
        let modulus = &denominator << 2_usize;
        let mut numerator = (self.coefficient.numerator()
            * BigUint::from(2_u8).modpow(&BigUint::from(power), &modulus)) % &modulus;
        if self.coefficient.sign() == Sign::Minus && !numerator.is_zero() {
            numerator = &modulus - numerator;
        }
        Rational::from_bigint_fraction(BigInt::from_biguint(Sign::Plus, numerator), denominator).ok()
    }

    fn normalize(&mut self) -> Option<()> {
        if !bounded_trig_coefficient(&self.coefficient) {
            return None;
        }
        if self.coefficient.is_zero() {
            return Some(());
        }
        let numerator_twos = self.coefficient.numerator().trailing_zeros()?;
        let denominator_twos = self.coefficient.denominator().trailing_zeros()?;
        self.exponent = self
            .exponent
            .checked_add(i64::try_from(numerator_twos).ok()?)?
            .checked_sub(i64::try_from(denominator_twos).ok()?)?;
        self.coefficient = Rational::from_bigint_fraction(
            BigInt::from_biguint(
                self.coefficient.sign(),
                self.coefficient.numerator() >> numerator_twos,
            ),
            self.coefficient.denominator() >> denominator_twos,
        )
        .ok()?;
        Some(())
    }
}

#[derive(Default)]
struct PiLinearForm(Vec<PiLinearTerm>);

impl PiLinearForm {
    /// Decompose the angle modulo 2*pi as k*pi/2 + q*pi + r, with
    /// -1/4 <= q < 1/4. Both remaining coefficients are bounded exact rationals.
    fn reduced_angle(self) -> Option<(u8, Rational, Rational)> {
        let mut half_pi = Rational::zero();
        let mut residual = Rational::zero();
        for term in self.0 {
            if term.pi {
                half_pi = half_pi + term.twice_pi_residue()?;
                if !bounded_trig_coefficient(&half_pi) {
                    return None;
                }
            } else {
                residual = residual + term.rational_value()?;
                if !bounded_trig_coefficient(&residual) {
                    return None;
                }
            }
        }
        // Each residue is nonnegative and less than four, so truncation of
        // h+1/2 is nearest-integer rounding even for negative source terms.
        let nearest = (&half_pi + &*HALF_RATIONAL).trunc().to_integer_i64()?;
        let coefficient = (half_pi - Rational::new(nearest)) * &*HALF_RATIONAL;
        if !bounded_trig_coefficient(&coefficient) {
            return None;
        }
        Some((u8::try_from(nearest % 4).ok()?, coefficient, residual))
    }

    fn insert(&mut self, mut term: PiLinearTerm) -> Option<()> {
        loop {
            term.normalize()?;
            if term.coefficient.is_zero() {
                return Some(());
            }
            if let Some(index) = self
                .0
                .iter()
                .position(|old| old.pi == term.pi && old.exponent == term.exponent)
            {
                // Each operand obeys the coefficient bit cap. Check the bounded
                // temporary sum before it can be reused.
                term.coefficient = term.coefficient + self.0.swap_remove(index).coefficient;
            } else {
                if self.0.len() == MAX_TRIG_TERMS {
                    return None;
                }
                self.0.push(term);
                return Some(());
            }
        }
    }

    fn leaf(pi: bool, coefficient: Rational) -> Option<Self> {
        let mut result = Self::default();
        result.insert(PiLinearTerm {
            pi,
            coefficient,
            exponent: 0,
        })?;
        Some(result)
    }
}

fn trig_linear_form(node: &Computable, depth: u16, work: &mut usize) -> Option<PiLinearForm> {
    if depth == 0 || *work == 0 {
        return None;
    }
    *work -= 1;
    use Approximation::*;
    match &node.internal.approximation {
        One => PiLinearForm::leaf(false, Rational::one()),
        Int(n) if n.bits() <= MAX_TRIG_COEFFICIENT_BITS => {
            PiLinearForm::leaf(false, Rational::from_bigint(n.clone()))
        }
        Ratio(q) if bounded_trig_coefficient(q) => PiLinearForm::leaf(false, q.clone()),
        Constant(SharedConstant::Pi) => PiLinearForm::leaf(true, Rational::one()),
        Constant(SharedConstant::Tau) => PiLinearForm::leaf(true, Rational::new(2)),
        Negate(child) => {
            let mut form = trig_linear_form(child, depth - 1, work)?;
            for term in &mut form.0 {
                term.coefficient = -term.coefficient.clone();
            }
            Some(form)
        }
        Offset(child, shift) => {
            let mut form = trig_linear_form(child, depth - 1, work)?;
            for term in &mut form.0 {
                term.exponent = term.exponent.checked_add(i64::from(*shift))?;
            }
            Some(form)
        }
        Add(left, right) => {
            let mut form = trig_linear_form(left, depth - 1, work)?;
            for term in trig_linear_form(right, depth - 1, work)?.0 {
                form.insert(term)?;
            }
            Some(form)
        }
        Multiply(left, right) => {
            // Visit both children even for a zero factor. Unsupported
            // partial functions cannot be erased by an annihilator.
            let left = trig_linear_form(left, depth - 1, work)?;
            let right = trig_linear_form(right, depth - 1, work)?;
            let mut result = PiLinearForm::default();
            for a in &left.0 {
                for b in &right.0 {
                    *work = work.checked_sub(1)?;
                    if a.pi && b.pi {
                        return None;
                    }
                    result.insert(PiLinearTerm {
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

impl Computable {
    /// Prove `self = k*pi/2 (mod 2*pi)` without expanding binary offsets.
    /// The accepted grammar has only finite rational/pi leaves and total
    /// arithmetic, so a successful result also proves operand existence.
    /// Limits decline recognition; they never assert a negative fact.
    pub(crate) fn exact_half_pi_index(&self) -> Option<u8> {
        // Ordinary rational trig inputs need no symbolic allocation.
        match &self.internal.approximation {
            Approximation::One => return None,
            Approximation::Int(n) => return n.is_zero().then_some(0),
            Approximation::Ratio(q) => return q.is_zero().then_some(0),
            _ => {}
        }

        let form = trig_linear_form(self, 128, &mut 256)?;
        // Every normalized coefficient has odd numerator and denominator;
        // distinct binary exponents therefore cannot cancel the lowest one.
        // Rational residuals cannot equal a nonzero rational multiple of pi.
        // A pi term below exponent -1 prevents an integral half-pi index.
        if form.0.iter().any(|term| !term.pi || term.exponent < -1) {
            return None;
        }
        let mut residue = Rational::zero();
        for term in form.0 {
            let value = term.twice_pi_residue()?;
            residue = residue + value;
            if !bounded_trig_coefficient(&residue) {
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
