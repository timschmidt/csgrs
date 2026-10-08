// Proof-only arithmetic in Q(sqrt(2), sqrt(3)). This does not rewrite the
// expression graph, request approximations, or trust cached Unknown/facts.
// The four coefficients multiply the independent basis 1, sqrt(2), sqrt(3),
// sqrt(6). Admission and arithmetic are bounded; failure is simply no proof.
struct TwelfthField([Rational; 4]);

impl TwelfthField {
    const BITS: u64 = 1024;

    fn bounded(q: &Rational) -> bool {
        q.numerator().bits().max(q.denominator().bits()) <= Self::BITS
    }

    fn checked(self) -> Option<Self> {
        self.0.iter().all(Self::bounded).then_some(self)
    }

    fn rational(q: Rational) -> Self {
        Self([q, Rational::zero(), Rational::zero(), Rational::zero()])
    }

    fn basis(index: usize, q: Rational) -> Self {
        let mut value = Self::rational(Rational::zero());
        value.0[index] = q;
        value
    }

    fn add(self, rhs: Self) -> Option<Self> {
        Self(std::array::from_fn(|i| &self.0[i] + &rhs.0[i])).checked()
    }

    fn scale(self, q: &Rational) -> Option<Self> {
        Self(self.0.map(|a| if a.is_zero() { a } else { a * q })).checked()
    }

    fn multiply(&self, rhs: &Self) -> Option<Self> {
        let mut result = Self::rational(Rational::zero());
        for i in 0..4 {
            if self.0[i].is_zero() {
                continue;
            }
            for j in 0..4 {
                if rhs.0[j].is_zero() {
                    continue;
                }
                let common = i & j;
                let factor =
                    (if common & 1 != 0 { 2 } else { 1 }) * (if common & 2 != 0 { 3 } else { 1 });
                let term = &self.0[i] * &rhs.0[j];
                let term = if factor == 1 {
                    term
                } else {
                    term * Rational::new(factor)
                };
                result.0[i ^ j] = &result.0[i ^ j] + term;
                if !Self::bounded(&result.0[i ^ j]) {
                    return None;
                }
            }
        }
        Some(result)
    }

    fn conjugate(&self, mask: usize) -> Self {
        Self(std::array::from_fn(|i| {
            if (i & mask).count_ones() & 1 == 0 {
                self.0[i].clone()
            } else {
                -self.0[i].clone()
            }
        }))
    }

    fn inverse(self) -> Option<Self> {
        // Product of the other three conjugates / field norm. A zero norm
        // cannot authorize division, even when a surrounding expression cancels.
        let numerator = self
            .conjugate(1)
            .multiply(&self.conjugate(2))?
            .multiply(&self.conjugate(3))?;
        let norm = self.multiply(&numerator)?;
        if norm.0[1..].iter().any(|q| !q.is_zero()) {
            return None;
        }
        numerator.scale(&norm.0[0].clone().inverse().ok()?)
    }

    fn rational_sqrt(q: Rational) -> Option<Self> {
        if q.sign() == Sign::Minus {
            return None;
        }
        // Test membership without integer factorization or approximate roots.
        for (i, d) in [1, 2, 3, 6].into_iter().enumerate() {
            let square = &q * Rational::fraction(1, d).ok()?;
            let n = square.numerator().sqrt();
            let den = square.denominator().sqrt();
            if &n * &n == *square.numerator() && &den * &den == *square.denominator() {
                let root = Rational::from_bigint_fraction(BigInt::from(n), den).ok()?;
                return Self::basis(i, root).checked();
            }
        }
        None
    }

    fn quadratic_sign(a: &Rational, b: &Rational) -> Sign {
        let sa = a.sign();
        let sb = b.sign();
        if sa == Sign::NoSign {
            return sb;
        }
        if sb == Sign::NoSign || sa == sb {
            return sa;
        }
        let norm = a * a - b * b * Rational::new(2);
        match norm.sign() {
            Sign::NoSign => Sign::NoSign,
            Sign::Plus => sa,
            Sign::Minus => sb,
        }
    }

    fn sign(&self) -> Sign {
        // A + B sqrt(3), A,B in Q(sqrt(2)). Squaring is only used when A and B
        // have opposite signs; otherwise it could select the wrong conjugate.
        let [a, b, c, d] = &self.0;
        let sa = Self::quadratic_sign(a, b);
        let sb = Self::quadratic_sign(c, d);
        if sa == Sign::NoSign {
            return sb;
        }
        if sb == Sign::NoSign || sa == sb {
            return sa;
        }
        let rational =
            a * a + b * b * Rational::new(2) - c * c * Rational::new(3) - d * d * Rational::new(6);
        let radical = a * b * Rational::new(2) - c * d * Rational::new(6);
        match Self::quadratic_sign(&rational, &radical) {
            Sign::NoSign => Sign::NoSign,
            Sign::Plus => sa,
            Sign::Minus => sb,
        }
    }

    fn sine(mut k: i32) -> Self {
        k = k.rem_euclid(24);
        let negative = k > 12;
        if negative {
            k -= 12;
        }
        if k > 6 {
            k = 12 - k;
        }
        let numerators = match k {
            0 => [0, 0, 0, 0],
            1 => [0, -1, 0, 1],
            2 => [2, 0, 0, 0],
            3 => [0, 2, 0, 0],
            4 => [0, 0, 2, 0],
            5 => [0, 1, 0, 1],
            6 => [4, 0, 0, 0],
            _ => unreachable!(),
        };
        Self(numerators.map(|n| Rational::fraction(if negative { -n } else { n }, 4).unwrap()))
    }

    fn tangent(k: i32) -> Option<Self> {
        // tan has period pi; the reflected second quadrant changes its sign.
        // These exact Q(sqrt(3)) values avoid a general degree-four inverse.
        // The odd half-turn pole is still rejected before any cancellation.
        let k = k.rem_euclid(12);
        let (k, sign) = if k > 6 { (12 - k, -1) } else { (k, 1) };
        let (a, b, denominator) = match k {
            0 => (0, 0, 1),
            1 => (2, -1, 1),
            2 => (0, 1, 3),
            3 => (1, 0, 1),
            4 => (0, 1, 1),
            5 => (2, 1, 1),
            6 => return None,
            _ => unreachable!(),
        };
        Some(Self([
            Rational::new(a * sign),
            Rational::zero(),
            Rational::fraction(b * sign, denominator).unwrap(),
            Rational::zero(),
        ]))
    }
}

struct TwelfthProof {
    remaining: usize,
}

impl TwelfthProof {
    fn charge(&mut self) -> Option<()> {
        self.remaining = self.remaining.checked_sub(1)?;
        Some(())
    }

    fn leaf(node: &Computable) -> Option<Rational> {
        match &node.internal.approximation {
            Approximation::One => Some(Rational::one()),
            Approximation::Int(n) if n.bits() <= TwelfthField::BITS => {
                Some(Rational::from_bigint(n.clone()))
            }
            Approximation::Ratio(q) if TwelfthField::bounded(q) => Some(q.clone()),
            _ => None,
        }
    }

    fn angle(&mut self, node: &Computable) -> Option<(Rational, Rational)> {
        self.charge()?;
        if let Some(q) = Self::leaf(node) {
            return Some((q, Rational::zero()));
        }
        let (r, p) = match &node.internal.approximation {
            Approximation::Constant(SharedConstant::Pi) => (Rational::zero(), Rational::one()),
            Approximation::Constant(SharedConstant::Tau) => (Rational::zero(), Rational::new(2)),
            Approximation::Negate(x) => {
                let (r, p) = self.angle(x)?;
                (-r, -p)
            }
            Approximation::Offset(x, shift) if shift.unsigned_abs() <= 1023 => {
                let (r, p) = self.angle(x)?;
                let scale = Computable::power_of_two_rational(*shift);
                (r * &scale, p * scale)
            }
            Approximation::Add(a, b) => {
                let (ar, ap) = self.angle(a)?;
                let (br, bp) = self.angle(b)?;
                (ar + br, ap + bp)
            }
            Approximation::Multiply(a, b) => {
                let (ar, ap) = self.angle(a)?;
                let (br, bp) = self.angle(b)?;
                if !ap.is_zero() && !bp.is_zero() {
                    return None;
                }
                (&ar * &br, ar * bp + ap * br)
            }
            Approximation::Inverse(x) => {
                let (r, p) = self.angle(x)?;
                if !p.is_zero() {
                    return None;
                }
                (r.inverse().ok()?, p)
            }
            _ => return None,
        };
        (TwelfthField::bounded(&r) && TwelfthField::bounded(&p)).then_some((r, p))
    }

    fn value(&mut self, node: &Computable) -> Option<TwelfthField> {
        self.charge()?;
        if let Some(q) = Self::leaf(node) {
            return Some(TwelfthField::rational(q));
        }
        match &node.internal.approximation {
            Approximation::Constant(SharedConstant::Sqrt2) => {
                Some(TwelfthField::basis(1, Rational::one()))
            }
            Approximation::Constant(SharedConstant::Sqrt3) => {
                Some(TwelfthField::basis(2, Rational::one()))
            }
            Approximation::Negate(x) => self.value(x)?.scale(&Rational::new(-1)),
            Approximation::Offset(x, shift) if shift.unsigned_abs() <= 1023 => self
                .value(x)?
                .scale(&Computable::power_of_two_rational(*shift)),
            Approximation::Add(a, b) => self.value(a)?.add(self.value(b)?),
            Approximation::Multiply(a, b) => self.value(a)?.multiply(&self.value(b)?),
            Approximation::Inverse(x) => self.value(x)?.inverse(),
            Approximation::Square(x) => {
                let x = self.value(x)?;
                x.multiply(&x)
            }
            Approximation::Sqrt(x) => {
                self.charge()?;
                TwelfthField::rational_sqrt(Self::leaf(x)?)
            }
            Approximation::PrescaledSin(x)
            | Approximation::PrescaledCos(x)
            | Approximation::PrescaledTan(x)
            | Approximation::PrescaledCot(x) => {
                let (r, p) = self.angle(x)?;
                if !r.is_zero() {
                    return None;
                }
                let turns = p * Rational::new(12);
                if !turns.denominator().is_one() {
                    return None;
                }
                let digits = (turns.numerator() % BigUint::from(24u8)).to_u32_digits();
                let k = digits.first().copied().unwrap_or(0) as i32;
                let k = if turns.sign() == Sign::Minus { -k } else { k };
                match &node.internal.approximation {
                    Approximation::PrescaledSin(_) => Some(TwelfthField::sine(k)),
                    Approximation::PrescaledCos(_) => Some(TwelfthField::sine(k + 6)),
                    Approximation::PrescaledTan(_) => TwelfthField::tangent(k),
                    Approximation::PrescaledCot(_) => TwelfthField::tangent(6 - k),
                    _ => unreachable!(),
                }
            }
            _ => None,
        }
    }

    fn contains_trig(node: &Computable, remaining: &mut usize) -> bool {
        let Some(next) = remaining.checked_sub(1) else {
            return false;
        };
        *remaining = next;
        match &node.internal.approximation {
            Approximation::PrescaledSin(_)
            | Approximation::PrescaledCos(_)
            | Approximation::PrescaledTan(_)
            | Approximation::PrescaledCot(_) => true,
            Approximation::Negate(x)
            | Approximation::Offset(x, _)
            | Approximation::Inverse(x)
            | Approximation::Square(x)
            | Approximation::Sqrt(x) => Self::contains_trig(x, remaining),
            Approximation::Add(a, b) | Approximation::Multiply(a, b) => {
                Self::contains_trig(a, remaining) || Self::contains_trig(b, remaining)
            }
            _ => false,
        }
    }
}

impl Computable {
    fn exact_twelfth_relation_sign(&self) -> Option<Sign> {
        // Allocation-free, bounded bypass. Pure algebraic expressions retain
        // their existing dispatch; this fallback is for relations involving trig.
        if !TwelfthProof::contains_trig(self, &mut 128) {
            return None;
        }
        Some(TwelfthProof { remaining: 128 }.value(self)?.sign())
    }
}
