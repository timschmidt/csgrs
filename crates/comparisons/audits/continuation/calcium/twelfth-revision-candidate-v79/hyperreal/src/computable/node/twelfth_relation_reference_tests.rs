#[cfg(test)]
mod twelfth_relation_reference_tests {
    use super::*;

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
            Self(self.0.map(|a| a * q)).checked()
        }

        fn multiply(&self, rhs: &Self) -> Option<Self> {
            let mut result = Self::rational(Rational::zero());
            for i in 0..4 {
                for j in 0..4 {
                    let common = i & j;
                    let factor = (if common & 1 != 0 { 2 } else { 1 })
                        * (if common & 2 != 0 { 3 } else { 1 });
                    let term = &self.0[i] * &rhs.0[j] * Rational::new(factor);
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
            let rational = a * a + b * b * Rational::new(2)
                - c * c * Rational::new(3)
                - d * d * Rational::new(6);
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
                Approximation::Constant(SharedConstant::Tau) => {
                    (Rational::zero(), Rational::new(2))
                }
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
                    let sine = TwelfthField::sine(k);
                    let cosine = TwelfthField::sine(k + 6);
                    match &node.internal.approximation {
                        Approximation::PrescaledSin(_) => Some(sine),
                        Approximation::PrescaledCos(_) => Some(cosine),
                        Approximation::PrescaledTan(_) => sine.multiply(&cosine.inverse()?),
                        Approximation::PrescaledCot(_) => cosine.multiply(&sine.inverse()?),
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

    fn raw(approximation: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    fn same_value(node: &Computable) {
        let old = TwelfthProof { remaining: 128 }.value(node);
        let new = super::TwelfthProof { remaining: 128 }.value(node);
        assert_eq!(old.as_ref().map(|x| &x.0), new.as_ref().map(|x| &x.0));
        let expected = if TwelfthProof::contains_trig(node, &mut 128) {
            old.map(|x| x.sign())
        } else {
            None
        };
        assert_eq!(node.exact_twelfth_relation_sign(), expected);
    }

    #[test]
    fn sparse_arithmetic_matches_frozen_dense_reference() {
        let mut count = 0;
        for left_mask in 0..16 {
            for right_mask in 0..16 {
                for seed in 0..4 {
                    let a = TwelfthField(std::array::from_fn(|i| {
                        Rational::fraction(
                            if left_mask & (1 << i) == 0 {
                                0
                            } else {
                                (seed * 7 + i as i64 * 3) % 13 - 6
                            },
                            (i + 1) as u64,
                        )
                        .unwrap()
                    }));
                    let b = TwelfthField(std::array::from_fn(|i| {
                        Rational::fraction(
                            if right_mask & (1 << i) == 0 {
                                0
                            } else {
                                (seed * 11 + i as i64 * 5) % 17 - 8
                            },
                            (i + 2) as u64,
                        )
                        .unwrap()
                    }));
                    let new_a = || super::TwelfthField(a.0.clone());
                    let new_b = || super::TwelfthField(b.0.clone());
                    assert_eq!(
                        a.multiply(&b).map(|x| x.0),
                        new_a().multiply(&new_b()).map(|x| x.0)
                    );
                    assert_eq!(
                        TwelfthField(a.0.clone()).inverse().map(|x| x.0),
                        new_a().inverse().map(|x| x.0)
                    );
                    for q in [
                        Rational::zero(),
                        Rational::one(),
                        Rational::fraction(-3, 7).unwrap(),
                    ] {
                        assert_eq!(
                            TwelfthField(a.0.clone()).scale(&q).map(|x| x.0),
                            new_a().scale(&q).map(|x| x.0)
                        );
                    }
                    count += 1;
                }
            }
        }
        assert_eq!(count, 1024);
        // Include internal out-of-bound operands to ensure zero elision cannot
        // hide an over-budget nonzero accumulator or change admission.
        for bits in [0, 63, 64, 127, 128, 511, 1022, 1023, 1024, 1025] {
            for mask in 0..16 {
                let a = TwelfthField(std::array::from_fn(|i| {
                    if mask & (1 << i) == 0 {
                        Rational::zero()
                    } else {
                        Rational::from_bigint(
                            (BigInt::one() << bits) * BigInt::from(if i % 2 == 0 { 1 } else { -1 }),
                        )
                    }
                }));
                let b = TwelfthField([1, -1, 0, 1].map(Rational::new));
                let new_a = || super::TwelfthField(a.0.clone());
                let new_b = || super::TwelfthField(b.0.clone());
                assert_eq!(
                    a.multiply(&b).map(|x| x.0),
                    new_a().multiply(&new_b()).map(|x| x.0)
                );
                for q in [
                    Rational::zero(),
                    Rational::one(),
                    Rational::fraction(1, 2).unwrap(),
                ] {
                    assert_eq!(
                        TwelfthField(a.0.clone()).scale(&q).map(|x| x.0),
                        new_a().scale(&q).map(|x| x.0)
                    );
                }
            }
        }
    }

    #[test]
    fn direct_tangent_matches_quotients_and_exact_poles() {
        for k in (-240..=240).chain([i32::MIN, i32::MIN + 1, i32::MAX - 1, i32::MAX]) {
            let r = k.rem_euclid(24);
            let sine = TwelfthField::sine(r);
            let cosine = TwelfthField::sine(r + 6);
            let expected = cosine.inverse().and_then(|v| sine.multiply(&v));
            let actual = super::TwelfthField::tangent(k);
            assert_eq!(actual.is_none(), k.rem_euclid(12) == 6, "k={k}");
            assert_eq!(actual.map(|x| x.0), expected.map(|x| x.0), "k={k}");
            let sine = TwelfthField::sine(r);
            let cosine = TwelfthField::sine(r + 6);
            let cotangent = sine.inverse().and_then(|v| cosine.multiply(&v));
            let actual = super::TwelfthField::tangent(6 - r);
            assert_eq!(actual.is_none(), k.rem_euclid(12) == 0, "cot k={k}");
            assert_eq!(actual.map(|x| x.0), cotangent.map(|x| x.0), "cot k={k}");
        }
    }

    #[test]
    fn revised_evaluation_preserves_frozen_domain_and_values() {
        let mut count = 0;
        for k in -24..=48 {
            for period in [BigInt::zero(), BigInt::from(5), BigInt::one() << 1000usize] {
                let q = Rational::from_bigint_fraction(
                    BigInt::from(k) + period * BigInt::from(24),
                    BigUint::from(12u8),
                )
                .unwrap();
                for operation in 0..4 {
                    let angle = raw(Approximation::Multiply(
                        Computable::pi(),
                        Computable::rational(q.clone()),
                    ));
                    let trig = raw(match operation {
                        0 => Approximation::PrescaledSin(angle),
                        1 => Approximation::PrescaledCos(angle),
                        2 => Approximation::PrescaledTan(angle),
                        _ => Approximation::PrescaledCot(angle),
                    });
                    for expression in [
                        trig.clone(),
                        raw(Approximation::Negate(trig.clone())),
                        raw(Approximation::Offset(trig.clone(), -1000)),
                        raw(Approximation::Offset(trig.clone(), 1000)),
                        raw(Approximation::Offset(trig.clone(), 1030)),
                        raw(Approximation::Add(
                            trig.clone(),
                            raw(Approximation::Constant(SharedConstant::Sqrt2)),
                        )),
                        raw(Approximation::Multiply(
                            trig.clone(),
                            raw(Approximation::Constant(SharedConstant::Sqrt3)),
                        )),
                        raw(Approximation::Multiply(trig.clone(), Computable::zero())),
                        raw(Approximation::Inverse(trig.clone())),
                        raw(Approximation::Square(trig)),
                    ] {
                        same_value(&expression);
                        count += 1;
                    }
                }
            }
        }
        assert_eq!(count, 8760);
        for denominator in [7, 11, 13, 24, 25] {
            let angle = raw(Approximation::Multiply(
                Computable::pi(),
                Computable::rational(Rational::fraction(1, denominator).unwrap()),
            ));
            let trig = raw(Approximation::PrescaledSin(angle));
            same_value(&trig);
            same_value(&raw(Approximation::Multiply(
                Computable::zero(),
                trig.clone(),
            )));
            let mut deep = trig;
            for _ in 0..132 {
                deep = raw(Approximation::Negate(deep));
                same_value(&deep);
            }
        }
    }
}
