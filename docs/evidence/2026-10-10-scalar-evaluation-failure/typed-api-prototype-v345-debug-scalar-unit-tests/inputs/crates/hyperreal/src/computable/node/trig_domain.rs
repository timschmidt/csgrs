impl Computable {
    /// A sufficient exact-zero certificate for a finite real trig expression.
    /// Callers must establish the domain of every operand before using this
    /// identity. Exhausting the structural budget leaves the result unknown.
    fn has_trig_zero_certificate_when_real(&self) -> bool {
        const MAX_BITS: u64 = 4_096;

        fn pi_coefficient(node: &Computable, depth: u8, work: &mut u8) -> Option<Rational> {
            if depth == 0 || *work == 0 {
                return None;
            }
            *work -= 1;
            use Approximation::*;
            let value = match &node.internal.approximation {
                Constant(SharedConstant::Pi) => Rational::one(),
                Constant(SharedConstant::Tau) => Rational::new(2),
                Int(n) if n.is_zero() => Rational::zero(),
                Ratio(r) if r.sign() == Sign::NoSign => Rational::zero(),
                Negate(child) => -pi_coefficient(child, depth - 1, work)?,
                Add(left, right) => {
                    pi_coefficient(left, depth - 1, work)? + pi_coefficient(right, depth - 1, work)?
                }
                Multiply(left, right) => {
                    if let Some(scale) = left.bounded_rational_value() {
                        scale * pi_coefficient(right, depth - 1, work)?
                    } else {
                        pi_coefficient(left, depth - 1, work)? * right.bounded_rational_value()?
                    }
                }
                Offset(child, shift) if shift.unsigned_abs() <= MAX_BITS as u32 => {
                    let value = pi_coefficient(child, depth - 1, work)?;
                    if *shift < 0 {
                        value.divide_by_power_of_two(-shift)?
                    } else {
                        value * Rational::from_bigint(BigInt::one() << *shift as usize)
                    }
                }
                _ => return None,
            };
            // Operands and shifts are bounded before arithmetic; even rejected
            // temporary coefficients have bounded width. Never expand an
            // arbitrary Offset through the unrestricted pi algebra recognizer.
            (value.numerator().bits() <= MAX_BITS && value.denominator().bits() <= MAX_BITS)
                .then_some(value)
        }

        fn visit(node: &Computable, depth: u8, work: &mut u8) -> bool {
            if depth == 0 || *work == 0 {
                return false;
            }
            *work -= 1;
            use Approximation::*;
            match &node.internal.approximation {
                Negate(child) | Offset(child, _) => visit(child, depth - 1, work),
                PrescaledSin(child) | SinDeferred(child) => {
                    child
                        .exact_half_pi_index()
                        .is_some_and(|index| index % 2 == 0)
                        || child
                            .bounded_rational_value()
                            .is_some_and(|r| r.sign() == Sign::NoSign)
                        || pi_coefficient(child, depth - 1, work).is_some_and(|q| q.is_integer())
                }
                PrescaledCos(child) | CosDeferred(child) => {
                    child
                        .exact_half_pi_index()
                        .is_some_and(|index| index % 2 == 1)
                        || pi_coefficient(child, depth - 1, work)
                            .and_then(|q| (q * Rational::new(2)).to_big_integer())
                            .is_some_and(|twice| twice.magnitude().bit(0))
                }
                _ => false,
            }
        }

        visit(self, 32, &mut 64)
    }
}
