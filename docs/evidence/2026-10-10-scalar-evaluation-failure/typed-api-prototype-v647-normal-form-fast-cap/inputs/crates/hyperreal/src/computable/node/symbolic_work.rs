/// Cooperative work for one synchronous symbolic operation. Never store this
/// request-local switch in a node or retained certificate: a later call may
/// have a different allowance. These helpers cannot introduce caller scopes.
#[derive(Clone, Copy)]
struct SymbolicWork {
    metered: bool,
}

impl SymbolicWork {
    #[inline]
    fn current() -> Self {
        Self {
            metered: crate::computable::budget::is_active(),
        }
    }

    #[inline]
    fn visit(self) -> Option<()> {
        (!self.metered || charge_evaluation_work()).then_some(())
    }

    fn words(bits: u64) -> u64 {
        bits.div_ceil(64).max(1)
    }

    fn rational_words(value: &Rational) -> u64 {
        Self::words(
            value
                .numerator()
                .bits()
                .saturating_add(value.denominator().bits()),
        )
    }

    fn allocate_bits(self, bits: u64) -> Option<()> {
        (!self.metered || crate::computable::budget::charge(Self::words(bits))).then_some(())
    }

    fn read(self, value: &Rational) -> Option<()> {
        (!self.metered || crate::computable::budget::charge(Self::rational_words(value)))
            .then_some(())
    }

    fn integer(self, value: &BigInt) -> Option<()> {
        (!self.metered || crate::computable::budget::charge(Self::words(value.magnitude().bits())))
            .then_some(())
    }

    fn binary(self, left: &Rational, right: &Rational) -> Option<()> {
        // Weight both operand sizes before exact arithmetic or comparison.
        // A wide coefficient times a small scalar need not be charged as a
        // square of the wide size. Units are not instruction/time/byte bounds.
        (!self.metered
            || crate::computable::budget::charge(
                Self::rational_words(left).saturating_mul(Self::rational_words(right)),
            ))
        .then_some(())
    }

    fn root(self, value: &Rational) -> Option<()> {
        self.binary(value, value)
    }

    fn same(self, left: &Computable, right: &Computable) -> Option<bool> {
        if self.metered {
            Computable::budgeted_structural_eq(left, right)
        } else {
            Some(Computable::internal_structural_eq(left, right))
        }
    }

    fn rational_leaf(self, node: &Computable) -> Option<Option<Rational>> {
        Some(match &node.internal.approximation {
            Approximation::One => Some(Rational::one()),
            Approximation::Int(value) => {
                self.integer(value)?;
                Some(Rational::from_bigint(value.clone()))
            }
            Approximation::Ratio(value) => {
                self.read(value)?;
                Some(value.clone())
            }
            _ => None,
        })
    }
}
