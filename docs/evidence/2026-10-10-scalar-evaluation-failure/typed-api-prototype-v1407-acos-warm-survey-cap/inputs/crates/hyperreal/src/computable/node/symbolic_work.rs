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

    #[inline]
    fn available(self) -> Option<()> {
        (!self.metered || !evaluation_exhausted()).then_some(())
    }

    #[inline]
    fn rational(self, value: Rational) -> Option<Computable> {
        // Materialization may canonicalize a lazy value or copy an integer
        // numerator. Pay before either operation, including constructor folds.
        self.read(&value)?;
        Some(Computable::rational(value))
    }

    fn words(bits: u64) -> u64 {
        bits.div_ceil(64).max(1)
    }

    fn rational_words(value: &Rational) -> u64 {
        let (numerator, denominator) = value.stored_component_bits();
        Self::words(numerator.saturating_add(denominator))
    }

    #[inline]
    fn allocate_bits(self, bits: u64) -> Option<()> {
        (!self.metered || crate::computable::budget::charge(Self::words(bits))).then_some(())
    }

    #[inline]
    fn read(self, value: &Rational) -> Option<()> {
        (!self.metered || crate::computable::budget::charge(Self::rational_words(value)))
            .then_some(())
    }

    fn bounded_rational(self, value: &Rational, max_bits: u64) -> Option<Option<&Rational>> {
        let unreduced = value.is_internally_unreduced();
        let value = if unreduced {
            // A stored value can be wide but reduce to a supported rational.
            // Pay for that reduction before asking for canonical components;
            // rejecting its raw widths would lose an existing proof path.
            self.read(value)?;
            value.canonicalized_ref()
        } else {
            value
        };
        let (numerator, denominator) = value.stored_component_bits();
        if numerator > max_bits || denominator > max_bits {
            return Some(None);
        }
        if !unreduced {
            // Canonical oversized values keep their allocation-free local
            // rejection without spending coefficient work.
            self.read(value)?;
        }
        Some(Some(value))
    }

    #[inline]
    fn integer(self, value: &BigInt) -> Option<()> {
        (!self.metered || crate::computable::budget::charge(Self::words(value.magnitude().bits())))
            .then_some(())
    }

    #[inline]
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

    #[inline]
    fn root(self, value: &Rational) -> Option<()> {
        self.binary(value, value)
    }

    #[inline]
    fn same(self, left: &Computable, right: &Computable) -> Option<bool> {
        if self.metered {
            Computable::budgeted_structural_eq(left, right)
        } else {
            Some(Computable::internal_structural_eq(left, right))
        }
    }

    #[inline]
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

    #[inline]
    fn power_of_two(self, shift: Precision) -> Option<Rational> {
        // Preserve representation-limit rejection before an allocation debit.
        let bits = shift.unsigned_abs();
        if bits > Computable::MAX_APPROXIMATION_BITS as u32 {
            return None;
        }
        self.allocate_bits(u64::from(bits) + 1)?;
        Computable::power_of_two_rational(shift)
    }
}
