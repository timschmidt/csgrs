// Shared nonzero operands admit reciprocal identities without expanding them.
// Each rewrite preserves the cancellation controls of every removed node.
impl Computable {
    fn shared_reciprocal_product(&self, other: &Self) -> Option<Self> {
        for (value, inverse) in [(self, other), (other, self)] {
            let Approximation::Inverse(divisor) = &inverse.internal.approximation else {
                continue;
            };
            if !Arc::ptr_eq(&value.internal, &divisor.internal)
                || !matches!(
                    divisor.immediate_sign(),
                    Some(RealSign::Positive | RealSign::Negative)
                )
            {
                continue;
            }
            // This identity removes the complete operand graph. Account for
            // every removed handle, including descendant handles, before
            // replacing it by one. Decline on conflicting control identities
            // or a graph larger than this optional inspection budget.
            let mut signal: Option<Signal> = None;
            let mut stack = InlineStack::<&Self, 8>::new();
            stack.push(value);
            stack.push(inverse);
            let mut remaining = 32_usize;
            while let Some(node) = stack.pop() {
                remaining = remaining.checked_sub(1)?;
                if let Some(candidate) = &node.signal {
                    if signal
                        .as_ref()
                        .is_some_and(|previous| !Arc::ptr_eq(previous, candidate))
                    {
                        return None;
                    }
                    signal = Some(candidate.clone());
                }
                node.internal
                    .approximation
                    .for_each_operand(|child| stack.push(child));
            }
            let mut result = Self::one();
            result.signal = signal;
            return Some(result);
        }
        None
    }

    // a²/a=a keeps the complete shared descendant graph; only the removed
    // wrappers need their control identities reconciled.
    fn shared_square_quotient(&self, other: &Self) -> Option<Self> {
        for (product, inverse) in [(self, other), (other, self)] {
            let Approximation::Inverse(divisor) = &inverse.internal.approximation else {
                continue;
            };
            let (first, second) = match &product.internal.approximation {
                Approximation::Square(value) => (value, value),
                Approximation::Multiply(first, second) => (first, second),
                _ => continue,
            };
            if !Arc::ptr_eq(&first.internal, &divisor.internal)
                || !Arc::ptr_eq(&second.internal, &divisor.internal)
                || !matches!(
                    divisor.immediate_sign(),
                    Some(RealSign::Positive | RealSign::Negative)
                )
            {
                continue;
            }
            let mut signal: Option<Signal> = None;
            let mut compatible = true;
            for candidate in [
                &product.signal,
                &inverse.signal,
                &first.signal,
                &second.signal,
                &divisor.signal,
            ]
            .into_iter()
            .flatten()
            {
                if signal
                    .as_ref()
                    .is_some_and(|previous| !Arc::ptr_eq(previous, candidate))
                {
                    compatible = false;
                    break;
                }
                signal = Some(candidate.clone());
            }
            if compatible {
                let mut result = first.clone();
                result.signal = signal;
                return Some(result);
            }
        }
        None
    }
}
