impl Computable {
    /// Recognize a shared nonzero operand times its explicit inverse, in either factor order, and return exact one.
    /// Require identical node allocations and an already-retained positive/negative
    /// sign; do not walk or numerically refine to prove nonzero. Equal but independently
    /// allocated values are outside this shortcut, and zero/unknown signs decline.
    ///
    /// Because cancellation removes the entire graph, inspect all operand handles
    /// with a 32-pop budget, including repeated visits to shared descendants. All
    /// attached signals must be the same Arc allocation, regardless of their Boolean
    /// values. Retain that common signal on one; conflicting identities or budget
    /// failure return None. An asserted common flag is carried rather than polled.
    /// The explicit stack has eight inline slots and can spill; the pop counter is
    /// not a total arithmetic/allocation bound or a typed resource outcome.
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

    /// Recognize a²/a or (a*a)/a through shared nonzero node identity, in either factor order.
    /// Both product children and the inverse divisor must share one allocation with
    /// an immediate positive/negative sign. This optional shortcut does not prove
    /// equality of independently authored expressions or refine an unresolved sign.
    ///
    /// Retain the original operand node and its complete descendant graph. Reconcile
    /// only the five wrapper handles: product, inverse, both product children and
    /// divisor. Every attached signal must share one Arc identity; return a clone
    /// with that signal or decline. Descendant controls remain in the retained graph.
    /// Signals are not polled, so an asserted compatible flag does not prevent this
    /// exact rewrite. None is an unmatched/unproved/incompatible case, not inequality.
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
