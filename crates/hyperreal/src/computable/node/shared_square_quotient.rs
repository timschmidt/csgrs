// A directly shared nonzero value satisfies a²/a=a without expanding a.
// The returned value keeps the entire shared descendant graph. Different
// root abort signals on removed wrappers make this optional rewrite decline.
impl Computable {
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
