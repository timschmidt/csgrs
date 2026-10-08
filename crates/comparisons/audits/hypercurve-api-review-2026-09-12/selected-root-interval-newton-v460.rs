
/// Every box contains the caller's selected root. Interval Newton intersects
/// that box with m - P(m)/P'(box); the mean value theorem preserves ownership.
/// Outward dyadic rounding bounds denominator growth without changing P or
/// replacing the caller's root certificate. Failure only declines this filter.
fn sign_on_refined_singleton(
    defining: &[Real],
    predicate: &[Real],
    interval: &IsolatedRootInterval,
) -> Option<Ordering> {
    use hyperreal::Rational;

    fn enclose(values: &[Real], precision: i32) -> Option<Vec<[Rational; 2]>> {
        values.iter().map(|value| value.certified_dyadic_interval(precision)).collect()
    }

    fn evaluate(polynomial: &[[Rational; 2]], point: &[Rational; 2]) -> [Rational; 2] {
        let mut value = [Rational::zero(), Rational::zero()];
        for [lower, upper] in polynomial.iter().rev() {
            let (lo, hi) = crate::interval::rational_interval_product(
                &value[0], &value[1], &point[0], &point[1],
            );
            value = [lo + lower, hi + upper];
        }
        value
    }

    fn separated([lower, upper]: &[Rational; 2]) -> Option<Ordering> {
        if lower.is_positive() {
            Some(Ordering::Greater)
        } else if upper.is_negative() {
            Some(Ordering::Less)
        } else if lower.is_zero() && upper.is_zero() {
            Some(Ordering::Equal)
        } else {
            None
        }
    }

    let mut retained: Option<[Rational; 2]> = None;
    // A bounded proof filter; inseparable values still use the exact chain.
    for precision in [-64, -128, -256, -512, -1024, -2048] {
        let mut bounds = [
            interval.lower.certified_dyadic_interval(precision)?[0].clone(),
            interval.upper.certified_dyadic_interval(precision)?[1].clone(),
        ];
        if let Some(previous) = retained.take() {
            if previous[0] > bounds[0] { bounds[0] = previous[0].clone(); }
            if previous[1] < bounds[1] { bounds[1] = previous[1].clone(); }
        }
        if bounds[0] > bounds[1] {
            return None;
        }
        let query = enclose(predicate, precision)?;
        if let Some(ordering) = separated(&evaluate(&query, &bounds)) {
            return Some(ordering);
        }
        let polynomial = enclose(defining, precision)?;
        let derivative = polynomial.iter().enumerate().skip(1).map(|(power, [lo, hi])| {
            let power = Rational::new(i64::try_from(power).ok()?);
            Some([lo * &power, hi * &power])
        }).collect::<Option<Vec<_>>>()?;
        for _ in 0..2 {
            let slope = evaluate(&derivative, &bounds);
            if !slope[0].is_positive() && !slope[1].is_negative() {
                return None;
            }
            let midpoint = (&bounds[0] + &bounds[1]) * Rational::fraction(1, 2).ok()?;
            let residual = evaluate(&polynomial, &[midpoint.clone(), midpoint.clone()]);
            let (lo, hi) = crate::interval::rational_interval_product(
                &residual[0], &residual[1],
                &slope[1].clone().inverse().ok()?, &slope[0].clone().inverse().ok()?,
            );
            let lower = Real::new(&midpoint - hi).certified_dyadic_interval(precision)?[0].clone();
            let upper = Real::new(&midpoint - lo).certified_dyadic_interval(precision)?[1].clone();
            if lower > bounds[0] { bounds[0] = lower; }
            if upper < bounds[1] { bounds[1] = upper; }
            if bounds[0] > bounds[1] {
                return None;
            }
        }
        if let Some(ordering) = separated(&evaluate(&query, &bounds)) {
            return Some(ordering);
        }
        retained = Some(bounds);
    }
    None
}
