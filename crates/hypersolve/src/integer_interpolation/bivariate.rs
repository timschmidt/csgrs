use super::*;

// Coefficients in the fiber variable are polynomials in the retained variable.
type Polynomial = Vec<Vec<BigInt>>;

/// Signed regular subresultant rows after the source polynomial, beginning
/// with its derivative. One common positive rational scale makes the input
/// integral. Brown's regular PRS divides only by squares of preceding leading
/// coefficient polynomials; exact division certifies every constructed row.
/// The caller must certify those leading coefficients on its selected root.
/// Degree gaps and nonrational payloads retain the general field algorithm.
pub(crate) fn regular_rational_fiber_sturm_rows(
    coefficients: &[Vec<Real>],
) -> Option<Vec<Vec<Vec<Real>>>> {
    let rationals = coefficients
        .iter()
        .flatten()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    let mut primitive = Rational::primitive_bigint_ratio(&rationals).into_iter();
    let first = trim(
        coefficients
            .iter()
            .map(|coefficient| {
                trim_integer_polynomial(
                    coefficient
                        .iter()
                        .map(|_| {
                            primitive
                                .next()
                                .expect("primitive integer ratios preserve coefficient count")
                        })
                        .collect(),
                )
            })
            .collect(),
    );
    if first.len() <= 2 {
        return None;
    }
    let mut second = first
        .iter()
        .enumerate()
        .skip(1)
        .map(|(power, coefficient)| {
            coefficient
                .iter()
                .map(|value| value * BigInt::from(power))
                .collect()
        })
        .collect::<Polynomial>();
    let mut sequence = vec![second.clone()];
    // The derivative has degree one less, so the initial Brown PRS sign is +1.
    let mut next = pseudo_remainder(first, &second)?;
    let mut leading = second.last()?.clone();
    while !next.is_empty() {
        let gap = second.len().checked_sub(next.len())?;
        // Regular PRS steps divide by a square. A degree gap uses a
        // different sign relation and remains on the general field path.
        if gap != 1 {
            return None;
        }
        sequence.push(next.clone());
        let divisor = product(&leading, &leading);
        let remainder = pseudo_remainder(second, &next)?;
        let remainder = remainder
            .iter()
            .map(|coefficient| {
                integer_polynomial_exact_quotient(coefficient, &divisor)
                    .map(trim_integer_polynomial)
            })
            .collect::<Option<Polynomial>>()?;
        leading = next.last()?.clone();
        second = next;
        next = trim(remainder);
    }
    Some(
        sequence
            .into_iter()
            .enumerate()
            .map(|(index, mut polynomial)| {
                // For consecutive degrees, -rem differs from the Brown PRS
                // row by a positive scale and signs +,-,-,+,+,-,-,... after F.
                if ((index + 1) / 2) % 2 == 1 {
                    for coefficient in &mut polynomial {
                        for value in coefficient {
                            *value = -std::mem::take(value);
                        }
                    }
                }
                polynomial
                    .into_iter()
                    .map(|coefficient| {
                        coefficient
                            .into_iter()
                            .map(Rational::from_bigint)
                            .map(Real::from)
                            .collect()
                    })
                    .collect()
            })
            .collect(),
    )
}

fn trim(mut polynomial: Polynomial) -> Polynomial {
    while polynomial
        .last()
        .is_some_and(|coefficient| is_zero_integer_polynomial(coefficient))
    {
        polynomial.pop();
    }
    polynomial
}

fn product(first: &[BigInt], second: &[BigInt]) -> Vec<BigInt> {
    let mut result = vec![BigInt::zero(); first.len() + second.len() - 1];
    for (i, first) in first.iter().enumerate() {
        for (j, second) in second.iter().enumerate() {
            result[i + j] += first * second;
        }
    }
    trim_integer_polynomial(result)
}

fn pseudo_remainder(mut dividend: Polynomial, divisor: &Polynomial) -> Option<Polynomial> {
    let leading = divisor.last()?;
    if divisor.len() == 1 {
        return Some(Vec::new());
    }
    let mut steps = dividend.len().checked_sub(divisor.len())?.checked_add(1)?;
    while dividend.len() >= divisor.len() {
        let shift = dividend.len() - divisor.len();
        let top = dividend.last()?.clone();
        for coefficient in &mut dividend {
            *coefficient = product(coefficient, leading);
        }
        for (index, coefficient) in divisor.iter().enumerate().take(divisor.len() - 1) {
            let subtract = product(coefficient, &top);
            let target = &mut dividend[shift + index];
            target.resize(target.len().max(subtract.len()), BigInt::zero());
            for (target, value) in target.iter_mut().zip(subtract) {
                *target -= value;
            }
            *target = trim_integer_polynomial(std::mem::take(target));
        }
        dividend.pop();
        dividend = trim(dividend);
        steps -= 1;
    }
    // A cancellation may skip a power during pseudo-division. Complete
    // the nominal square multiplier even when fewer eliminations were needed.
    for _ in 0..steps {
        for coefficient in &mut dividend {
            *coefficient = product(coefficient, leading);
        }
    }
    Some(dividend)
}
