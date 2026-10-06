use super::*;

// Coefficients in the fiber variable are polynomials in the retained variable.
type Polynomial = Vec<Vec<BigInt>>;

/// Subresultant rows after the first polynomial, beginning with the second.
/// Independent positive rational scales make the inputs integral. Brown's PRS
/// uses only exact polynomial division in Z[t], including abnormal degree gaps.
/// The caller must certify every retained leading coefficient at its selected
/// root before using the sequence there. The callback stops on `Some(false)`
/// after selected termination; `None` declines the sequence. Thus unused
/// rows are never constructed. No irreducibility is assumed.
pub(crate) fn rational_fiber_subresultants(
    first: &[Vec<Real>],
    second: &[Vec<Real>],
    mut retain: impl FnMut(Vec<Vec<Real>>) -> Option<bool>,
) -> Option<()> {
    let first = integer_coefficients(first)?;
    let mut second = integer_coefficients(second)?;
    let mut emit = |row: &Polynomial| {
        retain(
            row.iter()
                .map(|coefficient| {
                    coefficient
                        .iter()
                        .cloned()
                        .map(Rational::from_bigint)
                        .map(Real::from)
                        .collect()
                })
                .collect(),
        )
    };
    if !emit(&second)? {
        return Some(());
    }
    let gap = first.len().checked_sub(second.len())?;
    let mut next = pseudo_remainder(first, &second)?;
    if gap % 2 == 0 {
        negate(&mut next);
    }
    let mut leading = second.last()?.clone();
    let mut scale = power(&leading, gap);
    for value in &mut scale {
        *value = -std::mem::take(value);
    }
    while !next.is_empty() {
        let gap = second.len().checked_sub(next.len())?;
        if !emit(&next)? || next.len() == 1 {
            break;
        }
        let mut divisor = product(&leading, &power(&scale, gap));
        for value in &mut divisor {
            *value = -std::mem::take(value);
        }
        let remainder = pseudo_remainder(second, &next)?;
        let remainder = remainder
            .iter()
            .map(|coefficient| {
                integer_polynomial_exact_quotient(coefficient, &divisor)
                    .map(trim_integer_polynomial)
            })
            .collect::<Option<Polynomial>>()?;
        leading = next.last()?.clone();
        let negative_leading = leading.iter().map(|value| -value).collect::<Vec<_>>();
        scale = if gap > 1 {
            integer_polynomial_exact_quotient(
                &power(&negative_leading, gap),
                &power(&scale, gap - 1),
            )?
        } else {
            negative_leading
        };
        second = next;
        next = trim(remainder);
    }
    Some(())
}

fn integer_coefficients(coefficients: &[Vec<Real>]) -> Option<Polynomial> {
    let rationals = coefficients
        .iter()
        .flatten()
        .map(Real::exact_rational_ref)
        .collect::<Option<Vec<_>>>()?;
    let mut primitive = Rational::primitive_bigint_ratio(&rationals).into_iter();
    Some(trim(
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
    ))
}

fn power(polynomial: &[BigInt], exponent: usize) -> Vec<BigInt> {
    let mut result = vec![BigInt::one()];
    for _ in 0..exponent {
        result = product(&result, polynomial);
    }
    result
}

fn negate(polynomial: &mut Polynomial) {
    for coefficient in polynomial {
        for value in coefficient {
            *value = -std::mem::take(value);
        }
    }
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
    // the nominal leading-coefficient multiplier even when fewer eliminations were needed.
    for _ in 0..steps {
        for coefficient in &mut dividend {
            *coefficient = product(coefficient, leading);
        }
    }
    Some(dividend)
}
