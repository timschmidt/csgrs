/// Reduce a finite operand numerically without constructing another trig node.
/// A one-unit sample of 4*x/(pi/2), rounded to an integer, is within 3/4 of
/// the exact quotient. The remaining angle therefore has magnitude at most
/// 3*pi/8 < 3/2, within the shared sine/cosine series range.
fn sin_cos_finite_operand(
    signal: &Option<Signal>,
    argument: &Computable,
    p: Precision,
) -> Result<(BigInt, BigInt), Problem> {
    let p = crate::computable::checked_approximation_precision(p)?;
    if p >= 1 {
        return Ok((BigInt::zero(), BigInt::zero()));
    }
    if should_stop(signal) {
        return Err(Problem::Exhausted);
    }
    let ratio = argument.clone().multiply(
        Computable::pi()
            .multiply(Computable::rational(Rational::fraction(1, 2).unwrap()))
            .inverse(),
    );
    let multiple = scale(ratio.approx_signal(signal, -2)?, -2);
    let op_prec = p.checked_sub(4).ok_or(Problem::OutOfRange)?;
    let residual = if multiple.is_zero() {
        argument.approx_signal(signal, op_prec)?
    } else {
        // The original operand and the independently enclosed pi multiple
        // are subtracted at guard precision. This stays ordinary arithmetic,
        // so periodic normalization cannot undo this numerical reduction.
        let work = op_prec.checked_sub(3).ok_or(Problem::OutOfRange)?;
        let value = argument.approx_signal(signal, work)?;
        let bits =
            Precision::try_from(multiple.magnitude().bits()).map_err(|_| Problem::OutOfRange)?;
        let pi_precision = work
            .checked_sub(bits)
            .and_then(|p| p.checked_sub(4))
            .ok_or(Problem::OutOfRange)?;
        let pi = Computable::pi().approx_signal(signal, pi_precision)?;
        let exponent = pi_precision
            .checked_sub(work)
            .and_then(|p| p.checked_sub(1))
            .ok_or(Problem::OutOfRange)?;
        scale(value - scale(pi * &multiple, exponent), -3)
    };
    let (sine, cosine) = sin_cos_scaled_argument(signal, residual, op_prec, p)?;
    Ok(match large_rational_quadrant(&multiple).to_u8() {
        Some(0) => (sine, cosine),
        Some(1) => (cosine, -sine),
        Some(2) => (-sine, -cosine),
        Some(3) => (-cosine, sine),
        _ => unreachable!("integer quadrant is reduced modulo four"),
    })
}

fn sin_cos_pi_operand(
    signal: &Option<Signal>,
    original: &Computable,
    p: Precision,
) -> Result<Option<(BigInt, BigInt)>, Problem> {
    let Some(reduction) = original.pi_operand_reduction() else {
        return Ok(None);
    };
    // A partial atom may have cancelled algebraically. Its original domain
    // still has to exist, including when the reduced argument is exact zero.
    original.ensure_finite_real(signal)?;
    let (sine, cosine) = sin_cos_finite_operand(signal, &reduction.argument, p)?;
    Ok(Some(match reduction.quadrant {
        0 => (sine, cosine),
        1 => (cosine, -sine),
        2 => (-sine, -cosine),
        3 => (-cosine, sine),
        _ => unreachable!("symbolic quadrant is reduced modulo four"),
    }))
}
