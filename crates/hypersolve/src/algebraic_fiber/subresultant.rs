use super::*;

/// Failure to construct a subresultant over a retained algebraic fiber.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AlgebraicFiberSubresultantError {
    /// The selected base root did not replay its singleton evidence.
    InvalidEvidence,
    /// An input is empty or the source fiber is constant.
    InvalidPolynomial,
    /// The order exceeds one of the supplied polynomial degrees.
    InvalidOrder,
    /// An exact coefficient decision or arithmetic operation did not finish.
    Undecided,
}

/// Constructs a subresultant in `z` modulo `P(alpha)` and `F(alpha,u)`.
///
/// Each input stores its `z` coefficients in ascending order as polynomials
/// in `(alpha,u)`. The caller must certify that its last coefficient is
/// nonzero at the selected source root before using the subresultant sequence
/// as a GCD certificate. The source fiber may contain other conjugates.
///
/// Ring arithmetic reduces every product by the retained equations. No global
/// source projection or interpolation is needed, and no inverse in the source
/// quotient ring is taken. One common nonzero scale clears base-field
/// denominators across all returned coefficients, preserving the polynomial's
/// roots and its specialized GCD meaning. All evidence is replayed strictly.
pub fn subresultant_in_algebraic_fiber(
    first: &[BivariatePolynomial],
    second: &[BivariatePolynomial],
    order: usize,
    fiber_equation: &BivariatePolynomial,
    retained_root: &AlgebraicRootRepresentation,
) -> Result<Vec<BivariatePolynomial>, AlgebraicFiberSubresultantError> {
    use AlgebraicFiberSubresultantError as Error;
    if first.is_empty() || second.is_empty() {
        return Err(Error::InvalidPolynomial);
    }
    let first_degree = first.len() - 1;
    let second_degree = second.len() - 1;
    if order > first_degree.min(second_degree) {
        return Err(Error::InvalidOrder);
    }
    let mut field =
        LocalAlgebraicField::new(retained_root, PredicatePolicy::STRICT).map_err(|error| {
            match error {
                LocalFieldError::InvalidEvidence => Error::InvalidEvidence,
                _ => Error::Undecided,
            }
        })?;
    let construct = |field: &mut LocalAlgebraicField| -> Result<_, LocalFieldError> {
        let fiber = local_fiber_polynomial(fiber_equation, CurveResultantParameter::First, field)?;
        if fiber.len() <= 1 {
            return Ok(None);
        }
        let mut reduce = |coefficients: &[BivariatePolynomial]| {
            coefficients
                .iter()
                .map(|coefficient| {
                    let coefficient =
                        local_fiber_polynomial(coefficient, CurveResultantParameter::First, field)?;
                    local_polynomial_remainder(coefficient, &fiber, field)
                })
                .collect::<Result<Vec<_>, LocalFieldError>>()
        };
        let first = reduce(first)?;
        let second = reduce(second)?;
        let coefficients = if order > 0 && order == first_degree.min(second_degree) {
            if first_degree < second_degree {
                first
            } else {
                second
            }
        } else {
            let first_rows = second_degree - order;
            let second_rows = first_degree - order;
            let rows = first_rows
                .checked_add(second_rows)
                .ok_or(LocalFieldError::Undecided)?;
            let columns = rows.checked_add(order).ok_or(LocalFieldError::Undecided)?;
            let size = rows
                .checked_mul(columns)
                .ok_or(LocalFieldError::Undecided)?;
            let mut matrix = Vec::new();
            matrix
                .try_reserve_exact(size)
                .map_err(|_| LocalFieldError::Undecided)?;
            matrix.resize_with(size, local_image_polynomial_zero);
            for row in 0..first_rows {
                for (power, coefficient) in first.iter().enumerate() {
                    matrix[row * columns + row + first_degree - power] = coefficient.clone();
                }
            }
            for row in 0..second_rows {
                for (power, coefficient) in second.iter().enumerate() {
                    matrix[(first_rows + row) * columns + row + second_degree - power] =
                        coefficient.clone();
                }
            }
            let mut coefficients = Vec::with_capacity(order + 1);
            for power in 0..=order {
                let mut minor =
                    Vec::with_capacity(rows.checked_mul(rows).ok_or(LocalFieldError::Undecided)?);
                for row in 0..rows {
                    for column in 0..rows.saturating_sub(1) {
                        minor.push(matrix[row * columns + column].clone());
                    }
                    minor.push(matrix[row * columns + columns - 1 - power].clone());
                }
                coefficients.push(local_polynomial_matrix_determinant(
                    &minor,
                    rows,
                    Some(&fiber),
                    field,
                )?);
            }
            coefficients
        };
        // Clear denominators together: scaling each z coefficient separately
        // would change the very polynomial whose roots the caller replays.
        let lengths = coefficients.iter().map(Vec::len).collect::<Vec<_>>();
        let flattened = coefficients.into_iter().flatten().collect::<Vec<_>>();
        let mut cleared = local_polynomial_clear_denominators(flattened, field)?.into_iter();
        let coefficients = lengths
            .into_iter()
            .map(|length| {
                local_fiber_to_bivariate(
                    (0..length)
                        .map(|_| cleared.next().unwrap_or_else(LocalFieldElement::zero))
                        .collect(),
                    CurveResultantParameter::First,
                )
                .ok_or(LocalFieldError::Undecided)
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Some(coefficients))
    };
    construct(&mut field)
        .map_err(|_| Error::Undecided)?
        .ok_or(Error::InvalidPolynomial)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn coefficient(rows: &[&[i32]]) -> BivariatePolynomial {
        BivariatePolynomial::new(
            rows.iter()
                .map(|row| row.iter().copied().map(Real::from).collect())
                .collect(),
        )
    }

    fn retained_root() -> AlgebraicRootRepresentation {
        super::super::tests::represented_root(
            vec![Real::from(-2), Real::zero(), Real::one()],
            Real::one(),
            Real::from(2),
            PredicatePolicy::STRICT,
        )
    }

    #[test]
    fn subresultants_reduce_source_conjugates_without_global_projection() {
        let root = retained_root();
        let fiber = coefficient(&[&[0, 0, 1], &[-1]]); // u²-alpha
        let first = [
            coefficient(&[&[0, -1]]),
            coefficient(&[&[0]]),
            coefficient(&[&[1]]),
        ];
        let second = [
            coefficient(&[&[0, 1]]),
            coefficient(&[&[0]]),
            coefficient(&[&[1]]),
        ];
        assert_eq!(
            subresultant_in_algebraic_fiber(&first, &second, 0, &fiber, &root).unwrap(),
            vec![coefficient(&[&[0], &[4]])]
        ); // Res(z²-u,z²+u)=4u²=4alpha
        assert_eq!(
            subresultant_in_algebraic_fiber(&first, &second, 1, &fiber, &root).unwrap(),
            vec![coefficient(&[&[0, 2]]), coefficient(&[&[0]])]
        );
        let linear = [coefficient(&[&[0, -1]]), coefficient(&[&[1]])];
        let square = [
            coefficient(&[&[0], &[-1]]),
            coefficient(&[&[0]]),
            coefficient(&[&[1]]),
        ];
        assert_eq!(
            subresultant_in_algebraic_fiber(&linear, &square, 0, &fiber, &root).unwrap(),
            vec![coefficient(&[&[0]])]
        ); // u²-alpha vanishes on both source sheets
        assert_eq!(
            subresultant_in_algebraic_fiber(&linear, &square, 1, &fiber, &root).unwrap(),
            linear
        );
    }

    #[test]
    fn subresultants_preserve_one_scale_across_all_target_coefficients() {
        let root = retained_root();
        let fiber = coefficient(&[&[-1], &[0, 0, 1]]); // alpha*u²-1
        let first = [
            coefficient(&[&[0, -1]]),
            coefficient(&[&[0]]),
            coefficient(&[&[1]]),
        ];
        let second = [coefficient(&[&[0, 0, 0, -1]]), coefficient(&[&[1]])];
        let result = subresultant_in_algebraic_fiber(&first, &second, 1, &fiber, &root).unwrap();
        let alpha = Real::from(2).sqrt().unwrap();
        let beta = (Real::one() / &alpha).unwrap().sqrt().unwrap();
        let evaluate = |p: &BivariatePolynomial, u: &Real| {
            p.coefficients.iter().rev().fold(Real::zero(), |sum, row| {
                sum * &alpha + Real::eval_poly(row, u)
            })
        };
        for u in [beta.clone(), -beta] {
            let constant = evaluate(&result[0], &u);
            let leading = evaluate(&result[1], &u);
            assert_eq!(
                compare_reals(
                    &(constant + leading * &u * &u * &u),
                    &Real::zero(),
                    PredicatePolicy::STRICT
                )
                .value(),
                Some(Ordering::Equal)
            );
        }
        assert_eq!(
            subresultant_in_algebraic_fiber(&first, &second, 2, &fiber, &root),
            Err(AlgebraicFiberSubresultantError::InvalidOrder)
        );
    }
}
