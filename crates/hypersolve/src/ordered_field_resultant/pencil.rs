//! A formal perturbation retains isolated projection evidence beside overlaps.
use super::bounded::Counted;
use super::{
    BoundedOrderedFieldResultantError as Error, OrderedFieldResultantConfig,
    OrderedFieldResultantError as ArithmeticError, PolynomialArithmetic, check, reserve,
    resultant_bivariate_ordered_field, trim,
};
use crate::OrderedFieldPolynomialContext;
use crate::bareiss::DeterminantArithmetic;
use hyperreal::Real;
use std::cmp::Ordering;

/// First nonzero coefficient of `Res_y(F(x,y), P(x,y)+z)` in ascending `z`.
#[derive(Clone, Debug)]
pub struct OrderedFieldResultantPerturbation<C> {
    /// Lowest power of the formal perturbation with a nonzero coefficient.
    /// It counts common roots over the generic first-variable fiber, with
    /// their multiplicity in `F`; it does not select a real component.
    pub first_nonzero_power: usize,
    /// The nonzero coefficient, in ascending powers of the first variable.
    pub coefficients: Vec<C>,
    /// Calls to the original coefficient field, including lifting and scans.
    pub coefficient_operations: usize,
}

// K[z] embeds in the ordered field K(z), ordered at positive infinity.
// Only ring operations are needed. Reuse the determinant's polynomial
// arithmetic; no new multiplication or elimination kernel lives here.
struct PolynomialField<'a, F>(&'a mut F);
impl<C: Clone, F: OrderedFieldPolynomialContext<C>> OrderedFieldPolynomialContext<Vec<C>>
    for PolynomialField<'_, F>
{
    type Error = ArithmeticError<F::Error>;
    fn constant(&mut self, value: &Real) -> Result<Vec<C>, Self::Error> {
        PolynomialArithmetic(self.0).constant(value)
    }
    fn add(&mut self, a: &Vec<C>, b: &Vec<C>) -> Result<Vec<C>, Self::Error> {
        PolynomialArithmetic(self.0).add(a, b)
    }
    fn multiply(&mut self, a: &Vec<C>, b: &Vec<C>) -> Result<Vec<C>, Self::Error> {
        PolynomialArithmetic(self.0).multiply(a, b)
    }
    fn scale(&mut self, value: &Vec<C>, scale: &Real) -> Result<Vec<C>, Self::Error> {
        PolynomialArithmetic(self.0).scale(value, scale)
    }
    fn sign(&mut self, value: &Vec<C>) -> Result<Ordering, Self::Error> {
        for coefficient in value.iter().rev() {
            check()?;
            let sign = self.0.sign(coefficient).map_err(ArithmeticError::Context)?;
            if sign != Ordering::Equal {
                return Ok(sign);
            }
        }
        Ok(Ordering::Equal)
    }
    fn sign_if_separated(&mut self, value: &Vec<C>) -> Result<Option<Ordering>, Self::Error> {
        for coefficient in value.iter().rev() {
            check()?;
            let sign = self
                .0
                .sign_if_separated(coefficient)
                .map_err(ArithmeticError::Context)?;
            if sign != Some(Ordering::Equal) {
                return Ok(sign);
            }
        }
        Ok(Some(Ordering::Equal))
    }
    fn normalize_positive_scale(&mut self, _: &mut [Vec<C>]) {}
}

fn arithmetic_error<E>(error: ArithmeticError<Error<E>>) -> Error<E> {
    match error {
        ArithmeticError::Context(error) => error,
        ArithmeticError::StorageUnavailable => Error::StorageUnavailable,
        ArithmeticError::BudgetExhausted => Error::BudgetExhausted,
    }
}

fn nested_error<E>(error: Error<ArithmeticError<Error<E>>>) -> Error<E> {
    match error {
        Error::Coefficient(error) => arithmetic_error(error),
        Error::OperationLimit { limit } => Error::OperationLimit { limit },
        Error::MatrixLimit { required, limit } => Error::MatrixLimit { required, limit },
        Error::SizeOverflow => Error::SizeOverflow,
        Error::StorageUnavailable => Error::StorageUnavailable,
        Error::BudgetExhausted => Error::BudgetExhausted,
    }
}

/// Retains finite-contact candidates when an ordinary resultant vanishes.
///
/// Inputs have ascending `[first_power][second_power]` coefficients. The
/// original field and exact coefficient scales are preserved. `None` means
/// the entire formal resultant is zero (in particular, a zero first input),
/// rather than a completed empty intersection.
///
/// Over the generic first-variable fiber, split `F=C*Q`, where `C` contains
/// every factor supported on `P`, with its multiplicity in `F`. For positive
/// second-variable degree `m` of `P`, the first nonzero coefficient is
/// `lc_y(C)^m * Res_y(Q,P)`. Constant cases use the ordinary resultant identity.
/// Every finite isolated contact on `Q` therefore projects to a zero of this
/// coefficient. This is candidate evidence only: consumers must select the
/// original real branch, distinguish continuous contact, and validate original
/// equations, domains and denominator guards, including boundary fibers.
///
/// The existing resultant engine is lifted to polynomials in the formal `z`;
/// no specialization, interpolation, scalar materialization, or division is
/// used. One coefficient-call limit covers all original-field operations,
/// including the final coefficient scan. The original matrix admission limit
/// applies. Neither limit bounds coefficient growth, total work, heap, or one
/// caller-field operation; cancellation has the shared engine's contract.
pub fn resultant_bivariate_perturbation_ordered_field<
    C: Clone,
    F: OrderedFieldPolynomialContext<C>,
>(
    first: &[Vec<C>],
    second: &[Vec<C>],
    config: OrderedFieldResultantConfig,
    field: &mut F,
) -> Result<Option<OrderedFieldResultantPerturbation<C>>, Error<F::Error>> {
    let mut counted = Counted {
        field,
        limit: config.max_coefficient_operations,
        calls: 0,
    };
    let lift = |input: &[Vec<C>]| {
        let mut lifted = reserve(input.len())?;
        for row in input {
            check()?;
            let mut values = reserve(row.len())?;
            values.extend(row.iter().map(|c| vec![c.clone()]));
            lifted.push(values);
        }
        Ok::<_, ArithmeticError<Error<F::Error>>>(lifted)
    };
    let first = lift(first).map_err(arithmetic_error)?;
    let mut second = lift(second).map_err(arithmetic_error)?;
    if second.is_empty() {
        second
            .try_reserve(1)
            .map_err(|_| Error::StorageUnavailable)?;
        second.push(Vec::new());
    }
    if second[0].is_empty() {
        second[0]
            .try_reserve(1)
            .map_err(|_| Error::StorageUnavailable)?;
        second[0].push(vec![counted.constant(&Real::zero())?]);
    }
    second[0][0]
        .try_reserve(1)
        .map_err(|_| Error::StorageUnavailable)?;
    second[0][0].push(counted.constant(&Real::one())?);
    let report = resultant_bivariate_ordered_field(
        &first,
        &second,
        OrderedFieldResultantConfig {
            // Inner operations are counted at the original field, not at
            // the polynomial wrapper, whose calls have variable cost.
            max_coefficient_operations: usize::MAX,
            ..config
        },
        &mut PolynomialField(&mut counted),
    )
    .map_err(nested_error)?;
    let powers = report.coefficients.iter().map(Vec::len).max().unwrap_or(0);
    for power in 0..powers {
        let mut coefficients = reserve(report.coefficients.len()).map_err(arithmetic_error)?;
        for polynomial in &report.coefficients {
            coefficients.push(match polynomial.get(power) {
                Some(value) => value.clone(),
                None => counted.constant(&Real::zero())?,
            });
        }
        trim(&mut coefficients, &mut counted).map_err(arithmetic_error)?;
        if !coefficients.is_empty() {
            check().map_err(arithmetic_error)?;
            return Ok(Some(OrderedFieldResultantPerturbation {
                first_nonzero_power: power,
                coefficients,
                coefficient_operations: counted.calls,
            }));
        }
    }
    check().map_err(arithmetic_error)?;
    Ok(None)
}
