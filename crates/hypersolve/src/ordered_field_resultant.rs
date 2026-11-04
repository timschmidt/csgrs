//! Bivariate elimination over a retained exact ordered coefficient field.
//!
//! Coefficients never pass through independent scalar coordinates. The shared
//! pivot-free determinant uses only ring operations and rational scalings.
//! A zero resultant is candidate component evidence, not a selected real arc.
use std::cmp::Ordering;

use hyperreal::Real;

use crate::bareiss::{DeterminantArithmetic, pivot_free_determinant_with};
use crate::{CurveResultantParameter, OrderedFieldPolynomialContext};

/// Failure before a complete exact elimination polynomial is available.
#[derive(Debug)]
pub enum OrderedFieldResultantError<E> {
    /// The caller's arithmetic, embedding or exact-sign authority declined.
    Context(E),
    /// Checked dimensions or a fallible temporary allocation were unavailable.
    /// This is not a configured degree limit or a geometric zero.
    StorageUnavailable,
    /// The shared work scope declined an admission or observed cancellation.
    /// Its receipt distinguishes the reached limit from cancellation.
    BudgetExhausted,
}

type Result<T, E> = std::result::Result<T, OrderedFieldResultantError<E>>;

fn check<E>() -> Result<(), E> {
    if crate::work_budget::exhaustion().is_some() {
        Err(OrderedFieldResultantError::BudgetExhausted)
    } else {
        Ok(())
    }
}

fn reserve<T, E>(size: usize) -> Result<Vec<T>, E> {
    let mut values = Vec::new();
    values
        .try_reserve_exact(size)
        .map_err(|_| OrderedFieldResultantError::StorageUnavailable)?;
    Ok(values)
}

fn trim<C, F: OrderedFieldPolynomialContext<C>>(
    values: &mut Vec<C>,
    field: &mut F,
) -> Result<(), F::Error> {
    while let Some(value) = values.last() {
        check()?;
        if field
            .sign(value)
            .map_err(OrderedFieldResultantError::Context)?
            != Ordering::Equal
        {
            break;
        }
        values.pop();
    }
    Ok(())
}

/// Ascending eliminated powers, whose entries are retained-axis polynomials.
fn oriented<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    input: &[Vec<C>],
    retained: CurveResultantParameter,
    field: &mut F,
) -> Result<Vec<Vec<C>>, F::Error> {
    let outer = match retained {
        CurveResultantParameter::First => input.iter().map(Vec::len).max().unwrap_or(0),
        CurveResultantParameter::Second => input.len(),
    };
    let mut result = reserve(outer)?;
    result.resize_with(outer, Vec::new);
    for (first, row) in input.iter().enumerate() {
        for (second, value) in row.iter().enumerate() {
            check()?;
            let (power, coefficient) = match retained {
                CurveResultantParameter::First => (second, first),
                CurveResultantParameter::Second => (first, second),
            };
            let polynomial = &mut result[power];
            while polynomial.len() < coefficient {
                check()?;
                polynomial
                    .try_reserve(1)
                    .map_err(|_| OrderedFieldResultantError::StorageUnavailable)?;
                polynomial.push(
                    field
                        .constant(&Real::zero())
                        .map_err(OrderedFieldResultantError::Context)?,
                );
            }
            // Validate every supplied coefficient, including authored zeros.
            polynomial
                .try_reserve(1)
                .map_err(|_| OrderedFieldResultantError::StorageUnavailable)?;
            polynomial.push(
                field
                    .scale(value, &Real::one())
                    .map_err(OrderedFieldResultantError::Context)?,
            );
        }
    }
    for polynomial in &mut result {
        trim(polynomial, field)?;
    }
    while result.last().is_some_and(Vec::is_empty) {
        result.pop();
    }
    Ok(result)
}

struct PolynomialArithmetic<'a, F>(&'a mut F);
impl<C: Clone, F: OrderedFieldPolynomialContext<C>> DeterminantArithmetic<Vec<C>>
    for PolynomialArithmetic<'_, F>
{
    type Error = OrderedFieldResultantError<F::Error>;
    fn constant(&mut self, value: &Real) -> Result<Vec<C>, F::Error> {
        check()?;
        if value.definitely_zero() {
            Ok(Vec::new())
        } else {
            Ok(vec![
                self.0
                    .constant(value)
                    .map_err(OrderedFieldResultantError::Context)?,
            ])
        }
    }
    fn add(&mut self, a: &Vec<C>, b: &Vec<C>) -> Result<Vec<C>, F::Error> {
        check()?;
        let mut result = reserve(a.len().max(b.len()))?;
        for i in 0..a.len().max(b.len()) {
            check()?;
            result.push(match (a.get(i), b.get(i)) {
                (Some(a), Some(b)) => self
                    .0
                    .add(a, b)
                    .map_err(OrderedFieldResultantError::Context)?,
                (Some(value), None) | (None, Some(value)) => value.clone(),
                (None, None) => unreachable!("one polynomial has this coefficient"),
            });
        }
        Ok(result)
    }
    fn multiply(&mut self, a: &Vec<C>, b: &Vec<C>) -> Result<Vec<C>, F::Error> {
        check()?;
        if a.is_empty() || b.is_empty() {
            return Ok(Vec::new());
        }
        let size = a
            .len()
            .checked_add(b.len() - 1)
            .ok_or(OrderedFieldResultantError::StorageUnavailable)?;
        let mut result = reserve(size)?;
        result.resize_with(size, || None);
        for (i, a) in a.iter().enumerate() {
            check()?;
            for (j, b) in b.iter().enumerate() {
                check()?;
                let value = self
                    .0
                    .multiply(a, b)
                    .map_err(OrderedFieldResultantError::Context)?;
                result[i + j] = Some(match result[i + j].take() {
                    Some(old) => self
                        .0
                        .add(&old, &value)
                        .map_err(OrderedFieldResultantError::Context)?,
                    None => value,
                });
            }
        }
        result
            .into_iter()
            .map(|value| {
                value.map_or_else(
                    || {
                        self.0
                            .constant(&Real::zero())
                            .map_err(OrderedFieldResultantError::Context)
                    },
                    Ok,
                )
            })
            .collect()
    }
    fn scale(&mut self, value: &Vec<C>, scale: &Real) -> Result<Vec<C>, F::Error> {
        check()?;
        value
            .iter()
            .map(|value| {
                check()?;
                self.0
                    .scale(value, scale)
                    .map_err(OrderedFieldResultantError::Context)
            })
            .collect()
    }
    fn negate(&mut self, value: Vec<C>) -> Result<Vec<C>, F::Error> {
        self.scale(&value, &Real::from(-1))
    }
}

impl<C: Clone, F: OrderedFieldPolynomialContext<C>> crate::resultant::BezoutArithmetic<Vec<C>>
    for PolynomialArithmetic<'_, F>
{
    fn subtract(&mut self, a: &Vec<C>, b: &Vec<C>) -> Result<Vec<C>, F::Error> {
        let negative = self.scale(b, &Real::from(-1))?;
        self.add(a, &negative)
    }
    fn is_zero(&mut self, values: &Vec<C>) -> Result<bool, F::Error> {
        for value in values {
            check()?;
            if self
                .0
                .sign_if_separated(value)
                .map_err(OrderedFieldResultantError::Context)?
                != Some(Ordering::Equal)
            {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

impl<F> PolynomialArithmetic<'_, F> {
    fn homogeneous_value<C: Clone>(
        &mut self,
        coefficients: &[Vec<C>],
        numerator: Vec<C>,
        denominator: &[C],
    ) -> Result<Vec<C>, F::Error>
    where
        F: OrderedFieldPolynomialContext<C>,
    {
        let mut sum = coefficients.last().expect("nonzero polynomial").clone();
        let denominator = denominator.to_vec();
        let mut weight = self.constant(&Real::one())?;
        for coefficient in coefficients[..coefficients.len() - 1].iter().rev() {
            weight = self.multiply(&weight, &denominator)?;
            let retained = self.multiply(&sum, &numerator)?;
            let added = self.multiply(coefficient, &weight)?;
            sum = self.add(&retained, &added)?;
        }
        Ok(sum)
    }
    fn power<C: Clone>(
        &mut self,
        mut value: Vec<C>,
        mut exponent: usize,
    ) -> Result<Vec<C>, F::Error>
    where
        F: OrderedFieldPolynomialContext<C>,
    {
        let mut product = self.constant(&Real::one())?;
        while exponent != 0 {
            if exponent & 1 != 0 {
                product = self.multiply(&product, &value)?;
            }
            exponent >>= 1;
            if exponent != 0 {
                value = self.multiply(&value, &value)?;
            }
        }
        Ok(product)
    }
}

/// Exact resultant of two bivariate polynomials over the caller's field.
///
/// Inputs are ascending `[first_power][second_power]` coefficient grids; missing
/// entries are zero. The output is ascending in `retained`, with its exact
/// magnitude, including original coefficient scales. A zero input returns
/// `[0]`; two nonzero eliminated-degree-zero inputs return `[1]`. A zero
/// resultant does not prove a real common component or exclude isolated events.
/// Consumers must retain and validate the original equations and selections.
///
/// Constant and linear identities and shared Bezout/Sylvester determinants
/// have no finite degree or coefficient-field-degree prerequisite. The shared
/// pivot-free determinant makes no pivot decisions. Every
/// coefficient is checked through the supplied exact context. No primitive
/// element, independent coordinate projection or approximation is introduced.
///
/// One shared promotion unit admits the call. Cancellation is checked between
/// polynomial operations and multiplication rows, including coefficient scans.
/// This does not bound one field operation, coefficient growth, total work,
/// heap, or elapsed time. Checked storage failure and actual budget exhaustion
/// remain distinct; neither publishes a partial elimination polynomial.
pub fn ordered_field_resultant_bivariate<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    first: &[Vec<C>],
    second: &[Vec<C>],
    retained: CurveResultantParameter,
    field: &mut F,
) -> Result<Vec<C>, F::Error> {
    crate::work_budget::consume(crate::work_budget::WorkKind::AlgebraicPromotion, 1)
        .map_err(|_| OrderedFieldResultantError::BudgetExhausted)?;
    let first = oriented(first, retained, field)?;
    let second = oriented(second, retained, field)?;
    if first.is_empty() || second.is_empty() {
        let zero = field
            .constant(&Real::zero())
            .map_err(OrderedFieldResultantError::Context)?;
        check()?;
        return Ok(vec![zero]);
    }
    let (m, n) = (first.len() - 1, second.len() - 1);
    let mut result = if m == 0 || n == 0 {
        // Res(a(s), G(s,t)) = a(s)^deg_t(G), with the symmetric
        // identity for a constant second operand. No matrix is needed.
        let (value, exponent) = if m == 0 {
            (first[0].clone(), n)
        } else {
            (second[0].clone(), m)
        };
        PolynomialArithmetic(field).power(value, exponent)?
    } else if n == 1 || m == 1 {
        let (polynomial, linear, negate) = if n == 1 {
            (&first, &second, !m.is_multiple_of(2))
        } else {
            (&second, &first, false)
        };
        let mut arithmetic = PolynomialArithmetic(field);
        let numerator = arithmetic.negate(linear[0].clone())?;
        let result = arithmetic.homogeneous_value(polynomial, numerator, &linear[1])?;
        if negate {
            arithmetic.negate(result)?
        } else {
            result
        }
    } else if m == n {
        let mut arithmetic = PolynomialArithmetic(field);
        let flat =
            crate::resultant::bezout_matrix_with(&first, &second, m, Vec::new(), &mut arithmetic)?
                .ok_or(OrderedFieldResultantError::StorageUnavailable)?;
        let mut flat = flat.into_iter();
        let mut matrix = reserve(m)?;
        for _ in 0..m {
            let mut row = reserve(m)?;
            row.extend(flat.by_ref().take(m));
            matrix.push(row);
        }
        let result = pivot_free_determinant_with(&matrix, &mut arithmetic)?;
        if matches!(m % 4, 2 | 3) {
            arithmetic.negate(result)?
        } else {
            result
        }
    } else {
        let dimension = m
            .checked_add(n)
            .ok_or(OrderedFieldResultantError::StorageUnavailable)?;
        dimension
            .checked_mul(dimension)
            .ok_or(OrderedFieldResultantError::StorageUnavailable)?;
        let mut matrix = reserve(dimension)?;
        for row in 0..dimension {
            check()?;
            let mut entries = reserve(dimension)?;
            entries.resize_with(dimension, Vec::new);
            let (equation, shift) = if row < n {
                (&first, row)
            } else {
                (&second, row - n)
            };
            for (degree, coefficient) in equation.iter().rev().enumerate() {
                entries[shift + degree] = coefficient.clone();
            }
            matrix.push(entries);
        }
        pivot_free_determinant_with(&matrix, &mut PolynomialArithmetic(field))?
    };
    trim(&mut result, field)?;
    if result.is_empty() {
        result.push(
            field
                .constant(&Real::zero())
                .map_err(OrderedFieldResultantError::Context)?,
        );
    }
    check()?;
    Ok(result)
}
