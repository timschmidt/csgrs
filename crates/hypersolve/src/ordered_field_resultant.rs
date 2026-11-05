//! Bivariate elimination over a caller-owned exact ordered coefficient field.
//!
//! Coefficients stay in that field. A Sylvester determinant is evaluated by
//! Faddeev–LeVerrier polynomial arithmetic: only division by nonzero integers
//! occurs, so no pivot or primitive-element projection is required. Original
//! coefficient signs determine degrees; intermediate values are never rescaled
//! projectively. The result is the actual resultant, not only its zero set.
//!
//! Limits count coefficient-context calls and matrix entries separately. They
//! do not bound coefficient arithmetic, expression growth, heap bytes or
//! cancellation latency inside the caller's field. Platform storage-layout
//! and allocator limits are additional typed resource stops. Exhaustion publishes no
//! partial resultant, and a later larger-budget call has no failed cache state.

use crate::OrderedFieldPolynomialContext;
use hyperreal::{Problem, Real};
use std::cmp::Ordering;

/// Resource units of one bivariate resultant attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct OrderedFieldResultantConfig {
    /// Maximum total calls to coefficient arithmetic and sign authority.
    pub max_coefficient_operations: usize,
    /// Maximum entries in each square Sylvester/characteristic matrix.
    pub max_matrix_entries: usize,
}

impl Default for OrderedFieldResultantConfig {
    fn default() -> Self {
        Self {
            max_coefficient_operations: 1_000_000,
            max_matrix_entries: 262_144,
        }
    }
}

/// An attempt that did not publish a completed resultant.
#[derive(Debug)]
pub enum OrderedFieldResultantError<E> {
    /// The caller's exact coefficient authority failed or remained undecided.
    Coefficient(E),
    /// The declared coefficient-operation allowance was reached.
    OperationLimit {
        /// Configured operation allowance.
        limit: usize,
    },
    /// A required matrix exceeds its declared entry allowance.
    MatrixLimit {
        /// Required number of entries.
        required: usize,
        /// Configured entry allowance.
        limit: usize,
    },
    /// A storage dimension exceeds addressable machine sizes.
    SizeOverflow,
    /// Fallible storage reservation failed.
    AllocationFailed,
    /// Construction of an exact rational scale failed.
    Numeric(Problem),
}

/// A completed polynomial and its measured coefficient-context call count.
#[derive(Clone, Debug)]
pub struct OrderedFieldResultant<C> {
    /// Ascending powers of the retained first variable.
    pub coefficients: Vec<C>,
    /// Arithmetic and sign authority calls actually consumed.
    pub coefficient_operations: usize,
}

type Result<T, E> = std::result::Result<T, OrderedFieldResultantError<E>>;

fn filled<T: Clone, E>(count: usize, value: T) -> Result<Vec<T>, E> {
    std::alloc::Layout::array::<T>(count).map_err(|_| OrderedFieldResultantError::SizeOverflow)?;
    let mut values = Vec::new();
    values
        .try_reserve_exact(count)
        .map_err(|_| OrderedFieldResultantError::AllocationFailed)?;
    values.resize(count, value);
    Ok(values)
}

struct Arithmetic<'a, F> {
    field: &'a mut F,
    config: OrderedFieldResultantConfig,
    calls: usize,
}

impl<F> Arithmetic<'_, F> {
    fn tick<E>(&mut self) -> Result<(), E> {
        if self.calls == self.config.max_coefficient_operations {
            return Err(OrderedFieldResultantError::OperationLimit { limit: self.calls });
        }
        self.calls += 1;
        Ok(())
    }
    fn constant<C>(&mut self, value: &Real) -> Result<C, F::Error>
    where
        F: OrderedFieldPolynomialContext<C>,
    {
        self.tick()?;
        self.field
            .constant(value)
            .map_err(OrderedFieldResultantError::Coefficient)
    }
    fn add<C>(&mut self, a: &C, b: &C) -> Result<C, F::Error>
    where
        F: OrderedFieldPolynomialContext<C>,
    {
        self.tick()?;
        self.field
            .add(a, b)
            .map_err(OrderedFieldResultantError::Coefficient)
    }
    fn multiply<C>(&mut self, a: &C, b: &C) -> Result<C, F::Error>
    where
        F: OrderedFieldPolynomialContext<C>,
    {
        self.tick()?;
        self.field
            .multiply(a, b)
            .map_err(OrderedFieldResultantError::Coefficient)
    }
    fn scale<C>(&mut self, value: &C, scale: &Real) -> Result<C, F::Error>
    where
        F: OrderedFieldPolynomialContext<C>,
    {
        self.tick()?;
        self.field
            .scale(value, scale)
            .map_err(OrderedFieldResultantError::Coefficient)
    }
    fn sign<C>(&mut self, value: &C) -> Result<Ordering, F::Error>
    where
        F: OrderedFieldPolynomialContext<C>,
    {
        self.tick()?;
        self.field
            .sign(value)
            .map_err(OrderedFieldResultantError::Coefficient)
    }
    fn trim<C: Clone>(&mut self, p: &mut Vec<C>) -> Result<(), F::Error>
    where
        F: OrderedFieldPolynomialContext<C>,
    {
        while let Some(leading) = p.last() {
            if self.sign(leading)? != Ordering::Equal {
                break;
            }
            p.pop();
        }
        Ok(())
    }
    fn polynomial_add<C: Clone>(&mut self, a: &[C], b: &[C]) -> Result<Vec<C>, F::Error>
    where
        F: OrderedFieldPolynomialContext<C>,
    {
        let mut result = Vec::new();
        result
            .try_reserve_exact(a.len().max(b.len()))
            .map_err(|_| OrderedFieldResultantError::AllocationFailed)?;
        for i in 0..a.len().max(b.len()) {
            result.push(match (a.get(i), b.get(i)) {
                (Some(a), Some(b)) => self.add(a, b)?,
                (Some(a), None) | (None, Some(a)) => a.clone(),
                (None, None) => unreachable!("index is below the larger polynomial length"),
            });
        }
        self.trim(&mut result)?;
        Ok(result)
    }
    fn polynomial_multiply<C: Clone>(&mut self, a: &[C], b: &[C]) -> Result<Vec<C>, F::Error>
    where
        F: OrderedFieldPolynomialContext<C>,
    {
        if a.is_empty() || b.is_empty() {
            return Ok(Vec::new());
        }
        let count = a
            .len()
            .checked_add(b.len() - 1)
            .ok_or(OrderedFieldResultantError::SizeOverflow)?;
        let mut result = filled(count, None)?;
        for (i, a) in a.iter().enumerate() {
            for (j, b) in b.iter().enumerate() {
                let term = self.multiply(a, b)?;
                result[i + j] = Some(match &result[i + j] {
                    Some(sum) => self.add(sum, &term)?,
                    None => term,
                });
            }
        }
        let mut result = result
            .into_iter()
            .map(|value| value.expect("every convolution degree has a contributing pair"))
            .collect();
        self.trim(&mut result)?;
        Ok(result)
    }
    fn polynomial_scale<C: Clone>(&mut self, p: &[C], scale: &Real) -> Result<Vec<C>, F::Error>
    where
        F: OrderedFieldPolynomialContext<C>,
    {
        p.iter().map(|value| self.scale(value, scale)).collect()
    }
}

/// Eliminates the second variable from two bivariate polynomials over one
/// exact coefficient field. Input rows use ascending powers `[x][y]`.
/// Empty/zero operands return the zero polynomial by convention. Two nonzero
/// polynomials constant in `y` have resultant one (the empty determinant).
/// No approximate coefficient test or projective normalization is consumed.
pub fn resultant_bivariate_ordered_field<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    first: &[Vec<C>],
    second: &[Vec<C>],
    config: OrderedFieldResultantConfig,
    field: &mut F,
) -> Result<OrderedFieldResultant<C>, F::Error> {
    let mut arithmetic = Arithmetic {
        field,
        config,
        calls: 0,
    };
    let mut degrees = [None, None];
    for (index, polynomial) in [first, second].into_iter().enumerate() {
        for row in polynomial {
            for (power, coefficient) in row.iter().enumerate() {
                if arithmetic.sign(coefficient)? != Ordering::Equal {
                    degrees[index] =
                        Some(degrees[index].map_or(power, |known: usize| known.max(power)));
                }
            }
        }
    }
    let [Some(m), Some(n)] = degrees else {
        return Ok(OrderedFieldResultant {
            coefficients: vec![arithmetic.constant(&Real::zero())?],
            coefficient_operations: arithmetic.calls,
        });
    };
    let dimension = m
        .checked_add(n)
        .ok_or(OrderedFieldResultantError::SizeOverflow)?;
    let entries = dimension
        .checked_mul(dimension)
        .ok_or(OrderedFieldResultantError::SizeOverflow)?;
    if entries > config.max_matrix_entries {
        return Err(OrderedFieldResultantError::MatrixLimit {
            required: entries,
            limit: config.max_matrix_entries,
        });
    }
    let one = arithmetic.constant(&Real::one())?;
    if dimension == 0 {
        return Ok(OrderedFieldResultant {
            coefficients: vec![one],
            coefficient_operations: arithmetic.calls,
        });
    }
    let mut matrix = filled(entries, Vec::<C>::new())?;
    let zero = arithmetic.constant(&Real::zero())?;
    for (offset, count, polynomial, degree) in [(0, n, first, m), (n, m, second, n)] {
        for row in 0..count {
            for power in 0..=degree {
                let mut coefficient = Vec::new();
                coefficient
                    .try_reserve_exact(polynomial.len())
                    .map_err(|_| OrderedFieldResultantError::AllocationFailed)?;
                for terms in polynomial {
                    coefficient.push(terms.get(power).unwrap_or(&zero).clone());
                }
                arithmetic.trim(&mut coefficient)?;
                matrix[(offset + row) * dimension + row + degree - power] = coefficient;
            }
        }
    }
    let mut characteristic = filled(entries, Vec::<C>::new())?;
    for i in 0..dimension {
        characteristic[i * dimension + i] = vec![one.clone()];
    }
    for step in 1..=dimension {
        let mut product = filled(entries, Vec::<C>::new())?;
        for row in 0..dimension {
            for column in 0..dimension {
                for inner in 0..dimension {
                    let term = arithmetic.polynomial_multiply(
                        &matrix[row * dimension + inner],
                        &characteristic[inner * dimension + column],
                    )?;
                    product[row * dimension + column] =
                        arithmetic.polynomial_add(&product[row * dimension + column], &term)?;
                }
            }
        }
        let mut trace = Vec::new();
        for i in 0..dimension {
            trace = arithmetic.polynomial_add(&trace, &product[i * dimension + i])?;
        }
        let denominator =
            u64::try_from(step).map_err(|_| OrderedFieldResultantError::SizeOverflow)?;
        let scale = (Real::from(-1) / Real::from(denominator))
            .map_err(OrderedFieldResultantError::Numeric)?;
        let coefficient = arithmetic.polynomial_scale(&trace, &scale)?;
        if step == dimension {
            let mut coefficients = if dimension.is_multiple_of(2) {
                coefficient
            } else {
                arithmetic.polynomial_scale(&coefficient, &Real::from(-1))?
            };
            arithmetic.trim(&mut coefficients)?;
            if coefficients.is_empty() {
                coefficients.push(zero);
            }
            return Ok(OrderedFieldResultant {
                coefficients,
                coefficient_operations: arithmetic.calls,
            });
        }
        for i in 0..dimension {
            product[i * dimension + i] =
                arithmetic.polynomial_add(&product[i * dimension + i], &coefficient)?;
        }
        characteristic = product;
    }
    unreachable!("a nonempty determinant returns its final characteristic coefficient")
}
