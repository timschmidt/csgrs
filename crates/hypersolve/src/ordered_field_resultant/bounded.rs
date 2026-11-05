//! Resource admission around the shared exact resultant implementation.

use super::{OrderedFieldResultantError, ordered_field_resultant_bivariate};
use crate::{CurveResultantParameter, OrderedFieldPolynomialContext};
use hyperreal::Real;
use std::cmp::Ordering;

/// Resource units of one resultant attempt; neither unit bounds heap bytes or
/// the cost of a single coefficient operation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct OrderedFieldResultantConfig {
    /// Total calls to the coefficient arithmetic and sign authority.
    pub max_coefficient_operations: usize,
    /// Admission bound on the square Sylvester problem size `(m + n)^2`.
    /// This bound applies even when a constant, linear or Bezout shortcut uses
    /// less storage. It is not a measurement of allocated matrix entries.
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

/// A stopped bounded attempt never publishes a partial polynomial.
#[derive(Debug)]
pub enum BoundedOrderedFieldResultantError<E> {
    /// The caller's exact coefficient authority declined.
    Coefficient(E),
    /// The configured coefficient-context call allowance was reached.
    OperationLimit {
        /// Configured allowance.
        limit: usize,
    },
    /// The Sylvester problem exceeds its declared admission allowance.
    MatrixLimit {
        /// Required problem-size units.
        required: usize,
        /// Configured allowance.
        limit: usize,
    },
    /// Problem-size arithmetic exceeds addressable machine dimensions.
    SizeOverflow,
    /// Checked storage dimensions or a fallible reservation were unavailable.
    /// No configured operation allowance is asserted to have been reached.
    StorageUnavailable,
    /// The shared work scope declined admission or observed cancellation.
    BudgetExhausted,
}

/// A completed exact polynomial and the coefficient calls actually consumed.
#[derive(Clone, Debug)]
pub struct OrderedFieldResultant<C> {
    /// Ascending powers of the retained first variable.
    pub coefficients: Vec<C>,
    /// Arithmetic and sign authority calls, including admission scans.
    pub coefficient_operations: usize,
}

type Error<E> = BoundedOrderedFieldResultantError<E>;

struct Counted<'a, F> {
    field: &'a mut F,
    limit: usize,
    calls: usize,
}

impl<F> Counted<'_, F> {
    fn tick<E>(&mut self) -> Result<(), Error<E>> {
        if crate::work_budget::exhaustion().is_some() {
            return Err(Error::BudgetExhausted);
        }
        if self.calls == self.limit {
            return Err(Error::OperationLimit { limit: self.limit });
        }
        self.calls += 1;
        Ok(())
    }
}

impl<C, F: OrderedFieldPolynomialContext<C>> OrderedFieldPolynomialContext<C> for Counted<'_, F> {
    type Error = Error<F::Error>;

    fn constant(&mut self, value: &Real) -> Result<C, Self::Error> {
        self.tick()?;
        self.field.constant(value).map_err(Error::Coefficient)
    }
    fn add(&mut self, a: &C, b: &C) -> Result<C, Self::Error> {
        self.tick()?;
        self.field.add(a, b).map_err(Error::Coefficient)
    }
    fn multiply(&mut self, a: &C, b: &C) -> Result<C, Self::Error> {
        self.tick()?;
        self.field.multiply(a, b).map_err(Error::Coefficient)
    }
    fn scale(&mut self, value: &C, scale: &Real) -> Result<C, Self::Error> {
        self.tick()?;
        self.field.scale(value, scale).map_err(Error::Coefficient)
    }
    fn sign(&mut self, value: &C) -> Result<Ordering, Self::Error> {
        self.tick()?;
        self.field.sign(value).map_err(Error::Coefficient)
    }
    fn sign_if_separated(&mut self, value: &C) -> Result<Option<Ordering>, Self::Error> {
        self.tick()?;
        self.field
            .sign_if_separated(value)
            .map_err(Error::Coefficient)
    }
    fn normalize_positive_scale(&mut self, values: &mut [C]) {
        // Resultants preserve actual values. The shared engine never calls
        // projective normalization; the retained context's contract is intact.
        self.field.normalize_positive_scale(values);
    }
    fn remainder_elimination_budget(&self) -> Option<usize> {
        self.field.remainder_elimination_budget()
    }
}

/// Eliminates the second variable over the original coefficient field, with
/// explicit admission and coefficient-call limits. Coefficients use ascending
/// `[first_power][second_power]` grids. The shared engine owns all resultant
/// formulas, shortcuts and determinants; no independent elimination path,
/// coordinate projection or normalization is introduced here.
///
/// The call counter does not bound a field operation, coefficient growth,
/// retained or temporary heap bytes, or cancellation latency inside the field.
/// Failed calls retain no failed cache state and can be retried with new limits.
pub fn resultant_bivariate_ordered_field<C: Clone, F: OrderedFieldPolynomialContext<C>>(
    first: &[Vec<C>],
    second: &[Vec<C>],
    config: OrderedFieldResultantConfig,
    field: &mut F,
) -> Result<OrderedFieldResultant<C>, Error<F::Error>> {
    let mut counted = Counted {
        field,
        limit: config.max_coefficient_operations,
        calls: 0,
    };
    let mut degrees = [None, None];
    for (index, polynomial) in [first, second].into_iter().enumerate() {
        for row in polynomial {
            for (power, coefficient) in row.iter().enumerate() {
                if counted.sign(coefficient)? != Ordering::Equal {
                    degrees[index] = Some(degrees[index].map_or(power, |d: usize| d.max(power)));
                }
            }
        }
    }
    if let [Some(m), Some(n)] = degrees {
        let dimension = m.checked_add(n).ok_or(Error::SizeOverflow)?;
        let required = dimension
            .checked_mul(dimension)
            .ok_or(Error::SizeOverflow)?;
        if required > config.max_matrix_entries {
            return Err(Error::MatrixLimit {
                required,
                limit: config.max_matrix_entries,
            });
        }
    }
    let coefficients = ordered_field_resultant_bivariate(
        first,
        second,
        CurveResultantParameter::First,
        &mut counted,
    )
    .map_err(|error| match error {
        OrderedFieldResultantError::Context(error) => error,
        OrderedFieldResultantError::StorageUnavailable => Error::StorageUnavailable,
        OrderedFieldResultantError::BudgetExhausted => Error::BudgetExhausted,
    })?;
    Ok(OrderedFieldResultant {
        coefficients,
        coefficient_operations: counted.calls,
    })
}
