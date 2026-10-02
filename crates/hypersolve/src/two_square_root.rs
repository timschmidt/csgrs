//! Exact two-radical expressions over dense tensor polynomials.
//!
//! A value is `rational + first*sqrt(A) + second*sqrt(B) + product*sqrt(A)*sqrt(B)`
//! with dense tensor coefficients over selected algebraic roots. The radicands
//! `A` and `B` are supplied to the operations that need them.

use hyperreal::Real;

use crate::DenseTensorPolynomial;
use crate::algebraic::AlgebraicRootRepresentation;
use crate::tensor_support::{
    dense_reduce_selected_root_relations, dense_reduce_selected_tuple_relations,
    dense_tensor_is_stored_zero,
};

/// `rational + first*sqrt(A) + second*sqrt(B) + product*sqrt(A)*sqrt(B)`.
#[derive(Clone, Debug)]
pub struct DenseTwoSquareRootExpression {
    /// Rational part.
    pub rational: DenseTensorPolynomial,
    /// Coefficient of `sqrt(A)`.
    pub first: DenseTensorPolynomial,
    /// Coefficient of `sqrt(B)`.
    pub second: DenseTensorPolynomial,
    /// Coefficient of `sqrt(A)*sqrt(B)`.
    pub product: DenseTensorPolynomial,
}

impl DenseTwoSquareRootExpression {
    fn zero(rank: usize) -> Option<DenseTensorPolynomial> {
        DenseTensorPolynomial::zero(vec![1; rank])
    }

    /// Embeds a rational tensor with zero radical parts.
    pub fn from_rational(rational: DenseTensorPolynomial) -> Option<Self> {
        let zero = Self::zero(rational.dimensions().len())?;
        Some(Self {
            rational,
            first: zero.clone(),
            second: zero.clone(),
            product: zero,
        })
    }

    /// Returns `first * sqrt(A)`.
    pub fn from_first_radical(first: DenseTensorPolynomial) -> Option<Self> {
        let zero = Self::zero(first.dimensions().len())?;
        Some(Self {
            rational: zero.clone(),
            first,
            second: zero.clone(),
            product: zero,
        })
    }

    /// Returns `second * sqrt(B)`.
    pub fn from_second_radical(second: DenseTensorPolynomial) -> Option<Self> {
        let zero = Self::zero(second.dimensions().len())?;
        Some(Self {
            rational: zero.clone(),
            first: zero.clone(),
            second,
            product: zero,
        })
    }

    /// Whether every part is stored as the exact rational zero.
    pub fn is_stored_zero(&self) -> bool {
        [&self.rational, &self.first, &self.second, &self.product]
            .into_iter()
            .all(dense_tensor_is_stored_zero)
    }

    /// Whether the expression is stored as the exact constant one.
    pub fn is_stored_one(&self) -> bool {
        let Some((constant, remainder)) = self.rational.coefficients().split_first() else {
            return false;
        };
        constant
            .exact_rational_ref()
            .is_some_and(|value| value.is_one())
            && remainder.iter().all(|value| {
                value
                    .exact_rational_ref()
                    .is_some_and(|value| value.is_zero())
            })
            && [&self.first, &self.second, &self.product]
                .into_iter()
                .all(dense_tensor_is_stored_zero)
    }

    /// Returns the exact sum, or difference when `subtract` is set.
    pub fn combine(&self, other: &Self, subtract: bool) -> Option<Self> {
        let combine = |first: &DenseTensorPolynomial, second: &DenseTensorPolynomial| {
            if subtract {
                first.subtract(second)
            } else {
                first.add(second)
            }
        };
        Some(Self {
            rational: combine(&self.rational, &other.rational)?,
            first: combine(&self.first, &other.first)?,
            second: combine(&self.second, &other.second)?,
            product: combine(&self.product, &other.product)?,
        })
    }

    /// Returns the exact sum.
    pub fn add(&self, other: &Self) -> Option<Self> {
        self.combine(other, false)
    }

    /// Returns the exact difference.
    pub fn subtract(&self, other: &Self) -> Option<Self> {
        self.combine(other, true)
    }

    /// Multiplies every part by `scale`.
    pub fn scale(&self, scale: &Real) -> Option<Self> {
        Some(Self {
            rational: self.rational.scale(scale)?,
            first: self.first.scale(scale)?,
            second: self.second.scale(scale)?,
            product: self.product.scale(scale)?,
        })
    }

    /// Multiplies every part by a rational tensor.
    pub fn multiply_rational(&self, polynomial: &DenseTensorPolynomial) -> Option<Self> {
        Some(Self {
            rational: self.rational.multiply(polynomial)?,
            first: self.first.multiply(polynomial)?,
            second: self.second.multiply(polynomial)?,
            product: self.product.multiply(polynomial)?,
        })
    }

    /// Returns the exact product, given the radicands `A` and `B`.
    pub fn multiply(
        &self,
        other: &Self,
        first_speed_squared: &DenseTensorPolynomial,
        second_speed_squared: &DenseTensorPolynomial,
    ) -> Option<Self> {
        let speed_product = first_speed_squared.multiply(second_speed_squared)?;
        let rational = self
            .rational
            .multiply(&other.rational)?
            .add(
                &self
                    .first
                    .multiply(&other.first)?
                    .multiply(first_speed_squared)?,
            )?
            .add(
                &self
                    .second
                    .multiply(&other.second)?
                    .multiply(second_speed_squared)?,
            )?
            .add(
                &self
                    .product
                    .multiply(&other.product)?
                    .multiply(&speed_product)?,
            )?;
        let first = self
            .rational
            .multiply(&other.first)?
            .add(&self.first.multiply(&other.rational)?)?
            .add(
                &self
                    .second
                    .multiply(&other.product)?
                    .multiply(second_speed_squared)?,
            )?
            .add(
                &self
                    .product
                    .multiply(&other.second)?
                    .multiply(second_speed_squared)?,
            )?;
        let second = self
            .rational
            .multiply(&other.second)?
            .add(&self.second.multiply(&other.rational)?)?
            .add(
                &self
                    .first
                    .multiply(&other.product)?
                    .multiply(first_speed_squared)?,
            )?
            .add(
                &self
                    .product
                    .multiply(&other.first)?
                    .multiply(first_speed_squared)?,
            )?;
        let product = self
            .rational
            .multiply(&other.product)?
            .add(&self.product.multiply(&other.rational)?)?
            .add(&self.first.multiply(&other.second)?)?
            .add(&self.second.multiply(&other.first)?)?;
        Some(Self {
            rational,
            first,
            second,
            product,
        })
    }

    /// Returns the exact square, given the radicands `A` and `B`.
    pub fn square(
        &self,
        first_speed_squared: &DenseTensorPolynomial,
        second_speed_squared: &DenseTensorPolynomial,
    ) -> Option<Self> {
        let speed_product = first_speed_squared.multiply(second_speed_squared)?;
        let rational = self
            .rational
            .multiply(&self.rational)?
            .add(
                &self
                    .first
                    .multiply(&self.first)?
                    .multiply(first_speed_squared)?,
            )?
            .add(
                &self
                    .second
                    .multiply(&self.second)?
                    .multiply(second_speed_squared)?,
            )?
            .add(
                &self
                    .product
                    .multiply(&self.product)?
                    .multiply(&speed_product)?,
            )?;
        let two = Real::from(2_i8);
        let first = self
            .rational
            .multiply(&self.first)?
            .add(
                &self
                    .second
                    .multiply(&self.product)?
                    .multiply(second_speed_squared)?,
            )?
            .scale(&two)?;
        let second = self
            .rational
            .multiply(&self.second)?
            .add(
                &self
                    .first
                    .multiply(&self.product)?
                    .multiply(first_speed_squared)?,
            )?
            .scale(&two)?;
        let product = self
            .rational
            .multiply(&self.product)?
            .add(&self.first.multiply(&self.second)?)?
            .scale(&two)?;
        Some(Self {
            rational,
            first,
            second,
            product,
        })
    }

    /// Reduces every part modulo the selected-root defining relations.
    pub fn reduced(&self, sources: &[AlgebraicRootRepresentation]) -> Option<Self> {
        Some(Self {
            rational: dense_reduce_selected_root_relations(self.rational.clone(), sources)?,
            first: dense_reduce_selected_root_relations(self.first.clone(), sources)?,
            second: dense_reduce_selected_root_relations(self.second.clone(), sources)?,
            product: dense_reduce_selected_root_relations(self.product.clone(), sources)?,
        })
    }

    /// Canonicalizes a coefficient-field value at the retained source tuple.
    /// Unlike [`Self::reduced`], these tensors have no free output axis.
    pub fn reduced_at_source_tuple(&self, sources: &[AlgebraicRootRepresentation]) -> Option<Self> {
        Some(Self {
            rational: dense_reduce_selected_tuple_relations(self.rational.clone(), sources)?,
            first: dense_reduce_selected_tuple_relations(self.first.clone(), sources)?,
            second: dense_reduce_selected_tuple_relations(self.second.clone(), sources)?,
            product: dense_reduce_selected_tuple_relations(self.product.clone(), sources)?,
        })
    }

    /// Enumerates conjugate-sheet zeros without multiplicities introduced only
    /// by absent generators. Callers must still replay the authored sheet.
    pub fn projection(
        &self,
        first_speed_squared: &DenseTensorPolynomial,
        second_speed_squared: &DenseTensorPolynomial,
        sources: &[AlgebraicRootRepresentation],
    ) -> Option<DenseTensorPolynomial> {
        let expression = self.reduced(sources)?;
        let first_speed_squared =
            dense_reduce_selected_root_relations(first_speed_squared.clone(), sources)?;
        let second_speed_squared =
            dense_reduce_selected_root_relations(second_speed_squared.clone(), sources)?;
        let reduce = |polynomial| dense_reduce_selected_root_relations(polynomial, sources);
        let (retained_rational, retained_radical) =
            if dense_tensor_is_stored_zero(&expression.second)
                && dense_tensor_is_stored_zero(&expression.product)
            {
                // An absent second generator would only square the retained
                // first-generator equation, doubling its eventual norm degree.
                (expression.rational, expression.first)
            } else {
                let rational_squared = reduce(expression.rational.multiply(&expression.rational)?)?;
                let first_squared = reduce(expression.first.multiply(&expression.first)?)?;
                let second_squared = reduce(expression.second.multiply(&expression.second)?)?;
                let product_squared = reduce(expression.product.multiply(&expression.product)?)?;
                let retained_rational = reduce(
                    rational_squared
                        .add(&reduce(first_squared.multiply(&first_speed_squared)?)?)?
                        .subtract(&reduce(
                            second_speed_squared.multiply(&second_squared.add(&reduce(
                                product_squared.multiply(&first_speed_squared)?,
                            )?)?)?,
                        )?)?,
                )?;
                let retained_radical = reduce(
                    expression
                        .rational
                        .multiply(&expression.first)?
                        .subtract(&reduce(
                            second_speed_squared
                                .multiply(&expression.second.multiply(&expression.product)?)?,
                        )?)?
                        .scale(&Real::from(2_i8))?,
                )?;
                (retained_rational, retained_radical)
            };
        if dense_tensor_is_stored_zero(&retained_radical) {
            // The final norm would be a square. Keep its base, including any
            // radicand factors: their zeros have not been proved absent.
            return Some(retained_rational);
        }
        reduce(
            retained_rational
                .multiply(&retained_rational)?
                .subtract(&reduce(
                    retained_radical
                        .multiply(&retained_radical)?
                        .multiply(&first_speed_squared)?,
                )?)?,
        )
    }
}
