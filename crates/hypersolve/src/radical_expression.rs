//! Exact expressions with one or two square-root generators over a
//! polynomial coefficient ring.
//!
//! [`SquareRootExpression`] is `rational + radical*sqrt(D)` and
//! [`TwoSquareRootExpression`] is
//! `rational + first*sqrt(A) + second*sqrt(B) + product*sqrt(A)*sqrt(B)`.
//! The radicands are supplied to the operations that need them, so the same
//! coefficients can be replayed against either sign branch.

use hyperreal::Real;

use crate::DenseTensorPolynomial;
use crate::algebraic::AlgebraicRootRepresentation;
use crate::curve_resultant::TrivariatePolynomial;
use crate::tensor_support::{
    dense_reduce_selected_root_relations, dense_reduce_selected_tuple_relations,
    dense_tensor_is_stored_zero,
};

/// Exact polynomial arithmetic shared by radical expression coefficients.
///
/// Every operation returns `None` on a shape mismatch or allocation failure.
pub trait ExactPolynomialRing: Clone + Sized {
    /// Returns the zero polynomial with the smallest shape compatible with `self`.
    fn zero_like(&self) -> Option<Self>;
    /// Returns the exact sum.
    fn add(&self, other: &Self) -> Option<Self>;
    /// Returns the exact difference.
    fn subtract(&self, other: &Self) -> Option<Self>;
    /// Multiplies every coefficient by `scale`.
    fn scale(&self, scale: &Real) -> Option<Self>;
    /// Returns the exact product.
    fn multiply(&self, other: &Self) -> Option<Self>;
    /// Returns an exact weighted sum in one result.
    fn linear_combination(terms: &[(&Self, &Real)]) -> Option<Self>;
    /// Returns a signed sum of products in one result, subtracting each
    /// product whose flag is set.
    fn sum_products(terms: &[(&Self, &Self, bool)]) -> Option<Self>;
}

impl ExactPolynomialRing for DenseTensorPolynomial {
    fn zero_like(&self) -> Option<Self> {
        Self::zero(vec![1; self.dimensions().len()])
    }
    fn add(&self, other: &Self) -> Option<Self> {
        Self::add(self, other)
    }
    fn subtract(&self, other: &Self) -> Option<Self> {
        Self::subtract(self, other)
    }
    fn scale(&self, scale: &Real) -> Option<Self> {
        Self::scale(self, scale)
    }
    fn multiply(&self, other: &Self) -> Option<Self> {
        Self::multiply(self, other)
    }
    fn linear_combination(terms: &[(&Self, &Real)]) -> Option<Self> {
        Self::linear_combination(terms)
    }
    fn sum_products(terms: &[(&Self, &Self, bool)]) -> Option<Self> {
        Self::sum_products(terms)
    }
}

impl ExactPolynomialRing for TrivariatePolynomial {
    fn zero_like(&self) -> Option<Self> {
        Self::from_axis_polynomial(&[Real::zero()], 0)
    }
    fn add(&self, other: &Self) -> Option<Self> {
        Self::add(self, other)
    }
    fn subtract(&self, other: &Self) -> Option<Self> {
        Self::subtract(self, other)
    }
    fn scale(&self, scale: &Real) -> Option<Self> {
        Self::scale(self, scale)
    }
    fn multiply(&self, other: &Self) -> Option<Self> {
        Self::multiply(self, other)
    }
    fn linear_combination(terms: &[(&Self, &Real)]) -> Option<Self> {
        Self::linear_combination(terms)
    }
    fn sum_products(terms: &[(&Self, &Self, bool)]) -> Option<Self> {
        Self::sum_products(terms)
    }
}

/// `rational + radical*sqrt(D)` with exact polynomial coefficients.
#[derive(Clone, Debug)]
pub struct SquareRootExpression<P> {
    /// Rational part.
    pub rational: P,
    /// Coefficient of `sqrt(D)`.
    pub radical: P,
}

impl<P: ExactPolynomialRing> SquareRootExpression<P> {
    /// Embeds a rational polynomial with a zero radical part.
    pub fn from_rational(rational: P) -> Option<Self> {
        let radical = rational.zero_like()?;
        Some(Self { rational, radical })
    }

    /// Returns the exact sum.
    pub fn add(&self, other: &Self) -> Option<Self> {
        Some(Self {
            rational: self.rational.add(&other.rational)?,
            radical: self.radical.add(&other.radical)?,
        })
    }

    /// Returns the exact difference.
    pub fn subtract(&self, other: &Self) -> Option<Self> {
        Some(Self {
            rational: self.rational.subtract(&other.rational)?,
            radical: self.radical.subtract(&other.radical)?,
        })
    }

    /// Multiplies both parts by `scale`.
    pub fn scale(&self, scale: &Real) -> Option<Self> {
        Some(Self {
            rational: self.rational.scale(scale)?,
            radical: self.radical.scale(scale)?,
        })
    }

    /// Returns an exact weighted sum in one pass per part.
    pub fn linear_combination(terms: &[(&Self, &Real)]) -> Option<Self> {
        let rational = terms
            .iter()
            .map(|(expression, scale)| (&expression.rational, *scale))
            .collect::<Vec<_>>();
        let radical = terms
            .iter()
            .map(|(expression, scale)| (&expression.radical, *scale))
            .collect::<Vec<_>>();
        Some(Self {
            rational: P::linear_combination(&rational)?,
            radical: P::linear_combination(&radical)?,
        })
    }

    /// Multiplies both parts by a rational polynomial.
    pub fn multiply_rational(&self, polynomial: &P) -> Option<Self> {
        Some(Self {
            rational: self.rational.multiply(polynomial)?,
            radical: self.radical.multiply(polynomial)?,
        })
    }

    /// Returns the exact product for radicand `D`.
    pub fn multiply(&self, other: &Self, radicand: &P) -> Option<Self> {
        let radical_product = self.radical.multiply(&other.radical)?;
        Some(Self {
            rational: P::sum_products(&[
                (&self.rational, &other.rational, false),
                (&radical_product, radicand, false),
            ])?,
            radical: P::sum_products(&[
                (&self.rational, &other.radical, false),
                (&self.radical, &other.rational, false),
            ])?,
        })
    }

    /// Returns the exact square for radicand `D`.
    pub fn square(&self, radicand: &P) -> Option<Self> {
        self.multiply(self, radicand)
    }

    /// Returns the norm `rational^2 - radical^2*D`, whose zeros contain the
    /// zeros of both sign branches.
    pub fn projection(&self, radicand: &P) -> Option<P> {
        let rational_squared = self.rational.multiply(&self.rational)?;
        let radical_squared = self.radical.multiply(&self.radical)?;
        rational_squared.subtract(&radical_squared.multiply(radicand)?)
    }
}

/// `rational + first*sqrt(A) + second*sqrt(B) + product*sqrt(A)*sqrt(B)`
/// with exact polynomial coefficients.
#[derive(Clone, Debug)]
pub struct TwoSquareRootExpression<P> {
    /// Rational part.
    pub rational: P,
    /// Coefficient of `sqrt(A)`.
    pub first: P,
    /// Coefficient of `sqrt(B)`.
    pub second: P,
    /// Coefficient of `sqrt(A)*sqrt(B)`.
    pub product: P,
}

impl<P: ExactPolynomialRing> TwoSquareRootExpression<P> {
    /// Embeds a rational polynomial with zero radical parts.
    pub fn from_rational(rational: P) -> Option<Self> {
        let zero = rational.zero_like()?;
        Some(Self {
            rational,
            first: zero.clone(),
            second: zero.clone(),
            product: zero,
        })
    }

    /// Returns `first * sqrt(A)`.
    pub fn from_first_radical(first: P) -> Option<Self> {
        let zero = first.zero_like()?;
        Some(Self {
            rational: zero.clone(),
            first,
            second: zero.clone(),
            product: zero,
        })
    }

    /// Returns `second * sqrt(B)`.
    pub fn from_second_radical(second: P) -> Option<Self> {
        let zero = second.zero_like()?;
        Some(Self {
            rational: zero.clone(),
            first: zero.clone(),
            second,
            product: zero,
        })
    }

    /// Returns the exact sum, or difference when `subtract` is set.
    pub fn combine(&self, other: &Self, subtract: bool) -> Option<Self> {
        let combine = |left: &P, right: &P| {
            if subtract {
                left.subtract(right)
            } else {
                left.add(right)
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

    /// Multiplies every part by a rational polynomial.
    pub fn multiply_rational(&self, polynomial: &P) -> Option<Self> {
        Some(Self {
            rational: self.rational.multiply(polynomial)?,
            first: self.first.multiply(polynomial)?,
            second: self.second.multiply(polynomial)?,
            product: self.product.multiply(polynomial)?,
        })
    }

    /// Returns the exact product for radicands `A` and `B`.
    pub fn multiply(
        &self,
        other: &Self,
        first_speed_squared: &P,
        second_speed_squared: &P,
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
    pub fn square(&self, first_speed_squared: &P, second_speed_squared: &P) -> Option<Self> {
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

    /// Eliminates the second radical, returning `(rational + first*sqrt(A))^2
    /// - B*(second + product*sqrt(A))^2` as an expression in `sqrt(A)`.
    pub fn second_norm(
        &self,
        first_radicand: &P,
        second_radicand: &P,
    ) -> Option<SquareRootExpression<P>> {
        let first_squared = self.first.multiply(&self.first)?;
        let product_squared = self.product.multiply(&self.product)?;
        let rational = self
            .rational
            .multiply(&self.rational)?
            .add(&first_squared.multiply(first_radicand)?)?
            .subtract(
                &second_radicand.multiply(
                    &self
                        .second
                        .multiply(&self.second)?
                        .add(&product_squared.multiply(first_radicand)?)?,
                )?,
            )?;
        let radical = P::sum_products(&[
            (&self.rational, &self.first, false),
            (second_radicand, &self.second.multiply(&self.product)?, true),
        ])?
        .scale(&Real::from(2_i8))?;
        Some(SquareRootExpression { rational, radical })
    }
}

impl TwoSquareRootExpression<DenseTensorPolynomial> {
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
    /// Reduces every part modulo the selected-root defining relations.
    pub fn reduced(&self, sources: &[AlgebraicRootRepresentation]) -> Option<Self> {
        Some(TwoSquareRootExpression {
            rational: dense_reduce_selected_root_relations(self.rational.clone(), sources)?,
            first: dense_reduce_selected_root_relations(self.first.clone(), sources)?,
            second: dense_reduce_selected_root_relations(self.second.clone(), sources)?,
            product: dense_reduce_selected_root_relations(self.product.clone(), sources)?,
        })
    }
    /// Canonicalizes a coefficient-field value at the retained source tuple.
    /// Unlike [`Self::reduced`], these tensors have no free output axis.
    pub fn reduced_at_source_tuple(&self, sources: &[AlgebraicRootRepresentation]) -> Option<Self> {
        Some(TwoSquareRootExpression {
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
