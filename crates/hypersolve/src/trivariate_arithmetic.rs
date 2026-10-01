//! Exact arithmetic on dense trivariate parameter polynomials.

use hyperreal::{Real, ZeroKnowledge};

use crate::DenseTensorPolynomial;
use crate::curve_resultant::{BivariatePolynomial, TrivariatePolynomial};

/// Allocates a zero coefficient tensor with the given extents, or `None` on
/// allocation failure.
pub fn try_zero_trivariate_coefficients(dimensions: [usize; 3]) -> Option<Vec<Vec<Vec<Real>>>> {
    dimensions[0]
        .checked_mul(dimensions[1])?
        .checked_mul(dimensions[2])?;
    let mut coefficients = Vec::new();
    coefficients.try_reserve_exact(dimensions[0]).ok()?;
    for _ in 0..dimensions[0] {
        let mut rows = Vec::new();
        rows.try_reserve_exact(dimensions[1]).ok()?;
        for _ in 0..dimensions[1] {
            let mut row = Vec::new();
            row.try_reserve_exact(dimensions[2]).ok()?;
            row.resize_with(dimensions[2], Real::zero);
            rows.push(row);
        }
        coefficients.push(rows);
    }
    Some(coefficients)
}

impl TrivariatePolynomial {
    /// Forms `positive_ab * positive_ac - negative_ab * negative_ac`, where each `ab`
    /// factor is bivariate in the first two axes and each `ac` factor in the first and third.
    pub fn ab_ac_determinant(
        positive_ab: &BivariatePolynomial,
        positive_ac: &BivariatePolynomial,
        negative_ab: &BivariatePolynomial,
        negative_ac: &BivariatePolynomial,
    ) -> Option<Self> {
        let a_count = [(positive_ab, positive_ac), (negative_ab, negative_ac)]
            .into_iter()
            .map(|(ab, ac)| {
                ab.coefficients
                    .len()
                    .saturating_add(ac.coefficients.len())
                    .saturating_sub(1)
            })
            .max()
            .unwrap_or(0);
        let b_count = positive_ab
            .coefficients
            .iter()
            .chain(&negative_ab.coefficients)
            .map(Vec::len)
            .max()
            .unwrap_or(0);
        let c_count = positive_ac
            .coefficients
            .iter()
            .chain(&negative_ac.coefficients)
            .map(Vec::len)
            .max()
            .unwrap_or(0);
        let mut coefficients = try_zero_trivariate_coefficients([a_count, b_count, c_count])?;
        for (ab, ac, sign) in [
            (positive_ab, positive_ac, 1_i8),
            (negative_ab, negative_ac, -1_i8),
        ] {
            let sign = Real::from(sign);
            for (ab_a, row) in ab.coefficients.iter().enumerate() {
                for (b, ab_coefficient) in row.iter().enumerate() {
                    for (ac_a, column) in ac.coefficients.iter().enumerate() {
                        for (c, ac_coefficient) in column.iter().enumerate() {
                            coefficients[ab_a + ac_a][b][c] +=
                                &sign * ab_coefficient * ac_coefficient;
                        }
                    }
                }
            }
        }
        Some(Self { coefficients })
    }

    /// Returns the rectangular tensor extents `(first, second, third)`.
    pub fn dimensions(&self) -> (usize, usize, usize) {
        (
            self.coefficients.len(),
            self.coefficients.iter().map(Vec::len).max().unwrap_or(0),
            self.coefficients
                .iter()
                .flat_map(|rows| rows.iter())
                .map(Vec::len)
                .max()
                .unwrap_or(0),
        )
    }

    /// Lifts a nonempty univariate polynomial onto one axis (`0`, `1` or `2`).
    pub fn from_axis_polynomial(coefficients: &[Real], axis: usize) -> Option<Self> {
        if axis > 2 || coefficients.is_empty() {
            return None;
        }
        let dimensions: [usize; 3] =
            std::array::from_fn(|index| if index == axis { coefficients.len() } else { 1 });
        let mut tensor = try_zero_trivariate_coefficients(dimensions)?;
        for (power, coefficient) in coefficients.iter().enumerate() {
            let mut index = [0; 3];
            index[axis] = power;
            tensor[index[0]][index[1]][index[2]] = coefficient.clone();
        }
        Self::from_coefficients(tensor)
    }

    /// Lifts a univariate polynomial while preserving the conventional empty
    /// coefficient vector as the exact zero polynomial.
    pub fn from_axis_polynomial_or_zero(coefficients: &[Real], axis: usize) -> Option<Self> {
        if coefficients.is_empty() {
            Self::from_axis_polynomial(&[Real::zero()], axis)
        } else {
            Self::from_axis_polynomial(coefficients, axis)
        }
    }

    /// Builds a rectangular tensor, padding ragged rows with zeros and trimming
    /// trailing structurally zero planes, rows and columns. Returns `None` for an
    /// empty tensor or on allocation overflow.
    pub fn from_coefficients(mut coefficients: Vec<Vec<Vec<Real>>>) -> Option<Self> {
        let mut second_count = coefficients.iter().map(Vec::len).max().unwrap_or(0);
        let mut third_count = coefficients
            .iter()
            .flat_map(|rows| rows.iter())
            .map(Vec::len)
            .max()
            .unwrap_or(0);
        if coefficients.is_empty() || second_count == 0 || third_count == 0 {
            return None;
        }
        for rows in &mut coefficients {
            rows.try_reserve(second_count.saturating_sub(rows.len()))
                .ok()?;
            while rows.len() < second_count {
                let mut row = Vec::new();
                row.try_reserve_exact(third_count).ok()?;
                row.resize_with(third_count, Real::zero);
                rows.push(row);
            }
            for row in rows.iter_mut() {
                row.try_reserve(third_count.saturating_sub(row.len()))
                    .ok()?;
                row.resize(third_count, Real::zero());
            }
        }
        while coefficients.len() > 1
            && coefficients.last().is_some_and(|rows| {
                rows.iter()
                    .flatten()
                    .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
            })
        {
            coefficients.pop();
        }
        while second_count > 1
            && coefficients.iter().all(|rows| {
                rows[second_count - 1]
                    .iter()
                    .all(|coefficient| coefficient.zero_status() == ZeroKnowledge::Zero)
            })
        {
            second_count -= 1;
        }
        while third_count > 1
            && coefficients.iter().all(|rows| {
                rows.iter()
                    .all(|row| row[third_count - 1].zero_status() == ZeroKnowledge::Zero)
            })
        {
            third_count -= 1;
        }
        for rows in &mut coefficients {
            rows.truncate(second_count);
            for row in rows {
                row.truncate(third_count);
            }
        }
        coefficients
            .len()
            .checked_mul(second_count)?
            .checked_mul(third_count)?;
        Some(Self { coefficients })
    }

    /// Returns one stored coefficient, if present.
    pub fn coefficient(&self, first: usize, second: usize, third: usize) -> Option<&Real> {
        self.coefficients
            .get(first)
            .and_then(|rows| rows.get(second))
            .and_then(|row| row.get(third))
    }

    /// Returns the exact sum.
    pub fn add(&self, other: &Self) -> Option<Self> {
        self.combine(other, false)
    }

    /// Returns the exact difference.
    pub fn subtract(&self, other: &Self) -> Option<Self> {
        self.combine(other, true)
    }

    /// Returns the exact sum, or difference when `subtract` is set.
    pub fn combine(&self, other: &Self, subtract: bool) -> Option<Self> {
        let first = self.dimensions();
        let second = other.dimensions();
        let dimensions = (
            first.0.max(second.0),
            first.1.max(second.1),
            first.2.max(second.2),
        );
        let mut coefficients =
            try_zero_trivariate_coefficients([dimensions.0, dimensions.1, dimensions.2])?;
        for (first_index, rows) in coefficients.iter_mut().enumerate() {
            for (second_index, row) in rows.iter_mut().enumerate() {
                for (third_index, coefficient) in row.iter_mut().enumerate() {
                    if let Some(value) = self.coefficient(first_index, second_index, third_index) {
                        *coefficient += value;
                    }
                    if let Some(value) = other.coefficient(first_index, second_index, third_index) {
                        if subtract {
                            *coefficient -= value;
                        } else {
                            *coefficient += value;
                        }
                    }
                }
            }
        }
        Self::from_coefficients(coefficients)
    }

    /// Multiplies every coefficient by `scale`.
    pub fn scale(&self, scale: &Real) -> Option<Self> {
        let dimensions = self.dimensions();
        let mut coefficients =
            try_zero_trivariate_coefficients([dimensions.0, dimensions.1, dimensions.2])?;
        for (target_rows, source_rows) in coefficients.iter_mut().zip(&self.coefficients) {
            for (target_row, source_row) in target_rows.iter_mut().zip(source_rows) {
                for (target, source) in target_row.iter_mut().zip(source_row) {
                    *target = source * scale;
                }
            }
        }
        Self::from_coefficients(coefficients)
    }

    /// Forms an exact weighted sum with one rectangular tensor allocation.
    ///
    /// Predicate replay frequently needs a short affine combination of the
    /// retained contact polynomials.  Building each scaled term and then
    /// combining them materializes several dense temporary tensors; direct
    /// accumulation keeps the same coefficient arithmetic while
    /// allocating only the result.
    pub fn linear_combination(terms: &[(&Self, &Real)]) -> Option<Self> {
        let dimensions = terms.iter().fold((0, 0, 0), |dimensions, (term, _)| {
            let term = term.dimensions();
            (
                dimensions.0.max(term.0),
                dimensions.1.max(term.1),
                dimensions.2.max(term.2),
            )
        });
        if dimensions.0 == 0 || dimensions.1 == 0 || dimensions.2 == 0 {
            return None;
        }
        let mut coefficients =
            try_zero_trivariate_coefficients([dimensions.0, dimensions.1, dimensions.2])?;
        for (term, scale) in terms {
            if scale.zero_status() == ZeroKnowledge::Zero {
                continue;
            }
            for (first, rows) in term.coefficients.iter().enumerate() {
                for (second, row) in rows.iter().enumerate() {
                    for (third, coefficient) in row.iter().enumerate() {
                        coefficients[first][second][third] += coefficient * *scale;
                    }
                }
            }
        }
        Self::from_coefficients(coefficients)
    }

    /// Returns the exact product.
    pub fn multiply(&self, other: &Self) -> Option<Self> {
        Self::sum_products(&[(self, other, false)])
    }

    /// Forms a signed sum of polynomial products in one result tensor.
    /// Exact convolution order within each product is unchanged; only the
    /// dense product and subsequent add/subtract temporaries are elided.
    pub fn sum_products(terms: &[(&Self, &Self, bool)]) -> Option<Self> {
        let dimensions = terms.iter().try_fold(
            (0_usize, 0_usize, 0_usize),
            |dimensions, (first, second, _)| {
                let first = first.dimensions();
                let second = second.dimensions();
                Some((
                    dimensions
                        .0
                        .max(first.0.checked_add(second.0)?.checked_sub(1)?),
                    dimensions
                        .1
                        .max(first.1.checked_add(second.1)?.checked_sub(1)?),
                    dimensions
                        .2
                        .max(first.2.checked_add(second.2)?.checked_sub(1)?),
                ))
            },
        )?;
        if dimensions.0 == 0 || dimensions.1 == 0 || dimensions.2 == 0 {
            return None;
        }
        let mut coefficients =
            try_zero_trivariate_coefficients([dimensions.0, dimensions.1, dimensions.2])?;
        for (first, second, subtract) in terms {
            for (first_a, first_rows) in first.coefficients.iter().enumerate() {
                for (second_a, first_row) in first_rows.iter().enumerate() {
                    for (third_a, first_coefficient) in first_row.iter().enumerate() {
                        for (first_b, second_rows) in second.coefficients.iter().enumerate() {
                            for (second_b, second_row) in second_rows.iter().enumerate() {
                                for (third_b, second_coefficient) in second_row.iter().enumerate() {
                                    let target = &mut coefficients[first_a + first_b]
                                        [second_a + second_b][third_a + third_b];
                                    if *subtract {
                                        *target -= first_coefficient * second_coefficient;
                                    } else {
                                        *target += first_coefficient * second_coefficient;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        Self::from_coefficients(coefficients)
    }

    /// Converts to a dense three-axis tensor polynomial.
    pub fn to_dense_polynomial(&self) -> Option<DenseTensorPolynomial> {
        let dimensions = self.dimensions();
        let counts = [dimensions.0, dimensions.1, dimensions.2];
        let coefficient_count = counts.into_iter().try_fold(1_usize, usize::checked_mul)?;
        let mut coefficients = Vec::new();
        coefficients.try_reserve_exact(coefficient_count).ok()?;
        for first in 0..counts[0] {
            for second in 0..counts[1] {
                for third in 0..counts[2] {
                    coefficients.push(
                        self.coefficient(first, second, third)
                            .cloned()
                            .unwrap_or_else(Real::zero),
                    );
                }
            }
        }
        DenseTensorPolynomial::try_new(counts.to_vec(), coefficients)
    }
}
