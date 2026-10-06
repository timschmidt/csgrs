//! Exact arithmetic on dense trivariate parameter polynomials.

use hyperreal::{Real, RealSign, ZeroKnowledge};

use crate::DenseTensorPolynomial;
use crate::bivariate_arithmetic::*;
use crate::bivariate_components::parameter_component_bivariate_polynomial_system_complete;
use crate::curve_resultant::{
    BivariatePolynomial, BivariatePolynomialAxisFactorStatus, BivariatePolynomialComponentStatus,
    CurveIntersectionResultantConfig, CurveResultantParameter, TrivariatePolynomial,
    divide_bivariate_polynomial_exact, divide_univariate_polynomial_exact,
    extract_bivariate_polynomial_system_axis_factors,
    greatest_common_divisor_univariate_polynomials_exact,
};
use crate::exact_factor::*;
use crate::real_interval::strict_real_sign;

/// A balanced product of 24 multi-affine factors occupies 25^3 controls. Keep
/// that measured-safe symbolic recursion envelope while sending larger exact
/// products to the complete rank-independent projection below.
pub const MAX_TRIVARIATE_EXACT_FACTOR_SPLITS: usize = 24;
/// Coefficient extent of that bounded factor product along one axis.
pub const MAX_TRIVARIATE_EXACT_FACTOR_COEFFICIENTS: usize = MAX_TRIVARIATE_EXACT_FACTOR_SPLITS + 1;
/// Axis coefficient count up to which multi-affine factors are searched exhaustively.
pub const MAX_EXHAUSTIVE_MULTI_AFFINE_COEFFICIENTS: usize = 9;
/// Maximum bilinear factorizations retained from one bounded search.
pub const MAX_BOUNDED_BILINEAR_FACTORIZATIONS: usize = MAX_TRIVARIATE_EXACT_FACTOR_SPLITS;
/// Factor proposals tried on the first coefficient slice.
pub const MAX_FIRST_BILINEAR_FACTOR_PROPOSALS: usize = 64;
/// Higher-degree slices receive a bounded proposal pass. A proposal can only
/// be accepted by exact division, so exhaustion loses capability rather than
/// exactness.
pub const MAX_BOUNDED_BILINEAR_FACTOR_PROPOSALS: usize = 256;

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

/// Reduces one tensor axis modulo a selected root's defining polynomial.
///
/// The source tensor is rectangularized once, then high powers are consumed
/// in descending order. Each source slice is moved rather than cloned, keeping
/// the cold quotient-ring simplification bounded by the tensor itself.
pub fn trivariate_reduce_axis_mod_defining(
    mut polynomial: TrivariatePolynomial,
    axis: usize,
    defining: &[Real],
) -> Option<TrivariatePolynomial> {
    let degree = defining.len().checked_sub(1)?;
    if axis >= 3 || degree == 0 {
        return None;
    }
    let (first_count, second_count, third_count) = polynomial.dimensions();
    let counts = [first_count, second_count, third_count];
    if counts.into_iter().any(|count| count == 0) || counts[axis] <= degree {
        return None;
    }

    let leading = defining.last()?;
    let relation = defining[..degree]
        .iter()
        .map(|coefficient| ((-coefficient.clone()) / leading.clone()).ok())
        .collect::<Option<Vec<_>>>()?;
    for rows in &mut polynomial.coefficients {
        rows.try_reserve(second_count.saturating_sub(rows.len()))
            .ok()?;
        while rows.len() < second_count {
            let mut row = Vec::new();
            row.try_reserve_exact(third_count).ok()?;
            row.resize_with(third_count, Real::zero);
            rows.push(row);
        }
        for row in rows {
            row.try_reserve(third_count.saturating_sub(row.len()))
                .ok()?;
            row.resize(third_count, Real::zero());
        }
    }

    match axis {
        0 => {
            for power in (degree..first_count).rev() {
                let source = std::mem::take(&mut polynomial.coefficients[power]);
                for (second, row) in source.into_iter().enumerate() {
                    for (third, coefficient) in row.into_iter().enumerate() {
                        for (offset, factor) in relation.iter().enumerate() {
                            polynomial.coefficients[power - degree + offset][second][third] +=
                                &coefficient * factor;
                        }
                    }
                }
            }
            polynomial.coefficients.truncate(degree);
        }
        1 => {
            for rows in &mut polynomial.coefficients {
                for power in (degree..second_count).rev() {
                    let source = std::mem::take(&mut rows[power]);
                    for (third, coefficient) in source.into_iter().enumerate() {
                        for (offset, factor) in relation.iter().enumerate() {
                            rows[power - degree + offset][third] += &coefficient * factor;
                        }
                    }
                }
                rows.truncate(degree);
            }
        }
        2 => {
            for rows in &mut polynomial.coefficients {
                for row in rows {
                    for power in (degree..third_count).rev() {
                        let coefficient = std::mem::replace(&mut row[power], Real::zero());
                        for (offset, factor) in relation.iter().enumerate() {
                            row[power - degree + offset] += &coefficient * factor;
                        }
                    }
                    row.truncate(degree);
                }
            }
        }
        _ => unreachable!(),
    }
    Some(polynomial)
}

/// Returns the univariate coefficients along `axis` at fixed powers of the other two axes.
fn trivariate_axis_fiber(
    polynomial: &TrivariatePolynomial,
    axis: usize,
    first_other: usize,
    second_other: usize,
) -> Option<Vec<Real>> {
    let dimensions = polynomial.dimensions();
    let counts = [dimensions.0, dimensions.1, dimensions.2];
    if axis >= 3 || counts[axis] == 0 {
        return None;
    }
    let remaining = match axis {
        0 => [1, 2],
        1 => [0, 2],
        2 => [0, 1],
        _ => unreachable!(),
    };
    if first_other >= counts[remaining[0]] || second_other >= counts[remaining[1]] {
        return None;
    }
    let mut fiber = Vec::with_capacity(counts[axis]);
    for power in 0..counts[axis] {
        let mut exponents = [0, 0, 0];
        exponents[axis] = power;
        exponents[remaining[0]] = first_other;
        exponents[remaining[1]] = second_other;
        fiber.push(
            polynomial
                .coefficients
                .get(exponents[0])
                .and_then(|rows| rows.get(exponents[1]))
                .and_then(|row| row.get(exponents[2]))
                .cloned()
                .unwrap_or_else(Real::zero),
        );
    }
    while fiber
        .last()
        .is_some_and(|coefficient| strict_real_sign(coefficient) == Some(RealSign::Zero))
    {
        fiber.pop();
    }
    Some(fiber)
}

/// Returns the nonconstant univariate content shared by every tensor fiber
/// along one axis.
pub fn trivariate_axis_content(
    polynomial: &TrivariatePolynomial,
    axis: usize,
) -> Option<Vec<Real>> {
    let dimensions = polynomial.dimensions();
    let counts = [dimensions.0, dimensions.1, dimensions.2];
    let remaining = match axis {
        0 => [1, 2],
        1 => [0, 2],
        2 => [0, 1],
        _ => return None,
    };
    let mut content: Option<Vec<Real>> = None;
    for first_other in 0..counts[remaining[0]] {
        for second_other in 0..counts[remaining[1]] {
            let fiber = trivariate_axis_fiber(polynomial, axis, first_other, second_other)?;
            if fiber.is_empty() {
                continue;
            }
            content = Some(match content {
                None => fiber,
                Some(previous) => {
                    greatest_common_divisor_univariate_polynomials_exact(&previous, &fiber)?
                }
            });
            if content.as_ref().is_none_or(|content| content.len() <= 1) {
                return None;
            }
        }
    }
    content.filter(|content| content.len() > 1)
}

/// Divides every fiber along `axis` exactly by the univariate `content`.
pub fn trivariate_divide_axis_content(
    polynomial: &TrivariatePolynomial,
    axis: usize,
    content: &[Real],
) -> Option<TrivariatePolynomial> {
    let dimensions = polynomial.dimensions();
    let mut counts = [dimensions.0, dimensions.1, dimensions.2];
    if axis >= 3 || content.len() <= 1 || content.len() > counts[axis] {
        return None;
    }
    let remaining = match axis {
        0 => [1, 2],
        1 => [0, 2],
        2 => [0, 1],
        _ => unreachable!(),
    };
    counts[axis] = counts[axis].checked_sub(content.len())?.checked_add(1)?;
    let mut coefficients = try_zero_trivariate_coefficients(counts)?;
    for first_other in 0..counts[remaining[0]] {
        for second_other in 0..counts[remaining[1]] {
            let fiber = trivariate_axis_fiber(polynomial, axis, first_other, second_other)?;
            if fiber.is_empty() {
                continue;
            }
            let quotient = divide_univariate_polynomial_exact(&fiber, content)?;
            if quotient.len() > counts[axis] {
                return None;
            }
            for (power, coefficient) in quotient.into_iter().enumerate() {
                let mut exponents = [0, 0, 0];
                exponents[axis] = power;
                exponents[remaining[0]] = first_other;
                exponents[remaining[1]] = second_other;
                coefficients[exponents[0]][exponents[1]][exponents[2]] = coefficient;
            }
        }
    }
    Some(TrivariatePolynomial { coefficients })
}

/// Substitutes `sum = left + right`, leaving a bivariate polynomial in
/// `(left, right)`.
pub fn trivariate_substitute_sum_axis(
    polynomial: &TrivariatePolynomial,
    left_axis: usize,
    right_axis: usize,
    sum_axis: usize,
) -> Option<BivariatePolynomial> {
    let mut axes = [left_axis, right_axis, sum_axis];
    axes.sort_unstable();
    if axes != [0, 1, 2] {
        return None;
    }
    let dimensions = polynomial.dimensions();
    let counts = [dimensions.0, dimensions.1, dimensions.2];
    let left_count = counts[left_axis];
    let right_count = counts[right_axis];
    let sum_count = counts[sum_axis];
    if left_count == 0 || right_count == 0 || sum_count == 0 {
        return None;
    }
    let first_count = left_count.checked_add(sum_count)?.checked_sub(1)?;
    let second_count = right_count.checked_add(sum_count)?.checked_sub(1)?;
    let mut binomials = Vec::with_capacity(sum_count);
    binomials.push(vec![Real::one()]);
    for power in 1..sum_count {
        let previous = &binomials[power - 1];
        let mut row = vec![Real::one(); power + 1];
        for index in 1..power {
            row[index] = &previous[index - 1] + &previous[index];
        }
        binomials.push(row);
    }
    let mut reduced = try_zero_bivariate_coefficients(first_count, second_count)?;
    for (first, rows) in polynomial.coefficients.iter().enumerate() {
        for (second, row) in rows.iter().enumerate() {
            for (third, coefficient) in row.iter().enumerate() {
                let exponents = [first, second, third];
                let sum_power = exponents[sum_axis];
                for (right_power, binomial) in binomials[sum_power].iter().enumerate() {
                    reduced[exponents[left_axis] + sum_power - right_power]
                        [exponents[right_axis] + right_power] += coefficient * binomial;
                }
            }
        }
    }
    Some(BivariatePolynomial::new(reduced))
}

/// Substitutes `product = left * right`, leaving a bivariate polynomial in
/// `(left, right)`.
pub fn trivariate_substitute_product_axis(
    polynomial: &TrivariatePolynomial,
    left_axis: usize,
    right_axis: usize,
    product_axis: usize,
) -> Option<BivariatePolynomial> {
    let mut axes = [left_axis, right_axis, product_axis];
    axes.sort_unstable();
    if axes != [0, 1, 2] {
        return None;
    }
    let dimensions = polynomial.dimensions();
    let counts = [dimensions.0, dimensions.1, dimensions.2];
    let left_count = counts[left_axis];
    let right_count = counts[right_axis];
    let product_count = counts[product_axis];
    if left_count == 0 || right_count == 0 || product_count == 0 {
        return None;
    }
    let first_count = left_count.checked_add(product_count)?.checked_sub(1)?;
    let second_count = right_count.checked_add(product_count)?.checked_sub(1)?;
    let mut reduced = try_zero_bivariate_coefficients(first_count, second_count)?;
    for (first, rows) in polynomial.coefficients.iter().enumerate() {
        for (second, row) in rows.iter().enumerate() {
            for (third, coefficient) in row.iter().enumerate() {
                let exponents = [first, second, third];
                let product_power = exponents[product_axis];
                reduced[exponents[left_axis] + product_power]
                    [exponents[right_axis] + product_power] += coefficient;
            }
        }
    }
    Some(BivariatePolynomial::new(reduced))
}

/// Substitutes `substituted = scale * retained + offset`, returning a bivariate polynomial in the retained and remaining axes.
pub fn trivariate_substitute_affine_axis(
    polynomial: &TrivariatePolynomial,
    retained_axis: usize,
    substituted_axis: usize,
    scale: &Real,
    offset: &Real,
) -> Option<BivariatePolynomial> {
    if retained_axis >= 3 || substituted_axis >= 3 || retained_axis == substituted_axis {
        return None;
    }
    let remaining_axis = 3_usize.checked_sub(retained_axis + substituted_axis)?;
    if remaining_axis >= 3 || remaining_axis == retained_axis || remaining_axis == substituted_axis
    {
        return None;
    }
    let dimensions = polynomial.dimensions();
    let counts = [dimensions.0, dimensions.1, dimensions.2];
    let retained_count = counts[retained_axis];
    let substituted_count = counts[substituted_axis];
    let remaining_count = counts[remaining_axis];
    if retained_count == 0 || substituted_count == 0 || remaining_count == 0 {
        return None;
    }
    let combined_count = retained_count
        .checked_add(substituted_count)?
        .checked_sub(1)?;
    let affine_powers = polynomial_powers(&[offset.clone(), scale.clone()], substituted_count - 1);
    let mut reduced = try_zero_bivariate_coefficients(combined_count, remaining_count)?;
    for (first, rows) in polynomial.coefficients.iter().enumerate() {
        for (second, row) in rows.iter().enumerate() {
            for (third, coefficient) in row.iter().enumerate() {
                let exponents = [first, second, third];
                for (power, factor) in affine_powers[exponents[substituted_axis]]
                    .iter()
                    .enumerate()
                {
                    reduced[exponents[retained_axis] + power][exponents[remaining_axis]] +=
                        coefficient * factor;
                }
            }
        }
    }
    Some(BivariatePolynomial::new(reduced))
}

/// Splits the polynomial into bivariate coefficients of ascending powers along `axis`, with the remaining axes in ascending order.
pub fn trivariate_axis_bivariate_coefficients(
    polynomial: &TrivariatePolynomial,
    axis: usize,
) -> Option<(Vec<BivariatePolynomial>, [usize; 2])> {
    if axis >= 3 {
        return None;
    }
    let dimensions = polynomial.dimensions();
    let counts = [dimensions.0, dimensions.1, dimensions.2];
    if counts[axis] == 0 {
        return None;
    }
    let remaining = match axis {
        0 => [1, 2],
        1 => [0, 2],
        2 => [0, 1],
        _ => unreachable!(),
    };
    let first_count = counts[remaining[0]];
    let second_count = counts[remaining[1]];
    if first_count == 0 || second_count == 0 {
        return None;
    }
    let mut coefficients = Vec::new();
    coefficients.try_reserve_exact(counts[axis]).ok()?;
    for _ in 0..counts[axis] {
        coefficients.push(try_zero_bivariate_coefficients(first_count, second_count)?);
    }
    for (first, rows) in polynomial.coefficients.iter().enumerate() {
        for (second, row) in rows.iter().enumerate() {
            for (third, coefficient) in row.iter().enumerate() {
                let exponents = [first, second, third];
                coefficients[exponents[axis]][exponents[remaining[0]]][exponents[remaining[1]]] +=
                    coefficient;
            }
        }
    }
    Some((
        coefficients
            .into_iter()
            .map(BivariatePolynomial::new)
            .collect(),
        remaining,
    ))
}

/// Evaluates `axis` at `parameter`, returning the bivariate result and its remaining axes.
pub fn trivariate_specialize_axis_bivariate(
    polynomial: &TrivariatePolynomial,
    axis: usize,
    parameter: &Real,
) -> Option<(BivariatePolynomial, [usize; 2])> {
    let (coefficients, remaining) = trivariate_axis_bivariate_coefficients(polynomial, axis)?;
    let value = coefficients.into_iter().rev().fold(
        BivariatePolynomial::new(vec![vec![Real::zero()]]),
        |value, coefficient| bivariate_add(&bivariate_scale(value, parameter), &coefficient),
    );
    Some((value, remaining))
}

/// Returns the constant and linear bivariate coefficients of a polynomial of degree one along `axis`.
pub fn trivariate_linear_axis_coefficients(
    polynomial: &TrivariatePolynomial,
    axis: usize,
) -> Option<(BivariatePolynomial, BivariatePolynomial, [usize; 2])> {
    let (coefficients, remaining) = trivariate_axis_bivariate_coefficients(polynomial, axis)?;
    let [constant, linear]: [BivariatePolynomial; 2] = coefficients.try_into().ok()?;
    Some((constant, linear, remaining))
}

/// Embeds a bivariate polynomial on two of the three axes.
#[cold]
#[inline(never)]
pub fn trivariate_from_bivariate_axes(
    polynomial: &BivariatePolynomial,
    axes: [usize; 2],
) -> Option<TrivariatePolynomial> {
    if axes[0] >= 3 || axes[1] >= 3 || axes[0] == axes[1] {
        return None;
    }
    let first_count = polynomial.coefficients.len().max(1);
    let second_count = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(1)
        .max(1);
    let mut dimensions = [1; 3];
    dimensions[axes[0]] = first_count;
    dimensions[axes[1]] = second_count;
    let mut coefficients = try_zero_trivariate_coefficients(dimensions)?;
    for (first, row) in polynomial.coefficients.iter().enumerate() {
        for (second, value) in row.iter().enumerate() {
            let mut exponents = [0; 3];
            exponents[axes[0]] = first;
            exponents[axes[1]] = second;
            coefficients[exponents[0]][exponents[1]][exponents[2]] = value.clone();
        }
    }
    TrivariatePolynomial::from_coefficients(coefficients)
}

/// Rebuilds a trivariate polynomial from ascending bivariate coefficients along `axis`.
#[cold]
#[inline(never)]
pub fn trivariate_from_axis_bivariate_coefficients(
    coefficients: &[BivariatePolynomial],
    axis: usize,
    remaining: [usize; 2],
) -> Option<TrivariatePolynomial> {
    let mut axes = [axis, remaining[0], remaining[1]];
    axes.sort_unstable();
    if coefficients.is_empty() || axes != [0, 1, 2] {
        return None;
    }
    let first_count = coefficients
        .iter()
        .map(|coefficient| coefficient.coefficients.len())
        .max()?;
    let second_count = coefficients
        .iter()
        .flat_map(|coefficient| &coefficient.coefficients)
        .map(Vec::len)
        .max()?;
    let mut counts = [0; 3];
    counts[axis] = coefficients.len();
    counts[remaining[0]] = first_count;
    counts[remaining[1]] = second_count;
    let mut result = try_zero_trivariate_coefficients(counts)?;
    for (power, coefficient) in coefficients.iter().enumerate() {
        for (first, row) in coefficient.coefficients.iter().enumerate() {
            for (second, value) in row.iter().enumerate() {
                let mut exponents = [0; 3];
                exponents[axis] = power;
                exponents[remaining[0]] = first;
                exponents[remaining[1]] = second;
                result[exponents[0]][exponents[1]][exponents[2]] = value.clone();
            }
        }
    }
    Some(TrivariatePolynomial {
        coefficients: result,
    })
}

/// Divides exactly by `factor[0] + factor[1] * axis`, returning `None` when the division is not exact.
#[cold]
#[inline(never)]
pub fn trivariate_divide_linear_axis_factor(
    polynomial: &TrivariatePolynomial,
    axis: usize,
    factor: &[BivariatePolynomial; 2],
) -> Option<TrivariatePolynomial> {
    let (coefficients, remaining) = trivariate_axis_bivariate_coefficients(polynomial, axis)?;
    let degree = coefficients.len().checked_sub(1)?;
    if degree == 0 || bivariate_exact_nonzero_metadata(&factor[1])?.is_none() {
        return None;
    }
    let zero = BivariatePolynomial::new(vec![vec![Real::zero()]]);
    let mut quotient = vec![zero; degree];
    for power in (1..=degree).rev() {
        let numerator = if power == degree {
            coefficients[power].clone()
        } else {
            bivariate_subtract(
                &coefficients[power],
                &try_bivariate_multiply(&factor[0], &quotient[power])?,
            )
        };
        quotient[power - 1] = divide_bivariate_polynomial_exact(&numerator, &factor[1])?;
    }
    let remainder = bivariate_subtract(
        &coefficients[0],
        &try_bivariate_multiply(&factor[0], &quotient[0])?,
    );
    if bivariate_exact_nonzero_metadata(&remainder)?.is_some() {
        return None;
    }
    trivariate_from_axis_bivariate_coefficients(&quotient, axis, remaining)
}

/// Reparameterizes every axis from its `(lower, upper)` bounds onto the unit interval.
pub fn trivariate_restrict_to_box_bounds(
    polynomial: &TrivariatePolynomial,
    bounds: [(&Real, &Real); 3],
) -> TrivariatePolynomial {
    let mut restricted = polynomial.coefficients.clone();
    for rows in &mut restricted {
        for row in rows {
            *row = polynomial_restrict_to_interval(row, bounds[2].0, bounds[2].1);
        }
    }
    for rows in &mut restricted {
        let first_row = rows.first().cloned().unwrap_or_default();
        for (c, _) in first_row.iter().enumerate() {
            let coefficients = rows.iter().map(|row| row[c].clone()).collect::<Vec<_>>();
            for (row, coefficient) in rows.iter_mut().zip(polynomial_restrict_to_interval(
                &coefficients,
                bounds[1].0,
                bounds[1].1,
            )) {
                row[c] = coefficient;
            }
        }
    }
    let first_plane = restricted.first().cloned().unwrap_or_default();
    for (b, row) in first_plane.iter().enumerate() {
        for (c, _) in row.iter().enumerate() {
            let coefficients = restricted
                .iter()
                .map(|rows| rows[b][c].clone())
                .collect::<Vec<_>>();
            for (rows, coefficient) in restricted.iter_mut().zip(polynomial_restrict_to_interval(
                &coefficients,
                bounds[0].0,
                bounds[0].1,
            )) {
                rows[b][c] = coefficient;
            }
        }
    }
    TrivariatePolynomial {
        coefficients: restricted,
    }
}

#[cold]
#[inline(never)]
fn bivariate_remove_common_factors(
    mut equations: [BivariatePolynomial; 2],
    config: CurveIntersectionResultantConfig,
) -> [BivariatePolynomial; 2] {
    let axis_report =
        extract_bivariate_polynomial_system_axis_factors(&equations[0], &equations[1]);
    if axis_report.status == BivariatePolynomialAxisFactorStatus::Reduced
        && let Some(reduced) = axis_report.reduced_equations
    {
        equations = reduced;
    }
    loop {
        let degree = bivariate_storage_bidegree_sum(&equations[0])
            .saturating_add(bivariate_storage_bidegree_sum(&equations[1]));
        let mut next = None;
        for retained in [
            CurveResultantParameter::First,
            CurveResultantParameter::Second,
        ] {
            let report = parameter_component_bivariate_polynomial_system_complete(
                &equations[0],
                &equations[1],
                retained,
                config,
            );
            if !matches!(
                report.status,
                BivariatePolynomialComponentStatus::Rational
                    | BivariatePolynomialComponentStatus::Implicit
            ) {
                continue;
            }
            let Some(candidate) = report.reduced_equations else {
                continue;
            };
            let candidate_degree = bivariate_storage_bidegree_sum(&candidate[0])
                .saturating_add(bivariate_storage_bidegree_sum(&candidate[1]));
            if candidate_degree < degree {
                next = Some(candidate);
                break;
            }
        }
        let Some(reduced) = next else {
            return equations;
        };
        equations = reduced;
    }
}

/// Removes coefficient content only when the raw rational-function factor is
/// not already an exact polynomial divisor. Whole-tensor division remains the
/// authority in either case.
#[cold]
#[inline(never)]
pub fn trivariate_normalize_and_divide_linear_axis_factor(
    polynomial: &TrivariatePolynomial,
    axis: usize,
    raw: [BivariatePolynomial; 2],
    config: CurveIntersectionResultantConfig,
) -> Option<([BivariatePolynomial; 2], TrivariatePolynomial)> {
    if let Some(quotient) = trivariate_divide_linear_axis_factor(polynomial, axis, &raw) {
        return Some((raw, quotient));
    }
    let primitive = bivariate_remove_common_factors(raw, config);
    let quotient = trivariate_divide_linear_axis_factor(polynomial, axis, &primitive)?;
    Some((primitive, quotient))
}

#[allow(clippy::too_many_arguments)]
fn trivariate_rational_multi_affine_factor_from_scale(
    polynomial: &TrivariatePolynomial,
    axis: usize,
    remaining: [usize; 2],
    anchor_factor: &BivariatePolynomial,
    top_factor: &BivariatePolynomial,
    scale: &Real,
    anchor: &Real,
    lift_coordinate: usize,
    config: CurveIntersectionResultantConfig,
) -> Option<(TrivariatePolynomial, TrivariatePolynomial)> {
    let raw = rational_multi_affine_lift_factor_coefficients(
        anchor_factor,
        top_factor,
        scale,
        anchor,
        lift_coordinate,
    )?;
    let (factor_coefficients, quotient) =
        trivariate_normalize_and_divide_linear_axis_factor(polynomial, axis, raw, config)?;
    let factor =
        trivariate_from_axis_bivariate_coefficients(&factor_coefficients, axis, remaining)?;
    Some((factor, quotient))
}

/// Splits a quadratic tensor axis when its bivariate discriminant is an exact
/// square, then verifies each recovered factor by exact tensor division.
#[cold]
#[inline(never)]
pub fn trivariate_quadratic_axis_factorizations(
    polynomial: &TrivariatePolynomial,
    axis: usize,
    config: CurveIntersectionResultantConfig,
) -> Option<Vec<(TrivariatePolynomial, TrivariatePolynomial)>> {
    let (coefficients, remaining) = trivariate_axis_bivariate_coefficients(polynomial, axis)?;
    let [constant, linear, quadratic]: [BivariatePolynomial; 3] = coefficients.try_into().ok()?;
    let linear_square = try_bivariate_multiply(&linear, &linear)?;
    let constant_quadratic = try_bivariate_multiply(&constant, &quadratic)?;
    let discriminant = bivariate_subtract(
        &linear_square,
        &bivariate_scale(constant_quadratic, &Real::from(4_i8)),
    );
    let square_root = bivariate_exact_square_root(&discriminant)?;
    let doubled_quadratic = bivariate_scale(quadratic, &Real::from(2_i8));
    let mut factorizations: Vec<(TrivariatePolynomial, TrivariatePolynomial)> =
        Vec::with_capacity(2);
    for constant in [
        bivariate_add(&linear, &square_root),
        bivariate_subtract(&linear, &square_root),
    ] {
        let raw = [constant, doubled_quadratic.clone()];
        let Some((factor_coefficients, quotient)) =
            trivariate_normalize_and_divide_linear_axis_factor(polynomial, axis, raw, config)
        else {
            continue;
        };
        let factor =
            trivariate_from_axis_bivariate_coefficients(&factor_coefficients, axis, remaining)?;
        if factorizations.iter().any(|(existing, _)| {
            existing.coefficients == factor.coefficients
                || existing.coefficients == quotient.coefficients
        }) {
            continue;
        }
        factorizations.push((factor, quotient));
    }
    (!factorizations.is_empty()).then_some(factorizations)
}

/// Recovers one exact rational multi-affine factor from a resource-bounded
/// tensor in `axis`. Specializations only propose bilinear slice factors.
/// Hypersolve exact division proves each slice, derives the inter-slice scale
/// from the translated first-order coefficient or an exact two-anchor
/// projective alignment for repeated factors, and finally proves the complete
/// trivariate factor. Cubic through octic tensors retain exhaustive proposal
/// enumeration; higher degrees receive bounded first-factor passes. Unsupported
/// coefficient towers, exhausted proposal budgets, or degenerate slices make
/// no claim.
#[cold]
#[inline(never)]
pub fn trivariate_rational_multi_affine_axis_factorizations(
    polynomial: &TrivariatePolynomial,
    axis: usize,
    config: CurveIntersectionResultantConfig,
) -> Option<Vec<(TrivariatePolynomial, TrivariatePolynomial)>> {
    let (coefficients, remaining) = trivariate_axis_bivariate_coefficients(polynomial, axis)?;
    if !(4..=MAX_TRIVARIATE_EXACT_FACTOR_COEFFICIENTS).contains(&coefficients.len()) {
        return None;
    }
    let exhaustive = coefficients.len() <= MAX_EXHAUSTIVE_MULTI_AFFINE_COEFFICIENTS;
    // Try the first proved slice factors before enumerating every divisor. The
    // exhaustive pass remains authoritative in its measured-safe envelope.
    // Higher degrees get a capped first-Taylor pass and, when needed, a capped
    // repeated-factor alignment pass.
    for (maximum_factorizations, maximum_proposals, align_anchors, enabled) in [
        (1, MAX_FIRST_BILINEAR_FACTOR_PROPOSALS, false, true),
        (
            MAX_BOUNDED_BILINEAR_FACTORIZATIONS,
            MAX_BOUNDED_BILINEAR_FACTOR_PROPOSALS,
            false,
            !exhaustive,
        ),
        (usize::MAX, usize::MAX, false, exhaustive),
        (
            MAX_BOUNDED_BILINEAR_FACTORIZATIONS,
            MAX_BOUNDED_BILINEAR_FACTOR_PROPOSALS,
            true,
            !exhaustive,
        ),
        (usize::MAX, usize::MAX, true, exhaustive),
    ] {
        if !enabled {
            continue;
        }
        for lift_coordinate in 0..2 {
            let lift_degree = trivariate_axis_lift_degree(&coefficients, lift_coordinate)?;
            if lift_degree == 0 {
                continue;
            }
            let top_slice =
                trivariate_axis_lift_power_slice(&coefficients, lift_coordinate, lift_degree)?;
            let top_factorizations = bivariate_bilinear_factorizations_bounded(
                &top_slice,
                maximum_factorizations,
                maximum_proposals,
            );
            if top_factorizations.is_empty() {
                continue;
            }
            for anchor in [1_i8, 0, -1, 2].map(Real::from) {
                let anchor_slice = trivariate_axis_lift_taylor_slice(
                    &coefficients,
                    lift_coordinate,
                    &anchor,
                    false,
                )?;
                let first_taylor_slice = trivariate_axis_lift_taylor_slice(
                    &coefficients,
                    lift_coordinate,
                    &anchor,
                    true,
                )?;
                for (anchor_factor, anchor_quotient) in bivariate_bilinear_factorizations_bounded(
                    &anchor_slice,
                    maximum_factorizations,
                    maximum_proposals,
                ) {
                    for (top_factor, _) in &top_factorizations {
                        if !align_anchors {
                            if let Some(scale) = rational_multi_affine_lift_scale(
                                &first_taylor_slice,
                                &anchor_factor,
                                top_factor,
                                &anchor_quotient,
                            ) && let Some(factorization) =
                                trivariate_rational_multi_affine_factor_from_scale(
                                    polynomial,
                                    axis,
                                    remaining,
                                    &anchor_factor,
                                    top_factor,
                                    &scale,
                                    &anchor,
                                    lift_coordinate,
                                    config,
                                )
                            {
                                return Some(vec![factorization]);
                            }
                            continue;
                        }
                        for other_anchor in [1_i8, 0, -1, 2].map(Real::from) {
                            if other_anchor == anchor {
                                continue;
                            }
                            let other_slice = trivariate_axis_lift_taylor_slice(
                                &coefficients,
                                lift_coordinate,
                                &other_anchor,
                                false,
                            )?;
                            let anchor_delta = &other_anchor - &anchor;
                            for (other_factor, _) in bivariate_bilinear_factorizations_bounded(
                                &other_slice,
                                maximum_factorizations,
                                maximum_proposals,
                            ) {
                                let Some(scale) = rational_multi_affine_lift_scale_from_anchor_pair(
                                    &anchor_factor,
                                    &other_factor,
                                    top_factor,
                                    &anchor_delta,
                                ) else {
                                    continue;
                                };
                                let Some(factorization) =
                                    trivariate_rational_multi_affine_factor_from_scale(
                                        polynomial,
                                        axis,
                                        remaining,
                                        &anchor_factor,
                                        top_factor,
                                        &scale,
                                        &anchor,
                                        lift_coordinate,
                                        config,
                                    )
                                else {
                                    continue;
                                };
                                return Some(vec![factorization]);
                            }
                        }
                    }
                }
            }
        }
    }
    None
}
