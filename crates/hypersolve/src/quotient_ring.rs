//! Rational-map images over all roots of one polynomial via quotient-ring
//! multiplication matrices.
//!
//! For `y = numerator(x) / denominator(x)` and the roots of `source`, the
//! image values are the roots of `det(N - y*D)`, where `N` and `D` multiply by
//! the numerator and denominator in `Q[x] / (source)`.

use hyperreal::{Rational as HyperRational, Real, RealSign};

use crate::exact_factor::checked_binomial;

const MAX_QUOTIENT_RING_RATIONAL_IMAGE_DEGREE: usize = 12;

#[derive(Clone)]
#[cfg_attr(test, derive(Debug, PartialEq))]
struct CertifiedRationalInterval {
    lower: HyperRational,
    upper: HyperRational,
}

impl CertifiedRationalInterval {
    fn zero() -> Self {
        Self::point(HyperRational::zero())
    }

    fn point(value: HyperRational) -> Self {
        Self {
            lower: value.clone(),
            upper: value,
        }
    }

    fn add_assign(&mut self, other: &Self) {
        self.lower = &self.lower + &other.lower;
        self.upper = &self.upper + &other.upper;
    }

    fn subtract(&self, other: &Self) -> Self {
        Self {
            lower: &self.lower - &other.upper,
            upper: &self.upper - &other.lower,
        }
    }

    fn subtract_assign(&mut self, other: &Self) {
        self.lower = &self.lower - &other.upper;
        self.upper = &self.upper - &other.lower;
    }

    fn multiply_scalar(&self, scalar: &HyperRational) -> Self {
        if scalar >= &HyperRational::zero() {
            Self {
                lower: &self.lower * scalar,
                upper: &self.upper * scalar,
            }
        } else {
            Self {
                lower: &self.upper * scalar,
                upper: &self.lower * scalar,
            }
        }
    }

    fn multiply(&self, other: &Self) -> Self {
        let zero = HyperRational::zero();
        if self.lower >= zero {
            if other.lower >= zero {
                Self {
                    lower: &self.lower * &other.lower,
                    upper: &self.upper * &other.upper,
                }
            } else if other.upper <= zero {
                Self {
                    lower: &self.upper * &other.lower,
                    upper: &self.lower * &other.upper,
                }
            } else {
                Self {
                    lower: &self.upper * &other.lower,
                    upper: &self.upper * &other.upper,
                }
            }
        } else if self.upper <= zero {
            if other.lower >= zero {
                Self {
                    lower: &self.lower * &other.upper,
                    upper: &self.upper * &other.lower,
                }
            } else if other.upper <= zero {
                Self {
                    lower: &self.upper * &other.upper,
                    upper: &self.lower * &other.lower,
                }
            } else {
                Self {
                    lower: &self.lower * &other.upper,
                    upper: &self.lower * &other.lower,
                }
            }
        } else if other.lower >= zero {
            Self {
                lower: &self.lower * &other.upper,
                upper: &self.upper * &other.upper,
            }
        } else if other.upper <= zero {
            Self {
                lower: &self.upper * &other.lower,
                upper: &self.lower * &other.lower,
            }
        } else {
            let lower_left = &self.lower * &other.upper;
            let lower_right = &self.upper * &other.lower;
            let upper_left = &self.lower * &other.lower;
            let upper_right = &self.upper * &other.upper;
            Self {
                lower: if lower_left < lower_right {
                    lower_left
                } else {
                    lower_right
                },
                upper: if upper_left > upper_right {
                    upper_left
                } else {
                    upper_right
                },
            }
        }
    }

    fn sign(&self) -> Option<RealSign> {
        if self.lower > HyperRational::zero() {
            Some(RealSign::Positive)
        } else if self.upper < HyperRational::zero() {
            Some(RealSign::Negative)
        } else if self.lower == HyperRational::zero() && self.upper == HyperRational::zero() {
            Some(RealSign::Zero)
        } else {
            None
        }
    }
}

/// Multiplication matrices of a rational map `numerator / denominator` in the
/// quotient ring `Q[x] / (source)`, whose characteristic data determine the
/// map's image over every root of `source`.
pub struct QuotientRingRationalMapMatrices {
    /// Degree of the source polynomial and matrix dimension.
    pub degree: usize,
    /// Row-major multiplication matrix of the numerator.
    pub numerator: Vec<Real>,
    /// Row-major multiplication matrix of the denominator.
    pub denominator: Vec<Real>,
}

/// Builds the quotient-ring multiplication matrices of `numerator /
/// denominator` modulo `source`, for sources of bounded positive degree.
pub fn quotient_ring_rational_map_matrices(
    source: &[Real],
    numerator: &[Real],
    denominator: &[Real],
) -> Option<QuotientRingRationalMapMatrices> {
    let degree = source.len().checked_sub(1)?;
    if degree == 0
        || degree > MAX_QUOTIENT_RING_RATIONAL_IMAGE_DEGREE
        || numerator.is_empty()
        || denominator.is_empty()
    {
        return None;
    }
    let leading = source.last()?;
    let inverse_leading = (!leading.structural_facts().exact_rational)
        .then(|| leading.inverse_ref())
        .transpose()
        .ok()?;
    let numerator_matrix =
        quotient_multiplication_matrix(source, numerator, inverse_leading.as_ref())?;
    let denominator_matrix =
        quotient_multiplication_matrix(source, denominator, inverse_leading.as_ref())?;
    Some(QuotientRingRationalMapMatrices {
        degree,
        numerator: numerator_matrix,
        denominator: denominator_matrix,
    })
}

fn quotient_multiplication_matrix(
    source: &[Real],
    relation: &[Real],
    inverse_leading: Option<&Real>,
) -> Option<Vec<Real>> {
    let degree = source.len().checked_sub(1)?;
    let leading = source.last()?;
    let mut matrix = vec![Real::zero(); degree.checked_mul(degree)?];
    for column in 0..degree {
        let mut remainder = vec![Real::zero(); relation.len().checked_add(column)?];
        remainder[column..].clone_from_slice(relation);
        while remainder.len() > degree {
            let coefficient = remainder.pop()?;
            let shift = remainder.len().checked_sub(degree)?;
            let factor = if let Some(inverse) = inverse_leading {
                coefficient * inverse
            } else {
                (coefficient / leading).ok()?
            };
            for (index, source_coefficient) in source[..degree].iter().enumerate() {
                remainder[shift + index] -= &factor * source_coefficient;
            }
        }
        remainder.resize(degree, Real::zero());
        for (row, coefficient) in remainder.into_iter().enumerate() {
            matrix[row * degree + column] = coefficient;
        }
    }
    Some(matrix)
}

/// Encloses the local Bernstein coefficients of `det(N - y*D)` for `y` in
/// `[lower, upper]` from rational enclosures of the matrices at `precision`,
/// returning their certified signs and the sign of the leading power.
pub fn determinant_local_bernstein_signs_from_enclosures(
    matrices: &QuotientRingRationalMapMatrices,
    lower: &HyperRational,
    upper: &HyperRational,
    precision: i32,
) -> Option<(Vec<RealSign>, RealSign)> {
    let enclose = |value: &Real| {
        value
            .certified_rational_interval(precision)
            .map(|interval| CertifiedRationalInterval {
                lower: interval[0].clone(),
                upper: interval[1].clone(),
            })
    };
    let numerator = matrices
        .numerator
        .iter()
        .map(enclose)
        .collect::<Option<Vec<_>>>()?;
    let denominator = matrices
        .denominator
        .iter()
        .map(enclose)
        .collect::<Option<Vec<_>>>()?;
    let span = upper - lower;
    let constants = numerator
        .iter()
        .zip(&denominator)
        .map(|(numerator, denominator)| numerator.subtract(&denominator.multiply_scalar(lower)))
        .collect::<Vec<_>>();
    let linear = denominator
        .iter()
        .map(|coefficient| coefficient.multiply_scalar(&span))
        .collect::<Vec<_>>();

    let state_count = 1_usize.checked_shl(u32::try_from(matrices.degree).ok()?)?;
    let mut partials = vec![None; state_count];
    partials[0] = Some(vec![CertifiedRationalInterval::point(HyperRational::one())]);
    for mask in 0..state_count {
        let row = usize::try_from(mask.count_ones()).ok()?;
        if row == matrices.degree {
            continue;
        }
        let Some(partial) = partials[mask].take() else {
            continue;
        };
        for column in 0..matrices.degree {
            let column_bit = 1_usize.checked_shl(u32::try_from(column).ok()?)?;
            if mask & column_bit != 0 {
                continue;
            }
            let entry_index = row * matrices.degree + column;
            let negative = (mask >> (column + 1)).count_ones() % 2 != 0;
            let next = partials[mask | column_bit]
                .get_or_insert_with(|| vec![CertifiedRationalInterval::zero(); partial.len() + 1]);
            for (power, coefficient) in partial.iter().enumerate() {
                let constant = coefficient.multiply(&constants[entry_index]);
                let linear = coefficient.multiply(&linear[entry_index]);
                if negative {
                    next[power].subtract_assign(&constant);
                    next[power + 1].add_assign(&linear);
                } else {
                    next[power].add_assign(&constant);
                    next[power + 1].subtract_assign(&linear);
                }
            }
        }
    }
    let power = partials.pop()??;
    let leading_power_sign = match power.last()?.sign()? {
        RealSign::Positive => RealSign::Positive,
        RealSign::Negative => RealSign::Negative,
        RealSign::Zero => return None,
    };
    let mut signs = Vec::with_capacity(matrices.degree + 1);
    for index in 0..=matrices.degree {
        let mut coefficient = CertifiedRationalInterval::zero();
        for (power_index, power_coefficient) in power.iter().enumerate().take(index + 1) {
            let numerator = checked_binomial(index, power_index)?;
            let denominator = checked_binomial(matrices.degree, power_index)?;
            let weight =
                HyperRational::fraction(i64::try_from(numerator).ok()?, denominator).ok()?;
            coefficient.add_assign(&power_coefficient.multiply_scalar(&weight));
        }
        signs.push(coefficient.sign()?);
    }
    Some((signs, leading_power_sign))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn certified_rational_interval_sign_products_match_four_corner_enclosure() {
        for first_lower in -3_i64..=3 {
            for first_upper in first_lower..=3 {
                let first = CertifiedRationalInterval {
                    lower: HyperRational::new(first_lower),
                    upper: HyperRational::new(first_upper),
                };
                for second_lower in -3_i64..=3 {
                    for second_upper in second_lower..=3 {
                        let second = CertifiedRationalInterval {
                            lower: HyperRational::new(second_lower),
                            upper: HyperRational::new(second_upper),
                        };
                        let products = [
                            &first.lower * &second.lower,
                            &first.lower * &second.upper,
                            &first.upper * &second.lower,
                            &first.upper * &second.upper,
                        ];
                        let mut expected_lower = products[0].clone();
                        let mut expected_upper = products[0].clone();
                        for product in products.into_iter().skip(1) {
                            if product < expected_lower {
                                expected_lower = product.clone();
                            }
                            if product > expected_upper {
                                expected_upper = product;
                            }
                        }

                        let product = first.multiply(&second);
                        assert_eq!(product.lower, expected_lower);
                        assert_eq!(product.upper, expected_upper);
                        for scalar in -3_i64..=3 {
                            assert_eq!(
                                first.multiply_scalar(&HyperRational::new(scalar)),
                                first.multiply(&CertifiedRationalInterval::point(
                                    HyperRational::new(scalar),
                                ))
                            );
                        }
                    }
                }
            }
        }
    }
}
