//! D-dimensional exact determinant predicates.
//!
//! These functions provide the predicate-owner boundary for D-dimensional
//! triangulation crates. They accept coordinate slices, build exact determinant
//! expressions over [`Real`], and return [`PredicateOutcome`] values rather
//! than silent approximate signs. Object crates retain combinatorics while a
//! shared exact predicate layer decides orientation and lifted in-sphere signs.

use crate::predicate::PredicatePolicy;
use hyperreal::{Rational, Real};

use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed, Sign};
use crate::resolve::resolve_real_sign;

/// Decides the orientation sign of `dimension + 1` points in `dimension` space.
///
/// The determinant is built from edge vectors relative to the first point. The
/// function returns [`PredicateOutcome::Unknown`] when dimensions/arity are
/// invalid or when the active policy cannot decide the exact sign.
pub fn orient_d(points: &[Vec<Real>]) -> PredicateOutcome<Sign> {
    orient_d_with_policy(points, PredicatePolicy)
}

/// Policy-controlled variant of [`orient_d`].
pub(crate) fn orient_d_with_policy(
    points: &[Vec<Real>],
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    let Some(dimension) = validate_orientation_points(points) else {
        return PredicateOutcome::unknown(RefinementNeed::Unsupported, crate::Escalation::Exact);
    };
    if let Some(sign) = orient_d_exact_rational_sign(points, dimension) {
        crate::trace_dispatch!("hyperlimit", "orient_d", "exact-rational-bareiss");
        return PredicateOutcome::decided(sign, Certainty::Exact, Escalation::Structural);
    }
    let determinant = orient_d_determinant(points, dimension);
    resolve_real_sign(
        &determinant,
        policy,
        || None,
        || None,
        RefinementNeed::RealRefinement,
    )
}

/// Decides the lifted D-dimensional in-sphere determinant sign.
///
/// `simplex` must contain `dimension + 1` points and `query` must have the same
/// dimension. The returned sign is the raw lifted determinant sign under row
/// layout `[x_0, ..., x_d, ||x||^2, 1]`; triangulation code remains responsible
/// for applying its orientation convention.
pub fn insphere_d(simplex: &[Vec<Real>], query: &[Real]) -> PredicateOutcome<Sign> {
    insphere_d_with_policy(simplex, query, PredicatePolicy)
}

/// Policy-controlled variant of [`insphere_d`].
pub(crate) fn insphere_d_with_policy(
    simplex: &[Vec<Real>],
    query: &[Real],
    policy: PredicatePolicy,
) -> PredicateOutcome<Sign> {
    let Some(dimension) = validate_insphere_points(simplex, query) else {
        return PredicateOutcome::unknown(RefinementNeed::Unsupported, crate::Escalation::Exact);
    };
    if let Some(sign) = insphere_d_exact_rational_sign(simplex, query, dimension) {
        crate::trace_dispatch!("hyperlimit", "insphere_d", "exact-rational-bareiss");
        return PredicateOutcome::decided(sign, Certainty::Exact, Escalation::Structural);
    }
    let determinant = insphere_d_determinant(simplex, query, dimension);
    resolve_real_sign(
        &determinant,
        policy,
        || None,
        || None,
        RefinementNeed::RealRefinement,
    )
}

/// Returns whether a `dimension + 1` point set is affinely independent.
pub fn affine_independent_d(points: &[Vec<Real>]) -> PredicateOutcome<bool> {
    affine_independent_d_with_policy(points, PredicatePolicy)
}

/// Policy-controlled variant of [`affine_independent_d`].
pub(crate) fn affine_independent_d_with_policy(
    points: &[Vec<Real>],
    policy: PredicatePolicy,
) -> PredicateOutcome<bool> {
    match orient_d_with_policy(points, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => PredicateOutcome::decided(value != Sign::Zero, certainty, stage),
        PredicateOutcome::Unknown { needed, stage } => PredicateOutcome::unknown(needed, stage),
    }
}

fn validate_orientation_points(points: &[Vec<Real>]) -> Option<usize> {
    let dimension = points.first()?.len();
    if dimension == 0 || points.len() != dimension + 1 {
        return None;
    }
    points
        .iter()
        .all(|point| point.len() == dimension)
        .then_some(dimension)
}

fn validate_insphere_points(simplex: &[Vec<Real>], query: &[Real]) -> Option<usize> {
    let dimension = simplex.first()?.len();
    if dimension == 0 || simplex.len() != dimension + 1 || query.len() != dimension {
        return None;
    }
    simplex
        .iter()
        .all(|point| point.len() == dimension)
        .then_some(dimension)
}

fn orient_d_exact_rational_sign(points: &[Vec<Real>], dimension: usize) -> Option<Sign> {
    let anchor = points[0]
        .iter()
        .map(Real::exact_rational)
        .collect::<Option<Vec<_>>>()?;
    let mut matrix = Vec::with_capacity(dimension);
    for coordinate in 0..dimension {
        let mut row = Vec::with_capacity(dimension);
        for point in &points[1..] {
            row.push(point[coordinate].exact_rational()? - &anchor[coordinate]);
        }
        matrix.push(row);
    }
    Some(exact_rational_determinant_sign(matrix))
}

fn insphere_d_exact_rational_sign(
    simplex: &[Vec<Real>],
    query: &[Real],
    dimension: usize,
) -> Option<Sign> {
    let query = query
        .iter()
        .map(Real::exact_rational)
        .collect::<Option<Vec<_>>>()?;
    let query_norm = rational_squared_norm(&query);
    let mut matrix = Vec::with_capacity(dimension + 1);
    for point in simplex {
        let point = point
            .iter()
            .map(Real::exact_rational)
            .collect::<Option<Vec<_>>>()?;
        let mut row = Vec::with_capacity(dimension + 1);
        row.extend(
            point
                .iter()
                .zip(&query)
                .map(|(coordinate, query_coordinate)| coordinate - query_coordinate),
        );
        row.push(rational_squared_norm(&point) - &query_norm);
        matrix.push(row);
    }
    Some(exact_rational_determinant_sign(matrix))
}

fn rational_squared_norm(point: &[Rational]) -> Rational {
    point.iter().fold(Rational::zero(), |sum, coordinate| {
        sum + coordinate * coordinate
    })
}

/// Bareiss elimination over exact rationals.
///
/// Although rational inputs do not require the exact-division property for
/// correctness, Bareiss keeps intermediate numerator/denominator growth far
/// below recursive minor expansion and uses O(n^3) arithmetic operations.
fn exact_rational_determinant_sign(mut matrix: Vec<Vec<Rational>>) -> Sign {
    let size = matrix.len();
    debug_assert!(matrix.iter().all(|row| row.len() == size));
    if size == 0 {
        return Sign::Positive;
    }
    if size == 1 {
        return rational_sign(&matrix[0][0]);
    }

    let mut previous_pivot = Rational::one();
    let mut odd_swaps = false;
    for pivot_column in 0..size - 1 {
        let Some(pivot_row) =
            (pivot_column..size).find(|&row| !matrix[row][pivot_column].is_zero())
        else {
            return Sign::Zero;
        };
        if pivot_row != pivot_column {
            matrix.swap(pivot_row, pivot_column);
            odd_swaps = !odd_swaps;
        }

        let pivot = matrix[pivot_column][pivot_column].clone();
        let pivot_values = matrix[pivot_column].clone();
        for row in matrix.iter_mut().skip(pivot_column + 1) {
            let leading = row[pivot_column].clone();
            for column in pivot_column + 1..size {
                let numerator = &row[column] * &pivot - &leading * &pivot_values[column];
                row[column] = if pivot_column == 0 {
                    numerator
                } else {
                    numerator / &previous_pivot
                };
            }
            row[pivot_column] = Rational::zero();
        }
        previous_pivot = pivot;
    }

    let sign = rational_sign(&matrix[size - 1][size - 1]);
    if odd_swaps { sign.reversed() } else { sign }
}

fn rational_sign(value: &Rational) -> Sign {
    if value.is_positive() {
        Sign::Positive
    } else if value.is_negative() {
        Sign::Negative
    } else {
        Sign::Zero
    }
}

fn orient_d_determinant(points: &[Vec<Real>], dimension: usize) -> Real {
    let anchor = &points[0];
    let mut matrix = Vec::with_capacity(dimension);
    for row in 0..dimension {
        let mut values = Vec::with_capacity(dimension);
        for point in &points[1..] {
            values.push(&point[row] - &anchor[row]);
        }
        matrix.push(values);
    }
    determinant(&matrix)
}

fn insphere_d_determinant(simplex: &[Vec<Real>], query: &[Real], dimension: usize) -> Real {
    let mut matrix = Vec::with_capacity(dimension + 2);
    for point in simplex.iter().chain(std::iter::once(&query.to_vec())) {
        let mut row = Vec::with_capacity(dimension + 2);
        row.extend(point.iter().cloned());
        row.push(squared_norm(point));
        row.push(Real::one());
        matrix.push(row);
    }
    determinant(&matrix)
}

fn squared_norm(point: &[Real]) -> Real {
    point.iter().fold(Real::zero(), |sum, coordinate| {
        sum + coordinate * coordinate
    })
}

fn determinant(matrix: &[Vec<Real>]) -> Real {
    if matrix.len() <= 16 {
        determinant_subset_dp(matrix)
    } else {
        determinant_recursive(matrix)
    }
}

/// Division-free subset dynamic programming for determinant expansion.
///
/// A state records the signed sum for assigning the first `popcount(mask)`
/// rows to the selected columns. This evaluates the same Leibniz polynomial as
/// recursive minors in O(n 2^n) states without pivot sign decisions, so it is
/// valid for symbolic `Real` entries where Gaussian elimination cannot safely
/// choose a nonzero pivot.
fn determinant_subset_dp(matrix: &[Vec<Real>]) -> Real {
    let size = matrix.len();
    debug_assert!(matrix.iter().all(|row| row.len() == size));
    if size == 0 {
        return Real::one();
    }
    let state_count = 1_usize << size;
    let mut sums = vec![None; state_count];
    sums[0] = Some(Real::one());

    for mask in 0..state_count - 1 {
        let Some(partial) = sums[mask].take() else {
            continue;
        };
        let row = mask.count_ones() as usize;
        for (column, coefficient) in matrix[row].iter().enumerate() {
            let bit = 1_usize << column;
            if mask & bit != 0 {
                continue;
            }
            let mut term = &partial * coefficient;
            if (mask >> (column + 1)).count_ones() % 2 != 0 {
                term = -term;
            }
            let next = mask | bit;
            sums[next] = Some(match sums[next].take() {
                Some(sum) => sum + term,
                None => term,
            });
        }
    }

    sums[state_count - 1].take().unwrap_or_else(Real::zero)
}

fn determinant_recursive(matrix: &[Vec<Real>]) -> Real {
    match matrix.len() {
        0 => Real::one(),
        1 => matrix[0][0].clone(),
        size => {
            let mut total = Real::zero();
            for column in 0..size {
                let minor = determinant_minor(matrix, 0, column);
                let term = &matrix[0][column] * &determinant_recursive(&minor);
                if column % 2 == 0 {
                    total += term;
                } else {
                    total -= term;
                }
            }
            total
        }
    }
}

fn determinant_minor(
    matrix: &[Vec<Real>],
    remove_row: usize,
    remove_column: usize,
) -> Vec<Vec<Real>> {
    matrix
        .iter()
        .enumerate()
        .filter(|(row, _)| *row != remove_row)
        .map(|(_, row)| {
            row.iter()
                .enumerate()
                .filter(|(column, _)| *column != remove_column)
                .map(|(_, value)| value.clone())
                .collect()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn next_coordinate(state: &mut u64) -> Real {
        *state ^= *state << 13;
        *state ^= *state >> 7;
        *state ^= *state << 17;
        Real::from(((*state >> 32) % 11) as i32 - 5)
    }

    fn recursive_orient_d_determinant(points: &[Vec<Real>], dimension: usize) -> Real {
        let anchor = &points[0];
        let matrix = (0..dimension)
            .map(|row| {
                points[1..]
                    .iter()
                    .map(|point| &point[row] - &anchor[row])
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        determinant_recursive(&matrix)
    }

    fn recursive_insphere_d_determinant(
        simplex: &[Vec<Real>],
        query: &[Real],
        dimension: usize,
    ) -> Real {
        let query_row = query.to_vec();
        let matrix = simplex
            .iter()
            .chain(std::iter::once(&query_row))
            .map(|point| {
                let mut row = Vec::with_capacity(dimension + 2);
                row.extend(point.iter().cloned());
                row.push(squared_norm(point));
                row.push(Real::one());
                row
            })
            .collect::<Vec<_>>();
        determinant_recursive(&matrix)
    }

    #[test]
    fn exact_rational_orient_d_path_matches_recursive_determinant() {
        let mut state = 0x243f_6a88_85a3_08d3_u64;
        for dimension in 1..=5 {
            for _ in 0..200 {
                let points = (0..=dimension)
                    .map(|_| {
                        (0..dimension)
                            .map(|_| next_coordinate(&mut state))
                            .collect::<Vec<_>>()
                    })
                    .collect::<Vec<_>>();
                let expected = rational_sign(
                    &recursive_orient_d_determinant(&points, dimension)
                        .exact_rational()
                        .expect("integer determinant should stay rational"),
                );
                assert_eq!(
                    orient_d_exact_rational_sign(&points, dimension),
                    Some(expected),
                    "dimension={dimension}, points={points:?}",
                );
            }
        }
    }

    #[test]
    fn exact_rational_insphere_d_path_matches_recursive_determinant() {
        let mut state = 0x1319_8a2e_0370_7344_u64;
        for dimension in 1..=4 {
            for _ in 0..100 {
                let simplex = (0..=dimension)
                    .map(|_| {
                        (0..dimension)
                            .map(|_| next_coordinate(&mut state))
                            .collect::<Vec<_>>()
                    })
                    .collect::<Vec<_>>();
                let query = (0..dimension)
                    .map(|_| next_coordinate(&mut state))
                    .collect::<Vec<_>>();
                let expected = rational_sign(
                    &recursive_insphere_d_determinant(&simplex, &query, dimension)
                        .exact_rational()
                        .expect("integer determinant should stay rational"),
                );
                assert_eq!(
                    insphere_d_exact_rational_sign(&simplex, &query, dimension),
                    Some(expected),
                    "dimension={dimension}, simplex={simplex:?}, query={query:?}",
                );
            }
        }
    }

    #[test]
    fn symbolic_nd_inputs_retain_generic_real_fallback() {
        let points = vec![
            vec![Real::zero(), Real::zero()],
            vec![Real::pi(), Real::zero()],
            vec![Real::zero(), Real::one()],
        ];
        assert_eq!(orient_d_exact_rational_sign(&points, 2), None);
        assert_eq!(orient_d(&points).value(), Some(Sign::Positive));
    }

    #[test]
    fn subset_determinant_matches_recursive_minors() {
        let mut state = 0xa409_3822_299f_31d0_u64;
        for size in 0..=7 {
            for _ in 0..100 {
                let matrix = (0..size)
                    .map(|_| {
                        (0..size)
                            .map(|_| next_coordinate(&mut state))
                            .collect::<Vec<_>>()
                    })
                    .collect::<Vec<_>>();
                assert_eq!(
                    determinant_subset_dp(&matrix),
                    determinant_recursive(&matrix),
                    "size={size}, matrix={matrix:?}",
                );
            }
        }
    }
}
