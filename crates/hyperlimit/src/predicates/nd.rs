//! D-dimensional exact determinant predicates.
//!
//! These functions provide the predicate-owner boundary for D-dimensional
//! triangulation crates. They accept coordinate slices, build exact determinant
//! expressions over [`Real`](hyperreal::Real), and return
//! [`PredicateOutcome`](crate::PredicateOutcome) values rather
//! than silent approximate signs. Object crates retain combinatorics while a
//! shared exact predicate layer decides orientation and lifted in-sphere signs.
//!
//! Structurally rational inputs use Bareiss elimination; other inputs retain
//! a division-free exact expression whose sign is resolved under the policy.
//! Shape validation does not require a nondegenerate simplex. Neither path
//! imposes a total arithmetic-work, coefficient-size or allocation budget.

use crate::predicate::PredicatePolicy;
use hyperreal::{Rational, Real};

use crate::predicate::{Certainty, Escalation, PredicateOutcome, RefinementNeed, Sign};
use crate::resolve::resolve_real_sign_direct;

/// Decides the orientation sign of `dimension + 1` points in `dimension` space.
///
/// Columns are `points[i+1] - points[0]`, in the supplied coordinate order.
/// The origin followed by the ordered standard basis has positive sign.
/// In three dimensions this is the opposite sign to
/// [`crate::orient3`]'s last-point-anchored convention.
///
/// Requires a positive dimension and equal coordinate counts. Invalid shape
/// returns Unknown/Unsupported at Exact stage. Structurally rational inputs
/// return Exact certainty at Structural stage; other inputs use the policy's
/// scalar sign resolver. An approximate terminal sign remains qualified, and
/// an unresolved decision remains Unknown. Zero does not reject the input.
pub fn orient_d_with_policy(
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
    resolve_real_sign_direct(&determinant, policy, RefinementNeed::RealRefinement)
}

/// Decides the lifted D-dimensional in-sphere determinant sign.
///
/// `simplex` must contain `dimension + 1` points and `query` must have the same
/// positive dimension. Rows are the simplex points in order followed by the
/// query, with layout `[x_0, ..., x_(dimension-1), ||x||², 1]`.
/// The returned sign is raw; interpreting inside/outside requires a
/// nondegenerate simplex and the caller's orientation convention.
///
/// Shape errors return Unknown/Unsupported at Exact stage. Rational success is
/// Exact/Structural; otherwise the scalar sign resolver preserves the active
/// policy's qualification or unresolved outcome. A zero determinant alone
/// does not establish existence of a unique circumsphere.
pub fn insphere_d_with_policy(
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
    resolve_real_sign_direct(&determinant, policy, RefinementNeed::RealRefinement)
}

/// Returns whether a `dimension + 1` point set is affinely independent.
///
/// Maps the orientation sign to `sign != Zero`, preserving its certainty,
/// stage and any Unknown outcome. An approximate false result is not an
/// exact proof of affine dependence; callers must consume the receipt.
pub fn affine_independent_d_with_policy(
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

/// Returns the positive dimension of a consistently shaped simplex tuple.
///
/// Requires exactly `dimension + 1` rows; performs no independence test.
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

/// Checks positive dimension, simplex arity, and all row/query lengths.
///
/// Returns `None` for shape errors, without testing sphere existence.
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

/// Builds the rational edge-column matrix and returns its Bareiss sign.
///
/// Assumes validated shape. Copies canonical rational coordinates using
/// `Real::exact_rational`; `None` declines a representation, not a geometry
/// or a mathematically rational value hidden in a symbolic expression.
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

/// Eliminates the query row from a rational lifted determinant.
///
/// Assumes validated shape and declines any structurally nonrational coordinate.
/// Subtracting the final query row and expanding along the last column yields
/// rows `[point-query, |point|²-|query|²]`, with the full determinant's sign.
/// The last entry is a difference of norms, not a squared translated norm.
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

/// Sums exact coordinate squares in order, using zero for an empty slice.
fn rational_squared_norm(point: &[Rational]) -> Rational {
    point.iter().fold(Rational::zero(), |sum, coordinate| {
        sum + coordinate * coordinate
    })
}

/// Returns a square rational matrix's determinant sign by Bareiss elimination.
///
/// Requires square input (debug-asserted). Each column uses its first nonzero
/// remaining entry as pivot, tracking row-swap parity. Missing pivots yield
/// zero; division uses the preceding nonzero pivot. The empty determinant is
/// one. Rational arithmetic does not require integer exact divisibility.
///
/// The schedule uses O(n³) rational arithmetic operations and O(n²) stored
/// entries. These are not bit-complexity or byte bounds: coefficient growth
/// and scalar arithmetic costs remain input-dependent.
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

/// Classifies an exact rational, testing positive before negative.
fn rational_sign(value: &Rational) -> Sign {
    if value.is_positive() {
        Sign::Positive
    } else if value.is_negative() {
        Sign::Negative
    } else {
        Sign::Zero
    }
}

/// Builds an exact edge-column determinant expression from validated points.
///
/// Construction alone does not decide its sign or attach certainty.
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

/// Builds the full lifted determinant, with the query as the final row.
///
/// Assumes validated shape; retains coordinate, squared-norm and final-one
/// columns without a symbolic pivot or sign decision.
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

/// Builds the ordered sum of exact coordinate squares, starting at zero.
fn squared_norm(point: &[Real]) -> Real {
    point.iter().fold(Real::zero(), |sum, coordinate| {
        sum + coordinate * coordinate
    })
}

/// Constructs a square matrix determinant without scalar division.
///
/// Uses subset expansion through size sixteen, recursive minors above it.
/// This threshold limits the subset table, not total work or expression size;
/// the recursive fallback can take factorially many expansion steps.
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
/// recursive minors with 2^n table slots and O(n 2^n) scalar arithmetic steps,
/// without requiring a proof of a nonzero symbolic pivot. Those counts exclude
/// expression and scalar-operation costs. Requires a square matrix and a size
/// small enough for the bit mask and allocation; the dispatcher caps it at 16.
/// The empty determinant is one. Appending a column changes sign for each
/// previously selected column to its right, counting permutation inversions.
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
        let partial = sums[mask]
            .take()
            .expect("every determinant subset state is reachable");
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

/// Expands a square matrix along its first row, recursively cloning minors.
///
/// Skips only structurally known zero coefficients, preserving symbolic
/// possibilities. The empty determinant is one; dense expansion has factorial
/// work and supplies no total-resource bound.
fn determinant_recursive(matrix: &[Vec<Real>]) -> Real {
    match matrix.len() {
        0 => Real::one(),
        1 => matrix[0][0].clone(),
        size => {
            let mut total = Real::zero();
            for column in 0..size {
                if matrix[0][column].definitely_zero() {
                    continue;
                }
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

/// Clones entries except the selected row and column for cofactor expansion.
///
/// Internal callers supply valid indices in a square matrix; this helper
/// filters rather than validating those preconditions.
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

/// Determinant-schedule comparisons, symbolic fallbacks and explicit failures.
#[cfg(test)]
mod tests {
    use super::*;

    /// Policy permitting the qualified terminal used by fallback fixtures.
    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    /// Advances a deterministic xorshift stream and emits an integer in -5..=5.
    fn next_coordinate(state: &mut u64) -> Real {
        *state ^= *state << 13;
        *state ^= *state >> 7;
        *state ^= *state << 17;
        Real::from(((*state >> 32) % 11) as i32 - 5)
    }

    /// Constructs the same edge matrix using recursive minors as a schedule oracle.
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

    /// Constructs the full lifted matrix for comparison with reduced Bareiss.
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

    /// Compares rational orientation signs with recursive expansion in dimensions 1–5.
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

    /// Compares reduced rational sphere signs with full lifted minors in dimensions 1–4.
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

    /// Checks pi-coordinate rational declines and generic orientation/sphere values.
    ///
    /// Translation compares returned signs, not their certainty or dispatch counts.
    #[test]
    fn symbolic_nd_inputs_retain_generic_real_fallback() {
        let points = vec![
            vec![Real::zero(), Real::zero()],
            vec![Real::pi(), Real::zero()],
            vec![Real::zero(), Real::one()],
        ];
        assert_eq!(orient_d_exact_rational_sign(&points, 2), None);
        assert_eq!(
            crate::orient_d(&points, APPROX).value(),
            Some(Sign::Positive)
        );

        let quarter = Real::from(Rational::fraction(1, 4).unwrap());
        let rational_simplex = vec![
            vec![Real::zero(), Real::zero()],
            vec![Real::one(), Real::zero()],
            vec![Real::zero(), Real::one()],
        ];
        let rational_query = vec![quarter.clone(), quarter.clone()];
        let expected = crate::insphere_d(&rational_simplex, &rational_query, APPROX).value();

        let translated_simplex = vec![
            vec![Real::pi(), Real::zero()],
            vec![Real::pi() + Real::one(), Real::zero()],
            vec![Real::pi(), Real::one()],
        ];
        let translated_query = vec![Real::pi() + quarter.clone(), quarter];
        assert_eq!(
            insphere_d_exact_rational_sign(&translated_simplex, &translated_query, 2),
            None
        );
        assert_eq!(
            crate::insphere_d(&translated_simplex, &translated_query, APPROX).value(),
            expected
        );

        assert_eq!(
            insphere_d_exact_rational_sign(&translated_simplex, &rational_query, 2),
            None
        );
    }

    /// Retains invalid shape, strict unresolved identity, empty determinant and
    /// sparse size-seventeen fallback witnesses alongside the proved identity.
    #[test]
    fn invalid_and_unresolved_nd_inputs_remain_explicit() {
        let invalid_simplex = vec![vec![Real::zero()], vec![Real::one()]];
        assert_eq!(
            insphere_d_with_policy(&invalid_simplex, &[], PredicatePolicy::STRICT),
            PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Exact)
        );

        let terminal_zero = {
            let sine = Real::e().sin();
            let cosine = Real::e().cos();
            let original = &sine * &sine + &cosine * &cosine - Real::one();
            // Preserve the original identity as a newly certified proof case.
            assert_eq!(
                original.exact_rational_normal_form(),
                Some(hyperreal::Rational::zero())
            );
            // A separate angle-addition identity retains the terminal-policy control.
            crate::test_support::terminally_unresolved_zero()
        };
        let unresolved = vec![vec![Real::zero()], vec![terminal_zero]];
        assert!(matches!(
            affine_independent_d_with_policy(&unresolved, PredicatePolicy::STRICT),
            PredicateOutcome::Unknown { .. }
        ));

        assert_eq!(exact_rational_determinant_sign(Vec::new()), Sign::Positive);

        let sparse_large = vec![vec![Real::zero(); 17]; 17];
        assert_eq!(
            determinant(&sparse_large).exact_rational(),
            Some(Rational::zero())
        );
    }

    /// Compares subset and recursive exact values for integer matrices of sizes 0–7.
    ///
    /// This checks two schedules sharing scalar arithmetic, not a universal proof.
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
