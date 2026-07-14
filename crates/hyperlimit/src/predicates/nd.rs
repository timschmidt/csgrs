//! D-dimensional exact determinant predicates.
//!
//! These functions provide the predicate-owner boundary for D-dimensional
//! triangulation crates. They accept coordinate slices, build exact determinant
//! expressions over [`Real`], and return [`PredicateOutcome`] values rather
//! than silent approximate signs. This is the same layering advocated by Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997): object crates keep combinatorics, while predicate signs are decided
//! by a shared exact predicate layer. The orientation and lifted in-sphere
//! determinants follow the robust-predicate discipline described by Shewchuk,
//! "Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric
//! Predicates," *Discrete & Computational Geometry* 18.3 (1997).

use crate::predicate::PredicatePolicy;
use hyperreal::Real;

use crate::predicate::{PredicateOutcome, RefinementNeed, Sign};
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
    match matrix.len() {
        0 => Real::one(),
        1 => matrix[0][0].clone(),
        size => {
            let mut total = Real::zero();
            for column in 0..size {
                let minor = determinant_minor(matrix, 0, column);
                let term = &matrix[0][column] * &determinant(&minor);
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
