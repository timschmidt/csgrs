//! Exact projection of the origin onto a small convex simplex.
//!
//! Collision, distance, and geometric optimization algorithms repeatedly need
//! the closest point in the convex hull of a handful of points. This module
//! enumerates every nonempty face, solves its equality-constrained Gram system
//! with fraction-free Bareiss elimination, rejects negative barycentric
//! coordinates, and compares the retained squared distances exactly.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};

use crate::{BareissError, solve_dense_linear_system_bareiss};

/// Configuration for exact small-simplex projection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SimplexProjectionConfig {
    /// Maximum accepted vertex count. Exhaustive face enumeration is `O(2^n)`.
    pub max_vertices: usize,
    /// Minimum binary precision used by certified sign decisions.
    pub min_precision: i32,
}

impl Default for SimplexProjectionConfig {
    fn default() -> Self {
        Self {
            max_vertices: 8,
            min_precision: -64,
        }
    }
}

/// Invalid input to small-simplex projection.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SimplexProjectionError {
    /// No vertices were supplied.
    EmptySimplex,
    /// Vertex coordinate dimensions differ or are zero.
    DimensionMismatch,
    /// The configured exhaustive face limit was exceeded.
    VertexLimitExceeded {
        /// Supplied vertex count.
        vertex_count: usize,
        /// Configured maximum.
        max_vertices: usize,
    },
}

/// Proof status for the returned closest-point candidate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SimplexProjectionStatus {
    /// Every relevant sign/order decision was certified.
    Certified,
    /// At least one feasible face or distance ordering was undecidable.
    Unknown,
}

/// Exact closest-point and barycentric report.
#[derive(Clone, Debug, PartialEq)]
pub struct SimplexProjectionReport {
    /// Certification status.
    pub status: SimplexProjectionStatus,
    /// Closest point to the origin among certified face candidates.
    pub closest_point: Vec<Real>,
    /// Squared Euclidean distance of `closest_point`.
    pub squared_distance: Real,
    /// Barycentric weights in source vertex order.
    pub barycentric_weights: Vec<Real>,
    /// Source vertex indices carrying nonzero candidate weights.
    pub active_vertices: Vec<usize>,
    /// Number of nonempty faces inspected.
    pub inspected_faces: usize,
    /// Number of affine-dependent faces skipped after a certified singular solve.
    pub singular_faces: usize,
    /// Number of faces whose solve or nonnegativity could not be certified.
    pub unknown_faces: usize,
    /// Number of feasible candidates whose distance ordering was undecidable.
    pub unknown_orderings: usize,
}

/// Project the origin onto a small convex simplex using default limits.
pub fn project_origin_onto_simplex(
    vertices: &[Vec<Real>],
) -> Result<SimplexProjectionReport, SimplexProjectionError> {
    project_origin_onto_simplex_with_config(vertices, SimplexProjectionConfig::default())
}

/// Project the origin onto a small convex simplex with explicit limits.
pub fn project_origin_onto_simplex_with_config(
    vertices: &[Vec<Real>],
    config: SimplexProjectionConfig,
) -> Result<SimplexProjectionReport, SimplexProjectionError> {
    let Some(dimension) = vertices.first().map(Vec::len) else {
        return Err(SimplexProjectionError::EmptySimplex);
    };
    if dimension == 0 || vertices.iter().any(|vertex| vertex.len() != dimension) {
        return Err(SimplexProjectionError::DimensionMismatch);
    }
    if vertices.len() > config.max_vertices || vertices.len() >= usize::BITS as usize {
        return Err(SimplexProjectionError::VertexLimitExceeded {
            vertex_count: vertices.len(),
            max_vertices: config.max_vertices,
        });
    }

    let mut best: Option<FaceCandidate> = None;
    let mut inspected_faces = 0;
    let mut singular_faces = 0;
    let mut unknown_faces = 0;
    let mut unknown_orderings = 0;
    for mask in 1_usize..(1_usize << vertices.len()) {
        inspected_faces += 1;
        let indices = (0..vertices.len())
            .filter(|index| mask & (1_usize << index) != 0)
            .collect::<Vec<_>>();
        let candidate = if indices.len() == 1 {
            Some(single_vertex_candidate(vertices, indices[0]))
        } else {
            match solve_face(vertices, &indices, config.min_precision) {
                FaceSolve::Candidate(candidate) => Some(candidate),
                FaceSolve::Outside => None,
                FaceSolve::Singular => {
                    singular_faces += 1;
                    None
                }
                FaceSolve::Unknown => {
                    unknown_faces += 1;
                    None
                }
            }
        };
        let Some(candidate) = candidate else {
            continue;
        };
        match &best {
            None => best = Some(candidate),
            Some(current) => match compare_real(
                &candidate.squared_distance,
                &current.squared_distance,
                config.min_precision,
            ) {
                Some(Ordering::Less) => best = Some(candidate),
                Some(Ordering::Equal | Ordering::Greater) => {}
                None => unknown_orderings += 1,
            },
        }
    }

    // Singleton faces are always feasible, so a valid nonempty input has a best.
    let best = best.expect("a nonempty simplex always has a singleton face candidate");
    Ok(SimplexProjectionReport {
        status: if unknown_faces == 0 && unknown_orderings == 0 {
            SimplexProjectionStatus::Certified
        } else {
            SimplexProjectionStatus::Unknown
        },
        closest_point: best.point,
        squared_distance: best.squared_distance,
        barycentric_weights: best.weights,
        active_vertices: best.active_vertices,
        inspected_faces,
        singular_faces,
        unknown_faces,
        unknown_orderings,
    })
}

struct FaceCandidate {
    point: Vec<Real>,
    squared_distance: Real,
    weights: Vec<Real>,
    active_vertices: Vec<usize>,
}

enum FaceSolve {
    Candidate(FaceCandidate),
    Outside,
    Singular,
    Unknown,
}

fn single_vertex_candidate(vertices: &[Vec<Real>], index: usize) -> FaceCandidate {
    let point = vertices[index].clone();
    let mut weights = vec![Real::zero(); vertices.len()];
    weights[index] = Real::one();
    FaceCandidate {
        squared_distance: squared_norm(&point),
        point,
        weights,
        active_vertices: vec![index],
    }
}

fn solve_face(vertices: &[Vec<Real>], indices: &[usize], min_precision: i32) -> FaceSolve {
    let count = indices.len();
    let mut matrix = vec![vec![Real::zero(); count + 1]; count + 1];
    for row in 0..count {
        for column in 0..count {
            matrix[row][column] = dot(&vertices[indices[row]], &vertices[indices[column]]);
        }
        matrix[row][count] = Real::one();
        matrix[count][row] = Real::one();
    }
    let mut rhs = vec![Real::zero(); count + 1];
    rhs[count] = Real::one();
    let solution = match solve_dense_linear_system_bareiss(&matrix, &rhs, min_precision) {
        Ok(report) => report.solution,
        Err(BareissError::Singular { .. }) => return FaceSolve::Singular,
        Err(
            BareissError::UndecidedPivot { .. }
            | BareissError::UnsupportedDivision { .. }
            | BareissError::UnsupportedSolutionDivision { .. }
            | BareissError::UnknownResidual,
        ) => return FaceSolve::Unknown,
        Err(BareissError::DimensionMismatch) => {
            unreachable!("the constructed Gram system is square")
        }
    };
    let local_weights = &solution[..count];
    for weight in local_weights {
        match certified_sign(weight, min_precision) {
            Some(RealSign::Negative) => return FaceSolve::Outside,
            Some(RealSign::Zero | RealSign::Positive) => {}
            None => return FaceSolve::Unknown,
        }
    }

    let dimension = vertices[0].len();
    let mut point = vec![Real::zero(); dimension];
    let mut weights = vec![Real::zero(); vertices.len()];
    let mut active_vertices = Vec::new();
    for (local_index, &source_index) in indices.iter().enumerate() {
        let weight = local_weights[local_index].clone();
        weights[source_index] = weight.clone();
        if certified_sign(&weight, min_precision) != Some(RealSign::Zero) {
            active_vertices.push(source_index);
        }
        for axis in 0..dimension {
            point[axis] += weight.clone() * vertices[source_index][axis].clone();
        }
    }
    FaceSolve::Candidate(FaceCandidate {
        squared_distance: squared_norm(&point),
        point,
        weights,
        active_vertices,
    })
}

fn dot(left: &[Real], right: &[Real]) -> Real {
    left.iter()
        .zip(right)
        .fold(Real::zero(), |sum, (left, right)| {
            sum + left.clone() * right.clone()
        })
}

fn squared_norm(point: &[Real]) -> Real {
    dot(point, point)
}

fn compare_real(left: &Real, right: &Real, min_precision: i32) -> Option<Ordering> {
    match certified_sign(&(left - right), min_precision)? {
        RealSign::Negative => Some(Ordering::Less),
        RealSign::Zero => Some(Ordering::Equal),
        RealSign::Positive => Some(Ordering::Greater),
    }
}

fn certified_sign(value: &Real, min_precision: i32) -> Option<RealSign> {
    value.refine_sign_until(min_precision)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(value: i64) -> Real {
        Real::from(value)
    }

    fn point(values: &[i64]) -> Vec<Real> {
        values.iter().copied().map(r).collect()
    }

    fn q(numerator: i64, denominator: i64) -> Real {
        (r(numerator) / r(denominator)).unwrap()
    }

    #[test]
    fn segment_projection_has_exact_barycentric_witness() {
        let report = project_origin_onto_simplex(&[point(&[2, -1]), point(&[2, 1])]).unwrap();

        assert_eq!(report.status, SimplexProjectionStatus::Certified);
        assert_eq!(report.closest_point, point(&[2, 0]));
        assert_eq!(report.squared_distance, r(4));
        assert_eq!(report.barycentric_weights, vec![q(1, 2), q(1, 2)]);
        assert_eq!(report.active_vertices, vec![0, 1]);
    }

    #[test]
    fn triangle_containing_origin_projects_to_zero() {
        let report =
            project_origin_onto_simplex(&[point(&[-1, -1]), point(&[1, -1]), point(&[0, 1])])
                .unwrap();

        assert_eq!(report.status, SimplexProjectionStatus::Certified);
        assert_eq!(report.closest_point, point(&[0, 0]));
        assert_eq!(report.squared_distance, Real::zero());
        assert_eq!(report.active_vertices.len(), 3);
    }

    #[test]
    fn tetrahedron_containing_origin_projects_to_zero() {
        let report = project_origin_onto_simplex(&[
            point(&[1, 1, 1]),
            point(&[-1, -1, 1]),
            point(&[-1, 1, -1]),
            point(&[1, -1, -1]),
        ])
        .unwrap();

        assert_eq!(report.status, SimplexProjectionStatus::Certified);
        assert_eq!(report.closest_point, point(&[0, 0, 0]));
        assert_eq!(report.squared_distance, Real::zero());
        assert_eq!(report.barycentric_weights, vec![q(1, 4); 4]);
    }

    #[test]
    fn affine_dependent_faces_do_not_hide_a_valid_projection() {
        let report =
            project_origin_onto_simplex(&[point(&[1, 0]), point(&[2, 0]), point(&[3, 0])]).unwrap();

        assert_eq!(report.status, SimplexProjectionStatus::Certified);
        assert_eq!(report.closest_point, point(&[1, 0]));
        assert_eq!(report.squared_distance, r(1));
        assert!(report.singular_faces > 0);
    }
}
