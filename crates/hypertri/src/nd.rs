//! Exact D-dimensional Delaunay complex construction.
//!
//! This module is the `hypertri` bridge toward dimension-generic Delaunay
//! workflows like the external `delaunay` crate, but it keeps the arithmetic
//! contract local: inputs are [`Real`], predicates are exact
//! determinants, and validation is explicit. The implementation intentionally
//! returns a Delaunay **complex** rather than claiming a full triangulation data
//! structure: cospherical degeneracies can produce multiple empty-sphere
//! simplices, and this baseline does not yet own a CGAL-style TDS or bistellar
//! flip scheduler.
//!
//! The empty-circumsphere test is the D-dimensional Delaunay criterion from
//! Delaunay's "Sur la sphère vide" (1934). The determinant predicates follow
//! Shewchuk's robust-predicate discipline, while the separation between exact
//! scalar arithmetic, determinant predicates, and geometric object records
//! follows Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7.1-2 (1997).

use crate::error::{Error, Result};
use crate::kernel::{ExactKernel, Kernel};
use crate::types::{Real, Sign};

/// Exact point in an arbitrary-dimensional Euclidean coordinate space.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PointD {
    coordinates: Vec<Real>,
}

impl PointD {
    /// Construct a point from exact coordinates.
    pub fn new(coordinates: Vec<Real>) -> Self {
        Self { coordinates }
    }

    /// Borrow the point coordinates.
    pub fn coordinates(&self) -> &[Real] {
        &self.coordinates
    }

    /// Return the point dimension.
    pub fn dimension(&self) -> usize {
        self.coordinates.len()
    }
}

/// D-dimensional simplex expressed as point indices.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Simplex {
    indices: Vec<usize>,
}

impl Simplex {
    /// Construct a simplex from point indices.
    pub fn new(indices: Vec<usize>) -> Self {
        Self { indices }
    }

    /// Borrow simplex point indices.
    pub fn indices(&self) -> &[usize] {
        &self.indices
    }
}

/// Exact D-dimensional Delaunay complex over finite input points.
///
/// The cells are all affinely independent `dimension + 1` point subsets whose
/// circumsphere has no other input point strictly inside. Boundary and
/// cospherical degeneracies are preserved as explicit cells instead of being
/// hidden behind floating-point perturbation. That is a conservative Yap-style
/// API: the object reports exactly what the predicates certify, and callers can
/// run [`Self::validate`] before consuming the complex.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct DelaunayComplex {
    dimension: usize,
    points: Vec<PointD>,
    cells: Vec<Simplex>,
}

impl DelaunayComplex {
    /// Construct a Delaunay-complex record from raw parts.
    pub fn from_parts(dimension: usize, points: Vec<PointD>, cells: Vec<Simplex>) -> Self {
        Self {
            dimension,
            points,
            cells,
        }
    }

    /// Return the ambient dimension.
    pub const fn dimension(&self) -> usize {
        self.dimension
    }

    /// Borrow input points.
    pub fn points(&self) -> &[PointD] {
        &self.points
    }

    /// Borrow certified D-dimensional cells.
    pub fn cells(&self) -> &[Simplex] {
        &self.cells
    }

    /// Validate cell arity, index bounds, affine independence, and empty spheres.
    ///
    /// This is intentionally a geometric validation pass, not a PL-manifold or
    /// coverage proof. The validation hierarchy mirrors the external
    /// D-dimensional `delaunay` crate's documented separation between element,
    /// structure, topology, and Delaunay checks, while keeping `hypertri`'s
    /// exact predicate ownership local.
    pub fn validate(&self) -> Result<()> {
        validate_points(&self.points, self.dimension)?;
        for cell in &self.cells {
            validate_simplex_shape(cell, self.points.len(), self.dimension)?;
            let orientation = simplex_orientation(&self.points, cell.indices())?;
            if orientation == Sign::Zero {
                return Err(Error::InvalidInput {
                    reason: "D-dimensional simplex is affinely dependent",
                });
            }
            validate_empty_sphere(&self.points, cell.indices(), orientation)?;
        }
        Ok(())
    }
}

/// Build an exact D-dimensional Delaunay complex by exhaustive predicates.
///
/// This routine is deliberately small and exact. It enumerates all
/// `dimension + 1` point subsets, rejects affinely dependent subsets with an
/// exact orientation determinant, then applies the Delaunay empty-sphere test
/// with an exact lifted determinant. It is appropriate for validation,
/// regression tests, and small scientific inputs; large production D-dimensional
/// workloads should use a TDS/flip pipeline like CGAL or the external
/// `delaunay` crate and treat this exact path as a semantic oracle.
pub fn delaunay_complex(points: &[PointD]) -> Result<DelaunayComplex> {
    let dimension = infer_dimension(points)?;
    validate_points(points, dimension)?;

    if points.len() < dimension + 1 {
        return Ok(DelaunayComplex::from_parts(
            dimension,
            points.to_vec(),
            Vec::new(),
        ));
    }

    let mut cells = Vec::new();
    for indices in combinations(points.len(), dimension + 1) {
        let orientation = simplex_orientation(points, &indices)?;
        if orientation == Sign::Zero {
            continue;
        }
        if simplex_has_empty_sphere(points, &indices, orientation)? {
            cells.push(Simplex::new(indices));
        }
    }

    let complex = DelaunayComplex::from_parts(dimension, points.to_vec(), cells);
    complex.validate()?;
    Ok(complex)
}

fn infer_dimension(points: &[PointD]) -> Result<usize> {
    let Some(first) = points.first() else {
        return Err(Error::InvalidInput {
            reason: "D-dimensional Delaunay input must contain at least one point",
        });
    };
    if first.dimension() == 0 {
        return Err(Error::InvalidInput {
            reason: "D-dimensional points must have at least one coordinate",
        });
    }
    Ok(first.dimension())
}

fn validate_points(points: &[PointD], dimension: usize) -> Result<()> {
    if dimension == 0 {
        return Err(Error::InvalidInput {
            reason: "D-dimensional points must have at least one coordinate",
        });
    }
    for point in points {
        if point.dimension() != dimension {
            return Err(Error::InvalidInput {
                reason: "D-dimensional points must share one ambient dimension",
            });
        }
    }
    for first in 0..points.len() {
        for second in first + 1..points.len() {
            if points[first] == points[second] {
                return Err(Error::InvalidInput {
                    reason: "duplicate D-dimensional points are not supported",
                });
            }
        }
    }
    Ok(())
}

fn validate_simplex_shape(simplex: &Simplex, point_count: usize, dimension: usize) -> Result<()> {
    if simplex.indices.len() != dimension + 1 {
        return Err(Error::InvalidInput {
            reason: "D-dimensional simplex has wrong arity",
        });
    }
    for (offset, &index) in simplex.indices.iter().enumerate() {
        if index >= point_count {
            return Err(Error::InvalidInput {
                reason: "D-dimensional simplex index out of bounds",
            });
        }
        if simplex.indices[..offset].contains(&index) {
            return Err(Error::InvalidInput {
                reason: "D-dimensional simplex repeats a point index",
            });
        }
    }
    Ok(())
}

fn validate_empty_sphere(points: &[PointD], simplex: &[usize], orientation: Sign) -> Result<()> {
    if !simplex_has_empty_sphere(points, simplex, orientation)? {
        return Err(Error::InvalidInput {
            reason: "D-dimensional simplex violates empty-sphere legality",
        });
    }
    Ok(())
}

fn simplex_has_empty_sphere(
    points: &[PointD],
    simplex: &[usize],
    orientation: Sign,
) -> Result<bool> {
    for point_index in 0..points.len() {
        if simplex.contains(&point_index) {
            continue;
        }
        let sphere = insphere_sign(points, simplex, point_index)?;
        if sphere == insphere_inside_sign(points[point_index].dimension(), orientation) {
            return Ok(false);
        }
    }
    Ok(true)
}

fn insphere_inside_sign(dimension: usize, orientation: Sign) -> Sign {
    // The lifted determinant's inside sign alternates with dimension under the
    // row layout `[x_0, ..., x_d, ||x||^2, 1]`. The 2D case matches the usual
    // in-circle orientation, while 3D reverses it. Keeping that convention
    // explicit here prevents higher-dimensional code from inheriting an
    // accidental 2D-only sign rule; see Shewchuk's predicate treatment and
    // Yap's exact predicate/object separation.
    if dimension % 2 == 0 {
        orientation
    } else {
        orientation.reversed()
    }
}

fn simplex_orientation(points: &[PointD], simplex: &[usize]) -> Result<Sign> {
    let dimension = points[simplex[0]].dimension();
    let anchor = &points[simplex[0]];
    let mut matrix = Vec::with_capacity(dimension);
    for row in 0..dimension {
        let mut values = Vec::with_capacity(dimension);
        for &index in &simplex[1..] {
            values.push(ExactKernel::sub(
                &points[index].coordinates[row],
                &anchor.coordinates[row],
            ));
        }
        matrix.push(values);
    }
    sign_of_determinant(matrix, "D-dimensional orientation")
}

fn insphere_sign(points: &[PointD], simplex: &[usize], point_index: usize) -> Result<Sign> {
    let dimension = points[point_index].dimension();
    let mut matrix = Vec::with_capacity(dimension + 2);
    for &index in simplex.iter().chain(std::iter::once(&point_index)) {
        let point = &points[index];
        let mut row = Vec::with_capacity(dimension + 2);
        row.extend(point.coordinates.iter().cloned());
        row.push(squared_norm(point));
        row.push(ExactKernel::from_i64(1));
        matrix.push(row);
    }
    sign_of_determinant(matrix, "D-dimensional in-sphere")
}

fn squared_norm(point: &PointD) -> Real {
    point
        .coordinates
        .iter()
        .fold(ExactKernel::zero(), |sum, coordinate| {
            ExactKernel::add(&sum, &ExactKernel::mul(coordinate, coordinate))
        })
}

fn sign_of_determinant(matrix: Vec<Vec<Real>>, predicate: &'static str) -> Result<Sign> {
    ExactKernel::real_sign(&determinant(&matrix))
        .map_err(|_| Error::PredicateUndecided { predicate })
}

fn determinant(matrix: &[Vec<Real>]) -> Real {
    match matrix.len() {
        0 => ExactKernel::from_i64(1),
        1 => matrix[0][0].clone(),
        size => {
            let mut total = ExactKernel::zero();
            for column in 0..size {
                let minor = determinant_minor(matrix, 0, column);
                let term = ExactKernel::mul(&matrix[0][column], &determinant(&minor));
                if column % 2 == 0 {
                    total = ExactKernel::add(&total, &term);
                } else {
                    total = ExactKernel::sub(&total, &term);
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

fn combinations(n: usize, k: usize) -> Vec<Vec<usize>> {
    let mut result = Vec::new();
    let mut current = Vec::with_capacity(k);
    push_combinations(0, n, k, &mut current, &mut result);
    result
}

fn push_combinations(
    start: usize,
    n: usize,
    k: usize,
    current: &mut Vec<usize>,
    result: &mut Vec<Vec<usize>>,
) {
    if current.len() == k {
        result.push(current.clone());
        return;
    }
    let remaining = k - current.len();
    for index in start..=n - remaining {
        current.push(index);
        push_combinations(index + 1, n, k, current, result);
        current.pop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Rational;

    fn p(coords: &[i64]) -> PointD {
        PointD::new(coords.iter().copied().map(Real::from).collect())
    }

    fn q(numerator: i64, denominator: u64) -> Real {
        Real::from(Rational::fraction(numerator, denominator).unwrap())
    }

    #[test]
    fn tetrahedron_forms_one_exact_3d_cell() {
        let points = vec![p(&[0, 0, 0]), p(&[1, 0, 0]), p(&[0, 1, 0]), p(&[0, 0, 1])];

        let complex = delaunay_complex(&points).unwrap();

        assert_eq!(complex.dimension(), 3);
        assert_eq!(complex.cells().len(), 1);
        assert_eq!(complex.cells()[0].indices(), &[0, 1, 2, 3]);
        complex.validate().unwrap();
    }

    #[test]
    fn four_dimensional_simplex_forms_one_cell() {
        let points = vec![
            p(&[0, 0, 0, 0]),
            p(&[1, 0, 0, 0]),
            p(&[0, 1, 0, 0]),
            p(&[0, 0, 1, 0]),
            p(&[0, 0, 0, 1]),
        ];

        let complex = delaunay_complex(&points).unwrap();

        assert_eq!(complex.dimension(), 4);
        assert_eq!(complex.cells().len(), 1);
        complex.validate().unwrap();
    }

    #[test]
    fn tetrahedron_with_exact_interior_point_forms_four_star_cells() {
        let points = vec![
            p(&[0, 0, 0]),
            p(&[1, 0, 0]),
            p(&[0, 1, 0]),
            p(&[0, 0, 1]),
            PointD::new(vec![q(1, 4), q(1, 4), q(1, 4)]),
        ];

        let complex = delaunay_complex(&points).unwrap();

        assert_eq!(complex.dimension(), 3);
        assert_eq!(complex.cells().len(), 4);
        assert!(
            complex
                .cells()
                .iter()
                .all(|cell| cell.indices().contains(&4)),
            "interior point should star the original tetrahedron"
        );
        complex.validate().unwrap();
    }

    #[test]
    fn duplicate_nd_points_are_rejected() {
        let error = delaunay_complex(&[p(&[0, 0, 0]), p(&[0, 0, 0])]).unwrap_err();

        assert_eq!(
            error,
            Error::InvalidInput {
                reason: "duplicate D-dimensional points are not supported"
            }
        );
    }

    #[test]
    fn mixed_dimension_points_are_rejected() {
        let error = delaunay_complex(&[p(&[0, 0]), p(&[1, 0, 0])]).unwrap_err();

        assert_eq!(
            error,
            Error::InvalidInput {
                reason: "D-dimensional points must share one ambient dimension"
            }
        );
    }
}
