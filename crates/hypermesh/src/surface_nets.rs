//! Native-real Surface Nets over regular sampled scalar grids.
//!
//! This module constructs [`TriangleMesh`] coordinates directly from
//! [`hyperreal::Real`](crate::Real) samples. It therefore avoids a primitive
//! floating-point construction boundary, but it does not certify that a finite
//! sample grid captures the topology of the underlying continuous field. The
//! returned mesh is exact geometry for the sampled Surface Nets proposal.

use std::cmp::Ordering;
use std::error::Error;
use std::fmt;

use hyperlattice::{Point3, Real, Vector3};
use hyperlimit::{Sign, TriangleDegeneracy};

use crate::context::{DecisionContext, MeshContext, MeshOutcome};
use crate::mesh::{Triangle, TriangleMesh};

const NO_VERTEX: usize = usize::MAX;

const CUBE_CORNERS: [[u32; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [0, 1, 0],
    [1, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [0, 1, 1],
    [1, 1, 1],
];

const CUBE_EDGES: [[usize; 2]; 12] = [
    [0b000, 0b001],
    [0b000, 0b010],
    [0b000, 0b100],
    [0b001, 0b011],
    [0b001, 0b101],
    [0b010, 0b011],
    [0b010, 0b110],
    [0b011, 0b111],
    [0b100, 0b101],
    [0b100, 0b110],
    [0b101, 0b111],
    [0b110, 0b111],
];

/// Borrowed regular scalar grid consumed by [`surface_nets`].
///
/// Samples use x-fast, then y, then z ordering. `origin` is sample `[0, 0, 0]`
/// and `step` is the exact world-space displacement for one grid index on each
/// axis. Values below zero are inside; exact zero follows the non-negative side
/// of the deterministic Surface Nets ownership rule.
#[derive(Clone, Copy, Debug)]
pub struct SurfaceNetsGrid<'a> {
    /// Exact world-space point at grid index `[0, 0, 0]`.
    pub origin: &'a Point3,
    /// Exact axis-aligned world-space step per grid index.
    pub step: &'a Vector3,
    /// Sample dimensions `[x, y, z]`; every dimension must be at least two.
    pub dimensions: [u32; 3],
    /// Shifted scalar values in x-fast, y-then-z order.
    pub values: &'a [Real],
}

impl<'a> SurfaceNetsGrid<'a> {
    /// Constructs a borrowed regular scalar grid.
    pub const fn new(
        origin: &'a Point3,
        step: &'a Vector3,
        dimensions: [u32; 3],
        values: &'a [Real],
    ) -> Self {
        Self {
            origin,
            step,
            dimensions,
            values,
        }
    }
}

/// Native-real Surface Nets construction result.
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceNetsOutput {
    /// Exact-coordinate mesh for the sampled topology proposal.
    pub mesh: TriangleMesh,
    /// Number of sample cells containing both inside and non-inside corners.
    pub active_cell_count: usize,
}

/// Failure reported while constructing a native-real Surface Nets proposal.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SurfaceNetsError {
    /// Every grid dimension must contain at least two sample points.
    GridTooSmall {
        /// Rejected dimensions.
        dimensions: [u32; 3],
    },
    /// The declared grid point count overflowed the host address space.
    SampleCountOverflow {
        /// Rejected dimensions.
        dimensions: [u32; 3],
    },
    /// The value slice did not match the declared grid point count.
    SampleCountMismatch {
        /// Required number of values.
        expected: usize,
        /// Supplied number of values.
        actual: usize,
    },
    /// A sample sign could not be decided under the selected policy.
    UnknownSampleSign {
        /// Linear sample index.
        sample_index: usize,
    },
    /// Exact interpolation or centroid construction failed unexpectedly.
    VertexConstructionFailed {
        /// Grid cell whose proposal vertex failed.
        cell: [u32; 3],
    },
    /// A crossing edge did not find all four incident active-cell vertices.
    MissingIncidentVertex {
        /// Linear cell-corner index used for the missing lookup.
        sample_index: usize,
    },
    /// A construction predicate could not be decided under the selected policy.
    PredicateUndecided {
        /// Construction stage requiring the decision.
        operation: &'static str,
    },
    /// Surface Nets generated a geometrically degenerate triangle.
    DegenerateTriangle {
        /// Triangle ordinal in construction order.
        triangle_index: usize,
    },
}

impl fmt::Display for SurfaceNetsError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::GridTooSmall { dimensions } => write!(
                formatter,
                "Surface Nets grid dimensions must each be at least two; got {dimensions:?}"
            ),
            Self::SampleCountOverflow { dimensions } => write!(
                formatter,
                "Surface Nets grid point count overflows the host address space for {dimensions:?}"
            ),
            Self::SampleCountMismatch { expected, actual } => write!(
                formatter,
                "Surface Nets grid requires {expected} samples but received {actual}"
            ),
            Self::UnknownSampleSign { sample_index } => write!(
                formatter,
                "Surface Nets sample {sample_index} has no policy-authorized sign"
            ),
            Self::VertexConstructionFailed { cell } => write!(
                formatter,
                "Surface Nets could not construct a proposal vertex for cell {cell:?}"
            ),
            Self::MissingIncidentVertex { sample_index } => write!(
                formatter,
                "Surface Nets crossing edge is missing incident cell vertex {sample_index}"
            ),
            Self::PredicateUndecided { operation } => {
                write!(
                    formatter,
                    "Surface Nets predicate was undecided during {operation}"
                )
            }
            Self::DegenerateTriangle { triangle_index } => write!(
                formatter,
                "Surface Nets generated degenerate triangle {triangle_index}"
            ),
        }
    }
}

impl Error for SurfaceNetsError {}

#[derive(Clone, Copy, Debug)]
struct ActiveCell {
    index: [u32; 3],
    sample_index: usize,
}

struct SurfaceNetsBuilder<'a> {
    grid: SurfaceNetsGrid<'a>,
    negative: Vec<bool>,
    positions: Vec<Point3>,
    triangles: Vec<Triangle>,
    cell_to_vertex: Vec<usize>,
    active_cells: Vec<ActiveCell>,
}

/// Constructs an exact-coordinate Surface Nets proposal over a sampled grid.
///
/// All interpolation, centroid, diagonal-selection, and triangle-degeneracy
/// decisions remain in native `Real` arithmetic. The returned
/// [`MeshOutcome::certainty`] describes only predicates consumed while building
/// the sampled proposal; it is not a certificate that the sample grid captures
/// the continuous field's topology.
///
/// Faces on the positive outer grid planes are deliberately omitted, matching
/// the chunk-friendly Surface Nets ownership convention. Callers requiring a
/// closed solid must provide padding that keeps the level set away from the
/// sample-grid boundary or validate and reject the open result downstream.
pub fn surface_nets(
    context: &MeshContext,
    grid: SurfaceNetsGrid<'_>,
) -> Result<MeshOutcome<SurfaceNetsOutput>, SurfaceNetsError> {
    validate_grid(&grid)?;
    let decisions = DecisionContext::new(context);
    let negative = classify_samples(&decisions, grid.values)?;
    let mut builder = SurfaceNetsBuilder {
        grid,
        negative,
        positions: Vec::new(),
        triangles: Vec::new(),
        cell_to_vertex: vec![NO_VERTEX; grid.values.len()],
        active_cells: Vec::new(),
    };
    builder.estimate_vertices()?;
    builder.build_faces(&decisions)?;
    builder.transform_positions_to_world();
    builder.validate_triangles(&decisions)?;
    let active_cell_count = builder.active_cells.len();
    let mesh = TriangleMesh::new(builder.positions, builder.triangles);
    Ok(decisions.finish(SurfaceNetsOutput {
        mesh,
        active_cell_count,
    }))
}

fn validate_grid(grid: &SurfaceNetsGrid<'_>) -> Result<(), SurfaceNetsError> {
    if grid.dimensions.iter().any(|dimension| *dimension < 2) {
        return Err(SurfaceNetsError::GridTooSmall {
            dimensions: grid.dimensions,
        });
    }
    let expected = grid
        .dimensions
        .iter()
        .try_fold(1_usize, |count, dimension| {
            count.checked_mul(*dimension as usize)
        })
        .ok_or(SurfaceNetsError::SampleCountOverflow {
            dimensions: grid.dimensions,
        })?;
    if expected != grid.values.len() {
        return Err(SurfaceNetsError::SampleCountMismatch {
            expected,
            actual: grid.values.len(),
        });
    }
    Ok(())
}

fn classify_samples(
    decisions: &DecisionContext,
    values: &[Real],
) -> Result<Vec<bool>, SurfaceNetsError> {
    values
        .iter()
        .enumerate()
        .map(|(sample_index, value)| {
            decisions
                .probe(hyperlimit::classify_real_sign(value, decisions.policy()))
                .map(|sign| sign == Sign::Negative)
                .ok_or(SurfaceNetsError::UnknownSampleSign { sample_index })
        })
        .collect()
}

impl SurfaceNetsBuilder<'_> {
    fn estimate_vertices(&mut self) -> Result<(), SurfaceNetsError> {
        let [nx, ny, nz] = self.grid.dimensions;
        for z in 0..(nz - 1) {
            for y in 0..(ny - 1) {
                for x in 0..(nx - 1) {
                    let cell = [x, y, z];
                    let corner_indices = CUBE_CORNERS.map(|corner| {
                        self.sample_index(x + corner[0], y + corner[1], z + corner[2])
                    });
                    let negative_count = corner_indices
                        .iter()
                        .filter(|index| self.negative[**index])
                        .count();
                    if negative_count == 0 || negative_count == CUBE_CORNERS.len() {
                        continue;
                    }
                    let position = self.cell_vertex(cell, corner_indices)?;
                    let sample_index = self.sample_index(x, y, z);
                    self.cell_to_vertex[sample_index] = self.positions.len();
                    self.positions.push(position);
                    self.active_cells.push(ActiveCell {
                        index: cell,
                        sample_index,
                    });
                }
            }
        }
        Ok(())
    }

    fn cell_vertex(
        &self,
        cell: [u32; 3],
        corner_indices: [usize; 8],
    ) -> Result<Point3, SurfaceNetsError> {
        let mut intersections = Vec::with_capacity(CUBE_EDGES.len());
        for [first_corner, second_corner] in CUBE_EDGES {
            let first_index = corner_indices[first_corner];
            let second_index = corner_indices[second_corner];
            if self.negative[first_index] == self.negative[second_index] {
                continue;
            }
            let first_value = &self.grid.values[first_index];
            let second_value = &self.grid.values[second_index];
            let denominator = first_value.clone() - second_value.clone();
            let parameter = (first_value.clone() / denominator)
                .map_err(|_| SurfaceNetsError::VertexConstructionFailed { cell })?;
            let first = local_corner(cell, CUBE_CORNERS[first_corner]);
            let second = local_corner(cell, CUBE_CORNERS[second_corner]);
            intersections.push(first.lerp(&second, &parameter));
        }
        Point3::centroid(&intersections).ok_or(SurfaceNetsError::VertexConstructionFailed { cell })
    }

    fn build_faces(&mut self, decisions: &DecisionContext) -> Result<(), SurfaceNetsError> {
        let [nx, ny, nz] = self.grid.dimensions;
        let strides = [1_usize, nx as usize, (nx as usize) * (ny as usize)];
        for active_index in 0..self.active_cells.len() {
            let active = self.active_cells[active_index];
            let [x, y, z] = active.index;
            if y != 0 && z != 0 && x != nx - 2 {
                self.maybe_add_quad(
                    decisions,
                    active.sample_index,
                    strides[0],
                    strides[1],
                    strides[2],
                )?;
            }
            if x != 0 && z != 0 && y != ny - 2 {
                self.maybe_add_quad(
                    decisions,
                    active.sample_index,
                    strides[1],
                    strides[2],
                    strides[0],
                )?;
            }
            if x != 0 && y != 0 && z != nz - 2 {
                self.maybe_add_quad(
                    decisions,
                    active.sample_index,
                    strides[2],
                    strides[0],
                    strides[1],
                )?;
            }
        }
        Ok(())
    }

    fn maybe_add_quad(
        &mut self,
        decisions: &DecisionContext,
        first_sample: usize,
        axis_stride: usize,
        second_axis_stride: usize,
        third_axis_stride: usize,
    ) -> Result<(), SurfaceNetsError> {
        let second_sample = first_sample + axis_stride;
        let negative_face = match (self.negative[first_sample], self.negative[second_sample]) {
            (true, false) => false,
            (false, true) => true,
            _ => return Ok(()),
        };
        let cell_samples = [
            first_sample,
            first_sample - second_axis_stride,
            first_sample - third_axis_stride,
            first_sample - second_axis_stride - third_axis_stride,
        ];
        let vertices = cell_samples.map(|sample_index| {
            let vertex = self.cell_to_vertex[sample_index];
            (vertex != NO_VERTEX)
                .then_some(vertex)
                .ok_or(SurfaceNetsError::MissingIncidentVertex { sample_index })
        });
        let [first, second, third, fourth] = vertices;
        let [first, second, third, fourth] = [first?, second?, third?, fourth?];
        let first_diagonal = squared_distance(&self.positions[first], &self.positions[fourth]);
        let second_diagonal = squared_distance(&self.positions[second], &self.positions[third]);
        let first_is_shorter = decisions
            .probe(hyperlimit::compare_reals(
                &first_diagonal,
                &second_diagonal,
                decisions.policy(),
            ))
            .map(|ordering| ordering == Ordering::Less)
            .ok_or(SurfaceNetsError::PredicateUndecided {
                operation: "quad diagonal selection",
            })?;
        let triangles = if first_is_shorter {
            if negative_face {
                [[first, fourth, second], [first, third, fourth]]
            } else {
                [[first, second, fourth], [first, fourth, third]]
            }
        } else if negative_face {
            [[second, third, fourth], [second, first, third]]
        } else {
            [[second, fourth, third], [second, third, first]]
        };
        for indices in triangles {
            self.push_triangle(indices);
        }
        Ok(())
    }

    fn push_triangle(&mut self, [first, second, third]: [usize; 3]) {
        self.triangles.push(Triangle::new(first, second, third));
    }

    fn transform_positions_to_world(&mut self) {
        for position in &mut self.positions {
            *position = Point3::new(
                self.grid.origin.x.clone() + self.grid.step[0].clone() * position.x.clone(),
                self.grid.origin.y.clone() + self.grid.step[1].clone() * position.y.clone(),
                self.grid.origin.z.clone() + self.grid.step[2].clone() * position.z.clone(),
            );
        }
    }

    fn validate_triangles(&self, decisions: &DecisionContext) -> Result<(), SurfaceNetsError> {
        for (triangle_index, triangle) in self.triangles.iter().enumerate() {
            let [first, second, third] = triangle.indices();
            let degeneracy = decisions
                .probe(hyperlimit::classify_triangle3_degeneracy(
                    &self.positions[first],
                    &self.positions[second],
                    &self.positions[third],
                    decisions.policy(),
                ))
                .ok_or(SurfaceNetsError::PredicateUndecided {
                    operation: "triangle degeneracy validation",
                })?;
            if degeneracy == TriangleDegeneracy::Degenerate {
                return Err(SurfaceNetsError::DegenerateTriangle { triangle_index });
            }
        }
        Ok(())
    }

    fn sample_index(&self, x: u32, y: u32, z: u32) -> usize {
        let [nx, ny, _] = self.grid.dimensions;
        x as usize + (nx as usize) * (y as usize + (ny as usize) * (z as usize))
    }
}

fn local_corner(cell: [u32; 3], corner: [u32; 3]) -> Point3 {
    Point3::new(
        Real::from(u64::from(cell[0] + corner[0])),
        Real::from(u64::from(cell[1] + corner[1])),
        Real::from(u64::from(cell[2] + corner[2])),
    )
}

fn squared_distance(first: &Point3, second: &Point3) -> Real {
    let x = first.x.clone() - second.x.clone();
    let y = first.y.clone() - second.y.clone();
    let z = first.z.clone() - second.z.clone();
    x.clone() * x + y.clone() * y + z.clone() * z
}

#[cfg(test)]
mod tests {
    use hyperreal::Rational;

    use super::*;

    const STRICT: MeshContext = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);

    fn rational(numerator: i64, denominator: u64) -> Real {
        Real::from(Rational::fraction(numerator, denominator).expect("test rational"))
    }

    #[test]
    fn affine_plane_vertices_retain_non_binary_coordinate() {
        let origin = Point3::origin();
        let step = Vector3::new([rational(1, 2), rational(1, 2), rational(1, 2)]);
        let plane = rational(1, 3);
        let mut values = Vec::new();
        for _z in 0..3 {
            for _y in 0..3 {
                for x in 0..3 {
                    values.push(rational(i64::from(x), 2) - plane.clone());
                }
            }
        }

        let outcome = surface_nets(
            &STRICT,
            SurfaceNetsGrid::new(&origin, &step, [3, 3, 3], &values),
        )
        .expect("exact affine grid");

        assert_eq!(outcome.value.active_cell_count, 4);
        assert_eq!(outcome.value.mesh.positions.len(), 4);
        assert_eq!(outcome.value.mesh.triangles.len(), 2);
        assert!(
            outcome
                .value
                .mesh
                .positions
                .iter()
                .all(|position| position.x == plane)
        );
    }

    #[test]
    fn rejects_mismatched_sample_count() {
        let origin = Point3::origin();
        let step = Vector3::new([Real::one(), Real::one(), Real::one()]);
        let error = surface_nets(
            &STRICT,
            SurfaceNetsGrid::new(&origin, &step, [2, 2, 2], &[Real::zero()]),
        )
        .expect_err("malformed grid");

        assert_eq!(
            error,
            SurfaceNetsError::SampleCountMismatch {
                expected: 8,
                actual: 1,
            }
        );
    }
}
