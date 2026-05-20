//! Provides a `MetaBall` struct and functions for creating a `Mesh` from [MetaBalls](https://en.wikipedia.org/wiki/Metaballs)

use crate::float_types::{
    Real, hreal_from_f64, hreal_to_f64, htriangle_area2_exceeds_epsilon, hvector3_from_point3,
    hvector3_from_vector3, tolerance,
};
use crate::mesh::Mesh;
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use fast_surface_nets::{SurfaceNetsBuffer, surface_nets};
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

#[derive(Debug, Clone)]
pub struct MetaBall {
    pub center: Point3<Real>,
    pub radius: Real,
}

/// Diagnostics captured while sampling and meshing 3D metaballs.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MetaballDiagnostics {
    pub resolution: (u32, u32, u32),
    pub ball_count: usize,
    pub sample_count: usize,
    pub finite_sample_count: usize,
    pub non_finite_sample_count: usize,
    pub negative_sample_count: usize,
    pub zero_sample_count: usize,
    pub positive_sample_count: usize,
    pub min_finite_value: Option<Real>,
    pub max_finite_value: Option<Real>,
    pub crossing_cell_count: usize,
    pub surface_nets_vertex_count: usize,
    pub surface_nets_index_count: usize,
    pub emitted_triangle_count: usize,
    pub skipped_non_finite_triangle_count: usize,
    pub degenerate_triangle_count: usize,
}

impl MetaBall {
    pub const fn new(center: Point3<Real>, radius: Real) -> Self {
        Self { center, radius }
    }

    /// **Mathematical Foundation**: Metaball influence function I(p) = r²/(|p-c|² + ε)
    /// where ε prevents division by zero and maintains numerical stability.
    ///
    /// The distance predicate is evaluated through `hyperlattice` and exported
    /// only after the hyperreal squared distance is finite. That keeps
    /// non-finite API-boundary points out of local nalgebra arithmetic and
    /// follows Yap's exact-geometric-computation boundary split
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn influence(&self, p: &Point3<Real>) -> Real {
        if hreal_from_f64(self.radius).is_err() || self.radius <= 0.0 {
            return 0.0;
        }

        let Some(distance_squared) = hyper_point_distance_squared(p, &self.center) else {
            return 0.0;
        };

        // Early termination optimization: if point is very far from metaball,
        // influence approaches zero - can skip expensive division
        let threshold_distance_sq = self.radius * self.radius * 1000.0; // 1000x radius
        if distance_squared > threshold_distance_sq {
            return 0.0;
        }

        // Numerically stable influence calculation with tolerance
        let denominator = distance_squared + tolerance();
        (self.radius * self.radius) / denominator
    }
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// **Creates a Mesh from a list of metaballs** by sampling a 3D grid and using marching cubes.
    ///
    /// - `balls`: slice of metaball definitions (center + radius).
    /// - `resolution`: (nx, ny, nz) defines how many steps along x, y, z.
    /// - `iso_value`: threshold at which the isosurface is extracted.
    /// - `padding`: extra margin around the bounding region (e.g. 0.5) so the surface doesn't get truncated.
    pub fn metaballs(
        balls: &[MetaBall],
        resolution: (usize, usize, usize),
        iso_value: Real,
        padding: Real,
        metadata: M,
    ) -> Mesh<M> {
        Self::metaballs_with_diagnostics(balls, resolution, iso_value, padding, metadata).0
    }

    /// Creates a Mesh from metaballs and returns diagnostics for sampling and
    /// surface-net triangle conversion.
    ///
    /// Metaball centers, radii, padding, iso values, generated vertices, and
    /// generated normals are promoted through hyperreal/hyperlattice boundary
    /// adapters before they contribute to mesh topology. csgrs therefore only
    /// composes the sampled surface into its transitional `Mesh` carrier while
    /// exact-aware predicates remain in the hyper crates, following Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>). Surface
    /// extraction follows Gibson, "Constrained Elastic Surface Nets," MERL
    /// TR99-24, 1999.
    pub fn metaballs_with_diagnostics(
        balls: &[MetaBall],
        resolution: (usize, usize, usize),
        iso_value: Real,
        padding: Real,
        metadata: M,
    ) -> (Mesh<M>, MetaballDiagnostics) {
        let nx = resolution.0.max(2) as u32;
        let ny = resolution.1.max(2) as u32;
        let nz = resolution.2.max(2) as u32;
        let mut diagnostics = MetaballDiagnostics {
            resolution: (nx, ny, nz),
            ball_count: balls.len(),
            sample_count: (nx * ny * nz) as usize,
            ..MetaballDiagnostics::default()
        };

        if balls.is_empty() {
            diagnostics.sample_count = 0;
            return (Mesh::empty(metadata), diagnostics);
        }

        let valid_balls = balls
            .iter()
            .filter(|ball| {
                hvector3_from_point3(&ball.center).is_some()
                    && hreal_from_f64(ball.radius).is_ok()
                    && ball.radius > 0.0
            })
            .collect::<Vec<_>>();

        if valid_balls.is_empty()
            || hreal_from_f64(padding).is_err()
            || padding < 0.0
            || hreal_from_f64(iso_value).is_err()
        {
            diagnostics.non_finite_sample_count = diagnostics.sample_count;
            diagnostics.positive_sample_count = diagnostics.sample_count;
            return (Mesh::empty(metadata), diagnostics);
        }

        // Determine bounding box of all metaballs (plus padding).
        let (min_pt, max_pt) = valid_balls.iter().fold(
            (
                Point3::new(Real::MAX, Real::MAX, Real::MAX),
                Point3::new(-Real::MAX, -Real::MAX, -Real::MAX),
            ),
            |(mut min_p, mut max_p), mb| {
                let r = mb.radius + padding;
                min_p.x = min_p.x.min(mb.center.x - r);
                min_p.y = min_p.y.min(mb.center.y - r);
                min_p.z = min_p.z.min(mb.center.z - r);
                max_p.x = max_p.x.max(mb.center.x + r);
                max_p.y = max_p.y.max(mb.center.y + r);
                max_p.z = max_p.z.max(mb.center.z + r);
                (min_p, max_p)
            },
        );

        // Spacing in each axis
        let dx = (max_pt.x - min_pt.x) / (nx as Real - 1.0);
        let dy = (max_pt.y - min_pt.y) / (ny as Real - 1.0);
        let dz = (max_pt.z - min_pt.z) / (nz as Real - 1.0);

        // Create and fill the scalar-field array with "field_value - iso_value"
        // so that the isosurface will be at 0.
        let array_size = (nx * ny * nz) as usize;
        let mut field_values = vec![0.0; array_size];

        let index_3d = |ix: u32, iy: u32, iz: u32| -> usize {
            (iz * ny + iy) as usize * (nx as usize) + ix as usize
        };

        for iz in 0..nz {
            let zf = min_pt.z + (iz as Real) * dz;
            for iy in 0..ny {
                let yf = min_pt.y + (iy as Real) * dy;
                for ix in 0..nx {
                    let xf = min_pt.x + (ix as Real) * dx;
                    let p = Point3::new(xf, yf, zf);

                    let field_value = valid_balls
                        .iter()
                        .map(|ball| ball.influence(&p))
                        .sum::<Real>();
                    field_values[index_3d(ix, iy, iz)] = if hreal_from_f64(field_value).is_ok()
                    {
                        diagnostics.finite_sample_count += 1;
                        diagnostics.min_finite_value = Some(
                            diagnostics
                                .min_finite_value
                                .map_or(field_value, |current| current.min(field_value)),
                        );
                        diagnostics.max_finite_value = Some(
                            diagnostics
                                .max_finite_value
                                .map_or(field_value, |current| current.max(field_value)),
                        );
                        let shifted = field_value - iso_value;
                        if shifted < 0.0 {
                            diagnostics.negative_sample_count += 1;
                        } else if shifted > 0.0 {
                            diagnostics.positive_sample_count += 1;
                        } else {
                            diagnostics.zero_sample_count += 1;
                        }
                        shifted as f32
                    } else {
                        diagnostics.non_finite_sample_count += 1;
                        diagnostics.positive_sample_count += 1;
                        1e10_f32
                    };
                }
            }
        }

        // Use fast-surface-nets to extract a mesh from this 3D scalar field.
        // We'll define a shape type for ndshape:
        #[allow(non_snake_case)]
        #[derive(Clone, Copy)]
        struct GridShape {
            nx: u32,
            ny: u32,
            nz: u32,
        }
        impl fast_surface_nets::ndshape::Shape<3> for GridShape {
            type Coord = u32;

            #[inline]
            fn as_array(&self) -> [Self::Coord; 3] {
                [self.nx, self.ny, self.nz]
            }

            fn size(&self) -> Self::Coord {
                self.nx * self.ny * self.nz
            }

            fn usize(&self) -> usize {
                (self.nx * self.ny * self.nz) as usize
            }

            fn linearize(&self, coords: [Self::Coord; 3]) -> u32 {
                let [x, y, z] = coords;
                (z * self.ny + y) * self.nx + x
            }

            fn delinearize(&self, i: u32) -> [Self::Coord; 3] {
                let x = i % (self.nx);
                let yz = i / (self.nx);
                let y = yz % (self.ny);
                let z = yz / (self.ny);
                [x, y, z]
            }
        }

        let shape = GridShape { nx, ny, nz };
        diagnostics.crossing_cell_count = count_crossing_cells(&field_values, nx, ny, nz);

        // We'll collect the output into a SurfaceNetsBuffer
        let mut sn_buffer = SurfaceNetsBuffer::default();

        // The region we pass to surface_nets is the entire 3D range [0..nx, 0..ny, 0..nz]
        // minus 1 in each dimension to avoid indexing past the boundary:
        let (max_x, max_y, max_z) = (nx - 1, ny - 1, nz - 1);

        surface_nets(
            &field_values, // SDF array
            &shape,        // custom shape
            [0, 0, 0],     // minimum corner in lattice coords
            [max_x, max_y, max_z],
            &mut sn_buffer,
        );
        diagnostics.surface_nets_vertex_count = sn_buffer.positions.len();
        diagnostics.surface_nets_index_count = sn_buffer.indices.len();

        // Convert the resulting surface net indices/positions into Polygons
        // for the csgrs data structures.
        let mut triangles = Vec::with_capacity(sn_buffer.indices.len() / 3);

        for tri in sn_buffer.indices.chunks_exact(3) {
            let i0 = tri[0] as usize;
            let i1 = tri[1] as usize;
            let i2 = tri[2] as usize;

            let p0_index = sn_buffer.positions[i0];
            let p1_index = sn_buffer.positions[i1];
            let p2_index = sn_buffer.positions[i2];

            // Convert from index space to real (world) space:
            let p0_real = Point3::new(
                min_pt.x + p0_index[0] as Real * dx,
                min_pt.y + p0_index[1] as Real * dy,
                min_pt.z + p0_index[2] as Real * dz,
            );

            let p1_real = Point3::new(
                min_pt.x + p1_index[0] as Real * dx,
                min_pt.y + p1_index[1] as Real * dy,
                min_pt.z + p1_index[2] as Real * dz,
            );

            let p2_real = Point3::new(
                min_pt.x + p2_index[0] as Real * dx,
                min_pt.y + p2_index[1] as Real * dy,
                min_pt.z + p2_index[2] as Real * dz,
            );

            // Likewise for the normals if you want them in true world space.
            // Usually you'd need to do an inverse-transpose transform if your
            // scale is non-uniform. For uniform voxels, scaling is simpler:

            let n0 = sn_buffer.normals[i0];
            let n1 = sn_buffer.normals[i1];
            let n2 = sn_buffer.normals[i2];

            let n0v = Vector3::new(n0[0] as Real, n0[1] as Real, n0[2] as Real);
            let n1v = Vector3::new(n1[0] as Real, n1[1] as Real, n1[2] as Real);
            let n2v = Vector3::new(n2[0] as Real, n2[1] as Real, n2[2] as Real);

            if !(finite_point3(&p0_real)
                && finite_point3(&p1_real)
                && finite_point3(&p2_real)
                && finite_vector3(&n0v)
                && finite_vector3(&n1v)
                && finite_vector3(&n2v))
            {
                diagnostics.skipped_non_finite_triangle_count += 1;
                continue;
            }

            if !htriangle_area2_exceeds_epsilon(&p0_real, &p1_real, &p2_real, Real::EPSILON) {
                diagnostics.degenerate_triangle_count += 1;
            }

            // Construct your vertices:
            let v0 = Vertex::new(p0_real, n0v);
            let v1 = Vertex::new(p1_real, n1v);
            let v2 = Vertex::new(p2_real, n2v);

            // Each tri is turned into a Polygon with 3 vertices
            let poly = Polygon::new(vec![v0, v2, v1], metadata.clone());
            triangles.push(poly);
            diagnostics.emitted_triangle_count += 1;
        }

        // Build and return a Mesh from these polygons
        (Mesh::from_polygons(&triangles, metadata), diagnostics)
    }
}

fn hyper_point_distance_squared(lhs: &Point3<Real>, rhs: &Point3<Real>) -> Option<Real> {
    let lhs = hvector3_from_point3(lhs)?;
    let rhs = hvector3_from_point3(rhs)?;
    hreal_to_f64(&lhs.squared_distance(&rhs))
}

#[inline]
fn finite_point3(point: &Point3<Real>) -> bool {
    hvector3_from_point3(point).is_some()
}

#[inline]
fn finite_vector3(vector: &Vector3<Real>) -> bool {
    hvector3_from_vector3(vector).is_some()
}

fn count_crossing_cells(field_values: &[f32], nx: u32, ny: u32, nz: u32) -> usize {
    if nx < 2 || ny < 2 || nz < 2 {
        return 0;
    }

    let index = |x: u32, y: u32, z: u32| -> usize { ((z * ny + y) * nx + x) as usize };
    let mut count = 0;

    for z in 0..(nz - 1) {
        for y in 0..(ny - 1) {
            for x in 0..(nx - 1) {
                let corners = [
                    field_values[index(x, y, z)],
                    field_values[index(x + 1, y, z)],
                    field_values[index(x, y + 1, z)],
                    field_values[index(x + 1, y + 1, z)],
                    field_values[index(x, y, z + 1)],
                    field_values[index(x + 1, y, z + 1)],
                    field_values[index(x, y + 1, z + 1)],
                    field_values[index(x + 1, y + 1, z + 1)],
                ];
                let has_negative = corners.iter().any(|value| *value < 0.0);
                let has_positive = corners.iter().any(|value| *value > 0.0);
                let has_zero = corners.contains(&0.0);
                if (has_negative && has_positive) || has_zero {
                    count += 1;
                }
            }
        }
    }

    count
}
