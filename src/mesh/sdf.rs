//! Create `Mesh`s by meshing signed distance fields ([sdf](https://en.wikipedia.org/wiki/Signed_distance_function)) within a bounding box.

use crate::float_types::{
    Real, hreal_from_f64, htriangle_area2_exceeds_epsilon, hvector3_from_point3,
    hvector3_from_vector3,
};
use crate::mesh::Mesh;
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use fast_surface_nets::{SurfaceNetsBuffer, surface_nets};
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

/// Diagnostics captured while sampling and meshing an SDF.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SdfDiagnostics {
    pub resolution: (u32, u32, u32),
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

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// Return a Mesh created by meshing a signed distance field within a bounding box
    ///
    /// ```
    /// # use csgrs::{mesh::Mesh, float_types::Real};
    /// # use nalgebra::Point3;
    /// // Example SDF for a sphere of radius 1.5 centered at (0,0,0)
    /// let my_sdf = |p: &Point3<Real>| (p.x * p.x + p.y * p.y + p.z * p.z).sqrt() - 1.5;
    ///
    /// let resolution = (60, 60, 60);
    /// let min_pt = Point3::new(-2.0, -2.0, -2.0);
    /// let max_pt = Point3::new( 2.0,  2.0,  2.0);
    /// let iso_value = 0.0; // Typically zero for SDF-based surfaces
    ///
    ///    let mesh_shape = Mesh::<()>::sdf(my_sdf, resolution, min_pt, max_pt, iso_value, ());
    ///
    ///    // Now `mesh_shape` is your polygon mesh as a Mesh you can union, subtract, or export:
    ///    let _ = std::fs::write("stl/sdf_sphere.stl", mesh_shape.to_stl_binary("sdf_sphere").unwrap());
    pub fn sdf<F>(
        sdf: F,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
        metadata: M,
    ) -> Mesh<M>
    where
        // F is a closure or function that takes a 3D point and returns the signed distance.
        // Must be `Sync`/`Send` if you want to parallelize the sampling.
        F: Fn(&Point3<Real>) -> Real + Sync + Send,
    {
        Self::sdf_with_diagnostics(sdf, resolution, min_pt, max_pt, iso_value, metadata).0
    }

    /// Return a Mesh created by meshing a signed distance field and diagnostics
    /// describing the sampled field and generated triangle stream.
    ///
    /// Surface-net vertices still arrive from `fast_surface_nets` as primitive
    /// floats, but bounding inputs, iso values, triangle coordinates, normals,
    /// and degenerate-triangle classification are lifted through
    /// `hyperlattice`/`hyperreal` before topology diagnostics are recorded.
    /// This preserves csgrs as a CAD composition layer over hyper geometry
    /// types and follows Yap's exact-geometric-computation boundary model
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The sampled contouring
    /// method is Gibson's constrained elastic surface-net construction:
    /// Gibson, "Constrained Elastic Surface Nets," MERL TR99-24, 1999.
    pub fn sdf_with_diagnostics<F>(
        sdf: F,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
        metadata: M,
    ) -> (Mesh<M>, SdfDiagnostics)
    where
        F: Fn(&Point3<Real>) -> Real + Sync + Send,
    {
        // Early return if resolution is degenerate
        let nx = resolution.0.max(2) as u32;
        let ny = resolution.1.max(2) as u32;
        let nz = resolution.2.max(2) as u32;
        let mut diagnostics = SdfDiagnostics {
            resolution: (nx, ny, nz),
            sample_count: (nx * ny * nz) as usize,
            ..SdfDiagnostics::default()
        };

        if hvector3_from_point3(&min_pt).is_none()
            || hvector3_from_point3(&max_pt).is_none()
            || hreal_from_f64(iso_value).is_err()
        {
            diagnostics.non_finite_sample_count = diagnostics.sample_count;
            diagnostics.positive_sample_count = diagnostics.sample_count;
            return (Mesh::empty(metadata), diagnostics);
        }

        // Determine grid spacing based on bounding box and resolution
        let dx = (max_pt.x - min_pt.x) / (nx as Real - 1.0);
        let dy = (max_pt.y - min_pt.y) / (ny as Real - 1.0);
        let dz = (max_pt.z - min_pt.z) / (nz as Real - 1.0);

        // Allocate storage for field values:
        let array_size = (nx * ny * nz) as usize;
        let mut field_values = vec![0.0_f32; array_size];

        // Sample the SDF at each grid cell with optimized iteration pattern:
        // **Mathematical Foundation**: For SDF f(p), we sample at regular intervals
        // and store (f(p) - iso_value) so surface_nets finds zero-crossings at iso_value.
        // **Optimization**: Linear memory access pattern with better cache locality.
        #[allow(clippy::unnecessary_cast)]
        for i in 0..(nx * ny * nz) {
            let iz = i / (nx * ny);
            let remainder = i % (nx * ny);
            let iy = remainder / nx;
            let ix = remainder % nx;

            let xf = min_pt.x + (ix as Real) * dx;
            let yf = min_pt.y + (iy as Real) * dy;
            let zf = min_pt.z + (iz as Real) * dz;

            let p = Point3::new(xf, yf, zf);
            let sdf_val = sdf(&p);

            // Robust finite value handling with mathematical correctness
            field_values[i as usize] = if hreal_from_f64(sdf_val).is_ok() {
                diagnostics.finite_sample_count += 1;
                diagnostics.min_finite_value = Some(
                    diagnostics
                        .min_finite_value
                        .map_or(sdf_val, |current| current.min(sdf_val)),
                );
                diagnostics.max_finite_value = Some(
                    diagnostics
                        .max_finite_value
                        .map_or(sdf_val, |current| current.max(sdf_val)),
                );
                let shifted = sdf_val - iso_value;
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
                // For infinite/NaN values, use large positive value to indicate "far outside"
                // This preserves the mathematical properties of the distance field
                1e10_f32
            };
        }

        // The shape describing our discrete grid for Surface Nets:
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
                let x = i % self.nx;
                let yz = i / self.nx;
                let y = yz % self.ny;
                let z = yz / self.ny;
                [x, y, z]
            }
        }

        let shape = GridShape { nx, ny, nz };
        diagnostics.crossing_cell_count = count_crossing_cells(&field_values, nx, ny, nz);

        // `SurfaceNetsBuffer` collects the positions, normals, and triangle indices
        let mut sn_buffer = SurfaceNetsBuffer::default();

        // The max valid coordinate in each dimension
        let max_x = nx - 1;
        let max_y = ny - 1;
        let max_z = nz - 1;

        // Run surface nets
        surface_nets(
            &field_values,
            &shape,
            [0, 0, 0],
            [max_x, max_y, max_z],
            &mut sn_buffer,
        );
        diagnostics.surface_nets_vertex_count = sn_buffer.positions.len();
        diagnostics.surface_nets_index_count = sn_buffer.indices.len();

        // Convert the resulting triangles into Mesh polygons
        let mut triangles = Vec::with_capacity(sn_buffer.indices.len() / 3);

        for tri in sn_buffer.indices.chunks_exact(3) {
            let i0 = tri[0] as usize;
            let i1 = tri[1] as usize;
            let i2 = tri[2] as usize;

            let p0i = sn_buffer.positions[i0];
            let p1i = sn_buffer.positions[i1];
            let p2i = sn_buffer.positions[i2];

            // Convert from [u32; 3] to real coordinates:
            let p0 = Point3::new(
                min_pt.x + p0i[0] as Real * dx,
                min_pt.y + p0i[1] as Real * dy,
                min_pt.z + p0i[2] as Real * dz,
            );
            let p1 = Point3::new(
                min_pt.x + p1i[0] as Real * dx,
                min_pt.y + p1i[1] as Real * dy,
                min_pt.z + p1i[2] as Real * dz,
            );
            let p2 = Point3::new(
                min_pt.x + p2i[0] as Real * dx,
                min_pt.y + p2i[1] as Real * dy,
                min_pt.z + p2i[2] as Real * dz,
            );

            // Retrieve precomputed normal from Surface Nets:
            let n0 = sn_buffer.normals[i0];
            let n1 = sn_buffer.normals[i1];
            let n2 = sn_buffer.normals[i2];

            // Normals come out as [f32;3] – promote to `Real`
            let n0v = Vector3::new(n0[0] as Real, n0[1] as Real, n0[2] as Real);
            let n1v = Vector3::new(n1[0] as Real, n1[1] as Real, n1[2] as Real);
            let n2v = Vector3::new(n2[0] as Real, n2[1] as Real, n2[2] as Real);

            if !(finite_point3(&p0)
                && finite_point3(&p1)
                && finite_point3(&p2)
                && finite_vector3(&n0v)
                && finite_vector3(&n1v)
                && finite_vector3(&n2v))
            {
                // at least one coordinate was NaN/±∞ – ignore this triangle
                diagnostics.skipped_non_finite_triangle_count += 1;
                continue;
            }

            if !htriangle_area2_exceeds_epsilon(&p0, &p1, &p2, Real::EPSILON) {
                diagnostics.degenerate_triangle_count += 1;
            }

            let v0 =
                Vertex::new(p0, Vector3::new(n0[0] as Real, n0[1] as Real, n0[2] as Real));
            let v1 =
                Vertex::new(p1, Vector3::new(n1[0] as Real, n1[1] as Real, n1[2] as Real));
            let v2 =
                Vertex::new(p2, Vector3::new(n2[0] as Real, n2[1] as Real, n2[2] as Real));

            // Note: reverse v1, v2 if you need to fix winding
            let poly = Polygon::new(vec![v0, v1, v2], metadata.clone());
            triangles.push(poly);
            diagnostics.emitted_triangle_count += 1;
        }

        // Return as a Mesh
        (Mesh::from_polygons(&triangles, metadata), diagnostics)
    }
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
