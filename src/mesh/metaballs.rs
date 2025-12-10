//! Provides a `MetaBall` struct and functions for creating a `Mesh` from [MetaBalls](https://en.wikipedia.org/wiki/Metaballs)

use crate::float_types::{Real, tolerance};
use crate::mesh::Mesh;
use crate::mesh::polygon::Polygon;
use crate::vertex::Vertex;
use crate::csg::CSG;
use fast_surface_nets::{SurfaceNetsBuffer, surface_nets};
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

#[derive(Debug, Clone)]
pub struct MetaBall {
    pub center: Point3<Real>,
    pub radius: Real,
}

impl MetaBall {
    pub const fn new(center: Point3<Real>, radius: Real) -> Self {
        Self { center, radius }
    }

    /// **Mathematical Foundation**: Metaball influence function I(p) = r²/(|p-c|² + ε)
    /// where ε prevents division by zero and maintains numerical stability.
    /// **Optimization**: Early termination for distant points and vectorized computation.
    pub fn influence(&self, p: &Point3<Real>) -> Real {
        let distance_squared = (p - self.center).norm_squared();

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

/// **Mathematical Foundation**: Scalar field F(p) = Σ I_i(p) where I_i is the influence
/// function of the i-th metaball. This creates smooth isosurfaces at threshold values.
/// **Optimization**: Iterator-based summation with potential for vectorization.
fn scalar_field_metaballs(balls: &[MetaBall], p: &Point3<Real>) -> Real {
    balls.iter().map(|ball| ball.influence(p)).sum()
}

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
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
        metadata: Option<S>,
    ) -> Mesh<S> {
        if balls.is_empty() {
            return Mesh::new();
        }

        // Determine bounding box of all metaballs (plus padding).
        let (min_pt, max_pt) = balls.iter().fold(
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

        // Resolution for X, Y, Z
        let nx = resolution.0.max(2) as u32;
        let ny = resolution.1.max(2) as u32;
        let nz = resolution.2.max(2) as u32;

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

                    let val = scalar_field_metaballs(balls, &p) - iso_value;
                    field_values[index_3d(ix, iy, iz)] = val as f32;
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

            // Construct your vertices:
            let v0 = Vertex::new(
                p0_real,
                Vector3::new(n0[0] as Real, n0[1] as Real, n0[2] as Real),
            );
            let v1 = Vertex::new(
                p1_real,
                Vector3::new(n1[0] as Real, n1[1] as Real, n1[2] as Real),
            );
            let v2 = Vertex::new(
                p2_real,
                Vector3::new(n2[0] as Real, n2[1] as Real, n2[2] as Real),
            );

            // Each tri is turned into a Polygon with 3 vertices
            let poly = Polygon::new(vec![v0, v2, v1], metadata.clone());
            triangles.push(poly);
        }

        // Build and return a Mesh from these polygons
        Mesh::from_polygons(&triangles, metadata)
    }
}
