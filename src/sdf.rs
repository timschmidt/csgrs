use crate::csg::CSG;
use crate::float_types::Real;
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use fast_surface_nets::{SurfaceNetsBuffer, surface_nets};
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

impl<S: Clone + Debug> CSG<S>
where S: Clone + Send + Sync {
    /// Return a CSG created by meshing a signed distance field within a bounding box
    ///
    ///    // Example SDF for a sphere of radius 1.5 centered at (0,0,0)
    ///    let my_sdf = |p: &Point3<Real>| p.coords.norm() - 1.5;
    ///
    ///    let resolution = (60, 60, 60);
    ///    let min_pt = Point3::new(-2.0, -2.0, -2.0);
    ///    let max_pt = Point3::new( 2.0,  2.0,  2.0);
    ///    let iso_value = 0.0; // Typically zero for SDF-based surfaces
    ///
    ///    let csg_shape = CSG::from_sdf(my_sdf, resolution, min_pt, max_pt, iso_value);
    ///
    ///    // Now `csg_shape` is your polygon mesh as a CSG you can union, subtract, or export:
    ///    let _ = std::fs::write("stl/sdf_sphere.stl", csg_shape.to_stl_binary("sdf_sphere").unwrap());
    pub fn sdf<F>(
        sdf: F,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
        metadata: Option<S>,
    ) -> CSG<S>
    where
        // F is a closure or function that takes a 3D point and returns the signed distance.
        // Must be `Sync`/`Send` if you want to parallelize the sampling.
        F: Fn(&Point3<Real>) -> Real + Sync + Send,
    {
        // Early return if resolution is degenerate
        let nx = resolution.0.max(2) as u32;
        let ny = resolution.1.max(2) as u32;
        let nz = resolution.2.max(2) as u32;

        // Determine grid spacing based on bounding box and resolution
        let dx = (max_pt.x - min_pt.x) / (nx as Real - 1.0);
        let dy = (max_pt.y - min_pt.y) / (ny as Real - 1.0);
        let dz = (max_pt.z - min_pt.z) / (nz as Real - 1.0);

        // Allocate storage for field values:
        let array_size = (nx * ny * nz) as usize;
        let mut field_values = vec![0.0_f32; array_size];

        // Helper to map (ix, iy, iz) to 1D index:
        let index_3d = |ix: u32, iy: u32, iz: u32| -> usize {
            (iz * ny + iy) as usize * (nx as usize) + ix as usize
        };

        // Sample the SDF at each grid cell:
        // Note that for an "isosurface" at iso_value, we store (sdf_value - iso_value)
        // so that `surface_nets` zero-crossing aligns with iso_value.
        for iz in 0..nz {
            let zf = min_pt.z + (iz as Real) * dz;
            for iy in 0..ny {
                let yf = min_pt.y + (iy as Real) * dy;
                for ix in 0..nx {
                    let xf = min_pt.x + (ix as Real) * dx;
                    let p = Point3::new(xf, yf, zf);
                    let sdf_val = sdf(&p);
                    // Shift by iso_value so that the zero-level is the surface we want:
                    field_values[index_3d(ix, iy, iz)] = (sdf_val - iso_value) as f32;
                }
            }
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

        // Convert the resulting triangles into CSG polygons
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

            let v0 = Vertex::new(
                p0,
                Vector3::new(n0[0] as Real, n0[1] as Real, n0[2] as Real),
            );
            let v1 = Vertex::new(
                p1,
                Vector3::new(n1[0] as Real, n1[1] as Real, n1[2] as Real),
            );
            let v2 = Vertex::new(
                p2,
                Vector3::new(n2[0] as Real, n2[1] as Real, n2[2] as Real),
            );

            // Note: reverse v1, v2 if you need to fix winding
            let poly = Polygon::new(vec![v0, v1, v2], metadata.clone());
            triangles.push(poly);
        }

        // Return as a CSG
        CSG::from_polygons(&triangles)
    }
}
