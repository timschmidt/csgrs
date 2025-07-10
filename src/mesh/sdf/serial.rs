//! Serial implementation of SDF meshing

use crate::float_types::Real;
use crate::mesh::Mesh;
use crate::mesh::polygon::Polygon;
<<<<<<< HEAD
use crate::mesh::sdf::grid::GridShape;
use crate::mesh::sdf::traits::SdfOps;
use crate::mesh::vertex::Vertex;
=======
use crate::mesh::vertex::Vertex;
use crate::mesh::sdf::traits::SdfOps;
use crate::mesh::sdf::grid::GridShape;
>>>>>>> fff8770013ce723baabeaab7bc5c693c2d64bce5
use fast_surface_nets::{SurfaceNetsBuffer, surface_nets};
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

/// Serial implementation of SDF meshing
pub struct SerialSdfOps;

impl SerialSdfOps {
<<<<<<< HEAD
    pub const fn new() -> Self {
=======
    pub fn new() -> Self {
>>>>>>> fff8770013ce723baabeaab7bc5c693c2d64bce5
        Self
    }
}

impl Default for SerialSdfOps {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Clone + Debug + Send + Sync> SdfOps<S> for SerialSdfOps {
    fn mesh<F>(
        &self,
        sdf: F,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
        metadata: Option<S>,
    ) -> Mesh<S>
    where
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

        // Optimized finite value checking with iterator patterns
        #[inline]
        fn point_finite(p: &Point3<Real>) -> bool {
            p.coords.iter().all(|&c| c.is_finite())
        }

        #[inline]
        fn vec_finite(v: &Vector3<Real>) -> bool {
            v.iter().all(|&c| c.is_finite())
        }

        // Sample the SDF at each grid cell
        field_values.iter_mut().enumerate().for_each(|(i, value)| {
            let iz = i / (nx * ny) as usize;
            let remainder = i % (nx * ny) as usize;
            let iy = remainder / nx as usize;
            let ix = remainder % nx as usize;

            let xf = min_pt.x + (ix as Real) * dx;
            let yf = min_pt.y + (iy as Real) * dy;
            let zf = min_pt.z + (iz as Real) * dz;

            let p = Point3::new(xf, yf, zf);
            let sdf_val = sdf(&p);

            *value = if sdf_val.is_finite() {
                (sdf_val - iso_value) as f32
            } else {
                1e10_f32
            };
        });

        let shape = GridShape { nx, ny, nz };
        let mut sn_buffer = SurfaceNetsBuffer::default();
        let max_x = nx - 1;
        let max_y = ny - 1;
        let max_z = nz - 1;

        surface_nets(
            &field_values,
            &shape,
            [0, 0, 0],
            [max_x, max_y, max_z],
            &mut sn_buffer,
        );

<<<<<<< HEAD
        let triangles: Vec<Polygon<S>> = sn_buffer
            .indices
            .chunks_exact(3)
            .filter_map(|tri| {
                let i0 = tri[0] as usize;
                let i1 = tri[1] as usize;
                let i2 = tri[2] as usize;

                let p0i = sn_buffer.positions[i0];
                let p1i = sn_buffer.positions[i1];
                let p2i = sn_buffer.positions[i2];

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

                let n0 = sn_buffer.normals[i0];
                let n1 = sn_buffer.normals[i1];
                let n2 = sn_buffer.normals[i2];

                let n0v = Vector3::new(n0[0] as Real, n0[1] as Real, n0[2] as Real);
                let n1v = Vector3::new(n1[0] as Real, n1[1] as Real, n1[2] as Real);
                let n2v = Vector3::new(n2[0] as Real, n2[1] as Real, n2[2] as Real);

                if !(point_finite(&p0)
                    && point_finite(&p1)
                    && point_finite(&p2)
                    && vec_finite(&n0v)
                    && vec_finite(&n1v)
                    && vec_finite(&n2v))
                {
                    return None;
                }

                let v0 = Vertex::new(p0, n0v);
                let v1 = Vertex::new(p1, n1v);
                let v2 = Vertex::new(p2, n2v);

                Some(Polygon::new(vec![v0, v1, v2], metadata.clone()))
            })
            .collect();
=======
        let triangles: Vec<Polygon<S>> = sn_buffer.indices.chunks_exact(3).filter_map(|tri| {
            let i0 = tri[0] as usize;
            let i1 = tri[1] as usize;
            let i2 = tri[2] as usize;

            let p0i = sn_buffer.positions[i0];
            let p1i = sn_buffer.positions[i1];
            let p2i = sn_buffer.positions[i2];

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

            let n0 = sn_buffer.normals[i0];
            let n1 = sn_buffer.normals[i1];
            let n2 = sn_buffer.normals[i2];

            let n0v = Vector3::new(n0[0] as Real, n0[1] as Real, n0[2] as Real);
            let n1v = Vector3::new(n1[0] as Real, n1[1] as Real, n1[2] as Real);
            let n2v = Vector3::new(n2[0] as Real, n2[1] as Real, n2[2] as Real);

            if !(point_finite(&p0)
                && point_finite(&p1)
                && point_finite(&p2)
                && vec_finite(&n0v)
                && vec_finite(&n1v)
                && vec_finite(&n2v))
            {
                return None;
            }

            let v0 = Vertex::new(p0, n0v);
            let v1 = Vertex::new(p1, n1v);
            let v2 = Vertex::new(p2, n2v);

            Some(Polygon::new(vec![v0, v1, v2], metadata.clone()))
        }).collect();
>>>>>>> fff8770013ce723baabeaab7bc5c693c2d64bce5

        Mesh::from_polygons(&triangles, metadata)
    }
}
