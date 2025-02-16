use crate::float_types::{EPSILON, Real};
use crate::csg::CSG;
use crate::polygon::Polygon;
use crate::vertex::Vertex;

use nalgebra::{Point3, Vector3};

/// Import fast-surface-nets
use fast_surface_nets::{surface_nets, SurfaceNetsBuffer};

#[derive(Debug, Clone)]
pub struct MetaBall {
    pub center: Point3<Real>,
    pub radius: Real,
}

impl MetaBall {
    pub fn new(center: Point3<Real>, radius: Real) -> Self {
        Self { center, radius }
    }

    /// “Influence” function used by the scalar field for metaballs
    pub fn influence(&self, p: &Point3<Real>) -> Real {
        let dist_sq = (p - self.center).norm_squared() + EPSILON;
        self.radius * self.radius / dist_sq
    }
}

/// Summation of influences from multiple metaballs.
fn scalar_field_metaballs(balls: &[MetaBall], p: &Point3<Real>) -> Real {
    let mut value = 0.0;
    for ball in balls {
        value += ball.influence(p);
    }
    value
}

/// Build a CSG by extracting the 0-isosurface of the metaball field using fast-surface-nets.
pub fn metaballs_to_csg<S: Clone + Send + Sync>(
    balls: &[MetaBall],
    resolution: (usize, usize, usize),
    iso_value: Real,
    padding: Real,
) -> CSG<S> {
    if balls.is_empty() {
        return CSG::new();
    }

    // 1) Determine bounding box of all metaballs (plus padding).
    let mut min_pt = Point3::new(Real::MAX, Real::MAX, Real::MAX);
    let mut max_pt = Point3::new(-Real::MAX, -Real::MAX, -Real::MAX);

    for mb in balls {
        let c = &mb.center;
        let r = mb.radius + padding;

        if c.x - r < min_pt.x {
            min_pt.x = c.x - r;
        }
        if c.y - r < min_pt.y {
            min_pt.y = c.y - r;
        }
        if c.z - r < min_pt.z {
            min_pt.z = c.z - r;
        }

        if c.x + r > max_pt.x {
            max_pt.x = c.x + r;
        }
        if c.y + r > max_pt.y {
            max_pt.y = c.y + r;
        }
        if c.z + r > max_pt.z {
            max_pt.z = c.z + r;
        }
    }

    // 2) Resolution for X, Y, Z
    let nx = resolution.0.max(2) as u32;
    let ny = resolution.1.max(2) as u32;
    let nz = resolution.2.max(2) as u32;

    // Spacing in each axis
    let dx = (max_pt.x - min_pt.x) / (nx as Real - 1.0);
    let dy = (max_pt.y - min_pt.y) / (ny as Real - 1.0);
    let dz = (max_pt.z - min_pt.z) / (nz as Real - 1.0);

    // 3) Create and fill the scalar-field array with "field_value - iso_value"
    //    so that the isosurface will be at 0.
    let array_size = (nx * ny * nz) as usize;
    let mut field_values = vec![0.0 as f32; array_size];

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

    // 4) Use fast-surface-nets to extract a mesh from this 3D scalar field.
    //    We'll define a shape type for ndshape:
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
        &field_values,      // SDF array
        &shape,             // custom shape
        [0, 0, 0],          // minimum corner in lattice coords
        [max_x, max_y, max_z],
        &mut sn_buffer,
    );

    // 5) Convert the resulting surface net indices/positions into Polygons
    //    for the csgrs data structures.
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
            min_pt.z + p0_index[2] as Real * dz
        );
        
        let p1_real = Point3::new(
            min_pt.x + p1_index[0] as Real * dx,
            min_pt.y + p1_index[1] as Real * dy,
            min_pt.z + p1_index[2] as Real * dz
        );
        
        let p2_real = Point3::new(
            min_pt.x + p2_index[0] as Real * dx,
            min_pt.y + p2_index[1] as Real * dy,
            min_pt.z + p2_index[2] as Real * dz
        );
        
        // Likewise for the normals if you want them in true world space. 
        // Usually you'd need to do an inverse-transpose transform if your 
        // scale is non-uniform. For uniform voxels, scaling is simpler:
        
        let n0 = sn_buffer.normals[i0];
        let n1 = sn_buffer.normals[i1];
        let n2 = sn_buffer.normals[i2];
        
        // Construct your vertices:
        let v0 = Vertex::new(p0_real, Vector3::new(n0[0] as Real, n0[1] as Real, n0[2] as Real));
        let v1 = Vertex::new(p1_real, Vector3::new(n1[0] as Real, n1[1] as Real, n1[2] as Real));
        let v2 = Vertex::new(p2_real, Vector3::new(n2[0] as Real, n2[1] as Real, n2[2] as Real));

        // Each tri is turned into a Polygon with 3 vertices
        let poly = Polygon::new(vec![v0, v2, v1], false, None);
        triangles.push(poly);
    }

    // 6) Build and return a CSG from these polygons
    CSG::from_polygons(&triangles)
}

