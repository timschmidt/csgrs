use crate::csg::CSG;
use crate::float_types::{EPSILON, Real};
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use fast_surface_nets::{SurfaceNetsBuffer, surface_nets};
use geo::{CoordsIter, Geometry, GeometryCollection, LineString, Polygon as GeoPolygon, Coord, coord};
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;
use hashbrown::HashMap;

#[derive(Debug, Clone)]
pub struct MetaBall {
    pub center: Point3<Real>,
    pub radius: Real,
}

impl MetaBall {
    pub const fn new(center: Point3<Real>, radius: Real) -> Self {
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

// helper – quantise to avoid FP noise
#[inline]
fn key(x: Real, y: Real) -> (i64, i64) {
    ((x * 1e8).round() as i64, (y * 1e8).round() as i64)
}

/// stitch all 2-point segments into longer polylines,
/// close them when the ends meet
fn stitch(contours: &[LineString<Real>]) -> Vec<LineString<Real>> {
    // adjacency map  endpoint -> (line index, end-id 0|1)
    let mut adj: HashMap<(i64,i64), Vec<(usize, usize)>> = HashMap::new();
    for (idx, ls) in contours.iter().enumerate() {
        let p0 = ls[0];                       // first point
        let p1 = ls[1];                       // second point
        adj.entry(key(p0.x, p0.y)).or_default().push((idx, 0));
        adj.entry(key(p1.x, p1.y)).or_default().push((idx, 1));
    }

    let mut used = vec![false; contours.len()];
    let mut chains = Vec::new();

    for start in 0..contours.len() {
        if used[start] { continue; }
        used[start] = true;

        // current chain of points
        let mut chain = Vec::<Coord<Real>>::from(contours[start].0.clone());

        // walk forward
        loop {
            let last = *chain.last().unwrap();
            let Some(cands) = adj.get(&key(last.x, last.y)) else { break };
            let mut found = None;
            for &(idx,end_id) in cands {
                if used[idx] { continue; }
                used[idx] = true;
                // choose the *other* endpoint
                let other = contours[idx][1 - end_id];
                chain.push(other);
                found = Some(());
                break;
            }
            if found.is_none() { break; }
        }

        // close if ends coincide
        if chain.len() >= 3 && (chain[0] == *chain.last().unwrap()) == false {
            chain.push(chain[0]);
        }
        chains.push(LineString::new(chain));
    }
    chains
}

impl<S: Clone + Debug> CSG<S>
where S: Clone + Send + Sync {
    /// Create a 2D metaball iso-contour in XY plane from a set of 2D metaballs.
    /// - `balls`: array of (center, radius).
    /// - `resolution`: (nx, ny) grid resolution for marching squares.
    /// - `iso_value`: threshold for the iso-surface.
    /// - `padding`: extra boundary beyond each ball's radius.
    /// - `metadata`: optional user metadata.
    pub fn metaballs2d(
        balls: &[(nalgebra::Point2<Real>, Real)],
        resolution: (usize, usize),
        iso_value: Real,
        padding: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        let (nx, ny) = resolution;
        if balls.is_empty() || nx < 2 || ny < 2 {
            return CSG::new();
        }

        // 1) Compute bounding box around all metaballs
        let mut min_x = Real::MAX;
        let mut min_y = Real::MAX;
        let mut max_x = -Real::MAX;
        let mut max_y = -Real::MAX;
        for (center, r) in balls {
            let rr = *r + padding;
            if center.x - rr < min_x { min_x = center.x - rr; }
            if center.x + rr > max_x { max_x = center.x + rr; }
            if center.y - rr < min_y { min_y = center.y - rr; }
            if center.y + rr > max_y { max_y = center.y + rr; }
        }

        let dx = (max_x - min_x) / (nx as Real - 1.0);
        let dy = (max_y - min_y) / (ny as Real - 1.0);

        // 2) Fill a grid with the summed “influence” minus iso_value
        fn scalar_field(balls: &[(nalgebra::Point2<Real>, Real)], x: Real, y: Real) -> Real {
            let mut v = 0.0;
            for (c, r) in balls {
                let dx = x - c.x;
                let dy = y - c.y;
                let dist_sq = dx * dx + dy * dy + EPSILON;
                v += (r * r) / dist_sq;
            }
            v
        }

        let mut grid = vec![0.0; nx * ny];
        let index = |ix: usize, iy: usize| -> usize { iy * nx + ix };
        for iy in 0..ny {
            let yv = min_y + (iy as Real) * dy;
            for ix in 0..nx {
                let xv = min_x + (ix as Real) * dx;
                let val = scalar_field(balls, xv, yv) - iso_value;
                grid[index(ix, iy)] = val;
            }
        }

        // 3) Marching squares -> line segments
        let mut contours = Vec::<LineString<Real>>::new();

        // Interpolator:
        let interpolate =
            |(x1, y1, v1): (Real, Real, Real), (x2, y2, v2): (Real, Real, Real)| -> (Real, Real) {
                let denom = (v2 - v1).abs();
                if denom < EPSILON {
                    (x1, y1)
                } else {
                    let t = -v1 / (v2 - v1); // crossing at 0
                    (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
                }
            };

        for iy in 0..(ny - 1) {
            let y0 = min_y + (iy as Real) * dy;
            let y1 = min_y + ((iy + 1) as Real) * dy;

            for ix in 0..(nx - 1) {
                let x0 = min_x + (ix as Real) * dx;
                let x1 = min_x + ((ix + 1) as Real) * dx;

                let v0 = grid[index(ix, iy)];
                let v1 = grid[index(ix + 1, iy)];
                let v2 = grid[index(ix + 1, iy + 1)];
                let v3 = grid[index(ix, iy + 1)];

                // classification
                let mut c = 0u8;
                if v0 >= 0.0 { c |= 1; }
                if v1 >= 0.0 { c |= 2; }
                if v2 >= 0.0 { c |= 4; }
                if v3 >= 0.0 { c |= 8; }
                if c == 0 || c == 15 {
                    continue; // no crossing
                }

                let corners = [
                    (x0, y0, v0),
                    (x1, y0, v1),
                    (x1, y1, v2),
                    (x0, y1, v3),
                ];

                let mut pts = Vec::new();
                // function to check each edge
                let mut check_edge = |mask_a: u8, mask_b: u8, a: usize, b: usize| {
                    let inside_a = (c & mask_a) != 0;
                    let inside_b = (c & mask_b) != 0;
                    if inside_a != inside_b {
                        let (px, py) = interpolate(corners[a], corners[b]);
                        pts.push((px, py));
                    }
                };

                check_edge(1, 2, 0, 1);
                check_edge(2, 4, 1, 2);
                check_edge(4, 8, 2, 3);
                check_edge(8, 1, 3, 0);

                // we might get 2 intersection points => single line segment
                // or 4 => two line segments, etc.
                // For simplicity, we just store them in a small open polyline:
                if pts.len() >= 2 {
                    let mut pl = LineString::new(vec![]);
                    for &(px, py) in &pts {
                        pl.0.push(coord! {x: px, y: py});
                    }
                    // Do not close. These are just line segments from this cell.
                    contours.push(pl);
                }
            }
        }

        // 4) Convert these line segments into geo::LineStrings or geo::Polygons if closed.
        //    We store them in a GeometryCollection.
        let mut gc = GeometryCollection::default();

        let stitched = stitch(&contours);

        for pl in stitched {
            if pl.is_closed() && pl.coords_count() >= 4 {
                let polygon = GeoPolygon::new(pl, vec![]);
                gc.0.push(Geometry::Polygon(polygon));
            }
        }

        CSG::from_geo(gc, metadata)
    }

    /// **Creates a CSG from a list of metaballs** by sampling a 3D grid and using marching cubes.
    ///
    /// - `balls`: slice of metaball definitions (center + radius).
    /// - `resolution`: (nx, ny, nz) defines how many steps along x, y, z.
    /// - `iso_value`: threshold at which the isosurface is extracted.
    /// - `padding`: extra margin around the bounding region (e.g. 0.5) so the surface doesn’t get truncated.
    pub fn metaballs(
        balls: &[MetaBall],
        resolution: (usize, usize, usize),
        iso_value: Real,
        padding: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        if balls.is_empty() {
            return CSG::new();
        }

        // Determine bounding box of all metaballs (plus padding).
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
            let v0 = Vertex::new(p0_real, Vector3::new(n0[0] as Real, n0[1] as Real, n0[2] as Real));
            let v1 = Vertex::new(p1_real, Vector3::new(n1[0] as Real, n1[1] as Real, n1[2] as Real));
            let v2 = Vertex::new(p2_real, Vector3::new(n2[0] as Real, n2[1] as Real, n2[2] as Real));

            // Each tri is turned into a Polygon with 3 vertices
            let poly = Polygon::new(vec![v0, v2, v1], metadata.clone());
            triangles.push(poly);
        }

        // Build and return a CSG from these polygons
        CSG::from_polygons(&triangles)
    }
}
