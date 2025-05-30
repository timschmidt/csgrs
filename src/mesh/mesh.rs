use crate::float_types::Real;
use crate::float_types::parry3d::bounding_volume::{Aabb, BoundingVolume};
use crate::mesh::bsp::Node;
use crate::mesh::plane::Plane;
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use crate::traits::CSGOps;
use geo::{Coord, CoordsIter, Geometry, LineString, Polygon as GeoPolygon};
use nalgebra::{Matrix4, Point3, Vector3, partial_max, partial_min};
use std::fmt::Debug;
use std::sync::OnceLock;

#[derive(Clone, Debug)]
pub struct Mesh<S: Clone + Send + Sync + Debug> {
    /// 3D polygons for volumetric shapes
    pub polygons: Vec<Polygon<S>>,

    /// Lazily calculated AABB that spans `polygons`.
    pub bounding_box: OnceLock<Aabb>,

    /// Metadata
    pub metadata: Option<S>,
}

impl<S: Clone + Send + Sync + Debug> Mesh<S> {
    /// Split polygons into (may_touch, cannot_touch) using bounding‑box tests
    fn partition_polys(
        polys: &[Polygon<S>],
        other_bb: &Aabb,
    ) -> (Vec<Polygon<S>>, Vec<Polygon<S>>) {
        let mut maybe = Vec::new();
        let mut never = Vec::new();
        for p in polys {
            if p.bounding_box().intersects(other_bb) {
                maybe.push(p.clone());
            } else {
                never.push(p.clone());
            }
        }
        (maybe, never)
    }
    
	/// Helper to collect all vertices from the CSG.
    #[cfg(not(feature = "parallel"))]
    pub fn vertices(&self) -> Vec<Vertex> {
        self.polygons
            .iter()
            .flat_map(|p| p.vertices.clone())
            .collect()
    }

    /// Parallel helper to collect all vertices from the CSG.
    #[cfg(feature = "parallel")]
    pub fn vertices(&self) -> Vec<Vertex> {
        self.polygons
            .par_iter()
            .flat_map(|p| p.vertices.clone())
            .collect()
    }
    
    /// Rotate polygons into 2D to perform triangulation, then rotate triangles back to original 3D position
    pub fn triangulate_2d(outer: &[[Real; 2]], holes: &[&[[Real; 2]]]) -> Vec<[Point3<Real>; 3]> {
        // Convert the outer ring into a `LineString`
        let outer_coords: Vec<Coord<Real>> = outer.iter().map(|&[x, y]| Coord { x, y }).collect();

        // Convert each hole into its own `LineString`
        let holes_coords: Vec<LineString<Real>> = holes
            .iter()
            .map(|hole| {
                let coords: Vec<Coord<Real>> = hole.iter().map(|&[x, y]| Coord { x, y }).collect();
                LineString::new(coords)
            })
            .collect();

        // Ear-cut triangulation on the polygon (outer + holes)
        let polygon = GeoPolygon::new(LineString::new(outer_coords), holes_coords);

        #[cfg(feature = "earcut")]
        {
            use geo::TriangulateEarcut;
            let triangulation = polygon.earcut_triangles_raw();
            let triangle_indices = triangulation.triangle_indices;
            let vertices = triangulation.vertices;

            // Convert the 2D result (x,y) into 3D triangles with z=0
            let mut result = Vec::with_capacity(triangle_indices.len() / 3);
            for tri in triangle_indices.chunks_exact(3) {
                let pts = [
                    Point3::new(vertices[2 * tri[0]], vertices[2 * tri[0] + 1], 0.0),
                    Point3::new(vertices[2 * tri[1]], vertices[2 * tri[1] + 1], 0.0),
                    Point3::new(vertices[2 * tri[2]], vertices[2 * tri[2] + 1], 0.0),
                ];
                result.push(pts);
            }
            result
        }

        #[cfg(feature = "delaunay")]
        {
            use geo::TriangulateSpade;
            // We want polygons with holes => constrained triangulation.
            // For safety, handle the Result the trait returns:
            let Ok(tris) = polygon.constrained_triangulation(Default::default()) else {
                // If a triangulation error is a possibility,
                // pick the error-handling you want here:
                return Vec::new();
            };

            let mut result = Vec::with_capacity(tris.len());
            for triangle in tris {
                // Each `triangle` is a geo_types::Triangle whose `.0, .1, .2`
                // are the 2D coordinates. We'll embed them at z=0.
                let [a, b, c] = [triangle.0, triangle.1, triangle.2];
                result.push([
                    Point3::new(a.x, a.y, 0.0),
                    Point3::new(b.x, b.y, 0.0),
                    Point3::new(c.x, c.y, 0.0),
                ]);
            }
            result
        }
    }
}

impl<S: Clone + Send + Sync + Debug> CSGOps for Mesh<S> {
    /// Returns a new empty Mesh
    fn new() -> Self {
        Mesh {
            polygons: Vec::new(),
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }

    /// Return a new Mesh representing union of the two Meshes.
    ///
    /// ```no_run
    /// let c = a.union(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |       +----+
    ///     +----+--+    |       +----+       |
    ///          |   b   |            |   c   |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    fn union(&self, other: &Mesh<S>) -> Mesh<S> {
        // avoid splitting obvious non‑intersecting faces
        let (a_clip, a_passthru) = Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, b_passthru) = Self::partition_polys(&other.polygons, &self.bounding_box());

        let mut a = Node::new(&a_clip);
        let mut b = Node::new(&b_clip);

        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());

        // combine results and untouched faces
        let mut final_polys = a.all_polygons();
        final_polys.extend(a_passthru);
        final_polys.extend(b_passthru);

        Mesh {
            polygons: final_polys,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new Mesh representing diffarence of the two Meshes.
    ///
    /// ```no_run
    /// let c = a.difference(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   c   |
    ///     |    +--+----+   =   |    +--+
    ///     +----+--+    |       +----+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    fn difference(&self, other: &Mesh<S>) -> Mesh<S> {
        // avoid splitting obvious non‑intersecting faces
        let (a_clip, a_passthru) = Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, _b_passthru) = Self::partition_polys(&other.polygons, &self.bounding_box());

        let mut a = Node::new(&a_clip);
        let mut b = Node::new(&b_clip);

        a.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        b.invert();
        b.clip_to(&a);
        b.invert();
        a.build(&b.all_polygons());
        a.invert();

        // combine results and untouched faces
        let mut final_polys = a.all_polygons();
        final_polys.extend(a_passthru);

        Mesh {
            polygons: final_polys,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new CSG representing intersection of the two CSG's.
    ///
    /// ```no_run
    /// let c = a.intersect(b);
    ///     +-------+
    ///     |       |
    ///     |   a   |
    ///     |    +--+----+   =   +--+
    ///     +----+--+    |       +--+
    ///          |   b   |
    ///          |       |
    ///          +-------+
    /// ```
    fn intersection(&self, other: &Mesh<S>) -> Mesh<S> {
        // avoid splitting obvious non‑intersecting faces
        let (a_clip, _a_passthru) = Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, _b_passthru) = Self::partition_polys(&other.polygons, &self.bounding_box());

        let mut a = Node::new(&a_clip);
        let mut b = Node::new(&b_clip);

        a.invert();
        b.clip_to(&a);
        b.invert();
        a.clip_to(&b);
        b.clip_to(&a);
        a.build(&b.all_polygons());
        a.invert();

        Mesh {
            polygons: a.all_polygons(),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// Return a new CSG representing space in this CSG excluding the space in the
    /// other CSG plus the space in the other CSG excluding the space in this CSG.
    ///
    /// ```no_run
    /// let c = a.xor(b);
    ///     +-------+            +-------+
    ///     |       |            |       |
    ///     |   a   |            |   a   |
    ///     |    +--+----+   =   |    +--+----+
    ///     +----+--+    |       +----+--+    |
    ///          |   b   |            |       |
    ///          |       |            |       |
    ///          +-------+            +-------+
    /// ```
    fn xor(&self, other: &Mesh<S>) -> Mesh<S> {
        // 3D and 2D xor:
        // A \ B
        let a_sub_b = self.difference(other);

        // B \ A
        let b_sub_a = other.difference(self);

        // Union those two
        a_sub_b.union(&b_sub_a)
    }

    /// Apply an arbitrary 3D transform (as a 4x4 matrix) to the mesh.
    fn transform(&self, mat: &Matrix4<Real>) -> Mesh<S> {
        let mat_inv_transpose = mat
            .try_inverse()
            .expect("Matrix not invertible?")
            .transpose(); // todo catch error
        let mut mesh = self.clone();

        for poly in &mut mesh.polygons {
            for vert in &mut poly.vertices {
                // Position
                let hom_pos = mat * vert.pos.to_homogeneous();
                vert.pos = Point3::from_homogeneous(hom_pos).unwrap(); // todo catch error

                // Normal
                vert.normal = mat_inv_transpose.transform_vector(&vert.normal).normalize();
            }

            // keep the cached plane consistent with the new vertex positions
            poly.plane = Plane::from_vertices(poly.vertices.clone());
        }

        // invalidate the old cached bounding box
        mesh.bounding_box = OnceLock::new();

        mesh
    }

    /// Returns a [`parry3d::bounding_volume::Aabb`] indicating the 3D bounds of all `polygons`.
    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            // Track overall min/max in x, y, z among all 3D polygons
            let mut min_x = Real::MAX;
            let mut min_y = Real::MAX;
            let mut min_z = Real::MAX;
            let mut max_x = -Real::MAX;
            let mut max_y = -Real::MAX;
            let mut max_z = -Real::MAX;

            // 1) Gather from the 3D polygons
            for poly in &self.polygons {
                for v in &poly.vertices {
                    min_x = *partial_min(&min_x, &v.pos.x).unwrap();
                    min_y = *partial_min(&min_y, &v.pos.y).unwrap();
                    min_z = *partial_min(&min_z, &v.pos.z).unwrap();

                    max_x = *partial_max(&max_x, &v.pos.x).unwrap();
                    max_y = *partial_max(&max_y, &v.pos.y).unwrap();
                    max_z = *partial_max(&max_z, &v.pos.z).unwrap();
                }
            }

            // If still uninitialized (e.g., no polygons), return a trivial AABB at origin
            if min_x > max_x {
                return Aabb::new(Point3::origin(), Point3::origin());
            }

            // Build a parry3d Aabb from these min/max corners
            let mins = Point3::new(min_x, min_y, min_z);
            let maxs = Point3::new(max_x, max_y, max_z);
            Aabb::new(mins, maxs)
        })
    }

    /// Invalidates object's cached bounding box.
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }

    /// Invert this Mesh (flip inside vs. outside)
    fn inverse(&self) -> Mesh<S> {
        let mut mesh = self.clone();
        for p in &mut mesh.polygons {
            p.flip();
        }
        mesh
    }
}

impl<S: Clone + Send + Sync + Debug> From<crate::sketch::sketch::Sketch<S>> for Mesh<S> {
    /// Convert a Sketch into a Mesh.
    fn from(sketch: crate::sketch::sketch::Sketch<S>) -> Self {
        /// Helper function to convert a geo::Polygon into one or more Polygon<S> entries.
        fn process_polygon<S>(
            poly2d: &geo::Polygon<Real>,
            all_polygons: &mut Vec<Polygon<S>>,
            metadata: &Option<S>,
        ) where
            S: Clone + Send + Sync,
        {
            // 1. Convert the outer ring to 3D.
            let mut outer_vertices_3d = Vec::new();
            for c in poly2d.exterior().coords_iter() {
                outer_vertices_3d.push(Vertex::new(Point3::new(c.x, c.y, 0.0), Vector3::z()));
            }

            if outer_vertices_3d.len() >= 3 {
                all_polygons.push(Polygon::new(outer_vertices_3d, metadata.clone()));
            }

            // 2. Convert interior rings (holes), if needed as separate polygons.
            for ring in poly2d.interiors() {
                let mut hole_vertices_3d = Vec::new();
                for c in ring.coords_iter() {
                    hole_vertices_3d.push(Vertex::new(Point3::new(c.x, c.y, 0.0), Vector3::z()));
                }

                if hole_vertices_3d.len() >= 3 {
                    // Note: adjust this if your `Polygon<S>` type supports interior rings.
                    all_polygons.push(Polygon::new(hole_vertices_3d, metadata.clone()));
                }
            }
        }

        let mut all_polygons = Vec::new();

        for geom in &sketch.geometry {
            match geom {
                Geometry::Polygon(poly2d) => {
                    process_polygon(poly2d, &mut all_polygons, &sketch.metadata);
                }
                Geometry::MultiPolygon(multipoly) => {
                    for poly2d in multipoly {
                        process_polygon(poly2d, &mut all_polygons, &sketch.metadata);
                    }
                }
                // Optional: handle other geometry types like LineString here.
                _ => {}
            }
        }

        Mesh {
            polygons: all_polygons,
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }
}
