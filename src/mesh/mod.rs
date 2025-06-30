//! `Mesh` struct and implementations of the `CSGOps` trait for `Mesh`

use crate::float_types::{
    parry3d::{
        bounding_volume::{Aabb, BoundingVolume},
        query::RayCast,
        shape::Shape,
    },
    rapier3d::prelude::{
        ColliderBuilder, ColliderSet, Ray, RigidBodyBuilder, RigidBodyHandle,
        RigidBodySet, SharedShape, TriMesh, Triangle,
    },
    {EPSILON, Real},
};
use crate::mesh::{
    bsp::Node,
    plane::Plane,
    polygon::Polygon,
    vertex::Vertex,
};
use crate::sketch::Sketch;
use crate::traits::CSGOps;
use geo::{Coord, CoordsIter, Geometry, LineString, Polygon as GeoPolygon};
use nalgebra::{
    Isometry3, Matrix4, Point3, Quaternion, Unit, Vector3, partial_max, partial_min,
};
use std::{fmt::Debug, num::NonZeroU32, sync::OnceLock};

#[cfg(feature = "parallel")]
use rayon::{prelude::*, iter::IntoParallelRefIterator};

pub mod bsp;
pub mod bsp_parallel;
pub mod convex_hull;
pub mod flatten_slice;
pub mod metaballs;
pub mod plane;
pub mod polygon;
pub mod sdf;
pub mod shapes;
pub mod tpms;
pub mod vertex;
pub mod quality;
pub mod manifold;
pub mod connectivity;
pub mod smoothing;

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
    /// Build a Mesh from an existing polygon list
    pub fn from_polygons(polygons: &[Polygon<S>]) -> Self {
        let mut mesh = Mesh::new();
        mesh.polygons = polygons.to_vec();
        mesh
    }

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
    pub fn triangulate_2d(
        outer: &[[Real; 2]],
        holes: &[&[[Real; 2]]],
    ) -> Vec<[Point3<Real>; 3]> {
        // Convert the outer ring into a `LineString`
        let outer_coords: Vec<Coord<Real>> =
            outer.iter().map(|&[x, y]| Coord { x, y }).collect();

        // Convert each hole into its own `LineString`
        let holes_coords: Vec<LineString<Real>> = holes
            .iter()
            .map(|hole| {
                let coords: Vec<Coord<Real>> =
                    hole.iter().map(|&[x, y]| Coord { x, y }).collect();
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

    /// Triangulate each polygon in the Mesh returning a Mesh containing triangles
    pub fn triangulate(&self) -> Mesh<S> {
        let triangles = self
            .polygons
            .iter()
            .flat_map(|poly| {
                poly.triangulate().into_iter().map(move |triangle| {
                    Polygon::new(triangle.to_vec(), poly.metadata.clone())
                })
            })
            .collect::<Vec<_>>();

        Mesh::from_polygons(&triangles)
    }

    /// Subdivide all polygons in this Mesh 'levels' times, returning a new Mesh.
    /// This results in a triangular mesh with more detail.
    pub fn subdivide_triangles(&self, levels: NonZeroU32) -> Mesh<S> {
        #[cfg(feature = "parallel")]
        let new_polygons: Vec<Polygon<S>> = self
            .polygons
            .par_iter()
            .flat_map(|poly| {
                let sub_tris = poly.subdivide_triangles(levels);
                // Convert each small tri back to a Polygon
                sub_tris.into_par_iter().map(move |tri| {
                    Polygon::new(
                        vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                        poly.metadata.clone(),
                    )
                })
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let new_polygons: Vec<Polygon<S>> = self
            .polygons
            .iter()
            .flat_map(|poly| {
                let sub_tris = poly.subdivide_triangles(levels);
                sub_tris.into_iter().map(move |tri| {
                    Polygon::new(
                        vec![tri[0].clone(), tri[1].clone(), tri[2].clone()],
                        poly.metadata.clone(),
                    )
                })
            })
            .collect();

        Mesh::from_polygons(&new_polygons)
    }
    
    /// Subdivide all polygons in this CSG 'levels' times, in place.
    /// This results in a triangular mesh with more detail.
    ///
    /// ## Example
    /// ```
    /// use csgrs::CSG;
    /// use core::num::NonZeroU32;
    /// let mut cube: CSG<()> = CSG::cube(2.0, None);
    /// // subdivide_triangles(1) => each polygon (quad) is triangulated => 2 triangles => each tri subdivides => 4
    /// // So each face with 4 vertices => 2 triangles => each becomes 4 => total 8 per face => 6 faces => 48
    /// cube.subdivide_triangles_mut(1.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 48);
    ///
    /// let mut cube: CSG<()> = CSG::cube(2.0, None);
    /// cube.subdivide_triangles_mut(2.try_into().expect("not zero"));
    /// assert_eq!(cube.polygons.len(), 192);
    /// ```
    pub fn subdivide_triangles_mut(&mut self, levels: NonZeroU32) {
        #[cfg(feature = "parallel")]
        {
            self.polygons = self
                .polygons
                .par_iter_mut()
                .flat_map(|poly| {
                    let sub_tris = poly.subdivide_triangles(levels.into());
                    // Convert each small tri back to a Polygon
                    sub_tris
                        .into_par_iter()
                        .map(move |tri| Polygon::new(tri.to_vec(), poly.metadata.clone()))
                })
                .collect();
        }

        #[cfg(not(feature = "parallel"))]
        {
            self.polygons = self
                .polygons
                .iter()
                .flat_map(|poly| {
                    let polytri = poly.subdivide_triangles(levels.into());
                    polytri
                        .into_iter()
                        .map(move |tri| Polygon::new(tri.to_vec(), poly.metadata.clone()))
                })
                .collect();
        }
    }

    /// Renormalize all polygons in this Mesh by re-computing each polygon’s plane
    /// and assigning that plane’s normal to all vertices.
    pub fn renormalize(&mut self) {
        for poly in &mut self.polygons {
            poly.set_new_normal();
        }
    }
    
	/// **Mathematical Foundation: Dihedral Angle Calculation**
    ///
    /// Computes the dihedral angle between two polygons sharing an edge.
    /// The angle is computed as the angle between the normal vectors of the two polygons.
    ///
    /// Returns the angle in radians.
    fn dihedral_angle(p1: &Polygon<S>, p2: &Polygon<S>) -> Real {
        let n1 = p1.plane.normal();
        let n2 = p2.plane.normal();
        let dot = n1.dot(&n2).clamp(-1.0, 1.0);
        dot.acos()
    }
    
    /// Extracts vertices and indices from the Mesh's tessellated polygons.
    fn get_vertices_and_indices(&self) -> (Vec<Point3<Real>>, Vec<[u32; 3]>) {
        let tri_csg = self.triangulate();
        let vertices = tri_csg
            .polygons
            .iter()
            .flat_map(|p| [p.vertices[0].pos, p.vertices[1].pos, p.vertices[2].pos])
            .collect();

        let indices = (0..tri_csg.polygons.len())
            .map(|i| {
                let offset = i as u32 * 3;
                [offset, offset + 1, offset + 2]
            })
            .collect();

        (vertices, indices)
    }
    
    /// Casts a ray defined by `origin` + t * `direction` against all triangles
    /// of this CSG and returns a list of (intersection_point, distance),
    /// sorted by ascending distance.
    ///
    /// # Parameters
    /// - `origin`: The ray’s start point.
    /// - `direction`: The ray’s direction vector.
    ///
    /// # Returns
    /// A `Vec` of `(Point3<Real>, Real)` where:
    /// - `Point3<Real>` is the intersection coordinate in 3D,
    /// - `Real` is the distance (the ray parameter t) from `origin`.
    pub fn ray_intersections(
        &self,
        origin: &Point3<Real>,
        direction: &Vector3<Real>,
    ) -> Vec<(Point3<Real>, Real)> {
        let ray = Ray::new(*origin, *direction);
        let iso = Isometry3::identity(); // No transformation on the triangles themselves.

        let mut hits = Vec::new();

        // 1) For each polygon in the CSG:
        for poly in &self.polygons {
            // 2) Triangulate it if necessary:
            let triangles = poly.triangulate();

            // 3) For each triangle, do a ray–triangle intersection test:
            for tri in triangles {
                let a = tri[0].pos;
                let b = tri[1].pos;
                let c = tri[2].pos;

                // Construct a parry Triangle shape from the 3 vertices:
                let triangle = Triangle::new(a, b, c);

                // Ray-cast against the triangle:
                if let Some(hit) =
                    triangle.cast_ray_and_get_normal(&iso, &ray, Real::MAX, true)
                {
                    let point_on_ray = ray.point_at(hit.time_of_impact);
                    hits.push((Point3::from(point_on_ray.coords), hit.time_of_impact));
                }
            }
        }

        // 4) Sort hits by ascending distance (toi):
        hits.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        // 5) remove duplicate hits if they fall within tolerance
        hits.dedup_by(|a, b| (a.1 - b.1).abs() < EPSILON);

        hits
    }

    /// Convert the polygons in this CSG to a Parry `TriMesh`, wrapped in a `SharedShape` to be used in Rapier.\
    /// Useful for collision detection or physics simulations.
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices, or Parry returns a `TriMeshBuilderError`
    pub fn to_rapier_shape(&self) -> SharedShape {
        let (vertices, indices) = self.get_vertices_and_indices();
        let trimesh = TriMesh::new(vertices, indices).unwrap();
        SharedShape::new(trimesh)
    }
    
    /// Convert the polygons in this CSG to a Parry `TriMesh`.\
    /// Useful for collision detection.
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices, or Parry returns a `TriMeshBuilderError`
    pub fn to_trimesh(&self) -> Option<TriMesh> {
        let (vertices, indices) = self.get_vertices_and_indices();
        TriMesh::new(vertices, indices).ok()
    }

    /// Uses Parry to check if a point is inside a `CSG`'s as a `TriMesh`.\
    /// Note: this only use the 3d geometry of `CSG`
    ///
    /// ## Errors
    /// If any 3d polygon has fewer than 3 vertices
    ///
    /// ## Example
    /// ```
    /// # use csgrs::CSG;
    /// # use nalgebra::Point3;
    /// # use nalgebra::Vector3;
    /// let csg_cube = CSG::<()>::cube(6.0, None);
    ///
    /// assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 3.0)));
    /// assert!(csg_cube.contains_vertex(&Point3::new(1.0, 2.0, 5.9)));
    ///
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0)));
    /// assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, -6.0)));
    /// ```
    pub fn contains_vertex(&self, point: &Point3<Real>) -> bool {
        self.ray_intersections(point, &Vector3::new(1.0, 1.0, 1.0))
            .len()
            % 2
            == 1
    }

    /// Approximate mass properties using Rapier.
    pub fn mass_properties(
        &self,
        density: Real,
    ) -> (Real, Point3<Real>, Unit<Quaternion<Real>>) {
        let trimesh = self.to_trimesh().unwrap();
        let mp = trimesh.mass_properties(density);

        (
            mp.mass(),
            mp.local_com,                     // a Point3<Real>
            mp.principal_inertia_local_frame, // a Unit<Quaternion<Real>>
        )
    }

    /// Create a Rapier rigid body + collider from this Mesh, using
    /// an axis-angle `rotation` in 3D (the vector’s length is the
    /// rotation in radians, and its direction is the axis).
    pub fn to_rigid_body(
        &self,
        rb_set: &mut RigidBodySet,
        co_set: &mut ColliderSet,
        translation: Vector3<Real>,
        rotation: Vector3<Real>, // rotation axis scaled by angle (radians)
        density: Real,
    ) -> RigidBodyHandle {
        let shape = self.to_rapier_shape();

        // Build a Rapier RigidBody
        let rb = RigidBodyBuilder::dynamic()
            .translation(translation)
            // Now `rotation(...)` expects an axis-angle Vector3.
            .rotation(rotation)
            .build();
        let rb_handle = rb_set.insert(rb);

        // Build the collider
        let coll = ColliderBuilder::new(shape).density(density).build();
        co_set.insert_with_parent(coll, rb_handle, rb_set);

        rb_handle
    }

    /// Convert a Mesh into a Bevy `Mesh`.
    #[cfg(feature = "bevymesh")]
    pub fn to_bevy_mesh(&self) -> bevy_mesh::Mesh {
        use bevy_asset::RenderAssetUsages;
        use bevy_mesh::{Indices, Mesh};
        use wgpu_types::PrimitiveTopology;

        let triangulated_mesh = &self.triangulate();
        let polygons = &triangulated_mesh.polygons;

        // Prepare buffers
        let mut positions_32 = Vec::new();
        let mut normals_32 = Vec::new();
        let mut indices = Vec::with_capacity(polygons.len() * 3);

        let mut index_start = 0u32;

        // Each polygon is assumed to have exactly 3 vertices after tessellation.
        for poly in polygons {
            // skip any degenerate polygons
            if poly.vertices.len() != 3 {
                continue;
            }

            // push 3 positions/normals
            for v in &poly.vertices {
                positions_32.push([v.pos.x as f32, v.pos.y as f32, v.pos.z as f32]);
                normals_32.push([v.normal.x as f32, v.normal.y as f32, v.normal.z as f32]);
            }

            // triangle indices
            indices.push(index_start);
            indices.push(index_start + 1);
            indices.push(index_start + 2);
            index_start += 3;
        }

        // Create the mesh with the new 2-argument constructor
        let mut mesh =
            Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());

        // Insert attributes. Note the `<Vec<[f32; 3]>>` usage.
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions_32);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals_32);

        // Insert triangle indices
        mesh.insert_indices(Indices::U32(indices));

        mesh
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
        let (a_clip, a_passthru) =
            Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, b_passthru) =
            Self::partition_polys(&other.polygons, &self.bounding_box());

        let mut a = Node::from_polygons(&a_clip);
        let mut b = Node::from_polygons(&b_clip);

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
        let (a_clip, a_passthru) =
            Self::partition_polys(&self.polygons, &other.bounding_box());
        let (b_clip, _b_passthru) =
            Self::partition_polys(&other.polygons, &self.bounding_box());

        let mut a = Node::from_polygons(&a_clip);
        let mut b = Node::from_polygons(&b_clip);

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
        let mut a = Node::from_polygons(&self.polygons);
        let mut b = Node::from_polygons(&other.polygons);

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
        let mat_inv_transpose = mat.try_inverse().expect("Matrix not invertible?").transpose(); // todo catch error
        let mut mesh = self.clone();

        for poly in &mut mesh.polygons {
            for vert in &mut poly.vertices {
                // Position
                let homog_pos = mat * vert.pos.to_homogeneous();
                vert.pos = Point3::from_homogeneous(homog_pos).unwrap(); // todo catch error

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

impl<S: Clone + Send + Sync + Debug> From<Sketch<S>> for Mesh<S> {
    /// Convert a Sketch into a Mesh.
    fn from(sketch: Sketch<S>) -> Self {
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
                    hole_vertices_3d
                        .push(Vertex::new(Point3::new(c.x, c.y, 0.0), Vector3::z()));
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
                },
                Geometry::MultiPolygon(multipoly) => {
                    for poly2d in multipoly {
                        process_polygon(poly2d, &mut all_polygons, &sketch.metadata);
                    }
                },
                // Optional: handle other geometry types like LineString here.
                _ => {},
            }
        }

        Mesh {
            polygons: all_polygons,
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }
}
