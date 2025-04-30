use crate::float_types::{EPSILON, Real};
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use nalgebra::{Isometry3, Matrix4, Point3, Rotation3, Translation3, Vector3};
use robust::{orient3d, Coord3D};

pub const COPLANAR: i8 = 0;
pub const FRONT:    i8 = 1;
pub const BACK:     i8 = 2;
pub const SPANNING: i8 = 3;

/// A plane in 3D space defined by three points
#[derive(Debug, Clone)]
pub struct Plane {
    pub point_a: Point3<Real>,
    pub point_b: Point3<Real>,
    pub point_c: Point3<Real>,
}

impl Plane {
    /// Create a plane from three points
    pub const fn from_points(a: &Point3<Real>, b: &Point3<Real>, c: &Point3<Real>) -> Plane {
        Plane {
            point_a: *a,
            point_b: *b,
            point_c: *c,
        }
    }
    
    /// Build a new `Plane` from a (not‑necessarily‑unit) normal **n**
    /// and signed offset *o* (in the sense `n · p == o`).
    ///
    /// If `normal` is close to zero the function fails
    pub fn from_normal(normal: Vector3<Real>, offset: Real) -> Self {
        let n2 = normal.norm_squared();
        if n2 < EPSILON * EPSILON {
            panic!(); // degenerate normal
        }

        // Point on the plane:  p0 = n * o / (n·n)
        let p0 = Point3::from(normal * (offset / n2));

        // Build an orthonormal basis {u, v} that spans the plane.
        // Pick the largest component of n to avoid numerical problems.
        let mut u = if normal.z.abs() > normal.x.abs() || normal.z.abs() > normal.y.abs() {
            // n is closer to ±Z ⇒ cross with X
            Vector3::x().cross(&normal)
        } else {
            // otherwise cross with Z
            Vector3::z().cross(&normal)
        };
        u.normalize_mut();
        let v = normal.cross(&u).normalize();
        
        // Use p0, p0+u, p0+v  as the three defining points.
        Self {
            point_a: p0,
            point_b: p0 + u,
            point_c: p0 + v,
        }
    }
    
    #[inline] 
    pub fn orient_plane(&self, other: &Plane) -> i8 {
        // pick one vertex of the coplanar polygon and move along its normal
        let test_point = other.point_a + other.normal();
        self.orient_point(&test_point)
    }
    
    #[inline]
    pub fn orient_point(&self, point: &Point3<Real>) -> i8 {
        // Returns a positive value if the point `pd` lies below the plane passing through `pa`, `pb`, and `pc`
        // ("below" is defined so that `pa`, `pb`, and `pc` appear in counterclockwise order when viewed from above the plane).  
        // Returns a negative value if `pd` lies above the plane.  
        // Returns `0` if they are **coplanar**.
        let sign = orient3d(
            Coord3D { x: self.point_a.x, y: self.point_a.y, z: self.point_a.z },
            Coord3D { x: self.point_b.x, y: self.point_b.y, z: self.point_b.z },
            Coord3D { x: self.point_c.x, y: self.point_c.y, z: self.point_c.z },
            Coord3D { x: point.x, y: point.y, z: point.z},
        );
        if sign > EPSILON {
            BACK
        } else if sign < -EPSILON {
            FRONT
        } else {
            COPLANAR
        }
    }
    
    /// Return the (right‑handed) unit normal **n** of the plane
    /// `((b‑a) × (c‑a)).normalize()`.
    #[inline]
    pub fn normal(&self) -> Vector3<Real> {
        (self.point_b - self.point_a)
            .cross(&(self.point_c - self.point_a))
            .normalize()
    }

    /// Signed offset of the plane from the origin: `d = n · a`.
    #[inline]
    pub fn offset(&self) -> Real {
        self.normal().dot(&self.point_a.coords)
    }

    pub const fn flip(&mut self) {
        std::mem::swap(&mut self.point_a, &mut self.point_b);
    }

    // ────────────────────────────────────────────────────────────────
    //  Robust polygon split
    // ────────────────────────────────────────────────────────────────
    ///
    /// Returns four buckets:
    /// `(coplanar_front, coplanar_back, front, back)`.
    pub fn split_polygon<S: Clone + Send + Sync>(
        &self,
        polygon: &Polygon<S>,
    ) -> (
        Vec<Polygon<S>>,
        Vec<Polygon<S>>,
        Vec<Polygon<S>>,
        Vec<Polygon<S>>,
    ) {    
        let mut coplanar_front = Vec::new();
        let mut coplanar_back  = Vec::new();
        let mut front          = Vec::new();
        let mut back           = Vec::new();
        
        let normal = self.normal();
    
        let mut types = Vec::with_capacity(polygon.vertices.len());
        let mut polygon_type: i8 = 0;
        for vertex in &polygon.vertices {
            let vertex_type = self.orient_point(&vertex.pos);
            types.push(vertex_type);
            polygon_type |= vertex_type; // bitwise OR vertex types to figure polygon type
        }
    
        // -----------------------------------------------------------------
        // 2.  dispatch the easy cases
        // -----------------------------------------------------------------
        match polygon_type {
            COPLANAR => {
                if normal.dot(&polygon.plane().normal()) > 0.0 {  // >= ?
                    coplanar_front.push(polygon.clone());
                } else {
                    coplanar_back.push(polygon.clone());
                }
            }
            FRONT => front.push(polygon.clone()),
            BACK  => back.push(polygon.clone()),
    
            // -------------------------------------------------------------
            // 3.  true spanning – do the split
            // -------------------------------------------------------------
            _ => {    
                let mut split_front = Vec::<Vertex>::new();
                let mut split_back = Vec::<Vertex>::new();
    
                for i in 0..polygon.vertices.len() {
                    // j is the vertex following i, we modulo by len to wrap around to the first vertex after the last
                    let j  = (i + 1) % polygon.vertices.len();
                    let type_i = types[i];
                    let type_j = types[j];
                    let vertex_i = &polygon.vertices[i];
                    let vertex_j = &polygon.vertices[j];
    
                    // If current vertex is definitely not behind plane, it goes to split_front
                    if type_i != BACK  { split_front.push(vertex_i.clone()); }
                    // If current vertex is definitely not in front, it goes to split_back
                    if type_i != FRONT { split_back.push(vertex_i.clone()); }
    
                    // If the edge between these two vertices crosses the plane,
                    // compute intersection and add that intersection to both sets
                    if (type_i | type_j) == SPANNING {
                        let denom = normal.dot(&(vertex_j.pos - vertex_i.pos));
                        // Avoid dividing by zero
                        if denom.abs() > EPSILON {
                            let intersection = (self.offset() - normal.dot(&vertex_i.pos.coords)) / denom;
                            let vertex_new = vertex_i.interpolate(vertex_j, intersection);
                            split_front.push(vertex_new.clone());
                            split_back.push(vertex_new);
                        }
                    }
                }
    
                // Build new polygons from the front/back vertex lists
                // if they have at least 3 vertices
                if split_front.len() >= 3 { front.push(Polygon::new(split_front, polygon.metadata.clone())); }
                if split_back.len() >= 3 { back.push(Polygon::new(split_back, polygon.metadata.clone())); }
            }
        }
    
        (coplanar_front, coplanar_back, front, back)
    }

    /// Returns (T, T_inv), where:
    /// - `T`   maps a point on this plane into XY plane (z=0)
    ///   with the plane’s normal going to +Z,
    /// - `T_inv` is the inverse transform, mapping back.
    pub fn to_xy_transform(&self) -> (Matrix4<Real>, Matrix4<Real>) {
        // Normal
        let n = self.normal();
        let n_len = n.norm();
        if n_len < EPSILON {
            // Degenerate plane, return identity
            return (Matrix4::identity(), Matrix4::identity());
        }

        // Normalize
        let norm_dir = n / n_len;

        // Rotate plane.normal -> +Z
        let rot = Rotation3::rotation_between(&norm_dir, &Vector3::z())
            .unwrap_or_else(Rotation3::identity);
        let iso_rot = Isometry3::from_parts(Translation3::identity(), rot.into());

        // We want to translate so that the plane’s reference point
        //    (some point p0 with n·p0 = w) lands at z=0 in the new coords.
        // p0 = (plane.w / (n·n)) * n
        let denom = n.dot(&n);
        let p0_3d = norm_dir * (self.offset() / denom);
        let p0_rot = iso_rot.transform_point(&Point3::from(p0_3d));

        // We want p0_rot.z = 0, so we shift by -p0_rot.z
        let shift_z = -p0_rot.z;
        let iso_trans = Translation3::new(0.0, 0.0, shift_z);

        let transform_to_xy = iso_trans.to_homogeneous() * iso_rot.to_homogeneous();

        // Inverse for going back
        let transform_from_xy = transform_to_xy
            .try_inverse()
            .unwrap_or_else(Matrix4::identity);

        (transform_to_xy, transform_from_xy)
    }
}
