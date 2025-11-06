//! **Mathematical Foundations for 3D Plane Operations**
//!
//! This module implements robust geometric operations for planes in 3-space based on
//! established computational geometry principles:
//!
//! ## **Theoretical Foundation**
//!
//! ### **Plane Representation**
//! A plane π in 3D space can be represented as:
//! - **Implicit form**: ax + by + cz + d = 0, where (a,b,c)ᵀ is the normal vector
//! - **Point-normal form**: n⃗·(p⃗ - p₀⃗) = 0, where n⃗ is the unit normal and p₀⃗ is a point on the plane
//! - **Three-point form**: Defined by three non-collinear points A, B, C
//!
//! ### **Orientation Testing Algorithms**
//!
//! **Robust Geometric Predicates**: This implementation uses the `robust` crate's
//! `orient3d` predicate, which implements Shewchuk's exact arithmetic methods for
//! robust orientation testing. The predicate computes the sign of the determinant:
//!
//! ```text
//! |ax  ay  az  1|
//! |bx  by  bz  1|
//! |cx  cy  cz  1|
//! |dx  dy  dz  1|
//! ```
//!
//! This determines whether point D lies above, below, or on the plane defined by A, B, C.
//!
//! ### **Polygon Splitting Algorithm**
//!
//! **Sutherland-Hodgman Clipping**: The `split_polygon` function implements a 3D
//! generalization of the Sutherland-Hodgman polygon clipping algorithm:
//!
//! 1. **Classification**: Each vertex is classified as FRONT, BACK, COPLANAR, or SPANNING
//! 2. **Edge Processing**: For each edge (vᵢ, vⱼ):
//!    - If both vertices are on the same side, add appropriate vertex
//!    - If edge spans the plane, compute intersection and add to both output lists
//! 3. **Intersection Computation**: For spanning edges, solve for intersection parameter t:
//!    ```text
//!    t = (d - n⃗·vᵢ) / (n⃗·(vⱼ - vᵢ))
//!    ```
//!    where d is the plane's signed distance from origin.
//!
//! ### **Coordinate System Transformations**
//!
//! **Plane-to-XY Projection**: The `to_xy_transform` method computes an orthonormal
//! transformation that maps the plane to the XY-plane (z=0):
//!
//! 1. **Rotation**: Find rotation R such that plane normal n⃗ → (0,0,1)ᵀ
//! 2. **Translation**: Translate so plane passes through origin
//! 3. **Combined Transform**: T = T₂ · R · T₁
//!
//! This enables 2D algorithms to be applied to 3D planar polygons.
//!
//! ## **Numerical Stability**
//!
//! - **Robust Predicates**: Uses exact arithmetic for orientation tests
//! - **Epsilon Tolerances**: Governed by `float_types::EPSILON` for floating-point comparisons
//! - **Degenerate Case Handling**: Proper fallbacks for collinear points and zero-area triangles
//!
//! ## **Algorithm Complexity**
//!
//! - **Plane Construction**: O(n²) for optimal triangle selection, O(1) for basic construction
//! - **Orientation Testing**: O(1) per point with robust predicates
//! - **Polygon Splitting**: O(n) per polygon, where n is the number of vertices
//!
//! Unless stated otherwise, all tolerances are governed by `float_types::EPSILON`.

use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use crate::math_ndsp::{Isometry3, Matrix4, Point3, Rotation3, Translation3, Vector3, eps};
use robust::{Coord3D, orient3d};

/// Classification of a polygon or point that lies exactly in the plane
/// (i.e. within `±EPSILON` of the plane).
pub const COPLANAR: i8 = 0;

/// Classification of a polygon or point that lies strictly on the
/// *front* side of the plane (the side the normal points toward).
pub const FRONT: i8 = 1;

/// Classification of a polygon or point that lies strictly on the
/// *back* side of the plane (opposite the normal direction).
pub const BACK: i8 = 2;

/// A polygon or edge that straddles the plane, producing pieces
/// on both the front **and** the back.
pub const SPANNING: i8 = 3;

/// A plane in 3D space defined by three points
#[derive(Debug, Clone)]
pub struct Plane<T> {
    pub point_a: Point3<T>,
    pub point_b: Point3<T>,
    pub point_c: Point3<T>,
}

impl<T> PartialEq for Plane<T> {
    fn eq(&self, other: &Self) -> bool {
        if self.point_a == other.point_a
            && self.point_b == other.point_b
            && self.point_c == other.point_c
        {
            true
        } else {
            // check if co-planar
            robust::orient3d(
                point_to_coord3d(self.point_a),
                point_to_coord3d(self.point_b),
                point_to_coord3d(self.point_c),
                point_to_coord3d(other.point_a),
            ) == 0.0
                && robust::orient3d(
                    point_to_coord3d(self.point_a),
                    point_to_coord3d(self.point_b),
                    point_to_coord3d(self.point_c),
                    point_to_coord3d(other.point_b),
                ) == 0.0
                && robust::orient3d(
                    point_to_coord3d(self.point_a),
                    point_to_coord3d(self.point_b),
                    point_to_coord3d(self.point_c),
                    point_to_coord3d(other.point_c),
                ) == 0.0
        }
    }
}

fn point_to_coord3d(point: Point3<T>) -> robust::Coord3D<f64> {
    robust::Coord3D {
        x: point.coords.x,
        y: point.coords.y,
        z: point.coords.z,
    }
}

impl<T> Plane<T> {
    /// Tries to pick three vertices that span the largest area triangle
    /// (maximally well-spaced) and returns a plane defined by them.
    /// Care is taken to preserve the original winding of the vertices.
    ///
    /// Cost: O(n^2)
    /// A lower cost option may be a grid sub-sampled farthest pair search
    pub fn from_vertices(vertices: Vec<Vertex>) -> Plane<T> {
        let n = vertices.len();
        let reference_plane = Plane {
            point_a: vertices[0].pos,
            point_b: vertices[1].pos,
            point_c: vertices[2].pos,
        };
        if n == 3 {
            return reference_plane;
        } // Plane is already optimal

        // longest chord (i0,i1)
        let Some((i0, i1, _)) = (0..n)
            .flat_map(|i| (i + 1..n).map(move |j| (i, j)))
            .map(|(i, j)| {
                let d2 = (vertices[i].pos - vertices[j].pos).norm_squared();
                (i, j, d2)
            })
            .max_by(|a, b| a.2.total_cmp(&b.2))
        else {
            return reference_plane;
        };

        let p0 = vertices[i0].pos;
        let p1 = vertices[i1].pos;
        let dir = p1 - p0;
        if dir.norm_squared() < eps::<T>() * eps::<T>() {
            return reference_plane; // everything almost coincident
        }

        // vertex farthest from the line  p0-p1  → i2
        let Some((i2, max_area2)) = vertices
            .iter()
            .enumerate()
            .filter(|(idx, _)| *idx != i0 && *idx != i1)
            .map(|(idx, v)| {
                let a2 = (v.pos - p0).cross(&dir).norm_squared(); // ∝ area²
                (idx, a2)
            })
            .max_by(|a, b| a.1.total_cmp(&b.1))
        else {
            return reference_plane;
        };

        let i2 = if max_area2 > eps::<T>() * eps::<T>() {
            i2
        } else {
            return reference_plane; // all vertices collinear
        };
        let p2 = vertices[i2].pos;

        // build plane, then orient it to match original winding
        let mut plane_hq = Plane {
            point_a: p0,
            point_b: p1,
            point_c: p2,
        };

        // Construct the reference normal for the original polygon using Newell's Method.
        let reference_normal = vertices.iter().zip(vertices.iter().cycle().skip(1)).fold(
            Vector3::zeros(),
            |acc, (curr, next)| {
                acc + (curr.pos - Point3::origin()).cross(&(next.pos - Point3::origin()))
            },
        );

        if plane_hq.normal().dot(&reference_normal) < 0.0 {
            plane_hq.flip(); // flip in-place to agree with winding
        }
        plane_hq
    }

    /// Build a new `Plane` from a (not‑necessarily‑unit) normal **n**
    /// and signed offset *o* (in the sense `n · p == o`).
    ///
    /// If `normal` is close to zero the function fails
    pub fn from_normal(normal: Vector3<T>, offset: T) -> Self {
        let n2 = normal.norm_squared();
        if n2 < eps::<T>() * eps::<T>() {
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
    pub fn orient_plane(&self, other: &Plane<T>) -> i8 {
        // pick one vertex of the coplanar polygon and move along its normal
        let test_point = other.point_a + other.normal();
        self.orient_point(&test_point)
    }

    #[inline]
    pub fn orient_point(&self, point: &Point3<T>) -> i8 {
        // Returns a positive value if the point `pd` lies below the plane passing through `pa`, `pb`, and `pc`
        // ("below" is defined so that `pa`, `pb`, and `pc` appear in counterclockwise order when viewed from above the plane).
        // Returns a negative value if `pd` lies above the plane.
        // Returns `0` if they are **coplanar**.
        let sign = orient3d(
            Coord3D {
                x: self.point_a.x,
                y: self.point_a.y,
                z: self.point_a.z,
            },
            Coord3D {
                x: self.point_b.x,
                y: self.point_b.y,
                z: self.point_b.z,
            },
            Coord3D {
                x: self.point_c.x,
                y: self.point_c.y,
                z: self.point_c.z,
            },
            Coord3D {
                x: point.x,
                y: point.y,
                z: point.z,
            },
        );
        #[allow(clippy::useless_conversion)]
        if sign > eps::<T>().into() {
            BACK
        } else if sign < (-eps::<T>()).into() {
            FRONT
        } else {
            COPLANAR
        }
    }

    /// Return the (right‑handed) unit normal **n** of the plane
    /// `((b‑a) × (c‑a)).normalize()`.
    #[inline]
    pub fn normal(&self) -> Vector3<T> {
        let n = (self.point_b - self.point_a).cross(&(self.point_c - self.point_a));
        let len = n.norm();
        if len < eps::<T>() {
            Vector3::zeros()
        } else {
            n / len
        }
    }

    /// Signed offset of the plane from the origin: `d = n · a`.
    #[inline]
    pub fn offset(&self) -> T {
        self.normal().dot(&self.point_a.coords)
    }

    pub const fn flip(&mut self) {
        std::mem::swap(&mut self.point_a, &mut self.point_b);
    }

    /// Classify a polygon with respect to the plane.
    /// Returns a bitmask of `COPLANAR`, `FRONT`, and `BACK`.
    pub fn classify_polygon<S: Clone, T>(&self, polygon: &Polygon<S, T>) -> i8 {
        let mut polygon_type: i8 = 0;
        for vertex in &polygon.vertices {
            polygon_type |= self.orient_point(&vertex.pos);
        }
        polygon_type
    }

    /// Splits a polygon by this plane, returning four buckets:
    /// `(coplanar_front, coplanar_back, front, back)`.
    #[allow(clippy::type_complexity)]
    pub fn split_polygon<S: Clone + Send + Sync>(
        &self,
        polygon: &Polygon<S, T>,
    ) -> (
        Vec<Polygon<S, T>>,
        Vec<Polygon<S, T>>,
        Vec<Polygon<S, T>>,
        Vec<Polygon<S, T>>,
    ) {
        let mut coplanar_front = Vec::new();
        let mut coplanar_back = Vec::new();
        let mut front = Vec::new();
        let mut back = Vec::new();

        let normal = self.normal();

        let types: Vec<i8> = polygon
            .vertices
            .iter()
            .map(|v| self.orient_point(&v.pos))
            .collect();
        let polygon_type = types.iter().fold(0, |acc, &t| acc | t);

        // -----------------------------------------------------------------
        // 2.  dispatch the easy cases
        // -----------------------------------------------------------------
        match polygon_type {
            COPLANAR => {
                if normal.dot(&polygon.plane.normal()) > 0.0 {
                    // >= ?
                    coplanar_front.push(polygon.clone());
                } else {
                    coplanar_back.push(polygon.clone());
                }
            },
            FRONT => front.push(polygon.clone()),
            BACK => back.push(polygon.clone()),

            // -------------------------------------------------------------
            // 3.  true spanning – do the split
            // -------------------------------------------------------------
            _ => {
                let mut split_front = Vec::<Vertex>::new();
                let mut split_back = Vec::<Vertex>::new();

                for i in 0..polygon.vertices.len() {
                    // j is the vertex following i, we modulo by len to wrap around to the first vertex after the last
                    let j = (i + 1) % polygon.vertices.len();
                    let type_i = types[i];
                    let type_j = types[j];
                    let vertex_i = &polygon.vertices[i];
                    let vertex_j = &polygon.vertices[j];

                    // If current vertex is definitely not behind plane, it goes to split_front
                    if type_i != BACK {
                        split_front.push(*vertex_i);
                    }
                    // If current vertex is definitely not in front, it goes to split_back
                    if type_i != FRONT {
                        split_back.push(*vertex_i);
                    }

                    // If the edge between these two vertices crosses the plane,
                    // compute intersection and add that intersection to both sets
                    if (type_i | type_j) == SPANNING {
                        let denom = normal.dot(&(vertex_j.pos - vertex_i.pos));
                        // Avoid dividing by zero
                        if denom.abs() > eps::<T>() {
                            let intersection =
                                (self.offset() - normal.dot(&vertex_i.pos.coords)) / denom;
                            let vertex_new = vertex_i.interpolate(vertex_j, intersection);
                            split_front.push(vertex_new);
                            split_back.push(vertex_new);
                        }
                    }
                }

                // Build new polygons from the front/back vertex lists
                // if they have at least 3 vertices
                if split_front.len() >= 3 {
                    front.push(Polygon::new(split_front, polygon.metadata.clone()));
                }
                if split_back.len() >= 3 {
                    back.push(Polygon::new(split_back, polygon.metadata.clone()));
                }
            },
        }

        (coplanar_front, coplanar_back, front, back)
    }

    /// Returns (T, T_inv), where:
    /// - `T` maps a point on this plane into XY plane (z=0) with the plane's normal going to +Z
    /// - `T_inv` is the inverse transform, mapping back
    ///
    /// **Mathematical Foundation**: This implements an orthonormal transformation:
    /// 1. **Rotation Matrix**: R = rotation_between(plane_normal, +Z)
    /// 2. **Translation**: Translate so plane passes through origin
    /// 3. **Combined Transform**: T = T₂ · R · T₁
    ///
    /// The transformation preserves distances and angles, enabling 2D algorithms
    /// to be applied to 3D planar geometry.
    pub fn to_xy_transform(&self) -> (Matrix4<T>, Matrix4<T>) {
        // Normal
        let n = self.normal();
        let n_len = n.norm();
        if n_len < eps::<T>() {
            // Degenerate plane, return identity
            return (Matrix4::identity(), Matrix4::identity());
        }

        // Normalize
        let norm_dir = n / n_len;

        // Rotate plane.normal -> +Z
        let rot = Rotation3::rotation_between(&norm_dir, &Vector3::z())
            .unwrap_or_else(Rotation3::identity);
        let iso_rot = Isometry3::from_parts(Translation3::identity(), rot.into());

        // We want to translate so that the plane's reference point
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

#[test]
fn test_plane_orientation() {
    let vertices = [
        Vertex {
            pos: Point3::new(1152.0, 256.0, 512.0),
            normal: Vector3::new(0., 1., 0.),
        },
        Vertex {
            pos: Point3::new(1152.0, 256.0, 256.0),
            normal: Vector3::new(0., 1., 0.),
        },
        Vertex {
            pos: Point3::new(768.0, 256.0, 256.0),
            normal: Vector3::new(0., 1., 0.),
        },
        Vertex {
            pos: Point3::new(768.0, 256.0, 512.0),
            normal: Vector3::new(0., 1., 0.),
        },
        Vertex {
            pos: Point3::new(896.0, 256.0, 512.0),
            normal: Vector3::new(0., 1., 0.),
        },
        Vertex {
            pos: Point3::new(896.0, 256.0, 384.0),
            normal: Vector3::new(0., 1., 0.),
        },
        Vertex {
            pos: Point3::new(1024.0, 256.0, 384.0),
            normal: Vector3::new(0., 1., 0.),
        },
        Vertex {
            pos: Point3::new(1024.0, 256.0, 512.0),
            normal: Vector3::new(0., 1., 0.),
        },
    ];

    // Cycling the order of the vertices doesn't change the winding order of the shape,
    // so it should not change the resulting plane's normal.
    for cycle_rotation in 0..vertices.len() {
        let mut vertices = vertices;
        vertices.rotate_right(cycle_rotation);
        let plane = Plane::from_vertices(vertices.to_vec());

        assert!(
            plane.normal() == Vector3::new(0., 1., 0.),
            "the vertices {vertices:?} form a plane with unexpected normal {}, \
            expected (0., 1., 0.); \
            point list obtained by rotating {cycle_rotation} times",
            plane.normal(),
        );
    }
}
