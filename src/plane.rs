use crate::float_types::{EPSILON, Real};
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use nalgebra::{Isometry3, Matrix4, Point3, Rotation3, Translation3, Vector3};
use std::ops::Neg;

/// A plane in 3D space defined by a normal and an intercept
#[derive(Debug, Clone)]
pub struct Plane {
    /// The direction of the plane, which defines a vector that is perpendicular to the plane.
    pub normal: Vector3<Real>,
    /// Y-intercept of the plane, the distance along the normal at which it's crossed by the plane.
    pub intercept: Real,
}

impl Neg for Plane {
    type Output = Self;

    /// [`flip`](Plane::flip) the `Plane`
    fn neg(self) -> Self::Output {
        Self {
            normal: -self.normal,
            intercept: -self.intercept,
        }
    }
}

impl Plane {
    /// Create a plane from three points
    pub fn from_points(a: &Point3<Real>, b: &Point3<Real>, c: &Point3<Real>) -> Plane {
        let n = (b - a).cross(&(c - a)).normalize();
        if n.magnitude() < EPSILON {
            panic!("Degenerate polygon: vertices do not define a plane"); // todo: return error
        }

        Plane {
            normal: n,
            intercept: n.dot(&a.coords),
        }
    }

    /// Flips the normal of the plane, in place
    pub fn flip(&mut self) {
        self.normal = -self.normal;
        self.intercept = -self.intercept;
    }

    /// Split `polygon` by this plane if needed, distributing the results into
    /// `coplanar_front`, `coplanar_back`, `front`, and `back`.
    pub fn split_polygon<S: Clone + Send + Sync>(
        &self,
        polygon: &Polygon<S>,
        coplanar_front: &mut Vec<Polygon<S>>,
        coplanar_back: &mut Vec<Polygon<S>>,
        front: &mut Vec<Polygon<S>>,
        back: &mut Vec<Polygon<S>>,
    ) {
        const COPLANAR: i8 = 0;
        const FRONT: i8 = 1;
        const BACK: i8 = 2;
        const SPANNING: i8 = 3;

        // Classify each vertex
        let (types, polygon_type) = polygon
            .vertices
            .iter()
            .map(|v| {
                let t = self.normal.dot(&v.pos.coords) - self.intercept;
                if t < -EPSILON {
                    BACK
                } else if t > EPSILON {
                    FRONT
                } else {
                    COPLANAR
                }
            })
            .fold(
                (Vec::with_capacity(polygon.vertices.len()), 0),
                |(mut vec, acc), ty| {
                    vec.push(ty);
                    (vec, acc | ty)
                },
            );

        match polygon_type {
            COPLANAR => {
                // Coincident normals => belongs in front vs. back
                if self.normal.dot(&polygon.plane.normal) > 0.0 {
                    coplanar_front.push(polygon.clone());
                } else {
                    coplanar_back.push(polygon.clone());
                }
            }
            FRONT => {
                front.push(polygon.clone());
            }
            BACK => {
                back.push(polygon.clone());
            }
            _ => {
                // SPANNING
                let mut f: Vec<Vertex> = Vec::new();
                let mut b: Vec<Vertex> = Vec::new();

                // Use zip with cycle to pair (current, next)
                for ((&ti, vi), (&tj, vj)) in types.iter().zip(polygon.vertices.iter()).zip(
                    types
                        .iter()
                        .cycle()
                        .skip(1)
                        .zip(polygon.vertices.iter().cycle().skip(1)),
                ) {
                    // If current vertex is definitely not behind plane,
                    // it goes to f (front side)
                    if ti != BACK {
                        f.push(vi.clone());
                    }
                    // If current vertex is definitely not in front,
                    // it goes to b (back side)
                    if ti != FRONT {
                        b.push(vi.clone());
                    }

                    // If the edge between these two vertices crosses the plane,
                    // compute intersection and add that intersection to both sets
                    if (ti | tj) == SPANNING {
                        let denom = self.normal.dot(&(vj.pos - vi.pos));
                        // Avoid dividing by zero
                        if denom.abs() > EPSILON {
                            let t = (self.intercept - self.normal.dot(&vi.pos.coords)) / denom;
                            let v_new = vi.interpolate(vj, t);
                            f.push(v_new.clone());
                            b.push(v_new);
                        }
                    }
                }

                // Build new polygons from the front/back vertex lists
                // if they have at least 3 vertices
                if f.len() >= 3 {
                    front.push(Polygon::new(f, polygon.metadata.clone()));
                }
                if b.len() >= 3 {
                    back.push(Polygon::new(b, polygon.metadata.clone()));
                }
            }
        }
    }

    /// Returns (T, T_inv), where:
    /// - `T`   maps a point on this plane into XY plane (z=0)
    ///   with the plane’s normal going to +Z,
    /// - `T_inv` is the inverse transform, mapping back.
    pub fn to_xy_transform(&self) -> (Matrix4<Real>, Matrix4<Real>) {
        // Normal
        let n = self.normal;
        let n_len = n.norm();
        if n_len < 1e-12 {
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
        let p0_3d = norm_dir * (self.intercept / denom);
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
