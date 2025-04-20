use crate::float_types::{Real, EPSILON};
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use nalgebra::{Isometry3, Matrix4, Point3, Rotation3, Translation3, Vector3};
use robust::{orient3d, Coord3D};

/// A plane in 3D space defined by three points
#[derive(Debug, Clone)]
pub struct Plane {
    pub point_a: Point3<Real>,
    pub point_b: Point3<Real>,
    pub point_c: Point3<Real>,
}

impl Plane {
    /// Create a plane from three points
    pub fn from_points(a: &Point3<Real>, b: &Point3<Real>, c: &Point3<Real>) -> Plane {
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
            panic!();                // degenerate normal
        }

        // Point on the plane:  p0 = n * o / (n·n)
        let p0 = Point3::from(normal * (offset / n2));

        // Build an orthonormal basis {u, v} that spans the plane.
        // Pick the largest component of n to avoid numerical problems.
        let mut u = if normal.x.abs() < normal.y.abs() {
            // n is closer to (0,±1,0)
            Vector3::new(0.0, normal.z, -normal.y)
        } else {
            // n is closer to (±1,0,0)  or  z–axis
            Vector3::new(normal.y, -normal.x, 0.0)
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

    pub fn flip(&mut self) {
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
        const COPLANAR: i8 = 0;
        const FRONT: i8 = 1;
        const BACK: i8 = 2;
        const SPANNING: i8 = 3;

        let mut coplanar_front = Vec::new();
        let mut coplanar_back = Vec::new();
        let mut front = Vec::new();
        let mut back = Vec::new();

        let n = self.normal();
        let d = self.offset();

        // ───── vertex classification with Shewchuk orient3d ─────
        let classify = |pt: &Point3<Real>| -> i8 {
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
                    x: pt.x,
                    y: pt.y,
                    z: pt.z,
                },
            );
            if sign > EPSILON as f64 {
                FRONT
            } else if sign < -(EPSILON as f64) {
                BACK
            } else {
                COPLANAR
            }
        };

        let mut types = Vec::with_capacity(polygon.vertices.len());
        let mut poly_mask = 0;
        for v in &polygon.vertices {
            let c = classify(&v.pos);
            types.push(c);
            poly_mask |= 1 << (c as u8); // bitmask 1|2 => spanning
        }

        match poly_mask {
            0b001 => {
                // all coplanar
                if n.dot(&polygon.plane.normal()) >= 0.0 {
                    coplanar_front.push(polygon.clone());
                } else {
                    coplanar_back.push(polygon.clone());
                }
            }
            0b010 => front.push(polygon.clone()),
            0b100 => back.push(polygon.clone()),
            _ => {
                // ───── real split ─────
                let mut f: Vec<Vertex> = Vec::new();
                let mut b: Vec<Vertex> = Vec::new();

                for i in 0..polygon.vertices.len() {
                    let j = (i + 1) % polygon.vertices.len();
                    let ti = types[i];
                    let tj = types[j];
                    let vi = &polygon.vertices[i];
                    let vj = &polygon.vertices[j];

                    if ti != BACK {
                        f.push(vi.clone());
                    }
                    if ti != FRONT {
                        b.push(vi.clone());
                    }

                    // edge intersects the plane?
                    if (ti | tj) == SPANNING {
                        let denom = n.dot(&(vj.pos - vi.pos));
                        if denom.abs() > EPSILON {
                            let t = (d - n.dot(&vi.pos.coords)) / denom;
                            let v_new = vi.interpolate(vj, t);
                            f.push(v_new.clone());
                            b.push(v_new);
                        }
                    }
                }

                if f.len() >= 3 {
                    front.push(Polygon::new(f, polygon.metadata.clone()));
                }
                if b.len() >= 3 {
                    back.push(Polygon::new(b, polygon.metadata.clone()));
                }
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
