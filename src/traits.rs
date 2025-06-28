use crate::float_types::parry3d::bounding_volume::Aabb;
use crate::float_types::{EPSILON, Real};
use crate::mesh::plane::Plane;
use nalgebra::{Matrix3, Matrix4, Rotation3, Translation3, Vector3};

/// Boolean operations + transformations
pub trait CSGOps: Sized + Clone {
    fn new() -> Self;
    fn union(&self, other: &Self) -> Self;
    fn difference(&self, other: &Self) -> Self;
    fn intersection(&self, other: &Self) -> Self;
    fn xor(&self, other: &Self) -> Self;
    fn transform(&self, matrix: &Matrix4<Real>) -> Self;
    fn bounding_box(&self) -> Aabb;
    fn invalidate_bounding_box(&mut self);
    fn inverse(&self) -> Self;

    /// Returns a new Self translated by vector.
    fn translate_vector(&self, vector: Vector3<Real>) -> Self {
        self.transform(&Translation3::from(vector).to_homogeneous())
    }

    /// Returns a new Self translated by x, y, and z.
    fn translate(&self, x: Real, y: Real, z: Real) -> Self {
        self.translate_vector(Vector3::new(x, y, z))
    }

    /// Returns a new CSG translated so that its bounding-box center is at the origin (0,0,0).
    fn center(&self) -> Self {
        let aabb = self.bounding_box();

        // Compute the AABB center
        let center_x = (aabb.mins.x + aabb.maxs.x) * 0.5;
        let center_y = (aabb.mins.y + aabb.maxs.y) * 0.5;
        let center_z = (aabb.mins.z + aabb.maxs.z) * 0.5;

        // Translate so that the bounding-box center goes to the origin
        self.translate(-center_x, -center_y, -center_z)
    }

    /// Translates the object so that its bottommost point(s) sit exactly at z=0.
    ///
    /// - Shifts all vertices up or down such that the minimum z coordinate of the bounding box becomes 0.
    ///
    /// # Example
    /// ```
    /// let csg = CSG::cube(1.0, 1.0, 3.0, None).translate(2.0, 1.0, -2.0);
    /// let floated = csg.float();
    /// assert_eq!(floated.bounding_box().mins.z, 0.0);
    /// ```
    fn float(&self) -> Self {
        let aabb = self.bounding_box();
        let min_z = aabb.mins.z;
        self.translate(0.0, 0.0, -min_z)
    }

    /// Rotates the CSG by x_degrees, y_degrees, z_degrees
    fn rotate(&self, x_deg: Real, y_deg: Real, z_deg: Real) -> Self {
        let rx = Rotation3::from_axis_angle(&Vector3::x_axis(), x_deg.to_radians());
        let ry = Rotation3::from_axis_angle(&Vector3::y_axis(), y_deg.to_radians());
        let rz = Rotation3::from_axis_angle(&Vector3::z_axis(), z_deg.to_radians());

        // Compose them in the desired order
        let rot = rz * ry * rx;
        self.transform(&rot.to_homogeneous())
    }

    /// Scales the CSG by scale_x, scale_y, scale_z
    fn scale(&self, sx: Real, sy: Real, sz: Real) -> Self {
        let mat4 = Matrix4::new_nonuniform_scaling(&Vector3::new(sx, sy, sz));
        self.transform(&mat4)
    }

    /// Reflect (mirror) this CSG about an arbitrary plane `plane`.
    ///
    /// The plane is specified by:
    ///   `plane.normal` = the plane’s normal vector (need not be unit),
    ///   `plane.w`      = the dot-product with that normal for points on the plane (offset).
    ///
    /// Returns a new CSG whose geometry is mirrored accordingly.
    fn mirror(&self, plane: Plane) -> Self {
        // Normal might not be unit, so compute its length:
        let len = plane.normal().norm();
        if len.abs() < EPSILON {
            // Degenerate plane? Just return clone (no transform)
            return self.clone();
        }

        // Unit normal:
        let n = plane.normal() / len;
        // Adjusted offset = w / ||n||
        let w = plane.offset() / len;

        // Translate so the plane crosses the origin
        // The plane’s offset vector from origin is (w * n).
        let offset = n * w;
        let t1 = Translation3::from(-offset).to_homogeneous(); // push the plane to origin

        // Build the reflection matrix about a plane normal n at the origin
        // R = I - 2 n n^T
        let mut reflect_4 = Matrix4::identity();
        let reflect_3 = Matrix3::identity() - 2.0 * n * n.transpose();
        reflect_4.fixed_view_mut::<3, 3>(0, 0).copy_from(&reflect_3);

        // Translate back
        let t2 = Translation3::from(offset).to_homogeneous(); // pull the plane back out

        // Combine into a single 4×4
        let mirror_mat = t2 * reflect_4 * t1;

        // Apply to all polygons
        self.transform(&mirror_mat).inverse()
    }

    /// Distribute this CSG `count` times around an arc (in XY plane) of radius,
    /// from `start_angle_deg` to `end_angle_deg`.
    /// Returns a new CSG with all copies (their polygons).
    fn distribute_arc(
        &self,
        count: usize,
        radius: Real,
        start_angle_deg: Real,
        end_angle_deg: Real,
    ) -> Self {
        if count < 1 {
            return self.clone();
        }
        let start_rad = start_angle_deg.to_radians();
        let end_rad = end_angle_deg.to_radians();
        let sweep = end_rad - start_rad;

        // create a container to hold our unioned copies
        let mut all_csg = Self::new();

        for i in 0..count {
            // pick an angle fraction
            let t = if count == 1 {
                0.5
            } else {
                i as Real / ((count - 1) as Real)
            };

            let angle = start_rad + t * sweep;
            let rot =
                nalgebra::Rotation3::from_axis_angle(&nalgebra::Vector3::z_axis(), angle)
                    .to_homogeneous();

            // translate out to radius in x
            let trans = nalgebra::Translation3::new(radius, 0.0, 0.0).to_homogeneous();
            let mat = rot * trans;

            // Transform a copy of self and union with other copies
            all_csg = all_csg.union(&self.transform(&mat));
        }
        all_csg.invalidate_bounding_box();

        all_csg
    }

    /// Distribute this CSG `count` times along a straight line (vector),
    /// each copy spaced by `spacing`.
    /// E.g. if `dir=(1.0,0.0,0.0)` and `spacing=2.0`, you get copies at
    /// x=0, x=2, x=4, ... etc.
    fn distribute_linear(
        &self,
        count: usize,
        dir: nalgebra::Vector3<Real>,
        spacing: Real,
    ) -> Self {
        if count < 1 {
            return self.clone();
        }
        let step = dir.normalize() * spacing;

        // create a container to hold our unioned copies
        let mut all_csg = Self::new();

        for i in 0..count {
            let offset = step * (i as Real);
            let trans = nalgebra::Translation3::from(offset).to_homogeneous();

            // Transform a copy of self and union with other copies
            all_csg = all_csg.union(&self.transform(&trans));
        }
        all_csg.invalidate_bounding_box();

        all_csg
    }

    /// Distribute this CSG in a grid of `rows x cols`, with spacing dx, dy in XY plane.
    /// top-left or bottom-left depends on your usage of row/col iteration.
    fn distribute_grid(&self, rows: usize, cols: usize, dx: Real, dy: Real) -> Self {
        if rows < 1 || cols < 1 {
            return self.clone();
        }
        let step_x = nalgebra::Vector3::new(dx, 0.0, 0.0);
        let step_y = nalgebra::Vector3::new(0.0, dy, 0.0);

        // create a container to hold our unioned copies
        let mut all_csg = Self::new();

        for r in 0..rows {
            for c in 0..cols {
                let offset = step_x * (c as Real) + step_y * (r as Real);
                let trans = nalgebra::Translation3::from(offset).to_homogeneous();

                // Transform a copy of self and union with other copies
                all_csg = all_csg.union(&self.transform(&trans));
            }
        }
        all_csg.invalidate_bounding_box();

        all_csg
    }
}
