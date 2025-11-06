use crate::aabb::Aabb;
use crate::mesh::plane::Plane;
use mixed_num::{MixedAbs, MixedPi, MixedZero};
use crate::math_ndsp::{Matrix3, Matrix4, Rotation3, Translation3, Vector3, Scalar, eps};

/// Boolean operations + transformations
pub trait CSG: Sized + Clone {
	type Scalar: Scalar;
	
    fn new() -> Self;
    fn union(&self, other: &Self) -> Self;
    fn difference(&self, other: &Self) -> Self;
    fn intersection(&self, other: &Self) -> Self;
    fn xor(&self, other: &Self) -> Self;
    fn transform(&self, matrix: &Matrix4<Self::Scalar>) -> Self;
    fn inverse(&self) -> Self;
    fn bounding_box(&self) -> Aabb<Self::Scalar>;
    fn invalidate_bounding_box(&mut self);

    /// Returns a new Self translated by vector.
    fn translate_vector(&self, vector: Vector3<Self::Scalar>) -> Self {
        self.transform(&Translation3::from(vector).to_homogeneous())
    }

    /// Returns a new Self translated by x, y, and z.
    fn translate(&self, x: Self::Scalar, y: Self::Scalar, z: Self::Scalar) -> Self {
        self.translate_vector(Vector3::new(x, y, z))
    }

    /// Returns a new Self translated so that its bounding-box center is at the origin (0,0,0).
    fn center(&self) -> Self {
        let aabb = self.bounding_box();
        let c2 = self::Scalar::mixed_from(0.5);

        // Compute the AABB center
        let center_x = (aabb.mins.x + aabb.maxs.x) * c2;
        let center_y = (aabb.mins.y + aabb.maxs.y) * c2;
        let center_z = (aabb.mins.z + aabb.maxs.z) * c2;

        // Translate so that the bounding-box center goes to the origin
        self.translate(-center_x, -center_y, -center_z)
    }

    /// Translates Self so that its bottommost point(s) sit exactly at z=0.
    ///
    /// - Shifts all vertices up or down such that the minimum z coordinate of the bounding box becomes 0.
    ///
    /// # Example
    /// ```
    /// use csgrs::mesh::Mesh;
    /// use crate::csgrs::traits::CSG;
    /// let mesh = Mesh::<()>::cube(1.0, None).translate(2.0, 1.0, -2.0);
    /// let floated = mesh.float();
    /// assert_eq!(floated.bounding_box().mins.z, 0.0);
    /// ```
    fn float(&self) -> Self {
        let aabb = self.bounding_box();
        let min_z = aabb.mins.z;
        self.translate(Self::Scalar::mixed_zero(), Self::Scalar::mixed_zero(), -min_z)
    }

    /// Rotates Self by x_degrees, y_degrees, z_degrees
    fn rotate(&self, x_deg: Self::Scalar, y_deg: Self::Scalar, z_deg: Self::Scalar) -> Self {
		let deg2rad = Self::Scalar::mixed_pi() / Self::Scalar::mixed_from(180.0);
        let rx = Rotation3::from_axis_angle(&Vector3::x_axis(), x_deg * deg2rad);
        let ry = Rotation3::from_axis_angle(&Vector3::y_axis(), y_deg * deg2rad);
        let rz = Rotation3::from_axis_angle(&Vector3::z_axis(), z_deg * deg2rad);

        // Compose them in the desired order
        let rot = rz * ry * rx;
        self.transform(&rot.to_homogeneous())
    }

    /// Scales Self by scale_x, scale_y, scale_z
    fn scale(&self, sx: Self::Scalar, sy: Self::Scalar, sz: Self::Scalar) -> Self {
        let mat4 = Matrix4::scaling(sx, sy, sz);
        self.transform(&mat4)
    }

    /// **Mathematical Foundation: Reflection Across Arbitrary Planes**
    ///
    /// Reflect (mirror) this object about an arbitrary plane `plane`.
    /// This implements the complete mathematical theory of 3D reflections:
    ///
    /// ## **Reflection Mathematics**
    ///
    /// ### **Plane Representation**
    /// The plane is specified by:
    /// - `plane.normal` = the plane's normal vector n⃗ (need not be unit)
    /// - `plane.offset` = the signed distance d from origin to plane
    /// - **Plane Equation**: n⃗·p⃗ + d = 0
    ///
    /// ### **Reflection Matrix Derivation**
    /// For a unit normal n̂ and plane through origin, the reflection matrix is:
    /// ```text
    /// R = I - 2n̂n̂ᵀ
    /// ```
    /// **Proof**: For any vector v⃗, the reflection is:
    /// - **Component parallel to n̂**: v∥ = (v⃗·n̂)n̂  → reflected to -v∥
    /// - **Component perpendicular**: v⊥ = v⃗ - v∥  → unchanged
    /// - **Result**: v'⃗ = v⊥ - v∥ = v⃗ - 2(v⃗·n̂)n̂ = (I - 2n̂n̂ᵀ)v⃗
    ///
    /// ### **General Plane Reflection Algorithm**
    /// 1. **Normalize**: n̂ = n⃗/|n⃗|, d̂ = d/|n⃗|
    /// 2. **Translate to Origin**: T₁ = translate by -d̂n̂
    /// 3. **Reflect at Origin**: R = I - 2n̂n̂ᵀ
    /// 4. **Translate Back**: T₂ = translate by +d̂n̂
    /// 5. **Compose**: M = T₂ · R · T₁
    ///
    /// ### **Normal Vector Transformation**
    /// Normals transform by the inverse transpose: n'⃗ = (M⁻¹)ᵀn⃗
    /// For reflections, this simplifies to the same matrix M.
    ///
    /// ## **Geometric Properties**
    /// - **Isometry**: Preserves distances and angles
    /// - **Orientation Reversal**: Changes handedness (det(M) = -1)
    /// - **Involution**: M² = I (reflecting twice gives identity)
    /// - **Plane Invariance**: Points on the plane remain fixed
    ///
    /// **Note**: The result is inverted (.inverse()) because reflection reverses
    /// the orientation of polygons, affecting inside/outside semantics in CSG.
    ///
    /// Returns a new Self whose geometry is mirrored accordingly.
    fn mirror(&self, plane: Plane<Self::Scalar>) -> Self {
        // Normal might not be unit, so compute its length:
        let len = plane.normal().norm();
        if len.mixed_abs() <= eps::<Self::Scalar>() {
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
        //let mut reflect_4 = Matrix4::identity();
        //let reflect_3 = Matrix3::identity() - 2.0 * n * n.transpose();
        let reflect_3 = Matrix3::reflection_about_unit_normal(n);
        let r4 = Matrix4::from_matrix3_and_translation(reflect_3, Vector3::mixed_zeroes());
        //reflect_4.fixed_view_mut::<3, 3>(0, 0).copy_from(&reflect_3);

        // Translate back
        let t2 = Translation3::from(offset).to_homogeneous(); // pull the plane back out

        // Combine into a single 4×4
        let mirror_mat = t2 * r4 * t1;

        // Apply to all polygons
        self.transform(&mirror_mat).inverse()
    }

    /// Distribute Self `count` times around an arc (in XY plane) of radius,
    /// from `start_angle_deg` to `end_angle_deg`.
    /// Returns a new shape with all copies
    fn distribute_arc(
        &self,
        count: usize,
        radius: Self::Scalar,
        start_angle_deg: Self::Scalar,
        end_angle_deg: Self::Scalar,
    ) -> Self {
        if count < 1 {
            return self.clone();
        }
        let deg2rad = Self::Scalar::mixed_pi() / Self::Scalar::mixed_from(180.0);
        let start_rad = start_angle_deg * deg2rad;
        let end_rad = end_angle_deg * deg2rad;
        let sweep = end_rad - start_rad;

        (0..count)
            .map(|i| {
                let t = if count == 1 {
                    Self::Scalar::mixed_from(0.5)
                } else {
                    Self::Scalar::mixed_from(i)
                        / Self::Scalar::mixed_from((count - 1))
                };

                let angle = start_rad + t * sweep;
                let rot =
                    Rotation3::from_axis_angle(&Vector3::z_axis(), angle)
                        .to_homogeneous();

                // translate out to radius in x
                let trans = Translation3::new(radius, Self::Scalar::mixed_zero(), Self::Scalar::mixed_zero()).to_homogeneous();
                let mat = rot * trans;
                self.transform(&mat)
            })
            .reduce(|acc, csg| acc.union(&csg))
            .unwrap()
    }

    /// Distribute Self `count` times along a straight line (vector),
    /// each copy spaced by `spacing`.
    /// E.g. if `dir=(1.0,0.0,0.0)` and `spacing=2.0`, you get copies at
    /// x=0, x=2, x=4, ... etc.
    fn distribute_linear(
        &self,
        count: usize,
        dir: Vector3<Self::Scalar>,
        spacing: Self::Scalar,
    ) -> Self {
        if count < 1 {
            return self.clone();
        }
        let step = dir.normalize() * spacing;

        (0..count)
            .map(|i| {
                let offset = Self::Scalar::mixed_from(i);
                let trans = Translation3::from(offset * step).to_homogeneous();
                self.transform(&trans)
            })
            .reduce(|acc, csg| acc.union(&csg))
            .unwrap()
    }

    /// Distribute Self in a grid of `rows x cols`, with spacing dx, dy in XY plane.
    /// top-left or bottom-left depends on your usage of row/col iteration.
    fn distribute_grid(&self, rows: usize, cols: usize, dx: Self::Scalar, dy: Self::Scalar) -> Self {
        if rows < 1 || cols < 1 {
            return self.clone();
        }
        let step_x = Vector3::new(dx, Self::Scalar::mixed_zero(), Self::Scalar::mixed_zero());
        let step_y = Vector3::new(Self::Scalar::mixed_zero(), dy, Self::Scalar::mixed_zero());

        (0..rows)
            .flat_map(|r| {
                (0..cols).map(move |c| {
                    let rx = Self::Scalar::mixed_from(c);
                    let ry = Self::Scalar::mixed_from(r);
                    let offset = step_x * rx + step_y * ry;
                    let trans = Translation3::from(offset).to_homogeneous();
                    self.transform(&trans)
                })
            })
            .reduce(|acc, csg| acc.union(&csg))
            .unwrap()
    }
}
