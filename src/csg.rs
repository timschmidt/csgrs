//! Core constructive solid geometry trait shared by mesh, sketch, and related
//! representations.

use crate::float_types::parry3d::bounding_volume::Aabb;
use crate::float_types::{
    Real, hangle_sin_cos, hdegrees_to_radians, hpoint_centroid, hreal_affine, hreal_div,
    hreal_mul, hreal_sub, hscale_matrix, htranslation_matrix, hunit_vector3,
    hunit_vector3_and_magnitude, hvector3_weighted_sum,
};
use crate::mesh::plane::Plane;
use nalgebra::{Matrix4, Vector3};

/// Build a finite homogeneous translation matrix from a public boundary vector.
///
/// The vector is first promoted through `hyperlattice` so non-finite primitive
/// inputs cannot become CAD transforms. The returned `Matrix4<f64>` is still
/// the transitional carrier, but csgrs no longer depends on nalgebra's
/// translation constructor for this API boundary. This follows Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
fn finite_translation(vector: Vector3<Real>) -> Option<Matrix4<Real>> {
    htranslation_matrix(&vector)
}

fn finite_scale(sx: Real, sy: Real, sz: Real) -> Option<Matrix4<Real>> {
    hscale_matrix(sx, sy, sz)
}

fn finite_rotation_x(angle: Real) -> Option<Matrix4<Real>> {
    let (sin, cos) = hangle_sin_cos(angle)?;
    Some(Matrix4::new(
        1.0, 0.0, 0.0, 0.0, 0.0, cos, -sin, 0.0, 0.0, sin, cos, 0.0, 0.0, 0.0, 0.0, 1.0,
    ))
}

fn finite_rotation_y(angle: Real) -> Option<Matrix4<Real>> {
    let (sin, cos) = hangle_sin_cos(angle)?;
    Some(Matrix4::new(
        cos, 0.0, sin, 0.0, 0.0, 1.0, 0.0, 0.0, -sin, 0.0, cos, 0.0, 0.0, 0.0, 0.0, 1.0,
    ))
}

fn finite_rotation_z(angle: Real) -> Option<Matrix4<Real>> {
    let (sin, cos) = hangle_sin_cos(angle)?;
    Some(Matrix4::new(
        cos, -sin, 0.0, 0.0, sin, cos, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ))
}

/// Boolean operations + transformations
pub trait CSG: Sized + Clone {
    fn union(&self, other: &Self) -> Self;
    fn difference(&self, other: &Self) -> Self;
    fn intersection(&self, other: &Self) -> Self;
    fn xor(&self, other: &Self) -> Self;
    fn transform(&self, matrix: &Matrix4<Real>) -> Self;
    fn inverse(&self) -> Self;
    fn bounding_box(&self) -> Aabb;
    fn invalidate_bounding_box(&mut self);

    /// Returns a new Self translated by vector.
    ///
    /// Non-finite primitive offsets are rejected before constructing a
    /// transform matrix; finite boundary vectors are checked through
    /// `hyperlattice` promotion, following Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    fn translate_vector(&self, vector: Vector3<Real>) -> Self {
        let Some(mat) = finite_translation(vector) else {
            return self.clone();
        };
        self.transform(&mat)
    }

    /// Returns a new Self translated by x, y, and z.
    fn translate(&self, x: Real, y: Real, z: Real) -> Self {
        self.translate_vector(Vector3::new(x, y, z))
    }

    /// Returns a new Self translated so that its bounding-box center is at the origin (0,0,0).
    fn center(&self) -> Self {
        let aabb = self.bounding_box();

        let Some(center) = hpoint_centroid(&[aabb.mins, aabb.maxs]) else {
            return self.clone();
        };

        // Translate so that the bounding-box center goes to the origin
        self.translate(-center.x, -center.y, -center.z)
    }

    /// Translates Self so that its bottommost point(s) sit exactly at z=0.
    ///
    /// - Shifts all vertices up or down such that the minimum z coordinate of the bounding box becomes 0.
    ///
    /// # Example
    /// ```
    /// use csgrs::mesh::Mesh;
    /// use csgrs::csg::CSG;
    /// let mesh = Mesh::<()>::cube(1.0, ()).translate(2.0, 1.0, -2.0);
    /// let floated = mesh.float();
    /// assert_eq!(floated.bounding_box().mins.z, 0.0);
    /// ```
    fn float(&self) -> Self {
        let aabb = self.bounding_box();
        let min_z = aabb.mins.z;
        self.translate(0.0, 0.0, -min_z)
    }

    /// Rotates Self by x_degrees, y_degrees, z_degrees
    ///
    /// Primitive degree inputs are promoted into `hyperreal::Real` for the
    /// degree-to-radian conversion and trigonometric terms before returning to
    /// the current homogeneous `Matrix4<f64>` boundary. This keeps non-finite
    /// angles out of transform carriers and follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The matrix formula is
    /// the same trigonometric rotation formula underlying Rodrigues' theorem;
    /// see Rodrigues, "Des lois géométriques qui régissent les déplacements
    /// d'un système solide dans l'espace," *Journal de Mathématiques Pures et
    /// Appliquées* 5, 1840.
    fn rotate(&self, x_deg: Real, y_deg: Real, z_deg: Real) -> Self {
        let Some(x_rad) = hdegrees_to_radians(x_deg) else {
            return self.clone();
        };
        let Some(y_rad) = hdegrees_to_radians(y_deg) else {
            return self.clone();
        };
        let Some(z_rad) = hdegrees_to_radians(z_deg) else {
            return self.clone();
        };
        let Some(rx) = finite_rotation_x(x_rad) else {
            return self.clone();
        };
        let Some(ry) = finite_rotation_y(y_rad) else {
            return self.clone();
        };
        let Some(rz) = finite_rotation_z(z_rad) else {
            return self.clone();
        };

        self.transform(&(rz * ry * rx))
    }

    /// Scales Self by scale_x, scale_y, scale_z
    ///
    /// Non-finite primitive scale factors leave geometry unchanged; exact-aware
    /// geometry should only receive finite transform carriers.
    fn scale(&self, sx: Real, sy: Real, sz: Real) -> Self {
        let Some(mat4) = finite_scale(sx, sy, sz) else {
            return self.clone();
        };
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
    /// The normal length and unit vector are computed through `hyperlattice`
    /// rather than direct primitive normalization. Invalid or degenerate plane
    /// normals leave the object unchanged, keeping finite boundary data outside
    /// topology-sensitive CAD decisions. This follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The reflection matrix
    /// is the Householder form; see Householder, "Unitary Triangularization of a
    /// Nonsymmetric Matrix," *Journal of the ACM* 5(4), 1958
    /// (<https://doi.org/10.1145/320941.320947>).
    ///
    /// Returns a new Self whose geometry is mirrored accordingly.
    fn mirror(&self, plane: Plane) -> Self {
        let Some((n, len)) = hunit_vector3_and_magnitude(&plane.normal()) else {
            // Degenerate or hostile plane: leave geometry unchanged.
            return self.clone();
        };
        // Adjusted offset = w / ||n||
        let Some(w) = hreal_div(plane.offset(), len) else {
            return self.clone();
        };

        // Direct Householder reflection for `n . p == w`:
        // p' = (I - 2 n n^T) p + 2 w n.
        let mirror_mat = Matrix4::new(
            1.0 - 2.0 * n.x * n.x,
            -2.0 * n.x * n.y,
            -2.0 * n.x * n.z,
            2.0 * w * n.x,
            -2.0 * n.y * n.x,
            1.0 - 2.0 * n.y * n.y,
            -2.0 * n.y * n.z,
            2.0 * w * n.y,
            -2.0 * n.z * n.x,
            -2.0 * n.z * n.y,
            1.0 - 2.0 * n.z * n.z,
            2.0 * w * n.z,
            0.0,
            0.0,
            0.0,
            1.0,
        );

        // Apply to all polygons
        self.transform(&mirror_mat).inverse()
    }

    /// Distribute Self `count` times around an arc (in XY plane) of radius,
    /// from `start_angle_deg` to `end_angle_deg`.
    /// Returns a new shape with all copies
    ///
    /// Non-finite primitive radius or angle values fail closed before transform
    /// construction.
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
        if !radius.is_finite() || !start_angle_deg.is_finite() || !end_angle_deg.is_finite() {
            return self.clone();
        }
        let Some(start_rad) = hdegrees_to_radians(start_angle_deg) else {
            return self.clone();
        };
        let Some(end_rad) = hdegrees_to_radians(end_angle_deg) else {
            return self.clone();
        };
        let Some(sweep) = hreal_sub(end_rad, start_rad) else {
            return self.clone();
        };

        (0..count)
            .map(|i| {
                let t = if count == 1 {
                    0.5
                } else {
                    hreal_div(i as Real, (count - 1) as Real).unwrap_or(0.0)
                };

                let Some(angle) = hreal_affine(start_rad, t, sweep) else {
                    return self.clone();
                };
                let Some(rot) = finite_rotation_z(angle) else {
                    return self.clone();
                };

                // translate out to radius in x
                let Some(trans) = finite_translation(Vector3::new(radius, 0.0, 0.0)) else {
                    return self.clone();
                };
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
    ///
    /// Direction normalization is promoted to `hyperlattice::Vector3` and fails
    /// closed for zero or non-finite inputs. This keeps the default CSG API's
    /// primitive vector boundary aligned with Yap's exact-geometric-computation
    /// discipline (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    fn distribute_linear(&self, count: usize, dir: Vector3<Real>, spacing: Real) -> Self {
        if count < 1 {
            return self.clone();
        }
        if !spacing.is_finite() {
            return self.clone();
        }
        let Some(step) = hunit_vector3(&dir).map(|dir| dir * spacing) else {
            return self.clone();
        };

        (0..count)
            .map(|i| {
                let Some(step_index) = hreal_mul(1.0, i as Real) else {
                    return self.clone();
                };
                let Some(offset) = hvector3_weighted_sum(&[step], &[step_index]) else {
                    return self.clone();
                };
                let Some(trans) = finite_translation(offset) else {
                    return self.clone();
                };
                self.transform(&trans)
            })
            .reduce(|acc, csg| acc.union(&csg))
            .unwrap()
    }

    /// Distribute Self in a grid of `rows x cols`, with spacing dx, dy in XY plane.
    /// top-left or bottom-left depends on your usage of row/col iteration.
    ///
    /// Non-finite primitive spacings fail closed before transform construction.
    fn distribute_grid(&self, rows: usize, cols: usize, dx: Real, dy: Real) -> Self {
        if rows < 1 || cols < 1 {
            return self.clone();
        }
        if !dx.is_finite() || !dy.is_finite() {
            return self.clone();
        }
        let step_x = Vector3::new(dx, 0.0, 0.0);
        let step_y = Vector3::new(0.0, dy, 0.0);

        (0..rows)
            .flat_map(|r| {
                (0..cols).map(move |c| {
                    let Some(col) = hreal_mul(1.0, c as Real) else {
                        return self.clone();
                    };
                    let Some(row) = hreal_mul(1.0, r as Real) else {
                        return self.clone();
                    };
                    let Some(offset) = hvector3_weighted_sum(&[step_x, step_y], &[col, row])
                    else {
                        return self.clone();
                    };
                    let Some(trans) = finite_translation(offset) else {
                        return self.clone();
                    };
                    self.transform(&trans)
                })
            })
            .reduce(|acc, csg| acc.union(&csg))
            .unwrap()
    }
}
