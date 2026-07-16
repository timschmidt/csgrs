//! Core constructive solid geometry trait shared by mesh, sketch, and related
//! representations.

use crate::mesh::plane::Plane;
use hyperlattice::{Aabb, Matrix4, Real, Vector3};

/// Build a finite homogeneous translation matrix from a public boundary vector.
///
/// The vector is first promoted through `hyperlattice` so non-finite primitive
/// inputs cannot become CAD transforms. The returned matrix is a
/// hyperlattice-backed homogeneous transform. This follows Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
fn finite_translation(vector: Vector3) -> Option<Matrix4> {
    Some(Matrix4::affine_translation(vector.0))
}

fn finite_scale(sx: Real, sy: Real, sz: Real) -> Option<Matrix4> {
    Some(Matrix4::affine_nonuniform_scale([sx, sy, sz]))
}

fn finite_rotation_x(angle: Real) -> Option<Matrix4> {
    Some(Matrix4::rotation_x(angle))
}

fn finite_rotation_y(angle: Real) -> Option<Matrix4> {
    Some(Matrix4::rotation_y(angle))
}

fn finite_rotation_z(angle: Real) -> Option<Matrix4> {
    Some(Matrix4::rotation_z(angle))
}

pub(crate) fn finite_reflection(plane: &Plane) -> Option<Matrix4> {
    let normal = plane.normal();
    let len = normal.magnitude().ok()?;
    let n = normal.normalize_checked().ok()?;
    let w = (plane.offset() / len).ok()?;
    let [nx, ny, nz] = n.0;
    let two = Real::from(2_u8);
    let zero = Real::zero();
    let one = Real::one();
    Some(Matrix4::from_row_major([
        one.clone() - two.clone() * nx.clone() * nx.clone(),
        -two.clone() * nx.clone() * ny.clone(),
        -two.clone() * nx.clone() * nz.clone(),
        two.clone() * w.clone() * nx.clone(),
        -two.clone() * ny.clone() * nx.clone(),
        one.clone() - two.clone() * ny.clone() * ny.clone(),
        -two.clone() * ny.clone() * nz.clone(),
        two.clone() * w.clone() * ny.clone(),
        -two.clone() * nz.clone() * nx,
        -two.clone() * nz.clone() * ny,
        one.clone() - two.clone() * nz.clone() * nz.clone(),
        two * w * nz,
        zero.clone(),
        zero.clone(),
        zero,
        one,
    ]))
}

fn real_from_usize(value: usize) -> Option<Real> {
    Some(Real::from(u64::try_from(value).ok()?))
}

fn real_half() -> Real {
    (Real::one() / Real::from(2_u8)).expect("nonzero exact denominator")
}

/// Compatibility Boolean operations and shared transformations.
///
/// `Mesh` and `Profile` expose inherent `try_union`, `try_difference`,
/// `try_intersection`, and `try_xor` methods for typed error handling. The
/// Boolean methods on this trait preserve the established infallible API and
/// panic when the corresponding certified operation cannot be completed.
pub trait CSG: Sized + Clone {
    fn union(&self, other: &Self) -> Self;
    fn difference(&self, other: &Self) -> Self;
    fn intersection(&self, other: &Self) -> Self;
    fn xor(&self, other: &Self) -> Self;
    fn transform(&self, matrix: &Matrix4) -> Self;
    fn inverse(&self) -> Self;
    fn bounding_box(&self) -> Aabb;
    fn invalidate_bounding_box(&mut self);

    /// Combine copies produced by a distribution operation.
    ///
    /// Representations may override this internal customization point when
    /// they can certify a cheaper equivalent to a sequence of pairwise
    /// Booleans.
    #[doc(hidden)]
    fn union_distributed(copies: Vec<Self>) -> Self {
        copies
            .into_iter()
            .reduce(|acc, csg| acc.union(&csg))
            .expect("distribution always produces at least one copy")
    }

    /// Returns a new Self translated by vector.
    ///
    /// Non-finite primitive offsets are rejected before constructing a
    /// transform matrix; finite boundary vectors are checked through
    /// `hyperlattice` promotion, following Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    fn translate_vector(&self, vector: Vector3) -> Self {
        let Some(mat) = finite_translation(vector) else {
            return self.clone();
        };
        self.transform(&mat)
    }

    /// Returns a new Self translated by x, y, and z.
    fn translate(&self, x: Real, y: Real, z: Real) -> Self {
        self.translate_vector(Vector3::new([x, y, z]))
    }

    /// Returns a new Self translated so that its bounding-box center is at the origin (0,0,0).
    fn center(&self) -> Self {
        let aabb = self.bounding_box();

        let Some(center) =
            hyperlattice::Point3::centroid(&[aabb.mins.clone(), aabb.maxs.clone()])
        else {
            return self.clone();
        };

        // Translate so that the bounding-box center goes to the origin
        self.translate(-center.x.clone(), -center.y.clone(), -center.z.clone())
    }

    /// Translates Self so that its bottommost point(s) sit exactly at z=0.
    ///
    /// - Shifts all vertices up or down such that the minimum z coordinate of the bounding box becomes 0.
    ///
    /// # Example
    /// ```ignore
    /// use csgrs::mesh::Mesh;
    /// use csgrs::csg::CSG;
    /// let mesh = Mesh::<()>::cube(1.0, ()).translate(2.0, 1.0, -2.0);
    /// let floated = mesh.float();
    /// assert_eq!(floated.bounding_box().mins.z, 0.0);
    /// ```
    fn float(&self) -> Self {
        let aabb = self.bounding_box();
        let min_z = aabb.mins.z.clone();
        self.translate(Real::zero(), Real::zero(), -min_z)
    }

    /// Rotates Self by x_degrees, y_degrees, z_degrees
    ///
    /// Primitive degree inputs are promoted into `Real` for the
    /// degree-to-radian conversion and trigonometric terms while constructing
    /// the homogeneous hyperlattice [`Matrix4`]. This keeps non-finite angles
    /// out of transform carriers and follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The matrix formula is
    /// the same trigonometric rotation formula underlying Rodrigues' theorem;
    /// see Rodrigues, "Des lois géométriques qui régissent les déplacements
    /// d'un système solide dans l'espace," *Journal de Mathématiques Pures et
    /// Appliquées* 5, 1840.
    fn rotate(&self, x_deg: Real, y_deg: Real, z_deg: Real) -> Self {
        let x_rad = x_deg.to_radians();
        let y_rad = y_deg.to_radians();
        let z_rad = z_deg.to_radians();
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
        let Some(mirror_mat) = finite_reflection(&plane) else {
            return self.clone();
        };

        // Apply to all polygons
        self.transform(&mirror_mat).inverse()
    }

    /// Distribute Self `count` times around an arc (in XY plane) of radius,
    /// from `start_angle_deg` to `end_angle_deg`.
    /// Returns a new shape with all copies
    ///
    /// Radius and angle values are accepted through the hyperreal scalar
    /// surface. Primitive numbers are boundary conveniences only.
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
        let sweep = end_rad - start_rad.clone();

        let copies = (0..count)
            .map(|i| {
                let t = if count == 1 {
                    real_half()
                } else {
                    let Some(i) = real_from_usize(i) else {
                        return self.clone();
                    };
                    let Some(denom) = real_from_usize(count - 1) else {
                        return self.clone();
                    };
                    (i / denom).unwrap_or_else(|_| Real::zero())
                };

                let angle = Real::affine(&start_rad, &t, &sweep);
                let Some(rot) = finite_rotation_z(angle) else {
                    return self.clone();
                };

                // translate out to radius in x
                let Some(trans) = finite_translation(Vector3::new([
                    radius.clone(),
                    Real::zero(),
                    Real::zero(),
                ])) else {
                    return self.clone();
                };
                let mat = rot * trans;
                self.transform(&mat)
            })
            .collect();
        Self::union_distributed(copies)
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
    fn distribute_linear(&self, count: usize, dir: Vector3, spacing: Real) -> Self {
        if count < 1 {
            return self.clone();
        }
        let Some(step) = dir.normalize_checked().ok().map(|dir| dir * spacing.clone()) else {
            return self.clone();
        };

        let copies = (0..count)
            .map(|i| {
                let Some(step_index) = real_from_usize(i) else {
                    return self.clone();
                };
                let offset = step.clone() * step_index;
                let Some(trans) = finite_translation(offset) else {
                    return self.clone();
                };
                self.transform(&trans)
            })
            .collect();
        Self::union_distributed(copies)
    }

    /// Distribute Self in a grid of `rows x cols`, with spacing dx, dy in XY plane.
    /// top-left or bottom-left depends on your usage of row/col iteration.
    ///
    /// Non-finite primitive spacings fail closed before transform construction.
    fn distribute_grid(&self, rows: usize, cols: usize, dx: Real, dy: Real) -> Self {
        if rows < 1 || cols < 1 {
            return self.clone();
        }
        let step_x = Vector3::new([dx, Real::zero(), Real::zero()]);
        let step_y = Vector3::new([Real::zero(), dy, Real::zero()]);

        let copies = (0..rows)
            .flat_map(|r| {
                let step_x = step_x.clone();
                let step_y = step_y.clone();
                (0..cols).map(move |c| {
                    let Some(col) = real_from_usize(c) else {
                        return self.clone();
                    };
                    let Some(row) = real_from_usize(r) else {
                        return self.clone();
                    };
                    let offset = step_x.clone() * col + step_y.clone() * row;
                    let Some(trans) = finite_translation(offset) else {
                        return self.clone();
                    };
                    self.transform(&trans)
                })
            })
            .collect();
        Self::union_distributed(copies)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::Mesh;

    #[test]
    fn transforms_accept_hyperreal_primary_scalars_and_integer_promotion() {
        let cube = Mesh::cube(Real::one(), ());
        let cube_bb = cube.bounding_box();

        let translated = cube.translate(Real::from(1), Real::from(2), Real::from(3));
        let translated_bb = translated.bounding_box();
        assert_eq!(translated_bb.mins.x - cube_bb.mins.x, 1.0);
        assert_eq!(translated_bb.mins.y - cube_bb.mins.y, 2.0);
        assert_eq!(translated_bb.mins.z - cube_bb.mins.z, 3.0);

        let scaled = cube.scale(Real::from(2), Real::from(1), Real::from(3));
        let scaled_bb = scaled.bounding_box();
        assert_eq!(scaled_bb.maxs.x - scaled_bb.mins.x, 2.0);
        assert_eq!(scaled_bb.maxs.y - scaled_bb.mins.y, 1.0);
        assert_eq!(scaled_bb.maxs.z - scaled_bb.mins.z, 3.0);

        let rotated = cube.rotate(Real::from(0), Real::from(0), Real::from(90));
        assert_eq!(rotated.polygons.len(), cube.polygons.len());
    }

    #[test]
    fn arbitrary_angle_arc_distribution_unions_disjoint_mesh_copies() {
        let cube = Mesh::cube(Real::one(), ());
        for (start, end) in [(0_u16, 330_u16), (7_u16, 313_u16)] {
            let distributed =
                cube.distribute_arc(12, Real::from(10), Real::from(start), Real::from(end));

            assert_eq!(distributed.polygons.len(), 12 * cube.polygons.len());
        }
    }

    #[test]
    fn distribution_still_booleans_copies_that_are_not_disjoint() {
        let cube = Mesh::cube(Real::one(), ());
        let distributed = cube.distribute_linear(2, Vector3::x(), Real::zero());

        assert_eq!(distributed.polygons.len(), cube.polygons.len());
    }
}
