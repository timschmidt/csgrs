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
//! **Robust Geometric Predicates**: This implementation promotes finite boundary
//! coordinates to `hyperreal::Real` before making orientation and splitting
//! topology decisions. The predicate computes the determinant:
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
//! - **Robust Predicates**: Uses hyperreal arithmetic for orientation tests,
//!   with the current BSP tolerance applied as a hyperreal threshold
//! - **Boundary Tolerances**: Governed by `float_types::tolerance()` where the
//!   legacy f64 API still needs degeneracy filters
//! - **Degenerate Case Handling**: Proper fallbacks for collinear points and zero-area triangles
//!
//! The predicate split follows Yap's exact geometric computation model
//! (*Computational Geometry* 7(1-2), 1997,
//! <https://doi.org/10.1016/0925-7721(95)00040-2>) and the robust-predicate
//! motivation from Shewchuk, "Adaptive Precision Floating-Point Arithmetic and
//! Fast Robust Geometric Predicates" (*Discrete & Computational Geometry*
//! 18(3), 1997, <https://doi.org/10.1007/PL00009321>). The clipping workflow is
//! the Sutherland-Hodgman polygon clipping algorithm (Communications of the ACM
//! 17(1), 1974, <https://doi.org/10.1145/360767.360802>).
//!
//! ## **Algorithm Complexity**
//!
//! - **Plane Construction**: O(n²) for optimal triangle selection, O(1) for basic construction
//! - **Orientation Testing**: O(1) expression construction per point; sign
//!   refinement depends on the hyperreal expression
//! - **Polygon Splitting**: O(n) per polygon, where n is the number of vertices
//!
//! Unless stated otherwise, boundary degeneracy filters are governed by
//! `float_types::tolerance()`.

use crate::float_types::{
    HReal, Real, hperpendicular_basis, hreal_from_f64, hreal_gt_f64, hreal_sign, hreal_to_f64,
    hrotation_between_vectors, htranslation_matrix, hunit_vector3, hvector3_dot,
    hvector3_from_point3, hvector3_from_vector3, tolerance,
};
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use hyperlattice::Vector3 as HVector3;
use hyperreal::RealSign;
use nalgebra::{Matrix4, Point3, Vector3};
use std::cmp::Ordering;

/// Classification of a polygon or point whose hyperreal orientation determinant
/// is inside the current BSP tolerance band, or whose boundary coordinates
/// cannot be promoted.
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
pub struct Plane {
    pub point_a: Point3<Real>,
    pub point_b: Point3<Real>,
    pub point_c: Point3<Real>,
}

fn hreal_sign_with_tolerance(value: &HReal) -> Option<RealSign> {
    let tolerance = hreal_from_f64(tolerance()).ok()?;
    if matches!(
        hreal_sign(&(value.clone() - tolerance.clone())),
        Some(RealSign::Positive)
    ) {
        return Some(RealSign::Positive);
    }
    if matches!(
        hreal_sign(&(value.clone() + tolerance)),
        Some(RealSign::Negative)
    ) {
        return Some(RealSign::Negative);
    }
    Some(RealSign::Zero)
}

fn hreal_cmp_or_equal(lhs: &HReal, rhs: &HReal) -> Ordering {
    match hreal_sign(&(lhs.clone() - rhs.clone())) {
        Some(RealSign::Positive) => Ordering::Greater,
        Some(RealSign::Negative) => Ordering::Less,
        Some(RealSign::Zero) | None => Ordering::Equal,
    }
}

fn hlimit_point3_from_point3(point: &Point3<Real>) -> Option<hyperlimit::Point3> {
    hyperlimit::Point3::try_from_f64_array([point.x, point.y, point.z]).ok()
}

fn newell_hreal_normal(points: &[HVector3]) -> HVector3 {
    points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .fold(HVector3::zero(), |acc, (curr, next)| acc + curr.cross(next))
}

impl PartialEq for Plane {
    fn eq(&self, other: &Self) -> bool {
        if self.point_a == other.point_a
            && self.point_b == other.point_b
            && self.point_c == other.point_c
        {
            true
        } else {
            // check if co-planar
            self.orient_point(&other.point_a) == COPLANAR
                && self.orient_point(&other.point_b) == COPLANAR
                && self.orient_point(&other.point_c) == COPLANAR
        }
    }
}

impl Plane {
    /// Tries to pick three vertices that span the largest area triangle
    /// (maximally well-spaced) and returns a plane defined by them.
    /// Care is taken to preserve the original winding of the vertices.
    ///
    /// Candidate chord lengths and line-area metrics are evaluated as
    /// `hyperlattice::Vector3<hyperreal::Real>` expressions, then the winning
    /// support vertices are projected back to the current mesh boundary type.
    /// This follows Yap's exact-geometric-computation separation between
    /// primitive input coordinates and exact-aware topology predicates
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    ///
    /// Cost: O(n^2)
    /// A lower cost option may be a grid sub-sampled farthest pair search
    pub fn from_vertices(vertices: Vec<Vertex>) -> Plane {
        let n = vertices.len();
        let reference_plane = Plane {
            point_a: vertices[0].position,
            point_b: vertices[1].position,
            point_c: vertices[2].position,
        };
        if n == 3 {
            return reference_plane;
        } // Plane is already optimal

        let Some(hpoints) = vertices
            .iter()
            .map(|vertex| hvector3_from_point3(&vertex.position))
            .collect::<Option<Vec<_>>>()
        else {
            return reference_plane;
        };

        // longest chord (i0,i1)
        let Some((i0, i1, _)) = (0..n)
            .flat_map(|i| (i + 1..n).map(move |j| (i, j)))
            .map(|(i, j)| {
                let d2 = hpoints[i].squared_distance(&hpoints[j]);
                (i, j, d2)
            })
            .max_by(|a, b| hreal_cmp_or_equal(&a.2, &b.2))
        else {
            return reference_plane;
        };

        let p0 = vertices[i0].position;
        let p1 = vertices[i1].position;
        let hdir = &hpoints[i1] - &hpoints[i0];
        let hdir2 = hdir.dot(&hdir);
        if !hreal_gt_f64(&hdir2, tolerance() * tolerance()) {
            return reference_plane; // everything almost coincident
        }

        // vertex farthest from the line  p0-p1  → i2
        let Some((i2, max_area2)) = vertices
            .iter()
            .enumerate()
            .filter(|(idx, _)| *idx != i0 && *idx != i1)
            .map(|(idx, _)| {
                let offset = &hpoints[idx] - &hpoints[i0];
                let area = offset.cross(&hdir);
                let a2 = area.dot(&area); // ∝ area²
                (idx, a2)
            })
            .max_by(|a, b| hreal_cmp_or_equal(&a.1, &b.1))
        else {
            return reference_plane;
        };

        let i2 = if hreal_gt_f64(&max_area2, tolerance() * tolerance()) {
            i2
        } else {
            return reference_plane; // all vertices collinear
        };
        let p2 = vertices[i2].position;

        // build plane, then orient it to match original winding
        let mut plane_hq = Plane {
            point_a: p0,
            point_b: p1,
            point_c: p2,
        };

        // Construct the reference normal for the original polygon using Newell's Method.
        let reference_normal = newell_hreal_normal(&hpoints);
        let should_flip = plane_hq
            .unit_hreal_normal()
            .map(|plane_normal| plane_normal.dot(&reference_normal))
            .and_then(|dot| hreal_sign(&dot))
            .is_some_and(|sign| matches!(sign, RealSign::Negative));

        if should_flip {
            plane_hq.flip(); // flip in-place to agree with winding
        }
        plane_hq
    }

    /// Build a new `Plane` from a (not-necessarily-unit) normal **n**
    /// and signed offset *o* (in the sense `n · p == o`).
    ///
    /// Invalid primitive inputs produce the canonical XY plane rather than a
    /// panic. Call [`Plane::try_from_normal`] when the caller needs to
    /// distinguish a rejected boundary normal from a real XY plane. The
    /// normal-squared degeneracy check and point-on-plane scale `offset / (n · n)` are
    /// evaluated in `hyperlattice::Vector3`/`hyperreal::Real`; the resulting
    /// support point is exported to finite coordinates only because `Plane`
    /// remains a mesh API-boundary type. This follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), while retaining the
    /// ordinary implicit plane equation used by the Sutherland-Hodgman split
    /// path (<https://doi.org/10.1145/360767.360802>).
    pub fn from_normal(normal: Vector3<Real>, offset: Real) -> Self {
        Self::try_from_normal(normal, offset).unwrap_or_else(Self::xy)
    }

    /// Checked plane construction from a public finite normal and offset.
    ///
    /// This is the fallible form of [`Plane::from_normal`]. It keeps primitive
    /// API data out of plane topology unless both the normal and support point
    /// can be represented through hyperlattice/hyperreal.
    pub fn try_from_normal(normal: Vector3<Real>, offset: Real) -> Option<Self> {
        let hyper_normal = hvector3_from_vector3(&normal)?;
        let n2 = hyper_normal.dot(&hyper_normal);
        if !hreal_gt_f64(&n2, tolerance() * tolerance()) {
            return None;
        }

        let scale = (hreal_from_f64(offset).ok()? / n2).ok()?;
        let p0_h = hyper_normal * &scale;
        let p0_coords = p0_h.to_f64_array_lossy()?;

        // Point on the plane:  p0 = n * o / (n·n)
        let p0 = Point3::new(p0_coords[0], p0_coords[1], p0_coords[2]);

        // Build an orthonormal basis {u, v} in hyperlattice and export only
        // the finite boundary vectors used to define the public plane carrier.
        let (u, v) = hperpendicular_basis(&normal)?;

        // Use p0, p0+u, p0+v  as the three defining points.
        Some(Self {
            point_a: p0,
            point_b: p0 + u,
            point_c: p0 + v,
        })
    }

    fn xy() -> Self {
        Self {
            point_a: Point3::origin(),
            point_b: Point3::new(1.0, 0.0, 0.0),
            point_c: Point3::new(0.0, 1.0, 0.0),
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
        let Some(det) = self.orient_point_hreal(point) else {
            return COPLANAR;
        };
        match hreal_sign_with_tolerance(&det) {
            Some(RealSign::Positive) => self.orient_point_hyperlimit(point).unwrap_or(FRONT),
            Some(RealSign::Negative) => self.orient_point_hyperlimit(point).unwrap_or(BACK),
            Some(RealSign::Zero) | None => COPLANAR,
        }
    }

    /// Return the (right‑handed) checked unit normal **n** of the plane
    /// `checked_unit((b-a) x (c-a))`.
    ///
    /// The cross product and checked normalization are evaluated with
    /// `hyperlattice::Vector3<hyperreal::Real>` and only exported to `f64` at
    /// the mesh API boundary. This keeps the plane normal on the same exact
    /// geometric-computation path as orientation and splitting predicates
    /// (Yap, *Computational Geometry* 7(1-2), 1997,
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>). Degenerate or
    /// non-promotable support points fail closed to the zero normal instead of
    /// re-entering local primitive normalization.
    #[inline]
    pub fn normal(&self) -> Vector3<Real> {
        if let Some(normal) = self.unit_hreal_normal()
            && let Some([x, y, z]) = normal.to_f64_array_lossy()
        {
            return Vector3::new(x, y, z);
        }

        Vector3::zeros()
    }

    /// Signed offset of the plane from the origin: `d = n · a`.
    ///
    /// The dot product is evaluated through the shared hyperlattice boundary
    /// adapter instead of local primitive vector arithmetic, following Yap's
    /// exact-geometric-computation split between finite carriers and exact-aware
    /// predicates (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    #[inline]
    pub fn offset(&self) -> Real {
        hvector3_dot(&self.normal(), &self.point_a.coords).unwrap_or(0.0)
    }

    pub const fn flip(&mut self) {
        std::mem::swap(&mut self.point_a, &mut self.point_b);
    }

    /// Classify a polygon with respect to the plane.
    /// Returns a bitmask of `COPLANAR`, `FRONT`, and `BACK`.
    pub fn classify_polygon<M: Clone>(&self, polygon: &Polygon<M>) -> i8 {
        let mut polygon_type: i8 = 0;
        for vertex in &polygon.vertices {
            polygon_type |= self.orient_point(&vertex.position);
        }
        polygon_type
    }

    fn orient_point_hreal(&self, point: &Point3<Real>) -> Option<HReal> {
        let a = hvector3_from_point3(&self.point_a)?;
        let b = hvector3_from_point3(&self.point_b)?;
        let c = hvector3_from_point3(&self.point_c)?;
        let d = hvector3_from_point3(point)?;
        let ab = &b - &a;
        let ac = &c - &a;
        let ad = &d - &a;
        Some(ab.cross(&ac).dot(&ad))
    }

    fn orient_point_hyperlimit(&self, point: &Point3<Real>) -> Option<i8> {
        let a = hlimit_point3_from_point3(&self.point_a)?;
        let b = hlimit_point3_from_point3(&self.point_b)?;
        let c = hlimit_point3_from_point3(&self.point_c)?;
        let d = hlimit_point3_from_point3(point)?;
        match hyperlimit::orient3d(&a, &b, &c, &d).value()? {
            hyperlimit::Sign::Positive => Some(BACK),
            hyperlimit::Sign::Negative => Some(FRONT),
            hyperlimit::Sign::Zero => Some(COPLANAR),
        }
    }

    fn unscaled_hreal_normal(&self) -> Option<HVector3> {
        let a = hvector3_from_point3(&self.point_a)?;
        let b = hvector3_from_point3(&self.point_b)?;
        let c = hvector3_from_point3(&self.point_c)?;
        Some((&b - &a).cross(&(&c - &a)))
    }

    fn unit_hreal_normal(&self) -> Option<HVector3> {
        self.unscaled_hreal_normal()?.normalize_checked().ok()
    }

    fn edge_intersection_parameter(
        &self,
        start: &Vertex,
        end: &Vertex,
        normal: Option<&HVector3>,
    ) -> Option<Real> {
        let normal = normal?;
        let plane_point = hvector3_from_point3(&self.point_a)?;
        let start_point = hvector3_from_point3(&start.position)?;
        let end_point = hvector3_from_point3(&end.position)?;
        let direction = &end_point - &start_point;
        let denom = normal.dot(&direction);
        if !matches!(
            hreal_sign(&denom),
            Some(RealSign::Positive | RealSign::Negative)
        ) {
            return None;
        }

        let numerator = normal.dot(&(&plane_point - &start_point));
        let parameter = (numerator / denom).ok()?;
        hreal_to_f64(&parameter).filter(|parameter| parameter.is_finite())
    }

    /// Intersect an edge with this plane using the same hyperreal point-normal
    /// algebra as polygon splitting.
    ///
    /// The returned vertex is still an API-boundary `f64` [`Vertex`] because
    /// the mesh data model has not been fully moved to hyper geometry yet, but
    /// the topological decision and line-plane parameter are evaluated in
    /// `hyperreal::Real` through `hyperlattice::Vector3`. Keeping the
    /// determinant and dot-product work in the hyper geometry layer follows
    /// Yap's exact-geometric-computation split between geometric objects and
    /// scalar arithmetic (<https://doi.org/10.1016/0925-7721(95)00040-2>);
    /// the line-plane solve is the Sutherland-Hodgman clipping intersection
    /// used by this module (<https://doi.org/10.1145/360767.360802>).
    pub(crate) fn intersect_edge(&self, start: &Vertex, end: &Vertex) -> Option<Vertex> {
        let normal = self.unscaled_hreal_normal()?;
        let parameter = self.edge_intersection_parameter(start, end, Some(&normal))?;
        Some(start.interpolate(end, parameter))
    }

    /// Splits a polygon by this plane, returning four buckets:
    /// `(coplanar_front, coplanar_back, front, back)`.
    #[allow(clippy::type_complexity)]
    pub fn split_polygon<M: Clone + Send + Sync>(
        &self,
        polygon: &Polygon<M>,
    ) -> (
        Vec<Polygon<M>>,
        Vec<Polygon<M>>,
        Vec<Polygon<M>>,
        Vec<Polygon<M>>,
    ) {
        let mut coplanar_front = Vec::new();
        let mut coplanar_back = Vec::new();
        let mut front = Vec::new();
        let mut back = Vec::new();

        let normal = self.normal();
        let hnormal = self.unscaled_hreal_normal();

        let types: Vec<i8> = polygon
            .vertices
            .iter()
            .map(|v| self.orient_point(&v.position))
            .collect();
        let polygon_type = types.iter().fold(0, |acc, &t| acc | t);

        // -----------------------------------------------------------------
        // 2.  dispatch the easy cases
        // -----------------------------------------------------------------
        match polygon_type {
            COPLANAR => {
                if hvector3_dot(&normal, &polygon.plane.normal()).unwrap_or(0.0) > 0.0 {
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
                        if let Some(intersection) = self.edge_intersection_parameter(
                            vertex_i,
                            vertex_j,
                            hnormal.as_ref(),
                        ) {
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
    /// 1. **Rotation Matrix**: R maps `plane_normal` to +Z with hyperlattice
    ///    unit-vector, dot-product, and cross-product checks.
    /// 2. **Translation**: Translate so plane passes through origin
    /// 3. **Combined Transform**: T = T₂ · R · T₁
    ///
    /// The transformation preserves distances and angles, enabling 2D algorithms
    /// to be applied to 3D planar geometry.
    pub fn to_xy_transform(&self) -> (Matrix4<Real>, Matrix4<Real>) {
        // Normal
        let Some(norm_dir) = hunit_vector3(&self.normal()) else {
            // Degenerate plane, return identity
            return (Matrix4::identity(), Matrix4::identity());
        };

        // Rotate plane.normal -> +Z.
        let Some(rot) = hrotation_between_vectors(&norm_dir, &Vector3::z()) else {
            return (Matrix4::identity(), Matrix4::identity());
        };

        // We want to translate so that the plane's reference point
        //    (some point p0 with n·p0 = w) lands at z=0 in the new coords.
        // `norm_dir` is already checked as a unit vector in hyperlattice.
        let p0_3d = norm_dir * self.offset();
        let p0_rot = Point3::from_homogeneous(rot * Point3::from(p0_3d).to_homogeneous())
            .unwrap_or_else(Point3::origin);

        // We want p0_rot.z = 0, so we shift by -p0_rot.z
        let shift_z = -p0_rot.z;
        let Some(iso_trans) = htranslation_matrix(&Vector3::new(0.0, 0.0, shift_z)) else {
            return (Matrix4::identity(), Matrix4::identity());
        };

        let transform_to_xy = iso_trans * rot;

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
            position: Point3::new(1152.0, 256.0, 512.0),
            normal: Vector3::new(0., 1., 0.),
        },
        Vertex {
            position: Point3::new(1152.0, 256.0, 256.0),
            normal: Vector3::new(0., 1., 0.),
        },
        Vertex {
            position: Point3::new(768.0, 256.0, 256.0),
            normal: Vector3::new(0., 1., 0.),
        },
        Vertex {
            position: Point3::new(768.0, 256.0, 512.0),
            normal: Vector3::new(0., 1., 0.),
        },
        Vertex {
            position: Point3::new(896.0, 256.0, 512.0),
            normal: Vector3::new(0., 1., 0.),
        },
        Vertex {
            position: Point3::new(896.0, 256.0, 384.0),
            normal: Vector3::new(0., 1., 0.),
        },
        Vertex {
            position: Point3::new(1024.0, 256.0, 384.0),
            normal: Vector3::new(0., 1., 0.),
        },
        Vertex {
            position: Point3::new(1024.0, 256.0, 512.0),
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
