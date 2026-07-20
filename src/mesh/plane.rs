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
//! coordinates to `Real` before making orientation and splitting
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
//! ## **Exact Predicate Discipline**
//!
//! - **Robust Predicates**: Uses hyperreal arithmetic for orientation tests and
//!   treats only exact zero as coplanar.
//! - **Boundary Admission**: Primitive finite API values are promoted into
//!   `hyperlattice` / `hyperreal` before topology decisions.
//! - **Degenerate Case Handling**: Proper fallbacks for collinear points and
//!   exact zero-area triangles.
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
use crate::mesh::Polygon;
use crate::vertex::Vertex;
use hyperlattice::{Matrix4, Point3, Real, Vector3};
use hyperreal::RealSign;
/// Classification of a polygon or point whose hyperreal orientation determinant
/// is exact zero, or whose boundary coordinates cannot be promoted.
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
    pub point_a: Point3,
    pub point_b: Point3,
    pub point_c: Point3,
}

fn newell_hreal_normal(points: &[Vector3]) -> Vector3 {
    points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .fold(Vector3::zero(), |acc, (curr, next)| acc + curr.cross(next))
}

fn hreal_is_exact_zero(value: &Real) -> bool {
    matches!(value.refine_sign_until(-128), Some(RealSign::Zero))
}

fn hlimit_point3(point: &Point3) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(point.x.clone(), point.y.clone(), point.z.clone())
}

fn affine_support_component(a: [&Real; 2], b: [&Real; 2], c: [&Real; 2]) -> Real {
    Real::signed_product_sum(
        [true, false, false, false, true, true],
        [
            [b[0], c[1]],
            [b[0], a[1]],
            [a[0], c[1]],
            [b[1], c[0]],
            [b[1], a[0]],
            [a[1], c[0]],
        ],
    )
}

fn affine_support_normal(a: &Point3, b: &Point3, c: &Point3) -> Vector3 {
    Vector3::new([
        affine_support_component([&a.y, &a.z], [&b.y, &b.z], [&c.y, &c.z]),
        affine_support_component([&a.z, &a.x], [&b.z, &b.x], [&c.z, &c.x]),
        affine_support_component([&a.x, &a.y], [&b.x, &b.y], [&c.x, &c.y]),
    ])
}

pub(crate) fn first_nondegenerate_support<'a>(
    point_count: usize,
    point: impl Copy + Fn(usize) -> &'a Point3,
) -> Option<([usize; 3], Vector3)> {
    let n = point_count;
    for i in 0..n.saturating_sub(2) {
        for j in i + 1..n.saturating_sub(1) {
            for k in j + 1..n {
                let normal = affine_support_normal(point(i), point(j), point(k));
                if normal.0.iter().any(|component| {
                    matches!(
                        component.refine_sign_until(-128),
                        Some(RealSign::Positive | RealSign::Negative)
                    )
                }) {
                    return Some(([i, j, k], normal));
                }
            }
        }
    }
    None
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
    pub(crate) fn transform_affine_in_place(&mut self, matrix: &Matrix4) -> bool {
        let Ok(point_a) = matrix.transform_point3(&self.point_a) else {
            return false;
        };
        let Ok(point_b) = matrix.transform_point3(&self.point_b) else {
            return false;
        };
        let Ok(point_c) = matrix.transform_point3(&self.point_c) else {
            return false;
        };
        self.point_a = point_a;
        self.point_b = point_b;
        self.point_c = point_c;
        true
    }

    /// Picks the first certified non-collinear ordered vertex triple and
    /// returns the plane it defines.
    ///
    /// Exact arithmetic does not need a maximally separated support triple for
    /// numerical conditioning. Ordinary polygons therefore take the first
    /// three vertices in O(1); polygons with redundant leading vertices fall
    /// back to an ordered triple search. The candidate is oriented against
    /// Newell's exact area normal.
    pub fn from_vertices(vertices: &[Vertex]) -> Plane {
        let n = vertices.len();
        let reference_plane = Plane {
            point_a: vertices[0].position.clone(),
            point_b: vertices[1].position.clone(),
            point_c: vertices[2].position.clone(),
        };
        if n == 3 {
            return reference_plane;
        }

        let Some(([i, j, k], normal)) =
            first_nondegenerate_support(vertices.len(), |index| &vertices[index].position)
        else {
            return reference_plane;
        };
        let mut plane = Plane {
            point_a: vertices[i].position.clone(),
            point_b: vertices[j].position.clone(),
            point_c: vertices[k].position.clone(),
        };
        let points = vertices
            .iter()
            .map(|vertex| vertex.position.to_vector())
            .collect::<Vec<_>>();
        if matches!(
            normal
                .dot(&newell_hreal_normal(&points))
                .refine_sign_until(-128),
            Some(RealSign::Negative)
        ) {
            plane.flip();
        }
        plane
    }

    /// Build a new `Plane` from a (not-necessarily-unit) normal **n**
    /// and signed offset *o* (in the sense `n · p == o`).
    ///
    /// Invalid primitive inputs produce the canonical XY plane rather than a
    /// panic. Call [`Plane::try_from_normal`] when the caller needs to
    /// distinguish a rejected boundary normal from a real XY plane. The
    /// normal-squared degeneracy check and point-on-plane scale `offset / (n · n)` are
    /// evaluated in `hyperlattice::Vector3`/`Real`; the resulting
    /// support point is exported to finite coordinates only because `Plane`
    /// remains a mesh API-boundary type. This follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), while retaining the
    /// ordinary implicit plane equation used by the Sutherland-Hodgman split
    /// path (<https://doi.org/10.1145/360767.360802>).
    pub fn from_normal(normal: Vector3, offset: Real) -> Self {
        Self::try_from_normal(normal, offset).unwrap_or_else(Self::xy)
    }

    /// Checked plane construction from a public finite normal and offset.
    ///
    /// This is the fallible form of [`Plane::from_normal`]. It keeps primitive
    /// API data out of plane topology unless both the normal and support point
    /// can be represented through hyperlattice/hyperreal.
    pub fn try_from_normal(normal: Vector3, offset: Real) -> Option<Self> {
        let n2 = normal.dot(&normal);
        if hreal_is_exact_zero(&n2) {
            return None;
        }

        // Point on the plane:  p0 = n * o / (n·n)
        let scale = (offset / n2).ok()?;
        let p0 = Point3::from(normal.clone() * &scale);

        // Build an orthonormal basis {u, v} in hyperlattice and export only
        // the finite boundary vectors used to define the public plane carrier.
        let (u, v) = normal.orthonormal_basis_checked().ok()?;

        // Use p0, p0+u, p0+v  as the three defining points.
        Some(Self {
            point_a: p0.clone(),
            point_b: p0.clone() + u,
            point_c: p0 + v,
        })
    }

    fn xy() -> Self {
        Self {
            point_a: Point3::origin(),
            point_b: Point3::new(Real::one(), Real::zero(), Real::zero()),
            point_c: Point3::new(Real::zero(), Real::one(), Real::zero()),
        }
    }

    #[inline]
    pub fn orient_plane(&self, other: &Plane) -> i8 {
        // pick one vertex of the coplanar polygon and move along its normal
        let test_point = other.point_a.clone() + other.normal();
        self.orient_point(&test_point)
    }

    #[inline]
    pub fn orient_point(&self, point: &Point3) -> i8 {
        self.orient_point_hyperlimit(point).unwrap_or(COPLANAR)
    }

    /// Return the (right‑handed) checked unit normal **n** of the plane
    /// `checked_unit((b-a) x (c-a))`.
    ///
    /// The cross product and checked normalization are evaluated with
    /// `hyperlattice::Vector3` and only exported to `f64` at
    /// the mesh API boundary. This keeps the plane normal on the same exact
    /// geometric-computation path as orientation and splitting predicates
    /// (Yap, *Computational Geometry* 7(1-2), 1997,
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>). Degenerate or
    /// non-promotable support points fail closed to the zero normal instead of
    /// re-entering local primitive normalization.
    #[inline]
    pub fn normal(&self) -> Vector3 {
        self.unit_hreal_normal().unwrap_or_else(Vector3::zeros)
    }

    /// Signed offset of the plane from the origin: `d = n · a`.
    ///
    /// The dot product is evaluated through the shared hyperlattice boundary
    /// adapter instead of local primitive vector arithmetic, following Yap's
    /// exact-geometric-computation split between finite carriers and exact-aware
    /// predicates (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    #[inline]
    pub fn offset(&self) -> Real {
        self.normal().dot(&self.point_a.to_vector())
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

    fn orient_point_hyperlimit(&self, point: &Point3) -> Option<i8> {
        let a = hlimit_point3(&self.point_a);
        let b = hlimit_point3(&self.point_b);
        let c = hlimit_point3(&self.point_c);
        let d = hlimit_point3(point);
        match hyperlimit::orient3d(&a, &b, &c, &d).value()? {
            hyperlimit::Sign::Positive => Some(BACK),
            hyperlimit::Sign::Negative => Some(FRONT),
            hyperlimit::Sign::Zero => Some(COPLANAR),
        }
    }

    fn unscaled_hreal_normal(&self) -> Option<Vector3> {
        let a = self.point_a.to_vector();
        let b = self.point_b.to_vector();
        let c = self.point_c.to_vector();
        Some((&b - &a).cross(&(&c - &a)))
    }

    pub(crate) fn unit_hreal_normal(&self) -> Option<Vector3> {
        self.unscaled_hreal_normal()?.normalize_checked().ok()
    }

    fn edge_intersection_parameter(
        &self,
        start: &Vertex,
        end: &Vertex,
        normal: Option<&Vector3>,
    ) -> Option<Real> {
        normal?;
        let a = hlimit_point3(&self.point_a);
        let b = hlimit_point3(&self.point_b);
        let c = hlimit_point3(&self.point_c);
        let start = hlimit_point3(&start.position);
        let end = hlimit_point3(&end.position);
        let intersection =
            hyperlimit::intersect_segment_with_oriented_plane(&a, &b, &c, &start, &end);
        match intersection.relation {
            hyperlimit::SegmentPlaneRelation::EndpointOnPlane
            | hyperlimit::SegmentPlaneRelation::ProperCrossing => intersection.parameter,
            hyperlimit::SegmentPlaneRelation::Disjoint
            | hyperlimit::SegmentPlaneRelation::Coplanar
            | hyperlimit::SegmentPlaneRelation::Unknown
            | hyperlimit::SegmentPlaneRelation::ConstructionFailed => None,
        }
    }

    /// Intersect an edge with this plane using the same hyperreal point-normal
    /// algebra as polygon splitting.
    ///
    /// The returned vertex is still an API-boundary `f64` [`Vertex`] because
    /// the mesh data model has not been fully moved to hyper geometry yet, but
    /// the topological decision and line-plane parameter are evaluated in
    /// `Real` through `hyperlattice::Vector3`. Keeping the
    /// determinant and dot-product work in the hyper geometry layer follows
    /// Yap's exact-geometric-computation split between geometric objects and
    /// scalar arithmetic (<https://doi.org/10.1016/0925-7721(95)00040-2>);
    /// the line-plane solve is the Sutherland-Hodgman clipping intersection
    /// used by this module (<https://doi.org/10.1145/360767.360802>).
    #[cfg(feature = "sketch")]
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
                if normals_same_direction(&normal, &polygon.plane.normal()) {
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
                        split_front.push(vertex_i.clone());
                    }
                    // If current vertex is definitely not in front, it goes to split_back
                    if type_i != FRONT {
                        split_back.push(vertex_i.clone());
                    }

                    // If the edge between these two vertices crosses the plane,
                    // compute intersection and add that intersection to both sets
                    if (type_i | type_j) == SPANNING
                        && let Some(intersection) = self.edge_intersection_parameter(
                            vertex_i,
                            vertex_j,
                            hnormal.as_ref(),
                        )
                    {
                        let vertex_new = vertex_i.interpolate(vertex_j, intersection);
                        split_front.push(vertex_new.clone());
                        split_back.push(vertex_new);
                    }
                }

                // Build new polygons from the front/back vertex lists
                // if they have at least 3 vertices
                if split_front.len() >= 3 {
                    front.push(
                        Polygon::new(split_front, polygon.metadata.clone())
                            .with_plane_id(polygon.plane_id),
                    );
                }
                if split_back.len() >= 3 {
                    back.push(
                        Polygon::new(split_back, polygon.metadata.clone())
                            .with_plane_id(polygon.plane_id),
                    );
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
    pub fn to_xy_transform(&self) -> (Matrix4, Matrix4) {
        // Normal
        let Ok(norm_dir) = self.normal().normalize_checked() else {
            // Degenerate plane, return identity
            return (Matrix4::identity(), Matrix4::identity());
        };

        // Rotate plane.normal -> +Z.
        let Ok(rot) = Matrix4::rotation_between_vectors(&norm_dir, &Vector3::z()) else {
            return (Matrix4::identity(), Matrix4::identity());
        };

        // We want to translate so that the plane's reference point
        //    (some point p0 with n·p0 = w) lands at z=0 in the new coords.
        // `norm_dir` is already checked as a unit vector in hyperlattice.
        let p0_3d = Point3::from(norm_dir * self.offset());
        let p0_rot = rot
            .transform_point3(&p0_3d)
            .unwrap_or_else(|_| Point3::origin());

        // We want p0_rot.z = 0, so we shift by -p0_rot.z
        let shift_z = -p0_rot.z;
        let iso_trans = Matrix4::affine_translation([Real::zero(), Real::zero(), shift_z]);

        let transform_to_xy = iso_trans * rot;

        // Inverse for going back
        let transform_from_xy = transform_to_xy
            .clone()
            .inverse()
            .unwrap_or_else(|_| Matrix4::identity());

        (transform_to_xy, transform_from_xy)
    }
}

fn normals_same_direction(lhs: &Vector3, rhs: &Vector3) -> bool {
    matches!(lhs.dot(rhs).refine_sign_until(-128), Some(RealSign::Positive))
}

#[cfg(test)]
fn test_real(value: f64) -> Real {
    crate::hyper_math::hreal_from_f64(value).expect("test values must be finite")
}

#[cfg(test)]
fn test_point(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(test_real(x), test_real(y), test_real(z))
}

#[cfg(test)]
fn test_vector(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::from_xyz(test_real(x), test_real(y), test_real(z))
}

#[test]
fn affine_support_normal_matches_expanded_cross_product() {
    let a = test_point(1.25, -2.5, 0.75);
    let b = test_point(4.5, 3.25, -1.5);
    let c = test_point(-0.5, 2.0, 5.75);
    let expected = (&b.to_vector() - &a.to_vector()).cross(&(&c.to_vector() - &a.to_vector()));

    assert_eq!(affine_support_normal(&a, &b, &c), expected);
}

#[test]
fn test_plane_orientation() {
    let vertices = [
        Vertex {
            position: test_point(1152.0, 256.0, 512.0),
            normal: test_vector(0.0, 1.0, 0.0),
            position_id: crate::vertex::fresh_position_id(),
            coordinate_ids: [
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
            ],
            ruled_line: None,
            hull_candidate: true,
        },
        Vertex {
            position: test_point(1152.0, 256.0, 256.0),
            normal: test_vector(0.0, 1.0, 0.0),
            position_id: crate::vertex::fresh_position_id(),
            coordinate_ids: [
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
            ],
            ruled_line: None,
            hull_candidate: true,
        },
        Vertex {
            position: test_point(768.0, 256.0, 256.0),
            normal: test_vector(0.0, 1.0, 0.0),
            position_id: crate::vertex::fresh_position_id(),
            coordinate_ids: [
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
            ],
            ruled_line: None,
            hull_candidate: true,
        },
        Vertex {
            position: test_point(768.0, 256.0, 512.0),
            normal: test_vector(0.0, 1.0, 0.0),
            position_id: crate::vertex::fresh_position_id(),
            coordinate_ids: [
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
            ],
            ruled_line: None,
            hull_candidate: true,
        },
        Vertex {
            position: test_point(896.0, 256.0, 512.0),
            normal: test_vector(0.0, 1.0, 0.0),
            position_id: crate::vertex::fresh_position_id(),
            coordinate_ids: [
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
            ],
            ruled_line: None,
            hull_candidate: true,
        },
        Vertex {
            position: test_point(896.0, 256.0, 384.0),
            normal: test_vector(0.0, 1.0, 0.0),
            position_id: crate::vertex::fresh_position_id(),
            coordinate_ids: [
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
            ],
            ruled_line: None,
            hull_candidate: true,
        },
        Vertex {
            position: test_point(1024.0, 256.0, 384.0),
            normal: test_vector(0.0, 1.0, 0.0),
            position_id: crate::vertex::fresh_position_id(),
            coordinate_ids: [
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
            ],
            ruled_line: None,
            hull_candidate: true,
        },
        Vertex {
            position: test_point(1024.0, 256.0, 512.0),
            normal: test_vector(0.0, 1.0, 0.0),
            position_id: crate::vertex::fresh_position_id(),
            coordinate_ids: [
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
                crate::vertex::fresh_position_id(),
            ],
            ruled_line: None,
            hull_candidate: true,
        },
    ];

    // Cycling the order of the vertices doesn't change the winding order of the shape,
    // so it should not change the resulting plane's normal.
    for cycle_rotation in 0..vertices.len() {
        let mut vertices = vertices.clone();
        vertices.rotate_right(cycle_rotation);
        let plane = Plane::from_vertices(&vertices);

        assert!(
            plane.normal() == test_vector(0.0, 1.0, 0.0),
            "the vertices {vertices:?} form a plane with unexpected normal {}, \
            expected (0., 1., 0.); \
            point list obtained by rotating {cycle_rotation} times",
            plane.normal(),
        );
    }
}
