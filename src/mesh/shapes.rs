//! 3D Shapes as `Mesh`s

use crate::csg::CSG;
use crate::errors::ValidationError;
use crate::mesh::Mesh;
use crate::polygon::Polygon;
#[cfg(feature = "sketch")]
use crate::sketch::Profile;
use crate::vertex::Vertex;
use hyperlattice::{Matrix4, Point3, Real, Vector3};
use hyperreal::RealSign;
use std::cmp::Ordering;
use std::fmt::Debug;

fn finite_mesh_scalar(_value: &Real) -> bool {
    true
}

fn finite_mesh_point(_point: &Point3) -> bool {
    true
}

/// Accept any finite, strictly positive mesh scalar exactly.
///
/// Mesh constructors are still fed by primitive boundary scalars, but admission
/// decisions should not collapse small nonzero values through a tolerance band.
/// This follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
fn real_cmp(lhs: &Real, rhs: &Real) -> Ordering {
    hyperlimit::compare_reals(lhs, rhs)
        .value()
        .unwrap_or_else(|| match (lhs.clone() - rhs.clone()).refine_sign_until(128) {
            Some(RealSign::Positive) => Ordering::Greater,
            Some(RealSign::Negative) => Ordering::Less,
            Some(RealSign::Zero) | None => Ordering::Equal,
        })
}

fn hmesh_scalar_positive(value: &Real) -> bool {
    matches!(real_cmp(value, &Real::zero()), Ordering::Greater)
}

fn hmesh_scalar_nonzero(value: &Real) -> bool {
    !matches!(value.refine_sign_until(128), Some(RealSign::Zero))
}

fn real_from_ratio(numerator: u64, denominator: u64) -> Option<Real> {
    (Real::from(numerator) / Real::from(denominator)).ok()
}

fn fraction(index: usize, count: usize) -> Option<Real> {
    (Real::from(index as u64) / Real::from(count as u64)).ok()
}

fn tau() -> Real {
    Real::from(2_u8) * Real::pi()
}

#[cfg(feature = "sketch")]
fn degrees_to_radians(degrees: Real) -> Option<Real> {
    (degrees * Real::pi() / Real::from(180_u8)).ok()
}

#[cfg(feature = "sketch")]
fn radians_to_degrees(radians: Real) -> Option<Real> {
    (radians * Real::from(180_u8) / Real::pi()).ok()
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// **Mathematical Foundations for 3D Box Geometry**
    ///
    /// This module implements mathematically rigorous algorithms for generating
    /// axis-aligned rectangular prisms (cuboids) and cubes based on solid geometry
    /// and computational topology principles.
    ///
    /// ## **Theoretical Foundations**
    ///
    /// ### **Cuboid Geometry**
    /// A right rectangular prism (cuboid) in 3D space is defined by:
    /// - **Vertices**: 8 corner points forming a rectangular parallelepiped
    /// - **Edges**: 12 edges connecting adjacent vertices
    /// - **Faces**: 6 rectangular faces, each with consistent outward normal
    ///
    /// ### **Coordinate System**
    /// Standard axis-aligned cuboid from origin:
    /// ```text
    /// (0,0,0) → (width, length, height)
    /// ```text
    /// This creates a right-handed coordinate system with consistent face orientations.
    ///
    /// ### **Face Normal Calculation**
    /// Each face normal is computed using the right-hand rule:
    /// ```text
    /// n⃗ = (v⃗₁ - v⃗₀) × (v⃗₂ - v⃗₀)
    /// ```text
    /// where vertices are ordered counter-clockwise when viewed from outside.
    ///
    /// ### **Winding Order Convention**
    /// All faces use counter-clockwise vertex ordering when viewed from exterior:
    /// - **Ensures consistent outward normals**
    /// - **Enables proper backface culling**
    /// - **Maintains manifold topology for CSG operations**
    ///
    /// ## **Geometric Properties**
    /// - **Volume**: V = width × length × height
    /// - **Surface Area**: A = 2(wl + wh + lh)
    /// - **Diagonal**: d = √(w² + l² + h²)
    /// - **Centroid**: (w/2, l/2, h/2)
    pub fn cuboid(width: Real, length: Real, height: Real, metadata: M) -> Mesh<M> {
        if !(finite_mesh_scalar(&width)
            && finite_mesh_scalar(&length)
            && finite_mesh_scalar(&height))
        {
            return Mesh::empty(metadata);
        }
        // Define the eight corner points of the prism.
        //    (x, y, z)
        let p000 = Point3::origin();
        let p100 = Point3::new(width.clone(), Real::zero(), Real::zero());
        let p110 = Point3::new(width.clone(), length.clone(), Real::zero());
        let p010 = Point3::new(Real::zero(), length.clone(), Real::zero());

        let p001 = Point3::new(Real::zero(), Real::zero(), height.clone());
        let p101 = Point3::new(width.clone(), Real::zero(), height.clone());
        let p111 = Point3::new(width.clone(), length.clone(), height.clone());
        let p011 = Point3::new(Real::zero(), length.clone(), height.clone());

        // We’ll define 6 faces (each a Polygon), in an order that keeps outward-facing normals
        // and consistent (counter-clockwise) vertex winding as viewed from outside the prism.

        // Bottom face (z=0, normal approx. -Z)
        // p000 -> p100 -> p110 -> p010
        let bottom_normal = -Vector3::z();
        let bottom = Polygon::new(
            vec![
                Vertex::new(p000.clone(), bottom_normal.clone()),
                Vertex::new(p010.clone(), bottom_normal.clone()),
                Vertex::new(p110.clone(), bottom_normal.clone()),
                Vertex::new(p100.clone(), bottom_normal.clone()),
            ],
            metadata.clone(),
        );

        // Top face (z=depth, normal approx. +Z)
        // p001 -> p011 -> p111 -> p101
        let top_normal = Vector3::z();
        let top = Polygon::new(
            vec![
                Vertex::new(p001.clone(), top_normal.clone()),
                Vertex::new(p101.clone(), top_normal.clone()),
                Vertex::new(p111.clone(), top_normal.clone()),
                Vertex::new(p011.clone(), top_normal.clone()),
            ],
            metadata.clone(),
        );

        // Front face (y=0, normal approx. -Y)
        // p000 -> p001 -> p101 -> p100
        let front_normal = -Vector3::y();
        let front = Polygon::new(
            vec![
                Vertex::new(p000.clone(), front_normal.clone()),
                Vertex::new(p100.clone(), front_normal.clone()),
                Vertex::new(p101.clone(), front_normal.clone()),
                Vertex::new(p001.clone(), front_normal.clone()),
            ],
            metadata.clone(),
        );

        // Back face (y=height, normal approx. +Y)
        // p010 -> p110 -> p111 -> p011
        let back_normal = Vector3::y();
        let back = Polygon::new(
            vec![
                Vertex::new(p010.clone(), back_normal.clone()),
                Vertex::new(p011.clone(), back_normal.clone()),
                Vertex::new(p111.clone(), back_normal.clone()),
                Vertex::new(p110.clone(), back_normal.clone()),
            ],
            metadata.clone(),
        );

        // Left face (x=0, normal approx. -X)
        // p000 -> p010 -> p011 -> p001
        let left_normal = -Vector3::x();
        let left = Polygon::new(
            vec![
                Vertex::new(p000.clone(), left_normal.clone()),
                Vertex::new(p001.clone(), left_normal.clone()),
                Vertex::new(p011.clone(), left_normal.clone()),
                Vertex::new(p010.clone(), left_normal.clone()),
            ],
            metadata.clone(),
        );

        // Right face (x=width, normal approx. +X)
        // p100 -> p101 -> p111 -> p110
        let right_normal = Vector3::x();
        let right = Polygon::new(
            vec![
                Vertex::new(p100, right_normal.clone()),
                Vertex::new(p110, right_normal.clone()),
                Vertex::new(p111, right_normal.clone()),
                Vertex::new(p101, right_normal),
            ],
            metadata.clone(),
        );

        // Combine all faces into a Mesh
        Mesh::from_polygons(&[bottom, top, front, back, left, right], metadata)
    }

    pub fn cube(width: Real, metadata: M) -> Mesh<M> {
        Self::cuboid(width.clone(), width.clone(), width, metadata)
    }

    /// **Mathematical Foundation: Spherical Mesh Generation**
    ///
    /// Construct a sphere using UV-parameterized quadrilateral tessellation.
    /// This implements the standard spherical coordinate parameterization
    /// with adaptive handling of polar degeneracies.
    ///
    /// ## **Sphere Mathematics**
    ///
    /// ### **Parametric Surface Equations**
    /// The sphere surface is defined by:
    /// ```text
    /// M(u,v) = r(sin(πv)cos(2πu), cos(πv), sin(πv)sin(2πu))
    /// where u ∈ [0,1], v ∈ [0,1]
    /// ```text
    ///
    /// ### **Tessellation Algorithm**
    /// 1. **Parameter Grid**: Create (segments+1) × (stacks+1) parameter values
    /// 2. **Vertex Generation**: Evaluate M(u,v) at grid points
    /// 3. **Quadrilateral Formation**: Connect adjacent grid points
    /// 4. **Degeneracy Handling**: Poles require triangle adaptation
    ///
    /// ### **Pole Degeneracy Resolution**
    /// At poles (v=0 or v=1), the parameterization becomes singular:
    /// - **North pole** (v=0): All u values map to same point (0, r, 0)
    /// - **South pole** (v=1): All u values map to same point (0, -r, 0)
    /// - **Solution**: Use triangles instead of quads for polar caps
    ///
    /// ### **Normal Vector Computation**
    /// Sphere normals are simply the normalized position vectors:
    /// ```text
    /// n⃗ = p⃗/|p⃗| = (x,y,z)/r
    /// ```text
    /// This is mathematically exact for spheres (no approximation needed).
    ///
    /// ### **Mesh Quality Metrics**
    /// - **Aspect Ratio**: Best when segments ≈ 2×stacks
    /// - **Area Distortion**: Minimal at equator, maximal at poles
    /// - **Angular Distortion**: Increases towards poles (unavoidable)
    ///
    /// ### **Numerical Considerations**
    /// - **Trigonometric Precision**: Uses tau() and pi() for accuracy
    /// - **Pole Handling**: Avoids division by zero at singularities
    /// - **Winding Consistency**: Maintains outward-facing orientation
    ///
    /// ## **Geometric Properties**
    /// - **Surface Area**: A = 4πr²
    /// - **Volume**: V = (4/3)πr³
    /// - **Circumference** (any great circle): C = 2πr
    /// - **Curvature**: Gaussian K = 1/r², Mean H = 1/r
    ///
    /// # Parameters
    /// - `radius`: Sphere radius (> 0)
    /// - `segments`: Longitude divisions (≥ 3, recommend ≥ 8)
    /// - `stacks`: Latitude divisions (≥ 2, recommend ≥ 6)
    /// - `metadata`: Optional metadata for all faces
    ///
    /// Longitude/latitude sampling is evaluated through hyperreal trigonometry
    /// before the finite vertices are exported to hypermesh-compatible mesh
    /// polygons, following Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn sphere(radius: Real, segments: usize, stacks: usize, metadata: M) -> Mesh<M> {
        if !finite_mesh_scalar(&radius) {
            return Mesh::empty(metadata);
        }
        let segments = segments.max(3);
        let stacks = stacks.max(2);
        let mut polygons = Vec::new();

        for i in 0..segments {
            for j in 0..stacks {
                let mut vertices = Vec::new();

                let vertex = |theta: Real, phi: Real| -> Option<Vertex> {
                    let sin_theta = theta.clone().sin();
                    let cos_theta = theta.cos();
                    let sin_phi = phi.clone().sin();
                    let cos_phi = phi.cos();
                    let dir = Vector3::from_xyz(
                        cos_theta * sin_phi.clone(),
                        cos_phi,
                        sin_theta * sin_phi,
                    );
                    let normal = dir.normalize_checked().unwrap_or_else(|_| dir.clone());
                    Some(Vertex::new(
                        Point3::new(
                            dir.0[0].clone() * radius.clone(),
                            dir.0[1].clone() * radius.clone(),
                            dir.0[2].clone() * radius.clone(),
                        ),
                        normal,
                    ))
                };

                let Some(t0) = fraction(i, segments) else {
                    return Mesh::empty(metadata);
                };
                let Some(t1) = fraction(i + 1, segments) else {
                    return Mesh::empty(metadata);
                };
                let Some(p0) = fraction(j, stacks) else {
                    return Mesh::empty(metadata);
                };
                let Some(p1) = fraction(j + 1, stacks) else {
                    return Mesh::empty(metadata);
                };

                let theta0 = t0 * tau();
                let theta1 = t1 * tau();
                let phi0 = p0 * Real::pi();
                let phi1 = p1 * Real::pi();

                let Some(first) = vertex(theta0.clone(), phi0.clone()) else {
                    return Mesh::empty(metadata);
                };
                vertices.push(first);
                if j > 0 {
                    let Some(v) = vertex(theta1.clone(), phi0.clone()) else {
                        return Mesh::empty(metadata);
                    };
                    vertices.push(v);
                }
                if j < stacks - 1 {
                    let Some(v) = vertex(theta1.clone(), phi1.clone()) else {
                        return Mesh::empty(metadata);
                    };
                    vertices.push(v);
                }
                let Some(last) = vertex(theta0, phi1) else {
                    return Mesh::empty(metadata);
                };
                vertices.push(last);

                polygons.push(Polygon::new(vertices, metadata.clone()));
            }
        }
        Mesh::from_polygons(&polygons, metadata)
    }

    /// Constructs a frustum between `start` and `end` with bottom radius = `radius1` and
    /// top radius = `radius2`. In the normal case, it creates side quads and cap triangles.
    /// However, if one of the radii is 0 (within tolerance), then the degenerate face is treated
    /// as a single point and the side is stitched using triangles.
    ///
    /// # Parameters
    /// - `start`: the center of the bottom face
    /// - `end`: the center of the top face
    /// - `radius1`: the radius at the bottom face
    /// - `radius2`: the radius at the top face
    /// - `segments`: number of segments around the circle (must be ≥ 3)
    /// - `metadata`: optional metadata
    ///
    /// # Example
    /// ```ignore
    /// use csgrs::mesh::Mesh;
    /// use hyperlattice::Point3;
    /// let bottom = Point3::new(0.0, 0.0, 0.0);
    /// let top = Point3::new(0.0, 0.0, 5.0);
    /// // This will create a cone (bottom degenerate) because radius1 is 0:
    /// let cone = Mesh::<()>::frustum_ptp(bottom, top, 0.0, 2.0, 32, ());
    /// ```
    pub fn frustum_ptp(
        start: Point3,
        end: Point3,
        radius1: Real,
        radius2: Real,
        segments: usize,
        metadata: M,
    ) -> Mesh<M> {
        if !finite_mesh_point(&start)
            || !finite_mesh_point(&end)
            || !finite_mesh_scalar(&radius1)
            || !finite_mesh_scalar(&radius2)
        {
            return Mesh::empty(metadata);
        }
        let segments = segments.max(3);
        // Compute the axis and check that start and end do not coincide.
        //
        // The axis length and checked unit direction are evaluated through
        // `hyperlattice::Vector3`/`Real`, then exported only for the
        // finite mesh construction loops. This follows Yap's exact-geometric-
        // computation boundary discipline, *Computational Geometry* 7(1-2),
        // 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        let ray = &end - &start;
        let Some(axis_length) = ray.dot(&ray).sqrt().ok() else {
            return Mesh::empty(metadata);
        };
        if !hmesh_scalar_positive(&axis_length) {
            return Mesh::empty(metadata);
        }
        let Ok(axis_z) = ray.normalize_checked() else {
            return Mesh::empty(metadata);
        };

        // Pick axes in hyperlattice so primitive floats only carry the final
        // mesh-boundary basis.
        let Ok((axis_x, axis_y)) = axis_z.orthonormal_basis_checked() else {
            return Mesh::empty(metadata);
        };

        // The cap centers for the bottom and top.
        let start_v = Vertex::new(start.clone(), -axis_z.clone());
        let end_v = Vertex::new(end.clone(), axis_z.clone());

        // A closure that returns a vertex on the lateral surface.
        // For a given stack (0.0 for bottom, 1.0 for top), slice (fraction along the circle),
        // and a normal blend factor (used for cap smoothing), compute the vertex.
        let point = |stack: Real, slice: Real, normal_blend: Real| -> Option<Vertex> {
            // Linear interpolation of radius.
            let radius_delta = radius2.clone() - radius1.clone();
            let r = radius1.clone() + stack.clone() * radius_delta;
            let angle = slice * tau();
            let sin_angle = angle.clone().sin();
            let cos_angle = angle.cos();
            let radial_dir = Vector3::from_xyz(
                axis_x.0[0].clone() * cos_angle.clone()
                    + axis_y.0[0].clone() * sin_angle.clone(),
                axis_x.0[1].clone() * cos_angle.clone()
                    + axis_y.0[1].clone() * sin_angle.clone(),
                axis_x.0[2].clone() * cos_angle + axis_y.0[2].clone() * sin_angle,
            );
            let position = start.clone() + ray.clone() * stack + radial_dir.clone() * r;
            let radial_normal_scale = Real::one() - normal_blend.abs();
            let normal =
                radial_dir * radial_normal_scale + axis_z.clone() * normal_blend.clone();
            Some(Vertex::new(
                position,
                normal.normalize_checked().unwrap_or_else(|_| axis_z.clone()),
            ))
        };

        let mut polygons = Vec::new();

        // Special-case flags for degenerate faces.
        let bottom_degenerate = !hmesh_scalar_nonzero(&radius1);
        let top_degenerate = !hmesh_scalar_nonzero(&radius2);

        // If both faces are degenerate, we cannot build a meaningful volume.
        if bottom_degenerate && top_degenerate {
            return Mesh::empty(metadata);
        }

        // For each slice of the circle (0..segments)
        for i in 0..segments {
            let Some(slice0) = fraction(i, segments) else {
                return Mesh::empty(metadata);
            };
            let Some(slice1) = fraction(i + 1, segments) else {
                return Mesh::empty(metadata);
            };

            // In the normal frustum_ptp, we always add a bottom cap triangle (fan) and a top cap triangle.
            // Here, we only add the cap triangle if the corresponding radius is not degenerate.
            if !bottom_degenerate {
                // Bottom cap: a triangle fan from the bottom center to two consecutive points on the bottom ring.
                let (Some(p0), Some(p1)) = (
                    point(Real::zero(), slice0.clone(), -Real::one()),
                    point(Real::zero(), slice1.clone(), -Real::one()),
                ) else {
                    return Mesh::empty(metadata);
                };
                polygons.push(Polygon::new(vec![start_v.clone(), p0, p1], metadata.clone()));
            }
            if !top_degenerate {
                // Top cap: a triangle fan from the top center to two consecutive points on the top ring.
                let (Some(p0), Some(p1)) = (
                    point(Real::one(), slice1.clone(), Real::one()),
                    point(Real::one(), slice0.clone(), Real::one()),
                ) else {
                    return Mesh::empty(metadata);
                };
                polygons.push(Polygon::new(vec![end_v.clone(), p0, p1], metadata.clone()));
            }

            // For the side wall, we normally build a quad spanning from the bottom ring (stack=0)
            // to the top ring (stack=1). If one of the rings is degenerate, that ring reduces to a single point.
            // In that case, we output a triangle.
            if bottom_degenerate {
                // Bottom is a point (start_v); create a triangle from start_v to two consecutive points on the top ring.
                let (Some(p0), Some(p1)) = (
                    point(Real::one(), slice0.clone(), Real::zero()),
                    point(Real::one(), slice1.clone(), Real::zero()),
                ) else {
                    return Mesh::empty(metadata);
                };
                polygons.push(Polygon::new(vec![start_v.clone(), p0, p1], metadata.clone()));
            } else if top_degenerate {
                // Top is a point (end_v); create a triangle from two consecutive points on the bottom ring to end_v.
                let (Some(p0), Some(p1)) = (
                    point(Real::zero(), slice1.clone(), Real::zero()),
                    point(Real::zero(), slice0.clone(), Real::zero()),
                ) else {
                    return Mesh::empty(metadata);
                };
                polygons.push(Polygon::new(vec![p0, p1, end_v.clone()], metadata.clone()));
            } else {
                // Normal case: both rings are non-degenerate. Use a quad for the side wall.
                let (Some(p0), Some(p1), Some(p2), Some(p3)) = (
                    point(Real::zero(), slice1.clone(), Real::zero()),
                    point(Real::zero(), slice0.clone(), Real::zero()),
                    point(Real::one(), slice0.clone(), Real::zero()),
                    point(Real::one(), slice1.clone(), Real::zero()),
                ) else {
                    return Mesh::empty(metadata);
                };
                polygons.push(Polygon::new(vec![p0, p1, p2, p3], metadata.clone()));
            }
        }

        Mesh::from_polygons(&polygons, metadata)
    }

    /// A helper to create a vertical cylinder along Z from z=0..z=height
    /// with the specified radius (NOT diameter).
    pub fn frustum(
        radius1: Real,
        radius2: Real,
        height: Real,
        segments: usize,
        metadata: M,
    ) -> Mesh<M> {
        Mesh::frustum_ptp(
            Point3::origin(),
            Point3::new(Real::zero(), Real::zero(), height),
            radius1,
            radius2,
            segments,
            metadata,
        )
    }

    /// A helper to create a vertical cylinder along Z from z=0..z=height
    /// with the specified radius (NOT diameter).
    pub fn cylinder(radius: Real, height: Real, segments: usize, metadata: M) -> Mesh<M> {
        Mesh::frustum_ptp(
            Point3::origin(),
            Point3::new(Real::zero(), Real::zero(), height),
            radius.clone(),
            radius,
            segments,
            metadata,
        )
    }

    /// Creates a Mesh polyhedron from raw vertex data (`points`) and face indices.
    ///
    /// Raw coordinates are API-boundary data. Each selected point is promoted
    /// through `hyperlattice::Vector3` before a polygon vertex
    /// is built, so `NaN`/infinite values are rejected instead of being
    /// sanitized by the transitional vertex carrier. This follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    ///
    /// # Parameters
    ///
    /// - `points`: a slice of `[x,y,z]` coordinates.
    /// - `faces`: each element is a list of indices into `points`, describing one face.
    ///   Each face must have at least 3 indices.
    ///
    /// # Example
    /// ```ignore
    /// # use csgrs::mesh::Mesh;
    ///
    /// let pts = &[
    ///     [0.0, 0.0, 0.0], // point0
    ///     [1.0, 0.0, 0.0], // point1
    ///     [1.0, 1.0, 0.0], // point2
    ///     [0.0, 1.0, 0.0], // point3
    ///     [0.5, 0.5, 1.0], // point4 - top
    /// ];
    ///
    /// // Two faces: bottom square [0,1,2,3], and a pyramid side [0,1,4]
    /// let fcs: &[&[usize]] = &[
    ///     &[0, 1, 2, 3],
    ///     &[0, 1, 4],
    ///     &[1, 2, 4],
    ///     &[2, 3, 4],
    ///     &[3, 0, 4],
    /// ];
    ///
    /// let mesh_poly = Mesh::<()>::polyhedron(pts, fcs, ());
    /// ```
    pub fn polyhedron(
        points: &[[Real; 3]],
        faces: &[&[usize]],
        metadata: M,
    ) -> Result<Mesh<M>, ValidationError> {
        let mut polygons = Vec::new();

        for face in faces {
            // Skip degenerate faces
            if face.len() < 3 {
                continue;
            }

            // Gather the vertices for this face
            let mut face_vertices = Vec::with_capacity(face.len());
            for &idx in face.iter() {
                // Ensure the index is valid
                if idx >= points.len() {
                    return Err(ValidationError::IndexOutOfRangeWithLen {
                        index: idx,
                        len: points.len(),
                    });
                }
                let [x, y, z] = &points[idx];
                let point = Point3::new(x.clone(), y.clone(), z.clone());
                face_vertices.push(Vertex::new(
                    point,
                    Vector3::zeros(), // we'll set this later
                ));
            }

            // Build the polygon (plane is auto-computed from first 3 vertices).
            let mut poly = Polygon::new(face_vertices, metadata.clone());

            // Set each vertex normal to match the polygon’s plane normal,
            let plane_normal = poly.plane.normal();
            for v in &mut poly.vertices {
                v.normal = plane_normal.clone();
            }
            polygons.push(poly);
        }

        Ok(Mesh::from_polygons(&polygons, metadata))
    }

    /// Creates a 3D "egg" shape by revolving `Profile::egg()`.
    ///
    /// # Parameters
    /// - `width`: The "width" of the 2D egg outline.
    /// - `length`: The "length" (height) of the 2D egg outline.
    /// - `revolve_segments`: Number of segments for the revolution.
    /// - `outline_segments`: Number of segments for the 2D egg outline itself.
    /// - `metadata`: Optional metadata.
    #[cfg(all(feature = "chull-io", feature = "sketch"))]
    pub fn egg(
        width: Real,
        length: Real,
        revolve_segments: usize,
        outline_segments: usize,
        metadata: M,
    ) -> Self {
        if !finite_mesh_scalar(&width) || !finite_mesh_scalar(&length) || revolve_segments < 3
        {
            return Mesh::empty(metadata);
        }

        let egg_2d = Profile::egg(width, length, outline_segments, metadata.clone());

        // Build a large rectangle that cuts off everything
        let cutter_height = Real::from(9999_u16); // some large number
        let half_cutter =
            (cutter_height.clone() / Real::from(2_u8)).unwrap_or_else(|_| Real::zero());
        let rect_cutter = Profile::square(cutter_height.clone(), metadata.clone()).translate(
            -cutter_height,
            -half_cutter,
            Real::zero(),
        );

        let half_egg = egg_2d.difference(&rect_cutter);

        half_egg
            .revolve(Real::from(360_u16), revolve_segments)
            .map(|mesh| mesh.convex_hull())
            .unwrap_or_else(|_| Mesh::empty(metadata))
    }

    /// Creates a 3D "teardrop" solid by revolving the existing 2D `teardrop` profile 360° around the Y-axis (via revolve).
    ///
    /// # Parameters
    /// - `width`: Width of the 2D teardrop profile.
    /// - `length`: Length of the 2D teardrop profile.
    /// - `revolve_segments`: Number of segments for the revolution (the "circular" direction).
    /// - `shape_segments`: Number of segments for the 2D teardrop outline itself.
    /// - `metadata`: Optional metadata.
    #[cfg(all(feature = "chull-io", feature = "sketch"))]
    pub fn teardrop(
        width: Real,
        length: Real,
        revolve_segments: usize,
        shape_segments: usize,
        metadata: M,
    ) -> Self {
        if !finite_mesh_scalar(&width) || !finite_mesh_scalar(&length) || revolve_segments < 3
        {
            return Mesh::empty(metadata);
        }

        // Make a 2D teardrop in the XY plane.
        let td_2d = Profile::teardrop(width, length, shape_segments, metadata.clone());

        // Build a large rectangle that cuts off everything
        let cutter_height = Real::from(9999_u16); // some large number
        let half_cutter =
            (cutter_height.clone() / Real::from(2_u8)).unwrap_or_else(|_| Real::zero());
        let rect_cutter = Profile::square(cutter_height.clone(), metadata.clone()).translate(
            -cutter_height,
            -half_cutter,
            Real::zero(),
        );

        let half_teardrop = td_2d.difference(&rect_cutter);

        // revolve 360 degrees
        half_teardrop
            .revolve(Real::from(360_u16), revolve_segments)
            .map(|mesh| mesh.convex_hull())
            .unwrap_or_else(|_| Mesh::empty(metadata))
    }

    /// Creates a 3D "teardrop cylinder" by extruding the existing 2D `teardrop` in the Z+ axis.
    ///
    /// # Parameters
    /// - `width`: Width of the 2D teardrop profile.
    /// - `length`: Length of the 2D teardrop profile.
    /// - `revolve_segments`: Number of segments for the revolution (the "circular" direction).
    /// - `shape_segments`: Number of segments for the 2D teardrop outline itself.
    /// - `metadata`: Optional metadata.
    #[cfg(feature = "sketch")]
    pub fn teardrop_cylinder(
        width: Real,
        length: Real,
        height: Real,
        shape_segments: usize,
        metadata: M,
    ) -> Self {
        if !(finite_mesh_scalar(&width)
            && finite_mesh_scalar(&length)
            && finite_mesh_scalar(&height))
        {
            return Mesh::empty(metadata);
        }

        // Make a 2D teardrop in the XY plane.
        let td_2d = Profile::teardrop(width, length, shape_segments, metadata.clone());
        td_2d.extrude(height)
    }

    /// Creates an ellipsoid by taking a sphere of radius=1 and scaling it by (rx, ry, rz).
    ///
    /// # Parameters
    /// - `rx`: X-axis radius.
    /// - `ry`: Y-axis radius.
    /// - `rz`: Z-axis radius.
    /// - `segments`: Number of horizontal segments.
    /// - `stacks`: Number of vertical stacks.
    /// - `metadata`: Optional metadata.
    pub fn ellipsoid(
        rx: Real,
        ry: Real,
        rz: Real,
        segments: usize,
        stacks: usize,
        metadata: M,
    ) -> Self {
        if !(finite_mesh_scalar(&rx) && finite_mesh_scalar(&ry) && finite_mesh_scalar(&rz)) {
            return Mesh::empty(metadata);
        }

        let base_sphere = Self::sphere(Real::one(), segments, stacks, metadata.clone());
        base_sphere.scale(rx, ry, rz)
    }

    /// Creates an arrow Mesh. The arrow is composed of:
    ///   - a cylindrical shaft, and
    ///   - a cone–like head (a frustum from a larger base to a small tip)
    ///
    /// built along the canonical +Z axis. The arrow is then rotated so that +Z aligns with the given
    /// direction, and finally translated so that either its base (if `orientation` is false)
    /// or its tip (if `orientation` is true) is located at `start`.
    ///
    /// The arrow’s dimensions (shaft radius, head dimensions, etc.) are scaled proportionally to the
    /// total arrow length (the norm of the provided direction).
    ///
    /// # Parameters
    /// - `start`: the reference point (base or tip, depending on orientation)
    /// - `direction`: the vector defining arrow length and intended pointing direction
    /// - `segments`: number of segments for approximating the cylinder and frustum
    /// - `orientation`: when false (default) the arrow points away from start (its base is at start); when true the arrow points toward start (its tip is at start).
    /// - `metadata`: optional metadata for the generated polygons.
    ///
    /// Arrow proportions are derived through hyperreal scalar helpers before
    /// delegating shaft/head construction to [`Mesh::cylinder`] and
    /// [`Mesh::frustum_ptp`], following Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn arrow(
        start: Point3,
        direction: Vector3,
        segments: usize,
        orientation: bool,
        metadata: M,
    ) -> Mesh<M> {
        if !finite_mesh_point(&start) {
            return Mesh::empty(metadata);
        }
        // Compute the arrow's total length and unit direction through the
        // hyper geometry boundary used by primitive axis construction.
        let Some(arrow_length) = direction.dot(&direction).sqrt().ok() else {
            return Mesh::empty(metadata);
        };
        if !hmesh_scalar_positive(&arrow_length) {
            return Mesh::empty(metadata);
        }
        let Ok(unit_dir) = direction.normalize_checked() else {
            return Mesh::empty(metadata);
        };

        // Define proportions:
        // - Arrow head occupies 20% of total length.
        // - Shaft occupies the remainder.
        let Some(head_fraction) = real_from_ratio(1, 5) else {
            return Mesh::empty(metadata);
        };
        let head_length = arrow_length.clone() * head_fraction;
        let shaft_length = arrow_length.clone() - head_length.clone();

        // Define thickness parameters proportional to the arrow length.
        let Some(shaft_radius_fraction) = real_from_ratio(3, 100) else {
            return Mesh::empty(metadata);
        };
        let Some(head_base_radius_fraction) = real_from_ratio(3, 50) else {
            return Mesh::empty(metadata);
        };
        let shaft_radius = arrow_length.clone() * shaft_radius_fraction;
        let head_base_radius = arrow_length.clone() * head_base_radius_fraction;
        let tip_radius = Real::zero();

        // Build the shaft as a vertical cylinder along Z from 0 to shaft_length.
        let shaft =
            Mesh::cylinder(shaft_radius, shaft_length.clone(), segments, metadata.clone());

        // Build the arrow head as a frustum from z = shaft_length to z = shaft_length + head_length.
        let head = Mesh::frustum_ptp(
            Point3::new(Real::zero(), Real::zero(), shaft_length.clone()),
            Point3::new(Real::zero(), Real::zero(), arrow_length.clone()),
            head_base_radius,
            tip_radius,
            segments,
            metadata.clone(),
        );

        // Combine the shaft and head.
        let mut canonical_arrow = shaft.union(&head);

        // If the arrow should point toward start, mirror the geometry in canonical space.
        // The mirror transform about the plane z = arrow_length/2 maps any point (0,0,z) to (0,0, arrow_length - z).
        if orientation {
            let Some(half_length) = (arrow_length.clone() / Real::from(2_u8)).ok() else {
                return Mesh::empty(metadata);
            };
            let negative_half_length = -half_length.clone();
            let to_midpoint =
                Matrix4::affine_translation([Real::zero(), Real::zero(), half_length]);
            let reflect_z =
                Matrix4::affine_nonuniform_scale([Real::one(), Real::one(), -Real::one()]);
            let from_midpoint = Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                negative_half_length,
            ]);
            let mirror_mat: Matrix4 = to_midpoint * reflect_z * from_midpoint;
            canonical_arrow = canonical_arrow.transform(&mirror_mat).inverse();
        }
        // In both cases, we now have a canonical arrow that extends from z=0 to z=arrow_length.
        // For orientation == false, z=0 is the base.
        // For orientation == true, after mirroring z=0 is now the tip.

        // Compute the rotation that maps the canonical +Z axis to the provided
        // direction. Dot/cross/unit-vector work is delegated to hyperlattice via
        // `hrotation_between_vectors`, following Yap's exact geometric
        // computation boundary split.
        let z_axis = Vector3::z();
        let rot_mat: Matrix4 = Matrix4::rotation_between_vectors(&z_axis, &unit_dir)
            .unwrap_or_else(|_| Matrix4::identity());

        // Rotate the arrow.
        let rotated_arrow = canonical_arrow.transform(&rot_mat);

        // Finally, translate the arrow so that the anchored vertex (canonical (0,0,0)) moves to 'start'.
        // In the false case, (0,0,0) is the base (arrow extends from start to start+direction).
        // In the true case, after mirroring, (0,0,0) is the tip (arrow extends from start to start+direction).
        rotated_arrow.translate(start.x, start.y, start.z)
    }

    /// Regular octahedron scaled by `radius`
    pub fn octahedron(radius: Real, metadata: M) -> Self {
        if !finite_mesh_scalar(&radius) {
            return Mesh::empty(metadata);
        }
        let pts = &[
            [Real::one(), Real::zero(), Real::zero()],
            [-Real::one(), Real::zero(), Real::zero()],
            [Real::zero(), Real::one(), Real::zero()],
            [Real::zero(), -Real::one(), Real::zero()],
            [Real::zero(), Real::zero(), Real::one()],
            [Real::zero(), Real::zero(), -Real::one()],
        ];
        let faces: [&[usize]; 8] = [
            &[0, 2, 4],
            &[2, 1, 4],
            &[1, 3, 4],
            &[3, 0, 4],
            &[5, 2, 0],
            &[5, 1, 2],
            &[5, 3, 1],
            &[5, 0, 3],
        ];
        let scaled: Vec<[Real; 3]> = pts
            .iter()
            .map(|[x, y, z]| {
                [
                    x.clone() * radius.clone(),
                    y.clone() * radius.clone(),
                    z.clone() * radius.clone(),
                ]
            })
            .collect();
        Self::polyhedron(&scaled, &faces, metadata.clone())
            .unwrap_or_else(|_| Mesh::empty(metadata))
    }

    /// Regular icosahedron scaled by `radius`.
    ///
    /// The golden-ratio normalization is evaluated through `Real`
    /// before finite mesh vertices are emitted. This keeps even constant
    /// shape-construction algebra on the exact-aware side of the boundary,
    /// following Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The construction is
    /// the classical regular icosahedron coordinate model using the golden
    /// ratio; see Coxeter, *Regular Polytopes*, 3rd ed., 1973.
    pub fn icosahedron(radius: Real, metadata: M) -> Self {
        if !finite_mesh_scalar(&radius) {
            return Mesh::empty(metadata);
        }
        // radius scale factor
        let Some(factor_fraction) = real_from_ratio(2939, 5000) else {
            return Mesh::empty(metadata);
        };
        let factor = radius * factor_fraction;
        // golden ratio
        let Some(sqrt_five) = Real::from(5_u8).sqrt().ok() else {
            return Mesh::empty(metadata);
        };
        let Some(phi) = ((Real::one() + sqrt_five) / Real::from(2_u8)).ok() else {
            return Mesh::empty(metadata);
        };
        // normalise so the circum-radius is 1
        let phi_squared = phi.clone() * phi.clone();
        let Some(len) = (Real::one() + phi_squared).sqrt().ok() else {
            return Mesh::empty(metadata);
        };
        let Some(inv_len) = (Real::one() / len).ok() else {
            return Mesh::empty(metadata);
        };
        let a = inv_len;
        let b = phi * a.clone();

        // 12 vertices ----------------------------------------------------
        let pts: [[Real; 3]; 12] = [
            [-a.clone(), b.clone(), Real::zero()],
            [a.clone(), b.clone(), Real::zero()],
            [-a.clone(), -b.clone(), Real::zero()],
            [a.clone(), -b.clone(), Real::zero()],
            [Real::zero(), -a.clone(), b.clone()],
            [Real::zero(), a.clone(), b.clone()],
            [Real::zero(), -a.clone(), -b.clone()],
            [Real::zero(), a.clone(), -b.clone()],
            [b.clone(), Real::zero(), -a.clone()],
            [b.clone(), Real::zero(), a.clone()],
            [-b.clone(), Real::zero(), -a.clone()],
            [-b, Real::zero(), a],
        ];

        // 20 faces (counter-clockwise when viewed from outside) ----------
        let faces: [&[usize]; 20] = [
            &[0, 11, 5],
            &[0, 5, 1],
            &[0, 1, 7],
            &[0, 7, 10],
            &[0, 10, 11],
            &[1, 5, 9],
            &[5, 11, 4],
            &[11, 10, 2],
            &[10, 7, 6],
            &[7, 1, 8],
            &[3, 9, 4],
            &[3, 4, 2],
            &[3, 2, 6],
            &[3, 6, 8],
            &[3, 8, 9],
            &[4, 9, 5],
            &[2, 4, 11],
            &[6, 2, 10],
            &[8, 6, 7],
            &[9, 8, 1],
        ];

        Self::polyhedron(&pts, &faces, metadata.clone())
            .map(|mesh| mesh.scale(factor.clone(), factor.clone(), factor))
            .unwrap_or_else(|_| Mesh::empty(metadata))
    }

    /// Torus centred at the origin in the *XY* plane.
    ///
    /// * `major_r` – distance from centre to tube centre ( R )  
    /// * `minor_r` – tube radius ( r )  
    /// * `segments_major` – number of segments around the donut  
    /// * `segments_minor` – segments of the tube cross-section
    ///
    /// The torus is composed from a hypercurve-backed circular profile and
    /// revolved into mesh geometry. Radius validation is evaluated through
    /// hyperreal comparisons before crossing back to finite boundary scalars,
    /// following Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    #[cfg(feature = "sketch")]
    pub fn torus(
        major_r: Real,
        minor_r: Real,
        segments_major: usize,
        segments_minor: usize,
        metadata: M,
    ) -> Self {
        if !finite_mesh_scalar(&major_r)
            || !finite_mesh_scalar(&minor_r)
            || !hmesh_scalar_nonzero(&major_r)
            || !hmesh_scalar_nonzero(&minor_r)
            || segments_major < 3
            || segments_minor < 3
        {
            return Mesh::empty(metadata);
        }

        let profile_offset = major_r;
        let circle = Profile::circle(minor_r, segments_minor, metadata.clone()).translate(
            profile_offset,
            Real::zero(),
            Real::zero(),
        );
        circle
            .revolve(Real::from(360_u16), segments_major)
            .unwrap_or_else(|_| Mesh::empty(metadata))
    }

    #[allow(clippy::too_many_arguments)]
    #[cfg(feature = "sketch")]
    pub fn spur_gear_involute(
        module: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        thickness: Real,
        metadata: M,
    ) -> Mesh<M> {
        if !finite_mesh_scalar(&thickness) {
            return Mesh::empty(metadata);
        }

        Profile::involute_gear(
            module,
            teeth,
            pressure_angle_deg,
            clearance,
            backlash,
            segments_per_flank,
            metadata.clone(),
        )
        .extrude(thickness)
    }

    #[cfg(feature = "sketch")]
    pub fn spur_gear_cycloid(
        module: Real,
        teeth: usize,
        pin_teeth: usize,
        clearance: Real,
        segments_per_flank: usize,
        thickness: Real,
        metadata: M,
    ) -> Mesh<M> {
        if !finite_mesh_scalar(&thickness) {
            return Mesh::empty(metadata);
        }

        Profile::cycloidal_gear(
            module,
            teeth,
            pin_teeth,
            clearance,
            segments_per_flank,
            metadata.clone(),
        )
        .extrude(thickness)
    }

    #[allow(clippy::too_many_arguments)]
    #[cfg(feature = "sketch")]
    pub fn helical_involute_gear(
        module: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        thickness: Real,
        helix_angle_deg: Real, // β
        slices: usize,         // ≥ 2 – axial divisions
        metadata: M,
    ) -> Mesh<M> {
        if slices < 2
            || !finite_mesh_scalar(&thickness)
            || !finite_mesh_scalar(&helix_angle_deg)
        {
            return Mesh::empty(metadata);
        }

        let base_slice = Profile::involute_gear(
            module,
            teeth,
            pressure_angle_deg,
            clearance,
            backlash,
            segments_per_flank,
            metadata.clone(),
        );
        if base_slice.is_empty() {
            return Mesh::empty(metadata);
        }

        let Ok(dz) = thickness / Real::from(slices as u64) else {
            return Mesh::empty(metadata);
        };
        let Some(d_psi_rad) = degrees_to_radians(helix_angle_deg)
            .and_then(|angle| (angle / Real::from(slices as u64)).ok())
        else {
            return Mesh::empty(metadata);
        };
        if !finite_mesh_scalar(&dz) || !finite_mesh_scalar(&d_psi_rad) {
            return Mesh::empty(metadata);
        }

        let mut acc = Mesh::empty(metadata.clone());
        let mut z_curr = Real::zero();
        for i in 0..slices {
            let Some(rotation_deg) =
                radians_to_degrees(Real::from(i as u64) * d_psi_rad.clone())
            else {
                return Mesh::empty(metadata);
            };
            let slice = base_slice
                .rotate(Real::zero(), Real::zero(), rotation_deg)
                .extrude(dz.clone())
                .translate(Real::zero(), Real::zero(), z_curr.clone());
            acc = if i == 0 { slice } else { acc.union(&slice) };
            z_curr = z_curr + dz.clone();
        }
        acc
    }
}
