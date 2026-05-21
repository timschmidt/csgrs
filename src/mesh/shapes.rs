//! 3D Shapes as `Mesh`s

use crate::csg::CSG;
use crate::errors::ValidationError;
use crate::float_types::{
    PI, Real, TAU, hangle_sin_cos, hdegrees_to_radians, hperpendicular_basis,
    hradians_to_degrees, hreal_abs, hreal_affine, hreal_cmp_f64, hreal_div, hreal_from_f64,
    hreal_mul, hreal_sqrt, hreal_sub, hreal_sum, hrotation_between_vectors, hscale_matrix,
    htranslation_matrix, hunit_vector3, hunit_vector3_and_magnitude, hvector3_from_point3,
    tolerance,
};
use crate::mesh::Mesh;
use crate::polygon::Polygon;
#[cfg(feature = "sketch")]
use crate::sketch::Profile;
use crate::vertex::Vertex;
use nalgebra::{Matrix4, Point3, Vector3};
use std::cmp::Ordering;
use std::fmt::Debug;

fn finite_mesh_scalar(value: Real) -> bool {
    hreal_from_f64(value).is_ok()
}

fn finite_mesh_point(point: &Point3<Real>) -> bool {
    hvector3_from_point3(point).is_some()
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
    /// ```
    /// This creates a right-handed coordinate system with consistent face orientations.
    ///
    /// ### **Face Normal Calculation**
    /// Each face normal is computed using the right-hand rule:
    /// ```text
    /// n⃗ = (v⃗₁ - v⃗₀) × (v⃗₂ - v⃗₀)
    /// ```
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
        if !(finite_mesh_scalar(width)
            && finite_mesh_scalar(length)
            && finite_mesh_scalar(height))
        {
            return Mesh::empty(metadata);
        }
        // Define the eight corner points of the prism.
        //    (x, y, z)
        let p000 = Point3::new(0.0, 0.0, 0.0);
        let p100 = Point3::new(width, 0.0, 0.0);
        let p110 = Point3::new(width, length, 0.0);
        let p010 = Point3::new(0.0, length, 0.0);

        let p001 = Point3::new(0.0, 0.0, height);
        let p101 = Point3::new(width, 0.0, height);
        let p111 = Point3::new(width, length, height);
        let p011 = Point3::new(0.0, length, height);

        // We’ll define 6 faces (each a Polygon), in an order that keeps outward-facing normals
        // and consistent (counter-clockwise) vertex winding as viewed from outside the prism.

        // Bottom face (z=0, normal approx. -Z)
        // p000 -> p100 -> p110 -> p010
        let bottom_normal = -Vector3::z();
        let bottom = Polygon::new(
            vec![
                Vertex::new(p000, bottom_normal),
                Vertex::new(p010, bottom_normal),
                Vertex::new(p110, bottom_normal),
                Vertex::new(p100, bottom_normal),
            ],
            metadata.clone(),
        );

        // Top face (z=depth, normal approx. +Z)
        // p001 -> p011 -> p111 -> p101
        let top_normal = Vector3::z();
        let top = Polygon::new(
            vec![
                Vertex::new(p001, top_normal),
                Vertex::new(p101, top_normal),
                Vertex::new(p111, top_normal),
                Vertex::new(p011, top_normal),
            ],
            metadata.clone(),
        );

        // Front face (y=0, normal approx. -Y)
        // p000 -> p001 -> p101 -> p100
        let front_normal = -Vector3::y();
        let front = Polygon::new(
            vec![
                Vertex::new(p000, front_normal),
                Vertex::new(p100, front_normal),
                Vertex::new(p101, front_normal),
                Vertex::new(p001, front_normal),
            ],
            metadata.clone(),
        );

        // Back face (y=height, normal approx. +Y)
        // p010 -> p110 -> p111 -> p011
        let back_normal = Vector3::y();
        let back = Polygon::new(
            vec![
                Vertex::new(p010, back_normal),
                Vertex::new(p011, back_normal),
                Vertex::new(p111, back_normal),
                Vertex::new(p110, back_normal),
            ],
            metadata.clone(),
        );

        // Left face (x=0, normal approx. -X)
        // p000 -> p010 -> p011 -> p001
        let left_normal = -Vector3::x();
        let left = Polygon::new(
            vec![
                Vertex::new(p000, left_normal),
                Vertex::new(p001, left_normal),
                Vertex::new(p011, left_normal),
                Vertex::new(p010, left_normal),
            ],
            metadata.clone(),
        );

        // Right face (x=width, normal approx. +X)
        // p100 -> p101 -> p111 -> p110
        let right_normal = Vector3::x();
        let right = Polygon::new(
            vec![
                Vertex::new(p100, right_normal),
                Vertex::new(p110, right_normal),
                Vertex::new(p111, right_normal),
                Vertex::new(p101, right_normal),
            ],
            metadata.clone(),
        );

        // Combine all faces into a Mesh
        Mesh::from_polygons(&[bottom, top, front, back, left, right], metadata)
    }

    pub fn cube(width: Real, metadata: M) -> Mesh<M> {
        Self::cuboid(width, width, width, metadata)
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
    /// ```
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
    /// ```
    /// This is mathematically exact for spheres (no approximation needed).
    ///
    /// ### **Mesh Quality Metrics**
    /// - **Aspect Ratio**: Best when segments ≈ 2×stacks
    /// - **Area Distortion**: Minimal at equator, maximal at poles
    /// - **Angular Distortion**: Increases towards poles (unavoidable)
    ///
    /// ### **Numerical Considerations**
    /// - **Trigonometric Precision**: Uses TAU and PI for accuracy
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
        if !finite_mesh_scalar(radius) {
            return Mesh::empty(metadata);
        }
        let segments = segments.max(3);
        let stacks = stacks.max(2);
        let mut polygons = Vec::new();

        for i in 0..segments {
            for j in 0..stacks {
                let mut vertices = Vec::new();

                let vertex = |theta: Real, phi: Real| -> Option<Vertex> {
                    let (sin_theta, cos_theta) = hangle_sin_cos(theta)?;
                    let (sin_phi, cos_phi) = hangle_sin_cos(phi)?;
                    let dir = Vector3::new(
                        hreal_mul(cos_theta, sin_phi)?,
                        cos_phi,
                        hreal_mul(sin_theta, sin_phi)?,
                    );
                    Some(Vertex::new(
                        Point3::new(
                            hreal_mul(dir.x, radius)?,
                            hreal_mul(dir.y, radius)?,
                            hreal_mul(dir.z, radius)?,
                        ),
                        dir,
                    ))
                };

                let Some(t0) = hreal_div(i as Real, segments as Real) else {
                    return Mesh::empty(metadata);
                };
                let Some(t1) = hreal_div((i + 1) as Real, segments as Real) else {
                    return Mesh::empty(metadata);
                };
                let Some(p0) = hreal_div(j as Real, stacks as Real) else {
                    return Mesh::empty(metadata);
                };
                let Some(p1) = hreal_div((j + 1) as Real, stacks as Real) else {
                    return Mesh::empty(metadata);
                };

                let Some(theta0) = hreal_mul(t0, TAU) else {
                    return Mesh::empty(metadata);
                };
                let Some(theta1) = hreal_mul(t1, TAU) else {
                    return Mesh::empty(metadata);
                };
                let Some(phi0) = hreal_mul(p0, PI) else {
                    return Mesh::empty(metadata);
                };
                let Some(phi1) = hreal_mul(p1, PI) else {
                    return Mesh::empty(metadata);
                };

                let Some(first) = vertex(theta0, phi0) else {
                    return Mesh::empty(metadata);
                };
                vertices.push(first);
                if j > 0 {
                    let Some(v) = vertex(theta1, phi0) else {
                        return Mesh::empty(metadata);
                    };
                    vertices.push(v);
                }
                if j < stacks - 1 {
                    let Some(v) = vertex(theta1, phi1) else {
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
    /// ```
    /// use csgrs::mesh::Mesh;
    /// use nalgebra::Point3;
    /// let bottom = Point3::new(0.0, 0.0, 0.0);
    /// let top = Point3::new(0.0, 0.0, 5.0);
    /// // This will create a cone (bottom degenerate) because radius1 is 0:
    /// let cone = Mesh::<()>::frustum_ptp(bottom, top, 0.0, 2.0, 32, ());
    /// ```
    pub fn frustum_ptp(
        start: Point3<Real>,
        end: Point3<Real>,
        radius1: Real,
        radius2: Real,
        segments: usize,
        metadata: M,
    ) -> Mesh<M> {
        if !finite_mesh_point(&start)
            || !finite_mesh_point(&end)
            || !finite_mesh_scalar(radius1)
            || !finite_mesh_scalar(radius2)
        {
            return Mesh::empty(metadata);
        }
        let segments = segments.max(3);
        // Compute the axis and check that start and end do not coincide.
        //
        // The axis length and checked unit direction are evaluated through
        // `hyperlattice::Vector3`/`hyperreal::Real`, then exported only for the
        // finite mesh construction loops. This follows Yap's exact-geometric-
        // computation boundary discipline, *Computational Geometry* 7(1-2),
        // 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        let s = start.coords;
        let e = end.coords;
        let ray = e - s;
        let Some((axis_z, axis_length)) = hunit_vector3_and_magnitude(&ray) else {
            return Mesh::empty(metadata);
        };
        if matches!(
            hreal_cmp_f64(axis_length, tolerance()),
            Ordering::Less | Ordering::Equal
        ) {
            return Mesh::empty(metadata);
        }

        // Pick axes in hyperlattice so primitive floats only carry the final
        // mesh-boundary basis.
        let Some((axis_x, axis_y)) = hperpendicular_basis(&axis_z) else {
            return Mesh::empty(metadata);
        };

        // The cap centers for the bottom and top.
        let start_v = Vertex::new(start, -axis_z);
        let end_v = Vertex::new(end, axis_z);

        // A closure that returns a vertex on the lateral surface.
        // For a given stack (0.0 for bottom, 1.0 for top), slice (fraction along the circle),
        // and a normal blend factor (used for cap smoothing), compute the vertex.
        let point = |stack: Real, slice: Real, normal_blend: Real| -> Option<Vertex> {
            // Linear interpolation of radius.
            let radius_delta = hreal_sub(radius2, radius1)?;
            let r = hreal_affine(radius1, stack, radius_delta)?;
            let angle = hreal_mul(slice, TAU)?;
            let (sin_angle, cos_angle) = hangle_sin_cos(angle)?;
            let radial_dir = Vector3::new(
                hreal_sum(&[
                    hreal_mul(axis_x.x, cos_angle)?,
                    hreal_mul(axis_y.x, sin_angle)?,
                ])?,
                hreal_sum(&[
                    hreal_mul(axis_x.y, cos_angle)?,
                    hreal_mul(axis_y.y, sin_angle)?,
                ])?,
                hreal_sum(&[
                    hreal_mul(axis_x.z, cos_angle)?,
                    hreal_mul(axis_y.z, sin_angle)?,
                ])?,
            );
            let position = Vector3::new(
                hreal_sum(&[s.x, hreal_mul(ray.x, stack)?, hreal_mul(radial_dir.x, r)?])?,
                hreal_sum(&[s.y, hreal_mul(ray.y, stack)?, hreal_mul(radial_dir.y, r)?])?,
                hreal_sum(&[s.z, hreal_mul(ray.z, stack)?, hreal_mul(radial_dir.z, r)?])?,
            );
            let radial_normal_scale = hreal_sub(1.0, hreal_abs(normal_blend)?)?;
            let normal = Vector3::new(
                hreal_sum(&[
                    hreal_mul(radial_dir.x, radial_normal_scale)?,
                    hreal_mul(axis_z.x, normal_blend)?,
                ])?,
                hreal_sum(&[
                    hreal_mul(radial_dir.y, radial_normal_scale)?,
                    hreal_mul(axis_z.y, normal_blend)?,
                ])?,
                hreal_sum(&[
                    hreal_mul(radial_dir.z, radial_normal_scale)?,
                    hreal_mul(axis_z.z, normal_blend)?,
                ])?,
            );
            Some(Vertex::new(
                Point3::from(position),
                hunit_vector3(&normal).unwrap_or(axis_z),
            ))
        };

        let mut polygons = Vec::new();

        // Special-case flags for degenerate faces.
        let bottom_degenerate = matches!(
            hreal_abs(radius1).map(|value| hreal_cmp_f64(value, tolerance())),
            Some(Ordering::Less)
        );
        let top_degenerate = matches!(
            hreal_abs(radius2).map(|value| hreal_cmp_f64(value, tolerance())),
            Some(Ordering::Less)
        );

        // If both faces are degenerate, we cannot build a meaningful volume.
        if bottom_degenerate && top_degenerate {
            return Mesh::empty(metadata);
        }

        // For each slice of the circle (0..segments)
        for i in 0..segments {
            let Some(slice0) = hreal_div(i as Real, segments as Real) else {
                return Mesh::empty(metadata);
            };
            let Some(slice1) = hreal_div((i + 1) as Real, segments as Real) else {
                return Mesh::empty(metadata);
            };

            // In the normal frustum_ptp, we always add a bottom cap triangle (fan) and a top cap triangle.
            // Here, we only add the cap triangle if the corresponding radius is not degenerate.
            if !bottom_degenerate {
                // Bottom cap: a triangle fan from the bottom center to two consecutive points on the bottom ring.
                let (Some(p0), Some(p1)) =
                    (point(0.0, slice0, -1.0), point(0.0, slice1, -1.0))
                else {
                    return Mesh::empty(metadata);
                };
                polygons.push(Polygon::new(vec![start_v, p0, p1], metadata.clone()));
            }
            if !top_degenerate {
                // Top cap: a triangle fan from the top center to two consecutive points on the top ring.
                let (Some(p0), Some(p1)) = (point(1.0, slice1, 1.0), point(1.0, slice0, 1.0))
                else {
                    return Mesh::empty(metadata);
                };
                polygons.push(Polygon::new(vec![end_v, p0, p1], metadata.clone()));
            }

            // For the side wall, we normally build a quad spanning from the bottom ring (stack=0)
            // to the top ring (stack=1). If one of the rings is degenerate, that ring reduces to a single point.
            // In that case, we output a triangle.
            if bottom_degenerate {
                // Bottom is a point (start_v); create a triangle from start_v to two consecutive points on the top ring.
                let (Some(p0), Some(p1)) = (point(1.0, slice0, 0.0), point(1.0, slice1, 0.0))
                else {
                    return Mesh::empty(metadata);
                };
                polygons.push(Polygon::new(vec![start_v, p0, p1], metadata.clone()));
            } else if top_degenerate {
                // Top is a point (end_v); create a triangle from two consecutive points on the bottom ring to end_v.
                let (Some(p0), Some(p1)) = (point(0.0, slice1, 0.0), point(0.0, slice0, 0.0))
                else {
                    return Mesh::empty(metadata);
                };
                polygons.push(Polygon::new(vec![p0, p1, end_v], metadata.clone()));
            } else {
                // Normal case: both rings are non-degenerate. Use a quad for the side wall.
                let (Some(p0), Some(p1), Some(p2), Some(p3)) = (
                    point(0.0, slice1, 0.0),
                    point(0.0, slice0, 0.0),
                    point(1.0, slice0, 0.0),
                    point(1.0, slice1, 0.0),
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
            Point3::new(0.0, 0.0, height),
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
            Point3::new(0.0, 0.0, height),
            radius,
            radius,
            segments,
            metadata,
        )
    }

    /// Creates a Mesh polyhedron from raw vertex data (`points`) and face indices.
    ///
    /// Raw coordinates are API-boundary data. Each selected point is promoted
    /// through `hyperlattice::Vector3<hyperreal::Real>` before a polygon vertex
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
    /// ```
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
                let [x, y, z] = points[idx];
                let point = Point3::new(x, y, z);
                if hvector3_from_point3(&point).is_none() {
                    return Err(ValidationError::InvalidCoordinate(point));
                }
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
                v.normal = plane_normal;
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
        if !finite_mesh_scalar(width) || !finite_mesh_scalar(length) || revolve_segments < 3 {
            return Mesh::empty(metadata);
        }

        let egg_2d = Profile::egg(width, length, outline_segments, metadata.clone());

        // Build a large rectangle that cuts off everything
        let cutter_height = 9999.0; // some large number
        let rect_cutter = Profile::square(cutter_height, metadata.clone()).translate(
            -cutter_height,
            -cutter_height / 2.0,
            0.0,
        );

        let half_egg = egg_2d.difference(&rect_cutter);

        half_egg
            .revolve(360.0, revolve_segments)
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
        if !finite_mesh_scalar(width) || !finite_mesh_scalar(length) || revolve_segments < 3 {
            return Mesh::empty(metadata);
        }

        // Make a 2D teardrop in the XY plane.
        let td_2d = Profile::teardrop(width, length, shape_segments, metadata.clone());

        // Build a large rectangle that cuts off everything
        let cutter_height = 9999.0; // some large number
        let rect_cutter = Profile::square(cutter_height, metadata.clone()).translate(
            -cutter_height,
            -cutter_height / 2.0,
            0.0,
        );

        let half_teardrop = td_2d.difference(&rect_cutter);

        // revolve 360 degrees
        half_teardrop
            .revolve(360.0, revolve_segments)
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
        if !(finite_mesh_scalar(width)
            && finite_mesh_scalar(length)
            && finite_mesh_scalar(height))
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
        if !(finite_mesh_scalar(rx) && finite_mesh_scalar(ry) && finite_mesh_scalar(rz)) {
            return Mesh::empty(metadata);
        }

        let base_sphere = Self::sphere(1.0, segments, stacks, metadata.clone());
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
        start: Point3<Real>,
        direction: Vector3<Real>,
        segments: usize,
        orientation: bool,
        metadata: M,
    ) -> Mesh<M> {
        if !finite_mesh_point(&start) {
            return Mesh::empty(metadata);
        }
        // Compute the arrow's total length and unit direction through the
        // hyper geometry boundary used by primitive axis construction.
        let Some((unit_dir, arrow_length)) = hunit_vector3_and_magnitude(&direction) else {
            return Mesh::empty(metadata);
        };
        if matches!(
            hreal_cmp_f64(arrow_length, tolerance()),
            Ordering::Less | Ordering::Equal
        ) {
            return Mesh::empty(metadata);
        }

        // Define proportions:
        // - Arrow head occupies 20% of total length.
        // - Shaft occupies the remainder.
        let Some(head_length) = hreal_mul(arrow_length, 0.2) else {
            return Mesh::empty(metadata);
        };
        let Some(shaft_length) = hreal_sub(arrow_length, head_length) else {
            return Mesh::empty(metadata);
        };

        // Define thickness parameters proportional to the arrow length.
        let Some(shaft_radius) = hreal_mul(arrow_length, 0.03) else {
            return Mesh::empty(metadata);
        };
        let Some(head_base_radius) = hreal_mul(arrow_length, 0.06) else {
            return Mesh::empty(metadata);
        };
        let Some(tip_radius) = hreal_mul(arrow_length, 0.0) else {
            return Mesh::empty(metadata);
        };

        // Build the shaft as a vertical cylinder along Z from 0 to shaft_length.
        let shaft = Mesh::cylinder(shaft_radius, shaft_length, segments, metadata.clone());

        // Build the arrow head as a frustum from z = shaft_length to z = shaft_length + head_length.
        let head = Mesh::frustum_ptp(
            Point3::new(0.0, 0.0, shaft_length),
            Point3::new(0.0, 0.0, arrow_length),
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
            let l = arrow_length;
            let Some(half_length) = hreal_div(l, 2.0) else {
                return Mesh::empty(metadata);
            };
            let Some(negative_half_length) = hreal_sub(0.0, half_length) else {
                return Mesh::empty(metadata);
            };
            let Some(to_midpoint) = htranslation_matrix(&Vector3::new(0.0, 0.0, half_length))
            else {
                return Mesh::empty(metadata);
            };
            let Some(reflect_z) = hscale_matrix(1.0, 1.0, -1.0) else {
                return Mesh::empty(metadata);
            };
            let Some(from_midpoint) =
                htranslation_matrix(&Vector3::new(0.0, 0.0, negative_half_length))
            else {
                return Mesh::empty(metadata);
            };
            let mirror_mat: Matrix4<Real> = to_midpoint * reflect_z * from_midpoint;
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
        let rot_mat: Matrix4<Real> =
            hrotation_between_vectors(&z_axis, &unit_dir).unwrap_or_else(Matrix4::identity);

        // Rotate the arrow.
        let rotated_arrow = canonical_arrow.transform(&rot_mat);

        // Finally, translate the arrow so that the anchored vertex (canonical (0,0,0)) moves to 'start'.
        // In the false case, (0,0,0) is the base (arrow extends from start to start+direction).
        // In the true case, after mirroring, (0,0,0) is the tip (arrow extends from start to start+direction).
        rotated_arrow.translate(start.x, start.y, start.z)
    }

    /// Regular octahedron scaled by `radius`
    pub fn octahedron(radius: Real, metadata: M) -> Self {
        if !finite_mesh_scalar(radius) {
            return Mesh::empty(metadata);
        }
        let pts = &[
            [1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
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
            .map(|&[x, y, z]| [x * radius, y * radius, z * radius])
            .collect();
        Self::polyhedron(&scaled, &faces, metadata.clone())
            .unwrap_or_else(|_| Mesh::empty(metadata))
    }

    /// Regular icosahedron scaled by `radius`.
    ///
    /// The golden-ratio normalization is evaluated through `hyperreal::Real`
    /// before finite mesh vertices are emitted. This keeps even constant
    /// shape-construction algebra on the exact-aware side of the boundary,
    /// following Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The construction is
    /// the classical regular icosahedron coordinate model using the golden
    /// ratio; see Coxeter, *Regular Polytopes*, 3rd ed., 1973.
    pub fn icosahedron(radius: Real, metadata: M) -> Self {
        if !finite_mesh_scalar(radius) {
            return Mesh::empty(metadata);
        }
        // radius scale factor
        let Some(factor) = hreal_mul(radius, 0.5878) else {
            return Mesh::empty(metadata);
        };
        // golden ratio
        let Some(phi) = hreal_affine(0.5, 0.5, hreal_sqrt(5.0).unwrap_or(0.0)) else {
            return Mesh::empty(metadata);
        };
        // normalise so the circum-radius is 1
        let Some(phi_squared) = hreal_mul(phi, phi) else {
            return Mesh::empty(metadata);
        };
        let Some(len_squared) = hreal_affine(1.0, 1.0, phi_squared) else {
            return Mesh::empty(metadata);
        };
        let Some(len) = hreal_sqrt(len_squared) else {
            return Mesh::empty(metadata);
        };
        let Some(inv_len) = hreal_div(1.0, len) else {
            return Mesh::empty(metadata);
        };
        let a = inv_len;
        let Some(b) = hreal_mul(phi, inv_len) else {
            return Mesh::empty(metadata);
        };

        // 12 vertices ----------------------------------------------------
        let pts: [[Real; 3]; 12] = [
            [-a, b, 0.0],
            [a, b, 0.0],
            [-a, -b, 0.0],
            [a, -b, 0.0],
            [0.0, -a, b],
            [0.0, a, b],
            [0.0, -a, -b],
            [0.0, a, -b],
            [b, 0.0, -a],
            [b, 0.0, a],
            [-b, 0.0, -a],
            [-b, 0.0, a],
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
            .map(|mesh| mesh.scale(factor, factor, factor))
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
        if !finite_mesh_scalar(major_r)
            || !finite_mesh_scalar(minor_r)
            || !matches!(
                hreal_abs(major_r).map(|value| hreal_cmp_f64(value, tolerance())),
                Some(Ordering::Greater)
            )
            || !matches!(
                hreal_abs(minor_r).map(|value| hreal_cmp_f64(value, tolerance())),
                Some(Ordering::Greater)
            )
            || segments_major < 3
            || segments_minor < 3
        {
            return Mesh::empty(metadata);
        }

        let Some(profile_offset) = hreal_mul(major_r, 1.0) else {
            return Mesh::empty(metadata);
        };
        let circle = Profile::circle(minor_r, segments_minor, metadata.clone()).translate(
            profile_offset,
            0.0,
            0.0,
        );
        circle
            .revolve(360.0, segments_major)
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
        if !finite_mesh_scalar(thickness) {
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
        if !finite_mesh_scalar(thickness) {
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
        if slices < 2 || !finite_mesh_scalar(thickness) || !finite_mesh_scalar(helix_angle_deg)
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

        let dz = thickness / (slices as Real);
        let Some(d_psi_rad) = hdegrees_to_radians(helix_angle_deg)
            .and_then(|angle| hreal_div(angle, slices as Real))
        else {
            return Mesh::empty(metadata);
        };
        if !finite_mesh_scalar(dz) || !finite_mesh_scalar(d_psi_rad) {
            return Mesh::empty(metadata);
        }

        let mut acc = Mesh::empty(metadata.clone());
        let mut z_curr = 0.0;
        for i in 0..slices {
            let Some(rotation_deg) =
                hreal_mul(i as Real, d_psi_rad).and_then(hradians_to_degrees)
            else {
                return Mesh::empty(metadata);
            };
            let slice = base_slice
                .rotate(0.0, 0.0, rotation_deg)
                .extrude(dz)
                .translate(0.0, 0.0, z_curr);
            acc = if i == 0 { slice } else { acc.union(&slice) };
            z_curr += dz;
        }
        acc
    }
}
