use crate::csg::CSG;
use crate::float_types::{EPSILON, PI, Real, TAU};
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use nalgebra::{Matrix4, Point3, Rotation3, Translation3, Vector3};
use std::fmt::Debug;

impl<S: Clone + Debug> CSG<S>
where S: Clone + Send + Sync {
    /// Create a right prism (a box) that spans from (0, 0, 0)
    /// to (width, length, height). All dimensions must be >= 0.
    pub fn cube(width: Real, length: Real, height: Real, metadata: Option<S>) -> CSG<S> {
        // Define the eight corner points of the prism.
        //    (x, y, z)
        let p000 = Point3::new(0.0,      0.0,      0.0);
        let p100 = Point3::new(width,    0.0,      0.0);
        let p110 = Point3::new(width,    length,   0.0);
        let p010 = Point3::new(0.0,      length,   0.0);

        let p001 = Point3::new(0.0,      0.0,      height);
        let p101 = Point3::new(width,    0.0,      height);
        let p111 = Point3::new(width,    length,   height);
        let p011 = Point3::new(0.0,      length,   height);

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

        // Combine all faces into a CSG
        CSG::from_polygons(&[bottom, top, front, back, left, right])
    }

    /// Construct a sphere with radius, segments, stacks
    pub fn sphere(radius: Real, segments: usize, stacks: usize, metadata: Option<S>) -> CSG<S> {
        let mut polygons = Vec::new();

        for i in 0..segments {
            for j in 0..stacks {
                let mut vertices = Vec::new();

                let vertex = |theta: Real, phi: Real| {
                    let dir =
                        Vector3::new(theta.cos() * phi.sin(), phi.cos(), theta.sin() * phi.sin());
                    Vertex::new(
                        Point3::new(dir.x * radius, dir.y * radius, dir.z * radius),
                        dir,
                    )
                };

                let t0 = i as Real / segments as Real;
                let t1 = (i + 1) as Real / segments as Real;
                let p0 = j as Real / stacks as Real;
                let p1 = (j + 1) as Real / stacks as Real;

                let theta0 = t0 * TAU;
                let theta1 = t1 * TAU;
                let phi0 = p0 * PI;
                let phi1 = p1 * PI;

                vertices.push(vertex(theta0, phi0));
                if j > 0 {
                    vertices.push(vertex(theta1, phi0));
                }
                if j < stacks - 1 {
                    vertices.push(vertex(theta1, phi1));
                }
                vertices.push(vertex(theta0, phi1));

                polygons.push(Polygon::new(vertices, metadata.clone()));
            }
        }
        CSG::from_polygons(&polygons)
    }

    /// Constructs a frustum between `start` and `end` with bottom radius = `radius1` and
    /// top radius = `radius2`. In the normal case, it creates side quads and cap triangles.
    /// However, if one of the radii is 0 (within EPSILON), then the degenerate face is treated
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
    /// let bottom = Point3::new(0.0, 0.0, 0.0);
    /// let top = Point3::new(0.0, 0.0, 5.0);
    /// // This will create a cone (bottom degenerate) because radius1 is 0:
    /// let cone = CSG::frustum_ptp_special(bottom, top, 0.0, 2.0, 32, None);
    /// ```
    pub fn frustum_ptp(
        start: Point3<Real>,
        end: Point3<Real>,
        radius1: Real,
        radius2: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        // Compute the axis and check that start and end do not coincide.
        let s = start.coords;
        let e = end.coords;
        let ray = e - s;
        if ray.norm_squared() < EPSILON {
            return CSG::new();
        }
        let axis_z = ray.normalize();
        // Pick an axis not parallel to axis_z.
        let axis_x = if axis_z.y.abs() > 0.5 {
            Vector3::x()
        } else {
            Vector3::y()
        }
        .cross(&axis_z)
        .normalize();
        let axis_y = axis_x.cross(&axis_z).normalize();

        // The cap centers for the bottom and top.
        let start_v = Vertex::new(start, -axis_z);
        let end_v = Vertex::new(end, axis_z);

        // A closure that returns a vertex on the lateral surface.
        // For a given stack (0.0 for bottom, 1.0 for top), slice (fraction along the circle),
        // and a normal blend factor (used for cap smoothing), compute the vertex.
        let point = |stack: Real, slice: Real, normal_blend: Real| {
            // Linear interpolation of radius.
            let r = radius1 * (1.0 - stack) + radius2 * stack;
            let angle = slice * TAU;
            let radial_dir = axis_x * angle.cos() + axis_y * angle.sin();
            let pos = s + ray * stack + radial_dir * r;
            let normal = radial_dir * (1.0 - normal_blend.abs()) + axis_z * normal_blend;
            Vertex::new(Point3::from(pos), normal.normalize())
        };

        let mut polygons = Vec::new();

        // Special-case flags for degenerate faces.
        let bottom_degenerate = radius1.abs() < EPSILON;
        let top_degenerate = radius2.abs() < EPSILON;

        // If both faces are degenerate, we cannot build a meaningful volume.
        if bottom_degenerate && top_degenerate {
            return CSG::new();
        }

        // For each slice of the circle (0..segments)
        for i in 0..segments {
            let slice0 = i as Real / segments as Real;
            let slice1 = (i + 1) as Real / segments as Real;

            // In the normal frustum_ptp, we always add a bottom cap triangle (fan) and a top cap triangle.
            // Here, we only add the cap triangle if the corresponding radius is not degenerate.
            if !bottom_degenerate {
                // Bottom cap: a triangle fan from the bottom center to two consecutive points on the bottom ring.
                polygons.push(Polygon::new(
                    vec![
                        start_v.clone(),
                        point(0.0, slice0, -1.0),
                        point(0.0, slice1, -1.0),
                    ],
                    metadata.clone(),
                ));
            }
            if !top_degenerate {
                // Top cap: a triangle fan from the top center to two consecutive points on the top ring.
                polygons.push(Polygon::new(
                    vec![
                        end_v.clone(),
                        point(1.0, slice1, 1.0),
                        point(1.0, slice0, 1.0),
                    ],
                    metadata.clone(),
                ));
            }

            // For the side wall, we normally build a quad spanning from the bottom ring (stack=0)
            // to the top ring (stack=1). If one of the rings is degenerate, that ring reduces to a single point.
            // In that case, we output a triangle.
            if bottom_degenerate {
                // Bottom is a point (start_v); create a triangle from start_v to two consecutive points on the top ring.
                polygons.push(Polygon::new(
                    vec![
                        start_v.clone(),
                        point(1.0, slice0, 0.0),
                        point(1.0, slice1, 0.0),
                    ],
                    metadata.clone(),
                ));
            } else if top_degenerate {
                // Top is a point (end_v); create a triangle from two consecutive points on the bottom ring to end_v.
                polygons.push(Polygon::new(
                    vec![
                        point(0.0, slice1, 0.0),
                        point(0.0, slice0, 0.0),
                        end_v.clone(),
                    ],
                    metadata.clone(),
                ));
            } else {
                // Normal case: both rings are non-degenerate. Use a quad for the side wall.
                polygons.push(Polygon::new(
                    vec![
                        point(0.0, slice1, 0.0),
                        point(0.0, slice0, 0.0),
                        point(1.0, slice0, 0.0),
                        point(1.0, slice1, 0.0),
                    ],
                    metadata.clone(),
                ));
            }
        }

        CSG::from_polygons(&polygons)
    }

    /// A helper to create a vertical cylinder along Z from z=0..z=height
    /// with the specified radius (NOT diameter).
    pub fn frustum(
        radius1: Real,
        radius2: Real,
        height: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        CSG::frustum_ptp(
            Point3::origin(),
            Point3::new(0.0, 0.0, height),
            radius1,
            radius2,
            segments,
            metadata,
        )
    }
    
    /// A helper to create a vertical cylinder along Z from z=0..z=height
    // with the specified radius (NOT diameter).
    pub fn cylinder(radius: Real, height: Real, segments: usize, metadata: Option<S>) -> CSG<S> {
        CSG::frustum_ptp(
            Point3::origin(),
            Point3::new(0.0, 0.0, height),
            radius.clone(),
            radius,
            segments,
            metadata,
        )
    }

    /// Creates a CSG polyhedron from raw vertex data (`points`) and face indices.
    ///
    /// # Parameters
    ///
    /// - `points`: a slice of `[x,y,z]` coordinates.
    /// - `faces`: each element is a list of indices into `points`, describing one face.
    ///   Each face must have at least 3 indices.
    ///
    /// # Example
    /// ```
    /// let pts = &[
    ///     [0.0, 0.0, 0.0], // point0
    ///     [1.0, 0.0, 0.0], // point1
    ///     [1.0, 1.0, 0.0], // point2
    ///     [0.0, 1.0, 0.0], // point3
    ///     [0.5, 0.5, 1.0], // point4 - top
    /// ];
    ///
    /// // Two faces: bottom square [0,1,2,3], and a pyramid side [0,1,4]
    /// let fcs = vec![
    ///     vec![0, 1, 2, 3],
    ///     vec![0, 1, 4],
    ///     vec![1, 2, 4],
    ///     vec![2, 3, 4],
    ///     vec![3, 0, 4],
    /// ];
    ///
    /// let csg_poly = CSG::polyhedron(pts, &fcs);
    /// ```
    pub fn polyhedron(points: &[[Real; 3]], faces: &[Vec<usize>], metadata: Option<S>) -> CSG<S> {
        let mut polygons = Vec::new();

        for face in faces {
            // Skip degenerate faces
            if face.len() < 3 {
                continue;
            }

            // Gather the vertices for this face
            let mut face_vertices = Vec::with_capacity(face.len());
            for &idx in face {
                // Ensure the index is valid
                if idx >= points.len() {
                    panic!( // todo return error
                        "Face index {} is out of range (points.len = {}).",
                        idx,
                        points.len()
                    );
                }
                let [x, y, z] = points[idx];
                face_vertices.push(Vertex::new(
                    Point3::new(x, y, z),
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

        CSG::from_polygons(&polygons)
    }

    /// Creates a 3D "egg" shape by revolving the existing 2D `egg_outline` profile.
    ///
    /// # Parameters
    /// - `width`: The "width" of the 2D egg outline.
    /// - `length`: The "length" (height) of the 2D egg outline.
    /// - `revolve_segments`: Number of segments for the revolution.
    /// - `outline_segments`: Number of segments for the 2D egg outline itself.
    /// - `metadata`: Optional metadata.
    #[cfg(feature = "chull-io")]
    pub fn egg(
        width: Real,
        length: Real,
        revolve_segments: usize,
        outline_segments: usize,
        metadata: Option<S>,
    ) -> Self {
        let egg_2d = Self::egg_outline(width, length, outline_segments, metadata.clone());

        // Build a large rectangle that cuts off everything
        let cutter_height = 9999.0; // some large number
        let rect_cutter = CSG::square(cutter_height, cutter_height, metadata.clone()).translate(
            -cutter_height,
            -cutter_height / 2.0,
            0.0,
        );

        let half_egg = egg_2d.difference(&rect_cutter);

        half_egg
            .rotate_extrude(360.0, revolve_segments)
            .convex_hull()
    }

    /// Creates a 3D "teardrop" solid by revolving the existing 2D `teardrop` profile 360° around the Y-axis (via rotate_extrude).
    ///
    /// # Parameters
    /// - `width`: Width of the 2D teardrop profile.
    /// - `length`: Length of the 2D teardrop profile.
    /// - `revolve_segments`: Number of segments for the revolution (the "circular" direction).
    /// - `shape_segments`: Number of segments for the 2D teardrop outline itself.
    /// - `metadata`: Optional metadata.
    #[cfg(feature = "chull-io")]
    pub fn teardrop(
        width: Real,
        length: Real,
        revolve_segments: usize,
        shape_segments: usize,
        metadata: Option<S>,
    ) -> Self {
        // Make a 2D teardrop in the XY plane.
        let td_2d = Self::teardrop_outline(width, length, shape_segments, metadata.clone());

        // Build a large rectangle that cuts off everything
        let cutter_height = 9999.0; // some large number
        let rect_cutter = CSG::square(cutter_height, cutter_height, metadata.clone())
            .translate(-cutter_height, -cutter_height / 2.0, 0.0);

        let half_teardrop = td_2d.difference(&rect_cutter);

        // revolve 360 degrees
        half_teardrop.rotate_extrude(360.0, revolve_segments).convex_hull()
    }

    /// Creates a 3D "teardrop cylinder" by extruding the existing 2D `teardrop` in the Z+ axis.
    ///
    /// # Parameters
    /// - `width`: Width of the 2D teardrop profile.
    /// - `length`: Length of the 2D teardrop profile.
    /// - `revolve_segments`: Number of segments for the revolution (the "circular" direction).
    /// - `shape_segments`: Number of segments for the 2D teardrop outline itself.
    /// - `metadata`: Optional metadata.
    #[cfg(feature = "chull-io")]
    pub fn teardrop_cylinder(
        width: Real,
        length: Real,
        height: Real,
        shape_segments: usize,
        metadata: Option<S>,
    ) -> Self {
        // Make a 2D teardrop in the XY plane.
        let td_2d = Self::teardrop_outline(width, length, shape_segments, metadata.clone());
        td_2d.extrude(height).convex_hull()
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
        metadata: Option<S>,
    ) -> Self {
        let base_sphere = Self::sphere(1.0, segments, stacks, metadata.clone());
        base_sphere.scale(rx, ry, rz)
    }

    /// Creates an arrow CSG. The arrow is composed of:
    ///   - a cylindrical shaft, and
    ///   - a cone–like head (a frustum from a larger base to a small tip)
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
    /// - `orientation`: when false (default) the arrow points away from start (its base is at start);
    ///                        when true the arrow points toward start (its tip is at start).
    /// - `metadata`: optional metadata for the generated polygons.
    pub fn arrow(
        start: Point3<Real>,
        direction: Vector3<Real>,
        segments: usize,
        orientation: bool,
        metadata: Option<S>,
    ) -> CSG<S> {
        // Compute the arrow's total length.
        let arrow_length = direction.norm();
        if arrow_length < EPSILON {
            return CSG::new();
        }
        // Compute the unit direction.
        let unit_dir = direction / arrow_length;

        // Define proportions:
        // - Arrow head occupies 20% of total length.
        // - Shaft occupies the remainder.
        let head_length = arrow_length * 0.2;
        let shaft_length = arrow_length - head_length;

        // Define thickness parameters proportional to the arrow length.
        let shaft_radius = arrow_length * 0.03; // shaft radius
        let head_base_radius = arrow_length * 0.06; // head base radius (wider than shaft)
        let tip_radius = arrow_length * 0.0; // tip radius (nearly a point)

        // Build the shaft as a vertical cylinder along Z from 0 to shaft_length.
        let shaft = CSG::cylinder(shaft_radius, shaft_length, segments, metadata.clone());

        // Build the arrow head as a frustum from z = shaft_length to z = shaft_length + head_length.
        let head = CSG::frustum_ptp(
            Point3::new(0.0, 0.0, shaft_length),
            Point3::new(0.0, 0.0, shaft_length + head_length),
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
            let mirror_mat: Matrix4<Real> =
                Translation3::new(0.0, 0.0, l / 2.0).to_homogeneous()
                * Matrix4::new_nonuniform_scaling(&Vector3::new(1.0, 1.0, -1.0))
                * Translation3::new(0.0, 0.0, -l / 2.0).to_homogeneous();
            canonical_arrow = canonical_arrow.transform(&mirror_mat).inverse();
        }
        // In both cases, we now have a canonical arrow that extends from z=0 to z=arrow_length.
        // For orientation == false, z=0 is the base.
        // For orientation == true, after mirroring z=0 is now the tip.

        // Compute the rotation that maps the canonical +Z axis to the provided direction.
        let z_axis = Vector3::z();
        let rotation = Rotation3::rotation_between(&z_axis, &unit_dir)
            .unwrap_or_else(Rotation3::identity);
        let rot_mat: Matrix4<Real> = rotation.to_homogeneous();

        // Rotate the arrow.
        let rotated_arrow = canonical_arrow.transform(&rot_mat);

        // Finally, translate the arrow so that the anchored vertex (canonical (0,0,0)) moves to 'start'.
        // In the false case, (0,0,0) is the base (arrow extends from start to start+direction).
        // In the true case, after mirroring, (0,0,0) is the tip (arrow extends from start to start+direction).
        let final_arrow = rotated_arrow.translate(start.x, start.y, start.z);

        final_arrow
    }
    
    /// Regular octahedron scaled by `radius`
    pub fn octahedron(radius: Real, metadata: Option<S>) -> Self {
        let pts = &[
            [ 1.0,  0.0,  0.0],
            [-1.0,  0.0,  0.0],
            [ 0.0,  1.0,  0.0],
            [ 0.0, -1.0,  0.0],
            [ 0.0,  0.0,  1.0],
            [ 0.0,  0.0, -1.0],
        ];
        let faces = vec![
            vec![0, 2, 4], vec![2, 1, 4], vec![1, 3, 4], vec![3, 0, 4],
            vec![5, 2, 0], vec![5, 1, 2], vec![5, 3, 1], vec![5, 0, 3],
        ];
        let scaled: Vec<[Real; 3]> = pts.iter()
            .map(|&[x, y, z]| [x * radius, y * radius, z * radius])
            .collect();
        Self::polyhedron(&scaled, &faces, metadata)
    }

    /// Regular icosahedron scaled by `radius`
    pub fn icosahedron(radius: Real, metadata: Option<S>) -> Self {
        // radius scale factor
        let factor = radius * 0.5878;
        // golden ratio
        let phi: Real = (1.0 + 5.0_f64.sqrt() as Real) * 0.5;
        // normalise so the circum-radius is 1
        let inv_len = (1.0 + phi * phi).sqrt().recip();
        let a = inv_len;
        let b = phi * inv_len;
    
        // 12 vertices ----------------------------------------------------
        let pts: [[Real; 3]; 12] = [
            [-a,  b,  0.0], [ a,  b,  0.0], [-a, -b,  0.0], [ a, -b,  0.0],
            [ 0.0, -a,  b], [ 0.0,  a,  b], [ 0.0, -a, -b], [ 0.0,  a, -b],
            [  b,  0.0, -a], [  b,  0.0,  a], [ -b,  0.0, -a], [ -b, 0.0,  a],
        ];
    
        // 20 faces (counter-clockwise when viewed from outside) ----------
        let faces: [[usize; 3]; 20] = [
            [0,11,5], [0,5,1], [0,1,7], [0,7,10], [0,10,11],
            [1,5,9], [5,11,4], [11,10,2], [10,7,6], [7,1,8],
            [3,9,4], [3,4,2], [3,2,6], [3,6,8], [3,8,9],
            [4,9,5], [2,4,11], [6,2,10], [8,6,7], [9,8,1],
        ];
    
        let faces_vec: Vec<Vec<usize>> =
            faces.iter().map(|f| f.to_vec()).collect();
    
        Self::polyhedron(&pts, &faces_vec, metadata).scale(factor, factor, factor)
    }

    /// Torus centred at the origin in the *XY* plane.
    ///
    /// * `major_r` – distance from centre to tube centre ( R )  
    /// * `minor_r` – tube radius ( r )  
    /// * `segments_major` – number of segments around the donut  
    /// * `segments_minor` – segments of the tube cross-section
    pub fn torus(
        major_r: Real,
        minor_r: Real,
        segments_major: usize,
        segments_minor: usize,
        metadata: Option<S>,
    ) -> Self {
        let circle = CSG::circle(minor_r, segments_minor.max(3), metadata.clone())
            .translate(major_r, 0.0, 0.0);
        circle.rotate_extrude(360.0, segments_major.max(3))
    }
    
    pub fn spur_gear_involute(
        module_: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        thickness: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        CSG::involute_gear_2d(
            module_,
            teeth,
            pressure_angle_deg,
            clearance,
            backlash,
            segments_per_flank,
            metadata.clone(),
        )
        .extrude(thickness)
    }
    
    pub fn spur_gear_cycloid(
        module_: Real,
        teeth: usize,
        pin_teeth: usize,
        clearance: Real,
        segments_per_flank: usize,
        thickness: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        CSG::cycloidal_gear_2d(
            module_,
            teeth,
            pin_teeth,
            clearance,
            segments_per_flank,
            metadata.clone(),
        )
        .extrude(thickness)
    }
    
    // -------------------------------------------------------------------------------------------------
    // Helical involute gear (3‑D)                                                                    //
    // -------------------------------------------------------------------------------------------------
    
    pub fn helical_involute_gear(
        module_: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        thickness: Real,
        helix_angle_deg: Real,     // β
        slices: usize,             // ≥ 2 – axial divisions
        metadata: Option<S>,
    ) -> CSG<S> {
        assert!(slices >= 2);
        let base_slice = CSG::involute_gear_2d(
            module_,
            teeth,
            pressure_angle_deg,
            clearance,
            backlash,
            segments_per_flank,
            metadata.clone(),
        );
    
        let dz = thickness / (slices as Real);
        let d_ψ = helix_angle_deg.to_radians() / (slices as Real);
    
        let mut acc = CSG::<S>::new();
        let mut z_curr = 0.0;
        for i in 0..slices {
            let slice = base_slice
                .rotate(0.0, 0.0, (i as Real) * d_ψ.to_degrees())
                .extrude(dz)
                .translate(0.0, 0.0, z_curr);
            acc = if i == 0 { slice } else { acc.union(&slice) };
            z_curr += dz;
        }
        acc
    }

}
