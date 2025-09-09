//! 3D Shapes as `IndexedMesh`s with optimized indexed connectivity

use crate::IndexedMesh::plane::Plane;
use crate::IndexedMesh::{IndexedMesh, IndexedPolygon};
use crate::errors::ValidationError;
use crate::float_types::{EPSILON, PI, Real, TAU};
use crate::mesh::vertex::Vertex;
use crate::sketch::Sketch;
use crate::traits::CSG;
use nalgebra::{Matrix4, Point3, Rotation3, Translation3, Vector3};

use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> IndexedMesh<S> {
    /// **Mathematical Foundations for 3D Box Geometry with Indexed Connectivity**
    ///
    /// This implementation creates axis-aligned rectangular prisms (cuboids) using
    /// indexed mesh representation for optimal memory usage and connectivity performance.
    ///
    /// ## **Indexed Mesh Benefits**
    /// - **Memory Efficiency**: 8 vertices instead of 24 (6 faces × 4 vertices each)
    /// - **Connectivity Optimization**: Direct vertex index access for adjacency queries
    /// - **Cache Performance**: Better memory locality for vertex operations
    /// - **Topology Preservation**: Explicit vertex sharing maintains manifold properties
    ///
    /// ## **Vertex Indexing Strategy**
    /// ```text
    /// Vertex Layout (8 vertices total):
    ///     4-------5
    ///    /|      /|
    ///   0-------1 |
    ///   | |     | |
    ///   | 7-----|-6
    ///   |/      |/
    ///   3-------2
    /// ```
    ///
    /// ## **Face Connectivity (6 faces, each using 4 vertex indices)**
    /// - **Bottom**: [0,3,2,1] (z=0, normal -Z)
    /// - **Top**: [4,5,6,7] (z=height, normal +Z)  
    /// - **Front**: [0,1,5,4] (y=0, normal -Y)
    /// - **Back**: [3,7,6,2] (y=length, normal +Y)
    /// - **Left**: [0,4,7,3] (x=0, normal -X)
    /// - **Right**: [1,2,6,5] (x=width, normal +X)
    pub fn cuboid(
        width: Real,
        length: Real,
        height: Real,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        // Define the eight corner vertices once
        let vertices = vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::zeros()), // 0: origin
            Vertex::new(Point3::new(width, 0.0, 0.0), Vector3::zeros()), // 1: +X
            Vertex::new(Point3::new(width, length, 0.0), Vector3::zeros()), // 2: +X+Y
            Vertex::new(Point3::new(0.0, length, 0.0), Vector3::zeros()), // 3: +Y
            Vertex::new(Point3::new(0.0, 0.0, height), Vector3::zeros()), // 4: +Z
            Vertex::new(Point3::new(width, 0.0, height), Vector3::zeros()), // 5: +X+Z
            Vertex::new(Point3::new(width, length, height), Vector3::zeros()), // 6: +X+Y+Z
            Vertex::new(Point3::new(0.0, length, height), Vector3::zeros()), // 7: +Y+Z
        ];

        // Define faces using vertex indices with proper winding order (CCW from outside)
        let face_definitions = [
            // (indices, normal)
            (vec![0, 3, 2, 1], -Vector3::z()), // Bottom face
            (vec![4, 5, 6, 7], Vector3::z()),  // Top face
            (vec![0, 1, 5, 4], -Vector3::y()), // Front face
            (vec![3, 7, 6, 2], Vector3::y()),  // Back face
            (vec![0, 4, 7, 3], -Vector3::x()), // Left face
            (vec![1, 2, 6, 5], Vector3::x()),  // Right face
        ];

        let mut polygons = Vec::new();
        for (indices, normal) in face_definitions {
            let plane =
                Plane::from_normal(normal, normal.dot(&vertices[indices[0]].pos.coords));
            let indexed_poly = IndexedPolygon::new(indices, plane, metadata.clone());
            polygons.push(indexed_poly);
        }

        // Create the indexed mesh with shared vertices
        let mut mesh = IndexedMesh {
            vertices,
            polygons,
            bounding_box: std::sync::OnceLock::new(),
            metadata,
        };

        // Update vertex normals based on face adjacency
        mesh.compute_vertex_normals();
        mesh
    }

    pub fn cube(width: Real, metadata: Option<S>) -> IndexedMesh<S> {
        Self::cuboid(width, width, width, metadata)
    }

    /// **Mathematical Foundation: Spherical Mesh Generation with Indexed Connectivity**
    ///
    /// Construct a sphere using UV-parameterized tessellation with optimized vertex sharing.
    /// This implementation leverages indexed connectivity for significant memory savings
    /// and improved performance in connectivity-based operations.
    ///
    /// ## **Indexed Mesh Advantages**
    /// - **Memory Efficiency**: ~50% reduction in vertex storage vs. non-indexed
    /// - **Connectivity Performance**: O(1) vertex lookup for adjacency queries
    /// - **Topology Preservation**: Explicit vertex sharing maintains manifold structure
    /// - **Cache Optimization**: Better memory locality for vertex-based operations
    ///
    /// ## **Vertex Layout Strategy**
    /// ```text
    /// Grid: (segments+1) × (stacks+1) vertices
    /// Poles: North (0,r,0) and South (0,-r,0) shared by multiple triangles
    /// Equator: Maximum vertex sharing for optimal connectivity
    /// ```
    ///
    /// ## **Tessellation with Index Optimization**
    /// - **Pole Handling**: Single vertex per pole, shared by all adjacent triangles
    /// - **Regular Grid**: Structured indexing for predictable connectivity patterns
    /// - **Quad Decomposition**: Each grid cell becomes 2 triangles with shared edges
    pub fn sphere(
        radius: Real,
        segments: usize,
        stacks: usize,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        let mut vertices = Vec::new();
        let mut polygons = Vec::new();

        // Add north pole
        vertices.push(Vertex::new(
            Point3::new(0.0, radius, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ));

        // Generate vertices for intermediate stacks
        for j in 1..stacks {
            let v = j as Real / stacks as Real;
            let phi = v * PI;
            let y = radius * phi.cos();
            let ring_radius = radius * phi.sin();

            for i in 0..segments {
                let u = i as Real / segments as Real;
                let theta = u * TAU;
                let x = ring_radius * theta.cos();
                let z = ring_radius * theta.sin();

                let pos = Point3::new(x, y, z);
                let normal = pos.coords.normalize();
                vertices.push(Vertex::new(pos, normal));
            }
        }

        // Add south pole
        vertices.push(Vertex::new(
            Point3::new(0.0, -radius, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
        ));

        // Generate faces
        let north_pole = 0;
        let south_pole = vertices.len() - 1;

        // Top cap triangles (connecting to north pole)
        // Winding order: counter-clockwise when viewed from outside (above north pole)
        for i in 0..segments {
            let next_i = (i + 1) % segments;
            let v1 = 1 + i;
            let v2 = 1 + next_i;

            let plane =
                Plane::from_vertices(vec![vertices[north_pole], vertices[v2], vertices[v1]]);
            polygons.push(IndexedPolygon::new(
                vec![north_pole, v2, v1],
                plane,
                metadata.clone(),
            ));
        }

        // Middle section quads (split into triangles)
        for j in 1..stacks - 1 {
            let ring_start = 1 + (j - 1) * segments;
            let next_ring_start = 1 + j * segments;

            for i in 0..segments {
                let next_i = (i + 1) % segments;

                let v1 = ring_start + i;
                let v2 = ring_start + next_i;
                let v3 = next_ring_start + i;
                let v4 = next_ring_start + next_i;

                // First triangle of quad (counter-clockwise from outside)
                let plane1 =
                    Plane::from_vertices(vec![vertices[v1], vertices[v3], vertices[v2]]);
                polygons.push(IndexedPolygon::new(
                    vec![v1, v3, v2],
                    plane1,
                    metadata.clone(),
                ));

                // Second triangle of quad (counter-clockwise from outside)
                let plane2 =
                    Plane::from_vertices(vec![vertices[v2], vertices[v3], vertices[v4]]);
                polygons.push(IndexedPolygon::new(
                    vec![v2, v3, v4],
                    plane2,
                    metadata.clone(),
                ));
            }
        }

        // Bottom cap triangles (connecting to south pole)
        // Winding order: counter-clockwise when viewed from outside (below south pole)
        if stacks > 1 {
            let last_ring_start = 1 + (stacks - 2) * segments;
            for i in 0..segments {
                let next_i = (i + 1) % segments;
                let v1 = last_ring_start + i;
                let v2 = last_ring_start + next_i;

                let plane = Plane::from_vertices(vec![
                    vertices[v1],
                    vertices[v2],
                    vertices[south_pole],
                ]);
                polygons.push(IndexedPolygon::new(
                    vec![v1, v2, south_pole],
                    plane,
                    metadata.clone(),
                ));
            }
        }

        IndexedMesh {
            vertices,
            polygons,
            bounding_box: std::sync::OnceLock::new(),
            metadata,
        }
    }

    /// **Mathematical Foundation: Cylindrical Mesh Generation with Indexed Connectivity**
    ///
    /// Creates a cylinder using indexed mesh representation for optimal performance.
    /// Leverages vertex sharing between side faces and caps for memory efficiency.
    ///
    /// ## **Indexed Connectivity Benefits**
    /// - **Vertex Sharing**: Side vertices shared between adjacent faces and caps
    /// - **Memory Efficiency**: 2×(segments+1) vertices instead of 6×segments
    /// - **Topology Optimization**: Explicit connectivity for manifold operations
    pub fn cylinder(
        radius: Real,
        height: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        Self::frustum_indexed(radius, radius, height, segments, metadata)
    }

    /// Helper method for creating frustums with indexed connectivity
    pub fn frustum_indexed(
        radius1: Real,
        radius2: Real,
        height: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        let mut vertices = Vec::new();
        let mut polygons = Vec::new();

        // Center vertices for caps
        let bottom_center = vertices.len();
        vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0), -Vector3::z()));

        let top_center = vertices.len();
        vertices.push(Vertex::new(Point3::new(0.0, 0.0, height), Vector3::z()));

        // Ring vertices for bottom and top
        let bottom_ring_start = vertices.len();
        for i in 0..segments {
            let angle = (i as Real / segments as Real) * TAU;
            let x = angle.cos() * radius1;
            let y = angle.sin() * radius1;
            vertices.push(Vertex::new(Point3::new(x, y, 0.0), -Vector3::z()));
        }

        let top_ring_start = vertices.len();
        for i in 0..segments {
            let angle = (i as Real / segments as Real) * TAU;
            let x = angle.cos() * radius2;
            let y = angle.sin() * radius2;
            vertices.push(Vertex::new(Point3::new(x, y, height), Vector3::z()));
        }

        // Generate faces
        for i in 0..segments {
            let next_i = (i + 1) % segments;

            // Bottom cap triangle (counter-clockwise when viewed from below)
            if radius1 > EPSILON {
                let plane = Plane::from_normal(-Vector3::z(), 0.0);
                polygons.push(IndexedPolygon::new(
                    vec![
                        bottom_center,
                        bottom_ring_start + next_i,
                        bottom_ring_start + i,
                    ],
                    plane,
                    metadata.clone(),
                ));
            }

            // Top cap triangle (counter-clockwise when viewed from above)
            if radius2 > EPSILON {
                let plane = Plane::from_normal(Vector3::z(), height);
                polygons.push(IndexedPolygon::new(
                    vec![top_center, top_ring_start + i, top_ring_start + next_i],
                    plane,
                    metadata.clone(),
                ));
            }

            // Side faces (quads split into triangles)
            let b1 = bottom_ring_start + i;
            let b2 = bottom_ring_start + next_i;
            let t1 = top_ring_start + i;
            let t2 = top_ring_start + next_i;

            // Calculate side normal
            let side_normal = Vector3::new(
                (vertices[b1].pos.x + vertices[t1].pos.x) / 2.0,
                (vertices[b1].pos.y + vertices[t1].pos.y) / 2.0,
                0.0,
            )
            .normalize();

            let plane =
                Plane::from_normal(side_normal, side_normal.dot(&vertices[b1].pos.coords));

            // First triangle of quad (counter-clockwise from outside)
            polygons.push(IndexedPolygon::new(
                vec![b1, t1, b2],
                plane.clone(),
                metadata.clone(),
            ));

            // Second triangle of quad (counter-clockwise from outside)
            polygons.push(IndexedPolygon::new(vec![b2, t1, t2], plane, metadata.clone()));
        }

        let mut mesh = IndexedMesh {
            vertices,
            polygons,
            bounding_box: std::sync::OnceLock::new(),
            metadata,
        };

        mesh.compute_vertex_normals();
        mesh
    }

    /// Creates an IndexedMesh polyhedron from raw vertex data and face indices.
    /// This leverages the indexed representation directly for optimal performance.
    ///
    /// # Parameters
    /// - `points`: a slice of `[x,y,z]` coordinates.
    /// - `faces`: each element is a list of indices into `points`, describing one face.
    ///   Each face must have at least 3 indices.
    ///
    /// # Example
    /// ```
    /// # use csgrs::IndexedMesh::IndexedMesh;
    ///
    /// let pts = &[
    ///     [0.0, 0.0, 0.0], // point0
    ///     [1.0, 0.0, 0.0], // point1
    ///     [1.0, 1.0, 0.0], // point2
    ///     [0.0, 1.0, 0.0], // point3
    ///     [0.5, 0.5, 1.0], // point4 - top
    /// ];
    ///
    /// // Two faces: bottom square [0,1,2,3], and pyramid sides
    /// let fcs: &[&[usize]] = &[
    ///     &[0, 1, 2, 3],
    ///     &[0, 1, 4],
    ///     &[1, 2, 4],
    ///     &[2, 3, 4],
    ///     &[3, 0, 4],
    /// ];
    ///
    /// let mesh_poly = IndexedMesh::<()>::polyhedron(pts, fcs, None);
    /// ```
    pub fn polyhedron(
        points: &[[Real; 3]],
        faces: &[&[usize]],
        metadata: Option<S>,
    ) -> Result<IndexedMesh<S>, ValidationError> {
        // Convert points to vertices (normals will be computed later)
        let vertices: Vec<Vertex> = points
            .iter()
            .map(|&[x, y, z]| Vertex::new(Point3::new(x, y, z), Vector3::zeros()))
            .collect();

        let mut polygons = Vec::new();

        for face in faces {
            // Skip degenerate faces
            if face.len() < 3 {
                continue;
            }

            // Validate indices
            for &idx in face.iter() {
                if idx >= points.len() {
                    return Err(ValidationError::IndexOutOfRange);
                }
            }

            // Create indexed polygon
            let face_vertices: Vec<Vertex> = face.iter().map(|&idx| vertices[idx]).collect();

            let plane = Plane::from_vertices(face_vertices);
            let indexed_poly = IndexedPolygon::new(face.to_vec(), plane, metadata.clone());
            polygons.push(indexed_poly);
        }

        let mut mesh = IndexedMesh {
            vertices,
            polygons,
            bounding_box: std::sync::OnceLock::new(),
            metadata,
        };

        // Compute proper vertex normals
        mesh.compute_vertex_normals();

        Ok(mesh)
    }

    /// Regular octahedron scaled by `radius` using indexed connectivity
    pub fn octahedron(radius: Real, metadata: Option<S>) -> IndexedMesh<S> {
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
        Self::polyhedron(&scaled, &faces, metadata).unwrap()
    }

    /// Regular icosahedron scaled by `radius` using indexed connectivity
    pub fn icosahedron(radius: Real, metadata: Option<S>) -> IndexedMesh<S> {
        // radius scale factor
        let factor = radius * 0.5878; // empirically determined
        // golden ratio
        let phi: Real = (1.0 + 5.0_f64.sqrt() as Real) * 0.5;
        // normalise so the circum-radius is 1
        let inv_len = (1.0 + phi * phi).sqrt().recip();
        let a = inv_len;
        let b = phi * inv_len;

        // 12 vertices
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

        // 20 faces (counter-clockwise when viewed from outside)
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

        Self::polyhedron(&pts, &faces, metadata)
            .unwrap()
            .scale(factor, factor, factor)
    }

    /// Torus centered at the origin in the *XY* plane using indexed connectivity.
    /// This creates a torus by revolving a circle around the Y-axis.
    ///
    /// * `major_r` – distance from center to tube center (R)
    /// * `minor_r` – tube radius (r)
    /// * `segments_major` – number of segments around the donut
    /// * `segments_minor` – segments of the tube cross-section
    pub fn torus(
        major_r: Real,
        minor_r: Real,
        segments_major: usize,
        segments_minor: usize,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        let circle = Sketch::circle(minor_r, segments_minor.max(3), metadata.clone())
            .translate(major_r, 0.0, 0.0);
        let mesh = circle
            .revolve(360.0, segments_major.max(3))
            .expect("Revolve failed");

        // Convert regular mesh to IndexedMesh
        IndexedMesh::from_polygons(&mesh.polygons, mesh.metadata)
    }

    /// Creates an ellipsoid by taking a sphere of radius=1 and scaling it by (rx, ry, rz).
    /// Uses indexed connectivity for optimal performance.
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
    ) -> IndexedMesh<S> {
        let base_sphere = Self::sphere(1.0, segments, stacks, metadata.clone());
        base_sphere.scale(rx, ry, rz)
    }

    /// Creates an arrow IndexedMesh with indexed connectivity optimization.
    /// The arrow is composed of a cylindrical shaft and a cone-like head.
    ///
    /// # Parameters
    /// - `start`: the reference point (base or tip, depending on orientation)
    /// - `direction`: the vector defining arrow length and intended pointing direction
    /// - `segments`: number of segments for approximating the cylinder and frustum
    /// - `orientation`: when false (default) the arrow points away from start; when true the arrow points toward start
    /// - `metadata`: optional metadata for the generated polygons.
    pub fn arrow(
        start: Point3<Real>,
        direction: Vector3<Real>,
        segments: usize,
        orientation: bool,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        // Compute the arrow's total length.
        let arrow_length = direction.norm();
        if arrow_length < EPSILON {
            return IndexedMesh::new();
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
        let shaft =
            IndexedMesh::cylinder(shaft_radius, shaft_length, segments, metadata.clone());

        // Build the arrow head as a frustum from z = shaft_length to z = shaft_length + head_length.
        let head = IndexedMesh::frustum_indexed(
            head_base_radius,
            tip_radius,
            head_length,
            segments,
            metadata.clone(),
        )
        .translate(0.0, 0.0, shaft_length);

        // Combine the shaft and head.
        let mut canonical_arrow = shaft.union(&head);

        // If the arrow should point toward start, mirror the geometry in canonical space.
        if orientation {
            let l = arrow_length;
            let mirror_mat: Matrix4<Real> = Translation3::new(0.0, 0.0, l / 2.0)
                .to_homogeneous()
                * Matrix4::new_nonuniform_scaling(&Vector3::new(1.0, 1.0, -1.0))
                * Translation3::new(0.0, 0.0, -l / 2.0).to_homogeneous();
            canonical_arrow = canonical_arrow.transform(&mirror_mat).inverse();
        }

        // Compute the rotation that maps the canonical +Z axis to the provided direction.
        let z_axis = Vector3::z();
        let rotation = Rotation3::rotation_between(&z_axis, &unit_dir)
            .unwrap_or_else(Rotation3::identity);
        let rot_mat: Matrix4<Real> = rotation.to_homogeneous();

        // Rotate the arrow.
        let rotated_arrow = canonical_arrow.transform(&rot_mat);

        // Finally, translate the arrow so that the anchored vertex moves to 'start'.
        rotated_arrow.translate(start.x, start.y, start.z)
    }

    /// Creates a 3D "teardrop cylinder" by extruding a 2D teardrop profile.
    /// Uses indexed connectivity for optimal performance.
    ///
    /// # Parameters
    /// - `width`: Width of the 2D teardrop profile.
    /// - `length`: Length of the 2D teardrop profile.
    /// - `height`: Extrusion height.
    /// - `shape_segments`: Number of segments for the 2D teardrop outline.
    /// - `metadata`: Optional metadata.
    pub fn teardrop_cylinder(
        width: Real,
        length: Real,
        height: Real,
        shape_segments: usize,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        // Make a 2D teardrop in the XY plane.
        let td_2d = Sketch::teardrop(width, length, shape_segments, metadata.clone());
        let mesh = td_2d.extrude(height);

        // Convert regular mesh to IndexedMesh
        IndexedMesh::from_polygons(&mesh.polygons, mesh.metadata)
    }

    /// Creates spur gear with involute teeth using indexed connectivity.
    #[allow(clippy::too_many_arguments)]
    pub fn spur_gear_involute(
        module_: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        thickness: Real,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        let gear_2d = Sketch::involute_gear(
            module_,
            teeth,
            pressure_angle_deg,
            clearance,
            backlash,
            segments_per_flank,
            metadata.clone(),
        );
        let mesh = gear_2d.extrude(thickness);

        // Convert regular mesh to IndexedMesh
        IndexedMesh::from_polygons(&mesh.polygons, mesh.metadata)
    }

    /// Creates spur gear with cycloid teeth using indexed connectivity.
    pub fn spur_gear_cycloid(
        module_: Real,
        teeth: usize,
        pin_teeth: usize,
        clearance: Real,
        segments_per_flank: usize,
        thickness: Real,
        metadata: Option<S>,
    ) -> IndexedMesh<S> {
        let gear_2d = Sketch::cycloidal_gear(
            module_,
            teeth,
            pin_teeth,
            clearance,
            segments_per_flank,
            metadata.clone(),
        );
        let mesh = gear_2d.extrude(thickness);

        // Convert regular mesh to IndexedMesh
        IndexedMesh::from_polygons(&mesh.polygons, mesh.metadata)
    }
}
