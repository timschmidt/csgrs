//! Functions to extrude, revolve, loft, and otherwise transform 2D `Profile`s into 3D `Mesh`s

use crate::errors::ValidationError;
use crate::float_types::{
    Real, hangle_sin_cos, hdegrees_to_radians, hpoints_exactly_equal, hreal_div,
    hreal_f64s_exactly_equal, hreal_from_f64, hreal_gt_f64, hreal_lt_f64, hreal_mul,
    hreal_to_f64, hrotation_between_vectors, htranslation_matrix, hunit_cross_vector3,
    hunit_vector3, hvector3_from_point3, hvector3_from_vector3, hvectors_exactly_equal,
    hxy_ring_orientation_sign,
};
use crate::mesh::Mesh;
use crate::polygon::Polygon;
use crate::sketch::Profile;
use crate::vertex::Vertex;
use hypercurve::{
    Classification, FinitePolyline2, FiniteProjectionOptions, FiniteRegionProfile2,
    FiniteTriangle2, triangulate_finite_rings,
};
use hyperreal::RealSign;
use nalgebra::{Matrix4, Point3, Vector3};
use std::fmt::Debug;
use std::sync::OnceLock;

fn mesh_projection_options() -> FiniteProjectionOptions {
    FiniteProjectionOptions::try_new(1e-3)
        .expect("positive finite projection tolerance is valid")
}

/// Promote an extrusion scalar through hyperreal and export it only at the
/// current finite mesh boundary.
///
/// This keeps the public `Profile::extrude` API centered on
/// `hyperreal::Real`, while still accepting promotable primitive numbers at
/// caller edges. The boundary split follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
fn finite_extrude_scalar<S>(value: S) -> Option<Real>
where
    S: TryInto<hyperreal::Real>,
{
    hreal_to_f64(&value.try_into().ok()?)
}

fn finite_triangles_to_xy_points(triangles: Vec<FiniteTriangle2>) -> Vec<[Point3<Real>; 3]> {
    triangles
        .into_iter()
        .map(|tri| {
            [
                Point3::new(tri[0][0], tri[0][1], 0.0),
                Point3::new(tri[1][0], tri[1][1], 0.0),
                Point3::new(tri[2][0], tri[2][1], 0.0),
            ]
        })
        .collect()
}

fn hyper_direction_points_down(direction: &Vector3<Real>) -> bool {
    let Some(direction) = hvector3_from_vector3(direction) else {
        return false;
    };
    let Ok(z_axis) = hyperlattice::Vector3::try_from_f64_array([0.0, 0.0, 1.0]) else {
        return false;
    };
    hreal_lt_f64(&direction.dot(&z_axis), 0.0)
}

impl<M: Clone + Debug + Send + Sync> Profile<M> {
    fn projected_region_profiles_for_mesh(&self) -> Vec<FiniteRegionProfile2> {
        match self.project_region_profiles(&mesh_projection_options()) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
        }
    }

    fn projected_wire_polylines_for_mesh(&self) -> Vec<FinitePolyline2> {
        self.project_wire_polylines(&mesh_projection_options())
    }

    /// Linearly extrude this (2D) shape in the +Z direction by `height`.
    ///
    /// This is just a convenience wrapper around extrude_vector using Vector3::new(0.0, 0.0, height)
    pub fn extrude<H>(&self, height: H) -> Mesh<M>
    where
        H: TryInto<hyperreal::Real>,
    {
        let Some(height) = finite_extrude_scalar(height) else {
            return Mesh::empty(self.metadata.clone());
        };
        self.extrude_vector(Vector3::new(0.0, 0.0, height))
    }

    /// **Mathematical Foundation: Vector-Based Linear Extrusion**
    ///
    /// Linearly extrude any Profile along the given direction vector.
    /// This implements the complete mathematical theory of linear extrusion
    /// with proper surface generation and normal calculation.
    ///
    /// ## **Extrusion Mathematics**
    ///
    /// ### **Parametric Surface Definition**
    /// For a 2D boundary curve C(u) and direction vector d⃗:
    /// ```text
    /// M(u,v) = C(u) + v·d⃗
    /// where u ∈ [0,1] parameterizes the boundary
    ///       v ∈ [0,1] parameterizes the extrusion
    /// ```
    ///
    /// ### **Surface Normal Computation**
    /// For side surfaces, the normal is computed as:
    /// ```text
    /// n⃗ = checked_unit(∂M/∂u × ∂M/∂v)
    ///   = checked_unit(C'(u) × d⃗)
    /// ```
    /// where C'(u) is the tangent to the boundary curve. The checked unit
    /// vector is computed with `hyperlattice::Vector3<hyperreal::Real>`, so
    /// degenerate or non-finite side normals are rejected before finite mesh
    /// output. This follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    ///
    /// ### **Surface Classification**
    /// The extrusion generates three surface types:
    ///
    /// 1. **Bottom Caps** (v=0):
    ///    - Triangulated 2D regions at z=0
    ///    - Normal: n⃗ = -checked_unit(d⃗) (inward for solid)
    ///
    /// 2. **Top Caps** (v=1):
    ///    - Translated triangulated regions
    ///    - Normal: n⃗ = +checked_unit(d⃗) (outward for solid)
    ///
    /// 3. **Side Surfaces**:
    ///    - Quadrilateral strips connecting boundary edges
    ///    - Normal: n⃗ = checked_unit(edge × direction)
    ///
    /// ### **Boundary Orientation Rules**
    /// - **Exterior boundaries**: Counter-clockwise → outward-facing sides
    /// - **Interior boundaries (holes)**: Clockwise → inward-facing sides
    /// - **Winding preservation**: Maintains topological correctness
    ///
    /// ### **Geometric Properties**
    /// - **Volume**: V = Area(base) × |d⃗|
    /// - **Surface Area**: A = 2×Area(base) + Perimeter(base)×|d⃗|
    /// - **Centroid**: c⃗ = centroid(base) + 0.5×d⃗
    ///
    /// ## **Numerical Considerations**
    /// - **Degenerate Direction**: exact zero or non-finite directions return no geometry
    /// - **Normal Calculation**: Cross products normalized for unit normals
    /// - **Manifold Preservation**: Ensures watertight mesh topology
    ///
    /// ## **Algorithm Complexity**
    /// - **Triangulation**: O(n log n) for n boundary vertices
    /// - **Surface Generation**: O(n) for n boundary edges
    /// - **Total Complexity**: O(n log n) dominated by tessellation
    ///
    /// Builds top, bottom, and side polygons in 3D, storing them in the polygon list.
    /// Returns a new Mesh containing these extruded polygons.
    ///
    /// # Parameters
    /// - `direction`: 3D vector defining extrusion direction and magnitude
    ///
    /// Direction degeneracy is tested by promoting the boundary `Vector3<f64>`
    /// into `hyperlattice::Vector3` and comparing squared length exactly in
    /// `hyperreal::Real`. This keeps the decision to emit no solid for an
    /// exact-zero extrusion on the same exact predicate path used by mesh
    /// topology, while preserving nonzero infinitesimal-scale boundary values;
    /// see Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn extrude_vector(&self, direction: Vector3<Real>) -> Mesh<M> {
        if hvector3_from_vector3(&direction).is_none()
            || hvectors_exactly_equal(&direction, &Vector3::zeros())
        {
            return Mesh::empty(self.metadata.clone());
        }

        if !self.region.material_contours().is_empty()
            || !self.region.hole_contours().is_empty()
        {
            return self.extrude_region_vector(direction);
        }

        if !self.wires().is_empty() {
            return self.extrude_wires_vector(direction);
        }

        // Finite projection data is not Profile's CAD source of truth. Linear
        // extrusion therefore emits nothing when native
        // `Region2`/`CurveString2` topology is absent, matching the exact-object
        // boundary advocated by Yap, "Towards Exact Geometric Computation,"
        // *Computational Geometry* 7(1-2), 1997
        // (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        Mesh::empty(self.metadata.clone())
    }

    /// Extrude native hypercurve topology without routing through a separate 2D
    /// backend type.
    ///
    /// Filled `Region2` contours receive caps and side walls; open
    /// `CurveString2` wires are independent ruled side surfaces. Hypercurve
    /// contours are approximated only at the mesh boundary, while cap
    /// triangulation is delegated to `hypercurve::triangulate_finite_rings`,
    /// which promotes finite boundary coordinates back to hyperreal predicates
    /// before ear clipping. This keeps the data model aligned with Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), and Meisters,
    /// "Polygons Have Ears," *American Mathematical Monthly* 82(6), 1975
    /// (<https://doi.org/10.2307/2319703>).
    fn extrude_region_vector(&self, direction: Vector3<Real>) -> Mesh<M> {
        let dir_unit = hunit_vector3(&direction).unwrap_or_else(Vector3::z);
        let flip = hyper_direction_points_down(&direction);
        let bottom_normal = -dir_unit;
        let top_normal = dir_unit;
        let mut polygons = Vec::new();

        for profile in self.projected_region_profiles_for_mesh() {
            let exterior = profile.material().points().to_vec();
            let holes = profile
                .holes()
                .iter()
                .map(|hole| hole.points().to_vec())
                .collect::<Vec<_>>();
            let exterior = close_region_ring(exterior);
            let holes = holes.into_iter().map(close_region_ring).collect::<Vec<_>>();
            let hole_refs = holes.iter().map(Vec::as_slice).collect::<Vec<_>>();
            let triangles = finite_triangles_to_xy_points(
                triangulate_finite_rings(&exterior, &hole_refs).unwrap_or_default(),
            );

            for tri in &triangles {
                let (a, b, c) = if flip {
                    (tri[0], tri[1], tri[2])
                } else {
                    (tri[2], tri[1], tri[0])
                };
                polygons.push(Polygon::new(
                    vec![
                        Self::apply_origin_transform_vertex(
                            Vertex::new(a, bottom_normal),
                            self.origin_transform,
                        ),
                        Self::apply_origin_transform_vertex(
                            Vertex::new(b, bottom_normal),
                            self.origin_transform,
                        ),
                        Self::apply_origin_transform_vertex(
                            Vertex::new(c, bottom_normal),
                            self.origin_transform,
                        ),
                    ],
                    self.metadata.clone(),
                ));
            }

            for tri in &triangles {
                let p0 = tri[0] + direction;
                let p1 = tri[1] + direction;
                let p2 = tri[2] + direction;
                let (a, b, c) = if flip { (p2, p1, p0) } else { (p0, p1, p2) };
                polygons.push(Polygon::new(
                    vec![
                        Self::apply_origin_transform_vertex(
                            Vertex::new(a, top_normal),
                            self.origin_transform,
                        ),
                        Self::apply_origin_transform_vertex(
                            Vertex::new(b, top_normal),
                            self.origin_transform,
                        ),
                        Self::apply_origin_transform_vertex(
                            Vertex::new(c, top_normal),
                            self.origin_transform,
                        ),
                    ],
                    self.metadata.clone(),
                ));
            }

            self.push_region_ring_sides(&exterior, direction, flip, &mut polygons);
            for hole in &holes {
                self.push_region_ring_sides(hole, direction, flip, &mut polygons);
            }
        }

        for wire in self.projected_wire_polylines_for_mesh() {
            self.push_polyline_sides(wire.points(), direction, flip, &mut polygons);
        }

        Mesh::from_polygons(&polygons, self.metadata.clone())
    }

    /// Extrude native open hypercurve wires into ruled side surfaces.
    ///
    /// The source topology remains in `CurveString2`; only the final mesh
    /// boundary samples are finite points. This is the ruled-surface analogue
    /// of area extrusion and keeps CAD ownership on the exact object side of
    /// Yap's exact-geometric-computation boundary: Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    fn extrude_wires_vector(&self, direction: Vector3<Real>) -> Mesh<M> {
        let flip = hyper_direction_points_down(&direction);
        let mut polygons = Vec::new();
        for wire in self.projected_wire_polylines_for_mesh() {
            self.push_polyline_sides(wire.points(), direction, flip, &mut polygons);
        }
        Mesh::from_polygons(&polygons, self.metadata.clone())
    }

    fn push_region_ring_sides(
        &self,
        ring: &[[Real; 2]],
        direction: Vector3<Real>,
        flip: bool,
        polygons: &mut Vec<Polygon<M>>,
    ) {
        // Closed region rings and open wires share side-strip emission after
        // crossing the hypercurve-to-mesh sampling boundary.
        self.push_polyline_sides(ring, direction, flip, polygons);
    }

    fn push_polyline_sides(
        &self,
        polyline: &[[Real; 2]],
        direction: Vector3<Real>,
        flip: bool,
        polygons: &mut Vec<Polygon<M>>,
    ) {
        for window in polyline.windows(2) {
            let b_i = Point3::new(window[0][0], window[0][1], 0.0);
            let b_j = Point3::new(window[1][0], window[1][1], 0.0);
            let t_i = b_i + direction;
            let t_j = b_j + direction;
            let edge = b_j - b_i;
            let Some(raw_normal) = hunit_cross_vector3(&edge, &direction) else {
                continue;
            };
            let normal = if flip { -raw_normal } else { raw_normal };
            let vertices = if flip {
                [b_j, b_i, t_i, t_j]
            } else {
                [b_i, b_j, t_j, t_i]
            }
            .into_iter()
            .map(|point| {
                Self::apply_origin_transform_vertex(
                    Vertex::new(point, normal),
                    self.origin_transform,
                )
            })
            .collect::<Vec<_>>();
            polygons.push(Polygon::new(vertices, self.metadata.clone()));
        }
    }

    /// Extrudes (or "lofts") a closed 3D volume between two polygons in space.
    /// - `bottom` and `top` each have the same number of vertices `n`, in matching order.
    /// - Returns a new Mesh whose faces are:
    ///   - The `bottom` polygon,
    ///   - The `top` polygon,
    ///   - `n` rectangular side polygons bridging each edge of `bottom` to the corresponding edge of `top`.
    pub fn loft(
        bottom: &Polygon<M>,
        top: &Polygon<M>,
        flip_bottom_polygon: bool,
    ) -> Result<Mesh<M>, ValidationError> {
        let n = bottom.vertices.len();
        if n != top.vertices.len() {
            return Err(ValidationError::MismatchedVertexCount {
                left: n,
                right: top.vertices.len(),
            });
        }

        // Conditionally flip the bottom polygon if requested.
        let bottom_poly = if flip_bottom_polygon {
            let mut flipped = bottom.clone();
            flipped.flip();
            flipped
        } else {
            bottom.clone()
        };

        // Gather polygons: bottom + top
        // (Depending on the orientation, you might want to flip one of them.)

        let mut polygons = vec![bottom_poly.clone(), top.clone()];

        // For each edge (i -> i+1) in bottom, connect to the corresponding edge in top.
        for i in 0..n {
            let j = (i + 1) % n;
            let b_i = &bottom.vertices[i];
            let b_j = &bottom.vertices[j];
            let t_i = &top.vertices[i];
            let t_j = &top.vertices[j];

            // Build the side face as a 4-vertex polygon (quad).
            // Winding order here is chosen so that the polygon's normal faces outward
            // (depending on the orientation of bottom vs. top).
            let side_poly = Polygon::new(
                vec![
                    *b_i, // bottom[i]
                    *b_j, // bottom[i+1]
                    *t_j, // top[i+1]
                    *t_i, // top[i]
                ],
                bottom.metadata.clone(), // carry over bottom polygon metadata
            );
            polygons.push(side_poly);
        }

        Ok(Mesh::from_polygons(&polygons, bottom.metadata.clone()))
    }

    // Perform a linear extrusion along some axis, with optional twist, center, slices, scale, etc.
    //
    // # Parameters
    // - `direction`: Direction vector for the extrusion.
    // - `twist`: Total twist in degrees around the extrusion axis from bottom to top.
    // - `segments`: Number of intermediate subdivisions.
    // - `scale`: A uniform scale factor to apply at the top slice (bottom is scale=1.0).
    //
    // # Assumptions
    // - This CSG is assumed to represent one or more 2D polygons lying in or near the XY plane.
    // - The resulting shape is extruded *initially* along +Z, then finally rotated if `v != [0,0,1]`.
    //
    // # Returns
    // A new 3D CSG.
    //
    // # Example
    // ```
    // let shape_2d = CSG::square(2.0, None); // a 2D square in XY
    // let extruded = shape_2d.linear_extrude(
    //     direction = Vector3::new(0.0, 0.0, 10.0),
    //     twist = 360.0,
    //     segments = 32,
    //     scale = 1.2,
    // );
    // ```
    // pub fn linear_extrude(
    // shape: &CCShape<Real>,
    // direction: Vector3<Real>,
    // twist_degs: Real,
    // segments: usize,
    // scale_top: Real,
    // metadata: M,
    // ) -> CSG<M> {
    // let mut polygons_3d = Vec::new();
    // if segments < 1 {
    // return CSG::new();
    // }
    // let height = hyperlattice_magnitude(direction);
    // if height is exact zero {
    // no extrusion volume
    // return CSG::new();
    // }
    //
    // Step 1) Build a series of “transforms” from bottom=0..top=height, subdivided into `segments`.
    //   For each i in [0..=segments], compute fraction f and:
    //   - scale in XY => s_i
    //   - twist about Z => rot_i
    //   - translate in Z => z_i
    //
    //   We'll store each “slice” in 3D form as a Vec<Vec<Point3<Real>>>,
    //   i.e. one 3D polyline for each boundary or hole in the shape.
    // let mut slices: Vec<Vec<Vec<Point3<Real>>>> = Vec::with_capacity(segments + 1);
    // The axis to rotate around is the unit of `direction`. We'll do final alignment after constructing them along +Z.
    // let axis_dir = hyperlattice_checked_unit(direction);
    //
    // for i in 0..=segments {
    // let f = i as Real / segments as Real;
    // let s_i = 1.0 + (scale_top - 1.0) * f;  // lerp(1, scale_top, f)
    // let twist_rad = twist_degs.to_radians() * f;
    // let z_i = height * f;
    //
    // Build transform T = Tz * Rz * Sxy
    //  - scale in XY
    //  - twist around Z
    //  - translate in Z
    // let mat_scale = hscale_matrix(s_i, s_i, 1.0)?;
    // let mat_rot = finite_rotation_z(twist_rad)?;
    // let mat_trans = htranslation_matrix(&Vector3::new(0.0, 0.0, z_i))?;
    // let slice_mat = mat_trans * mat_rot * mat_scale;
    //
    // let slice_3d = project_shape_3d(shape, &slice_mat);
    // slices.push(slice_3d);
    // }
    //
    // Step 2) “Stitch” consecutive slices to form side polygons.
    // For each pair of slices[i], slices[i+1], for each boundary polyline j,
    // connect edges. We assume each polyline has the same vertex_count in both slices.
    // (If the shape is closed, we do wrap edges [n..0].)
    // Then we optionally build bottom & top caps if the polylines are closed.
    //
    // a) bottom + top caps, similar to extrude_vector approach
    //    For slices[0], build a “bottom” by triangulating in XY, flipping normal.
    //    For slices[segments], build a “top” by normal up.
    //
    //    But we only do it if each boundary is closed.
    //    We must group CCW with matching holes. This is the same logic as `extrude_vector`.
    //
    // We'll do a small helper that triangulates shape in 2D, then lifts that triangulation to slice_3d.
    // You can re‐use the logic from `extrude_vector`.
    //
    // Build the “bottom” from slices[0] if polylines are all or partially closed
    // polygons_3d.extend(
    // build_caps_from_slice(shape, &slices[0], true, metadata.clone())
    // );
    // Build the “top” from slices[segments]
    // polygons_3d.extend(
    // build_caps_from_slice(shape, &slices[segments], false, metadata.clone())
    // );
    //
    // b) side walls
    // for i in 0..segments {
    // let bottom_slice = &slices[i];
    // let top_slice = &slices[i + 1];
    //
    // We know bottom_slice has shape.ccw_plines.len() + shape.cw_plines.len() polylines
    // in the same order. Each polyline has the same vertex_count as in top_slice.
    // So we can do a direct 1:1 match: bottom_slice[j] <-> top_slice[j].
    // for (pline_idx, bot3d) in bottom_slice.iter().enumerate() {
    // let top3d = &top_slice[pline_idx];
    // if bot3d.len() < 2 {
    // continue;
    // }
    // is it closed? We can check shape’s corresponding polyline
    // let is_closed = if pline_idx < shape.ccw_plines.len() {
    // shape.ccw_plines[pline_idx].polyline.is_closed()
    // } else {
    // shape.cw_plines[pline_idx - shape.ccw_plines.len()].polyline.is_closed()
    // };
    // let n = bot3d.len();
    // let edge_count = if is_closed { n } else { n - 1 };
    //
    // for k in 0..edge_count {
    // let k_next = (k + 1) % n;
    // let b_i = bot3d[k];
    // let b_j = bot3d[k_next];
    // let t_i = top3d[k];
    // let t_j = top3d[k_next];
    //
    // let poly_side = Polygon::new(
    // vec![
    // Vertex::new(b_i, Vector3::zeros()),
    // Vertex::new(b_j, Vector3::zeros()),
    // Vertex::new(t_j, Vector3::zeros()),
    // Vertex::new(t_i, Vector3::zeros()),
    // ],
    // metadata.clone(),
    // );
    // polygons_3d.push(poly_side);
    // }
    // }
    // }
    //
    // Step 3) If direction is not along +Z, rotate final mesh so +Z aligns with your direction
    // (This is optional or can be done up front. Typical OpenSCAD style is to do everything
    // along +Z, then rotate the final.)
    // if axis_dir is not exactly Vector3::z() {
    // rotate from +Z to axis_dir
    // let rot_axis = hvector3_cross(&Vector3::z(), &axis_dir)?;
    // let sin_theta = hyperlattice_magnitude(rot_axis);
    // if sin_theta is not exact zero {
    // let cos_theta = hvector3_dot(&Vector3::z(), &axis_dir)?;
    // let angle = cos_theta.acos();
    // let mat = hrotation_between_vectors(&Vector3::z(), &axis_dir)?;
    // transform the polygons
    // let mut final_polys = Vec::with_capacity(polygons_3d.len());
    // for mut poly in polygons_3d {
    // for v in &mut poly.vertices {
    // let pos4 = mat * nalgebra::Vector4::new(v.position.x, v.position.y, v.position.z, 1.0);
    // v.position = Point3::new(pos4.x / pos4.w, pos4.y / pos4.w, pos4.z / pos4.w);
    // }
    // poly.set_new_normal();
    // final_polys.push(poly);
    // }
    // return CSG::from_polygons(&final_polys);
    // }
    // }
    //
    // otherwise, just return as is
    // CSG::from_polygons(&polygons_3d)
    // }

    /// **Mathematical Foundation: Surface of Revolution Generation**
    ///
    /// Revolve 2D Profile around the Y-axis to create surfaces of revolution.
    /// This implements the complete mathematical theory of revolution surfaces with
    /// proper orientation handling and cap generation.
    ///
    /// ## **Revolution Mathematics**
    ///
    /// ### **Parametric Surface Generation**
    /// For each 2D boundary point (x,y), generate revolution surface:
    /// ```text
    /// M(θ) = (x·cos(θ), y, x·sin(θ))
    /// where θ ∈ [0, angle_radians]
    /// ```
    ///
    /// ### **Surface Mesh Construction**
    /// The algorithm creates quadrilateral strips:
    /// 1. **Vertex Grid**: (n_segments+1) × (n_boundary_points) vertices
    /// 2. **Quad Formation**: Connect adjacent vertices in parameter space
    /// 3. **Orientation**: Preserve winding from 2D profile
    ///
    /// ### **Normal Vector Calculation**
    /// For each quad, compute normals using right-hand rule:
    /// ```text
    /// n⃗ = (v⃗₁ - v⃗₀) × (v⃗₂ - v⃗₀)
    /// ```
    /// Direction depends on profile curve orientation.
    ///
    /// ### **Boundary Orientation Handling**
    /// - **Exterior boundaries (CCW)**: Generate outward-facing surfaces
    /// - **Interior boundaries (CW)**: Generate inward-facing surfaces (holes)
    /// - **Winding preservation**: Essential for manifold topology
    ///
    /// ### **Partial Revolution Caps**
    /// For angle < 360°, generate planar caps:
    /// 1. **Start cap** (θ=0): Triangulated profile at initial position
    /// 2. **End cap** (θ=angle): Triangulated profile at final position
    /// 3. **Cap normals**: Point outward from solid interior
    /// 4. **Manifold closure**: Ensures watertight geometry
    ///
    /// ### **Multi-Polygon Support**
    /// - **Exterior polygons**: Create main solid boundaries
    /// - **Interior polygons**: Create holes and cavities
    /// - **Nesting rules**: Interior must be properly contained
    ///
    /// ## **Algorithm Complexity**
    /// - **Boundary Processing**: O(n) for n boundary edges
    /// - **Surface Generation**: O(n×s) for s segments
    /// - **Cap Triangulation**: O(n log n) for complex profiles
    ///
    /// ## **Geometric Properties**
    /// - **Surface continuity**: C⁰ (positional) at segment boundaries
    /// - **Normal continuity**: Discontinuous at segment boundaries (faceted)
    /// - **Manifold property**: Maintained for valid input profiles
    ///
    /// ## **Applications**
    /// - **Turned objects**: Lathe-created components
    /// - **Vessels**: Bowls, vases, containers
    /// - **Mechanical parts**: Pulleys, gears, shafts
    /// - **Architectural elements**: Columns, balusters
    ///
    /// ## **Numerical Considerations**
    /// - **Trigonometric precomputation**: Improves performance
    /// - **Degeneracy handling**: Skips zero-length edges
    /// - **Precision**: Maintains accuracy for small angles
    ///
    /// Native area profiles are consumed from hypercurve region rings before
    /// finite mesh generation. Native open `CurveString2` wires are revolved as
    /// independent profile curves without end caps. This keeps profile topology
    /// in hyper geometry and treats mesh vertices as boundary output, following
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    /// Degree conversion, angular step division, and sine/cosine evaluation
    /// are promoted through `hyperreal::Real` before finite vertices are
    /// emitted. The Y-axis sweep is the same trigonometric rotation formula
    /// underlying Rodrigues' theorem; see Rodrigues, "Des lois géométriques
    /// qui régissent les déplacements d'un système solide dans l'espace,"
    /// *Journal de Mathématiques Pures et Appliquées* 5, 1840.
    ///
    /// # Parameters
    /// - `angle_degs`: Revolution angle in degrees (0-360)
    /// - `segments`: Number of angular subdivisions (≥ 2)
    ///
    /// Returns Mesh with revolution surfaces only
    pub fn revolve(
        &self,
        angle_degs: Real,
        segments: usize,
    ) -> Result<Mesh<M>, ValidationError> {
        if segments < 2 {
            return Err(ValidationError::FieldLessThan {
                name: "segments",
                min: 2,
            });
        }
        let Some(angle_radians) = hdegrees_to_radians(angle_degs) else {
            return Err(ValidationError::InvalidArguments);
        };

        let mut new_polygons = Vec::new();

        // A small helper to revolve a point (x,y) in the XY plane around the Y-axis by theta.
        // The output is a 3D point (X, Y, Z).
        fn revolve_around_y(x: Real, y: Real, theta: Real) -> Option<Point3<Real>> {
            let (sin_t, cos_t) = hangle_sin_cos(theta)?;
            // Map (x, y, 0) => ( x*cos θ, y, x*sin θ )
            Some(Point3::new(hreal_mul(x, cos_t)?, y, hreal_mul(x, sin_t)?))
        }

        fn is_ccw(ring: &[[Real; 2]]) -> bool {
            matches!(hxy_ring_orientation_sign(ring), Some(RealSign::Positive))
        }

        // A helper to extrude one ring of coordinates (including the last->first if needed),
        // pushing its side polygons into `out_polygons`.
        // - `ring_coords`: The ring’s sequence of points. Usually closed (last=first).
        // - `ring_is_ccw`: true if it's an exterior ring, false if interior/hole.
        // - `angle_radians`: total revolve sweep in radians.
        // - `segments`: how many discrete slices around the revolve.
        // - `metadata`: user metadata to attach to side polygons.
        fn revolve_ring<M: Clone + Send + Sync>(
            ring_coords: &[[Real; 2]],
            ring_is_ccw: bool,
            angle_radians: Real,
            segments: usize,
            metadata: &M,
        ) -> Vec<Polygon<M>> {
            if ring_coords.len() < 2 {
                return vec![];
            }

            let mut out_polygons = Vec::new();
            // Typically the last point = first point for a closed ring.
            // We'll iterate over each edge i..i+1, and revolve them around by segments slices.

            // The revolve step size in radians:
            let Some(step) = hreal_div(angle_radians, segments as Real) else {
                return vec![];
            };

            // For each edge in the ring:
            for i in 0..(ring_coords.len() - 1) {
                let c_i = ring_coords[i];
                let c_j = ring_coords[i + 1];

                // If these two points are the same, skip degenerate edge
                if same_xy((c_i[0], c_i[1]), (c_j[0], c_j[1])) {
                    continue;
                }

                // For each revolve slice j..j+1
                for s in 0..segments {
                    let Some(th0) = hreal_mul(s as Real, step) else {
                        continue;
                    };
                    let Some(th1) = hreal_mul(s as Real + 1.0, step) else {
                        continue;
                    };

                    // revolve bottom edge endpoints at angle th0
                    let Some(b_i) = revolve_around_y(c_i[0], c_i[1], th0) else {
                        continue;
                    };
                    let Some(b_j) = revolve_around_y(c_j[0], c_j[1], th0) else {
                        continue;
                    };
                    // revolve top edge endpoints at angle th1
                    let Some(t_i) = revolve_around_y(c_i[0], c_i[1], th1) else {
                        continue;
                    };
                    let Some(t_j) = revolve_around_y(c_j[0], c_j[1], th1) else {
                        continue;
                    };

                    // Build a 4-vertex side polygon for the ring edge.
                    // The orientation depends on ring_is_ccw:
                    //    If CCW => outward walls -> [b_i, b_j, t_j, t_i]
                    //    If CW  => reverse it -> [b_j, b_i, t_i, t_j]
                    let quad_verts = if ring_is_ccw {
                        vec![b_i, b_j, t_j, t_i]
                    } else {
                        vec![b_j, b_i, t_i, t_j]
                    }
                    .into_iter()
                    .map(|position| Vertex::new(position, Vector3::zeros()))
                    .collect();

                    out_polygons.push(Polygon::new(quad_verts, metadata.clone()));
                }
            }
            out_polygons
        }

        // Build a single “cap” polygon from ring_coords at a given angle (0 or angle_radians).
        //  - revolve each 2D point by `angle`, produce a 3D ring
        //  - if `flip` is true, reverse the ring so the normal is inverted
        fn build_cap_polygon<M: Clone + Send + Sync>(
            ring_coords: &[[Real; 2]],
            angle: Real,
            flip: bool,
            metadata: &M,
        ) -> Option<Polygon<M>> {
            if ring_coords.len() < 3 {
                return None;
            }
            // revolve each coordinate at the given angle
            let mut pts_3d: Vec<_> = ring_coords
                .iter()
                .filter_map(|c| revolve_around_y(c[0], c[1], angle))
                .collect();
            if pts_3d.len() < 3 {
                return None;
            }

            // Ensure closed if the projected hypercurve ring was not emitted
            // with an explicit duplicate endpoint.
            let last = pts_3d.last().unwrap();
            let first = pts_3d.first().unwrap();
            if !hpoints_exactly_equal(last, first) {
                pts_3d.push(*first);
            }

            // Turn into Vertex
            let mut verts: Vec<_> = pts_3d
                .into_iter()
                .map(|p3| Vertex::new(p3, Vector3::zeros()))
                .collect();

            // If flip == true, reverse them and flip each vertex
            if flip {
                verts.reverse();
                for v in &mut verts {
                    v.flip();
                }
            }

            // Build the polygon
            let poly = Polygon::new(verts, metadata.clone());
            Some(poly)
        }

        //----------------------------------------------------------------------
        // 2) Iterate over each hypercurve finite profile, revolve side walls,
        //    and possibly add caps when angle_degs < 360.
        //----------------------------------------------------------------------
        let full_revolve = hreal_f64s_exactly_equal(angle_degs, 360.0);
        let angle_positive = hreal_from_f64(angle_degs)
            .ok()
            .is_some_and(|angle| hreal_gt_f64(&angle, 0.0));
        let do_caps = !full_revolve && angle_positive;

        let mut emit_profile = |outer: &[[Real; 2]], holes: &[Vec<[Real; 2]>]| {
            let ext_ccw = is_ccw(outer);

            new_polygons.extend(revolve_ring(
                outer,
                ext_ccw,
                angle_radians,
                segments,
                &self.metadata,
            ));

            if do_caps {
                if let Some(cap) = build_cap_polygon(outer, 0.0, ext_ccw, &self.metadata) {
                    new_polygons.push(cap);
                }
                if let Some(cap) =
                    build_cap_polygon(outer, angle_radians, !ext_ccw, &self.metadata)
                {
                    new_polygons.push(cap);
                }
            }

            for hole in holes {
                let hole_ccw = is_ccw(hole);
                new_polygons.extend(revolve_ring(
                    hole,
                    hole_ccw,
                    angle_radians,
                    segments,
                    &self.metadata,
                ));
            }
        };

        let region_profiles = self.projected_region_profiles_for_mesh();
        if !region_profiles.is_empty() {
            for profile in region_profiles {
                let holes = profile
                    .holes()
                    .iter()
                    .map(|hole| hole.points().to_vec())
                    .collect::<Vec<_>>();
                emit_profile(profile.material().points(), &holes);
            }
        }

        for wire in self.projected_wire_polylines_for_mesh() {
            new_polygons.extend(revolve_ring(
                wire.points(),
                true,
                angle_radians,
                segments,
                &self.metadata,
            ));
        }

        //----------------------------------------------------------------------
        // 3) Return the new CSG:
        //----------------------------------------------------------------------
        Ok(Mesh {
            polygons: new_polygons,
            bounding_box: OnceLock::new(),
            query_trimesh: OnceLock::new(),
            metadata: self.metadata.clone(),
        })
    }

    /// Sweep (a.k.a. “extrude along path”) –
    /// duplicates the 2-D sketch at every vertex of `path`,
    /// aims the sketch’s +Z at the local path tangent,
    /// stitches side walls, and caps open ends.
    ///
    /// Closed area profiles are read from Profile's native hypercurve
    /// [`Region2`](hypercurve::Region2) boundary rings and capped when the path
    /// is open. Native [`CurveString2`](hypercurve::CurveString2) wires are
    /// swept as independent open profile curves without caps. The path frames
    /// and mesh vertices are finite output, while the swept profile topology
    /// stays in hyper geometry until the mesh boundary. This follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>). The
    /// frame propagation is the standard rotation-minimizing/parallel-transport
    /// sweep approach; see Bishop, "There is more than one way to frame a
    /// curve," *American Mathematical Monthly* 82(3), 1975
    /// (<https://doi.org/10.2307/2319846>).
    ///
    /// * `path` - ordered list of 3-D points. If the first and last points
    ///   coincide under the exact hyperreal squared-distance predicate, the path is
    ///   treated as **closed** and no caps are added.
    ///
    /// * returns - a `Mesh<M>` containing all side quads plus automatically triangulated caps (respecting any holes).
    pub fn sweep(&self, path: &[Point3<Real>]) -> Mesh<M> {
        // sanity checks
        if path.len() < 2 {
            return Mesh::empty(self.metadata.clone());
        }
        if path.iter().any(|p| hvector3_from_point3(p).is_none()) {
            return Mesh::empty(self.metadata.clone());
        }
        let n_path = path.len();
        let path_is_closed = hpoints_exactly_equal(&path[0], &path[n_path - 1]);

        // pre-compute a transform for each path vertex
        let mut slice_xforms: Vec<Matrix4<Real>> = Vec::with_capacity(n_path);

        // first slice
        let mut dir_prev = hunit_vector3(&(path[1] - path[0])).unwrap_or_else(Vector3::z);
        let mut orientation = hrotation_between_vectors(&Vector3::z(), &dir_prev)
            .unwrap_or_else(Matrix4::identity);
        let Some(first_translation) = htranslation_matrix(&path[0].coords) else {
            return Mesh::empty(self.metadata.clone());
        };
        slice_xforms.push(first_translation * orientation);

        // propagate frame with parallel transport
        for i in 1..n_path {
            // pick the outgoing tangent _now_
            let dir_curr = if i == n_path - 1 && !path_is_closed {
                hunit_vector3(&(path[i] - path[i - 1])).unwrap_or(dir_prev) // look back at the end
            } else {
                hunit_vector3(&(path[(i + 1) % n_path] - path[i])).unwrap_or(dir_prev)
            };

            // Rotate the frame exactly once. The rotation matrix is assembled
            // from hyperlattice-checked unit vectors, dot, and cross products.
            let rot_between = hrotation_between_vectors(&dir_prev, &dir_curr)
                .unwrap_or_else(Matrix4::identity);
            orientation = rot_between * orientation;

            // now the slice that lives at path[i]
            let Some(translation) = htranslation_matrix(&path[i].coords) else {
                return Mesh::empty(self.metadata.clone());
            };
            slice_xforms.push(translation * orientation);

            // ...and _immediately_ remember this tangent for the next turn
            dir_prev = dir_curr;
        }

        // helper: map a 2-D point (x,y,0) through a slice transform
        #[inline]
        fn map_pt(p2: [Real; 2], m: &Matrix4<Real>) -> Point3<Real> {
            Point3::from_homogeneous(*m * Point3::new(p2[0], p2[1], 0.0).to_homogeneous())
                .expect("homogeneous w != 0")
        }

        // collect every closed region ring and open wire profile of the sketch
        #[derive(Debug)]
        struct Ring {
            coords_2d: Vec<[Real; 2]>,      // original XY coords
            slices: Vec<Vec<Point3<Real>>>, // one Vec<Point3> per path vertex
        }
        let mut rings: Vec<Ring> = Vec::new();

        let mut add_ring = |coords: Vec<[Real; 2]>| {
            if coords.len() < 2 {
                return;
            }
            let mut slices: Vec<Vec<Point3<Real>>> = Vec::with_capacity(n_path);
            for xf in &slice_xforms {
                let slice: Vec<Point3<Real>> = coords.iter().map(|&p| map_pt(p, xf)).collect();
                slices.push(slice);
            }
            rings.push(Ring {
                coords_2d: coords,
                slices,
            });
        };

        let mut cap_profiles: Vec<(Vec<[Real; 2]>, Vec<Vec<[Real; 2]>>)> = Vec::new();
        let region_profiles = self.projected_region_profiles_for_mesh();
        if !region_profiles.is_empty() {
            for profile in region_profiles {
                let material = profile.material().points().to_vec();
                add_ring(material.clone());
                let mut holes = Vec::new();
                for hole in profile.holes() {
                    let hole = hole.points().to_vec();
                    add_ring(hole.clone());
                    holes.push(hole);
                }
                cap_profiles.push((material, holes));
            }
        }

        for wire in self.projected_wire_polylines_for_mesh() {
            add_ring(wire.into_points());
        }

        if rings.is_empty() {
            return Mesh::empty(self.metadata.clone());
        }

        // build polygons
        let mut out_polys: Vec<Polygon<M>> = Vec::new();

        // side walls, ring-by-ring
        let end_idx = if path_is_closed { n_path } else { n_path - 1 };

        for ring in &rings {
            let v_per_ring = ring.coords_2d.len() - 1;
            for i in 0..end_idx {
                let j = (i + 1) % n_path;
                let slice_i = &ring.slices[i];
                let slice_j = &ring.slices[j];

                for k in 0..v_per_ring {
                    let v0 = slice_i[k];
                    let v1 = slice_i[k + 1];
                    let v2 = slice_j[k + 1];
                    let v3 = slice_j[k];

                    // triangle 1  (v0-v1-v2)
                    out_polys.push(Polygon::new(
                        vec![
                            Vertex::new(v0, Vector3::zeros()),
                            Vertex::new(v1, Vector3::zeros()),
                            Vertex::new(v2, Vector3::zeros()),
                        ],
                        self.metadata.clone(),
                    ));
                    // triangle 2  (v0-v2-v3)
                    out_polys.push(Polygon::new(
                        vec![
                            Vertex::new(v0, Vector3::zeros()),
                            Vertex::new(v2, Vector3::zeros()),
                            Vertex::new(v3, Vector3::zeros()),
                        ],
                        self.metadata.clone(),
                    ));
                }
            }
        }

        // caps for open paths
        if !path_is_closed {
            // Triangulate every 2-D polygon (outer + holes) once,
            // then reuse the triangles for both ends.

            // helper so we don’t repeat the capping code twice
            let mut add_caps = |ext: &[[Real; 2]], holes: &[Vec<[Real; 2]>]| {
                let hole_refs: Vec<&[[Real; 2]]> = holes.iter().map(|v| &v[..]).collect();

                let tris = finite_triangles_to_xy_points(
                    triangulate_finite_rings(ext, &hole_refs).unwrap_or_default(),
                );

                // cap at the start of the path (flip winding)
                for t in &tris {
                    let p0 = map_pt([t[0].x, t[0].y], &slice_xforms[0]);
                    let p1 = map_pt([t[1].x, t[1].y], &slice_xforms[0]);
                    let p2 = map_pt([t[2].x, t[2].y], &slice_xforms[0]);
                    out_polys.push(Polygon::new(
                        vec![
                            Vertex::new(p2, Vector3::zeros()),
                            Vertex::new(p1, Vector3::zeros()),
                            Vertex::new(p0, Vector3::zeros()),
                        ],
                        self.metadata.clone(),
                    ));
                }

                // cap at the end of the path
                for t in &tris {
                    let p0 = map_pt([t[0].x, t[0].y], &slice_xforms[n_path - 1]);
                    let p1 = map_pt([t[1].x, t[1].y], &slice_xforms[n_path - 1]);
                    let p2 = map_pt([t[2].x, t[2].y], &slice_xforms[n_path - 1]);
                    out_polys.push(Polygon::new(
                        vec![
                            Vertex::new(p0, Vector3::zeros()),
                            Vertex::new(p1, Vector3::zeros()),
                            Vertex::new(p2, Vector3::zeros()),
                        ],
                        self.metadata.clone(),
                    ));
                }
            };

            for (outer, holes) in &cap_profiles {
                add_caps(outer, holes);
            }
        }

        Mesh::from_polygons(&out_polys, self.metadata.clone())
    }
}

/// Helper to build a single Polygon from a “slice” of 3D points.
///
/// If `flip_winding` is true, we reverse the vertex order (so the polygon’s normal flips).
fn _polygon_from_slice<M: Clone + Send + Sync>(
    slice_pts: &[Point3<Real>],
    flip_winding: bool,
    metadata: M,
) -> Polygon<M> {
    if slice_pts.len() < 3 {
        // degenerate polygon
        return Polygon::new(vec![], metadata);
    }
    // Build the vertex list
    let mut verts: Vec<Vertex> = slice_pts
        .iter()
        .map(|p| Vertex::new(*p, Vector3::zeros()))
        .collect();

    if flip_winding {
        verts.reverse();
        for v in &mut verts {
            v.flip();
        }
    }

    Polygon::new(verts, metadata)
}

fn close_region_ring(mut points: Vec<[Real; 2]>) -> Vec<[Real; 2]> {
    if points.len() >= 2
        && points
            .first()
            .zip(points.last())
            .is_some_and(|(first, last)| !same_xy((first[0], first[1]), (last[0], last[1])))
    {
        let first = points[0];
        points.push(first);
    }
    points
}

fn same_xy(a: (Real, Real), b: (Real, Real)) -> bool {
    hreal_f64s_exactly_equal(a.0, b.0) && hreal_f64s_exactly_equal(a.1, b.1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_xy_uses_exact_hyperreal_identity() {
        assert!(same_xy((0.0, 0.0), (0.0, -0.0)));
        assert!(!same_xy((0.0, 0.0), (1.0e-12, 0.0)));
        assert!(!same_xy((0.0, 0.0), (Real::NAN, 0.0)));
    }

    #[test]
    fn ring_orientation_sign_skips_leading_collinear_points() {
        let ccw = [
            [0.0, 0.0],
            [0.25, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
            [0.0, 1.0],
            [0.0, 0.0],
        ];
        let cw = [
            [0.0, 0.0],
            [0.0, 0.25],
            [0.0, 1.0],
            [1.0, 1.0],
            [1.0, 0.0],
            [0.0, 0.0],
        ];
        let collinear = [[0.0, 0.0], [0.25, 0.0], [1.0, 0.0]];

        assert_eq!(hxy_ring_orientation_sign(&ccw), Some(RealSign::Positive));
        assert_eq!(hxy_ring_orientation_sign(&cw), Some(RealSign::Negative));
        assert_eq!(hxy_ring_orientation_sign(&collinear), None);
    }

    #[test]
    fn close_region_ring_rejects_nonfinite_duplicate_endpoint() {
        let points = close_region_ring(vec![[0.0, 0.0], [1.0, 0.0], [Real::NAN, 0.0]]);
        assert_eq!(points.len(), 4);
        assert_eq!(points[3], [0.0, 0.0]);
    }
}
