//! Functions to extrude, revolve, loft, and otherwise transform 2D `Profile`s into 3D `Mesh`s

use crate::errors::ValidationError;
use crate::hyper_math::{
    hreal_lt_f64, hreal_mul, hreal_sign, hrotation_between_vectors, htranslation_matrix,
    hunit_cross_vector3, hunit_vector3, hvector3_from_point3, hvector3_from_vector3,
};
use crate::mesh::Mesh;
use crate::polygon::Polygon;
use crate::sketch::Profile;
use crate::vertex::Vertex;
use hypercurve::{
    Classification, FinitePolyline2, FiniteProjectionOptions, FiniteRegionProfile2,
    FiniteTriangle2, triangulate_finite_rings,
};
use hyperlattice::{Matrix4, Point3, Real, Vector3};
use hyperreal::RealSign;
use std::fmt::Debug;

type FiniteRing = Vec<[f64; 2]>;
type FiniteRegionRings = (FiniteRing, Vec<FiniteRing>);

fn mesh_projection_options() -> FiniteProjectionOptions {
    FiniteProjectionOptions::try_new(1e-3)
        .expect("positive finite projection tolerance is valid")
}

fn finite_real(value: f64) -> Option<Real> {
    Real::try_from(value).ok()
}

fn finite_xy_point3(x: f64, y: f64) -> Option<Point3> {
    Some(Point3::new(finite_real(x)?, finite_real(y)?, Real::zero()))
}

fn materialize_finite_point3(point: Point3) -> Option<Point3> {
    let materialize = |value: &Real| {
        value
            .to_f64_lossy()
            .filter(|value| value.is_finite())
            .and_then(finite_real)
    };
    Some(Point3::new(
        materialize(&point.x)?,
        materialize(&point.y)?,
        materialize(&point.z)?,
    ))
}

fn finite_triangles_to_xy_points(triangles: Vec<FiniteTriangle2>) -> Vec<[Point3; 3]> {
    triangles
        .into_iter()
        .filter_map(|tri| {
            let [[ax, ay], [bx, by], [cx, cy]] = tri;
            Some([
                finite_xy_point3(ax, ay)?,
                finite_xy_point3(bx, by)?,
                finite_xy_point3(cx, cy)?,
            ])
        })
        .collect()
}

fn exact_points_equal(lhs: &Point3, rhs: &Point3) -> bool {
    let lhs = hyperlimit::Point3::new(lhs.x.clone(), lhs.y.clone(), lhs.z.clone());
    let rhs = hyperlimit::Point3::new(rhs.x.clone(), rhs.y.clone(), rhs.z.clone());
    matches!(hyperlimit::point3_equal(&lhs, &rhs).value(), Some(true))
}

fn push_clean_face<M: Clone + Send + Sync>(
    mut points: Vec<Point3>,
    reverse: bool,
    metadata: &M,
    output: &mut Vec<Polygon<M>>,
) {
    points.dedup_by(|right, left| exact_points_equal(left, right));
    if points.len() > 1 && exact_points_equal(&points[0], points.last().unwrap()) {
        points.pop();
    }
    if points.len() < 3 {
        return;
    }
    if reverse {
        points.reverse();
    }
    output.push(Polygon::new(
        points
            .into_iter()
            .map(|point| Vertex::new(point, Vector3::zeros()))
            .collect(),
        metadata.clone(),
    ));
}

fn hyper_direction_points_down(direction: &Vector3) -> bool {
    let Some(direction) = hvector3_from_vector3(direction) else {
        return false;
    };
    let z_axis = Vector3::z();
    hreal_lt_f64(&direction.dot(&z_axis), 0.0)
}

impl Profile {
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
    /// This is just a convenience wrapper around `extrude_vector` with a z-axis direction.
    pub fn extrude<M: Clone + Debug + Send + Sync>(
        &self,
        height: Real,
        metadata: M,
    ) -> Mesh<M> {
        self.extrude_vector(
            Vector3::from_xyz(Real::zero(), Real::zero(), height),
            metadata,
        )
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
    /// vector is computed with `hyperlattice::Vector3`, so
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
    /// `Real`. This keeps the decision to emit no solid for an
    /// exact-zero extrusion on the same exact predicate path used by mesh
    /// topology, while preserving nonzero infinitesimal-scale boundary values;
    /// see Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn extrude_vector<M: Clone + Debug + Send + Sync>(
        &self,
        direction: Vector3,
        metadata: M,
    ) -> Mesh<M> {
        let direction_point = hyperlimit::Point3::new(
            direction.0[0].clone(),
            direction.0[1].clone(),
            direction.0[2].clone(),
        );
        let zero = Vector3::zeros();
        let zero_point =
            hyperlimit::Point3::new(zero.0[0].clone(), zero.0[1].clone(), zero.0[2].clone());
        if hvector3_from_vector3(&direction).is_none()
            || matches!(
                hyperlimit::point3_equal(&direction_point, &zero_point).value(),
                Some(true)
            )
        {
            return Mesh::empty();
        }

        if !self.region.material_contours().is_empty()
            || !self.region.hole_contours().is_empty()
        {
            return self.extrude_region_vector(direction, &metadata);
        }

        if !self.wires().is_empty() {
            return self.extrude_wires_vector(direction, &metadata);
        }

        // Finite projection data is not Profile's CAD source of truth. Linear
        // extrusion therefore emits nothing when native
        // `Region2`/`CurveString2` topology is absent, matching the exact-object
        // boundary advocated by Yap, "Towards Exact Geometric Computation,"
        // *Computational Geometry* 7(1-2), 1997
        // (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        Mesh::empty()
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
    fn extrude_region_vector<M: Clone + Debug + Send + Sync>(
        &self,
        direction: Vector3,
        metadata: &M,
    ) -> Mesh<M> {
        let dir_unit = hunit_vector3(&direction).unwrap_or_else(Vector3::z);
        let flip = hyper_direction_points_down(&direction);
        let bottom_normal = -dir_unit.clone();
        let top_normal = dir_unit;
        let mut polygons = Vec::new();

        for profile in self.projected_region_profiles_for_mesh() {
            let exterior = profile.material().points().to_vec();
            let holes = profile
                .holes()
                .iter()
                .map(|hole| hole.points().to_vec())
                .collect::<Vec<_>>();
            let Some(exterior) = orient_region_ring(exterior, true) else {
                continue;
            };
            let Some(holes) = holes
                .into_iter()
                .map(|hole| orient_region_ring(hole, false))
                .collect::<Option<Vec<_>>>()
            else {
                continue;
            };
            let hole_refs = holes.iter().map(Vec::as_slice).collect::<Vec<_>>();
            let triangles = finite_triangles_to_xy_points(
                triangulate_finite_rings(&exterior, &hole_refs).unwrap_or_default(),
            );

            for tri in &triangles {
                let (a, b, c) = if flip {
                    (tri[0].clone(), tri[1].clone(), tri[2].clone())
                } else {
                    (tri[2].clone(), tri[1].clone(), tri[0].clone())
                };
                polygons.push(Polygon::new(
                    vec![
                        Self::apply_origin_transform_vertex(
                            Vertex::new(a, bottom_normal.clone()),
                            self.origin_transform.clone(),
                        ),
                        Self::apply_origin_transform_vertex(
                            Vertex::new(b, bottom_normal.clone()),
                            self.origin_transform.clone(),
                        ),
                        Self::apply_origin_transform_vertex(
                            Vertex::new(c, bottom_normal.clone()),
                            self.origin_transform.clone(),
                        ),
                    ],
                    metadata.clone(),
                ));
            }

            for tri in &triangles {
                let p0 = tri[0].clone() + &direction;
                let p1 = tri[1].clone() + &direction;
                let p2 = tri[2].clone() + &direction;
                let (a, b, c) = if flip { (p2, p1, p0) } else { (p0, p1, p2) };
                polygons.push(Polygon::new(
                    vec![
                        Self::apply_origin_transform_vertex(
                            Vertex::new(a, top_normal.clone()),
                            self.origin_transform.clone(),
                        ),
                        Self::apply_origin_transform_vertex(
                            Vertex::new(b, top_normal.clone()),
                            self.origin_transform.clone(),
                        ),
                        Self::apply_origin_transform_vertex(
                            Vertex::new(c, top_normal.clone()),
                            self.origin_transform.clone(),
                        ),
                    ],
                    metadata.clone(),
                ));
            }

            self.push_region_ring_sides(
                &exterior,
                direction.clone(),
                flip,
                metadata,
                &mut polygons,
            );
            for hole in &holes {
                self.push_region_ring_sides(
                    hole,
                    direction.clone(),
                    flip,
                    metadata,
                    &mut polygons,
                );
            }
        }

        for wire in self.projected_wire_polylines_for_mesh() {
            self.push_polyline_sides(
                wire.points(),
                direction.clone(),
                flip,
                metadata,
                &mut polygons,
            );
        }

        Mesh::from_polygons(&polygons)
    }

    /// Extrude native open hypercurve wires into ruled side surfaces.
    ///
    /// The source topology remains in `CurveString2`; only the final mesh
    /// boundary samples are finite points. This is the ruled-surface analogue
    /// of area extrusion and keeps CAD ownership on the exact object side of
    /// Yap's exact-geometric-computation boundary: Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    fn extrude_wires_vector<M: Clone + Debug + Send + Sync>(
        &self,
        direction: Vector3,
        metadata: &M,
    ) -> Mesh<M> {
        let flip = hyper_direction_points_down(&direction);
        let mut polygons = Vec::new();
        for wire in self.projected_wire_polylines_for_mesh() {
            self.push_polyline_sides(
                wire.points(),
                direction.clone(),
                flip,
                metadata,
                &mut polygons,
            );
        }
        Mesh::from_polygons(&polygons)
    }

    fn push_region_ring_sides<M: Clone + Send + Sync>(
        &self,
        ring: &[[f64; 2]],
        direction: Vector3,
        flip: bool,
        metadata: &M,
        polygons: &mut Vec<Polygon<M>>,
    ) {
        // Closed region rings and open wires share side-strip emission after
        // crossing the hypercurve-to-mesh sampling boundary.
        self.push_polyline_sides(ring, direction, flip, metadata, polygons);
    }

    fn push_polyline_sides<M: Clone + Send + Sync>(
        &self,
        polyline: &[[f64; 2]],
        direction: Vector3,
        flip: bool,
        metadata: &M,
        polygons: &mut Vec<Polygon<M>>,
    ) {
        for window in polyline.windows(2) {
            let Some(b_i) = finite_xy_point3(window[0][0], window[0][1]) else {
                continue;
            };
            let Some(b_j) = finite_xy_point3(window[1][0], window[1][1]) else {
                continue;
            };
            let t_i = b_i.clone() + &direction;
            let t_j = b_j.clone() + &direction;
            let edge = &b_j - &b_i;
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
                    Vertex::new(point, normal.clone()),
                    self.origin_transform.clone(),
                )
            })
            .collect::<Vec<_>>();
            polygons.push(Polygon::new(vertices, metadata.clone()));
        }
    }

    /// Loft a closed mesh through two or more corresponding polygon sections.
    ///
    /// Every section must have at least three vertices and the same vertex
    /// count in matching cyclic order. Cap and side winding are derived from
    /// the first section's exact plane orientation relative to the aggregate
    /// travel direction. Side patches are triangulated because corresponding
    /// edges in arbitrary sections do not generally form planar quads.
    pub fn loft<M: Clone + Debug + Send + Sync>(
        sections: &[Polygon<M>],
    ) -> Result<Mesh<M>, ValidationError> {
        if sections.len() < 2 {
            return Err(ValidationError::FieldLessThan {
                name: "sections",
                min: 2,
            });
        }
        let vertex_count = sections[0].vertices.len();
        if vertex_count < 3 {
            return Err(ValidationError::InvalidArguments);
        }
        for section in &sections[1..] {
            if section.vertices.len() != vertex_count {
                return Err(ValidationError::MismatchedVertexCount {
                    left: vertex_count,
                    right: section.vertices.len(),
                });
            }
        }

        let sum_positions = |section: &Polygon<M>| {
            section.vertices.iter().fold(Vector3::zeros(), |sum, vertex| {
                sum + vertex.position.to_vector()
            })
        };
        let travel = sum_positions(sections.last().unwrap()) - sum_positions(&sections[0]);
        let Some(orientation) = hreal_sign(&sections[0].plane.normal().dot(&travel)) else {
            return Err(ValidationError::InvalidArguments);
        };
        if orientation == RealSign::Zero {
            return Err(ValidationError::InvalidArguments);
        }
        let forward_winding = orientation == RealSign::Positive;

        let mut bottom = sections[0].clone();
        if forward_winding {
            bottom.flip();
        }
        let mut top = sections.last().unwrap().clone();
        if !forward_winding {
            top.flip();
        }
        let mut polygons = vec![bottom, top];

        for pair in sections.windows(2) {
            for index in 0..vertex_count {
                let next = (index + 1) % vertex_count;
                let mut first = vec![
                    pair[0].vertices[index].clone(),
                    pair[0].vertices[next].clone(),
                    pair[1].vertices[next].clone(),
                ];
                let mut second = vec![
                    pair[0].vertices[index].clone(),
                    pair[1].vertices[next].clone(),
                    pair[1].vertices[index].clone(),
                ];
                if !forward_winding {
                    first.reverse();
                    second.reverse();
                }
                polygons.push(Polygon::new(first, pair[0].metadata.clone()));
                polygons.push(Polygon::new(second, pair[0].metadata.clone()));
            }
        }

        Ok(Mesh::from_polygons(&polygons))
    }

    /// Extrude along +Z while linearly varying rotation and XY scale.
    ///
    /// Slice heights are exact rational multiples of `height`. Rotation and
    /// scale are sampled once per slice at the finite mesh boundary, then
    /// promoted to exact dyadic coordinates. Filled regions retain grouped
    /// holes and receive one cap at each end; wires produce side strips only.
    pub fn extrude_twisted<M: Clone + Debug + Send + Sync>(
        &self,
        height: Real,
        twist_degrees: Real,
        end_scale: [Real; 2],
        slices: usize,
        metadata: M,
    ) -> Result<Mesh<M>, ValidationError> {
        if slices < 1 {
            return Err(ValidationError::FieldLessThan {
                name: "slices",
                min: 1,
            });
        }
        let Some(height_sign) = hreal_sign(&height) else {
            return Err(ValidationError::InvalidArguments);
        };
        if height_sign == RealSign::Zero {
            return Err(ValidationError::InvalidArguments);
        }
        let Some(twist) = twist_degrees.to_f64_lossy().filter(|value| value.is_finite())
        else {
            return Err(ValidationError::InvalidArguments);
        };
        let Some(scale_x) = end_scale[0]
            .to_f64_lossy()
            .filter(|value| value.is_finite() && *value >= 0.0)
        else {
            return Err(ValidationError::InvalidArguments);
        };
        let Some(scale_y) = end_scale[1]
            .to_f64_lossy()
            .filter(|value| value.is_finite() && *value >= 0.0)
        else {
            return Err(ValidationError::InvalidArguments);
        };

        #[derive(Clone, Copy)]
        struct Slice {
            sin: f64,
            cos: f64,
            scale_x: f64,
            scale_y: f64,
        }

        fn map_point(point: [f64; 2], slice: Slice, z: &Real) -> Option<Point3> {
            let x = point[0] * slice.scale_x;
            let y = point[1] * slice.scale_y;
            Some(Point3::new(
                finite_real(x.mul_add(slice.cos, -y * slice.sin))?,
                finite_real(x.mul_add(slice.sin, y * slice.cos))?,
                z.clone(),
            ))
        }

        fn emit_ring<M: Clone + Send + Sync>(
            ring: &[[f64; 2]],
            slice_parameters: &[Slice],
            heights: &[Real],
            height_positive: bool,
            metadata: &M,
            polygons: &mut Vec<Polygon<M>>,
        ) {
            for edge in ring.windows(2) {
                for slice in 0..slice_parameters.len() - 1 {
                    let points = [
                        map_point(edge[0], slice_parameters[slice], &heights[slice]),
                        map_point(edge[1], slice_parameters[slice], &heights[slice]),
                        map_point(edge[1], slice_parameters[slice + 1], &heights[slice + 1]),
                        map_point(edge[0], slice_parameters[slice + 1], &heights[slice + 1]),
                    ]
                    .into_iter()
                    .collect::<Option<Vec<_>>>();
                    if let Some(points) = points {
                        push_clean_face(
                            vec![points[0].clone(), points[1].clone(), points[2].clone()],
                            !height_positive,
                            metadata,
                            polygons,
                        );
                        push_clean_face(
                            vec![points[0].clone(), points[2].clone(), points[3].clone()],
                            !height_positive,
                            metadata,
                            polygons,
                        );
                    }
                }
            }
        }

        let mut slice_parameters = Vec::with_capacity(slices + 1);
        let mut heights = Vec::with_capacity(slices + 1);
        for index in 0..=slices {
            let fraction = index as f64 / slices as f64;
            let radians = (twist * fraction).to_radians();
            let (sin, cos) = radians.sin_cos();
            slice_parameters.push(Slice {
                sin,
                cos,
                scale_x: (scale_x - 1.0).mul_add(fraction, 1.0),
                scale_y: (scale_y - 1.0).mul_add(fraction, 1.0),
            });
            let fraction = Real::from(index as u64) / Real::from(slices as u64);
            let Some(z) = fraction
                .ok()
                .and_then(|fraction| hreal_mul(&height, fraction))
            else {
                return Err(ValidationError::InvalidArguments);
            };
            heights.push(z);
        }

        let height_positive = height_sign == RealSign::Positive;
        let mut polygons = Vec::new();

        for profile in self.projected_region_profiles_for_mesh() {
            let Some(material) =
                orient_region_ring(profile.material().points().to_vec(), true)
            else {
                continue;
            };
            let Some(holes) = profile
                .holes()
                .iter()
                .map(|hole| orient_region_ring(hole.points().to_vec(), false))
                .collect::<Option<Vec<_>>>()
            else {
                continue;
            };
            emit_ring(
                &material,
                &slice_parameters,
                &heights,
                height_positive,
                &metadata,
                &mut polygons,
            );
            for hole in &holes {
                emit_ring(
                    hole,
                    &slice_parameters,
                    &heights,
                    height_positive,
                    &metadata,
                    &mut polygons,
                );
            }

            let hole_refs = holes.iter().map(Vec::as_slice).collect::<Vec<_>>();
            let triangles =
                triangulate_finite_rings(&material, &hole_refs).unwrap_or_default();
            for triangle in triangles {
                let bottom = triangle
                    .iter()
                    .filter_map(|point| map_point(*point, slice_parameters[0], &heights[0]))
                    .collect::<Vec<_>>();
                push_clean_face(bottom, height_positive, &metadata, &mut polygons);
                let top = triangle
                    .iter()
                    .filter_map(|point| {
                        map_point(*point, slice_parameters[slices], &heights[slices])
                    })
                    .collect::<Vec<_>>();
                push_clean_face(top, !height_positive, &metadata, &mut polygons);
            }
        }

        for wire in self.projected_wire_polylines_for_mesh() {
            emit_ring(
                wire.points(),
                &slice_parameters,
                &heights,
                height_positive,
                &metadata,
                &mut polygons,
            );
        }

        Ok(Mesh::from_polygons(&polygons))
    }

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
    /// Angular samples are evaluated once as finite mesh-boundary values and
    /// promoted to exact dyadic vertex coordinates. This avoids retaining
    /// symbolic trigonometric expressions in hypermesh topology. The Y-axis
    /// sweep is the same trigonometric rotation formula
    /// underlying Rodrigues' theorem; see Rodrigues, "Des lois géométriques
    /// qui régissent les déplacements d'un système solide dans l'espace,"
    /// *Journal de Mathématiques Pures et Appliquées* 5, 1840.
    ///
    /// # Parameters
    /// - `angle_degs`: Nonzero revolution angle in degrees in `[-360, 360]`
    /// - `segments`: Number of angular subdivisions (≥ 2)
    ///
    /// Filled profiles produce closed meshes; open wires produce surfaces.
    pub fn revolve<M: Clone + Debug + Send + Sync>(
        &self,
        angle_degs: Real,
        segments: usize,
        metadata: M,
    ) -> Result<Mesh<M>, ValidationError> {
        if segments < 2 {
            return Err(ValidationError::FieldLessThan {
                name: "segments",
                min: 2,
            });
        }
        let Some(angle_degs) = angle_degs.to_f64_lossy().filter(|angle| angle.is_finite())
        else {
            return Err(ValidationError::InvalidArguments);
        };
        if angle_degs == 0.0 || angle_degs.abs() > 360.0 {
            return Err(ValidationError::InvalidArguments);
        }

        fn map_point(point: [f64; 2], sin_cos: (f64, f64)) -> Option<Point3> {
            let (sin, cos) = sin_cos;
            Some(Point3::new(
                finite_real(point[0] * cos)?,
                finite_real(point[1])?,
                finite_real(point[0] * sin)?,
            ))
        }

        fn emit_ring<M: Clone + Send + Sync>(
            ring: &[[f64; 2]],
            sweep_positive: bool,
            full_revolve: bool,
            segments: usize,
            samples: &[(f64, f64)],
            metadata: &M,
            polygons: &mut Vec<Polygon<M>>,
        ) {
            for edge in ring.windows(2) {
                for slice in 0..segments {
                    let next_slice = if full_revolve {
                        (slice + 1) % samples.len()
                    } else {
                        slice + 1
                    };
                    let Some(a) = map_point(edge[0], samples[slice]) else {
                        continue;
                    };
                    let Some(b) = map_point(edge[1], samples[slice]) else {
                        continue;
                    };
                    let Some(c) = map_point(edge[1], samples[next_slice]) else {
                        continue;
                    };
                    let Some(d) = map_point(edge[0], samples[next_slice]) else {
                        continue;
                    };
                    push_clean_face(vec![a, b, c, d], !sweep_positive, metadata, polygons);
                }
            }
        }

        fn radial_orientation(ring: &[[f64; 2]]) -> Option<bool> {
            let has_negative = ring.iter().any(|point| point[0] < 0.0);
            let has_positive = ring.iter().any(|point| point[0] > 0.0);
            match (has_negative, has_positive) {
                (false, false) | (true, true) => None,
                (false, true) => Some(true),
                (true, false) => Some(false),
            }
        }

        let full_revolve = angle_degs.abs() == 360.0;
        let angle_positive = angle_degs > 0.0;
        let slice_count = if full_revolve { segments } else { segments + 1 };
        let samples = (0..slice_count)
            .map(|slice| {
                let radians = angle_degs.to_radians() * slice as f64 / segments as f64;
                radians.sin_cos()
            })
            .collect::<Vec<_>>();
        let mut polygons = Vec::new();

        for profile in self.projected_region_profiles_for_mesh() {
            let Some(material) =
                orient_region_ring(profile.material().points().to_vec(), true)
            else {
                continue;
            };
            let Some(holes) = profile
                .holes()
                .iter()
                .map(|hole| orient_region_ring(hole.points().to_vec(), false))
                .collect::<Option<Vec<_>>>()
            else {
                continue;
            };
            let Some(radial_positive) = radial_orientation(&material) else {
                return Err(ValidationError::InvalidArguments);
            };
            if holes
                .iter()
                .any(|hole| radial_orientation(hole) != Some(radial_positive))
            {
                return Err(ValidationError::InvalidArguments);
            }

            emit_ring(
                &material,
                angle_positive == radial_positive,
                full_revolve,
                segments,
                &samples,
                &metadata,
                &mut polygons,
            );
            for hole in &holes {
                emit_ring(
                    hole,
                    angle_positive == radial_positive,
                    full_revolve,
                    segments,
                    &samples,
                    &metadata,
                    &mut polygons,
                );
            }

            if !full_revolve {
                let sweep_positive = angle_positive == radial_positive;
                let hole_refs = holes.iter().map(Vec::as_slice).collect::<Vec<_>>();
                let triangles =
                    triangulate_finite_rings(&material, &hole_refs).unwrap_or_default();
                for triangle in triangles {
                    let start = triangle
                        .iter()
                        .filter_map(|point| map_point(*point, samples[0]))
                        .collect::<Vec<_>>();
                    push_clean_face(start, sweep_positive, &metadata, &mut polygons);
                    let end = triangle
                        .iter()
                        .filter_map(|point| map_point(*point, samples[slice_count - 1]))
                        .collect::<Vec<_>>();
                    push_clean_face(end, !sweep_positive, &metadata, &mut polygons);
                }
            }
        }

        for wire in self.projected_wire_polylines_for_mesh() {
            let Some(radial_positive) = radial_orientation(wire.points()) else {
                return Err(ValidationError::InvalidArguments);
            };
            emit_ring(
                wire.points(),
                angle_positive == radial_positive,
                full_revolve,
                segments,
                &samples,
                &metadata,
                &mut polygons,
            );
        }

        Ok(Mesh::from_polygons(&polygons))
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
    pub fn sweep<M: Clone + Debug + Send + Sync>(
        &self,
        path: &[Point3],
        metadata: M,
    ) -> Mesh<M> {
        if path.len() < 2 {
            return Mesh::empty();
        }
        if path.iter().any(|p| hvector3_from_point3(p).is_none()) {
            return Mesh::empty();
        }

        let mut canonical_path = Vec::with_capacity(path.len());
        for point in path {
            let is_duplicate = canonical_path.last().is_some_and(|previous: &Point3| {
                let previous = hyperlimit::Point3::new(
                    previous.x.clone(),
                    previous.y.clone(),
                    previous.z.clone(),
                );
                let point =
                    hyperlimit::Point3::new(point.x.clone(), point.y.clone(), point.z.clone());
                matches!(
                    hyperlimit::point3_equal(&previous, &point).value(),
                    Some(true)
                )
            });
            if !is_duplicate {
                canonical_path.push(point.clone());
            }
        }
        if canonical_path.len() < 2 {
            return Mesh::empty();
        }

        let n_path = canonical_path.len();
        let path_start = hyperlimit::Point3::new(
            canonical_path[0].x.clone(),
            canonical_path[0].y.clone(),
            canonical_path[0].z.clone(),
        );
        let path_end = hyperlimit::Point3::new(
            canonical_path[n_path - 1].x.clone(),
            canonical_path[n_path - 1].y.clone(),
            canonical_path[n_path - 1].z.clone(),
        );
        let path_is_closed = matches!(
            hyperlimit::point3_equal(&path_start, &path_end).value(),
            Some(true)
        );
        if path_is_closed {
            canonical_path.pop();
        }
        if canonical_path.len() < if path_is_closed { 3 } else { 2 } {
            return Mesh::empty();
        }
        let path = canonical_path;
        let n_path = path.len();

        let segment_count = if path_is_closed { n_path } else { n_path - 1 };
        let mut segment_directions = Vec::with_capacity(segment_count);
        for index in 0..segment_count {
            let Some(direction) = hunit_vector3(&(&path[(index + 1) % n_path] - &path[index]))
            else {
                return Mesh::empty();
            };
            segment_directions.push(direction);
        }
        let mut tangents = Vec::with_capacity(n_path);
        for index in 0..n_path {
            let tangent = if !path_is_closed && index == 0 {
                segment_directions[0].clone()
            } else if !path_is_closed && index == n_path - 1 {
                segment_directions[segment_count - 1].clone()
            } else {
                let incoming =
                    &segment_directions[(index + segment_count - 1) % segment_count];
                let outgoing = &segment_directions[index % segment_count];
                let Some(bisector) = hunit_vector3(&(incoming + outgoing)) else {
                    return Mesh::empty();
                };
                bisector
            };
            tangents.push(tangent);
        }

        // pre-compute a transform for each path vertex
        let mut slice_xforms: Vec<Matrix4> = Vec::with_capacity(n_path);

        // first slice
        let Some(mut orientation) = hrotation_between_vectors(&Vector3::z(), &tangents[0])
        else {
            return Mesh::empty();
        };
        let Some(first_translation) = htranslation_matrix(&path[0].to_vector()) else {
            return Mesh::empty();
        };
        slice_xforms.push(first_translation * orientation.clone());

        // propagate frame with parallel transport
        for i in 1..n_path {
            // Rotate the frame exactly once. The rotation matrix is assembled
            // from hyperlattice-checked unit vectors, dot, and cross products.
            let Some(rot_between) = hrotation_between_vectors(&tangents[i - 1], &tangents[i])
            else {
                return Mesh::empty();
            };
            orientation = rot_between * orientation;

            // now the slice that lives at path[i]
            let Some(translation) = htranslation_matrix(&path[i].to_vector()) else {
                return Mesh::empty();
            };
            slice_xforms.push(translation * orientation.clone());
        }

        // helper: map a 2-D point (x,y,0) through a slice transform
        #[inline]
        fn map_pt(p2: [f64; 2], m: &Matrix4) -> Option<Point3> {
            let [x, y] = p2;
            materialize_finite_point3(m.transform_point3(&finite_xy_point3(x, y)?).ok()?)
        }

        #[inline]
        fn map_real_xy(x: Real, y: Real, m: &Matrix4) -> Option<Point3> {
            materialize_finite_point3(
                m.transform_point3(&Point3::new(x, y, Real::zero())).ok()?,
            )
        }

        // collect every closed region ring and open wire profile of the sketch
        #[derive(Debug)]
        struct Ring {
            coords_2d: Vec<[f64; 2]>, // original XY coords
            slices: Vec<Vec<Point3>>, // one Vec<Point3> per path vertex
        }
        let mut rings: Vec<Ring> = Vec::new();

        let mut add_ring = |coords: Vec<[f64; 2]>| {
            if coords.len() < 2 {
                return;
            }
            let mut slices: Vec<Vec<Point3>> = Vec::with_capacity(n_path);
            for xf in &slice_xforms {
                let slice: Vec<Point3> =
                    coords.iter().filter_map(|p| map_pt(*p, xf)).collect();
                slices.push(slice);
            }
            rings.push(Ring {
                coords_2d: coords,
                slices,
            });
        };

        let mut cap_profiles: Vec<FiniteRegionRings> = Vec::new();
        let region_profiles = self.projected_region_profiles_for_mesh();
        if !region_profiles.is_empty() {
            for profile in region_profiles {
                let Some(material) =
                    orient_region_ring(profile.material().points().to_vec(), true)
                else {
                    continue;
                };
                let Some(holes) = profile
                    .holes()
                    .iter()
                    .map(|hole| orient_region_ring(hole.points().to_vec(), false))
                    .collect::<Option<Vec<_>>>()
                else {
                    continue;
                };
                add_ring(material.clone());
                for hole in &holes {
                    add_ring(hole.clone());
                }
                cap_profiles.push((material, holes));
            }
        }

        for wire in self.projected_wire_polylines_for_mesh() {
            add_ring(wire.into_points());
        }

        if rings.is_empty() {
            return Mesh::empty();
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
                    let v0 = slice_i[k].clone();
                    let v1 = slice_i[k + 1].clone();
                    let v2 = slice_j[k + 1].clone();
                    let v3 = slice_j[k].clone();

                    // triangle 1  (v0-v1-v2)
                    out_polys.push(Polygon::new(
                        vec![
                            Vertex::new(v0.clone(), Vector3::zeros()),
                            Vertex::new(v1.clone(), Vector3::zeros()),
                            Vertex::new(v2.clone(), Vector3::zeros()),
                        ],
                        metadata.clone(),
                    ));
                    // triangle 2  (v0-v2-v3)
                    out_polys.push(Polygon::new(
                        vec![
                            Vertex::new(v0.clone(), Vector3::zeros()),
                            Vertex::new(v2.clone(), Vector3::zeros()),
                            Vertex::new(v3.clone(), Vector3::zeros()),
                        ],
                        metadata.clone(),
                    ));
                }
            }
        }

        // caps for open paths
        if !path_is_closed {
            // Triangulate every 2-D polygon (outer + holes) once,
            // then reuse the triangles for both ends.

            // helper so we don’t repeat the capping code twice
            let mut add_caps = |ext: &[[f64; 2]], holes: &[Vec<[f64; 2]>]| {
                let hole_refs: Vec<&[[f64; 2]]> = holes.iter().map(|v| &v[..]).collect();

                let tris = finite_triangles_to_xy_points(
                    triangulate_finite_rings(ext, &hole_refs).unwrap_or_default(),
                );

                // cap at the start of the path (flip winding)
                for t in &tris {
                    let Some(p0) =
                        map_real_xy(t[0].x.clone(), t[0].y.clone(), &slice_xforms[0])
                    else {
                        continue;
                    };
                    let Some(p1) =
                        map_real_xy(t[1].x.clone(), t[1].y.clone(), &slice_xforms[0])
                    else {
                        continue;
                    };
                    let Some(p2) =
                        map_real_xy(t[2].x.clone(), t[2].y.clone(), &slice_xforms[0])
                    else {
                        continue;
                    };
                    out_polys.push(Polygon::new(
                        vec![
                            Vertex::new(p2, Vector3::zeros()),
                            Vertex::new(p1, Vector3::zeros()),
                            Vertex::new(p0, Vector3::zeros()),
                        ],
                        metadata.clone(),
                    ));
                }

                // cap at the end of the path
                for t in &tris {
                    let Some(p0) =
                        map_real_xy(t[0].x.clone(), t[0].y.clone(), &slice_xforms[n_path - 1])
                    else {
                        continue;
                    };
                    let Some(p1) =
                        map_real_xy(t[1].x.clone(), t[1].y.clone(), &slice_xforms[n_path - 1])
                    else {
                        continue;
                    };
                    let Some(p2) =
                        map_real_xy(t[2].x.clone(), t[2].y.clone(), &slice_xforms[n_path - 1])
                    else {
                        continue;
                    };
                    out_polys.push(Polygon::new(
                        vec![
                            Vertex::new(p0, Vector3::zeros()),
                            Vertex::new(p1, Vector3::zeros()),
                            Vertex::new(p2, Vector3::zeros()),
                        ],
                        metadata.clone(),
                    ));
                }
            };

            for (outer, holes) in &cap_profiles {
                add_caps(outer, holes);
            }
        }

        Mesh::from_polygons(&out_polys)
    }
}

fn close_region_ring(mut points: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    if points.len() >= 2
        && points
            .first()
            .zip(points.last())
            .is_some_and(|(first, last)| first != last)
    {
        let first = points[0];
        points.push(first);
    }
    points
}

fn orient_region_ring(points: Vec<[f64; 2]>, counterclockwise: bool) -> Option<Vec<[f64; 2]>> {
    let mut points = close_region_ring(points);
    let exact = points
        .iter()
        .map(|point| {
            Some(hyperlimit::Point2::new(
                Real::try_from(point[0]).ok()?,
                Real::try_from(point[1]).ok()?,
            ))
        })
        .collect::<Option<Vec<_>>>()?;
    let sign = hyperlimit::ring_area_sign(&exact).value()?;
    let is_counterclockwise = match sign {
        hyperlimit::Sign::Positive => true,
        hyperlimit::Sign::Negative => false,
        hyperlimit::Sign::Zero => return None,
    };
    if is_counterclockwise != counterclockwise {
        points.reverse();
    }
    Some(points)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csg::CSG;
    use crate::hyper_math::{Real, hreal_from_f64};

    fn r(value: f64) -> Real {
        hreal_from_f64(value).expect("test values must be finite")
    }

    #[test]
    fn close_region_ring_uses_exact_hyperreal_endpoint_identity() {
        assert_eq!(close_region_ring(vec![[0.0, 0.0], [0.0, -0.0]]).len(), 2);
        assert_eq!(close_region_ring(vec![[0.0, 0.0], [1.0e-12, 0.0]]).len(), 3);
    }

    #[test]
    fn orient_region_ring_enforces_material_and_hole_winding() {
        let clockwise = vec![[0.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0]];
        let material = orient_region_ring(clockwise.clone(), true).expect("material ring");
        let hole = orient_region_ring(clockwise, false).expect("hole ring");

        let sign = |ring: &[[f64; 2]]| {
            let exact = ring
                .iter()
                .map(|point| {
                    hyperlimit::Point2::new(
                        Real::try_from(point[0]).expect("finite x"),
                        Real::try_from(point[1]).expect("finite y"),
                    )
                })
                .collect::<Vec<_>>();
            hyperlimit::ring_area_sign(&exact).value()
        };
        assert_eq!(sign(&material), Some(hyperlimit::Sign::Positive));
        assert_eq!(sign(&hole), Some(hyperlimit::Sign::Negative));
    }

    #[test]
    fn clockwise_material_profile_extrudes_to_closed_mesh() {
        let profile = Profile::polygon(&[
            [r(0.0), r(0.0)],
            [r(0.0), r(2.0)],
            [r(1.0), r(2.0)],
            [r(1.0), r(0.0)],
        ]);
        let mesh = profile.extrude(r(3.0), ());

        assert!(!mesh.polygons.is_empty());
        mesh.to_hypermesh_exact()
            .expect("clockwise material extrusion must be closed and consistently wound");
    }

    #[test]
    fn profile_with_hole_extrudes_to_closed_mesh() {
        let outer = Profile::rectangle(r(4.0), r(4.0));
        let inner = Profile::rectangle(r(2.0), r(2.0)).translate(r(1.0), r(1.0), r(0.0));
        let mesh = outer.difference(&inner).extrude(r(3.0), ());

        assert!(!mesh.polygons.is_empty());
        mesh.to_hypermesh_exact()
            .expect("material and hole side walls must balance both caps");
    }

    #[test]
    fn sweep_discards_consecutive_duplicate_path_points() {
        let profile = Profile::square(r(1.0));
        let canonical = [
            Point3::new(r(0.0), r(0.0), r(0.0)),
            Point3::new(r(0.0), r(0.0), r(1.0)),
            Point3::new(r(0.0), r(0.0), r(2.0)),
        ];
        let with_duplicate = [
            canonical[0].clone(),
            canonical[1].clone(),
            canonical[1].clone(),
            canonical[2].clone(),
        ];

        let expected = profile.sweep(&canonical, ());
        let actual = profile.sweep(&with_duplicate, ());

        assert_eq!(actual.polygons.len(), expected.polygons.len());
        actual
            .to_hypermesh_exact()
            .expect("duplicate path points must not create degenerate sweep faces");
    }

    #[test]
    fn sweep_uses_miter_tangents_at_sharp_path_corners() {
        let profile = Profile::square(r(1.0)).translate(r(-0.5), r(-0.5), r(0.0));
        let path = [
            Point3::new(r(0.0), r(0.0), r(0.0)),
            Point3::new(r(0.0), r(0.0), r(2.0)),
            Point3::new(r(2.0), r(0.0), r(2.0)),
        ];
        let mesh = profile.sweep(&path, ());

        assert!(!mesh.polygons.is_empty());
        mesh.to_hypermesh_exact()
            .expect("a sharp path corner must not collapse its join triangles");
    }

    #[test]
    fn sweep_closes_an_explicitly_closed_path_without_a_duplicate_slice() {
        let profile = Profile::square(r(0.25)).translate(r(-0.125), r(-0.125), r(0.0));
        let path = [
            Point3::new(r(0.0), r(0.0), r(0.0)),
            Point3::new(r(1.0), r(0.0), r(0.0)),
            Point3::new(r(1.0), r(1.0), r(0.0)),
            Point3::new(r(0.0), r(1.0), r(0.0)),
            Point3::new(r(0.0), r(0.0), r(0.0)),
        ];
        let mesh = profile.sweep(&path, ());

        assert!(!mesh.polygons.is_empty());
        mesh.to_hypermesh_exact()
            .expect("an explicitly closed sweep path must produce a closed manifold");
    }

    #[test]
    fn clockwise_profile_with_hole_sweeps_to_closed_mesh() {
        let outer = Profile::polygon(&[
            [r(0.0), r(0.0)],
            [r(0.0), r(4.0)],
            [r(4.0), r(4.0)],
            [r(4.0), r(0.0)],
        ]);
        let hole = Profile::rectangle(r(2.0), r(2.0)).translate(r(1.0), r(1.0), r(0.0));
        let profile = outer.difference(&hole);
        let path = [
            Point3::new(r(0.0), r(0.0), r(0.0)),
            Point3::new(r(0.0), r(0.0), r(2.0)),
        ];
        let mesh = profile.sweep(&path, ());

        assert!(!mesh.polygons.is_empty());
        mesh.to_hypermesh_exact()
            .expect("swept material and hole walls must balance both caps");
    }

    #[test]
    fn full_revolve_reuses_the_seam_and_collapses_axis_cells() {
        let profile = Profile::rectangle(r(1.0), r(2.0));
        let mesh = profile.revolve(r(360.0), 24, ()).expect("revolve");

        assert!(!mesh.polygons.is_empty());
        mesh.to_hypermesh_exact()
            .expect("a profile touching the axis must revolve to a closed manifold");
    }

    #[test]
    fn signed_partial_revolves_have_hole_aware_caps() {
        let outer = Profile::rectangle(r(4.0), r(4.0)).translate(r(1.0), r(0.0), r(0.0));
        let hole = Profile::rectangle(r(2.0), r(2.0)).translate(r(2.0), r(1.0), r(0.0));
        let profile = outer.difference(&hole);

        for angle in [120.0, -120.0] {
            let mesh = profile.revolve(r(angle), 16, ()).expect("revolve");
            assert!(!mesh.polygons.is_empty());
            mesh.to_hypermesh_exact()
                .expect("partial revolve caps must preserve holes and signed winding");
        }
    }

    #[test]
    fn negative_radius_revolve_adjusts_angular_winding() {
        let profile = Profile::rectangle(r(2.0), r(3.0)).translate(r(-3.0), r(0.0), r(0.0));

        for angle in [360.0, -360.0, 120.0, -120.0] {
            let mesh = profile.revolve(r(angle), 16, ()).expect("revolve");
            mesh.to_hypermesh_exact()
                .expect("negative-radius profiles must retain manifold winding");
        }
    }

    #[test]
    fn revolve_rejects_profiles_that_span_the_axis() {
        let profile = Profile::rectangle(r(2.0), r(1.0)).translate(r(-1.0), r(0.0), r(0.0));
        assert!(matches!(
            profile.revolve(r(180.0), 12, ()),
            Err(ValidationError::InvalidArguments)
        ));
    }

    #[test]
    fn twisted_anisotropic_extrusion_is_one_closed_mesh() {
        let outer = Profile::rectangle(r(4.0), r(4.0));
        let hole = Profile::rectangle(r(2.0), r(2.0)).translate(r(1.0), r(1.0), r(0.0));
        let profile = outer.difference(&hole);
        let mesh = profile
            .extrude_twisted(r(5.0), r(135.0), [r(0.75), r(1.25)], 12, ())
            .expect("twisted extrusion");

        assert!(!mesh.polygons.is_empty());
        mesh.to_hypermesh_exact()
            .expect("twisted extrusion must stitch every slice and preserve holes");
    }

    #[test]
    fn negative_twisted_extrusion_preserves_winding() {
        let mesh = Profile::square(r(2.0))
            .extrude_twisted(r(-3.0), r(-90.0), [r(0.5), r(0.75)], 8, ())
            .expect("twisted extrusion");

        assert!(!mesh.polygons.is_empty());
        mesh.to_hypermesh_exact()
            .expect("negative twisted extrusion must remain a closed manifold");
    }

    #[test]
    fn multi_section_loft_derives_caps_and_triangulates_side_patches() {
        let section = |z: f64, inset: f64| {
            Polygon::new(
                vec![
                    Vertex::new(Point3::new(r(inset), r(inset), r(z)), Vector3::z()),
                    Vertex::new(Point3::new(r(2.0 - inset), r(inset), r(z)), Vector3::z()),
                    Vertex::new(
                        Point3::new(r(2.0 - inset), r(2.0 - inset), r(z)),
                        Vector3::z(),
                    ),
                    Vertex::new(Point3::new(r(inset), r(2.0 - inset), r(z)), Vector3::z()),
                ],
                (),
            )
        };
        let mesh = Profile::loft(&[section(0.0, 0.0), section(1.0, 0.25), section(2.0, 0.0)])
            .expect("loft");

        assert_eq!(mesh.polygons.len(), 18);
        mesh.to_hypermesh_exact()
            .expect("corresponding loft sections must form one closed manifold");
    }

    #[test]
    fn ring_orientation_sign_skips_leading_collinear_points() {
        let ccw = [
            [r(0.0), r(0.0)],
            [r(0.25), r(0.0)],
            [r(1.0), r(0.0)],
            [r(1.0), r(1.0)],
            [r(0.0), r(1.0)],
            [r(0.0), r(0.0)],
        ];
        let cw = [
            [r(0.0), r(0.0)],
            [r(0.0), r(0.25)],
            [r(0.0), r(1.0)],
            [r(1.0), r(1.0)],
            [r(1.0), r(0.0)],
            [r(0.0), r(0.0)],
        ];
        let collinear = [[r(0.0), r(0.0)], [r(0.25), r(0.0)], [r(1.0), r(0.0)]];

        let ccw = ccw
            .iter()
            .map(|point| hyperlimit::Point2::new(point[0].clone(), point[1].clone()))
            .collect::<Vec<_>>();
        let cw = cw
            .iter()
            .map(|point| hyperlimit::Point2::new(point[0].clone(), point[1].clone()))
            .collect::<Vec<_>>();
        let collinear = collinear
            .iter()
            .map(|point| hyperlimit::Point2::new(point[0].clone(), point[1].clone()))
            .collect::<Vec<_>>();

        assert_eq!(
            hyperlimit::ring_area_sign(&ccw).value(),
            Some(hyperlimit::Sign::Positive)
        );
        assert_eq!(
            hyperlimit::ring_area_sign(&cw).value(),
            Some(hyperlimit::Sign::Negative)
        );
        assert_eq!(
            hyperlimit::ring_area_sign(&collinear).value(),
            Some(hyperlimit::Sign::Zero)
        );
    }
}
