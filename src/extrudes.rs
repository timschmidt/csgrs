use crate::csg::{CSGError, CSG};
use crate::float_types::{EPSILON, Real};
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use geo::{Area, CoordsIter, GeometryCollection, LineString, Polygon as GeoPolygon};
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> CSG<S> {
    /// Linearly extrude this (2D) shape in the +Z direction by `height`.
    ///
    /// This is just a convenience wrapper around extrude_vector for extruding along **Z** using `Vector3::new(0.0, 0.0, height)`
    pub fn extrude(&self, height: Real) -> CSG<S> {
        self.extrude_vector(Vector3::new(0.0, 0.0, height)).expect("expected height to be > then EPSILON")
    }

    /// Linearly extrude any 2D geometry (Polygons, MultiPolygons, or sub-geometries in a GeometryCollection)
    /// in `self.geometry` along the given `direction`.
    ///
    /// Builds top, bottom, and side polygons in 3D, storing them in `csg.polygons`.
    /// Returns a new CSG containing these extruded polygons (plus any existing 3D polygons
    /// you already had in `self.polygons`).
    pub fn extrude_vector(&self, direction: Vector3<Real>) -> Result<CSG<S>, CSGError> {
        // If the direction is near zero length, nothing to extrude:
        if direction.norm() < EPSILON {
            return Err(CSGError::FieldLessThenFloat { name: "direction.norm()", min: EPSILON })
        }

        // Collect the new 3D polygons generated by extrusion:
        let mut new_polygons = Vec::new();

        // A helper to handle any Geometry
        fn extrude_geometry<S: Clone + Send + Sync>(
            geom: &geo::Geometry<Real>,
            direction: Vector3<Real>,
            metadata: &Option<S>,
            out_polygons: &mut Vec<Polygon<S>>,
        ) {
            match geom {
                geo::Geometry::Polygon(poly) => {
                    let exterior_coords: Vec<[Real; 2]> =
                        poly.exterior().coords_iter().map(|c| [c.x, c.y]).collect();
                    let interior_rings: Vec<Vec<[Real; 2]>> = poly
                        .interiors()
                        .iter()
                        .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
                        .collect();

                    // bottom
                    let bottom_tris = CSG::<()>::tessellate_2d(
                        &exterior_coords,
                        &interior_rings.iter().map(|r| &r[..]).collect::<Vec<_>>(),
                    );
                    for tri in bottom_tris {
                        let v0 = Vertex::new(tri[2], -Vector3::z());
                        let v1 = Vertex::new(tri[1], -Vector3::z());
                        let v2 = Vertex::new(tri[0], -Vector3::z());
                        out_polygons.push(Polygon::from_tri(&[v0, v1, v2], metadata.clone()));
                    }
                    // top
                    let top_tris = CSG::<()>::tessellate_2d(
                        &exterior_coords,
                        &interior_rings.iter().map(|r| &r[..]).collect::<Vec<_>>(),
                    );
                    for tri in top_tris {
                        let p0 = tri[0] + direction;
                        let p1 = tri[1] + direction;
                        let p2 = tri[2] + direction;
                        let v0 = Vertex::new(p0, Vector3::z());
                        let v1 = Vertex::new(p1, Vector3::z());
                        let v2 = Vertex::new(p2, Vector3::z());
                        out_polygons.push(Polygon::from_tri(&[v0, v1, v2], metadata.clone()));
                    }

                    // sides
                    let all_rings = std::iter::once(poly.exterior()).chain(poly.interiors());
                    for ring in all_rings {
                        let coords: Vec<_> = ring.coords_iter().collect();
                        let n = coords.len();
                        if n < 2 {
                            continue;
                        }
                        for i in 0..(n - 1) {
                            let j = i + 1;
                            let c_i = coords[i];
                            let c_j = coords[j];
                            let b_i = Point3::new(c_i.x, c_i.y, 0.0);
                            let b_j = Point3::new(c_j.x, c_j.y, 0.0);
                            let t_i = b_i + direction;
                            let t_j = b_j + direction;
                            out_polygons.push(Polygon::new(
                                vec![
                                    Vertex::new(b_i, Vector3::zeros()),
                                    Vertex::new(b_j, Vector3::zeros()),
                                    Vertex::new(t_j, Vector3::zeros()),
                                    Vertex::new(t_i, Vector3::zeros()),
                                ],
                                metadata.clone(),
                            ).expect("More then three points provided"));
                        }
                    }
                }
                geo::Geometry::MultiPolygon(mp) => {
                    for poly in &mp.0 {
                        extrude_geometry(&geo::Geometry::Polygon(poly.clone()), direction, metadata, out_polygons);
                    }
                }
                geo::Geometry::GeometryCollection(gc) => {
                    for sub in &gc.0 {
                        extrude_geometry(sub, direction, metadata, out_polygons);
                    }
                }
                // Other geometry types (LineString, Point, etc.) are skipped or could be handled differently:
                _ => { /* skip */ }
            }
        }

        // -- 1) Extrude the polygons from self.polygons if you also want to re-extrude them.
        //    (Often, your `polygons` are already 3D, so you might skip extruding them again.)
        //    We'll skip it here for clarity.  If you do want to extrude existing 2D polygons
        //    in `self.polygons`, you'd add similar logic.

        // -- 2) Extrude from self.geometry (the `geo::GeometryCollection`).
        for geom in &self.geometry {
            extrude_geometry(geom, direction, &self.metadata, &mut new_polygons);
        }

        // Combine new extruded polygons with any existing 3D polygons:
        let mut final_polygons = self.polygons.clone();
        final_polygons.extend(new_polygons);

        // Return a new CSG
        Ok(CSG {
            polygons: final_polygons,
            geometry: self.geometry.clone(),
            metadata: self.metadata.clone(),
        })
    }

    /// Extrudes (or "lofts") a closed 3D volume between two polygons in space.
    /// - `bottom` and `top` each have the same number of vertices `n`, in matching order.
    /// - Returns a new CSG whose faces are:
    ///   - The `bottom` polygon,
    ///   - The `top` polygon,
    ///   - `n` rectangular side polygons bridging each edge of `bottom` to the corresponding edge of `top`.
    pub fn extrude_between(
        bottom: &Polygon<S>,
        top: &Polygon<S>,
        flip_bottom_polygon: bool,
    ) -> CSG<S> {
        let n = bottom.vertices.len();
        assert_eq!(
            n,
            top.vertices.len(),
            "extrude_between: both polygons must have the same number of vertices" // todo: return error
        );

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
                    b_i.clone(), // bottom[i]
                    b_j.clone(), // bottom[i+1]
                    t_j.clone(), // top[i+1]
                    t_i.clone(), // top[i]
                ],
                bottom.metadata.clone(), // carry over bottom polygon metadata
            ).expect("More then three points provided");
            polygons.push(side_poly);
        }

        CSG::from_polygons(&polygons)
    }

    /*
    /// Perform a linear extrusion along some axis, with optional twist, center, slices, scale, etc.
    ///
    /// # Parameters
    /// - `direction`: Direction vector for the extrusion.
    /// - `twist`: Total twist in degrees around the extrusion axis from bottom to top.
    /// - `segments`: Number of intermediate subdivisions.
    /// - `scale`: A uniform scale factor to apply at the top slice (bottom is scale=1.0).
    ///
    /// # Assumptions
    /// - This CSG is assumed to represent one or more 2D polygons lying in or near the XY plane.
    /// - The resulting shape is extruded *initially* along +Z, then finally rotated if `v != [0,0,1]`.
    ///
    /// # Returns
    /// A new 3D CSG.
    ///
    /// # Example
    /// ```
    /// let shape_2d = CSG::square(2.0, 2.0, None); // a 2D square in XY
    /// let extruded = shape_2d.linear_extrude(
    ///     direction = Vector3::new(0.0, 0.0, 10.0),
    ///     twist = 360.0,
    ///     segments = 32,
    ///     scale = 1.2,
    /// );
    /// ```
    pub fn linear_extrude(
        shape: &CCShape<Real>,
        direction: Vector3<Real>,
        twist_degs: Real,
        segments: usize,
        scale_top: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        let mut polygons_3d = Vec::new();
        if segments < 1 {
            return CSG::new();
        }
        let height = direction.norm();
        if height < EPSILON {
            // no real extrusion
            return CSG::new();
        }

        // Step 1) Build a series of “transforms” from bottom=0..top=height, subdivided into `segments`.
        //   For each i in [0..=segments], compute fraction f and:
        //   - scale in XY => s_i
        //   - twist about Z => rot_i
        //   - translate in Z => z_i
        //
        //   We'll store each “slice” in 3D form as a Vec<Vec<Point3<Real>>>,
        //   i.e. one 3D polyline for each boundary or hole in the shape.
        let mut slices: Vec<Vec<Vec<Point3<Real>>>> = Vec::with_capacity(segments + 1);
        // The axis to rotate around is the unit of `direction`. We'll do final alignment after constructing them along +Z.
        let axis_dir = direction.normalize();

        for i in 0..=segments {
            let f = i as Real / segments as Real;
            let s_i = 1.0 + (scale_top - 1.0) * f;  // lerp(1, scale_top, f)
            let twist_rad = twist_degs.to_radians() * f;
            let z_i = height * f;

            // Build transform T = Tz * Rz * Sxy
            //  - scale in XY
            //  - twist around Z
            //  - translate in Z
            let mat_scale = Matrix4::new_nonuniform_scaling(&Vector3::new(s_i, s_i, 1.0));
            let mat_rot = Rotation3::from_axis_angle(&Vector3::z_axis(), twist_rad).to_homogeneous();
            let mat_trans = Translation3::new(0.0, 0.0, z_i).to_homogeneous();
            let slice_mat = mat_trans * mat_rot * mat_scale;

            let slice_3d = project_shape_3d(shape, &slice_mat);
            slices.push(slice_3d);
        }

        // Step 2) “Stitch” consecutive slices to form side polygons.
        // For each pair of slices[i], slices[i+1], for each boundary polyline j,
        // connect edges. We assume each polyline has the same vertex_count in both slices.
        // (If the shape is closed, we do wrap edges [n..0].)
        // Then we optionally build bottom & top caps if the polylines are closed.

        // a) bottom + top caps, similar to extrude_vector approach
        //    For slices[0], build a “bottom” by triangulating in XY, flipping normal.
        //    For slices[segments], build a “top” by normal up.
        //
        //    But we only do it if each boundary is closed.
        //    We must group CCW with matching holes. This is the same logic as `extrude_vector`.

        // We'll do a small helper that triangulates shape in 2D, then lifts that triangulation to slice_3d.
        // You can re‐use the logic from `extrude_vector`.

        // Build the “bottom” from slices[0] if polylines are all or partially closed
        polygons_3d.extend(
            build_caps_from_slice(shape, &slices[0], true, metadata.clone())
        );
        // Build the “top” from slices[segments]
        polygons_3d.extend(
            build_caps_from_slice(shape, &slices[segments], false, metadata.clone())
        );

        // b) side walls
        for i in 0..segments {
            let bottom_slice = &slices[i];
            let top_slice = &slices[i + 1];

            // We know bottom_slice has shape.ccw_plines.len() + shape.cw_plines.len() polylines
            // in the same order. Each polyline has the same vertex_count as in top_slice.
            // So we can do a direct 1:1 match: bottom_slice[j] <-> top_slice[j].
            for (pline_idx, bot3d) in bottom_slice.iter().enumerate() {
                let top3d = &top_slice[pline_idx];
                if bot3d.len() < 2 {
                    continue;
                }
                // is it closed? We can check shape’s corresponding polyline
                let is_closed = if pline_idx < shape.ccw_plines.len() {
                    shape.ccw_plines[pline_idx].polyline.is_closed()
                } else {
                    shape.cw_plines[pline_idx - shape.ccw_plines.len()].polyline.is_closed()
                };
                let n = bot3d.len();
                let edge_count = if is_closed { n } else { n - 1 };

                for k in 0..edge_count {
                    let k_next = (k + 1) % n;
                    let b_i = bot3d[k];
                    let b_j = bot3d[k_next];
                    let t_i = top3d[k];
                    let t_j = top3d[k_next];

                    let poly_side = Polygon::new(
                        vec![
                            Vertex::new(b_i, Vector3::zeros()),
                            Vertex::new(b_j, Vector3::zeros()),
                            Vertex::new(t_j, Vector3::zeros()),
                            Vertex::new(t_i, Vector3::zeros()),
                        ],
                        metadata.clone(),
                    );
                    polygons_3d.push(poly_side);
                }
            }
        }

        // Step 3) If direction is not along +Z, rotate final mesh so +Z aligns with your direction
        // (This is optional or can be done up front. Typical OpenSCAD style is to do everything
        // along +Z, then rotate the final.)
        if (axis_dir - Vector3::z()).norm() > EPSILON {
            // rotate from +Z to axis_dir
            let rot_axis = Vector3::z().cross(&axis_dir);
            let sin_theta = rot_axis.norm();
            if sin_theta > EPSILON {
                let cos_theta = Vector3::z().dot(&axis_dir);
                let angle = cos_theta.acos();
                let rot = Rotation3::from_axis_angle(&Unit::new_normalize(rot_axis), angle);
                let mat = rot.to_homogeneous();
                // transform the polygons
                let mut final_polys = Vec::with_capacity(polygons_3d.len());
                for mut poly in polygons_3d {
                    for v in &mut poly.vertices {
                        let pos4 = mat * nalgebra::Vector4::new(v.pos.x, v.pos.y, v.pos.z, 1.0);
                        v.pos = Point3::new(pos4.x / pos4.w, pos4.y / pos4.w, pos4.z / pos4.w);
                    }
                    poly.set_new_normal();
                    final_polys.push(poly);
                }
                return CSG::from_polygons(&final_polys);
            }
        }

        // otherwise, just return as is
        CSG::from_polygons(&polygons_3d)
    }
    */

    /// Rotate-extrude (revolve) only the Polygon and MultiPolygon geometry in `self.geometry`
    /// around the Y-axis from 0..`angle_degs` in `segments` steps, producing side walls
    /// in an orientation consistent with the polygon's winding.
    ///
    /// - Ignores `self.polygons`.
    /// - Returns a new CSG containing **only** the newly extruded side polygons (no end caps).
    /// - `angle_degs`: how far to revolve, in degrees (e.g. 360 for a full revolve).
    /// - `segments`: number of subdivisions around the revolve.
    ///
    /// ## Key Points
    /// - Only 2D geometry in `self.geometry` is used. Any `self.polygons` are ignored.
    /// - Axis of revolution: **Y-axis**. We treat each ring's (x,y) -> revolve_around_y(x,y,theta).
    /// - Exterior rings (CCW in Geo) produce outward-facing side polygons.
    /// - Interior rings (CW) produce inward-facing side polygons ("holes").
    /// - If `angle_degs < 360`, we add **two caps**: one at angle=0, one at angle=angle_degs.
    ///   - Cap orientation is set so that normals face outward, consistent with a solid.
    /// - Returns a new CSG with `.polygons` containing only the side walls + any caps.
    ///   The `.geometry` is empty, i.e. `GeometryCollection::default()`.
    ///
    /// ## Errors
    /// Returns an error if `segments` is less then 2
    pub fn rotate_extrude(&self, angle_degs: Real, segments: usize) -> Result<CSG<S>, CSGError> {
        if segments < 2 {
            // rotate_extrude requires at least 2 segments
            return Err(CSGError::LessThen2ExtrudeSegments);
        }

        let angle_radians = angle_degs.to_radians();
        let mut new_polygons = Vec::new();

        // A small helper to revolve a point (x,y) in the XY plane around the Y-axis by theta.
        // The output is a 3D point (X, Y, Z).
        fn revolve_around_y(x: Real, y: Real, theta: Real) -> Point3<Real> {
            let cos_t = theta.cos();
            let sin_t = theta.sin();
            // Map (x, y, 0) => ( x*cos θ, y, x*sin θ )
            Point3::new(x * cos_t, y, x * sin_t)
        }

        // Another helper to determine if a ring (LineString) is CCW or CW in Geo.
        // In `geo`, ring.exterior() is CCW for an outer boundary, CW for holes.
        // If the signed area > 0 => CCW; < 0 => CW.
        fn is_ccw(ring: &LineString<Real>) -> bool {
            let poly = GeoPolygon::new(ring.clone(), vec![]);
            poly.signed_area() > 0.0
        }

        // A helper to extrude one ring of coordinates (including the last->first if needed),
        // pushing its side polygons into `out_polygons`.
        // - `ring_coords`: The ring’s sequence of points. Usually closed (last=first).
        // - `ring_is_ccw`: true if it's an exterior ring, false if interior/hole.
        // - `angle_radians`: total revolve sweep in radians.
        // - `segments`: how many discrete slices around the revolve.
        // - `metadata`: user metadata to attach to side polygons.
        fn revolve_ring<S: Clone + Send + Sync>(
            ring_coords: &[geo::Coord<Real>],
            ring_is_ccw: bool,
            angle_radians: Real,
            segments: usize,
            metadata: &Option<S>,
        ) -> Vec<Polygon<S>> {
            if ring_coords.len() < 2 {
                return vec![];
            }

            let mut out_polygons = Vec::new();
            // Typically the last point = first point for a closed ring.
            // We'll iterate over each edge i..i+1, and revolve them around by segments slices.

            // The revolve step size in radians:
            let step = angle_radians / (segments as Real);

            // For each edge in the ring:
            for i in 0..(ring_coords.len() - 1) {
                let c_i = ring_coords[i];
                let c_j = ring_coords[i + 1];

                // If these two points are the same, skip degenerate edge
                if (c_i.x - c_j.x).abs() < EPSILON && (c_i.y - c_j.y).abs() < EPSILON {
                    continue;
                }

                // For each revolve slice j..j+1
                for s in 0..segments {
                    let th0 = s as Real * step;
                    let th1 = (s as Real + 1.0) * step;

                    // revolve bottom edge endpoints at angle th0
                    let b_i = revolve_around_y(c_i.x, c_i.y, th0);
                    let b_j = revolve_around_y(c_j.x, c_j.y, th0);
                    // revolve top edge endpoints at angle th1
                    let t_i = revolve_around_y(c_i.x, c_i.y, th1);
                    let t_j = revolve_around_y(c_j.x, c_j.y, th1);

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
                    .map(|pos| Vertex::new(pos, Vector3::zeros()))
                    .collect();

                    out_polygons.push(Polygon::new(quad_verts, metadata.clone())
                        .expect("Four points are provided"));
                }
            }
            out_polygons
        }

        // Build a single “cap” polygon from ring_coords at a given angle (0 or angle_radians).
        //  - revolve each 2D point by `angle`, produce a 3D ring
        //  - if `flip` is true, reverse the ring so the normal is inverted
        fn build_cap_polygon<S: Clone + Send + Sync>(
            ring_coords: &[geo::Coord<Real>],
            angle: Real,
            flip: bool,
            metadata: &Option<S>,
        ) -> Result<Polygon<S>, CSGError> {
            if ring_coords.len() < 3 {
                return Err(CSGError::FieldLessThen { name: "ring_coords.len()", min: 3 });
            }

            // revolve each coordinate at the given angle
            let mut pts_3d: Vec<_> = ring_coords
                .iter()
                .map(|c| revolve_around_y(c.x, c.y, angle))
                .collect();

            // ensure closed if the ring wasn't strictly closed
            // (the last point in a Geo ring is typically the same as the first)
            let last = pts_3d.last().unwrap();
            let first = pts_3d.first().unwrap();
            if (last.x - first.x).abs() > EPSILON
                || (last.y - first.y).abs() > EPSILON
                || (last.z - first.z).abs() > EPSILON
            {
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
            Polygon::new(verts, metadata.clone())
        }

        //----------------------------------------------------------------------
        // 2) Iterate over each geometry (Polygon or MultiPolygon),
        //    revolve the side walls, and possibly add caps if angle_degs < 360.
        //----------------------------------------------------------------------
        let full_revolve = (angle_degs - 360.0).abs() < EPSILON; // or angle_degs >= 359.999..., etc.
        let do_caps = !full_revolve && (angle_degs > 0.0);

        for geom in &self.geometry {
            match geom {
                geo::Geometry::Polygon(poly2d) => {
                    // Exterior ring
                    let ext_ring = poly2d.exterior();
                    let ext_ccw = is_ccw(ext_ring);

                    // (A) side walls
                    new_polygons.extend(revolve_ring(
                        &ext_ring.0,
                        ext_ccw,
                        angle_radians,
                        segments,
                        &self.metadata,
                    ));

                    // (B) cap(s) if partial revolve
                    if do_caps {
                        // start-cap at angle=0
                        //   flip if ext_ccw == true
                        if let Ok(cap) = build_cap_polygon(
                            &ext_ring.0,
                            0.0,
                            ext_ccw, // exterior ring => flip the start cap
                            &self.metadata,
                        ) {
                            new_polygons.push(cap);
                        }

                        // end-cap at angle= angle_radians
                        //   flip if ext_ccw == false
                        if let Ok(cap) = build_cap_polygon(
                            &ext_ring.0,
                            angle_radians,
                            !ext_ccw, // exterior ring => keep normal orientation for end
                            &self.metadata,
                        ) {
                            new_polygons.push(cap);
                        }
                    }

                    // Interior rings (holes)
                    for hole in poly2d.interiors() {
                        let hole_ccw = is_ccw(hole);
                        new_polygons.extend(revolve_ring(
                            &hole.0,
                            hole_ccw,
                            angle_radians,
                            segments,
                            &self.metadata,
                        ));
                    }
                }

                geo::Geometry::MultiPolygon(mpoly) => {
                    // Each Polygon inside
                    for poly2d in &mpoly.0 {
                        let ext_ring = poly2d.exterior();
                        let ext_ccw = is_ccw(ext_ring);

                        new_polygons.extend(revolve_ring(
                            &ext_ring.0,
                            ext_ccw,
                            angle_radians,
                            segments,
                            &self.metadata,
                        ));
                        if do_caps {
                            if let Ok(cap) = build_cap_polygon(&ext_ring.0, 0.0, ext_ccw, &self.metadata) {
                                new_polygons.push(cap);
                            }
                            if let Ok(cap) = build_cap_polygon(
                                &ext_ring.0,
                                angle_radians,
                                !ext_ccw,
                                &self.metadata,
                            ) {
                                new_polygons.push(cap);
                            }
                        }

                        // holes
                        for hole in poly2d.interiors() {
                            let hole_ccw = is_ccw(hole);
                            new_polygons.extend(revolve_ring(
                                &hole.0,
                                hole_ccw,
                                angle_radians,
                                segments,
                                &self.metadata,
                            ));
                        }
                    }
                }

                // Ignore lines, points, etc.
                _ => {}
            }
        }

        //----------------------------------------------------------------------
        // 3) Return the new CSG:
        //----------------------------------------------------------------------
        Ok(CSG {
            polygons: new_polygons,
            geometry: GeometryCollection::default(),
            metadata: self.metadata.clone(),
        })
    }

    // Sweep a 2D shape `shape_2d` (in XY plane, normal=+Z) along a 2D path `path_2d` (also in XY).
    // Produces a 3D CSG whose cross-sections match `shape_2d` at each vertex of `path_2d`.
    //
    // - If `path_2d` is open, the shape is capped at the start/end.
    // - If `path_2d` is closed, we connect the last copy back to the first, forming a loop without caps.
    //
    // # Assumptions
    // - `shape_2d` is a single Polygon in the XY plane. Its normal should be +Z.
    // - `path_2d` is a single Polygon (or open polyline) also in XY. If `path_2d.open==false`, it’s closed.
    // - Both polygons have enough vertices to be meaningful (e.g. 3+ for the shape if closed, 2+ for the path).
    //
    // # Returns
    // A new 3D `CSG` that is the swept volume.
    /*
    pub fn sweep(shape_2d: &Polygon<S>, path_2d: &Polygon<S>) -> CSG<S> {
        // Gather the path’s vertices in XY
        if path_2d.vertices.len() < 2 {
            // Degenerate path => no sweep
            return CSG::new();
        }
        let path_is_closed = !path_2d.open;  // If false => open path, if true => closed path

        // Extract path points (x,y,0) from path_2d
        let mut path_points = Vec::with_capacity(path_2d.vertices.len());
        for v in &path_2d.vertices {
            // We only take X & Y; Z is typically 0 for a 2D path
            path_points.push(Point3::new(v.pos.x, v.pos.y, 0.0));
        }

        // Convert the shape_2d into a list of its vertices in local coords (usually in XY).
        // We assume shape_2d is a single polygon (can also handle multiple if needed).
        let shape_is_closed = !shape_2d.open && shape_2d.vertices.len() >= 3;
        let shape_count = shape_2d.vertices.len();

        // For each path vertex, compute the orientation that aligns +Z to the path tangent.
        // Then transform the shape’s 2D vertices into 3D “slice[i]”.
        let n_path = path_points.len();
        let mut slices: Vec<Vec<Point3<Real>>> = Vec::with_capacity(n_path);

        for i in 0..n_path {
            // The path tangent is p[i+1] - p[i] (or wrap if path is closed)
            // If open and i == n_path-1 => we’ll copy the tangent from the last segment
            let next_i = if i == n_path - 1 {
                if path_is_closed { 0 } else { i - 1 } // if closed, wrap, else reuse the previous
            } else {
                i + 1
            };

            let mut dir = path_points[next_i] - path_points[i];
            if dir.norm_squared() < EPSILON {
                // Degenerate segment => fallback to the previous direction or just use +Z
                dir = Vector3::z();
            } else {
                dir.normalize_mut();
            }

            // Build a rotation that maps +Z to `dir`.
            // We'll rotate the z-axis (0,0,1) onto `dir`.
            let z = Vector3::z();
            let dot = z.dot(&dir);
            // If dir is basically the same as z, no rotation needed
            if (dot - 1.0).abs() < EPSILON {
                return Matrix4::identity();
            }
            // If dir is basically opposite z
            if (dot + 1.0).abs() < EPSILON {
                // 180 deg around X or Y axis
                let rot180 = Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::x()), PI);
                return rot180.to_homogeneous();
            }
            // Otherwise, general axis = z × dir
            let axis = z.cross(&dir).normalize();
            let angle = z.dot(&dir).acos();
            let initial_rot = Rotation3::from_axis_angle(&Unit::new_unchecked(axis), angle);
            let rot = initial_rot.to_homogeneous()

            // Build a translation that puts shape origin at path_points[i]
            let trans = Translation3::from(path_points[i].coords);

            // Combined transform = T * R
            let mat = trans.to_homogeneous() * rot;

            // Apply that transform to all shape_2d vertices => slice[i]
            let mut slice_i = Vec::with_capacity(shape_count);
            for sv in &shape_2d.vertices {
                let local_pt = sv.pos;  // (x, y, z=0)
                let p4 = local_pt.to_homogeneous();
                let p4_trans = mat * p4;
                slice_i.push(Point3::from_homogeneous(p4_trans).unwrap());
            }
            slices.push(slice_i);
        }

        // Build polygons for the new 3D swept solid.
        // - (A) “Cap” polygons at start & end if path is open.
        // - (B) “Side wall” quads between slice[i] and slice[i+1].
        //
        // We’ll gather them all into a Vec<Polygon<S>>, then make a CSG.

        let mut all_polygons = Vec::new();

        // Caps if path is open
        //  We replicate the shape_2d as polygons at slice[0] and slice[n_path-1].
        //  We flip the first one so its normal faces outward. The last we keep as is.
        if !path_is_closed {
            // “Bottom” cap = slice[0], but we flip its winding so outward normal is “down” the path
            if shape_is_closed {
                let bottom_poly = polygon_from_slice(
                    &slices[0],
                    true, // flip
                    shape_2d.metadata.clone(),
                );
                all_polygons.push(bottom_poly);
            }
            // “Top” cap = slice[n_path-1] (no flip)
            if shape_is_closed {
                let top_poly = polygon_from_slice(
                    &slices[n_path - 1],
                    false, // no flip
                    shape_2d.metadata.clone(),
                );
                all_polygons.push(top_poly);
            }
        }

        // Side walls: For i in [0..n_path-1], or [0..n_path] if closed
        let end_index = if path_is_closed { n_path } else { n_path - 1 };

        for i in 0..end_index {
            let i_next = (i + 1) % n_path;  // wraps if closed
            let slice_i = &slices[i];
            let slice_next = &slices[i_next];

            // For each edge in the shape, connect vertices k..k+1
            // shape_2d may be open or closed. If open, we do shape_count-1 edges; if closed, shape_count edges.
            let edge_count = if shape_is_closed {
                shape_count  // because last edge wraps
            } else {
                shape_count - 1
            };

            for k in 0..edge_count {
                let k_next = (k + 1) % shape_count;

                let v_i_k     = slice_i[k];
                let v_i_knext = slice_i[k_next];
                let v_next_k     = slice_next[k];
                let v_next_knext = slice_next[k_next];

                // Build a quad polygon in CCW order for outward normal
                // or you might choose a different ordering.  Typically:
                //   [v_i_k, v_i_knext, v_next_knext, v_next_k]
                // forms an outward-facing side wall if the shape_2d was originally CCW in XY.
                let side_poly = Polygon::new(
                    vec![
                        Vertex::new(v_i_k,     Vector3::zeros()),
                        Vertex::new(v_i_knext, Vector3::zeros()),
                        Vertex::new(v_next_knext, Vector3::zeros()),
                        Vertex::new(v_next_k,     Vector3::zeros()),
                    ],
                    shape_2d.metadata.clone(),
                );
                all_polygons.push(side_poly);
            }
        }

        // Combine into a final CSG
        CSG::from_polygons(&all_polygons)
    }
    */
}

/// Helper to build a single Polygon from a “slice” of 3D points.
///
/// If `flip_winding` is true, we reverse the vertex order (so the polygon’s normal flips).
fn _polygon_from_slice<S: Clone + Send + Sync>(
    slice_pts: &[Point3<Real>],
    flip_winding: bool,
    metadata: Option<S>,
) -> Result<Polygon<S>, CSGError> {
    if slice_pts.len() < 3 {
        // degenerate polygon
        return Err(CSGError::FieldLessThen { name: "slice_pts.len()", min: 3 });
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

    let mut poly = Polygon::new(verts, metadata)?;
    poly.set_new_normal(); // Recompute its plane & normal for consistency

    Ok(poly)
}
