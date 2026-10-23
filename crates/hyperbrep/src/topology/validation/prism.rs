//! Validated construction: validation prism.

use crate::topology::*;

impl ArenaModelBuilder {
    pub(in crate::topology) fn certify_curved_prism_shell(
        &self,
        shell: ShellId,
    ) -> Result<bool, BuildError> {
        let mut cap_surfaces = HashSet::new();
        for face_id in &self.shell_ref(shell)?.faces {
            let face = self.face_ref(*face_id)?;
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::Plane => {
                    cap_surfaces.insert(face.surface);
                }
                SurfaceKind::Extrusion | SurfaceKind::Cylinder => {}
                _ => return Ok(false),
            }
            for wire_id in face.boundary_wires() {
                for edge_use_id in &self.wire_ref(*wire_id)?.edge_uses {
                    let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
                    if !matches!(
                        self.curve_ref(edge.curve)?.kind(),
                        Curve3Kind::Line
                            | Curve3Kind::Translation
                            | Curve3Kind::CircleArc
                            | Curve3Kind::EllipseArc
                            | Curve3Kind::RationalBezier
                            | Curve3Kind::Nurbs
                            | Curve3Kind::PlanarImage
                    ) {
                        return Ok(false);
                    }
                }
            }
        }
        if cap_surfaces.len() < 2 {
            return Ok(false);
        }
        self.certify_translation_shell_topology(shell)
    }

    pub(in crate::topology) fn certify_translation_shell_topology(
        &self,
        shell: ShellId,
    ) -> Result<bool, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        if faces.len() < 4 {
            return Ok(false);
        }
        let groups = self.canonical_prism_face_groups(faces)?;
        for first_index in 0..groups.len() {
            for second_index in (first_index + 1)..groups.len() {
                if self.certify_prism_cap_pair(
                    faces,
                    &groups[first_index],
                    &groups[second_index],
                )? {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }

    pub(in crate::topology) fn certified_prism_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedPrismShell>, BuildError> {
        Ok(self
            .certified_prism_shell_and_orientation(shell)?
            .map(|(prism, _)| prism))
    }

    /// Cap orientation and transverse translation establish shell orientation.
    /// A closed-form volume is optional; an inward shell must still be rejected.
    pub(in crate::topology) fn certified_prism_shell_and_orientation(
        &self,
        shell: ShellId,
    ) -> Result<Option<(CertifiedPrismShell, std::cmp::Ordering)>, BuildError> {
        if !self.certify_simple_prism_shell(shell)?
            && !self.certify_internally_partitioned_prism_shell(shell)?
            && !self.certify_curved_prism_shell(shell)?
        {
            return Ok(None);
        }
        let faces = &self.shell_ref(shell)?.faces;
        let groups = self.canonical_prism_face_groups(faces)?;
        for first_index in 0..groups.len() {
            for second_index in (first_index + 1)..groups.len() {
                let first_group = &groups[first_index];
                let second_group = &groups[second_index];
                if !self.certify_prism_cap_pair(faces, first_group, second_group)? {
                    continue;
                }
                let first_face = self.face_ref(faces[first_group[0]])?;
                let Some(first_plane) = self.canonical_prism_plane(first_face.surface)? else {
                    continue;
                };
                let SurfaceExactData::Plane { origin, u, v } = first_plane.exact_data() else {
                    unreachable!("canonical prism support is a plane");
                };
                let first_boundary = self
                    .cap_boundary_use_loops(faces, first_group)?
                    .expect("certified cap group has a manifold boundary");
                if !self.curved_cap_pcurves_use_frame(
                    &first_boundary,
                    first_face.surface,
                    &first_plane,
                )? {
                    continue;
                }
                let second_boundary = self
                    .cap_boundary_use_loops(faces, second_group)?
                    .expect("certified cap group has a manifold boundary");
                let first_vertices = self.boundary_loop_vertex_set(&first_boundary)?;
                let second_vertices = self.boundary_loop_vertex_set(&second_boundary)?;
                let first_edges = self.boundary_loop_edge_set(&first_boundary)?;
                let second_edges = self.boundary_loop_edge_set(&second_boundary)?;
                let paths = self
                    .prism_cross_paths(
                        faces,
                        &first_vertices,
                        &second_vertices,
                        &first_edges,
                        &second_edges,
                    )?
                    .ok_or(BuildError::UnsupportedSolidShell(shell))?;
                let extrusion = paths
                    .into_iter()
                    .next()
                    .ok_or(BuildError::UnsupportedSolidShell(shell))?
                    .displacement;
                let determinant = u.dot(&v.cross(&extrusion));
                if decided_model_order(compare_reals(
                    &determinant,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? == std::cmp::Ordering::Equal
                {
                    return Ok(None);
                }
                let mut outer = None;
                let mut holes = Vec::new();
                let mut profile_area = Some(Real::zero());
                let mut projected_sources = HashMap::new();
                for uses in first_boundary {
                    let curves = uses
                        .iter()
                        .map(|edge_use_id| -> Result<Curve2, BuildError> {
                            let edge_use = self.edge_use_ref(*edge_use_id)?;
                            let edge = self.edge_ref(edge_use.edge)?;
                            if self.curve_ref(edge.curve)?.kind() == Curve3Kind::Line {
                                let (start, end) = self.directed_vertices(*edge_use_id)?;
                                if self.vertex_ref(start)?.point().try_coordinates().is_none()
                                    || self.vertex_ref(end)?.point().try_coordinates().is_none()
                                {
                                    // Project the authored source once; retain each exact trim
                                    // and its traversal instead of evaluating selected vertices.
                                    let source = match projected_sources.entry(edge.curve) {
                                        std::collections::hash_map::Entry::Occupied(entry) => {
                                            entry.into_mut()
                                        }
                                        std::collections::hash_map::Entry::Vacant(entry) => entry
                                            .insert(
                                                crate::geometry::project_curve_to_plane_frame(
                                                    self.curve_ref(edge.curve)?,
                                                    &origin,
                                                    &u,
                                                    &v,
                                                )?
                                                .expect("a cap line projects to the cap plane"),
                                            ),
                                    };
                                    let range = hypercurve::CurveParameterRange2::try_new(
                                        edge.domain.start().curve_parameter(),
                                        edge.domain.end().curve_parameter(),
                                    )
                                    .map_err(GeometryError::from)?;
                                    let curve = source
                                        .restrict_source(range)
                                        .map_err(GeometryError::from)?;
                                    return Ok(if edge_use.direction == Direction::Reversed {
                                        curve.reversed().map_err(GeometryError::from)?
                                    } else {
                                        curve
                                    });
                                }
                                Ok(Curve2::from(
                                    LineSeg2::try_new(
                                        project_point_to_plane_frame(
                                            self.vertex_ref(start)?.represented_point()?,
                                            &origin,
                                            &u,
                                            &v,
                                        )?,
                                        project_point_to_plane_frame(
                                            self.vertex_ref(end)?.represented_point()?,
                                            &origin,
                                            &u,
                                            &v,
                                        )?,
                                    )
                                    .map_err(GeometryError::from)?,
                                ))
                            } else {
                                Ok(self.pcurve_ref(edge_use.pcurve)?.curve().clone())
                            }
                        })
                        .collect::<Result<Vec<_>, _>>()?;
                    let path = CurvePath2::try_new(wire_geometry::coalesce_restrictions(curves)?)
                        .map_err(GeometryError::from)?;
                    let area = match self.conic_cap_signed_area(&uses, &u, &v)? {
                        Some(area) => Some(area),
                        None => resolve_planar_classification(
                            crate::error::classified(
                                path.boundary_loop()
                                    .map_err(GeometryError::from)?
                                    .signed_area(),
                            )
                            .map_err(GeometryError::from)?,
                        )?,
                    };
                    let order = match &area {
                        Some(area) => decided_model_order(compare_reals(
                            area,
                            &Real::zero(),
                            crate::STRICT_PREDICATES,
                        ))?,
                        None => curve_path_orientation(&path)?,
                    };
                    let positive = order == std::cmp::Ordering::Greater;
                    let is_outer = positive == (first_face.orientation == Orientation::Forward);
                    if is_outer {
                        if outer.replace(path).is_some() {
                            return Ok(None);
                        }
                        profile_area = profile_area.zip(area).map(|(sum, area)| sum + area.abs());
                    } else {
                        holes.push(path);
                        profile_area = profile_area.zip(area).map(|(sum, area)| sum - area.abs());
                    }
                }
                let Some(outer) = outer else {
                    return Ok(None);
                };
                // With no integral, prove positive material from exact nested
                // simple loops. This also covers boundaries assembled from a
                // group of coplanar cap faces, whose holes need a group proof.
                if profile_area.is_none() && !prism_paths_have_positive_material(&outer, &holes)? {
                    return Ok(None);
                }
                let oriented = match &profile_area {
                    Some(area) => area * &determinant,
                    None => determinant,
                };
                let order = decided_model_order(compare_reals(
                    &oriented,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))?;
                let orientation = match first_face.orientation {
                    Orientation::Forward => order.reverse(),
                    Orientation::Reversed => order,
                };
                return Ok(Some((
                    CertifiedPrismShell {
                        profile: CertifiedPrismProfile::Paths {
                            outer,
                            holes,
                            area: profile_area,
                        },
                        origin,
                        u,
                        v,
                        extrusion,
                        parameter_min: Real::zero(),
                        parameter_max: Real::one(),
                    },
                    orientation,
                )));
            }
        }
        Ok(None)
    }

    /// A curved cap edge contributes its face-local pcurve, which is a
    /// coordinate curve in the cap frame only when its face's chart is that
    /// frame. A tensor patch recognized as a plane need not be parameterized
    /// by the frame (a straight degree-elevated sweep side is not), so such a
    /// cap pair is declined rather than mixing coordinate systems.
    fn curved_cap_pcurves_use_frame(
        &self,
        boundary: &[Vec<EdgeUseId>],
        frame_surface: SurfaceId,
        frame: &Surface,
    ) -> Result<bool, BuildError> {
        // A native plane is its own canonical frame, so the common cap needs
        // no comparison and no allocation.
        let native_frame = self.surface_ref(frame_surface)?.kind() == SurfaceKind::Plane;
        let mut checked = Vec::new();
        for edge_use_id in boundary.iter().flatten() {
            let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
            if self.curve_ref(edge.curve)?.kind() == Curve3Kind::Line {
                continue;
            }
            let face = self.wire_face[self.edge_use_wire[edge_use_id.index()]
                .expect("cap boundary uses belong to wires")
                .index()]
            .expect("cap boundary wires belong to faces");
            let surface = self.face_ref(face)?.surface;
            if (native_frame && surface == frame_surface) || checked.contains(&surface) {
                continue;
            }
            checked.push(surface);
            let SurfaceExactData::Plane { origin, u, v } = frame.exact_data() else {
                unreachable!("canonical prism support is a plane");
            };
            let chart_is_frame = match self.surface_ref(surface)?.exact_data() {
                SurfaceExactData::Plane {
                    origin: chart_origin,
                    u: chart_u,
                    v: chart_v,
                } => {
                    points_equal(&chart_origin, &origin)?
                        && vectors_equal(&chart_u, &u)?
                        && vectors_equal(&chart_v, &v)?
                }
                // `S(s, t) = origin + s u + t v` holds exactly when the
                // weights are constant and every control lies on that grid.
                SurfaceExactData::RationalBezier {
                    control_points,
                    weights,
                } => {
                    tensor_weights_are_constant(&weights)?
                        && affine_grid_matches_frame(&control_points, &origin, &u, &v)?
                }
                _ => false,
            };
            if !chart_is_frame {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Exact signed profile-coordinate area of an admitted planar cap with
    /// native conics. Multiple closed wires contribute their oriented integrals.
    pub(crate) fn conic_cap_area(&self, face: FaceId) -> Result<Option<Real>, BuildError> {
        let face = self.face_ref(face)?;
        let surface = self.surface_ref(face.surface)?;
        let Some((u, v)) = surface.plane_directions() else {
            return Ok(None);
        };
        let mut uses = Vec::new();
        for wire in face.boundary_wires() {
            uses.extend_from_slice(&self.wire_ref(*wire)?.edge_uses);
        }
        self.conic_cap_signed_area(&uses, u, v)
    }

    /// Integrates retained spatial conic intervals in the already certified
    /// cap frame. Replaying split pcurves need not reconstruct those angles.
    fn conic_cap_signed_area(
        &self,
        uses: &[EdgeUseId],
        u: &Vector3,
        v: &Vector3,
    ) -> Result<Option<Real>, BuildError> {
        let mut conic_origin = None;
        for use_id in uses {
            let edge_use = self.edge_use_ref(*use_id)?;
            let edge = self.edge_ref(edge_use.edge)?;
            if let Some(center) = self.curve_ref(edge.curve)?.analytic_area_origin() {
                conic_origin = Some(center);
                break;
            }
        }
        let Some(conic_origin) = conic_origin else {
            return Ok(None);
        };
        // As in planar face measurement, centering on a retained conic avoids
        // translated endpoint terms which would have to cancel after splitting.
        let normal = u.cross(v);
        let mut doubled_area = Real::zero();
        for use_id in uses {
            let edge_use = self.edge_use_ref(*use_id)?;
            let edge = self.edge_ref(edge_use.edge)?;
            let (Some(start), Some(end)) =
                (edge.domain.start().try_real(), edge.domain.end().try_real())
            else {
                return Ok(None);
            };
            let Some(integral) =
                self.curve_ref(edge.curve)?
                    .analytic_area_vector(start, end, conic_origin)?
            else {
                return Ok(None);
            };
            let contribution = integral.dot(&normal);
            match edge_use.direction {
                Direction::Forward => doubled_area += contribution,
                Direction::Reversed => doubled_area -= contribution,
            }
        }
        (doubled_area / (Real::from(2) * normal.norm_squared()))
            .map(Some)
            .map_err(GeometryError::Numeric)
            .map_err(Into::into)
    }
}

/// A simple outer loop minus strictly contained, disjoint simple holes has
/// positive area independently of whether its Green integral is represented.
fn prism_paths_have_positive_material(
    outer: &CurvePath2,
    holes: &[CurvePath2],
) -> Result<bool, BuildError> {
    for (index, hole) in holes.iter().enumerate() {
        if !outer
            .intersect_path(hole)
            .map_err(GeometryError::from)?
            .is_disjoint()
            || !classification_is_inside(
                crate::error::classified(outer.classify_point(&hole.start()))
                    .map_err(GeometryError::from)?,
            )?
        {
            return Ok(false);
        }
        for other in &holes[..index] {
            if !other
                .intersect_path(hole)
                .map_err(GeometryError::from)?
                .is_disjoint()
                || classification_is_inside(
                    crate::error::classified(other.classify_point(&hole.start()))
                        .map_err(GeometryError::from)?,
                )?
                || classification_is_inside(
                    crate::error::classified(hole.classify_point(&other.start()))
                        .map_err(GeometryError::from)?,
                )?
            {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

fn affine_grid_matches_frame(
    control_points: &[Vec<Point3>],
    origin: &Point3,
    u: &Vector3,
    v: &Vector3,
) -> Result<bool, BuildError> {
    let column_span = Real::from(
        u128::try_from(control_points[0].len() - 1).expect("usize is representable as u128"),
    );
    let row_span = Real::from(
        u128::try_from(control_points.len() - 1).expect("usize is representable as u128"),
    );
    for (row_index, row) in control_points.iter().enumerate() {
        let t = (Real::from(u128::try_from(row_index).expect("usize is representable as u128"))
            / &row_span)
            .map_err(GeometryError::Numeric)?;
        for (column_index, point) in row.iter().enumerate() {
            let s =
                (Real::from(u128::try_from(column_index).expect("usize is representable as u128"))
                    / &column_span)
                    .map_err(GeometryError::Numeric)?;
            if !points_equal(point, &(origin.clone() + u.clone() * &s + v.clone() * &t))? {
                return Ok(false);
            }
        }
    }
    Ok(true)
}
