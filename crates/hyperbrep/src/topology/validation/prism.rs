//! Validated construction: validation prism.

use crate::topology::*;

impl ArenaModelBuilder {
    pub(in crate::topology) fn certify_extrusion_prism_shell(
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
                SurfaceKind::Extrusion => {}
                _ => return Ok(false),
            }
            for wire_id in face.boundary_wires() {
                for edge_use_id in &self.wire_ref(*wire_id)?.edge_uses {
                    let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
                    if !matches!(
                        self.curve_ref(edge.curve)?.kind(),
                        Curve3Kind::Line
                            | Curve3Kind::CircleArc
                            | Curve3Kind::EllipseArc
                            | Curve3Kind::RationalBezier
                            | Curve3Kind::Nurbs
                    ) {
                        return Ok(false);
                    }
                }
            }
        }
        if cap_surfaces.len() != 2 {
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
        if !self.certify_simple_prism_shell(shell)?
            && !self.certify_internally_partitioned_prism_shell(shell)?
            && !self.certify_extrusion_prism_shell(shell)?
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
                let second_boundary = self
                    .cap_boundary_use_loops(faces, second_group)?
                    .expect("certified cap group has a manifold boundary");
                let first_vertices = self.boundary_loop_vertex_set(&first_boundary)?;
                let second_vertices = self.boundary_loop_vertex_set(&second_boundary)?;
                let cross_edge = self
                    .shell_edge_set(faces)?
                    .into_iter()
                    .find(|edge_id| {
                        let edge = &self.edges[edge_id.index()];
                        (first_vertices.contains(&edge.start)
                            && second_vertices.contains(&edge.end))
                            || (first_vertices.contains(&edge.end)
                                && second_vertices.contains(&edge.start))
                    })
                    .expect("certified cap pair has cross edges");
                let edge = self.edge_ref(cross_edge)?;
                let (first_vertex, second_vertex) = if first_vertices.contains(&edge.start) {
                    (edge.start, edge.end)
                } else {
                    (edge.end, edge.start)
                };
                let extrusion = self.vertex_ref(second_vertex)?.represented_point()?
                    - self.vertex_ref(first_vertex)?.represented_point()?;
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
                    let area = curve_path_signed_area(&path)?;
                    let positive = decided_model_order(compare_reals(
                        &area,
                        &Real::zero(),
                        crate::STRICT_PREDICATES,
                    ))? == std::cmp::Ordering::Greater;
                    let is_outer = positive == (first_face.orientation == Orientation::Forward);
                    if is_outer {
                        if outer.replace(path).is_some() {
                            return Ok(None);
                        }
                    } else {
                        holes.push(path);
                    }
                }
                let Some(outer) = outer else {
                    return Ok(None);
                };
                return Ok(Some(CertifiedPrismShell {
                    profile: CertifiedPrismProfile::Paths { outer, holes },
                    origin,
                    u,
                    v,
                    extrusion,
                    parameter_min: Real::zero(),
                    parameter_max: Real::one(),
                }));
            }
        }
        Ok(None)
    }
}
