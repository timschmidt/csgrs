//! Validated construction: validation sweep.

use crate::topology::*;

impl ArenaModelBuilder {
    pub(in crate::topology) fn certified_curve_sweep_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedCurveSweepShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        if faces.len() < 5 {
            return Ok(None);
        }
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            let kind = self.surface_ref(face.surface)?.kind();
            if !matches!(kind, SurfaceKind::Plane | SurfaceKind::RationalBezier)
                || (kind == SurfaceKind::RationalBezier && !face.inner().is_empty())
            {
                return Ok(None);
            }
        }
        if let Some(certificate) = self.certified_grouped_curve_sweep_shell(shell)? {
            return Ok(Some(certificate));
        }
        for lower in faces {
            let lower_face = self.face_ref(*lower)?;
            if lower_face.orientation != Orientation::Reversed
                || self.surface_ref(lower_face.surface)?.kind() != SurfaceKind::Plane
            {
                continue;
            }
            for upper in faces {
                let upper_face = self.face_ref(*upper)?;
                if upper_face.orientation != Orientation::Forward
                    || self.surface_ref(upper_face.surface)?.kind() != SurfaceKind::Plane
                {
                    continue;
                }
                if let Some(certificate) =
                    self.certified_curve_sweep_cap_pair(shell, *lower, *upper)?
                {
                    return Ok(Some(certificate));
                }
            }
        }
        Ok(None)
    }

    pub(in crate::topology) fn certified_grouped_curve_sweep_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedCurveSweepShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let planar_groups = self.planar_face_groups(faces)?;
        for lower_group in &planar_groups {
            if lower_group
                .iter()
                .any(|index| self.faces[faces[*index].index()].orientation != Orientation::Reversed)
            {
                continue;
            }
            for upper_group in &planar_groups {
                if lower_group == upper_group
                    || upper_group.iter().any(|index| {
                        self.faces[faces[*index].index()].orientation != Orientation::Forward
                    })
                {
                    continue;
                }
                if let Some(certificate) =
                    self.certified_grouped_curve_sweep_cap_pair(faces, lower_group, upper_group)?
                {
                    return Ok(Some(certificate));
                }
            }
        }
        Ok(None)
    }

    pub(in crate::topology) fn certified_grouped_curve_sweep_cap_pair(
        &self,
        faces: &[FaceId],
        lower_group: &[usize],
        upper_group: &[usize],
    ) -> Result<Option<CertifiedCurveSweepShell>, BuildError> {
        let Some(lower_boundaries) = self.cap_boundary_use_loops(faces, lower_group)? else {
            return Ok(None);
        };
        let Some(upper_boundaries) = self.cap_boundary_use_loops(faces, upper_group)? else {
            return Ok(None);
        };
        if lower_boundaries.is_empty()
            || lower_boundaries.len() != upper_boundaries.len()
            || lower_boundaries.iter().any(|boundary| boundary.len() < 3)
            || upper_boundaries.iter().any(|boundary| boundary.len() < 3)
        {
            return Ok(None);
        }

        let lower_face = self.face_ref(faces[lower_group[0]])?;
        let upper_face = self.face_ref(faces[upper_group[0]])?;
        let SurfaceExactData::Plane {
            u: lower_u,
            v: lower_v,
            ..
        } = self.surface_ref(lower_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        let SurfaceExactData::Plane {
            u: upper_u,
            v: upper_v,
            ..
        } = self.surface_ref(upper_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        if decided_model_order(compare_reals(
            &lower_u
                .cross(&lower_v)
                .cross(&upper_u.cross(&upper_v))
                .norm_squared(),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }

        let lower_vertices = self.boundary_loop_vertex_set(&lower_boundaries)?;
        let upper_vertices = self.boundary_loop_vertex_set(&upper_boundaries)?;
        if !lower_vertices.is_disjoint(&upper_vertices) {
            return Ok(None);
        }
        let face_set = faces.iter().copied().collect::<HashSet<_>>();
        let mut correspondence = HashMap::new();
        let mut connector_by_lower = HashMap::new();
        let mut path_parameter_bounds = None;
        for chain in
            self.complete_tensor_path_chains(faces, &face_set, &lower_vertices, &upper_vertices)?
        {
            if let Some((start, end)) = &path_parameter_bounds {
                if !real_values_equal(start, &chain.parameter_start)?
                    || !real_values_equal(end, &chain.parameter_end)?
                {
                    return Ok(None);
                }
            } else {
                path_parameter_bounds =
                    Some((chain.parameter_start.clone(), chain.parameter_end.clone()));
            }
            if correspondence.insert(chain.lower, chain.upper).is_some()
                || connector_by_lower
                    .insert(chain.lower, chain.curve)
                    .is_some()
            {
                return Ok(None);
            }
        }
        let Some((path_parameter_start, path_parameter_end)) = path_parameter_bounds else {
            return Ok(None);
        };
        let count = correspondence.len();
        if count < 3
            || connector_by_lower.len() != count
            || correspondence
                .values()
                .copied()
                .collect::<HashSet<_>>()
                .len()
                != count
        {
            return Ok(None);
        }

        let reference = *correspondence
            .keys()
            .min()
            .expect("grouped curve sweep has connectors");
        let reference_point = self.vertex_ref(reference)?.point().clone();
        let path = connector_by_lower[&reference].clone();
        if !real_values_equal(path.domain().start(), &Real::zero())?
            || !real_values_equal(path.domain().end(), &Real::one())?
            || !certified_affine_sweep_progress(&path, &lower_u.cross(&lower_v))?
        {
            return Ok(None);
        }
        let Curve3ExactData::RationalBezier {
            control_points: path_controls,
            weights: path_weights,
        } = path.exact_data()
        else {
            unreachable!("connector kind was checked");
        };
        let mut lower_coordinates = HashMap::with_capacity(count);
        let mut connector_controls = HashMap::with_capacity(count);
        let mut sorted_lower = correspondence.keys().copied().collect::<Vec<_>>();
        sorted_lower.sort();
        for lower in &sorted_lower {
            lower_coordinates.insert(
                *lower,
                project_point_to_plane_frame(
                    self.vertex_ref(*lower)?.point(),
                    &reference_point,
                    &lower_u,
                    &lower_v,
                )?,
            );
            let Curve3ExactData::RationalBezier {
                control_points,
                weights,
            } = connector_by_lower[lower].exact_data()
            else {
                unreachable!("connector kind was checked");
            };
            if control_points.len() != path_controls.len()
                || !rational_bezier_weights_proportional(&weights, &path_weights)?
            {
                return Ok(None);
            }
            connector_controls.insert(*lower, control_points);
        }

        let mut basis = None;
        for first in &sorted_lower {
            let first_coordinate = &lower_coordinates[first];
            for second in &sorted_lower {
                let second_coordinate = &lower_coordinates[second];
                let determinant = first_coordinate.x() * second_coordinate.y()
                    - first_coordinate.y() * second_coordinate.x();
                if decided_model_order(compare_reals(
                    &determinant,
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))? != std::cmp::Ordering::Equal
                {
                    basis = Some((*first, *second, determinant));
                    break;
                }
            }
            if basis.is_some() {
                break;
            }
        }
        let Some((first_basis, second_basis, basis_determinant)) = basis else {
            return Ok(None);
        };
        let first_coordinate = &lower_coordinates[&first_basis];
        let second_coordinate = &lower_coordinates[&second_basis];
        let first_controls = &connector_controls[&first_basis];
        let second_controls = &connector_controls[&second_basis];
        let mut u_controls = Vec::with_capacity(path_controls.len());
        let mut v_controls = Vec::with_capacity(path_controls.len());
        for index in 0..path_controls.len() {
            let first_difference = &first_controls[index] - &path_controls[index];
            let second_difference = &second_controls[index] - &path_controls[index];
            u_controls.push(divide_vector_by_real(
                first_difference.clone() * second_coordinate.y()
                    - second_difference.clone() * first_coordinate.y(),
                &basis_determinant,
            )?);
            v_controls.push(divide_vector_by_real(
                second_difference * first_coordinate.x() - first_difference * second_coordinate.x(),
                &basis_determinant,
            )?);
        }
        let Some(area_scale_integral) = certified_sweep_frame_area_integral(
            &u_controls,
            &v_controls,
            &path_weights,
            &lower_u.cross(&lower_v),
        )?
        else {
            return Ok(None);
        };
        for lower in &sorted_lower {
            let coordinate = &lower_coordinates[lower];
            for (index, actual) in connector_controls[lower].iter().enumerate() {
                let expected = path_controls[index].clone()
                    + u_controls[index].clone() * coordinate.x()
                    + v_controls[index].clone() * coordinate.y();
                if !points_equal(actual, &expected)? {
                    return Ok(None);
                }
            }
        }

        let ordered_boundaries = lower_boundaries
            .iter()
            .map(|boundary| {
                let mut ordered = boundary
                    .iter()
                    .map(|edge_use| self.directed_vertices(*edge_use).map(|vertices| vertices.0))
                    .collect::<Result<Vec<_>, _>>()?;
                ordered.reverse();
                Ok(ordered)
            })
            .collect::<Result<Vec<_>, BuildError>>()?;
        let ordered_corner_boundaries = ordered_boundaries
            .iter()
            .map(|boundary| {
                boundary
                    .iter()
                    .copied()
                    .filter(|vertex| correspondence.contains_key(vertex))
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        let ordered_corners = ordered_corner_boundaries
            .iter()
            .flatten()
            .copied()
            .collect::<Vec<_>>();
        if ordered_corner_boundaries
            .iter()
            .any(|boundary| boundary.len() < 3)
            || ordered_corners.len() != count
            || ordered_corners
                .iter()
                .copied()
                .collect::<HashSet<_>>()
                .len()
                != count
        {
            return Ok(None);
        }

        let cap_indices = lower_group
            .iter()
            .chain(upper_group)
            .copied()
            .collect::<HashSet<_>>();
        let mut side_groups = HashMap::<SurfaceId, Vec<usize>>::new();
        for (index, face_id) in faces.iter().enumerate() {
            if cap_indices.contains(&index) {
                continue;
            }
            let face = self.face_ref(*face_id)?;
            if face.orientation != Orientation::Forward
                || self.surface_ref(face.surface)?.kind() != SurfaceKind::RationalBezier
            {
                return Ok(None);
            }
            side_groups.entry(face.surface).or_default().push(index);
        }
        if side_groups.len() != count {
            return Ok(None);
        }
        let mut represented_surfaces = HashSet::with_capacity(count);
        for ordered_corners in &ordered_corner_boundaries {
            for index in 0..ordered_corners.len() {
                let start = ordered_corners[index];
                let end = ordered_corners[(index + 1) % ordered_corners.len()];
                let start_coordinate = &lower_coordinates[&start];
                let end_coordinate = &lower_coordinates[&end];
                let mut matching = None;
                for (surface_id, group) in &side_groups {
                    let restricted_surface = self.restricted_curve_sweep_side_surface(
                        *surface_id,
                        &path_parameter_start,
                        &path_parameter_end,
                    )?;
                    let SurfaceExactData::RationalBezier {
                        control_points,
                        weights,
                    } = restricted_surface.exact_data()
                    else {
                        unreachable!("side surface kind was checked");
                    };
                    if framed_rational_bezier_surface_equal(
                        &control_points,
                        &weights,
                        RationalBezierSweepControlView {
                            path_points: &path_controls,
                            path_weights: &path_weights,
                            u_controls: &u_controls,
                            v_controls: &v_controls,
                        },
                        start_coordinate,
                        end_coordinate,
                    )? && matching.replace((*surface_id, group)).is_some()
                    {
                        return Ok(None);
                    }
                }
                let Some((surface_id, group)) = matching else {
                    return Ok(None);
                };
                if !represented_surfaces.insert(surface_id)
                    || !self.certifies_tensor_face_group(
                        faces,
                        group,
                        &Real::zero(),
                        &Real::one(),
                        &path_parameter_start,
                        &path_parameter_end,
                    )?
                {
                    return Ok(None);
                }
            }
        }
        if represented_surfaces.len() != side_groups.len() {
            return Ok(None);
        }

        let contours = ordered_boundaries
            .iter()
            .map(|boundary| {
                let points = boundary
                    .iter()
                    .map(|vertex| {
                        project_point_to_plane_frame(
                            self.vertex_ref(*vertex)?.point(),
                            &reference_point,
                            &lower_u,
                            &lower_v,
                        )
                    })
                    .collect::<Result<Vec<_>, BuildError>>()?;
                Ok(Contour2::try_new(
                    (0..points.len())
                        .map(|index| {
                            LineSeg2::try_new(
                                points[index].clone(),
                                points[(index + 1) % points.len()].clone(),
                            )
                            .map(Segment2::Line)
                            .map_err(GeometryError::from)
                        })
                        .collect::<Result<Vec<_>, _>>()?,
                )
                .map_err(GeometryError::from)?)
            })
            .collect::<Result<Vec<_>, BuildError>>()?;
        let mut profile = None;
        let mut holes = Vec::new();
        for contour in contours {
            let area = contour.signed_area().map_err(GeometryError::from)?.ok_or(
                BuildError::DegenerateShellVolume(
                    self.face_shell[faces[0].index()].expect("shell faces are assigned"),
                ),
            )?;
            match decided_model_order(compare_reals(
                &area,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? {
                std::cmp::Ordering::Greater => {
                    if profile.replace(contour).is_some() {
                        return Ok(None);
                    }
                }
                std::cmp::Ordering::Less => holes.push(contour),
                std::cmp::Ordering::Equal => return Ok(None),
            }
        }
        let Some(profile) = profile else {
            return Ok(None);
        };
        Ok(Some(CertifiedCurveSweepShell {
            profile,
            holes,
            path,
            u_path: vector_control_curve(u_controls, &path_weights)?,
            v_path: vector_control_curve(v_controls, &path_weights)?,
            area_scale_integral,
        }))
    }

    pub(in crate::topology) fn complete_tensor_path_chains(
        &self,
        faces: &[FaceId],
        shell_faces: &HashSet<FaceId>,
        lower_vertices: &HashSet<VertexId>,
        upper_vertices: &HashSet<VertexId>,
    ) -> Result<Vec<TensorPathChain>, BuildError> {
        let mut groups = HashMap::<Curve3Id, Vec<(EdgeId, Real, Real)>>::new();
        for edge_id in self.shell_edge_set(faces)? {
            let edge = self.edge_ref(edge_id)?;
            if self.curve_ref(edge.curve)?.kind() != Curve3Kind::RationalBezier {
                continue;
            }
            let Some((start, end)) = self.tensor_path_boundary_interval(edge_id, shell_faces)?
            else {
                continue;
            };
            groups
                .entry(edge.curve)
                .or_default()
                .push((edge_id, start, end));
        }

        let mut chains = Vec::new();
        for (curve_id, segments) in groups {
            let curve = self.curve_ref(curve_id)?;
            let edge_intervals = segments
                .iter()
                .map(|(edge, _, _)| {
                    let domain = self.edge_ref(*edge)?.domain.clone();
                    Ok((domain.start().clone(), domain.end().clone()))
                })
                .collect::<Result<Vec<_>, BuildError>>()?;
            if !self.exact_intervals_tile(
                &edge_intervals,
                curve.domain().start(),
                curve.domain().end(),
            )? {
                continue;
            }
            let parameter_start = segments
                .iter()
                .map(|(_, start, _)| start.clone())
                .try_fold(None, |minimum, candidate| {
                    Ok::<_, BuildError>(Some(match minimum {
                        Some(minimum) => minimum_real(&minimum, &candidate)?,
                        None => candidate,
                    }))
                })?
                .expect("one or more path segments");
            let parameter_end = segments
                .iter()
                .map(|(_, _, end)| end.clone())
                .try_fold(None, |maximum, candidate| {
                    Ok::<_, BuildError>(Some(match maximum {
                        Some(maximum) => maximum_real(&maximum, &candidate)?,
                        None => candidate,
                    }))
                })?
                .expect("one or more path segments");
            let parameter_intervals = segments
                .iter()
                .map(|(_, start, end)| (start.clone(), end.clone()))
                .collect::<Vec<_>>();
            if !self.exact_intervals_tile(&parameter_intervals, &parameter_start, &parameter_end)? {
                continue;
            }

            let mut adjacency = HashMap::<VertexId, Vec<VertexId>>::new();
            for (edge_id, _, _) in &segments {
                let edge = self.edge_ref(*edge_id)?;
                adjacency.entry(edge.start).or_default().push(edge.end);
                adjacency.entry(edge.end).or_default().push(edge.start);
            }
            let lowers = adjacency
                .keys()
                .copied()
                .filter(|vertex| lower_vertices.contains(vertex))
                .collect::<Vec<_>>();
            let uppers = adjacency
                .keys()
                .copied()
                .filter(|vertex| upper_vertices.contains(vertex))
                .collect::<Vec<_>>();
            if lowers.len() != 1
                || uppers.len() != 1
                || adjacency.iter().any(|(vertex, adjacent)| {
                    let expected = if *vertex == lowers[0] || *vertex == uppers[0] {
                        1
                    } else {
                        2
                    };
                    adjacent.len() != expected
                })
            {
                continue;
            }
            let mut visited = HashSet::new();
            let mut pending = vec![lowers[0]];
            while let Some(vertex) = pending.pop() {
                if visited.insert(vertex) {
                    pending.extend(adjacency[&vertex].iter().copied());
                }
            }
            if visited.len() != adjacency.len() {
                continue;
            }
            let lower_point = self.vertex_ref(lowers[0])?.point();
            let upper_point = self.vertex_ref(uppers[0])?.point();
            let curve_start = curve.point_at(curve.domain().start())?;
            let curve_end = curve.point_at(curve.domain().end())?;
            let curve = if points_equal(&curve_start, lower_point)?
                && points_equal(&curve_end, upper_point)?
            {
                curve.clone()
            } else if points_equal(&curve_start, upper_point)?
                && points_equal(&curve_end, lower_point)?
            {
                curve.reversed()?
            } else {
                continue;
            };
            chains.push(TensorPathChain {
                lower: lowers[0],
                upper: uppers[0],
                curve,
                parameter_start,
                parameter_end,
            });
        }
        Ok(chains)
    }

    pub(in crate::topology) fn tensor_path_boundary_interval(
        &self,
        edge: EdgeId,
        shell_faces: &HashSet<FaceId>,
    ) -> Result<Option<(Real, Real)>, BuildError> {
        let mut interval = None;
        for edge_use in &self.edge_uses_by_edge[edge.index()] {
            let Some(wire) = self.edge_use_wire[edge_use.index()] else {
                continue;
            };
            let Some(face_id) = self.wire_face[wire.index()] else {
                continue;
            };
            if !shell_faces.contains(&face_id)
                || self.surface_ref(self.face_ref(face_id)?.surface)?.kind()
                    != SurfaceKind::RationalBezier
            {
                continue;
            }
            let edge_use = self.edge_use_ref(*edge_use)?;
            let Some(line) = self.pcurve_ref(edge_use.pcurve)?.line_segment() else {
                continue;
            };
            let on_path_boundary = real_values_equal(line.start().x(), line.end().x())?
                && (real_values_equal(line.start().x(), &Real::zero())?
                    || real_values_equal(line.start().x(), &Real::one())?);
            if !on_path_boundary {
                continue;
            }
            let order = decided_model_order(compare_reals(
                line.start().y(),
                line.end().y(),
                crate::STRICT_PREDICATES,
            ))?;
            if order == std::cmp::Ordering::Equal {
                continue;
            }
            let candidate = if order == std::cmp::Ordering::Less {
                (line.start().y().clone(), line.end().y().clone())
            } else {
                (line.end().y().clone(), line.start().y().clone())
            };
            if let Some((start, end)) = &interval {
                if !real_values_equal(start, &candidate.0)?
                    || !real_values_equal(end, &candidate.1)?
                {
                    return Ok(None);
                }
            } else {
                interval = Some(candidate);
            }
        }
        Ok(interval)
    }

    pub(in crate::topology) fn restricted_curve_sweep_side_surface(
        &self,
        surface: SurfaceId,
        start: &Real,
        end: &Real,
    ) -> Result<Surface, BuildError> {
        let source = self.surface_ref(surface)?.clone();
        if real_values_equal(start, &Real::zero())? && real_values_equal(end, &Real::one())? {
            return Ok(source);
        }
        if decided_model_order(compare_reals(
            start,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Less
            || decided_model_order(compare_reals(end, &Real::one(), crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Greater
            || decided_model_order(compare_reals(start, end, crate::STRICT_PREDICATES))?
                != std::cmp::Ordering::Less
        {
            return Err(GeometryError::InvalidParameterDomain.into());
        }
        let selected =
            if decided_model_order(compare_reals(end, &Real::one(), crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Less
            {
                source.split_v_at(end)?.0
            } else {
                source
            };
        if decided_model_order(compare_reals(
            start,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? == std::cmp::Ordering::Equal
        {
            return Ok(selected);
        }
        let relative = (start / end).map_err(GeometryError::Numeric)?;
        Ok(selected.split_v_at(&relative)?.1)
    }

    pub(in crate::topology) fn certifies_tensor_face_group(
        &self,
        faces: &[FaceId],
        group: &[usize],
        u_start: &Real,
        u_end: &Real,
        v_start: &Real,
        v_end: &Real,
    ) -> Result<bool, BuildError> {
        let Some(boundaries) = self.cap_boundary_use_loops(faces, group)? else {
            return Ok(false);
        };
        if boundaries.len() != 1 {
            return Ok(false);
        }
        let mut sides = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
        for edge_use in &boundaries[0] {
            let edge_use = self.edge_use_ref(*edge_use)?;
            let Some(line) = self.pcurve_ref(edge_use.pcurve)?.line_segment() else {
                return Ok(false);
            };
            let x_constant = real_values_equal(line.start().x(), line.end().x())?;
            let y_constant = real_values_equal(line.start().y(), line.end().y())?;
            let (side, first, second) =
                if x_constant && real_values_equal(line.start().x(), u_start)? {
                    (0, line.start().y(), line.end().y())
                } else if x_constant && real_values_equal(line.start().x(), u_end)? {
                    (1, line.start().y(), line.end().y())
                } else if y_constant && real_values_equal(line.start().y(), v_start)? {
                    (2, line.start().x(), line.end().x())
                } else if y_constant && real_values_equal(line.start().y(), v_end)? {
                    (3, line.start().x(), line.end().x())
                } else {
                    return Ok(false);
                };
            let order =
                decided_model_order(compare_reals(first, second, crate::STRICT_PREDICATES))?;
            if order == std::cmp::Ordering::Equal {
                return Ok(false);
            }
            sides[side].push(if order == std::cmp::Ordering::Less {
                (first.clone(), second.clone())
            } else {
                (second.clone(), first.clone())
            });
        }
        for (intervals, start, end) in [
            (&sides[0], v_start, v_end),
            (&sides[1], v_start, v_end),
            (&sides[2], u_start, u_end),
            (&sides[3], u_start, u_end),
        ] {
            if !self.exact_intervals_tile(intervals, start, end)? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    pub(in crate::topology) fn axis_aligned_lines_tile_rectangle(
        &self,
        lines: &[LineSeg2],
        u_start: &Real,
        u_end: &Real,
        v_start: &Real,
        v_end: &Real,
    ) -> Result<bool, BuildError> {
        let mut sides = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
        for line in lines {
            let x_constant = real_values_equal(line.start().x(), line.end().x())?;
            let y_constant = real_values_equal(line.start().y(), line.end().y())?;
            let (side, first, second) =
                if x_constant && real_values_equal(line.start().x(), u_start)? {
                    (0, line.start().y(), line.end().y())
                } else if x_constant && real_values_equal(line.start().x(), u_end)? {
                    (1, line.start().y(), line.end().y())
                } else if y_constant && real_values_equal(line.start().y(), v_start)? {
                    (2, line.start().x(), line.end().x())
                } else if y_constant && real_values_equal(line.start().y(), v_end)? {
                    (3, line.start().x(), line.end().x())
                } else {
                    return Ok(false);
                };
            let order =
                decided_model_order(compare_reals(first, second, crate::STRICT_PREDICATES))?;
            if order == std::cmp::Ordering::Equal {
                return Ok(false);
            }
            sides[side].push(if order == std::cmp::Ordering::Less {
                (first.clone(), second.clone())
            } else {
                (second.clone(), first.clone())
            });
        }
        for (intervals, start, end) in [
            (&sides[0], v_start, v_end),
            (&sides[1], v_start, v_end),
            (&sides[2], u_start, u_end),
            (&sides[3], u_start, u_end),
        ] {
            if !self.exact_intervals_tile(intervals, start, end)? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    pub(in crate::topology) fn exact_intervals_tile(
        &self,
        intervals: &[(Real, Real)],
        start: &Real,
        end: &Real,
    ) -> Result<bool, BuildError> {
        if intervals.is_empty() {
            return Ok(false);
        }
        let mut used = vec![false; intervals.len()];
        let mut current = start.clone();
        for _ in 0..intervals.len() {
            let mut next = None;
            for (index, (start, end)) in intervals.iter().enumerate() {
                if used[index] || !real_values_equal(start, &current)? {
                    continue;
                }
                if next.replace((index, end)).is_some() {
                    return Ok(false);
                }
            }
            let Some((index, end)) = next else {
                return Ok(false);
            };
            used[index] = true;
            current = end.clone();
        }
        real_values_equal(&current, end)
    }

    pub(in crate::topology) fn certified_curve_sweep_cap_pair(
        &self,
        shell: ShellId,
        lower_face_id: FaceId,
        upper_face_id: FaceId,
    ) -> Result<Option<CertifiedCurveSweepShell>, BuildError> {
        if lower_face_id == upper_face_id {
            return Ok(None);
        }
        let lower_face = self.face_ref(lower_face_id)?;
        let upper_face = self.face_ref(upper_face_id)?;
        let (Some(lower_wire_id), Some(upper_wire_id)) = (lower_face.outer(), upper_face.outer())
        else {
            return Ok(None);
        };
        let lower_wire = self.wire_ref(lower_wire_id)?;
        let upper_wire = self.wire_ref(upper_wire_id)?;
        let count = lower_wire.edge_uses.len();
        if count < 3 || upper_wire.edge_uses.len() != count {
            return Ok(None);
        }
        let SurfaceExactData::Plane {
            u: lower_u,
            v: lower_v,
            ..
        } = self.surface_ref(lower_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        let SurfaceExactData::Plane {
            u: upper_u,
            v: upper_v,
            ..
        } = self.surface_ref(upper_face.surface)?.exact_data()
        else {
            return Ok(None);
        };
        if decided_model_order(compare_reals(
            &lower_u
                .cross(&lower_v)
                .cross(&upper_u.cross(&upper_v))
                .norm_squared(),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }

        let lower_vertices = self.wire_vertex_set(lower_wire_id)?;
        let upper_vertices = self.wire_vertex_set(upper_wire_id)?;
        if lower_vertices.len() != count
            || upper_vertices.len() != count
            || !lower_vertices.is_disjoint(&upper_vertices)
        {
            return Ok(None);
        }
        let lower_edges = self.wire_edge_set(lower_wire_id)?;
        let upper_edges = self.wire_edge_set(upper_wire_id)?;
        if lower_edges.iter().any(|edge_id| {
            self.curves[self.edges[edge_id.index()].curve.index()].kind() != Curve3Kind::Line
        }) || upper_edges.iter().any(|edge_id| {
            self.curves[self.edges[edge_id.index()].curve.index()].kind() != Curve3Kind::Line
        }) {
            return Ok(None);
        }
        let shell_edges = self.shell_edge_set(&self.shell_ref(shell)?.faces)?;
        let cross_edges = shell_edges
            .iter()
            .filter_map(|edge_id| {
                let edge = &self.edges[edge_id.index()];
                ((lower_vertices.contains(&edge.start) && upper_vertices.contains(&edge.end))
                    || (lower_vertices.contains(&edge.end) && upper_vertices.contains(&edge.start)))
                .then_some(*edge_id)
            })
            .collect::<HashSet<_>>();
        if cross_edges.len() != count
            || shell_edges.len() != count * 3
            || self.shell_ref(shell)?.faces.len() != count + 2
        {
            return Ok(None);
        }

        let mut correspondence = HashMap::with_capacity(count);
        let mut connector_by_lower = HashMap::with_capacity(count);
        for edge_id in &cross_edges {
            let edge = self.edge_ref(*edge_id)?;
            let (lower, upper, curve) = if lower_vertices.contains(&edge.start) {
                (edge.start, edge.end, self.curve_ref(edge.curve)?.clone())
            } else {
                (
                    edge.end,
                    edge.start,
                    self.curve_ref(edge.curve)?.reversed()?,
                )
            };
            if curve.kind() != Curve3Kind::RationalBezier
                || correspondence.insert(lower, upper).is_some()
                || connector_by_lower.insert(lower, curve).is_some()
            {
                return Ok(None);
            }
        }
        if correspondence.len() != count
            || correspondence
                .values()
                .copied()
                .collect::<HashSet<_>>()
                .len()
                != count
        {
            return Ok(None);
        }
        for lower_edge_id in &lower_edges {
            let lower_edge = self.edge_ref(*lower_edge_id)?;
            let mapped_start = correspondence[&lower_edge.start];
            let mapped_end = correspondence[&lower_edge.end];
            if !upper_edges.iter().any(|upper_edge_id| {
                let upper_edge = &self.edges[upper_edge_id.index()];
                (upper_edge.start == mapped_start && upper_edge.end == mapped_end)
                    || (upper_edge.start == mapped_end && upper_edge.end == mapped_start)
            }) {
                return Ok(None);
            }
        }

        let reference = *lower_vertices
            .iter()
            .min()
            .expect("curve sweep cap has vertices");
        let reference_point = self.vertex_ref(reference)?.point().clone();
        let path = connector_by_lower[&reference].clone();
        if !real_values_equal(path.domain().start(), &Real::zero())?
            || !real_values_equal(path.domain().end(), &Real::one())?
            || !certified_affine_sweep_progress(&path, &lower_u.cross(&lower_v))?
        {
            return Ok(None);
        }
        let Curve3ExactData::RationalBezier {
            control_points: path_controls,
            weights: path_weights,
        } = path.exact_data()
        else {
            unreachable!("connector kind was checked");
        };
        for lower in &lower_vertices {
            let offset = self.vertex_ref(*lower)?.point() - &reference_point;
            let Curve3ExactData::RationalBezier {
                control_points,
                weights,
            } = connector_by_lower[lower].exact_data()
            else {
                unreachable!("connector kind was checked");
            };
            if !translated_rational_bezier_data_equal(
                &control_points,
                &weights,
                &path_controls,
                &path_weights,
                &offset,
            )? {
                return Ok(None);
            }
        }

        let mut represented_lower_edges = HashSet::with_capacity(count);
        for face_id in &self.shell_ref(shell)?.faces {
            if *face_id == lower_face_id || *face_id == upper_face_id {
                continue;
            }
            let face = self.face_ref(*face_id)?;
            if face.orientation != Orientation::Forward
                || self.surface_ref(face.surface)?.kind() != SurfaceKind::RationalBezier
            {
                return Ok(None);
            }
            let Some(wire_id) = face.outer() else {
                return Ok(None);
            };
            let edges = self.wire_edge_set(wire_id)?;
            let lower_hits = edges
                .intersection(&lower_edges)
                .copied()
                .collect::<Vec<_>>();
            let upper_hits = edges
                .intersection(&upper_edges)
                .copied()
                .collect::<Vec<_>>();
            let cross_hits = edges
                .intersection(&cross_edges)
                .copied()
                .collect::<Vec<_>>();
            if edges.len() != 4
                || lower_hits.len() != 1
                || upper_hits.len() != 1
                || cross_hits.len() != 2
                || !represented_lower_edges.insert(lower_hits[0])
            {
                return Ok(None);
            }
            let lower_edge = self.edge_ref(lower_hits[0])?;
            let mapped_start = correspondence[&lower_edge.start];
            let mapped_end = correspondence[&lower_edge.end];
            let upper_edge = self.edge_ref(upper_hits[0])?;
            if !((upper_edge.start == mapped_start && upper_edge.end == mapped_end)
                || (upper_edge.start == mapped_end && upper_edge.end == mapped_start))
            {
                return Ok(None);
            }
            for (lower, upper) in [
                (lower_edge.start, mapped_start),
                (lower_edge.end, mapped_end),
            ] {
                if !cross_hits.iter().any(|edge_id| {
                    let edge = &self.edges[edge_id.index()];
                    (edge.start == lower && edge.end == upper)
                        || (edge.start == upper && edge.end == lower)
                }) {
                    return Ok(None);
                }
            }
            let start_offset = self.vertex_ref(lower_edge.start)?.point() - &reference_point;
            let end_offset = self.vertex_ref(lower_edge.end)?.point() - &reference_point;
            let SurfaceExactData::RationalBezier {
                control_points,
                weights,
            } = self.surface_ref(face.surface)?.exact_data()
            else {
                unreachable!("side surface kind was checked");
            };
            if !translated_rational_bezier_surface_equal(
                &control_points,
                &weights,
                &path_controls,
                &path_weights,
                &start_offset,
                &end_offset,
            )? {
                return Ok(None);
            }
        }
        if represented_lower_edges != lower_edges {
            return Ok(None);
        }

        let mut ordered = lower_wire
            .edge_uses
            .iter()
            .map(|use_id| self.directed_vertices(*use_id).map(|vertices| vertices.0))
            .collect::<Result<Vec<_>, _>>()?;
        ordered.reverse();
        let reference_position = ordered
            .iter()
            .position(|vertex| *vertex == reference)
            .expect("lower cap order contains every vertex");
        ordered.rotate_left(reference_position);
        let profile_points = ordered
            .iter()
            .map(|vertex| {
                project_point_to_plane_frame(
                    self.vertex_ref(*vertex)?.point(),
                    &reference_point,
                    &lower_u,
                    &lower_v,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let profile = Contour2::try_new(
            (0..profile_points.len())
                .map(|index| {
                    LineSeg2::try_new(
                        profile_points[index].clone(),
                        profile_points[(index + 1) % profile_points.len()].clone(),
                    )
                    .map(Segment2::Line)
                    .map_err(GeometryError::from)
                })
                .collect::<Result<Vec<_>, _>>()?,
        )
        .map_err(GeometryError::from)?;
        let area = profile
            .signed_area()
            .map_err(GeometryError::from)?
            .ok_or(BuildError::DegenerateShellVolume(shell))?;
        if decided_model_order(compare_reals(
            &area,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        Ok(Some(CertifiedCurveSweepShell {
            profile,
            holes: Vec::new(),
            path,
            u_path: constant_vector_curve(&lower_u, &path_weights)?,
            v_path: constant_vector_curve(&lower_v, &path_weights)?,
            area_scale_integral: Real::one(),
        }))
    }
}
