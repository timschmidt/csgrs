//! Validated construction: validation torus.

use crate::topology::*;

impl ArenaModelBuilder {
    pub(in crate::topology) fn certified_torus_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedTorusShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut side_faces = Vec::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::Plane => {}
                SurfaceKind::Torus
                    if face.orientation == Orientation::Forward && face.inner().is_empty() =>
                {
                    side_faces.push(*face_id);
                }
                _ => return Ok(None),
            }
        }
        if side_faces.is_empty() {
            return Ok(None);
        }
        let cap_groups = self.planar_face_groups(faces)?;
        if cap_groups.len() > 2 {
            return Ok(None);
        }

        let mut surface_id = None;
        let mut u_values = Vec::new();
        let quarter = (Real::pi() / Real::from(2)).map_err(GeometryError::Numeric)?;
        let mut v_values = (0..=4)
            .map(|index| &quarter * Real::from(index))
            .collect::<Vec<_>>();
        let mut face_coordinates = Vec::with_capacity(side_faces.len());
        let mut edge_parameterization_coordinates =
            Vec::<(Curve3, Option<Real>, Option<Real>)>::new();
        for face_id in &side_faces {
            let face = self.face_ref(*face_id)?;
            match surface_id {
                Some(expected) if expected != face.surface => return Ok(None),
                Some(_) => {}
                None => surface_id = Some(face.surface),
            }
            let Some(outer) = face.outer() else {
                return Ok(None);
            };
            let wire = self.wire_ref(outer)?;
            if wire.edge_uses.len() < 4 {
                return Ok(None);
            }
            let mut face_u = Vec::with_capacity(2);
            let mut face_v = Vec::with_capacity(2);
            let mut parameter_segments = Vec::with_capacity(wire.edge_uses.len());
            for edge_use_id in &wire.edge_uses {
                let edge_use = self.edge_use_ref(*edge_use_id)?;
                let edge = self.edge_ref(edge_use.edge)?;
                let spatial_curve = self.curve_ref(edge.curve)?;
                if spatial_curve.kind() != Curve3Kind::CircleArc {
                    return Ok(None);
                }
                let pcurve = self.pcurve_ref(edge_use.pcurve)?;
                let Some(line) = pcurve.line_segment() else {
                    return Ok(None);
                };
                let u_constant = real_values_equal(line.start().x(), line.end().x())?;
                let v_constant = real_values_equal(line.start().y(), line.end().y())?;
                if u_constant == v_constant {
                    return Ok(None);
                }
                let proposed_coordinate = if u_constant {
                    line.start().x()
                } else {
                    line.start().y()
                };
                let coordinate_index = edge_parameterization_coordinates
                    .iter()
                    .position(|(candidate, retained_u, retained_v)| {
                        let retained = if u_constant { retained_u } else { retained_v };
                        candidate.has_certified_same_parameterization(spatial_curve)
                            && retained.as_ref().is_none_or(|retained| {
                                torus_parameter_coordinates_can_share_representative(
                                    proposed_coordinate,
                                    retained,
                                )
                            })
                    })
                    .unwrap_or_else(|| {
                        edge_parameterization_coordinates.push((spatial_curve.clone(), None, None));
                        edge_parameterization_coordinates.len() - 1
                    });
                let (_, retained_u, retained_v) =
                    &mut edge_parameterization_coordinates[coordinate_index];
                let mut start = line.start().clone();
                let mut end = line.end().clone();
                if u_constant {
                    let coordinate = retained_u.get_or_insert_with(|| start.x().clone()).clone();
                    start = CurvePoint2::new(coordinate.clone(), start.y().clone());
                    end = CurvePoint2::new(coordinate.clone(), end.y().clone());
                    face_u.push(coordinate);
                } else {
                    let coordinate = retained_v.get_or_insert_with(|| start.y().clone()).clone();
                    start = CurvePoint2::new(start.x().clone(), coordinate.clone());
                    end = CurvePoint2::new(end.x().clone(), coordinate.clone());
                    face_v.push(coordinate);
                }
                parameter_segments.push(Segment2::Line(
                    LineSeg2::try_new(start, end).map_err(GeometryError::from)?,
                ));
            }
            let (u_min, u_max) = exact_real_min_max(&face_u)?;
            let (v_min, v_max) = exact_real_min_max(&face_v)?;
            let mut side_intervals: [Vec<(Real, Real)>; 4] = std::array::from_fn(|_| Vec::new());
            for segment in &parameter_segments {
                let Segment2::Line(line) = segment else {
                    unreachable!("torus parameter segments are exact lines");
                };
                let u_constant = real_values_equal(line.start().x(), line.end().x())?;
                let v_constant = real_values_equal(line.start().y(), line.end().y())?;
                let (side, first, second) = if v_constant {
                    if real_values_equal(line.start().y(), &v_min)? {
                        (0, line.start().x(), line.end().x())
                    } else if real_values_equal(line.start().y(), &v_max)? {
                        (1, line.start().x(), line.end().x())
                    } else {
                        return Ok(None);
                    }
                } else if u_constant {
                    if real_values_equal(line.start().x(), &u_min)? {
                        (2, line.start().y(), line.end().y())
                    } else if real_values_equal(line.start().x(), &u_max)? {
                        (3, line.start().y(), line.end().y())
                    } else {
                        return Ok(None);
                    }
                } else {
                    return Ok(None);
                };
                let (start, end) = exact_real_min_max(&[first.clone(), second.clone()])?;
                side_intervals[side].push((start, end));
            }
            if !exact_intervals_cover(&mut side_intervals[0], &u_min, &u_max)?
                || !exact_intervals_cover(&mut side_intervals[1], &u_min, &u_max)?
                || !exact_intervals_cover(&mut side_intervals[2], &v_min, &v_max)?
                || !exact_intervals_cover(&mut side_intervals[3], &v_min, &v_max)?
            {
                return Ok(None);
            }
            insert_sorted_real(&mut u_values, &u_min)?;
            insert_sorted_real(&mut u_values, &u_max)?;
            insert_sorted_real(&mut v_values, &v_min)?;
            insert_sorted_real(&mut v_values, &v_max)?;
            face_coordinates.push((u_min, u_max, v_min, v_max));
        }
        let zero = Real::zero();
        let tau = Real::tau();
        for (_, _, v_min, v_max) in &face_coordinates {
            if decided_model_order(compare_reals(v_min, &zero, crate::STRICT_PREDICATES))?
                == std::cmp::Ordering::Less
                || decided_model_order(compare_reals(v_max, &tau, crate::STRICT_PREDICATES))?
                    == std::cmp::Ordering::Greater
            {
                return Ok(None);
            }
        }
        let SurfaceExactData::Torus {
            center,
            x,
            y,
            axis,
            major_radius,
            minor_radius,
            ..
        } = self
            .surface_ref(surface_id.expect("torus shell has faces"))?
            .exact_data()
        else {
            unreachable!("torus kind carries torus exact data");
        };
        let mut cells = HashSet::new();
        for (u_min, u_max, v_min, v_max) in &face_coordinates {
            let u_start = exact_real_index(&u_values, u_min)?;
            let u_end = exact_real_index(&u_values, u_max)?;
            let v_start = exact_real_index(&v_values, v_min)?;
            let v_end = exact_real_index(&v_values, v_max)?;
            if u_start >= u_end || v_start >= v_end {
                return Ok(None);
            }
            for u_cell in u_start..u_end {
                for v_cell in v_start..v_end {
                    if !cells.insert((u_cell, v_cell)) {
                        return Ok(None);
                    }
                }
            }
        }
        if let [group] = cap_groups.as_slice()
            && let Some(interior_normal) = self.certified_torus_longitudinal_cap_group(
                faces,
                group,
                &center,
                &axis,
                &major_radius,
                &minor_radius,
            )?
        {
            if !real_values_equal(
                &(v_values.last().expect("torus v grid") - &v_values[0]),
                &Real::tau(),
            )? || !certified_periodic_longitudinal_half_coverage(
                &u_values,
                &v_values,
                &cells,
                &x,
                &y,
                &interior_normal,
            )? {
                return Ok(None);
            }
            return Ok(Some(CertifiedTorusShell {
                center,
                axis,
                major_radius,
                minor_radius,
                region: CertifiedTorusRegion::LongitudinalHalf { interior_normal },
            }));
        }

        if u_values.len() < 5
            || !real_values_equal(
                &(u_values.last().expect("torus u grid") - &u_values[0]),
                &Real::tau(),
            )?
        {
            return Ok(None);
        }
        let mut caps = Vec::with_capacity(cap_groups.len());
        for group in &cap_groups {
            let Some(cap) = self.certified_torus_cap_group(
                faces,
                group,
                &center,
                &axis,
                &major_radius,
                &minor_radius,
            )?
            else {
                return Ok(None);
            };
            caps.push(cap);
        }
        let (axial_min, axial_max) = match caps.len() {
            0 => (-minor_radius.clone(), minor_radius.clone()),
            1 if caps[0].1 == std::cmp::Ordering::Less => (caps[0].0.clone(), minor_radius.clone()),
            1 if caps[0].1 == std::cmp::Ordering::Greater => {
                (-minor_radius.clone(), caps[0].0.clone())
            }
            1 => return Ok(None),
            2 => {
                if decided_model_order(compare_reals(
                    &caps[0].0,
                    &caps[1].0,
                    crate::STRICT_PREDICATES,
                ))? == std::cmp::Ordering::Greater
                {
                    caps.swap(0, 1);
                }
                if decided_model_order(compare_reals(
                    &caps[0].0,
                    &caps[1].0,
                    crate::STRICT_PREDICATES,
                ))? != std::cmp::Ordering::Less
                    || caps[0].1 != std::cmp::Ordering::Less
                    || caps[1].1 != std::cmp::Ordering::Greater
                {
                    return Ok(None);
                }
                (caps[0].0.clone(), caps[1].0.clone())
            }
            _ => unreachable!("at most two torus cap groups"),
        };

        for v_cell in 0..v_values.len() - 1 {
            let first_height = &minor_radius * v_values[v_cell].clone().sin();
            let second_height = &minor_radius * v_values[v_cell + 1].clone().sin();
            let (cell_min, cell_max) = exact_real_min_max(&[first_height, second_height])?;
            let expected = decided_model_order(compare_reals(
                &cell_min,
                &axial_min,
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Less
                && decided_model_order(compare_reals(
                    &cell_max,
                    &axial_max,
                    crate::STRICT_PREDICATES,
                ))? != std::cmp::Ordering::Greater;
            let actual = (0..u_values.len() - 1).all(|u_cell| cells.contains(&(u_cell, v_cell)));
            let partial = (0..u_values.len() - 1).any(|u_cell| cells.contains(&(u_cell, v_cell)));
            if actual != expected || (partial && !actual) {
                return Ok(None);
            }
        }
        Ok(Some(CertifiedTorusShell {
            center,
            axis,
            major_radius,
            minor_radius,
            region: if cap_groups.is_empty() {
                CertifiedTorusRegion::Whole
            } else {
                CertifiedTorusRegion::Axial {
                    min: axial_min,
                    max: axial_max,
                }
            },
        }))
    }

    pub(in crate::topology) fn certified_torus_longitudinal_cap_group(
        &self,
        faces: &[FaceId],
        group: &[usize],
        center: &Point3,
        axis: &Vector3,
        major_radius: &Real,
        minor_radius: &Real,
    ) -> Result<Option<Vector3>, BuildError> {
        let Some(boundaries) = self.cap_boundary_use_loops(faces, group)? else {
            return Ok(None);
        };
        if boundaries.len() != 2 {
            return Ok(None);
        }

        let mut circle_centers = Vec::with_capacity(2);
        for boundary in &boundaries {
            let mut boundary_center = None;
            let mut sweep = Real::zero();
            for edge_use_id in boundary {
                let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
                let Curve3ExactData::EllipseArc(circle) = self.curve_ref(edge.curve)?.exact_data()
                else {
                    return Ok(None);
                };
                if !circle.circle
                    || !real_values_equal(&circle.x_radius, minor_radius)?
                    || !real_values_equal(&circle.y_radius, minor_radius)?
                {
                    return Ok(None);
                }
                if let Some(expected) = &boundary_center {
                    if !points_equal(expected, &circle.center)? {
                        return Ok(None);
                    }
                } else {
                    boundary_center = Some(circle.center.clone());
                }
                sweep += edge.domain.end() - edge.domain.start();
            }
            if !real_values_equal(&sweep, &Real::tau())? {
                return Ok(None);
            }
            let Some(boundary_center) = boundary_center else {
                return Ok(None);
            };
            let offset = &boundary_center - center;
            if !real_values_equal(&offset.dot(axis), &Real::zero())?
                || !real_values_equal(&offset.norm_squared(), &(major_radius * major_radius))?
            {
                return Ok(None);
            }
            circle_centers.push(boundary_center);
        }
        let first_offset = &circle_centers[0] - center;
        let second_offset = &circle_centers[1] - center;
        if !real_values_equal(
            &(first_offset + second_offset).norm_squared(),
            &Real::zero(),
        )? {
            return Ok(None);
        }

        let mut outward = None::<Vector3>;
        for index in group {
            let face = self.face_ref(faces[*index])?;
            let SurfaceExactData::Plane { origin, u, v } =
                self.surface_ref(face.surface)?.exact_data()
            else {
                return Ok(None);
            };
            let normal = u.cross(&v);
            if !real_values_equal(&normal.dot(axis), &Real::zero())?
                || !real_values_equal(&normal.dot(&(center - &origin)), &Real::zero())?
            {
                return Ok(None);
            }
            let oriented = match face.orientation {
                Orientation::Forward => normal,
                Orientation::Reversed => -normal,
            }
            .normalize()
            .map_err(GeometryError::Numeric)?;
            if let Some(expected) = &outward {
                if !vectors_equal(expected, &oriented)? {
                    return Ok(None);
                }
            } else {
                outward = Some(oriented);
            }
        }
        Ok(outward.map(|outward| -outward))
    }

    pub(in crate::topology) fn certified_torus_cap_group(
        &self,
        faces: &[FaceId],
        group: &[usize],
        center: &Point3,
        axis: &Vector3,
        major_radius: &Real,
        minor_radius: &Real,
    ) -> Result<Option<(Real, std::cmp::Ordering)>, BuildError> {
        let Some(boundaries) = self.cap_boundary_use_loops(faces, group)? else {
            return Ok(None);
        };
        if boundaries.len() != 2 {
            return Ok(None);
        }
        let first_use = *boundaries[0].first().ok_or(BuildError::EmptyWire)?;
        let first_edge = self.edge_ref(self.edge_use_ref(first_use)?.edge)?;
        let Curve3ExactData::EllipseArc(first_circle) =
            self.curve_ref(first_edge.curve)?.exact_data()
        else {
            return Ok(None);
        };
        if !first_circle.circle {
            return Ok(None);
        }
        let axial = (&first_circle.center - center).dot(axis);
        let expected_center = center.clone() + axis.clone() * &axial;
        if !points_equal(&first_circle.center, &expected_center)?
            || decided_model_order(compare_reals(
                &axial,
                &(-minor_radius.clone()),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Greater
            || decided_model_order(compare_reals(
                &axial,
                minor_radius,
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Less
        {
            return Ok(None);
        }
        let radial_offset = (minor_radius * minor_radius - &axial * &axial)
            .sqrt()
            .map_err(GeometryError::Numeric)?;
        let expected_radii = [major_radius - &radial_offset, major_radius + &radial_offset];
        let mut matched_radii = [false; 2];
        for boundary in &boundaries {
            let mut sweep = Real::zero();
            let mut radius_index = None;
            for edge_use_id in boundary {
                let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
                let Curve3ExactData::EllipseArc(circle) = self.curve_ref(edge.curve)?.exact_data()
                else {
                    return Ok(None);
                };
                if !circle.circle || !points_equal(&circle.center, &expected_center)? {
                    return Ok(None);
                }
                let mut this_radius = None;
                for (index, expected) in expected_radii.iter().enumerate() {
                    if real_values_equal(&circle.x_radius, expected)?
                        && real_values_equal(&circle.y_radius, expected)?
                    {
                        this_radius = Some(index);
                        break;
                    }
                }
                let Some(this_radius) = this_radius else {
                    return Ok(None);
                };
                if radius_index.is_some_and(|index| index != this_radius) {
                    return Ok(None);
                }
                radius_index = Some(this_radius);
                sweep += edge.domain.end() - edge.domain.start();
            }
            if !real_values_equal(&sweep, &Real::tau())? {
                return Ok(None);
            }
            let Some(radius_index) = radius_index else {
                return Ok(None);
            };
            if matched_radii[radius_index] {
                return Ok(None);
            }
            matched_radii[radius_index] = true;
        }
        if matched_radii != [true, true] {
            return Ok(None);
        }

        let mut outward = None;
        for index in group {
            let face = self.face_ref(faces[*index])?;
            let SurfaceExactData::Plane { origin, u, v } =
                self.surface_ref(face.surface)?.exact_data()
            else {
                return Ok(None);
            };
            let normal = u.cross(&v);
            let axial_normal = normal.dot(axis);
            if decided_model_order(compare_reals(
                &normal.cross(axis).norm_squared(),
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Equal
                || !real_values_equal(&(origin - center).dot(axis), &axial)?
            {
                return Ok(None);
            }
            let oriented_axial = match face.orientation {
                Orientation::Forward => axial_normal,
                Orientation::Reversed => -axial_normal,
            };
            let this_outward = decided_model_order(compare_reals(
                &oriented_axial,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))?;
            if this_outward == std::cmp::Ordering::Equal
                || outward.is_some_and(|expected| expected != this_outward)
            {
                return Ok(None);
            }
            outward = Some(this_outward);
        }
        Ok(outward.map(|outward| (axial, outward)))
    }
}
