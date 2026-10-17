//! Validated construction: validation cone.

use super::sector::certified_sector_coverage;
use crate::topology::*;

impl ArenaModelBuilder {
    pub(in crate::topology) fn certified_cone_frustum_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedConeFrustumShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut cone_surface = None;
        let mut u_values = Vec::new();
        let mut v_values = Vec::new();
        let mut face_coordinates = Vec::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            if !face.inner().is_empty() {
                return Ok(None);
            }
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::Plane => continue,
                SurfaceKind::Cone if face.orientation == Orientation::Forward => {}
                _ => return Ok(None),
            }
            match cone_surface {
                Some(expected) if expected != face.surface => {
                    if !self
                        .surface_ref(expected)?
                        .same_periodic_analytic_parameterization(self.surface_ref(face.surface)?)?
                    {
                        return Ok(None);
                    }
                }
                Some(_) => {}
                None => cone_surface = Some(face.surface),
            }
            let Some(outer) = face.outer() else {
                return Ok(None);
            };
            let wire = self.wire_ref(outer)?;
            if wire.edge_uses.len() < 4 {
                return Ok(None);
            }
            let mut face_u = Vec::with_capacity(8);
            let mut face_v = Vec::with_capacity(8);
            let mut parameter_segments = Vec::with_capacity(wire.edge_uses.len());
            for edge_use_id in &wire.edge_uses {
                let edge_use = self.edge_use_ref(*edge_use_id)?;
                let pcurve = self.pcurve_ref(edge_use.pcurve)?;
                let Some(line) = pcurve.line_segment() else {
                    return Ok(None);
                };
                let u_constant = real_values_equal(line.start().x(), line.end().x())?;
                let v_constant = real_values_equal(line.start().y(), line.end().y())?;
                if u_constant == v_constant {
                    return Ok(None);
                }
                face_u.extend([line.start().x().clone(), line.end().x().clone()]);
                face_v.extend([line.start().y().clone(), line.end().y().clone()]);
                parameter_segments.push(Segment2::Line(line.clone()));
            }
            let (u_min, u_max) = exact_real_min_max(&face_u)?;
            let (v_min, v_max) = exact_real_min_max(&face_v)?;
            let mut side_intervals: [Vec<(Real, Real)>; 4] = std::array::from_fn(|_| Vec::new());
            for segment in &parameter_segments {
                let Segment2::Line(line) = segment else {
                    unreachable!("cone parameter segments are exact lines");
                };
                let u_constant = real_values_equal(line.start().x(), line.end().x())?;
                let (side, first, second) = if u_constant {
                    if real_values_equal(line.start().x(), &u_min)? {
                        (2, line.start().y(), line.end().y())
                    } else if real_values_equal(line.start().x(), &u_max)? {
                        (3, line.start().y(), line.end().y())
                    } else {
                        return Ok(None);
                    }
                } else if real_values_equal(line.start().y(), &v_min)? {
                    (0, line.start().x(), line.end().x())
                } else if real_values_equal(line.start().y(), &v_max)? {
                    (1, line.start().x(), line.end().x())
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
            face_coordinates.push([u_min, u_max, v_min, v_max]);
        }
        // Independently authored charts can use different integral U lifts.
        // Keep the original topology and maps; reduce only the coverage
        // evidence when it cannot already fit in one common period.
        if let (Some(first), Some(last)) = (u_values.first(), u_values.last())
            && decided_model_order(compare_reals(
                &(last - first),
                &Real::tau(),
                crate::STRICT_PREDICATES,
            ))? == std::cmp::Ordering::Greater
        {
            let Some(rebased) = super::periodic_rectangles::rebase(&face_coordinates)? else {
                return Ok(None);
            };
            face_coordinates = rebased;
            u_values.clear();
            for [u0, u1, _, _] in &face_coordinates {
                insert_sorted_real(&mut u_values, u0)?;
                insert_sorted_real(&mut u_values, u1)?;
            }
        }
        let cap_groups = self.planar_face_groups(faces)?;
        if !matches!(cap_groups.len(), 2..=4)
            || face_coordinates.is_empty()
            || u_values.len() < 2
            || v_values.len() < 2
        {
            return Ok(None);
        }
        let v_min = v_values[0].clone();
        let v_max = v_values.last().expect("frustum has axial values").clone();
        if decided_model_order(compare_reals(
            &v_min,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Greater
        {
            return Ok(None);
        }
        let SurfaceExactData::Cone {
            apex,
            x,
            y,
            axis,
            semi_angle,
        } = self
            .surface_ref(cone_surface.expect("frustum has cone faces"))?
            .exact_data()
        else {
            unreachable!("cone kind carries cone exact data");
        };
        let mut cells = HashSet::new();
        for [u_min, u_max, face_v_min, face_v_max] in &face_coordinates {
            let u_start = exact_real_index(&u_values, u_min)?;
            let u_end = exact_real_index(&u_values, u_max)?;
            let v_start = exact_real_index(&v_values, face_v_min)?;
            let v_end = exact_real_index(&v_values, face_v_max)?;
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

        let region = if cap_groups.len() == 4 {
            let cosine = semi_angle.clone().cos();
            let height_min = &v_min * &cosine;
            let height_max = &v_max * cosine;
            let Some(bounds) = self.certified_sector_plane_bounds(
                faces,
                &cap_groups,
                &apex,
                &axis,
                (&height_min, &height_max),
                true,
            )?
            else {
                return Ok(None);
            };
            let Some(sector) = certified_sector_coverage(
                &u_values,
                &vec![true; v_values.len() - 1],
                &cells,
                &x,
                &y,
                bounds.normals,
            )?
            else {
                return Ok(None);
            };
            CertifiedConeFrustumRegion::Sector(sector)
        } else if cap_groups.len() == 3 {
            let Some(interior_normal) = self.certified_cone_longitudinal_cap_groups(
                faces,
                &cap_groups,
                &apex,
                &axis,
                &semi_angle,
                &v_min,
                &v_max,
            )?
            else {
                return Ok(None);
            };
            if !certified_periodic_longitudinal_half_coverage(
                &u_values,
                &v_values,
                &cells,
                &x,
                &y,
                &interior_normal,
            )? {
                return Ok(None);
            }
            CertifiedConeFrustumRegion::LongitudinalHalf { interior_normal }
        } else {
            // Full-period edges can bound one periodic side face. Completeness
            // follows from the exact tau span and nonoverlapping rectangular
            // cell coverage above, independently of the number of partitions.
            if !real_values_equal(
                &(u_values.last().expect("frustum u grid") - &u_values[0]),
                &Real::tau(),
            )? || cells.len() != (u_values.len() - 1) * (v_values.len() - 1)
            {
                return Ok(None);
            }
            CertifiedConeFrustumRegion::Whole
        };
        Ok(Some(CertifiedConeFrustumShell {
            apex,
            axis,
            semi_angle,
            v_min,
            v_max,
            region,
        }))
    }

    #[allow(clippy::too_many_arguments)]
    pub(in crate::topology) fn certified_cone_longitudinal_cap_groups(
        &self,
        faces: &[FaceId],
        groups: &[Vec<usize>],
        apex: &Point3,
        axis: &Vector3,
        semi_angle: &Real,
        v_min: &Real,
        v_max: &Real,
    ) -> Result<Option<Vector3>, BuildError> {
        let cosine = semi_angle.clone().cos();
        let mut lower = false;
        let mut upper = false;
        let mut interior_normal = None;
        for group in groups {
            let Some(boundaries) = self.cap_boundary_use_loops(faces, group)? else {
                return Ok(None);
            };
            if boundaries.len() != 1 {
                return Ok(None);
            }
            let mut group_origin = None;
            let mut group_normal = None::<Vector3>;
            for index in group {
                let face = self.face_ref(faces[*index])?;
                let SurfaceExactData::Plane { origin, u, v } =
                    self.surface_ref(face.surface)?.exact_data()
                else {
                    return Ok(None);
                };
                let normal = u.cross(&v);
                let oriented = match face.orientation {
                    Orientation::Forward => normal,
                    Orientation::Reversed => -normal,
                }
                .normalize()
                .map_err(GeometryError::Numeric)?;
                if let Some(expected) = &group_normal {
                    if !vectors_equal(expected, &oriented)? {
                        return Ok(None);
                    }
                } else {
                    group_origin = Some(origin);
                    group_normal = Some(oriented);
                }
            }
            let Some(origin) = group_origin else {
                return Ok(None);
            };
            let outward = group_normal.expect("nonempty planar group");
            if real_values_equal(&outward.cross(axis).norm_squared(), &Real::zero())? {
                let parameter =
                    ((&origin - apex).dot(axis) / &cosine).map_err(GeometryError::Numeric)?;
                let direction = decided_model_order(compare_reals(
                    &outward.dot(axis),
                    &Real::zero(),
                    crate::STRICT_PREDICATES,
                ))?;
                if real_values_equal(&parameter, v_min)?
                    && direction == std::cmp::Ordering::Less
                    && !lower
                {
                    lower = true;
                } else if real_values_equal(&parameter, v_max)?
                    && direction == std::cmp::Ordering::Greater
                    && !upper
                {
                    upper = true;
                } else {
                    return Ok(None);
                }
            } else if real_values_equal(&outward.dot(axis), &Real::zero())?
                && real_values_equal(&outward.dot(&(apex - &origin)), &Real::zero())?
                && interior_normal.is_none()
            {
                interior_normal = Some(-outward);
            } else {
                return Ok(None);
            }
        }
        if lower && upper {
            Ok(interior_normal)
        } else {
            Ok(None)
        }
    }
}
