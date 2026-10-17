//! Validated construction: validation cylinder.

use crate::topology::*;

impl ArenaModelBuilder {
    /// Native pcurves stay cheap. Selected subdivisions may instead prove the
    /// same rectangle through reassembled, injective boundary images.
    fn cylinder_wire_lines(&self, wire: WireId) -> Result<Option<Vec<LineSeg2>>, BuildError> {
        let native = self
            .wire_ref(wire)?
            .edge_uses
            .iter()
            .map(|edge_use| {
                Ok(self
                    .pcurve_ref(self.edge_use_ref(*edge_use)?.pcurve)?
                    .line_segment()
                    .cloned())
            })
            .collect::<Result<Option<Vec<_>>, BuildError>>()?;
        if native.is_some() {
            return Ok(native);
        }
        let contour = match self.build_wire_contour(wire) {
            Ok(contour) => contour,
            Err(BuildError::Geometry(GeometryError::UnsupportedPcurveContour)) => return Ok(None),
            Err(error) => return Err(error),
        };
        Ok(contour
            .segments()
            .iter()
            .map(|segment| match segment {
                Segment2::Line(line) => Some(line.clone()),
                _ => None,
            })
            .collect())
    }

    pub(in crate::topology) fn certified_cylinder_side_faces(
        &self,
        _shell: ShellId,
        side_faces: &[FaceId],
        orientation: Orientation,
    ) -> Result<Option<CertifiedCylinderShell>, BuildError> {
        if side_faces.is_empty() {
            return Ok(None);
        }
        let cylinder_surface_id = self.face_ref(side_faces[0])?.surface;
        if side_faces
            .iter()
            .any(|face| self.face_ref(*face).map(|face| face.surface) != Ok(cylinder_surface_id))
        {
            return Ok(None);
        }
        let SurfaceExactData::Cylinder {
            origin,
            axis,
            radius,
            ..
        } = self.surface_ref(cylinder_surface_id)?.exact_data()
        else {
            return Ok(None);
        };

        let mut u_values = Vec::new();
        let mut v_values = Vec::new();
        let mut face_coordinates = Vec::with_capacity(side_faces.len());
        for face_id in side_faces {
            let face = self.face_ref(*face_id)?;
            if face.orientation != orientation {
                return Ok(None);
            }
            let Some((u_min, u_max, v_min, v_max)) = self.cylinder_face_rectangle(*face_id)? else {
                return Ok(None);
            };
            insert_sorted_real(&mut u_values, &u_min)?;
            insert_sorted_real(&mut u_values, &u_max)?;
            insert_sorted_real(&mut v_values, &v_min)?;
            insert_sorted_real(&mut v_values, &v_max)?;
            face_coordinates.push((u_min, u_max, v_min, v_max));
        }
        if u_values.len() < 2 || v_values.len() < 2 {
            return Ok(None);
        }
        if !real_values_equal(
            &(u_values.last().expect("cylinder has angular values") - &u_values[0]),
            &Real::tau(),
        )? {
            return Ok(None);
        }
        let v_min = v_values[0].clone();
        let v_max = v_values.last().expect("cylinder has axial values").clone();
        if decided_model_order(compare_reals(&v_min, &v_max, crate::STRICT_PREDICATES))?
            != std::cmp::Ordering::Less
        {
            return Ok(None);
        }

        let mut rectangles = Vec::new();
        rectangles
            .try_reserve_exact(face_coordinates.len())
            .map_err(|_| GeometryError::Numeric(hyperreal::Problem::Exhausted))?;
        for (u_min, u_max, face_v_min, face_v_max) in face_coordinates {
            let u_start = exact_real_index(&u_values, &u_min)?;
            let u_end = exact_real_index(&u_values, &u_max)?;
            let v_start = exact_real_index(&v_values, &face_v_min)?;
            let v_end = exact_real_index(&v_values, &face_v_max)?;
            rectangles.push([u_start, u_end, v_start, v_end]);
        }
        if !super::rectangular_cover::exactly_covers(
            u_values.len() - 1,
            v_values.len() - 1,
            &rectangles,
        )? {
            return Ok(None);
        }
        Ok(Some(CertifiedCylinderShell {
            origin,
            axis,
            radius,
            v_min,
            v_max,
            sphere_subtraction: None,
        }))
    }

    /// Exact rectangle in a native cylinder chart, including subdivided sides.
    /// This is a boundary-image proof, independent of a complete cylindrical solid.
    pub(in crate::topology) fn cylinder_face_rectangle(
        &self,
        face_id: FaceId,
    ) -> Result<Option<(Real, Real, Real, Real)>, BuildError> {
        let face = self.face_ref(face_id)?;
        if self.surface_ref(face.surface)?.kind() != SurfaceKind::Cylinder
            || !face.inner().is_empty()
        {
            return Ok(None);
        }
        let Some(outer) = face.outer() else {
            return Ok(None);
        };
        if self.wire_ref(outer)?.edge_uses.len() < 4 {
            return Ok(None);
        }
        let Some(lines) = self.cylinder_wire_lines(outer)? else {
            return Ok(None);
        };
        let mut u = Vec::with_capacity(8);
        let mut v = Vec::with_capacity(8);
        for line in &lines {
            let u_constant = real_values_equal(line.start().x(), line.end().x())?;
            let v_constant = real_values_equal(line.start().y(), line.end().y())?;
            if u_constant == v_constant {
                return Ok(None);
            }
            u.extend([line.start().x().clone(), line.end().x().clone()]);
            v.extend([line.start().y().clone(), line.end().y().clone()]);
        }
        let (u_min, u_max) = exact_real_min_max(&u)?;
        let (v_min, v_max) = exact_real_min_max(&v)?;
        if !self.axis_aligned_lines_tile_rectangle(&lines, &u_min, &u_max, &v_min, &v_max)? {
            return Ok(None);
        }
        Ok(Some((u_min, u_max, v_min, v_max)))
    }

    pub(in crate::topology) fn cylinder_face_v_bounds(
        &self,
        face_id: FaceId,
    ) -> Result<Option<(Real, Real)>, BuildError> {
        let face = self.face_ref(face_id)?;
        if self.surface_ref(face.surface)?.kind() != SurfaceKind::Cylinder {
            return Ok(None);
        }
        let Some(outer) = face.outer() else {
            return Ok(None);
        };
        let mut values = Vec::new();
        let Some(lines) = self.cylinder_wire_lines(outer)? else {
            return Ok(None);
        };
        for line in lines {
            values.extend([line.start().y().clone(), line.end().y().clone()]);
        }
        exact_real_min_max(&values).map(Some)
    }

    pub(in crate::topology) fn certified_cylinder_cap_group(
        &self,
        faces: &[FaceId],
        cap_group: &[usize],
        cylinder: &CertifiedCylinderShell,
        cap_parameter: &Real,
        expected_direction: std::cmp::Ordering,
    ) -> Result<bool, BuildError> {
        let Some(boundaries) = self.cap_boundary_use_loops(faces, cap_group)? else {
            return Ok(false);
        };
        let [boundary] = boundaries.as_slice() else {
            return Ok(false);
        };
        let expected_center = cylinder.origin.clone() + cylinder.axis.clone() * cap_parameter;
        let mut sweep = Real::zero();
        for edge_use_id in boundary {
            let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
            let Curve3ExactData::EllipseArc(circle) = self.curve_ref(edge.curve)?.exact_data()
            else {
                return Ok(false);
            };
            if !circle.circle
                || !points_equal(&circle.center, &expected_center)?
                || !real_values_equal(&circle.x_radius, &cylinder.radius)?
                || !real_values_equal(&circle.y_radius, &cylinder.radius)?
            {
                return Ok(false);
            }
            sweep += edge.domain.represented_end()? - edge.domain.represented_start()?;
        }
        if !real_values_equal(&sweep, &Real::tau())? {
            return Ok(false);
        }
        for index in cap_group {
            let face = self.face_ref(faces[*index])?;
            let SurfaceExactData::Plane { origin, u, v } =
                self.surface_ref(face.surface)?.exact_data()
            else {
                return Ok(false);
            };
            let normal = u.cross(&v);
            if decided_model_order(compare_reals(
                &normal.cross(&cylinder.axis).norm_squared(),
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Equal
                || !real_values_equal(
                    &(origin - &cylinder.origin).dot(&cylinder.axis),
                    cap_parameter,
                )?
            {
                return Ok(false);
            }
            let oriented = match face.orientation {
                Orientation::Forward => normal.dot(&cylinder.axis),
                Orientation::Reversed => -normal.dot(&cylinder.axis),
            };
            if decided_model_order(compare_reals(
                &oriented,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != expected_direction
            {
                return Ok(false);
            }
        }
        Ok(true)
    }

    pub(in crate::topology) fn certified_cylinder_sphere_component_shell(
        &self,
        shell: ShellId,
        faces: &[FaceId],
        side_faces: &[FaceId],
        sphere_faces: &[FaceId],
    ) -> Result<Option<CertifiedCylinderShell>, BuildError> {
        let [sphere_face] = sphere_faces else {
            return Ok(None);
        };
        let Some(sphere_cap) = self.certified_spherical_cap_face(*sphere_face)? else {
            return Ok(None);
        };
        if sphere_cap.orientation != Orientation::Reversed {
            return Ok(None);
        }
        let Some(mut cylinder) =
            self.certified_cylinder_side_faces(shell, side_faces, Orientation::Forward)?
        else {
            return Ok(None);
        };
        if !vectors_equal(&sphere_cap.axis, &cylinder.axis)?
            || decided_model_order(compare_reals(
                &cylinder.radius,
                &sphere_cap.radius,
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Less
        {
            return Ok(None);
        }
        let center_offset = &sphere_cap.center - &cylinder.origin;
        let center_parameter = center_offset.dot(&cylinder.axis);
        let radial_offset = center_offset - cylinder.axis.clone() * &center_parameter;
        if decided_model_order(compare_reals(
            &radial_offset.norm_squared(),
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != std::cmp::Ordering::Equal
        {
            return Ok(None);
        }
        let intersection_height = &sphere_cap.radius * sphere_cap.latitude.clone().sin();
        if !real_values_equal(
            &(&sphere_cap.radius * sphere_cap.latitude.clone().cos()),
            &cylinder.radius,
        )? {
            return Ok(None);
        }
        let (side, cap_parameter, expected_cap_direction) = if sphere_cap.upper {
            if decided_model_order(compare_reals(
                &intersection_height,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Greater
                || !real_values_equal(&cylinder.v_min, &(&center_parameter + &intersection_height))?
                || decided_model_order(compare_reals(
                    &(&cylinder.v_max - &center_parameter),
                    &sphere_cap.radius,
                    crate::STRICT_PREDICATES,
                ))? != std::cmp::Ordering::Greater
            {
                return Ok(None);
            }
            (
                CertifiedCylinderSphereComponentSide::Upper,
                cylinder.v_max.clone(),
                std::cmp::Ordering::Greater,
            )
        } else {
            if decided_model_order(compare_reals(
                &intersection_height,
                &Real::zero(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Less
                || !real_values_equal(&cylinder.v_max, &(&center_parameter + &intersection_height))?
                || decided_model_order(compare_reals(
                    &(&cylinder.v_min - &center_parameter),
                    &-sphere_cap.radius.clone(),
                    crate::STRICT_PREDICATES,
                ))? != std::cmp::Ordering::Less
            {
                return Ok(None);
            }
            (
                CertifiedCylinderSphereComponentSide::Lower,
                cylinder.v_min.clone(),
                std::cmp::Ordering::Less,
            )
        };

        let cap_groups = self.planar_face_groups(faces)?;
        let [cap_group] = cap_groups.as_slice() else {
            return Ok(None);
        };
        if !self.certified_cylinder_cap_group(
            faces,
            cap_group,
            &cylinder,
            &cap_parameter,
            expected_cap_direction,
        )? {
            return Ok(None);
        }
        cylinder.sphere_subtraction = Some(CertifiedCylinderSphereSubtraction::Component {
            center: sphere_cap.center,
            radius: sphere_cap.radius,
            side,
        });
        Ok(Some(cylinder))
    }

    pub(in crate::topology) fn certified_oriented_cylinder_shell(
        &self,
        shell: ShellId,
        orientation: Orientation,
    ) -> Result<Option<CertifiedCylinderShell>, BuildError> {
        let faces = &self.shell_ref(shell)?.faces;
        let mut side_faces = Vec::new();
        let mut sphere_faces = Vec::new();
        for face_id in faces {
            let face = self.face_ref(*face_id)?;
            if !face.inner().is_empty() {
                return Ok(None);
            }
            match self.surface_ref(face.surface)?.kind() {
                SurfaceKind::Plane => {}
                SurfaceKind::Cylinder => side_faces.push(*face_id),
                SurfaceKind::Sphere => sphere_faces.push(*face_id),
                _ => return Ok(None),
            }
        }
        if !sphere_faces.is_empty() {
            if orientation != Orientation::Forward {
                return Ok(None);
            }
            return self.certified_cylinder_sphere_component_shell(
                shell,
                faces,
                &side_faces,
                &sphere_faces,
            );
        }
        let cap_groups = self.planar_face_groups(faces)?;
        if cap_groups.len() != 2 || side_faces.is_empty() {
            return Ok(None);
        }
        let Some(cylinder) = self.certified_cylinder_side_faces(shell, &side_faces, orientation)?
        else {
            return Ok(None);
        };

        let expected_centers = [
            cylinder.origin.clone() + cylinder.axis.clone() * &cylinder.v_min,
            cylinder.origin.clone() + cylinder.axis.clone() * &cylinder.v_max,
        ];
        let mut matched_centers = [false; 2];
        for cap_group in cap_groups {
            let Some(boundaries) = self.cap_boundary_use_loops(faces, &cap_group)? else {
                return Ok(None);
            };
            if boundaries.len() != 1 {
                return Ok(None);
            }
            let mut sweep = Real::zero();
            let mut center_index = None;
            for edge_use_id in &boundaries[0] {
                let edge = self.edge_ref(self.edge_use_ref(*edge_use_id)?.edge)?;
                let curve = self.curve_ref(edge.curve)?;
                let Curve3ExactData::EllipseArc(data) = curve.exact_data() else {
                    return Ok(None);
                };
                if !data.circle
                    || decided_model_order(compare_reals(
                        &data.x_radius,
                        &cylinder.radius,
                        crate::STRICT_PREDICATES,
                    ))? != std::cmp::Ordering::Equal
                {
                    return Ok(None);
                }
                let mut this_center = None;
                for (index, expected) in expected_centers.iter().enumerate() {
                    if points_equal(&data.center, expected)? {
                        this_center = Some(index);
                        break;
                    }
                }
                let Some(this_center) = this_center else {
                    return Ok(None);
                };
                if center_index.is_some_and(|index| index != this_center) {
                    return Ok(None);
                }
                center_index = Some(this_center);
                sweep += edge.domain.represented_end()? - edge.domain.represented_start()?;
            }
            if decided_model_order(compare_reals(
                &sweep,
                &Real::tau(),
                crate::STRICT_PREDICATES,
            ))? != std::cmp::Ordering::Equal
            {
                return Ok(None);
            }
            let Some(center_index) = center_index else {
                return Ok(None);
            };
            if matched_centers[center_index] {
                return Ok(None);
            }
            matched_centers[center_index] = true;
            let outward = if center_index == 0 {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            };
            let expected_direction = if orientation == Orientation::Forward {
                outward
            } else {
                outward.reverse()
            };
            let cap_parameter = if center_index == 0 {
                &cylinder.v_min
            } else {
                &cylinder.v_max
            };
            if !self.certified_cylinder_cap_group(
                faces,
                &cap_group,
                &cylinder,
                cap_parameter,
                expected_direction,
            )? {
                return Ok(None);
            }
        }
        if matched_centers != [true, true] {
            return Ok(None);
        }
        Ok(Some(cylinder))
    }

    pub(in crate::topology) fn certified_cylinder_shell(
        &self,
        shell: ShellId,
    ) -> Result<Option<CertifiedCylinderShell>, BuildError> {
        self.certified_oriented_cylinder_shell(shell, Orientation::Forward)
    }

    pub(in crate::topology) fn certified_cylinder_solid(
        &self,
        solid: &Solid,
    ) -> Result<Option<CertifiedCylinderShell>, BuildError> {
        let Some(mut cylinder) = self.certified_cylinder_shell(solid.outer)? else {
            return Ok(None);
        };
        if solid.voids.is_empty() {
            return Ok(Some(cylinder));
        }
        let [void_shell] = solid.voids.as_slice() else {
            return Ok(None);
        };
        let Some((center, radius)) =
            self.certified_oriented_sphere_shell(*void_shell, Orientation::Reversed)?
        else {
            return Ok(None);
        };
        if !self.cylinder_strictly_contains_sphere(&cylinder, &center, &radius)? {
            return Ok(None);
        }
        cylinder.sphere_subtraction =
            Some(CertifiedCylinderSphereSubtraction::Void { center, radius });
        Ok(Some(cylinder))
    }
}
